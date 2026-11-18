/*
 * This file is part of the MeatPi components project.
 *
 * Copyright (C) 2022-2026 MeatPi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file wifi_manager.c
 * @brief Lifecycle and the radio's state: init, start, stop, status.
 *
 * Configuration is boot-applied via the settings descriptor
 * (wifi_manager_settings.c); STA candidate selection and the auth-failure
 * ban list are pure logic (wifi_manager_select.c). This file owns the
 * radio: netifs, esp_wifi bring-up, the status, the lock and the event
 * group. The station connect path is wifi_manager_sta.c, the access point
 * side wifi_manager_ap.c, the driver event handler wifi_manager_handler.c,
 * the reconnect task wifi_manager_reconnect.c, and the gates of the
 * connected network wifi_manager_network.c.
 */
#include "wifi_manager.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "dev_status_manager.h"
#include "log_manager.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

wm_status_t       wm_status EXT_RAM_BSS_ATTR;
wm_select_state_t wm_select_state EXT_RAM_BSS_ATTR;

static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;

static EventGroupHandle_t s_events;
static StaticEventGroup_t s_events_buf;      /* internal: FreeRTOS object */
SemaphoreHandle_t  wm_lock;
static StaticSemaphore_t  s_lock_buf;        /* internal: FreeRTOS object */

TaskHandle_t wm_reconnect_task_handle;
static StaticTask_t s_reconnect_tcb;         /* internal: FreeRTOS object */
static StackType_t  s_reconnect_stack[4096] EXT_RAM_BSS_ATTR;

uint32_t wm_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

bool wm_core_started(void)
{
    return wm_status.started;
}

/* ---- lifecycle -------------------------------------------------------------------- */

esp_err_t wifi_manager_init(void)
{
    static const log_descriptor_t LOG_DESC = { "wifi_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */
    wm_events_register();            /* wifi.sta + ${wifi.ssid} (rules) */

    if (s_events == NULL)
    {
        s_events = xEventGroupCreateStatic(&s_events_buf);
        wm_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    return wm_settings_register();
}

esp_err_t wifi_manager_start(void)
{
    if (!wm_settings_is_configured())
    {
        ESP_LOGE(TAG, "unconfigured (settings boot pass failed); not starting");
        return ESP_ERR_INVALID_STATE; /* Coding Standard §3 / §4.3 step 5 */
    }

    if (wm_status.started)
    {
        return ESP_OK;
    }

    const wm_config_t *cfg = wm_settings_config();

    wm_select_init(&wm_select_state); /* first sequential pick = the primary */

    if (cfg->mode == WM_MODE_OFF)
    {
        ESP_LOGI(TAG, "mode off; radio stays down");
        wm_status.started = true;
        return ESP_OK;
    }

    esp_err_t err;

    if (!wm_status.radio_inited)
    {
        /* tolerate main having created these already */
        err = esp_netif_init();

        if (err != ESP_OK)
        {
            return err;
        }

        err = esp_event_loop_create_default();

        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            return err;
        }

        s_ap_netif = esp_netif_create_default_wifi_ap();
        s_sta_netif = esp_netif_create_default_wifi_sta();

        if (s_ap_netif == NULL || s_sta_netif == NULL)
        {
            return ESP_ERR_NO_MEM;
        }

        wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();

        /* WiFi memory profile (v5): the driver buffer counts are runtime
           fields, so a setting trades internal RAM for throughput with no
           recompile. Only the internal-heap consumers are touched; the
           dynamic RX/TX pools stay (they land in PSRAM) and rx_ba_win is
           left at its default to avoid the buffer/BA-window constraint. */
        switch (cfg->ram_profile)
        {
            case WM_RAM_LEAN:
                init_cfg.static_rx_buf_num = 6;  /* 10 default */
                init_cfg.static_tx_buf_num = 4;  /* 8  default */
                init_cfg.cache_tx_buf_num = 16;  /* 32 default */
                break;
            case WM_RAM_CUSTOM:
                init_cfg.static_rx_buf_num = cfg->wifi_static_rx;
                init_cfg.static_tx_buf_num = cfg->wifi_static_tx;
                init_cfg.cache_tx_buf_num = cfg->wifi_cache_tx;
                break;
            case WM_RAM_FULL:
            default:
                break; /* IDF defaults */
        }

        ESP_LOGI(TAG, "wifi ram profile=%d (rx=%d tx=%d cache=%d)",
                 (int)cfg->ram_profile, init_cfg.static_rx_buf_num,
                 init_cfg.static_tx_buf_num, init_cfg.cache_tx_buf_num);

        err = esp_wifi_init(&init_cfg);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
            return err;
        }

        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         wm_event_handler, NULL);

        if (err == ESP_OK)
        {
            err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wm_event_handler, NULL);
        }

        if (err != ESP_OK)
        {
            return err;
        }

        static const wifi_country_t world =
            { .cc = "01", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_AUTO };

        esp_wifi_set_country(&world);
        esp_wifi_set_storage(WIFI_STORAGE_RAM); /* settings own persistence */
        wm_status.radio_inited = true;
    }

    /* mode */
    wifi_mode_t mode = WIFI_MODE_APSTA;

    if (cfg->mode == WM_MODE_STA)
    {
        mode = WIFI_MODE_STA;
    }
    else if (cfg->mode == WM_MODE_AP)
    {
        mode = WIFI_MODE_AP;
    }

    err = esp_wifi_set_mode(mode);

    if (err != ESP_OK)
    {
        return err;
    }

    /* STA side */
    if (cfg->mode == WM_MODE_STA || cfg->mode == WM_MODE_APSTA)
    {
        dev_status_manager_set(DEV_STATUS_BIT_STA_ENABLED);

        /* DHCP/DNS host name: the `hostname` setting, else
           wican_<device id> (what mdns_manager advertises as .local).
           lwIP's default "espressif" is what every router showed
           before (meatpi 2026-09-08). Set BEFORE wm_sta_apply_network
           starts the DHCP client so the first DISCOVER carries it. */
        char host[WM_HOSTNAME_LEN];

        if (wm_default_hostname(cfg->hostname,
                                dev_status_manager_device_id(), host,
                                sizeof(host)))
        {
            esp_err_t herr = esp_netif_set_hostname(s_sta_netif, host);

            if (herr == ESP_OK)
            {
                ESP_LOGI(TAG, "STA hostname %s", host);
            }
            else
            {
                ESP_LOGW(TAG, "STA hostname '%s' rejected: %s", host,
                         esp_err_to_name(herr));
            }
        }

        /* v6: per-network addressing is applied inside wm_sta_apply_network
           (each candidate carries its own static/DHCP choice) */
        if (cfg->sta_count > 0)
        {
            wm_sta_apply_network(cfg, 0, 0);
        }
    }

    /* AP side */
    if (cfg->mode == WM_MODE_AP || cfg->mode == WM_MODE_APSTA)
    {
        err = wm_ap_apply_side(cfg);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    err = esp_wifi_start();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start: %s", esp_err_to_name(err));
        return err;
    }

    /* AP channel bandwidth (v4): after start so the interface exists */
    if (cfg->mode == WM_MODE_AP || cfg->mode == WM_MODE_APSTA)
    {
        esp_wifi_set_bandwidth(WIFI_IF_AP,
                               cfg->ap_ht40 ? WIFI_BW40 : WIFI_BW20);
    }

    switch (cfg->power_save)
    {
        case WM_PS_MIN: esp_wifi_set_ps(WIFI_PS_MIN_MODEM); break;
        case WM_PS_MAX: esp_wifi_set_ps(WIFI_PS_MAX_MODEM); break;
        default:        esp_wifi_set_ps(WIFI_PS_NONE);      break;
    }

    wm_status.started = true;
    xEventGroupClearBits(s_events, WM_BIT_STOP_REQ | WM_BIT_TASK_EXITED);
    xEventGroupSetBits(s_events, WIFI_MANAGER_BIT_ENABLED);

    if ((cfg->mode == WM_MODE_STA || cfg->mode == WM_MODE_APSTA) &&
        cfg->sta_auto_reconnect && cfg->sta_count > 0 &&
        wm_reconnect_task_handle == NULL)
    {
        wm_reconnect_task_handle = xTaskCreateStatic(
            wm_reconnect_task, "wm_reconnect",
            sizeof(s_reconnect_stack) / sizeof(s_reconnect_stack[0]), NULL,
            5, s_reconnect_stack, &s_reconnect_tcb);
    }

    ESP_LOGI(TAG, "started (mode=%d, %u STA candidates)", (int)cfg->mode,
             (unsigned)cfg->sta_count);
    return ESP_OK;
}

esp_err_t wifi_manager_stop(void)
{
    if (!wm_status.started)
    {
        return ESP_OK;
    }

    if (wm_reconnect_task_handle != NULL)
    {
        xEventGroupSetBits(s_events, WM_BIT_STOP_REQ);
        xEventGroupWaitBits(s_events, WM_BIT_TASK_EXITED, pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(2 * WM_RECONNECT_PERIOD_MS));
    }

    if (wm_status.sta_connected)
    {
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (wm_status.radio_inited)
    {
        esp_wifi_stop();
    }

    wm_status.started = false;
    wm_status.sta_connected = false;
    wm_status.ap_started = false;
    wm_status.ap_station_count = 0;
    wm_status.sta_retry_count = 0;
    wm_status.last_attempted_idx = -1;
    wm_status.last_reason = 0;
    wm_sta_last_resort = false;
    wm_sta_roam = WM_ROAM_IDLE;
    wm_sta_ever_connected = false; /* a restart is a fresh session */

    xSemaphoreTake(wm_lock, portMAX_DELAY);
    wm_status.sta_ip[0] = '\0';
    xSemaphoreGive(wm_lock);

    xEventGroupClearBits(s_events, WIFI_MANAGER_BIT_ENABLED |
        WIFI_MANAGER_BIT_STA_CONNECTED | WIFI_MANAGER_BIT_STA_DISCONNECTED |
        WIFI_MANAGER_BIT_AP_STARTED | WIFI_MANAGER_BIT_STA_AP_OVERLAP);
    dev_status_manager_clear(DEV_STATUS_BIT_STA_CONNECTED |
                             DEV_STATUS_BIT_STA_ENABLED |
                             DEV_STATUS_BIT_AP_ENABLED |
                             DEV_STATUS_BIT_STA_AP_OVERLAP);

    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

/* ---- bridge to wifi_manager_status.c (see wifi_manager_bridge.h) ----------- */

esp_err_t wm_status_copy_ip(char *buf, size_t buf_len)
{
    xSemaphoreTake(wm_lock, portMAX_DELAY);
    strlcpy(buf, wm_status.sta_ip, buf_len);
    xSemaphoreGive(wm_lock);
    return ESP_OK;
}

bool wm_status_flag(int which)
{
    if (s_events == NULL)
    {
        return false;
    }

    EventBits_t mask = (which == 0) ? WIFI_MANAGER_BIT_ENABLED
                     : (which == 1) ? WIFI_MANAGER_BIT_STA_CONNECTED
                     : (which == 2) ? WIFI_MANAGER_BIT_AP_STARTED : 0;

    return (xEventGroupGetBits(s_events) & mask) != 0;
}

uint16_t wm_status_ap_clients(void)
{
    return wm_status.ap_station_count;
}

esp_netif_t *wm_sta_netif(void)
{
    return s_sta_netif;
}

esp_netif_t *wm_ap_netif(void)
{
    return s_ap_netif;
}

EventGroupHandle_t wm_event_group(void)
{
    return s_events;
}
