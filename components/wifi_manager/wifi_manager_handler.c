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
 * @file wifi_manager_handler.c
 * @brief The driver event handler: what the manager does on every
 *        WIFI_EVENT and on the station's got-IP (the status and the
 *        event-group bits, the attempt memory, DNS, the AP housekeeping of
 *        APSTA mode) and the callbacks it reports to.
 */
#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"

#include "dev_status_manager.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

static wifi_manager_callbacks_t s_callbacks EXT_RAM_BSS_ATTR;

/* ---- event handling --------------------------------------------------------- */

/* A disconnect that ends an ATTEMPT (we never got an IP) counts as a
 * failed attempt whatever the reason (wrong password, an AP that never
 * finishes the handshake, a full AP, a refused association) except
 * "not found" (visibility, not a failure) and our own leave (2026-09-06,
 * meatpi: "three failed attempts in a row: try the others first"). The
 * memory only reorders candidates, so a strike costs nothing when the
 * entry is the only network around. */
static bool attempt_failed_reason(uint8_t reason)
{
    return reason != WIFI_REASON_NO_AP_FOUND &&
           reason != WIFI_REASON_ASSOC_LEAVE;
}

static void on_sta_got_ip(const ip_event_got_ip_t *event)
{
    const wm_config_t *cfg = wm_settings_config();

    if (wm_try_active())
    {
        char ipstr[16];

        snprintf(ipstr, sizeof(ipstr), IPSTR, IP2STR(&event->ip_info.ip));
        wm_ap_sync_channel_to_sta();

        if (wm_try_on_got_ip(ipstr))
        {
            return; /* the trial's address: not a connection of ours */
        }
    }

    wm_status.sta_connected = true;
    wm_status.sta_retry_count = 0;
    wm_sta_ever_connected = true;
    wm_sta_connect_started_ms = 0; /* attempt concluded */

    /* which candidate did we land on? (drives roam-to-preferred): the
     * attempt's entry index; the name walk stays as the fallback */
    wm_sta_connected_idx = -1;

    if (wm_status.last_attempted_idx >= 0 &&
        (size_t)wm_status.last_attempted_idx < cfg->sta_count &&
        strcmp(cfg->sta[wm_status.last_attempted_idx].ssid,
               wm_status.last_attempted_ssid) == 0)
    {
        wm_sta_connected_idx = wm_status.last_attempted_idx;
    }
    else
    {
        for (size_t i = 0; i < cfg->sta_count; i++)
        {
            if (strcmp(cfg->sta[i].ssid, wm_status.last_attempted_ssid) == 0)
            {
                wm_sta_connected_idx = (int)i;
                break;
            }
        }
    }

    wm_sta_roam = WM_ROAM_IDLE;
    wm_sta_last_resort = false;
    wm_sta_last_roam_ms = wm_now_ms();

    xSemaphoreTake(wm_lock, portMAX_DELAY);
    snprintf(wm_status.sta_ip, sizeof(wm_status.sta_ip), IPSTR,
             IP2STR(&event->ip_info.ip));
    xSemaphoreGive(wm_lock);

    ESP_LOGI(TAG, "STA got IP " IPSTR, IP2STR(&event->ip_info.ip));

    wm_select_on_success(&wm_select_state, wm_sta_connected_idx);

    xEventGroupSetBits(wm_event_group(), WIFI_MANAGER_BIT_STA_CONNECTED);
    xEventGroupClearBits(wm_event_group(), WIFI_MANAGER_BIT_STA_DISCONNECTED);
    dev_status_manager_set(DEV_STATUS_BIT_STA_CONNECTED);
    wm_events_sta(true, (wm_sta_connected_idx >= 0)
                            ? cfg->sta[wm_sta_connected_idx].ssid
                            : wm_status.last_attempted_ssid);

    if (s_callbacks.sta_connected != NULL)
    {
        s_callbacks.sta_connected();
    }

    /* v6: custom DNS override in DHCP mode, the lease just overwrote
       MAIN DNS, so re-assert the user's server on every got-ip. (For a
       static network the client is stopped; the attempt set it once.) */
    bool cur_static = wm_sta_connected_idx >= 0 &&
                      cfg->sta_addr[wm_sta_connected_idx].is_static;

    if (wm_sta_netif() != NULL && !cur_static && cfg->sta_dns != 0)
    {
        esp_netif_dns_info_t dns = { 0 };

        dns.ip.type = ESP_IPADDR_TYPE_V4;
        dns.ip.u_addr.ip4.addr = htonl(cfg->sta_dns);

        if (esp_netif_set_dns_info(wm_sta_netif(), ESP_NETIF_DNS_MAIN,
                                   &dns) == ESP_OK)
        {
            ESP_LOGI(TAG, "DNS override: " IPSTR,
                     IP2STR(&dns.ip.u_addr.ip4));
        }
    }

    /* Resilience: if DHCP gave no backup DNS, install a public one. */
    if (wm_sta_netif() != NULL)
    {
        esp_netif_dns_info_t dns = { 0 };

        if (esp_netif_get_dns_info(wm_sta_netif(), ESP_NETIF_DNS_BACKUP, &dns)
                == ESP_OK &&
            dns.ip.type == ESP_IPADDR_TYPE_V4 &&
            dns.ip.u_addr.ip4.addr == 0)
        {
            dns.ip.type = ESP_IPADDR_TYPE_V4;
            dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(1, 1, 1, 1);

            if (esp_netif_set_dns_info(wm_sta_netif(), ESP_NETIF_DNS_BACKUP,
                                       &dns) == ESP_OK)
            {
                ESP_LOGI(TAG, "backup DNS missing; set 1.1.1.1");
            }
        }
    }

    /* APSTA housekeeping */
    if (cfg->mode == WM_MODE_APSTA && cfg->ap_auto_disable)
    {
        ESP_LOGI(TAG, "STA up; auto-disabling AP");
        esp_wifi_set_mode(WIFI_MODE_STA);
        return;
    }

    if (cfg->mode == WM_MODE_APSTA)
    {
        /* keep AP on the STA channel to avoid the radio hopping */
        wm_ap_sync_channel_to_sta();

        /* flag STA/AP subnet overlap so the UI can warn */
        esp_netif_ip_info_t ap_ip;

        if (wm_ap_netif() != NULL &&
            esp_netif_get_ip_info(wm_ap_netif(), &ap_ip) == ESP_OK)
        {
            uint32_t sta_ip = event->ip_info.ip.addr;
            uint32_t sta_mask = event->ip_info.netmask.addr;
            bool overlap =
                ((ap_ip.ip.addr & sta_mask) == (sta_ip & sta_mask)) ||
                ((sta_ip & ap_ip.netmask.addr) ==
                 (ap_ip.ip.addr & ap_ip.netmask.addr));

            if (overlap)
            {
                ESP_LOGW(TAG, "STA and AP subnets overlap");
                xEventGroupSetBits(wm_event_group(), WIFI_MANAGER_BIT_STA_AP_OVERLAP);
                dev_status_manager_set(DEV_STATUS_BIT_STA_AP_OVERLAP);
            }
            else
            {
                xEventGroupClearBits(wm_event_group(),
                                     WIFI_MANAGER_BIT_STA_AP_OVERLAP);
                dev_status_manager_clear(DEV_STATUS_BIT_STA_AP_OVERLAP);
            }
        }
    }
}

static void on_sta_disconnected(const wifi_event_sta_disconnected_t *event)
{
    const wm_config_t *cfg = wm_settings_config();
    bool was_connected = wm_status.sta_connected; /* an attempt, or a drop? */
    /* a connection trial's disconnect (its result, or its letting-go)
       belongs to the trial: the status still resets below, the attempt
       memory of the configured networks never hears of it (2026-10-06) */
    bool trial = wm_try_on_disconnected(event != NULL ? event->reason : 0);

    wm_status.sta_connected = false;
    wm_sta_connected_idx = -1;
    wm_sta_connect_started_ms = 0; /* attempt concluded (failed or dropped) */

    xSemaphoreTake(wm_lock, portMAX_DELAY);
    wm_status.sta_ip[0] = '\0';
    xSemaphoreGive(wm_lock);

    xEventGroupClearBits(wm_event_group(), WIFI_MANAGER_BIT_STA_CONNECTED |
                                           WIFI_MANAGER_BIT_STA_AP_OVERLAP);
    xEventGroupSetBits(wm_event_group(), WIFI_MANAGER_BIT_STA_DISCONNECTED);
    dev_status_manager_clear(DEV_STATUS_BIT_STA_CONNECTED |
                             DEV_STATUS_BIT_STA_AP_OVERLAP);

    if (was_connected)
    {
        wm_events_sta(false, ""); /* a real drop, not a failed attempt */
    }

    if (event != NULL && !trial)
    {
        int idx = wm_status.last_attempted_idx;

        wm_status.last_reason = event->reason;
        ESP_LOGD(TAG, "STA disconnected, reason %d (ssid '%s')",
                 event->reason, wm_status.last_attempted_ssid);

        if (wm_sta_roam == WM_ROAM_LEAVING)
        {
            wm_sta_roam = WM_ROAM_TRYING; /* our own leave; the trial follows */
        }
        else if (!was_connected && attempt_failed_reason(event->reason) &&
                 idx >= 0)
        {
            bool was = wm_select_is_deprioritised(&wm_select_state, idx, wm_now_ms());

            if (wm_sta_roam == WM_ROAM_TRYING && idx == wm_sta_roam_target)
            {
                wm_select_deprioritise(&wm_select_state, idx, wm_now_ms());
            }
            else
            {
                wm_select_on_attempt_fail(&wm_select_state, idx, wm_now_ms());
            }

            if (!was && wm_select_is_deprioritised(&wm_select_state, idx, wm_now_ms()))
            {
                ESP_LOGI(TAG, "'%s' failed %u attempts in a row (last "
                              "reason %d): other networks on the list are "
                              "tried first; it is retried whenever it is "
                              "all there is",
                         wm_status.last_attempted_ssid,
                         (unsigned)wm_select_fail_count(&wm_select_state, idx,
                                                        wm_now_ms()),
                         event->reason);
            }

            wm_sta_roam = WM_ROAM_IDLE;
        }
        else if (wm_sta_roam == WM_ROAM_TRYING)
        {
            wm_sta_roam = WM_ROAM_IDLE; /* the trial ended some other way */
        }
    }

    if (s_callbacks.sta_disconnected != NULL)
    {
        s_callbacks.sta_disconnected();
    }

    /* if AP was auto-disabled while STA was up, bring it back (unless the
       interface policy holds it suspended) */
    if (cfg->mode == WM_MODE_APSTA && cfg->ap_auto_disable &&
        !wm_suspend_ap_active())
    {
        wifi_mode_t mode;

        if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA)
        {
            ESP_LOGI(TAG, "STA down; re-enabling AP");
            esp_wifi_set_mode(WIFI_MODE_APSTA);
        }
    }
}

void wm_event_handler(void *arg, esp_event_base_t base,
                      int32_t event_id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_STA_START:
                if (wm_settings_config()->sta_auto_reconnect)
                {
                    wm_sta_select_and_connect();
                }
                break;

            /* NOTE: no sync at WIFI_EVENT_STA_CONNECTED, the driver
               still holds its "connecting" state there and REFUSES
               esp_wifi_set_config with an ERROR log (racy: sometimes
               accepted, usually not, caught by the HIL error budgets
               2026-07-19). The got-ip sync below always lands; the AP
               beacons on the right channel either way (hardware). */

            case WIFI_EVENT_STA_CONNECTED:
                /* only the trial listens here (its address budget starts
                   at the association); see the NOTE above for why nothing
                   else does */
                wm_try_on_associated(
                    ((const wifi_event_sta_connected_t *)data)->channel);
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                on_sta_disconnected(data);
                break;

            case WIFI_EVENT_HOME_CHANNEL_CHANGE:
                /* upstream router CSA moves the STA (and radio) without a
                   reassociation: follow it here too. Only while FULLY
                   connected: the event also fires mid-connect (channel
                   switch precedes association) where esp_wifi_set_config
                   is refused with a driver ERROR log ("sta is
                   connecting"), and boot-clean error counts are
                   load-bearing now (health net). The association case is
                   covered by WIFI_EVENT_STA_CONNECTED above. */
                if (wm_status.sta_connected)
                {
                    wm_ap_sync_channel_to_sta();
                }
                break;

            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "AP started");
                wm_status.ap_started = true;
                xEventGroupSetBits(wm_event_group(), WIFI_MANAGER_BIT_AP_STARTED);
                dev_status_manager_set(DEV_STATUS_BIT_AP_ENABLED);
                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "AP stopped");
                wm_status.ap_started = false;
                wm_status.ap_station_count = 0;
                xEventGroupClearBits(wm_event_group(),
                                     WIFI_MANAGER_BIT_AP_STARTED |
                                     WIFI_MANAGER_BIT_STA_AP_OVERLAP);
                dev_status_manager_clear(DEV_STATUS_BIT_AP_ENABLED |
                                         DEV_STATUS_BIT_STA_AP_OVERLAP);
                break;

            case WIFI_EVENT_AP_STACONNECTED:
            {
                const wifi_event_ap_staconnected_t *e = data;

                wm_status.ap_station_count++;
                ESP_LOGI(TAG, "AP client " MACSTR " joined (%u total)",
                         MAC2STR(e->mac), wm_status.ap_station_count);

                if (s_callbacks.ap_station_connected != NULL)
                {
                    s_callbacks.ap_station_connected(e->mac, e->aid);
                }
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED:
            {
                const wifi_event_ap_stadisconnected_t *e = data;

                if (wm_status.ap_station_count > 0)
                {
                    wm_status.ap_station_count--;
                }

                ESP_LOGI(TAG, "AP client " MACSTR " left (%u total)",
                         MAC2STR(e->mac), wm_status.ap_station_count);

                if (s_callbacks.ap_station_disconnected != NULL)
                {
                    s_callbacks.ap_station_disconnected(e->mac, e->aid);
                }

                /* the AP config catches up with the radio's channel once
                   nobody is on the AP to be kicked by the write */
                if (wm_status.ap_station_count == 0 && wm_ap_channel_stale &&
                    wm_status.sta_connected)
                {
                    wm_ap_sync_channel_to_sta();
                }
                break;
            }

            default:
                break;
        }
    }
    else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        on_sta_got_ip(data);
    }
}

/* ---- the callbacks (wifi_manager_status.c, see wifi_manager_bridge.h) ------ */

void wm_set_callbacks(const wifi_manager_callbacks_t *cbs)
{
    if (cbs != NULL)
    {
        s_callbacks = *cbs;
    }
    else
    {
        memset(&s_callbacks, 0, sizeof(s_callbacks));
    }
}
