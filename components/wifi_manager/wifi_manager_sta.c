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
 * @file wifi_manager_sta.c
 * @brief The station connect path: one configured network into the driver
 *        (its config and its per-network addressing), the scan and its
 *        scratch, the pick among the candidates and the connect attempt
 *        with its in-flight guard, and what the connection trial
 *        (wifi_manager_try.c) needs of the station.
 */
#include "wifi_manager.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

/* scan scratch, serialized by wm_lock (reconnect task + API scans) */
wifi_ap_record_t wm_scan_records[WM_SCAN_MAX_RECORDS] EXT_RAM_BSS_ATTR;
char wm_scan_present[WM_SCAN_MAX_RECORDS][WM_SSID_LEN] EXT_RAM_BSS_ATTR;

/* ---- STA config helpers --------------------------------------------------- */

/* v6: per-network STA addressing, configure the netif for THIS
   candidate before its connect attempt (each network may live on a
   different LAN, so a fallback carries its own static/DHCP choice).
   esp_netif still raises IP_EVENT_STA_GOT_IP for a static netif when
   the link comes up, so the connect state machine is unchanged. */
static void apply_sta_addressing(const wm_config_t *wcfg, size_t idx)
{
    const wm_addr_t *a = &wcfg->sta_addr[idx];

    if (wm_sta_netif() == NULL)
    {
        return;
    }

    if (a->is_static)
    {
        esp_netif_ip_info_t ip = { 0 };

        ip.ip.addr = htonl(a->ip);
        ip.netmask.addr = htonl(a->netmask);
        ip.gw.addr = htonl(a->gw);

        esp_netif_dhcpc_stop(wm_sta_netif());
        esp_netif_set_ip_info(wm_sta_netif(), &ip);

        /* static addressing loses the DHCP-provided DNS: the global
           override wins, else this network's gateway (the got-ip
           handler still installs the 1.1.1.1 backup) */
        uint32_t dns_host = wcfg->sta_dns ? wcfg->sta_dns : a->gw;

        if (dns_host != 0)
        {
            esp_netif_dns_info_t dns = { 0 };

            dns.ip.type = ESP_IPADDR_TYPE_V4;
            dns.ip.u_addr.ip4.addr = htonl(dns_host);
            esp_netif_set_dns_info(wm_sta_netif(), ESP_NETIF_DNS_MAIN, &dns);
        }

        ESP_LOGI(TAG, "STA static IP " IPSTR " for '%s'",
                 IP2STR(&ip.ip), wcfg->sta[idx].ssid);
    }
    else
    {
        /* back to DHCP: drop any stale static address, restart the
           client (ALREADY_STARTED is the common no-op) */
        esp_netif_dhcp_status_t st;

        if (esp_netif_dhcpc_get_status(wm_sta_netif(), &st) == ESP_OK &&
            st == ESP_NETIF_DHCP_STOPPED)
        {
            esp_netif_ip_info_t zero = { 0 };

            esp_netif_set_ip_info(wm_sta_netif(), &zero);
        }

        (void)esp_netif_dhcpc_start(wm_sta_netif());
    }
}

/* the entry the station was on, and its channel, when a connection trial
   made it let go (wm_sta_note_rejoin_hint): the one re-join after the
   trial goes straight back to that entry, no scan of our own, and the
   driver scans that channel first and stops there, so the phone on our
   access point sees one announced hop and no second silence while the
   radio walks all thirteen channels (bench 2026-10-08: that silence, not
   the announced switch, is what a phone drops on) */
static volatile uint8_t s_rejoin_hint;
static volatile int     s_rejoin_idx = -1;

/* `first_channel` 1..13: scan it first and stop there (one association
   only; a mesh may land on a weaker node for it, roaming sorts that out);
   0: all channels, by signal */
esp_err_t wm_sta_apply_network(const wm_config_t *wcfg, size_t idx,
                               uint8_t first_channel)
{
    const wm_network_t *net = &wcfg->sta[idx];
    wifi_config_t cfg = { 0 };

    cfg.sta.threshold.authmode =
        (net->password[0] != '\0') ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    cfg.sta.rm_enabled = 1;
    cfg.sta.btm_enabled = 1;
    cfg.sta.channel = first_channel;
    cfg.sta.scan_method = (first_channel != 0) ? WIFI_FAST_SCAN
                                               : WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.pmf_cfg.required = false;

    strncpy((char *)cfg.sta.ssid, net->ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, net->password,
            sizeof(cfg.sta.password) - 1);

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);

    if (err == ESP_OK)
    {
        apply_sta_addressing(wcfg, idx);
        strncpy(wm_status.last_attempted_ssid, net->ssid, WM_SSID_LEN - 1);
        wm_status.last_attempted_ssid[WM_SSID_LEN - 1] = '\0';
        wm_status.last_attempted_idx = (int)idx;
    }

    return err;
}

/** Blocking scan into wm_scan_records (caller holds wm_lock). Returns count. */
uint16_t wm_do_scan(void)
{
    wifi_scan_config_t scan_cfg =
    {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    esp_err_t err = ESP_FAIL;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        err = esp_wifi_scan_start(&scan_cfg, true);

        if (err == ESP_OK)
        {
            break;
        }

        if (err != ESP_ERR_WIFI_STATE)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(200 + attempt * 200));
    }

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "scan failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t count = WM_SCAN_MAX_RECORDS;

    if (esp_wifi_scan_get_ap_records(&count, wm_scan_records) != ESP_OK)
    {
        return 0;
    }

    return count;
}

/** Temporarily ensure STA is available for scanning in AP-only mode. */
static bool enter_scan_mode(void)
{
    wifi_mode_t mode;

    if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_AP)
    {
        if (esp_wifi_set_mode(WIFI_MODE_APSTA) == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            return true;
        }
    }

    return false;
}

static void leave_scan_mode(bool switched)
{
    if (switched)
    {
        esp_wifi_set_mode(WIFI_MODE_AP);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* connect-in-flight guard (2026-07-19, caught by the HIL error budgets):
 * the reconnect task's 5 s cadence re-selected and re-applied the STA
 * config while the PREVIOUS esp_wifi_connect was still pending, the
 * driver refuses with an ERROR log ("sta is connecting, cannot set
 * config") and the manager retried, self-healing but noisy on every
 * multi-candidate walk. Cleared by got-ip / disconnected; the timeout
 * unsticks a driver attempt that never reports back. */
#define WM_CONNECT_INFLIGHT_MS 15000

int64_t wm_sta_connect_started_ms;

/* One INFO line per transition when the pick is an entry that has been
 * rejecting us and nothing better is visible ("keep trying: best
 * chance"): never per cycle (§10 hot path). */
bool wm_sta_last_resort;

static void note_last_resort(int idx)
{
    const wm_config_t *cfg = wm_settings_config();
    bool now_last = wm_select_is_deprioritised(&wm_select_state, idx, wm_now_ms());

    if (now_last && !wm_sta_last_resort)
    {
        ESP_LOGI(TAG, "no other network available: keeping on with '%s' "
                      "(%u failed attempts lately)",
                 cfg->sta[idx].ssid,
                 (unsigned)wm_select_fail_count(&wm_select_state, idx, wm_now_ms()));
    }

    wm_sta_last_resort = now_last;
}

/* ---- the connection trial's view of the station (wifi_manager_try.c) ---- */

bool wm_sta_attempt_in_flight(void)
{
    return wm_sta_connect_started_ms != 0 &&
           wm_now_ms() - wm_sta_connect_started_ms < WM_CONNECT_INFLIGHT_MS;
}

void wm_sta_note_rejoin_hint(void)
{
    wifi_ap_record_t ap = { 0 };
    bool on = wm_status.sta_connected &&
              esp_wifi_sta_get_ap_info(&ap) == ESP_OK;

    s_rejoin_hint = on ? ap.primary : 0;
    s_rejoin_idx = on ? wm_sta_connected_idx : -1;
}

/* the trial's credentials into the driver, DHCP addressing, and no entry
   index: nothing the attempt memory could charge. `channel` is the scan
   row's when the wizard knows it (2026-10-08): the station scans that
   channel first and stops there, about one dwell instead of the thirteen
   during which the access point cannot beacon; 0 keeps the full scan (a
   name typed by hand). */
esp_err_t wm_sta_apply_trial(const char *ssid, const char *password,
                             uint8_t channel)
{
    wifi_config_t cfg = { 0 };

    cfg.sta.threshold.authmode =
        (password[0] != '\0') ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    cfg.sta.channel = channel;
    cfg.sta.scan_method = (channel != 0) ? WIFI_FAST_SCAN
                                         : WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.pmf_cfg.required = false;
    strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, password, sizeof(cfg.sta.password) - 1);

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);

    if (err == ESP_OK && wm_sta_netif() != NULL)
    {
        esp_netif_dhcp_status_t st;

        if (esp_netif_dhcpc_get_status(wm_sta_netif(), &st) == ESP_OK &&
            st == ESP_NETIF_DHCP_STOPPED)
        {
            esp_netif_ip_info_t zero = { 0 };

            esp_netif_set_ip_info(wm_sta_netif(), &zero);
        }

        (void)esp_netif_dhcpc_start(wm_sta_netif());
        wm_status.last_attempted_idx = -1;
        strncpy(wm_status.last_attempted_ssid, ssid, WM_SSID_LEN - 1);
        wm_status.last_attempted_ssid[WM_SSID_LEN - 1] = '\0';
    }

    return err;
}

/* after a trial: the configured primary back in the driver (the reconnect
   task re-applies the pick it makes anyway), or an empty config when no
   network is configured, so the trial's credentials are gone */
void wm_sta_restore_config(void)
{
    const wm_config_t *cfg = wm_settings_config();

    if (cfg != NULL && cfg->sta_count > 0)
    {
        (void)wm_sta_apply_network(cfg, 0, 0);
    }
    else
    {
        wifi_config_t empty = { 0 };

        (void)esp_wifi_set_config(WIFI_IF_STA, &empty);
        wm_status.last_attempted_idx = -1;
        wm_status.last_attempted_ssid[0] = '\0';
    }

    wm_sta_connect_started_ms = 0;
}

/* one entry into the driver and a connect attempt, no scan of our own */
static bool connect_entry(const wm_config_t *cfg, size_t idx,
                          uint8_t first_channel)
{
    wm_sta_apply_network(cfg, idx, first_channel);

    /* stamp BEFORE the call: a fast DISCONNECTED (e.g. NO_AP_FOUND
     * from the driver's cached scan) can land in the event task and
     * clear the guard before esp_wifi_connect() even returns: the
     * late stamp then blocked the next WM_CONNECT_INFLIGHT_MS worth
     * of cycles per attempt (bench 2026-08-22, ESPNetLink AP coming
     * up ~10 s after the WiCAN's first attempt) */
    wm_sta_connect_started_ms = wm_now_ms();

    if (esp_wifi_connect() != ESP_OK)
    {
        wm_sta_connect_started_ms = 0;
    }

    return true;
}

/** Pick the best visible candidate (or rotate blindly) and connect. */
/** @return true when a connect attempt was actually issued (the retry
 *  counter must only advance on real attempts, not deferred cycles). */
bool wm_sta_select_and_connect(void)
{
    const wm_config_t *cfg = wm_settings_config();

    if (cfg->sta_count == 0)
    {
        return false;
    }

    if (wm_sta_connect_started_ms != 0 &&
        wm_now_ms() - wm_sta_connect_started_ms < WM_CONNECT_INFLIGHT_MS)
    {
        ESP_LOGD(TAG, "connect attempt still in flight; deferring");
        return false;
    }

    if (wm_try_active())
    {
        return false; /* a connection trial has the station */
    }

    /* the entry a connection trial made the station leave, and its
       channel: one shot, taken whether or not it is usable */
    int     rejoin = s_rejoin_idx;
    uint8_t first = s_rejoin_hint;

    s_rejoin_idx = -1;
    s_rejoin_hint = 0;

    if (rejoin >= 0 && rejoin < (int)cfg->sta_count)
    {
        ESP_LOGD(TAG, "back to '%s' after the trial, channel %u first",
                 cfg->sta[rejoin].ssid, first);
        note_last_resort(rejoin);
        return connect_entry(cfg, (size_t)rejoin, first);
    }

    if (cfg->sta_count == 1)
    {
        /* single network: no scan, and no deferral either, a network
         * that keeps rejecting us is still our best chance (a wrong
         * password here may be right back home; meatpi 2026-09-06) */
        (void)wm_select_sequential(&wm_select_state, cfg->sta, cfg->sta_count,
                                   wm_now_ms());
        note_last_resort(0);
        return connect_entry(cfg, 0, 0);
    }

    xSemaphoreTake(wm_lock, portMAX_DELAY);

    bool     switched = enter_scan_mode();
    uint16_t found = wm_do_scan();
    int      pick = -1;

    if (found > 0)
    {
        for (uint16_t i = 0; i < found; i++)
        {
            strncpy(wm_scan_present[i], (const char *)wm_scan_records[i].ssid,
                    WM_SSID_LEN - 1);
            wm_scan_present[i][WM_SSID_LEN - 1] = '\0';
        }

        pick = wm_select_from_scan(&wm_select_state, cfg->sta, cfg->sta_count,
                                   wm_scan_present, found, wm_now_ms());
    }

    if (pick < 0)
    {
        /* scan failed OR nothing visible matched: rotate blind,
         * hidden SSIDs never appear in scan results (their records
         * carry an empty name) and dense airspace can truncate the
         * record table past a configured network */
        pick = wm_select_sequential(&wm_select_state, cfg->sta, cfg->sta_count,
                                    wm_now_ms());

        if (pick >= 0)
        {
            ESP_LOGD(TAG, "nothing visible matched; blind attempt on "
                          "'%s' (hidden?)", cfg->sta[pick].ssid);
        }
    }

    leave_scan_mode(switched);
    xSemaphoreGive(wm_lock);

    if (pick >= 0 && wm_try_active())
    {
        return false; /* a trial began during the scan: it has the station */
    }

    if (pick >= 0)
    {
        /* DEBUG: repeats every retry cycle during an outage (§10 hot path) */
        ESP_LOGD(TAG, "connecting to candidate %d: %s", pick,
                 cfg->sta[pick].ssid);
        note_last_resort(pick);
        return connect_entry(cfg, (size_t)pick, 0);
    }

    ESP_LOGD(TAG, "no eligible network this cycle; deferring connect");
    return false;
}

/* ---- the scan for wifi_manager_status.c (see wifi_manager_bridge.h) -------- */

static bool s_scan_switched; /* AP->APSTA restore flag, guarded by wm_lock */

esp_err_t wm_scan_locked(wifi_ap_record_t **records, uint16_t *count)
{
    if (!wm_status.started || !wm_status.radio_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(wm_lock, portMAX_DELAY);
    s_scan_switched = enter_scan_mode();

    uint16_t found = wm_do_scan();

    *records = wm_scan_records;
    *count = found;
    return ESP_OK; /* caller releases via wm_scan_unlock() */
}

void wm_scan_unlock(void)
{
    leave_scan_mode(s_scan_switched);
    s_scan_switched = false;
    xSemaphoreGive(wm_lock);
}
