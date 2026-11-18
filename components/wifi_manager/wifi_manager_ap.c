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
 * @file wifi_manager_ap.c
 * @brief The access point side: the softAP config with its security mode,
 *        gateway address and DHCP server, the AP forced up for config mode
 *        (wifi_manager_config_ap) and the AP channel that follows the
 *        station's.
 */
#include "wifi_manager.h"

#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

/* ---- AP config helpers ---------------------------------------------------- */

/** Resolve the effective softAP authmode from the config's ap_auth
 *  selection. AUTO keeps the historic behaviour (WPA2 with a password,
 *  OPEN without); an explicit encrypted mode with no password was already
 *  rejected by the settings validator, but degrade to OPEN defensively. */
static wifi_auth_mode_t ap_authmode(const wm_config_t *cfg)
{
    bool has_pw = cfg->ap.password[0] != '\0';

    switch (cfg->ap_auth)
    {
        case WM_AP_AUTH_OPEN:     return WIFI_AUTH_OPEN;
        case WM_AP_AUTH_WPA2:     return has_pw ? WIFI_AUTH_WPA2_PSK
                                                : WIFI_AUTH_OPEN;
        case WM_AP_AUTH_WPA2WPA3: return has_pw ? WIFI_AUTH_WPA2_WPA3_PSK
                                                : WIFI_AUTH_OPEN;
        case WM_AP_AUTH_WPA3:     return has_pw ? WIFI_AUTH_WPA3_PSK
                                                : WIFI_AUTH_OPEN;
        case WM_AP_AUTH_AUTO:
        default:                  return has_pw ? WIFI_AUTH_WPA2_PSK
                                                : WIFI_AUTH_OPEN;
    }
}

/* The single radio parks on the associated AP's channel, so in APSTA the
   softAP beacons move there no matter what its config says. Mirror the real
   channel into the AP config so it survives a STA drop (no hop back to the
   stale configured channel) and so status/scan behaviour stay consistent.
   The STA config's own channel field is a scan hint we never set: the
   associated AP record is the authoritative source.
   Never while somebody is on the AP (2026-10-08): writing the AP config
   restarts the soft AP and deauthenticates its stations (reason 2, 2 ms
   after the station's got-ip on the bench: the phone on the access point
   was kicked at every successful Quick Setup WiFi test). The radio already
   beacons on the station's channel; the config catches up when the last
   client leaves. */
bool wm_ap_channel_stale;

void wm_ap_sync_channel_to_sta(void)
{
    wifi_mode_t mode;

    if (esp_wifi_get_mode(&mode) != ESP_OK || mode != WIFI_MODE_APSTA)
    {
        return;
    }

    wifi_ap_record_t ap_info;

    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK || ap_info.primary == 0)
    {
        return; /* not associated; nothing to follow */
    }

    wifi_config_t ap_cfg;

    if (esp_wifi_get_config(WIFI_IF_AP, &ap_cfg) == ESP_OK &&
        ap_cfg.ap.channel != ap_info.primary)
    {
        if (wm_status.ap_station_count > 0)
        {
            ESP_LOGD(TAG, "AP config stays on channel %d while %u clients "
                          "are on it (the radio is on %d)",
                     ap_cfg.ap.channel, wm_status.ap_station_count,
                     ap_info.primary);
            wm_ap_channel_stale = true;
            return;
        }

        wm_ap_channel_stale = false;
        ESP_LOGI(TAG, "moving AP to STA channel %d", ap_info.primary);
        ap_cfg.ap.channel = ap_info.primary;
        esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    }
}

/** Apply the boot-applied AP config to the (already created) AP netif:
 *  softAP config, gateway IP, DHCP server. Shared by the normal start
 *  path and wifi_manager_config_ap() (the button-manager hook). */
esp_err_t wm_ap_apply_side(const wm_config_t *cfg)
{
    wifi_config_t ap_cfg = { 0 };

    strncpy((char *)ap_cfg.ap.ssid, cfg->ap.ssid,
            sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, cfg->ap.password,
            sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len = strlen(cfg->ap.ssid);
    ap_cfg.ap.channel = cfg->ap_channel;
    ap_cfg.ap.max_connection = cfg->ap_max_connections;
    ap_cfg.ap.ssid_hidden = cfg->ap_hidden ? 1 : 0;
    ap_cfg.ap.authmode = ap_authmode(cfg);
    ap_cfg.ap.beacon_interval = 100;

    /* WPA3-SAE (and the WPA2/WPA3 transitional mix) need PMF; the
       hash-to-element PWE interops with modern clients */
    if (ap_cfg.ap.authmode == WIFI_AUTH_WPA3_PSK ||
        ap_cfg.ap.authmode == WIFI_AUTH_WPA2_WPA3_PSK)
    {
        ap_cfg.ap.pmf_cfg.capable = true;
        ap_cfg.ap.pmf_cfg.required =
            (ap_cfg.ap.authmode == WIFI_AUTH_WPA3_PSK);
        ap_cfg.ap.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    }

    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);

    if (err != ESP_OK)
    {
        return err;
    }

    esp_netif_ip_info_t ip = { 0 };
    /* AP gateway IP (v4, configurable). A blank/invalid setting parsed
       to 0 in on_apply; fall back to the legacy WiCAN 192.168.0.10:
       the classic ELM327-WiFi-adapter address that OBD apps expect
       (192.168.0.10:35000; meatpi 2026-07-18 defaults pass). A /24 is
       assumed; the DHCP pool follows the gateway automatically. */
    uint32_t host =
        cfg->ap_ip ? cfg->ap_ip : (uint32_t)0xC0A8000A; /* 192.168.0.10 */

    ip.ip.addr = htonl(host);
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);

    esp_netif_dhcps_stop(wm_ap_netif());
    esp_netif_set_ip_info(wm_ap_netif(), &ip);
    esp_netif_dhcps_start(wm_ap_netif());
    return ESP_OK;
}

esp_err_t wifi_manager_config_ap(void)
{
    /* the ONE sanctioned runtime mode change (meatpi 2026-07-19: button
       long-press = "let me configure the device"): bring the AP up with
       the boot-applied AP config regardless of the running mode. The
       reverse direction is deliberately absent: config mode ends with
       a reboot back into the configured mode (reboot-to-apply, §4.2). */
    const wm_config_t *cfg = wm_settings_config();

    if (!wm_status.started)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (wm_status.ap_started)
    {
        return ESP_OK; /* already serving: nothing to do */
    }

    wifi_mode_t mode = WIFI_MODE_AP;

    if (esp_wifi_get_mode(&mode) == ESP_OK &&
        (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA))
    {
        mode = WIFI_MODE_APSTA; /* keep the STA link alive */
    }
    else
    {
        mode = WIFI_MODE_AP;
    }

    esp_err_t err = esp_wifi_set_mode(mode);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "config_ap: set_mode failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = wm_ap_apply_side(cfg);

    if (err == ESP_OK)
    {
        esp_wifi_set_bandwidth(WIFI_IF_AP,
                               cfg->ap_ht40 ? WIFI_BW40 : WIFI_BW20);
        ESP_LOGI(TAG, "config mode: AP forced up ('%s')", cfg->ap.ssid);
    }

    return err;
}
