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
 * @file wifi_manager_network.c
 * @brief The network the station is on: its name, its "Pause VPN" flag and
 *        the network-trust admission gate (for inbound requests and for
 *        the device's own connections), all answered from the connected
 *        candidate's index and the boot-applied config.
 */
#include "wifi_manager.h"

#include <string.h>

#include "esp_netif.h"
#include "lwip/sockets.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

/* ---- "Pause VPN" per network (issue #976, 2026-10-11) ---------------------
 * vpn_manager asks on its one-second lap whether the station is on a
 * network the user marked; the answer is the connected candidate's flag
 * (the same index the trust gate uses), false without a station session. */
bool wifi_manager_sta_vpn_pause(void)
{
    const wm_config_t *cfg = wm_settings_config();

    return wm_vpn_pause_applies(cfg->vpn_pause, cfg->sta_count,
                                wm_sta_connected_idx);
}

esp_err_t wifi_manager_get_sta_ssid(char *buf, size_t buf_len)
{
    int idx = wm_sta_connected_idx;
    const wm_config_t *cfg = wm_settings_config();

    if (buf == NULL || buf_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    buf[0] = '\0';

    if (idx < 0 || (size_t)idx >= cfg->sta_count)
    {
        return ESP_ERR_INVALID_STATE;
    }

    strlcpy(buf, cfg->sta[idx].ssid, buf_len);
    return ESP_OK;
}

/* ---- network-trust admission gate (meatpi 2026-07-08) ---------------------
 * Untrusted (shared/office) networks must not expose configuration: when
 * the CURRENT STA network is marked untrusted, inbound admin requests
 * that arrived VIA the STA address are refused. The device's own AP and
 * the USB link classify as non-STA and keep full admin: that is the
 * recovery path. Outbound clients (autopid HTTP posts, MQTT) don't go
 * through httpd and are unaffected. Wired into http_server_manager's
 * request gate by main (composition root). */
bool wifi_manager_http_request_allowed(int sockfd)
{
    int idx = wm_sta_connected_idx;
    const wm_config_t *cfg = wm_settings_config();

    if (idx < 0 || (size_t)idx >= cfg->sta_count || cfg->trusted[idx])
    {
        return true;            /* no STA session, or a trusted network */
    }

    if (wm_sta_netif() == NULL)
    {
        return true;
    }

    esp_netif_ip_info_t ip;

    if (esp_netif_get_ip_info(wm_sta_netif(), &ip) != ESP_OK ||
        ip.ip.addr == 0)
    {
        return true;            /* no STA address to arrive on          */
    }

    struct sockaddr_storage local;
    socklen_t len = sizeof(local);

    if (getsockname(sockfd, (struct sockaddr *)&local, &len) != 0)
    {
        return false;           /* can't classify on an untrusted net:
                                   fail CLOSED                          */
    }

    if (local.ss_family == AF_INET)
    {
        const struct sockaddr_in *a = (const struct sockaddr_in *)&local;

        return a->sin_addr.s_addr != ip.ip.addr;
    }

#if LWIP_IPV6
    if (local.ss_family == AF_INET6)
    {
        /* lwIP dual-stack accepts v4 clients as v4-mapped v6 */
        const struct sockaddr_in6 *a6 =
            (const struct sockaddr_in6 *)&local;
        const uint8_t *b = (const uint8_t *)&a6->sin6_addr;
        static const uint8_t V4MAP[12] =
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF };

        if (memcmp(b, V4MAP, 12) == 0)
        {
            uint32_t v4;

            memcpy(&v4, b + 12, 4);
            return v4 != ip.ip.addr;
        }

        return false;           /* native v6 on untrusted STA: closed   */
    }
#endif

    return false;
}

/* The same question for a connection the device opened itself (2026-11-14):
 * did it leave through the station's address while the station's network is
 * marked untrusted? Outbound traffic is not stopped by the gate above, but
 * what comes back on it may not reconfigure the device: ha_webhooks takes
 * no settings from the reply to such a push. */
bool wifi_manager_addr_on_untrusted_sta(uint32_t local_ip4)
{
    int idx = wm_sta_connected_idx;
    const wm_config_t *cfg = wm_settings_config();

    if (idx < 0 || (size_t)idx >= cfg->sta_count || cfg->trusted[idx])
    {
        return false;           /* no STA session, or a trusted network */
    }

    if (local_ip4 == 0)
    {
        return true;            /* can't classify on an untrusted net:
                                   fail CLOSED                          */
    }

    esp_netif_ip_info_t ip;

    if (wm_sta_netif() == NULL ||
        esp_netif_get_ip_info(wm_sta_netif(), &ip) != ESP_OK ||
        ip.ip.addr == 0)
    {
        return false;           /* no STA address to leave through      */
    }

    return local_ip4 == ip.ip.addr;
}
