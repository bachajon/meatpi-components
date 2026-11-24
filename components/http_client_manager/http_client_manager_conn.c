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
 * @file http_client_manager_conn.c
 * @brief What became of a request's connection, read from the client while
 *        it still exists. Why it failed (2026-11-22, hc_fail_read): what
 *        esp-tls kept about a connection that did not open; the reason is
 *        worked out by the pure http_client_manager_fail.c. And where a
 *        request went (2026-11-14, hc_conn_read): the two ends of the open
 *        connection, read from its socket. The response carries whether the
 *        server's address is a private one and which of the device's own
 *        addresses the request left from; ha_webhooks decides with them
 *        whether a push's reply may change the webhook settings. The
 *        classification itself is pure (hc_ip4_private, hc_ip6_private:
 *        http_client_manager_auth.c, host tested).
 */
#include "http_client_manager.h"

#include <string.h>

#include "esp_tls_errors.h"
#include "lwip/sockets.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl.h"
#include "mbedtls/x509.h"

#include "http_client_manager_engine.h"
#include "http_client_manager_private.h"

/* The numbers http_client_manager_fail.c works a reason out of are the
   libraries' own: a library that moves one stops the build here. */
_Static_assert(HC_TLS_ERR_RESOLVE == ESP_ERR_ESP_TLS_CANNOT_RESOLVE_HOSTNAME,
               "esp-tls code");
_Static_assert(HC_TLS_ERR_CONNECT == ESP_ERR_ESP_TLS_FAILED_CONNECT_TO_HOST,
               "esp-tls code");
_Static_assert(HC_TLS_ERR_TIMEOUT == ESP_ERR_ESP_TLS_CONNECTION_TIMEOUT,
               "esp-tls code");
_Static_assert(HC_TLS_ERR_HANDSHAKE == ESP_ERR_MBEDTLS_SSL_HANDSHAKE_FAILED,
               "esp-tls code");
_Static_assert(HC_MBED_NET_RESET == -(MBEDTLS_ERR_NET_CONN_RESET), "TLS code");
_Static_assert(HC_MBED_X509_VERIFY == -(MBEDTLS_ERR_X509_CERT_VERIFY_FAILED),
               "TLS code");
_Static_assert(HC_MBED_X509_FATAL == -(MBEDTLS_ERR_X509_FATAL_ERROR),
               "TLS code");
_Static_assert(HC_MBED_SSL_TIMEOUT == -(MBEDTLS_ERR_SSL_TIMEOUT), "TLS code");
_Static_assert(HC_MBED_INVALID_RECORD == -(MBEDTLS_ERR_SSL_INVALID_RECORD),
               "TLS code");
_Static_assert(HC_MBED_CONN_EOF == -(MBEDTLS_ERR_SSL_CONN_EOF), "TLS code");
_Static_assert(HC_MBED_FATAL_ALERT == -(MBEDTLS_ERR_SSL_FATAL_ALERT_MESSAGE),
               "TLS code");

void hc_fail_read(esp_http_client_handle_t client, esp_err_t err,
                  http_client_fail_t *out)
{
    int code = 0;
    int flags = 0;

    memset(out, 0, sizeof(*out));
    /* esp-tls's word for the connection (also for a plain http one: the
       resolve and the connect are its code), the TLS library's and the
       certificate check's; the call clears them */
    out->tls_err = (int)esp_http_client_get_and_clear_last_tls_error(
        client, &code, &flags);
    out->tls_code = code;
    out->cert_flags = (uint32_t)flags;

    int e = esp_http_client_get_errno(client);

    out->sock_errno = (e > 0) ? e : 0;
    /* every connection that did not open is ESP_ERR_HTTP_CONNECT; any
       other failure came after it was open */
    out->opened = err != ESP_ERR_HTTP_CONNECT;
}

void hc_conn_read(esp_http_client_handle_t client, hc_conn_t *out)
{
    memset(out, 0, sizeof(*out));

    int fd = esp_http_client_get_socket(client);

    if (fd < 0)
    {
        return;
    }

    struct sockaddr_storage peer;
    struct sockaddr_storage local;
    socklen_t plen = sizeof(peer);
    socklen_t llen = sizeof(local);

    if (getpeername(fd, (struct sockaddr *)&peer, &plen) != 0 ||
        getsockname(fd, (struct sockaddr *)&local, &llen) != 0)
    {
        return;
    }

    if (peer.ss_family == AF_INET)
    {
        const struct sockaddr_in *a = (const struct sockaddr_in *)&peer;

        out->peer_private = hc_ip4_private((const uint8_t *)&a->sin_addr);
    }

#if LWIP_IPV6
    else if (peer.ss_family == AF_INET6)
    {
        const struct sockaddr_in6 *a6 = (const struct sockaddr_in6 *)&peer;

        out->peer_private = hc_ip6_private((const uint8_t *)&a6->sin6_addr);
    }

#endif
    else
    {
        return;
    }

    if (local.ss_family == AF_INET)
    {
        const struct sockaddr_in *a = (const struct sockaddr_in *)&local;

        out->local_ip4 = a->sin_addr.s_addr;
    }

#if LWIP_IPV6
    else if (local.ss_family == AF_INET6)
    {
        /* lwIP dual-stack: an IPv4 connection on a v6 socket is v4-mapped */
        const struct sockaddr_in6 *a6 = (const struct sockaddr_in6 *)&local;
        const uint8_t *b = (const uint8_t *)&a6->sin6_addr;
        static const uint8_t V4MAP[12] =
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF };

        if (memcmp(b, V4MAP, sizeof(V4MAP)) == 0)
        {
            memcpy(&out->local_ip4, b + 12, 4);
        }
    }

#endif

    out->known = true;
}
