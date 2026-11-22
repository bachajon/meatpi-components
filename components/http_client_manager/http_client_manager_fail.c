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
 * @file http_client_manager_fail.c
 * @brief Why a request's transport failed, in a word (2026-11-22). PURE:
 *        numbers in, a reason out; no esp-tls, no mbedTLS (host tested).
 *
 * Every connection that does not open comes back from esp_http_client as
 * ESP_ERR_HTTP_CONNECT; what tells a name that was not found from a far end
 * that closed the handshake is kept by esp-tls beside it (its own code, the
 * TLS library's code, the certificate check's flags, the socket's errno).
 * A field report of 2026-11-22 needed the device log and two rounds of
 * questions to read `-0x7280` (the far end closed the handshake): a caller
 * can now say it on its own status.
 */
#include "http_client_manager.h"

#include <stdio.h>

#include "http_client_manager_private.h"

/* The certificate cases (measured on the bench unit, IDF 6.1, 2026-11-22):
 * the built-in bundle that finds no root for the chain ends the handshake
 * with -0x3000; every other "no" of the certificate check is -0x2700, be it
 * a certificate for another name or, with a certificate set, a chain that
 * does not end at the set's CA. What would tell those apart is the check's
 * flags, and esp-tls does not hand them over for a handshake that failed, so
 * they stay one reason. (The check does not look at the dates at all while
 * CONFIG_MBEDTLS_HAVE_TIME_DATE is off, IDF's default and this firmware's:
 * an expired certificate is taken.) */
static http_client_fail_reason_t tls_reason(int code)
{
    switch (code)
    {
        case HC_MBED_CONN_EOF:
        case HC_MBED_NET_RESET:
            return HTTP_CLIENT_FAIL_TLS_CLOSED;

        case HC_MBED_FATAL_ALERT:
            return HTTP_CLIENT_FAIL_TLS_REFUSED;

        case HC_MBED_INVALID_RECORD:
            return HTTP_CLIENT_FAIL_NOT_TLS;

        case HC_MBED_X509_VERIFY:
            return HTTP_CLIENT_FAIL_CERT;

        case HC_MBED_X509_FATAL:
            return HTTP_CLIENT_FAIL_CERT_UNTRUSTED;

        case HC_MBED_SSL_TIMEOUT:
            return HTTP_CLIENT_FAIL_NO_ANSWER;

        default:
            return HTTP_CLIENT_FAIL_TLS_OTHER;
    }
}

http_client_fail_reason_t http_client_manager_fail_reason(
    esp_err_t err, const http_client_fail_t *f)
{
    if (err == ESP_OK)
    {
        return HTTP_CLIENT_FAIL_NONE;
    }

    if (f == NULL)
    {
        return HTTP_CLIENT_FAIL_OTHER;
    }

    if (f->opened)
    {
        return HTTP_CLIENT_FAIL_NO_REPLY;
    }

    if (f->tls_code != 0)
    {
        return tls_reason(f->tls_code);
    }

    switch (f->tls_err)
    {
        case HC_TLS_ERR_RESOLVE:
            return HTTP_CLIENT_FAIL_DNS;

        case HC_TLS_ERR_CONNECT:
        case HC_TLS_ERR_TIMEOUT:
            return HTTP_CLIENT_FAIL_NO_ANSWER;

        case HC_TLS_ERR_HANDSHAKE:
            return HTTP_CLIENT_FAIL_TLS_OTHER;

        default:
            return HTTP_CLIENT_FAIL_OTHER;
    }
}

const char *http_client_manager_fail_reason_str(http_client_fail_reason_t r)
{
    switch (r)
    {
        case HTTP_CLIENT_FAIL_NONE:           return "";
        case HTTP_CLIENT_FAIL_DNS:            return "dns";
        case HTTP_CLIENT_FAIL_NO_ANSWER:      return "no_answer";
        case HTTP_CLIENT_FAIL_TLS_CLOSED:     return "tls_closed";
        case HTTP_CLIENT_FAIL_TLS_REFUSED:    return "tls_refused";
        case HTTP_CLIENT_FAIL_NOT_TLS:        return "not_https";
        case HTTP_CLIENT_FAIL_CERT_UNTRUSTED: return "cert_untrusted";
        case HTTP_CLIENT_FAIL_CERT:           return "cert";
        case HTTP_CLIENT_FAIL_TLS_OTHER:      return "tls";
        case HTTP_CLIENT_FAIL_NO_REPLY:       return "no_reply";
        default:                              return "other";
    }
}

void http_client_manager_fail_code(const http_client_fail_t *f, char *out,
                                   size_t cap)
{
    if (out == NULL || cap == 0)
    {
        return;
    }

    out[0] = '\0';

    if (f == NULL)
    {
        return;
    }

    if (f->tls_code != 0)
    {
        bool cert = f->tls_code == HC_MBED_X509_VERIFY &&
                    f->cert_flags != 0 && f->cert_flags != HC_CERT_NO_RESULT;

        if (cert)
        {
            snprintf(out, cap, "tls -0x%04X flags 0x%X", (unsigned)f->tls_code,
                     (unsigned)f->cert_flags);
        }
        else
        {
            snprintf(out, cap, "tls -0x%04X", (unsigned)f->tls_code);
        }

        return;
    }

    if (f->tls_err != 0 && f->sock_errno != 0)
    {
        snprintf(out, cap, "esp-tls 0x%X errno %d", (unsigned)f->tls_err,
                 f->sock_errno);
    }
    else if (f->tls_err != 0)
    {
        snprintf(out, cap, "esp-tls 0x%X", (unsigned)f->tls_err);
    }
    else if (f->sock_errno != 0)
    {
        snprintf(out, cap, "errno %d", f->sock_errno);
    }
}
