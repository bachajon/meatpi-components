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
 * @file ha_webhooks_fail.c
 * @brief Why an address did not take a push (2026-11-22). Every kind of
 *        failure used to read "failed" and `esp_err=ESP_ERR_HTTP_CONNECT`:
 *        a name that was not found, a port nothing listens on, a far end
 *        that closed the TLS handshake, a certificate that was not accepted.
 *        A field report of that day needed the device log and two rounds of
 *        questions to tell them apart. The poster now keeps, per address,
 *        the reason http_client_manager works out (or the answer's status),
 *        a code for support and the time; `GET /api/webhook` hands them out
 *        as `failures` and the web interface says them in plain words.
 */
#include <stdio.h>
#include <string.h>

#include "esp_err.h"

#include "http_client_manager.h"

#include "ha_webhooks_private.h"

_Static_assert(HW_FAIL_CODE_LEN >= HTTP_CLIENT_FAIL_CODE_LEN,
               "a failure's code is as long as http_client_manager's");

/** Why the address in @p slot (1 or 2) did not take the push: the
 *  transport's reason from http_client_manager, or the answer's status. */
void hw_fail_note(hw_fail_t *f, int slot, esp_err_t r, const void *resp_any)
{
    const http_client_response_t *resp = resp_any;

    memset(f, 0, sizeof(*f));
    f->slot = (uint8_t)slot;

    if (r != ESP_OK)
    {
        strlcpy(f->reason,
                http_client_manager_fail_reason_str(
                    http_client_manager_fail_reason(r, &resp->fail)),
                sizeof(f->reason));
        http_client_manager_fail_code(&resp->fail, f->code, sizeof(f->code));

        if (f->code[0] == '\0')
        {
            strlcpy(f->code, esp_err_to_name(r), sizeof(f->code));
        }
    }
    else if (resp->status_code <= 0)
    {
        /* the connection opened and no status line came back in time */
        strlcpy(f->reason, "no_reply", sizeof(f->reason));
        strlcpy(f->code, "no status", sizeof(f->code));
    }
    else
    {
        f->http = (int16_t)resp->status_code;
        strlcpy(f->reason, "http", sizeof(f->reason));
        snprintf(f->code, sizeof(f->code), "http %d", resp->status_code);
    }

    hw_format_utc(f->time);
}

/** The failures of a cycle for the log line: "1 no_answer [esp-tls 0x8006],
 *  2 tls_closed [tls -0x7280]". No address in it: it carries the webhook id. */
void hw_fail_words(const hw_fail_t *fails, int n, char *out, size_t cap)
{
    size_t at = 0;

    out[0] = '\0';

    for (int i = 0; i < n && at < cap; i++)
    {
        int w = snprintf(out + at, cap - at, "%s%u %s [%s]", i ? ", " : "",
                         (unsigned)fails[i].slot, fails[i].reason,
                         fails[i].code);

        if (w < 0)
        {
            break;
        }

        at += (size_t)w;
    }
}
