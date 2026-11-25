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
 * @file uds_transport_at.c
 * @brief The AT-hex transaction behind the obd_chip transport. Its pure
 *        half (the parser of the chip's answer, the line counts, the wait
 *        behind a cut answer) is uds_at_parse.c.
 *
 * Per target: set the protocol (ATTP6/7 at 500k, ATTP8/9 at 250k,
 * RAM-only), tx header (ATSH /
 * ATCP+ATSH for 29-bit), rx filter (ATCRA), headers-off + auto-formatting
 * so the chip does ISO-TP and hands us the pure UDS payload, and the
 * chip's own reply wait (ATST) from p2. The setup goes out when the
 * caller says so (uds_at_transceive_ex skip_setup=false): the MIC is
 * shared with autopid, which re-addresses it between our transactions;
 * the obd_chip transport detects that with the driver's tx_bytes counter
 * and skips the setup in a steady conversation (one request = one AT
 * line). The transaction itself is held with obd_chip_txn_begin().
 */
#include "uds_transport.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uds_proto.h"

static const char *TAG = "uds_manager";

/* the chip's reply as it prints it: a line of 25 characters per frame
 * (`1: XX XX XX XX XX XX XX` and its end), so the largest answer, 4095 bytes
 * in 586 frames, is 14.7 KB of text (bench 2026-09-08). As large as the
 * driver's own accumulator: 8192 here held about 2290 bytes of answer, and
 * one of 4095 ended in ESP_ERR_INVALID_SIZE (bench 2026-11-26). PSRAM,
 * not the 4 KB httpd worker stack; one transaction at a time (the caller's
 * chip hold) */
#define UDS_AT_RBUF 16384
EXT_RAM_BSS_ATTR static char s_rbuf[UDS_AT_RBUF];

/* one AT line on the chip: 64 request bytes (the route caps there too) */
#define UDS_AT_REQ_MAX 64

/* the bitrate the target setup pins: 500 kbit/s unless the caller measured
   the bus at 250 (uds_at_set_can_kbps) */
static uint16_t s_can_kbps = 500;

void uds_at_set_can_kbps(uint16_t kbps)
{
    s_can_kbps = (kbps == 250) ? 250 : 500;
}

char uds_at_protocol(bool ext_id)
{
    /* ISO 15765-4: 6 = 11-bit 500k, 7 = 29-bit 500k, 8 = 11-bit 250k,
       9 = 29-bit 250k */
    if (s_can_kbps == 250)
    {
        return ext_id ? '9' : '8';
    }

    return ext_id ? '7' : '6';
}

/* ---- shared AT transaction -------------------------------------------------- */

/* The target setup behind the protocol, the header and the filter. First how
 * the answer is printed: headers off, ISO-TP formatting on, spaces on. Then
 * what ANOTHER requester may have left on the chip, an AutoPID row's init or
 * an app (autopid puts the same back for its own requests, autopid_chain.c).
 * Bench 2026-11-26: under a row's `ATFCSH710;ATFCSD300000;ATFCSM1` the chip
 * sent its flow control to 0x710, the ECU on 7E0 sent its first frame and
 * nothing more, and no answer of more than one frame came back. Not sent:
 * ATL0 and ATD0, which change the print in ways the parser reads anyway. */
static const char *const SETUP_REST[] =
{
    "ATH0", "ATCAF1", "ATS1",
    "ATCEA",   /* no extended address byte in front of the request   */
    "ATFCSM0", /* flow control from the chip, to the request's header */
    "ATCFC1",  /* ... and sent at all                                 */
    "ATAT1",   /* the default adaptive timing                         */
    "ATV0",    /* requests padded to 8 bytes                          */
    "ATE0",    /* no echo of the request in front of the answer       */
    "ATR1",    /* the answers are printed                             */
};

static void hex_no_space(const uint8_t *b, size_t len, char *out, size_t cap)
{
    static const char H[] = "0123456789ABCDEF";
    size_t o = 0;

    for (size_t i = 0; i < len && o + 2 < cap; i++)
    {
        out[o++] = H[b[i] >> 4];
        out[o++] = H[b[i] & 0x0F];
    }

    out[o] = '\0';
}

esp_err_t uds_at_transceive(uds_at_request_fn req_fn,
                            const uds_addr_t *addr,
                            const uint8_t *req, size_t req_len,
                            uint8_t *resp, size_t resp_cap, size_t *resp_len,
                            uint32_t p2_ms, uint32_t p2star_ms,
                            uint8_t *pending_out)
{
    return uds_at_transceive_ex(req_fn, addr, req, req_len, resp, resp_cap,
                                resp_len, p2_ms, p2star_ms, pending_out,
                                false);
}

esp_err_t uds_at_transceive_ex(uds_at_request_fn req_fn,
                               const uds_addr_t *addr,
                               const uint8_t *req, size_t req_len,
                               uint8_t *resp, size_t resp_cap,
                               size_t *resp_len, uint32_t p2_ms,
                               uint32_t p2star_ms, uint8_t *pending_out,
                               bool skip_setup)
{
    if (req_fn == NULL || addr == NULL || req == NULL || req_len == 0 ||
        resp == NULL || resp_len == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (req_len > UDS_AT_REQ_MAX)
    {
        ESP_LOGW(TAG, "request of %u B: the chip takes one AT line of %d B "
                      "(use the native ISO-TP path for larger PDUs)",
                 (unsigned)req_len, UDS_AT_REQ_MAX);
        return ESP_ERR_INVALID_ARG; /* not silently truncated */
    }

    if (pending_out != NULL)
    {
        *pending_out = 0; /* counted from the chip's lines below */
    }

    /* give the chip the extended window: it waits out responsePending
     * itself, so use p2* (fall back to p2) as the request timeout */
    uint32_t timeout_ms = (p2star_ms > p2_ms) ? p2star_ms : p2_ms;

    char at[48];
    char *rbuf = s_rbuf;

    /* Target setup: protocol, tx header, rx filter, the print format and
     * the flow control (SETUP_REST), and the chip's reply wait.
     * Skipped when the caller knows the chip is still in this setup. */
    if (!skip_setup)
    {
        /* CAN protocol: ISO 15765-4, 11-bit or 29-bit, at the bitrate
           the bus runs at (6/7 at 500k, 8/9 at 250k: a request pinned to
           the other bitrate destroys a live bus's traffic), ATTP (try
           protocol, RAM) not ATSP, which writes the chip's EEPROM on
           every request (the driver's guard would rewrite it anyway;
           obd_chip_guard.h) */
        snprintf(at, sizeof(at), "ATTP%c", uds_at_protocol(addr->ext_id));
        (void)req_fn(at, rbuf, UDS_AT_RBUF, 800);

        if (addr->ext_id)
        {
            snprintf(at, sizeof(at), "ATCP%02lX",
                     (unsigned long)((addr->tx_id >> 24) & 0x1F));
            (void)req_fn(at, rbuf, UDS_AT_RBUF, 800);
            snprintf(at, sizeof(at), "ATSH%06lX",
                     (unsigned long)(addr->tx_id & 0xFFFFFF));
            (void)req_fn(at, rbuf, UDS_AT_RBUF, 800);
            snprintf(at, sizeof(at), "ATCRA%08lX",
                     (unsigned long)addr->rx_id);
        }
        else
        {
            snprintf(at, sizeof(at), "ATSH%03lX",
                     (unsigned long)(addr->tx_id & 0x7FF));
            (void)req_fn(at, rbuf, UDS_AT_RBUF, 800);
            snprintf(at, sizeof(at), "ATCRA%03lX",
                     (unsigned long)(addr->rx_id & 0x7FF));
        }

        (void)req_fn(at, rbuf, UDS_AT_RBUF, 800);

        for (size_t i = 0; i < sizeof(SETUP_REST) / sizeof(SETUP_REST[0]); i++)
        {
            (void)req_fn(SETUP_REST[i], rbuf, UDS_AT_RBUF, 800);
        }

        /* the chip's own wait for the first reply frame: ATST in 4 ms
           units from p2 (floor 40 ms, cap 1.02 s) so a single-frame answer
           comes back in ~p2 instead of the chip's 0.5 s default. 0x78
           responsePending stays chip-bound: the MIC rides it out itself
           and P2* beyond ~1 s is not reachable on this path, the native
           ISO-TP path is the answer for slow ECUs. RAM-only (no EEPROM). */
        uint32_t st = p2_ms / 4;

        if (st < 0x0A)
        {
            st = 0x0A;
        }

        if (st > 0xFF)
        {
            st = 0xFF;
        }

        snprintf(at, sizeof(at), "ATST%02lX", (unsigned long)st);
        (void)req_fn(at, rbuf, UDS_AT_RBUF, 800);
    }

    /* the request */
    char cmd[2 * UDS_AT_REQ_MAX + 2]; /* + the response-count digit */

    hex_no_space(req, req_len, cmd, sizeof(cmd));

    /* the ELM response-count digit: ONE response expected, so the chip
       hands it over as soon as it is complete instead of waiting out
       ATST for more ECUs (bench 2026-09-16: 4-8 ms vs 180-260 ms). The
       chip still rides out 7F xx 78 responsePending before the final
       answer and prints every one of them: the parser drops + counts
       them. The digit counts printed LINES, so "1" cuts a multi-frame
       answer after its first frame: the length line says how long the
       message is, and a cut answer is asked for again with the digit
       its line count needs (none above 15 lines; bench 2026-10-02:
       7 ms for the cut answer + 48 ms for a 3-frame one, against 132 ms
       without a digit). Services that must not go out twice start
       without the digit (uds_at_first_digit). */
    size_t cmd_len = strlen(cmd);
    char digit = uds_at_first_digit(req[0]);
    bool repeatable = (digit != '\0');

    /* Re-send the request (setup stays, held by the caller's transaction)
       up to 3 times on a NO-DATA / unparseable answer. The first attempt
       after grabbing the chip can catch the tail of another requester's
       ISO-TP conversation still settling on the bus; a re-send lands in a
       quiet window because the caller holds the chip across the retries
       (bench 2026-09-16: 3E 00 got NO DATA under autopid's multiframe
       load, clean on the re-send). A hard req_fn error (timeout) is the
       chip's own wait and is not retried here. */
    esp_err_t err = ESP_ERR_INVALID_RESPONSE;
    int cuts = 0; /* answers that came back short of their length line */

    for (int attempt = 0; attempt < 3; attempt++)
    {
        cmd[cmd_len] = digit;
        cmd[cmd_len + ((digit != '\0') ? 1 : 0)] = '\0';

        err = req_fn(cmd, rbuf, UDS_AT_RBUF, timeout_ms);

        if (err != ESP_OK)
        {
            return err;
        }

        uint8_t pend = 0;
        size_t announced = 0;

        if (uds_at_parse_response_len(rbuf, resp, resp_cap, resp_len, &pend,
                                      &announced))
        {
            if (announced > *resp_len)
            {
                /* cut short: never handed over as a message */
                size_t got = *resp_len;

                *resp_len = 0;
                err = ESP_ERR_INVALID_SIZE;

                if (!repeatable)
                {
                    ESP_LOGW(TAG, "answer to %02X cut at %u of %u B, not "
                                  "asked again (the service is not "
                                  "repeatable)",
                             req[0], (unsigned)got, (unsigned)announced);
                    break;
                }

                /* The ECU has the chip's flow control and goes on sending
                   the rest of this answer. Asked again at once, those
                   frames count as lines of the new request and it is cut
                   as well (bench 2026-11-26: asked 0.9 ms later, the first
                   frame three times and no answer). So the rest passes
                   first; one more tick, a delay of ticks may end early. */
                vTaskDelay(pdMS_TO_TICKS(uds_at_tail_wait_ms(announced, got,
                                                             cuts)) + 1);

                /* ask for every line this time; a second time without a
                   digit, where no line count can run out */
                uint8_t lines = uds_at_lines_for(announced);

                digit = (cuts == 0 && lines <= 15) ? "0123456789ABCDEF"[lines]
                                                   : '\0';
                cuts++;
                continue;
            }

            if (pending_out != NULL)
            {
                *pending_out = pend; /* the 7F xx 78 lines the chip rode out */
            }

            return ESP_OK;
        }

        err = ESP_ERR_INVALID_RESPONSE;
        vTaskDelay(pdMS_TO_TICKS(30)); /* let the bus settle, then re-send */
    }

    return err;
}
