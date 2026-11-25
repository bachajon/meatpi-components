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
 * @file uds_transport.h
 * @brief The transport vtable behind uds_manager. ONE op: a single
 *        request→response transaction with NO UDS semantics (the 0x78
 *        loop / NRC decode / tester-present live in uds_manager above).
 *
 * Implementations:
 *   uds_transport_obd:    obd_chip AT (MIC3624), always available.
 *   uds_transport_isotp: raw UDS PDU over the registered ISO-TP
 *                         provider (can_isotp.h) on can_manager.
 *
 * The shared AT-hex helpers below are pure and host-tested.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "uds_manager.h" /* uds_addr_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct uds_transport_s
{
    const char *name;

    /** Bring the backend up (open engine / isotp handle). Idempotent.
     *  ESP_ERR_INVALID_STATE if the backend's bus isn't available. */
    esp_err_t (*open)(void);

    /** One request→FINAL response. Sets addressing as needed, sends
     *  @p req, and transparently consumes 0x78 responsePending frames
     *  (using @p p2star_ms for each) so the caller sees only the final
     *  UDS PDU (headers stripped). @p pending_out (optional) receives
     *  the number of 0x78 frames seen. ESP_ERR_TIMEOUT on no response. */
    esp_err_t (*transceive)(const uds_addr_t *addr,
                            const uint8_t *req, size_t req_len,
                            uint8_t *resp, size_t resp_cap, size_t *resp_len,
                            uint32_t p2_ms, uint32_t p2star_ms,
                            uint8_t *pending_out);
} uds_transport_t;

/* Backend singletons (open lazily). */
const uds_transport_t *uds_transport_obd(void);
const uds_transport_t *uds_transport_isotp(void);

/** N_Cr of ISO 15765-2: the longest gap between two frames of a message. */
#define UDS_ISOTP_N_CR_MS 1000u

/** PURE: is an answer still arriving on the native ISO-TP path? P2 and P2*
 *  are the ECU's time to START its answer, and the provider hands over whole
 *  messages only, so a wait that ran out asks this before it gives up.
 *  @param rx_idle_ms       since the last frame on the answer's id
 *                          (UINT32_MAX: none ever)
 *  @param since_request_ms since the request went out, or since the last
 *                          responsePending
 *  @return true when a frame came in behind the request and less than N_Cr
 *          ago: the message is still being sent. */
static inline bool uds_isotp_answer_arriving(uint32_t rx_idle_ms,
                                             uint32_t since_request_ms)
{
    return rx_idle_ms != UINT32_MAX && rx_idle_ms < since_request_ms &&
           rx_idle_ms < UDS_ISOTP_N_CR_MS;
}

/* The AT-hex transaction (the request fn is injected so the transaction
 * stays testable). */
typedef esp_err_t (*uds_at_request_fn)(const char *cmd, char *resp,
                                       size_t resp_len, uint32_t timeout_ms);

/** The bitrate the target setup pins: 500 (ATTP6 / ATTP7, the default) or
 *  250 (ATTP8 / ATTP9). The obd_chip transport sets it from what the native
 *  controller heard on the bus. Anything but 250 means 500. */
void uds_at_set_can_kbps(uint16_t kbps);

/** The chip protocol character the setup pins for an 11-bit / 29-bit
 *  target at the bitrate set above. PURE. */
char uds_at_protocol(bool ext_id);

/** The same with @p skip_setup: the caller knows the chip is still in this
 *  address' setup (nobody else wrote to it since), so only the request
 *  line goes out, one request = one AT round-trip. */
esp_err_t uds_at_transceive_ex(uds_at_request_fn req_fn,
                               const uds_addr_t *addr,
                               const uint8_t *req, size_t req_len,
                               uint8_t *resp, size_t resp_cap,
                               size_t *resp_len, uint32_t p2_ms,
                               uint32_t p2star_ms, uint8_t *pending_out,
                               bool skip_setup);

esp_err_t uds_at_transceive(uds_at_request_fn req_fn,
                            const uds_addr_t *addr,
                            const uint8_t *req, size_t req_len,
                            uint8_t *resp, size_t resp_cap, size_t *resp_len,
                            uint32_t p2_ms, uint32_t p2star_ms,
                            uint8_t *pending_out);

/** PURE: parse an ELM/MIC headers-off hex response (single- or
 *  multi-frame, tolerant of index tokens and an ISO-TP length prefix)
 *  into UDS payload bytes. Host-tested. Returns false on no hex / a
 *  chip error token / overflow. */
/** The same, also counting the chip's '7F xx 78' responsePending lines
 *  that preceded the final answer (dropped from @p out). A multi-frame
 *  message is trimmed to the size its length line announces; one that
 *  is SHORTER than announced (cut by the response-count digit) is not a
 *  message: false. */
bool uds_at_parse_response_ex(const char *resp, uint8_t *out, size_t cap,
                              size_t *out_len, uint8_t *pending_out);

/** The worker behind both: also hands back the announced ISO-TP total of
 *  the returned message (0 = single frame) and DOES return a cut message
 *  (@p out_len < @p announced_out), so the transport can ask again. PURE. */
bool uds_at_parse_response_len(const char *resp, uint8_t *out, size_t cap,
                               size_t *out_len, uint8_t *pending_out,
                               size_t *announced_out);

/** Lines the chip prints for an ISO-TP message of @p total bytes (first
 *  frame 6 payload bytes, then 7 per line; 1 up to 7 bytes): the
 *  response-count digit a multi-frame request needs. PURE. */
uint8_t uds_at_lines_for(size_t total);

/** The response-count digit of the FIRST attempt for service @p sid: '1',
 *  or '\0' (none) for the services that must not be sent twice. PURE. */
char uds_at_first_digit(uint8_t sid);

/** How long to let the bus be before a cut answer is asked for again: the
 *  chip stopped printing at its line count but sent its flow control, so
 *  the ECU still sends the rest, and those frames would count as lines of
 *  the next request. From the frames still owed (@p announced bytes in all,
 *  @p got of them printed); @p cuts = how often this request was cut
 *  before (the wait grows). PURE. */
uint32_t uds_at_tail_wait_ms(size_t announced, size_t got, int cuts);

bool uds_at_parse_response(const char *resp, uint8_t *out, size_t cap,
                           size_t *out_len);

#ifdef __cplusplus
}
#endif
