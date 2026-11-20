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
 * @file autopid_send.h
 * @brief can.send (2026-11-21, GitHub issue #1072): raw frames on any bus of
 *        the OBD connector, for the HTTP route, the rule action and the
 *        script binding.
 *
 * A request names a bus and one to AP_SEND_FRAMES_MAX frames:
 *
 *   `can`  WiCAN's own CAN controller (pins 6 and 14). No OBD chip involved,
 *          so it works beside an app that holds the chip; the controller
 *          must be on and on the bus's bit rate (can_manager).
 *   `obd`  the car's OBD protocol, through the OBD chip. On a CAN car (ISO
 *          15765-4, J1939) the frame goes out on pins 6 and 14 at that
 *          protocol's bit rate, with an 11-bit or a 29-bit id whatever the
 *          protocol's own id length is. On a car before CAN (SAE J1850, ISO
 *          9141-2, ISO 14230-4) the "id" is the message's three header
 *          bytes and the chip adds the checksum.
 *   `ms`, `sw`, `ch`  MS CAN (pins 3 and 11), single-wire CAN (pin 1) and
 *          CH CAN (pins 12 and 13), through the chip, at the bus's usual
 *          bit rate or at `bus_kbps`, as a row's bus (autopid_bus.h).
 *
 * NOTHING of it writes the chip's memory. A bus is reached through a slot
 * defined in the chip's RAM (autopid_bus.h), the frame goes out with the
 * chip's CAN formatting off and the data length as given, and behind the
 * last frame the chip is taken back to the car's OBD protocol with every
 * word of the send undone (the chain bookkeeping of autopid_chain.c).
 *
 * `wake` on single-wire CAN sends that frame in the transceiver's
 * high-voltage wake mode (GMLAN: the frame that wakes a sleeping car). It
 * is the one path that can: the frame was asked for by name. The frames
 * behind it go out in normal mode, and the transceiver is put back to
 * normal before the chip leaves the bus.
 *
 * The pure half (autopid_send_core.c: the request's JSON, the checks, the
 * chip words, how a reply reads) is host-tested; the device half is
 * autopid_send.c.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AP_SEND_FRAMES_MAX    8     /* frames in one request                 */
#define AP_SEND_GAP_MAX_MS    2000  /* from one frame to the next            */
#define AP_SEND_TOTAL_MAX_MS  5000  /* all the gaps of a request together    */
#define AP_SEND_PRECAN_MAX    7     /* data bytes of a message before CAN    */

typedef enum
{
    AP_SEND_CAN = 0,  /* WiCAN's own CAN controller                         */
    AP_SEND_OBD,      /* the OBD chip, the car's OBD protocol               */
    AP_SEND_MS,       /* the OBD chip, MS CAN                               */
    AP_SEND_SW,       /* the OBD chip, single-wire CAN                      */
    AP_SEND_CH,       /* the OBD chip, CH CAN                               */
    AP_SEND_WHERE_COUNT
} ap_send_where_t;

typedef struct
{
    uint32_t id;       /* the CAN id, or three header bytes before CAN      */
    bool     ext;      /* a 29-bit id                                       */
    bool     wake;     /* single-wire CAN: the high-voltage wake mode       */
    uint8_t  len;      /* data bytes                                        */
    uint8_t  data[8];
    uint16_t gap_ms;   /* from this frame's start to the next one's         */
} ap_send_frame_t;

typedef struct
{
    ap_send_where_t where;
    uint8_t         rate;   /* ms / sw / ch: the rate code (autopid_bus.h)  */
    uint8_t         n;
    bool            precan; /* `obd` on a car before CAN: the ids are three
                               header bytes (set once the car is known)     */
    ap_send_frame_t f[AP_SEND_FRAMES_MAX];
} ap_send_req_t;

/** What became of one frame. */
typedef enum
{
    AP_SEND_SENT = 0,   /* on the bus (CAN: somebody acknowledged it)       */
    AP_SEND_NO_ACK,     /* CAN: sent, and nobody acknowledged it            */
    AP_SEND_ERROR,      /* the chip or the controller refused it            */
    AP_SEND_NOT_SENT,   /* never tried (an earlier step failed)             */
} ap_send_result_t;

/* ---- pure (autopid_send_core.c, host-tested) -------------------------------- */

/** The config word of a place ("can", "obd", "ms", "sw", "ch"). */
const char *ap_send_where_word(ap_send_where_t where);

/** Hex text to bytes: any case, spaces, commas and one "0x" per byte
 *  allowed ("80 01 FF", "8001ff", "0x80,0x01"). @return false = not hex, an
 *  odd digit count, or more than @p cap bytes. "" and NULL = 0 bytes. */
bool ap_send_hex(const char *text, uint8_t *out, size_t cap, size_t *n);

/**
 * A request as the route, the action and the script hand it over:
 *
 *   {"bus": "can"|"obd"|"ms"|"sw"|"ch", "bus_kbps"?: N,
 *    "id": "<hex>"|N, "ext"?: bool, "data": "<hex>", "wake"?: bool}
 *   or, for a sequence, "frames": [{"id","ext"?,"data","wake"?,"gap_ms"?}]
 *   in place of "id" and "data".
 *
 * Everything that can be said without knowing the car is checked here.
 * @return false with a sentence in @p err: the request is refused whole,
 * nothing is sent.
 */
bool ap_send_parse(const cJSON *body, ap_send_req_t *out, char *err,
                   size_t err_len);

/** The rest of the checks, for a request on `obd`, once the car's OBD
 *  protocol is known (@p proto: the ELM number, '0' = not known). On a car
 *  before CAN the id is three header bytes and a message carries 1 to
 *  AP_SEND_PRECAN_MAX data bytes. @return false with a sentence in @p err.
 *  Other places: true. */
bool ap_send_check_car(const ap_send_req_t *req, char proto, char *err,
                       size_t err_len);

/** Where the chip must be for frame @p i (AP_BUS_KEY with the id length and
 *  the wake mode, autopid_bus.h): 0 = at home, on the car's own protocol (a
 *  car before CAN). Not for AP_SEND_CAN. */
uint16_t ap_send_key(const ap_send_req_t *req, int i, char proto);

/** The words that make the chip send what it is given and nothing else: its
 *  CAN formatting off and the data length as given (CAN), and a short wait
 *  for an answer nobody asked for. Every one of them is undone by the
 *  restore of autopid_chain.c. */
const char *ap_send_setup(bool precan);

/** The words that set a frame's id (`ATSH7DF`, `ATCP10;ATSH24E097`), or a
 *  message's three header bytes before CAN (`ATSH686AF1`). @return the
 *  length written, 0 when it does not fit. */
size_t ap_send_header(const ap_send_frame_t *f, bool precan, char *out,
                      size_t cap);

/** The data as the chip takes it ("8001FF"). @return the length written. */
size_t ap_send_data(const ap_send_frame_t *f, char *out, size_t cap);

/** What the chip said to the data line: `NO DATA` and anything that reads
 *  as an answer = on the bus; `CAN ERROR` = nobody acknowledged; an error
 *  word, `?` or nothing = refused. */
ap_send_result_t ap_send_classify(const char *reply);

/** "sent", "no_ack", "error", "not_sent". */
const char *ap_send_result_word(ap_send_result_t r);

#ifndef AUTOPID_HOST_TEST
/* ---- the device half: autopid_send.c (its entry is autopid_can_send() in
 * autopid.h) and its two surfaces here, autopid_http_send.c ------------------ */
#include "esp_err.h"
#include "esp_http_server.h"

/** POST /api/can/send (the route table is autopid_http.c's). */
esp_err_t ap_send_post_handler(httpd_req_t *req);

/** Register the `can.send` rule action (from autopid's events glue). */
void ap_send_events_register(void);
#endif

#ifdef __cplusplus
}
#endif
