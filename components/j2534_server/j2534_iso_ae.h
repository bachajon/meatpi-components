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
 * @file j2534_iso_ae.h
 * @brief ISO15765 extended addressing on a J2534 channel, the pure part
 *        (2026-11-20): what a FLOW_CONTROL filter asks for, the message
 *        layout [4-byte id][address extension][payload] both ways (SAE
 *        J2534-1, ISO15765_ADDR_TYPE), the single frame behind an address
 *        extension, and what the receive path asks of a first frame. No
 *        I/O and no RTOS: host tested (host_test/main/test_iso_ae.c). The
 *        normal layout's helpers are j2534_proto.h's.
 *
 * ISO 15765-2 extended addressing puts one byte, the address extension, in
 * front of every frame of a message (single, first, consecutive and flow
 * control alike), so each frame carries one byte less: a single frame 6,
 * a first frame 5, a consecutive frame 6. It is not the 29-bit id
 * (CAN_29BIT_ID); the two combine.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "j2534_proto.h"

/** A single frame's payload: 7 bytes, 6 behind an address extension. */
#define J2534_ISO_SF_MAX     7u
#define J2534_ISO_SF_MAX_AE  6u

/** The extended addressing of a session: the fifth byte of the FLOW_CONTROL
 *  filter's pattern (rx) and of its flow-control message (tx). */
typedef struct
{
    bool    on;
    uint8_t tx;     /* in front of every frame sent to the ECU    */
    uint8_t rx;     /* in front of every frame the ECU sends back */
} j2534_iso_ae_t;

/**
 * The addressing a FLOW_CONTROL filter asks for. ISO15765_ADDR_TYPE in the
 * TxFlags of the pattern or of the flow-control message means extended
 * addressing, and both then carry 5 bytes (the id and the address extension
 * of that direction); without it they carry at least the 4 id bytes.
 * False when a message is missing or shorter than that (@p ae untouched).
 */
bool j2534_iso_filter_ae(const j2534_msg_t *pattern,
                         const j2534_msg_t *flow_control,
                         j2534_iso_ae_t *ae);

/**
 * Split a tester's extended-addressing message, [id][address extension]
 * [payload], into its parts (the id not masked to 11 or 29 bits). False
 * without ISO15765_ADDR_TYPE in its TxFlags or when it is shorter than
 * 5 bytes. A 5-byte message yields len 0.
 */
bool j2534_iso_ae_split(const j2534_msg_t *msg, uint32_t *id, uint8_t *ae,
                        const uint8_t **payload, size_t *len);

/**
 * A single frame behind an address extension, for a request no session
 * carries: [ae][0n][payload], padded to 8 bytes with 0xAA when @p pad.
 * Returns the DLC, 0 when len is 0 or above 6.
 */
uint8_t j2534_iso_ae_single_frame(uint8_t ae, const uint8_t *payload,
                                  size_t len, bool pad, uint8_t frame[8]);

/**
 * Build a device-to-tester message of an extended-addressing session:
 * data = [id][ae][payload], rx_status = ISO15765_ADDR_TYPE (and
 * CAN_29BIT_ID when @p ext). False when the payload does not fit
 * (len > J2534_MAX_DATA - 5), @p out untouched.
 */
bool j2534_iso_ae_join(j2534_msg_t *out, uint32_t id, bool ext, uint8_t ae,
                       const uint8_t *payload, size_t len, uint32_t ts_us);

/**
 * Turn an indication built by j2534_iso_indication() into one of an
 * extended-addressing session: the address extension follows the id (5
 * bytes) and ISO15765_ADDR_TYPE is set in rx_status.
 */
void j2534_iso_ae_indication(j2534_msg_t *out, uint8_t ae);

/**
 * Is the reassembled message @p pdu (@p n bytes) the one the first frame
 * @p ff starts? @p ff begins at its PCI byte (behind the address extension
 * when there is one) and holds @p dlc bytes. The length the frame names
 * and the bytes it carries decide; both first frame forms are read (the
 * 12-bit length, and the 32-bit one of a message above 4095 bytes).
 */
bool j2534_iso_pdu_of_first_frame(const uint8_t *pdu, size_t n,
                                  const uint8_t *ff, uint8_t dlc);

/** The frames a message of @p len bytes takes on the bus (1 for a single
 *  frame; else the first frame and its consecutive frames). */
uint32_t j2534_iso_frame_count(size_t len, bool has_ae);
