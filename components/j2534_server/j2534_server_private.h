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

/** Internals shared across the j2534_server .c files. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "j2534_proto.h"
#include "j2534_server.h"

esp_err_t j2534_server_register_cli(void);

/* ---- core (j2534_server.c) <-> transport layer (j2534_server_transport.c) --- */

/* core -> transport */
esp_err_t j2534_srv_transport_init(void);           /* mutexes            */
esp_err_t j2534_srv_listener_start(uint16_t port);  /* the TCP task       */
bool      j2534_srv_listening(void);
const j2534_transport_t *j2534_srv_active(void);    /* NULL = idle        */
/** The same with the session's number (it counts the sessions): what the
 *  RX pump hands back with the batch it filled meanwhile. */
const j2534_transport_t *j2534_srv_active_session(uint32_t *session);
/* The three sends write to @p t only while it IS the active session's
 * transport (checked under the send lock): false otherwise, nothing sent. */
bool      j2534_srv_send_frame(const j2534_transport_t *t, uint8_t type,
                               uint16_t seq, uint16_t channel,
                               const uint8_t *payload, uint32_t len);
/** An ACK. @p status is one of j2534_proto.h's (SAE J2534-1's numbers); it
 *  goes out in the numbers of the session's wire version. */
bool      j2534_srv_send_ack(const j2534_transport_t *t, uint16_t seq,
                             uint16_t channel, uint32_t status,
                             const uint32_t *result);
/** A run of ready-made frames (header + payload each) in ONE transport
 *  write, under the send lock; counts every frame as sent. The RX pump's
 *  batch (2026-11-06). @p session: the session it was filled in
 *  (j2534_srv_active_session); a batch of a session that has ended is not
 *  written. */
bool      j2534_srv_send_raw(const j2534_transport_t *t, uint32_t session,
                             const uint8_t *buf, size_t len);

/** The active session's wire version: 1 until its HELLO asked for more
 *  (j2534_hello_version), 1 again when it ends. Version 2 gets SAE's
 *  status numbers and the EVENT frames. */
void      j2534_srv_set_wire_version(uint32_t version);
uint32_t  j2534_srv_wire_version(void);
/** @p status in the numbers of the active session's wire version, for an
 *  ACK that is built by hand (a status word followed by a list). */
uint32_t  j2534_srv_wire_status(uint32_t status);

/* transport -> core */
bool      j2534_srv_running(void);
volatile const bool *j2534_srv_run_flag(void);
uint8_t  *j2534_srv_req_payload(size_t *cap);       /* PSRAM, one session */
void      j2534_srv_count_rx(void);
void      j2534_srv_count_tx(void);
void      j2534_srv_count_tx_refused(void); /* a write the CAN node refused */
void      j2534_srv_count_tx_timeout(void); /* a send the tester took nothing
                                               of: its session was ended    */
void      j2534_srv_count_rx_lost(uint32_t n); /* received messages a channel
                                                  lost before the tester    */
void      j2534_srv_count_periodic_skipped(void); /* a periodic message that
                                               was still waiting to go when
                                               its next period came         */
void      j2534_srv_session_begin(const char *transport_name);
void      j2534_srv_session_end(void);
void      j2534_srv_handle_frame(const j2534_transport_t *t,
                                 const j2534_hdr_t *h,
                                 const uint8_t *payload);

/** A UDS memory-transfer service (0x34..0x37) in a tester's request:
 *  counted on the status, never gated (2026-10-27). */
void j2534_srv_count_reflash_seen(void);

/** READ_VBATT: rounded millivolts from the battery owner, output untouched on failure. */
uint32_t j2534_srv_read_vbatt(bool device_open, uint32_t *out);

/* ---- the RX pump (j2534_server_pump.c) --------------------------------- */

/** Start the `j2534_rx` task. It runs while the core's run flag is set. */
esp_err_t j2534_srv_pump_start(void);

/** The session's channel table for the pump: J2534_MAX_CHANNELS flags, true
 *  for a connected slot. The core writes it, the pump only reads it. */
const bool *j2534_srv_channels_used(void);

/* ---- periodic messages (j2534_server_periodic.c) ----------------------- */

/** One-time: the table's lock. */
esp_err_t j2534_srv_periodic_init(void);

/** Start the `j2534_per` task, which sends the periodic messages that are
 *  due. It runs while the core's run flag is set. */
esp_err_t j2534_srv_periodic_start(void);

/** START_PERIODIC and STOP_PERIODIC, from the session's task; each sends
 *  its own ACK. */
void j2534_srv_periodic_handle_start(const j2534_transport_t *t,
                                     const j2534_hdr_t *h,
                                     const uint8_t *payload);
void j2534_srv_periodic_handle_stop(const j2534_transport_t *t,
                                    const j2534_hdr_t *h,
                                    const uint8_t *payload);

/** Stop every periodic message (CLOSE, a session's start and end,
 *  CLEAR_PERIODIC_MSGS). A message that is going out right now is waited
 *  for: when this returns nothing more is sent. */
void j2534_srv_periodic_clear_all(void);

/** The same for the periodic messages of one channel slot (DISCONNECT:
 *  the slot's next channel must not inherit them). */
void j2534_srv_periodic_clear_channel(int slot);

/* ---- settings (j2534_server_settings.c) -------------------------------- */

/** Boot-applied config (filled by on_apply). */
typedef struct
{
    bool     enabled;
    bool     allow_lan;        /* accept on STA / USB-eth uplinks too */
    bool     exclusive;        /* boot default: pause autopid while attached */
    uint16_t port;
} j2534_config_t;

/** Register the "j2534_server" descriptor with settings_manager. */
esp_err_t j2534_settings_register(void);

const j2534_config_t *j2534_settings_config(void);

bool j2534_settings_is_configured(void); /* boot apply ran (standard §4.3) */
