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
 * @file can_isotp_esp_session.h
 * @brief The provider's RTOS side, shared by can_isotp_esp.c (sessions,
 *        registration) and can_isotp_esp_task.c (the provider task): the
 *        session record, the session table and its lock, the counters,
 *        the clock and the task's frame queue. Not in
 *        can_isotp_esp_private.h: that header is the pure mailbox, which
 *        the host test builds without an RTOS and without esp_isotp.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_isotp.h"

#include "can_isotp_esp.h"
#include "can_isotp_esp_private.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ISOTP_MAX_SESSIONS   12   /* 1 uds_manager + up to 11 J2534 flow-control
                                     sessions (4 channels x up to 4 each,
                                     2026-10-19); each takes a can_manager
                                     subscriber slot while open           */

/* after any activity a session is polled at tick cadence for this long:
 * longer than the stack's N_Bs/N_Cr (Kconfig response timeout, 1000 ms
 * since 2026-10-19: ISO 15765-2's value, 100 ms before) so a stalled
 * transfer is always flagged by a poll and the link never stays "in
 * progress" forever */
#define ISOTP_BUSY_MS        1200

struct can_isotp_session
{
    esp_isotp_handle_t  tp;
    SemaphoreHandle_t   lock;      /* every esp_isotp_* call on tp        */
    SemaphoreHandle_t   tx_done;   /* binary: tx-complete callback        */
    SemaphoreHandle_t   rx_sem;    /* counting: one token per mailbox PDU */
    isotp_mbox_t        mbox;
    isotp_mbox_slot_t  *slots;     /* PSRAM                               */
    int                 sub_idx;   /* can_manager queue subscription      */
    uint32_t            tx_id;
    uint32_t            rx_id;
    bool                ext;
    volatile bool       tx_inflight;    /* multi-frame send not complete  */
    volatile uint32_t   tx_deadline_ms; /* give up tracking it after this */
    volatile uint32_t   busy_until_ms;  /* tick-cadence polling until     */
};

/* ---- can_isotp_esp.c, used by the task ------------------------------------- */

/** The session table: a slot is NULL or an open session. Read and written
 *  with can_isotp_esp_tbl_lock held. */
extern struct can_isotp_session *can_isotp_esp_tbl[ISOTP_MAX_SESSIONS];

/** The table's mutex: open() and close() take it, and the task holds it
 *  while it feeds and polls, so close() cannot free a session under it. */
extern SemaphoreHandle_t can_isotp_esp_tbl_lock;

/** The provider's counters (can_isotp_esp_get_stats) and the spinlock
 *  around every change to them: count with STAT_INC. */
extern can_isotp_esp_stats_t can_isotp_esp_stats;
extern portMUX_TYPE can_isotp_esp_stats_mux;

#define STAT_INC(field)                                    \
    do                                                     \
    {                                                      \
        portENTER_CRITICAL(&can_isotp_esp_stats_mux);      \
        can_isotp_esp_stats.field++;                       \
        portEXIT_CRITICAL(&can_isotp_esp_stats_mux);       \
    } while (0)

/** Milliseconds since boot (esp_timer) in 32 bits: it wraps, so two
 *  values are compared by their difference. */
uint32_t can_isotp_esp_now_ms(void);

/* ---- can_isotp_esp_task.c -------------------------------------------------- */

/** The ONE frame queue (can_core_frame_t): every session subscribes it to
 *  its rx id. NULL until can_isotp_esp_ensure_started() has run. */
extern QueueHandle_t can_isotp_esp_frame_q;

/** Create the frame queue and the "isotp" task, once (the first open()).
 *  Call with can_isotp_esp_tbl_lock held. ESP_OK or ESP_ERR_NO_MEM. */
esp_err_t can_isotp_esp_ensure_started(void);

#ifdef __cplusplus
}
#endif
