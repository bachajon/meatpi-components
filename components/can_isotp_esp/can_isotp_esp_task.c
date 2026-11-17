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
 * @file can_isotp_esp_task.c
 * @brief The provider task: the ONE frame queue every session's
 *        subscription fills, and the "isotp" task that drains it, feeds
 *        each frame to its session's stack and polls the stacks that are
 *        mid-transfer at tick cadence. Both come up at the first open()
 *        (can_isotp_esp_ensure_started). Out of can_isotp_esp.c
 *        (2026-11-18, the 700-line rule: it had grown to 773 lines); the
 *        code is unchanged but for the names of what the two files now
 *        share. Threads and locking: see can_isotp_esp.c.
 */
#include "can_isotp_esp_session.h"

#include <string.h>

#include "sdkconfig.h"

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_isotp.h"

#include "can_core.h"

#define ISOTP_TASK_STACK     4096 /* PSRAM: no flash from this task        */
#define ISOTP_TASK_PRIO      9    /* just under can_core_rx (10)           */
#define ISOTP_FRAME_Q_LEN    256  /* can_core_frame_t, 16 B each, PSRAM:
                                     what the CAN node's receive queue can
                                     release at once after a flash write
                                     (CAN_CORE_RXQ_DEPTH; 128 until
                                     2026-11-17, which the 90 frames of the
                                     erases measured that day did not fill) */
#define ISOTP_IDLE_WAIT_MS   100  /* wake cadence with nothing in flight   */

QueueHandle_t can_isotp_esp_frame_q;
static StaticQueue_t s_frame_q_buf;
EXT_RAM_BSS_ATTR static uint8_t
    s_frame_q_store[ISOTP_FRAME_Q_LEN * sizeof(can_core_frame_t)];

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
EXT_RAM_BSS_ATTR static StackType_t s_task_stack[ISOTP_TASK_STACK];

static bool session_busy(const struct can_isotp_session *s, uint32_t now)
{
    return s->tx_inflight || (int32_t)(s->busy_until_ms - now) > 0;
}

static struct can_isotp_session *find_by_rx(uint32_t id, bool ext)
{
    for (int i = 0; i < ISOTP_MAX_SESSIONS; i++)
    {
        if (can_isotp_esp_tbl[i] != NULL && can_isotp_esp_tbl[i]->rx_id == id &&
            can_isotp_esp_tbl[i]->ext == ext)
        {
            return can_isotp_esp_tbl[i];
        }
    }

    return NULL;
}

/* can_isotp_esp_tbl_lock held */
static void feed_frame(const can_core_frame_t *fr)
{
    if (fr->rtr || fr->dlc > 8)
    {
        return;
    }

    struct can_isotp_session *s = find_by_rx(fr->id, fr->ext);

    if (s == NULL)
    {
        STAT_INC(frames_orphan); /* closed between dispatch and here */
        return;
    }

    uint8_t payload[8];
    twai_frame_t tf;

    memset(&tf, 0, sizeof(tf));
    tf.header.id = fr->id;
    tf.header.ide = fr->ext;
    tf.header.dlc = fr->dlc;
    memcpy(payload, fr->data, fr->dlc);
    tf.buffer = payload;
    tf.buffer_len = fr->dlc;

    xSemaphoreTake(s->lock, portMAX_DELAY);
    (void)esp_isotp_feed_can_frame(s->tp, &tf);
    (void)esp_isotp_poll(s->tp); /* a flow control just unblocked CFs */
    xSemaphoreGive(s->lock);

    s->busy_until_ms = can_isotp_esp_now_ms() + ISOTP_BUSY_MS;
    STAT_INC(frames_fed);
}

/* can_isotp_esp_tbl_lock held; returns whether any session still needs tick polling */
static bool poll_busy_sessions(void)
{
    uint32_t now = can_isotp_esp_now_ms();
    bool any = false;

    for (int i = 0; i < ISOTP_MAX_SESSIONS; i++)
    {
        struct can_isotp_session *s = can_isotp_esp_tbl[i];

        if (s == NULL)
        {
            continue;
        }

        if (s->tx_inflight && (int32_t)(now - s->tx_deadline_ms) > 0)
        {
            /* the sender gave up on it long ago; the stack has flagged
               N_Bs by now (busy window > N_Bs): stop tracking */
            s->tx_inflight = false;
        }

        if (!session_busy(s, now))
        {
            continue;
        }

        any = true;
        xSemaphoreTake(s->lock, portMAX_DELAY);
        (void)esp_isotp_poll(s->tp);
        xSemaphoreGive(s->lock);
    }

    return any;
}

static void isotp_task(void *arg)
{
    TickType_t wait = pdMS_TO_TICKS(ISOTP_IDLE_WAIT_MS);

    (void)arg;

    for (;;)
    {
        can_core_frame_t fr;
        bool got = (xQueueReceive(can_isotp_esp_frame_q, &fr, wait) == pdTRUE);

        xSemaphoreTake(can_isotp_esp_tbl_lock, portMAX_DELAY);

        if (got)
        {
            feed_frame(&fr);
        }

        /* drain a burst before polling: consecutive frames arrive every
           ~250 us at 500 kbps */
        while (xQueueReceive(can_isotp_esp_frame_q, &fr, 0) == pdTRUE)
        {
            feed_frame(&fr);
        }

        bool busy = poll_busy_sessions();

        xSemaphoreGive(can_isotp_esp_tbl_lock);

        wait = busy ? 1 : pdMS_TO_TICKS(ISOTP_IDLE_WAIT_MS);
    }
}

/* can_isotp_esp_tbl_lock held */
esp_err_t can_isotp_esp_ensure_started(void)
{
    if (can_isotp_esp_frame_q == NULL)
    {
        can_isotp_esp_frame_q = xQueueCreateStatic(ISOTP_FRAME_Q_LEN,
                                                   sizeof(can_core_frame_t),
                                                   s_frame_q_store, &s_frame_q_buf);

        if (can_isotp_esp_frame_q == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_task == NULL)
    {
        s_task = xTaskCreateStatic(isotp_task, "isotp", ISOTP_TASK_STACK,
                                   NULL, ISOTP_TASK_PRIO, s_task_stack,
                                   &s_task_tcb);

        if (s_task == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}
