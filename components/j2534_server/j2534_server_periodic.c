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
 * @file j2534_server_periodic.c
 * @brief Periodic messages (PassThruStartPeriodicMsg): the table, one
 *        esp_timer per message as its clock, and the `j2534_per` task that
 *        sends what is due. Out of j2534_server.c on 2026-11-18, with the
 *        change below.
 *
 * Until that day the timer's callback sent the message itself, so the send
 * ran in the esp_timer task, the one task that serves every software timer
 * of the unit. A send waits: for the channel lock (seconds while a long
 * ISO 15765 write held it), for room in the CAN driver (up to 100 ms), for
 * the conversation gate (up to 3 s), for a TP2.0 acknowledge. All of that
 * time no other timer of the unit fired, and when the task was free again
 * the timer fired once for every period it had missed: the copies of the
 * message went out in one burst (bench 2026-11-18: a 20 ms message silent
 * for the 2.2 s of a write, then 121 frames in the next quarter second).
 *
 * Now the callback only marks the message due and wakes the task. A
 * message that is still waiting when its next period comes is sent once,
 * not twice (`periodic_skipped` on /api/j2534 counts the periods that went
 * that way): SAE J2534-1 wants no copies of a periodic message stacked up.
 *
 * Locking: the table's lock is held by the task while it sends a message
 * and by the session's task while it starts or stops one, so a stop
 * returns only when nothing more of that message can go out, and a channel
 * is never closed under a periodic write (the session's task clears the
 * channel's messages first). Order: this lock, then the channel lock,
 * never the other way round.
 */
#include "j2534_server_private.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "j2534_channel.h"
#include "j2534_proto.h"

#define J2534_MAX_PERIODIC   4
#define J2534_PER_STACK      4096   /* PSRAM: no flash from this task */
#define J2534_PER_PRIO       5
#define J2534_PER_RETRY_MS   2      /* a message that could not go: look again */
#define J2534_PER_IDLE_MS    1000   /* nothing due: wake for the run flag */
#define J2534_PER_MIN_MS     5      /* the shortest interval taken */

typedef struct
{
    bool                in_use;
    volatile bool       due;       /* the timer sets it, the task clears it */
    uint32_t            id;
    int                 slot;      /* channel slot */
    uint32_t            tick;      /* sends so far (0 = the one at START) */
    esp_timer_handle_t  timer;
    j2534_msg_t         msg;
} periodic_t;

/* PSRAM: each slot embeds a 4 KB j2534_msg_t, which must not sit in
 * internal BSS (it starves WiFi; section 2 of the standard, the memory
 * budget). Neither an ISR nor a flash path touches it: the timer's
 * callback runs in the esp_timer task. */
EXT_RAM_BSS_ATTR static periodic_t s_periodic[J2534_MAX_PERIODIC];
static uint32_t s_next_periodic_id = 1;
/* a START_PERIODIC's message while it is looked at (the session's task,
 * under the table's lock): PSRAM, 4 KB, never on a task stack */
EXT_RAM_BSS_ATTR static j2534_msg_t s_new_msg;

static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf;   /* internal: FreeRTOS object */

static TaskHandle_t s_task;
static StaticTask_t s_tcb;             /* internal: FreeRTOS */
EXT_RAM_BSS_ATTR static StackType_t s_stack[J2534_PER_STACK];

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---- the clock: esp_timer task, never blocks --------------------------- */

static void periodic_cb(void *arg)
{
    periodic_t *pm = arg;
    TaskHandle_t task = s_task;

    if (pm->due)
    {
        /* the last one has not gone yet: this period joins it */
        j2534_srv_count_periodic_skipped();
    }

    pm->due = true;

    if (task != NULL)
    {
        xTaskNotifyGive(task);
    }
}

/* ---- the sender --------------------------------------------------------- */

/* the table's lock held; true = it could not go now, look again soon */
static bool send_due(periodic_t *pm)
{
    bool due = pm->due;

    pm->due = false; /* also a mark a stopped message's last tick left */

    if (!pm->in_use || !due)
    {
        return false;
    }

    /* a periodic message raises no TxDone indication (SAE J2534-1) */
    if (!j2534_channel_active(pm->slot))
    {
        return false;
    }

    uint32_t st = j2534_channel_write_periodic(pm->slot, &pm->msg,
                                               pm->tick + 1);

    if (st == J2534_ERR_BUFFER_FULL)
    {
        /* the tester's own transfer to that ECU is on the bus: the message
           stays due and goes when the transfer has ended */
        pm->due = true;
        return true;
    }

    pm->tick++;
    return false;
}

static void periodic_task(void *arg)
{
    volatile const bool *run = j2534_srv_run_flag();
    bool again = false;

    (void)arg;

    while (*run)
    {
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS(again ? J2534_PER_RETRY_MS
                                                   : J2534_PER_IDLE_MS));
        again = false;

        for (int i = 0; i < J2534_MAX_PERIODIC; i++)
        {
            if (!s_periodic[i].due)
            {
                continue; /* looked at without the lock: nothing to do */
            }

            xSemaphoreTake(s_lock, portMAX_DELAY);
            again |= send_due(&s_periodic[i]);
            xSemaphoreGive(s_lock);
        }
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/* ---- start / stop (the session's task) ---------------------------------- */

/* the table's lock held */
static void stop_one(periodic_t *pm)
{
    if (!pm->in_use)
    {
        return;
    }

    esp_timer_stop(pm->timer);
    esp_timer_delete(pm->timer);
    pm->in_use = false;
    pm->due = false;
}

/* the table's lock held; J2534 status, *id set on NOERROR */
static uint32_t start_locked(int slot, const j2534_hdr_t *h,
                             const uint8_t *payload, uint32_t *id)
{
    periodic_t *pm = NULL;

    /* payload: interval_ms u32, then one PASSTHRU_MSG */
    if (h->length < 4 + 24 || !j2534_channel_active(slot) ||
        !j2534_msg_decode(payload + 4, h->length - 4, &s_new_msg))
    {
        return J2534_ERR_INVALID_MSG;
    }

    for (int i = 0; i < J2534_MAX_PERIODIC; i++)
    {
        if (!s_periodic[i].in_use)
        {
            pm = &s_periodic[i];
            break;
        }
    }

    if (pm == NULL)
    {
        return J2534_ERR_FAILED;
    }

    uint32_t interval = rd_u32(payload);

    if (interval < J2534_PER_MIN_MS)
    {
        interval = J2534_PER_MIN_MS;
    }

    pm->msg = s_new_msg;
    pm->slot = slot;
    pm->id = s_next_periodic_id++;
    pm->tick = 0;
    pm->due = false;

    esp_timer_create_args_t ta = { .callback = periodic_cb, .arg = pm,
                                   .name = "j2534_per" };

    if (esp_timer_create(&ta, &pm->timer) != ESP_OK)
    {
        return J2534_ERR_FAILED;
    }

    /* TP2_0_BROADCAST_MSG, the re-triggered broadcast (SAE J2534-2
     * 19.3.2.3): the five telegrams go out now, from this task and before
     * the timer runs, then one telegram per period. A message the channel
     * refuses starts nothing. */
    if ((pm->msg.tx_flags & J2534_TX_TP2_0_BROADCAST) != 0 &&
        j2534_channel_is_tp20(slot))
    {
        uint32_t st = j2534_channel_write_periodic(slot, &pm->msg, 0);

        if (st != J2534_STATUS_NOERROR)
        {
            esp_timer_delete(pm->timer);
            return st;
        }
    }

    if (esp_timer_start_periodic(pm->timer, (uint64_t)interval * 1000) != ESP_OK)
    {
        esp_timer_delete(pm->timer);
        return J2534_ERR_FAILED;
    }

    pm->in_use = true;
    *id = pm->id;
    return J2534_STATUS_NOERROR;
}

void j2534_srv_periodic_handle_start(const j2534_transport_t *t,
                                     const j2534_hdr_t *h,
                                     const uint8_t *payload)
{
    uint32_t id = 0;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t st = start_locked((int)h->channel - 1, h, payload, &id);
    xSemaphoreGive(s_lock);

    (void)j2534_srv_send_ack(t, h->seq, h->channel, st,
                             st == J2534_STATUS_NOERROR ? &id : NULL);
}

void j2534_srv_periodic_handle_stop(const j2534_transport_t *t,
                                    const j2534_hdr_t *h,
                                    const uint8_t *payload)
{
    uint32_t pid = (h->length >= 4) ? rd_u32(payload) : 0;
    uint32_t st = J2534_ERR_INVALID_MSG;

    xSemaphoreTake(s_lock, portMAX_DELAY);

    for (int i = 0; i < J2534_MAX_PERIODIC; i++)
    {
        if (s_periodic[i].in_use && s_periodic[i].id == pid)
        {
            stop_one(&s_periodic[i]);
            st = J2534_STATUS_NOERROR;
            break;
        }
    }

    xSemaphoreGive(s_lock);
    (void)j2534_srv_send_ack(t, h->seq, h->channel, st, NULL);
}

void j2534_srv_periodic_clear_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    for (int i = 0; i < J2534_MAX_PERIODIC; i++)
    {
        stop_one(&s_periodic[i]);
    }

    xSemaphoreGive(s_lock);
}

void j2534_srv_periodic_clear_channel(int slot)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    for (int i = 0; i < J2534_MAX_PERIODIC; i++)
    {
        if (s_periodic[i].in_use && s_periodic[i].slot == slot)
        {
            stop_one(&s_periodic[i]);
        }
    }

    xSemaphoreGive(s_lock);
}

/* ---- lifecycle ----------------------------------------------------------- */

esp_err_t j2534_srv_periodic_init(void)
{
    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    return (s_lock != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t j2534_srv_periodic_start(void)
{
    s_task = xTaskCreateStatic(periodic_task, "j2534_per", J2534_PER_STACK,
                               NULL, J2534_PER_PRIO, s_stack, &s_tcb);
    return (s_task != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}
