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
 * @file j2534_server_pump.c
 * @brief The RX pump of the J2534 server: the `j2534_rx` task drains every
 *        active channel and pushes what it received to the tester as RX_MSG
 *        frames, batched into one transport write. Out of j2534_server.c
 *        (2026-11-13, the 700-line rule: the core had grown to 743 lines).
 *        The core starts it (j2534_srv_pump_start) and lends it the run
 *        flag and the session's channel table. Since 2026-11-18 it also
 *        tells the tester what a channel lost (tell_lost).
 */
#include "j2534_server_private.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "can_core.h"      /* can_core_frame_t: the pump's queue peek */

#include "j2534_channel.h"
#include "j2534_proto.h"

static const char *TAG = "j2534_server";

#define J2534_TASK_STACK     4096   /* PSRAM: no flash from this task */
#define J2534_TASK_PRIO      5

static TaskHandle_t s_rx_task;
static StaticTask_t s_rx_tcb;       /* internal: FreeRTOS */
EXT_RAM_BSS_ATTR static StackType_t s_rx_stack[J2534_TASK_STACK];

/* big scratch buffers - PSRAM (the pump task alone owns them).
 * j2534_msg_t is ~4 KB; NEVER put one on a task stack (the listener/RX
 * tasks have 4 KB stacks - three on the stack overflowed and reset the
 * device, caught 2026-07-07 via the DLL's FLOW_CONTROL filter). */
EXT_RAM_BSS_ATTR static uint8_t s_rx_wire[J2534_MAX_DATA + 32];
EXT_RAM_BSS_ATTR static j2534_msg_t s_rx_msg;

/* RX_MSG frames waiting to go out in ONE transport write (PSRAM; the
 * pump task only). Up to a maximal frame's worth: every transport takes
 * that size today (CDC's DMA buffer, BLE's channel write), and the wire
 * contract lets frames be coalesced (J2534_WIRE_PROTOCOL.md section 1). */
#define J2534_RX_BATCH_MAX  32   /* frames per write at most */
EXT_RAM_BSS_ATTR static uint8_t s_rx_batch[J2534_MAX_FRAME];
static uint32_t s_session;       /* the session the batch is being filled in */

/* A channel lost received messages (its bus queue ran over while this
 * pump was held up: a tester that does not read its link, a link slower
 * than the bus). Until 2026-11-18 nobody was told. Counted (`rx_lost` on
 * /api/j2534), said in the log at most once a second, and a tester of wire
 * version 2 gets an EVENT RX_LOST with the count in the batch, in front of
 * the channel's next message: the first one behind the gap, the place SAE
 * J2534-1 wants the application told at (the Windows driver returns
 * ERR_BUFFER_OVERFLOW from the PassThruReadMsgs that hands it over). A
 * version 1 tester was never sent an EVENT and is sent none. Returns the
 * frames appended (0 or 1). */
static unsigned tell_lost(const j2534_transport_t *t, int slot, uint32_t lost,
                          size_t *used)
{
    static uint32_t s_unsaid;
    static TickType_t s_said_at;
    TickType_t now = xTaskGetTickCount();

    j2534_srv_count_rx_lost(lost);
    s_unsaid += lost;

    if ((TickType_t)(now - s_said_at) >= pdMS_TO_TICKS(1000))
    {
        ESP_LOGW(TAG, "channel %d: %lu received message(s) lost, its queue "
                      "ran over (the tester is not reading, or its link is "
                      "slower than the bus)", slot + 1,
                 (unsigned long)s_unsaid);
        s_unsaid = 0;
        s_said_at = now;
    }

    if (j2534_srv_wire_version() < 2 || j2534_srv_active() != t)
    {
        return 0;
    }

    if (*used + J2534_EVENT_FRAME_SIZE > sizeof(s_rx_batch))
    {
        (void)j2534_srv_send_raw(t, s_session, s_rx_batch, *used);
        *used = 0;
    }

    *used += j2534_event_encode(s_rx_batch + *used, sizeof(s_rx_batch) - *used,
                                (uint16_t)(slot + 1), J2534_EVENT_RX_LOST,
                                lost);
    return 1;
}

/* One pass over the active channels into the batch. Returns the number
 * of frames appended (0 = nothing was pending anywhere). */
static unsigned pump_pass(const j2534_transport_t *t, const bool *ch_used,
                          size_t *used)
{
    unsigned got = 0;

    for (int slot = 0; slot < J2534_MAX_CHANNELS; slot++)
    {
        uint32_t lost = 0;

        if (!ch_used[slot])
        {
            continue;
        }

        /* what the channel lost is asked for before a message is taken
           (inside the poll), and told before that message is pushed */
        bool have = j2534_channel_poll_rx(slot, &s_rx_msg, &lost);

        if (lost > 0)
        {
            got += tell_lost(t, slot, lost, used);
        }

        if (!have)
        {
            continue;
        }

        size_t n = j2534_msg_encode(&s_rx_msg, s_rx_wire, sizeof(s_rx_wire));

        if (n == 0 || j2534_srv_active() != t)
        {
            continue;
        }

        if (*used + J2534_HDR_SIZE + n > sizeof(s_rx_batch))
        {
            /* no room beside what waits: send that first, alone */
            (void)j2534_srv_send_raw(t, s_session, s_rx_batch, *used);
            *used = 0;
        }

        j2534_hdr_encode(s_rx_batch + *used, J2534_MT_RX_MSG, 0,
                         (uint16_t)(slot + 1), (uint32_t)n);
        memcpy(s_rx_batch + *used + J2534_HDR_SIZE, s_rx_wire, n);
        *used += J2534_HDR_SIZE + n;
        got++;
    }

    return got;
}

/* The pump pushes what the channels received as it arrives. 2026-11-06
 * (the VW TP 2.0 speed report: a tool that acknowledges every block of a
 * transfer pays the device's round trip at every block end): the pump
 * blocks on the active channel's bus queue instead of sleeping 2 ms
 * between polls (a frame woke the tester up to 2 ms late), and what has
 * piled up goes out in one transport write instead of one per frame. */
static void rx_pump_task(void *arg)
{
    volatile const bool *run = j2534_srv_run_flag();
    const bool *ch_used = j2534_srv_channels_used();

    (void)arg;

    while (*run)
    {
        /* the session this round's batch belongs to: one that outlives
           its session is not written into the next */
        const j2534_transport_t *t = j2534_srv_active_session(&s_session);

        if (t == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t used = 0;
        unsigned frames = 0;
        unsigned got;

        /* drain: a burst that arrives while the pass runs joins the batch */
        while ((got = pump_pass(t, ch_used, &used)) > 0)
        {
            frames += got;

            if (frames >= J2534_RX_BATCH_MAX)
            {
                break;
            }
        }

        if (used > 0)
        {
            (void)j2534_srv_send_raw(t, s_session, s_rx_batch, used);
            continue; /* straight back for what came in meanwhile */
        }

        /* nothing pending: wait for the next bus frame on the active
           channel's queue (peek: the poll above consumes), or 2 ms for
           what no queue announces (an ISO-TP PDU the provider completes,
           a J1850 reply, an indication) */
        QueueHandle_t q = j2534_channel_wait_queue();
        can_core_frame_t peek;

        if (q == NULL || xQueuePeek(q, &peek, pdMS_TO_TICKS(2)) != pdTRUE)
        {
            if (q == NULL)
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
    }

    s_rx_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t j2534_srv_pump_start(void)
{
    s_rx_task = xTaskCreateStatic(rx_pump_task, "j2534_rx", J2534_TASK_STACK,
                                  NULL, J2534_TASK_PRIO, s_rx_stack, &s_rx_tcb);
    return s_rx_task != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}
