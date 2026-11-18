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
 * @file bridge_endpoints_can.c
 * @brief The `can` jack (internal CAN-frame endpoint for the translators).
 *
 * Wraps can_manager: send() decodes a can_frame_wire chunk and injects it
 * on the bus; a pump task subscribes to every RX frame, encodes each into
 * a chunk, and feeds the subscribed bridges' queues.
 *
 * multi_consumer since 2026-07-26 (mqtt_can v2 backlog): up to
 * BEP_CAN_MAX_SUBS bridges may sit on the jack at once (slcan + mqtt
 * simultaneously), every subscriber gets a full copy of each RX chunk,
 * TX from any bridge serializes through can_manager_send. ONE ingress
 * filter for the whole jack (bep_can_set_filter, mqtt_can's setting):
 * with several consumers attached the filter restricts what ALL of them
 * see, per-consumer filters are out of scope (documented in the README).
 * The pump runs only while at least one bridge is subscribed.
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bridge_manager.h"
#include "can_frame_wire.h"
#include "can_manager.h"
#include "can_core.h"

#include "bridge_endpoints_private.h"

#define BEP_CAN_MAX_SUBS 4

static const char *TAG = "bridge_endpoints";

/* The frames can_manager hands the jack. 256 since 2026-11-18, as the CAN
 * node's own receive queue: a flash erase stops every task for about 23 ms,
 * the frames of that time then arrive here at once, and 32 places lost an
 * slcan client 384 frames of a bus at 2000 frames a second while three
 * files were written (886 at 3000, none at 1100). PSRAM, static. */
#define BEP_CAN_RXQ_FRAMES 256

static QueueHandle_t s_can_rx_q;        /* can_core_frame_t from can_manager */
static StaticQueue_t s_can_rx_q_buf;                         /* internal: FreeRTOS */
static uint8_t s_can_rx_q_store[BEP_CAN_RXQ_FRAMES * sizeof(can_core_frame_t)]
    EXT_RAM_BSS_ATTR;                                        /* PSRAM: 5 KB        */
static QueueHandle_t s_can_qs[BEP_CAN_MAX_SUBS]; /* bridge queues (fan-out)   */
static int s_can_nsubs;
static portMUX_TYPE s_can_lock = portMUX_INITIALIZER_UNLOCKED; /* internal: spinlock */
static int s_can_sub_idx = -1;
static uint32_t s_can_filter;           /* mask==0 -> monitor all (default)    */
static uint32_t s_can_mask;
static bool s_can_filter_ext;
static TaskHandle_t s_can_pump;
static StaticTask_t s_can_pump_tcb;                          /* internal: FreeRTOS */
static StackType_t s_can_pump_stack[3072] EXT_RAM_BSS_ATTR;  /* PSRAM: no flash    */
static volatile bool s_can_pump_run;
/* chunks a bridge's full queue did not take, per subscriber slot (zeroed
   when a bridge takes the slot): each bridge's own share on /api/bridges */
static uint32_t s_can_rx_drops[BEP_CAN_MAX_SUBS];
static uint32_t s_can_tx_malformed;     /* chunks that were not whole frames   */

/* ---- TX: a queue of the jack's own, so no bridge pump waits for the bus ------
 *
 * A client's burst (an slcan tool that writes a whole ISO-TP block, a log
 * replay) reaches this jack faster than the bus takes it. The controller
 * holds 16 frames in flight and refuses the next one at once, so of 400
 * lines written in one go 161 reached the bus (2026-11-17, the same on
 * 6.0.0-beta.11).
 *
 * Waiting for room inside send() delivered the burst and cost the other
 * direction: send() runs in the bridge's pump, which serves both, and 2000
 * lines written into a bus that streamed 1100 frames a second to the same
 * client lost that client 162 frames, 3938 at 3000 (2026-11-18). So send()
 * only queues, a task of this jack feeds the controller at the bus's pace,
 * and bep_can_tx_ready() tells the pump when the queue is too full for
 * another pass: the pump leaves the client's chunks where they are, and
 * the client's own flow control (TCP) holds it back. */
#define BEP_CAN_TXQ_FRAMES 1024

/* The most frames one pump pass hands over: BRIDGE_MANAGER_PASS_MAX bytes
 * of the shortest slcan line ("t1230" and a CR, 6 bytes). "Ready" promises
 * room for one such pass from each bridge on the jack, so pumps that ask
 * at the same moment all fit. */
#define BEP_CAN_TX_PASS_FRAMES (BRIDGE_MANAGER_PASS_MAX / 6 + 1)
#define BEP_CAN_TX_READY_ROOM  (BEP_CAN_TX_PASS_FRAMES * BEP_CAN_MAX_SUBS)

_Static_assert(BEP_CAN_TX_READY_ROOM < BEP_CAN_TXQ_FRAMES,
               "the can jack's TX queue is smaller than what ready promises");

static QueueHandle_t s_can_txq;
static StaticQueue_t s_can_txq_buf;                          /* internal: FreeRTOS */
static uint8_t s_can_txq_store[BEP_CAN_TXQ_FRAMES * sizeof(can_core_frame_t)]
    EXT_RAM_BSS_ATTR;                                        /* PSRAM: 20 KB       */
static StaticTask_t s_can_tx_tcb;                            /* internal: FreeRTOS */
static StackType_t s_can_tx_stack[3072] EXT_RAM_BSS_ATTR;    /* PSRAM: no flash    */
static uint32_t s_can_tx_drops;         /* frames that found the queue full    */
static uint32_t s_can_tx_refused;       /* frames the controller did not take  */

void bep_can_set_filter(uint32_t filter, uint32_t mask, bool ext)
{
    /* pre-bridge-start config (mqtt_can's 1:1 filter mapping, meatpi
       2026-07-22). mask==0 keeps the monitor-all default. Applies at
       the FIRST subscribe: set before bridge_manager_start(). Jack-wide:
       every consumer sees the same (filtered) stream. */
    s_can_filter = filter;
    s_can_mask = mask;
    s_can_filter_ext = ext;
}

/* The TX task waits for room in the controller, up to this long per frame:
 * at 500 kbit/s a frame leaves every quarter of a millisecond. */
#define BEP_CAN_TX_WAIT_MS 100

/* A line that never goes idle (held dominant) frees no slot. A bus without
 * another node is not that case: the driver gives a frame nobody
 * acknowledges up after three more tries and its slot comes back. After a
 * wait that ran out, frames are handed over without a wait for this long:
 * the controller refuses them at once and the queue runs empty, so the
 * clients are not held back for ever by a bus that is gone. */
#define BEP_CAN_TX_GIVE_UP_MS 1000

static volatile TickType_t s_can_tx_no_wait_until;
static volatile bool s_can_tx_no_wait;

/** Room for one frame, waited for when the controller is full. Returns at
 *  once when there is room, when the node may not talk (a state no wait
 *  changes) and during the give-up time. */
static void can_tx_wait_room(void)
{
    if (can_manager_tx_room() || !can_manager_tx_ready())
    {
        return;
    }

    TickType_t now = xTaskGetTickCount();

    if (s_can_tx_no_wait)
    {
        if ((int32_t)(now - s_can_tx_no_wait_until) < 0)
        {
            return;
        }

        s_can_tx_no_wait = false;
    }

    TickType_t step = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;

    while (!can_manager_tx_room())
    {
        if (xTaskGetTickCount() - now >= pdMS_TO_TICKS(BEP_CAN_TX_WAIT_MS))
        {
            s_can_tx_no_wait_until = xTaskGetTickCount() +
                                     pdMS_TO_TICKS(BEP_CAN_TX_GIVE_UP_MS);
            s_can_tx_no_wait = true;
            ESP_LOGW(TAG, "can: the bus took no frame for %d ms, a client's "
                     "frames are dropped (%lu refused so far, %lu found the "
                     "queue full)", BEP_CAN_TX_WAIT_MS,
                     (unsigned long)s_can_tx_refused,
                     (unsigned long)s_can_tx_drops);
            return;
        }

        vTaskDelay(step);
    }
}

static void can_tx_task(void *arg)
{
    can_core_frame_t f;

    (void)arg;

    while (true)
    {
        if (xQueueReceive(s_can_txq, &f, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        can_tx_wait_room();

        if (can_manager_send(f.id, f.ext, f.rtr, f.data, f.dlc) != ESP_OK)
        {
            s_can_tx_refused++;
        }
    }
}

void bep_can_tx_init(void)
{
    if (s_can_txq != NULL)
    {
        return;
    }

    s_can_txq = xQueueCreateStatic(BEP_CAN_TXQ_FRAMES, sizeof(can_core_frame_t),
                                   s_can_txq_store, &s_can_txq_buf);

    if (s_can_txq == NULL ||
        xTaskCreateStatic(can_tx_task, "can_bridge_tx",
                          sizeof(s_can_tx_stack) / sizeof(s_can_tx_stack[0]),
                          NULL, 10, s_can_tx_stack, &s_can_tx_tcb) == NULL)
    {
        ESP_LOGE(TAG, "can: no TX task, the jack sends nothing");
    }
}

bool bep_can_tx_ready(void)
{
    return s_can_txq == NULL ||
           uxQueueSpacesAvailable(s_can_txq) >= BEP_CAN_TX_READY_ROOM;
}

esp_err_t bep_can_send(const uint8_t *d, size_t l)
{
    can_core_frame_t f;
    size_t off = 0;
    size_t n;

    while ((n = can_wire_decode_next(d + off, l - off, &f)) > 0)
    {
        if (s_can_txq == NULL || !can_manager_tx_ready())
        {
            /* the node may not talk (listen-only, stopped, no bitrate): a
               state no wait changes. Refused at once, counted and said by
               can_manager, and the bridge counts a send error as before */
            esp_err_t err = can_manager_send(f.id, f.ext, f.rtr, f.data, f.dlc);

            if (err != ESP_OK)
            {
                return err;
            }
        }
        else if (xQueueSend(s_can_txq, &f, 0) != pdTRUE)
        {
            s_can_tx_drops++;
            return ESP_ERR_NO_MEM;
        }

        off += n;
    }

    if (l == 0 || off < l)
    {
        s_can_tx_malformed++;
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/* Until 2026-11-18 these four stood in a log line at best: a client's burst
   that the bus did not take was lost with every counter of every route at
   rest. */
int bep_can_losses(QueueHandle_t q, bridge_loss_t *out, int cap)
{
    uint32_t rx = 0;
    int n = 0;

    for (int i = 0; i < BEP_CAN_MAX_SUBS; i++)
    {
        if (s_can_qs[i] == q)
        {
            rx = s_can_rx_drops[i];
        }
    }

    const bridge_loss_t all[] =
    {
        { "rx_drops", rx },                   /* this bridge's queue was full */
        { "tx_drops", s_can_tx_drops },       /* the jack's TX queue was full */
        { "tx_refused", s_can_tx_refused },   /* the controller did not take  */
        { "tx_malformed", s_can_tx_malformed },
    };

    while (n < cap && n < (int)(sizeof(all) / sizeof(all[0])))
    {
        out[n] = all[n];
        n++;
    }

    return n;
}

static void can_pump_task(void *arg)
{
    can_core_frame_t f;
    bridge_chunk_t c;

    (void)arg;

    while (s_can_pump_run)
    {
        if (xQueueReceive(s_can_rx_q, &f, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            c.len = (uint16_t)can_wire_encode(&f, c.data);

            /* COALESCE: pack whatever the bus already queued into the
             * same chunk (a chunk is a plain concatenation of wire
             * frames; every consumer loops via can_wire_decode_next).
             * Lifts frame-bound transports (WS was 1 frame = 1 WS
             * frame = ~676 fps) toward bus rate at zero added latency
             * (only frames ALREADY waiting are packed; an idle bus
             * still ships 1-frame chunks immediately). */
            while (c.len + CAN_WIRE_MAX <= BRIDGE_MANAGER_CHUNK_SIZE &&
                   xQueueReceive(s_can_rx_q, &f, 0) == pdTRUE)
            {
                c.len += (uint16_t)can_wire_encode(&f, c.data + c.len);
            }

            /* fan-out: every subscribed bridge gets a full copy.
             * Snapshot under the lock, send OUTSIDE it (queue API is
             * forbidden inside an IDF critical section). */
            QueueHandle_t qs[BEP_CAN_MAX_SUBS];

            portENTER_CRITICAL(&s_can_lock);
            memcpy(qs, (const void *)s_can_qs, sizeof(qs));
            portEXIT_CRITICAL(&s_can_lock);

            for (int i = 0; i < BEP_CAN_MAX_SUBS; i++)
            {
                if (qs[i] != NULL && xQueueSend(qs[i], &c, 0) != pdTRUE)
                {
                    s_can_rx_drops[i]++; /* that bridge's queue is full */
                }
            }
        }
    }

    s_can_pump = NULL;
    vTaskDelete(NULL);
}

static void bep_can_rebind(void)
{
    esp_err_t err;

    if (s_can_nsubs <= 0 || s_can_rx_q == NULL)
    {
        return;
    }

    err = can_manager_subscribe_queue(
        s_can_rx_q, s_can_filter, s_can_mask, s_can_filter_ext,
        s_can_mask == 0, &s_can_sub_idx);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "can bridge resubscribe failed: %s", esp_err_to_name(err));
        return;
    }

    (void)can_manager_subscriber_name(s_can_sub_idx, "bridge");
    ESP_LOGI(TAG, "can bridge resubscribed after the bus came back");
}

void bep_can_bind_restart(void)
{
    can_manager_on_bus_up(bep_can_rebind);
}

esp_err_t bep_can_subscribe(QueueHandle_t q)
{
    int slot = -1;

    portENTER_CRITICAL(&s_can_lock);

    for (int i = 0; i < BEP_CAN_MAX_SUBS; i++)
    {
        if (s_can_qs[i] == NULL && slot < 0)
        {
            slot = i;
        }
    }

    portEXIT_CRITICAL(&s_can_lock);

    if (slot < 0)
    {
        return ESP_ERR_NO_MEM; /* BEP_CAN_MAX_SUBS bridges already on */
    }

    s_can_rx_drops[slot] = 0;

    if (s_can_nsubs > 0)
    {
        /* pump + can_manager subscription already live: just join */
        portENTER_CRITICAL(&s_can_lock);
        s_can_qs[slot] = q;
        s_can_nsubs++;
        portEXIT_CRITICAL(&s_can_lock);
        return ESP_OK;
    }

    if (s_can_rx_q == NULL)
    {
        s_can_rx_q = xQueueCreateStatic(BEP_CAN_RXQ_FRAMES,
                                        sizeof(can_core_frame_t),
                                        s_can_rx_q_store, &s_can_rx_q_buf);

        if (s_can_rx_q == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = can_manager_subscribe_queue(
        s_can_rx_q, s_can_filter, s_can_mask, s_can_filter_ext,
        s_can_mask == 0, &s_can_sub_idx);
    if (err != ESP_OK)
    {
        return err;
    }

    (void)can_manager_subscriber_name(s_can_sub_idx, "bridge");

    s_can_qs[slot] = q;
    s_can_nsubs = 1;
    s_can_pump_run = true;
    s_can_pump = xTaskCreateStatic(can_pump_task, "can_bridge",
                                   sizeof(s_can_pump_stack) /
                                       sizeof(s_can_pump_stack[0]),
                                   NULL, 9, s_can_pump_stack, &s_can_pump_tcb);
    if (s_can_pump == NULL)
    {
        can_manager_unsubscribe_queue(s_can_sub_idx);
        s_can_sub_idx = -1;
        s_can_qs[slot] = NULL;
        s_can_nsubs = 0;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bep_can_unsubscribe(QueueHandle_t q)
{
    portENTER_CRITICAL(&s_can_lock);

    for (int i = 0; i < BEP_CAN_MAX_SUBS; i++)
    {
        if (s_can_qs[i] == q)
        {
            s_can_qs[i] = NULL;
            s_can_nsubs--;
        }
    }

    int left = s_can_nsubs;

    portEXIT_CRITICAL(&s_can_lock);

    if (left > 0)
    {
        return ESP_OK; /* other bridges still on the jack */
    }

    s_can_pump_run = false; /* last one out: the pump self-deletes */

    if (s_can_sub_idx >= 0)
    {
        can_manager_unsubscribe_queue(s_can_sub_idx);
        s_can_sub_idx = -1;
    }

    return ESP_OK;
}
