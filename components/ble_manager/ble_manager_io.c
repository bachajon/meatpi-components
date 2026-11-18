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
 * @file ble_manager_io.c
 * @brief The TX path: a queue-fed task that packs chunks to the negotiated
 *        MTU and notifies FFF1 using the legacy free-packets + congestion
 *        pacing (pure math in ble_manager_pack.c), plus the MTU-chunked CLI
 *        OUT writer.
 *
 * What the pipe may lose, and where (2026-11-18): only a WHOLE unit, at
 * the queue's door, counted (blm_io_queue_tx). A unit is what the bridge
 * hands over in one call: whole slcan lines, whole CAN records. Once it is
 * in the queue every byte of it reaches the central while the link lives:
 * a packet the link does not take in time is kept and offered again, so
 * the central never gets a stream with a piece missing from its middle.
 * Before, a full queue cut a unit at a 128-byte chunk and a packet the
 * link did not take inside 100 ms was dropped with no count; both reached
 * a central as half a record, and a binary reader then read every later
 * record from the wrong place.
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "ble_manager.h"
#include "ble_manager_private.h"

static const char *TAG = "ble_manager";

static QueueHandle_t s_tx_q;
static StaticQueue_t s_tx_q_buf; /* internal: FreeRTOS object */
static uint8_t s_tx_store[BLM_TX_QUEUE_DEPTH * sizeof(ble_chunk_t)]
    EXT_RAM_BSS_ATTR;

static TaskHandle_t s_task;
static StaticTask_t s_tcb; /* internal: FreeRTOS object */
static StackType_t s_stack[4096] EXT_RAM_BSS_ATTR;
static volatile bool s_running;
static blm_pipe_stats_t s_stats; /* the tx fields; small: internal .bss */
static TickType_t s_warned;      /* when "tx queue full" was last said */

/* One packet to the radio. True = it has left this task: sent, or refused
   by the stack for a reason that waiting does not cure (the link is not
   secured, the stack reported an error), then dropped and counted. False =
   the link did not take it inside the notify wait: the caller keeps the
   packet and offers it again. */
static bool emit(const uint8_t *buf, size_t len)
{
    esp_err_t err = blm_gatt_notify_data(buf, (uint16_t)len);

    if (err == ESP_OK)
    {
        blm_core_on_tx(len);
        return true;
    }

    if (err == ESP_ERR_TIMEOUT)
    {
        s_stats.tx_holds++;
        return false;
    }

    s_stats.notify_drops++;
    s_stats.notify_drop_bytes += (uint32_t)len;
    return true;
}

static void tx_task(void *arg)
{
    static uint8_t send_buf[BLM_SEND_BUF_SIZE] EXT_RAM_BSS_ATTR;
    static ble_chunk_t cur;  /* the chunk being packed                   */
    static ble_chunk_t peek;
    size_t send_len = 0;     /* bytes in send_buf                        */
    size_t cur_off = 0;      /* bytes of cur already in a packet         */
    bool have_cur = false;
    bool held = false;       /* send_buf is a packet the link refused    */

    (void)arg;
    ESP_LOGD(TAG, "tx task up");

    while (s_running)
    {
        if (!have_cur && !held &&
            xQueuePeek(s_tx_q, &peek, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            continue;
        }

        if (!blm_gatt_connected())
        {
            /* the link is gone: what was queued for it is dropped (legacy
               behavior), the whole backlog at once, so that the next
               central starts on an empty pipe */
            while (xQueueReceive(s_tx_q, &cur, 0) == pdTRUE)
            {
            }

            send_len = 0;
            have_cur = false;
            held = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int free_packets = blm_gatt_free_packets();

        if (free_packets <= 0 || blm_gatt_congested())
        {
            vTaskDelay(pdMS_TO_TICKS(1)); /* legacy 1 ms pacing */
            continue;
        }

        size_t max_data = blm_gatt_max_data();

        if (send_len > max_data)
        {
            /* packed for a link that is gone (a reconnect inside one
               notify wait, with a smaller MTU): it cannot be sent */
            s_stats.notify_drops++;
            s_stats.notify_drop_bytes += (uint32_t)send_len;
            send_len = 0;
            held = false;
        }

        if (held)
        {
            if (!emit(send_buf, send_len))
            {
                continue; /* still not taken (emit waited for it) */
            }

            held = false;
            send_len = 0;
            free_packets--;
        }

        /* pack as many chunks as the free packets allow (legacy loop) */
        while (!held && free_packets > 0)
        {
            if (!have_cur)
            {
                if (xQueuePeek(s_tx_q, &peek, 0) != pdTRUE ||
                    free_packets < blm_pack_packets_needed(send_len, peek.len,
                                                           max_data))
                {
                    break; /* nothing queued, or not enough credits for it */
                }

                xQueueReceive(s_tx_q, &cur, 0);
                cur_off = 0;
                have_cur = true;
            }

            while (cur_off < cur.len && free_packets > 0 && !held)
            {
                if (blm_pack_fill(send_buf, &send_len, max_data, cur.data,
                                  cur.len, &cur_off))
                {
                    if (emit(send_buf, send_len))
                    {
                        send_len = 0;
                        free_packets--;
                    }
                    else
                    {
                        held = true;
                    }
                }
            }

            if (cur_off < cur.len)
            {
                /* out of credits, or a packet is held, inside the chunk:
                   the rest of it goes out in a later pass */
                break;
            }

            have_cur = false;
        }

        /* a packet that is not full leaves when nothing more can join it
           now (legacy tail flush) */
        if (!held && !have_cur && free_packets > 0 && send_len != 0)
        {
            if (emit(send_buf, send_len))
            {
                send_len = 0;
            }
            else
            {
                held = true;
            }
        }
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t blm_io_start(void)
{
    if (s_tx_q == NULL)
    {
        s_tx_q = xQueueCreateStatic(BLM_TX_QUEUE_DEPTH, sizeof(ble_chunk_t),
                                    s_tx_store, &s_tx_q_buf);
    }

    xQueueReset(s_tx_q);
    s_running = true;
    s_task = xTaskCreateStatic(tx_task, "ble_tx",
                               sizeof(s_stack) / sizeof(s_stack[0]), NULL, 5,
                               s_stack, &s_tcb);
    return (s_task != NULL) ? ESP_OK : ESP_FAIL;
}

void blm_io_stop(void)
{
    s_running = false;

    for (int i = 0; i < 25 && s_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t blm_io_queue_tx(const uint8_t *data, size_t len)
{
    ble_chunk_t chunk;

    /* a unit goes in whole or not at all (never block the caller, the
       bridge pump). One producer feeds the pipe at a time (the one bridge
       on it, or the bench blast), so the room seen here is still there
       in the loop below */
    if (uxQueueSpacesAvailable(s_tx_q) < blm_pack_chunks(len))
    {
        s_stats.tx_drops++;
        s_stats.tx_drop_bytes += (uint32_t)len;

        /* said once a second at most. A busy bus is a thousand drops a
           second: a line for every hundredth was ten lines a second on
           the console exactly while the pipe was busiest */
        TickType_t now = xTaskGetTickCount();

        if (s_stats.tx_drops == 1 || (now - s_warned) >= pdMS_TO_TICKS(1000))
        {
            s_warned = now;
            ESP_LOGW(TAG, "tx queue full (%lu units dropped, %lu bytes)",
                     (unsigned long)s_stats.tx_drops,
                     (unsigned long)s_stats.tx_drop_bytes);
        }

        return ESP_FAIL;
    }

    while (len > 0)
    {
        size_t n = (len > BLE_MANAGER_CHUNK_SIZE) ? BLE_MANAGER_CHUNK_SIZE
                                                  : len;

        chunk.len = (uint16_t)n;
        memcpy(chunk.data, data, n);

        if (xQueueSend(s_tx_q, &chunk, 0) != pdTRUE)
        {
            /* a second producer took the room in between: the rest of
               this unit is lost, and counted */
            s_stats.tx_drops++;
            s_stats.tx_drop_bytes += (uint32_t)len;
            return ESP_FAIL;
        }

        data += n;
        len -= n;
    }

    return ESP_OK;
}

uint32_t blm_io_tx_drops(void)
{
    return s_stats.tx_drops;
}

void blm_io_pipe_stats(blm_pipe_stats_t *out)
{
    out->tx_drops = s_stats.tx_drops;
    out->tx_drop_bytes = s_stats.tx_drop_bytes;
    out->tx_holds = s_stats.tx_holds;
    out->notify_drops = s_stats.notify_drops;
    out->notify_drop_bytes = s_stats.notify_drop_bytes;
}

esp_err_t blm_io_cli_write(const char *data, size_t len)
{
    if (!blm_gatt_connected())
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* legacy ble_cmdline_output: MTU-chunked, bounded wait for credits */
    size_t offset = 0;

    while (offset < len)
    {
        int tries = 0;

        while (blm_gatt_free_packets() <= 0 && tries++ < 100)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        size_t chunk = len - offset;
        size_t max_data = blm_gatt_max_data();

        if (chunk > max_data)
        {
            chunk = max_data;
        }

        blm_gatt_notify_cli((const uint8_t *)(data + offset),
                            (uint16_t)chunk);
        offset += chunk;
    }

    return ESP_OK;
}
