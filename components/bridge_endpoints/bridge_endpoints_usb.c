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
 * @file bridge_endpoints_usb.c
 * @brief The `usb_obd` jack: WiCAN Pro's USB port-B UART (second CDC
 *        channel, UART2 TX=GPIO17/RX=GPIO18). The USB↔OBD transparent
 *        passthrough is a configured `raw` bridge over this jack
 *        (TASK_obd_chip_manager_new §2), enabled in settings.
 */
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bridge_manager.h"

#include "bridge_endpoints_private.h"

static const char *TAG = "bridge_endpoints";

#define USB_UART      UART_NUM_2
#define USB_TX_PIN    17
#define USB_RX_PIN    18
#define USB_BAUD      2000000 /* matches the OBD chip side (meatpi):
                                 no bottleneck through the passthrough */

/* The reader waits for the UART driver's events, not for a count of bytes.
 * Until 2026-11-18 it asked for 128 bytes with a 20 ms limit, and the
 * driver waits its whole limit again behind every arrival, so a command of
 * a few bytes left the jack 20 ms after it came (bench: of 40 short writes
 * to a TCP client the fastest took 20.8 ms, the median 21.5 ms). The wait
 * below is only what a lost event costs. */
#define USB_EVT_QUEUE   32
#define USB_EVT_WAIT_MS 20

static QueueHandle_t s_usb_sub;
static QueueHandle_t s_usb_evt_q;     /* the UART driver's own            */
static uint32_t s_usb_drops;          /* chunks the bridge's queue did not
                                         take                             */
/* Bytes from the PC that the UART lost: its 128 byte FIFO ran over (the
 * interrupt came late: it waits for a flash write, 0.64 ms fill the FIFO
 * at this speed) or the driver's ring was full. Counted per event, the
 * driver does not say how many bytes. In no counter at all until
 * 2026-11-18. */
static uint32_t s_usb_overruns;
static TaskHandle_t s_usb_task;
static StaticTask_t s_usb_tcb;        /* internal: FreeRTOS object       */
static StackType_t s_usb_stack[3072]; /* internal: UART driver path (§2) */

esp_err_t bep_usb_send(const uint8_t *d, size_t l)
{
    return (uart_write_bytes(USB_UART, d, l) == (int)l) ? ESP_OK : ESP_FAIL;
}

int bep_usb_losses(QueueHandle_t q, bridge_loss_t *out, int cap)
{
    (void)q;

    if (cap < 1)
    {
        return 0;
    }

    /* chunks from the PC that the bridge's queue had no room for */
    out[0] = (bridge_loss_t){ "rx_drops", s_usb_drops };

    if (cap < 2)
    {
        return 1;
    }

    out[1] = (bridge_loss_t){ "rx_overruns", s_usb_overruns };
    return 2;
}

esp_err_t bep_usb_subscribe(QueueHandle_t q)
{
    if (s_usb_sub != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_usb_sub = q;
    return ESP_OK;
}

esp_err_t bep_usb_unsubscribe(QueueHandle_t q)
{
    if (s_usb_sub != q)
    {
        return ESP_ERR_NOT_FOUND;
    }

    s_usb_sub = NULL;
    return ESP_OK;
}

static void usb_rx_task(void *arg)
{
    static bridge_chunk_t chunk;

    (void)arg;

    while (true)
    {
        uart_event_t evt;

        if (xQueueReceive(s_usb_evt_q, &evt,
                          pdMS_TO_TICKS(USB_EVT_WAIT_MS)) == pdTRUE &&
            (evt.type == UART_FIFO_OVF || evt.type == UART_BUFFER_FULL))
        {
            s_usb_overruns++;

            if ((s_usb_overruns % 100) == 1)
            {
                ESP_LOGW(TAG, "usb_obd: the UART lost bytes from the PC (%s "
                         "ran over, %lu times so far)",
                         evt.type == UART_FIFO_OVF ? "its FIFO" : "the ring",
                         (unsigned long)s_usb_overruns);
            }
        }

        /* by what the driver holds, not by the event's size: an event lost
           to a full event queue costs the wait above and no byte */
        for (;;)
        {
            size_t have = 0;

            (void)uart_get_buffered_data_len(USB_UART, &have);

            if (have == 0)
            {
                break;
            }

            if (have > sizeof(chunk.data))
            {
                have = sizeof(chunk.data);
            }

            int got = uart_read_bytes(USB_UART, chunk.data, have, 0);

            if (got <= 0)
            {
                break;
            }

            if (s_usb_sub == NULL)
            {
                continue; /* no bridge on the jack: read and gone, as before */
            }

            chunk.len = (uint16_t)got;

            if (xQueueSend(s_usb_sub, &chunk, 0) != pdTRUE)
            {
                s_usb_drops++; /* bounded queue: drop-and-count, never block */

                if ((s_usb_drops % 100) == 1)
                {
                    ESP_LOGW(TAG, "usb_obd subscriber full (%lu drops)",
                             (unsigned long)s_usb_drops);
                }
            }
        }
    }
}

esp_err_t bep_usb_start(void)
{
    uart_config_t cfg =
    {
        .baud_rate = USB_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(USB_UART, 4096, 4096, USB_EVT_QUEUE,
                                        &s_usb_evt_q, 0);

    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_param_config(USB_UART, &cfg);

    if (err == ESP_OK)
    {
        err = uart_set_pin(USB_UART, USB_TX_PIN, USB_RX_PIN,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    if (err != ESP_OK)
    {
        return err;
    }

    s_usb_task = xTaskCreateStatic(usb_rx_task, "usb_obd_rx",
                                   sizeof(s_usb_stack) /
                                       sizeof(s_usb_stack[0]),
                                   NULL, 10, s_usb_stack, &s_usb_tcb);
    return (s_usb_task != NULL) ? ESP_OK : ESP_FAIL;
}
