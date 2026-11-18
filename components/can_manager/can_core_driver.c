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
 * @file can_core_driver.c
 * @brief The TWAI node driver half of can_core (esp_driver_twai node API,
 *        port 2026-07-21): the ISR callbacks that copy every frame into a
 *        static raw queue, and the node's creation. Split out of can_core.c
 *        2026-10-02 (700-line rule); the start on the configured ISR core,
 *        the stop and the handle's lock are in can_core_node.c since
 *        2026-10-05.
 */

#include <string.h>

#include "can_core.h"
#include "can_core_driver.h"
#include "can_timing_core.h"

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_clk_tree.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "hal/twai_ll.h"      /* the storm throttle masks the controller's
                                 interrupts from the ISR (see can_on_error) */
#include "soc/twai_struct.h"  /* TWAI: the one controller of the ESP32-S3  */
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "can_core";

/* ---- esp_driver_twai node state (port 2026-07-21) ------------------------
 * RX frames are copied OUT of the driver in the on_rx_done ISR into a
 * STATIC queue that can_core owns and NEVER deletes: a reader blocked
 * on it cannot race a driver teardown, so the class of the 2026-07-21
 * sleep-entry panic (twai_receive vs twai_driver_uninstall) is
 * structurally impossible with this shape.
 *
 * The depth is what a flash write needs (2026-11-17, 64 until then). An
 * erase stops every task for its length and leaves the receive interrupt
 * running, so the frames of that time have nowhere to go but this queue,
 * and it has to be internal RAM: the interrupt runs with the cache off,
 * where PSRAM cannot be reached. Measured on the bench, one erase holds
 * the RX task about 23 ms, 90 frames at the 500 kbit/s line rate. 256
 * frames are 64 ms at 500 kbit/s and 32 ms at 1 Mbit/s; 20 bytes a frame,
 * 5120 bytes in all, 3840 more than at 64. The proof is the flash tier of
 * the firmware repo's can_conservation_test.py. */
#define CAN_CORE_RXQ_DEPTH 256

twai_node_handle_t can_drv_node;

/* What the ISR counts. INTERNAL RAM, on purpose: the TWAI interrupt is
 * cache-safe (it runs while a flash write has the cache disabled) and the
 * handle it gets as context lives in PSRAM, which is unreachable then. A
 * PSRAM touch from the ISR is a "Cache error" panic: bench 2026-10-03, the
 * error callback counted into handle->stats and the DUT crashed on the
 * first flash write under a mismatched bus (thousands of error interrupts
 * per second). The ISR never dereferences its context. */
static volatile can_drv_counters_t s_isr;
static volatile bool s_isr_listen;                /* node is listen-only */

static StaticQueue_t      s_rxq_buf;              /* internal: FreeRTOS */
static uint8_t            s_rxq_store[CAN_CORE_RXQ_DEPTH *
                                      sizeof(can_rx_raw_t)];
QueueHandle_t      can_drv_rx_q;
volatile bool      can_drv_evt_bus_off;
volatile bool      can_drv_evt_recovered;
volatile bool      can_drv_evt_storm;

/* ---- the receive-error storm -------------------------------------------------------
 * A controller listening at a bitrate close to the bus's (83.3 against 95.2
 * kbit/s, 95.2 against 100) sees a bit error every few bit times and raises
 * a bus-error interrupt for each: tens of thousands a second. The level-1
 * dispatcher serves the interrupt that re-asserts fastest first, so the
 * tick starves and the interrupt watchdog resets the chip (caught on the
 * console 2026-10-03 16:31:00 during the autobaud bench's walk from 83 to 95
 * kbit/s: core 0 idle, `_xt_lowint1` busy for 300 ms; two such resets in
 * five runs before that). Nothing at task level can help: the task never
 * runs. So the error ISR counts errors per window and, past the storm
 * threshold, MASKS the controller's interrupts (twai_ll_set_enabled_intrs:
 * the register, not the driver). The node is deaf from then on, which the
 * link policy already handles (link_look_at_the_line: a busy RX line with a
 * controller that says nothing = mismatch evidence) by bouncing the node,
 * and every bounce starts a new node with its interrupts on. */
#define CAN_DRV_STORM_WINDOW_US 20000u /* errors are counted per window     */
#define CAN_DRV_STORM_ERRS      400u   /* 20 000/s: a wrong bitrate on a bus
                                          at line rate makes ~1800/s        */
static uint32_t s_storm_t0;          /* window start, esp_timer low word   */
static uint32_t s_storm_n;
static volatile bool s_storm_masked; /* the interrupts are masked right now */

/* ---- TX slots -----------------------------------------------------------------
 * The frame handed to twai_node_transmit() must live until its tx_done
 * interrupt: the node driver keeps the pointer (esp_twai_onchip.c,
 * _node_queue_tx) and formats the frame into the hardware when it comes
 * free. INTERNAL RAM: the HAL reads the buffer from the interrupt, and a
 * flash write has the cache (PSRAM too) disabled meanwhile. A slot is busy
 * from the transmit call to the tx_done that ends its attempts, or to the
 * bus-off that ends the frame inside the controller without a tx_done
 * (can_tx_slots.h: which slot is taken and which one the controller holds;
 * until 2026-11-18 that slot stayed taken, one gone with every bus-off). */
typedef struct
{
    twai_frame_t     f;
    uint8_t          buf[8];
    uint32_t         arb_at;     /* s_isr.arb_lost when the attempt went out:
                                    moved = the attempt lost arbitration    */
} can_drv_tx_slot_t;

static can_drv_tx_slot_t s_tx[CAN_DRV_TX_SLOTS];
static can_txs_t         s_txs;  /* internal: the interrupt reads it */
static portMUX_TYPE      s_tx_lock = portMUX_INITIALIZER_UNLOCKED;

/* One task at a time between "slot taken" and "frame handed to the node
 * driver": the driver's queue then holds the frames in the order the slots
 * were taken, which is how can_tx_slots tells the frame inside the
 * controller from the ones behind it. */
static SemaphoreHandle_t s_tx_order;
static StaticSemaphore_t s_tx_order_buf;          /* internal: FreeRTOS */

static bool IRAM_ATTR can_on_rx_done(twai_node_handle_t node,
                                     const twai_rx_done_event_data_t *e,
                                     void *ctx)
{
    BaseType_t hp = pdFALSE;
    uint8_t buf[8];
    twai_frame_t f = { .buffer = buf, .buffer_len = sizeof(buf) };

    (void)e;
    (void)ctx; /* PSRAM: not from an ISR (see s_isr) */

    /* ONE receive per on_rx_done event: the driver fires the callback
     * per frame, and re-calling receive_from_isr re-reads the SAME
     * frame (found at full 500k line rate 2026-07-21: a drain loop
     * here duplicated frames ~30x into the queue) */
    if (twai_node_receive_from_isr(node, &f) == ESP_OK)
    {
        can_rx_raw_t raw;

        raw.id    = f.header.id;
        /* stamp HERE, not in the dispatch task: the queue adds up to a
           few ms of jitter under load and consumers (BLE raw stream,
           GVRET, J2534, the logger) want bus time. ISR-safe. */
        raw.ts_us = (uint32_t)esp_timer_get_time();
        raw.flags = (uint8_t)((f.header.ide ? CAN_RX_RAW_EXT : 0) |
                              (f.header.rtr ? CAN_RX_RAW_RTR : 0));
        raw.dlc   = (uint8_t)(f.header.dlc > 8 ? 8 : f.header.dlc);
        memcpy(raw.data, buf, raw.dlc);

        if (xQueueSendFromISR(can_drv_rx_q, &raw, &hp) != pdTRUE)
        {
            s_isr.rx_missed++;
        }
    }
    else
    {
        s_isr.rx_missed++; /* the driver had a frame and did not hand it over */
    }

    return hp == pdTRUE;
}

/* ---- frames the controller lost ------------------------------------------------
 * The controller's receive FIFO is 64 BYTES: four to five 29-bit frames. When
 * the interrupt is served later than that (2.5 ms at 250 kbit/s line rate,
 * 1.2 ms at 500), the frames that arrive meanwhile are kept as "overrun"
 * slots without data. The IDF node driver (esp_twai_onchip.c, v6.0.2) reads
 * the FIFO with twai_hal_read_rx_fifo(), which releases such a slot and
 * returns false, and the driver moves on: no callback, no counter. Bench
 * 2026-10-02 (M4): 1 or 2 frames of 20000 gone at 250 kbit/s line rate with
 * every counter at zero.
 *
 * The linker wraps that HAL call (CMakeLists.txt: -Wl,--wrap=...), so every
 * read of the driver's interrupt passes through here and a false is counted.
 * If a later IDF renames the function the build fails on __real_..., which
 * is the wanted outcome: silent is what this exists to end. The arguments
 * are the HAL's context and frame pointers, passed through untouched. */
bool __real_twai_hal_read_rx_fifo(void *hal_ctx, void *rx_frame);

bool IRAM_ATTR __wrap_twai_hal_read_rx_fifo(void *hal_ctx, void *rx_frame)
{
    bool ok = __real_twai_hal_read_rx_fifo(hal_ctx, rx_frame);

    if (!ok)
    {
        s_isr.rx_overrun++;
    }

    return ok;
}

/** tx_done: free the slot, or give the lost frame another go (the controller
 *  is single shot: fail_retry_cnt 0, see can_drv_create). The node driver
 *  takes a transmit from an interrupt (its queue path is ISR-aware).
 *
 *  Two ways a single-shot frame fails, two budgets: lost ARBITRATION (a
 *  higher-priority frame started at the same time: normal on a busy bus, no
 *  error, the controller's error counters stay put) is retried up to
 *  CAN_DRV_TX_RETRIES_ARB times, as a controller with automatic
 *  retransmission would keep trying; an ERROR (no acknowledge: nobody on the
 *  bus, or a bit read back wrong) only CAN_DRV_TX_RETRIES times, because
 *  every such attempt adds 8 to the transmit error counter and a frame
 *  retried for ever on a dead bus parks the node error-passive, then
 *  bus-off. The two are told apart by the arb_lost error event the HAL
 *  raises for the attempt (s_isr.arb_lost moved since it went out); when
 *  that event comes after this one, the attempt counts as an error, which
 *  is the safe side. Bench 2026-10-03 (J1939 active, a 1000 frame/s flood of
 *  priority-0 frames): 2 of 20 requests lost with one budget of 3. */
static bool IRAM_ATTR can_on_tx_done(twai_node_handle_t node,
                                     const twai_tx_done_event_data_t *ev,
                                     void *ctx)
{
    (void)ctx; /* PSRAM: not from an ISR (see s_isr) */

    for (int i = 0; i < CAN_DRV_TX_SLOTS; i++)
    {
        can_drv_tx_slot_t *slot = &s_tx[i];

        if (ev->done_tx_frame != &slot->f || s_txs.tries[i] == 0)
        {
            continue;
        }

        if (ev->is_tx_success)
        {
            s_isr.tx_done++;
            can_txs_release(&s_txs, i);
            break;
        }

        bool arb = (s_isr.arb_lost != slot->arb_at);
        uint8_t budget = arb ? CAN_DRV_TX_RETRIES_ARB : CAN_DRV_TX_RETRIES;

        if (s_txs.tries[i] <= budget)
        {
            slot->arb_at = s_isr.arb_lost;

            /* ahead of the frames handed over after it: the node driver's
               queue takes the higher priority first, and a frame sent
               again at the queue's own priority went out BEHIND up to 15
               later ones (bench 2026-11-18: of a burst of 2000 frames on
               a bus with 2000 frames a second of other traffic 349 left
               out of place). One frame is on the wire at a time, so at
               most one waits at this priority. */
            slot->f.tx_queue_priority = 1;

            if (twai_node_transmit(node, &slot->f, 0) == ESP_OK)
            {
                s_txs.tries[i]++;
                s_isr.tx_retries++;
                break;
            }
        }

        s_isr.tx_lost++;
        can_txs_release(&s_txs, i);
        break;
    }

    return false;
}

static bool IRAM_ATTR can_on_state_change(
    twai_node_handle_t node, const twai_state_change_event_data_t *e,
    void *ctx)
{
    (void)node;
    (void)ctx;

    if (e->new_sta == TWAI_ERROR_BUS_OFF)
    {
        can_drv_evt_bus_off = true;

        /* The frame inside the controller is ended by the bus-off and the
           node driver forgets it without a tx_done (esp_twai_onchip.c: at
           the recovery it sends what its queue holds and overwrites or
           clears the frame it had handed to the hardware). Its slot comes
           back here, counted as a lost frame. The frames behind it are
           still in the driver's queue and go out after the recovery, each
           with its own tx_done: their slots stay taken. Should the
           hardware report this frame after all, in this same interrupt,
           tx_done finds the slot free and leaves it alone. */
        if (can_txs_bus_off(&s_txs) >= 0)
        {
            s_isr.tx_lost++;
        }
    }
    else if (e->old_sta == TWAI_ERROR_BUS_OFF)
    {
        can_drv_evt_recovered = true;
    }

    return false;
}

/* Receive errors are the evidence the listen-before-talk policy reads: a
 * wrong bitrate shows as thousands of them per second and not one frame.
 * While the node is listen-only nothing is transmitted, so every error
 * counts; in normal mode the errors of our own transmissions (no ACK, a bit
 * read back differently, arbitration lost) are not evidence about the bus. */
static bool IRAM_ATTR can_on_error(twai_node_handle_t node,
                                   const twai_error_event_data_t *e,
                                   void *ctx)
{
    BaseType_t hp = pdFALSE;
    uint32_t now = (uint32_t)esp_timer_get_time();

    (void)node;
    (void)ctx; /* PSRAM: not from an ISR (see s_isr) */

    if (now - s_storm_t0 > CAN_DRV_STORM_WINDOW_US)
    {
        s_storm_t0 = now;
        s_storm_n = 0;
    }

    if (++s_storm_n >= CAN_DRV_STORM_ERRS && !s_storm_masked)
    {
        /* the storm: no more interrupts from this node until the link
           policy bounces it; the RX task is woken to look at the line */
        can_rx_raw_t ev = { .flags = CAN_RX_RAW_EVT };

        twai_ll_set_enabled_intrs(&TWAI, 0);
        s_storm_masked = true;
        s_isr.rx_storms++;
        can_drv_evt_storm = true;
        (void)xQueueSendFromISR(can_drv_rx_q, &ev, &hp);
        return hp == pdTRUE;
    }

    if (e->err_flags.stuff_err)
    {
        s_isr.err_stuff++;
    }
    else if (e->err_flags.form_err)
    {
        s_isr.err_form++;
    }
    else if (e->err_flags.bit_err)
    {
        s_isr.err_bit++;
    }
    else if (e->err_flags.ack_err)
    {
        s_isr.err_ack++;
    }
    else if (!e->err_flags.arb_lost)
    {
        s_isr.err_other++;
    }

    if (e->err_flags.arb_lost)
    {
        s_isr.arb_lost++;
    }
    else if (s_isr_listen ||
             (!e->err_flags.ack_err && !e->err_flags.bit_err))
    {
        s_isr.rx_bad++;

        /* every CAN_AB_BAD_MIN of them wake the RX task: a talking node
           on a bus that turned unreadable must be demoted within
           milliseconds, not at the end of a 100 ms receive slice. A full
           queue means the task is busy anyway. */
        if ((s_isr.rx_bad % CAN_AB_BAD_MIN) == 0)
        {
            can_rx_raw_t ev = { .flags = CAN_RX_RAW_EVT };

            (void)xQueueSendFromISR(can_drv_rx_q, &ev, &hp);
        }
    }

    return hp == pdTRUE;
}

void can_drv_counters(can_drv_counters_t *out)
{
    /* plain 32-bit reads of counters only the ISR writes */
    out->rx_missed = s_isr.rx_missed;
    out->rx_overrun = s_isr.rx_overrun;
    out->rx_bad = s_isr.rx_bad;
    out->arb_lost = s_isr.arb_lost;
    out->err_stuff = s_isr.err_stuff;
    out->err_form = s_isr.err_form;
    out->err_bit = s_isr.err_bit;
    out->err_ack = s_isr.err_ack;
    out->err_other = s_isr.err_other;
    out->tx_retries = s_isr.tx_retries;
    out->tx_lost = s_isr.tx_lost;
    out->tx_done = s_isr.tx_done;
    out->rx_storms = s_isr.rx_storms;
}

bool can_drv_storm_masked(void)
{
    return s_storm_masked;
}

elm327_err_t can_drv_transmit(const can_core_frame_t *frame,
                              uint32_t timeout_ms)
{
    can_drv_tx_slot_t *slot = NULL;

    if (can_drv_node == NULL || s_tx_order == NULL)
    {
        return ELM327_ERR_CAN;
    }

    /* held from the slot to the hand-over: see s_tx_order */
    xSemaphoreTake(s_tx_order, portMAX_DELAY);
    portENTER_CRITICAL(&s_tx_lock);

    int idx = can_txs_take(&s_txs);

    portEXIT_CRITICAL(&s_tx_lock);

    if (idx < 0)
    {
        xSemaphoreGive(s_tx_order);
        return ELM327_ERR_BUSY;   /* CAN_DRV_TX_SLOTS frames in flight */
    }

    slot = &s_tx[idx];
    memset(&slot->f, 0, sizeof(slot->f));
    slot->arb_at = s_isr.arb_lost;
    slot->f.header.id  = frame->id;
    slot->f.header.ide = frame->ext ? 1 : 0;
    slot->f.header.rtr = frame->rtr ? 1 : 0;
    slot->f.header.dlc = frame->dlc;

    if (!frame->rtr && frame->dlc > 0)
    {
        memcpy(slot->buf, frame->data, frame->dlc);
        slot->f.buffer     = slot->buf;
        slot->f.buffer_len = frame->dlc;
    }

    esp_err_t ret = twai_node_transmit(can_drv_node, &slot->f,
                                       (int)timeout_ms);

    if (ret != ESP_OK)
    {
        can_txs_release(&s_txs, idx); /* never queued: no tx_done will come */
    }

    xSemaphoreGive(s_tx_order);

    if (ret != ESP_OK)
    {
        return (ret == ESP_ERR_TIMEOUT) ? ELM327_ERR_BUSY : ELM327_ERR_CAN;
    }

    return ELM327_OK;
}

bool can_drv_tx_room(void)
{
    /* plain reads of bytes the interrupt clears and can_drv_transmit sets */
    return can_txs_room(&s_txs);
}

void can_drv_rx_reset(void)
{
    if (can_drv_rx_q == NULL)
    {
        can_drv_rx_q = xQueueCreateStatic(CAN_CORE_RXQ_DEPTH,
                                    sizeof(can_rx_raw_t), s_rxq_store,
                                    &s_rxq_buf);
    }

    if (s_tx_order == NULL)
    {
        s_tx_order = xSemaphoreCreateMutexStatic(&s_tx_order_buf);
    }

    xQueueReset(can_drv_rx_q);
}

void can_drv_tx_recessive(int tx_gpio)
{
    /* level first: no low pulse while the direction changes */
    gpio_set_level((gpio_num_t)tx_gpio, 1);
    gpio_set_direction((gpio_num_t)tx_gpio, GPIO_MODE_OUTPUT);
}

elm327_err_t can_drv_create(can_core_handle_t *handle, uint32_t baud_kbps,
                            bool listen_only)
{
    const can_core_config_t *config = &handle->config;

    if (baud_kbps < 25 || baud_kbps > 1000)
    {
        ESP_LOGE(TAG, "Unsupported baud rate: %lu kbps",
                 (unsigned long)baud_kbps);
        return ELM327_ERR_CAN;
    }

    if (can_drv_rx_q == NULL)
    {
        can_drv_rx_reset();
    }

    /* the raw queue is NOT emptied here: a policy bounce (promotion to
       normal mode) keeps the frames already received for the RX task */
    can_drv_evt_bus_off = false;
    can_drv_evt_recovered = false;
    can_drv_evt_storm = false;
    s_storm_masked = false; /* a new node comes up with its interrupts on */
    s_storm_n = 0;

    /* The bit timing is ours (can_timing_core.h): the driver's own choice
       below 500 kbit/s mis-reads frames on this controller. */
    uint32_t src_hz = 0;
    can_timing_t timing = { 0 };
    bool own_timing =
        esp_clk_tree_src_get_freq_hz((soc_module_clk_t)TWAI_CLK_SRC_DEFAULT,
                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
                                     &src_hz) == ESP_OK &&
        can_timing_for(src_hz, (uint16_t)baud_kbps, &timing);

    /* Listen-only must not be ABLE to drive the bus, and on this chip it
       is: at the wrong bitrate the controller puts dominant bits out in
       listen-only mode too (IDF: TWAI_LL_HAS_LOM_DOM_ISSUE; its REC = 128
       workaround does not cover it). Bench 2026-10-02: 445 error frames/s
       on a 250k bus with this node listening at 500k, the sending node
       driven to bus-off again and again. So in listen-only mode the TX
       signal is not routed to the pad at all (the driver takes tx = -1
       there) and the pad is held recessive as a plain GPIO. */
    can_drv_tx_recessive(config->tx_gpio);

    twai_onchip_node_config_t node_cfg =
    {
        .io_cfg =
        {
            .tx = listen_only ? GPIO_NUM_NC
                              : (gpio_num_t)config->tx_gpio,
            .rx = (gpio_num_t)config->rx_gpio,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        /* a rate the driver can always make; the real timing follows
           below, before the node is enabled */
        .bit_timing = { .bitrate = own_timing
                                       ? 500000u
                                       : can_timing_nominal_bps(
                                             (uint16_t)baud_kbps),
                        .sp_permill = 800 },
        .tx_queue_depth = (uint32_t)(config->tx_queue_depth > 0
                                     ? config->tx_queue_depth : 16),
        /* SINGLE SHOT. Not -1 (retry for ever): a tx nobody ACKs, or on
           a mismatched-baud bus, parks the node error-passive (TEC pinned
           at 128, never bus-off) spewing ~1800 error frames/s UNTIL the
           next driver bounce: the reconfig hammer surfaced it
           2026-07-22. This read 512 ("~130 ms of attempts") until
           2026-10-02: the field is an int8_t, 512 truncated to 0, and on
           this controller every value but -1 is single shot
           (twai_hal_v1.c: .ss = retry_cnt != -1). So a frame that loses
           arbitration or meets an error frame is dropped by the
           controller; since 2026-10-03 the tx_done interrupt re-queues it
           up to CAN_DRV_TX_RETRIES times (can_on_tx_done), the bounded
           software retry the J1939 active mode needed. */
        .fail_retry_cnt = 0,
        .flags = { .enable_listen_only = listen_only },
    };

    can_core_recovery_reset(&handle->recovery);

    esp_err_t ret = twai_new_node_onchip(&node_cfg, &can_drv_node);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %d", ret);
        can_drv_node = NULL;
        return ELM327_ERR_CAN;
    }

    if (own_timing)
    {
        twai_timing_advanced_config_t adv =
        {
            .brp = timing.brp,
            .tseg_1 = timing.tseg1,
            .tseg_2 = timing.tseg2,
            .sjw = timing.sjw,
        };

        ret = twai_node_reconfig_timing(can_drv_node, &adv, NULL);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "bit timing for %lu kbit/s refused: %d",
                     (unsigned long)baud_kbps, ret);
            (void)twai_node_delete(can_drv_node);
            can_drv_node = NULL;
            return ELM327_ERR_CAN;
        }

        ESP_LOGD(TAG, "timing: %lu bit/s = %lu Hz / (%lu x %u), sample "
                      "point 80 %%, sjw %u",
                 (unsigned long)timing.bitrate, (unsigned long)src_hz,
                 (unsigned long)timing.brp, (unsigned)CAN_TIMING_QUANTA,
                 (unsigned)timing.sjw);
    }
    else
    {
        ESP_LOGW(TAG, "%lu kbit/s: no 20-quanta timing from a %lu Hz clock, "
                      "the driver's calculation is used",
                 (unsigned long)baud_kbps, (unsigned long)src_hz);
    }

    twai_event_callbacks_t cbs =
    {
        .on_rx_done = can_on_rx_done,
        .on_tx_done = can_on_tx_done,
        .on_state_change = can_on_state_change,
        .on_error = can_on_error,
    };

    /* a node that comes up finds no frame in flight (a bounce ends the
       attempts of whatever was queued; the counters stay) */
    /* Counted: until 2026-11-18 the frames a bounce ended were in `tx`
       and in neither `tx_done` nor `tx_lost`. No node is enabled here, so
       no interrupt writes the counter meanwhile. */
    s_isr.tx_lost += (uint32_t)can_txs_in_flight(&s_txs);
    can_txs_reset(&s_txs);

    s_isr_listen = listen_only;        /* before the first interrupt */
    handle->node_listen = listen_only;

    ret = twai_node_register_event_callbacks(can_drv_node, &cbs, handle);

    if (ret == ESP_OK)
    {
        ret = twai_node_enable(can_drv_node);
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "twai node enable failed: %d", ret);
        (void)twai_node_delete(can_drv_node);
        can_drv_node = NULL;
        return ELM327_ERR_CAN;
    }

    return ELM327_OK;
}

#endif /* ESP_PLATFORM */
