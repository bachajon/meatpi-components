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
 * @file can_tx_slots.h
 * @brief Which transmit slot is taken, and which one the controller holds
 *        (pure, host-tested: no FreeRTOS, no TWAI). The frames themselves
 *        stay in can_core_driver.c.
 *
 * A slot is taken from the transmit call to the interrupt that ends the
 * frame's attempts (tx_done). One frame gets no such interrupt: the one
 * inside the controller when the controller goes bus-off. The IDF node
 * driver forgets it (esp_twai_onchip.c: the frames still in its queue go
 * out after the recovery, the one it had handed to the hardware is
 * overwritten or cleared without a callback), so until 2026-11-18 its slot
 * stayed taken: one slot gone with every bus-off, and after 16 of them
 * nothing was transmitted until the node was started again.
 *
 * The frame the controller holds is the taken slot that was handed over
 * first: the driver sends in the order it was given the frames, and a
 * frame sent again after a lost attempt goes ahead of the rest (it keeps
 * its place here). The caller hands the frames over in the order it took
 * the slots (one task at a time between the two).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TXS_SLOTS 16 /* frames in flight */

typedef struct
{
    volatile uint8_t tries[CAN_TXS_SLOTS]; /* 0 = free; attempts made otherwise */
    uint32_t         order[CAN_TXS_SLOTS]; /* when the slot was taken           */
    uint32_t         next;                 /* the next slot's order             */
} can_txs_t;

/** Every slot free (a node that comes up finds no frame in flight). */
void can_txs_reset(can_txs_t *t);

/** Take a free slot for a first attempt: its index, or -1 when all
 *  CAN_TXS_SLOTS frames are in flight. Task context, under the caller's
 *  lock. */
int can_txs_take(can_txs_t *t);

/** The slot's frame is done with (sent, given up, or never handed over). */
void can_txs_release(can_txs_t *t, int slot);

/** True while a slot is free. Plain reads: a peek, as before. */
bool can_txs_room(const can_txs_t *t);

/** The taken slot handed over first, the frame the controller holds or
 *  will get next: its index, or -1 when no frame is in flight. */
int can_txs_in_controller(const can_txs_t *t);

/** The controller went bus-off: the frame it held gets no tx_done. Its
 *  slot is released; the index for the caller's count, or -1 when no frame
 *  was in flight. The other taken slots are still in the driver's queue
 *  and stay taken. Interrupt context. */
int can_txs_bus_off(can_txs_t *t);

/** How many frames are in flight (taken slots). A node that is stopped
 *  with frames in flight ends them without a tx_done: the caller counts
 *  them lost before can_txs_reset(). */
int can_txs_in_flight(const can_txs_t *t);

#ifdef __cplusplus
}
#endif
