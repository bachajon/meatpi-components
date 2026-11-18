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
 * @file can_tx_slots.c
 * @brief The transmit slot table of the CAN driver (see can_tx_slots.h).
 *        Pure: the host suite compiles this file alone.
 *
 * On the device the TWAI interrupt calls into this file while a flash write
 * has the cache off, so every function it reaches is in IRAM and the table
 * it is given lives in internal RAM (can_core_driver.c).
 */
#include "can_tx_slots.h"

#include <string.h>

#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#endif

#if defined(ESP_PLATFORM) && !defined(CONFIG_IDF_TARGET_LINUX)
#include "esp_attr.h"
#define CAN_TXS_ISR IRAM_ATTR
#else
#define CAN_TXS_ISR
#endif

void can_txs_reset(can_txs_t *t)
{
    memset(t, 0, sizeof(*t));
}

int can_txs_take(can_txs_t *t)
{
    for (int i = 0; i < CAN_TXS_SLOTS; i++)
    {
        if (t->tries[i] == 0)
        {
            /* the order first: the interrupt never sees a taken slot with
               the order of the frame that had it before */
            t->order[i] = t->next++;
            t->tries[i] = 1;
            return i;
        }
    }

    return -1;
}

void CAN_TXS_ISR can_txs_release(can_txs_t *t, int slot)
{
    if (slot >= 0 && slot < CAN_TXS_SLOTS)
    {
        t->tries[slot] = 0;
    }
}

bool can_txs_room(const can_txs_t *t)
{
    for (int i = 0; i < CAN_TXS_SLOTS; i++)
    {
        if (t->tries[i] == 0)
        {
            return true;
        }
    }

    return false;
}

int CAN_TXS_ISR can_txs_in_controller(const can_txs_t *t)
{
    int first = -1;

    for (int i = 0; i < CAN_TXS_SLOTS; i++)
    {
        if (t->tries[i] == 0)
        {
            continue;
        }

        /* the counter may wrap: the taken slots are at most CAN_TXS_SLOTS
           apart, so the signed difference tells which came first */
        if (first < 0 || (int32_t)(t->order[i] - t->order[first]) < 0)
        {
            first = i;
        }
    }

    return first;
}

int CAN_TXS_ISR can_txs_bus_off(can_txs_t *t)
{
    int slot = can_txs_in_controller(t);

    can_txs_release(t, slot);
    return slot;
}

int can_txs_in_flight(const can_txs_t *t)
{
    int n = 0;

    for (int i = 0; i < CAN_TXS_SLOTS; i++)
    {
        n += (t->tries[i] != 0) ? 1 : 0;
    }

    return n;
}
