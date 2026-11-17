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
 * @file bridge_manager_batch.c
 * @brief A translated bridge's output gathered into one endpoint send().
 *        Pure (no FreeRTOS, no endpoint knowledge): host-tested.
 *
 * The codec's sink calls fill the batch. What does not fit sends the batch
 * first, here in the sink, so the bytes of one sink call stay together (an
 * slcan line is never split) and no codec call ever has to be run a second
 * time. Running one again would hand the destination what the first run had
 * already put into the batch (an slcan line transmitted twice on the bus),
 * and a stream codec has moved its state on by then.
 */
#include <string.h>

#include "bridge_manager.h"
#include "bridge_manager_private.h"

void bm_batch_flush(bm_batch_t *b)
{
    if (b->len == 0)
    {
        return;
    }

    if (b->send(b->buf, b->len) != ESP_OK)
    {
        (*b->send_errors)++;
    }

    b->len = 0;
}

esp_err_t bm_batch_sink(void *arg, const uint8_t *out, size_t out_len)
{
    bm_batch_t *b = arg;

    if (out_len > b->cap - b->len)
    {
        bm_batch_flush(b);
    }

    if (out_len > b->cap)
    {
        /* more than a whole batch: straight through, behind what waited */
        if (b->send(out, out_len) != ESP_OK)
        {
            (*b->send_errors)++;
        }

        return ESP_OK;
    }

    memcpy(b->buf + b->len, out, out_len);
    b->len += out_len;
    return ESP_OK;
}
