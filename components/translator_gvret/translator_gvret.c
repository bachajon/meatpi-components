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
 * @file translator_gvret.c
 * @brief Registration (target-only edge; the codec is pure/host-tested).
 */
#include "translator_gvret.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bridge_manager.h"

#include "translator_gvret_private.h"

extern const bridge_translator_t translator_gvret_desc;

/* the clock a TIME_SYNC answer carries: the one can_core stamps a received
   frame with (ts_us: esp_timer in the CAN RX interrupt, its low 32 bits), so
   the answer and the frame records are on one time line (2026-11-18) */
static uint32_t gvret_clock_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

esp_err_t translator_gvret_init(void)
{
    esp_err_t err = bridge_manager_register_translator(&translator_gvret_desc);
    if (err == ESP_OK)
    {
        gvret_clock_us_hook = gvret_clock_us;
        ESP_LOGI("translator_gvret", "registered 'gvret' codec");
    }
    return err;
}
