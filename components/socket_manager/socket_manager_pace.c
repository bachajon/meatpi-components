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
 * @file socket_manager_pace.c
 * @brief The pace task: what waits for a TCP client leaves on a tick.
 *
 * A client's stream is spaced (tx_backlog.h, the pace): units that come
 * faster than the gap wait in the client's backlog, and this task sends
 * what has gathered every SM_PACE_TICK_MS. It also asks a client again
 * whose socket did not take everything. It sleeps while nothing waits: a
 * sender that leaves something in a backlog wakes it (sm_pace_kick).
 *
 * Why a task of its own and not the net task: that one sits in select(),
 * for up to 200 ms when nothing happens, and nothing wakes a select() from
 * another task. A socket that takes data is writable all the time, so
 * putting it into the write set would make select() return at once for
 * ever.
 */
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "socket_manager_private.h"

static TaskHandle_t s_task;
static StaticTask_t s_tcb;                /* internal: FreeRTOS object */
static StackType_t  s_stack[3072] EXT_RAM_BSS_ATTR; /* PSRAM: no flash access */
static volatile bool s_running;

static void pace_task(void *arg)
{
    bool waiting = false;

    (void)arg;

    while (s_running)
    {
        /* asleep until a send leaves something waiting, then one push a
           tick for as long as something waits */
        if (waiting)
        {
            vTaskDelay(pdMS_TO_TICKS(SM_PACE_TICK_MS) > 0
                           ? pdMS_TO_TICKS(SM_PACE_TICK_MS) : 1);
            (void)ulTaskNotifyTake(pdTRUE, 0);
        }
        else
        {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
        }

        waiting = s_running && sm_net_pace_push();
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t sm_pace_start(void)
{
    if (s_task != NULL)
    {
        return ESP_OK;
    }

    s_running = true;
    s_task = xTaskCreateStatic(pace_task, "socket_pace",
                               sizeof(s_stack) / sizeof(s_stack[0]), NULL, 10,
                               s_stack, &s_tcb);
    return (s_task != NULL) ? ESP_OK : ESP_FAIL;
}

void sm_pace_stop(void)
{
    s_running = false;
    sm_pace_kick(); /* out of its wait, to see s_running */

    for (int i = 0; i < 25 && s_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void sm_pace_kick(void)
{
    TaskHandle_t t = s_task;

    if (t != NULL)
    {
        xTaskNotifyGive(t);
    }
}
