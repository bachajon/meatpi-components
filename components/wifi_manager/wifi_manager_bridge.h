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
 * @file wifi_manager_bridge.h
 * @brief Component-private bridge between wifi_manager.c (owns the state),
 *        the radio files split from it and wifi_manager_status.c (public
 *        status/scan surface): everything that needs an IDF or FreeRTOS
 *        type. Kept out of wifi_manager_private.h so the pure selection
 *        module stays IDF-free.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "wifi_manager.h"
#include "wifi_manager_private.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wm_status_copy_ip(char *buf, size_t buf_len);

/** which: 0 = enabled, 1 = STA connected, 2 = AP started. */
bool wm_status_flag(int which);

uint16_t wm_status_ap_clients(void);
esp_netif_t *wm_sta_netif(void);
esp_netif_t *wm_ap_netif(void);
EventGroupHandle_t wm_event_group(void);
void wm_set_callbacks(const wifi_manager_callbacks_t *cbs);

/**
 * Blocking scan under the component lock. On ESP_OK the records stay valid
 * until wm_scan_unlock(), which the caller MUST invoke (it also restores an
 * AP-only radio mode switched for the scan).
 */
esp_err_t wm_scan_locked(wifi_ap_record_t **records, uint16_t *count);
void wm_scan_unlock(void);

/* ---- between the radio files: the IDF-typed half (the rest, and which
 * file holds what, is in wifi_manager_private.h) ------------------------- */

/* private event-group bits (public ones are BIT0..BIT4 in wifi_manager.h) */
#define WM_BIT_STOP_REQ    BIT6
#define WM_BIT_TASK_EXITED BIT7

#define WM_RECONNECT_PERIOD_MS   5000
#define WM_SCAN_MAX_RECORDS      32

/** The component lock (wifi_manager.c): one scan at a time, with its
 *  scratch, and the copies of the status's strings. */
extern SemaphoreHandle_t wm_lock;

/** The reconnect task's handle (wifi_manager.c creates the task and asks it
 *  to stop; the task clears the handle on its way out). */
extern TaskHandle_t wm_reconnect_task_handle;

/** The handler of every WIFI_EVENT and of IP_EVENT_STA_GOT_IP
 *  (wifi_manager_handler.c); wifi_manager_start() registers it. */
void wm_event_handler(void *arg, esp_event_base_t base,
                      int32_t event_id, void *data);

/** Scan scratch, serialized by wm_lock (wifi_manager_sta.c): the records of
 *  the last scan and the names copied out of them for the selection. */
extern wifi_ap_record_t wm_scan_records[WM_SCAN_MAX_RECORDS];
extern char wm_scan_present[WM_SCAN_MAX_RECORDS][WM_SSID_LEN];

/** Blocking scan into wm_scan_records (caller holds wm_lock). Returns count. */
uint16_t wm_do_scan(void);

#ifdef __cplusplus
}
#endif
