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
 * @file wifi_manager_reconnect.c
 * @brief The reconnect task: the 5 s lap that re-joins a dropped station
 *        (backoff, the retry limit, the pause for clients on the AP) and
 *        roams from a fallback to a preferred network, with its
 *        bookkeeping and wifi_manager_get_sta_attempt().
 */
#include "wifi_manager.h"

#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

#define WM_AP_CLIENT_PAUSE_MS    10000
#define WM_RETRY_COOLDOWN_LOOPS  12 /* x 5 s = 1 minute */

volatile wm_roam_state_t wm_sta_roam = WM_ROAM_IDLE;
volatile int             wm_sta_roam_target = -1;

/* roam-to-preferred bookkeeping (reconnect task + got-ip handler) */
volatile int wm_sta_connected_idx = -1; /* config index we're on      */
volatile bool wm_sta_ever_connected;  /* an association this boot   */
static volatile uint32_t s_sta_tries;       /* reconnect tries this boot  */
uint32_t     wm_sta_last_roam_ms;

/* ---- reconnect task ----------------------------------------------------------- */

void wm_reconnect_task(void *arg)
{
    const wm_config_t *cfg = wm_settings_config();
    int cooldown_loops = 0;
    int backoff_loops = 0;
    int max_skip = wm_backoff_max_skip(cfg->sta_search_interval_s);
    uint32_t ap_pauses = 0;   /* consecutive AP-client deferrals        */

    (void)arg;
    ESP_LOGD(TAG, "reconnect task up");

    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(
            wm_event_group(), WM_BIT_STOP_REQ, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(WM_RECONNECT_PERIOD_MS));

        if ((bits & WM_BIT_STOP_REQ) != 0)
        {
            break;
        }

        if (!wm_status.started || wm_suspend_sta_active() ||
            wm_try_active())
        {
            continue; /* suspended, or a connection trial has the STA */
        }

        if (wm_status.sta_connected)
        {
            backoff_loops = 0; /* fresh curve for the next outage */

            /* roam-to-preferred (meatpi: users prefer HOME over the car
             * hotspot): while parked on a FALLBACK, periodically look
             * for a higher-priority network and migrate to it */
            int cur = wm_sta_connected_idx;

            if (cfg->sta_roam_interval_s > 0 && cur > 0 &&
                wm_status.ap_station_count == 0 &&
                (uint32_t)(wm_now_ms() - wm_sta_last_roam_ms) >=
                    cfg->sta_roam_interval_s * 1000u)
            {
                wm_sta_last_roam_ms = wm_now_ms();

                xSemaphoreTake(wm_lock, portMAX_DELAY);

                uint16_t found = wm_do_scan();
                int      better = -1;

                if (found > 0)
                {
                    for (uint16_t i = 0; i < found; i++)
                    {
                        strncpy(wm_scan_present[i],
                                (const char *)wm_scan_records[i].ssid,
                                WM_SSID_LEN - 1);
                        wm_scan_present[i][WM_SSID_LEN - 1] = '\0';
                    }

                    better = wm_select_better(&wm_select_state, cfg->sta,
                                              cfg->sta_count, cur,
                                              wm_scan_present, found, wm_now_ms());
                }

                xSemaphoreGive(wm_lock);

                if (better >= 0)
                {
                    ESP_LOGI(TAG, "preferred network '%s' visible; "
                                  "leaving '%s' to roam to it",
                             cfg->sta[better].ssid, cfg->sta[cur].ssid);
                    wm_sta_roam_target = better;
                    wm_sta_roam = WM_ROAM_LEAVING;
                    esp_wifi_disconnect();
                    /* the disconnect event + this loop reconnect via the
                     * normal selection path, which prefers `better` */
                }
            }

            continue;
        }

        if (cooldown_loops > 0)
        {
            cooldown_loops--;
            continue;
        }

        /* a connection trial just ended: the station was up before it and
           the user asked for the radio activity, so re-join at once, with
           no backoff and no deferral for the phone on our AP (2026-10-06) */
        bool after_trial = wm_try_take_restore_cue();

        if (after_trial)
        {
            backoff_loops = 0;
            ap_pauses = WM_AP_CLIENT_MAX_PAUSES;
        }

        if (backoff_loops > 0)
        {
            backoff_loops--;
            continue;
        }

        /* don't yank the radio's channel while someone is on our AP,
         * except for the first association of the boot, and never for
         * longer than WM_AP_CLIENT_MAX_PAUSES (wm_sta_pause_for_ap_clients) */
        if (wm_sta_pause_for_ap_clients(wm_status.ap_station_count,
                                        wm_sta_ever_connected, ap_pauses,
                                        WM_AP_CLIENT_MAX_PAUSES))
        {
            ap_pauses++;
            ESP_LOGD(TAG, "AP has %u clients; pausing STA reconnect (%lu)",
                     wm_status.ap_station_count, (unsigned long)ap_pauses);
            xEventGroupWaitBits(wm_event_group(), WM_BIT_STOP_REQ, pdFALSE, pdFALSE,
                                pdMS_TO_TICKS(WM_AP_CLIENT_PAUSE_MS -
                                              WM_RECONNECT_PERIOD_MS));
            continue;
        }
        if (wm_status.ap_station_count > 0)
        {
            ESP_LOGI(TAG, "AP has %u clients but %s: connecting anyway "
                     "(they may see one channel hop)",
                     wm_status.ap_station_count,
                     wm_sta_ever_connected ? "the pause budget is spent"
                                          : "no STA session yet this boot");
        }
        ap_pauses = 0;

        if (cfg->sta_max_retry >= 0 &&
            wm_status.sta_retry_count >= cfg->sta_max_retry)
        {
            ESP_LOGW(TAG, "retry limit (%ld) hit; cooling down 1 min",
                     (long)cfg->sta_max_retry);
            wm_status.sta_retry_count = 0;
            cooldown_loops = WM_RETRY_COOLDOWN_LOOPS;
            continue;
        }

        if (wm_sta_select_and_connect())
        {
            /* count only REAL attempts: deferred cycles (banned/idle)
             * must not burn through a finite sta_max_retry */
            wm_status.sta_retry_count++;
            s_sta_tries++;
            backoff_loops =
                wm_backoff_skip_loops(wm_status.sta_retry_count, max_skip);
            ESP_LOGD(TAG, "reconnect attempt %ld (next in %d s)",
                     (long)wm_status.sta_retry_count,
                     (backoff_loops + 1) *
                         (WM_RECONNECT_PERIOD_MS / 1000));
        }
    }

    ESP_LOGD(TAG, "reconnect task exiting");
    xEventGroupSetBits(wm_event_group(), WM_BIT_TASK_EXITED);
    wm_reconnect_task_handle = NULL;
    vTaskDelete(NULL);
}

void wifi_manager_get_sta_attempt(wifi_manager_sta_attempt_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    xSemaphoreTake(wm_lock, portMAX_DELAY);
    strncpy(out->ssid, wm_status.last_attempted_ssid, sizeof(out->ssid) - 1);
    xSemaphoreGive(wm_lock);
    out->last_reason = wm_status.last_reason;
    out->tries = s_sta_tries;
    out->fail_count = wm_select_fail_count(&wm_select_state,
                                           wm_status.last_attempted_idx,
                                           wm_now_ms());
    out->deprioritised = wm_select_is_deprioritised(
        &wm_select_state, wm_status.last_attempted_idx, wm_now_ms());
}
