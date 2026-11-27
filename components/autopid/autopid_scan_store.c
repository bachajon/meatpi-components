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
 * @file autopid_scan_store.c
 * @brief The detection job's hand-over to the vehicle store: what the job
 *        found becomes the car's entry and its rows. Split out of
 *        autopid_std_scan.c 2026-11-27 (700-line rule), unchanged. Scan-task
 *        context (an INTERNAL stack: the store's writes follow).
 */
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

#include "cJSON.h"

#include "autopid_private.h"

static const char *TAG = "autopid";

void ap_scan_store(cJSON *root, const cJSON *supported, ap_veh_seen_t *seen,
                   uint16_t found)
{
    static ap_veh_entry_t entry EXT_RAM_BSS_ATTR; /* scan task only     */
    bool known = false;
    /* whatever the table says, no parameter name twice in what is stored
       (the result document and the car's rows alike) */
    int renamed = ap_names_dedupe((cJSON *)supported);
    /* a new car's rows are stored OFF: nothing is read until the user
       ticks the ones they want (Ali, 2026-10-09) */
    char *tables = ap_scan_rows_config(supported);
    esp_err_t err = ESP_ERR_NO_MEM;

    if (renamed > 0)
    {
        ESP_LOGW(TAG, "detection: %d repeated parameter name%s made unique",
                 renamed, renamed == 1 ? "" : "s");
    }

    seen->std_supported = found;

    if (tables != NULL)
    {
        err = autopid_vehicle_detected(seen, tables, strlen(tables), &entry,
                                       &known);
        free(tables);
    }

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "key", entry.key);
        cJSON_AddBoolToObject(root, "known", known);
        cJSON_AddStringToObject(root, "name", entry.name);

        if (!known && found > 0)
        {
            ESP_LOGI(TAG, "detection: the %u rows of %s are stored off: "
                          "tick the ones to read under Automate > Parameters "
                          "or in the Quick Setup", (unsigned)found,
                     entry.key);
        }

        /* the wizard named the car before the probe (2026-10-26): a new
           entry is stored under that name with the profile attached (its
           init applies live when the car is current); a known car keeps
           what it has until Finish */
        const ap_scan_probe_t *probe = ap_scan_probe();

        if (!known && probe->armed && probe->profile[0] != '\0')
        {
            esp_err_t u = autopid_vehicle_update(
                entry.key, probe->name[0] != '\0' ? probe->name : NULL,
                probe->profile, probe->init[0] != '\0' ? probe->init : NULL);

            if (u == ESP_OK)
            {
                cJSON_ReplaceItemInObject(root, "name", cJSON_CreateString(
                    probe->name[0] != '\0' ? probe->name : entry.name));
            }
            else
            {
                ESP_LOGW(TAG, "detection: profile %s not attached to %s (%s)",
                         probe->profile, entry.key, esp_err_to_name(u));
            }
        }
    }
    else
    {
        /* a car without VIN or responders cannot be stored: the table
           still is (the user can merge it by hand) */
        ESP_LOGW(TAG, "detection: not stored (%s)", esp_err_to_name(err));
        cJSON_AddStringToObject(root, "key", "");
        cJSON_AddBoolToObject(root, "known", false);
        cJSON_AddStringToObject(root, "name", "");
    }
}
