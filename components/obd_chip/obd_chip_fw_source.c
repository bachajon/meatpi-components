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
 * @file obd_chip_fw_source.c
 * @brief Where a chip firmware image comes from: a vendor file on the
 *        filesystem, read whole into PSRAM, or the image packaged with this
 *        firmware. Both hand it to the engine (obd_fw_run, obd_chip_fw.c);
 *        split out of that file 2026-11-21 (700-line rule).
 */
#include "obd_chip.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "filesystem.h"

#include "obd_chip_private.h"

static const char *TAG = "obd_chip"; /* one TAG per component (§10) */

/* keep in lockstep with EMBED_TXTFILES in CMakeLists.txt and the accepted
 * list in obd_chip_fw_version.c (obd_fw_version_is_current) */
#define OBD_FW_BUILTIN_VERSION "V2.3.24"

esp_err_t obd_chip_firmware_update(const char *fs_path)
{
    /* whole file into PSRAM: vendor images are ~500 KB of text */
    static char *s_fw_buf;
    static size_t s_fw_cap;

    /* the image's version is the file's name (the established contract):
       without one there is nothing to compare the chip against, and a
       chip is written only when the image is newer */
    char ver[16];

    if (fs_path == NULL || obd_fw_version_token(fs_path, ver, sizeof(ver)) == 0)
    {
        ESP_LOGW(TAG, "fw file '%s': no version in its name (V2.3.24.txt), "
                 "not flashing", (fs_path != NULL) ? fs_path : "");
        return ESP_ERR_INVALID_ARG;
    }

    size_t fw_size = 0;
    esp_err_t err = filesystem_size(fs_path, &fw_size);

    if (err != ESP_OK || fw_size == 0)
    {
        ESP_LOGE(TAG, "fw file '%s' unavailable (%s)", fs_path,
                 esp_err_to_name(err));
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_SIZE;
    }

    if (s_fw_buf != NULL && s_fw_cap < fw_size + 1)
    {
        heap_caps_free(s_fw_buf); /* a bigger image than the last one */
        s_fw_buf = NULL;
    }

    if (s_fw_buf == NULL)
    {
        s_fw_buf = heap_caps_malloc(fw_size + 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_fw_cap = fw_size + 1;
    }

    if (s_fw_buf == NULL)
    {
        s_fw_cap = 0;
        return ESP_ERR_NO_MEM;
    }

    size_t got = 0;

    err = filesystem_read(fs_path, s_fw_buf, fw_size + 1, &got);

    if (err != ESP_OK)
    {
        return err;
    }

    return obd_fw_run(s_fw_buf, got, ver, OBD_FW_RULE_NEWER, fs_path);
}

/* The packaged image (EMBED_TXTFILES appends a NUL). Bumping the
 * packaged version = new file in CMakeLists + this constant + the
 * current list in obd_chip_fw_version.c. */
extern const char _binary_V2_3_24_txt_start[];
extern const char _binary_V2_3_24_txt_end[];

const char *obd_chip_builtin_fw_version(void)
{
    return OBD_FW_BUILTIN_VERSION;
}

esp_err_t obd_chip_firmware_update_builtin(obd_fw_rule_t rule)
{
    size_t len = (size_t)(_binary_V2_3_24_txt_end -
                          _binary_V2_3_24_txt_start);

    return obd_fw_run(_binary_V2_3_24_txt_start, len, OBD_FW_BUILTIN_VERSION,
                      rule, "builtin " OBD_FW_BUILTIN_VERSION);
}
