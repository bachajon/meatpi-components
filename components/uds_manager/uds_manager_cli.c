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
 * @file uds_manager_cli.c
 * @brief The `uds` console command: the terminal's serial/BLE/WS face.
 *        `uds -t 7E0 -r 7E8 22 F1 90`  (hex request after the flags)
 *
 * The answer is printed whole on one line, whatever its length, up to the
 * route's 4096 bytes: `<- 62 F1 90 ..  [isotp, 5 ms]`. Until 2026-11-26
 * the line went out through one cmdline_printf and ended after the 84th
 * byte of the answer (that function's 256-byte buffer), the command itself
 * showed 128 bytes at most, and an answer over 512 bytes was an error.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_attr.h"

#include "cmdline_manager.h"

#include "uds_manager.h"
#include "uds_manager_private.h"
#include "uds_proto.h"

static struct
{
    struct arg_str *tx;
    struct arg_str *rx;
    struct arg_lit *ext;
    struct arg_str *data;
    struct arg_end *end;
} s_args;

static int cmd_uds(int argc, char **argv)
{
    int nerr = arg_parse(argc, argv, (void **)&s_args);

    if (nerr != 0)
    {
        arg_print_errors(stdout, s_args.end, "uds");
        cmdline_printf("usage: uds -t <txid> -r <rxid> [-e] <hex request>\n");
        return 1;
    }

    if (s_args.tx->count == 0 || s_args.rx->count == 0 ||
        s_args.data->count == 0)
    {
        cmdline_printf("usage: uds -t 7E0 -r 7E8 [-e] 22 F1 90\n");
        return 1;
    }

    uds_addr_t addr =
    {
        .tx_id  = (uint32_t)strtoul(s_args.tx->sval[0], NULL, 16),
        .rx_id  = (uint32_t)strtoul(s_args.rx->sval[0], NULL, 16),
        .ext_id = s_args.ext->count > 0,
    };

    /* join the data args back into one hex string: as long as the longest
       line the console takes, so the join cannot lose a byte (192 did: 70
       bytes typed as one `22:F1:..` word went out as their first 64) */
    char hexin[CMDLINE_MANAGER_LINE_MAX];
    size_t o = 0;
    bool joined = true;

    for (int i = 0; i < s_args.data->count && joined; i++)
    {
        int n = snprintf(hexin + o, sizeof(hexin) - o, "%s",
                         s_args.data->sval[i]);

        joined = (n >= 0 && (size_t)n < sizeof(hexin) - o);
        o += joined ? (size_t)n : 0;
    }

    uint8_t reqb[UDS_REQ_MAX];
    size_t reqn = 0;

    if (!joined || !uds_hex_to_bytes(hexin, reqb, sizeof(reqb), &reqn) ||
        reqn == 0)
    {
        cmdline_printf("bad hex request (hex bytes, %d at most)\n",
                       UDS_REQ_MAX);
        return 1;
    }

    /* the answer: PSRAM, one command at a time (the dispatcher's lock) */
    EXT_RAM_BSS_ATTR static uint8_t respb[UDS_RESP_CAP];
    size_t respn = 0;
    uds_result_t res;

    esp_err_t err = uds_request(&addr, reqb, reqn, respb, sizeof(respb),
                                &respn, NULL, &res);

    if (err != ESP_OK)
    {
        cmdline_printf("uds error: %s (backend %s)\n",
                       esp_err_to_name(err), res.backend ? res.backend : "?");
        return 1;
    }

    /* the whole answer on one line, a console chunk at a time through
       cmdline_write: no buffer has to hold the line */
    char piece[CMDLINE_MANAGER_CHUNK_SIZE];
    size_t off = 0;
    size_t n;

    cmdline_printf("<- ");

    while ((n = uds_hex_piece(respb, respn, &off, piece, sizeof(piece))) > 0)
    {
        cmdline_write(piece, n);
    }

    cmdline_printf("  [%s, %lu ms", res.backend,
                   (unsigned long)res.elapsed_ms);
    if (res.pending_count)
    {
        cmdline_printf(", %u pending", res.pending_count);
    }
    cmdline_printf("]\n");

    if (res.negative)
    {
        cmdline_printf("NEGATIVE: NRC 0x%02X %s\n", res.nrc, res.nrc_name);
    }
    else
    {
        cmdline_printf("positive (SID 0x%02X)\nOK\n", res.sid);
    }

    return 0;
}

esp_err_t uds_manager_register_cli(void)
{
    s_args.tx = arg_str0("t", "tx", "<hhh>", "request CAN ID (hex)");
    s_args.rx = arg_str0("r", "rx", "<hhh>", "response CAN ID (hex)");
    s_args.ext = arg_lit0("e", "ext", "29-bit extended IDs");
    /* every byte of the longest request may be a word of its own */
    s_args.data = arg_strn(NULL, NULL, "<hex>", 0, UDS_REQ_MAX,
                           "request bytes");
    s_args.end = arg_end(4);

    static const esp_console_cmd_t CMD =
    {
        .command = "uds",
        .help = "UDS request: uds -t 7E0 -r 7E8 [-e] 22 F1 90",
        .func = cmd_uds,
        .argtable = &s_args,
    };

    return cmdline_manager_register(&CMD);
}
