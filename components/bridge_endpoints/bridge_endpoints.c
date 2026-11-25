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
 * @file bridge_endpoints.c
 * @brief Jack registration + the thin wrappers: obd_chip, ble_manager,
 *        cmdline_manager, and the per-slot socket/WS adapters registered
 *        under their CONFIGURED names. The stateful adapters live in
 *        bridge_endpoints_usb.c (usb_obd) and bridge_endpoints_can.c
 *        (can). Add-on packs register further jacks directly with
 *        bridge_manager (ext init phase).
 */
#include "bridge_endpoints.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "ble_manager.h"
#include "bridge_manager.h"
#include "cmdline_manager.h"
#include "log_manager.h"
#include "obd_chip.h"
#include "socket_manager.h"
#include "websocket_manager.h"

#include "bridge_endpoints_private.h"

static const char *TAG = "bridge_endpoints";

_Static_assert(sizeof(bridge_chunk_t) == sizeof(obd_chunk_t),
               "chunk conventions diverged");
_Static_assert(sizeof(bridge_chunk_t) == sizeof(cmdline_chunk_t),
               "chunk conventions diverged");
_Static_assert(sizeof(bridge_chunk_t) == sizeof(socket_chunk_t),
               "chunk conventions diverged");
_Static_assert(sizeof(bridge_chunk_t) == sizeof(ble_chunk_t),
               "chunk conventions diverged");
_Static_assert(sizeof(bridge_chunk_t) == sizeof(websocket_chunk_t),
               "chunk conventions diverged");

/* ---- what a jack lost (bridge_endpoint_t.losses, 2026-11-18) ------------------
 * Two names every jack here shares: `rx_drops`, chunks the jack read and the
 * bridge's queue had no room for, and `tx_drops`, chunks a bridge handed it
 * that it dropped. The `can` and `usb_obd` jacks add their own
 * (bridge_endpoints_can.c, bridge_endpoints_usb.c). */
int bep_losses_rx_tx(bridge_loss_t *out, int cap, uint32_t rx_drops,
                     uint32_t tx_drops)
{
    int n = 0;

    if (n < cap)
    {
        out[n++] = (bridge_loss_t){ "rx_drops", rx_drops };
    }

    if (n < cap)
    {
        out[n++] = (bridge_loss_t){ "tx_drops", tx_drops };
    }

    return n;
}

/* ---- obd_chip ---------------------------------------------------------------- */

/** @p from = the endpoint the bytes come from (the bridge's other end),
 *  NULL when the bridge did not say (a translated one). */
static esp_err_t obd_ep_send_from(const void *from, const uint8_t *d,
                                  size_t l)
{
    /* every byte from a bridged app (TCP/BLE/USB/WS) counts as client
       activity: autopid yields the chip while the app drives it (legacy
       DEV_AUTOPID_ELM327_APP_BIT parity, see obd_chip_client.h), and since
       2026-11-21 a monitor the app opened counts until it ends (GitHub
       issue #1078: an ATMA was cut ten seconds in). Since 2026-11-25 the
       call also waits until the poller's pass on the chip has ended: the
       app's first command landed in it and was lost */
    obd_chip_client_write(from, d, l);
    return obd_chip_send(d, l);
}

static esp_err_t obd_ep_send(const uint8_t *d, size_t l)
{
    return obd_ep_send_from(NULL, d, l);
}

/* the endpoint at the other end of a bridge has no peer left (2026-11-25):
   when it is the one that wrote to the chip last, its monitor's hold is
   over */
static void obd_ep_far_gone(const void *from)
{
    obd_chip_client_gone(from);
}

/* an app's seat on the chip (2026-11-25): what the chip answers to the
   firmware's own requests is not handed to it */
static esp_err_t obd_ep_subscribe(QueueHandle_t q)
{
    return obd_chip_subscribe_app(q, "bridge");
}

static esp_err_t obd_ep_unsubscribe(QueueHandle_t q)
{
    return obd_chip_unsubscribe(q);
}

/* the chip's fan-out keeps a count per subscriber: this bridge's own. What
   the chip itself lost on its UART is on /api/obd_chip. */
static int obd_ep_losses(QueueHandle_t q, bridge_loss_t *out, int cap)
{
    if (cap < 1)
    {
        return 0;
    }

    out[0] = (bridge_loss_t){ "rx_drops", obd_chip_dropped(q) };
    return 1;
}

/* ---- ble_manager --------------------------------------------------------------- */

static esp_err_t ble_ep_send(const uint8_t *d, size_t l)
{
    return ble_manager_send(d, l);
}

static esp_err_t ble_ep_subscribe(QueueHandle_t q)
{
    return ble_manager_subscribe(q);
}

static esp_err_t ble_ep_unsubscribe(QueueHandle_t q)
{
    return ble_manager_unsubscribe(q);
}

static int ble_ep_peers(void)
{
    return ble_manager_is_connected() ? 1 : 0;
}

static int ble_ep_losses(QueueHandle_t q, bridge_loss_t *out, int cap)
{
    uint32_t rx = 0;
    uint32_t tx = 0;

    (void)q;
    ble_manager_pipe_drops(&rx, &tx);
    return bep_losses_rx_tx(out, cap, rx, tx);
}

/* ---- cmdline_manager (the CLI reachable over any bridged transport) ------------ */

static esp_err_t cli_send(const uint8_t *d, size_t l)
{
    return cmdline_manager_send(d, l);
}

static esp_err_t cli_subscribe(QueueHandle_t q)
{
    return cmdline_manager_subscribe(q);
}

static esp_err_t cli_unsubscribe(QueueHandle_t q)
{
    return cmdline_manager_unsubscribe(q);
}

/* the console's two queues: rx = output a command printed that the bridge's
   queue did not take, tx = input a bridge handed over that found the
   console's queue full */
static int cli_losses(QueueHandle_t q, bridge_loss_t *out, int cap)
{
    uint32_t in = 0;
    uint32_t outp = 0;

    (void)q;
    cmdline_manager_drops(&in, &outp);
    return bep_losses_rx_tx(out, cap, outp, in);
}

/* ---- socket/WS slots: jacks under their CONFIGURED names ------------------------ *
 * The endpoint ABI carries no ctx pointer, so each slot gets generated
 * wrappers bound to a name buffer filled at start() from the applied
 * settings. Renaming a server/channel renames its jack (bridges follow
 * the settings name: the old fixed-name-only wart is gone). */

static char s_sock_names[SOCKET_MANAGER_MAX_SERVERS][16];
static char s_ws_names[WEBSOCKET_MANAGER_MAX_CHANNELS][16];

#define SOCK_EP(i)                                                     \
    static esp_err_t sock##i##_send(const uint8_t *d, size_t l)       \
    {                                                                  \
        return socket_manager_send(s_sock_names[i], d, l);            \
    }                                                                  \
    static esp_err_t sock##i##_sub(QueueHandle_t q)                   \
    {                                                                  \
        return socket_manager_subscribe(s_sock_names[i], q);          \
    }                                                                  \
    static esp_err_t sock##i##_unsub(QueueHandle_t q)                 \
    {                                                                  \
        return socket_manager_unsubscribe(s_sock_names[i], q);        \
    }                                                                 \
    static int sock##i##_losses(QueueHandle_t q, bridge_loss_t *out,  \
                                int cap)                              \
    {                                                                  \
        socket_stats_t st = { 0 };                                    \
                                                                      \
        (void)q;                                                      \
        (void)socket_manager_stats(s_sock_names[i], &st);             \
        return bep_losses_rx_tx(out, cap, st.rx_drops, st.tx_drops);  \
    }                                                                 \
    static int sock##i##_peers(void)                                  \
    {                                                                  \
        socket_stats_t st = { 0 };                                    \
                                                                      \
        (void)socket_manager_stats(s_sock_names[i], &st);             \
        return st.clients;                                            \
    }

SOCK_EP(0)
SOCK_EP(1)
SOCK_EP(2)
SOCK_EP(3)
#undef SOCK_EP

#define WS_EP(i)                                                       \
    static esp_err_t ws##i##_send(const uint8_t *d, size_t l)         \
    {                                                                  \
        return websocket_manager_send(s_ws_names[i], d, l);           \
    }                                                                  \
    static esp_err_t ws##i##_sub(QueueHandle_t q)                     \
    {                                                                  \
        return websocket_manager_subscribe(s_ws_names[i], q);         \
    }                                                                  \
    static esp_err_t ws##i##_unsub(QueueHandle_t q)                   \
    {                                                                  \
        return websocket_manager_unsubscribe(s_ws_names[i], q);       \
    }                                                                 \
    static int ws##i##_losses(QueueHandle_t q, bridge_loss_t *out,    \
                              int cap)                                \
    {                                                                  \
        websocket_stats_t st = { 0 };                                 \
                                                                      \
        (void)q;                                                      \
        (void)websocket_manager_stats(s_ws_names[i], &st);            \
        return bep_losses_rx_tx(out, cap, st.rx_drops, st.tx_drops);  \
    }                                                                 \
    static int ws##i##_peers(void)                                    \
    {                                                                  \
        websocket_stats_t st = { 0 };                                 \
                                                                      \
        (void)websocket_manager_stats(s_ws_names[i], &st);            \
        return st.clients;                                            \
    }

WS_EP(0)
WS_EP(1)
WS_EP(2)
WS_EP(3)
WS_EP(4) /* MAX_CHANNELS grew 4 -> 6 with websocket_manager v2 (ws_log) */
WS_EP(5)
#undef WS_EP

static const bridge_endpoint_t SOCK_EPS[SOCKET_MANAGER_MAX_SERVERS] =
{
    { s_sock_names[0], sock0_send, sock0_sub, sock0_unsub,
      .losses = sock0_losses, .peers = sock0_peers },
    { s_sock_names[1], sock1_send, sock1_sub, sock1_unsub,
      .losses = sock1_losses, .peers = sock1_peers },
    { s_sock_names[2], sock2_send, sock2_sub, sock2_unsub,
      .losses = sock2_losses, .peers = sock2_peers },
    { s_sock_names[3], sock3_send, sock3_sub, sock3_unsub,
      .losses = sock3_losses, .peers = sock3_peers },
};

static const bridge_endpoint_t WS_EPS[WEBSOCKET_MANAGER_MAX_CHANNELS] =
{
    { s_ws_names[0], ws0_send, ws0_sub, ws0_unsub, .losses = ws0_losses,
      .peers = ws0_peers },
    { s_ws_names[1], ws1_send, ws1_sub, ws1_unsub, .losses = ws1_losses,
      .peers = ws1_peers },
    { s_ws_names[2], ws2_send, ws2_sub, ws2_unsub, .losses = ws2_losses,
      .peers = ws2_peers },
    { s_ws_names[3], ws3_send, ws3_sub, ws3_unsub, .losses = ws3_losses,
      .peers = ws3_peers },
    { s_ws_names[4], ws4_send, ws4_sub, ws4_unsub, .losses = ws4_losses,
      .peers = ws4_peers },
    { s_ws_names[5], ws5_send, ws5_sub, ws5_unsub, .losses = ws5_losses,
      .peers = ws5_peers },
};

/* ---- lifecycle ------------------------------------------------------------------ */

esp_err_t bridge_endpoints_init(void)
{
    bep_can_bind_restart();
    bep_can_tx_init();

    static const log_descriptor_t LOG_DESC =
        { "bridge_endpoints", ESP_LOG_INFO };
    static const bridge_endpoint_t FIXED[] =
    {
        /* multi_consumer: obd_chip fans RX out to every subscriber queue
           (each gets a full copy) and serializes TX, the single-consumer
           rule doesn't apply, so TCP + USB passthrough can both be live */
        { "obd", obd_ep_send, obd_ep_subscribe, obd_ep_unsubscribe,
          .multi_consumer = true, .losses = obd_ep_losses,
          .send_from = obd_ep_send_from, .far_gone = obd_ep_far_gone },
        { "ble", ble_ep_send, ble_ep_subscribe, ble_ep_unsubscribe,
          .losses = ble_ep_losses, .peers = ble_ep_peers },
        /* multi_consumer since 2026-07-26: the can pump fans RX chunks
           out to every subscribed bridge (slcan + mqtt_can together);
           TX serializes through can_manager_send. ONE jack-wide ingress
           filter (bep_can_set_filter). */
        { "can", bep_can_send, bep_can_subscribe, bep_can_unsubscribe,
          .multi_consumer = true, .tx_ready = bep_can_tx_ready,
          .losses = bep_can_losses },
        { "cli", cli_send, cli_subscribe, cli_unsubscribe,
          .losses = cli_losses },
        { "usb_obd", bep_usb_send, bep_usb_subscribe, bep_usb_unsubscribe,
          .losses = bep_usb_losses },
    };

    log_manager_register(&LOG_DESC);

    esp_err_t first_err = ESP_OK;

    for (size_t i = 0; i < sizeof(FIXED) / sizeof(FIXED[0]); i++)
    {
        esp_err_t err = bridge_manager_register_endpoint(&FIXED[i]);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "jack '%s' registration failed (%s)",
                     FIXED[i].name, esp_err_to_name(err));

            if (first_err == ESP_OK)
            {
                first_err = err;
            }
        }
    }

    return first_err;
}

esp_err_t bridge_endpoints_start(void)
{
    /* dynamic jacks: one per CONFIGURED server/channel, under its
       configured name (settings applied by now; registration must
       precede bridge_manager_start). A name colliding with an existing
       jack is refused by the registry: log and degrade alone. */
    for (int i = 0; i < SOCKET_MANAGER_MAX_SERVERS; i++)
    {
        const char *name = socket_manager_server_name(i);

        if (name == NULL)
        {
            break;
        }

        snprintf(s_sock_names[i], sizeof(s_sock_names[i]), "%s", name);

        if (bridge_manager_register_endpoint(&SOCK_EPS[i]) != ESP_OK)
        {
            ESP_LOGW(TAG, "socket jack '%s' not registered "
                     "(duplicate name?)", name);
        }
    }

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CHANNELS; i++)
    {
        const char *name = websocket_manager_channel_name(i);

        if (name == NULL)
        {
            break;
        }

        snprintf(s_ws_names[i], sizeof(s_ws_names[i]), "%s", name);

        if (bridge_manager_register_endpoint(&WS_EPS[i]) != ESP_OK)
        {
            ESP_LOGW(TAG, "ws jack '%s' not registered "
                     "(duplicate name?)", name);
        }
    }

    return bep_usb_start();
}

esp_err_t bridge_endpoints_stop(void)
{
    /* jacks live as long as the registry; the adapters' own tasks park
       when unsubscribed (init-once, §3: no teardown path) */
    return ESP_OK;
}
