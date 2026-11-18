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
 * @file j2534_server_transport.c
 * @brief The transport layer of the J2534 server: one framed wire
 *        protocol over any reliable in-order byte link. A transport is a
 *        j2534_transport_t (read/write + ctx): the TCP listener here,
 *        usb_cdc_device (CDC-ACM, through the legacy serial vtable
 *        adapter) and ble_j2534 (a ble_manager stream channel). This file
 *        owns the session lock (ONE tester across all transports), the
 *        send lock (RX pump vs request handler on the wire), the session's
 *        wire version (the status numbers of its ACKs), the frame
 *        read/write plumbing and the TCP listener; the protocol core
 *        (dispatch, channels) lives in j2534_server.c, the periodic
 *        messages in j2534_server_periodic.c.
 */
#include "j2534_server.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <errno.h>

#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "j2534_proto.h"
#include "j2534_server_private.h"

static const char *TAG = "j2534_server";

#define J2534_LISTEN_BACKLOG  1
#define J2534_TASK_STACK      4096   /* PSRAM: no flash from this task */
#define J2534_TASK_PRIO       5
#define J2534_READ_SLICE_MS   200    /* one read() poll                */
#define J2534_REFUSE_SLICES   15     /* 3 s for a refused tester's hello*/

/* A TCP tester that vanished without closing (a pulled cable, a laptop
 * asleep) kept the one session until the unit restarted: the send blocked
 * for good and nothing ever asked whether the peer was still there. Now
 * the stack asks: keepalive probes after 10 s of silence, 5 s apart, and
 * the connection is given up after 3 unanswered ones (25 s), and a send
 * the peer takes nothing of for 5 s ends the session. A tester that is
 * alive answers the probes in its TCP stack without knowing, so a long
 * silence on a healthy link costs nothing (stage j2534session, legs K and
 * V). A second connection while a session is alive is refused as before. */
#define J2534_TCP_KEEPIDLE_S   10
#define J2534_TCP_KEEPINTVL_S  5
#define J2534_TCP_KEEPCNT      3
#define J2534_TCP_SEND_WAIT_S  5

/* the active tester's transport (NULL = idle) + a lock so the RX pump
 * and the request handler don't interleave on the wire. s_active changes
 * under the send lock and every send checks it there: once a session has
 * ended nothing is written to its transport any more (the pump holds a
 * transport pointer of its own for a moment, and the TCP listener's
 * lives on its stack, where the next session's is). */
static const j2534_transport_t *volatile s_active;
/* counts the sessions: the RX pump's batch names the session it was
 * filled in, so one that outlived its session is not written into the
 * next (whose transport can sit at the same address) */
static uint32_t s_session_no;
static SemaphoreHandle_t s_send_lock;
static StaticSemaphore_t s_send_lock_buf;   /* internal: FreeRTOS object */
/* the wire version the active session asked for in its HELLO (1 until
 * then): the status numbers of its ACKs, and whether it is sent EVENTs */
static volatile uint32_t s_wire_ver = 1;
/* held by whichever transport currently owns the single tester session */
static SemaphoreHandle_t s_session_lock;
static StaticSemaphore_t s_session_lock_buf;

/* one whole frame, header + payload, so every wire frame is ONE write
 * (one BLE notification burst / one USB bulk transfer / one TCP send) */
EXT_RAM_BSS_ATTR static uint8_t s_tx_frame[J2534_MAX_FRAME];

static TaskHandle_t s_listener;
static StaticTask_t s_listener_tcb;
EXT_RAM_BSS_ATTR static StackType_t s_listener_stack[J2534_TASK_STACK];
static volatile bool s_listening;

/* ---- wire writes -------------------------------------------------------- */

static bool write_all(const j2534_transport_t *t, const uint8_t *buf, size_t n)
{
    return t->write(t->ctx, buf, n) == (int)n;
}

bool j2534_srv_send_frame(const j2534_transport_t *t, uint8_t type,
                          uint16_t seq, uint16_t channel,
                          const uint8_t *payload, uint32_t len)
{
    if (t == NULL || J2534_HDR_SIZE + len > sizeof(s_tx_frame))
    {
        return false;
    }

    xSemaphoreTake(s_send_lock, portMAX_DELAY);

    bool ok = (t == s_active);

    if (ok)
    {
        j2534_hdr_encode(s_tx_frame, type, seq, channel, len);

        if (len > 0)
        {
            memcpy(s_tx_frame + J2534_HDR_SIZE, payload, len);
        }

        ok = write_all(t, s_tx_frame, J2534_HDR_SIZE + len);
    }

    if (ok)
    {
        j2534_srv_count_tx();
    }

    xSemaphoreGive(s_send_lock);
    return ok;
}

bool j2534_srv_send_raw(const j2534_transport_t *t, uint32_t session,
                        const uint8_t *buf, size_t len)
{
    if (t == NULL || buf == NULL || len < J2534_HDR_SIZE)
    {
        return false;
    }

    xSemaphoreTake(s_send_lock, portMAX_DELAY);

    bool ok = (t == s_active) && (session == s_session_no) &&
              write_all(t, buf, len);

    if (ok)
    {
        /* count the frames in the run (every one starts with a header) */
        for (size_t off = 0; off + J2534_HDR_SIZE <= len;)
        {
            j2534_hdr_t h;

            if (!j2534_hdr_decode(buf + off, J2534_HDR_SIZE, &h))
            {
                break;
            }

            j2534_srv_count_tx();
            off += J2534_HDR_SIZE + h.length;
        }
    }

    xSemaphoreGive(s_send_lock);
    return ok;
}

/* One ACK, its status in the numbers of wire version @p ver. @p refusal:
   the answer to a tester that gets no session, on a transport that is not
   the active one. */
static bool send_ack_at(const j2534_transport_t *t, uint16_t seq,
                        uint16_t channel, uint32_t status,
                        const uint32_t *result, uint32_t ver, bool refusal)
{
    if (t == NULL)
    {
        return false;
    }

    xSemaphoreTake(s_send_lock, portMAX_DELAY);

    size_t n = (refusal || t == s_active)
                   ? j2534_ack_encode(s_tx_frame, sizeof(s_tx_frame), seq,
                                      channel, j2534_status_wire(status, ver),
                                      result)
                   : 0;
    bool ok = (n > 0) && write_all(t, s_tx_frame, n);

    if (ok)
    {
        j2534_srv_count_tx();
    }

    xSemaphoreGive(s_send_lock);
    return ok;
}

bool j2534_srv_send_ack(const j2534_transport_t *t, uint16_t seq,
                        uint16_t channel, uint32_t status,
                        const uint32_t *result)
{
    return send_ack_at(t, seq, channel, status, result, s_wire_ver, false);
}

const j2534_transport_t *j2534_srv_active(void)
{
    return s_active;
}

const j2534_transport_t *j2534_srv_active_session(uint32_t *session)
{
    xSemaphoreTake(s_send_lock, portMAX_DELAY);

    const j2534_transport_t *t = s_active;

    *session = s_session_no;
    xSemaphoreGive(s_send_lock);
    return t;
}

void j2534_srv_set_wire_version(uint32_t version)
{
    s_wire_ver = (version >= 2) ? J2534_SESSION_VERSION : 1;
}

uint32_t j2534_srv_wire_version(void)
{
    return s_wire_ver;
}

uint32_t j2534_srv_wire_status(uint32_t status)
{
    return j2534_status_wire(status, s_wire_ver);
}

/* ---- one tester session ---------------------------------------------------- */

void j2534_server_serve_transport(const j2534_transport_t *t)
{
    if (t == NULL || t->read == NULL || t->write == NULL || !j2534_srv_running())
    {
        return; /* server disabled - nothing to serve */
    }

    /* single tester across all transports: a second one is told so
       (ACK ERR_DEVICE_IN_USE to its first frame) instead of being left
       hanging on a link that stays up (BLE), then its session ends */
    if (xSemaphoreTake(s_session_lock, 0) != pdTRUE)
    {
        j2534_hdr_t h;
        uint8_t first[8]; /* a HELLO's payload: the version it asks for */
        volatile bool run = true;
        j2534_frame_rc_t rc = j2534_frame_read(t->read, t->ctx,
                                               J2534_READ_SLICE_MS,
                                               J2534_REFUSE_SLICES, &run, &h,
                                               first, sizeof(first));
        const j2534_transport_t *owner = s_active;

        ESP_LOGW(TAG, "tester already attached on %s; refusing %s",
                 (owner != NULL) ? owner->name : "?", t->name);

        /* TOO_BIG: a first frame that is not a HELLO; its header was read,
           and that is all the answer needs. The refusal carries the number
           the refused client knows: 0x0E for one that asked for wire
           version 2 in its HELLO, 0x1A for every other. */
        if (rc == J2534_FR_OK || rc == J2534_FR_TOO_BIG)
        {
            uint32_t ver = (rc == J2534_FR_OK && h.type == J2534_MT_HELLO)
                               ? j2534_hello_version(first, h.length) : 1;

            (void)send_ack_at(t, h.seq, h.channel, J2534_ERR_DEVICE_IN_USE,
                              NULL, ver, true);
        }

        return;
    }

    size_t cap = 0;
    uint8_t *payload = j2534_srv_req_payload(&cap); /* PSRAM; one session */

    j2534_srv_session_begin(t->name);
    xSemaphoreTake(s_send_lock, portMAX_DELAY);
    s_wire_ver = 1;                        /* until its HELLO says more */
    s_session_no++;
    s_active = t;                          /* the RX pump may now push */
    xSemaphoreGive(s_send_lock);

    while (j2534_srv_running())
    {
        j2534_hdr_t h;
        j2534_frame_rc_t rc = j2534_frame_read(t->read, t->ctx,
                                               J2534_READ_SLICE_MS, 0,
                                               j2534_srv_run_flag(), &h,
                                               payload, cap);

        if (rc == J2534_FR_BAD_HDR || rc == J2534_FR_TOO_BIG)
        {
            ESP_LOGW(TAG, "bad frame header; dropping tester (%s)", t->name);
            break;
        }

        if (rc != J2534_FR_OK)
        {
            break; /* link down / server stopping */
        }

        j2534_srv_count_rx();
        j2534_srv_handle_frame(t, &h, payload);
    }

    /* under the send lock: a send that is under way on this transport ends
       first (a TCP one within its 5 s), and none starts after */
    xSemaphoreTake(s_send_lock, portMAX_DELAY);
    s_active = NULL;                       /* stop the RX pump pushing */
    s_wire_ver = 1;
    xSemaphoreGive(s_send_lock);
    j2534_srv_session_end();
    xSemaphoreGive(s_session_lock);
}

/* ---- legacy serial vtable (usb_cdc_device) ----------------------------------- */

static int serial_read_thunk(void *ctx, uint8_t *buf, size_t n,
                             uint32_t timeout_ms)
{
    const j2534_serial_transport_t *s = ctx;

    return (s != NULL) ? s->read(buf, n, timeout_ms) : -1;
}

static int serial_write_thunk(void *ctx, const uint8_t *buf, size_t n)
{
    const j2534_serial_transport_t *s = ctx;

    return (s != NULL) ? s->write(buf, n) : -1;
}

static j2534_transport_t s_serial_adapter =
{
    .name = "serial", .read = serial_read_thunk, .write = serial_write_thunk,
};

void j2534_server_set_serial_transport(const j2534_serial_transport_t *t)
{
    s_serial_adapter.ctx = (void *)t;
}

void j2534_server_serve_serial(void)
{
    if (s_serial_adapter.ctx != NULL)
    {
        j2534_server_serve_transport(&s_serial_adapter);
    }
}

/* ---- TCP ------------------------------------------------------------------------ */

/* One accepted connection: the transport's ctx, on the listener's stack
   for the session's life. */
typedef struct
{
    int           fd;
    volatile bool dead; /* a send failed or timed out (the pump's task or
                           the session's): the session's next read says
                           "link down" and the session ends */
} tcp_link_t;

/* The options of an accepted connection (the block comment at the top of
   this file). false: one was refused by the stack; the connection is
   served all the same, as every connection was before. */
static bool tcp_link_options(int fd)
{
    int one = 1;
    int idle = J2534_TCP_KEEPIDLE_S;
    int intvl = J2534_TCP_KEEPINTVL_S;
    int cnt = J2534_TCP_KEEPCNT;
    struct timeval snd = { .tv_sec = J2534_TCP_SEND_WAIT_S, .tv_usec = 0 };
    int bad = 0;

    bad |= setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    bad |= setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
    bad |= setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    bad |= setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    bad |= setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    bad |= setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    return bad == 0;
}

static int tcp_read(void *ctx, uint8_t *buf, size_t n, uint32_t timeout_ms)
{
    tcp_link_t *l = ctx;
    struct timeval tv = { .tv_sec = timeout_ms / 1000,
                          .tv_usec = (timeout_ms % 1000) * 1000 };

    if (l->dead)
    {
        return -1; /* a send gave the tester up */
    }

    setsockopt(l->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int r = recv(l->fd, buf, n, 0);

    if (r == 0)
    {
        return -1; /* peer closed the socket */
    }

    if (r < 0)
    {
        /* anything but "nothing yet" ends the session: the peer's reset,
           or the stack's own verdict after the keepalive probes */
        return (errno == EWOULDBLOCK || errno == EAGAIN) ? 0 : -1;
    }

    return r;
}

static int tcp_write(void *ctx, const uint8_t *buf, size_t n)
{
    tcp_link_t *l = ctx;
    size_t off = 0;

    if (l->dead)
    {
        return -1; /* given up already: no second wait */
    }

    while (off < n)
    {
        /* blocks while the tester's window is shut, J2534_TCP_SEND_WAIT_S
           at most: it returns what it could hand over in that time, and
           fails when that was nothing */
        int w = send(l->fd, buf + off, n - off, 0);

        if (w <= 0)
        {
            bool timed_out = (w < 0) &&
                             (errno == EWOULDBLOCK || errno == EAGAIN);

            l->dead = true;

            if (timed_out)
            {
                j2534_srv_count_tx_timeout();
                ESP_LOGW(TAG, "tcp: the tester took nothing for %d s; its "
                              "session ends (a PC that vanished, or one "
                              "that stopped reading)", J2534_TCP_SEND_WAIT_S);
            }

            return -1;
        }

        off += (size_t)w;
    }

    return (int)n;
}

/* True if @p addr is the current IP of the netif with @p ifkey. */
static bool addr_is_ifkey(uint32_t addr, const char *ifkey)
{
    esp_netif_t *n = esp_netif_get_handle_from_ifkey(ifkey);
    esp_netif_ip_info_t ip;

    return n != NULL &&
           esp_netif_get_ip_info(n, &ip) == ESP_OK &&
           ip.ip.addr == addr;
}

/* Interface-exposure gate (TCP only - serial and BLE are paired / cabled
 * local links and never reach it): which of WiCAN's own IPs did the
 * connection land on? WIFI_AP_DEF / "USBND" are the ifkeys of the SoftAP
 * and the usb_net_device NCM netif. */
static bool local_iface_allowed(uint32_t local_addr)
{
    return j2534_tcp_gate_allowed(j2534_settings_config()->allow_lan,
                                  local_addr == htonl(INADDR_LOOPBACK),
                                  addr_is_ifkey(local_addr, "WIFI_AP_DEF"),
                                  addr_is_ifkey(local_addr, "USBND"));
}

static void listener_task(void *arg)
{
    const uint16_t port = (uint16_t)(uintptr_t)arg;
    int listen_fd = -1;

    while (j2534_srv_running())
    {
        if (listen_fd < 0)
        {
            listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

            if (listen_fd < 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            int opt = 1;
            setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt,
                       sizeof(opt));

            struct sockaddr_in addr = { 0 };
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port = htons(port);

            if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
                listen(listen_fd, J2534_LISTEN_BACKLOG) != 0)
            {
                ESP_LOGE(TAG, "bind/listen on :%u failed", port);
                close(listen_fd);
                listen_fd = -1;
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            s_listening = true;
            ESP_LOGI(TAG, "listening on tcp/%u", port);
        }

        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int cli = accept(listen_fd, (struct sockaddr *)&peer, &plen);

        if (cli < 0)
        {
            if (!j2534_srv_running())
            {
                break;
            }

            continue;
        }

        struct sockaddr_in local;
        socklen_t llen = sizeof(local);

        if (getsockname(cli, (struct sockaddr *)&local, &llen) != 0 ||
            !local_iface_allowed(local.sin_addr.s_addr))
        {
            ESP_LOGW(TAG, "refused connection on a non-AP/USB interface "
                     "(enable j2534_server.allow_lan to permit LAN access)");
            close(cli);
            continue;
        }

        if (!tcp_link_options(cli))
        {
            ESP_LOGW(TAG, "tcp: a socket option was refused (keepalive / "
                          "send timeout): a tester that vanishes may hold "
                          "this session");
        }

        tcp_link_t link = { .fd = cli, .dead = false };
        j2534_transport_t tcp =
        {
            .name = "tcp", .read = tcp_read, .write = tcp_write,
            .ctx = &link,
        };

        j2534_server_serve_transport(&tcp);
        close(cli);
    }

    if (listen_fd >= 0)
    {
        close(listen_fd);
    }

    s_listening = false;
    s_listener = NULL;
    vTaskDelete(NULL);
}

/* ---- lifecycle hooks for the core ------------------------------------------------ */

esp_err_t j2534_srv_transport_init(void)
{
    if (s_send_lock == NULL)
    {
        s_send_lock = xSemaphoreCreateMutexStatic(&s_send_lock_buf);
    }

    if (s_session_lock == NULL)
    {
        s_session_lock = xSemaphoreCreateMutexStatic(&s_session_lock_buf);
    }

    return (s_send_lock != NULL && s_session_lock != NULL) ? ESP_OK
                                                           : ESP_ERR_NO_MEM;
}

esp_err_t j2534_srv_listener_start(uint16_t port)
{
    if (s_listener != NULL)
    {
        return ESP_OK;
    }

    s_listener = xTaskCreateStatic(listener_task, "j2534", J2534_TASK_STACK,
                                   (void *)(uintptr_t)port, J2534_TASK_PRIO,
                                   s_listener_stack, &s_listener_tcb);
    return (s_listener != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

bool j2534_srv_listening(void)
{
    return s_listening;
}
