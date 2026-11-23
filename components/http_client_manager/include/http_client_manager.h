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
 * @file http_client_manager.h
 * @brief WiCAN HTTP(S) client owner (service component).
 *
 * The one way components do HTTP-client work (rewrite of the legacy
 * https_client_mgr): synchronous request/response with TLS, auth
 * helpers, and download-to-filesystem. Consumers: autopid cloud posts,
 * web-asset fetch-on-miss, future integrations.
 *
 * THREAD SAFETY (the point of the "manager"): every request builds its
 * own esp_http_client instance on the CALLER's task (no shared mutable
 * state between requests) and a counting semaphore caps concurrent
 * requests (default 4) so many tasks can't exhaust sockets/TLS heap;
 * excess callers briefly block. Calls are SYNCHRONOUS and can take
 * seconds (network): call from ordinary tasks only, never from ISRs,
 * timer callbacks, or event handlers that must not block (mqtt/imu/
 * battery handler contexts included).
 *
 * CALLER STACK REQUIREMENT (found the hard way on the bench):
 * https:// requests run the mbedTLS handshake on YOUR stack, budget
 * ≥8 KB (a 4 KB task stack-overflowed and panicked). download() is
 * stack-agnostic: all media writes run on the manager's own
 * internal-stack writer task, so PSRAM-stack callers are fine.
 *
 * TLS: `https://` URLs verify against, in priority order, a
 * cert_manager SET (`cert_set`, CA + optional client pair = mutual
 * TLS, same model as mqtt_manager) > explicit PEM fields > the built-in
 * certificate bundle.
 *
 * Responses are capped (default 16 KB, config override ≤256 KB) into a
 * PSRAM buffer owned by the caller via http_client_manager_free().
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The caller stack requirement above as a number, in bytes, for a caller
   to check its task against at build time (_Static_assert). Measured
   2026-11-16 on an event manager worker: an https post used 4.3 to
   4.5 KB of the task's stack, a plain one 2.7 KB (the README). */
#define HTTP_CLIENT_MANAGER_TLS_STACK_MIN  (8 * 1024)

#define HTTP_CLIENT_MANAGER_MAX_CONCURRENT 4
#define HTTP_CLIENT_MANAGER_DEFAULT_RESP   (16 * 1024)
#define HTTP_CLIENT_MANAGER_MAX_RESP       (256 * 1024)

typedef enum
{
    HTTP_CLIENT_GET = 0,
    HTTP_CLIENT_POST,
    HTTP_CLIENT_PUT,
    HTTP_CLIENT_DELETE,
} http_client_method_t;

/**
 * Why a request's transport failed (2026-11-22): what esp-tls kept about a
 * connection that did not open, or about a request that got no answer.
 * Enough to tell a name that was not found from a far end that closed the
 * handshake or a certificate that was not accepted; every one of them comes
 * back from the request as the same ESP_ERR_HTTP_CONNECT. All zero after a
 * request that went through.
 */
typedef struct
{
    int      tls_err;    /* esp-tls's own word (ESP_ERR_ESP_TLS_*,
                            ESP_ERR_MBEDTLS_*); 0 = none                    */
    int      tls_code;   /* the TLS library's code as a positive number
                            (0x7280 for -0x7280); 0 = none                  */
    uint32_t cert_flags; /* the certificate check's flags
                            (MBEDTLS_X509_BADCERT_*) where esp-tls hands
                            them over; 0 or 0xFFFFFFFF = none. IDF 6.1 does
                            not for a handshake that failed (it asks for the
                            peer certificate first, and there is none before
                            the handshake is done), so the reason is worked
                            out without them                                */
    int      sock_errno; /* the socket's errno; 0 = none                    */
    bool     opened;     /* the connection was open: the request left and
                            the answer did not come                         */
} http_client_fail_t;

/** What a failed request comes down to, for a person. */
typedef enum
{
    HTTP_CLIENT_FAIL_NONE = 0,       /* the request went through            */
    HTTP_CLIENT_FAIL_DNS,            /* the name was not found              */
    HTTP_CLIENT_FAIL_NO_ANSWER,      /* refused, unreachable, or no answer
                                        to the connection in time           */
    HTTP_CLIENT_FAIL_TLS_CLOSED,     /* the far end closed the connection
                                        during the handshake, no alert      */
    HTTP_CLIENT_FAIL_TLS_REFUSED,    /* the far end answered the hello with
                                        an alert                            */
    HTTP_CLIENT_FAIL_NOT_TLS,        /* the port does not speak TLS         */
    HTTP_CLIENT_FAIL_CERT_UNTRUSTED, /* the built-in bundle holds no root
                                        the certificate's chain ends at     */
    HTTP_CLIENT_FAIL_CERT,           /* the certificate check said no: the
                                        name in it is another one, or (with
                                        a certificate set or an explicit CA)
                                        the chain does not end at that CA.
                                        The two cannot be told apart: see
                                        cert_flags                          */
    HTTP_CLIENT_FAIL_TLS_OTHER,      /* another handshake failure           */
    HTTP_CLIENT_FAIL_NO_REPLY,       /* connected, the answer did not come  */
    HTTP_CLIENT_FAIL_OTHER,
} http_client_fail_reason_t;

/** A code a person can pass on to support, as text: "tls -0x7280",
 *  "tls -0x2700 flags 0x8", "esp-tls 0x8006 errno 113". */
#define HTTP_CLIENT_FAIL_CODE_LEN 48

typedef struct
{
    char *data;        /* PSRAM, NUL-terminated; free via _free()       */
    size_t len;
    int status_code;   /* -1 when the transport failed                  */
    bool truncated;    /* the body went on past max_response (2026-10-18) */
    /* why the transport failed (2026-11-22); zero after a request that
       went through. The one part of the response a failed request fills. */
    http_client_fail_t fail;
    /* where the request went (2026-11-14): read from the connection's
       socket once the headers were in. ha_webhooks takes settings from a
       push's reply only where it would take a registration.              */
    bool conn_known;   /* the two below are valid                         */
    bool peer_private; /* the server's address is private, link-local or
                          unique-local (not routed on the internet)       */
    uint32_t local_ip4; /* the device's own IPv4 the request left from,
                           network byte order; 0 = not an IPv4 connection */
} http_client_response_t;

/** Optional auth; set whichever apply (they combine). */
typedef struct
{
    const char *bearer_token;    /* Authorization: Bearer <token>       */
    const char *basic_username;  /* Authorization: Basic base64(u:p)    */
    const char *basic_password;
    const char *api_key;         /* <api_key_header>: <api_key>         */
    const char *api_key_header;  /* NULL = "x-api-key"                  */
} http_client_auth_t;

typedef struct
{
    const char *url;             /* required; http:// or https://       */
    http_client_method_t method;
    const void *body;            /* POST/PUT payload (NULL = none)      */
    size_t body_len;
    const char *content_type;    /* NULL = "application/octet-stream"   */
    const http_client_auth_t *auth;      /* NULL = none                 */
    const char *const *extra_headers;    /* "Key: Value" strings,       */
    size_t extra_header_count;           /*   pre-formatted             */
    const char *cert_set;        /* cert_manager set for TLS (NULL/"")  */
    const char *ca_pem;          /* explicit CA override (NULL = none)  */
    bool skip_common_name;       /* tolerate CN mismatch (rare devices) */
    int timeout_ms;              /* 0 = 10000                           */
    size_t max_response;         /* 0 = HTTP_CLIENT_MANAGER_DEFAULT_RESP*/
} http_client_request_t;

/** Register the log descriptor + the concurrency limiter. No network. */
esp_err_t http_client_manager_init(void);

/** Lifecycle uniformity (§3); passive: both trivial. */
esp_err_t http_client_manager_start(void);
esp_err_t http_client_manager_stop(void);

/**
 * Perform @p req synchronously. On ESP_OK, @p out holds the (possibly
 * truncated at max_response) body and status_code: release with
 * http_client_manager_free(). Transport failures return an error and a
 * response that is zeroed but for `fail` (why it failed) and status_code
 * -1. HTTP error statuses (4xx/5xx) return ESP_OK with the status_code set:
 * the caller decides what an error is.
 */
esp_err_t http_client_manager_request(const http_client_request_t *req,
                                      http_client_response_t *out);

/** What a request that returned @p err failed on, from the response's
 *  `fail`. Pure (host tested). HTTP_CLIENT_FAIL_NONE for ESP_OK. */
http_client_fail_reason_t http_client_manager_fail_reason(
    esp_err_t err, const http_client_fail_t *f);

/** A short stable word for a reason, for an API field or a log line:
 *  "dns", "no_answer", "tls_closed", "tls_refused", "not_https",
 *  "cert_untrusted", "cert", "tls", "no_reply", "other"; "" for
 *  HTTP_CLIENT_FAIL_NONE. */
const char *http_client_manager_fail_reason_str(http_client_fail_reason_t r);

/** The code for support into @p out (HTTP_CLIENT_FAIL_CODE_LEN is enough):
 *  the TLS library's code and, for a certificate, the check's flags; else
 *  esp-tls's own code and the socket's errno. "" when @p f holds nothing. */
void http_client_manager_fail_code(const http_client_fail_t *f, char *out,
                                   size_t cap);

/** GET convenience. */
esp_err_t http_client_manager_get(const char *url,
                                  http_client_response_t *out);

/** POST convenience. */
esp_err_t http_client_manager_post(const char *url, const void *body,
                                   size_t body_len,
                                   const char *content_type,
                                   http_client_response_t *out);

/**
 * STREAMED download of @p url into @p save_path (filesystem paths,
 * /data or /sd): no size limit beyond free space: the caller receives
 * into ping-pong PSRAM buffers while the manager's writer task drains
 * them to the media, so network RX and media writes OVERLAP (the slower
 * of the two sets the pace). Atomic like every filesystem write: the
 * stream targets a temp sibling, committed by rename, a failed or
 * interrupted download never leaves a torn file. One download at a
 * time (a second caller blocks). @p progress_cb (nullable) reports
 * (downloaded, total; total 0 when unknown).
 */
esp_err_t http_client_manager_download(const char *url,
                                       const char *save_path,
                                       void (*progress_cb)(size_t done,
                                                           size_t total));

/* ---- the streamed GET (2026-10-18: fw_update's manifest and image) ------------ */

typedef struct
{
    /** After the final headers (redirects followed): return false to stop
     *  before any body byte (a status or a Content-Length the caller does
     *  not want; the call then returns ESP_ERR_INVALID_RESPONSE). NULL =
     *  take any 2xx. @p content_length is -1 when unknown (chunked). */
    bool (*on_headers)(int status, int64_t content_length, void *ctx);
    /** The body of a 2xx answer, chunk by chunk (4 KB at most); an error
     *  stops the stream and is returned. Required. */
    esp_err_t (*on_data)(const uint8_t *data, size_t len, void *ctx);
} http_client_stream_cb_t;

typedef struct
{
    int     status_code;    /* the final status, -1 when the transport failed */
    int64_t content_length; /* -1 unknown                                    */
    size_t  received;       /* body bytes handed to on_data                  */
    bool    complete;       /* the body ended where the server said          */
    int     hops;           /* redirects followed (at most 3)                */
    char    etag[64];       /* the ETag header, "" when none                 */
} http_client_stream_info_t;

/**
 * GET @p req->url and hand the body to @p cb->on_data as it arrives: no
 * size cap, no buffering beyond one 4 KB PSRAM chunk. Redirects (301, 302,
 * 303, 307, 308) are followed by hand, at most three hops: a GitHub release
 * asset's 302 to its signed address works. `extra_headers` carries an
 * If-None-Match when the caller has an ETag (a 304 returns ESP_OK with no
 * body). The method is always GET; TLS as request(). The caller's stack
 * carries the handshake (>= 8 KB). Transport failures return an error
 * with @p info->status_code -1; an HTTP error status returns ESP_OK with
 * the status and no body (the caller decides).
 */
esp_err_t http_client_manager_stream(const http_client_request_t *req,
                                     const http_client_stream_cb_t *cb,
                                     void *ctx,
                                     http_client_stream_info_t *info);

/* ---- the chunked upload (2026-10-19: the image pushed into the dongle) ---------- */

/** Called after every chunk; return false to stop (the request ends with
 *  ESP_ERR_INVALID_STATE and info->aborted set). */
typedef bool (*http_client_upload_progress_t)(void *ctx, size_t sent,
                                              size_t total);

typedef struct
{
    int    status_code; /* -1 when the transport failed                    */
    size_t sent;        /* body bytes written                              */
    bool   aborted;     /* the progress callback said stop                 */
    char   body[256];   /* the answer, NUL-terminated, cut at 255 bytes    */
    size_t body_len;
} http_client_upload_info_t;

/**
 * POST (or PUT) @p req->body in pieces of @p chunk bytes (0 = 8 KB) with
 * the Content-Length announced at the open, @p progress after every piece,
 * the answer read into @p info (its first 255 bytes). No redirect is
 * followed. The request's timeout bounds every socket write and the wait
 * for the answer (a peer that verifies an image before answering needs
 * tens of seconds). Transport failures return an error with
 * @p info->status_code -1 and @p info->sent telling how far it got.
 */
esp_err_t http_client_manager_upload(const http_client_request_t *req,
                                     size_t chunk,
                                     http_client_upload_progress_t progress,
                                     void *ctx,
                                     http_client_upload_info_t *info);

void http_client_manager_free(http_client_response_t *resp);

typedef struct
{
    uint32_t requests;    /* completed (any status)                     */
    uint32_t failures;    /* transport-level failures                   */
    uint32_t bytes_rx;
} http_client_stats_t;

esp_err_t http_client_manager_stats(http_client_stats_t *out);

#ifdef __cplusplus
}
#endif
