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
 * @file data_logger.c
 * @brief The core: the param registry, the PSRAM record rings, the
 *        lifecycle and the public API (see include/data_logger.h for the
 *        model). Settings live in data_logger_settings.c (standard 4.1),
 *        the writer task and its streams in data_logger_writer.c, the
 *        card-side file operations in data_logger_store.c, the clock in
 *        data_logger_journal.c (split 2026-10-13: the file had grown to
 *        twice the standard's size).
 *
 * Two streams since 2026-07-09 (TASK addendum): params
 * (dl_<epoch>.<ext>, `format` setting) and CAN frames
 * (can_<epoch>.<ext>, `can_format`/`can_log`), each with its own
 * engine instance, rotation and retention. Separate rings so a CAN
 * flood can never evict param records.
 *
 * The clock (2026-10-13, issue #984): every record is stamped with the
 * wall clock AND the monotonic clock; the writer re-bases the records
 * queued before a clock step (data_logger_journal.c).
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "log_manager.h"
#include "restart_tracker.h"

#include "data_logger_private.h"

static const char *TAG = "data_logger";

/* param registry (RAM; registration is allowed before start) */
/* the registry lives in the PSRAM .noinit envelope below (s_params) */
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf;   /* internal: FreeRTOS object */

/* record rings (PSRAM), producers push under s_lock, writer pops.
 * Params and frames are SEPARATE rings so a busy CAN bus can never
 * evict param records; the frame ring's live length comes from the
 * ring_len setting (static at the cap: caps not rationing). */
#define DL_PS_CACHE_LINE 32 /* a data cache line of the S3 as configured
                               (32 bytes): a record and a ring's indices
                               are aligned to it, so neither spans two
                               lines (see the envelope's notes below) */

typedef struct
{
    dl_record_t *buf;
    uint32_t     cap;
    uint32_t     head;
    uint32_t     tail;
    uint32_t     fill;
    uint32_t     dropped;
    uint32_t     seq;     /* pushes so far: the next record's number, the
                             one its check is made with */
} __attribute__((aligned(DL_PS_CACHE_LINE))) dl_ring_t;

/* ---- PSRAM .noinit envelope (2026-09-07, ROBUSTNESS.md case 6) ------------
   The registry and both rings survive warm resets: after a panic, a
   watchdog or a restart that skipped the clean stop, the next boot
   validates this envelope (magic/version/caps, a CRC over the registry
   names, index + per-record sanity: data_logger_recover.c) and writes
   the records that were still queued BEFORE anything new. Power cuts and
   EN-pin resets leave random or bit-rotten PSRAM; the envelope catches
   that and starts clean. Same pattern as restart_tracker / log_manager's
   ring, including the 64-byte MSPI tuning guard that must lead the object.
   Version 2 since 2026-10-13 (the record grew its monotonic stamp).

   Version 3 since 2026-11-22. PSRAM sits behind the CPU's data cache and
   a reset drops the lines that were not written to the chip yet (the S3's
   esp_restart() writes nothing back). The records lay across two cache
   lines each, split between "stamp and value" and "which parameter": the
   newest record of a restart came back with its stamp and value and the
   parameter number of the record that slot held a ring length before
   (hadrive run 3: `autopid.EngineRPM` = 95, the oil temperature; 2 of 24
   restarts on the bench). Since then: a record and a ring's indices are
   each one cache line (the alignment below), every record carries a check
   over its fields and its push number (dl_recover_record_check), the
   shutdown hook writes the envelope to the chip (ps_commit), and the
   adoption keeps a record only when its check holds. */
#define DL_PS_MAGIC   0x53504C44u /* "DLPS" */

typedef struct
{
    uint8_t          guard[64];
    uint32_t         magic;
    uint32_t         version;
    uint32_t         param_cap;
    uint32_t         frame_cap;
    uint32_t         reg_crc;          /* used + names of params[]         */
    dl_ring_t        param_ring;       /* the two rings' indices sit    */
    dl_ring_t        frame_ring;       /* side by side: one write-back  */
    dl_param_entry_t params[DL_MAX_PARAMS];
    dl_record_t      param_slots[DL_RING_LEN]
                         __attribute__((aligned(DL_PS_CACHE_LINE)));
    dl_record_t      frame_slots[DL_CAN_RING_MAX]
                         __attribute__((aligned(DL_PS_CACHE_LINE)));
} dl_psram_t;

_Static_assert(sizeof(dl_ring_t) == DL_PS_CACHE_LINE,
               "a ring's indices must stay one cache line");
_Static_assert(sizeof(dl_record_t) == DL_PS_CACHE_LINE,
               "a record must stay one cache line");

static dl_psram_t s_ps EXT_RAM_NOINIT_ATTR;
#define s_params     (s_ps.params)
#define s_param_ring (s_ps.param_ring)
#define s_frame_ring (s_ps.frame_ring)

static uint32_t reg_crc_calc(void)
{
    uint32_t crc = 0;

    for (int i = 0; i < DL_MAX_PARAMS; i++)
    {
        const dl_param_entry_t *p = &s_ps.params[i];
        uint8_t used = p->used ? 1 : 0;

        crc = dl_recover_crc32_update(crc, &used, 1);
        crc = dl_recover_crc32_update(crc, p->source, sizeof(p->source));
        crc = dl_recover_crc32_update(crc, p->name, sizeof(p->name));
    }

    return crc;
}

static void ps_reset(void)
{
    memset(&s_ps.magic, 0, sizeof(s_ps) - offsetof(dl_psram_t, magic));
    s_ps.magic = DL_PS_MAGIC;
    s_ps.version = DL_PS_VERSION;
    s_ps.param_cap = DL_RING_LEN;
    s_ps.frame_cap = DL_CAN_RING_MAX;
    s_ps.param_ring.buf = s_ps.param_slots;
    s_ps.param_ring.cap = DL_RING_LEN;
    s_ps.frame_ring.buf = s_ps.frame_slots;
    s_ps.frame_ring.cap = 2048; /* live length set by dl_core_apply */
    s_ps.reg_crc = reg_crc_calc();
}

/* The records a ring carried over a reset: each is kept only when its
 * check holds for its place (dl_recover_ring_keep), the rest is dropped
 * and counted. The kept ones were stamped against the previous boot's
 * monotonic clock: they are marked absolute, their wall stamps stay as
 * they are whatever this boot's clock does (the writer never re-bases
 * them). Returns how many were dropped. */
static uint32_t ring_adopt(dl_ring_t *r, int kind)
{
    uint32_t kept = dl_recover_ring_keep(
        r->buf, r->cap, r->tail, r->fill, r->seq, kind,
        (kind == DL_REC_PARAM) ? s_ps.params : NULL);
    uint32_t lost = r->fill - kept;

    r->fill = kept;
    r->head = (r->tail + kept) % r->cap;
    r->dropped = 0;
    return lost;
}

/* Write what the cache still holds of the envelope to the PSRAM chip: the
 * indices and the registry, then the queued part of each ring. A reset
 * drops the cache; this is what the next boot adopts. */
static void ring_commit(const dl_ring_t *r)
{
    const int flags = ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                      ESP_CACHE_MSYNC_FLAG_UNALIGNED;
    uint32_t first = (r->fill <= r->cap - r->tail) ? r->fill
                                                   : r->cap - r->tail;

    if (first > 0)
    {
        (void)esp_cache_msync(&r->buf[r->tail], first * sizeof(dl_record_t),
                              flags);
    }

    if (r->fill > first)
    {
        (void)esp_cache_msync(r->buf, (r->fill - first) * sizeof(dl_record_t),
                              flags);
    }
}

static void ps_commit(void)
{
    /* a restart never waits for a producer: without the lock the indices
       may move meanwhile, and the record that moved them has its check */
    bool locked = xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) == pdTRUE;

    ring_commit(&s_ps.param_ring);
    ring_commit(&s_ps.frame_ring);
    (void)esp_cache_msync(&s_ps, offsetof(dl_psram_t, param_slots),
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                          ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    if (locked)
    {
        xSemaphoreGive(s_lock);
    }
}

/* records carried over a reset that did not pass their check */
static uint32_t s_salvage_dropped;

/* returns the number of records carried over from before a warm reset */
static uint32_t ps_adopt(void)
{
    bool ok = s_ps.magic == DL_PS_MAGIC && s_ps.version == DL_PS_VERSION &&
              s_ps.param_cap == DL_RING_LEN &&
              s_ps.frame_cap == DL_CAN_RING_MAX &&
              s_ps.reg_crc == reg_crc_calc() &&
              dl_recover_ring_sane(s_ps.param_ring.cap, DL_RING_LEN,
                                   s_ps.param_ring.head, s_ps.param_ring.tail,
                                   &s_ps.param_ring.fill) &&
              dl_recover_ring_sane(s_ps.frame_ring.cap, DL_CAN_RING_MAX,
                                   s_ps.frame_ring.head, s_ps.frame_ring.tail,
                                   &s_ps.frame_ring.fill);

    for (int i = 0; ok && i < DL_MAX_PARAMS; i++)
    {
        const dl_param_entry_t *p = &s_ps.params[i];

        if (p->used &&
            (strnlen(p->source, DL_SOURCE_MAX) >= DL_SOURCE_MAX ||
             strnlen(p->name, DL_NAME_MAX) >= DL_NAME_MAX ||
             p->source[0] == '\0' || p->name[0] == '\0'))
        {
            ok = false;
        }
    }

    if (!ok)
    {
        ps_reset();
        return 0;
    }

    s_ps.param_ring.buf = s_ps.param_slots;
    s_ps.frame_ring.buf = s_ps.frame_slots;
    s_salvage_dropped = ring_adopt(&s_ps.param_ring, DL_REC_PARAM) +
                        ring_adopt(&s_ps.frame_ring, DL_REC_FRAME);

    for (int i = 0; i < DL_MAX_PARAMS; i++)
    {
        s_ps.params[i].db_id = 0; /* interned per open file */
    }

    return s_ps.param_ring.fill + s_ps.frame_ring.fill;
}

/* the restart tracker's state, read once at start on the main task (the
 * boot count names this boot in the clock journal); PSRAM: it is big */
static restart_tracker_state_t s_rt EXT_RAM_BSS_ATTR;

static data_logger_stats_t s_stats;  /* enabled, running, salvaged; the
                                        streams and the storage counters
                                        are the writer's and the store's */
static bool s_hooked;                /* esp_restart shutdown handler in place */

static void hook_once(void);

/* ---- rings ------------------------------------------------------------------ */

static esp_err_t ring_push(dl_ring_t *r, const dl_record_t *rec)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    if (r->fill == r->cap)
    {
        r->tail = (r->tail + 1) % r->cap; /* drop-oldest */
        r->fill--;
        r->dropped++;

        if ((r->dropped % 256) == 1)
        {
            ESP_LOGW(TAG, "%s ring full (%lu drops)",
                     (r == &s_frame_ring) ? "frame" : "param",
                     (unsigned long)r->dropped);
        }
    }

    dl_record_t *slot = &r->buf[r->head];

    *slot = *rec;
    slot->check = dl_recover_record_check(slot, r->seq);
    r->seq++;
    r->head = (r->head + 1) % r->cap;
    r->fill++;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

static bool ring_pop(dl_ring_t *r, dl_record_t *rec)
{
    bool got = false;

    xSemaphoreTake(s_lock, portMAX_DELAY);

    if (r->fill > 0)
    {
        *rec = r->buf[r->tail];
        r->tail = (r->tail + 1) % r->cap;
        r->fill--;
        got = true;
    }

    xSemaphoreGive(s_lock);
    return got;
}

static dl_ring_t *ring_of(int st)
{
    return (st == DL_ST_CAN) ? &s_frame_ring : &s_param_ring;
}

esp_err_t dl_frame_push(const dl_record_t *rec)
{
    if (!dl_settings_config()->enabled || !dl_writer_stream_active(DL_ST_CAN))
    {
        return ESP_ERR_INVALID_STATE;
    }

    return ring_push(&s_frame_ring, rec);
}

/* ---- the core as the writer sees it ------------------------------------------ */

bool dl_core_pop(int st, dl_record_t *rec)
{
    return ring_pop(ring_of(st), rec);
}

uint32_t dl_core_fill(int st)
{
    return ring_of(st)->fill;
}

dl_param_entry_t *dl_core_param(int idx)
{
    return (idx >= 0 && idx < DL_MAX_PARAMS) ? &s_params[idx] : NULL;
}

/* After a batch reached the card: the rings' indices go to the PSRAM chip,
 * so that a reset finds the read position behind that batch. Until then the
 * chip may hold the position from before it, and the next boot would adopt
 * the batch's records again (they pass their checks: they ARE the records
 * of those places) and write them a second time. Two cache lines, once a
 * batch. A reset between the card's commit and this leaves that one batch
 * twice in the file; a reset before the commit leaves it queued, as it
 * should be. */
void dl_core_commit_indices(void)
{
    (void)esp_cache_msync(&s_ps.param_ring, 2 * sizeof(dl_ring_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                          ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

void dl_core_forget_db_ids(void)
{
    for (int i = 0; i < DL_MAX_PARAMS; i++)
    {
        s_params[i].db_id = 0;
    }
}

/* on_apply's landing point for the writer-owned knobs: the frame ring's
 * live length here, the engine binding and the stream limits in the
 * writer */
void dl_core_apply(const dl_cfg_t *cfg)
{
    uint32_t want = (cfg->ring_len > DL_CAN_RING_MAX) ? DL_CAN_RING_MAX
                                                      : cfg->ring_len;

    if (s_frame_ring.cap != want)
    {
        /* a different live length re-bases the modular indices: anything
           carried over a warm reset in this ring is void (a settings
           change always came with a clean restart anyway) */
        s_frame_ring.cap = want;
        s_frame_ring.head = 0;
        s_frame_ring.tail = 0;
        s_frame_ring.fill = 0;
    }

    if (cfg->enabled)
    {
        hook_once(); /* records queue from here on, started or not */
    }

    dl_writer_bind(cfg);
}

/* ---- lifecycle -------------------------------------------------------------- */

esp_err_t data_logger_init(void)
{
    static const log_descriptor_t LOG_DESC =
        { "data_logger", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC);

    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    uint32_t carried = ps_adopt();

    if (carried > 0)
    {
        s_stats.salvaged = carried;
        ESP_LOGW(TAG, "salvaged %lu record(s) queued before the last reset",
                 (unsigned long)carried);
    }

    if (s_salvage_dropped > 0)
    {
        s_stats.salvage_dropped = s_salvage_dropped;
        ESP_LOGW(TAG, "%lu queued record(s) did not reach the PSRAM chip "
                 "whole before the last reset: dropped",
                 (unsigned long)s_salvage_dropped);
    }

    dl_events_register();
    return dl_settings_register();
}

/* esp_restart() runs this before the reset: user restart, settings
 * apply, OTA, CLI, factory reset: the files are flushed and closed the
 * way sleep entry does it (ROBUSTNESS.md case 7), and what is queued
 * after that (a producer that still ran, a logger a settings write
 * enabled that starts at the next boot) is written to the PSRAM chip:
 * the reset drops the cache. Panics never get here; the PSRAM envelope
 * and the records' checks cover those (case 6). */
static void dl_on_shutdown(void)
{
    if (s_stats.running)
    {
        ESP_LOGI(TAG, "restart: flushing and closing the log files");
        data_logger_stop();
    }

    ps_commit();
}

/* registered once, the first time records can be queued (the logger
 * starts, or a settings write enables it): a build with the logger off
 * leaves the shutdown handler's slot to others (IDF has five) */
static void hook_once(void)
{
    if (!s_hooked && esp_register_shutdown_handler(dl_on_shutdown) == ESP_OK)
    {
        s_hooked = true;
    }
}

esp_err_t data_logger_start(void)
{
    const dl_cfg_t *cfg = dl_settings_config();

    if (!dl_settings_is_configured())
    {
        ESP_LOGE(TAG, "unconfigured; not starting");
        return ESP_ERR_INVALID_STATE; /* standard 4.3 step 5 */
    }

    s_stats.enabled = cfg->enabled;

    if (!cfg->enabled)
    {
        ESP_LOGI(TAG, "disabled in settings");
        return ESP_OK;
    }

    /* the boot count names this boot in the clock journal (main task:
       the tracker's state is copied under its lock, into PSRAM) */
    if (restart_tracker_get_state(&s_rt) == ESP_OK)
    {
        dl_journal_set_boot(s_rt.boot_count);
    }

    hook_once();

    if (dl_writer_start() != ESP_OK)
    {
        return ESP_FAIL;
    }

    s_stats.running = true;
    ESP_LOGI(TAG, "started (params=%s%s, can=%s%s, batch %lu, "
             "flush %lu ms)", dl_writer_engine_name(DL_ST_PARAM),
             cfg->autopid_log == 2 ? "+autopid.all"
             : cfg->autopid_log == 1 ? "+autopid.changed" : "",
             dl_writer_stream_active(DL_ST_CAN)
                 ? dl_writer_engine_name(DL_ST_CAN) : "off",
             (dl_writer_stream_active(DL_ST_CAN) && cfg->can.monitor_all)
                 ? "/all-ids" : "",
             (unsigned long)cfg->batch_rows,
             (unsigned long)cfg->flush_ms);
    return ESP_OK;
}

esp_err_t data_logger_stop(void)
{
    s_stats.running = false;
    dl_writer_stop();
    return ESP_OK;
}

/* ---- public API --------------------------------------------------------------- */

esp_err_t data_logger_register_param(const char *source, const char *name,
                                     dl_param_t *out)
{
    if (source == NULL || name == NULL || out == NULL ||
        strlen(source) >= DL_SOURCE_MAX || strlen(name) >= DL_NAME_MAX ||
        source[0] == '\0' || name[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_ERR_NO_MEM;

    xSemaphoreTake(s_lock, portMAX_DELAY);

    for (int i = 0; i < DL_MAX_PARAMS; i++)
    {
        if (s_params[i].used)
        {
            if (strcmp(s_params[i].source, source) == 0 &&
                strcmp(s_params[i].name, name) == 0)
            {
                *out = (dl_param_t)i;
                err = ESP_OK;
                break;
            }

            continue;
        }

        strcpy(s_params[i].source, source);
        strcpy(s_params[i].name, name);
        s_params[i].db_id = 0;
        s_params[i].used = true;
        s_ps.reg_crc = reg_crc_calc(); /* the envelope must match at boot */
        *out = (dl_param_t)i;
        err = ESP_OK;
        break;
    }

    xSemaphoreGive(s_lock);
    return err;
}

static esp_err_t param_push(dl_param_t param, int64_t epoch_ms, double value,
                            uint8_t flags)
{
    if (param < 0 || param >= DL_MAX_PARAMS || !s_params[param].used)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dl_settings_config()->enabled)
    {
        return ESP_ERR_INVALID_STATE;
    }

    dl_record_t rec =
    {
        .ts_ms = epoch_ms,
        .u.p = { .value = value, .param = param },
        .kind = DL_REC_PARAM,
        .flags = flags,
        .mono_ms = (uint32_t)(esp_timer_get_time() / 1000),
    };

    return ring_push(&s_param_ring, &rec);
}

esp_err_t data_logger_write_at(dl_param_t param, int64_t epoch_ms,
                               double value)
{
    /* an explicit stamp is the producer's word: never re-based */
    return param_push(param, epoch_ms, value, DL_REC_ABS);
}

esp_err_t data_logger_write(dl_param_t param, double value)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return param_push(param,
                      (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000,
                      value, 0);
}

/* the value sinks' common part: the `autopid_log` gate (off: nothing,
 * changed: when the producer says so, all: every sample), the registry
 * lookup and the ring push. No IO. */
static void value_sink(const char *source, const char *name, double value,
                       bool changed)
{
    const dl_cfg_t *cfg = dl_settings_config();

    if (cfg->autopid_log == 0 || (cfg->autopid_log == 1 && !changed) ||
        !cfg->enabled)
    {
        return;
    }

    dl_param_t param;

    if (data_logger_register_param(source, name, &param) != ESP_OK)
    {
        return; /* registry full: counted nowhere, params are bounded */
    }

    (void)data_logger_write(param, value);
}

/* autopid value sink: main wires it to autopid_set_value_sink().
 * Poller/filter task context: registry lookup + ring push, no IO. */
void data_logger_autopid_sink(const char *name, const char *unit,
                              double value, bool changed)
{
    (void)unit;
    value_sink("autopid", name, value, changed);
}

/* device value sink (2026-11-22): what the unit measures itself, under the
 * name its status push uses, so the record of a drive holds it next to
 * the car's parameters (`device.batt_voltage`). Same gate. */
void data_logger_device_sink(const char *name, double value, bool changed)
{
    value_sink("device", name, value, changed);
}

esp_err_t data_logger_stats(data_logger_stats_t *out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = s_stats;
    dl_writer_stats(out);   /* the streams and the storage counters */

    out->queued = s_param_ring.fill;
    out->dropped = s_param_ring.dropped;
    out->can_queued = s_frame_ring.fill + dl_can_queued();
    out->frames_dropped = s_frame_ring.dropped;
    out->paused = dl_writer_gated();

    /* the clock */
    dl_clock_stats_t cs;

    dl_journal_stats(&cs);
    out->clock_valid = cs.valid;
    snprintf(out->clock_source, sizeof(out->clock_source), "%s",
             dl_clock_source_name(cs.source));
    out->clock_steps = cs.steps_this_boot;
    out->clock_rotations = cs.rotations_by_clock;
    out->journal_bytes = cs.journal_bytes;

    /* the params stream as configured, and what the export served */
    const dl_cfg_t *cfg = dl_settings_config();
    static const char *const AP_LOG[] = { "off", "changed", "all" };

    snprintf(out->format, sizeof(out->format), "%s", cfg->format);
    snprintf(out->autopid_log, sizeof(out->autopid_log), "%s",
             AP_LOG[cfg->autopid_log < 3 ? cfg->autopid_log : 0]);
    dl_export_stats(out);
    return ESP_OK;
}

void dl_runtime_gate(bool on)
{
    dl_writer_set_gate(on);
}
