/* Host tests for data_logger_recover.c: the pure recovery decisions
   (ROBUSTNESS.md): torn text tails, .wdl resync, corrupt names, the
   PSRAM salvage sanity checks, the CRC, and an open against the internal
   heap (the floor and the fault's detail, case 14). */
#include <string.h>

#include "unity.h"

#include "data_logger_private.h"

void test_text_keep_whole_when_newline_terminated(void)
{
    const char *s = "ts_ms,param,value\n1,2,3\n";

    TEST_ASSERT_EQUAL_UINT(strlen(s), dl_recover_text_keep(s, strlen(s)));
}

void test_text_keep_drops_the_torn_last_line(void)
{
    const char *s = "1,2,3\n4,5,6\n7,8";

    TEST_ASSERT_EQUAL_UINT(12, dl_recover_text_keep(s, strlen(s)));
    TEST_ASSERT_EQUAL_UINT(0, dl_recover_text_keep("no newline at all", 17));
    TEST_ASSERT_EQUAL_UINT(0, dl_recover_text_keep("", 0));
}

void test_wdl_scan_counts_complete_records_only(void)
{
    uint8_t buf[64];
    size_t n = 0;
    bool bad = true;

    /* def "a.b" (id 1), a param (19 B), a 3-byte frame (14 + 3) */
    buf[n++] = 0x01; buf[n++] = 1; buf[n++] = 0; buf[n++] = 3;
    memcpy(&buf[n], "a.b", 3); n += 3;
    buf[n++] = 0x02; memset(&buf[n], 0, 18); n += 18;
    size_t frame_at = n;
    buf[n++] = 0x03; memset(&buf[n], 0, 12); n += 12; buf[n++] = 3;
    buf[n++] = 0xAA; buf[n++] = 0xBB; buf[n++] = 0xCC;

    TEST_ASSERT_EQUAL_UINT(n, dl_recover_wdl_scan(buf, n, &bad));
    TEST_ASSERT_FALSE(bad);
    /* one data byte missing from the frame: the frame is not counted */
    TEST_ASSERT_EQUAL_UINT(frame_at, dl_recover_wdl_scan(buf, n - 1, &bad));
    TEST_ASSERT_FALSE(bad);
    /* cut inside the frame header (dlc byte not there yet) */
    TEST_ASSERT_EQUAL_UINT(frame_at, dl_recover_wdl_scan(buf, frame_at + 5, &bad));
    TEST_ASSERT_FALSE(bad);
}

void test_wdl_scan_flags_garbage(void)
{
    uint8_t buf[8] = { 0x02, 0, 0, 0, 0, 0, 0, 0 };
    bool bad = false;

    /* a partial param record is just partial … */
    TEST_ASSERT_EQUAL_UINT(0, dl_recover_wdl_scan(buf, sizeof(buf), &bad));
    TEST_ASSERT_FALSE(bad);
    /* … an unknown type byte is garbage */
    buf[0] = 0x7F;
    TEST_ASSERT_EQUAL_UINT(0, dl_recover_wdl_scan(buf, sizeof(buf), &bad));
    TEST_ASSERT_TRUE(bad);
    /* a frame claiming 9 data bytes is garbage too */
    uint8_t f[16] = { 0x03 };
    f[13] = 9;
    TEST_ASSERT_EQUAL_UINT(0, dl_recover_wdl_scan(f, sizeof(f), &bad));
    TEST_ASSERT_TRUE(bad);
}

void test_corrupt_names_round_trip(void)
{
    char out[64];
    int64_t epoch = 0;

    TEST_ASSERT_TRUE(dl_recover_corrupt_name("dl_1784563545.db", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("dl_1784563545.db.corrupt", out);
    TEST_ASSERT_TRUE(dl_recover_is_corrupt_name(out, "dl_", &epoch));
    TEST_ASSERT_EQUAL_INT64(1784563545, epoch);
    TEST_ASSERT_FALSE(dl_recover_is_corrupt_name(out, "can_", NULL));
    TEST_ASSERT_FALSE(dl_recover_is_corrupt_name("dl_1784563545.db", "dl_", NULL));
    TEST_ASSERT_FALSE(dl_recover_is_corrupt_name("notes.corrupt", NULL, NULL));
    TEST_ASSERT_FALSE(dl_recover_corrupt_name("dl_1784563545.db", out, 10));
}

void test_record_sanity(void)
{
    dl_record_t p = { .ts_ms = 1784563545000LL, .kind = DL_REC_PARAM,
                      .u.p = { .value = 1.5, .param = 3 } };
    dl_record_t f = { .ts_ms = 0, .kind = DL_REC_FRAME,
                      .u.f = { .id = 0x7E8, .dlc = 8, .flags = DL_FRAME_EXT } };

    TEST_ASSERT_TRUE(dl_recover_record_sane(&p));
    TEST_ASSERT_TRUE(dl_recover_record_sane(&f));
    p.u.p.param = DL_MAX_PARAMS;
    TEST_ASSERT_FALSE(dl_recover_record_sane(&p));
    p.u.p.param = 0; p.ts_ms = -1;
    TEST_ASSERT_FALSE(dl_recover_record_sane(&p));
    p.ts_ms = 4000000000001LL;
    TEST_ASSERT_FALSE(dl_recover_record_sane(&p));
    f.u.f.dlc = 9;
    TEST_ASSERT_FALSE(dl_recover_record_sane(&f));
    f.u.f.dlc = 8; f.u.f.flags = 0x10;
    TEST_ASSERT_FALSE(dl_recover_record_sane(&f));
    f.u.f.flags = 0; f.kind = 7;
    TEST_ASSERT_FALSE(dl_recover_record_sane(&f));
}

void test_ring_sanity_and_fill_recovery(void)
{
    uint32_t fill = 3;

    TEST_ASSERT_TRUE(dl_recover_ring_sane(8, 8, 5, 2, &fill));
    TEST_ASSERT_EQUAL_UINT32(3, fill);
    /* the trio disagrees (reset between two index writes): trust head/tail */
    fill = 9;
    TEST_ASSERT_TRUE(dl_recover_ring_sane(8, 8, 5, 2, &fill));
    TEST_ASSERT_EQUAL_UINT32(3, fill);
    fill = 2;
    TEST_ASSERT_TRUE(dl_recover_ring_sane(8, 8, 1, 6, &fill));
    TEST_ASSERT_EQUAL_UINT32(3, fill);
    /* a full ring is the one legitimate head == tail with fill == cap */
    fill = 8;
    TEST_ASSERT_TRUE(dl_recover_ring_sane(8, 8, 4, 4, &fill));
    TEST_ASSERT_EQUAL_UINT32(8, fill);
    /* head == tail otherwise means empty */
    fill = 5;
    TEST_ASSERT_TRUE(dl_recover_ring_sane(8, 8, 4, 4, &fill));
    TEST_ASSERT_EQUAL_UINT32(0, fill);
    /* garbage indices */
    fill = 0;
    TEST_ASSERT_FALSE(dl_recover_ring_sane(8, 8, 8, 0, &fill));
    TEST_ASSERT_FALSE(dl_recover_ring_sane(0, 8, 0, 0, &fill));
    TEST_ASSERT_FALSE(dl_recover_ring_sane(16, 8, 0, 0, &fill));
    TEST_ASSERT_FALSE(dl_recover_ring_sane(8, 8, 0, 0, NULL));
}

/* ---- the records a ring carries over a reset (2026-11-22) ------------------ */

static dl_record_t param_rec(int64_t ts_ms, double value, int16_t param,
                             uint32_t mono_ms, uint32_t seq)
{
    dl_record_t r = { .ts_ms = ts_ms, .kind = DL_REC_PARAM, .mono_ms = mono_ms,
                      .u.p = { .value = value, .param = param } };

    r.check = dl_recover_record_check(&r, seq);
    return r;
}

void test_record_check_binds_the_fields_and_the_push_number(void)
{
    dl_record_t r = param_rec(1795281224006LL, 95.0, 23, 1000, 7);

    TEST_ASSERT_EQUAL_UINT16(r.check, dl_recover_record_check(&r, 7));
    /* another place in the ring's sequence of pushes */
    TEST_ASSERT_NOT_EQUAL(r.check, dl_recover_record_check(&r, 8));
    /* each field counts */
    dl_record_t x = r;
    x.u.p.param = 12;
    TEST_ASSERT_NOT_EQUAL(r.check, dl_recover_record_check(&x, 7));
    x = r; x.u.p.value = 800.0;
    TEST_ASSERT_NOT_EQUAL(r.check, dl_recover_record_check(&x, 7));
    x = r; x.ts_ms += 1;
    TEST_ASSERT_NOT_EQUAL(r.check, dl_recover_record_check(&x, 7));
    x = r; x.mono_ms += 1;
    TEST_ASSERT_NOT_EQUAL(r.check, dl_recover_record_check(&x, 7));
    x = r; x.flags = DL_REC_ABS;
    TEST_ASSERT_NOT_EQUAL(r.check, dl_recover_record_check(&x, 7));
    /* the padding does not: a struct copy may leave it as it likes */
    x = r;
    memset((uint8_t *)&x + 18, 0xA5, 6);
    TEST_ASSERT_EQUAL_UINT16(r.check, dl_recover_record_check(&x, 7));

    dl_record_t f = { .ts_ms = 5, .kind = DL_REC_FRAME, .mono_ms = 9,
                      .u.f = { .id = 0x7E8, .dlc = 8, .flags = DL_FRAME_EXT,
                               .data = { 1, 2, 3, 4, 5, 6, 7, 8 } } };
    uint16_t c = dl_recover_record_check(&f, 3);

    f.u.f.data[7] = 9;
    TEST_ASSERT_NOT_EQUAL(c, dl_recover_record_check(&f, 3));
    f.u.f.data[7] = 8; f.u.f.id = 0x7E9;
    TEST_ASSERT_NOT_EQUAL(c, dl_recover_record_check(&f, 3));
    f.u.f.id = 0x7E8; f.u.f.dlc = 7;
    TEST_ASSERT_NOT_EQUAL(c, dl_recover_record_check(&f, 3));
}

void test_ring_keep_takes_whole_records_only(void)
{
    static dl_param_entry_t params[DL_MAX_PARAMS];
    dl_record_t buf[8];

    memset(params, 0, sizeof(params));
    for (int i = 0; i < 30; i++)
    {
        params[i].used = true;
    }

    /* four records pushed as numbers 100..103, tail at slot 6 (the ring
       wraps): all of them whole */
    for (uint32_t i = 0; i < 4; i++)
    {
        buf[(6 + i) % 8] = param_rec(1000 + i, 10.0 * i, (int16_t)i, 50 + i,
                                     100 + i);
    }

    TEST_ASSERT_EQUAL_UINT32(4, dl_recover_ring_keep(buf, 8, 6, 4, 104,
                                                     DL_REC_PARAM, params));
    for (uint32_t i = 0; i < 4; i++)
    {
        const dl_record_t *r = &buf[(6 + i) % 8];

        TEST_ASSERT_EQUAL_INT64(1000 + i, r->ts_ms);
        TEST_ASSERT_EQUAL_INT16((int16_t)i, r->u.p.param);
        TEST_ASSERT_EQUAL_UINT8(DL_REC_ABS, r->flags);
        /* checked anew with the flag: a second reset keeps them */
        TEST_ASSERT_EQUAL_UINT16(dl_recover_record_check(r, 100 + i), r->check);
    }
    TEST_ASSERT_EQUAL_UINT32(4, dl_recover_ring_keep(buf, 8, 6, 4, 104,
                                                     DL_REC_PARAM, params));
}

void test_ring_keep_drops_the_torn_and_the_stale_record(void)
{
    static dl_param_entry_t params[DL_MAX_PARAMS];
    dl_record_t buf[8];

    memset(params, 0, sizeof(params));
    for (int i = 0; i < 30; i++)
    {
        params[i].used = true;
    }

    /* the bench's case: the newest record reached the chip with its stamp
       and its value only; the parameter number, the flags and the check
       are still those of the record the slot held a ring length ago */
    dl_record_t old = param_rec(900, 800.0, 12, 40, 103 - 8);

    for (uint32_t i = 0; i < 4; i++)
    {
        buf[i] = param_rec(1000 + i, 10.0 * i, (int16_t)(20 + i), 50 + i,
                           100 + i);
    }

    dl_record_t torn = old;

    torn.ts_ms = buf[3].ts_ms;
    torn.u.p.value = 95.0;
    buf[3] = torn;
    TEST_ASSERT_TRUE(dl_recover_record_sane(&buf[3])); /* ranges pass it */
    TEST_ASSERT_EQUAL_UINT32(3, dl_recover_ring_keep(buf, 8, 0, 4, 104,
                                                     DL_REC_PARAM, params));
    TEST_ASSERT_EQUAL_INT16(22, buf[2].u.p.param);

    /* a slot the indices count while its line never left the cache: the
       whole previous record, valid for ITS push number only */
    for (uint32_t i = 0; i < 4; i++)
    {
        buf[i] = param_rec(1000 + i, 10.0 * i, (int16_t)(20 + i), 50 + i,
                           100 + i);
    }
    buf[1] = param_rec(900, 800.0, 12, 40, 101 - 8);
    TEST_ASSERT_EQUAL_UINT32(3, dl_recover_ring_keep(buf, 8, 0, 4, 104,
                                                     DL_REC_PARAM, params));
    /* the ones behind it stay, moved together, checked for their new places */
    TEST_ASSERT_EQUAL_INT64(1000, buf[0].ts_ms);
    TEST_ASSERT_EQUAL_INT64(1002, buf[1].ts_ms);
    TEST_ASSERT_EQUAL_INT64(1003, buf[2].ts_ms);
    TEST_ASSERT_EQUAL_UINT16(dl_recover_record_check(&buf[2], 103), buf[2].check);
    TEST_ASSERT_EQUAL_UINT32(3, dl_recover_ring_keep(buf, 8, 0, 3, 104,
                                                     DL_REC_PARAM, params));

    /* a record of the other stream, a registry slot not in use */
    for (uint32_t i = 0; i < 3; i++)
    {
        buf[i] = param_rec(1000 + i, 1.0, (int16_t)i, 50 + i, 200 + i);
    }
    TEST_ASSERT_EQUAL_UINT32(0, dl_recover_ring_keep(buf, 8, 0, 3, 203,
                                                     DL_REC_FRAME, NULL));
    for (uint32_t i = 0; i < 3; i++)
    {
        buf[i] = param_rec(1000 + i, 1.0, (int16_t)(29 + i), 50 + i, 200 + i);
    }
    TEST_ASSERT_EQUAL_UINT32(1, dl_recover_ring_keep(buf, 8, 0, 3, 203,
                                                     DL_REC_PARAM, params));
    /* garbage arguments keep nothing */
    TEST_ASSERT_EQUAL_UINT32(0, dl_recover_ring_keep(NULL, 8, 0, 3, 3,
                                                     DL_REC_PARAM, params));
    TEST_ASSERT_EQUAL_UINT32(0, dl_recover_ring_keep(buf, 8, 8, 3, 3,
                                                     DL_REC_PARAM, params));
    TEST_ASSERT_EQUAL_UINT32(0, dl_recover_ring_keep(buf, 8, 0, 9, 3,
                                                     DL_REC_PARAM, params));
}

void test_crc32_known_vector(void)
{
    /* CRC-32/ISO-HDLC of "123456789" */
    uint32_t crc = dl_recover_crc32_update(0, "123456789", 9);

    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc);
    /* incremental == one shot */
    uint32_t a = dl_recover_crc32_update(0, "1234", 4);
    TEST_ASSERT_EQUAL_HEX32(crc, dl_recover_crc32_update(a, "56789", 5));
}

/* ---- an open and the internal heap (ROBUSTNESS.md case 14) ---------------- */

void test_open_floor_is_one_kilobyte_in_one_block(void)
{
    /* the field report of 2026-11-13: 5.3 KB free, the largest block
       3.6 KB. With the stdio buffer kept such a unit opens its file */
    TEST_ASSERT_TRUE(dl_recover_open_ram_ok(3686));
    TEST_ASSERT_TRUE(dl_recover_open_ram_ok(DL_OPEN_RAM_MIN));
    /* under the floor no open is tried: fopen() would abort the unit when
       the C library cannot have its lock (the crash of 2026-11-07) */
    TEST_ASSERT_FALSE(dl_recover_open_ram_ok(DL_OPEN_RAM_MIN - 1));
    TEST_ASSERT_FALSE(dl_recover_open_ram_ok(512));
    TEST_ASSERT_FALSE(dl_recover_open_ram_ok(0));
}

void test_corrupt_is_no_verdict_under_the_floor(void)
{
    /* the engine says corrupt and the unit can read its card: the file is
       set aside (ROBUSTNESS.md case 5) */
    TEST_ASSERT_TRUE(dl_recover_corrupt_counts(true, 31744));
    TEST_ASSERT_TRUE(dl_recover_corrupt_counts(true, DL_OPEN_RAM_MIN));
    /* under the floor the card reads fail for want of a bounce buffer and
       sqlite calls the zeros "not a database": a good file is not touched */
    TEST_ASSERT_FALSE(dl_recover_corrupt_counts(true, DL_OPEN_RAM_MIN - 1));
    TEST_ASSERT_FALSE(dl_recover_corrupt_counts(true, 12));
    TEST_ASSERT_FALSE(dl_recover_corrupt_counts(true, 0));
    /* and a file the engine did not call corrupt never is */
    TEST_ASSERT_FALSE(dl_recover_corrupt_counts(false, 31744));
    TEST_ASSERT_FALSE(dl_recover_corrupt_counts(false, 0));
}

void test_fault_detail_tells_the_memory_from_the_card(void)
{
    char out[48];

    /* the card: the detail as it has been since 2026-09-07 */
    TEST_ASSERT_EQUAL_INT(6, dl_recover_fault_detail(out, sizeof(out), false, false));
    TEST_ASSERT_EQUAL_STRING("params", out);
    TEST_ASSERT_EQUAL_INT(3, dl_recover_fault_detail(out, sizeof(out), true, false));
    TEST_ASSERT_EQUAL_STRING("can", out);
    /* the internal heap: named, so nobody looks for a fault on a good card */
    TEST_ASSERT_EQUAL_INT(27, dl_recover_fault_detail(out, sizeof(out), false, true));
    TEST_ASSERT_EQUAL_STRING("params: out of internal RAM", out);
    TEST_ASSERT_EQUAL_INT(24, dl_recover_fault_detail(out, sizeof(out), true, true));
    TEST_ASSERT_EQUAL_STRING("can: out of internal RAM", out);
    /* it fits the fault store's 48-byte detail; a buffer too small for it
       answers 0 and never a cut text read as the other reason */
    TEST_ASSERT_TRUE(strlen("params: out of internal RAM") < 48);
    TEST_ASSERT_EQUAL_INT(0, dl_recover_fault_detail(out, 10, false, true));
    TEST_ASSERT_EQUAL_STRING("", out);
}
