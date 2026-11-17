# can_isotp_esp: the public build's native ISO-TP provider

`can_manager` owns the TWAI bus and offers an ISO-TP (ISO 15765-2) **provider
slot** (`can_isotp.h`): whoever registers a `can_isotp_ops_t` there serves
`uds_manager`'s "isotp" transport and the J2534 ISO15765 channel. Until
2026-09-16 only the internal add-on pack registered one, so the public
firmware ran UDS through the OBD chip's AT commands and could not bind a
J2534 ISO15765 channel at all (`TASK_isotp_public.md` §0).

This component is the public provider: the vendored `esp_isotp` stack in its
**externally-managed mode** over `can_manager`'s shared bus.

## API (`include/can_isotp_esp.h`)

- `esp_err_t can_isotp_esp_init(void)` - init-only lifecycle: registers the
  `can_isotp_ops_t` iff the slot is empty (main, right after `ext_manager_init`);
  nothing to start (task + queue come up at the first `open()`), nothing to
  stop (sessions are the consumers', the ops table lives forever).
- `bool can_isotp_esp_active(void)` - true when THIS component is the provider.
- `void can_isotp_esp_get_stats(can_isotp_esp_stats_t *out)` - sessions open/peak,
  opens/open_fails, frames_fed/orphan, pdus_tx/rx, tx_timeouts, rx_dropped,
  rx_oversize (surfaced by `GET /api/uds` as `provider_stats`).
- The provider itself is reached through `can_isotp()` (can_manager's slot):
  open / close / send / recv / set_flow per `can_isotp.h`.

## Dependencies

`PRIV_REQUIRES can_manager esp_isotp log_manager esp_timer`. Init order: after
`can_manager_init` and `ext_manager_init` (a pack's provider wins the slot),
before `uds_manager`/`j2534_server` start; `open()` needs the bus up
(`can_manager_start`). No settings (the consumers carry the knobs).

## Memory footprint

- Static: task stack 4096 B PSRAM (`EXT_RAM_BSS_ATTR`; measured high-water
  3296 B free after a J2534 reflash leg), frame queue 4096 B PSRAM (256 frames since 2026-11-17, static
  storage), TCB + queue control ~250 B internal, table/stats ~100 B internal.
- Per session (heap, freed on close): esp_isotp tx+rx buffers 2 x 4128 B PSRAM
  + its handle ~200 B internal + TX frame pool 16 x ~40 B internal, mailbox
  4 x 4130 B PSRAM, three semaphores ~300 B internal - about 25 KB PSRAM +
  1.2 KB internal per open session, 9 sessions max (1 UDS + 8 J2534 flow
  control sessions since 2026-10-19; estimated from the allocation sizes;
  bench PSRAM min_free stayed 3.89 MB with a session open). Each open
  session holds one of can_manager's 16 subscriber slots.

## Design

```
 consumer task            can_core_rx task              "isotp" task (prio 9, PSRAM stack)
 (uds / j2534)                 |                                |
  open()  ---- subscribe rx_id -> can_isotp_esp_frame_q <------ xQueueReceive
  send()  -> esp_isotp_send ----> FF/SF out via can_manager_send |
             (under s->lock)                                      feed_frame(): esp_isotp_feed_can_frame
  recv()  <- s->rx_sem + mailbox <---------------- on_rx_pdu() <-  + esp_isotp_poll (CFs, FC, N_Bs/N_Cr)
  close() -> unsubscribe, delete transport
```

- **One frame queue, one subscription per session** on the session's exact
  rx id (`can_manager_subscribe_queue(filter=rx_id, mask=0x7FF|0x1FFFFFFF)`).
  Every other bus client keeps getting its own copy of every frame: the CAN
  monitor, bridges and autopid (on the MIC chip) do not change while a
  session is open.
- **The task owns the stack** (`can_isotp_esp_task.c`). `esp_isotp` has no locking, so every
  `esp_isotp_*` call on a session runs under the session mutex; the task
  feeds + polls under it, the consumer's `send()` takes it for the
  `esp_isotp_send` call only. Sessions are looked up under the table mutex,
  which the task holds while feeding, so `close()` cannot free a session
  the task is inside.
- **Polling cadence:** idle = a wake every 100 ms; after any activity a
  session is polled every tick for 300 ms (longer than the stack's N_Bs/N_Cr
  of 100 ms), and for as long as a multi-frame send is in flight. This is
  what guarantees the stack always gets to flag a stalled transfer, so a
  link is never left "in progress" after a timeout.
- **Mailbox** (`can_isotp_esp_mbox.c`, pure, host-tested): a ring of 4 PDUs
  per session in PSRAM; the consumer's `recv()` waits on a counting
  semaphore paired with every put that grows the ring. Full → oldest
  dropped (the newest reply wins). `recv()` into a buffer too small returns
  `ESP_ERR_NO_MEM` and has CONSUMED the PDU: the `can_isotp.h` contract the
  UDS transport drains stale traffic on.
- **Memory:** nothing until the first `open()`: then the task (4 KB PSRAM
  stack, static) and the frame queue (4 KB PSRAM, static); per session
  ~8.3 KB (esp_isotp tx/rx buffers, PSRAM) + 16.5 KB mailbox (PSRAM) + a
  few hundred bytes internal for the semaphores.
- **Registration:** `can_isotp_esp_init()` is called by main right
  after `ext_manager_init` and registers ONLY when `can_isotp()` is still
  NULL, a build carrying the add-on pack keeps the pack's provider (the
  slot is single-writer). It also registers the `esp_isotp` log tag at WARN
  (the stack logs every send at INFO).

## Bus sharing with the OBD chip (autopid)

The WiCAN Pro has two requesters on one bus: the MIC chip (autopid, apps)
and the ESP side (these sessions). `obd_gate` serializes the conversations:
the chip already holds the gate around every command; the two consumers of
this provider hold it around theirs: the UDS isotp transport per
request→final response, the J2534 ISO15765 channel from `WRITE_MSGS` until
`READ_MSGS` delivers the reply (or the 2 s hold self-expires). The gate is
fail-open (3 s wait, then take), so a wedged side can never brick the other.

## Limits (esp_isotp 0.1.1)

- As a SENDER this side follows the ECU's flow control. When that asks for
  no gap (`30 00 00`) the stack hands its consecutive frames over in a
  loop, faster than the bus takes them, and the CAN driver holds 16 frames
  in flight. Until 2026-11-18 the driver's refusal of the 17th came back
  to the stack as an error and the transfer was abandoned: a request above
  118 bytes stopped after 16 or 17 consecutive frames and `send()` timed
  out (bench: 125 bytes and more; with `30 00 01` up to 4000 bytes went
  through). `on_can_tx` now answers `ESP_ERR_TIMEOUT` ("not now") while
  `can_manager_tx_room()` is false and the stack sends that frame at its
  next poll: 4000 bytes leave in 0.14 s. Bench: `j2534_flow_control_bench.py`
  leg Q.
- The flow control THIS side sends as a receiver is the session's own
  (2026-11-17, Elias Kotlyar's pull request 3): `can_isotp_cfg_t`
  `block_size` and `stmin_ms` (or `stmin_us`, which wins when it is not 0:
  ISO 15765-2 knows 100..900 us), and what `set_flow()` puts in their
  place on an open session, from the next first frame or block end on.
  0 and 0 mean BS 0 / STmin 0, the ECU streams the whole message; a BS of
  0 sends one flow control however long the message is (the block counter
  no longer wraps at 255 frames). The Kconfig values
  (`CONFIG_ISO_TP_DEFAULT_BLOCK_SIZE`, `CONFIG_ISO_TP_DEFAULT_ST_MIN_US`)
  no longer reach a session of this provider. Until that day they were
  what every session sent (8 and 1000) and the two fields were ignored.
  The consumers state theirs: `uds_manager` 8 / 1 ms as before, a J2534
  channel 0 / 0 (SAE J2534-1's defaults) until its tool sets
  `ISO15765_BS` / `ISO15765_STMIN`.
- BS 0 at the line rate and a flash write on the unit: an erase holds
  every task about 23 ms, and the frames of that time wait in the CAN
  node's receive queue. At 64 frames it lost 28 and 30 of them
  (`rx_missed`) and with them the 4095-byte PDU that was arriving
  (2026-11-17, the configuration backup written during it). The queue
  holds 256 since that day, and so does this provider's frame queue, which
  takes what the node releases at once: 4095-byte PDUs in a row with
  three 32 KB files written to the flash meanwhile arrive whole
  (`.\test.ps1 j2534fc`, leg W; leg N is the same with the flash at
  rest). An erase longer than the queue lasts (64 ms at 500 kbit/s) would
  still cost a PDU.
- N_Bs / N_Cr = `CONFIG_ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US` (1000 ms since
  2026-10-19, ISO 15765-2's value; 100 ms before). The task polls a session
  at tick cadence for 1.2 s after any activity so the stack's timeout is
  always observed.
- One PDU ≤ `CAN_ISOTP_ESP_MAX_PDU` (4128 B, SAE J2534's cap); 9 sessions
  (1 UDS + 8 J2534 flow-control sessions, up to 4 per J2534 channel); a
  rx id can have one session at a time.
- 29-bit ids through `ext_id`; padding per session (`use_padding`,
  `padding_byte`).

## Stats

`can_isotp_esp_get_stats()`: sessions open/peak, opens/open_fails,
frames_fed/orphan, pdus_tx/rx, tx_timeouts, rx_dropped (mailbox full),
rx_oversize (`recv()` cap too small).

## Bench results (2026-09-16, public build)

- `j2534_transport_probe.py`: raw CAN PASS, ISO15765 bind PASS.
- `j2534_bench.py --reflash --tx 7E2 --rx 7EA` vs `pcan_reflash_ecu.py --scenario
  happy --req 7E2 --resp 7EA`: J2534 REFLASH PASS (10 02, seed/key, the multi-frame 34 -> 74, the 31 01 erase routine).
- `uds_route_bench.py --expect-backend isotp`, autopid running: 20/20 on 7E0
  (median 22 ms) and 20/20 on the same ECU autopid floods (7E4, 28 ms), 0
  corrupt; autopid kept ~11 polls/s; `/api/can` counters unaffected.
- `isotp` task high-water 3296 B free of 4096 after the reflash leg; PSRAM
  min_free 3.89 MB with a session open.

## Testing

- Host: `host_test/` (the mailbox ring), `run_host_tests.sh can_isotp_esp`.
- Bench (public build, DUT + ECU simulator; `TESTING.md`):
  `tools/testbench/obd/uds_route_bench.py --expect-backend isotp`,
  `tools/testbench/usb/j2534_transport_probe.py` (raw CAN + ISO15765 bind),
  `tools/testbench/usb/j2534_bench.py --reflash` against
  `actors/pcan_reflash_ecu.py`, and `autopid_matrix_bench.py` for
  coexistence.
