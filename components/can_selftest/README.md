# can_selftest

The CAN self-test (feature component, issue #1023, 2026-10-28). The
ESP32-S3's own CAN controller (can_manager, the TWAI transceiver) and the
MIC3624 OBD chip (obd_chip) sit on the same CAN_H and CAN_L wires (the
schematic's CAN block: two transceivers on one pair). The test makes them
talk to each other with one standard OBD exchange: the chip asks for engine
speed the way every OBD app does (`010C` to the functional id 7DF) and the
ESP32 answers it as an engine ECU from **7EF**, ECU number 8's response id,
which no production car uses. One exchange proves the chip's transmit and
the ESP32's receive (the request), the ESP32's transmit and the chip's
receive (the reply), and both acknowledgements, with or without a vehicle:
on a live vehicle the real ECUs answer 7E8.. too and the test looks only for
its own line (Ali, 2026-10-28: a live bus runs, reported as `live`).

## API (see `include/can_selftest.h`)

| Function | What |
|---|---|
| `can_selftest_init/start/stop` | lifecycle (§3): the settings descriptor (`{cli}`) and the log descriptor; nothing runs until a test is asked for |
| `can_selftest_run_start()` | the test on a one-shot task (the HTTP route); `ESP_ERR_INVALID_STATE` while one runs; the preconditions land in the result's `refusal` |
| `can_selftest_run(out)` | the test in the caller's task (the console), about 2 s |
| `can_selftest_last(out)` | the job's result (a copy), running or done |
| `can_selftest_parse_reply(resp, want_id, &rpm)` | pure: the chip's answer classified (`ours` with the rpm decoded, `other`, `no_data`, `can_error`, `error`, `none`) |
| `can_selftest_verdict(esp_heard, reply, &pass)` | pure: the one-line verdict, PASS or where to look |
| `can_selftest_register_http()` / `can_selftest_register_cli()` | `GET/POST /api/can/selftest`; the `can_selftest` command (self-registered on settings apply, §6b) |

## The sequence (about 2 s)

1. Preconditions: the CAN bus running (`can_manager_status`), the chip up
   (`obd_chip_ready`) and idle (`obd_chip_client_idle_ms` over 10 s: an app
   on the chip would read the exchange as its own), the bus readable
   (`can_manager_probe`: silent or live both run; unreadable refuses). A
   listen-only node on a silent `auto` bus adopts the configured bitrate
   (500 when auto) so it may transmit; `can_manager_tx_ready` decides.
2. AutoPID is taken off the chip (2026-11-21, below): `obd_gate_diag_hold`,
   then `obd_gate_diag_wait_ack` for up to 3 s, the poller's word that its
   poll in flight has ended. The exchange carries the line `apid   off the
   chip for the test (N ms)`. Then the ESP32 subscribes a queue to 7DF and
   reads the TWAI counters.
3. The chip, in one transaction (`obd_chip_txn_begin`): `ATD` and `ATE0`
   (its defaults, below), `ATTP6` (500 kbit/s) or `ATTP8` (250), `ATH1`,
   `ATSH 7DF`, `ATCAF1`, then `010C` written raw (`obd_chip_send`) while this
   task plays the ECU: the 7DF frame `[02 01 0C]` arrives on the queue within
   400 ms, the reply `7EF [04 41 0C 1A F8]` goes out (`can_manager_send`),
   and the chip's output is collected through the fan-out to its prompt
   (900 ms).
4. Restore: `ATPC`, `ATD`, `ATE0` (ATD puts the chip at its defaults, echo
   on included; the bring-up had the echo off, so it goes off again), the
   transaction ended, `obd_chip_settings_changed()` (AutoPID's baseline
   memory compares the chip's settings generation before every poll and
   re-sends its prelude: spaces, headers, timeout, protocol, functional
   header, filter; bench 2026-10-28, the chip echoed after a test until
   then), the queue released.
5. The verdict (`can_selftest_parse.c`): PASS when the ESP32 heard the request
   and the chip printed the ESP32's line (engine speed 1726 rpm). Otherwise
   where to look: `CAN ERROR` = nothing acknowledged the chip (the shared
   wires or the termination); the ESP32 heard nothing while other ECUs
   answered = the ESP32's transceiver or standby pin; the ESP32 heard nothing
   and nobody answered = the chip's CAN side or protocol; `NO DATA` after the
   ESP32 replied = the ESP32's transmit path.

Three steps with their ms and the exchange line by line are kept for the
card and the console; a failed test logs W, never E (a user-run diagnostic).

### AutoPID is off the chip, and the chip at its defaults (2026-11-21, Ali)

Ali, 2026-11-21: "The canselftest should pause the Autopid, but we should
warn user with a popup if Autopid is active". Until then the test kept
AutoPID out by its transaction's claim alone and took the chip as it found
it. Measured that day on the bench unit, whose table polls without a pause
(29 rows in one 1000 ms group, 13 of them with an ECU of their own), 24 tests
in a row: **12 failed.**

- **Nine ended with a hardware verdict on healthy hardware**: "the ESP32
  answered but the chip never printed it: the ESP32's transmit path or its
  standby pin". The ESP32 had heard the chip's request and answered it; the
  chip printed `NO DATA`. Cause: the chip still carried the receive filter of
  the profile row polled last. A row with an ECU of its own sets one in its
  init (`ATSP6;ATSH746;...;ATCRA7B0;`), nothing takes it away behind the
  poll, and a chip that accepts 7B0 alone drops the ESP32's 7EF and the
  car's 7E8. Proven by hand (`internal/issue_1075/tools/selftest_state.py`,
  AutoPID off, the state set through /ws/obd): `ATTP6;ATSH746;...;ATCRA7B0`
  fails every time; a 29 bit filter does not (the test's own change to an 11
  bit protocol clears it); `ATCRA7EF` passes with the ESP32's line alone.
- **Three were refused** with "the OBD chip could not be claimed": a claim
  is polled, and the poller took the chip again ahead of the test's next
  10 ms look.

What the test does now:

- **The diagnostics hold** (`obd_gate_diag_hold`, the hold the UDS Tool, the
  J2534 server and a script take) before anything listens, and the poller's
  acknowledgement (`obd_gate_diag_wait_ack`, 3 s at most; without it the test
  goes on and says so in its exchange). AutoPID logs `paused: a diagnostic
  tool holds the bus (...)` and `resumed: the diagnostic tool released the
  bus`, counts no failed poll, and sends its baseline again on its own. The
  hold is released on every way out (a refused subscription, a refused
  claim, the end).
- **The chip's defaults first**: `ATD`, then `ATE0` (the bring-up had the
  echo off), ahead of `ATTPn`, `ATH1`, `ATSH 7DF`, `ATCAF1`. ATD clears the
  receive filter, the headers, the timing and the flow control, whoever set
  them: AutoPID, an app that left ten seconds ago, a script.
- The ESP32's queue is emptied (`xQueueReset`) right before the chip is
  asked. The chip's own request arrives within a millisecond of the write,
  so a 7DF frame from before it is somebody else's.

With the hold alone 3 tests of 30 still failed (the filter); with the
defaults as well: see `testing/canselftest.md` in the firmware repo. The
exchange grew by three lines (`apid`, `ATD`, `ATE0`): `CAN_SELFTEST_LINES`
is 16.

The page asks before it starts a test while AutoPID polls (web_ui_v2
README, the diagnostics cards). The console and the route run the test
without asking: the pause is the device's, the question is the page's.

## Surfaces

- `POST /api/can/selftest` -> 202 `{"started":true}` (409 while a run is in
  progress); `GET /api/can/selftest` -> `{running, done, pass, verdict,
  refusal, bus:{baud_kbps, live}, esp_heard, reply, rpm, took_ms,
  steps:[{name, ok, ms, detail}], lines:[]}`. Documented in
  `components/HTTP_API.md` of the app repo.
- `can_selftest` (the last result) and `can_selftest run`.
- The web UI: the CAN self-test card on System > Maintenance (web_ui_v2
  `web/diag.js`).

## Settings (`"can_selftest"`, version 1)

| Key | Type | Default | Notes |
|---|---|---|---|
| `cli` | bool | `true` | the `can_selftest` console command (§6b) |

## Dependencies

can_manager, obd_chip (both must be init'd and started by main first),
obd_gate (the diagnostics hold that takes AutoPID off the chip),
settings_manager, log_manager, cmdline_manager, http_server_manager.

## Memory footprint (estimated)

| Where | What | Size |
|---|---|---|
| PSRAM `.bss` | the job's result and the console's copy (2 x ~1.6 KB), the chip output buffer (512 B), the HTTP copy (~1.6 KB) | ~5.5 KB |
| Internal heap, transient | the job task's stack (4 KB, UART and TWAI driver paths), the two queues (8 CAN frames + 32 chip chunks, ~4.5 KB) during a run | ~8.5 KB for 2 s |
| PSRAM heap, transient | the GET's body (4 KB) | 4 KB per request |

## Tests

- Host (`host_test/`, 9 tests): the reply classifier on the chip's real
  line forms (headers on, with and without spaces, 29-bit ids, a reply from
  another ECU only, `NO DATA`, `CAN ERROR`, `?`, empty), the rpm decode,
  the verdict table.
- Bench: `tools/testbench/can/can_selftest_bench.py` on rpi001 =
  `.\test.ps1 canselftest` in the app repo (simulator broadcast off = PASS
  on a silent bus; broadcast on = PASS with `live:true`; CAN disabled and
  an app on the chip = the refusals), TESTING.md row.
