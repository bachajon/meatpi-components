# script_engine

Berry scripting for WiCAN: a stored script (`/data/scripts/<name>.be`) or
inline source runs on the engine's own PSRAM-stack runner task against
the device bindings (UDS/OBD requests, a claimed multi-step conversation,
raw CAN, trouble codes, events) with a runtime budget and a kill switch.
The bindings ARE the scripting API; there is no privileged path (design:
`event_manager/SCRIPTING.md`).

Settings (`script_engine`, reboot-to-apply): `enabled` (default **off**),
`max_runtime_ms` (100–3600000 since 2026-11-27, was 120000; default 10 s),
`allow_reflash` (default off: gates UDS 0x34–0x37, `obd_transfer_file` and
every `flash_*` binding that transmits), `exclusive` (default
**on**, 2026-09-16: from a script's first ECU access to the end of the run
AutoPID polling and DTC scans stay off the bus: obd_gate's diagnostics
hold, independent of the UDS Tool's switch), `cli`.

## The engine describes itself (2026-09-07)

The web UI's Scripts page (editor · examples · reference) renders what
the firmware reports, so the page never drifts from the bindings:

| Route | What |
|---|---|
| `GET /api/scripts/reference` | `{language, enabled, allow_reflash, limits{src_max,file_max,out_max,sleep_max_ms,name_max,resp_max_bytes,max_runtime_ms}, groups[{id,title}], bindings[{name,sig,group,ret,doc,ex}], globals[{name,doc}], primer[{title,code,note}], errors[{match,hint}], rules{action,with,event}}` |
| `GET /api/scripts/examples` | `{examples:[{id,title,desc,needs,level,size}]}`: the gallery |
| `GET /api/scripts/examples?id=` | one example's Berry source (`text/plain`; 404 unknown): the same handler as the list |
| `POST /api/scripts/check {src}` | compile only → `{ok}` or `{ok:false,error:"syntax_error: string:3: …"}` |

`script_engine_doc.c` holds the reference tables (one line per binding,
the readable globals, a Berry primer, hints keyed on error-message
substrings) and `script_engine_examples.c` the example programs. Both are
PURE and host-tested (`host_test/main/test_doc.c`): every binding must be
documented in a known group with a signature that names it, every example
must be a valid script name under the inline cap. The function table in
`script_engine_bind.c` is cross-checked against the docs at boot
(`se_bindings_selfcheck()` logs any binding without a reference line, or
vice versa). **Adding a binding = one line in each table.** Every example
is run on the bench against the ECU simulator; keep them runnable.

## Run surface

| Route | What |
|---|---|
| `GET /api/scripts` | `{scripts:[{name,size}], dir, busy, enabled, max_runtime_ms}` |
| `POST /api/scripts/run {src}` or `{name}` | run inline source (≤ 8 KB) or a stored script → `{ok, output}` (output ≤ 4 KB, `ERROR: <type>: <message>` + Berry's stack traceback appended on failure); 404 unknown name, 409 busy/disabled |
| `POST /api/scripts/run {src\|name\|path, async: true[, params]}` | start a run and return at once: `202 {ok, async}`; 409 busy/disabled. `path` is a script on the SD card (`/sd/....be`, ≤ 64 KB). `params` (an object) reaches the script through `flash_param(name)` |
| `GET /api/scripts/status[?since=n]` | the run: `{lock, running, async, run, phase, progress{done,total}, ecu_state, commit, abort_requested, prompt{text,choices}?, result{ok,error}?, log[{n,ms,msg}], next, allow_reflash, output}`; `log` holds the `flash_log()` lines after `n` (the last 96 kept) |
| `POST /api/scripts/answer {value}` | answers an open `flash_ask()` prompt; 409 when none is open |
| `POST /api/scripts/release {ack: true}` | leaves a failed (or restart-interrupted) flash's lock; 409 when nothing is held or a run owns it |
| `POST /api/scripts/stop` | kill switch (checked at every binding call); after `flash_commit()` a request the script reads with `flash_abort_requested()`: `{ok, abort_requested: true}` |

Script files are plain files: create/replace with
`POST /api/fs/upload?path=/data/scripts/<name>.be` (raw body), read with
`/api/fs/download`, remove with `DELETE /api/fs/file`. Names:
`[A-Za-z0-9_.-]`, ≤ 40 chars, no `..`.

Error messages name the line of the inline source as `string:<line>:`
(syntax errors) and in the traceback (`string:<line>: in function
\`main\``); the editor turns those into jump-to-line links.

## Rules

A rule runs a script with the `script.run {name}` action (or the sugar
`{"on":"…","script":"name"}`); the trigger event reaches the script as the
`evt_source` / `evt_name` / `evt_<field>` globals: a field exists only
when the trigger carries it (`import global` + `global.contains('evt_param')`
to test; a bare undeclared name is a compile error in Berry). Scripts
publish `script.done {value}` through `emit()` for rules to chain on.
Event-triggered runs block the dispatcher for their duration: keep them
short.

## Files

| File | Role |
|---|---|
| `script_engine.c` | VM lifecycle, runner task, run/check/kill, stored-script loader (internal-stack marshal) |
| `script_engine_bind.c` | the device bindings + the function table |
| `script_engine_bind_can.c` | the bindings on the CAN controller itself: `can_rx`, `can_start`, `can_stop`, `digitalWrite`, `digitalPWM`, and the epilogue of every run; `can_send` (2026-11-21), a raw frame on any bus of the OBD connector |
| `script_engine_gpio.c` / `.h` | the pin rule and the pulse sender of `digitalWrite` / `digitalPWM` (the rule and the pulse plan are pure) |
| `script_engine_doc.c` / `.h` | the reference tables + JSON builders, the engine limits (pure) |
| `script_engine_examples.c` | the built-in examples (pure) |
| `script_engine_obd.c` | the claimed-conversation core over an injected port (pure) |
| `script_engine_name.c` | script-name guard (pure) |
| `script_engine_http.c` | `/api/scripts*` |
| `script_engine_events.c` | the `script.run` action + `script.done` source |
| `script_engine_settings.c` / `_cli.c` | settings descriptor, `script run|test|stop` console command |
| `script_engine_flash.c` / `.h` | ECU flashing: the run's state (phase, progress, prompt, log), the lock and its NVS record, the TX / HTTP / sleep gates |
| `script_engine_flash_bind.c` | the `flash_*` bindings |
| `script_engine_flash_core.c` / `.h` | the lock's states and rules, CRC / Adler-32 / word sums (pure) |
| `script_engine_tp_core.c` / `.h` | ISO 15765-2 frames: segmentation, reassembly, flow control, extended addressing (pure) |
| `script_engine_tp.c` | ISO-TP sessions and raw frames on the runner: queue subscriptions, consecutive-frame pacing |
| `script_engine_flash_file.c` | SD-card reads, writes and SHA-256 through `flash_jobs` (an internal stack) |
| `script_engine_doc_flash.c` | the reference entries of the `flash_*` bindings (pure) |
| `script_engine_http_flash.c` | async runs, `/api/scripts/status`, `answer`, `release` |

Host suite: `./test.ps1 host script_engine` (name guard, obd core, reflash
streamer, reference + examples, the pin rule and the pulse plan, ISO-TP
frames, the flash lock, the checksums).

## ECU flashing: the `flash_*` bindings and the lock (2026-11-27)

A script can now run an ECU's whole programming sequence, UDS
(`34`/`36`/`37`) or a manufacturer's own loader, with the per-frame work in
C. The vehicle knowledge stays in the script; a desktop tool uploads the
script and its data to the SD card, starts it with `"async": true`, and
follows it through `/api/scripts/status`.

**Bytes in, bytes out.** `flash_isotp(tx, rx, opts)` opens an ISO-TP
session on any IDs (11 or 29 bit), with extended addressing (`ae_tx`,
`ae_rx`: a different byte each way is allowed), padding, the BS/STmin it
advertises, and `gap_us`: the least time between its own consecutive
frames, whatever the ECU's flow control allows. PDUs up to 4095 bytes.
`flash_request` waits through `7F xx 78` and raises `nrc` on a negative
reply. `flash_can_send/recv/quiet` handle raw frames (a gateway frame, a
broadcast to wait out). Everything runs on the runner task, so the frames
are paced from one place and the lock can admit exactly them.

**Data on the SD card.** `flash_file_read` (4 KiB a call),
`flash_file_write` (a backup, a readback), `flash_file_sha256` (check a
staged image before the commit). They run through `flash_jobs`, on an
internal stack: an SD read through `obd_file_read` on the runner's PSRAM
stack crashed a WiCAN Pro (cache error, 2026-11-27, firmware
6.0.0-beta.12). Checks: `flash_crc` (any CRC up to 32 bits), `flash_adler32`,
`flash_sum`, `flash_sha256`.

**The point of no return.** `flash_commit(label)` is called just before the
first erase. It writes an NVS record (`se_flash`/`commit`) and only then
locks the device:

- Only the runner may transmit: `can_manager`'s TX gate refuses every other
  sender (SLCAN, J2534, GVRET, RealDash, AutoPID, MQTT, other bindings),
  counted in `tx_gated`.
- HTTP requests that change anything get `423 Locked`, except
  `/api/scripts/stop|answer|release`: no reboot, OTA, settings or file
  changes under a half-written ECU (http_server_manager's lock gate).
- `sleep_manager` holds a running countdown away, as for a firmware update
  (reason `ecu_flash`).
- `max_runtime_ms` stops counting and Stop becomes a request
  (`flash_abort_requested()`): the script decides how to leave the ECU
  recoverable.
- The run ends well: the record is erased, the lock opens. It ends with an
  error: the lock stays (`failed`). The WiCAN restarts in between: it comes
  back locked (`recovery`). Either is left by a recovery script that commits
  and ends well, or by `POST /api/scripts/release {"ack": true}`.

`flash_state('untouched'|'loader'|'unknown')`, `flash_phase`,
`flash_progress`, `flash_log` and `flash_ask` (an operator prompt, e.g.
"turn the ignition off, then on") fill the status a tool shows.

```berry
# a UDS download, sketched (the real sequence is the vehicle's)
var ecu = flash_isotp(0x7E0, 0x7E8, {'pad': 0xAA})
var img = flash_param('image')                     # '/sd/flash/app.bin'
if flash_file_sha256(img) != flash_param('sha256') raise 'flash_error', 'image' end
flash_request(ecu, bytes('1002'), 1000)
var seed = flash_request(ecu, bytes('2701'), 1000)
flash_request(ecu, bytes('2702') + my_key(seed), 1000)
flash_commit('erase')
flash_request(ecu, bytes('3101FF00'), 30000, {'pending_ms': 30000})
flash_state('loader')
# 34 / 36 per block from flash_file_read(img, off, n) / 37 / 11 01 ...
```

Needs `can_manager` enabled (the WiCAN's own CAN controller) and Allow
reflash. Memory (estimated): PSRAM ~51 KB static (4 session buffers of
4095 B, 5 frame queues of 128, the 96-line log, a 4 KiB read buffer, the
async output); internal RAM ~0.5 KB (queue and semaphore objects); no new
task (the SD jobs ride the `flash_jobs` worker, at most 784 B of frame).

## A raw frame on any bus: `can_send` (2026-11-21, GitHub issue #1072)

`can_tx(id, ext, hexbytes)` reaches WiCAN's own CAN controller, which is on
pins 6 and 14 and off as shipped. `can_send(bus, id, hexbytes[, wake[,
ext]])` is autopid's can.send (`autopid_can_send()`, the autopid README,
"can.send"): `'obd'` = the car's OBD protocol through the OBD chip, `'ms'`,
`'sw'`, `'ch'` = MS, single-wire and CH CAN through the chip, `'can'` =
that controller. One frame a call; a sequence is several calls with
`sleep_ms()` between them (each call through the chip takes the chip to the
bus and home again, 0.25 to 0.6 s).

- An id above `0x7FF` is a 29-bit one; `ext = 1` makes a small id one.
  `wake = 1` on `'sw'` sends the frame in the single-wire transceiver's
  high-voltage wake mode.
- Returns true when the frame is on the bus (CAN: acknowledged). False with
  the reason in the global `can_send_error` when nobody acknowledged, the
  chip is with an app or another job, or the CAN controller is off. A
  malformed call (a bus that does not exist, bad hex) raises `can_error`
  with the sentence.
- The script's own Exclusive bus hold does not stop it (the rule action
  `can.send` is the one a diagnostic hold stops).

Bench: the `script` leg of `.\test.ps1 cansend` in the firmware repo.

## Raw CAN receive, bus stop / start, the CAN_TX pulse (2026-11-17)

Pull request 2 (Elias Kotlyar) added five bindings: `can_rx(id,
timeout_ms[, ext])`, `can_start()`, `can_stop()`, `digitalWrite(pin,
value)` and `digitalPWM(pin, ontime_ms, offtime_ms[, rounds[, idle]])`;
the reference (`script_engine_doc.c`) is the description a user reads.

- `can_rx` subscribes a queue to that one id for the length of the call.
- `can_stop()` is `can_manager_stop()`: the controller uninstalled, the
  transceiver in standby, the pins free. Whatever the script does next,
  the runner's epilogue after EVERY run (`se_script_epilogue`, beside
  `se_obd_autorelease`) drops a `can_rx` subscription, hands the pins back
  and starts the bus again if the script stopped it. The `can` bridge jack
  joins the bus again by itself (bridge_endpoints).
- **The pin rule (`se_gpio_check`): a script drives `CAN_TX` and nothing
  else**, and only while the bus is stopped. On the WiCAN PRO every other
  GPIO the module brings out carries a function of the device (the OBD
  chip's reset, power and UART lines, the card, I2C, the USB switches, the
  battery sense, the LED), none is on a header, and handing a pad back at
  the script's end (`gpio_reset_pin`) would leave that function dead until
  a restart. `CAN_RX` is the transceiver's output: a second driver on it is
  a short between two outputs. The pull request's own list refused the
  module's flash, PSRAM, USB, strapping and console pins and left the
  board's lines open; the board rule sits on top of it. A board with free
  pins adds them in `se_gpio_check`.
- `CAN_TX` is bit-banged with a busy loop (the first thing that drives the
  pad is the on-level; a level is timed in steps of 1 us and other work of
  the device can stretch it by a few microseconds). The RMT sender in the
  same file (hardware-timed edges) is for other pins and is not reached on
  the WiCAN PRO.
- The pulses are dominant levels on the vehicle bus: they destroy the
  frames of the other nodes for as long as they last. One `digitalPWM` is
  capped at 1 s; nothing caps a script that calls it in a loop but the
  script's run time (`max_runtime_ms`).

Bench: `python tools/testbench/can/script_can_bench.py` (in the firmware
repository; PCAN as the witness, prints `SCRIPT CAN PASS`).
