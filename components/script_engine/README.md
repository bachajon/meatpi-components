# script_engine

Berry scripting for WiCAN: a stored script (`/data/scripts/<name>.be`) or
inline source runs on the engine's own PSRAM-stack runner task against
the device bindings (UDS/OBD requests, a claimed multi-step conversation,
raw CAN, trouble codes, events) with a runtime budget and a kill switch.
The bindings ARE the scripting API; there is no privileged path (design:
`event_manager/SCRIPTING.md`).

Settings (`script_engine`, reboot-to-apply): `enabled` (default **off**),
`max_runtime_ms` (100–120000, default 10 s), `allow_reflash` (default
off: gates UDS 0x34–0x37 and `obd_transfer_file`), `exclusive` (default
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
| `POST /api/scripts/stop` | kill switch (checked at every binding call) |

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

Host suite: `./test.ps1 host script_engine` (name guard, obd core, reflash
streamer, reference + examples, the pin rule and the pulse plan).

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
