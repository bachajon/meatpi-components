# j2534_server

> Wire protocol (all transports): [`J2534_WIRE_PROTOCOL.md`](J2534_WIRE_PROTOCOL.md).
> Windows DLL: `drivers/j2534/` in the WiCAN app repo. BLE binding:
> `ble_j2534` + `ble_manager/BLE_API.md`.

## Summary

SAE J2534 PassThru device: a tester (the Windows DLL, a phone app, a
bench script) opens a session, connects CAN, ISO15765, J1850 VPW and
TP 2.0 channels, writes messages, sets filters and periodic messages, and
receives bus traffic as unsolicited `RX_MSG` frames. One framed wire
protocol runs over three transports: the TCP listener in this component
(port 6809), USB CDC-ACM (`usb_cdc_device`) and BLE (`ble_j2534`, since
2026-09-21). Exactly one tester at a time across all of them; a second one
is answered `ERR_DEVICE_IN_USE` (over TCP it is not answered while the
first session lives, and is served when that one ends).

**A session when something goes wrong beside it (2026-11-18).** Four
defects found by reading the code, each shown on the bench first (the
firmware checkout's `.\test.ps1 j2534session`, `testing/j2534session.md`):

- *The PC vanishes.* A TCP tester that disappeared without closing (a
  pulled cable, a laptop asleep) kept the one session until the unit
  restarted: the send blocked for good and nothing asked whether the peer
  was still there. The connection now carries TCP keepalive (the first
  probe after 10 s of silence, one every 5 s, given up after 3) and a
  send timeout of 5 s. An idle session whose peer is gone ends after
  about 25 s, one the device is sending to after about 6 s
  (`tx_timeouts` on `/api/j2534`), and the next connection is served. A
  second connection while a session is ALIVE is still not served: a
  second tool must never interrupt a flash in progress. A live tester
  answers the probes in its TCP stack, so a session may be silent for
  as long as it likes.
- *Messages are lost.* A channel's receive queue holds 256 bus frames.
  When the tester's link is slower than the bus, or the tester stops
  reading, it runs over and drops the oldest frame for every new one.
  The tester was never told. The loss is counted now (`rx_lost`), logged
  at most once a second, and a tester that speaks wire version 2 is sent
  an `EVENT RX_LOST` with the count in front of the first message behind
  the gap (`j2534_channel_lost.c`, the pump's `tell_lost`). That is where
  SAE J2534-1 has the application told: the Windows driver (1.10) returns
  `ERR_BUFFER_OVERFLOW` from the `PassThruReadMsgs` that hands that
  message over.
- *A long ISO15765 write held everything up.* One mutex guards the
  channel records, and the write held it from the first frame to the
  last, seconds at an ECU's STmin. The RX pump waited on it: nothing
  received on any channel reached the tester meanwhile, and what did not
  fit a queue was lost. The tester's own ISO15765 write gives the
  lock up while the message is on the bus (`j2534_iso_write`); the
  session it uses is marked `tx_busy` so nobody reopens it under the
  transfer. A TP2.0 and a J1850 write still hold the lock to their end
  (the TP2.0 write reads the channel's queue itself).
- *Periodic messages ran in the esp_timer task.* The timer's callback
  sent the message, and a send waits: for that lock, for room in the CAN
  driver, for the conversation gate. All that time no other software
  timer of the unit fired, and afterwards the timer fired once for every
  period it had missed, so the copies went out in one burst. The
  callback now only marks the message due; a task of its own
  (`j2534_per`, `j2534_server_periodic.c`) sends it. A message that is
  still waiting when its next period comes goes once
  (`periodic_skipped` counts those periods). `DISCONNECT` now stops the
  periodic messages of its channel; they stayed in the table before and
  went out again on the next channel that took the slot.

**Return codes: SAE J2534-1's numbers (2026-11-18).** Seven codes were
numbered wrong since the first version, here and in the Windows driver
alike: `ERR_MSG_PROTOCOL_ID` to `ERR_INVALID_DEVICE_ID` one too low (0x14
to 0x19, the standard has 0x15 to 0x1A) and `ERR_DEVICE_IN_USE` 0x1A (the
standard: 0x0E). A standard application read "the bus runs at another bit
rate" as `ERR_NOT_UNIQUE`. `j2534_proto.h` has the standard's numbers
now. What goes on the wire follows the session (`j2534_proto_session.c`,
pure): a client that asks for wire version 2 in its `HELLO` gets the
standard's numbers and the `EVENT` frames; every other client (wire
version 1: drivers up to 1.9, the benches, a phone application) gets the
numbers it was written against and no `EVENT`, byte for byte as before.
Driver 1.10 asks for version 2 and, on a firmware that answers 1, turns
the old numbers into the standard's itself, so its application gets the
right ones with any firmware. `J2534_WIRE_PROTOCOL.md` sections 5 and 13.

`READ_VBATT` returns the battery monitor's latest calibrated, averaged voltage,
rounded to millivolts, without a CAN channel or bus transaction. OPEN is required.
No sample or an invalid reading returns `ERR_FAILED` with no result. The Windows
DLL also needs the voltage update: older DLLs return a fixed 12,000 mV locally.
The wire version and settings schema are unchanged.

TP 2.0 as its own protocol (`TP2_0_PS`, protocol id 0x800E, SAE J2534-2
section 19; `j2534_channel_tp20.c`, contributed by Elias Kotlyar, merged
2026-11-17). The device does VW's transport as SAE J2819 describes it:
the channel setup and the connection setup (`REQUEST_CONNECTION`), a
message split into 7-byte telegrams and a received one put together, the
block acknowledges, a block sent again from the telegram the ECU names,
"receiver not ready" and a break, and one connection test per `T_CTA` on
an idle connection, answered with a connection acknowledge. Four
connections on the one channel, messages of up to 4092 bytes, broadcast
requests with their key bytes, `GET_DEVICE_INFO`. The application's
message goes out as written and the ECU's comes in as sent: the two length
bytes a VW ECU uses are the application's (SAE J2534-2 19.3.1). The
tester sees whole messages and two indications (`CONNECTION_ESTABLISHED`,
`CONNECTION_LOST`); a tool that does the transport itself still uses a raw
CAN channel, which does not read TP 2.0. The Windows DLL needs the TP 2.0
update: driver 1.7 does not know the protocol. Layouts, defaults and
limits: `J2534_WIRE_PROTOCOL.md` section 11. Proven against a model of
the document and one published real exchange, not yet on a car.

The flow control of an ISO15765 channel is the tool's to set
(2026-11-17; the per-session values in the ISO-TP stack and the first
version are Elias Kotlyar's, pull request 3). `SET_CONFIG` takes
`ISO15765_BS` and `ISO15765_STMIN`, for the channel's open sessions and
the ones opened after, and `GET_CONFIG` answers with the values in force
(`j2534_iso_cfg.c`: SAE J2534-1's ranges, a refused list changes
nothing). Until then the device sent BS 8 / STmin 1 ms whatever a tool
set. A channel opens with BS 0 / STmin 0 now, the defaults SAE J2534-1
names, so a tool that sets nothing gets an ECU's long answer in one go
as well; a tool that wants the old pace sets 8 and 1. With BS 0 an ECU
sends a whole message without a pause; a write to the unit's own flash
during it cost frames while the CAN receive queue held 64, and costs none
since it holds 256 (`J2534_WIRE_PROTOCOL.md` section 12 has the
measurements and the layouts). The Windows DLL sends and reads the two since driver
1.9; older drivers send the IOCTLs as their id alone, which changes
nothing.

ISO 15765-2 extended addressing (`ISO15765_ADDR_TYPE`, 2026-11-19;
contributed by bachajon, pull request 5, with the address extension in
the vendored ISO-TP link, `esp_isotp/PROVENANCE.md`). A gateway that
answers for the ECUs behind it on one pair of CAN ids tells them apart by
one byte in front of every frame (Toyota's from 2019 on: 0x750 / 0x758);
the device refused the flag until then, so Techstream and the like could
not reach an engine ECU behind such a gateway. A FLOW_CONTROL filter
with the flag carries five bytes in its pattern and its flow-control
message, the id and the address extension of that direction, and opens
a session of its own: several share the two ids. Messages are
`[id][address extension][payload]` both ways, single or segmented, and
what comes back (messages, TxDone, START_OF_MESSAGE) carries the flag in
`RxStatus`. One response id holds one kind of addressing: a filter of the
other kind takes the place of every session on it. The layout is pure
and host tested (`j2534_iso_ae.c`). The Windows DLL passes the flag and
the fifth byte through as they are: no driver change. With it came room
for the tools that use it: ten ISO-TP sessions on a channel (four before:
a tool that opens an engine ECU arms a filter for each of 7E8 to 7EF), 63
pending indications (three before: five messages in one write got three
TxDone and the tester was told two were lost), and a message a session
finished before the next first frame on its id is read before that
frame's START_OF_MESSAGE. Layouts and limits: `J2534_WIRE_PROTOCOL.md`
section 3. Proven against a gateway played by a PCAN
(`.\test.ps1 j2534ext`), and by the contributor with Techstream on an
engine ECU on a bench; not yet behind a real gateway.

The RX pump (the `j2534_rx` task) pushes what the channels received as it
arrives: it blocks on the open channel's bus queue between frames (the
channel queues are static for the module's life, reset at CONNECT, so a
waiter never sees one deleted; `j2534_channel_wait_queue`) and falls back
to a 2 ms poll only for what no queue announces (an ISO-TP PDU the
provider completes, a J1850 reply, an indication). A burst of received
frames goes out in ONE transport write (`j2534_srv_send_raw`, up to 32
frames or a maximal frame's worth; the wire contract lets frames be
coalesced). Until 2026-11-06 the pump slept 2 ms between polls and wrote
one frame per transport write: a tool that acknowledges every block of a
transfer (a VW TP 2.0 flasher, the field report of 2026-10-27) paid up to
2 ms of that on every block end, on top of the link's round trip.

J1850 VPW (GM Class 2, protocol id 1) rides the OBD chip, not the native
CAN controller (2026-10-19, modelled on the nightjoker7 4.47p fork but
rebuilt on the v6 `obd_chip` claim model and the chip's own
`VTFullyRequestCk` USDT segmenter). Its messages are `[3-byte
header][payload]` whole, no CAN-id prefix. CONNECT runs the VPW init on the
chip (ATTP2, headers on, auto-receive); WRITE sets the header (ATSH,
cached) and sends the payload, parsing the chip's reply lines back into the
channel; PASS/BLOCK filters are applied in software over the header.
Request/response only (no monitor task for unsolicited frames); GM 4x
programming speed is not implemented. `J1850PWM`/`ISO9141`/`ISO14230` are
chip-wired on the hardware but not built yet (`ERR_NOT_SUPPORTED`).

**The VPW init, first run against a VPW node on 2026-11-11.** Until ECU
sim v2 the bench had no J1850 node, so the channel's reply path had never
run, and it could not have heard one: CONNECT succeeded, the write was
accepted, and no reply came back. Two faults in the init
(`run_vpw_init()`):

- `ATD` stood behind `ATTP2`. "All to defaults" includes the protocol:
  the chip went back to its stored one (`ATDPN` `6` on the bench) and the
  request went out on CAN. `ATD` is first now. The request engine sends
  its `ATM0` ahead of the `ATTP2` behind it, so the chip's memory function,
  which `ATD` switches on again, is off for the session.
- three ST pass filters (`STFAP 00F000,00FF00`, `00FE00`, `006C00`: replies
  to F0, broadcast FE, "ECUs on 6C"). With any ST pass filter set the
  MIC3624 handed on no reply at all: a functional `48 6B 10 41 00 ..` came
  back as `NO DATA`, also with a filter for 6B added. Under CAN, where the
  chip sat because of the first fault, the same words answer `?`. They are
  gone. The chip receives as an ELM does (`ATAR`), and what the
  application gets is decided by the channel's PASS / BLOCK filters in
  software (`j2534_legacy_poll`), which is the J2534 model. `STFCP` stays,
  to clear what an earlier session left.

Bench: the app repo's `.\test.ps1 j2534vpw` (the simulator on VPW alone,
`01 00` and `01 0C` each answered `48 6B 10 41 ..`, nothing written to the
chip). Not checked there: physical addressing (the simulator answered no
`6C 10 F1` request on the app path either), and the last byte of a
delivered message, which is the J1850 CRC as the chip prints it with
headers on (`48 6B 10 41 00 18 3F 80 03 0F`): whether a J2534 application
expects it stripped is an open question.

ISO15765 messages are laid out as SAE J2534-1 says (2026-10-19): the
4-byte CAN id first, then the UDS payload, in both directions, and the
pad flag on each message. The device took the bare payload before, which
no standard tool sends: a udsoncan tester's `10 03` reached the bus as the
6-byte PDU `00 00 07 E0 10 03`, unpadded, and every request timed out
(the field report of 2026-10-19, `test-reports/2026-10-19_0815_j2534_std_tool_repro.md`
in the app repo). The Windows DLL copies messages verbatim and did not
change. The wire version stays 1 (`J2534_WIRE_PROTOCOL.md` section 10).

The CAN bitrate is the device's, not the tester's (2026-10-03): the native
node runs at the `can_manager` setting, or at what its listen-before-talk
policy read off the bus. `CONNECT` (CAN or ISO15765) with another baud is
answered `ERR_INVALID_BAUDRATE` instead of a channel that reads and writes
at the wrong rate (the baud was ignored before). With `can_manager.baud =
auto` and nothing proven yet the tester's baud is taken when the bus has
been silent (2026-10-17, `can_manager_adopt_baud()`: a gatewayed OBD port
speaks only when asked, and a tester there read the refused writes as
"connected, no comm to the CAN bus"); the node runs at it until frames
prove it or the watchdog demotes it. When it cannot be taken (the bus not
silent, unreadable traffic seen, a rate this node does not know) the
channel opens and writes answer `ERR_FAILED`: one `W` line per `CONNECT`,
`tx_refused` on `/api/j2534`, and can_core's own `W transmit refused`
line names the cause. Pure rule for the check: `j2534_connect_baud_check()`.

Layout (each file under the 700-line cap):

| File | Role |
|---|---|
| `include/j2534_proto.h`, `j2534_proto.c` | PURE codec + constants: header/message encode/decode, filter match, status names, the transport-agnostic framer (`j2534_frame_read`), `j2534_ack_encode`, the TCP gate predicate. Host-tested |
| `j2534_proto_session.c` | PURE (2026-11-18): what a session's wire version decides: the version a `HELLO` asks for, a status in the numbers of that version, the `EVENT` frame, what a loss counter gained. Host-tested |
| `j2534_server.c` | core: lifecycle, request dispatch (`j2534_srv_handle_frame`), status, the Exclusive-bus hold |
| `j2534_server_periodic.c` | periodic messages (2026-11-18, out of the core): the table, one esp_timer per message that only marks it due, the `j2534_per` task that sends it |
| `j2534_server_pump.c` | the RX pump (the `j2534_rx` task, its batch and scratch buffers). Out of the core on 2026-11-13: the core had grown to 743 lines, past the cap this table states. Since 2026-11-18 it tells the tester what a channel lost |
| `j2534_server_voltage.c` | READ_VBATT: the battery owner and checked millivolt conversion; host-tested |
| `j2534_server_transport.c` | the `j2534_transport_t` seam: session lock, send lock, the session's wire version, coalesced frame writes, `j2534_server_serve_transport()`, the TCP listener (keepalive, send timeout) + `allow_lan` gate, the legacy serial vtable adapter |
| `j2534_channel.c/.h`, `j2534_channel_private.h` | channels: lifecycle, CAN via `can_manager`, the bus-bitrate rule, the `reflash_seen` count (the content gate went 2026-10-27) |
| `j2534_channel_filter.c` | the filter table: adding a PASS, BLOCK or FLOW_CONTROL filter, stopping one, clearing them, and the PASS/BLOCK verdict the receive paths ask for. Out of `j2534_channel.c` on 2026-11-18: it had grown to 801 lines, past the cap this table states; the code moved unchanged |
| `j2534_channel_lost.c` | what a channel lost on the way in (2026-11-18): its bus queue's drop count and the indications it had no room for, taken before a message is handed over |
| `j2534_channel_iso.c` | the ISO15765 side (2026-10-19), sessions and write: one ISO-TP session per FLOW_CONTROL filter on the registered provider (`can_isotp.h`; `can_isotp_esp` in public builds), in normal or extended addressing, the SAE J2534-1 message layout `[4-byte id][payload]`, padding per message, single frames without a session (functional addressing); the tester's write gives the channel lock up while it is on the bus |
| `j2534_channel_iso_rx.c` | the ISO15765 receive side: the ring of TxDone and START_OF_MESSAGE indications and the RX pump's poll, which hands messages and indications over in the order of the bus. Out of `j2534_channel_iso.c` on 2026-11-20: with extended addressing it had grown to 848 lines, past the cap this table states |
| `j2534_iso_ae.c/.h` | PURE (2026-11-20): ISO15765 extended addressing, the layout: what a FLOW_CONTROL filter asks for, `[id][address extension][payload]` both ways, the single frame behind an address extension, what the receive path asks of a first frame. Host-tested |
| `j2534_iso_cfg.c/.h` | PURE: the `SET_CONFIG` / `GET_CONFIG` lists of an ISO15765 channel (`ISO15765_BS`, `ISO15765_STMIN`): the ranges, a list taken whole or not at all, the answer a GET makes, the values a channel opens with. Host-tested |
| `j2534_channel_legacy.c` | the J1850 VPW side (2026-10-19): the VPW init, header-cached WRITE through `obd_chip` (short payload as one frame, longer through `VTFullyRequestCk`), reply-line parsing with `7F xx 78` ridden out, a small PSRAM RX ring the pump drains; pure chip-command helpers live in `j2534_proto.c` |
| `j2534_server_settings.c` / `_http.c` / `_cli.c` | descriptor, `/api/j2534`, the `j2534` command |

## API

| Function | Purpose |
|---|---|
| `j2534_server_init/start/stop()` | lifecycle; `start` is a no-op when `enabled` is false |
| `j2534_server_status(out)` | `j2534_server_status_t` incl. `transport` (`tcp`/`serial`/`ble`/`none`) |
| `j2534_server_is_enabled()` | the setting (stable across runtime stop): BLE glue keys its channel registration on it |
| `j2534_server_is_running()` | started and enabled |
| `j2534_server_set_exclusive(on)` / `_exclusive()` | the runtime Exclusive-bus switch |
| `j2534_server_serve_transport(t)` | run one tester session on a `j2534_transport_t` (`name`, `read(ctx,buf,n,timeout_ms)` >0 / 0 timeout / <0 link down, `write(ctx,buf,n)` one whole frame per call). Blocks for the session; returns at once when not running; refuses with `ERR_DEVICE_IN_USE` when a tester holds the session |
| `j2534_server_set_serial_transport(t)` / `_serve_serial()` | the legacy CDC-ACM vtable, thin wrappers over the above (kept for `usb_cdc_device`) |
| `j2534_server_register_http()` | `GET/POST /api/j2534` |

A transport owner runs a task that waits for its link, drains stale
bytes, calls `serve_transport`, and throttles 200 ms when it returns
early (`usb_cdc_device.c`, `ble_j2534.c` are the two templates).

## Dependencies

`battery_monitor` (started before the server; cached ADC reading),
`can_manager`, `obd_gate` (diagnostics hold + per-conversation gate),
`obd_chip` (J1850 VPW: the ELM/STN transactions), the ISO-TP provider slot
(`can_isotp.h`), `settings_manager`, `log_manager`, `http_server_manager`,
`cmdline_manager`, lwIP, `esp_netif`, `esp_timer`, cJSON.

## Settings (`j2534_server`, version 2, reboot-to-apply)

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `false` | the whole feature (TCP listener, serial and BLE sessions, the BLE characteristics) |
| `port` | `6809` | TCP listener port |
| `allow_lan` | `false` | TCP only: also accept connections arriving over WiFi STA / USB-Ethernet uplinks |
| `exclusive` | `true` | boot default of the runtime switch: AutoPID off the bus while a tester is attached |
| `cli` | `true` | register the `j2534` command |

`allow_reflash` (the ECU-flashing content gate) was removed on 2026-10-27:
version 2, `on_migrate` drops the stored key (host-tested; the schema would
also tolerate it, unknown keys are permitted, but a stored shape that
changed gets a migration per the standard §8). A backup from version 1
restores through the same migration.
The device forwards whatever a tool writes, on raw CAN, ISO15765 and
J1850; the memory-transfer services it sees are counted on `/api/j2534`
(`reflash_seen`). Why: on raw CAN the gate read every frame as ISO-TP and
refused ordinary frames of a VW TP 2.0 flasher (field report 2026-10-27),
and on ISO15765 it blocked memory reads (0x35/0x36 upload) while missing
every other write service. The owner's consent is `enabled` + `allow_lan`.
A raw CAN frame waits for room in the CAN driver, up to 100 ms
(2026-11-18): the driver holds 16 frames in flight, and of 40 or 100
frames in one `PassThruWriteMsgs` 17 reached the bus before the call ended
`ERR_FAILED` (bench: `j2534_flow_control_bench.py` leg K, 121 frames now).

## Memory footprint (estimated; PSRAM unless noted)

| What | Size |
|---|---|
| `s_req_payload` (request payload), `s_rx_wire`, `s_tx_frame` (one whole frame) | 4384 + 4160 + 4164 B |
| `j2534_msg_t` scratch: RX pump 1, handler 4, periodic slots 4 and the one a START_PERIODIC is looked at in (each ~4.2 KB) | ~42 KB |
| `s_pdu` (the ISO-TP reassembly scratch of the RX pump) and `s_held` (a message the pump holds back for one poll, since 2026-11-19) | 4128 B each |
| channel records `j2534_ch_tbl` (4 channels: 10 filters, 10 ISO-TP session slots, the ring of 64 indications) | 6720 B, measured (the link map; ~1.7 KB with 4 session slots and a ring of 4) |
| J1850 legacy: one chip-reply buffer `s_resp` + the RX ring (8 × 520 B) | ~8.2 KB |
| ISO-TP sessions themselves: the provider's (about 25 KB PSRAM each while open, and 288 B of internal RAM, measured 2026-11-20 over ten sessions, `can_isotp_esp` README); each response id takes one of `can_manager`'s 16 subscriber slots (extended-addressing sessions on one id share theirs), as does every channel's own CAN queue (256 frames since 2026-11-17, 4 KB of PSRAM each: what the CAN node releases at once after a flash write; 32 before). Ten sessions on one channel take ten slots (measured); the device's twelve on distinct ids and four channels would take all sixteen | per open session |
| a TP 2.0 channel while it is open: its state and, for each of the 4 connections, a reassembly buffer and a delivery buffer of 4092 B; taken from the heap at CONNECT, freed at DISCONNECT (not a PSRAM static yet: an open item against the standard) | ~33 KB heap |
| tasks `j2534` (TCP listener) + `j2534_rx` (RX pump) + `j2534_per` (periodic messages, since 2026-11-18): 4 KB stacks each | 12 KB (+ 3 TCBs internal) |
| mutexes (send, session, channels, the periodic table) | internal, ~360 B |
| READ_VBATT helper | measured compiler frame: 64 B; no heap or mutable static storage |

**Never put a `j2534_msg_t` on a task stack** (three of them overflowed a
4 KB stack and reset the device, 2026-07-07). No flash writes anywhere in
this component.

## HTTP / CLI

`GET /api/j2534` (status incl. `transport`), `POST /api/j2534
{"exclusive":bool}`; `j2534` prints the same. Documented in
`components/HTTP_API.md` 6e13 and `cmdline_manager/README.md`. Since
2026-11-18 the status carries four more fields, each loss with a name:
`rx_lost` (received messages a channel's queue dropped before the tester
got them), `tx_timeouts` (TCP sends the tester took nothing of for 5 s;
each ended a session), `periodic_skipped` (periods of a periodic message
that passed while its last copy was still waiting to go) and
`wire_version` (the attached tester's, 1 or 2; 0 with none attached).

## Testing

- Host: `host_test/` (53 Unity cases: codec round trips and bounds,
  filter match, status names, the framer with partial / delayed /
  dropped / malformed / discarded input, the ACK vectors of
  `J2534_WIRE_PROTOCOL.md` section 7, the TCP gate predicate, the
  CONNECT baud rule, the ISO15765 layout helpers: split / join /
  indication / single frame build and parse, and the J1850 VPW chip
  helpers: header/payload split, ATSH, the VTFullyRequestCk builder +
  checksum, reply-line decode / status / responsePending, settings migration,
  and READ_VBATT conversion, session and unavailable/invalid-sample cases,
  and the byte-exact READ_VBATT request and replies of the wire protocol's
  section 3 as a vector, and the TP 2.0 helpers: the timing byte, the
  broadcast key, where a block is sent again from, the device information
  table, and the ISO15765 channel's lists: the two codings of the
  separation time, a list taken whole or refused whole, what is passed
  over, the answer a GET makes, and the byte-exact example of the wire
  protocol's section 12; since 2026-11-18 the session's wire version in
  `test_session.c`: the return codes against SAE J2534-1's numbers (the
  case that failed on the header of the day before), the version a
  `HELLO` asks for, every status in both versions' numbers, the refusal
  of a second tester as each kind of client gets it, the `EVENT RX_LOST`
  vector of the wire protocol's section 13, the loss count; since
  2026-11-20 extended addressing in `test_iso_ae.c`: what a FLOW_CONTROL
  filter asks for, the message split and join, the single frame behind
  an address extension, the two indications, the first frame questions
  of the receive path, the frames a message takes).
- ISO 15765 extended addressing: `.\test.ps1 j2534ext` in the firmware
  checkout (`tools/testbench/usb/j2534_ext_addr_bench.py` -> `J2534 EXT
  ADDR PASS`): a gateway with two ECUs behind one pair of ids, played by
  a PCAN, every frame the device sends read off the bus; single frames,
  messages in several frames both ways up to 4095 bytes, two answers
  frame by frame in turn, a frame for nobody, the flow control as
  `SET_CONFIG` sets it, five TxDone for five messages of one write, an
  acknowledgement in front of the START_OF_MESSAGE that followed it, a
  filter of the other kind on the same ids, ten sessions on a channel
  and the device's twelve with what they cost, a busy bus beside it, and
  the driver DLL called like an application.
- A session when something goes wrong beside it: `.\test.ps1
  j2534session` in the firmware checkout
  (`tools/testbench/usb/j2534_session_bench.py` -> `J2534 SESSION PASS`):
  a second connection while a session is alive, the return codes by wire
  version, a 2 s ISO15765 write with two periodic messages and 500 frames
  arriving on a second channel, a periodic message that must end with
  its channel, a client that stops reading while 2400
  frames arrive and is then told what is missing and where, a session
  silent for 32 s, a peer whose packets are dropped (idle, and with the
  device sending), and the driver DLL against fake devices of both wire
  versions and against the unit.
- ISO 15765 flow control: `.\test.ps1 j2534fc` in the firmware checkout
  (`tools/testbench/usb/j2534_flow_control_bench.py` -> `J2534 FLOW
  CONTROL PASS`): ECUs that follow ISO 15765-2, played by a PCAN, every
  flow control read off the bus; the values a channel opens with, each
  one set and read back, their scope, BS 0 over 585 consecutive frames,
  what is refused, 100 answers of 4095 bytes at the full line rate with
  every frame counted, the same rate with three files written to the
  unit's flash meanwhile, the device's own UDS client, and the driver DLL
  called like an application.
- TP 2.0: `.\test.ps1 j2534tp20` in the firmware checkout
  (`tools/testbench/usb/j2534_tp20_ps_bench.py` -> `J2534 TP20 PASS`):
  ECUs that follow SAE J2819, played by a PCAN, with faults on purpose (a
  lost telegram, "not ready", a break), a published exchange of a real
  engine ECU byte for byte, four connections, 4092-byte messages both
  ways, and the driver DLL called like an application.
- Voltage: `.\test.ps1 j2534vbat` in the firmware checkout, the wire ACK
  and Windows DLL at two real supply voltages, plus fake-device checks for
  older firmware without a result and an unavailable ADC.
- Benches (`tools/testbench/usb/`): `j2534_std_tool_bench.py` (the gate
  `.\test.ps1 j2534std`: a standard tool's byte stream through the
  installed DLL, udsoncan itself when the 32-bit Python is present ->
  `J2534 STD TOOL PASS`), `j2534_bench.py` (TCP handshake +
  `--reflash`), `j2534_transport_probe.py` (raw CAN + ISO15765 on the
  bus), `j2534_auto_baud_bench.py` (`.\test.ps1 j2534auto`),
  `j2534_gate_test.py` (no content gate: 0x34..0x37 forwarded, `reflash_seen`
  counts), `j2534_tp20_gate_bench.py` (raw CAN forwards every TP 2.0 frame
  shape, `TP20 RAW CAN PASS`), `j2534_serial_test.py`
  (CDC-ACM); (`tools/testbench/ble/`): `ble_j2534_bench.py` +
  `ble_j2534_pi.py` -> `BLE J2534 PASS`. `TESTING.md` rows.
