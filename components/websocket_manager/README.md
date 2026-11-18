# websocket_manager: WebSocket channel owner (service)

Owns WiCAN's WebSocket CHANNELS: up to 6 named `/ws/...` endpoints
(settings-defined) served by THE one httpd: the component registers routes
with `http_server_manager` (ownership inversion; it never starts a server).
Each channel presents the firmware's standard chunk-stream face (queue RX
via `subscribe`, `send()` TX), so a channel wraps into a `bridge_manager`
endpoint with three one-line functions: OBD↔WS, CAN↔WS, the command line
over WS, all configured bridges, no per-use code here. It knows nothing
about what flows through it.

## API

| Call | Behavior |
|---|---|
| `websocket_manager_init()` | Registers the settings (`"websocket_manager"`) + log descriptors. No routes. |
| `websocket_manager_start()` | Registers the enabled channels' `/ws` routes per the boot-applied settings. **Must run before `http_server_manager_start()`** (main's order guarantees this). Refuses `ESP_ERR_INVALID_STATE` when unconfigured (§4.3 step 5). |
| `websocket_manager_stop()` | Drops the client tables (routes live as long as the server). |
| `websocket_manager_subscribe(channel, q)` | Attach the ONE subscriber queue (items `websocket_chunk_t`). Second subscriber → `ESP_ERR_INVALID_STATE` (a channel is one bridge endpoint). |
| `websocket_manager_unsubscribe(channel, q)` | Detach. |
| `websocket_manager_send(channel, data, len)` | One WS frame (channel's binary/text mode) fanned out to ALL connected clients. |
| `websocket_manager_stats(channel, *out)` | clients, frames/bytes in/out, rx/tx drops, refused. |
| `websocket_manager_channel_name(idx)` | configured channel name per slot (NULL past end): bridge_endpoints registers a jack per configured name. |

## Decided semantics (mirroring socket_manager)

- **Multi-client aggregate**: one channel = one stream. TX fans out one
  frame to every connected client, RX from any client merges into the one
  queue. Per-client addressing: out of scope v1.
- **max_clients** reached → the upgrade is refused **before the 101** (the
  excess client sees the handshake fail, counted in `refused`).
- **A client that is behind keeps its connection, and costs the others
  nothing** (2026-11-18, see "A client that stalls" below): a frame is
  pushed to each client's socket without waiting, what a client does not
  take waits in a 64 KB backlog and leaves in order, a frame that finds the
  backlog full is dropped whole and counted (`tx_drops`), and only a client
  that took no byte for 10 s is closed.
- **A client leaves the table when its session ends** (2026-11-18): in the
  web server's session close hook, before the socket is closed. Until then
  a client that left a quiet channel stayed listed until a send failed on
  it, and the socket number, which is given out again at once, could by
  then belong to a client of another channel or to a page request, which
  was sent this channel's frames. The check at every handshake stays as a
  second net.
- **Control frames** (2026-11-18, `handle_ws_control_frames=true`): the
  answer to a PING and the echo of a CLOSE go out through the client's own
  send path, behind what waits for it; a PONG is read and dropped. The
  server's own PONG was written from the server's task into a socket that
  a bridge's task was sending frames on.
- **Dead clients**: a client that sent CLOSE, or whose session the server
  ended, is out of the table at once; the channel keeps running.
  Since 2026-10-21 a client whose handshake is still in flight is not a
  ghost: the pre-handshake hook registers the fd before the 101 goes out,
  and a fan-out send in that moment used to reap the newcomer (it then got
  no frames and a third client took its slot: found by the release test).
  An fd that stays `HTTPD_WS_CLIENT_HTTP` past a 3 s grace is a ghost.
- **TCP_NODELAY** is set on every client at handshake: httpd transmits a
  WS frame as two `send()`s (header, payload), with Nagle the payload
  stalls ~40 ms on the peer's delayed ACK (measured; see BENCHMARKS.md).
- **TX holds the lock, and never waits under it**: every send on a
  client's socket is a `send(MSG_DONTWAIT)` under `wsm_lock`, the lock the
  close hook takes before a socket is closed. (Until 2026-11-18 the sends
  were blocking and therefore made outside the lock, on a snapshot of the
  client list.)
- **RX never blocks the httpd worker**: frames are read into request-scoped
  PSRAM (cap 4 KB/frame), chunked into the subscriber queue with
  `xQueueSend(..., 0)`, queue full → `rx_drops`. A message above 4 KB is
  read off the socket and answered with close code 1009 since 2026-11-18
  (it was a bare reset). Known and left as it is: a message that finds the
  queue full is dropped chunk by chunk, so a part of it can go through. A
  burst of 6000 slcan lines over the WiFi did not get there on the bench
  (the link is slower than a 500 kbit/s bus), so nothing was changed.
- **Reboot-to-apply** (standard §4.2): channels are registered once at
  `start()`.

## The IDF v6 handshake trap (important)

In ESP-IDF v6 httpd **does not call the uri handler at the WebSocket
handshake**: it completes the 101 internally and invokes the handler for
data frames only (`httpd_uri.c`: "If the request is websocket handshake,
then do not call the uri->handler"). Client registration and the
max_clients gate therefore live in **`.ws_pre_handshake_cb`** (returning
non-OK refuses before the 101), which requires
`CONFIG_HTTPD_WS_PRE_HANDSHAKE_CB_SUPPORT=y` (set in the root
`sdkconfig.defaults`). A `req->method == HTTP_GET` branch inside the
handler is dead code: that was this component's first (broken) attempt:
clients connected fine but the client table stayed empty and fan-out went
to nobody.

## Settings (`"websocket_manager"`, version 2)

Field-table bounded-array schema (`websocket_manager_settings.c`, standard
§4.1): `channels`, array (maxItems 6) of `{name (a-z0-9_ ≤15), path (must start
"/ws/", ≤31), max_clients 1..4 (default 2), mode binary|text (default
binary), enabled (default false)}`. Cross-item rules (on_validate → pure
`wsm_validate_channels`): unique names, unique paths.

Defaults: the known consumers: `ws_obd` `/ws/obd` binary **enabled**
(ships live, paired with bridge_manager's default `obd<->ws_obd` bridge =
OBD over WebSocket out of the box, 2026-07-05); `ws_log` `/ws/log` text
**enabled** (the log_sinks live log stream: the route only: no line
flows until log_sinks' `ws_enabled` gate, default false, opens); `ws_can`
`/ws/can` binary and `ws_cli` `/ws/cli` text parked (disabled) until
enabled from the UI/API.

**v1→v2 migration** (`on_migrate` → pure `wsm_migrate_channels`,
host-tested): appends the `ws_log` entry to a stored v1 channel array,
skipped when the user already claimed the name/path or the table is full.
maxItems went 4→6 in the same change (`WEBSOCKET_MANAGER_MAX_CHANNELS`,
§12 headroom).

## Files

- `websocket_manager.c`: lifecycle, settings descriptor, name→index API.
- `websocket_manager_ws.c`: the httpd layer: route registration,
  `ws_pre_handshake` gate + client table, the session close hook, frame RX
  (data and control).
- `websocket_manager_tx.c`: the send side: one frame to every client of a
  channel without waiting, the backlogs (`tx_backlog`), the `ws_tx` task
  that pushes them, the 10 s rule.
- `websocket_manager_internal.h`: what those two share (the channel table,
  its lock); not for the host tests.
- `websocket_manager_policy.c`: PURE parse/validate and the frame header
  (host-testable).

## Tests

- Host suite (`host_test/`, 9 tests on rpi001): parse defaults, text mode,
  the `/ws/` namespace rule, duplicate names/paths, max_clients bounds, the
  v1 to v2 migration, and since 2026-11-18 the frame header (FIN, no mask,
  the three length forms). Run: `.\test.ps1 host websocket_manager`.
- Bench (firmware repo, `.\test.ps1 slcan`, leg W): see "A client that
  stalls".
- Live (2026-07-04, DUT COM2030 + rpi001 via `ws://10.42.0.62/ws/obd`,
  bridge `br_obd = obd <-raw-> ws_obd`, live MIC3624 + ECU sim): `ATI` →
  `ELM327 v2.3`, `VTVERS` → `MIC3624 V2.3.22`, `0100` → `41 00 FF FF FF FF`;
  two-client fan-out byte-identical; third client refused at handshake
  (max_clients 2). **WS LIVE PASS** (`tools/testbench/ws_live_test.py`).
- Benchmarks (2026-07-04, `tools/testbench/ws_bench.py`): see
  `BENCHMARKS.md`, 128 B ~85 KB/s/direction (frame-rate-bound), 1 KB
  WS→TCP 359 KB/s, RTT p50 5.4 ms, OBD polls p50 54.5 ms.

## What a client writes, and a client that only listens (2026-11-18)

**What a client writes waits for the bridge.** A message goes into the subscriber's queue while
that has room (`wsm_rx_deliver`, `websocket_manager_rx.c`; the server's task never waits). When
it has none, the message waits in order in a buffer in external RAM and the flush task moves it
on as the bridge takes it (`wsm_rx_feed`, on the 5 ms tick). One buffer of 128 KB is lent to the
channel that needs it and goes back when the channel has caught up, so one channel can be behind
at a time. It is not larger, and there is one, because the unit's free external RAM has a floor
to keep: two buffers of 256 KB took its lowest point to 723 KB where the `live` stage's stack
audit asks for 1 MB (with one of 128 KB the lowest point is 1,116 KB). A message that does not
fit there is dropped whole, counted in `rx_drops` and said once in the log; the log says again
when the channel has caught up, with the count. `rx_drops` counted chunks before, and there was
nothing to wait in: what found the subscriber's queue of 64 chunks full was gone, in the middle
of a message. Shown by the slcan stage's leg W6 (a burst of 6000 slcan lines over `/ws/can`):
2,167 of 6,000 frames reached the bus before, 6,000 after. The leg had passed on the firmware of
that morning for the wrong reason: the server's task was short of CPU and read the burst slower
than the bus took it.

**A client that only listens is in use.** The web server has seven places and, when another
connection arrives, closes the one used longest ago; it counts a use only when a connection
sends. A client of the CAN monitor, the log or the live values sends nothing after its
handshake, so every plain request after it made it the oldest. Once a second a client that took
data in the last 2 s is reported as in use (`httpd_sess_update_lru_counter`, called on the
server's own task through `httpd_queue_work`). Leg W8: a reader of `/ws/can` beside eight plain
connections was closed before; now it stays and the server closes two of the plain ones.

## A client that stalls (2026-11-18)

Found by reading after the same defect in `socket_manager`, then reproduced
on the bench (firmware repo, `.\test.ps1 slcan`, leg W: a CAN bus at 1,100
frames a second bridged to `/ws/can`, the clients on the Pi). A frame went
out through `httpd_ws_send_frame_async()` in the bridge's own task, one
client after the other, and each send could wait for the server's send
timeout, 5 s:

- beside one client that took nothing, a client that kept reading lost
  11,367 to 13,419 of about 26,000 to 31,000 frames, with a hole of up to
  6 s; the silent one was never closed and nothing was logged;
- beside one client that took 2 KB a second, it got a third of its frames
  (6,600 of 19,800);
- of 750 lines written the other way meanwhile, 353 to 383 reached the bus:
  the bridge's task serves both directions.

What the send path does now (`websocket_manager_tx.c`):

- The frame's header is built here and the frame is pushed to each client's
  socket with `MSG_DONTWAIT` (the socket itself stays blocking: the web
  server reads it). What a client does not take waits in its backlog, a
  ring of 64 KB lent from a pool of four for as long as that client is
  behind (the `tx_backlog` component, shared with the TCP servers), and
  leaves in order, before anything newer.
- A frame that finds the backlog full, or the pool empty, is dropped whole
  and counted in `tx_drops`; a frame is never cut. The log says it once per
  stall: `W <name>: a client is behind (N bytes waiting), what does not fit
  is dropped`, then `W <name>: the client caught up, N frame(s) were dropped
  meanwhile`.
- A client that moved no byte for 10 s is closed: `W <name>: client dropped
  (took no data for 10 s, ...)`. A slow client that still takes data stays.
- The task `ws_tx` pushes the backlogs every 5 ms while one holds
  something, so they leave on a quiet channel too.

`tx_drops` therefore changed its meaning: it counted clients reaped on a
failed send and counts frames dropped for a client that was behind.

Memory: the pool is 4 x 64 KB = 256 KB of PSRAM `.bss`, the task's stack
3 KB of PSRAM; 48 bytes of state per client slot. What clients write waits
in another 128 KB of PSRAM `.bss` (one buffer, see above). Not a setting:
external RAM is cheap, so it is sized with room instead, up to the floor
of free external RAM the unit keeps.

## HTTP observability (endpoint reference: `components/HTTP_API.md` §6e3)

`GET /api/ws` (2026-07-05, served by the `api_http` glue: this component
stays transport-free): configured entries from this component's settings
+ live counters from `websocket_manager_stats()`; `up:false` = configured but not
running. `bridge` (2026-10-16): the enabled bridge on the channel, or
`null` (information, not a fault: `ws_log` is fed by the log sinks, not a
bridge).
