# socket_manager: TCP/UDP server owner (service)

Owns WiCAN's TCP and UDP listeners: up to 4 simultaneous servers on
different ports (settings-defined), each robust across client churn, Wi-Fi
drops and interface restarts. Every server presents the firmware's standard
chunk-stream face (queue RX via `subscribe`, `send()` TX), so a server wraps
into a `bridge_manager` endpoint with three one-line functions. It knows
nothing about what flows through it: protocol logic is translator territory.

## API

| Call | Behavior |
|---|---|
| `socket_manager_init()` | Registers the settings (`"socket_manager"`) + log descriptors. No sockets. |
| `socket_manager_start()` | Opens the enabled listeners per the boot-applied settings; starts the net task. Refuses `ESP_ERR_INVALID_STATE` when unconfigured (§4.3 step 5). |
| `socket_manager_stop()` | Closes everything, stops the task. |
| `socket_manager_subscribe(server, q)` | Attach the ONE subscriber queue (items `socket_chunk_t`). Second subscriber → `ESP_ERR_INVALID_STATE` (a server is one bridge endpoint). |
| `socket_manager_unsubscribe(server, q)` | Detach. |
| `socket_manager_send(server, data, len)` | TCP: fan-out to ALL clients; UDP: to the last peer heard from (`ESP_ERR_INVALID_STATE` before any datagram). |
| `socket_manager_stats(server, *out)` | clients, bytes in/out, rx/tx drops, reconnects, refused. |
| `socket_manager_server_name(idx)` | configured server name per slot (NULL past end): bridge_endpoints registers a jack per configured name. |

## Decided semantics (spec §2)

- **TCP aggregate**: one server = one stream. TX fans out to every client, RX
  from any client merges into the one queue: classic transparent-bridge
  behavior (what OBD/GVRET tools expect). Per-client addressing: out of
  scope v1.
- **UDP last-peer**: TX targets the source of the most recent datagram (the
  standard gateway trick for UDP "connections").
- **max_clients** reached → **accept-then-close** (counted in `refused`) so
  the excess client sees an immediate close instead of a hang.
- **A client that is behind keeps its connection** (2026-11-17, see "A
  client that stalls" below): TX never blocks, what a client's socket does
  not take waits in a 32 KB backlog and leaves in order, a chunk that finds
  the backlog full is dropped whole and counted (`tx_drops`). Only a client
  that took no byte for 10 s is closed, and a hard socket error closes THAT
  client only; the server and the other clients keep running. An idle dead
  client goes by TCP keepalive (idle=`keepalive_s`, intvl 5 s, cnt 3).
- **`TCP_NODELAY` on every accepted client** (2026-09-08): the streams
  these servers carry are interactive request/response (ELM327 apps on
  35000, slcan, GVRET) and a chip response often leaves as two small
  writes (the data line, then the `>` prompt after the chip's multi-ECU
  wait). With Nagle on, the second write waits for the peer's ACK of the
  first - a phone's delayed ACK is 40-200 ms of jitter per request. The
  bridge already batches by chunk, so there is nothing for Nagle to win.
- **Listener recovery**: creation failures (netif down) retry with bounded
  backoff 1→2→4→8 s (cap), never busy-spinning; recreations counted in
  `reconnects`.
- **RX never blocks**: subscriber queue full → chunk dropped + `rx_drops`.
- Plaintext only, listeners bind `INADDR_ANY` (v1 scope per meatpi
  2026-07-03).

## Task model (stack math)

ONE shared net task multiplexing every server via lwIP `select()`:
1 × 6 KB PSRAM stack instead of 4 × 4 KB per-server tasks (the loop performs
no flash writes and no cache-off work, so a PSRAM stack is safe per §2).
Accept/recv/reap all happen in this task; `send()` runs in the caller under
the same mutex that guards the client tables. Every socket is non-blocking
and nothing under that mutex waits: a caller (a bridge pump) is back at
once whatever a client's link does, and a socket is never closed while
another task is inside a call on it. The net task also watches the sockets
of the clients that are behind for writability and pushes their backlog, so
it leaves on a quiet bus too, and it closes a client that took nothing for
10 s on its 200 ms tick.

## Settings (`"socket_manager"`, version 1)

Field-table bounded-array schema (`socket_manager_settings.c`, standard
§4.1): `servers`, array (maxItems 4) of `{name (a-z0-9_), proto tcp|udp,
port 1..65535, max_clients 1..4 (default 2), keepalive_s 0..600 (default
30), enabled (default false)}`. Cross-item rules (on_validate → pure
`smp_validate_servers`): unique names, unique ports among ENABLED servers,
and **tcp:80 is refused** (2026-07-26: lwip SO_REUSEADDR lets a second
LISTEN pcb bind httpd's port and incoming SYNs then ALTERNATE between the
web server and the socket server: a nondeterministically dead UI/API,
found live by the system_bench degraded leg).

Defaults (meatpi 2026-07-03): `obd0` tcp:35000 **enabled**; `slcan0`
tcp:3333, `gvret0` tcp:23, `udp0` udp:17 parked (disabled).

## Dependencies

`lwip`, `settings_manager`, `log_manager`, `esp_timer`: all private;
`espressif/cjson` (managed, private). Init after settings_manager; start
after `settings_manager_start()`. Wi-Fi/netif bring-up is wifi_manager's
business: listeners on ANY simply start failing/backing off until an
interface exists. Must NOT depend on bridge_manager (no upward edge);
glue/main wraps servers as endpoints.

## Memory footprint (estimated: measure before release)

| Where | What | ~Size |
|---|---|---|
| PSRAM `.bss` | server table (fds, client tables, stats) + config + net-task stack (6 KB) | ~7 KB |
| PSRAM `.bss` | the backlog pool: 4 buffers of 32 KB, lent to the clients that are behind (`socket_manager_tx.c`) | 128 KB |
| Internal `.bss` | mutex + TCB + queue control blocks | ~1 KB |
| Internal heap | lwIP sockets/PCBs (lwIP's own pools) | lwIP-owned |

## Tests

- **Host (`host_test/`, 14 tests)**: pure policy, backoff progression + cap,
  accept/reject at max_clients, config parse defaults, name charset, unique
  names/ports, disabled-port parking, max_clients bounds; since 2026-11-17
  the backlog (order kept around the end of the storage, many laps without
  a byte lost or repeated, a chunk taken whole or not at all, the stall
  clock). **Green on rpi001 2026-11-17.**
- **Bench (`.\test.ps1 slcan`, leg S)**: the stalls on the real WiFi path
  with 1,100 CAN frames a second, see "A client that stalls".
- **Target (`test_apps/`)**: lwIP-loopback suite (TCP RX/TX, 2-client
  fan-out + merge, accept-then-close at max, abrupt-close reap, UDP
  last-peer + no-peer refusal), see its README. Builds clean; on-target run
  pending DUT reconnection (2026-07-03).
- **Wi-Fi bench + BENCHMARKS.md**: see `BENCHMARKS.md` (methodology +
  scripts committed; numbers pending the DUT).

## HTTP observability (endpoint reference: `components/HTTP_API.md` §6e3)

`GET /api/sockets` (2026-07-05, served by the `api_http` glue: this component
stays transport-free): configured entries from this component's settings
+ live counters from `socket_manager_stats()`; `up:false` = configured but not
running. `bridge` (2026-10-16, the glue cross-references bridge_manager's
settings): the enabled bridge that uses the server, or `null`.

## A server nobody bridged (2026-10-16)

A server with no subscriber accepts clients and discards their bytes
(`deliver()`: "nobody attached yet, by design"). A field report read that
as "slcan over TCP connects but never receives": `slcan0` switched on under
Socket servers, no `can <-slcan-> slcan0` connection row, nothing in the
log to say why. The net task now logs, right after `client connected`
(and on a UDP server's first datagram of the boot),
`W <name>: no connection uses this server, its data goes nowhere
(Settings, Connections)`: once per connect, so a client in a reconnect
loop prints it at the loop's cadence like the INFO line. The gate
`.\test.ps1 slcan` provokes it on purpose (its first leg) and then proves
the bridged path.

## A client that stalls (2026-11-17)

A field report on 6.0.0-beta.11: slcan on TCP 3333 beside ELM327 on 35000,
a 500 kbit/s bus at about 1,100 frames a second. A laptop on a strong link
held the connection; an Android phone lost it in bursts, about once a
minute while driving, always as a clean close, and could reconnect at once.

Reproduced on the bench with that firmware: a client whose ACKs came
166 ms late was closed every time (127 ms late: never). The cause was the
send path of this component. `send()` was blocking with a 100 ms limit and
a send that did not finish closed the client. The slcan bridge writes one
small chunk per CAN frame with `TCP_NODELAY`, and lwIP counts pbufs, not
bytes: `TCP_SND_QUEUELEN` is 57 unacknowledged writes, 52 ms of that bus.
So 52 + 100 ms without an ACK ended the session, and a phone's radio is
away that long many times an hour (a scan, power save, a weak link).

What the send path does now (`socket_manager_tx.c`, the pure part in
`socket_manager_policy.c`):

- The sockets stay non-blocking. What a client's socket does not take
  waits in a backlog, a ring of 32 KB lent from a pool of four for as long
  as that client is behind (about 1.3 s of the bus above). It leaves in
  order, before anything newer.
- A chunk that finds the backlog full, or the pool empty, is dropped whole
  and counted in `tx_drops`; a text protocol's line is never cut. The log
  says it once per stall: `W <name>: a client is behind (N bytes waiting),
  what does not fit is dropped`, then `W <name>: the client caught up, N
  chunk(s) were dropped meanwhile`.
- A client that moved no byte for 10 s is closed: `W <name>: client dropped
  (took no data for 10 s, ...)`. A slow client that still takes data is
  not a dead one and stays.
- One client's stall costs the others nothing: no send waits.

`tx_drops` therefore changed its meaning: it was "a client was closed on a
stalled send" and is "a chunk was dropped for a client that was behind".

Pull request 2 (Elias Kotlyar) had gone at the same code first: it kept the
client and one unsent tail of up to 1 KB, with the blocking send moved out
of the mutex. On the bench that ended the closes, and lost every frame
past the first 110 ms of a stall without a counter (the bridge's queue
overflowed while its pump waited in `send()`); it also let the net task
close a socket another task was sending on. This version is what stands on
top of his commits.
