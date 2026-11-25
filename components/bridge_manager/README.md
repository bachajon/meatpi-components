# bridge_manager: the data-path pump (service)

Moves chunks between any two registered endpoints (OBD ↔ TCP, CAN ↔ UDP,
USB ↔ OBD, anything) fast, with no per-pairing code. Three-piece model
(spec §2, decided): **endpoints** register descriptors, **translators**
(external components) register codecs, **bridges** are settings-configured
pairings. Only `raw` (byte-transparent) is built in; the manager stays
protocol-blind. New transport = one endpoint registration; new protocol =
one translator component. No combinatorial code.

## API

| Call | Behavior |
|---|---|
| `bridge_manager_init()` | Settings (`"bridge_manager"`) + log descriptors. |
| `bridge_manager_start()` | Builds the enabled bridges from the boot-applied settings (subscribe both endpoints, spawn pumps). Unconfigured → `ESP_ERR_INVALID_STATE`. |
| `bridge_manager_stop()` | Unsubscribes + stops every pump. |
| `bridge_manager_register_endpoint(ep)` | Pre-start only. Registered by thin glue or main: providers (obd_chip, socket_manager) never depend on this component. |
| `bridge_manager_register_translator(tr)` | Pre-start only; the translator component registers itself in its `_init()` (a legitimate downward edge). `ctx_size ≤ 256`. |
| `bridge_manager_stats(name, *out)` | Per-bridge chunk/byte counters + send/codec errors. |
| `bridge_manager_losses(name, side_b, out, cap, &count)` | What the endpoint at one end of a running bridge lost, by name (2026-11-18): the endpoint's optional `losses` hook, asked with the queue that bridge subscribed. `count` 0 = the endpoint reports none. |

## Data path

- Chunks are `{uint16_t len; uint8_t data[128]}`: layout-identical to
  `obd_chunk_t` and `socket_chunk_t` (static-asserted in the test app), so
  provider queues carry bridge chunks without adaptation.
- **One pump task per active bridge**, draining BOTH directions via a
  FreeRTOS queue set (README justification: one 4 KB PSRAM stack per bridge
  vs two slim tasks; the queue-set wait gives per-chunk fairness between
  directions with zero cross-blocking). `raw` bridges are pure move-bytes:
  no parsing on the hot path.
- **Raw coalescing (2026-09-08)**: a raw pump sends the chunk in hand plus
  every chunk already waiting behind it in the same queue as ONE endpoint
  `send()` (up to `BM_COALESCE_MAX` = 1 KB). Zero cost for a lone chunk (a
  short ELM reply leaves at once); under an ATMA flood the event-driven
  obd_chip RX emits one ~29-byte chunk per CAN frame, and one socket send
  per chunk capped the TCP bridge around 1000 frames/s (bench: at 2500
  frames/s the fan-out dropped 6052 chunks while every other counter was
  clean). Translators still see chunks one at a time; since 2026-11-17
  (pull request 2, Elias Kotlyar) their OUTPUT is gathered the same way:
  the chunk in hand and every chunk already waiting are run through the
  codec one by one and what it emits goes out as one endpoint `send()` of
  up to 1 KB. The batch is sent from inside the sink when the next output
  does not fit, so the bytes of one sink call stay together (an slcan line
  is never split) and no codec call is ever run a second time (a stream
  codec has moved its state on, and its earlier output would reach the
  destination twice). A destination that takes records must therefore
  take several per `send()`: the `can` jack walks the batch frame by
  frame. **Queue-set rule:**
  every extra chunk is claimed through `xQueueSelectFromSet(set, 0)`, a
  direct `xQueueReceive` on a member queue leaves the set's container
  holding stale notifications until it overflows (`assert
  prvNotifyQueueSetContainer queue.c:3362`, a panic seen live at 2500
  frames/s on the first cut); when the set hands out the other
  direction, the entry is counted (`owed[]`) and that direction's pass is
  next. One pass takes at most `BRIDGE_MANAGER_PASS_MAX` (1 KB) from its
  source.
- **Translators** run per direction with a per-direction reassembly ctx from
  a static PSRAM pool (`decode` on a→b, `encode` on b→a); the sink callback
  lets one input emit zero/one/many frames (slcan/GVRET reassembly shape),
  keeping codecs allocation-free and pure.
- **Slow-side policy**: the pump blocks only inside an endpoint's bounded
  `send()`; meanwhile the bridge-owned RX queues (depth 64 since 2026-09-08,
  was 32) fill and the
  PRODUCING endpoint drops-and-counts (obd fan-out drops, socket rx_drops).
  A bridge can never block its faster side indefinitely.
- **A destination that is slower than its source (`tx_ready`, 2026-11-18)**:
  an endpoint may give the optional `bool (*tx_ready)(void)`. While it
  answers false the pump leaves the chunks meant for it in their queue
  (it counts the queue-set entries it took in `owed[]` and reads the
  chunks later), goes on serving the other direction, and asks again
  every 2 ms. The queue then fills and the SOURCE's own flow control
  acts: a TCP server stops reading its client while the queue has no room
  (socket_manager), so nothing is dropped and nothing waits in the pump.
  The `can` jack is the one endpoint that sets it: a client writes a burst
  faster than the bus takes it. Why a hook and not a wait in `send()`:
  bench 2026-11-18, a wait for the bus inside the `can` jack's `send()`
  delivered a burst of 2000 slcan lines and lost the same client 162
  frames of a bus streaming 1100 frames a second the other way (1015 at
  2000, 3938 at 3000), because the pump serves both directions. An
  endpoint without the hook is always ready and `send()` takes or refuses
  at once, as before.
- **What an endpoint lost (`losses`, 2026-11-18)**: an endpoint may give
  the optional `int (*losses)(QueueHandle_t q, bridge_loss_t *out, int
  cap)`: its own loss counters as `{name, value}` pairs, at most
  `BRIDGE_MANAGER_MAX_LOSSES` (12). The bridge's `send_errors` only counts
  what `send()` refused at the call. What an endpoint takes and loses
  later (a transmit queue of its own, a codec that throws a message away)
  and what it reads and cannot hand to the bridge (the bridge's queue was
  full) stood in the endpoint's own counter, which no route showed: bench
  2026-11-18, a tx message that CAN over MQTT dropped for its length moved
  no number on `/api/bridges`. The names are the endpoint's; the jacks of
  `bridge_endpoints` share `rx_drops` (read, the bridge's queue had no
  room) and `tx_drops` (taken from a bridge, dropped). `q` is the queue
  the asking bridge subscribed with, so an endpoint that fans out reports
  that bridge's share. The hook is called from the web server's task: it
  reads counters and nothing else.
- **Who is at the far end (`peers`, `send_from`, `far_gone`, 2026-11-25,
  GitHub issue #1078)**: three more optional members. `int (*peers)(void)`:
  how many peers the endpoint has now (a TCP server's clients, a WebSocket
  channel's, a Bluetooth central), for an endpoint that can count them.
  `send_from(from, data, len)`: `send()` for an endpoint that wants to know
  where its bytes come from; a raw bridge calls it when it is set, with the
  far endpoint's descriptor address as `from` (an identity, nothing to read
  through). `far_gone(from)`: the far endpoint had peers and has none left.
  Each pump looks about ten times a second (`bridge_manager_peers.c`, pure,
  host-tested) and tells an end once per time the other empties; an end
  that cannot count is never reported. The `obd` jack uses all of it: an
  app that went away in the middle of its monitor kept the poller off the
  OBD chip until a cap of two minutes. A translated bridge calls `send()`:
  no codec sits on the chip's jack.

## Settings (`"bridge_manager"`, version 2)

`bridges`: bounded array (maxItems `BRIDGE_MANAGER_MAX_BRIDGES`, 6) of
`{name, a, b, translator (default "raw"), enabled (default false)}`.
`on_validate` (pure `bm_validate_bridges`) enforces: translator names
registered (a codec is a compile-time registration, so a name nobody
registered is a typo), `a != b`, unique bridge names, and the
**single-consumer rule** (an endpoint may appear in at most ONE enabled
bridge, because two bridges subscribing one endpoint would split its RX
stream between their queues nondeterministically) UNLESS the endpoint
declares `multi_consumer` (2026-07-18): a fan-out provider (obd, obd_chip
copies every RX chunk to every subscriber queue and serializes TX) may
sit in several enabled bridges; a name not in the registry counts as
single-stream. **Endpoint names are NOT required to be registered**
(2026-10-29, the field report in `internal/unable-to-save-config`): the
jacks of socket servers and WebSocket channels register at the NEXT boot
under their configured names, so the Connections card creates a server
and its bridge in ONE submit, and a row left behind by a server renamed
or removed must not refuse every later save of the list (two such rows,
`udp3333` and `udp23`, locked a device out of all its settings pages
with "unknown endpoint" on every Submit, and latched a `boot_errors`
fault at every start). At the BOOT apply the registry-dependent rules
are skipped entirely (capabilities are unknowable before the jacks
register; a single-stream provider refuses the second subscribe at build
time and that bridge degrades alone). Defaults (2026-07-18, legacy
parity): `br_obd = obd <-raw-> ws_obd` (web-UI terminal), `br_tcp_obd =
obd0 <-raw-> obd` (TCP:35000, the classic ELM327-WiFi-adapter convention
192.168.0.10:35000), `br_usb_obd = usb_obd <-raw-> obd` (USB port-B
serial ELM327) and, since version 2 (2026-11-02, Ali: as 4.5x behaved),
`br_ble_obd = obd <-raw-> ble` (the Bluetooth data pipe FFF1/FFF2: on
4.x Bluetooth on WAS the ELM pipe to the chip, and a v6 user who paired an
app got a link that answered nothing until this row existed), ALL enabled
on the obd fan-out. Migration v1 to v2 (`on_migrate`, the pure
`bm_bridges_add_ble_default`, host-tested): a stored list without any row
on the `ble` end gets `br_ble_obd` appended once; a list that already
uses `ble` (a `br_ble_can` row, enabled or not) and a full table are left
alone (one W line for the full table). A user who removes the row again
keeps it removed. The `ble` end carries ONE enabled bridge: a CAN-over-BLE
row disables this one first. A bridge naming an
endpoint or translator its composition never registered degrades ALONE
at start: one **W** line (not E: the health net must not latch a fault
at every boot for a row the user can see and fix), the bridge skipped,
`bridge_manager_reason(name)` keeps the sentence ("endpoint 'udp3333'
does not exist on this device") for `/api/bridges` `reason`; the baked
default must not take other bridges down in endpoint-less compositions;
a refused second subscribe degrades alone too (its own reason, its two
queues and their set released: until 2026-10-29 the set stayed
allocated); any other build failure (OOM) still aborts the start.
Reboot-to-apply per the standard: the spec's "live re-apply" wording is
superseded (spec defers to the standard).

## Dependencies

`settings_manager`, `log_manager` (private); `espressif/cjson` (managed,
private). **No compile-time dependency on any endpoint provider or
translator**: that is the point. Init after settings_manager; start after
the providers' `_start()` (their subscribe/send must be live).

## Memory footprint (estimated: measure before release)

| Where | What | ~Size |
|---|---|---|
| PSRAM `.bss` | 4 bridges × (2 queues × 64 × 130 B + 4 KB stack + 2 × 256 B ctx + 1 KB coalescing buffer) | ~88 KB |
| PSRAM `.bss` | endpoint (8) + translator (4) + config (4) tables | ~1.5 KB |
| Internal `.bss` | queue/task control blocks | ~1 KB |

## Tests

- **Host (`host_test/`, 27 tests)**: the peers at the far end
  (`test_peers.c`, 4 cases, 2026-11-25: the source named for an endpoint
  that asks and `send()` for one that does not, an end that empties
  reported once, a bridge written either way round, an end that cannot
  count never reported); config parse defaults + validation
  (an unregistered endpoint accepted at runtime but still single-stream
  for the rule, unknown translator refused, `raw` always known, a==b,
  single-consumer rule incl. the disabled-bridge exemption, duplicate
  names) and the
  **translator contract** driven by a line-splitter test codec:
  fragmentation reassembly across chunk boundaries, zero/one/many outputs
  per input, ctx isolation between directions. **Green on rpi001
  2026-10-29.**
- **Target (`test_apps/`)**: stub-endpoint pump (ordered + lossless +
  measured ceiling), overflow drop-and-count under a slow consumer, a
  REAL end-to-end leg, TCP client ↔ socket_manager(`tcp0`) ↔ bridge ↔ echo
  endpoint on the lwIP loopback, the runtime PUT rules (`CFG-BAD`,
  `CFG-NEW`), a row refused at start by a single-stream jack already in
  use (`REASON`) and stop/start cycles that prove the refusal leaves no
  queue set behind (`CYCLES`). Last green run: see `test_apps/README.md`.
- **Benchmarks**: `BENCHMARKS.md` (spec §8 scenarios; scripts + methodology
  committed, numbers pending the DUT).

## v1 scope notes

Translators: `raw` only (meatpi 2026-07-03), `translator_slcan` /
`translator_gvret` have one-page specs (`components/translator_slcan/
TASK_translator_slcan.md`, `.../TASK_translator_gvret.md`) and land with the
internal-CAN endpoint. The USB↔OBD passthrough (TASK_obd_chip_manager_new §2)
is a configured `raw` bridge over the `usb_obd` UART endpoint + the `obd`
endpoint: glue registers both, settings enable it.

## HTTP observability (endpoint reference: `components/HTTP_API.md` §6e3)

`GET /api/bridges` (2026-07-05, served by the `api_http` glue: this component
stays transport-free): configured entries from this component's settings
+ live counters from `bridge_manager_stats()`; `up:false` = configured but not
running, with `reason` (2026-10-29, `bridge_manager_reason()`) when an
enabled bridge did not build at the last start: the endpoint or translator
its row names does not exist on this device, or an endpoint is already used
by another bridge. The bench: `.\test.ps1 connections` (`CONNECTIONS PASS`).
Since 2026-11-18 a running bridge also carries `losses`: `{"a":{...},
"b":{...}}`, what the endpoint at each end lost, by the names it reports
(`bridge_manager_losses()`); an end whose endpoint reports nothing is left
out. The bench: the firmware repo's `tools/testbench/can/
mqtt_can_batch_bench.py`, leg C (a tx message dropped for its length shows
as `losses.b.tx_oversize` on the `mqtt0` end, and every running bridge
reports both its ends).
