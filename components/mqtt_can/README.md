# mqtt_can: CAN ⇄ MQTT bridge building blocks

Legacy WiCAN's raw-frames-over-MQTT, rebuilt as bridge parts (meatpi
2026-07-22: "a bridge with configurable tx and Rx topics … both tx and
Rx safety gate"): the **`mqtt0` bridge endpoint** + the **`canmqtt`
translator** (legacy JSON, batched). Compose with a normal bridge row:

```jsonc
// bridge_manager settings
{"name":"br_mqtt_can", "a":"can", "b":"mqtt0", "translator":"canmqtt"}
```

Design + rulings + test plan: `TASK_mqtt_can.md`.

## Wire contract (byte parity with legacy `main/mqtt.c`)

- publish (device→broker), default topic `~/can/rx`
  (`~` = mqtt_manager's topic prefix, `wican/<device_id>`):
  `{"bus":"0","type":"rx","ts":34610,"frame":[{"id":123,"dlc":8,
  "rtr":false,"extd":false,"data":[1,…,8]},…]}`, frames batched.
- subscribe (broker→device), default `~/can/tx`: same shape,
  `"type":"tx"`; `ts` tolerated, `bus` number or string, `extd`
  implied for id > 0x7FF.

## Settings (`mqtt_can`, v1)

| key | default | notes |
|---|---|---|
| `enabled` | false | registers mqtt0 + canmqtt (reboot-to-apply) |
| `pub_topic` / `sub_topic` | `~/can/rx` / `~/can/tx` | fully configurable; `~` expands to the mqtt prefix |
| `qos` | 0 | publish QoS |
| `allow_rx` | **false** | GATE: bus traffic may leave the device |
| `allow_tx` | **false** | GATE: broker frames may reach the bus |
| `batch_ms` | 50 | max frame age before a batch flushes (pump flush hook) |
| `batch_frames` | 24 | flush at this many frames (cap `MC_BATCH_MAX`) |
| `filter`/`mask`/`filter_ext` | "" | hex; maps 1:1 onto the `can` jack's RX filter (`bep_can_set_filter`); empty mask = all |
| `cli` | true | `mqtt_can` stats command |

**Both gates default OFF** (project gate convention): an enabled
bridge moves NOTHING until each direction is explicitly consented.
Gate-blocked traffic is counted (`mqtt_can` CLI: rx-gate / tx-gate),
never silent.

## Mechanics

- RX: the `can` jack's chunk coalescing + the translator's in-ctx
  accumulator → one legacy-JSON batch per `mqtt_manager_publish_async`
  (never blocks the pump; broker-down drops are counted by
  mqtt_manager). Batches flush on `batch_frames`, or `batch_ms` via
  the bridge pump's translator-flush hook (added for this codec;
  `bridge_manager.h`). `batch_frames` is looked at when a chunk is in:
  the frames of one chunk stay in one publish, so a batch may carry up
  to a chunk more than `batch_frames`, never more than 24.
  **A chunk past a full batch (2026-11-18):** the `can` jack packs the
  frames that wait into one chunk (7 frames of 8 data bytes), so a
  chunk often arrives when the batch has room for a part of it only.
  The batch then goes out and the rest of the chunk starts the next one
  (`mc_batch_feed`). Until that day the rest was dropped: with the
  default `batch_frames` 24 the fourth chunk lost 4 of its 7 frames, 4
  of every 28 on a busy bus, counted in a counter nothing read.
- TX: one MQTT message → bridge chunks → the translator's stream
  reassembler (fragment-safe, brace/string-aware, oversize counted)
  → parse → frames onto the bus. **Every frame of a message**
  (2026-11-18): a message is one object of 1024 bytes at the most
  (`MC_TXBUF_MAX`: some 55 frames without data, 10 to 17 with 8 data
  bytes) and all its frames go to the bus, in order, or none when one of them
  is malformed (`parse-err`). Until that day the first 8 went and the
  rest was dropped without a count, and a malformed frame behind the
  ninth was not even looked at.
  **A message cut short (2026-11-18):** an object that never closes (a
  publisher's mistake, the tail of a message lost to a full bridge
  queue) is dropped when the next message begins, and counted
  (`tx-cut`). Until that day the reassembler waited for the closing
  brace for ever, took every later message for more of the cut one,
  and nothing reached the bus again before a restart. How it knows a
  new message: `mqtt0` cuts a message into chunks so that only its
  FIRST chunk can begin with `{` (`mc_tx_chunk_len` ends a chunk one
  byte early when the next would begin with one; bytes and order are
  untouched, a raw bridge on `mqtt0` sees the same stream), and a chunk
  that begins with `{` while an object is open is a new message
  (`mc_tx_feed`). So `canmqtt`'s broker side wants a message-framed
  source like `mqtt0`: a byte stream cut at random (a TCP socket) can
  begin a chunk with the `{` of a frame object and loses that message.
  A message that begins with something else (white space) is taken as
  before but does not end a cut one in front of it.
- The whole codec state lives in the translator ctx (`BM_CTX_MAX`
  4096): nothing big on the 4 KB pump stack (stack-audit rule).
- `can` jack is **multi_consumer since 2026-07-26** (the v2 backlog
  item): slcan-on-can and mqtt-on-can run SIMULTANEOUSLY, the jack's
  pump fans each RX chunk out to every subscribed bridge (up to 4).
  Caveat: the jack has ONE ingress filter (this component's `filter`
  setting via `bep_can_set_filter`), with several consumers attached
  it restricts what ALL of them see. Verified live: CANFAN BENCH.

## Counters (`mqtt_can_stats()`, the `mqtt_can` console command)

| Console | `mqtt_can_stats_t` | Counts |
|---|---|---|
| `frames rx` (`batches`) | `frames_rx`, `batches` | bus frames batched toward the broker, publishes handed to mqtt_manager |
| `tx` | `frames_tx` | broker frames handed to the bus side |
| `rx-gate` / `tx-gate` | `dropped_rx_gate` / `dropped_tx_gate` | publishes / messages refused by a closed gate |
| `q-full` | `dropped_q_full` | messages that lost their tail to a full bridge queue |
| `parse-err` | `parse_errors` | tx objects refused: no `"type":"tx"`, no `frame` array, a malformed frame |
| `tx-cut` | `tx_cut` | tx objects dropped because a new message began before they closed (2026-11-18) |
| `tx-oversize` | `tx_oversize` | tx objects above 1024 bytes (counted by the codec all along, shown since 2026-11-18) |
| `rx-batch` | `dropped_rx_frames` | bus frames of a batch that did not fit its JSON buffer (0 by sizing, the host suite proves the worst case) |
| `pub-refused` | `pub_refused` | rx messages mqtt_manager did not take: its queue was full, or no broker (2026-11-18; one message is one batch of frames) |

The last four are a line of their own (`lost: ...`), the first two
lines are as they were (the bench reads them by their shape). Since
2026-11-18 every loss counter of this table is also on `GET /api/bridges`,
by its `mqtt_can_stats_t` name, as `losses` on the end of the bridge that
uses `mqtt0` (the jack's `losses` hook, bridge_manager). Two more stand
beside them: `mqtt_dropped_full` and `mqtt_dropped_offline`, the MQTT
client's own drops (`mqtt_manager_stats()`: its queue had no room; the
broker was away when a queued message's turn came). They count everything
the device publishes, not this bridge alone, and no route showed them: a
message this jack handed over and the client then dropped was lost with
every other number at rest.

**One MQTT message is one JSON object (2026-11-18).** bridge_manager
gathers what a translator emits during one pass of its pump into one
`send()` (2026-11-17). With a small `batch_frames` on a busy bus the pump
finishes several batches in one pass, and the endpoint published them as
they came: one message with two or more rx objects back to back, which no
JSON parser takes (bench, `batch_frames` 1 at 1,000 frames a second: 115
of 4,878 messages; `batch_frames` 4 at 2,000: 1 of 1,522; the default 24
never, a batch of 24 frames is longer than what the pump gathers). The
endpoint now publishes each top-level object of a `send()` as a message of
its own (`mc_pub_each`, pure, host-tested); bytes that are no JSON object
(a raw bridge on `mqtt0`) go out as they came.

## Memory

PSRAM: none beyond the shared bridge ctx slots. Internal: statics
< 1 KB (topics + stats). No task of its own: rides the bridge pump
and the esp-mqtt event task (handler only queues chunks). The tx parse
no longer keeps 8 frames (160 bytes) on the pump's stack: a frame at a
time (2026-11-18).

## Tests

Host `host_test` 24/24 (`test_publish.c`, 2026-11-18, what the endpoint
publishes: two batches of one pump pass are two messages, one object is
untouched, braces inside strings do not cut, bytes that are no object go
out as they are, a refused publish does not hold the rest; and as before:
legacy-exact strings incl. both documented
legacy examples, worst-case batch sizing vs `MC_JSON_MAX`, fragmented/
noisy/oversized tx streams, parse rejects; `test_steps.c`, the two
steps as the bridge runs them: a chunk past a full batch at
`batch_frames` 24 and 20, a chunk in one publish, a batch that does not
fit counted, four kinds of cut message each followed by 20 good ones,
whole messages at every chunk alignment, 12 frames in one message and
as many as the reassembly buffer holds, a malformed eleventh frame).
Live:
`tools/testbench/mqtt_can_bench_test.py` (→ `MQTT CAN BENCH PASS`),
gates-closed leg FIRST, conservation-checked RX at rate, byte-exact TX
round-trip via PCAN, custom-topics leg; and
`tools/testbench/can/mqtt_can_batch_bench.py` (→ `MQTT CAN BATCH PASS`,
2026-11-18): small batches on a busy bus, every message one JSON object,
every frame once and in order, a dropped tx message on `/api/bridges`.
