# bridge_endpoints

The adapter layer between providers and `bridge_manager`: every data
interface wrapped as a named JACK (bridge endpoint). Providers stay
bridge-agnostic; this component knows both sides: it is the one
component that may depend on bridge_manager AND the providers
(ARCHITECTURE §10; formerly `main/main_endpoints.c` glue).

| Jack | Wraps | Registered |
|---|---|---|
| `obd` | `obd_chip` (MIC3624; **multi_consumer**: the chip's RX fan-out copies every chunk to every subscriber, TX serialized, so obd may sit in several enabled bridges: TCP+USB+WS defaults) | init (fixed) |
| `can` | `can_manager`: frame⇄chunk pump task, owned here (**multi_consumer** since 2026-07-26: up to 4 bridges at once, slcan + mqtt_can together; RX chunks fan out to every subscriber, TX serializes via `can_manager_send`; ONE jack-wide ingress filter `bep_can_set_filter`, per-consumer filters out of scope. Since 2026-11-17 (pull request 2, Elias Kotlyar): one `send()` may carry several wire frames, each is transmitted; since 2026-11-18 `send()` only puts the frames into the jack's TX queue (1024 frames) and a task of the jack (`can_bridge_tx`) hands them to the controller, waiting for room first (`can_manager_tx_room`, up to 100 ms a frame), so a client's burst leaves at the bus's pace where the frame behind the 16 in flight used to be refused at once and lost with the rest of its batch. The jack's `tx_ready` tells the bridge pump when the queue has no room for one more pass from each of its bridges: the pump leaves the client's chunks queued, the client's TCP holds it back, and the pump keeps serving the other direction (a wait for the bus inside `send()` lost the client 162 frames of a bus streaming 1100 frames a second to it, bench 2026-11-18). A line that frees no slot for 100 ms gets the next second's frames without a wait (one W line; the controller refuses them and they are dropped), so a dead bus cannot hold a client for ever; a node that may not talk (listen-only, stopped) refuses in `send()` at once, as before; and the jack joins the bus again by itself when it comes back after a stop (`can_manager_on_bus_up`: a script's `can_stop()` / `can_start()`), so its bridges keep receiving) | init (fixed) |
| `ble` | `ble_manager` GATT pipe | init (fixed) |
| `cli` | `cmdline_manager` | init (fixed) |
| `usb_obd` | USB port-B UART (UART2 TX17/RX18 @2 Mbaud), owned here | init (fixed) |
| socket servers | `socket_manager`: one jack PER CONFIGURED server, under its **configured name** (defaults `obd0 slcan0 gvret0 udp0`) | start (dynamic) |
| WS channels | `websocket_manager`: one jack per configured channel (defaults `ws_obd ws_can ws_cli ws_log`; slots for all 6 channels since wsm v2) | start (dynamic) |

**What each jack lost (2026-11-18).** Every jack gives bridge_manager's
`losses` hook, so `/api/bridges` shows, on each end of a running bridge,
the counters that stood in a log line or nowhere until then:

| Jack | Names | Counts |
|---|---|---|
| every jack | `rx_drops` | chunks the jack read that the bridge's queue had no room for (`obd` and `can`: this bridge's share of the fan-out) |
| `ble`, `cli`, socket servers, WS channels | `tx_drops` | chunks a bridge handed over that the jack dropped: the Bluetooth transmit queue or the console's input queue was full, a TCP or WebSocket client was behind by more than its buffer |
| `can` | `tx_drops`, `tx_refused`, `tx_malformed` | frames that found the jack's TX queue full; frames the controller did not take (a bus that took nothing for 100 ms, a bus-off); chunks that were not whole frames |
| `usb_obd` | `rx_overruns` | times the UART lost bytes from the PC: its FIFO or the driver's ring ran over |

The `usb_obd` reader (2026-11-18) waits for the UART driver's events and
reads what the driver holds. It used to ask for 128 bytes with a 20 ms
limit, which the driver waits out in full behind every arrival: a command
of a few bytes left the jack 20 ms late. The bytes the UART loses while
the unit writes its flash (the UART interrupt waits for the write, the
FIFO is 128 bytes, 0.64 ms at 2 Mbaud) are counted now (`rx_overruns`, one
W line per hundred).

Add-on component packs may register further jacks straight with
bridge_manager during their ext init hook (same pre-start window).
Dynamic socket/WS jacks follow the settings names, renaming a server
renames its jack; a name colliding with an existing jack is refused by
the registry and logged (that jack alone degrades).

## API

- `bridge_endpoints_init()`: log descriptor + the 5 fixed jacks.
- `bridge_endpoints_start()`: dynamic socket/WS jacks (reads the applied
  settings names) + USB port-B UART bring-up.
- `bridge_endpoints_stop()`: no-op (jacks live as long as the registry).

## Dependencies / ordering

`bridge_manager`, `obd_chip`, `ble_manager`, `can_manager` (+
`can_core` frame type), `cmdline_manager`,
`socket_manager`, `websocket_manager`, `log_manager`, `esp_driver_uart`:
all private. main composes: `init` after `bridge_manager_init`; `start`
after the settings boot pass and BEFORE `bridge_manager_start`
(endpoint registration is pre-start only).

## Settings

None. Which jacks carry traffic is bridge_manager's `bridges[]` config;
the socket/WS jack NAMES come from those components' own settings.

## Memory footprint (estimated: measure before release)

| Where | What | ~Size |
|---|---|---|
| Internal `.bss` | USB RX task stack (3 KB, UART driver path §2) + TCBs | ~3.5 KB |
| Internal heap | UART2 driver rings (4 KB + 4 KB) and, since 2026-11-18, its event queue (32 events) at start | ~8.5 KB |
| PSRAM `.bss` | CAN pump stack (3 KB) + name buffers | ~3.2 KB |
| PSRAM `.bss` | the `can` jack's TX queue (1024 frames of 20 B) + its task's stack (3 KB) | ~23.5 KB |
| Internal `.bss` | that queue's and task's control blocks | ~0.5 KB |
| PSRAM `.bss` | the `can` jack's RX queue (256 frames since 2026-11-18, was 32 on the internal heap: a flash erase of 23 ms on a bus above 1400 frames a second ran it over) | ~5 KB |

## Tests

Glue: no host suite (thin wrappers + two FreeRTOS adapters with no
isolatable pure logic; the frame⇄chunk codec it uses is host-tested in
`can_manager`'s `can_frame_wire`). Covered live by the bridge benches:
translator suites (slcan/gvret TCP+WS), the USB passthrough leg, and
since 2026-11-18 the firmware repo's `tools/testbench/usb/
usb_obd_pipe_bench.py` (`USB OBD PIPE PASS`: the `usb_obd` jack bridged
raw to a TCP server, a short write's delay, every byte of a stream, the
bytes lost during a flash write in `rx_overruns`) and leg C of
`tools/testbench/can/mqtt_can_batch_bench.py` (every running bridge
reports the losses of both its ends).
