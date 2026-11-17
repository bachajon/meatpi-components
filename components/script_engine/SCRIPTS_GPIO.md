# Paste into the scripts page (bindings + limits)

Bindings to add:

| Group | Binding | What |
| --- | --- | --- |
| Raw CAN | `can_start()`, `can_stop()` | start or park the CAN controller and its transceiver. `can_stop()` frees `CAN_TX`. The bus is started again when the script ends. `can_start()` is false when CAN is disabled in settings. |
| Raw CAN | `can_rx(id, timeout_ms[, ext])` | one frame with that id, as a hex string, or `nil`. `timeout_ms` is milliseconds (`1000` = 1 s, at most 60 s). Sets `can_rx_id`, `can_rx_ext`, `can_rx_dlc`. |
| GPIO | `digitalWrite(pin, HIGH\|LOW)` | drive `CAN_TX`. `HIGH` is 1 (recessive), `LOW` is 0 (dominant). |
| GPIO | `digitalPWM(pin, ontime_ms, offtime_ms[, rounds[, idle]])` | pulse `CAN_TX`. The pin idles at `idle` (`HIGH` or `LOW`) and `ontime` is the opposite level. The default idle on `CAN_TX` is `HIGH` (recessive, so the transceiver's dominant timeout never trips; `ontime` is a dominant bit). The transceiver is taken out of standby when `CAN_TX` is driven. Times are milliseconds and may be fractional (`1.6`, `0.4`). `rounds` defaults to 1. |

Limits to add:

| One `can_rx` wait | up to 60 s |
| One `digitalPWM` | `rounds * (ontime + offtime)` up to 1 s, at most 49104 rounds |
| `digitalPWM` timing on `CAN_TX` | steps of 1 us; the first edge is the on-level |
| Pins | `CAN_TX` only, and only while the bus is stopped (`can_stop()`). The pin goes back when the script ends. |

Why `CAN_TX` only (WiCAN PRO): every other GPIO of the module carries a function of the device (the OBD chip's lines, the card, I2C, USB, the battery sense, the LED) and none is on a header, so a script that drove one would take that function down until a restart. `CAN_RX` is the transceiver's output and cannot be driven.

On `CAN_TX`, `digitalPWM` times the levels with a busy loop, so that the first thing that drives the pin is the on-level; the call waits until the pulses have been sent and other work of the device can stretch a level by a few microseconds. The code also holds an RMT sender (hardware-timed edges, 0.1 us steps up to 3.2 ms, more than 1023 rounds as a block of 48 pulses repeated by the hardware) for a board that has free pins; it is not reached on this one.
