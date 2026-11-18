# translator_slcan

The Lawicel **slcan** ASCII CAN framing codec, as a `bridge_manager`
translator (`"slcan"`). Bridges the internal `can` endpoint to a byte stream
(TCP/USB/WS), e.g. for `python-can`'s `slcan` interface.

- `decode` (can→client): CAN-wire chunk → `tIIIL DD..\r` / `TIIIIIIIIL..\r`
  (`t/T`=data 11/29-bit, `r/R`=RTR).
- `encode` (client→can): `t/T/r/R` lines (reassembled across chunks in the
  ctx) → CAN frames. Channel commands (`O/C/S/Z/M/m/V/N/F`) are **absorbed**:
  the bus is owned by `can_manager` settings; `python-can` needs no replies.
  A line is 47 characters at the most (the longest frame line has 26).

## What the codec refuses (2026-11-18)

A chunk that carried input the codec dropped makes `decode` / `encode`
return `ESP_ERR_INVALID_ARG`, and the pump counts that call in the bridge's
`codec_errors` (`GET /api/bridges`, one count per chunk, however many lines
of it were refused). Until that day every refusal returned `ESP_OK` and the
counter stayed 0 whatever arrived. Only the refused part is dropped: the
lines in front of it and behind it in the same chunk still reach the bus.

| Refused | Example |
|---|---|
| a frame line that does not parse (too short, id or data not hex, dlc above 8, data shorter than its dlc) | `t10`, `tZZ01AA`, `t1009AA`, `t1002AA` |
| a line that does not begin with a letter (no Lawicel command does) | bytes of another protocol |
| a line longer than 47 characters: dropped whole, up to its CR | 48 characters, then `t1001AA`, then CR |
| `decode`: bytes that are no wire frame (side `a` is not the `can` jack) | a tail cut short, a dlc above 8 |

Not refused: every line that begins with a letter other than `t/T/r/R` is
a channel command and is absorbed, whatever the letter (the dialects
differ), and so is an empty line.

**The over-long line.** Until 2026-11-18 the codec forgot a line at its
48th character and read what followed as a line of its own: 48 characters
and then `t1001AA` before the CR put frame 0x100 on the bus though the
client never wrote that line. The whole line is now dropped up to its CR,
every chunk that carried a dropped byte of it is counted, and the line
behind the CR is a line again.

Wire format shared with the `can` endpoint: `can_manager/include/can_frame_wire.h`.
Design: `bridge_manager/DESIGN_translators.md §4`. Registered by
`translator_slcan_init()` (main's init pass).

## Use
Configure a bridge + socket (web UI / API), then any slcan client connects:
```
socket_manager.servers += {name:"slcan0", proto:"tcp", port:3333}
bridge_manager.bridges += {name:"br_slcan", a:"can", b:"slcan0", translator:"slcan"}
```
`a` MUST be the CAN endpoint (a→b = decode). Enable the CAN bus first.

## Tests
- Host (18 tests): `./test.ps1 host translator_slcan`, encode/decode
  round-trips (11/29/RTR/dlc), byte-fragmentation, multi-frame, malformed,
  control absorb, the refusals reported (a bad line among good ones, a
  malformed wire chunk), the over-long line whole and byte by byte.
- Bench: `.\test.ps1 slcan` (= `tools/testbench/can/slcan_bridge_test.py`,
  python-can + PCAN both directions, the /ws/can monitor beside the TCP
  client, the Linux clients on rpi001, and first the trap: a server with
  no connection row) → **SLCAN BRIDGE PASS**. Throughput:
  `slcan_perf_test.py`.
