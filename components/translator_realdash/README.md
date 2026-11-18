# translator_realdash

The **RealDash** CAN framing codec, as a `bridge_manager` translator
(`"realdash"`). Feeds the RealDash app a CAN stream and accepts its frames.

- `decode` (can→app): CAN-wire chunk → RealDash **66** frame
  (`66 33 22 11 <id:4 LE> <data:8> <crc32:4>`, 20 bytes).
- `encode` (app→can): RealDash **44** (`44..`, chksum8, 17 B) **or** **66**
  (`66..`, CRC32, 20 B) frames, flavor auto-detected from the header byte,
  reassembled in the ctx, checksum-validated. 29-bit id when `id & 0x1FFFF800`.
- Stateless per frame → no reply channel. CRC32/chksum8 ported verbatim from
  the legacy WiCAN impl.

## What the codec refuses (2026-11-18)

A chunk that carried input the codec dropped makes `decode` / `encode`
return `ESP_ERR_INVALID_ARG`, and the pump counts that call in the bridge's
`codec_errors` (`GET /api/bridges`, one count per chunk). Until that day
every refusal returned `ESP_OK` and the counter stayed 0 whatever arrived.
Only the refused bytes are dropped: the frames around them in the same
chunk still reach the bus.

| Refused | Example |
|---|---|
| a byte outside a frame that is neither `66` nor `44` | noise, another protocol |
| a header that does not go on with `33 22 11` | `66 33 99` |
| a 66 frame whose CRC32 is wrong, a 44 frame whose checksum is wrong | a corrupted frame |
| `decode`: bytes that are no wire frame (side `a` is not the `can` jack) | a tail cut short, a dlc above 8 |

Design: `bridge_manager/DESIGN_translators.md §7`. Registered by
`translator_realdash_init()`.

## Use
```
bridge_manager.bridges += {name:"br_rd", a:"can", b:"<tcp/ws endpoint>", translator:"realdash"}
```
`a` = CAN endpoint. Point RealDash at the socket as a "CAN/serial" custom
connection.

## Tests
- Host (9 tests): `./test.ps1 host translator_realdash`, 66/44 round-trip,
  CRC32 & chksum8 reject, byte-fragmentation, resync after garbage, 29-bit
  id, the refusals reported (noise, a bad header, a bad CRC32 or checksum, a
  good frame behind a bad one, a malformed wire chunk).
- Bench: RealDash app ↔ `can↔realdash↔tcp` ↔ PCAN (manual; app required).
