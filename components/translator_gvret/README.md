# translator_gvret

The **SavvyCAN GVRET** binary CAN codec, as a `bridge_manager` translator
(`"gvret"`). Lets SavvyCAN connect to WiCAN's CAN bus over TCP/USB/WS.

- `decode` (can→SavvyCAN): CAN-wire chunk → GVRET frame record (ts = the frame's RX-interrupt time since 2026-09-21; the 1 ms synthetic clock only for frames without one)
  `F1 00 <ts:4> <id:4, bit31=ext> <dlc> <data> <xor-chk>`.
- `encode` (SavvyCAN→can): the F1-prefixed command stream (reassembled in the
  ctx) → CAN TX frames (BUILD_CAN_FRAME) **plus handshake replies**
  (TIME_SYNC / GET_CANBUS_PARAMS / GET_DEV_INFO / KEEPALIVE / GET_NUMBUSES /
  …) sent back to the client via the bridge **reply channel** (`wants_reply`).
  SETUP_CANBUS is parsed + absorbed (the bus is owned by `can_manager`).

## TIME_SYNC (2026-11-18)

`F1 01` is answered with `F1 01 <now:4 LE>`: the low 32 bits of the
microseconds since boot, the clock the frame records carry (`ts_us`, taken
by `can_manager` in the CAN RX interrupt), so SavvyCAN, which lines its own
clock up with this answer, reads the records on the same time line. Until
that day the answer was 0 at every request: it came from a counter of the
encode ctx that only a decode ctx advances. The codec stays pure: the clock
is a hook (`translator_gvret_private.h`), `translator_gvret_init()` points
it at `esp_timer` (a private dependency of the component since then), the
host test at a stub.

## What the codec refuses (2026-11-18)

A chunk that carried input the codec dropped makes `decode` / `encode`
return `ESP_ERR_INVALID_ARG`, and the pump counts that call in the bridge's
`codec_errors` (`GET /api/bridges`, one count per chunk). Until that day
every refusal returned `ESP_OK` and the counter stayed 0 whatever arrived.
Only the refused bytes are dropped: the commands around them in the same
chunk still run.

| Refused | Example |
|---|---|
| a byte outside a command other than `F1`, `E7` and `00` | `ATZ` from another protocol's client |
| `F1` and a byte that is no GVRET command | `F1 7F` |
| `decode`: bytes that are no wire frame (side `a` is not the `can` jack) | a tail cut short, a dlc above 8 |

Not refused: `E7` (binary mode, SavvyCAN sends it twice on connect), `00`
outside a command (SavvyCAN ends SETUP_CANBUS and SET_EXT_BUSES with a
zero where a checksum would be, and the 8 / 12 bytes this parser takes
leave it over), every GVRET command, answered or absorbed, and a
BUILD_CAN_FRAME whatever its checksum byte (SavvyCAN sends 0). What a real
SavvyCAN session sends outside a command is known from reading its source,
not from a capture (see Bench below): should a SavvyCAN session raise
`codec_errors`, the idle rule of `gvret_encode` is where to look first.

The **reply channel** (`bridge_reply_hdr_t`, `bridge_manager.h`) is the clean,
opt-in mechanism for protocols that must answer the client on the near side;
the pump fills it after `ctx_init`. Design: `DESIGN_translators.md §5/§6`.
Registered by `translator_gvret_init()`.

## Use
```
bridge_manager.bridges += {name:"br_gvret", a:"can", b:"gvret0", translator:"gvret"}
```
`a` = CAN endpoint. SavvyCAN → "Connect" → GVRET over TCP to the socket.

## Tests
- Host (12 tests): `./test.ps1 host translator_gvret`, decode records
  (checksum, ext bit31), encode BUILD_CAN_FRAME (+ fragmentation), handshake
  replies via the reply channel, the TIME_SYNC answer on the clock of the
  frame records, the refusals reported (bytes of another protocol, an
  unknown command, a frame behind a stray byte, a malformed wire chunk).
- Bench (pending SavvyCAN): capture a legacy TCP:23 session as
  `fixtures/savvycan_session.bin`, replay in the host test, then SavvyCAN ↔
  `can↔gvret↔tcp` ↔ PCAN live. Confirm the GVRET dialect first (TASK §Mission).
