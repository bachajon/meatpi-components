# tx_backlog

## Summary

What a stream client has not taken yet. A sender that writes to a TCP socket
for a client it does not control (a phone on WiFi, a browser tab) must not
wait for that client and must not close it for being slow for a moment.
This component is the three pieces that make that possible, shared by every
sender that needs them:

- **a ring** per client that takes a unit (a chunk, a frame with its header)
  whole or not at all, hands it back in order and keeps a stall clock;
- **a pool** that lends the rings their storage for as long as a client is
  behind, so the memory is the number of clients that may be behind at the
  same time and not the number of clients;
- **a socket push** that never blocks: what waits goes first, then the new
  unit, what the socket does not take waits, what no longer fits is dropped
  and counted.

It sits below `socket_manager` (the TCP servers) and `websocket_manager`
(the WebSocket channels) and depends on lwIP only. It has no task, no lock,
no settings and no log lines: its owner calls it under the owner's lock and
says in its own log what happened to a client.

History: written inside `socket_manager` on 2026-11-17 after a field report
(slcan over TCP closed on a phone about once a minute while driving: a
client whose send did not finish in 100 ms was closed), moved here on
2026-11-18 when the WebSocket channels turned out to need the same (one
client that stopped reading held the bridge for the server's 5 s send
timeout and cost the other clients of the channel a third of their frames).

## The pace (2026-11-18)

A stream of small units sent unit by unit is a TCP segment and a WiFi frame for each. With a
CAN bus at 4,200 frames a second bridged to one slcan client the unit sent about 950 segments
a second (one per pass of the bridge's pump, 60 to 240 bytes each), the TCP/IP task used a
whole CPU core for them and their acknowledgements, and a plain web request took 0.15 to 0.5 s
where it takes 25 ms, with single answers of 2 to 6 s.

So a pool can carry a pace (`pace_us`, `pace_after`; 0 = none, the default). A client that
gets units closer together than the gap is a stream: after `pace_after` such units in a row a
new unit waits in the client's backlog until the gap since the last send is over. Then the
next unit takes what has gathered with it, in one send, or the owner's next push does when no
unit comes. A unit that arrives after a pause of the gap leaves at once, and so do the next
`pace_after`: a request and its answer in a few pieces are never held. A held unit is not a
stall (the stall clock is left alone), and with no buffer free in the pool a unit is not held.
The owner makes its pushes with no new unit on a tick about as long as the gap for as long as
something waits (`socket_manager`: gap 2.5 ms, a task of its own on a 3 ms tick;
`websocket_manager`: gap 4 ms on its 5 ms flush tick, control frames leave at once through
`tx_backlog_pace_now`).

Measured on the bench with the `canmax` stage (a full bus at 500 and 1000 kbit/s to one slcan
client, the same six tiers): 55,229 segments of 132 bytes before, 24,585 of 307 bytes after;
the TCP/IP task from about a whole core to about a third of one. Two things were wrong on the
way and are why the rule reads as it does: a pause is measured from the last UNIT, not from the
last send (the pace itself stretches the time between sends, and a rule that looked at the
send let most of a stream through unpaced), and a unit that follows waiting bytes joins them
before the send (as two sends they were two segments for every release).

## API (`include/tx_backlog.h`)

| Function | What it does |
|---|---|
| `tx_backlog_clear(b)` | empty the ring, stop the stall clock, zero `dropped` |
| `tx_backlog_put(b, data, len)` | queue a unit behind what waits, whole or not at all |
| `tx_backlog_put2(b, head, head_len, data, len)` | the same for a unit in two parts (a frame's header and payload) |
| `tx_backlog_peek(b, &p)` / `tx_backlog_took(b, n)` | the first run of waiting bytes; the client took `n` of them |
| `tx_backlog_attempt(b, moved, refused, now_us)` | one send attempt's result: nothing moved and refused starts the stall clock |
| `tx_backlog_expired(b, now_us, limit_ms)` | the client moved no byte for `limit_ms` |
| `tx_backlog_pace_hold(b, now_us, gap_us, after)` | a new unit: true = it waits for the gap to be over (notes the unit, counts the run) |
| `tx_backlog_pace_due(b, now_us, gap_us)` | a push with no new unit: true = what waits may leave |
| `tx_backlog_pace_now(b)` | the next unit leaves at once, and what waits with it (a frame that ends a connection) |
| `tx_backlog_lend(pool, b)` / `tx_backlog_return(pool, b)` | a buffer from the owner's pool, and back |
| `tx_backlog_sock_push(fd, flags, b, pool, head, head_len, data, len, now_us, &bytes)` | the push: `TX_BACKLOG_SENT`, `_DROPPED` (the new unit dropped whole), `_CUT` (a part was on the wire and the rest found no buffer), `_ERROR` (a hard socket error) |

`flags` goes to `send()`: `0` for a socket that is non-blocking itself (the
TCP servers), `MSG_DONTWAIT` for one that must stay blocking for its other
user (a WebSocket session, which the web server reads).

The owner decides what a result means. `socket_manager` counts `_DROPPED`
and `_CUT` in `tx_drops` and keeps the client (a line protocol resyncs at
the next line end). `websocket_manager` closes a client on `_CUT`, because a
frame cut on the wire leaves a stream that is no longer WebSocket.

## Dependencies

`lwip` (private, for `tx_backlog_sock.c`). No init, no order: plain
functions on the owner's data.

## Settings

None. The size of a buffer and the number of buffers are the owner's
build-time choice (64 KB and four for both owners today): external RAM is
cheap, so they are sized with room and are not settings.

## Memory footprint

Nothing of its own. The owner declares the pool's storage as a static array
in PSRAM (`EXT_RAM_BSS_ATTR`) and one `tx_backlog_t` (64 bytes) per client
slot.

## Tests

- **Host (`host_test/`, 10 tests)**: the pace holds a stream and not an
  exchange (150 units 200 us apart leave in 10 sends, a unit after a pause
  leaves at once, a late send does not end the stream), what waits is due
  when the gap is over; the ring keeps the order around the end
  of its storage; many laps lose and repeat nothing; a unit goes in whole or
  not at all, in one part and in two; the stall clock; the pool lends each
  buffer once, gives nothing when empty and takes a buffer back.
- **Bench (firmware repo, `.\test.ps1 slcan`)**: leg S (a TCP client whose
  ACKs come late) and leg W (a WebSocket client that takes nothing or takes
  slowly), both on the real WiFi path with 1,100 CAN frames a second.
