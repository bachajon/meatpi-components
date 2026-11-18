# tx_backlog: host unit suite

Compiles ONLY the pure part (`tx_backlog.c`: the ring and the pool, no lwIP)
on the IDF **linux** target. Runs on the bench Pi via `.\test.ps1 host
tx_backlog`. The socket push (`tx_backlog_sock.c`) is proven on the bench:
legs S and W of `.\test.ps1 slcan` in the firmware repo.

## What is covered (8 tests)

- The ring keeps the order around the end of its storage, through a partial
  send, and starts again from the front once it is empty.
- 20,000 chunks of random length against random partial sends: nothing lost,
  nothing twice.
- A unit goes in whole or not at all (a text line is never cut), in one part
  and in two (a frame's header with its payload), also around the end.
- The stall clock: starts when nothing moves and the socket refuses, stops
  at the first byte or at a send taken whole, a new client starts clean.
- The pool lends each buffer once, gives nothing when empty or without
  storage, and takes a buffer back to lend it again.

## Expected result

```
8 Tests 0 Failures 0 Ignored
OK
```

Last verified green: 2026-11-18 on rpi001.
