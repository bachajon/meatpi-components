# esp_isotp host tests

From the firmware checkout: `.\test.ps1 host esp_isotp`.
Expected: `5 Tests 0 Failures 0 Ignored`.

The suite builds the vendored link, `isotp-c/isotp.c`, the way the
component builds it (`src/isotp_config.h` forced in front, with the
firmware's Kconfig values from `main/isotp_test_kconfig.h`) and nothing
else: `src/esp_isotp.c` needs the TWAI driver and does not build for the
host. It guards the local changes `PROVENANCE.md` lists against the next
one and against a new copy from upstream.

Two links are wired back to back and every frame either one sends is
checked against ISO 15765-2, beside the bytes that arrive:

- **Normal addressing, every size.** Each message size from 1 to 4095
  bytes, both directions, with one flow control for the whole message
  (BS 0) and with one every eight frames. The frames of the whole sweep
  hash to a pinned value: the one the library gave before extended
  addressing was added (taken 2026-11-20 from the copy on main at eb15b34
  and from the changed one, the same on x86-64 and on ARM64). A change
  that moves one byte of a normal-addressing frame fails here.
- **Extended addressing, every size.** The same sweep with an address
  extension in front of every frame, a different one in each direction:
  single frames hold 6 bytes, first frames 5, consecutive frames 6, and
  the flow control is led by the receiver's own address extension.
- **The largest message by count.** 4095 bytes are 586 frames in normal
  addressing and 683 behind an address extension, with one flow control.
- **A link takes only its own.** Two links listen on one CAN id for
  different address extensions, as two sessions of a gateway do: a frame
  led by another address extension, a normal-addressing frame, the
  address extension alone, a single frame of 7 bytes and a first frame
  that names 6 are all left alone and answered by nothing.
- **The flow control as set.** `isotp_set_flow_control()` (the local
  change of 2026-11-17): the block size as given, the separation time in
  the standard's coding (whole milliseconds, 0xF1 to 0xF9 for 100 to 900
  microseconds), in both addressings.

The bus half is the firmware's `.\test.ps1 j2534ext` (extended
addressing through the J2534 server against a gateway played by a PCAN)
and `.\test.ps1 j2534fc` (the flow control).
