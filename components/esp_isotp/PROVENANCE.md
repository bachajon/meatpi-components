# esp_isotp: vendored component

Espressif upstream **esp_isotp v0.1.1** (isotp-c based), copied into
an earlier meatpi project and adopted here 2026-07-07 with it. Uses the
NEW `esp_driver_twai` node API. `src/isotp_config.h` is force-included
via CMake (upstream arrangement) to pin the isotp-c config.

## Local changes (this repo)

- 2026-10-19: the `Kconfig` default of `ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US`
  raised from 100000 to 1000000 (N_Bs / N_Cr per ISO 15765-2; an ECU busy
  with a flash block that answered flow control late aborted the transfer
  at 100 ms). Nothing else beyond the inherited isotp_config.h arrangement.
- 2026-11-17 (Elias Kotlyar, pull request 3): the flow control a link sends
  as a receiver is the link's own instead of the two Kconfig values.
  `isotp-c/isotp.c` and `isotp.h`: `fc_bs` and `fc_st_min_us` in
  `IsoTpLink`, set by `isotp_set_flow_control()`, used for the flow control
  after a first frame and at a block end; a block size of 0 no longer
  lets the block counter wrap (it sent a second flow control after 255
  consecutive frames). `inc/esp_isotp.h` and `src/esp_isotp.c`:
  `flow_override`, `flow_block_size` and `flow_st_min_us` in
  `esp_isotp_config_t`, and `esp_isotp_set_flow_control()` for a transport
  that is open. `Kconfig`: `ISO_TP_DEFAULT_BLOCK_SIZE` takes 0 (range 0 to
  32) and defaults to 0, `ISO_TP_DEFAULT_ST_MIN_US` defaults to 0. The
  firmware's tracked sdkconfig still says 8 and 1000, and no session of
  `can_isotp_esp` uses the Kconfig values any more (it always overrides).
- 2026-11-19: ISO 15765-2 extended addressing, per link and off by default.
  `isotp-c/isotp.c` and `isotp.h`: `addr_ext`, `send_ae` and `receive_ae`
  in `IsoTpLink`, set by `isotp_set_addr_ext()`; every frame sent (single,
  first, consecutive, flow control) is led by `send_ae` and carries one
  byte less, a received frame is taken only when it starts with
  `receive_ae`, which is stripped before the PCI is read. `inc/esp_isotp.h`
  and `src/esp_isotp.c`: `addr_ext`, `tx_addr_ext` and `rx_addr_ext` in
  `esp_isotp_config_t`. A link without it behaves as before.
- 2026-11-20: `host_test/` (ours, not upstream's): the link's two local
  changes above, two links back to back over every message size, with
  the frames of normal addressing pinned to what the library sent before
  extended addressing was added. Run it after taking a new copy from
  upstream (`host_test/README.md`).
