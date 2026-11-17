# J2534 host tests

From the firmware checkout: `.\test.ps1 host j2534_server`.
Expected: `41 Tests 0 Failures 0 Ignored`.

The suite covers header/message codecs and bounds, filter matching, stream
framing, ACK vectors, TCP exposure, baud checks, ISO15765 layouts and
indications, J1850 request/response helpers, the TP 2.0 helpers, and the
settings migration.

The lists of an ISO15765 channel (`j2534_iso_cfg.c`, `test_iso_cfg.c`, five
cases): the values a channel opens with (0 and 0), `ISO15765_STMIN` in
its two codings and the reserved values, a
`SET_CONFIG` list taken whole (one value alone, the list udsoncan sends, a
parameter the channel passes over) or refused whole (a value out of range,
a good pair in front of it, a count without its pairs, too many pairs),
the id alone acknowledged without effect, the answer a `GET_CONFIG` makes
(the two it holds, the rest left out, an answer that does not fit), and
the byte-exact bodies of the wire protocol's section 12. The bus half is
the firmware's `.\test.ps1 j2534fc`.

The production READ_VBATT helper runs against a fake battery owner: voltage
conversion and rounding, a valid zero, refusal without an open device, no
sample, NaN/infinity, negative and out-of-range readings. Failures leave the
result untouched. The byte-exact request and replies of the wire protocol's
section 3 are a vector of the suite (the helper and the ACK encoder together). No ADC hardware is emulated here. The device and real DLL
half is the firmware's `.\test.ps1 j2534vbat`.
