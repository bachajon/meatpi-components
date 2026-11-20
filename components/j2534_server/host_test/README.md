# J2534 host tests

From the firmware checkout: `.\test.ps1 host j2534_server`.
Expected: `53 Tests 0 Failures 0 Ignored`.

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

What a session's wire version decides (`j2534_proto_session.c`,
`test_session.c`, six cases, 2026-11-18): the return codes against SAE
J2534-1's numbers (on the header of the day before this case fails at its
first line: `ERR_DEVICE_IN_USE` was 0x1A), the version a `HELLO` asks for
(no payload, a short one, 0, 1, 2, a later one), every status in the
numbers of version 1 and of version 2 with no two sharing a version 1
number, the refusal of a second tester as each kind of client is
answered, the `EVENT RX_LOST` frame of the wire protocol's section 13
byte for byte, and what a loss counter gained since the last look (told
once, a counter that started again). The bus half is the firmware's
`.\test.ps1 j2534session`.

ISO15765 extended addressing, the layout (`j2534_iso_ae.c`, `test_iso_ae.c`,
six cases, 2026-11-20): what a FLOW_CONTROL filter asks for (the flag on
either message, five bytes each, the two address extensions by direction,
the flag with four bytes refused and the caller's record left alone), a
tester's message split into id, address extension and payload, the single
frame behind an address extension bare and padded (six bytes fill it,
seven do not fit), a received message and the two indications with
`ISO15765_ADDR_TYPE` in `RxStatus` and five bytes of id and address
extension, whether a reassembled message is the one a first frame starts
(both length forms, and the frame one byte shorter behind an address
extension), and the frames a message takes in each addressing. The
vectors are the ones of the wire protocol's section 3. The bus half is the
firmware's `.\test.ps1 j2534ext`.

The production READ_VBATT helper runs against a fake battery owner: voltage
conversion and rounding, a valid zero, refusal without an open device, no
sample, NaN/infinity, negative and out-of-range readings. Failures leave the
result untouched. The byte-exact request and replies of the wire protocol's
section 3 are a vector of the suite (the helper and the ACK encoder together). No ADC hardware is emulated here. The device and real DLL
half is the firmware's `.\test.ps1 j2534vbat`.
