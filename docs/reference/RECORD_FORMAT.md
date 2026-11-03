> Next measured encoding/budget: [METADATA_V2](METADATA_V2.md). It is a design, not enabled firmware.

> Time/identity proposals are superseded where they conflict by [TIME_MODEL](TIME_MODEL.md).
> Existing v1 bytes/nominal-time semantics remain unchanged; new measured metadata requires versioning.

> Detailed specification/evaluation; distinguish proposed policy from implemented
> behavior. For current validation and remaining work, see [current status](../CURRENT_STATUS.md).
> Commands and source paths assume the repository root as working directory.

# Recording binary format v1

2026-11-02. The .241 bounded nominal-time DATA/MANIFEST and lifecycle paths are
implemented and bench-validated. Measured-trigger timing, event logging and UTC
metadata described as proposed below are not implemented; see CURRENT_STATUS
and TIME_MODEL for qualified scope. Integers are little-endian, unsigned, explicitly
serialized. UUID fields use RFC byte order (Python UUID.bytes), not GUID mixed
endianness. All-zero recording IDs are invalid. Offsets below are decimal.

## Physical layout

Partition [0x20000,0x40000), 128 blocks of 1024 bytes:

| Relative blocks | Role |
| --- | --- |
| 0, 1 | Alternating lifecycle journal snapshots |
| 2 | Final recording manifest |
| 3..6 | Gap/event blocks |
| 7..8 | Recovery staging spares, excluded from nominal capacity |
| 9..127 | Sample blocks |

One protected recording for v1. No background reuse or implicit overwrite.
Treat any non-erased uncommitted block as quarantined until explicitly authorized
reclamation. Spares are an allowance, not a completed relocation algorithm.
A future multi-record allocator requires another version/design review.

## Common block envelope (exactly 1024 bytes)

| Offset | Bytes | Field |
| --- | --- | --- |
| 0 | 4 | ASCII IPLG |
| 4 | 2 | Version = 1 |
| 6 | 2 | Kind: 1 DATA, 2 EVENTS, 3 MANIFEST, 4 JOURNAL |
| 8 | 16 | RecordingId UUID |
| 24 | 4 | Block sequence |
| 28 | 2 | Record count |
| 30 | 2 | Flags |
| 32 | 8 | First acquisition slot |
| 40 | 8 | First MCU trigger time, microseconds since boot/segment time domain |
| 48 | 4 | Nominal period = 1000 microseconds |
| 52 | 12 | Reserved, zero |
| 64 | 952 | Records followed by erased padding (0xFF) |
| 1016 | 4 | CRC-32 of bytes 0..1015 inclusive |
| 1020 | 4 | ASCII CMIT, programmed last |

Flags: bit0 ACTUAL_TRIGGER_TIME, bit1 NOMINAL_TIME_ONLY (exactly one set);
bit2 GAP_BEFORE; bit3 DEGRADED. Other bits rejected. ACTUAL refers to measured MCU
trigger proxy, not exact ADC aperture or UTC. NOMINAL uses synthesized schedule
time and must never be exported as measured acquisition time. Legacy raw RAM
can only use NOMINAL with honest quality/unknown-gap indicators; it lacks actual
timing metadata. A reset starts a new time domain and recording identity.

## Data records (8 bytes, maximum 119 per block)

uint32 trigger_delta_us; uint16 P1; uint16 P2. Counts limited to 0..4095;
high bits must be zero. First delta is zero; later deltas strictly increase.
Actual time = header.first_us + delta, checked for uint64 overflow. Slot =
header.first_slot + record ordinal, checked for overflow. Therefore a block
contains only a consecutive slot run. Split BEFORE any slot discontinuity,
uint32 delta overflow, time-basis change, reboot, or incompatible quality change.
A gap produces an event and a new block with GAP_BEFORE. Nominal mode requires
delta=ordinal*1000; actual mode preserves observed timing, including irregular
intervals. Timing faults within consecutive slots may be event-tagged without
inventing missing samples. Do not extrapolate across block boundaries blindly.
Block base/time and deltas remain immutable after committing.

## Event records (48 bytes, maximum 19 per block)

| Relative offset | Bytes | Field |
| --- | --- | --- |
| 0 | 8 | Event sequence, starts zero within recording |
| 8 | 4 | Reason bitmask |
| 12 | 4 | Validity flags |
| 16 | 8 | Current/next slot, if known |
| 24 | 8 | Previous trigger time, if known |
| 32 | 8 | Current trigger time, if known |
| 40 | 8 | Missing pair count, if known/estimated |

Reasons bits 0..9: ADC_TIMEOUT, ADC_ERROR, TIMING_EARLY, TIMING_LATE,
UNSERVICED_SLOTS, CLOCK_DISCONTINUITY, DEBUG_HALT_CONFIRMED, RAM_LOSS,
TX_LOSS, EVENT_DETAIL_LOST. Remaining bits invalid; at least one reason required.
Validity bits: 0 slot known, 1 previous time known, 2 current time known,
3 missing count exact, 4 missing count estimated. Exact/estimated mutually
exclusive; neither means unknown. Unknown fields serialize zero and are never
interpreted as known zero. Count estimates are not measured loss. Event times
use the recording time basis; a clock discontinuity invalidates cross-boundary
durations. Detailed timeout/transport semantics follow the accounting contract.
Header slot/time mirror first event's known current fields, otherwise zero.
Four event blocks hold 76 events before partial-block losses. Overflow sets
manifest EVENT_DETAIL_LOST; never silently promise every event detail survived.

## Final manifest record (128 bytes, count=1)

| Relative offset | Bytes | Field |
| --- | --- | --- |
| 0 | 16 | Device unique-ID bytes (12 MCU ID bytes in documented word order, then four zero bytes) |
| 16 | 16 | Optional RunId UUID; all zero means absent |
| 32 | 8 | Segment number (zero if run absent) |
| 40 | 8 | Valid sample-pair count |
| 48 | 8 | Trigger attempts |
| 56 | 8 | ADC timeouts |
| 64 | 8 | Estimated unserviced slots |
| 72 | 8 | First valid slot |
| 80 | 8 | Last valid slot |
| 88 | 8 | Capture stop time in the same time domain |
| 96 | 4 | DATA block count |
| 100 | 4 | EVENTS block count |
| 104 | 4 | Recording stream CRC |
| 108 | 4 | Recording quality flags |
| 112 | 4 | Stop reason |
| 116 | 8 | Other ADC failed attempts |
| 124 | 4 | Manifest schema = 1 |

Device ID is zero in the stage-one implementation, explicitly UNKNOWN. A
factory unique-ID register source was not established for this part/toolchain;
the earlier assumed three-word ID source is withdrawn. Do not use DID0/DID1
as unique module identity. Provisioning a stable module ID remains required
before multi-module deployment. Nonzero DeviceId serialization must be defined
and capability-versioned before deployment.
Manifest header uses first valid slot/time; its sequence equals total DATA plus
EVENTS blocks. At least one valid pair required in v1. Check attempts = valid
pairs + timeouts + other ADC failures at finalization; invalid equality is an
integrity error. Stop time must not precede last valid time if timing is valid.

Quality bits 0..7: NOMINAL_ONLY, ACQUISITION_GAPS, TIMING_VIOLATIONS,
RAM_LOSS, TX_LOSS, EVENT_DETAIL_LOST, TIME_UNCERTAIN, BEST_EFFORT.
Unknown higher bits rejected. TIME_UNCERTAIN means skipped-slot estimate can
be only a lower bound, never an exact total. Stop reason values: 1 requested,
2 RAM_FULL, 3 configured limit, 4 acquisition fault, 5 storage/backpressure.
Totals not present here (e.g. TX details) must be retained in events or separately
versioned status, not assumed durable. Event exhaustion preserves manifest's
sticky flags but exact lost-detail totals require a future schema extension.

## Block order and recording integrity

DATA and EVENTS share one uint32 sequence starting at zero, increasing without
gaps, independent of physical address. Reject duplicates, missing sequences,
foreign recording IDs, illegal region/kind and out-of-bounds counts. The manifest
is sequence N after N data/event blocks. Slot indices and times in sample order
must increase across DATA blocks; clock discontinuity requires a new recording.

Recording stream CRC feeds each complete committed 1024-byte DATA/EVENTS block
in sequence order (including block CRC and CMIT). It excludes manifest and journal
to avoid circular checksums. Counts, first/last indices, events and stream CRC
must agree with manifest before reporting a complete recovered recording. Valid
blocks without a valid final manifest are an incomplete capture, not a successful
recording. Diagnostic salvage is read-only and explicitly incomplete.

## CRC and commit transaction

CRC-32/ISO-HDLC: polynomial 0x04C11DB7, reflected implementation polynomial
0xEDB88320, refin/refout true, initial register 0xFFFFFFFF, xorout 0xFFFFFFFF.
Check string ASCII 123456789 gives 0xCBF43926. Stored little-endian.
Python zlib.crc32(data) yields the specified finalized value. For streaming use
zlib.crc32(next_block, previous_crc), starting with API seed zero; do not feed a
manually initialized 0xFFFFFFFF into this API. CRC detects accidental corruption;
it is not authentication or an absolute guarantee against undetected corruption.

Serialize complete block in RAM with FF padding. Verify destination erased,
program header/payload/CRC words at most once per erase, read back and validate
CRC, then program ONLY the separate marker word, then validate the whole block.
Skip padding/marker in preceding writes; don't program FF into marker and rewrite.
Commit data/events before the final manifest; freeze the recording before
finalization. Power loss may tear any write including marker: require exact
marker AND CRC/header/semantic validation; never trust marker alone. A failed
operation quarantines its block and retains original RAM until safe recovery.
Power-failure safety is a test requirement, not established by this definition.

## Lifecycle journals (64-byte record, count=1 per snapshot block)

The implemented schema and transaction rules are in [FLASH_LIFECYCLE.md](../archive/FLASH_LIFECYCLE.md).
Payload: RecordingId(16), manifest CRC(4), operation(2), zero flags(2),
generation(8), request UUID(16), DATA block count(4), sample count(4), zero
reserved(8). Counts replace the previous unimplemented proposal's final 16
reserved bytes. Header kind 4, flags NOMINAL_TIME_ONLY, slot/time zero; sequence
is generation low 32 bits. Exact CRC/CMIT envelope remains as above.

ACK retains samples. Explicit CLEAR requires matching durable ACK, commits a
DELETE snapshot, then erases only owned data and manifest last. DELETE proof is
retained across a new capture; generations continue across records. Boot does
not erase automatically; CLEAR_PENDING can be resumed explicitly with the
matching saved binary. Unknown/conflicting journal data stays read-only.
These additions do not change saved DATA/MANIFEST binary compatibility.

## Capacity and implementation status

Compact sample layout matches 14.161 s ideal flash sample capacity (119*119).
Event size is now 48, replacing the illustrative 32-byte assumption: 76 events
maximum, not 116. Discontinuities can consume partially filled blocks, reducing
sample capacity. RAM staging duration remains a separate limit.

`tools/log_format.py` is an offline envelope/sample codec; `test_log_format.py`
checks CRC vector, roundtrip, every single-byte corruption, torn prefix writes,
semantic bounds, capacity and time overflow. Event/manifest/journal PAYLOAD
semantics and full recording recovery are specified here but not implemented
by that codec. On-device stopped persistence and recovery are implemented; lifecycle ACK/clear
is implemented in build 1008 with bench testing pending. Actual timing capture,
continuous allocation and physical power-cut validation remain pending. Do not claim complete
recovery from passing envelope tests.
