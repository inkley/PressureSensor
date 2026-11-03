> Detailed specification/evaluation; distinguish proposed policy from implemented
> behavior. For current validation and remaining work, see [current status](../CURRENT_STATUS.md).
> Commands and source paths assume the repository root as working directory.

# Provisional flash capacity budget

2026-11-02. Calculations from current Debug linker map and linker command file.
No flash erase/program code or new allocation is introduced. This is a capacity
model, not an approved on-flash binary format or a recovery proof.

## Verified memory boundaries

| Region | Start | End exclusive | Size | Current map used | Current map unused |
| --- | --- | --- | --- | --- | --- |
| Application | 0x00000000 | 0x00020000 | 131072 B | 17856 B (0x45C0) | 113216 B |
| Logging reservation | 0x00020000 | 0x00040000 | 131072 B | Not allocated by linker | Not yet formatted |
| SRAM | 0x20000000 | 0x20008000 | 32768 B | 19137 B | 13631 B |

Source: Debug/Inkley_PressureSensor.map (linked November 2, 2026) and
`__flash_log_start__`, `__flash_log_end__` in tm4c123gh6pm.cmd. Application growth
space stays in the lower partition; it is not added to logging capacity. There
is no separate flash-resident bootloader allocation today. A future bootloader
requires explicit repartitioning and recalculation. SRAM free-map bytes are not
a demonstrated runtime stack margin.

TI specifies individually erasable 1 KiB blocks for this MCU. Both log boundaries
are aligned. [TI datasheet, flash chapter](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf).
ELF layout checker passed: no application load segment occupies the reservation.
Linker exclusion does not prevent debugger mass erase or software writes.

## Proposed partition budget: one active recording

| Purpose | 1 KiB blocks | Bytes |
| --- | --- | --- |
| Two alternating metadata journals | 2 | 2048 |
| Session descriptor/finalization allowance | 1 | 1024 |
| Gap/error events | 4 | 4096 |
| Recovery/erase staging spares | 2 | 2048 |
| Data blocks | 119 | 121856 |
| Total | 128 | 131072 |

This deliberately reserves 9216 bytes before sample blocks. Journals/spares are
space allowances, not proof of wear leveling, transaction atomicity or indefinite
power-loss recovery. Descriptor finalization must be append-only or separately
committed; do not erase the only recoverable descriptor. Event overflow must
set a sticky integrity flag and report lost event detail. Additional sessions,
spare retirement, journal exhaustion or abandoned partial blocks reduce capacity.
Do not overwrite unacknowledged records to meet these theoretical maxima.

## Candidate block envelope

Each 1024-byte block has a 64-byte header and 8-byte trailer. All fields are
explicitly serialized; do not depend on compiler structure padding or native
endianness. Proposed byte budget (not a protocol assignment):

| Header field | Bytes |
| --- | --- |
| Magic | 4 |
| Version and record-format ID | 2 + 2 |
| Session UUID | 16 |
| Block sequence | 4 |
| Valid record count and flags | 2 + 2 |
| First acquisition slot and first MCU trigger time | 8 + 8 |
| Nominal period | 4 |
| Reserved/extension | 12 |
| Header total | 64 |

Trailer: 4-byte CRC plus separate 4-byte commit marker written last. Define CRC
algorithm, coverage (including header, count, payload/padding) and recovery rules
in the record-format step. A marker is not an assertion of atomic programming;
recovery must validate bounds, identity, CRC and marker and tolerate torn writes.

Remaining payload = 1024 - 64 - 8 = 952 bytes per data block.

## Capacity comparison at nominal 1000 pairs/s

| Candidate | Bytes/pair | Pairs/block | Maximum pairs | Nominal successful-sample duration |
| --- | --- | --- | --- | --- |
| Raw P1/P2 only | 4 | 238 | 28322 | 28.322 s |
| Compact timed candidate | 8 | 119 | 14161 | 14.161 s |
| Explicit index/timed candidate | 16 | 59 | 7021 | 7.021 s |

Formula: 119 * floor(952 / record_bytes). The 16-byte option leaves 8 bytes
unused per data block. These are ideal fully packed single-session capacities,
not guaranteed wall-time retention during faults. Partial blocks and metadata
changes reduce sample capacity; missing acquisitions may lengthen the wall-time
span without creating more valid samples.

- Raw option: two uint16 ADC counts, index/time inferred from block and nominal
  cadence. Not sufficient for measured per-sample jitter; cannot silently claim
  actual acquisition timestamps. Not recommended as the final timed format.
- Compact timed candidate: uint32 trigger-time offset (us) plus two uint16 ADC
  counts. Block supplies first uint64 slot/time. Slot indices inferred only for
  a consecutive run; start a new block on any slot discontinuity, maintain gap
  events, and split before time-offset overflow. Flags in block/event metadata.
  Gap-induced partial blocks can substantially reduce capacity. Encoding slot
  discontinuities, flags and ambiguous timing must be finalized before choosing.
- Explicit candidate: uint64 slot index, uint32 trigger-time offset from block
  base, two uint16 ADC counts. Flags/event references need block/event metadata;
  if per-record flags are added, increase record size and recalculate. This
  retains explicit indices/time with less encoding complexity but lower capacity.

RECORD_FORMAT.md now specifies 48-byte events. With the same block envelope,
the four event blocks fit 4*floor(952/48)=76 events before partial-block losses.
The earlier 32-byte/116-event calculation was illustrative and is superseded.

| Capture target | Raw data blocks | Compact timed blocks | Explicit index/timed blocks |
| --- | --- | --- | --- |
| 10 s / 10000 valid pairs | 43 | 85 | 170 |
| 30 s / 30000 valid pairs | 127 | 253 | 509 |

Only 119 data blocks are available in this budget. Therefore 10 s fits the raw
or compact options under ideal packing; 30 s fits none. Even the raw option's
127 data blocks plus 9 reservation blocks exceeds the partition. These are flash
capacity statements, not proof that acquisition can run continuously that long.

## RAM and mission architecture implications

Current RAM array is 4094*4=16376 bytes: 4.094 s of nominal raw pairs. Capturing
then stopping cannot create a 10 s recording with this array. Adding per-sample
RAM timestamps/indices requires smaller capacity or a compact block design.
No second full-size RAM buffer is assumed. Background storage/transfer must be
implemented and tested separately, including flash-induced execution stalls.

For the first persistence/reboot-recovery test, use a bounded RAM-supported
capture. Keep 10-30 s as the later logical segmentation objective for incremental
transfer to the IVER backseat CPU. Internal flash provides bounded backlog;
maximum outage tolerance is smaller than the capacities above if some space
already holds protected unacknowledged data. Never promise indefinite continuous
capture with finite storage and an unavailable CPU sink.

## Reproduction and completion status

Run from firmware repository:

```powershell
python tools/calculate_flash_capacity.py
python tools/verify_firmware_layout.py Debug/Inkley_PressureSensor.out
```

Calculator reads current map/boundaries and emits the assumptions and arithmetic;
it does not validate hypothetical record serialization or write flash.
Current provisional capacity analysis complete. Final capacity is pending binary
format, event budget, metadata/recovery implementation and measured RAM budget.
Endurance, write latency and allowable backseat outage remain separate open work.

User accepted the provisional duration ranges on 2026-11-02. Capacity checklist
item addressed on that basis; final binary-format capacity remains pending.

The v1 design is now specified in RECORD_FORMAT.md: 8-byte compact timed
samples, 64-byte headers, 8-byte trailers, 48-byte events and a 128-byte final
manifest. Ideal data capacity remains 14161 pairs; implementation/serialization
and fault-dependent fragmentation remain to be validated on the MCU.
