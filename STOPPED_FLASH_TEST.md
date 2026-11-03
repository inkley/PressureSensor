# Stage-one stopped RAM-to-flash persistence

Implementation 2026-11-02. Firmware build 1006: CAN version 0.0.3.238.
This image has NOT been flashed or hardware-tested by the implementation step.

## Implemented scope

- New bounded RAM capture with a host UUID, 1..configured capacity (max 4094).
  No CAN sample streaming during this mode. Auto-stop at limit, no ring overwrite.
- Acquisition gate prevents ADC triggers while stopped; SysTick still increments
  GlobalTimer when serviced. ADC timeout freezes capture as FAULT and prevents
  persistence; no stale late conversion is admitted into a new capture.
- On acceptance capture resets RAM intentionally; start/resize commands are
  rejected while bounded recording is retained. Stop never clears RAM.
- Persist only while RAM_READY and entire log partition is blank. Hardware
  interrupts disabled during flash work and restored to prior state; CAN can
  pause. Partition symbols checked against constants at runtime. Bounded erase
  only targets verified blank selected log blocks; no mass erase/clear command.
- V1 compact DATA records and final manifest, marker last, per-block CRC,
  readback, then whole-record stream CRC check. Original RAM remains unchanged.
  A failed write quarantines storage (FAULT); no automatic retry/erase.
- Persist retry after COMMITTED is read-only verify. Nonblank media on boot is
  OCCUPIED: read-only verify/download permitted, new capture/start rejected.
  This is not a general interrupted-write recovery implementation.
- NOMINAL_TIME_ONLY|DEGRADED blocks and manifest NOMINAL_ONLY|TIME_UNCERTAIN|
  BEST_EFFORT. Timestamps synthesized from successful pair ordinal, NOT measured
  acquisition time. No claim of detecting unserviced SysTicks or timing jitter.
- DeviceId is UNKNOWN (zeros). A hardware factory unique-ID source was not
  established for this part/toolchain; provisioning remains open. UUID identifies
  this capture, not the physical module. RECORD_FORMAT.md clarifies this correction.
- ADC init resets the peripheral after enabling its clock (ADC#14) and selects
  SS2 priority 2, leaving unique priorities. New capture timeout requires reset
  before reuse, so ADC#01 late completion cannot enter a second capture.
- Legacy pending TX ownership fixed: ISR does not overwrite a pending frame;
  main loop copies/clears it in a short critical section. Queue-loss counters
  and complete acquisition accounting remain separate unfinished work.

Flash API busy polling has no software deadline; host 90 s persistence wait is
not a firmware timeout or proof the operation failed if no reply. Power-loss
fault tests, voltage/watchdog fault handling and actual save latency are pending.
No claim of full lifecycle ACK/CLEAR, autonomous restart, continuous flash logging,
or measured monotonic acquisition timestamps. Do not mark those milestones done.

## Bench commands after flashing

Use refreshed/rebuilt CCS Debug image; expect version 0.0.3.238. Restart Python
CLI to load new methods. Keep ordinary streaming stopped. Leave CCS running,
without breakpoints during capture/save. Flashing may erase old logging data:
back up any existing records and inspect programmer erase policy first.

```text
version
flash_status
```

Expect EMPTY. If OCCUPIED, stop: do not mass erase or force overwrite. Read-only
verify_flash/dump_flash can inspect a valid existing recording. Reclamation is
not exposed yet; unexpected contents need a separate deliberate decision.

For first test, 1000 pairs (~1 nominal second, no live CSV):

```text
flash_capture 1000
```

Wait at least 2 seconds, then:

```text
flash_status
set_filename flash_ram_before_01.csv
dump_buffer
persist_buffer
flash_status
verify_flash
dump_flash flash_persist_01
```

Expected RAM_READY/1000 before persist; dump_buffer receives 1000 in sequence.
Persist returns committed/readback verified 1000; status COMMITTED/1000;
verify_flash returns 1000. dump_flash saves a new directory with samples.csv and
recording.bin after host CRC/sequence/count validation. Compare Pressure1/
Pressure2 values exactly against flash_ram_before_01.csv in row order.
The binary reconstructs the partition using transferred manifest/data blocks;
untransferred areas are FF, not verified physical contents. CSV timestamps blank,
NominalTimeUs explicit; analyze_pressure can plot by SampleIndex/fs nominal only.

Repeat verify_flash and dump_flash: decoded samples and UUID should be identical.
Repeat persist_buffer: verification only, no extra writes. start/set_buffer_size/
flash_capture should be rejected while recording retained. The new CLI waits for
start ACK, so rejected legacy start does not start an empty host CSV logger.

Power-cycle/complete recovery testing is next milestone. If optionally checking
simple reboot retention, first save/verify host data, reboot, expect OCCUPIED
with RAM count zero and verify_flash/dump_flash still readable. Do not infer
interrupted-write recovery from a successful clean reboot.

## Command assignments (extension protocol, capability-gated)

Standard request/reply framing unchanged. 0x0F capability reply 0x49500101.
0x10..0x13 stage four UUID words (CAN argument big-endian, UUID bytes in RFC order).
0x14 starts bounded capture with pair limit. 0x15 persists/returns count.
0x16 returns state in top byte, RAM count in lower 24 bits.
0x17 verifies flash/returns count. 0x18 requests relative physical block 0..127;
reply count=256 then 256 frames tagged 0x1A, 16-bit word ordinal in bytes 1..2,
four raw little-endian flash bytes in bytes 4..7. Drop/reorder means retry block.
High bit set in a command result indicates error, not a record count.
Codes: 1 busy, 2 state, 3 size, 4 UUID, 5 nonblank/bounds, 6 flash driver error,
7 verify. State values: 0 EMPTY, 1 CAPTURING, 2 RAM_READY, 3 COMMITTED,
4 FAULT, 5 OCCUPIED. Legacy read_flash remains the RAM-dump alias.

## Validation performed without hardware

TI Debug compile/link passed; ELF partition/stack/vector check passed. Host tests
cover valid sizes (1/119/120/1000/4094), corrupt/missing/reordered/foreign blocks,
manifest commit, UUID/limit commands, failed start and flash-frame ordering.
Existing host tests and format/timer suites pass. These are not execution tests
of C flash routines or physical fault injection. Flash/reboot/power-loss tests
must still be performed on the eval board before persistence is accepted.
## First physical persistence result: 2026-11-02

User flashed firmware 0.0.3.238 and observed EMPTY/0. Bounded capture 1000
accepted UUID 608e18dd-b8ca-49dd-ac3f-36c82bb02629, auto-stopped RAM_READY/1000.
RAM dump received all 1000 pairs with sequence check passing. persist_buffer
reported committed/readback verified 1000; status COMMITTED/1000;
verify_flash passed; dump_flash host integrity checks passed.

Independent file comparison by the implementation agent: all 1000 ordered
(SampleIndex,Pressure1,Pressure2) tuples match exactly across the RAM CSV,
flash CSV, and separately decoded/CRC-validated reconstructed binary. UUID
matches capture and all CSV rows. Nine data blocks plus final manifest.

Local host artifacts (ignored): MasterPython/Data/flash_ram_before_01.csv and
MasterPython/Data/flash_persist_01_20261102_133214_082097/{samples.csv,recording.bin}.
No save latency measurement, clean reboot, interrupted-write recovery, repeated
read/idempotent persist or protection-rejection bench test recorded yet.
This passes the initial stopped-acquisition persistence/readback checkpoint;
it does not establish continuous acquisition or exact acquisition time.

Repeat verification and idempotent persist command also passed on the eval board.
Second dump: MasterPython/Data/flash_persist_repeat_01_20261102_133338_763134/.
Both samples.csv (87854 bytes) and reconstructed recording.bin (131072 bytes)
are byte-for-byte identical to the first download. Binary SHA256:
9b5560e402f721db6377d431f9b9850e58a29e6b0915c90ebeeca22163633ac8.
This confirms unchanged downloaded content after retry, not a physical erase
pulse measurement. Code path for COMMITTED retry invokes only LogVerify.
All 25 offline checks and firmware layout verification passed again before commit.
Clean reboot and interrupted-write testing remain pending.

## Clean reboot retention: 2026-11-02

User power-cycled the eval board without reflashing. Version 0.0.3.238;
flash_status OCCUPIED with RAM count zero; verify_flash returned 1000 pairs.
Downloaded MasterPython/Data/flash_after_reboot_01_20261102_133752_933305/.
Agent independently validated the binary with host parser and compared against
first pre-reboot download: samples.csv and reconstructed recording.bin are
byte-for-byte identical. All 1000 pairs retained in order across clean reboot.
Binary SHA256 remains
9b5560e402f721db6377d431f9b9850e58a29e6b0915c90ebeeca22163633ac8.

Clean-reboot retention passes. This is read-only recognition/download of a valid
manifest, not complete boot state reconstruction or interrupted-write recovery.
Interrupted writes/erases, journal ACK/CLEAR, and backpressure tests remain pending.
