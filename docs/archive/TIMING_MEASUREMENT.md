> Historical checkpoint/reference, preserved for evidence. Statements about what was
> current or pending apply to the dated sections, not the latest firmware.
> Commands/source paths assume repository-root working directories. See
> [current status](../CURRENT_STATUS.md) and the [acceptance procedure](../ACCEPTANCE_PROCEDURE.md).

# Acquisition, CAN and flash timing checkpoint (.241 / build 1009)

Status: .241 acquisition-only, normal CAN and 1000-pair stopped-storage
measurements collected on 2026-11-02. Maximum-size 4094-pair capture, persist and exact RAM/flash comparison passed;
clean-reboot retention and exact pre/post-reboot comparison also passed.
Keep the measurement checklist item open; continuous flash overlap is untested.

## What is measured

WTIMER0 is reserved as a 64-bit free-running system-clock counter. At the verified
20 MHz clock one tick is nominally 50 ns. TimerValueGet64 reads high/low/high
coherently. The timer continues through CPU flash stalls, masked interrupts and
CCS halts; measurements must run without breakpoints, stepping or suspend.
It is not an independent clock reference, UTC, or a replacement for recording
acquisition timestamps. Reference: [TI device data sheet, General-Purpose Timers](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf).

- ADC attempts, complete pairs, timeout and FIFO/error outcomes; coherent snapshot.
- Software trigger-to-trigger min/max intervals and intervals outside 1000 +/-10 us.
  This provisional interval tolerance does not certify absolute sampling phase.
- Estimated missing slots: sum(max(floor(interval/nominal_period)-1,0)). This can
  undercount irregular intervals; it is not an exact hardware-trigger loss count.
- Successful ADC software-path min/max time, including trigger/poll/read overhead;
  this does not measure P1/P2 aperture separation.
- CAN pending-frame publication drops (frame opportunities, not pair counts),
  realtime transmit failures and command-receive software/hardware overruns.
  RAM overwrites are intentional legacy circular-buffer overwrites, not ADC loss.
- Flash erase/program call counts, summed and maximum driver-call duration,
  driver errors and rejected writes while acquisition is active. Program counts
  are API calls, not individual flash words or endurance cycle counts.
- Last storage command kind (1 persist, 2 ACK, 3 clear, 4 scratch), total elapsed
  time, ADC attempts and flash calls during it. Rejected/idempotent commands can
  have zero flash calls; do not treat these as write-performance measurements.
- Serviced SysTick min/max spacing and long intervals, including deliberate
  stopped-acquisition storage pauses. These pauses are not lost active samples.

Counters are volatile since boot or timing_reset. Separate runs require separate
resets. TIM1 snapshots include schema/count/sequence, ordered words and CRC32;
host rejects incomplete, reordered, duplicate or corrupt transfers. Snapshot
copy masks interrupts briefly and can affect timing; retrieve after timing_stop.
Instrumentation itself adds ISR work. CAN counters do not prove delivery through
USB/host or all network/controller error conditions. Saved flash sample times
remain nominal-only. No continuous-flash acquisition is enabled by this change.

## Bench procedure: preserve the current recording

1. Save/close the CLI. Build and flash .241 using CCS **Necessary Pages Only**,
   preserving [0x20000,0x40000). Resume target; do not leave breakpoints enabled.
2. Restart updated InkleySensor.py. Check version (.241), flash_recovery and
   verify_flash. Existing recording should still verify; do not clear it.
3. Acquisition alone (no RAM writes, no sample CAN frames):
   ```text
   timing_stop
   timing_reset
   timing_start idle
   ```
   Wait approximately 10 seconds without CCS interaction, then:
   ```text
   timing_stop
   timing_status acquisition_only_01
   ```
4. Normal packed realtime CAN traffic, still no RAM/flash writes:
   ```text
   timing_reset
   timing_start can
   ```
   Wait approximately 10 seconds, then:
   ```text
   timing_stop
   timing_status acquisition_can_01
   verify_flash
   ```
   Host logs a unique timing_can CSV with host-estimated times; use the JSON for
   timing evidence. The first/last MCU attempt times determine average cadence,
   not how quickly the operator types or host log receipt times.
5. Measure a real flash write while retaining the current unacknowledged second
   recording. Verify this path matches the recording currently on the MCU:
   ```text
   timing_reset
   ack_flash Data/lifecycle_second_flash_01_20261102_142246_462870/recording.bin
   timing_status flash_ack_01
   flash_recovery
   verify_flash
   ```
   This durably ACKs but does not delete. An already durable ACK is idempotent,
   so may produce zero writes. Review erase/program call counts before claiming
   a measured write. Do not erase merely to manufacture timing results.

## Subsequent full-cycle timing run

After reviewing the above, use the established explicit clear/new capture test
with verified host evidence. Reset diagnostics immediately before each clear or
persist command and save a report immediately after. Example for a NEW stopped
RAM_READY capture: timing_reset, persist_buffer, timing_status flash_persist_01,
verify_flash. Repeating persist on COMMITTED only verifies and is not a flash
write timing test. Measure 1000 pairs first and then maximum 4094 pairs. Clear
and ACK must always use the matching saved recording, never an older identity.

## Acceptance and remaining work

Record clock_hz, attempts/completed/errors, pair rate, interval extrema, tolerance
violations, estimated gaps, CAN drops/overruns, flash call counts/maxima and whole
storage duration. Zero active ADC timeouts/errors and no unexplained CAN loss are
targets, not assumed results. Report every measured tolerance violation for review.
During stopped storage, storage_attempts must be zero; a long SysTick interval
is expected when interrupts are masked. Flash errors and unsafe writes must be
zero. Preserve reports even if targets fail.

Compare acquisition-only versus CAN runs before concluding timing is acceptable.
Debugger-halt-contaminated runs must be repeated. Present measurements describe
this instrumented image and traffic, not worst-case flash aging, mission CPU
load, independent oscillator accuracy, acquisition phase or continuous storage.
Busy-window power-cut testing, per-sample timestamp/gap records and safe
continuous flash execution/queueing remain separate open work.

## Measured eval-board results: 2026-11-02, firmware .241

Source: CRC-verified CLI snapshots saved in MasterPython/Data, inspected directly.
All five reports show clock_hz=20000000 and zero ADC timeouts/errors, CAN TX queue
drops/failures, software/hardware RX overruns, flash errors and unsafe-write
rejections. These are observations for these runs, not universal guarantees.

| Acquisition run | Attempts / complete | Pair rate (Hz) | Trigger interval min/max (us) | Outside +/-10 us / estimated missing |
|---|---:|---:|---:|---:|
| Acquisition only | 13551 / 13551 | 1000.000000 | 1000.000 / 1000.000 | 0 / 0 |
| Normal packed CAN | 10539 / 10539 | 1000.000052 | 999.450 / 1000.050 | 0 / 0 |

The CAN CSV contains 10538 pairs. One terminal unpaired acquisition is consistent
with stopping the two-pairs-per-frame publisher after an odd attempt count; this
is not proof that every frame reached the host. Successful ADC software-path
min/max were 23.150/30.200 us (alone), 23.150/30.400 us (CAN), not channel aperture
spacing. No SysTick long-interval flags occurred in either acquisition run.

| Stopped storage operation | Erase/program API calls | Erase max (ms) | Program max (ms) | Whole operation (ms) | ADC attempts during storage |
|---|---:|---:|---:|---:|---:|
| ACK second lifecycle recording | 1 / 2 | 6.115000 | 6.291200 | 411.985950 | 0 |
| Clear second lifecycle recording | 11 / 2 | 6.113100 | 6.289450 | 738.352500 | 0 |
| Persist new 1000-pair capture | 10 / 20 | 0.076700 | 9.144050 | 749.765250 | 0 |

Storage kind values were 2,3,1 respectively. Flash-call totals during each command
were 3,13,30. Whole-operation durations include validation/scans and processing,
so must not be equated with erase/program duration or the interrupt-mask window.
Serviced SysTick maximum intervals were 68.157500 ms (ACK), 394.150650 ms (clear),
and 407.497800 ms (persist); long-interval flags were 1,3,1. These are measured
stopped-acquisition interrupt-service pauses, not successful continuous sampling.
The much shorter erase calls on the freshly cleared destination are observations,
not a timing guarantee for programmed pages, future cycles or temperature.

Counters were reset AFTER the new 1000-pair capture, before persist. Thus its
zero attempts say nothing about capture cadence. RAM_READY=1000, committed
readback, subsequent verify and host download passed. No exact RAM/flash sample
comparison was performed for this particular new capture.

Retained current recording: a3d6148d-ac85-4cbd-9bfe-0e211643ac8b, 1000 pairs,
COMMITTED (no ACK/deletion sent for this identity yet). Host backup:
Data/timing_persist_01_20261102_150018_608399/recording.bin.
Offline validation confirms identity, count, block/record integrity and manifest
CRC 2007920151; SHA256:
`f34f03ad743e8f54fd2a4fda2d738497a8de9f7740863486de6fe25021e21c95`.
The binary is reconstructed; omitted journal/scratch areas are FF, not physically
read bytes. Validation does not independently establish current MCU contents.

### Report evidence

- `acquisition_only_01_20261102_145406_849267.json` SHA256 `f89d01162cd6d2dfa7971d81e215929c09cee72d98a76e5c568fbbcc20f38a94`
- `acquisition_can_01_20261102_145621_302626.json` SHA256 `9f3995039d0c28c0a73c5c9987bf6433ed34331fddd91b0276acfa6303a8eb6d`
- `flash_ack_01_20261102_145847_806816.json` SHA256 `b3f15f684a8730616b16dbd04c0a5ff42adbeb240797b40d7fd9d38b6ca66bf5`
- `flash_clear_01_20261102_145947_718577.json` SHA256 `4b715bc1b62d90843be33985a46d399f62d317b32ca88f208daccc4c2251d664`
- `flash_persist_01_20261102_150016_202673.json` SHA256 `18d539bcc9d5ae2926ffa1a3c6f6d34a458bd7d9eee81c37f221e6f7109fcd0c`

## Maximum-size test: pending, manual bench sequence

The current recording stays intact during documentation. Maximum-size capture
requires explicit reclamation of the single-record slot. Use only its verified
host binary above for ACK/CLEAR; retain that file. No firmware reload is needed.
Do not continue after any failed command or unexpected state.

```text
verify_flash
ack_flash Data/timing_persist_01_20261102_150018_608399/recording.bin
flash_recovery
clear_flash Data/timing_persist_01_20261102_150018_608399/recording.bin
flash_recovery
set_buffer_size 4094
buffer_status
timing_reset
flash_capture 4094
```

Require ACKNOWLEDGED before clear and CLEARED after. Buffer capacity must be
4094 before capture. Wait at least 6 seconds without halting/stepping CCS. Then:

```text
flash_status
timing_status max4094_capture_01
set_filename max4094_ram_before_01.csv
dump_buffer
```

Require RAM_READY with exactly 4094 pairs, 4094 complete attempts and no ADC errors,
then review timing/overrun counters. Dump must report all 4094 pairs and sequence
check passed. Preserve this RAM CSV for exact index/P1/P2 comparison after flash.
Only after capture and RAM download succeed:

```text
timing_reset
persist_buffer
timing_status max4094_persist_01
verify_flash
dump_flash max4094_flash_01
flash_recovery
```

Expected successful persistence is 35 data pages plus manifest: 36 erase and 72
program API calls (108 total), 4094 verified pairs, zero ADC attempts during the
stopped storage command and zero flash errors. Actual durations must be measured.
Compare all 4094 RAM and decoded flash pairs before accepting; then test clean
reboot retention of this maximum-size recording. Do not clear it afterward.
Maximum-size results and acceptance remain pending until bench output/files are
reviewed. Continuous acquisition/flash overlap and physical power-cut acceptance
stay open independently of this bounded-storage checkpoint.

## Maximum-size measured result: 2026-11-02 (.241)

Capture identity `64f906c8-461f-44a0-955c-17e21460d71d` reached RAM_READY with
4094 pairs. Snapshot: 4094 attempts/completed, 1000.0 Hz, trigger intervals
999.950--1000.050 us, no ADC errors/timeouts, tolerance violations, estimated
missing slots, reported CAN losses or RAM overwrites. No flash writes occurred
during capture. ADC software-path time was 23.150--30.400 us.

Stopped persistence committed/readback verified 4094 pairs, followed by MCU CRC
verification, verified host download and COMMITTED recovery classification.
36 erase + 72 program API calls (108 total) matched 35 data pages plus manifest.
Whole operation: 2402.929200 ms. Longest erase: 0.076750 ms; longest program:
9.145200 ms. Sum of erase calls: 2.759250 ms; sum of program calls: 327.487350 ms.
The whole-command duration includes processing/scans/verification and is not the
sum of flash-call durations. Maximum serviced SysTick interval was 1506.318900 ms
with one long-interval flag. ADC attempts during storage, unsafe-write rejections
and flash errors were zero. This is a deliberate acquisition pause, not a
continuous 1 kHz result.

Host comparison checked exactly all ordered (SampleIndex,Pressure1,Pressure2)
triples in max4094_ram_before_01.csv against downloaded samples.csv: 4094/4094
identical with contiguous indices 0..4093. Host binary block/record integrity,
count and UUID validated independently. Timestamp columns were not compared:
RAM CSV timestamps are host receipt; flash times remain nominal-only.

Current maximum-size recording remains COMMITTED and unacknowledged. No clear
or ACK was sent by the documentation/comparison work. Clean-reboot retention of
this particular maximum-size recording is the remaining bounded checkpoint test.
After a normal board restart (no reflash/erase), run version, flash_recovery,
verify_flash and dump_flash max4094_after_reboot_01. Compare the resulting binary
and all samples against this saved pre-reboot copy before accepting reboot
retention. No physical interrupted-write or continuous-overlap result is implied.

Evidence below is under MasterPython/Data (raw artifacts remain ignored by Git):

- `Data/max4094_ram_before_01.csv` SHA256 `9369ee7c09eb8fa24f73c72025ac4a51a0ce2e404e21b3c03f602637bf91e31e`
- `Data/max4094_flash_01_20261102_150925_176265/samples.csv` SHA256 `6b8746785bcd5ce8043e9b5d3b595e967b7861df325f5d5fd82e97fa1ec775c7`
- `Data/max4094_flash_01_20261102_150925_176265/recording.bin` SHA256 `7c8387eb72a0e2a49be899b3922ab7eb722c841fbb54b3adea2b5ef11d63ad35`
- `Data/max4094_capture_01_20261102_150808_303400.json` SHA256 `9cd8f6736a5e6e98d5811800ed53d5af0ad49f07fd3ccd86aaa4c1a1252bd4d3`
- `Data/max4094_persist_01_20261102_150921_704375.json` SHA256 `08f7de545805195f257e360b9b283b558e70d766f14187fc386144e829ed69a9`

## Maximum-size clean-reboot retention: passed 2026-11-02

After the user-reported normal restart, .241 reported COMMITTED with 4094 pairs,
MCU verification passed, and dump_flash max4094_after_reboot_01 downloaded all
4094 verified pairs. Offline comparison found the reconstructed recording.bin
byte-identical to the pre-reboot file, and all 4094 ordered index/P1/P2 triples
identical. Host validation independently confirmed UUID
64f906c8-461f-44a0-955c-17e21460d71d, count and block/record integrity.

Post-reboot directory: Data/max4094_after_reboot_01_20261102_151106_784281.
Reconstructed binary SHA256: `7c8387eb72a0e2a49be899b3922ab7eb722c841fbb54b3adea2b5ef11d63ad35`.
Omitted physical journal/scratch bytes are not covered by binary equality.
The recording remains COMMITTED; no ACK or deletion was sent by this comparison.

The bounded .241 timing/storage checkpoint now passes: acquisition-only and
normal packed CAN timing, 1000/4094-pair stopped persistence, exact maximum-size
RAM/flash comparison and maximum-size clean-reboot retention. The broad checklist
item must retain a stopped-storage qualification: acquisition during flash is
not implemented/tested; CAN saturation, worst-case aged flash, physical interrupted
writes/erase and independent clock calibration are not established by these runs.
