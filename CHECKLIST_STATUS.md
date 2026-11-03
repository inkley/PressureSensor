# Checklist disposition: tested checkpoint 2026-11-02

Evidence: STOPPED_FLASH_TEST.md, FLASH_RECOVERY_TEST.md; firmware .238/.239,
1000-pair eval recording compared exactly to RAM and retained across reboot and
firmware upgrade. All 35 offline checks and ELF layout verification pass.

## Addressed for the current bounded, single-record milestone

- Implement stopped-acquisition RAM-to-flash persistence.
- Verify flash readback against original RAM samples.
- Implement recovery of committed records after reboot (supported nominal v1).
- Implement CAN retrieval with completeness/integrity checks: per-frame order,
  block/record CRC, counts, identity and exact bench comparisons. Dropped/reordered
  cases tested offline; no claim of mission integration with IVER CPU yet.
- Preserve recordings until verified transfer or explicit deletion: current
  implementation refuses overwrite/restart, dump/retry/reboot preserve recording.
  Retention rule is implemented conservatively. ACK and explicit recording
  deletion/reclamation commands remain unimplemented; this is not a reusable
  capture/transfer/reclaim cycle yet.
- Record acceptance results and commit validated changes to both repositories:
  satisfied for this checkpoint only, not final project acceptance or push.

## Partial or pending

- Interrupted programming/erase/corrupted records: missing-marker and corrupt-CRC
  fixtures plus completed erase passed. Actual loss of power in an active program
  or erase window not demonstrated; leave aggregate checklist item unchecked.
- Full-storage/overwrite/wear policies: reject/retain behavior defined, endurance
  reviewed; complete allocator, journal wear and reclaim handling not implemented.
- Continuous acquisition RAM/execution requirements documented but not validated.
- Acquisition timing/overruns during flash and CAN: not measured; nominal-only
  timestamps and stopped acquisition are explicit limitations.
- RTC hardware, UTC synchronization/mapping, drift/resynchronization: pending.
- Python decoding for measured acquisition metadata: current decoder marks nominal
  time and validates identity/integrity; measured acquisition timing is pending.

## Recommended next practical implementation

Verified-transfer ACK and explicit ID-bound recording CLEAR with durable intent
and interrupted-reclaim handling, then new-session restart without MCU reset.
This enables repeated captures and full-storage tests without debugger mass erase.
Keep real interrupted-program/erase acceptance open and plan a controllable
power-interruption test separately; do not equate simulated faults with physical
busy-window evidence. Do not erase the current recording until the next step's
explicit deletion procedure and verified host backup are established.

## Subsequent checkpoint: .240 lifecycle and storage policies

FLASH_LIFECYCLE.md records bench ACK retention/reboot, explicit clear, fresh-ID
capture/persist without reset and rejection of stale old-record clear; exact
second-recording RAM/flash comparisons passed. Lifecycle changes committed in
both repositories. STORAGE_POLICY.md now defines full-storage/overwrite/wear
policies; wear enforcement/leveling remains pending. The policy-definition
checklist item may be checked with that implementation qualification.

Continuous RAM/execution evaluation is documented in CONTINUOUS_LOGGING_EVALUATION.md.
The evaluation item is addressed; continuous firmware, measured acquisition
timing/gap reporting and overlapping flash operation acceptance are pending.

## .241 measured timing checkpoint (maximum-size test pending)

TIMING_MEASUREMENT.md records inspected acquisition-only and normal packed CAN
reports: 13551 and 10539 complete pairs, nominal 1 kHz, no observed ADC errors,
tolerance violations, estimated missing slots or reported CAN overruns. ACK,
clear and 1000-pair persistence timing were measured with acquisition stopped;
flash integrity and download passed. Whole persist duration was 749.765250 ms.
No continuous-flash acquisition performance is established. Keep the measurement
item open pending 4094-pair capture/cadence, RAM/flash comparison and storage timing
results. Physical busy-window power cuts and continuous overlap remain open.

Maximum-size update: .241 capture completed 4094/4094 at nominal 1 kHz with no
reported acquisition errors/interval violations. Stopped persistence measured
2402.929200 ms (36 erase/72 program calls), verified and downloaded. Exact ordered
RAM/flash index/P1/P2 comparison passed all 4094 pairs; saved binary integrity
validated. Maximum-size clean-reboot verification remains pending. The broad
continuous-flash timing/overrun item remains qualified/open; current results
cover acquisition-only, normal CAN, and deliberately stopped flash operations.

Clean-reboot update: .241 recovered COMMITTED with 4094 verified pairs. Post-reboot
reconstructed binary is byte-identical to pre-reboot; all 4094 ordered samples
match, identity/count/integrity validated. Bounded timing/storage checkpoint
passed. 'Measure acquisition timing and overruns during flash operations and CAN
traffic' is addressed for normal CAN and stopped-acquisition flash; continuous
flash overlap remains open and must not be marked validated. Current maximum-size
recording remains protected and unacknowledged. See TIMING_MEASUREMENT.md.
