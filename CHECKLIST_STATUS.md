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
