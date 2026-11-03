# Recovery and fault-test checkpoint

2026-11-02. Build 1007, firmware 0.0.3.239. No hardware download or scratch
operation performed by this implementation step. Current retained 1000-pair
recording must remain intact.

## Implemented recovery scope

Boot performs read-only bounds, block CRC, nominal v1 semantic, count, identity,
sequence and recording-stream CRC validation. It restores COMMITTED and the
recording UUID if a complete supported recording validates; RAM remains empty.
Other nonblank storage is protected, not auto-formatted. This is single-record,
nominal-time v1 recovery, not journal replay or general multi-record recovery.

`flash_recovery` scans the actual full partition:

- EMPTY: all bytes observed FF. After a real interrupted erase this observation
  alone does not establish reliable electrical erase; re-erase/verify disposable
  space before reuse. Physical interrupted-erase history is not inferable from
  FF bytes without durable intent metadata (not implemented).
- COMMITTED: validated recording, no other programmed blocks.
- COMMITTED_WITH_QUARANTINE: valid recording plus unreferenced programmed blocks.
  Valid recording remains available; extra blocks never automatically reclaimed.
- INCOMPLETE_DAMAGED_OR_UNSUPPORTED: no complete supported recording validates.
  Cannot distinguish torn writes, corruption or unsupported media with certainty.
  Do not export as complete, erase automatically or restart acquisition.

This detects/rejects damaged captures; it cannot repair corrupted samples or
promise to recover uncommitted RAM. ACK/delete journals and interrupted authorized
record-deletion recovery are not implemented. Rejecting corrupt metadata is not
proof of comprehensive journal recovery.

## Offline evidence

Synthetic image tests cover 4-byte prefix-program boundaries through both data
blocks and final manifest, all 24576 single-bit mutations across those three
blocks, partial/scattered erase patterns and unsupported media. Every incomplete
or mutated referenced capture is rejected. Damage in scratch block 7 leaves
committed recording valid but flags extra block quarantine. A disposable in-memory
copy of the real 1000-pair bench recording also passes preservation scenarios;
the original file is never modified. These are models, not flash physics tests.

22 host tests + 8 format tests + 5 timer tests pass. TI Debug build/link and
ELF layout checks pass. C validation compiled but is not executed by the Python
model. Hardware/C parity and power-failure behavior remain to test.

## Preserve the recording before firmware upgrade

Verified existing binary SHA256:
9b5560e402f721db6377d431f9b9850e58a29e6b0915c90ebeeca22163633ac8.
Existing copies are under MasterPython/Data/flash_persist_01_20261102_133214_082097
and flash_after_reboot_01_20261102_133752_933305.

Before loading the new image, inspect the active CCS launch/flash erase policy.
TI documents Project Properties > Debug > Flash Settings, Program Load Settings:
Necessary Pages Only erases allocated program pages; Entire Flash erases logs too.
[TI CCS debug guide, TM4C flash settings](https://software-dl.ti.com/ccs/esd/documents/users_guide_10.3.0/ccs_debug-main.html).
No CCS setting is assumed from the saved project: it does not contain an explicit
verified erase policy. Confirm the actual launch in use, keep logging addresses
0x20000..0x3FFFF out of erase ranges, and retain the host backup. Do not use
Do Not Erase as a substitute for a valid program-page update. Preservation across
firmware loading is itself verified afterward; it is not guaranteed by the linker.

## First checks after a preservation-aware upgrade

Restart Python CLI; flash/resume expected version 0.0.3.239. Do not capture/clear.

```text
version
flash_status
flash_recovery
verify_flash
dump_flash recovery_baseline_01
flash_scratch inspect
```

Expect COMMITTED with RAM zero, recovery COMMITTED and 1000 pairs. Compare baseline
CSV/binary against the saved .238 recording. Scratch should be erased initially.
If recording missing/damaged or scratch unexpectedly nonblank, stop and inspect.
No scratch writes are necessary for these first read-only checks.

## Controlled scratch fixtures, after baseline is verified

A separate bench command modifies ONLY relative block 7, address 0x21C00..0x21FFF.
It requires a valid supported recording, no acquisition, and blank unrelated
journal/event/spare/unallocated blocks. It never accepts an address from the host.
Its physical address is disjoint from manifest block 2 and data blocks >=9.
It re-verifies the protected recording/manifest after the operation. Every call
can pause CAN and preserves the prior interrupt-mask state. No normal boot,
capture, persist or recovery path calls the fixture. Never use it with a future
allocator that assigns block 7 without revisiting the guard/layout.

1. `flash_scratch torn`: erase disposable block, program body/CRC, omit marker.
   `flash_scratch inspect` must report invalid/uncommitted.
   `flash_recovery` must show COMMITTED_WITH_QUARANTINE/1000.
2. `flash_scratch commit`: writes only previously-erased marker after body check.
   Inspect must show valid fixture envelope. Recovery still quarantines it:
   an unreferenced valid block is not added to the recording.
3. `flash_scratch corrupt`: erase disposable block and write a fixture changed
   BEFORE programming but with its old CRC and valid marker. Inspect rejects CRC.
   No already-programmed word is programmed twice to create corruption.
4. `flash_scratch valid`: erase/write complete fixture for later erase testing.
5. `flash_scratch erase`: erase only disposable block, verify FF. Recovery returns
   COMMITTED, protected recording still 1000 pairs. Download and compare again.

These test actual writes/CRC rejection on hardware but DO NOT physically interrupt
an active program/erase. A missing marker is deliberately staged, not a brownout.
They must not be logged as completed interrupted-program/erase tests.

## Actual power interruption: separate pending test

After scratch fixture validation and baseline backup, plan supervised power cuts
while ONLY scratch programming/erase is active. No mass erase, debug unlock,
nonvolatile configuration commit, or protected-record erase. All power sources
must actually be removed; debugger/USB may otherwise back-power the board.
Reboot and read recovery plus scratch bytes; compare protected recording again.

Ordinary page erase/program may finish before a manual cut. A changed or erased
scratch block does not prove interruption occurred in the busy window. Without
instrumented supply control or independent busy-window evidence, record the
attempt as timing-unconfirmed. log_test_phase (1 before erase, 2 after erase,
3 before body, 4 after body, 5 before marker, 6 done) is a diagnostic snapshot,
not durable evidence after power loss. Halting just before an operation and
removing power tests the boundary, not an interruption during the operation.
Do not claim fully addressed power-cut acceptance from the present equipment.

No record-repair/reclaim command for arbitrary media is exposed. If a scratch
operation leaves unknown bytes, the fixed block can be deliberately re-erased
only while its guard validates. If protected recording is invalid, stop rather
than weakening that guard. Physical tests and fault outcomes will be added here.

## Hardware checkpoint: firmware update preservation

2026-11-02: user changed CCS program-load erase policy from Entire Flash to
Necessary Pages Only and crystal setting from 8 to 16 MHz, then loaded .239.
CLI version .239; boot state COMMITTED, RAM zero; flash_recovery COMMITTED/1000;
verify_flash passed. Download recovery_baseline_01_20261102_135434_905570 under
MasterPython/Data. Agent compared samples.csv and recording.bin with original
.238 download: both byte-identical. This update preserved the existing capture.
No scratch operations or real power cuts recorded at this checkpoint.

## Hardware scratch fixture results: 2026-11-02

User ran controlled scratch operations on firmware .239:

- Initial inspect: erased bytes (scratch_20261102_135520_488252).
- torn: body/CRC programmed without marker; host rejected incomplete block;
  recovery COMMITTED_WITH_QUARANTINE/1000; protected recording verified.
  Saved scratch_20261102_135550_620655.
- commit: marker-only completion accepted VALID_FIXTURE_ENVELOPE; recovery still
  quarantined the unreferenced block; protected recording verified.
  Saved scratch_20261102_135709_155675.
- corrupt: deliberate stale CRC with valid marker rejected as Block CRC mismatch;
  recovery quarantined it; protected recording verified.
  Saved scratch_20261102_135747_671874.
- erase: scratch returned to all FF; recovery COMMITTED/1000, protected recording
  verified. Saved scratch_20261102_135818_772359.

Agent independently inspected these local MasterPython/Data scratch artifacts:
blank/erased both exactly 1024 FF bytes; torn body's CRC valid, marker erased;
torn and completed bodies identical through byte 1019; completed envelope valid;
corrupt fixture has CMIT but CRC mismatch.
Final download recovery_after_scratch_01_20261102_135834_871995: samples.csv and
reconstructed recording.bin byte-identical to original .238 capture. Binary hash
9b5560e402f721db6377d431f9b9850e58a29e6b0915c90ebeeca22163633ac8.

Missing-commit rejection, marker completion, corrupt-record rejection, quarantine,
normal scratch erase and preservation across these operations pass. No actual
power loss during an active program/erase was performed or demonstrated.
Interrupted-program/erase physical acceptance remains pending; do not mark the
entire fault-interruption checklist complete based on these fixtures.
