# Verified-transfer ACK, explicit clear and next capture

2026-11-02. Build 1008 / firmware 0.0.3.240. Implementation built and offline
checked; no hardware download, ACK or recording deletion performed in this step.

## Implemented lifecycle

Download/verify -> ack_flash(saved binary) -> ACKNOWLEDGED (copy retained) ->
clear_flash(same saved binary) -> durable DELETE intent -> erase owned data,
manifest last -> CLEARED -> flash_capture(new UUID). No MCU reset required.

Host opens the existing binary read/write without modifying bytes, validates
all supported recording blocks/counts/CRC/identity, and calls flush + os.fsync
before sending ACK. Failure to validate or flush sends no mutation. fsync is an
OS durability request, not a guarantee of an arbitrary storage device's power-loss
behavior. The agreed receiver boundary here is the local validated binary file;
IVER backseat durability semantics still need integration validation.

Commands bind target RecordingId and manifest CRC (which covers its stream CRC)
to a fresh request UUID. Firmware verifies the current flash recording again.
This is accidental-integrity/identity checking, not authentication. ACK does not
erase samples or mark reception alone as verification. Normal CLEAR requires a
matching durable ACK; explicit discard of an unacknowledged recording is NOT
implemented. File/record identity mismatch and stale commands are rejected.

## Journal schema and transaction

Use existing relative blocks 0/1, alternating committed snapshots, common v1
block envelope. Kind 4; count 1; flags NOMINAL_TIME_ONLY (journal timestamp fields
unused). Header ID equals target; sequence is low 32 bits of uint64 generation;
first slot/time zero, nominal period 1000, header reserved bytes zero. Generations
start at 1, strictly increase and never wrap. Equal-generation different snapshots
are ambiguous and read-only. No valid snapshot + nonblank journals is uncertain.

64-byte journal payload, relative offsets:

| Offset | Bytes | Value |
| --- | --- | --- |
| 0 | 16 | Target RecordingId |
| 16 | 4 | Target manifest block CRC |
| 20 | 2 | Operation: 1 ACK, 2 DELETE |
| 22 | 2 | Zero flags |
| 24 | 8 | Generation |
| 32 | 16 | Request UUID |
| 48 | 4 | Target DATA block count |
| 52 | 4 | Target sample-pair count |
| 56 | 8 | Zero reserved |

Payload padding FF; common CRC/CMIT rules unchanged. Counts bound erase footprint
and must agree with the supported stage-one capacity. This replaces the previous
unimplemented journal proposal's last 16 reserved bytes; DATA/MANIFEST and saved
.238/.239 recording binaries remain unchanged.

Before new snapshot, erase only inactive journal, leaving newest valid snapshot
intact. Program/verify body/CRC then marker; verify snapshot before acting on it.
A torn snapshot leaves prior committed journal authoritative. First-ever journal
write interrupted with no valid predecessor becomes JOURNAL_UNCERTAIN; original
recording remains readable, but mutation needs a future explicit metadata-service
procedure. Do not auto-erase unknown metadata to make progress.

CLEAR commits DELETE intent before erasing any recording data. Erase only its
recorded DATA pages (starting relative 9), verify FF, erase manifest page 2 last,
verify body empty. Unreferenced data (including scratch), foreign IDs, unsupported
media, or inconsistent footprint cause rejection. No mass erase or event/scratch
cleanup is implied. Oldest journal snapshot alone can be reused; newest deletion
proof remains. No first-recording sample words are reprogrammed to change flags.

Boot is read-only. Valid ACK restores ACKNOWLEDGED. Valid DELETE plus already
empty body restores CLEARED; partially reclaimed original becomes CLEAR_PENDING.
Repeat clear_flash with the matching saved binary to resume authorized erases.
If matching old manifest is absent/corrupt and DATA is nonblank, retain read-only:
it may be a later interrupted recording. Do not use an old deletion intent to
erase it. If all DATA is already erased and manifest erase was interrupted,
matching intent permits completing manifest reclamation explicitly.

CLEARED is logically available, not all-partition FF: DELETE proof remains in
journal. New capture requires body FF, safe journal state and new UUID different
from most recent lifecycle target. Journal generation continues across recordings.
A newer committed foreign-ID recording takes precedence over stale DELETE; stale
requests cannot erase it. A new retained RAM capture also blocks old lifecycle
requests. ACK cannot cancel pending DELETE of the same identity.

The implementation only remembers recent IDs through two journal snapshots;
UUID generation by the host provides long-term uniqueness. There is no unlimited
history of every ID ever used. All retries must supply matching recording evidence.
Duplicate ACK returns verified count without writing; duplicate CLEAR after
completion returns success without another erase. Interrupted clear retries may
re-erase authorized pages before completion; wear from retries must be budgeted.
Two-page journals remain an endurance hotspot, not a wear-leveled mission design.

## Commands and status

Capability 0x49500103. Commands 0x20..0x23 stage request UUID words; 0x24..0x27
stage target UUID words; 0x28 ACK / 0x29 CLEAR take manifest CRC as uint32 argument.
Full staging mask required per mutation; word arguments use existing big-endian
CAN framing and UUID RFC byte order. Nonzero IDs required. No arbitrary address
or erase range accepted. Host is a single control client; no multi-client locking.

ACK success returns sample count; CLEAR success returns zero. Errors add 8 stale/
mismatched identity, 9 uncertain/exhausted journal, 10 durable ACK required.
Timeout remains UNKNOWN outcome; query status instead of starting a new recording.
Flash API busy polling still lacks a firmware deadline; host 90 s is not one.

flash_status adds 6 CLEARED, 7 CLEAR_PENDING, 8 ACKNOWLEDGED.
flash_recovery adds 4 ACKNOWLEDGED, 5 CLEARED, 6 CLEAR_PENDING,
7 JOURNAL_UNCERTAIN. Recovery and status never erase. Snapshot classification
for ACK does not certify absence of unrelated data; CLEAR separately checks all
unowned blocks and rejects unexpected content. Scratch fixtures are rejected
once nonblank lifecycle journals exist by the original scratch guard.

## Offline validation / remaining gates

46 tests total: 33 host (including 11 lifecycle) + 8 format + 5 timer. Journal
prefix writes, interrupted clear at 4-byte boundaries, bad CRC/conflicting
snapshots, foreign/stale IDs, partial newer capture headers, receiver validation/
fsync and old-firmware gating covered. These are Python reference/host tests,
not execution of C flash erases or physical power-cut tests. TI Debug compile/link
and ELF layout pass. main.obj explicitly depends on both flash headers through
source-owned makefile.defs so command-line incremental builds cannot miss edits.
Hardware ACK/retention/reboot/clear/restart and actual in-flight interruption
remain pending. No automatic clear on ACK or host startup.

## Bench checkpoint: read-only first

Keep CCS Necessary Pages Only and 16 MHz crystal settings. Refresh, Clean/Build,
flash .240, resume, restart Python CLI. First run:

```text
version
flash_status
flash_recovery
verify_flash
dump_flash lifecycle_before_ack_01
```

Expect .240 and same 1000-pair COMMITTED recording. Compare download to original
before lifecycle mutations. Do not run flash_capture to replace a retained record.

After read-only preservation passes, use the actual validated binary path:

```text
ack_flash Data/recovery_after_scratch_01_20261102_135834_871995/recording.bin
flash_status
flash_recovery
verify_flash
dump_flash lifecycle_after_ack_01
```

Expect ACKNOWLEDGED, 1000 pairs, identical data. Repeat ACK and verify, optionally
clean reboot: ACKNOWLEDGED should persist, RAM zero. This has NOT deleted data.

Only when deliberately ready to remove this recording from the MODULE:

```text
clear_flash Data/recovery_after_scratch_01_20261102_135834_871995/recording.bin
flash_status
flash_recovery
```

Expect CLEARED, RAM zero. Local binary remains. Repeating same CLEAR is a no-op.
verify_flash now fails because that recording was intentionally deleted; this is
expected, not a corrupt-record recovery success. No new capture has started yet.

Then flash_capture 1000, wait 2 s, flash_status -> RAM_READY. Dump RAM to a NEW
CSV, persist_buffer, verify_flash, dump_flash with a NEW stem, compare pairs.
New UUID differs. Attempting clear_flash with OLD binary must reject stale ID
and leave new data intact; repeat verify/download. A second ACK/clear can use
only the new verified binary. No reset between these normal cycles.

Do not cross off physical interrupted-program/erase testing or continuous
acquisition on this evidence. No lifecycle bench results recorded yet.

## Hardware checkpoint: .240 upgrade preserved recording

2026-11-02: user loaded .240, observed COMMITTED / RAM zero, recovery COMMITTED /
1000, MCU verification passed. Downloaded
MasterPython/Data/lifecycle_before_ack_01_20261102_141712_962003/.
Agent compared CSV and reconstructed binary byte-for-byte to original .238 capture:
both identical. Host verified_file validation/flush passed; UUID remains
608e18dd-b8ca-49dd-ac3f-36c82bb02629, 1000 pairs. No ACK or CLEAR sent yet.

## Hardware checkpoint: ACK retains recording

User ACKed saved lifecycle_before_ack_01 binary. Firmware reported ACK durable,
flash_status and flash_recovery ACKNOWLEDGED, verify_flash passed 1000 pairs.
Download lifecycle_after_ack_01_20261102_141830_694608 under MasterPython/Data:
agent compared samples.csv and reconstructed recording.bin to original .238
capture; both byte-identical. ACK retention passes; ACK survival across reboot,
explicit clear and new-session restart remain to be checked on hardware.

## Hardware checkpoint: ACK reboot, clear and second capture

User power-cycled .240: ACKNOWLEDGED restored with RAM zero; 1000 flash pairs
verified. Explicit clear using saved original binary succeeded, status/recovery
CLEARED and RAM zero. Without another reset, flash_capture 1000 accepted new UUID
d1805d10-f4a1-405c-a62a-b6571318c110 and auto-stopped RAM_READY/1000.
RAM dump complete/sequence check passed; second persistence/readback verification
passed; state COMMITTED/1000.

Artifacts: MasterPython/Data/lifecycle_second_ram_01.csv and
lifecycle_second_flash_01_20261102_142246_462870/{samples.csv,recording.bin}.
Agent independently validated binary CRC/counts and compared all ordered tuples:
1000/1000 identical across RAM CSV, flash CSV and binary, with new UUID matching.
The 8 queued CAN frames drained before RAM dump did not cause a dump sequence
failure. Stale-old-record clear protection test remains pending on hardware.

## Hardware checkpoint: stale clear rejected

User issued clear_flash with original recording UUID
608e18dd-b8ca-49dd-ac3f-36c82bb02629 while second recording
 d1805d10-f4a1-405c-a62a-b6571318c110 was committed. Module rejected stale or
mismatched recording identity; state remained COMMITTED with RAM 1000;
verify_flash passed 1000. Download
MasterPython/Data/lifecycle_after_stale_clear_01_20261102_142350_396120/.
Agent compared samples.csv and reconstructed recording.bin byte-for-byte against
lifecycle_second_flash_01: both identical. Stale-request protection passes.

Basic reusable lifecycle bench checkpoint passes: validated transfer ACK retains
data; ACK survives reboot; explicit clear succeeds; fresh-UUID capture/persist
works without resetting; stale old clear cannot delete new committed data.
Duplicate ACK/clear, lifecycle power interruption, journal exhaustion, multi-client
operation and real flash-busy-window power cuts remain untested on hardware.
No commit/push made as part of recording this result.
