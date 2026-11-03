> Detailed specification/evaluation; distinguish proposed policy from implemented
> behavior. For current validation and remaining work, see [current status](../CURRENT_STATUS.md).
> Commands and source paths assume the repository root as working directory.

# Recording lifecycle and command contract

2026-11-02. Defined behavior for implementation; current firmware/CLI do not yet
implement this state machine. No new command IDs or binary messages assigned.

## Scope and identity

Stage one supports one protected recording at a time: bounded RAM capture,
stop, persist, verified dump, acknowledge, explicit clear, new capture. It is
intentionally discontinuous during persistence/transfer. Do not use MCU reset
as the normal next-recording operation.

RecordingId is the host-supplied session UUID from ACQUISITION_ACCOUNTING.md.
RunId plus SegmentNumber groups later 10-30 s segments; RecordingId remains
unique for each segment. DeviceId identifies the sensor independently. Stage
one can omit the optional run grouping without changing RecordingId semantics.
No volatile boot counter is a globally unique ID. New IDs cannot erase old data.

## States and invariants

| State | Meaning | Acquisition |
| --- | --- | --- |
| RECOVERING | Scan/validate existing metadata and commits after boot | Disabled |
| EMPTY | No retained recording; capacity/storage health reported | Disabled |
| CAPTURING | New recording accumulating in bounded RAM | Enabled |
| STOPPING | Stop future triggers and settle/recover an in-flight sequence | Stopping |
| RAM_READY | Frozen recording/counters in RAM, not durable | Disabled |
| PERSISTING | Serialize/erase/program/readback/commit within reserved partition | Disabled |
| COMMITTED | Valid durable recording, not yet transfer-acknowledged | Disabled |
| DUMPING | Read-only transfer from RAM or committed flash; remember origin | Disabled |
| ACKNOWLEDGED | Receiver verified recording; local copy still retained | Disabled |
| CLEARING | Authorized deletion/reclamation transaction | Disabled |
| RECOVERY_REQUIRED | Storage status ambiguous/corrupt or operation failed | Disabled |

Keep recording presence, durability, transfer verification and health as explicit
status fields, not inferred just from a state name. Faults include preserved
record references and last operation outcome. An error must not automatically
clear a record, reset counters or silently resume acquisition.

EMPTY is a logical state, not a claim every flash byte is erased. A quarantined
partial allocation cannot be reused until explicitly reclaimed safely. No state
may erase firmware, an unrelated record or the only valid metadata copy.
The current circular overwrite mode remains a separately advertised legacy
workflow; the new bounded capture mode must not silently wrap RAM. It stops at
capacity, freezes the valid prefix and reports stop_reason=RAM_FULL. It is not
an acquisition-failure count for samples never requested after capture ends.

## Command transitions

| Command | Allowed state | Successful result | Rejection/failure behavior |
| --- | --- | --- | --- |
| CAPTURE(id, limit) | EMPTY | CAPTURING; initialize ID, counters and epoch atomically | Reject reused ID, invalid/oversized limit or unsafe storage; no deletion |
| STOP | CAPTURING | STOPPING then RAM_READY | Repeat in RAM_READY is a no-op; already stopped elsewhere returns status |
| PERSIST(id) | RAM_READY with nonzero valid pairs | PERSISTING then COMMITTED after readback and final validation | Preserve RAM; failed/uncertain allocation is quarantined; report error |
| DUMP(id, offset) | RAM_READY, COMMITTED, ACKNOWLEDGED | DUMPING then return to exact origin state | Retry/resume allowed; interruption does not change ownership or delete |
| ACK(id, integrity evidence) | COMMITTED | ACKNOWLEDGED after durable ACK entry verified | Wrong ID/count/digest rejected; matching repeated ACK is idempotent |
| CLEAR(id, explicit discard authorization) | RAM_READY, COMMITTED, ACKNOWLEDGED | CLEARING then EMPTY when deletion/reclamation safely recorded | Reject active writer/dump/capture; never act on unspecified/all IDs |
| RESTART(new id, limit) | EMPTY | Same as CAPTURE | Else return RECORDING_RETAINED/BUSY; does not imply CLEAR |
| STATUS | Any state if CPU can service it | Coherent snapshot; no changes | During flash stall, host may receive no immediate response |

CAPTURE during CAPTURING: same request token/parameters returns the original
operation status; a distinct start returns BUSY, never resets the recording.
A repeated STOP during STOPPING returns its operation status. STOP never deletes.
Zero-pair STOP creates an explicit empty RAM result; allow DUMP/status or explicit
CLEAR, but reject PERSIST(NO_SAMPLES) for stage one. Best-effort degraded captures
can be persisted with quality/gap flags; never label them loss-free.

DUMP from RAM is explicitly VOLATILE. Stage-one ACK requires a committed flash
copy; exporting RAM does not authorize its implicit deletion. A RAM copy may
only be discarded by explicit CLEAR. Dump success means transfer completed,
not that the receiver verified/durably stored it. During DUMPING, permit STATUS,
matching duplicate request and CANCEL_DUMP; reject state-changing commands BUSY.
Cancel/transport timeout returns to origin, preserving all recording data.

ACK evidence includes ID, format version, final sample/block counts and the
recording integrity value from its committed manifest. Receiver sends ACK only
after the agreed durable-storage boundary is met. Firmware can validate evidence,
not independently prove remote disk durability. ACK marks verified only: stage
one requires explicit CLEAR to reclaim even an acknowledged recording.

CLEAR on an unacknowledged recording is an explicit discard with a recorded
reason; no hidden permission inferred from START, resize, dump, disconnect or
reboot. A matching repeated CLEAR reports already cleared and never deletes a
new recording. If uncertain, return UNKNOWN_OPERATION and require STATUS rather
than apply an old deletion to current storage. ID checks include generation/
manifest identity to prevent stale commands acting on reused allocations.

## Transaction replies, concurrency and retries

Use versioned capability discovery; do not overload legacy start/dump or treat
read_flash (currently a RAM dump alias) as evidence of persistence support.
Each mutating request carries a request token and RecordingId. Reply ACCEPTED
with an operation token before long work when possible, then expose final
SUCCESS/FAILED/UNKNOWN via status/completion. Acceptance is not durability.
Serialize state transitions in the main loop; use short critical sections for
ISR handoff and coherent counters. Never make the ISR erase/program or wait on
host storage. Reject or queue conflicting operations explicitly, with bounded
queue capacity and reported queue overflow.

No response within host deadline means outcome unknown, not permission to erase,
retry blindly or restart. Query status and recover operation by ID. Runtime
retry cache is bounded; persistence/recovery must derive idempotence from durable
identity/manifest/journal, not claim tokens survive reboot automatically.
Future wire formats must provide coherent multi-frame snapshot IDs and integrity.

## Stop, persistence and power-loss rules

STOP disables new acquisition triggers; it is more than smStopped (which currently
still triggers ADC). Resolve in-flight conversion, count final complete pair or
explicit failure, flush/finalize metadata and report any live unpaired tail.
Global scheduling interrupts may continue but must not produce new ADC attempts.

PERSIST checks bounds and free-safe capacity before modifying flash, writes only
unallocated/reclaimable blocks, verifies content, and writes commit markers last.
A recording becomes COMMITTED only when its final manifest and all referenced
blocks validate. Commit details/CRC/journal layout are deferred to record format;
block commit alone does not make an entire recording complete. Do not free the
RAM source on write success until recovery-critical verification succeeds.

Boot never auto-formats unexpected nonblank media. Recover valid committed data
as COMMITTED or ACKNOWLEDGED using validated durable state. Interrupted/incomplete
records are distinguished from empty; preserve committed prefixes for diagnostic
salvage if supported, but do not call them complete captures. RAM-only captures
are lost on power loss; report reset reason/volatile-loss uncertainty instead of
inventing a recoverable record. Do not automatically restart capture on boot.

Interrupted ACK: retain as unacknowledged if durable proof is missing. Interrupted
CLEAR: recover its durable deletion intent before completing authorized erases;
never touch another ID. Do not acknowledge deletion completion before required
reclamation checks pass. Retry/recovery must not erase committed recordings as
part of repairing unrelated metadata. Unknown format/corruption enters
RECOVERY_REQUIRED with read-only diagnostics, not automatic erase.

## Later continuous pipeline (separate milestone)

Acquisition state and storage/transfer state become independent: RAM blocks move
FREE -> FILLING -> SEALED -> STORED/TRANSFERRED -> VERIFIED -> FREE. A segment
may span blocks and have its own ID; segment closure need not pause acquisition.
This does not require both flash and CPU transfer before reuse unless retention
policy explicitly says so: determine authoritative durable owner for each block.
Do not free the only retained copy before required receiver verification.
Explicitly configure any future automatic reclamation of ACKNOWLEDGED segments;
it is not implied by stage-one ACK semantics. Preserve all unacknowledged copies.

Backpressure with no safe space stops capture or rejects samples with explicit
counts/reasons; it cannot preserve indefinite acquisition with finite buffers.
Flash interference, queue ownership, coherent timing/counters, reboot recovery
and host protocol must be validated before enabling this mode.

## Acceptance tests for implementation

- Empty capture, normal limit stop, manual stop, repeated start/stop and oversized
  request: no silent RAM wrap, ID reset or loss of old recording.
- Persist success/readback and failure at each block/CRC/final-manifest commit;
  preserve source and old valid recordings. Retry after unknown result is safe.
- Dump RAM/flash, cancel, disconnect and repeat: identical data, no deletion.
- Wrong ACK evidence rejected; successful durable ACK and duplicates retain copy.
- CLEAR explicitly authorized, wrong/stale ID rejected; power-loss recovery of
  deletion intent never reclaims a newer recording.
- Reboot with empty/unformatted/unknown/corrupt/partial/committed/acknowledged
  media; no accidental auto-format or auto-start, no false complete recording.
- Conflicting commands return BUSY without corruption; status coherent under ISR
  activity. Flash stall host timeout handled as unknown outcome.
- Full capacity/backpressure, event overflow and duplicate/reordered wire requests
  reported. Baseline pressure/stream/dump tests remain for legacy compatibility.

Status: lifecycle defined; implementation and fault-injection acceptance pending.
See RECORDING_REQUIREMENTS.md, FLASH_REVIEW.md and ACQUISITION_ACCOUNTING.md.
