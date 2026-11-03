> Time/identity proposals are superseded where they conflict by [TIME_MODEL](TIME_MODEL.md).
> Existing v1 bytes/nominal-time semantics remain unchanged; new measured metadata requires versioning.

> Detailed specification/evaluation; distinguish proposed policy from implemented
> behavior. For current validation and remaining work, see [current status](../CURRENT_STATUS.md).
> Commands and source paths assume the repository root as working directory.

# Acquisition identity, counters, and gap reporting

Definition agreed for implementation planning, 2026-11-02. This is a proposed
contract, not a claim that the current firmware or CLI implements these fields.
The current on-board timer validation is recorded in TIMER_VALIDATION.md.

## Scope and identity

A sample means one P1/P2 pair from one ADC sequence. P1 and P2 are sequential
conversions, each measuring its own differential pressure; they share an
acquisition identity but not an exactly simultaneous conversion instant.

Use a 128-bit recording/session ID supplied by the host (UUID) before capture.
Every accepted new capture requires a new ID. Repeated starts while capturing
are rejected without resetting counters or deleting data. Stop freezes the
session; dump, retry, acknowledge, and persist retain the same ID and counters.
Restart creates a new ID. A disconnected deployment must obtain IDs from a
separately defined durable allocator before supporting autonomous capture;
reboot must never silently reuse a volatile counter as a globally unique ID.
Device identity must be provisioned separately; DID0/DID1 are not unique IDs.
The earlier assumed factory unique-ID source is withdrawn (see record-format qualification).
A recovered recording retains its stored ID; a new capture cannot inherit it.

Firmware retains the most recent ID in RAM until explicitly replaced. Reboot
invalidates RAM identity/status. Durable recording recovery will come with the
flash record format; no new metadata flash writes are introduced for this step.

## Time and acquisition index

GlobalTimer remains a uint32 serviced-SysTick counter used for scheduling.
It wraps and may lose elapsed time while halted or interrupts are blocked.
It is not the authoritative acquisition clock, a successful-sample count, or UTC.

Introduce boot-relative uint64 monotonic MCU time in microseconds from a
free-running hardware timer. Exact peripheral, rollover extension, clock source,
and debug-halt behavior must be implemented and validated before claiming this
time is available. Capture trigger time immediately around the software ADC
trigger; report timestamp resolution and instrumentation offset. This is a
trigger timestamp, not an assertion that both ADC conversions occur then.
Counters/timestamps sampled together require a short critical section; volatile
alone does not make multiword reads coherent on the Cortex-M4.

The first accepted session trigger establishes t0 and nominal slot index 0.
For subsequent serviced triggers:

    slot = max(previous_slot + 1, floor((trigger_us - t0_us) / 1000))

Use uint64 slot indices and MCU timestamps. Store actual trigger_us as well as
slot: nominal slots do not replace actual timing. A difference greater than one
identifies skipped nominal slots. This is an estimate of unsatisfied schedule
slots, not a hardware count of unobserved ADC triggers. Timing phase/jitter may
change slot assignment; expose actual intervals and qualify the estimate.
A hardware-triggered ADC design may later provide stronger slot accounting.

## Per-session counters

All totals use uint64, reset only on acceptance of a new session, and freeze on
stop. Index/total overflow ends capture with an explicit error rather than reuse.
Boot-lifetime diagnostics, if added, are separate from session totals.

| Field | Definition |
| --- | --- |
| trigger_attempts | ADC triggers issued while capture is active; increment before trigger |
| samples_acquired | Complete, valid P1/P2 pairs read from ADC |
| adc_timeouts | Trigger attempts with no completed pair before timeout |
| adc_errors | Other detected invalid/failed pairs; do not also count as timeout |
| estimated_unserviced_slots | Sum of skipped nominal slots from trigger timing; estimate only |
| gap_events | Number of recorded acquisition/timing discontinuity events |
| ram_overwrites | Valid retained sample pairs evicted by the current circular RAM policy |
| ram_rejected_samples | Acquired pairs not accepted into RAM; distinct from eviction |
| tx_frames_dropped | Complete live frames rejected because the pending queue is full |
| tx_failures | Transmit operations whose completion cannot be confirmed; delivery unknown |
| unpaired_tail_samples | Valid final samples not emitted by the two-sample live packer at stop |

When no attempt is in flight:
trigger_attempts = samples_acquired + adc_timeouts + adc_errors.
RAM count is retained records, not samples_acquired. RAM eviction and live CAN
loss do not imply acquisition failure. A dump retry does not change acquisition
totals. Count a stored event once even if several flags describe it.

A timed-out ADC sequence must be quiesced/recovered before issuing a new trigger;
a late result must not be attributed to a later slot. Recovery time is included
in subsequent timing-gap evidence. Return to idle must not hide an in-flight
attempt or silently clear its error.

## Sample and event records

Each retained pair carries session identity (in its block/header), slot_index,
trigger_us, P1/P2 ADC counts, and quality flags. Never reconstruct these solely
from dump row number. Later record-format work may compress consecutive slots
into blocks but must preserve missing slots and discontinuities exactly.

A gap event contains session ID, reason flags, previous/current slot and trigger
time, and known/estimated missing count with explicit validity. Reasons include
ADC_TIMEOUT, ADC_ERROR, LATE_TRIGGER, CLOCK_DISCONTINUITY, and DEBUG_HALT.
DEBUG_HALT is used only if positively identified; an unexplained time gap is
not automatically classified as debugger activity. If the timer stopped too,
mark elapsed time/missing count unknown. Absence of a flag is not proof of
complete detection. Event-log overflow increments a separate lost-events counter
and sets a sticky integrity flag. Start with zero permitted silent loss;
allowable physical timing jitter remains a separate requirements item.

RAM eviction, TX queue overflow, TX failure, and dump transport gaps have separate
integrity flags. Preserve known totals even if detailed event capacity runs out.

## CAN and host reporting

Existing packed live frames consume all eight bytes; they cannot acquire a
reliable counter/timestamp by relabeling the current data. Implement a versioned
protocol with explicit capability discovery. Legacy streaming/dumps remain
labeled as lacking acquisition IDs/time, and their losses remain undetectable.
Do not silently reinterpret frame type 0x06 or the dump's 16-bit playback index.

New sample transfer blocks carry session ID, first slot/time, record count,
format version, block sequence, gap/quality information, and integrity checks.
Their exact fragmentation, payload layout and CRC are deferred to the record
format checklist item. Sequence information must bind to the data block;
unassociated periodic counter frames cannot locate a dropped live sample.

Expose a coherent session-status snapshot on request and at stop: ID, active
state, all counters, first/last slot/time, retained range, sticky flags, timing
validity and nominal rate. Multi-frame replies share a snapshot token and are
reassembled/validated as a unit, never mixed from different instants/sessions.
Status retrieval/dump is read-only and never clears errors or recordings.

Host export uses explicit columns: SessionId, AcquisitionIndex, MCUTriggerUs,
HostReceiptTime, Pressure1, Pressure2, QualityFlags. UTC mapping is separate and
includes source/uncertainty; do not claim UTC before synchronization. Missing
indices appear as plotted gaps, not interpolated data or a compressed timeline.
A new session starts a new trace. Host detects transfer gaps/duplicates/reordering
independently of MCU acquisition gaps. A CAN transmit success is not verified
host receipt or recording-transfer acknowledgement.

## RAM and implementation gates

Current array: 4094 * 4 = 16376 bytes, within 32 KiB SRAM with other allocations.
A naive 24-byte record would consume 98256 bytes at that capacity and cannot fit.
Choose bounded blocks plus gap events or reduce capacity from the actual linker
map before adding metadata. Report actual capacity to the host. Event storage,
TX queues, stack, and growth margin all count against SRAM. No second full-size
sample array may be assumed to fit. Preserve the current acquisition ISR budget.

Implement in stages: (1) coherent counters/session state and TX queue accounting,
(2) validated monotonic hardware time and gap events, (3) versioned export and
host checks. Do not call the reporting feature complete with debugger-only
variables or estimated host timestamps. Persistent ID retention is implemented
with the later flash commit/recovery workflow.

## Required acceptance tests

- Start/stop/repeated start, rejected duplicate IDs, new-session reset, read-only
  status/dump, coherent snapshots during streaming, and retained stop statistics.
- Normal acquisition: completed-attempt accounting identity, distinct slot/time
  values, no ADC errors, acquired/retained/evicted totals agree.
- Inject one ADC timeout and a late completion: exactly one failed attempt,
  correct slot/event, no stale result labeled as the next successful sample.
- Delay service across multiple slots: gap visible, estimated count qualified,
  actual measured interval retained. Test across timer rollover and debug halt.
- Slow CAN consumption: queue-full losses counted without overwriting the
  published pending frame; separate RAM/acquisition totals remain correct.
  Current single pending frame is written even while pending and its main-loop
  copy is unsynchronized: fix that ownership race before trusting TX counters.
- Stop after an odd successful sample count: report the live unpaired tail;
  preserve that pair in RAM. Dropped/reordered/duplicate transport blocks detected.
- Fill/overflow gap storage: sticky integrity flag and lost-events total survive.
- Reboot: no false continuity; recovered committed IDs retained once flash exists.
- Build/link SRAM budget plus bench rate/timing checks before flash writes.

Status: definitions complete; firmware/host implementation and acceptance tests
remain pending. Exact jitter tolerance and autonomous ID allocation are open.

Timing thresholds and schedule-boundary qualifications are defined in [acquisition timing requirements](ACQUISITION_TIMING.md).
