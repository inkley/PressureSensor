> Detailed specification/evaluation; distinguish proposed policy from implemented
> behavior. For current validation and remaining work, see [current status](../CURRENT_STATUS.md).
> Commands and source paths assume the repository root as working directory.

# Recording requirements worksheet

2026-11-02. User requirements captured below; mission duration and external
storage interface/capabilities remain open. This is a requirements/design document, not
an assertion that flash logging or gap reporting is implemented.

## Verified implementation constraints

- Nominal 1000 P1/P2 pairs per second; current raw payload 4 bytes per pair.
- Current RAM capacity 4094 pairs = 4.094 seconds of nominal payload. Live
  streaming can run longer because this buffer overwrites its oldest entries.
  That is current behavior, not the desired preserve-until-verified policy.
- Linker reserves 0x20000..0x40000: 131072 bytes for future logging.
  At 4000 payload bytes/s the upper bound is 32.768 seconds, before record
  headers, time/counter metadata, CRC, commit markers and recovery spare space.
- Stage-one capture/stop/persist cannot retain an entire long capture merely by
  reserving more flash: RAM is the staging limit until background storage or
  another explicit batching workflow is implemented. New sample metadata may
  reduce RAM duration. Final capacity requires the selected record format/map.

## Policies inherited from the project request

- Preserve a valid committed recording until verified transfer or explicit
  deletion. Dump start, dump completion, link disconnect, reboot, and new
  capture request are not deletion authorization.
- Acknowledge verified transfer using recording identity and integrity evidence.
  Whether verified acknowledgement immediately reclaims space or only marks
  the recording reclaimable is still a user policy decision; default proposal
  is mark verified, then explicit clear for stage one.
- When storage cannot fit the requested capture, reject capture with a capacity
  report or stop before destroying retained data; never silently overwrite an
  unacknowledged committed recording.
- UTC retention and flash retention are separate: loss of UTC mapping must not
  invalidate otherwise valid monotonic recording data.

## User-selected operating objective

Application: differential-pressure measurements on an underwater vehicle to
characterize its hydrodynamic pressure/forcing envelope. P1/P2 remain independent
pressure channels; conversion to physical force requires a separate geometry
and calibration model.

User selected 10-30 seconds per capture and best effort provided all detected
gaps are reported. Intended operation repeats acquire/store/buffer/transfer and
restart, continuously if possible. Capture frequency, total mission duration,
retention time and destination interface specifications are not yet known.

Working design: 10-30-second logical segments over an ongoing acquisition
pipeline, not necessarily a pause every segment. Target 100% acquisition duty
while recording is enabled, subject to explicit gap/overrun reporting; this is
an objective, not a measured guarantee. Avoid MCU resets in the normal cycle;
release only verified data and start the next segment with continuity metadata.
Segment IDs should include a run ID and increasing segment number so intentional
segmentation is distinguishable from reboot or discontinuity. Reconcile that
hierarchy with the session identity contract when implementing the protocol.

The user's best-effort policy permits degraded recordings with reported gaps;
quality/status must distinguish these from loss-free runs. There is no selected
maximum acceptable gap or loss fraction yet. Report measured gap duration,
reason, known/estimated loss count and uncertainty. Undetectable gaps remain a
known limitation until the reporting implementation is validated. Retain the
provisional +/-10 us target as a quality criterion, not grounds to silently
delete an otherwise useful best-effort recording.

## Requirements still to settle

| Requirement | Pending input |
| --- | --- |
| Stage-one capture duration | Bounded by actual RAM capacity; shorter proof capture explicitly separate from 10-30 s deployment objective |
| Deployment capture duration | Selected: 10-30 s logical segments, repeated during the run |
| Duty cycle | Continuous if possible; 100% target while enabled, mission duration unknown |
| Retention capacity | Number of recordings or total retained duration before download |
| Retention time | Required unpowered storage duration and environmental range |
| Autonomous operation | Capture after reboot without host; trigger/start policy |
| Between-recording gaps | Allowed save/erase/download pauses and maximum duration |
| Within-capture gaps | Selected: best effort with all detected gaps reported; maximum gap/loss budget unspecified |
| Full storage | Stop/reject versus explicitly authorized overwrite of verified recordings |
| Verified transfer | Mark verified only versus automatically reclaim space |

## Recommended staged implementation

One bounded RAM capture, stop, persist, verify readback, reboot and recover,
then dump and verify host integrity. No background acquisition during erase,
program, or dump. Use actual reported sample capacity rather than promise
4.094 seconds after adding metadata. No silent loss within a passing capture.
Save/download latency is measured and reported; no invented maximum bound.
A new recording requires free safe storage and a new session identity.
This short stopped-acquisition proof is not fulfillment of the selected 10-30 s
continuous-segment objective. It validates flash integrity/recovery first.

Next implement bounded RAM staging with background transfer/storage, validated
accounting and backpressure. Never clear a staging block until verified transfer
or explicit deletion. Reuse acknowledged blocks without rebooting. If destination
is unavailable and bounded storage fills, stop accepting samples or apply an
explicit approved discard policy; count/report rejected samples and preserve
unacknowledged valid recordings. Cannot guarantee indefinite acquisition,
unbounded outage retention and zero loss with finite storage.

If an onboard computer/logger is available, prefer CAN transfer to that logger
for mission storage; sensor flash can provide bounded buffering/recovery. If
storage attaches to the sensor, confirm actual PCB interface/capacity before
selecting hardware. If download is only after recovery, size local storage for
the entire mission; the internal partition is insufficient for long missions.
User confirmed the intended destination is external storage on the IVER 3EP
backseat CPU: sensor module(s) -> backseat CPU -> mission storage. Its interface,
software, storage capacity, and throughput specifications are not yet confirmed.
CAN is the current bench transport, not a confirmed IVER backseat interface.
The module scope is sensing/acquisition and reliable delivery; real-time control
is future work outside this effort. Do not impose control-loop latency guarantees.

The 10-30 s segments are logical recording/transfer units, not a requirement to
wait 30 s before delivering samples. Prefer incremental delivery to the CPU when
supported, while RAM/flash provide explicitly bounded staging/outage retention.
The CPU should verify integrity and acknowledge recording/block identity before
the module reclaims data. ACK means accepted into the agreed durable storage
boundary, not merely received by a process or CAN controller. The host software
and durability protocol must define that boundary before claiming safe deletion.

Integration information required: physical link and electrical compatibility,
available bus bandwidth/load, number of sensor modules, CPU service/API,
durable write/ACK semantics, worst-case service stalls and disconnect duration,
mission duration and storage quota, power sequencing, and time synchronization.
Aggregate raw load for N modules is N*4000 bytes/s plus protocol/metadata; size
staging from measured worst-case service stalls. No IVER specifications are
assumed. Each module must preserve its own identity and independent timeline.

Payload feasibility: 10 s = 40000 bytes; 30 s = 120000 bytes (decimal), before
metadata. A 30 s raw segment leaves only 11072 bytes of the 131072-byte partition
for metadata/spares and cannot coexist with a second complete 30 s segment.
This does not establish that the chosen future record format will fit. Raw data
production is 4000 bytes/s = 14.4 MB/hour, before integrity/time overhead. Sustained
sink throughput must exceed production including protocol overhead, with RAM
headroom for bounded stalls. CAN bus bitrate alone does not prove sink capacity.
No erase endurance/lifetime claim is possible until duty, retention and actual
allocation/reclaim policy are specified.

## Capacity and endurance calculations after user input

Compute records fitting usable bytes after selected metadata/spare reservations.
For repeated capture interval R seconds and duration D seconds, duty fraction is
D/R only if captures do not overlap and R includes persistence overhead. Include
worst-case bursts, not just daily averages. Calculate erases per physical block
per day from actual allocation/reclaim policy, including metadata wear and
retry/power-loss overhead. Compare with the exact device endurance specification
under required retention/temperature conditions; no endurance-life claim yet.

Acceptance: selected duration/retention/duty/gap policies documented, capacity
feasible or architecture limitation explicit, full-storage behavior exercised,
and measured save latency and recovery results recorded at later milestones.

## Checklist disposition

Defined: intended 10-30 s segments, continuous-if-possible duty objective,
best-effort policy with detected gaps explicit, backseat CPU as mission sink,
and preservation until verified transfer or explicit deletion.
Pending numerical requirements: mission/retention duration, module count,
maximum permitted outage/gap/loss fraction, and demonstrated sink capacity.
Internal flash endurance and final buffer sizing therefore remain open.
This item is documented with explicit TBDs, not fully performance-qualified.
