# Full-storage, overwrite and wear-management policy

2026-11-02. Policy definition for the underwater pressure logger. Current firmware
.240 remains the bounded single-record bench implementation. No firmware behavior,
flash addresses or on-device wear limits are changed by this document.

## Defaults and full-storage behavior

1. Default retention mode is MANUAL_RECLAIM. ACK marks a recording verified but
   retains it; explicit ID/digest-bound CLEAR is required. Full storage is never
   implicit deletion authorization. A capture restart never resets the MCU or
   clears another recording.
2. Preserve committed unacknowledged recordings, all valid pending metadata and
   frozen RAM source until transfer verification or explicit authorized deletion.
   Current normal CLEAR requires durable ACK; unacknowledged discard is not
   exposed. Unknown/corrupt/unsupported blocks are quarantined, not formatted.
3. Current single-record mode is logically full whenever a record or incomplete
   allocation is retained, even if many bytes are FF. Reject new CAPTURE/resize/
   legacy start as appropriate; return retained state and capacity explanation.
   RAM capture auto-stops at configured pair limit with valid prefix retained.
4. Admission checks must reserve worst-case capture data, manifest, journal work,
   required event space and recovery margin before enabling acquisition. Never
   consume recovery spares to advertise normal sample capacity. Only space safe
   for the intended operation counts as free; FF-looking media alone after an
   interrupted erase is not proof of a successful erase cycle.
5. Future continuous mode default is STOP_ON_BACKPRESSURE. If no safe RAM block
   or log allocation is available, stop acquisition at a defined boundary, freeze
   valid data and report STORAGE_FULL/BACKPRESSURE with last valid slot/time.
   Do not silently overwrite oldest samples or fabricate catch-up samples.
   Report measured pause/start boundaries, reason and known/estimated missing
   count when acquisition resumes; no continuous-coverage claim across the pause.
6. A future DROP_NEWEST mode may be added only as explicit configuration, with
   rejected-pair counters, sticky loss flags and bounded event reporting. It is
   not currently implemented or selected by the user's best-effort preference.
   Loss of volatile diagnostics at power failure must remain an uncertainty,
   not be reported as zero loss. Never sacrifice an unacknowledged recording to
   make room for diagnostic details.
7. Metadata/event capacity exhaustion produces an explicit sticky integrity/
   storage error and stops capture when information required for recoverability
   cannot be retained. Previously committed records remain readable. A degraded
   best-effort capture can be retained with loss flags; do not label it complete
   in time or loss-free simply because its stored block CRCs pass.

## Overwrite/reclamation eligibility

| Data class | Default eligibility |
| --- | --- |
| Unacknowledged committed recording | Never automatic; current CLEAR rejects |
| ACKNOWLEDGED committed recording | Explicit matching CLEAR only in current mode |
| Incomplete or corrupt allocation | Quarantine; no generic erase-all/auto-repair |
| New RAM recording | Protected from stale old-record requests |
| Authorized CLEAR_PENDING footprint | Explicit same-ID clear retry under durable intent |
| Old journal snapshot | Reuse only after a newer valid snapshot is durable |
| Scratch/recovery spares | Never general overwrite capacity; dedicated guarded use |

For later IVER integration, AUTO_RECLAIM_VERIFIED may be explicitly enabled only
after ACK means verified durable CPU storage, block ownership and recovery tests
pass. It applies only to acknowledged, unpinned records, oldest eligible first;
use the same durable deletion-intent protocol. Default remains manual until that
configuration and tests exist. Never infer durable receipt from CAN TX success,
a USB transfer, or receipt by a process before its agreed storage boundary.

No circular overwrite of unacknowledged data is selected. If mission autonomy
requires different behavior, revise this policy explicitly rather than silently
inherit the old legacy circular RAM buffer's behavior.

## Wear accounting and deployment guardrails

Manufacturer figures and conditions are in FLASH_REVIEW.md. This policy uses
conservative erase-command accounting for budgeting; it does not reinterpret the
manufacturer's program/erase-cycle definition. A command issued against an
already erased page still consumes a conservative policy allowance. Count
attempts and interrupted/unknown outcomes, not just successful replies.

Track per physical log erase block (not just per recording): erase allowance
reserved/used, program failures, erase failures, verification failures, unknown
power-loss outcomes, last operation identity and health. Include journal/event/
spare pages. Data movement and reclaim retries count too. Firmware updates and
manual programmer erases must be recorded separately; they cannot silently reset
the physical part's lifetime usage. Untracked historical prototype wear is
UNKNOWN, not zero.

Proposed deployment thresholds (engineering margins, NOT enforced by .240):

- Advisory at 80000 conservative erase attempts per physical block.
- Do not issue a new erase at 90000 accumulated/reserved attempts. Preserve and
  permit reads/verified export; stop writes that require that exhausted block.
- A permanent failure or ambiguous integrity fault quarantines the affected
  allocation. Retry only with a defined bounded procedure and safe source copy;
  do not spin indefinitely on a failing page. A byte-level nonvolatile-physics
  qualification cannot be inferred from software CRC checks.

These margins are provisional and must be reviewed against actual temperature,
retention, supplier specifications and mission duty before deployment acceptance.
They are not predictions of failure at any particular day/count. Read access
must not be blocked by reaching a software wear budget.

## Durable counters and allocator requirements (not yet implemented)

A deployment implementation needs a power-loss-safe wear ledger plus a rotation
policy. Precharge a bounded batch of allowed erases in committed metadata BEFORE
using them; after reset, count unused reserved allowances as consumed. This trades
some capacity/lifetime for avoiding wear undercount after an interrupted erase.
The ledger's own update/reclaim operations also require a reserved wear budget.
Define the exact bounded reservation protocol and test it before claiming counters
are durable; a volatile counter incremented after FlashErase is insufficient.

Rotate eligible data and metadata allocations by physical wear while respecting
unacknowledged ownership. Current fixed DATA pages and alternating two journal
pages are not wear leveling. Moving a record requires a validated new copy and
recoverable ownership update before old erasure. Exhausted metadata/spare budgets
can stop writing even when nominal data bytes remain free.

Wear-ledger storage is NOT included in the current on-flash binary layout or
128 KiB capacity claim. As a sizing sensitivity, two additional 1 KiB ledger
pages would reduce data pages from 119 to 117 and ideal compact capacity from
14161 to 13923 pairs (13.923 s at nominal 1 kHz). That is not an allocated ledger
or proof two pages suffice. Final layout/version/capacity must be recalculated
with its journal/rotation implementation; do not consume existing recovery spares
without re-budgeting. Keep .240 addresses unchanged for the tested capture.

## Actual .240 wear pattern and sensitivity

Each ordinary persisted capture programs the used DATA pages and manifest once.
The writer erases its known-blank destination first; explicit clear erases that
footprint again. Therefore budget two erase COMMANDS per used data/manifest page
per successful capture+clear cycle, plus interrupted-clear retries. A repeated
completed clear is a no-op. ACK and DELETE each write a snapshot in alternating
journal pages: normally one erase command per journal page per complete lifecycle
in steady state. Duplicate ACK does not write another snapshot.

An erased page's extra blank erase is not necessarily another full manufacturer
1->0->1 bit cycle. Both the conservative command budget and vendor cycle figure
must be reported with their own definitions. Metadata wear and data wear can be
optimized later; there is no automatic rotation of DATA starting at page 9 today.

Illustration only, with a 90000-command limit and two erase commands per hottest
DATA/manifest page per cycle (45000 lifecycle cycles):

| Start-to-start interval | Cycles/day | Conservative time to software budget |
| --- | --- | --- |
| 10 s | 8640 | 5.208 days |
| 30 s | 2880 | 15.625 days |

These assume uninterrupted 24-hour repetition, no retries, and those same pages
every cycle. They are arithmetic budget examples, not measured endurance or
predicted device life. The interval includes capture and save/transfer/clear;
no claim the present implementation meets a 10 s wall-time cycle. Reducing active
duty to 10% multiplies calendar time by ten only if cycle frequency scales the
same way. Real duty and backseat outage distribution are still unspecified.

Prefer incremental durable transfer to the IVER backseat CPU for mission storage.
Internal flash can be an explicitly bounded outage buffer; do not write every
sample through internal flash unnecessarily when a verified sink is available.
This is a future routing policy, not current continuous-transfer behavior.

## Status/reporting required for deployment

Report physical capacity, usable safely writable blocks, retained/ACKed records,
quarantine, reserved spares, metadata headroom, per-page wear high-water and
UNKNOWN history. Distinguish RAM_FULL, LOG_FULL, JOURNAL_FULL, EVENT_FULL,
WEAR_LIMIT, WRITE_FAILURE and UNKNOWN_OUTCOME. Stop reason and retention state
must survive as far as the selected durable mechanism guarantees; otherwise
state uncertainty explicitly after reboot. No success response until the relevant
state transition is verified. Current status protocol does not expose this full
set; it reports single-record state/count and errors only.

## Acceptance work before enforcement/continuous logging

- Fill bounded RAM and flash/logical slot, reject new work without changing
  retained records, exercise maximum/invalid capture sizes and metadata limits.
- Confirm ACK retains data, authorized clear reclaims only its footprint and
  stale requests cannot erase a new capture (basic .240 bench already passed).
- Model/inject event/spare exhaustion and an unavailable CPU sink, check explicit
  stop/loss reporting, preserve old commits and resume only with safe space.
- Exercise wear-budget thresholds with simulated counters; no need to wear out
  the physical prototype to test policy. Verify counts survive reset/interruption,
  include metadata and conservative reserved-but-unused allowances.
- Test quarantine/bounded retry, allocator rotation and interrupted copy/reclaim
  recovery before enabling automatic reclamation or background persistence.
- Establish mission duration, duty/outage profile, module count and CPU durable
  throughput before claiming sufficient endurance/retention for vehicle service.

Status: full-storage, overwrite and wear-management POLICIES defined. Current
retention/clear rules are implemented and bench-tested; wear counters, thresholds,
wear leveling and continuous backpressure enforcement remain implementation gates.
No flash, rebuild, endurance run or lifecycle deletion occurred in this step.
