# RAM staging and execution evaluation for continuous logging

2026-11-02. Evaluation against the tested .240 Debug build. No firmware behavior,
linker placement, allocation or flash contents changed by this evaluation.
This establishes implementation requirements, not continuous-logging acceptance.

## Conclusion and preferred route

Continuous acquisition is feasible enough to prototype with bounded RAM queues,
but the current stopped logger cannot be called during active acquisition and
be described as continuous. Prefer normal acquisition -> RAM blocks -> CAN ->
IVER backseat durable storage, with explicit bounded backpressure. Internal
flash remains bounded fallback/outage storage until its execution path and wear
behavior are validated. This does not assume the IVER interface supports CAN;
vehicle integration specifications remain open.

First prototype continuous RAM-to-host transfer WITHOUT internal flash operations
while acquiring. A RAM-only block verified/durably owned by the host may be freed
without a sensor flash ACK journal; it has no local persistent copy to reclaim.
Do not invoke the present flash ACK/CLEAR functions from that active RAM pipeline.
Existing flash-spooled recordings still require their durable lifecycle. Normal
RAM streaming plus explicit stopped fallback is a useful best-effort intermediate
milestone; it must expose pauses, not claim gap-free background flash logging.

## Measured-by-map allocations (not runtime stack measurements)

Debug/Inkley_PressureSensor.map, linked .240:

| Allocation | Bytes |
| --- | --- |
| Total SRAM | 32768 |
| Linker allocated | 20248 (0x4F18) |
| Unallocated | 12520 (0x30E8) |
| SensorBufferData: 4094 raw pairs | 16376 |
| log_image: serialization staging | 1024 |
| Existing stack reservation | 2048 |
| Existing RAM .vtable reservation | 620 |
| Other allocations, excluding raw array and log_image | 2848 (includes stack/vtable) |

Unallocated memory is not a runtime high-water guarantee. There is no spare
16376-byte full-size raw array within today's 12520 free bytes. Adding a second
array or 4094 timestamp/index-rich entries is not a valid staging plan.

## Proposed bounded queue replacing the raw array

Use eight fixed 1024-byte block slots, matching compact timed records:
119 pairs/block after 64-byte header and 8-byte trailer. A slot passes through
FREE -> FILLING -> SEALED -> IN_FLIGHT -> VERIFIED -> FREE. Distinguish transport
receipt from agreed durable ownership before freeing the only valid copy.
Publication/ownership needs short atomic handoff; producer never overwrites a
sealed/in-flight block. ISR records samples/time; formatting/CRC/transfer work
stays outside the ISR unless specifically audited for flash-busy execution.

Each slot can hold the serialized header/payload/trailer, avoiding another
full-record copy. Conservatively keep existing 1 KiB log_image for serialization
in this budget. Use actual head/tail/free counts and high-water diagnostics.
The current ring is the legacy/bounded mode's storage; selecting continuous mode
requires a distinct allocation/configuration, not running both arrays at once.

| Planning allocation | Bytes |
| --- | --- |
| Current used - 16376 raw array + 8192 queue (retain log_image/stack) | 12064 |
| Selective SRAM execution allowance | 8192 |
| Additional stack allowance, beyond existing 2048 | 2048 |
| Queue descriptors/events/timing diagnostics allowance | 1024 |
| Planned subtotal | 23328 |
| Remaining margin before optional DMA | 9440 |
| Optional two raw ADC DMA buffers, 256 pairs each, 2 uint32 words/pair | 4096 |
| Remaining margin with optional buffers | 5344 |

These are provisional ALLOWANCES, not measured compiled sizes or permission to
allocate them all blindly. A DMA control table may require another aligned
1024-byte reservation, reducing the last margin to 4320. Actual code, alignment,
record/event metadata and stack high-water must be measured after linking.
ROM calls or selective register-level helpers might reduce SRAM code needs;
copying the whole application is not the proposed method.

## Stall sizing and sustained throughput

At nominal 1000 pairs/s, compact record bytes arrive at 8000 B/s before headers.
Eight slots hold 952 pairs total. If one slot is filling and one in flight, and
all SIX others start free, conservatively 714 additional pairs fit (714 ms).
This is not guaranteed outage tolerance with an already occupied queue, early
block splitting due to gaps, delayed durable ACK, or exhausted event space.

For a single bounded sink stall S:

    additional_pairs = ceil(1000*S)
    total_slots >= ceil(additional_pairs/119) + 2 withheld slots

| Illustrative stall | New pairs | Total slots with two withheld |
| --- | --- | --- |
| 15 ms | 15 | 3 |
| 40 ms | 40 | 3 |
| 500 ms | 500 | 7 |
| 538.7 ms | 539 | 7 |
| 638.7 ms | 639 | 8 |

538.7 ms illustrates 500 ms erase plus 129*300 us programming allowance: 128
aligned programming units for body/CRC plus a separate marker operation. The
extra 100 ms in the last row is a proposed software/service margin, not measured
latency. Driver buffering, metadata, readback and retries need measurement; these
rows are sensitivity calculations, not a guaranteed flash transaction bound.

One full block represents 119 ms of acquisition. At the datasheet's 500 ms
page-erase maximum at the 100000-cycle condition, erase alone exceeds production
interval. Even ignoring program/metadata, service ceiling for that assumed
repeated latency is 119/0.5=238 pairs/s. A finite queue cannot guarantee indefinite
1000-pair/s flash logging under repeated such latencies. Pre-erasure defers that
cost until space must be reused; it does not eliminate wear or endless backlog.

The present LogPersist holds interrupts disabled across the whole RAM recording,
not just one page: 1000 pairs use nine DATA pages plus manifest. Ten page erases
at 500 ms would already be 5 seconds, far beyond the proposed queue budget.
Background storage requires a new incremental worker and execution strategy,
not merely changing acquisition_enabled to true in LogPersist.

Every sustained sink must retire VERIFIED blocks faster than arrival on average,
with bounded worst-case latency. A timer-backed queue must stop/report full when
that condition fails. No finite RAM size solves an unlimited IVER outage.

## CAN service requirement

Full serialized blocks cost 1024*1000/119 = approximately 8605 bytes/s per module
before event/manifest/transport overhead. Today's flash download carries only
four data bytes per CAN frame: approximately 2151 DATA frames/s for this rate,
plus requests/replies/retries. This is over four times the legacy packed live
frame rate of 500/s. Multiple modules scale the demand. Do not infer throughput
from successful 1000-pair stopped dumps or nominal 1 Mbps bitrate.

The new continuous transport needs bounded asynchronous TX queues, credits or
ACK windows, recording/block/sequence association, integrity checks and service
priorities. Measure actual bus load and host USB/SLCAN consumption plus durable
write ACK latency. Backseat CPU storage throughput is not yet known. Avoid
one blocking request/response per sample. Current dump/control code is explicitly
not a concurrent streaming transport.

## Flash execution dependency audit

The current map places SysTickIntHandler, ADC trigger/status/read/overflow helpers,
FlashErase/FlashProgram and CANSendMSG_Obj in application FLASH. The .vtable RAM
allocation exists but main/startup source does not relocate NVIC_VTABLE; presence
of RAM storage does not prove active RAM vectors. Confirm actual VTOR in CCS
before a future flash-busy execution experiment.

TI states flash instruction/literal access is stalled during program/erase and
requires SRAM execution when code must run concurrently. ADC supports timer
triggers and DMA, but their configuration and service behavior need validation.
Sources: [TI datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf),
8.2.3.7, 13.3.2.2 and ADC trigger configuration; applicable
[MEM#14 and DMA errata](https://www.ti.com/lit/er/spmz849f/spmz849f.pdf).

For a CPU-serviced SRAM approach, ALL items below must pass before enabling:

- A narrow acquisition ISR, all called helpers, runtime arithmetic helpers,
  literals/constants, and exception vectors must avoid flash access while busy.
  Existing modulo/division, memcpy, library dispatch and debug assertions must
  be audited in disassembly, not assumed safe because the top-level ISR moved.
- The flash launch/wait path must remain in SRAM (or verified safe ROM). A
  foreground flash fetch stall must not be assumed interruptible in time merely
  because vectors moved. Copy/link dedicated sections at boot and validate both
  flash load and SRAM run bounds, references and vector targets.
- Current global interrupt masking prevents SysTick acquisition even with RAM
  code. Design an explicitly reviewed selective interrupt policy at verified
  20 MHz, or a proven autonomous hardware path; never just remove the mask.
  All interrupts/exceptions allowed during flash busy need safe paths. At higher
  clocks revisit MEM#14 workarounds; keep 20 MHz for feasibility experiments.
- Acquisition time must come from a coherent free-running hardware timebase;
  GlobalTimer counts serviced ticks and cannot reconstruct masked/stalled time.
  Add successful/failed/missed-slot counters, gap flags and event capacity.
- CAN ISR, command handlers, CRC/serialization, lifecycle journal writes and page
  reclaim must not introduce an unbounded acquisition dependency. Queue control
  requests while busy; bound their memory, latency and rejection behavior.

A hardware timer -> ADC -> DMA alternative reduces CPU-trigger jitter, but is
not automatically continuous. DMA descriptors and destination RAM must remain
serviceable during flash operations; prove bus behavior and applicable errata.
Two 256-pair raw buffers cover only 512 ms if CPU cannot rearm them: little margin
beyond a 500 ms erase, and insufficient for a whole-record persistence blackout.
ADC FIFO DMA words are 32 bits; budget two words per pair rather than treating
raw DMA storage as already packed uint16 data. DMA ADC data alone does not capture
an independent per-sample timestamp: define trigger schedule/counter correlation,
FIFO errors and descriptor transition gaps explicitly. No DMA implementation is
selected or claimed in this evaluation.

## Ordered implementation and acceptance gates

1. Implement coherent counters, hardware timebase and detectable gap reporting;
   validate against current baseline before changing acquisition/storage concurrency.
2. Replace raw staging in a separate continuous mode with the bounded queue.
   Start with RAM -> host transfer and no internal flash while acquiring.
3. Inject host stalls (15/40/500/640 ms), delayed ACKs, drops/reorders, queue full,
   event exhaustion and reconnect. Measure queue high-water and prove ownership,
   integrity and explicit stop/gap behavior; verify actual MCU timing metadata.
4. Measure stack high-water, linked SRAM code/table alignment and worst-case ISR
   and service latency. Existing +/-10 us timing goals remain provisional.
5. Only then prototype audited per-operation internal flash overlap, pre-erasure
   and reclamation. Sweep program/erase, CRC, CAN load, journal/clear and error
   paths. Keep real power-cut tests separate from modeled boundary injection.
6. Bounded runs can establish behavior under stated conditions, not infinite
   loss-free operation. Final acceptance requires backseat interface, module
   count, mission duty/outage, storage capacity and endurance limits.

Reproduce arithmetic: `python tools/evaluate_staging.py` reads the current Debug
map. Current result: RAM budget plausible; execution and sustained flash service
NOT qualified for continuous logging. Checklist evaluation addressed with these
constraints; implementation, measurement and on-vehicle integration remain open.
