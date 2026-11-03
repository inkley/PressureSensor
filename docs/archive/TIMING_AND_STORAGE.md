> Historical checkpoint/reference, preserved for evidence. Statements about what was
> current or pending apply to the dated sections, not the latest firmware.
> Commands/source paths assume repository-root working directories. See
> [current status](../CURRENT_STATUS.md) and the [acceptance procedure](../ACCEPTANCE_PROCEDURE.md).

# Timing, persistent logging, and PCB deployment

Planning baseline: September 22, 2026. This document incorporates the development
notes and distinguishes current behavior from proposed work. The subsequent
[GH6PM build update](GH6PM_BUILD_VALIDATION.md) reserves a logging partition;
flash programming and persistence remain unimplemented.

## Current timing implementation

`main.c` configures the main oscillator for a 16 MHz external crystal and uses
the PLL with `SYSCTL_SYSDIV_10`. This configuration implies a 20 MHz system clock,
not the approximately 40 MHz stated in the older notes. Confirm the running
clock using `SysCtlClockGet()` and an instrumented GPIO before changing timing.

`SYSTICK_TIMING=1000` and `SysTickPeriodSet(SysCtlClockGet()/SYSTICK_TIMING)`
configure nominal 1 ms interrupts. The CPU itself runs much faster; it is the
sampling trigger that runs at 1 kHz, not the main-loop polling frequency.

The SysTick ISR triggers ADC0 sequencer 2, waits for completion, and obtains P1
and P2 from successive conversion steps on PE3/AIN0 and PE2/AIN1. The channels
share a trigger but are not sampled at exactly the same instant. Each channel
already measures differential pressure across its own pair of ports.

`GlobalTimer` now increments at SysTick entry, including early returns, and
heartbeat deadlines use wrap-safe arithmetic. Build 1005 passed basic eval-board
rollover and streaming-timer checks; see [timer validation](TIMER_VALIDATION.md).
It counts serviced ticks, not independently measured elapsed time or successful
acquisitions. It still wraps after about 49.7 days at nominal 1 kHz.

See [acquisition accounting definitions](../reference/ACQUISITION_ACCOUNTING.md) for the
proposed session IDs, counters, timestamp semantics, and explicit gap reporting.
Those definitions are not yet implemented in the firmware or host protocol.

There are currently no per-sample MCU timestamps in RAM records or sample CAN
frames. Dump sequence indices restart at zero for each dump. Python timestamps
represent host receipt estimates or dump-file creation, not ADC acquisition.
SysTick provides a lightweight local cadence without an external RTC, but does
not by itself provide UTC, cross-module synchronization, or loss detection.

## Recommended deployed time model

Keep a monotonic acquisition counter independent of UTC adjustments. Associate
each stored block with its first sample counter, nominal sample period, valid
sample count, session ID, and timing/error flags. Count missed acquisitions
explicitly; do not disguise gaps by assigning consecutive indices only to
successful samples. Evaluate a hardware timer-triggered ADC and DMA if measured
ISR jitter or storage interference exceeds the timing requirement.

For the PCB, evaluate the selected MCU's battery-backed hibernation RTC with a
32.768 kHz crystal and backup supply. Use a separate RTC if the required drift,
temperature stability, power budget, or backup behavior warrants it. Budget
crystal tolerance, load capacitors, layout, backup power, and oscillator start-up.
TI documents the hibernation RTC clock and backup behavior in chapter 7 of the
[TM4C123GH6PM datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf).

At session start, store an atomic mapping between the monotonic counter and
UTC, including synchronization source, validity, and uncertainty. For bench
work, a PC can supply time over CAN, with communication latency characterized.
For tighter multi-module alignment, evaluate a shared hardware trigger or PPS
capture plus a source of UTC date/time. RTC retention alone does not synchronize
independent modules. Test drift, power loss, reboot, resynchronization, and
counter rollover; preserve acquisition ordering when wall time changes.

## Flash logging: staged feasibility experiment

RAM remains the acquisition staging buffer. Current dumps preserve its contents;
starting streaming or resizing clears it. An automatic capture/dump/reset/restart
cycle is proposed behavior, not implemented behavior. Define whether gaps are
acceptable and record them if acquisition pauses during persistence or download.

Hardware clarification from the user: both the bench MCU and planned PCB MCU
are TM4C123GH6PM. [TI specifies 256 KB flash and 32 KB RAM for
GH6PM](https://www.ti.com/product/TM4C123GH6PM).

The former GE6PM build configuration has now been updated to GH6PM for Debug
and Release. `tm4c123gh6pm.cmd` keeps the lower 128 KiB for the application and
reserves the upper 128 KiB for logging, with 32 KiB SRAM. Both configurations
built and passed executable layout/vector checks; see the build validation
document for results. The silicon revision and post-build bench behavior still
need hardware verification.

At the current four bytes per two-channel sample and 1,000 samples/s, payload
storage costs 4,000 bytes/s. Even an entire 256 KiB device would hold only
65.536 seconds before reserving firmware, bootloader, metadata, and spare blocks.
A hypothetical 128 KiB log partition holds at most 32.768 seconds of payload.
Per-sample timestamps increase this cost; block-level timing can reduce overhead.
Calculate capacity from the actual linker map and final record format.

Reserve an explicit, erase-aligned data partition excluded from executable code
and future firmware growth. TI describes 1 KiB erase blocks and inhibited flash
access during write/erase operations in chapter 8 of the GH6PM datasheet.
Thus, main-loop flash writes can still stall flash-resident sampling code.
Begin with capture-to-RAM, stop, persist, read back, and validate. This proves
persistence, not continuous recording beyond RAM capacity.

Continuous logging is a separate milestone: pre-erase blocks where appropriate,
size RAM staging for worst-case latency, and audit all execution dependencies
during flash operations, including vectors, ISRs, called functions, and constants.
Use SRAM/ROM execution only after confirming the relevant device requirements.
Measure sample timing under worst-case erase/write and CAN load before accepting
the design. Consult the exact silicon revision's datasheet and errata.

Design append-only records/blocks with format version, sequence, CRC, and a
commit marker written last. On boot, recover valid committed data and reject
partial writes. Define full-buffer, overwrite, erase, retry, and transfer-ACK
behavior; do not erase the only valid capture merely because transmission began.
Budget erase endurance and metadata wear from actual logging duty cycle.

Internal flash is a useful bounded persistence experiment, not a RAM-equivalent
circular buffer. Retain external SPI FRAM as a candidate for frequent writes and
external SPI flash for larger capacity. Choose after specifying duration,
endurance, power-loss retention, permissible gaps, and PCB constraints.
