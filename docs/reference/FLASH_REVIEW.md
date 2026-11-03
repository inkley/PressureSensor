> Detailed specification/evaluation; distinguish proposed policy from implemented
> behavior. For current validation and remaining work, see [current status](../CURRENT_STATUS.md).
> Commands and source paths assume the repository root as working directory.

# Flash timing, endurance and silicon review

2026-11-02. Review for TM4C123GH6PM. Eval U1 revision 7 verified by DID0;
PCB U2 revision still unknown. No flash operations performed in this review.

## Manufacturer specifications

[TI SPMS376E datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf),
section 8.2.3 and Table 24-27, printed page 1384:

| Parameter | Minimum | Nominal | Maximum |
| --- | --- | --- | --- |
| Program aligned 64 bits | 30 us | 50 us | 300 us |
| Page erase, <1000 cycles | - | 8 ms | 15 ms |
| Page erase, 10000 cycles | - | 15 ms | 40 ms |
| Page erase, 100000 cycles | - | 75 ms | 500 ms |
| Program/erase endurance | 100000 cycles | - | - |
| Retention, -40 to +85 C | 20 years | - | - |
| Retention, 105 C | 11 years | - | - |

Programming fewer than 64 bits takes the same time. Flash instruction/literal
fetches stall during flash operations; execution needed during the operation
must reside in SRAM. Retention/endurance figures are device specifications,
not a demonstrated system lifetime. Do not substitute EEPROM specifications.

## Relevant errata and disposition

[TI SPMZ849F errata](https://www.ti.com/lit/er/spmz849f/spmz849f.pdf):

- MEM#14, revisions 6/7: flash execution during program/erase above 40 MHz can
  fetch wrong instructions. TI workarounds disable interrupts and either reduce
  clock to 40 MHz or execute the programming/erase code in SRAM. Current 20 MHz
  is below the stated failure condition; normal flash stalls remain.
- MEM#05, revisions 6/7: power/reset interruption of debug unlock or nonvolatile
  register commits can render the device unusable. These are not ordinary
  application log commit-marker writes. Do not use protection/BOOTCFG commits
  as recording transactions.
- MEM#03/#04 are revision-6 EEPROM hazards; revision-7 EEPROM has separate
  MEM#07/#10/#11 restrictions. EEPROM is not selected for this logger.
- ADC#01: never retrigger an incomplete sequence. Timeout recovery is required.
- ADC#14: reset the ADC after enabling its clock, before configuration.
- ADC#07: multi-sequencer arbitration issue; only SS2 is currently enabled.
  The separate unique-priority requirement also needs cleanup.
- ADC#08: current active-PLL clock choice follows a documented workaround.
- ADC#13: PE3 analog measurements are unaffected when source resistance meets
  specification; verify the analog drive, rather than adding a blind pull resistor.

These ADC advisories affect revisions 6/7. This is a relevant-subsystem review,
not certification that every device advisory is addressed. Recheck applicability
against PCB DID0 and any new DMA/timer/RTC/boot features before deployment.

## Local driver inspection

Repository driverlib/flash.c FlashErase and FlashProgram block while polling;
neither provides a software timeout in its busy loop. FlashProgram requires
4-byte-aligned address/count and uses the 32-word write buffer. Its API contract
forbids programming a word twice between erases; caller verifies readback.
Neither routine masks all interrupts on behalf of the logger. ASSERT is not a
runtime partition guard. No current main.c call implements persistent logging.

## Engineering consequences and implementation gates

1. Stage one stops the acquisition trigger, resolves any in-flight ADC sequence,
   freezes session statistics and preserves RAM before any erase/program. Merely
   setting StreamingMode=smStopped does NOT stop ADC triggering: the present ISR
   triggers it unconditionally. Add an explicit stopped-acquisition mechanism.
   Define scheduled pause and restart metadata; don't claim time continuity from
   GlobalTimer across flash stalls or masked interrupts.
2. Keep the established 20 MHz clock for initial persistence. Use a short,
   reviewed flash transaction wrapper with explicit prior interrupt-state
   restoration, valid-voltage handling and watchdog policy. Document resulting
   CAN unavailability. No command handler may reset/erase the MCU mid-transaction.
3. Enforce reservation bounds/alignment at runtime, then check API results,
   readback and CRC. CRC/marker writes each target erased words once; exclude
   marker bytes from earlier programming, rather than programming erased values
   there and writing the marker again. Treat interrupted writes as uncommitted.
4. For a planning illustration only, 1024/8=128 programming units at 300 us
   totals 38.4 ms, plus erase at the selected wear condition (up to 500 ms).
   This excludes software, split-commit transactions, readback and metadata;
   buffered-driver operation is not independently measured by that arithmetic.
   It is not a transaction timeout or system worst-case bound.
5. One 500 ms interval produces 500 raw pairs = 2000 bytes at nominal 1 kHz,
   IF acquisition can continue independently. RAM capacity alone cannot prevent
   losses when flash-resident ISR execution stalls. Pre-erasure removes erase
   from capture but not programming stalls. Later continuous storage needs an
   audited SRAM/ROM execution path, vectors/constants/callees and interrupt
   policy, or an independently validated hardware acquisition path. Bench-test
   all dependencies; moving only FlashProgram is insufficient.
6. Endurance is per physical erase location. Keep a per-block wear budget and
   append/rotate journal entries; never erase a fixed metadata page every sample.
   Pure sensitivity example: one erase of the same page each 10 s is 8640/day;
   100000/8640 = 11.57 days. At 30 s it is 2880/day, or 34.72 days. These are
   arithmetic cycle-budget exhaustion examples, not predicted failure times or
   guaranteed service life. Rotation, idle duty, faults and metadata policy
   change wear; final lifetime remains unknown until those are specified.
7. CPU mission storage is the long-term sink. Avoid unnecessary internal-flash
   rewrites for already verified transfers; use bounded flash buffering where
   needed. Exact logging duty and outage retention remain open requirements.

## Required later validation

- Stable-power readback/CRC after bounded capture-stop-persist; ensure firmware
  partition unchanged and no erase outside logging. Preserve original RAM on error.
- Power interruption at erase/header/data/CRC/marker boundaries using disposable
  log blocks, then reboot and recover only valid committed records. Test prior
  recordings remain accessible; do not invoke debug-unlock/nonvolatile-register
  commit sequences as fault injection.
- Measure erase, program, verification and whole-save latency under stated
  conditions, including aged-block behavior when feasible. Do not accelerate
  endurance testing on the only functional prototype by default.
- Exercise watchdog/reset/voltage failure behavior with an explicit recovery
  policy; timestamps from serviced SysTick cannot measure a flash-induced stall.
- After ADC initialization fixes: rebuild/flash and repeat pressure/stream/dump
  baseline. These fixes and power-loss tests are pending, not silently applied.

Status: specifications and relevant errata reviewed; checklist review addressed.
No firmware changes, build, flash, endurance run or recovery test in this step.
Implementation and runtime verification remain gates before claiming persistence
or loss-free continuous logging. See FLASH_CAPACITY.md and FLASH_PARTITION.md.
