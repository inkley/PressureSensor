# Acquisition timing requirements and evidence

2026-11-02. Documentation only; no acquisition configuration changes.
Status: channel order and nominal cadence documented. Limits below are
provisional engineering targets, not measured performance or a user-approved
application accuracy specification.

## Current configuration and evidence

Source main.c uses a 20 MHz nominal system clock and SysTick reload 19999.
The eval-board register inspection confirmed PLL lock and that configuration.
Each serviced SysTick issues a software trigger to ADC0 sequencer 2:

1. Step 0: PE3/AIN0, P1.
2. Step 1: PE2/AIN1, P2, end of sequence and completion status.
3. ISR polls completion, then reads both results into RAM while capturing.

A pair is two sequential ADC conversions, not simultaneous sampling. Both
pressure sensors independently measure differential pressure, but MCU ADC inputs
are configured as single-ended voltages. CAN transmission groups two successive
pairs; its nominal 500 frames/s is not the ADC pair rate.

ADC clock source/rate, phase and hardware averaging are not explicitly set by
Init_ADC. On a reset-initialized device, TI documents a 16 MHz ADC clock derived
from the active PLL, ADCPC.SR=7 (1 Msps), averaging disabled and phase zero.
These are expectations, not captured running-register values. Legacy clock
register writes or retained peripheral state can invalidate default assumptions.

TI lists 250 ns sample time, 1 us conversion time including sampling, and
nominal trigger-to-conversion latency of two ADC clocks at the specified clock.
For successive unaveraged conversions at 1 Msps, infer approximately 1 us between
P1/P2 sample apertures and approximately 2 us conversion time for the pair plus
trigger latency. This excludes software trigger overhead, polling and FIFO read.
It is neither a measured skew nor a worst-case bound. Do not add 250 ns again
to each 1 us conversion. Slower configured rates/averaging change the estimate.

Source: [TI TM4C123GH6PM datasheet](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf),
sections 13.3.2.6, ADCPC/ADCSAC/ADCSPC/ADCCC descriptions, and Table 24-33
(including its conversion-time footnote).

## Proposed limits for the first logging implementation

Let T=1000 us. For a capture segment, use an established schedule epoch t0 and
nominal slot n: intended trigger time s[n]=t0+n*T. A fixed known instrumentation
or trigger latency is calibrated separately, not repeatedly absorbed into t0.
Measure actual software trigger time t[n] using the planned free-running MCU
timebase; it is a proxy for aperture time, not the ADC aperture itself.

| Quantity | Provisional target | Interpretation |
| --- | --- | --- |
| Pair cadence | 1000 pairs/s nominal | Each channel has 1000 readings/s |
| Consecutive trigger interval | 990 to 1010 us inclusive | Interval error no larger than +/-10 us |
| Trigger phase error t[n]-s[n] | -10 to +10 us inclusive | Prevent slow drift from hiding behind acceptable intervals |
| P1-to-P2 aperture separation | Target <=10 us; expected about 1 us conditionally | Must validate ADC setup and later measure; no claim of simultaneity |
| Silent acquisition loss | Zero | Every known failed pair and detected discontinuity reported |
| Continuous-logging integrity | Zero known lost pairs for a passing run | Detected loss is explicit but still fails the loss-free target |

The 10 us jitter budget is 1% of the pair period. At an illustrative 150 Hz
signal, 10 us corresponds to 0.54 degrees of phase; the analysis filter's
150 Hz default does not establish the sensor/application bandwidth. These are
starting targets to guide implementation. Revisit with required signal bandwidth,
phase/amplitude accuracy, sensor response time, analog filtering and measured
CPU load before product acceptance. They are not derived from an application
requirement already supplied by the user. Absolute clock ppm/UTC accuracy and
multi-module alignment remain separate requirements, currently unspecified.

## Gap and fault policy

- Any ADC timeout, FIFO/error condition, or invalid pair is a known failed
  acquisition attempt, whether or not a timing threshold was crossed. Record
  the slot, reason and time; never duplicate a previous pair to fill it.
- Any interval outside [990,1010] us or phase outside [-10,+10] us is a timing
  violation. Flag early as well as late triggers. A threshold breach alone is
  not proof of a missing sample and must not invent an exact missing count.
- A delay spanning one or more unsatisfied intended slots requires a gap event.
  Bind slot assignment to the established schedule and use the timestamp
  uncertainty/jitter budget to distinguish boundary ambiguity. If service history
  cannot establish exact loss, report an estimate or unknown count explicitly.
  The accounting document's floor-based slot estimator is provisional and must
  not classify normal boundary jitter as an exact missed acquisition.
- On timeout, quiesce/recover the sequencer before the next attempt. Report the
  recovery discontinuity. Do not burst-trigger catch-up samples and label them
  as historical acquisitions.
- Retain valid irregularly timed samples with actual times and quality flags.
  Host plots show time gaps; filtering must not bridge discontinuities silently.
- RAM evictions, live CAN queue drops, uncertain TX completion and host transfer
  gaps are separate integrity events, not automatically ADC acquisition losses.
- Explicit stop/persist/dump intervals are intentional acquisition pauses. For
  stage-one flash persistence, acquisition is stopped before writes. Report the
  capture end and new capture start as segment/session boundaries; never claim
  continuous 1 kHz coverage across that pause. Duration is unknown if the MCU
  timebase did not run continuously.
- Debugger suspension invalidates uninterrupted-timing evidence. If detectable,
  tag it; otherwise mark unexplained discontinuity. No attempt to infer accurate
  elapsed wall time from GlobalTimer across a halt.

## Verification before claiming timing compliance

1. Read ADC0 ADCCC, ADCPC, ADCSAC, ADCSPC, ADCEMUX, ADCSSMUX2 and ADCSSCTL2 in
   CCS. Expected relevant fields: clock source 0 with PLL active, SR=7,
   averaging=0, phase=0, EM2=0, MUX0=0/MUX1=1, IE1=END1=1, D0=D1=0.
   MUX2 raw value normally 0x10 and CTL2 normally 0x60 if unused fields are zero;
   decode fields rather than relying on entire-register equality.
   Read no FIFO data registers for this inspection (reads consume samples).
2. Add the planned coherent MCU timing/counter reporting; benchmark its overhead.
   Test idle acquisition, live CAN streaming, RAM wrap and injected service delay.
   Do not take single-step/halted observations as cadence measurements.
3. For each bounded test, retain duration, valid/failed/estimated-missed totals,
   min/max interval and phase error, violation count and interval distribution.
   Require zero known loss and zero target violations to pass that test; state
   exact conditions and duration. A bounded run is not a worst-case proof.
4. With future instruments, measure calibrated trigger markers and completion
   markers under load. Trigger/completion pulses alone do not measure interchannel
   aperture skew; use a shared characterized analog stimulus or an appropriate
   ADC timing experiment. Independent clock reference needed for absolute rate.
5. Repeat under background flash program/erase only at the continuous-storage
   milestone; the stopped-acquisition persistence stage has no such claim.

## Eval-board ADC register confirmation: 2026-11-02

User supplied CCS ADC0 screenshots while suspended:

| Register | Observed | Interpretation |
| --- | --- | --- |
| ADC_CC | 0x00000000 | CS=0; with previously confirmed active PLL, nominal 16 MHz ADC clock |
| ADC_PC | 0x00000007 | SR=7, 1 Msps conversion-rate setting |
| ADC_SAC | 0x00000000 | Hardware averaging disabled |
| ADC_SPC | 0x00000000 | No programmed phase delay |
| ADC_EMUX | 0x00000000 | SS2 uses processor trigger |
| ADC_SSMUX2 | 0x00000010 | Step 0 AIN0/P1, then step 1 AIN1/P2 |
| ADC_SSCTL2 | 0x00000060 | End/completion on step 1; both inputs single-ended |

These confirm the configuration assumptions behind the approximately 1 us
interchannel separation estimate. 1 Msps is conversion capability/settings,
not the application's 1000-pair/s trigger schedule. Actual aperture separation,
trigger latency and jitter have not been measured.

Additional observation: ADC_SSPRI=0x00003010 gives SS0 and SS2 priority zero,
SS1 priority one, SS3 priority three. TI requires unique sequencer priorities.
Follow-up CCS screenshot confirmed ADC_ACTSS=0x00000004: only SS2 enabled,
SS0/SS1/SS3 disabled, BUSY=0 at inspection. Correct the duplicate priority
assignment as a separate ADC initialization change before enabling other
sequencers; do not infer a demonstrated acquisition failure from this
snapshot. No ADC initialization code was changed in this documentation step.

Current evidence: nominal clock/SysTick and ADC configuration registers verified;
basic rollover and streaming timer tests passed. Actual jitter/channel-skew
measurement, timeout injection and timestamp/gap reporting remain pending.

Related: [accounting contract](ACQUISITION_ACCOUNTING.md),
[timer bench results](TIMER_VALIDATION.md), [storage plan](TIMING_AND_STORAGE.md).
