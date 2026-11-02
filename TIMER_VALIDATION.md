# GlobalTimer correction and rollover validation

Firmware build 1005 (CAN version 0.0.3.237).

GlobalTimer increments once at SysTick entry, including ADC-timeout and
packed-frame early returns. It counts serviced interrupts, not elapsed UTC,
successful samples, or missed/coalesced ticks. Its uint32_t value wraps after
approximately 49.7 days at nominal 1 kHz; it is not a long-term monotonic timestamp.

Heartbeat deadlines use unsigned modular subtraction and are due at equality.
This assumes deadlines and checking delays are less than 2^31 ticks apart.
The first heartbeat is scheduled 10000 ticks after boot; commands defer it by
10000 ticks, preserving the existing command behavior. Streaming suppresses
heartbeats. A single tick snapshot supplies each heartbeat and its next deadline.

Offline validation: TI Debug compilation/link and firmware layout validation
passed. `python tools/test_timer.py` passes five arithmetic/source regression
checks. These checks are not execution tests of the ARM ISR.

## CCS procedure for repeat testing

1. Load Debug/Inkley_PressureSensor.out using the normal CCS program download
   workflow; this intentionally replaces the old image. Resume and query version
   in the host CLI: expect 0.0.3.237.
2. In CCS Expressions, observe GlobalTimer with target suspended, resume for
   approximately 10 seconds, then suspend and compare. Repeat while streaming.
   Expect roughly 10000 ticks in either mode; manual timing is only approximate.
3. Stop streaming and suspend. Set GlobalTimer to 0xFFFFFF00 and HeartbeatTrigger
   to 0x00002610 (10000 ticks after that starting value). Set a breakpoint on
   CAN_RESP[0] in the stopped heartbeat block. Resume without sending CLI commands.
   Expect the breakpoint after approximately 10 seconds, after rollover, with
   local now at or shortly after 0x00002610. No immediate pre-wrap heartbeat.
4. Remove breakpoint. Resume to send the heartbeat; confirm continued CLI
   version/start/stop/status/dump operation. Power-cycle afterward to restore
   ordinary uptime and deadline state.

Halting disturbs acquisition; do not use these captures as uninterrupted
cadence evidence. ADC-timeout runtime fault injection remains untested.
## Eval-board results: 2026-11-02

User flashed build 1005 and confirmed CAN version 0.0.3.237.

- Rollover: initialized GlobalTimer=0xFFFFFF00, deadline=0x00002610.
  Heartbeat breakpoint reached with both values equal to 9744 (0x2610).
  User observed approximately 10 seconds, with no immediate stop; no stopwatch.
- Streaming: GlobalTimer advanced from 82670 to 92869: 10199 serviced ticks
  during an approximately 10-second manual interval. Supports correction of
  the former half-rate streaming timer; not a precise frequency measurement.
- Host logged 38036 sample pairs. Two no-CAN warnings occurred during deliberate
  debugger halts; streaming resumed, stop/status/version passed.
- RAM dump: 4094/4094 records received and sequence check passed. Capacity/count
  4094, full/wrapped yes. Sixteen queued CAN frames drained before dump; this
  does not indicate missing dump records. Saved locally in MasterPython/Data/
  as timer_fix_streaming_01.csv and timer_fix_buffer_01.csv (ignored by Git).
- Separate timed idle comparison, ADC timeout injection, exact acquisition
  cadence/jitter, and loss detection were not established by these tests.

After these bench tests, the internal symbol was renamed from the misspelled
HeatbeatTrigger to HeartbeatTrigger in source, regression check, and procedure.
No protocol or runtime behavior changed; version remains 0.0.3.237. The renamed
Debug image was rebuilt and checked but has not been flashed or bench-tested.
The board still uses the old debug symbol until that image is loaded. Avoid
loading the renamed image's symbols onto the previously flashed image.

