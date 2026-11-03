# Inkley Pressure Sensor Firmware

Firmware for the Inkley pressure sensor module, built for TI Tiva C Series
microcontrollers using Code Composer Studio (CCS).

The current firmware streams two pressure channels over CAN and stores recent
samples in a circular RAM buffer that can be dumped to a host PC after
streaming stops.

See [Timing and storage development notes](TIMING_AND_STORAGE.md) for the
verified timing architecture, RTC/PCB recommendations, and staged flash-logging
plan, including the current versus proposed MCU memory sizes.

## Hardware And Toolchain

- Hardware MCU: TI Tiva C / TM4C123GH6PM (bench and planned PCB, user-confirmed;
  256 KiB flash, 32 KiB SRAM).
- Debug/Release builds target GH6PM, with 128 KiB application flash and 128 KiB
  reserved for future logging. See [build validation](GH6PM_BUILD_VALIDATION.md).
- IDE: Code Composer Studio
- CAN bitrate: 1 Mbps
- Host CLI: <https://github.com/inkley/MasterPython>

## Repository Contents

- `main.c` - application entry point and firmware logic.
- `tm4c123gh6pm_startup_ccs.c` - CCS startup file.
- `tm4c123gh6pm.cmd` - linker command file and reserved log boundaries.
- `driverlib/`, `sensorlib/`, `usblib/`, `utils/`, `inc/` - TivaWare support code.
- `targetConfigs/` - CCS target configuration files.
- `Debug/` - generated build output and intermediate files.

Generated files under `Debug/` should generally not be committed.

## Build And Flash

1. Open this project in Code Composer Studio.
2. Build the desired configuration.
3. Flash the firmware to the TM4C123G-based evaluation board.
4. Connect the CAN transceiver and host USB CAN adapter.
5. Use the MasterPython CLI to run command, streaming, and buffer-dump tests.

## CAN Protocol Summary

- Module command CAN ID: `0x107`
- Realtime broadcast CAN ID: `0x7DF`
- Host response CAN ID: provided in command payload, commonly `0x108`
- Frame size: 8 bytes

Command payload sent to `0x107`:

```text
[0]    command
[1..2] response CAN ID
[3..6] uint32 argument, big-endian
[7]    reserved
```

Response payload:

```text
[0]    length
[1..2] source CAN ID
[3]    command echo or frame tag
[4..7] uint32 value or packed sample data
```

Implemented commands:

- `0x01` - read firmware version.
- `0x02` - start real-time streaming and RAM buffering.
- `0x03` - dump buffered RAM samples.
- `0x04` - stop streaming.
- `0x05` - read packed streaming/RAM buffer status.
- `0x06` - set RAM stream buffer size.
- `0x07` - buffered sample playback payload.

Status response `0x05` packs mode, flags, capacity, and count into bytes
`[4..7]`:

```text
bits 31..28  streaming mode (0=stopped, 1=realtime, 2=buffered)
bit  27      RAM buffer has wrapped/full flag
bit  26      buffer dump active flag
bits 23..12  configured RAM buffer capacity
bits 11..0   current buffered sample count
```

## Realtime Streaming

When streaming is active, SysTick samples two ADC channels at the configured
period and stores each `Pressure1`/`Pressure2` pair in the RAM buffer.

Realtime CAN broadcast frames use ID `0x7DF`. The optimized frame type `0x06`
packs two pressure sample pairs into a single 8-byte frame to reduce bus load.
If the host adapter cannot keep up, increase `STREAM_DECIMATE` in `main.c`.

## RAM Buffer Dump

The RAM buffer is circular. Starting real-time streaming resets it, then each
sample pair is added while streaming remains active. Stopping streaming leaves
the buffered records in RAM so the host can request them with command `0x03`.

The firmware replies first with the buffered record count, then sends each
sample pair as a `0x07` playback frame. Bytes `[1..2]` of each `0x07` frame
carry a 16-bit sequence index so the host can detect dropped or reordered
buffered playback frames. RAM contents are expected to be lost after power
cycle.

Current validation results:

- Empty buffer after power cycle returns `0` records.
- A 256-sample buffer wraps and dumps exactly `256` records.
- An oversized buffer request, such as `999999`, is capped at `4094` records.
- A full `4094`-sample RAM dump completes successfully over CAN.

## Local Storage Guidance

RAM buffering is the right first-stage architecture for deterministic sampling
and CAN retrieval validation. For longer-duration local logging, this TM4C
part's internal RAM is capacity-limited to seconds, so the next hardware step
should be external FRAM or flash. FRAM is preferred for high-endurance
time-series logging.

Current RAM storage math:

- Buffered sample format: one `Pressure1`/`Pressure2` pair per `uint32_t`.
- Storage rate at 1000 Hz: `4 bytes/sample * 1000 samples/s = 4 KB/s`.
- Current tested capacity: `4094 samples`.
- Current tested duration: `4094 / 1000 Hz = 4.094 seconds`.

Longer-duration storage estimates at the current 4-byte sample-pair format:

```text
30 sec  -> 120 KB
60 sec  -> 240 KB
5 min   -> 1.2 MB
10 min  -> 2.4 MB
```

Storage recommendations:

- Keep RAM as the primary recent-sample ring buffer. It is useful for
  debugging, smoke tests, short bursts, and validating CAN dump behavior.
- Avoid internal flash for normal high-rate sample logging. Flash has
  erase-block constraints, slower writes, and endurance limits, and it can
  disturb sampling timing if handled poorly.
- Use MCU EEPROM only for configuration/calibration records. It is appropriate
  for settings, calibration constants, and device ID, but it is too small for
  sampled data.
- For longer local storage, add external SPI FRAM. FRAM avoids
  erase-before-write complexity, has high endurance, supports simple circular
  log semantics, and is more predictable for repeated sampling.
- Use external SPI flash only when storage capacity matters more than endurance
  and erase-block management complexity.

## Host Test Sequence

Run the Python CLI from the MasterPython repo:

```powershell
python .\InkleySensor.py
```

Inside the CLI:

```text
version
buffer_status
set_buffer_size 999999
set_filename realtime_maxcap.csv
start
```

Wait about 5 seconds, then:

```text
stop
buffer_status
set_filename buffer_maxcap.csv
dump_buffer
```

Expected result: `Buffered record count: 4094` and a saved CSV containing
`4094` buffered samples.

## Notes

- Keep high-rate CAN transmit work out of interrupt handlers when possible.
- The sampling path should remain constant time and should not write flash.
- Buffer dumps should be run after stopping real-time streaming.
- Buffered playback frames include a 16-bit sequence index for host-side
  dropped/reordered frame checks. If dumps grow beyond 65535 records later,
  extend this to a wider counter or multi-frame payload format.

See [acquisition accounting definitions](ACQUISITION_ACCOUNTING.md) for proposed
session identity, counters, gap reporting, and implementation acceptance tests.

See [acquisition timing requirements](ACQUISITION_TIMING.md) for sequential channel timing, provisional jitter limits, and gap policy.

See [provisional flash capacity](FLASH_CAPACITY.md) for linker-map evidence, metadata/recovery allowances, and record-size comparisons.

See [logging partition verification](FLASH_PARTITION.md) for aligned boundaries, bootloader status, and storage implementation constraints.

See [flash timing, endurance and errata review](FLASH_REVIEW.md) for device limits and implementation gates.

See [recording lifecycle](RECORDING_STATES.md) for capture, stop, persist, transfer, acknowledgement, clear and restart behavior.

See [binary recording format v1](RECORD_FORMAT.md) and the offline reference codec/tests in tools/log_format.py and tools/test_log_format.py.

See [stopped-acquisition flash implementation and bench procedure](STOPPED_FLASH_TEST.md) for build 1006 and new capability-gated commands. Hardware validation pending.

See [recovery and fault testing](FLASH_RECOVERY_TEST.md) for build 1007, read-only recovery, scratch-only fixtures, and pending physical power-cut validation.

See [verified-transfer lifecycle](FLASH_LIFECYCLE.md) for build 1008 ACK, explicit clear, durable intent and capture restart; hardware validation pending.

See [full-storage, overwrite and wear policies](STORAGE_POLICY.md) for default retention/backpressure rules, conservative wear budgets and pending deployment enforcement.

See [continuous logging evaluation](CONTINUOUS_LOGGING_EVALUATION.md) and tools/evaluate_staging.py for RAM budgets, flash execution dependencies and implementation gates.

See [timing measurements](TIMING_MEASUREMENT.md) for .241 counters, measured acquisition/CAN and 1000-pair stopped-flash results, and the pending 4094-pair bench procedure.
