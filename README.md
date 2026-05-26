# Inkley Pressure Sensor Firmware

Firmware for the Inkley pressure sensor module, built for TI Tiva C Series
microcontrollers using Code Composer Studio (CCS).

The current firmware streams two pressure channels over CAN and stores recent
samples in a circular RAM buffer that can be dumped to a host PC after
streaming stops.

## Hardware And Toolchain

- Target MCU family: TI Tiva C / TM4C123G
- IDE: Code Composer Studio
- CAN bitrate: 1 Mbps
- Host CLI: <https://github.com/inkley/MasterPython>

## Repository Contents

- `main.c` - application entry point and firmware logic.
- `tm4c123ge6pm_startup_ccs.c` - CCS startup file.
- `tm4c123ge6pm.cmd` - linker command file.
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
