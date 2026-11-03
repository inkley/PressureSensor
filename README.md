# Inkley pressure sensor firmware

TM4C123GH6PM firmware for two independent differential-pressure channels at nominal
1000 P1/P2 pairs/s. Validated baseline **build 1009 / version 0.0.3.241** supports
bounded RAM capture, stopped flash persistence, verified CAN retrieval, durable
ACK, explicit identity-bound clear/restart, and clean-reboot retention.

- [Current validated state and open work](docs/CURRENT_STATUS.md)
- [Repeatable acceptance procedure](docs/ACCEPTANCE_PROCEDURE.md)
- [Reference specifications and historical test evidence](docs/README.md)
- Host tools: [MasterPython](https://github.com/inkley/MasterPython)

Candidate **.242/build1010** implements measured v2 capture, events, persistence
and lifecycle with Python exports/plots. [Hardware validation pending](docs/reference/MEASURED_IMPLEMENTATION.md).

## Hardware and layout

256 KiB flash,32 KiB SRAM. Application [0x00000,0x20000); erase-aligned logging
[0x20000,0x40000). No separate bootloader currently reserved. Current capture limit
4,094 pairs (~4.094s nominal); single retained recording. 10--30s and continuous
logging remain goals. Eval target revision7; custom PCB revision pending readout.
Verified system clock 20 MHz; SysTick reload 19999. P1/AIN0 then P2/AIN1 are sequential.

## Build and flash

Validated: CCS 12.8, TI ARM 20.2.7.LTS. From this directory in PowerShell:

```powershell
& C:\ti\ccs1281\ccs\utils\bin\gmake.exe -s -C Debug all
python tools/verify_firmware_layout.py Debug/Inkley_PressureSensor.out
```

CCS: correct GH6PM target, 16 MHz external crystal setting, **Necessary Pages Only**.
Load the matching built .out, then resume. Preserve/download valid recordings
before updating; do not mass erase. No reflash is needed for documentation changes.
Manual debug attach can inspect without a program download; load matching symbols
when needed. Avoid halt/step/breakpoints during timing measurements.

## Source map

- main.c: commands, ADC, SysTick and CAN ownership.
- flash_log.h: bounded capture and flash format/persistence.
- flash_lifecycle.h: journalled ACK/clear and recovery.
- measured_capture.h: measured1024-pair capture, v2 serialization/recovery/lifecycle.
- timing_diag.h: free-running timer diagnostics and CRC snapshot.
- tm4c123gh6pm.cmd / startup file: linker layout and vector/startup configuration.
- makefile.defs: source-owned header dependencies for incremental builds.
- tools/: offline format/timer checks, layout and staging evaluation.
- driverlib/, sensorlib/, usblib/, utils/, inc/: TivaWare support.
- targetConfigs/: CCS target configurations.
- Debug/: existing tracked build artifacts; this documentation change does not
  change their tracking policy. Build/source identity matters for acceptance.

CAN 1 Mbps: command 0x107, replies typically0x108, broadcast 0x7DF. Host commands and
the complete acceptance sequence are in the linked runbook. Detailed binary format,
partition, storage policies and timing requirements are indexed under docs/reference.
Historical benchmark evidence is under docs/archive with explicit scope notices.

## Verification and limitations

```powershell
python tools/test_timer.py
python tools/test_log_format.py
```

Baseline: 5 timer + 8 format tests, plus 76 host checks (including21 offline-only v2 groups). Bounded hardware checkpoint
passed through 4094 pairs, exact RAM/flash comparison and clean reboot. Normal CAN
acquisition timing passed; flash operations deliberately stop acquisition and can
mask interrupts for long intervals. Saved sample timestamps remain nominal-only.
Physical busy-window power cuts, continuous acquisition through flash, UTC/RTC,
independent oscillator calibration and custom PCB acceptance remain open.
