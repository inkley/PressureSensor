# Inkley Pressure Sensor Firmware

This repository contains firmware and supporting code for the Inkley pressure sensor project. It is built for TI Tiva C Series microcontrollers (TM4C123G) using Code Composer Studio (CCS).

## Contents

- `main.c` - Application entry point and main firmware logic.
- `driverlib/`, `sensorlib/`, `usblib/` - TI library code and sensor drivers.
- `Debug/` - Build output and intermediate files (should be ignored by git).

## Getting Started

1. Open `Inkley_PressureSensor.ccsproject` in Code Composer Studio.
2. Build and flash the firmware to a TM4C123G-based board.

## Notes

- This repository currently contains generated build artifacts in `Debug/`; avoid committing those changes when possible.
- If the firmware stops streaming after ~1 second, it may be due to CAN adapter buffering; adjust `STREAM_DECIMATE` in `main.c` to reduce bus load.
- Use the `CHANGES.md` file to track release notes and notable changes.
