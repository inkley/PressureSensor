# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]
- Avoid blocking in SysTick ISR: CAN transmit frames are now deferred to the main loop.
- Mark shared timing/streaming state as `volatile` for safe ISR/main access.
- Added pending CAN TX buffer for real-time streaming.
- Fixed streaming stopping unexpectedly after ~1000 samples by decimating CAN output and avoiding adapter buffer overflow.
- Added `STREAM_DECIMATE` option to adjust CAN output rate (tune for different adapters).
- Improved host-side logging by buffering CSV writes to prevent I/O stalls.
- Implemented local storage logging: RAM buffering + flash persistence.
- Added command to stream stored flash data back over CAN.
