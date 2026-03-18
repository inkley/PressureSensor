# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]
- Avoid blocking in SysTick ISR: CAN transmit frames are now deferred to the main loop.
- Mark shared timing/streaming state as `volatile` for safe ISR/main access.
- Added pending CAN TX buffer for real-time streaming.
- Implemented local storage logging: RAM buffering + flash persistence.
- Added command to stream stored flash data back over CAN.
