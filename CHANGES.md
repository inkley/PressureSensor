# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

- Replaced flash-backed sample logging with a RAM-first circular buffer dump workflow.
- Added CAN command `0x05` packed buffer status reporting:
  - streaming mode
  - configured RAM buffer capacity
  - current buffered sample count
  - wrapped/full flag
  - dump-active flag
- Kept wire value `0x07` for buffered playback payloads, but renamed the active firmware concept to buffered sample data.
- Added 16-bit sequence indexes to buffered playback frames in bytes `[1..2]`.
- Validated RAM buffer behavior on 2026-05-26:
  - empty buffer after power cycle returned `0`
  - 256-sample wrap test returned `256` with sequence validation passing
  - oversized `999999` buffer request capped at `4094`
  - full `4094`-sample dump completed over CAN with sequence validation passing
- Avoided flash writes in the active sampling path.
- Real-time CAN transmit frames are deferred to the main loop rather than sent directly from SysTick.
- `STREAM_DECIMATE` remains available to tune realtime bus load for different CAN adapters.
