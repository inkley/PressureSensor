> Historical checkpoint/reference, preserved for evidence. Statements about what was
> current or pending apply to the dated sections, not the latest firmware.
> Commands/source paths assume repository-root working directories. See
> [current status](../CURRENT_STATUS.md) and the [acceptance procedure](../ACCEPTANCE_PROCEDURE.md).

# GH6PM configuration and flash reservation: 2026-09-22

## Implemented

Debug and Release now select TM4C123GH6PM and define PART_TM4C123GH6PM.
The linker and startup files were renamed to `tm4c123gh6pm.cmd` and
`tm4c123gh6pm_startup_ccs.c`. Startup handler logic was preserved.
The GH6PM target configuration uses the existing Stellaris ICDI connection;
the launch configuration references its GH6PM filename. The older GE6PM-named
target file remains a legacy alias whose device was already GH6PM.

The linker now uses the end of the actual `.stack` allocation for the initial
stack pointer. Previously `__stack + 512` disagreed with Debug's 2048-byte
allocation. Stack sizes remain 2048 bytes in Debug and 512 bytes in Release;
neither size has been validated by a runtime high-water measurement.

## Provisional memory layout

| Purpose | Start | End (exclusive) | Size |
| --- | --- | --- | --- |
| Application flash | 0x00000000 | 0x00020000 | 128 KiB |
| Reserved log flash | 0x00020000 | 0x00040000 | 128 KiB |
| SRAM | 0x20000000 | 0x20008000 | 32 KiB |

The upper flash partition is deliberately absent from linker MEMORY allocation.
Absolute symbols `__flash_log_start__`, `__flash_log_end__`, and
`__flash_log_size__` expose its boundaries for a future storage implementation.
No logger, flash erase/program command, or log data is present in this change.
Both boundaries meet the device's 1 KiB erase alignment.

The application currently boots at address zero; no separate flash-resident
bootloader partition is allocated. Revisit this layout if one is introduced.
Do not treat compiler exclusion as hardware write protection or protection
against debugger mass erase. Configure and test the programmer's erase policy
before relying on stored recordings surviving a firmware update.

At four bytes per two-channel sample and 1000 samples/s, the log partition holds
at most 32768 samples = 32.768 seconds before headers, CRCs, commit markers, spare
blocks, and timestamps. Final capacity remains pending the record format.

## Validation

The installed CCS 12.8.1 / TI ARM 20.2.7.LTS tools built an isolated project copy
from source in both configurations. Both builds completed with zero reported
errors and warnings. Existing tracked Debug artifacts in the source repository
were not replaced.

| Configuration | Flash used (map accounting) | SRAM used | Initial SP |
| --- | --- | --- | --- |
| Debug | 17860 bytes | 19137 bytes | 0x20004A80 |
| Release | 11860 bytes | 17601 bytes | 0x20004480 |

Debug's final flash load extent is 0x45C8 (alignment gaps explain the difference
from map used-byte accounting), well below the 0x20000 application boundary.

`tools/verify_firmware_layout.py` checks each ELF image: partition symbols,
load-segment bounds, initial stack pointer/alignment, and Reset, SysTick, and
CAN0 vector entries. Both images passed. A synthetic 131073-byte constant was
rejected by the linker because it exceeds the 128 KiB application partition;
it could not spill into the logging area.

Validation artifacts are at:
`C:/Users/Lab-User/workspace_v12/gh6pm_validation_20260922/`

Build logs: `Debug_build.log`, `Release_build.log`, `overflow_guard.log`.
Images and maps: `project/Debug/` and `project/Release/`.

For future CCS builds, refresh/reopen the project so renamed source/linker files
are discovered, then use Project > Clean followed by Build for Debug and Release.
Do not invoke the old generated Debug makefile before CCS regenerates it.

```powershell
python tools/verify_firmware_layout.py Debug/Inkley_PressureSensor.out
python tools/verify_firmware_layout.py Release/Inkley_PressureSensor.out
```

## Checklist status

Complete: align Debug/Release device definitions and linker with GH6PM.
Complete: reserve an erase-aligned log partition outside current application code.
Complete: compile/link both configurations and validate image/stack/vector bounds.
Partially complete: MCU model confirmed by user; silicon revisions remain unread.
Partially complete: raw payload capacity calculated; final record format pending.

Not performed: board flashing or post-change hardware tests, clock/cadence
measurement, GlobalTimer correction, persistent logging, RTC synchronization,
power-loss recovery, and flash endurance/fault testing. The currently running
board image remains the previously flashed build. These changes remain local.
