# Logging partition reservation verification

2026-11-02. Reservation already implemented in tm4c123gh6pm.cmd; no firmware
behavior or linker addresses changed in this verification step.

| Region | Start | End exclusive | Size |
| --- | --- | --- | --- |
| Application, including growth allowance | 0x00000000 | 0x00020000 | 128 KiB |
| Reserved logging | 0x00020000 | 0x00040000 | 128 KiB |

The log reservation consists of 128 individually erasable 1 KiB blocks,
physical block indices 128 through 255 inclusive. Both boundaries are multiples
of 0x400; the last block begins at 0x3FC00. No log erase may touch the application.

Both CCS Debug and Release select tm4c123gh6pm.cmd. Logging is deliberately
absent from the linker's allocatable MEMORY list. FLASH_APP is limited to
0x20000 bytes, and explicit __flash_log_start__, __flash_log_end__, and
__flash_log_size__ symbols expose the bounds to future storage code. Current
ELF verification checks these exact symbols and rejects flash load/run segments
outside the application area. Debug image validation passed in this step;
Release selection was inspected, not rebuilt in this step. Historical Debug
and Release/overflow-guard validation is in GH6PM_BUILD_VALIDATION.md.

No custom flash-resident bootloader is currently configured: vectors start at
zero and the application boots directly. Therefore there is no separate existing
bootloader flash region to avoid. MCU ROM facilities are outside this writable
flash layout. If a custom flash bootloader is added, reserve its lower region,
relocate the application/vectors as required and update validation before use;
do not assume today's application layout is compatible automatically.

The nine metadata/event/spare blocks in FLASH_CAPACITY.md are a proposed
subdivision inside this reservation, not additional linker regions or currently
written flash contents. Final record-format allocation remains pending.

## Required storage-code enforcement (not implemented yet)

- Accept only block-aligned erase addresses inside [start,end); validate length
  with subtraction/bounds checks that cannot overflow. Never issue mass erase
  from the logger. Program operations must satisfy TI alignment/size rules and
  remain wholly within the same reservation.
- Preserve committed unacknowledged recordings. Linker reservation alone does
  not authorize their erase and does not implement power-loss recovery.
- Run the layout verifier after every firmware build used for storage tests.
- Before trusting log survival across firmware downloads, inspect CCS erase
  policy and test preservation with disposable data. No programmer erase-policy
  change or preservation test occurred here.

Status: erase-aligned partition outside the current application is reserved and
verified; no custom flash bootloader exists. This checklist item is addressed
for the current layout. Physical write/erase, recovery and firmware-update
preservation tests belong to later implementation milestones. No board flash
or new build is required merely to confirm this existing reservation.
