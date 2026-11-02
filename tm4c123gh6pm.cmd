/******************************************************************************
 *
 * Default Linker Command file for the Texas Instruments TM4C123GH6PM
 *
 * This is derived from revision 15071 of the TivaWare Library.
 *
 *****************************************************************************/

--retain=g_pfnVectors

MEMORY
{
    FLASH_APP (RX) : origin = 0x00000000, length = 0x00020000
    SRAM (RWX) : origin = 0x20000000, length = 0x00008000
}

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M4_T_le_eabi.lib                                           */

/* Section allocation in memory */

SECTIONS
{
    .intvecs:   > 0x00000000
    .text   :   > FLASH_APP
    .const  :   > FLASH_APP
    .cinit  :   > FLASH_APP
    .pinit  :   > FLASH_APP
    .init_array : > FLASH_APP

    .vtable :   > 0x20000000
    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM, RUN_END(__STACK_TOP)
}

// __STACK_TOP is the end of the actual .stack allocation above.

// Upper 128 KiB reserved for future logging; deliberately absent from MEMORY
// so no executable, constant, or orphan section can be allocated there.
// End address is exclusive. No logging contents are emitted by this image.
__flash_log_start__ = 0x00020000;
__flash_log_end__ = 0x00040000;
__flash_log_size__ = __flash_log_end__ - __flash_log_start__;
