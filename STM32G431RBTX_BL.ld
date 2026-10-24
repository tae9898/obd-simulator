/**
  ******************************************************************************
  * @file      STM32G431RBTx_BL.ld
  * @brief     STM32G431RB BOOTLOADER linker script (28KB)
  * @note      CubeMX compatible structure
  ******************************************************************************
  * Memory map (see Core/Inc/bl_handoff.h):
  *   FLASH (ORIGIN = 0x08000000, LENGTH = 28K)    bootloader, app slot at 0x08007000
  *   RAM   (ORIGIN = 0x20000000, LENGTH = 32K-256B) top 256B reserved: BL handoff flag
  ******************************************************************************
  */

/* Input section definitions */
ENTRY(Reset_Handler)

/* Memory region definitions */
MEMORY
{
  RAM    (xrw)      : ORIGIN = 0x20000000, LENGTH = 0x7F00   /* top 256B: BL handoff flag @0x20007F00 */
  FLASH  (rx)       : ORIGIN = 0x08000000, LENGTH = 0x7000   /* bootloader 0x08000000..0x08006FFF */
}

/* Stack and heap sizes */
_estack         = ORIGIN(RAM) + LENGTH(RAM);  /* Stack top = RAM end */
_Min_Heap_Size  = 0x400 ; /* Minimum heap: 1KB */
_Min_Stack_Size = 0x800 ; /* Minimum stack: 2KB (for ISO-TP/UDS ISR handling) */

/* Section definitions */
SECTIONS
{
  /* ====================================================
   * .isr_vector - Interrupt vector table (Flash start)
   * ==================================================== */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))       /* Preserve vector table */
    . = ALIGN(4);
  } >FLASH

  /* ====================================================
   * .text - Program code
   * ==================================================== */
  .text :
  {
    . = ALIGN(4);
    *(.text)                   /* All .text sections (code) */
    *(.text*)                  /* Compiler-generated code sections */
    *(.glue_7)                 /* ARM-Thumb interworking */
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;               /* Code section end marker */
  } >FLASH

  /* ====================================================
   * .rodata - Read-only data
   * ==================================================== */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)                /* Read-only constant data */
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* ====================================================
   * .ARM.extab / .ARM.exidx - Exception handling tables
   * ==================================================== */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* ====================================================
   * .preinit_array / .init_array / .fini_array
   * Constructor/destructor arrays (C++ and newlib init)
   * ==================================================== */
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* ====================================================
   * _sidata - Initialized data source address (Flash)
   * ==================================================== */
  _sidata = LOADADDR(.data);

  /* ====================================================
   * .data - Initialized global/static variables (RAM, copied from Flash)
   * ==================================================== */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;               /* Data start marker */
    *(.data)
    *(.data*)
    *(.RamFunc)               /* Functions executed from RAM */
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;               /* Data end marker */
  } >RAM AT> FLASH

  /* ====================================================
   * .bss - Uninitialized global/static variables (RAM, zero-initialized)
   * ==================================================== */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;                /* BSS start marker */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;                /* BSS end marker */
    __bss_end__ = _ebss;
  } >RAM

  /* ====================================================
   * ._user_heap_stack - User heap and stack
   * ==================================================== */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* ====================================================
   * ARM attributes removal (suppress warnings)
   * ==================================================== */
  /DISCARD/ :
  {
    *(.ARM.attributes)
  }
}
