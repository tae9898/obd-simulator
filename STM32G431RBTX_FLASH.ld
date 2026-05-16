/**
  ******************************************************************************
  * @file      STM32G431RBTx_FLASH.ld
  * @brief     STM32G431RB 링커 스크립트 (128KB Flash, 32KB RAM)
  * @note      CubeMX 호환 구조
  ******************************************************************************
  * 메모리 맵:
  *   FLASH (ORIGIN = 0x08000000, LENGTH = 128K)
  *   RAM   (ORIGIN = 0x20000000, LENGTH =  32K)
  ******************************************************************************
  */

/* 입력 섹션 정의 */
ENTRY(Reset_Handler)

/* 메모리 영역 정의 */
MEMORY
{
  RAM    (xrw)      : ORIGIN = 0x20000000, LENGTH = 32K
  FLASH  (rx)       : ORIGIN = 0x08000000, LENGTH = 128K
}

/* 스택 및 힙 크기 */
_estack         = ORIGIN(RAM) + LENGTH(RAM);  /* 스택 top = RAM 끝 */
_Min_Heap_Size  = 0x400 ; /* 최소 힙: 1KB */
_Min_Stack_Size = 0x800 ; /* 최소 스택: 2KB (ISO-TP/UDS ISR 처리용) */

/* 섹션 정의 */
SECTIONS
{
  /* ====================================================
   * .isr_vector - 인터럽트 벡터 테이블 (Flash 시작)
   * ==================================================== */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))       /* 벡터 테이블 보존 */
    . = ALIGN(4);
  } >FLASH

  /* ====================================================
   * .text - 프로그램 코드
   * ==================================================== */
  .text :
  {
    . = ALIGN(4);
    *(.text)                   /* 모든 .text 섹션 (코드) */
    *(.text*)                  /* 컴파일러 생성 코드 섹션 */
    *(.glue_7)                 /* ARM-Thumb 상호호환 */
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;               /* 코드 섹션 끝 마커 */
  } >FLASH

  /* ====================================================
   * .rodata - 읽기 전용 데이터
   * ==================================================== */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)                /* 읽기 전용 상수 데이터 */
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* ====================================================
   * .ARM.extab / .ARM.exidx - 예외 처리 테이블
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
   * 생성자/소멸자 배열 (C++ 및 newlib 초기화)
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
   * _sidata - 초기화된 데이터 소스 주소 (Flash)
   * ==================================================== */
  _sidata = LOADADDR(.data);

  /* ====================================================
   * .data - 초기화된 전역/정적 변수 (RAM, Flash에서 복사)
   * ==================================================== */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;               /* 데이터 시작 마커 */
    *(.data)
    *(.data*)
    *(.RamFunc)               /* RAM에서 실행되는 함수 */
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;               /* 데이터 끝 마커 */
  } >RAM AT> FLASH

  /* ====================================================
   * .bss - 초기화되지 않은 전역/정적 변수 (RAM, 제로 초기화)
   * ==================================================== */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;                /* BSS 시작 마커 */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;                /* BSS 끝 마커 */
    __bss_end__ = _ebss;
  } >RAM

  /* ====================================================
   * ._user_heap_stack - 사용자 힙 및 스택
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
   * ARM 속성 제거 (경고 억제)
   * ==================================================== */
  /DISCARD/ :
  {
    *(.ARM.attributes)
  }
}
