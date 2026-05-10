/**
  ******************************************************************************
  * @file      startup_stm32g431xx.s
  * @brief     STM32G431xx Cortex-M4 시작 어셈블리
  * @note      CubeMX 호환 startup 파일
  ******************************************************************************
  * 기능:
  *   - 스택 및 힙 크기 정의
  *   - 인터럽트 벡터 테이블 구성
  *   - Reset 핸들러: .data 복사, .bss 제로 초기화, main 호출
  *   - 미구현 인터럽트 핸들러: 무한 루프
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* ============================================
 * 시작 코드는 Flash에 배치
 * ============================================ */
.startup:

/* ============================================
 * 스택 크기 설정 (8바이트 정렬)
 * ============================================ */
.stack:
  .syntax unified
  .thumb
  .align 3
  .word  Stack_Size            /* 스택 크기 값 저장 */

/* ============================================
 * 힙 크기 설정
 * ============================================ */
.heap:
  .syntax unified
  .thumb
  .align 3
  .word  Heap_Size             /* 힙 크기 값 저장 */

/* ============================================
 * Vector Table (벡터 테이블)
 * STM32G431 전체 인터럽트 벡터
 * ============================================ */
  .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word  _estack               /* 초기 스택 포인터 */
  .word  Reset_Handler         /* 리셋 핸들러 */
  .word  NMI_Handler           /* NMI 핸들러 */
  .word  HardFault_Handler     /* 하드폴트 핸들러 */
  .word  MemManage_Handler     /* 메모리 관리 폴트 */
  .word  BusFault_Handler      /* 버스 폴트 */
  .word  UsageFault_Handler    /* 사용 폴트 */
  .word  0                     /* 예약됨 */
  .word  0                     /* 예약됨 */
  .word  0                     /* 예약됨 */
  .word  0                     /* 예약됨 */
  .word  SVC_Handler           /* SVCall 핸들러 */
  .word  DebugMon_Handler      /* 디버그 모니터 */
  .word  0                     /* 예약됨 */
  .word  PendSV_Handler        /* PendSV 핸들러 */
  .word  SysTick_Handler       /* SysTick 핸들러 */

  /* --- 외부 인터럽트 벡터 --- */

  .word  WWDG_IRQHandler                   /* [0]  Window Watchdog */
  .word  PVD_IRQHandler                    /* [1]  PVM through EXTI Line detection */
  .word  RTC_TAMP_IRQHandler               /* [2]  RTC through EXTI Line */
  .word  FLASH_IRQHandler                  /* [3]  FLASH */
  .word  RCC_IRQHandler                    /* [4]  RCC */
  .word  EXTI0_IRQHandler                  /* [5]  EXTI Line 0 */
  .word  EXTI1_IRQHandler                  /* [6]  EXTI Line 1 */
  .word  EXTI2_IRQHandler                  /* [7]  EXTI Line 2 */
  .word  EXTI3_IRQHandler                  /* [8]  EXTI Line 3 */
  .word  EXTI4_IRQHandler                  /* [9]  EXTI Line 4 */
  .word  DMA1_Channel1_IRQHandler          /* [10] DMA1 Channel 1 */
  .word  DMA1_Channel2_IRQHandler          /* [11] DMA1 Channel 2 */
  .word  DMA1_Channel3_IRQHandler          /* [12] DMA1 Channel 3 */
  .word  DMA1_Channel4_IRQHandler          /* [13] DMA1 Channel 4 */
  .word  DMA1_Channel5_IRQHandler          /* [14] DMA1 Channel 5 */
  .word  DMA1_Channel6_IRQHandler          /* [15] DMA1 Channel 6 */
  .word  0                                 /* [16] 예약됨 */
  .word  ADC1_2_IRQHandler                 /* [17] ADC1 & ADC2 */
  .word  USB_HP_IRQHandler                 /* [18] USB Device High Priority */
  .word  USB_LP_IRQHandler                 /* [19] USB Device Low Priority */
  .word  FDCAN1_IT0_IRQHandler            /* [20] FDCAN1 Interrupt 0 */
  .word  FDCAN1_IT1_IRQHandler            /* [21] FDCAN1 Interrupt 1 */
  .word  EXTI9_5_IRQHandler               /* [22] EXTI Lines [9:5] */
  .word  TIM1_BRK_TIM15_IRQHandler        /* [23] TIM1 Break & TIM15 */
  .word  TIM1_UP_TIM16_IRQHandler         /* [24] TIM1 Update & TIM16 */
  .word  TIM1_TRG_COM_TIM17_IRQHandler    /* [25] TIM1 Trigger/Commutation & TIM17 */
  .word  TIM1_CC_IRQHandler                /* [26] TIM1 Capture Compare */
  .word  TIM2_IRQHandler                   /* [27] TIM2 */
  .word  TIM3_IRQHandler                   /* [28] TIM3 */
  .word  TIM4_IRQHandler                   /* [29] TIM4 */
  .word  I2C1_EV_IRQHandler                /* [30] I2C1 Event */
  .word  I2C1_ER_IRQHandler                /* [31] I2C1 Error */
  .word  I2C2_EV_IRQHandler                /* [32] I2C2 Event */
  .word  I2C2_ER_IRQHandler                /* [33] I2C2 Error */
  .word  SPI1_IRQHandler                   /* [34] SPI1 */
  .word  SPI2_IRQHandler                   /* [35] SPI2 */
  .word  USART1_IRQHandler                 /* [36] USART1 */
  .word  USART2_IRQHandler                 /* [37] USART2 */
  .word  USART3_IRQHandler                 /* [38] USART3 */
  .word  EXTI15_10_IRQHandler              /* [39] EXTI Lines [15:10] */
  .word  RTC_Alarm_IRQHandler              /* [40] RTC Alarm through EXTI */
  .word  USBWakeUp_IRQHandler              /* [41] USB Wakeup through EXTI */
  .word  TIM8_BRK_IRQHandler              /* [42] TIM8 Break */
  .word  TIM8_UP_IRQHandler               /* [43] TIM8 Update */
  .word  TIM8_TRG_COM_IRQHandler          /* [44] TIM8 Trigger/Commutation */
  .word  TIM8_CC_IRQHandler               /* [45] TIM8 Capture Compare */
  .word  0                                 /* [46] 예약됨 */
  .word  0                                 /* [47] 예약됨 */
  .word  0                                 /* [48] 예약됨 */
  .word  0                                 /* [49] 예약됨 */
  .word  SPI3_IRQHandler                   /* [50] SPI3 */
  .word  UART4_IRQHandler                  /* [51] UART4 */
  .word  0                                 /* [52] 예약됨 */
  .word  TIM6_DAC_IRQHandler               /* [53] TIM6 & DAC1 underrun */
  .word  TIM7_IRQHandler                   /* [54] TIM7 */
  .word  DMA2_Channel1_IRQHandler          /* [55] DMA2 Channel 1 */
  .word  DMA2_Channel2_IRQHandler          /* [56] DMA2 Channel 2 */
  .word  DMA2_Channel3_IRQHandler          /* [57] DMA2 Channel 3 */
  .word  DMA2_Channel4_IRQHandler          /* [58] DMA2 Channel 4 */
  .word  DMA2_Channel5_IRQHandler          /* [59] DMA2 Channel 5 */
  .word  0                                 /* [60] 예약됨 */
  .word  0                                 /* [61] 예약됨 */
  .word  UCPD1_IRQHandler                  /* [62] UCPD1 */
  .word  COMP1_2_3_IRQHandler              /* [63] COMP1, COMP2 & COMP3 */
  .word  COMP4_IRQHandler                  /* [64] COMP4 */
  .word  0                                 /* [65] 예약됨 */
  .word  0                                 /* [66] 예약됨 */
  .word  0                                 /* [67] 예약됨 */
  .word  0                                 /* [68] 예약됨 */
  .word  0                                 /* [69] 예약됨 */
  .word  0                                 /* [70] 예약됨 */
  .word  0                                 /* [71] 예약됨 */
  .word  0                                 /* [72] 예약됨 */
  .word  0                                 /* [73] 예약됨 */
  .word  0                                 /* [74] 예약됨 */
  .word  CRS_IRQHandler                    /* [75] CRS */
  .word  SAI1_IRQHandler                   /* [76] SAI1 */
  .word  0                                 /* [77] 예약됨 */
  .word  0                                 /* [78] 예약됨 */
  .word  0                                 /* [79] 예약됨 */
  .word  FPU_IRQHandler                    /* [80] FPU */
  .word  0                                 /* [81] 예약됨 */
  .word  0                                 /* [82] 예약됨 */
  .word  0                                 /* [83] 예약됨 */
  .word  0                                 /* [84] 예약됨 */
  .word  RNG_IRQHandler                    /* [85] RNG */
  .word  LPUART1_IRQHandler                /* [86] LPUART1 */
  .word  I2C3_EV_IRQHandler                /* [87] I2C3 Event */
  .word  I2C3_ER_IRQHandler                /* [88] I2C3 Error */
  .word  DMAMUX_OVR_IRQHandler             /* [89] DMAMUX Overrun */
  .word  0                                 /* [90] 예약됨 */
  .word  0                                 /* [91] 예약됨 */
  .word  0                                 /* [92] 예약됨 */
  .word  0                                 /* [93] 예약됨 */
  .word  0                                 /* [94] 예약됨 */
  .word  0                                 /* [95] 예약됨 */
  .word  0                                 /* [96] 예약됨 */
  .word  0                                 /* [97] 예약됨 */
  .word  0                                 /* [98] 예약됨 */
  .word  0                                 /* [99] 예약됨 */
  .word  0                                 /* [100] 예약됨 */
  .word  0                                 /* [101] 예약됨 */
  .word  DAC2_IRQHandler                   /* [102] DAC2 */
  .word  0                                 /* [103] 예약됨 */
  .word  0                                 /* [104] 예약됨 */
  .word  0                                 /* [105] 예약됨 */
  .word  LPTIM1_IRQHandler                 /* [106] LPTIM1 */
  .word  LPTIM2_IRQHandler                 /* [107] LPTIM2 */

/*******************************************************************************
*
* Reset_Handler: 프로세서 리셋 후 최초 실행 코드
*   1. .data 섹션을 Flash에서 RAM으로 복사
*   2. .bss 섹션을 제로로 초기화
*   3. FPU 활성화 (Cortex-M4F)
*   4. SystemInit() 호출 (클럭 설정)
*   5. __libc_init_array() 호출 (C 런타임 초기화)
*   6. main() 호출
*
*******************************************************************************/
  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0                   /* 스택 포인터 설정 */

  /* .data 섹션을 Flash에서 RAM으로 복사 */
  ldr   r0, =_sdata              /* RAM 목적지 시작 */
  ldr   r1, =_edata              /* RAM 목적지 끝 */
  ldr   r2, =_sidata             /* Flash 소스 시작 */
copy_data_init:
  cmp   r0, r1
  ittt  lt
  ldrlt r3, [r2], #4
  strlt r3, [r0], #4
  blt   copy_data_init

  /* .bss 섹션을 제로로 초기화 */
  ldr   r0, =_sbss               /* BSS 시작 */
  ldr   r1, =_ebss               /* BSS 끝 */
  mov   r2, #0
zero_bss_init:
  cmp   r0, r1
  itt   lt
  strlt r2, [r0], #4
  blt   zero_bss_init

  /* FPU (부동소수점 유닛) 활성화 */
  ldr   r0, =0xE000ED88          /* CPACR 레지스터 주소 */
  ldr   r1, [r0]
  orr   r1, r1, #(0xF << 20)     /* CP10, CP11 Full Access 설정 */
  str   r1, [r0]
  dsb
  isb

  /* SystemInit 호출 (클럭 시스템 초기화) */
  bl    SystemInit

  /* C 런타임 초기화 (생성자 호출 등) */
  bl    __libc_init_array

  /* main 함수 호출 */
  bl    main

  /* main이 리턴되면 무한 루프 */
halt_loop:
  b     halt_loop

.size  Reset_Handler, .-Reset_Handler

/*******************************************************************************
*
* 기본 핸들러 (미구현 인터럽트)
* 모든 정의되지 않은 인터럽트는 여기로 분기하여 무한 루프
*
*******************************************************************************/
  .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b     Infinite_Loop
.size  Default_Handler, .-Default_Handler

/*******************************************************************************
*
* Cortex-M4 코어 예외 핸들러 (기본 구현: 무한 루프)
* 사용자가 stm32g4xx_it.c 에서 재정의 가능
*
*******************************************************************************/
  .weak  NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak  HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak  MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak  BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak  UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak  SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak  DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak  PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak  SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

/*******************************************************************************
*
* 외부 인터럽트 핸들러 (기본 구현: 무한 루프)
* 사용자가 stm32g4xx_it.c 에서 재정의 가능
*
*******************************************************************************/
  .weak  WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak  PVD_IRQHandler
  .thumb_set PVD_IRQHandler,Default_Handler

  .weak  RTC_TAMP_IRQHandler
  .thumb_set RTC_TAMP_IRQHandler,Default_Handler

  .weak  FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak  RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak  EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak  EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak  EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak  EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak  EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak  DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak  DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

  .weak  DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

  .weak  DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

  .weak  DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

  .weak  DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

  .weak  ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler,Default_Handler

  .weak  USB_HP_IRQHandler
  .thumb_set USB_HP_IRQHandler,Default_Handler

  .weak  USB_LP_IRQHandler
  .thumb_set USB_LP_IRQHandler,Default_Handler

  .weak  FDCAN1_IT0_IRQHandler
  .thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

  .weak  FDCAN1_IT1_IRQHandler
  .thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

  .weak  EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Default_Handler

  .weak  TIM1_BRK_TIM15_IRQHandler
  .thumb_set TIM1_BRK_TIM15_IRQHandler,Default_Handler

  .weak  TIM1_UP_TIM16_IRQHandler
  .thumb_set TIM1_UP_TIM16_IRQHandler,Default_Handler

  .weak  TIM1_TRG_COM_TIM17_IRQHandler
  .thumb_set TIM1_TRG_COM_TIM17_IRQHandler,Default_Handler

  .weak  TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak  TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak  TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak  TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak  I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak  I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak  I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak  I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak  SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak  SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak  USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak  USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak  USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Default_Handler

  .weak  EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Default_Handler

  .weak  RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,Default_Handler

  .weak  USBWakeUp_IRQHandler
  .thumb_set USBWakeUp_IRQHandler,Default_Handler

  .weak  TIM8_BRK_IRQHandler
  .thumb_set TIM8_BRK_IRQHandler,Default_Handler

  .weak  TIM8_UP_IRQHandler
  .thumb_set TIM8_UP_IRQHandler,Default_Handler

  .weak  TIM8_TRG_COM_IRQHandler
  .thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

  .weak  TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler,Default_Handler

  .weak  SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak  UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak  TIM6_DAC_IRQHandler
  .thumb_set TIM6_DAC_IRQHandler,Default_Handler

  .weak  TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak  DMA2_Channel1_IRQHandler
  .thumb_set DMA2_Channel1_IRQHandler,Default_Handler

  .weak  DMA2_Channel2_IRQHandler
  .thumb_set DMA2_Channel2_IRQHandler,Default_Handler

  .weak  DMA2_Channel3_IRQHandler
  .thumb_set DMA2_Channel3_IRQHandler,Default_Handler

  .weak  DMA2_Channel4_IRQHandler
  .thumb_set DMA2_Channel4_IRQHandler,Default_Handler

  .weak  DMA2_Channel5_IRQHandler
  .thumb_set DMA2_Channel5_IRQHandler,Default_Handler

  .weak  UCPD1_IRQHandler
  .thumb_set UCPD1_IRQHandler,Default_Handler

  .weak  COMP1_2_3_IRQHandler
  .thumb_set COMP1_2_3_IRQHandler,Default_Handler

  .weak  COMP4_IRQHandler
  .thumb_set COMP4_IRQHandler,Default_Handler

  .weak  CRS_IRQHandler
  .thumb_set CRS_IRQHandler,Default_Handler

  .weak  SAI1_IRQHandler
  .thumb_set SAI1_IRQHandler,Default_Handler

  .weak  FPU_IRQHandler
  .thumb_set FPU_IRQHandler,Default_Handler

  .weak  RNG_IRQHandler
  .thumb_set RNG_IRQHandler,Default_Handler

  .weak  LPUART1_IRQHandler
  .thumb_set LPUART1_IRQHandler,Default_Handler

  .weak  I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak  I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak  DMAMUX_OVR_IRQHandler
  .thumb_set DMAMUX_OVR_IRQHandler,Default_Handler

  .weak  DAC2_IRQHandler
  .thumb_set DAC2_IRQHandler,Default_Handler

  .weak  LPTIM1_IRQHandler
  .thumb_set LPTIM1_IRQHandler,Default_Handler

  .weak  LPTIM2_IRQHandler
  .thumb_set LPTIM2_IRQHandler,Default_Handler

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
