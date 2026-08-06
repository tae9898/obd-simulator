/**
  ******************************************************************************
  * @file      startup_stm32g431xx.s
  * @brief     STM32G431xx Cortex-M4 startup assembly
  * @note      CubeMX compatible startup file
  ******************************************************************************
  * Features:
  *   - Stack and heap size definitions
  *   - Interrupt vector table configuration
  *   - Reset handler: .data copy, .bss zero init, call main
  *   - Unimplemented interrupt handlers: infinite loop
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* ============================================
 * Startup code is placed in Flash
 * ============================================ */
.startup:

/* ============================================
 * Stack size setting (8-byte aligned)
 * ============================================ */
.stack:
  .syntax unified
  .thumb
  .align 3
  .word  Stack_Size            /* Store stack size value */

/* ============================================
 * Heap size setting
 * ============================================ */
.heap:
  .syntax unified
  .thumb
  .align 3
  .word  Heap_Size             /* Store heap size value */

/* ============================================
 * Vector Table
 * All STM32G431 interrupt vectors
 * ============================================ */
  .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word  _estack               /* Initial stack pointer */
  .word  Reset_Handler         /* Reset handler */
  .word  NMI_Handler           /* NMI handler */
  .word  HardFault_Handler     /* Hard fault handler */
  .word  MemManage_Handler     /* Memory management fault */
  .word  BusFault_Handler      /* Bus fault */
  .word  UsageFault_Handler    /* Usage fault */
  .word  0                     /* Reserved */
  .word  0                     /* Reserved */
  .word  0                     /* Reserved */
  .word  0                     /* Reserved */
  .word  SVC_Handler           /* SVCall handler */
  .word  DebugMon_Handler      /* Debug monitor */
  .word  0                     /* Reserved */
  .word  PendSV_Handler        /* PendSV handler */
  .word  SysTick_Handler       /* SysTick handler */

  /* --- External interrupt vectors --- */

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
  .word  0                                 /* [16] Reserved */
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
  .word  0                                 /* [46] Reserved */
  .word  0                                 /* [47] Reserved */
  .word  0                                 /* [48] Reserved */
  .word  0                                 /* [49] Reserved */
  .word  SPI3_IRQHandler                   /* [50] SPI3 */
  .word  UART4_IRQHandler                  /* [51] UART4 */
  .word  0                                 /* [52] Reserved */
  .word  TIM6_DAC_IRQHandler               /* [53] TIM6 & DAC1 underrun */
  .word  TIM7_IRQHandler                   /* [54] TIM7 */
  .word  DMA2_Channel1_IRQHandler          /* [55] DMA2 Channel 1 */
  .word  DMA2_Channel2_IRQHandler          /* [56] DMA2 Channel 2 */
  .word  DMA2_Channel3_IRQHandler          /* [57] DMA2 Channel 3 */
  .word  DMA2_Channel4_IRQHandler          /* [58] DMA2 Channel 4 */
  .word  DMA2_Channel5_IRQHandler          /* [59] DMA2 Channel 5 */
  .word  0                                 /* [60] Reserved */
  .word  0                                 /* [61] Reserved */
  .word  UCPD1_IRQHandler                  /* [62] UCPD1 */
  .word  COMP1_2_3_IRQHandler              /* [63] COMP1, COMP2 & COMP3 */
  .word  COMP4_IRQHandler                  /* [64] COMP4 */
  .word  0                                 /* [65] Reserved */
  .word  0                                 /* [66] Reserved */
  .word  0                                 /* [67] Reserved */
  .word  0                                 /* [68] Reserved */
  .word  0                                 /* [69] Reserved */
  .word  0                                 /* [70] Reserved */
  .word  0                                 /* [71] Reserved */
  .word  0                                 /* [72] Reserved */
  .word  0                                 /* [73] Reserved */
  .word  0                                 /* [74] Reserved */
  .word  CRS_IRQHandler                    /* [75] CRS */
  .word  SAI1_IRQHandler                   /* [76] SAI1 */
  .word  0                                 /* [77] Reserved */
  .word  0                                 /* [78] Reserved */
  .word  0                                 /* [79] Reserved */
  .word  FPU_IRQHandler                    /* [80] FPU */
  .word  0                                 /* [81] Reserved */
  .word  0                                 /* [82] Reserved */
  .word  0                                 /* [83] Reserved */
  .word  0                                 /* [84] Reserved */
  .word  RNG_IRQHandler                    /* [85] RNG */
  .word  LPUART1_IRQHandler                /* [86] LPUART1 */
  .word  I2C3_EV_IRQHandler                /* [87] I2C3 Event */
  .word  I2C3_ER_IRQHandler                /* [88] I2C3 Error */
  .word  DMAMUX_OVR_IRQHandler             /* [89] DMAMUX Overrun */
  .word  0                                 /* [90] Reserved */
  .word  0                                 /* [91] Reserved */
  .word  0                                 /* [92] Reserved */
  .word  0                                 /* [93] Reserved */
  .word  0                                 /* [94] Reserved */
  .word  0                                 /* [95] Reserved */
  .word  0                                 /* [96] Reserved */
  .word  0                                 /* [97] Reserved */
  .word  0                                 /* [98] Reserved */
  .word  0                                 /* [99] Reserved */
  .word  0                                 /* [100] Reserved */
  .word  0                                 /* [101] Reserved */
  .word  DAC2_IRQHandler                   /* [102] DAC2 */
  .word  0                                 /* [103] Reserved */
  .word  0                                 /* [104] Reserved */
  .word  0                                 /* [105] Reserved */
  .word  LPTIM1_IRQHandler                 /* [106] LPTIM1 */
  .word  LPTIM2_IRQHandler                 /* [107] LPTIM2 */

/*******************************************************************************
*
* Reset_Handler: First code executed after processor reset
*   1. Copy .data section from Flash to RAM
*   2. Zero-initialize .bss section
*   3. Enable FPU (Cortex-M4F)
*   4. Call SystemInit() (clock configuration)
*   5. Call __libc_init_array() (C runtime initialization)
*   6. Call main()
*
*******************************************************************************/
  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0                   /* Set stack pointer */

  /* Copy .data section from Flash to RAM */
  ldr   r0, =_sdata              /* RAM destination start */
  ldr   r1, =_edata              /* RAM destination end */
  ldr   r2, =_sidata             /* Flash source start */
copy_data_init:
  cmp   r0, r1
  ittt  lt
  ldrlt r3, [r2], #4
  strlt r3, [r0], #4
  blt   copy_data_init

  /* Zero-initialize .bss section */
  ldr   r0, =_sbss               /* BSS start */
  ldr   r1, =_ebss               /* BSS end */
  mov   r2, #0
zero_bss_init:
  cmp   r0, r1
  itt   lt
  strlt r2, [r0], #4
  blt   zero_bss_init

  /* Enable FPU (floating-point unit) */
  ldr   r0, =0xE000ED88          /* CPACR register address */
  ldr   r1, [r0]
  orr   r1, r1, #(0xF << 20)     /* Set CP10, CP11 Full Access */
  str   r1, [r0]
  dsb
  isb

  /* Call SystemInit (clock system initialization) */
  bl    SystemInit

  /* C runtime initialization (constructor calls, etc.) */
  bl    __libc_init_array

  /* Call main function */
  bl    main

  /* Infinite loop if main returns */
halt_loop:
  b     halt_loop

.size  Reset_Handler, .-Reset_Handler

/*******************************************************************************
*
* Default handler (unimplemented interrupts)
* All undefined interrupts branch here and loop infinitely
*
*******************************************************************************/
  .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b     Infinite_Loop
.size  Default_Handler, .-Default_Handler

/*******************************************************************************
*
* Cortex-M4 core exception handlers (default: infinite loop)
* Users can override in stm32g4xx_it.c
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
* External interrupt handlers (default: infinite loop)
* Users can override in stm32g4xx_it.c
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
