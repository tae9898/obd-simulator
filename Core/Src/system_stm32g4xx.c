/**
 * @file    system_stm32g4xx.c
 * @brief   STM32G431 system clock configuration
 * @note    HSI 16MHz -> PLL -> SYSCLK 170MHz
 *         This file is executed first after the Reset handler
 */

#include "stm32g4xx.h"

/* === System clock constants === */
/* HSI_VALUE is defined in stm32g4xx_hal_conf.h */
#define SYSCLK_FREQ  170000000U /* SYSCLK target = 170MHz */

/** @brief System clock frequency global variable (referenced by HAL) */
uint32_t SystemCoreClock = SYSCLK_FREQ;

/** @brief AHB prescaler values (used by CMSIS SystemCoreClockUpdate) */
const uint8_t AHBPrescTable[16] = {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U
};

/** @brief APB prescaler values */
const uint8_t APBPrescTable[8] = {
    0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U
};

/**
 * @brief  System initialization function
 * @retval None
 *
 * @note   Called from startup_stm32g431rb.s after Reset, before C runtime init.
 *         Only FPU is enabled here; actual clock configuration is done in main.c
 *
 *         STM32G431 FPU enable sequence:
 *         1. Allow CPACR register access
 *         2. Allow CP10, CP11 (single-precision FPU) access
 */
void SystemInit(void)
{
    /* --- FPU (Floating Point Unit) enable --- */
    /* STM32G431 is Cortex-M4F, has hardware FPU */
    SCB->CPACR |= ((3U << 10U * 2U) |   /* CP10 = single-precision FPU access allowed */
                   (3U << 11U * 2U));   /* CP11 = single-precision FPU access allowed */

    /*
     * Note: Actual clock configuration (PLL etc.) is done in main() via SystemClock_Config()
     * Immediately after reset, runs on HSI 16MHz; only FPU is enabled before PLL setup
     *
     * This pattern matches CubeMX generated code:
     * SystemInit() -> minimal init (FPU, VTOR etc.)
     * main() -> SystemClock_Config() -> detailed clock configuration
     */

    /* Vector table offset setting (Flash start address, no ITM/ETM) */
    SCB->VTOR = FLASH_BASE;
}

/**
 * @brief  SystemCoreClock variable update
 * @retval None
 *
 * @note   After clock configuration changes, computes the current system clock frequency
 *         and stores it in the SystemCoreClock global variable.
 *         Required to maintain HAL_GetTick() accuracy.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp;
    uint32_t pllvco;
    uint32_t pllr;
    uint32_t pllsource;
    uint32_t pllm;

    /* --- Read SWS (System Clock Switch Status) from CFGR register --- */
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
        case 0x00U:
            /* HSI in use */
            SystemCoreClock = HSI_VALUE;
            break;

        case 0x04U:
            /* HSE in use */
            SystemCoreClock = HSE_VALUE;
            break;

        case 0x08U:
            /* PLL in use - PLLR output */
            pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;

            if (pllsource == 0x00U) {
                /* PLL source = HSI */
                pllvco = (HSI_VALUE / pllm);
            } else {
                /* PLL source = HSE */
                pllvco = (HSE_VALUE / pllm);
            }

            pllvco *= ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
            pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U);
            SystemCoreClock = pllvco / pllr;
            break;

        case 0x0CU:
            /* HSI48 in use (G431 supported) */
            SystemCoreClock = 48000000U;
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Apply AHB prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
    SystemCoreClock >>= tmp;
}
