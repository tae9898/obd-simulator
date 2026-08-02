/**
 * @file    system_stm32g4xx.c
 * @brief   STM32G431 시스템 클럭 설정
 * @note    HSI 16MHz -> PLL -> SYSCLK 170MHz
 *         이 파일은 Reset 핸들러 이후 가장 먼저 실행됨
 */

#include "stm32g4xx.h"

/* === 시스템 클럭 상수 === */
/* HSI_VALUE는 stm32g4xx_hal_conf.h에서 정의됨 */
#define SYSCLK_FREQ  170000000U /* SYSCLK 목표 = 170MHz */

/** @brief 시스템 클럭 주파수 전역 변수 (HAL에서 참조) */
uint32_t SystemCoreClock = SYSCLK_FREQ;

/** @brief AHB 프리스케일러 값 (CMSIS SystemCoreClockUpdate에서 사용) */
const uint8_t AHBPrescTable[16] = {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U
};

/** @brief APB 프리스케일러 값 */
const uint8_t APBPrescTable[8] = {
    0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U
};

/**
 * @brief  시스템 초기화 함수
 * @retval None
 *
 * @note   Reset 후 C 런타임 초기화 전에 startup_stm32g431rb.s에서 호출됨
 *         여기서는 FPU만 활성화하고, 실제 클럭 설정은 main.c에서 수행
 *
 *         STM32G431 FPU 활성화 순서:
 *         1. CPACR 레지스터 접근 허용
 *         2. CP10, CP11 (단정밀도 FPU) 접근 허용
 */
void SystemInit(void)
{
    /* --- FPU (Floating Point Unit) 활성화 --- */
    /* STM32G431은 Cortex-M4F이므로 하드웨어 FPU 보유 */
    SCB->CPACR |= ((3U << 10U * 2U) |   /* CP10 = 단정밀도 FPU 접근 허용 */
                   (3U << 11U * 2U));   /* CP11 = 단정밀도 FPU 접근 허용 */

    /*
     * 주의: 실제 클럭 설정 (PLL 등)은 main()에서 SystemClock_Config()로 수행
     * Reset 직후에는 HSI 16MHz로 동작하며, PLL 설정 전까지 FPU만 활성화
     *
     * 이 방식은 CubeMX 생성 코드와 동일한 패턴:
     * SystemInit() -> 최소 초기화 (FPU, VTOR 등)
     * main() -> SystemClock_Config() -> 상세 클럭 설정
     */

    /* 벡터 테이블 오프셋 설정 (Flash 시작 주소, ITM/ETM 없음) */
    SCB->VTOR = FLASH_BASE;
}

/**
 * @brief  SystemCoreClock 변수 업데이트
 * @retval None
 *
 * @note   클럭 설정 변경 후 현재 시스템 클럭 주파수를 계산하여
 *         SystemCoreClock 전역 변수에 저장
 *         HAL_GetTick() 정확도 유지에 필요
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp;
    uint32_t pllvco;
    uint32_t pllr;
    uint32_t pllsource;
    uint32_t pllm;

    /* --- CFGR 레지스터에서 SWS (System Clock Switch Status) 읽기 --- */
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
        case 0x00U:
            /* HSI 사용 중 */
            SystemCoreClock = HSI_VALUE;
            break;

        case 0x04U:
            /* HSE 사용 중 */
            SystemCoreClock = HSE_VALUE;
            break;

        case 0x08U:
            /* PLL 사용 중 - PLLR 출력 */
            pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;

            if (pllsource == 0x00U) {
                /* PLL 소스 = HSI */
                pllvco = (HSI_VALUE / pllm);
            } else {
                /* PLL 소스 = HSE */
                pllvco = (HSE_VALUE / pllm);
            }

            pllvco *= ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
            pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U);
            SystemCoreClock = pllvco / pllr;
            break;

        case 0x0CU:
            /* HSI48 사용 중 (G431 지원) */
            SystemCoreClock = 48000000U;
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* AHB 프리스케일러 적용 */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
    SystemCoreClock >>= tmp;
}
