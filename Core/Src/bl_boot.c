/**
 * @file    bl_boot.c
 * @brief   Bootloader boot gate: meta validation, CRC32, app vector sanity, jump
 * @note    Bare-metal, no RTOS. Compiled only into the BL target.
 */

#include "main.h"
#include "bl_handoff.h"

/* ponytail: CRC32-only integrity; add signature verify (ECDSA/HSM, AIS-189)
 * before real-vehicle use. */

/* === Reflected CRC-32 (poly 0xEDB88320, init/xorout 0xFFFFFFFF) ===
 * Same as zlib/binascii.crc32 so the tester side is literally binascii.crc32(). */
uint32_t bl_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* === Meta page access === */

const bl_meta_t *bl_meta(void)
{
    return (const bl_meta_t *)META_BASE;
}

/** meta page fully erased (virgin flash / after st-flash erase)? */
int bl_meta_absent(void)
{
    const uint32_t *p = (const uint32_t *)META_BASE;
    for (uint32_t i = 0; i < sizeof(bl_meta_t) / 4U; i++) {
        if (p[i] != 0xFFFFFFFFU) {
            return 0;
        }
    }
    return 1;
}

/** VALID meta whose stored CRC matches the flashed app image? */
int bl_app_crc_ok(void)
{
    const bl_meta_t *m = bl_meta();
    if (m->magic != BL_META_MAGIC || m->state != BL_META_STATE_VALID) {
        return 0;
    }
    if (m->size == 0U || m->size > APP_SLOT_SIZE) {
        return 0;
    }
    return bl_crc32((const uint8_t *)APP_BASE, m->size) == m->crc32;
}

/** Old anti-rollback counter to enforce across the meta erase at 0x34
 * (returns 0 when meta is absent/garbage). */
uint32_t bl_meta_rollback_counter(void)
{
    const bl_meta_t *m = bl_meta();
    if (m->magic == BL_META_MAGIC &&
        (m->state == BL_META_STATE_VALID || m->state == BL_META_STATE_UPDATING)) {
        return m->version;
    }
    return 0U;
}

/* === App vector sanity + jump === */

/** Plausible app at APP_BASE? (initial SP inside RAM, reset vector inside slot)
 *  Used only when meta is ABSENT (dev/SWD flash path) -- a corrupted app under
 *  an interrupted update is caught by the UPDATING marker instead. */
int bl_app_vectors_ok(void)
{
    uint32_t sp   = *(volatile uint32_t *)(APP_BASE);
    uint32_t rst  = *(volatile uint32_t *)(APP_BASE + 4U);
    return (sp >= 0x20000000U && sp <= 0x20008000U) &&
           (rst >= APP_BASE && rst < (APP_BASE + APP_SLOT_SIZE)) &&
           ((rst & 1U) != 0U);   /* thumb bit */
}

typedef void (*pFunc)(void);

/** Jump to the app at APP_BASE. Never returns.
 *  Order: stop systick -> deinit FDCAN (if started) -> global IRQ disable ->
 *  clear every NVIC enable/pending bit -> VTOR/MSP -> reset handler. */
void bl_jump_to_app(void)
{
    uint32_t sp   = *(volatile uint32_t *)APP_BASE;
    pFunc  reset = (pFunc)(*(volatile uint32_t *)(APP_BASE + 4U));

    __disable_irq();
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;

    /* Deinit peripherals the BL touched. MspDeInit also drops NVIC + pins. */
    if (hfdcan1.Instance != NULL) {
        HAL_FDCAN_DeInit(&hfdcan1);
    }

    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;   /* disable all IRQ lines */
        NVIC->ICPR[i] = 0xFFFFFFFFU;   /* clear all pending */
    }

    SCB->VTOR = APP_BASE;
    __DSB();
    __ISB();
    __set_MSP(sp);
    __set_CONTROL(0U);   /* privileged, MSP */
    reset();             /* no return */
}
