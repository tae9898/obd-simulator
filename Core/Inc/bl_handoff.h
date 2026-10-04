/**
 * @file    bl_handoff.h
 * @brief   Bootloader <-> App shared constants (flash map, noinit flag, meta)
 * @note    Included by BOTH bl_* (bootloader) and uds_service.c (app).
 *          Single source of truth for the FBL memory map -- keep in sync with
 *          STM32G431RBTX_BL.ld / STM32G431RBTX_FLASH.ld.
 *
 * Flash map (128KB single bank, 2KB pages):
 *   0x08000000  BL       28KB
 *   0x08007000  App slot 92KB
 *   0x0801E000  Meta      8KB (first 2KB page used; old OTA region)
 */

#ifndef __BL_HANDOFF_H
#define __BL_HANDOFF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* === Flash map === */
#define BL_BASE          0x08000000U
#define BL_SIZE          0x7000U            /* 28KB */
#define APP_BASE         0x08007000U
#define APP_SLOT_SIZE    0x17000U           /* 92KB (APP_BASE .. META_BASE) */
#define META_BASE        0x0801E000U        /* first page of old OTA region */
#define FLASH_END        0x08020000U

/* === Stay-in-BL handoff flag (no-init RAM, survives reset not power loss) ===
 * Address 0x20007F00 is carved out of BOTH linker scripts (RAM LENGTH 0x7F00,
 * _estack = 0x20007F00) so nothing allocates over it.
 * App writes {magic, reason=UPDATE} right before requesting reset;
 * BL consumes (clears) it on boot.
 * Power loss between flag-write and reset is harmless: flash is untouched,
 * next boot re-evaluates the (still valid) app normally. */
#define BL_HANDOFF_ADDR  0x20007F00U
#define BL_HANDOFF_MAGIC 0xB007B007U        /* "BOOTBOOT" */
#define BL_REASON_UPDATE 1U

typedef struct {
    uint32_t magic;
    uint32_t reason;
} bl_handoff_t;

/* === Image meta page (META_BASE, erase-rewritten once per update) === */
#define BL_META_MAGIC    0x4D424C4DU        /* "MBLM" (little-endian) */
#define BL_META_STATE_UPDATING 0x55555555U  /* written at 0x34, before app erase */
#define BL_META_STATE_VALID    0xAAAAAAAAU  /* written at 0x31 CheckMemory pass */

typedef struct {
    uint32_t magic;    /* BL_META_MAGIC */
    uint32_t state;    /* BL_META_STATE_* */
    uint32_t version;  /* firmware version == anti-rollback counter (monotonic) */
    uint32_t size;     /* app image size in bytes */
    uint32_t crc32;    /* reflected CRC-32 (zlib/binascii) over app[0..size) */
} bl_meta_t;           /* 20B, padded to 24B when doubleword-programmed */

#ifdef __cplusplus
}
#endif

#endif /* __BL_HANDOFF_H */
