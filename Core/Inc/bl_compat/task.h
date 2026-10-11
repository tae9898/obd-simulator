/**
 * @file    bl_compat/task.h
 * @brief   FreeRTOS task.h stub for the bare-metal bootloader build
 * @note    Only on the BL include path (-ICore/Inc/bl_compat before the
 *          FreeRTOS include dirs). iso_tp.c pulls task.h for critical
 *          sections; in the single-context BL, IRQ masking is equivalent.
 */
#ifndef __BL_COMPAT_TASK_H
#define __BL_COMPAT_TASK_H

#define taskENTER_CRITICAL()  __disable_irq()
#define taskEXIT_CRITICAL()   __enable_irq()

#endif /* __BL_COMPAT_TASK_H */
