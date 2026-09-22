/**
 * @file    task.h
 * @brief   Host-side FreeRTOS stub: critical sections are no-ops (single thread)
 */
#ifndef __TASK_H
#define __TASK_H

#define taskENTER_CRITICAL() do {} while (0)
#define taskEXIT_CRITICAL()  do {} while (0)

#endif /* __TASK_H */
