/**
 * util.h - small helpers shared by the application modules.
 */
#ifndef UTIL_H
#define UTIL_H

#include "FreeRTOS.h"
#include "task.h"

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/** Clamp to [-lim, +lim]. */
static inline float clamp_absf(float v, float lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* Critical section for API functions that may also be called BEFORE the scheduler starts
   (module init, early configuration). A FreeRTOS critical section taken that early leaves
   interrupts masked until the scheduler starts, so it is skipped then. Code that only ever runs
   inside a task can use taskENTER_CRITICAL() / taskEXIT_CRITICAL() directly. */
static inline void crit_enter(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) taskENTER_CRITICAL();
}

static inline void crit_exit(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) taskEXIT_CRITICAL();
}

#endif /* UTIL_H */
