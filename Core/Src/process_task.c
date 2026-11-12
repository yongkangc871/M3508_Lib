/**
 * processTask - the 1 kHz control loop.
 *
 * Every millisecond, in this order:
 *   1. take the latest velocity command (with its failsafe)
 *   2. chassis: ramp the robot velocity, convert it to four wheel speed setpoints
 *   3. motors: decode CAN feedback, run the four speed PIDs -> four currents
 *   4. trigger outputTask, which sends the currents in one CAN frame
 *
 * The period is fixed (vTaskDelayUntil), so dt is a constant and the PIDs behave the same every
 * cycle. Nothing in here waits for anything.
 */
#include "app_tasks.h"
#include "bsp_can.h"
#include "chassis.h"
#include "cmd_link.h"
#include "m3508.h"
#include "robot_config.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile process_stats_t s_proc_stats;

void processTask(void *arg)
{
    (void)arg;

    /* Cycle counter for execution-time measurement (costs nothing at run time). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    /* Bring up CAN: accept motor feedback IDs 0x201..0x208. */
    s_proc_stats.can_ok = (bsp_can_init(&ROBOT_CAN_HANDLE, M3508_CAN_ID_RX_BASE + 1U,
                                        M3508_CAN_ID_RX_BASE + 8U) == HAL_OK);

    const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS);
    const float      dt     = (float)CTRL_PERIOD_MS * 0.001f;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);   /* drift-free 1 kHz */
        const uint32_t t0 = DWT->CYCCNT;

        cmd_vel_t cmd;
        cmd_link_get(&cmd);                    /* 1. command (zero if the link is down)        */
        chassis_update(&cmd, dt);              /* 2. robot velocity -> wheel speed setpoints   */
        m3508_update_all(dt);                  /* 3. feedback + speed PIDs -> currents         */
        outputTask_trigger();                  /* 4. send (outputTask runs right here: it has
                                                     higher priority, so this returns after
                                                     the frame has been queued)                */

        const uint32_t us = (DWT->CYCCNT - t0) / cycles_per_us;
        s_proc_stats.exec_us_last = us;
        if (us > s_proc_stats.exec_us_max) s_proc_stats.exec_us_max = us;
        if ((xTaskGetTickCount() - last_wake) >= period) s_proc_stats.overruns++;
        s_proc_stats.loops++;
    }
}

void processTask_get_stats(process_stats_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    *out = s_proc_stats;
    taskEXIT_CRITICAL();
}
