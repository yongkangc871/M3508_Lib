/**
 * outputTask - sends the motor commands.
 *
 * Sleeps until processTask says a new set of currents is ready, then packs all four into ONE CAN
 * frame (ID 0x200) and queues it. One frame for four motors means the C620s receive their new
 * currents at the same instant, which is what keeps the wheels synchronised on the wire.
 *
 * The send never blocks: if the CAN TX FIFO is full (nobody on the bus is acknowledging), the frame
 * is dropped and counted in bsp_can_tx_fail_count().
 */
#include "app_tasks.h"
#include "m3508.h"
#include "FreeRTOS.h"
#include "task.h"

static TaskHandle_t      s_task;
static volatile uint32_t s_frames;

void outputTask_trigger(void)
{
    if (s_task) xTaskNotifyGive(s_task);
}

uint32_t outputTask_frame_count(void)
{
    return s_frames;
}

void outputTask(void *arg)
{
    (void)arg;
    s_task = xTaskGetCurrentTaskHandle();

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);     /* wait for processTask */

        if (m3508_send_group(0) == HAL_OK) s_frames++;     /* motors 1..4 in one frame */
#if M3508_NUM_MOTORS > 4
        (void)m3508_send_group(1);                         /* motors 5..8 */
#endif
    }
}
