/**
 * inputTask - receives velocity commands from the Jetson.
 *
 * Sleeps until the UART reports that a burst of bytes has ended (idle-line interrupt), then drains
 * the DMA ring buffer and feeds the parser. A command is therefore available to processTask well
 * under a millisecond after its last byte arrives, and no CPU time is spent while the line is quiet.
 */
#include "app_tasks.h"
#include "bsp_uart.h"
#include "cmd_link.h"
#include "robot_config.h"
#include "FreeRTOS.h"
#include "task.h"

static TaskHandle_t s_task;

/* UART interrupt -> wake inputTask */
static void on_uart_rx_event(void)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    portYIELD_FROM_ISR(woken);
}

void inputTask(void *arg)
{
    (void)arg;
    static uint8_t chunk[128];

    s_task = xTaskGetCurrentTaskHandle();
    (void)bsp_uart_rx_start(on_uart_rx_event);

    for (;;) {
        /* Wait for data. The timeout is only a safety net: it lets bsp_uart_read() restart
           reception if the UART stopped after a line error and no interrupt arrives. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(INPUT_FALLBACK_POLL_MS));

        uint32_t n;
        while ((n = bsp_uart_read(chunk, sizeof(chunk))) > 0U) {
            cmd_link_feed(chunk, n);
        }
    }
}
