/**
 * bsp_uart.h - non-blocking receive on the Jetson link (USART1, circular DMA + idle-line event).
 *
 * The DMA writes incoming bytes into a ring buffer continuously, with no CPU involvement.
 * When the line goes quiet after a burst (= a frame has just ended), the UART raises one interrupt
 * and the registered callback runs, so the receiving task can wake immediately instead of polling.
 * bsp_uart_read() then collects whatever has arrived. Nothing here blocks.
 * If the HAL stops reception after a line error (overrun etc.), the next bsp_uart_read() restarts it.
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include "main.h"

typedef void (*bsp_uart_rx_event_cb_t)(void);   /* runs in interrupt context: keep it tiny */

/** Enable the RX pull-up, register the "data arrived" callback (may be NULL), start reception. */
HAL_StatusTypeDef bsp_uart_rx_start(bsp_uart_rx_event_cb_t on_rx_event);

/** Copy up to `max` newly received bytes into dst. Returns the number copied (0 if none). */
uint32_t bsp_uart_read(uint8_t *dst, uint32_t max);

/* diagnostics */
uint32_t bsp_uart_error_count(void);     /* line errors reported by the HAL (noise, framing, overrun) */
uint32_t bsp_uart_restart_count(void);   /* times reception had to be restarted */

#endif /* BSP_UART_H */
