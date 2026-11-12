/**
 * bsp_can.h - thin FDCAN wrapper (classic CAN 2.0A frames, 1 Mbit/s).
 * Owns: filter config, start, RX interrupt draining. Does not know about motors:
 * it forwards every received frame to a registered callback (m3508 registers itself).
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "main.h"

typedef void (*bsp_can_rx_cb_t)(uint32_t std_id, const uint8_t data[8], uint8_t len);

/** Configure filter (std IDs lo..hi -> RX FIFO0), start peripheral, enable RX-FIFO0 interrupt. */
HAL_StatusTypeDef bsp_can_init(FDCAN_HandleTypeDef *h, uint32_t std_id_lo, uint32_t std_id_hi);

/** Register a function to receive frames from the ISR context. Must be ISR-safe and fast. */
void bsp_can_register_rx(bsp_can_rx_cb_t cb);

/** Queue one 8-byte classic CAN data frame. Non-blocking; returns HAL_ERROR if TX FIFO full. */
HAL_StatusTypeDef bsp_can_send_std(FDCAN_HandleTypeDef *h, uint32_t std_id, const uint8_t data[8]);

/* diagnostics */
uint32_t bsp_can_rx_count(void);
uint32_t bsp_can_tx_fail_count(void);
uint32_t bsp_can_bus_off_count(void);
uint32_t bsp_can_rx_lost_count(void);    /* frames dropped because the 3-deep RX FIFO was full */

#endif /* BSP_CAN_H */
