#include "bsp_can.h"
#include <string.h>

static bsp_can_rx_cb_t   s_rx_cb    = 0;
static volatile uint32_t s_rx_count = 0;
static volatile uint32_t s_tx_fail  = 0;
static volatile uint32_t s_bus_off  = 0;
static volatile uint32_t s_rx_lost  = 0;

HAL_StatusTypeDef bsp_can_init(FDCAN_HandleTypeDef *h, uint32_t std_id_lo, uint32_t std_id_hi)
{
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;          /* FilterID1 <= id <= FilterID2 */
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = std_id_lo;
    f.FilterID2    = std_id_hi;
    if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK) return HAL_ERROR;

    /* Everything that does not match a filter is dropped (no remote frames either). */
    if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return HAL_ERROR;

    if (HAL_FDCAN_Start(h) != HAL_OK) return HAL_ERROR;

    if (HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST
                                          | FDCAN_IT_BUS_OFF, 0) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

void bsp_can_register_rx(bsp_can_rx_cb_t cb)
{
    s_rx_cb = cb;
}

HAL_StatusTypeDef bsp_can_send_std(FDCAN_HandleTypeDef *h, uint32_t std_id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = std_id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(h) == 0U) {
        s_tx_fail++;
        return HAL_ERROR;
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, (uint8_t *)data) != HAL_OK) {
        s_tx_fail++;
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint32_t bsp_can_rx_count(void)      { return s_rx_count; }
uint32_t bsp_can_tx_fail_count(void) { return s_tx_fail; }
uint32_t bsp_can_bus_off_count(void) { return s_bus_off; }
uint32_t bsp_can_rx_lost_count(void) { return s_rx_lost; }

/* ---------------- HAL callbacks (ISR context) ---------------- */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) s_rx_lost++;   /* FIFO was full: a frame was dropped */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    /* G4 RX FIFO0 is only 3 deep: drain everything that is pending in one go. */
    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) break;
        s_rx_count++;
        if (s_rx_cb && rx.IdType == FDCAN_STANDARD_ID) {
            /* Older HAL encodes DLC as (n << 16), newer HAL as plain n. Handle both. */
            uint8_t len = (uint8_t)((rx.DataLength > 8U) ? (rx.DataLength >> 16) : rx.DataLength);
            s_rx_cb(rx.Identifier, data, len);
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h, uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        s_bus_off++;
        /* Bosch M_CAN sets INIT after bus-off; clearing it restarts the controller. */
        CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}
