#include "bsp_uart.h"
#include "robot_config.h"

/* Must hold everything that can arrive between two bsp_uart_read() calls.
   921600 baud = ~92 bytes/ms, so 1024 bytes covers an 11 ms gap at full line rate. */
#define RX_BUF_SIZE   1024U

static uint8_t                s_rx_buf[RX_BUF_SIZE];
static uint32_t               s_rd;              /* next index to read */
static bsp_uart_rx_event_cb_t s_on_rx_event;
static volatile uint32_t      s_errors;
static uint32_t               s_restarts;

static HAL_StatusTypeDef rx_arm(void)
{
    s_rd = 0;
    /* DMA is circular (CubeMX), so this keeps running and reports every idle-line event. */
    return HAL_UARTEx_ReceiveToIdle_DMA(&ROBOT_CMD_UART, s_rx_buf, RX_BUF_SIZE);
}

HAL_StatusTypeDef bsp_uart_rx_start(bsp_uart_rx_event_cb_t on_rx_event)
{
    s_on_rx_event = on_rx_event;

    /* CubeMX leaves PA10 (RX) floating. Pull it up so an unplugged or booting Jetson reads as
       an idle line instead of noise. Done here so regenerating the project does not undo it. */
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD10, GPIO_PUPDR_PUPD10_0);
    return rx_arm();
}

uint32_t bsp_uart_read(uint8_t *dst, uint32_t max)
{
    UART_HandleTypeDef *h = &ROBOT_CMD_UART;

    /* The HAL aborts DMA reception on an overrun. Bring it back up. */
    if (h->RxState != HAL_UART_STATE_BUSY_RX) {
        if (rx_arm() == HAL_OK) s_restarts++;
        return 0;
    }

    /* The DMA counter counts down from RX_BUF_SIZE: this is the index it will write next. */
    const uint32_t wr = (RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(h->hdmarx)) % RX_BUF_SIZE;

    uint32_t n = 0;
    while (s_rd != wr && n < max) {
        dst[n++] = s_rx_buf[s_rd];
        s_rd = (s_rd + 1U) % RX_BUF_SIZE;
    }
    return n;
}

uint32_t bsp_uart_error_count(void)   { return s_errors; }
uint32_t bsp_uart_restart_count(void) { return s_restarts; }

/* ---------------- HAL callbacks (interrupt context) ---------------- */

/* Fires on idle line (end of a burst) and when the DMA passes the middle / end of the buffer. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size)
{
    (void)size;                       /* the reader works from the DMA counter instead */
    if (h == &ROBOT_CMD_UART && s_on_rx_event) s_on_rx_event();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h == &ROBOT_CMD_UART) {
        s_errors++;
        if (s_on_rx_event) s_on_rx_event();   /* wake the reader so it can restart reception */
    }
}
