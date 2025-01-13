#include "uart.h"
#include "bflb_uart.h"
struct bflb_device_s *uartx;

void uart_isr(int irq, void *arg)
{
    uint32_t intstatus = bflb_uart_get_intstatus(uartx);

    if (intstatus & UART_INTSTS_RX_FIFO) {
        while (bflb_uart_rxavailable(uartx)) {
            // uart_rxbuf[uart_rx_count++] = bflb_uart_getchar(uartx);
        }
    }
    if (intstatus & UART_INTSTS_RTO) {
        bflb_uart_int_clear(uartx, UART_INTCLR_RTO);
        while (bflb_uart_rxavailable(uartx)) {
            // uart_rxbuf[uart_rx_count++] = bflb_uart_getchar(uartx);
        }
    }
    if (intstatus & UART_INTSTS_TX_FIFO) {
        for (uint8_t i = 0; i < 27; i++) {
            // bflb_uart_putchar(uartx, uart_txbuf[i]);
        }
        bflb_uart_txint_mask(uartx, true);
    }
}
void uart_init(struct bflb_device_s * gpio)
{
    bflb_gpio_uart_init(gpio, GPIO_PIN_0, GPIO_UART_FUNC_UART1_TX);
    bflb_gpio_uart_init(gpio, GPIO_PIN_1, GPIO_UART_FUNC_UART1_RX);
    uartx = bflb_device_get_by_name("uart1");
    struct bflb_uart_config_s cfg;
    cfg.baudrate = 115200;
    cfg.data_bits = UART_DATA_BITS_8;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.parity = UART_PARITY_NONE;
    cfg.flow_ctrl = 0;
    cfg.tx_fifo_threshold = 7;
    cfg.rx_fifo_threshold = 7;
    cfg.bit_order = UART_LSB_FIRST;
    bflb_uart_init(uartx, &cfg);

    bflb_irq_attach(uartx->irq_num, uart_isr, NULL);
    bflb_irq_enable(uartx->irq_num);
}
void uart_write(uint8_t* msg, int len)
{
    for(int i=0;i<len;i++)
    {
        bflb_uart_putchar(uartx, msg[i]);
    }
}