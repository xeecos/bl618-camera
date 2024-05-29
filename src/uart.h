#pragma once
#include "bflb_gpio.h"
#include "bflb_uart.h"
struct bflb_device_s *uartx;
void uart_init(struct bflb_device_s * gpio, int tx, int rx)
{
    bflb_gpio_uart_init(gpio, tx, GPIO_UART_FUNC_UART0_TX);
    bflb_gpio_uart_init(gpio, rx, GPIO_UART_FUNC_UART0_RX);
    uartx = bflb_device_get_by_name("uart0");
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
}
void uart_write(char* msg)
{

}