#pragma once
#include "bflb_gpio.h"

void uart_init(struct bflb_device_s * gpio);
void uart_write(uint8_t* msg, int len);