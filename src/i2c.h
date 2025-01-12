#pragma once
#include "bflb_gpio.h"
#include "bflb_i2c.h"

void i2c_init(struct bflb_device_s * gpio, int scl, int sda);
void i2c_transfer(struct bflb_i2c_msg_s* msg, int len);