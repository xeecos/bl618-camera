#pragma once
#include "bflb_gpio.h"
#include "bflb_i2c.h"
struct bflb_device_s *i2c0;
void i2c_init(struct bflb_device_s * gpio, int scl, int sda)
{
    bflb_gpio_init(gpio, scl, (GPIO_FUNC_I2C0 | (1 << 8) | (1 << (9)) | (1 << (11)) | (1 << (12))));
    bflb_gpio_init(gpio, sda, (GPIO_FUNC_I2C0 | (1 << 8) | (1 << (9)) | (1 << (11)) | (1 << (12))));

    i2c0 = bflb_device_get_by_name("i2c0");
    bflb_i2c_init(i2c0, 100000);
}
void i2c_transfer(struct bflb_i2c_msg_s* msg, int len)
{
    bflb_i2c_transfer(i2c0, msg, len);
}