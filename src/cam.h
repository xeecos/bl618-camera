#pragma once
#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_pwm_v2.h"
#include "bflb_clock.h"

#define PIN_HREF    GPIO_PIN_1
#define PIN_VSYNC   GPIO_PIN_0
#define PIN_PCLK    GPIO_PIN_13
#define PIN_XCLK    GPIO_PIN_20
#define PIN_D0      GPIO_PIN_3
#define PIN_D1      GPIO_PIN_10
#define PIN_D2      GPIO_PIN_11
#define PIN_D3      GPIO_PIN_12
#define PIN_D4      GPIO_PIN_14
#define PIN_D5      GPIO_PIN_15
#define PIN_D6      GPIO_PIN_16
#define PIN_D7      GPIO_PIN_17

#define BF3003_ADDR                0x6E
#define BF3003_PID_ADDR            0xFC

struct bflb_device_s *gpio;
struct bflb_device_s *pwm;
static struct bflb_device_s *i2c0;
void cam_isr(int irq, void *arg)
{
    if (bflb_gpio_get_intstatus(gpio, PIN_HREF)) 
    {
        bflb_gpio_int_clear(gpio, PIN_HREF);
    }
    if (bflb_gpio_get_intstatus(gpio, PIN_VSYNC)) 
    {
        bflb_gpio_int_clear(gpio, PIN_VSYNC);
    }
    if (bflb_gpio_get_intstatus(gpio, PIN_PCLK)) 
    {
        bflb_gpio_int_clear(gpio, PIN_PCLK);
    }
}
uint8_t cam_sensor_read(uint8_t address)
{
    struct bflb_i2c_msg_s msgs[2];
    uint8_t buffer[2];
    uint8_t readOut[1];
    
    msgs[0].addr = BF3003_ADDR;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = NULL;
    msgs[0].length = 1;

    msgs[1].addr = BF3003_ADDR;
    msgs[1].flags = I2C_M_READ;
    msgs[1].buffer = NULL;
    msgs[1].length = 1;
    
    buffer[0] = address & 0xff;
    msgs[0].buffer = buffer;
    msgs[1].buffer = readOut;
    bflb_i2c_transfer(i2c0, msgs, 2);
    return readOut[0];
}
void cam_probe()
{
    while(1)
    {
        if(cam_sensor_read(BF3003_PID_ADDR) == 0x30)
        {
            break;
        }
        bflb_mtimer_delay_ms(1000);
    }
}
void cam_init()
{
    pwm = bflb_device_get_by_name("pwm_v2_0");
    gpio = bflb_device_get_by_name("gpio");
    i2c0 = bflb_device_get_by_name("i2c0");

    bflb_i2c_init(i2c0, 100000);

    cam_probe();
    
    bflb_gpio_int_init(gpio, PIN_VSYNC, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_int_mask(gpio, PIN_VSYNC, false);

    bflb_gpio_int_init(gpio, PIN_HREF, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_int_mask(gpio, PIN_HREF, false);

    bflb_gpio_int_init(gpio, PIN_PCLK, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_int_mask(gpio, PIN_PCLK, false);

    bflb_irq_attach(gpio->irq_num, cam_isr, gpio);
    bflb_irq_enable(gpio->irq_num);

    /* period = .XCLK / .clk_div / .period = 40MHz / 4 / 10 = 1000KHz */
    struct bflb_pwm_v2_config_s cfg = {
        .clk_source = BFLB_SYSTEM_XCLK,
        .clk_div = 4,
        .period = 10,
    };

    bflb_pwm_v2_init(pwm, &cfg);
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 1, 6); 
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
    bflb_pwm_v2_start(pwm);
}