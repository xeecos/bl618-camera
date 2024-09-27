#include "led.h"
#include "config.h"
#include "bflb_gpio.h"
volatile int led = 0;
void led_init()
{
    struct bflb_device_s *gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, PIN_LED, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
}
void light_toggle()
{
    struct bflb_device_s *gpio = bflb_device_get_by_name("gpio");
    if(led)
    {
        bflb_gpio_set(gpio, PIN_LED);
    }
    else
    {
        bflb_gpio_reset(gpio, PIN_LED);
    }
    led = 1 - led;
}