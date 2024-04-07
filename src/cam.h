#pragma once
#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_uart.h"
#include "bflb_pwm_v2.h"
#include "bflb_clock.h"
#include "bf3003.h"

struct bflb_device_s *gpio;
struct bflb_device_s *pwm;
struct bflb_device_s *i2c0;
struct bflb_device_s *uartx;

uint8_t frame[2][640];
int lineCount = 0;
int led = 0;
int pixelCount = 0;
void cam_isr(int irq, void *arg)
{
    if (bflb_gpio_get_intstatus(gpio, PIN_PCLK)) 
    {
        bflb_gpio_int_clear(gpio, PIN_PCLK);
        pixelCount++;
    }
    if (bflb_gpio_get_intstatus(gpio, PIN_HREF)) 
    {
        bflb_gpio_int_clear(gpio, PIN_HREF);
		lineCount++;
    }
    if (bflb_gpio_get_intstatus(gpio, PIN_VSYNC)) 
    {
        bflb_gpio_int_clear(gpio, PIN_VSYNC);
        if(led)bflb_gpio_set(gpio, GPIO_PIN_29);
		else bflb_gpio_reset(gpio, GPIO_PIN_29);
		led = 1-led;
		printf("frame:%d\n",pixelCount);
		lineCount = 0;
        pixelCount = 0;
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

void cam_sensor_write(uint8_t address, uint8_t paramete)
{
    struct bflb_i2c_msg_s msgs[2];
    uint8_t buffer[2];
    
    msgs[0].addr = BF3003_ADDR;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = NULL;
    msgs[0].length = 1;

    msgs[1].addr = BF3003_ADDR;
    msgs[1].flags = 0;
    msgs[1].buffer = NULL;
    msgs[1].length = 1;
    
    uint8_t writeIn[1] = { paramete };
    buffer[0] = address & 0xff;
    msgs[0].buffer = buffer;
    msgs[1].buffer = writeIn;

    bflb_i2c_transfer(i2c0, msgs, 2);
}
void cam_probe()
{
    while(1)
    {
		uint8_t ver = cam_sensor_read(BF3003_PID);
        if(ver == 0x30)
        {
            break;
        }
		printf("ver:%x\n",ver);
        bflb_mtimer_delay_ms(1000);
    }
    for(int i=0;i<sizeof(bf3003_init_list)/sizeof(bf3003_init_list[0]);i++){
        cam_sensor_write(bf3003_init_list[i].address, bf3003_init_list[i].paramete);
        bflb_mtimer_delay_ms(1);
    }
}
void cam_init()
{
    pwm = bflb_device_get_by_name("pwm_v2_0");
    gpio = bflb_device_get_by_name("gpio");
    
    bflb_gpio_uart_init(gpio, GPIO_PIN_21, GPIO_UART_FUNC_UART0_TX);
    bflb_gpio_uart_init(gpio, GPIO_PIN_22, GPIO_UART_FUNC_UART0_RX);
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

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

    bflb_gpio_init(gpio, PIN_SCLK, (GPIO_FUNC_I2C0| (1 << 8) | (1 << (9)) | (1 << (11)) | (1 << (12))));
    bflb_gpio_init(gpio, PIN_SDATA, (GPIO_FUNC_I2C0| (1 << 8) | (1 << (9)) | (1 << (11)) | (1 << (12))));

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

    bflb_gpio_init(gpio, PIN_XCLK, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_1);

    /* period = .XCLK / .clk_div / .period = 40MHz / 4 / 10 = 1000KHz */
    struct bflb_pwm_v2_config_s pwm_cfg = {
        .clk_source = BFLB_SYSTEM_XCLK,
        .clk_div = 1,
        .period = 4,
    };

    bflb_pwm_v2_init(pwm, &pwm_cfg);
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 1, 3); 
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
    bflb_pwm_v2_start(pwm);
    while(bflb_gpio_read(gpio,PIN_PCLK));
    while(!bflb_gpio_read(gpio,PIN_PCLK));
}