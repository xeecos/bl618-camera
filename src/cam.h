#pragma once
#include "bflb_gpio.h"
#include "bflb_cam.h"
#include "bflb_clock.h"
#include "bf3003.h"
#include "config.h"
#include "sensor.h"
#include "uart.h"
#include "i2c.h"
struct bflb_device_s *gpio;
struct bflb_device_s *cam0;

#define CAM_FRAME_COUNT_USE 50
static volatile uint32_t cam_int_cnt = 0, pic_size;
static uint8_t *pic;
void cam_isr(int irq, void *arg)
{
    bflb_cam_int_clear(cam0, CAM_INTCLR_NORMAL);
    cam_int_cnt++;
    pic_size = bflb_cam_get_frame_info(cam0, &pic);
    bflb_cam_pop_one_frame(cam0);
    printf("CAM interrupt, pop picture %d: 0x%08x, len: %d\r\n", cam_int_cnt, (uint32_t)pic, pic_size);
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
    i2c_transfer(msgs, 2);
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

    i2c_transfer(msgs, 2);
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
    gpio = bflb_device_get_by_name("gpio");

    bflb_gpio_init(gpio, PIN_LED, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    uart_init(gpio, PIN_TX0, PIN_RX0);
    i2c_init(gpio, PIN_SCL0, PIN_SDA0);

    cam_probe();
    int led = 0;
    while(1)
    {
        if(led)bflb_gpio_set(gpio, PIN_LED);
        else bflb_gpio_reset(gpio, PIN_LED);
        led = 1-led;
        bflb_mtimer_delay_ms(100);
    }

    bflb_gpio_init(gpio, PIN_CAM_XCLK, GPIO_FUNC_CLKOUT | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D0, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D1, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D2, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D3, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D4, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D5, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D6, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_D7, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_HREF, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_VSYNC, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, PIN_CAM_PIXCLK, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    cam0 = bflb_device_get_by_name("cam0");
    struct bflb_cam_config_s cam_config;
    struct image_sensor_config_s *sensor_config;

    if (sensor_scan(i2c0, &sensor_config)) {
        printf("\r\nSensor name: %s\r\n", sensor_config->name);
    } else {
        printf("\r\nError! Can't identify sensor!\r\n");
        while (1) {
        }
    }

    bflb_cam_int_mask(cam0, CAM_INTMASK_NORMAL, false);
    bflb_irq_attach(cam0->irq_num, cam_isr, NULL);
    bflb_irq_enable(cam0->irq_num);

    memcpy(&cam_config, sensor_config, IMAGE_SENSOR_INFO_COPY_SIZE);
    cam_config.with_mjpeg = false;
    cam_config.input_source = CAM_INPUT_SOURCE_DVP;
    cam_config.output_format = CAM_OUTPUT_FORMAT_AUTO;
    cam_config.output_bufaddr = BFLB_PSRAM_BASE;
    cam_config.output_bufsize = cam_config.resolution_x * cam_config.resolution_y * 12;

    bflb_cam_init(cam0, &cam_config);
    bflb_cam_start(cam0);
    // pwm = bflb_device_get_by_name("pwm_v2_0");
    // bflb_gpio_init(gpio, PIN_CAM_D0, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D1, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D2, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D3, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D4, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D5, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D6, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_init(gpio, PIN_CAM_D7, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // bflb_gpio_int_init(gpio, PIN_CAM_VSYNC, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    // bflb_gpio_int_mask(gpio, PIN_CAM_VSYNC, false);

    // bflb_gpio_int_init(gpio, PIN_CAM_HREF, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    // bflb_gpio_int_mask(gpio, PIN_CAM_HREF, false);

    // bflb_gpio_int_init(gpio, PIN_CAM_PIXCLK, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    // bflb_gpio_int_mask(gpio, PIN_CAM_PIXCLK, false);


    // bflb_gpio_init(gpio, PIN_CAM_XCLK, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_1);

    // /* period = .XCLK / .clk_div / .period = 80MHz / 2 / 4 = 10MHz */
    // struct bflb_pwm_v2_config_s pwm_cfg = {
    //     .clk_source = BFLB_SYSTEM_PBCLK,
    //     .clk_div = 4,
    //     .period = 4,
    // };

    // bflb_pwm_v2_init(pwm, &pwm_cfg);
    // bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 1, 3); 
    // bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
    // bflb_pwm_v2_start(pwm);
}