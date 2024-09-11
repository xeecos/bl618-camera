#include "FreeRTOS.h"
#include "task.h"
#include "bflb_gpio.h"
#include "bflb_cam.h"
#include "bflb_clock.h"
#include "bflb_uart.h"
#include "bflb_mjpeg.h"
#include "bf3003.h"
#include "config.h"
#include "sensor.h"
#include "uart.h"
#include "i2c.h"
#include "jpeg_head.h"
#include "cam.h"

static struct bflb_device_s *uart0;
static struct bflb_device_s *gpio;
static struct bflb_device_s *cam0;
static struct bflb_device_s *mjpeg;

volatile uint32_t pic_count = 0;
volatile uint32_t pic_addr[CAM_FRAME_COUNT_USE] = { 0 };
volatile uint32_t pic_len[CAM_FRAME_COUNT_USE] = { 0 };
volatile int led = 0;

uint8_t jpg_head_buf[800] = { 0 };
uint32_t jpg_head_len;

uint8_t MJPEG_QUALITY = 50;
static TaskHandle_t cam_process_task_hd;
static TaskHandle_t print_task_hd;
volatile uint32_t jpeg_len = 0;

static __attribute__((aligned(32))) ATTR_NOINIT_PSRAM_SECTION uint8_t dvp_buffer[640 * 2 * ROW_NUM];
static __attribute__((aligned(32))) ATTR_NOINIT_PSRAM_SECTION uint8_t mjpeg_buffer[50 * 1024 * CAM_FRAME_COUNT_USE];
void cam_isr(int irq, void *arg)
{
    // bflb_cam_int_clear(cam0, CAM_INTCLR_NORMAL);
    // cam_int_cnt++;
    // while(bflb_cam_get_frame_count(cam0)==0){}
    // // printf("CAM interrupt, pop picture %d len: %d\r\n", cam_int_cnt,  pic_size);
    bflb_cam_int_clear(cam0, CAM_INTCLR_NORMAL);
}
void mjpeg_isr(int irq, void *arg)
{
    uint8_t *pic;
    uint32_t intstatus = bflb_mjpeg_get_intstatus(mjpeg);
    bflb_mjpeg_stop(mjpeg);
    bflb_cam_stop(cam0);
    if (intstatus & MJPEG_INTSTS_ONE_FRAME) {
        bflb_mjpeg_int_clear(mjpeg, MJPEG_INTCLR_ONE_FRAME); 
        // bflb_mjpeg_int_clear(mjpeg, 1 << 10);
        
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(cam_process_task_hd, &pxHigherPriorityTaskWoken);
        if (pxHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
        }
        // if (pic_count == CAM_FRAME_COUNT_USE) {
        //     pic_count = 0;
            // bflb_cam_stop(cam0);
            // bflb_mjpeg_stop(mjpeg);
        // }
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
    for(int i=0; i<sizeof(bf3003_init_list)/sizeof(bf3003_init_list[0]); i++)
    {
        cam_sensor_write(bf3003_init_list[i].address, bf3003_init_list[i].paramete);
        if (i % 2 == 0)
        {
            bflb_gpio_set(gpio, PIN_LED);
        }
        else
        {
            bflb_gpio_reset(gpio, PIN_LED);
        }
        bflb_mtimer_delay_ms(20);
    }
}
void printf_uart(char *buf)
{
    int i = 0;
    while(buf[i])
    {

        bflb_uart_putchar(uart0, buf[i]);
        i++;
        if(i>100||buf[i]==0)
        {
            break;
        }
    }
}
void print_task(void* param)
{
    while(1)
    {
        printf("len:%d\n", 100);
        vTaskDelay(1000);
    }
}
void cam_task(void*param)
{
    cam_probe();
    uint8_t *pic;
    while (1)
    {
        vTaskDelay(25);
        // if(jpeg_len>0)
        {
            jpeg_len = 0;
            bflb_mjpeg_start(mjpeg);
            bflb_cam_start(cam0);
            // printf("len:%d\n", jpeg_len);
        }
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
        // bflb_mjpeg_int_clear(mjpeg, 1 << 10);
        // jpeg_len = bflb_mjpeg_get_frame_info(mjpeg, &pic);
        // printf("len:%d\n", jpeg_len);
        // pic_addr[pic_count] = (uint32_t)pic;
        // pic_len[pic_count] = jpeg_len;
        // pic_count++;
        bflb_mjpeg_pop_one_frame(mjpeg);

        if (led)
            bflb_gpio_set(gpio, PIN_LED);
        else bflb_gpio_reset(gpio, PIN_LED);
        led = 1-led;
    }
}
void cam_init()
{
    
    board_uartx_gpio_init();

    uart0 = bflb_device_get_by_name("uart0");

    struct bflb_uart_config_s cfg;

    cfg.baudrate = 2000000;
    cfg.data_bits = UART_DATA_BITS_8;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.parity = UART_PARITY_NONE;
    cfg.flow_ctrl = 0;
    cfg.tx_fifo_threshold = 7;
    cfg.rx_fifo_threshold = 7;
    cfg.bit_order = UART_LSB_FIRST;
    bflb_uart_init(uart0, &cfg);

    gpio = bflb_device_get_by_name("gpio");

    bflb_gpio_init(gpio, PIN_LED, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    // uart_init(gpio, PIN_TX0, PIN_RX0);
    i2c_init(gpio, PIN_SCL0, PIN_SDA0);

    
    
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
    // struct image_sensor_config_s *sensor_config = &bf3003_config;


    bflb_cam_int_mask(cam0, CAM_INTMASK_NORMAL, false);
    bflb_irq_attach(cam0->irq_num, cam_isr, NULL);
    bflb_irq_enable(cam0->irq_num);

    // memcpy(&cam_config, sensor_config, IMAGE_SENSOR_INFO_COPY_SIZE);
    cam_config.input_format = CAM_INPUT_FORMAT_YUV422_YUYV;
    cam_config.with_mjpeg = true;
    cam_config.resolution_x = 640;
    cam_config.resolution_y = 480;
    cam_config.h_blank = 0x80;
    cam_config.pixel_clock = 24000000;
    cam_config.input_source = CAM_INPUT_SOURCE_DVP;
    cam_config.output_format = CAM_OUTPUT_FORMAT_AUTO;
    cam_config.output_bufaddr = (uint32_t)dvp_buffer;
    cam_config.output_bufsize = cam_config.resolution_x * ROW_NUM * 2 ;
    //ROW_NUM;

    bflb_cam_init(cam0, &cam_config);
    bflb_cam_feature_control(cam0, CAM_CMD_COUNT_TRIGGER_NORMAL_INT, 1);
    mjpeg = bflb_device_get_by_name("mjpeg");

    struct bflb_mjpeg_config_s config;

    config.format = MJPEG_FORMAT_YUV422_YUYV;
    config.quality = MJPEG_QUALITY;
    config.rows = ROW_NUM;
    config.resolution_x = cam_config.resolution_x;
    config.resolution_y = cam_config.resolution_y;
    config.input_bufaddr0 = (uint32_t)dvp_buffer;
    config.input_bufaddr1 = 0;
    config.output_bufaddr = (uint32_t)mjpeg_buffer;
    config.output_bufsize = sizeof(mjpeg_buffer);
    config.input_yy_table = NULL; /* use default table */
    config.input_uv_table = NULL; /* use default table */

    bflb_mjpeg_init(mjpeg, &config);

    jpg_head_len = JpegHeadCreate(YUV_MODE_422, MJPEG_QUALITY, cam_config.resolution_x, cam_config.resolution_y, jpg_head_buf);
    bflb_mjpeg_fill_jpeg_header_tail(mjpeg, jpg_head_buf, jpg_head_len);

    bflb_mjpeg_tcint_mask(mjpeg, false);
    bflb_irq_attach(mjpeg->irq_num, mjpeg_isr, NULL);
    bflb_irq_enable(mjpeg->irq_num);

    // uint32_t reg_temp = getreg32(mjpeg->reg_base + 0x00);
    // putreg32((reg_temp | (1 << 3)), (mjpeg->reg_base + 0x00)); /* enable over_size int */

    // bflb_mjpeg_tcint_mask(mjpeg, false);
    // reg_temp = getreg32(mjpeg->reg_base + 0x1C);
    // putreg32((reg_temp | (1 << 2)), (mjpeg->reg_base + 0x1C)); /* enable over_size int */
    // bflb_irq_attach(mjpeg->irq_num, mjpeg_isr, NULL);
    // bflb_irq_enable(mjpeg->irq_num);

    
    // while(1)
    // {
    //     printf("hi\n");
    //     bflb_mtimer_delay_ms(500);
    // }
    
    xTaskCreate(cam_task, (char *)"cam_task", 1024, NULL, 14, &cam_process_task_hd);
    // xTaskCreate(print_task, (char *)"print_task", 1024, NULL, 14, &print_task_hd);
}