#pragma once

#define BLOCK_NUM               2
#define ROW_NUM                 (8 * BLOCK_NUM)
#define CAM_FRAME_COUNT_USE     1
#define SIZE_BUFFER             (4 * 1024 * 1024)

#define DVP_ISR_NOTIFY_INDEX  (0)
#define JPEG_ISR_NOTIFY_INDEX (1)
void cam_isr(int irq, void *arg);
void mjpeg_isr(int irq, void *arg);
uint8_t cam_sensor_read(uint8_t address);

void cam_sensor_write(uint8_t address, uint8_t paramete);
void cam_probe();
void printf_uart(char *buf);
void print_task(void *param);
void cam_task(void *param);
void cam_init();
int cam_status();
void cam_set_status(int status);
uint32_t cam_data_length();
uint8_t *cam_data();