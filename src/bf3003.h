#pragma once

#include "sensor.h"

#define BF3003_ADDR                 0x6E
#define BF3003_PID                  0xFC
#define BF3003_VER                  0xfd
#define BF3003_VHREF                0x03
#define BF3003_OFFSET_4x_DOWN       0x04
#define BF3003_LOFFN1E              0x05
#define BF3003_LOFFN0E              0x06
#define BF3003_OFFSET_2X            0x07
#define BF3003_COM2                 0x09
#define BF3003_COM3                 0x0C
#define BF3003_CLKRC                0x11
#define BF3003_COM7                 0x12
#define BF3003_COM8                 0x13
#define BF3003_COM10                0x15
#define BF3003_HSTART               0x17
#define BF3003_HSTOP                0x18
#define BF3003_VSTART               0x19
#define BF3003_VSTOP                0x1A
#define BF3003_PLLCTL               0x1B
#define BF3003_AVER_LOCK1           0x1C
#define BF3003_AVER_LOCK2           0x1D
#define BF3003_MVFP                 0x1E
#define BF3003_DBLK_TARO            0x1F
#define BF3003_DBLK_TARE            0x22
#define BF3003_BLUE_GAIN            0x01
#define BF3003_RED_GAIN             0x02
#define BF3003_GREEN_GAIN           0x23
#define BF3003_GN_GAIN              0x6A
#define BF3003_BLUE_GAIN_LOW        0xA2
#define BF3003_BLUE_GAIN_HIGH       0xA3
#define BF3003_RED_GAIN_LOW         0xA4
#define BF3003_REDE_GAIN_HIGH       0xA5
#define BF3003_GLB_GAIN             0x87
#define BF3003_GLB_GAIN_MIN         0x82
#define BF3003_GLB_GAIN_MAX         0x86
#define BF3003_STEPO                0x27
#define BF3003_DBLK_CNTL            0x28
#define BF3003_EXHCH                0x2A
#define BF3003_EXHCL                0x2B
#define BF3003_HREF_CNTL            0xE2
#define BF3003_DM_LNL               0x92
#define BF3003_DM_LNH               0x93
#define BF3003_DM_ROWL              0xE4
#define BF3003_DM_ROWH              0xE3
#define BF3003_TSLB                 0x3A
#define BF3003_AE_MODE              0x80
#define BF3003_AE_SPEED             0x81
#define BF3003_TEST_MODE            0xB9
#define BF3003_MODE_SEL             0xf0
#define BF3003_SUBSAMPLE            0x4a
#define BF3003_DICOM1               0x69
#define BF3003_INT_MEAN_H           0X89
#define BF3003_INT_MEAN_L           0X8A
#define BF3003_INT_TIM_MIN          0X8B
#define BF3003_INT_TIM_HI           0X8C
#define BF3003_INT_TIM_LO           0X8D
#define BF3003_INT_TIM_MAX_HI       0X8E
#define BF3003_INT_TIM_MAX_LO       0X8F
#define BF3003_LINE_CTR             0xF5 

static struct image_sensor_command_s bf3003_init_list[] = {
    {BF3003_COM7, 0b10000000},
	{BF3003_COM2, 0b00000001},
	/*
	Common control 2
	Bit[7:6]: vclk output drive capability
		00:1x 01:1.5x 10:2.5x 11:3x
	Bit[5]:Tri-state option for output data at power down period
		0:tri-state at this period
		1:No tri-state at this period
	Bit[4]:Tri-state option for output clock at power down period
		0:tri-state at this period
		1:No tri-state at this period
	Bit[3:2]: hsync output drive capability
		00:1x 01:1.5x 10:2.5x 11:3x
	when drivesel=0
	Bit[1:0]: data&clk&Hsync output drive capability
		00:1x 01:1.5x 10:2.5x 11:3x
	when drivesel=1
		Bit[1:0]: data output drive capability
		00:1x 01:1.5x 10:2.5x 11:3x
	*/
	{BF3003_COM3, 0b00000000},
	/*
	Bit[7]:PROCRSS RAW selection
		0: process raw from ycbcr to rgb conversion in datformat
		1: process raw from color interpolation(deniose,gamma,lsc is selectable)
	Bit[6]:Output data MSB and LSB swap
	Bit[5:4]:PROCESS RAW sequence(when 0x0c[7]=0):
		00: (LINE0:BGBG/LINE1:GRGR)
		01: (LINE0:GBGB/LINE1:RGRG)
		10: (LINE0:GRGR/LINE1:BGBG)
		11: (LINE0:RGRG/LINE1:GBGB)
	Bit[3]:
		0:no HREF when VSYNC_DAT=0;
		1:always has HREF no matter VSYNC_DAT=0 or not;
	Bit[2]:DATA ahead 1 clk(YUV MCLK,RawData PCLK) or not
	Bit[1]:HREF ahead 1 clk(YUV MCLK,RawData PCLK) or not
	Bit[0]:HREF ahead 0.5 clk(YUV MCLK,RawData PCLK) or not
	0x0c[1:0]: Internal use only
	*/
	{BF3003_CLKRC, 0b1000},
	/*
	Mclk_div control
	Bit[7：2]: Internal use only
	Bit[1:0]:Internal MCLK pre-scalar
		00:divided by 1 F(MCLK)=F(pll output clock)
		01:divided by 2 F(MCLK)=F(pll output clock)/2
		10:divided by 4 F(MCLK)=F(pll output clock)/4
		11: no clocking, digital stand by mode(all clocks freeze)
	*/
	{BF3003_COM7, 0b00000000},
	/*
		Bit[7]: SCCB Register Reset
			0: No change
			1: Resets all registers to default values 
		Bit[6]: Reserved
		Bit[5]: (when 0x4a =03h)0: row 1/2 sub,1: output input image.
		Bit[4]: 1/2 digital subsample Selection(only for YUV422/RGB565/RGB555/RGB444 output).
		Bit[3]: data selection
			0:normal(YUV422/RGB565/RGB555/RGB444/BAYER RAW/PRO RAW)
			1:CCIR656 output enable(for TV)
		Bit[2]: YUV422/RGB565/RGB555/RGB444 Selection.
		Bit[1]: Reserved.
		Bit[0]: Raw RGB Selection.
			{0x12[2],0x12[0]}
			00: YUV422
			01: Bayer RAW
			10: RGB565/RGB555/RGB444(use with 0x3a) 
			11: Process RAW (use with 0x0c[7]) 
 	*/
	{BF3003_TSLB, 0b00000000},
	/*
		if YUV422 is selected,the Sequence is:
		Bit[1:0]:Output YUV422 Sequence
		00: YUYV, 01: YVYU
		10: UYVY, 11: VYUY
		if RGB565/RGB555/RGB444 is selected,the Sequence is:
		Bit[4:0]:Output RGB565/RGB555/RGB444 Sequence
		RGB565:
		00h: R5G3H,G3LB5 01h: B5G3H,G3LR5
		02h: B5R3H,R2LG6 03h: R5B3H,B2LG6
		04h: G3HB5,R5G3L 05h: G3LB5,R5G3H
		06h: G3HR5,B5G3L 07h: G3LR5,B5G3H
		08h: G6B2H,B3LR5 09h: G6R2H,R3LB5
	*/
	{BF3003_COM8, 0b00010111},
	/*
		Auto mode Contrl
		Bit[7:6] reserved
		Bit[5:4]：Sensitivity enable,
		Bit[5]: 0:manual adjust , 1 :auto adjust
		Bit[4]: when manual adjust,write 1, high sensitivity
		write 0, low sensitivity
		Bit[4]: select which gain to be used,when short int_tim
		adjust:
		0: use glb_gain_short
		1: use glb_gain
		Bit[3]: Reserved.
		Bit[2]: AGC Enable. 0:OFF , 1: ON.
		Bit[1]: AWB Enable. 0:OFF , 1: ON.
		Bit[0]: AEC Enable. 0:OFF , 1: ON. 
	*/
	{BF3003_COM10, 0b00000010},
	/* BF3003_COM10
		Bit[7]: Reserved
		Bit[6]: 0:HREF, 1:HSYNC
		Bit[5]: 0:VSYNC_IMAGE, 1:VSYNC_DAT
		Bit[4]: VCLK reverse
		Bit[3]: HREF option, 0:active high, 1:active low.
		Bit[2]: Reserved
		Bit[1]: VSYNC option, 0:active low, 1:active high.
		Bit[0]: HSYNC option, 0:active high, 1:active low.
	*/
	{BF3003_VHREF, 0b0100},
	{BF3003_HSTART, 0x1},
	{BF3003_HSTOP, 0xA0},
	{BF3003_VSTART, 0x0},
	{BF3003_VSTOP, 0x78},
	{BF3003_PLLCTL, 0b00101010},
	/*
	PLLCTL[7]: PLL Enable
		0:enable
		1:disable
	PLLCTL[6:0]: Reserved
	*/
	{BF3003_HREF_CNTL,0b000},
	/*
	HREF_CNTL[2:0]: 000:delay third,delay two pclk;
		001:delay fourth,delay three pclk;
		010:delay fifth,delay four pclk;
		011:delay sixth,delay five pclk;
		100:delay seventh,delay six pclk;
		101:delay eighth,delay seven pclk;
		110:delay ninth,delay eight pclk;
		111:delay tenth,delay nine pclk;
	*/
	{BF3003_EXHCH,0x00},
	{BF3003_EXHCL,0x80},
	/*
	Dummy Pixel Insert MSB
		Bit[7:4]: 4MSB for dummy pixel insert in horizontal direction
	Dummy Pixel Insert LSB
		8 LSB for dummy pixel insert in horizontal direction
	*/
	{BF3003_DM_ROWH, 0x00},
	/*
		Dummy line insert before active line low 8 bits 
	*/
	{BF3003_DM_ROWL, 0x00},
	/*
		Dummy line insert before active line high 8 bits 
	*/
	{BF3003_DM_LNL, 0x00},
	/*
		insert the dummy line after active line(Dummy line low 8bits)
		it's default value is 0x28;
	*/
	{BF3003_DM_LNH, 0x00},
	/*
		insert the dummy line after active line(Dummy line high 8bits) 
	*/
	{BF3003_AE_MODE, 0b11000000},
	/*
	Bit[7]: AE mode select:
		0: use Y (from color space module).
		1: use rawdata (from gamma module), (when special effect in color interpolation module is selected,0x80[7] must set
	to be 1'b1)
	Bit[6]: INT_TIM lower than INT_STEP_5060 or not:
		0: limit int_tim>=step(no flicker)
		1: int_tim can be less than 1*int_step(existing flicker).
	Bit[5:4]: center window select:
	vga and ntsc mode 00: 512*384(full) ,256*192(1/2sub when normal mode)
	and ntsc have no sub 
		01: 384*288(full) ,192*144(1/2sub when normal mode)
		10 : 288*216(full) ,144*108(1/2sub when normal mode)
		11: 216*160(full) ,108*80 (1/2sub when normal mode)
	pal mode 00: 512*448
		01: 384*336
		10: 288*256
		11: 216*192
	Bit[3:1]: weight select: weight_sel region1 region2 region3 region4
		000: 1/4 1/4 1/4 1/4
		001: 1/2 1/4 1/8 1/8
		010: 5/8 1/8 1/8 1/8
		011: 3/8 3/8 1/8 1/8
		100: 3/4 1/4 0 0
		101: 5/8 3/8 0 0
		110: 1/2 1/2 0 0
		111: 1 0 0 0
	Bit[0]: Banding filter value select
		0: Select {0x89[5],0x9E[7:0]} as Banding Filter Value.
		1: Select {0x89[4],0x9D[7:0]} as Banding Filter Value
	*/
	{BF3003_TEST_MODE, 0b10000000},
	/*
	BIT[7] : 
		1: test pattern enable
		0: bypass test pattern
	BIT[6:5]: 
		00: output color bar pattern
		01: output gradual pattern
		1x: output manual write R/G/B
	BIT[4] : 0:vertical pattern, 1:horizontal pattern
	BIT[3:0]: gradual gray pattern mode control 
	*/
	{BF3003_MODE_SEL, 0b0},
	{BF3003_SUBSAMPLE, 0b0},
	{BF3003_BLUE_GAIN, 0x1},
	{BF3003_RED_GAIN, 0x1},
	{BF3003_GREEN_GAIN, 0x1},
	{BF3003_DICOM1, 0x80},
	/*Bit[7]: YCBCR RANGE select
		0: YCBCR 0~255
		1: Y 16~235, CBCR 16~240
	Bit[6]: Negative image enable
		0: Normal image, 1: Negative image
	Bit[5]: UV output value select.
		0: output normal value
		1: output fixed value set in MANU and MANV
	Bit[4]:U、V dither when ycbcr mode/R、B dither when rgb
	mode:
		0: low 2 bits, 1: low 3bits
	Bit[3]:Y dither when ycbcr mode/G dither when rgb mode:
		0: low 2 bits, 1: low 3bits
	Bit[2]:Y dither enable
	Bit[1]:U、V dither enable
	Bit[0]:RGB dither enable 
	*/
	{BF3003_INT_MEAN_H,  0x32},
	{BF3003_INT_MEAN_L,  0xAA},
	{BF3003_INT_TIM_MIN, 0x00},
	{BF3003_INT_TIM_HI,  0x00},
	{BF3003_INT_TIM_LO,  0x00},
	{BF3003_INT_TIM_MAX_HI, 0xFF},
	{BF3003_INT_TIM_MAX_LO, 0xFF},
	{BF3003_LINE_CTR, 0x1},
	{BF3003_GLB_GAIN_MIN, 0x00},
	{BF3003_GLB_GAIN_MAX, 0xFF},
	{BF3003_GLB_GAIN, 0x10},
};


static struct image_sensor_config_s bf3003_config = {
    .name = "BF3003",
    .output_format = IMAGE_SENSOR_FORMAT_YUV422_YUYV,
    .slave_addr = BF3003_ADDR,
    .id_size = 1,
    .reg_size = 1,
    .h_blank = 0x90,
    .resolution_x = 640,
    .resolution_y = 480,
    .id_addr = BF3003_PID,
    .id_value = 0x30,
    .pixel_clock = 24000000,
    .init_list_len = sizeof(bf3003_init_list)/sizeof(bf3003_init_list[0]),
    .init_list = bf3003_init_list,
};