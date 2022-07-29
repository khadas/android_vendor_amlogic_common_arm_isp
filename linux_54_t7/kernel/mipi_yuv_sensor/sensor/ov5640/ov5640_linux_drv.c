// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AML_SENSOR_NAME  "ov5640-%u"

/* min/typical/max system clock (xclk) frequencies 6M ~ 27M */
#define OV5640_XCLK_MIN  6000000
#define OV5640_XCLK_MAX 27000000

#define OV5640_DEFAULT_SLAVE_ID    0x3c
#define OV5640_REG_SYS_RESET02          0x3002
#define OV5640_REG_SYS_CLOCK_ENABLE02   0x3006
#define OV5640_REG_SYS_CTRL0            0x3008
#define OV5640_REG_SYS_CTRL0_SW_PWDN    0x42
#define OV5640_REG_SYS_CTRL0_SW_PWUP    0x02
#define OV5640_REG_CHIP_ID              0x300a
#define OV5640_REG_IO_MIPI_CTRL00       0x300e
#define OV5640_REG_PAD_OUTPUT_ENABLE01  0x3017
#define OV5640_REG_PAD_OUTPUT_ENABLE02  0x3018
#define OV5640_REG_PAD_OUTPUT00         0x3019
#define OV5640_REG_SYSTEM_CONTROL1      0x302e
#define OV5640_REG_SC_PLL_CTRL0         0x3034
#define OV5640_REG_SC_PLL_CTRL1         0x3035
#define OV5640_REG_SC_PLL_CTRL2         0x3036
#define OV5640_REG_SC_PLL_CTRL3         0x3037
#define OV5640_REG_SLAVE_ID             0x3100
#define OV5640_REG_SCCB_SYS_CTRL1       0x3103
#define OV5640_REG_SYS_ROOT_DIVIDER     0x3108
#define OV5640_REG_AWB_R_GAIN           0x3400
#define OV5640_REG_AWB_G_GAIN           0x3402
#define OV5640_REG_AWB_B_GAIN           0x3404
#define OV5640_REG_AWB_MANUAL_CTRL      0x3406
#define OV5640_REG_AEC_PK_EXPOSURE_HI   0x3500
#define OV5640_REG_AEC_PK_EXPOSURE_MED  0x3501
#define OV5640_REG_AEC_PK_EXPOSURE_LO   0x3502
#define OV5640_REG_AEC_PK_MANUAL        0x3503
#define OV5640_REG_AEC_PK_REAL_GAIN     0x350a
#define OV5640_REG_AEC_PK_VTS           0x350c
#define OV5640_REG_TIMING_DVPHO         0x3808
#define OV5640_REG_TIMING_DVPVO         0x380a
#define OV5640_REG_TIMING_HTS           0x380c
#define OV5640_REG_TIMING_VTS           0x380e
#define OV5640_REG_TIMING_TC_REG20      0x3820
#define OV5640_REG_TIMING_TC_REG21      0x3821
#define OV5640_REG_AEC_CTRL00           0x3a00
#define OV5640_REG_AEC_B50_STEP         0x3a08
#define OV5640_REG_AEC_B60_STEP         0x3a0a
#define OV5640_REG_AEC_CTRL0D           0x3a0d
#define OV5640_REG_AEC_CTRL0E           0x3a0e
#define OV5640_REG_AEC_CTRL0F           0x3a0f
#define OV5640_REG_AEC_CTRL10           0x3a10
#define OV5640_REG_AEC_CTRL11           0x3a11
#define OV5640_REG_AEC_CTRL1B           0x3a1b
#define OV5640_REG_AEC_CTRL1E           0x3a1e
#define OV5640_REG_AEC_CTRL1F           0x3a1f
#define OV5640_REG_HZ5060_CTRL00        0x3c00
#define OV5640_REG_HZ5060_CTRL01        0x3c01
#define OV5640_REG_SIGMADELTA_CTRL0C    0x3c0c
#define OV5640_REG_FRAME_CTRL01         0x4202
#define OV5640_REG_FORMAT_CONTROL00     0x4300
#define OV5640_REG_VFIFO_HSIZE          0x4602
#define OV5640_REG_VFIFO_VSIZE          0x4604
#define OV5640_REG_JPG_MODE_SELECT      0x4713
#define OV5640_REG_CCIR656_CTRL00       0x4730
#define OV5640_REG_POLARITY_CTRL00      0x4740
#define OV5640_REG_MIPI_CTRL00          0x4800
#define OV5640_REG_DEBUG_MODE           0x4814
#define OV5640_REG_ISP_FORMAT_MUX_CTRL  0x501f
#define OV5640_REG_PRE_ISP_TEST_SET1    0x503d
#define OV5640_REG_SDE_CTRL0            0x5580
#define OV5640_REG_SDE_CTRL1            0x5581
#define OV5640_REG_SDE_CTRL3            0x5583
#define OV5640_REG_SDE_CTRL4            0x5584
#define OV5640_REG_SDE_CTRL5            0x5585
#define OV5640_REG_AVG_READOUT          0x56a1


enum ov5640_mode_id {
	OV5640_MODE_1080P_1920_1080,
	//OV5640_MODE_QSXGA_2592_1944,
	OV5640_NUM_MODES,
};

enum ov5640_frame_rate {
	OV5640_15_FPS = 0,
	OV5640_30_FPS,
	OV5640_60_FPS,
	OV5640_NUM_FRAMERATES,
};

enum ov5640_format_mux {
	OV5640_FMT_MUX_YUV422 = 0,
	OV5640_FMT_MUX_RGB,
	OV5640_FMT_MUX_DITHER,
	OV5640_FMT_MUX_RAW_DPC,
	OV5640_FMT_MUX_SNR_RAW,
	OV5640_FMT_MUX_RAW_CIP,
};

struct ov5640_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct ov5640_pixfmt ov5640_formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8,     V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_UYVY8_1X16,    V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_VYUY8_2X8,     V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_VYUY8_1X16,    V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_2X8,     V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_1X16,    V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YVYU8_2X8,     V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YVYU8_1X16,    V4L2_COLORSPACE_SRGB, },
};

/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
static unsigned int virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
		 "MIPI CSI-2 virtual channel (0..3), default 0");

static const int ov5640_framerates[] = {
	[OV5640_15_FPS] = 15,
	[OV5640_30_FPS] = 30,
	[OV5640_60_FPS] = 60,
};

/*
 * Image size under 1280 * 960 are SUBSAMPLING
 * Image size upper 1280 * 960 are SCALING
 */
enum ov5640_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct ov5640_mode_info {
	enum ov5640_mode_id id;
	enum ov5640_downsize_mode dn_mode;
	u32 hact;  // h active
	u32 htot;  // h total
	u32 vact;  // v active
	u32 vtot;  // v total
	const struct reg_value *reg_data;
	u32 reg_data_size;
	u32 max_fps;
};

struct ov5640_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *light_freq;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

struct ov5640_dev {
	int index;
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad   pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to OV5640 */
	struct clk *clk_mclk_x_pre;
	struct clk *clk_xtal;

	u32         xclk_freq;

	int reset_gpio;
	int pwdn_gpio;
	bool   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;


	const struct ov5640_mode_info *current_mode;
	const struct ov5640_mode_info *last_mode;
	enum ov5640_frame_rate         current_fr;
	struct v4l2_fract              frame_interval;

	struct ov5640_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;
	bool pending_mode_change;

	bool streaming;
};

static inline struct ov5640_dev *to_ov5640_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5640_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov5640_dev,
			     ctrls.handler)->sd;
}

static const struct reg_value ov5640_init_setting_30fps_1920x1080[] = {
	{0x3103, 0x11, 0, 0}, // system clk from pad. bit[1] = 0
	{0x3008, 0x82, 0, 5}, // soft reset. bit[7] = 1. sleep 5ms
	{0x3008, 0x42, 0, 0}, // soft power down. bit[6] = 1
	{0x3103, 0x03, 0, 0}, // system clk from pll bit[1] = 1.

	{0x3017, 0x00, 0, 0}, // pin pad for mipi
	{0x3018, 0x00, 0, 0}, // pin pad for mipi

	{0x3034, 0x18, 0, 0}, //mipi 8 bit
	{0x3035, 0x11, 0, 0},
	{0x3036, 0x54, 0, 0},
	{0x3037, 0x13, 0, 0}, // system control regs
	{0x3108, 0x01, 0, 0}, // SYSTEM ROOT DIVIDER

	{0x3630, 0x36, 0, 0},
	{0x3631, 0x0e, 0, 0},
	{0x3632, 0xe2, 0, 0},
	{0x3633, 0x12, 0, 0},
	{0x3621, 0xe0, 0, 0},
	{0x3704, 0xa0, 0, 0},
	{0x3703, 0x5a, 0, 0},
	{0x3715, 0x78, 0, 0},
	{0x3717, 0x01, 0, 0},
	{0x370b, 0x60, 0, 0},
	{0x3705, 0x1a, 0, 0},
	{0x3905, 0x02, 0, 0},
	{0x3906, 0x10, 0, 0},
	{0x3901, 0x0a, 0, 0},
	{0x3731, 0x12, 0, 0},
	{0x3600, 0x08, 0, 0},
	{0x3601, 0x33, 0, 0},
	{0x302d, 0x60, 0, 0},
	{0x3620, 0x52, 0, 0},
	{0x371b, 0x20, 0, 0},
	{0x471c, 0x50, 0, 0},
	{0x3a13, 0x43, 0, 0},
	{0x3a18, 0x00, 0, 0},
	{0x3a19, 0xf8, 0, 0},
	{0x3635, 0x13, 0, 0},
	{0x3636, 0x03, 0, 0},
	{0x3634, 0x40, 0, 0},
	{0x3622, 0x01, 0, 0},
	{0x3c01, 0x34, 0, 0},
	{0x3c04, 0x28, 0, 0},
	{0x3c05, 0x98, 0, 0},
	{0x3c06, 0x00, 0, 0},
	{0x3c07, 0x07, 0, 0},
	{0x3c08, 0x00, 0, 0},
	{0x3c09, 0x1c, 0, 0},
	{0x3c0a, 0x9c, 0, 0},
	{0x3c0b, 0x40, 0, 0},
	{0x3820, 0x40, 0, 0},
	{0x3821, 0x06, 0, 0},
	{0x3814, 0x11, 0, 0},
	{0x3815, 0x11, 0, 0},
	{0x3800, 0x01, 0, 0},
	{0x3801, 0x50, 0, 0},
	{0x3802, 0x01, 0, 0},
	{0x3803, 0xb2, 0, 0},
	{0x3804, 0x08, 0, 0},
	{0x3805, 0xef, 0, 0},
	{0x3806, 0x05, 0, 0},
	{0x3807, 0xf1, 0, 0},
	{0x3808, 0x07, 0, 0},
	{0x3809, 0x80, 0, 0},
	{0x380a, 0x04, 0, 0},
	{0x380b, 0x38, 0, 0},
	{0x380c, 0x09, 0, 0},
	{0x380d, 0xc4, 0, 0},
	{0x380e, 0x04, 0, 0},
	{0x380f, 0x60, 0, 0},
	{0x3810, 0x00, 0, 0},
	{0x3811, 0x10, 0, 0},
	{0x3812, 0x00, 0, 0},
	{0x3813, 0x04, 0, 0},
	{0x3618, 0x04, 0, 0},
	{0x3612, 0x2b, 0, 0},
	{0x3708, 0x63, 0, 0},
	{0x3709, 0x12, 0, 0},
	{0x370c, 0x00, 0, 0},
	{0x3a02, 0x04, 0, 0},
	{0x3a03, 0x60, 0, 0},
	{0x3a08, 0x01, 0, 0},
	{0x3a09, 0x50, 0, 0},
	{0x3a0a, 0x01, 0, 0},
	{0x3a0b, 0x18, 0, 0},//
	{0x3a0e, 0x03, 0, 0},
	{0x3a0d, 0x04, 0, 0},
	{0x3a14, 0x04, 0, 0},//
	{0x3a15, 0x60, 0, 0},//
	{0x4001, 0x02, 0, 0},
	{0x4004, 0x06, 0, 0},//
	{0x4050, 0x6e, 0, 0},//
	{0x4051, 0x8f, 0, 0},//
	{0x3000, 0x00, 0, 0},
	{0x3002, 0x1c, 0, 0},
	{0x3004, 0xff, 0, 0},
	{0x3006, 0xc3, 0, 0},
	{0x300e, 0x45, 0, 0},
	{0x302e, 0x08, 0, 0},
	{0x4300, 0x33, 0, 0},// 30 vyuy / 31 uyvy / 32 yvyu / 33 yuyv
	{0x501f, 0x00, 0, 0},
	{0x5684, 0x07, 0, 0},//
	{0x5685, 0xa0, 0, 0},//
	{0x5686, 0x04, 0, 0},//
	{0x5687, 0x40, 0, 0},//
	{0x4713, 0x02, 0, 0},
	{0x4407, 0x04, 0, 0},
	{0x440e, 0x00, 0, 0},
	{0x460b, 0x37, 0, 0},//
	{0x460c, 0x20, 0, 0},//
	{0x4837, 0x0a, 0, 0},
	{0x3824, 0x04, 0, 0},//
	{0x5000, 0xa7, 0, 0},
	{0x5001, 0x83, 0, 0},//
	{0x5180, 0xff, 0, 0},
	{0x5181, 0xf2, 0, 0},
	{0x5182, 0x00, 0, 0},
	{0x5183, 0x14, 0, 0},
	{0x5184, 0x25, 0, 0},
	{0x5185, 0x24, 0, 0},
	{0x5186, 0x09, 0, 0},
	{0x5187, 0x09, 0, 0},
	{0x5188, 0x09, 0, 0},
	{0x5189, 0x75, 0, 0},//
	{0x518a, 0x54, 0, 0},
	{0x518b, 0xe0, 0, 0},//
	{0x518c, 0xb2, 0, 0},
	{0x518d, 0x42, 0, 0},//
	{0x518e, 0x3d, 0, 0},//
	{0x518f, 0x56, 0, 0},//
	{0x5190, 0x46, 0, 0},
	{0x5191, 0xf8, 0, 0},
	{0x5192, 0x04, 0, 0},
	{0x5193, 0x70, 0, 0},
	{0x5194, 0xf0, 0, 0},
	{0x5195, 0xf0, 0, 0},
	{0x5196, 0x03, 0, 0},
	{0x5197, 0x01, 0, 0},
	{0x5198, 0x04, 0, 0},
	{0x5199, 0x12, 0, 0},//
	{0x519a, 0x04, 0, 0},
	{0x519b, 0x00, 0, 0},
	{0x519c, 0x06, 0, 0},//
	{0x519d, 0x82, 0, 0},//
	{0x519e, 0x38, 0, 0},
	{0x5381, 0x1e, 0, 0},
	{0x5382, 0x5b, 0, 0},
	{0x5383, 0x08, 0, 0},
	{0x5384, 0x0a, 0, 0},
	{0x5385, 0x7e, 0, 0},
	{0x5386, 0x88, 0, 0},
	{0x5387, 0x7c, 0, 0},
	{0x5388, 0x6c, 0, 0},
	{0x5389, 0x10, 0, 0},
	{0x538a, 0x01, 0, 0},
	{0x538b, 0x98, 0, 0},
	{0x5300, 0x08, 0, 0},
	{0x5301, 0x30, 0, 0},
	{0x5302, 0x10, 0, 0},
	{0x5303, 0x00, 0, 0},
	{0x5304, 0x08, 0, 0},
	{0x5305, 0x30, 0, 0},
	{0x5306, 0x08, 0, 0},
	{0x5307, 0x16, 0, 0},
	{0x5309, 0x08, 0, 0},
	{0x530a, 0x30, 0, 0},
	{0x530b, 0x04, 0, 0},
	{0x530c, 0x06, 0, 0},
	{0x5480, 0x01, 0, 0},
	{0x5481, 0x08, 0, 0},
	{0x5482, 0x14, 0, 0},
	{0x5483, 0x28, 0, 0},
	{0x5484, 0x51, 0, 0},
	{0x5485, 0x65, 0, 0},
	{0x5486, 0x71, 0, 0},
	{0x5487, 0x7d, 0, 0},
	{0x5488, 0x87, 0, 0},
	{0x5489, 0x91, 0, 0},
	{0x548a, 0x9a, 0, 0},
	{0x548b, 0xaa, 0, 0},
	{0x548c, 0xb8, 0, 0},
	{0x548d, 0xcd, 0, 0},
	{0x548e, 0xdd, 0, 0},
	{0x548f, 0xea, 0, 0},
	{0x5490, 0x1d, 0, 0},
	{0x5580, 0x02, 0, 0},
	{0x5583, 0x40, 0, 0},
	{0x5584, 0x10, 0, 0},
	{0x5589, 0x10, 0, 0},
	{0x558a, 0x00, 0, 0},
	{0x558b, 0xf8, 0, 0},
	{0x5800, 0x23, 0, 0},
	{0x5801, 0x14, 0, 0},
	{0x5802, 0x0f, 0, 0},
	{0x5803, 0x0f, 0, 0},
	{0x5804, 0x12, 0, 0},
	{0x5805, 0x26, 0, 0},
	{0x5806, 0x0c, 0, 0},
	{0x5807, 0x08, 0, 0},
	{0x5808, 0x05, 0, 0},
	{0x5809, 0x05, 0, 0},
	{0x580a, 0x08, 0, 0},
	{0x580b, 0x0d, 0, 0},
	{0x580c, 0x08, 0, 0},
	{0x580d, 0x03, 0, 0},
	{0x580e, 0x00, 0, 0},
	{0x580f, 0x00, 0, 0},
	{0x5810, 0x03, 0, 0},
	{0x5811, 0x09, 0, 0},
	{0x5812, 0x07, 0, 0},
	{0x5813, 0x03, 0, 0},
	{0x5814, 0x00, 0, 0},
	{0x5815, 0x01, 0, 0},
	{0x5816, 0x03, 0, 0},
	{0x5817, 0x08, 0, 0},
	{0x5818, 0x0d, 0, 0},
	{0x5819, 0x08, 0, 0},
	{0x581a, 0x05, 0, 0},
	{0x581b, 0x06, 0, 0},
	{0x581c, 0x08, 0, 0},
	{0x581d, 0x0e, 0, 0},
	{0x581e, 0x29, 0, 0},
	{0x581f, 0x17, 0, 0},
	{0x5820, 0x11, 0, 0},
	{0x5821, 0x11, 0, 0},
	{0x5822, 0x15, 0, 0},
	{0x5823, 0x28, 0, 0},
	{0x5824, 0x46, 0, 0},
	{0x5825, 0x26, 0, 0},
	{0x5826, 0x08, 0, 0},
	{0x5827, 0x26, 0, 0},
	{0x5828, 0x64, 0, 0},
	{0x5829, 0x26, 0, 0},
	{0x582a, 0x24, 0, 0},
	{0x582b, 0x22, 0, 0},
	{0x582c, 0x24, 0, 0},
	{0x582d, 0x24, 0, 0},
	{0x582e, 0x06, 0, 0},
	{0x582f, 0x22, 0, 0},
	{0x5830, 0x40, 0, 0},
	{0x5831, 0x42, 0, 0},
	{0x5832, 0x24, 0, 0},
	{0x5833, 0x26, 0, 0},
	{0x5834, 0x24, 0, 0},
	{0x5835, 0x22, 0, 0},
	{0x5836, 0x22, 0, 0},
	{0x5837, 0x26, 0, 0},
	{0x5838, 0x44, 0, 0},
	{0x5839, 0x24, 0, 0},
	{0x583a, 0x26, 0, 0},
	{0x583b, 0x28, 0, 0},
	{0x583c, 0x42, 0, 0},
	{0x583d, 0xce, 0, 0},
	{0x5025, 0x00, 0, 0},
	{0x3a0f, 0x30, 0, 0},
	{0x3a10, 0x28, 0, 0},
	{0x3a1b, 0x30, 0, 0},
	{0x3a1e, 0x26, 0, 0},
	{0x3a11, 0x60, 0, 0},
	{0x3a1f, 0x14, 0, 0},
	{0x3008, 0x02, 0, 0}, // power up bit[6] = 0.
};

static const struct ov5640_mode_info
ov5640_mode_data[OV5640_NUM_MODES] = {
	{
		.id = OV5640_MODE_1080P_1920_1080,
		.dn_mode = SCALING,
		.hact = 1920,
		.htot = 2500,
		.vact = 1080,
		.vtot = 1120,
		.reg_data = ov5640_init_setting_30fps_1920x1080,
		.reg_data_size = ARRAY_SIZE(ov5640_init_setting_30fps_1920x1080),
		.max_fps = OV5640_30_FPS,
	},
#if 0
	{
		.id = OV5640_MODE_QSXGA_2592_1944,
		.dn_mode = SCALING,
		.hact = 2592,
		.htot = 2844,
		.vact = 1944,
		.vtot = 1968,
		.reg_data = ov5640_init_setting_15fps_2592_1944,
		.reg_data_size = ARRAY_SIZE(ov5640_init_setting_15fps_2592_1944),
		.max_fps = OV5640_15_FPS,
	},
#endif
};

static const s64 ov5640_link_freq[] = {
	672000000
};

static int ov5640_init_slave_id(struct ov5640_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	if (client->addr == OV5640_DEFAULT_SLAVE_ID)
		return 0;

	buf[0] = OV5640_REG_SLAVE_ID >> 8;
	buf[1] = OV5640_REG_SLAVE_ID & 0xff;
	buf[2] = client->addr << 1;

	msg.addr = OV5640_DEFAULT_SLAVE_ID;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed with %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int ov5640_write_reg(struct ov5640_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}

static int ov5640_read_reg(struct ov5640_dev *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x\n",
			__func__, reg);
		return ret;
	}

	*val = buf[0];
	return 0;
}

static int ov5640_read_reg16(struct ov5640_dev *sensor, u16 reg, u16 *val)
{
	u8 hi, lo;
	int ret;

	ret = ov5640_read_reg(sensor, reg, &hi);
	if (ret)
		return ret;
	ret = ov5640_read_reg(sensor, reg + 1, &lo);
	if (ret)
		return ret;

	*val = ((u16)hi << 8) | (u16)lo;
	return 0;
}

static int ov5640_write_reg16(struct ov5640_dev *sensor, u16 reg, u16 val)
{
	int ret;

	ret = ov5640_write_reg(sensor, reg, val >> 8);
	if (ret)
		return ret;

	return ov5640_write_reg(sensor, reg + 1, val & 0xff);
}

static int ov5640_mod_reg(struct ov5640_dev *sensor, u16 reg,
			  u8 mask, u8 val)
{
	u8 readval;
	int ret;

	ret = ov5640_read_reg(sensor, reg, &readval);
	if (ret)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov5640_write_reg(sensor, reg, val);
}



/* download ov5640 settings to sensor through i2c */
static int ov5640_set_timings(struct ov5640_dev *sensor,
			      const struct ov5640_mode_info *mode)
{
	int ret;

	ret = ov5640_write_reg16(sensor, OV5640_REG_TIMING_DVPHO, mode->hact);
	if (ret < 0)
		return ret;

	ret = ov5640_write_reg16(sensor, OV5640_REG_TIMING_DVPVO, mode->vact);
	if (ret < 0)
		return ret;

	ret = ov5640_write_reg16(sensor, OV5640_REG_TIMING_HTS, mode->htot);
	if (ret < 0)
		return ret;

	return ov5640_write_reg16(sensor, OV5640_REG_TIMING_VTS, mode->vtot);
}

static int ov5640_load_regs(struct ov5640_dev *sensor,
			    const struct ov5640_mode_info *mode)
{
	const struct reg_value *regs = mode->reg_data;
	unsigned int i;
	u32 delay_ms;
	u16 reg_addr;
	u8 mask, val;
	int ret = 0;

	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		delay_ms = regs->delay_ms;
		reg_addr = regs->reg_addr;
		val = regs->val;
		mask = regs->mask;

		/* remain in power down mode for DVP */
		if (regs->reg_addr == OV5640_REG_SYS_CTRL0 &&
		    val == OV5640_REG_SYS_CTRL0_SW_PWUP &&
		    sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY)
			continue;

		if (mask)
			ret = ov5640_mod_reg(sensor, reg_addr, mask, val);
		else
			ret = ov5640_write_reg(sensor, reg_addr, val);
		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
	}

	return ov5640_set_timings(sensor, mode);
}

static int ov5640_set_autoexposure(struct ov5640_dev *sensor, bool on)
{
	return ov5640_mod_reg(sensor, OV5640_REG_AEC_PK_MANUAL,
			      BIT(0), on ? 0 : BIT(0));
}

/* read exposure, in number of line periods */
static int ov5640_get_exposure(struct ov5640_dev *sensor)
{
	int exp, ret;
	u8 temp;

	ret = ov5640_read_reg(sensor, OV5640_REG_AEC_PK_EXPOSURE_HI, &temp);
	if (ret)
		return ret;
	exp = ((int)temp & 0x0f) << 16;
	ret = ov5640_read_reg(sensor, OV5640_REG_AEC_PK_EXPOSURE_MED, &temp);
	if (ret)
		return ret;
	exp |= ((int)temp << 8);
	ret = ov5640_read_reg(sensor, OV5640_REG_AEC_PK_EXPOSURE_LO, &temp);
	if (ret)
		return ret;
	exp |= (int)temp;

	return exp >> 4;
}

/* write exposure, given number of line periods */
static int ov5640_set_exposure(struct ov5640_dev *sensor, u32 exposure)
{
	int ret;

	exposure <<= 4;

	ret = ov5640_write_reg(sensor,
			       OV5640_REG_AEC_PK_EXPOSURE_LO,
			       exposure & 0xff);
	if (ret)
		return ret;
	ret = ov5640_write_reg(sensor,
			       OV5640_REG_AEC_PK_EXPOSURE_MED,
			       (exposure >> 8) & 0xff);
	if (ret)
		return ret;
	return ov5640_write_reg(sensor,
				OV5640_REG_AEC_PK_EXPOSURE_HI,
				(exposure >> 16) & 0x0f);
}

static int ov5640_get_gain(struct ov5640_dev *sensor)
{
	u16 gain;
	int ret;

	ret = ov5640_read_reg16(sensor, OV5640_REG_AEC_PK_REAL_GAIN, &gain);
	if (ret)
		return ret;

	return gain & 0x3ff;
}

static int ov5640_set_gain(struct ov5640_dev *sensor, int gain)
{
	return ov5640_write_reg16(sensor, OV5640_REG_AEC_PK_REAL_GAIN,
				  (u16)gain & 0x3ff);
}

static int ov5640_set_autogain(struct ov5640_dev *sensor, bool on)
{
	return ov5640_mod_reg(sensor, OV5640_REG_AEC_PK_MANUAL,
			      BIT(1), on ? 0 : BIT(1));
}


static int ov5640_get_vts(struct ov5640_dev *sensor)
{
	u16 vts;
	int ret;

	ret = ov5640_read_reg16(sensor, OV5640_REG_TIMING_VTS, &vts);
	if (ret)
		return ret;
	return vts;
}

static const struct ov5640_mode_info *
ov5640_find_mode(struct ov5640_dev *sensor, enum ov5640_frame_rate fr,
		 int width, int height, bool nearest)
{
	const struct ov5640_mode_info *mode;
	pr_info("%s, w %d ,h %d ,fr %d ++", __func__, width, height,ov5640_framerates[fr]);
	mode = v4l2_find_nearest_size(ov5640_mode_data,
				      ARRAY_SIZE(ov5640_mode_data),
				      hact, vact,
				      width, height);

	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height))) {
		pr_err("%s find with w&h fail", __func__);
		return NULL;
	}

	/* Check to see if the current mode exceeds the max frame rate */
	if (ov5640_framerates[fr] > ov5640_framerates[mode->max_fps]) {
		return NULL;
		pr_err("%s find with fr fail", __func__);
	}

	pr_info("%s success --", __func__);

	return mode;
}

static u64 ov5640_calc_pixel_rate(struct ov5640_dev *sensor)
{
	u64 rate;

	rate = sensor->current_mode->vtot * sensor->current_mode->htot;
	rate *= ov5640_framerates[sensor->current_fr];

	return rate;
}


static int ov5640_set_mode(struct ov5640_dev *sensor)
{
	// all thing is done in restore mode.
	int ret = 0;
	pr_info("%s ++", __func__);
	/* first load the initial register values */
	ret = ov5640_load_regs(sensor, sensor->current_mode );
	if (ret < 0)
		return ret;
	sensor->last_mode = sensor->current_mode;

	return ret;
}


static void ov5640_power(struct ov5640_dev *sensor, bool enable)
{
	int ret = -1;

	if (!gpio_is_valid(sensor->pwdn_gpio)) {
		pr_err("invalid pwdn gpio");
		return;
	}
	devm_gpio_request(&sensor->i2c_client->dev, sensor->pwdn_gpio, "PWDN");
	ret = gpio_direction_output(sensor->pwdn_gpio, enable ? 0 : 1);
	if (0 != ret ) {
		pr_err("set pwdn to 0 fail. ret %d ", ret);
	}
}

static void ov5640_reset(struct ov5640_dev *sensor)
{
	int ret = -1;

	if (!gpio_is_valid(sensor->reset_gpio)) {
		pr_err("invalid reset gpio");
		return;
	}
	devm_gpio_request(&sensor->i2c_client->dev, sensor->reset_gpio, "RESET");

	ret = gpio_direction_output(sensor->reset_gpio, 1);
	if (0 != ret ) {
		pr_err("set reset to 1 fail. ret %d ", ret);
	}

	usleep_range(130000, 150000);
}

static int ov5640_set_power_on(struct ov5640_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = -1;

	if (!gpio_is_valid(sensor->reset_gpio)) {
		pr_err("invalid reset gpio");
		return -1;
	}
	devm_gpio_request(&sensor->i2c_client->dev, sensor->reset_gpio, "RESET");

	ret = gpio_direction_output(sensor->reset_gpio, 0);
	if (0 != ret ) {
		pr_err("set reset to 1 fail. ret %d ", ret);
	}
	usleep_range(10000, 15000);

	ov5640_power(sensor, true);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}
	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	pr_err("pwr on - mclk is %d MHZ\n", sensor->xclk_freq/1000000);

	usleep_range(10000, 15000);

	ov5640_reset(sensor);

	ret = ov5640_init_slave_id(sensor);
	if (ret)
		goto power_off;

	return 0;

power_off:
	ov5640_power(sensor, false);
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void ov5640_set_power_off(struct ov5640_dev *sensor)
{
	ov5640_power(sensor, false);
	clk_disable_unprepare(sensor->xclk);
}



static int ov5640_set_power(struct ov5640_dev *sensor, bool on)
{
	int ret = 0;
	pr_info("%s++, on = %d \n", __func__, on);
	if (on) {
		ret = ov5640_set_power_on(sensor);
		if (ret)
			return ret;

		ret = ov5640_set_mode(sensor);
		if (ret)
			goto power_off;
	}

	if (sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY)
		pr_err("not supported");

	if (ret)
		goto power_off;

	if (!on)
		ov5640_set_power_off(sensor);

	return 0;

power_off:
	ov5640_set_power_off(sensor);
	return ret;
}

/* --------------- Subdev Operations --------------- */

static int ov5640_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		ret = ov5640_set_power(sensor, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);

	if (on && !ret && sensor->power_count == 1) {
		/* restore controls */
		ret = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	}

	return ret;
}

static int ov5640_try_frame_interval(struct ov5640_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct ov5640_mode_info *mode;
	enum ov5640_frame_rate rate = OV5640_15_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;

	minfps = ov5640_framerates[OV5640_15_FPS];
	maxfps = ov5640_framerates[OV5640_60_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = OV5640_60_FPS;
		goto find_mode;
	}

	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	for (i = 0; i < ARRAY_SIZE(ov5640_framerates); i++) {
		int curr_fps = ov5640_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}

	fi->numerator = 1;
	fi->denominator = best_fps;

find_mode:
	mode = ov5640_find_mode(sensor, rate, width, height, false);
	return mode ? rate : -EINVAL;
}

static int ov5640_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int ov5640_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum ov5640_frame_rate fr,
				   const struct ov5640_mode_info **new_mode)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	const struct ov5640_mode_info *mode;
	int i;

	pr_info("%s++ \n", __func__);

	mode = ov5640_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->hact;
	fmt->height = mode->vact;

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(ov5640_formats); i++)
		if (ov5640_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(ov5640_formats))
		i = 0;

	fmt->code = ov5640_formats[i].code;
	fmt->colorspace = ov5640_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int ov5640_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	const struct ov5640_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	pr_info("%s++, code %d w %d h %d \n", __func__, mbus_fmt->code, mbus_fmt->width, mbus_fmt->height);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = ov5640_try_fmt_internal(sd, mbus_fmt,
				      sensor->current_fr, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	else
		fmt = &sensor->fmt;


	if (new_mode != sensor->current_mode) {
		sensor->current_mode = new_mode;
		sensor->pending_mode_change = true;
	}

	if (mbus_fmt->code != sensor->fmt.code) {
		sensor->pending_fmt_change = true;
	}

	/* update format even if code is unchanged, resolution might change */
	sensor->fmt = *mbus_fmt;

	if (sensor->ctrls.link_freq)
		__v4l2_ctrl_s_ctrl(sensor->ctrls.link_freq, 0);
	if (sensor->ctrls.pixel_rate)
		__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate,
				 ov5640_calc_pixel_rate(sensor));
out:
	mutex_unlock(&sensor->lock);
	return ret;
}
static int ov5640_set_framefmt(struct ov5640_dev *sensor,
			       struct v4l2_mbus_framefmt *format)
{
	int ret = 0;

	u8 fmt, mux;

	switch (format->code) {
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		/* YUV422, VYUY */
		fmt = 0x30;
		mux = OV5640_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
		/* YUV422, UYVY */
		fmt = 0x31;
		mux = OV5640_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		/* YUV422, YVYU */
		fmt = 0x32;
		mux = OV5640_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		/* YUV422, YUYV */
		fmt = 0x33;
		mux = OV5640_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		/* RGB565 {g[2:0],b[4:0]},{r[4:0],g[5:3]} */
		fmt = 0x6F;
		mux = OV5640_FMT_MUX_RGB;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
		/* RGB565 {r[4:0],g[5:3]},{g[2:0],b[4:0]} */
		fmt = 0x61;
		mux = OV5640_FMT_MUX_RGB;
		break;
	default:
		return -EINVAL;
	}

	/* FORMAT CONTROL00: YUV and RGB formatting */
	pr_info("set framefmt 0x%x", fmt);
	ret = ov5640_write_reg(sensor, OV5640_REG_FORMAT_CONTROL00, fmt);
	if (ret)
		return ret;

	/* FORMAT MUX CONTROL: ISP YUV or RGB */
	ret = ov5640_write_reg(sensor, OV5640_REG_ISP_FORMAT_MUX_CTRL, mux);
	return ret;
}


/*
 * Sensor Controls.
 */

static int ov5640_set_ctrl_hue(struct ov5640_dev *sensor, int value)
{
	int ret;

	if (value) {
		ret = ov5640_mod_reg(sensor, OV5640_REG_SDE_CTRL0,
				     BIT(0), BIT(0));
		if (ret)
			return ret;
		ret = ov5640_write_reg16(sensor, OV5640_REG_SDE_CTRL1, value);
	} else {
		ret = ov5640_mod_reg(sensor, OV5640_REG_SDE_CTRL0, BIT(0), 0);
	}

	return ret;
}

static int ov5640_set_ctrl_contrast(struct ov5640_dev *sensor, int value)
{
	int ret;

	if (value) {
		ret = ov5640_mod_reg(sensor, OV5640_REG_SDE_CTRL0,
				     BIT(2), BIT(2));
		if (ret)
			return ret;
		ret = ov5640_write_reg(sensor, OV5640_REG_SDE_CTRL5,
				       value & 0xff);
	} else {
		ret = ov5640_mod_reg(sensor, OV5640_REG_SDE_CTRL0, BIT(2), 0);
	}

	return ret;
}

static int ov5640_set_ctrl_saturation(struct ov5640_dev *sensor, int value)
{
	int ret;

	if (value) {
		ret = ov5640_mod_reg(sensor, OV5640_REG_SDE_CTRL0,
				     BIT(1), BIT(1));
		if (ret)
			return ret;
		ret = ov5640_write_reg(sensor, OV5640_REG_SDE_CTRL3,
				       value & 0xff);
		if (ret)
			return ret;
		ret = ov5640_write_reg(sensor, OV5640_REG_SDE_CTRL4,
				       value & 0xff);
	} else {
		ret = ov5640_mod_reg(sensor, OV5640_REG_SDE_CTRL0, BIT(1), 0);
	}

	return ret;
}

static int ov5640_set_ctrl_white_balance(struct ov5640_dev *sensor, int awb)
{
	int ret;

	ret = ov5640_mod_reg(sensor, OV5640_REG_AWB_MANUAL_CTRL,
			     BIT(0), awb ? 0 : 1);
	if (ret)
		return ret;

	if (!awb) {
		u16 red = (u16)sensor->ctrls.red_balance->val;
		u16 blue = (u16)sensor->ctrls.blue_balance->val;

		ret = ov5640_write_reg16(sensor, OV5640_REG_AWB_R_GAIN, red);
		if (ret)
			return ret;
		ret = ov5640_write_reg16(sensor, OV5640_REG_AWB_B_GAIN, blue);
	}

	return ret;
}

static int ov5640_set_ctrl_exposure(struct ov5640_dev *sensor,
				    enum v4l2_exposure_auto_type auto_exposure)
{
	struct ov5640_ctrls *ctrls = &sensor->ctrls;
	bool auto_exp = (auto_exposure == V4L2_EXPOSURE_AUTO);
	int ret = 0;

	if (ctrls->auto_exp->is_new) {
		ret = ov5640_set_autoexposure(sensor, auto_exp);
		if (ret)
			return ret;
	}

	if (!auto_exp && ctrls->exposure->is_new) {
		u16 max_exp;

		ret = ov5640_read_reg16(sensor, OV5640_REG_AEC_PK_VTS,
					&max_exp);
		if (ret)
			return ret;
		ret = ov5640_get_vts(sensor);
		if (ret < 0)
			return ret;
		max_exp += ret;
		ret = 0;

		if (ctrls->exposure->val < max_exp)
			ret = ov5640_set_exposure(sensor, ctrls->exposure->val);
	}

	return ret;
}

static int ov5640_set_ctrl_gain(struct ov5640_dev *sensor, bool auto_gain)
{
	struct ov5640_ctrls *ctrls = &sensor->ctrls;
	int ret = 0;

	if (ctrls->auto_gain->is_new) {
		ret = ov5640_set_autogain(sensor, auto_gain);
		if (ret)
			return ret;
	}

	if (!auto_gain && ctrls->gain->is_new)
		ret = ov5640_set_gain(sensor, ctrls->gain->val);

	return ret;
}

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Color bars",
	"Color bars w/ rolling bar",
	"Color squares",
	"Color squares w/ rolling bar",
};

#define OV5640_TEST_ENABLE              BIT(7)
#define OV5640_TEST_ROLLING             BIT(6)  /* rolling horizontal bar */
#define OV5640_TEST_TRANSPARENT         BIT(5)
#define OV5640_TEST_SQUARE_BW           BIT(4)  /* black & white squares */
#define OV5640_TEST_BAR_STANDARD        (0 << 2)
#define OV5640_TEST_BAR_VERT_CHANGE_1   (1 << 2)
#define OV5640_TEST_BAR_HOR_CHANGE      (2 << 2)
#define OV5640_TEST_BAR_VERT_CHANGE_2   (3 << 2)
#define OV5640_TEST_BAR                 (0 << 0)
#define OV5640_TEST_RANDOM              (1 << 0)
#define OV5640_TEST_SQUARE              (2 << 0)
#define OV5640_TEST_BLACK               (3 << 0)

static const u8 test_pattern_val[] = {
	0,
	OV5640_TEST_ENABLE | OV5640_TEST_BAR_VERT_CHANGE_1 |
		OV5640_TEST_BAR,
	OV5640_TEST_ENABLE | OV5640_TEST_ROLLING |
		OV5640_TEST_BAR_VERT_CHANGE_1 | OV5640_TEST_BAR,
	OV5640_TEST_ENABLE | OV5640_TEST_SQUARE,
	OV5640_TEST_ENABLE | OV5640_TEST_ROLLING | OV5640_TEST_SQUARE,
};

static int ov5640_set_ctrl_test_pattern(struct ov5640_dev *sensor, int value)
{
	return ov5640_write_reg(sensor, OV5640_REG_PRE_ISP_TEST_SET1,
				test_pattern_val[value]);
}

static int ov5640_set_ctrl_light_freq(struct ov5640_dev *sensor, int value)
{
	int ret;

	ret = ov5640_mod_reg(sensor, OV5640_REG_HZ5060_CTRL01, BIT(7),
			     (value == V4L2_CID_POWER_LINE_FREQUENCY_AUTO) ?
			     0 : BIT(7));
	if (ret)
		return ret;

	return ov5640_mod_reg(sensor, OV5640_REG_HZ5060_CTRL00, BIT(2),
			      (value == V4L2_CID_POWER_LINE_FREQUENCY_50HZ) ?
			      BIT(2) : 0);
}

static int ov5640_set_ctrl_hflip(struct ov5640_dev *sensor, int value)
{
	/*
	 * If sensor is mounted upside down, mirror logic is inversed.
	 *
	 * Sensor is a BSI (Back Side Illuminated) one,
	 * so image captured is physically mirrored.
	 * This is why mirror logic is inversed in
	 * order to cancel this mirror effect.
	 */

	/*
	 * TIMING TC REG21:
	 * - [2]:	ISP mirror
	 * - [1]:	Sensor mirror
	 */
	return ov5640_mod_reg(sensor, OV5640_REG_TIMING_TC_REG21,
			      BIT(2) | BIT(1),
			      (!(value ^ sensor->upside_down)) ?
			      (BIT(2) | BIT(1)) : 0);
}

static int ov5640_set_ctrl_vflip(struct ov5640_dev *sensor, int value)
{
	/* If sensor is mounted upside down, flip logic is inversed */

	/*
	 * TIMING TC REG20:
	 * - [2]:	ISP vflip
	 * - [1]:	Sensor vflip
	 */
	return ov5640_mod_reg(sensor, OV5640_REG_TIMING_TC_REG20,
			      BIT(2) | BIT(1),
			      (value ^ sensor->upside_down) ?
			      (BIT(2) | BIT(1)) : 0);
}

static int ov5640_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	int val;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		val = ov5640_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		val = ov5640_get_exposure(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.exposure->val = val;
		break;
	}

	return 0;
}

static int ov5640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (sensor->power_count == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = ov5640_set_ctrl_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov5640_set_ctrl_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov5640_set_ctrl_white_balance(sensor, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = ov5640_set_ctrl_hue(sensor, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = ov5640_set_ctrl_contrast(sensor, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = ov5640_set_ctrl_saturation(sensor, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov5640_set_ctrl_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = ov5640_set_ctrl_light_freq(sensor, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov5640_set_ctrl_hflip(sensor, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = ov5640_set_ctrl_vflip(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov5640_ctrl_ops = {
	.g_volatile_ctrl = ov5640_g_volatile_ctrl,
	.s_ctrl = ov5640_s_ctrl,
};

static int ov5640_init_controls(struct ov5640_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ov5640_ctrl_ops;
	struct ov5640_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	v4l2_ctrl_handler_init(hdl, 33);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Clock related controls */
	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					      0, INT_MAX, 1,
					      ov5640_calc_pixel_rate(sensor));

	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl,
							   ops,
							   V4L2_CID_LINK_FREQ,
							   ARRAY_SIZE(ov5640_link_freq) - 1,
							   0, ov5640_link_freq);
	if (ctrls->link_freq)
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Auto/manual white balance */
	ctrls->auto_wb = v4l2_ctrl_new_std(hdl, ops,
					   V4L2_CID_AUTO_WHITE_BALANCE,
					   0, 1, 1, 1);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4095, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4095, 1, 0);
	/* Auto/manual exposure */
	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
						 V4L2_CID_EXPOSURE_AUTO,
						 V4L2_EXPOSURE_MANUAL, 0,
						 V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN,
					     0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 1023, 1, 0);

	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 255, 1, 64);
	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE,
				       0, 359, 1, 0);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 255, 1, 0);
	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					 0, 1, 1, 0);

	ctrls->light_freq =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_POWER_LINE_FREQUENCY,
				       V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
				       V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, false);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, true);

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int ov5640_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= OV5640_NUM_MODES)
		return -EINVAL;

	fse->min_width =
		ov5640_mode_data[fse->index].hact;
	fse->max_width = fse->min_width;
	fse->min_height =
		ov5640_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov5640_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	struct v4l2_fract tpf;
	int ret;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= OV5640_NUM_FRAMERATES)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = ov5640_framerates[fie->index];

	ret = ov5640_try_frame_interval(sensor, &tpf,
					fie->width, fie->height);
	if (ret < 0)
		return -EINVAL;

	fie->interval = tpf;
	return 0;
}

static int ov5640_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int ov5640_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	const struct ov5640_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = ov5640_try_frame_interval(sensor, &fi->interval,
					       mode->hact, mode->vact);
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}

	mode = ov5640_find_mode(sensor, frame_rate, mode->hact,
				mode->vact, true);
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}

	if (mode != sensor->current_mode ||
	    frame_rate != sensor->current_fr) {
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
		sensor->pending_mode_change = true;

		__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate,
					 ov5640_calc_pixel_rate(sensor));
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ov5640_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(ov5640_formats))
		return -EINVAL;

	code->code = ov5640_formats[code->index].code;
	return 0;
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5640_dev *sensor = to_ov5640_dev(sd);
	int ret = 0;

	pr_info("%s ++", __func__);

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !enable) {
		if (enable && sensor->pending_mode_change) {
			ret = ov5640_set_mode(sensor);
			if (ret)
				goto out;
		}

		pr_info("%s bef set framefmt", __func__);

		if (enable && sensor->pending_fmt_change) {
			ret = ov5640_set_framefmt(sensor, &sensor->fmt);
			if (ret)
				goto out;
			sensor->pending_fmt_change = false;
		}

		if (sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
			pr_info("%s only support stream mipi", __func__);
		}

		if (!ret)
			sensor->streaming = enable;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops ov5640_core_ops = {
	.s_power = ov5640_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ov5640_video_ops = {
	.g_frame_interval = ov5640_g_frame_interval,
	.s_frame_interval = ov5640_s_frame_interval,
	.s_stream = ov5640_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5640_pad_ops = {
	.enum_mbus_code = ov5640_enum_mbus_code,
	.get_fmt = ov5640_get_fmt,
	.set_fmt = ov5640_set_fmt,
	.enum_frame_size = ov5640_enum_frame_size,
	.enum_frame_interval = ov5640_enum_frame_interval,
};

static const struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core = &ov5640_core_ops,
	.video = &ov5640_video_ops,
	.pad = &ov5640_pad_ops,
};


static int ov5640_check_chip_id(struct ov5640_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	u16 chip_id;

	ret = ov5640_set_power_on(sensor);
	if (ret)
		return ret;

	ret = ov5640_read_reg16(sensor, OV5640_REG_CHIP_ID, &chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}

	if (chip_id != 0x5640) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x5640, got 0x%x\n",
			__func__, chip_id);
		ret = -ENXIO;
	} else {
		dev_err(&client->dev, "%s: check id success got 0x%x\n",
			__func__, chip_id);
	}

power_off:
	ov5640_set_power_off(sensor);
	return ret;
}

static int ov5640_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ov5640_dev *sensor;
	u32 xclk_freq_dts = 0;
		struct v4l2_subdev_format subdevfmt = { 0 };
	u32 rotation;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;


	if (of_property_read_u32(dev->of_node, "index", &sensor->index)) {
		dev_err(dev, "Failed to read sensor index. default to 0\n");
		sensor->index = 0;
	}

	/* optional indication of physical rotation of sensor */
	ret = fwnode_property_read_u32(dev_fwnode(&client->dev), "rotation",
				       &rotation);
	if (!ret) {
		switch (rotation) {
		case 180:
			sensor->upside_down = true;
			fallthrough;
		case 0:
			break;
		default:
			dev_warn(dev, "%u degrees rotation is not supported, ignoring...\n",
				 rotation);
		}
	}

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	if (sensor->ep.bus_type != V4L2_MBUS_PARALLEL &&
	    sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY &&
	    sensor->ep.bus_type != V4L2_MBUS_BT656) {
		dev_err(dev, "Unsupported bus type %d\n", sensor->ep.bus_type);
		return -EINVAL;
	}

	if ( 0 == sensor->index ) {
		/* get system clock (xclk) */
		sensor->xclk = devm_clk_get(dev, "mclk_0");
		if (IS_ERR_OR_NULL(sensor->xclk)) {
			dev_err(dev, "failed to get mclk\n");
			return PTR_ERR(sensor->xclk);
		}

		sensor->clk_mclk_x_pre = devm_clk_get(dev, "mclk_0_pre");
		if (IS_ERR_OR_NULL(sensor->clk_mclk_x_pre)) {
			dev_err(dev, "failed to get mclk_0_pre\n");
			return PTR_ERR(sensor->clk_mclk_x_pre);
		}
	} else if (1 == sensor->index) {
		/* get system clock (xclk) */
		sensor->xclk = devm_clk_get(dev, "mclk_1");
		if (IS_ERR_OR_NULL(sensor->xclk)) {
			dev_err(dev, "failed to get mclk\n");
			return PTR_ERR(sensor->xclk);
		}

		sensor->clk_mclk_x_pre = devm_clk_get(dev, "mclk_1_pre");
		if (IS_ERR_OR_NULL(sensor->clk_mclk_x_pre)) {
			dev_err(dev, "failed to get mclk_1_pre\n");
			return PTR_ERR(sensor->clk_mclk_x_pre);
		}
	} else {
		dev_err(dev, "bad sensor index.\n");
		return -1;
	}

	sensor->clk_xtal = devm_clk_get(dev, "xtal");
	if (IS_ERR_OR_NULL(sensor->clk_xtal)) {
		dev_err(dev, "failed to get xtal\n");
		return PTR_ERR(sensor->clk_xtal);
	}

	clk_set_parent(sensor->xclk, sensor->clk_xtal);


	ret = fwnode_property_read_u32(dev_fwnode(&client->dev), "clock-frequency",
				       &xclk_freq_dts);
	if (ret) {
		dev_err(&client->dev, "Could not get xclk frequency\n");
		//return -EINVAL; // not return. get and check.
	} else {
		dev_info(&client->dev, "07291547 get dts clk frequency %d \n", xclk_freq_dts);
	}


	ret = clk_set_rate(sensor->xclk, xclk_freq_dts);
	if (ret) {
		dev_err(&client->dev, "Could not set xclk frequency\n");
	}

	udelay(100);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret < 0)
		dev_err(&client->dev, "clk_prepare_enable failed\n");

	clk_disable_unprepare(sensor->clk_mclk_x_pre);

	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	dev_err(&client->dev, "init mclk is %d MHZ\n", sensor->xclk_freq/1000000);

	if (sensor->xclk_freq < OV5640_XCLK_MIN ||
	    sensor->xclk_freq > OV5640_XCLK_MAX) {
		dev_err(dev, "xclk frequency out of range: %d Hz\n",
			sensor->xclk_freq);
		return -EINVAL;
	}

	/* request optional power down pin */
	sensor->pwdn_gpio = of_get_named_gpio(dev->of_node, "pwdn", 0);
	if (!gpio_is_valid(sensor->pwdn_gpio)) {
		dev_err(&client->dev, "Cannot get pwdn gpio");
	}

	/* request optional reset pin */
	sensor->reset_gpio = of_get_named_gpio(dev->of_node, "reset", 0);
	if (!gpio_is_valid(sensor->reset_gpio)) {
		dev_err(&client->dev, "Cannot get reset gpio");
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &ov5640_subdev_ops);

	snprintf(sensor->sd.name, sizeof(sensor->sd.name), AML_SENSOR_NAME, sensor->index);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = ov5640_check_chip_id(sensor);
	if (ret)
		goto entity_cleanup;



	// init controls depends on a valid sensor->current_fr; & sensor->current_mode
	// s power on depend on a valid sensor->current_mode;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = ov5640_framerates[OV5640_30_FPS];
	sensor->current_fr = OV5640_30_FPS;
	sensor->current_mode =
		&ov5640_mode_data[OV5640_MODE_1080P_1920_1080];
	sensor->last_mode = sensor->current_mode;


	ret = ov5640_init_controls(sensor);
	if (ret)
		goto entity_cleanup;


	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;


	// power on depend on a valid sensor->current_mode;
	// load current mode setting
	pr_err("pwr on after probe\n");
	ov5640_s_power(&sensor->sd,1);

	// call set_fmt to init other fmt related info.
	subdevfmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	subdevfmt.format.width = 1920;
	subdevfmt.format.height = 1080;
	subdevfmt.format.code = MEDIA_BUS_FMT_YUYV8_2X8;

	ov5640_set_fmt(&sensor->sd, NULL, &subdevfmt);

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640_dev *sensor = to_ov5640_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);

	ov5640_s_power(&sensor->sd, 0);

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{"ov5640", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

static const struct of_device_id ov5640_dt_ids[] = {
	{ .compatible = "ov, ov5640" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov5640_dt_ids);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name  = "ov5640",
		.of_match_table	= ov5640_dt_ids,
	},
	.id_table = ov5640_id,
	.probe_new = ov5640_probe,
	.remove   = ov5640_remove,
};

module_i2c_driver(ov5640_i2c_driver);

MODULE_DESCRIPTION("OV5640 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");

