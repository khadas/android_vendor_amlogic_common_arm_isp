// SPDX-License-Identifier: GPL-2.0
/*
 * Sony OV9285 CMOS Image Sensor Driver
 *
 * Copyright (C) 2019 FRAMOS GmbH.
 *
 * Copyright (C) 2019 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */
#include <linux/version.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV9285_STANDBY 0x0100
#define OV9285_ID 0x9286
#define OV9285_MCLK 24000000

#define OV9285_DEFAULT_LINK_FREQ 400000000

struct ov9285_regval {
	u16 reg;
	u8 val;
};

struct ov9285_mode {
	u32 width;
	u32 height;
	u32 pixel_rate;
	u32 link_freq_index;
	u32 hblank;

	const struct ov9285_regval *data;
	u32 data_size;
};

struct ov9285 {
	struct device *dev;
	struct clk *xclk;
	struct regmap *regmap;

	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct v4l2_fwnode_endpoint ep;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct ov9285_mode *current_mode;

	struct gpio_desc *pwdn_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;

	int status;
	struct mutex lock;
};

struct ov9285_pixfmt {
	u32 code;
	u32 min_width;
	u32 max_width;
	u32 min_height;
	u32 max_height;
};

static const struct ov9285_pixfmt ov9285_formats[] = {
	{MEDIA_BUS_FMT_SRGGB10_1X10, 640, 640, 400, 400},
};

static const struct regmap_config ov9285_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static const struct ov9285_regval ov9285_480p_settings[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x0300, 0x06},
	{0x0301, 0x01},
	{0x0302, 0x90},
	{0x0303, 0x01},
	{0x030a, 0x01},
	{0x030d, 0x50},
	{0x030f, 0x03},
	{0x0312, 0x07},
	{0x0313, 0x01},
	{0x3004, 0x00},
	{0x3006, 0x06},
	{0x300f, 0xf4},
	{0x3011, 0x0a},
	{0x3013, 0x18},
	{0x302c, 0x00},
	{0x302d, 0x00},
	{0x302e, 0x00},
	{0x302f, 0x05},
	{0x3030, 0x10},
	{0x303f, 0x03},
	{0x31ff, 0x01},
	{0x3210, 0x04},
	{0x3500, 0x00},
	{0x3501, 0xde},
	{0x3502, 0x70},
	{0x3503, 0x08},
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3509, 0x10},
	{0x3610, 0x80},
	{0x3611, 0xb8},
	{0x3612, 0x02},
	{0x3614, 0x80},
	{0x3620, 0x6e},
	{0x3632, 0x56},
	{0x3633, 0x78},
	{0x3662, 0x15},
	{0x3666, 0x70},
	{0x3670, 0x68},
	{0x367e, 0x90},
	{0x3680, 0x84},
	{0x3707, 0x6c},
	{0x3712, 0x80},
	{0x372d, 0x22},
	{0x3731, 0x90},
	{0x3732, 0x30},
	{0x3778, 0x10},
	{0x377d, 0x22},
	{0x3788, 0x02},
	{0x3789, 0xa4},
	{0x378a, 0x00},
	{0x378b, 0x44},
	{0x3799, 0x20},
	{0x379b, 0x01},
	{0x379c, 0x10},
	{0x37a8, 0x42},
	{0x37aa, 0x52},
	{0x37ab, 0x3c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x70},
	{0x3804, 0x05},
	{0x3805, 0x3f},
	{0x3806, 0x03},
	{0x3807, 0xff},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0x90},
	{0x380c, 0x02},
	{0x380d, 0xe8},
	{0x380e, 0x0e},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x22},
	{0x3815, 0x22},
	{0x3816, 0x00},
	{0x3817, 0x01},
	{0x3818, 0x00},
	{0x3819, 0x05},
	{0x3820, 0x61},
	{0x3821, 0x01},
	{0x382b, 0x3a},
	{0x382c, 0x09},
	{0x382d, 0x9a},
	{0x3880, 0x10},
	{0x3881, 0x42},
	{0x3882, 0x01},
	{0x3883, 0xcc},
	{0x3885, 0x07},
	{0x389d, 0x03},
	{0x38a6, 0x00},
	{0x38a7, 0x01},
	{0x38a8, 0x00},
	{0x38a9, 0xf0},
	{0x38b1, 0x00},
	{0x38b3, 0x07},
	{0x38c4, 0x01},
	{0x38c5, 0x18},
	{0x38c6, 0x02},
	{0x38c7, 0xa8},
	{0x38e5, 0x02},
	{0x38e7, 0x00},
	{0x38e8, 0x00},
	{0x38ed, 0x00},
	{0x38ee, 0x00},
	{0x38ef, 0x00},
	{0x3920, 0xa5},
	{0x3921, 0x00},
	{0x3922, 0x00},
	{0x3923, 0x00},
	{0x3924, 0x05},
	{0x3925, 0x00},
	{0x3926, 0x00},
	{0x3927, 0x00},
	{0x3928, 0x1a},
	{0x3929, 0x01},
	{0x392a, 0xb4},
	{0x392b, 0x00},
	{0x392c, 0x10},
	{0x392f, 0x40},
	{0x393e, 0x00},
	{0x393f, 0x00},
	{0x4001, 0x00},
	{0x4003, 0x10},
	{0x4008, 0x02},
	{0x4009, 0x05},
	{0x400a, 0x01},
	{0x400b, 0x70},
	{0x400c, 0x02},
	{0x400d, 0x05},
	{0x4010, 0xf0},
	{0x4016, 0x00},
	{0x4012, 0x08},
	{0x4017, 0x10},
	{0x4042, 0xd1},
	{0x4043, 0x60},
	{0x4045, 0x20},
	{0x404b, 0x20},
	{0x4507, 0xd0},
	{0x450b, 0xb0},
	{0x450f, 0x00},
	{0x4800, 0x60},
	{0x481f, 0x30},
	{0x4825, 0x35},
	{0x4837, 0x28},
	{0x4f00, 0x00},
	{0x4f07, 0x00},
	{0x4f08, 0x03},
	{0x4f09, 0x08},
	{0x4f0c, 0x02},
	{0x4f0d, 0x64},
	{0x4f10, 0x00},
	{0x4f11, 0x00},
	{0x4f12, 0x0f},
	{0x4f13, 0xc4},
	{0x4f07, 0x00},
	{0x5000, 0x9f},
	{0x5e00, 0x00},
};

/* supported link frequencies */
static const s64 ov9285_link_freq[] = {
	OV9285_DEFAULT_LINK_FREQ,
};

/* Mode configs */
static const struct ov9285_mode ov9285_modes[] = {
	{
		.width = 640,
		.height = 400,
		.data = ov9285_480p_settings,
		.data_size = ARRAY_SIZE(ov9285_480p_settings),
		.pixel_rate = 1955555,
		.hblank = 280,
		.link_freq_index = 0,
	},
};

static inline struct ov9285 *to_ov9285(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ov9285, sd);
}

static inline int ov9285_read_reg(struct ov9285 *ov9285, u16 addr, u8 *value)
{
	unsigned int regval;
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_read(ov9285->regmap, addr, &regval);
		if (!ret)
			break;
	}

	if (ret)
		dev_err(ov9285->dev, "I2C read failed for addr: %x, ret %d\n", addr, ret);

	*value = regval & 0xff;

	return 0;
}

static int ov9285_write_reg(struct ov9285 *ov9285, u16 addr, u8 value)
{
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_write(ov9285->regmap, addr, value);
		if (!ret)
			break;
	}

	if (ret)
		dev_err(ov9285->dev, "I2C write failed for addr: %x, ret %d\n", addr, ret);

	return ret;
}

static int ov9285_get_id(struct ov9285 *ov9285)
{
	int rtn = -EINVAL;
	u32 id = 0;
	u8 val = 0;

	ov9285_read_reg(ov9285, 0x300a, &val);
	id |= (val << 8);

	ov9285_read_reg(ov9285, 0x300b, &val);
	id |= val;

	if (id != OV9285_ID) {
		dev_err(ov9285->dev, "Failed to get ov9285 id: 0x%x\n", id);
		return rtn;
	}

	dev_info(ov9285->dev, "success get id\n");

	return 0;
}

static int ov9285_set_register_array(struct ov9285 *ov9285,
				     const struct ov9285_regval *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = ov9285_write_reg(ov9285, settings->reg, settings->val);
		if (ret < 0)
			return ret;

		msleep(1);
	}

	return 0;
}

static int ov9285_set_gain(struct ov9285 *ov9285, u32 value)
{
	return 0;
}

static int ov9285_set_exposure(struct ov9285 *ov9285, u32 value)
{
	return 0;
}

/* Stop streaming */
static int ov9285_stop_streaming(struct ov9285 *ov9285)
{
	int ret;

	ret = ov9285_write_reg(ov9285, OV9285_STANDBY, 0x00);

	return ret;
}

static int ov9285_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9285 *ov9285 = container_of(ctrl->handler,
					     struct ov9285, ctrls);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(ov9285->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ret = ov9285_set_gain(ov9285, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ov9285_set_exposure(ov9285, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		break;
	default:
		dev_err(ov9285->dev, "Error ctrl->id %u, flag 0x%lx\n",
			ctrl->id, ctrl->flags);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov9285_ctrl_ops = {
	.s_ctrl = ov9285_set_ctrl,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int ov9285_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int ov9285_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	if (code->index >= ARRAY_SIZE(ov9285_formats))
		return -EINVAL;

	code->code = ov9285_formats[code->index].code;

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int ov9285_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
#else
static int ov9285_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
#endif
{
	if (fse->index >= ARRAY_SIZE(ov9285_formats))
		return -EINVAL;

	fse->min_width = ov9285_formats[fse->index].min_width;
	fse->min_height = ov9285_formats[fse->index].min_height;
	fse->max_width = ov9285_formats[fse->index].max_width;
	fse->max_height = ov9285_formats[fse->index].max_height;

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int ov9285_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *cfg,
			  struct v4l2_subdev_format *fmt)
#else
static int ov9285_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
#endif
{
	struct ov9285 *ov9285 = to_ov9285(sd);
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&ov9285->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(&ov9285->sd, cfg,
						      fmt->pad);
	else
		framefmt = &ov9285->current_format;

	fmt->format = *framefmt;

	mutex_unlock(&ov9285->lock);

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int ov9285_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *cfg,
			struct v4l2_subdev_format *fmt)
#else
static int ov9285_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *fmt)
#endif
{
	struct ov9285 *ov9285 = to_ov9285(sd);
	const struct ov9285_mode *mode;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	mutex_lock(&ov9285->lock);

	mode = v4l2_find_nearest_size(ov9285_modes,
				ARRAY_SIZE(ov9285_modes),
				width, height,
				fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;

	for (i = 0; i < ARRAY_SIZE(ov9285_formats); i++)
		if (ov9285_formats[i].code == fmt->format.code)
			break;

	if (i >= ARRAY_SIZE(ov9285_formats))
		i = 0;

	fmt->format.code = ov9285_formats[i].code;
	fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		format = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	} else {
		format = &ov9285->current_format;
		__v4l2_ctrl_s_ctrl(ov9285->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(ov9285->pixel_rate, mode->pixel_rate);
		__v4l2_ctrl_s_ctrl(ov9285->hblank, mode->hblank);

		ov9285->current_mode = mode;
	}

	*format = fmt->format;
	ov9285->status = 0;

	mutex_unlock(&ov9285->lock);

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
int ov9285_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_selection *sel)
#else
int ov9285_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
#endif
{
	int rtn = 0;
	struct ov9285 *ov9285 = to_ov9285(sd);
	const struct ov9285_mode *mode = ov9285->current_mode;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = mode->width;
		sel->r.height = mode->height;
	break;
	case V4L2_SEL_TGT_CROP:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = mode->width;
		sel->r.height = mode->height;
	break;
	default:
		rtn = -EINVAL;
		dev_err(ov9285->dev, "Error support target: 0x%x\n", sel->target);
	break;
	}

	return rtn;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int ov9285_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *cfg)
#else
static int ov9285_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
#endif
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 640;
	fmt.format.height = 480;

	ov9285_set_fmt(subdev, cfg, &fmt);

	return 0;
}

static int ov9285_write_current_format(struct ov9285 *ov9285,
				       struct v4l2_mbus_framefmt *format)
{
	switch (format->code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	break;
	default:
		dev_err(ov9285->dev, "Unknown pixel format\n");
		return -EINVAL;
	}

	return 0;
}

/* Start streaming */
static int ov9285_start_streaming(struct ov9285 *ov9285)
{
	int ret;

	/* Set current frame format */
	ret = ov9285_write_current_format(ov9285, &ov9285->current_format);
	if (ret < 0) {
		dev_err(ov9285->dev, "Could not set frame format\n");
		return ret;
	}

	/* Apply default values of current mode */
	ret = ov9285_set_register_array(ov9285, ov9285->current_mode->data,
				ov9285->current_mode->data_size);
	if (ret < 0) {
		dev_err(ov9285->dev, "Could not set current mode\n");
		return ret;
	}

	/* Apply customized values from user */
	ret = v4l2_ctrl_handler_setup(ov9285->sd.ctrl_handler);
	if (ret) {
		dev_err(ov9285->dev, "Could not sync v4l2 controls\n");
		return ret;
	}

	msleep(2);

	/* Start streaming */
	return ov9285_write_reg(ov9285, OV9285_STANDBY, 0x01);
}

static int ov9285_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov9285 *ov9285 = to_ov9285(sd);
	int ret = 0;

	if (ov9285->status == enable)
		return ret;
	else
		ov9285->status = enable;

	if (enable) {
		ret = ov9285_start_streaming(ov9285);
		if (ret) {
			dev_err(ov9285->dev, "Start stream failed\n");
			goto unlock_and_return;
		}

		dev_info(ov9285->dev, "stream on\n");
	} else {
		ov9285_stop_streaming(ov9285);

		dev_info(ov9285->dev, "stream off\n");
	}

unlock_and_return:

	return ret;
}

static int ov9285_power_on(struct ov9285 *ov9285)
{
	int ret;

	ret = clk_prepare_enable(ov9285->xclk);
	if (ret) {
		dev_err(ov9285->dev, "Failed to enable clock\n");
		return ret;
	}

	udelay(500);

	if (ov9285->pwdn_gpio) {
		ret = gpiod_direction_output(ov9285->pwdn_gpio, 1);
		if (ret) {
			dev_err(ov9285->dev, "Failed to set pwdn: ret %d\n", ret);
			return ret;
		}

		udelay(1000);
	}

	return 0;
}

static int ov9285_power_off(struct ov9285 *ov9285)
{

	int ret = 0;

	if (ov9285->pwdn_gpio) {
		ret = gpiod_direction_output(ov9285->pwdn_gpio, 0);
		if (ret) {
			dev_err(ov9285->dev, "Failed to set pwdn: ret %d\n", ret);
			return ret;
		}
	}

	clk_disable_unprepare(ov9285->xclk);

	return 0;
}

static int ov9285_power_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9285 *ov9285 = to_ov9285(sd);

	if (ov9285->pwdn_gpio)
		gpiod_direction_output(ov9285->pwdn_gpio, 0);

	return 0;
}

static int ov9285_power_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9285 *ov9285 = to_ov9285(sd);

	if (ov9285->pwdn_gpio)
		gpiod_direction_output(ov9285->pwdn_gpio, 1);

	return 0;
}

static int ov9285_log_status(struct v4l2_subdev *sd)
{
	struct ov9285 *ov9285 = to_ov9285(sd);

	dev_info(ov9285->dev, "log status done\n");

	return 0;
}

static const struct dev_pm_ops ov9285_pm_ops = {
	SET_RUNTIME_PM_OPS(ov9285_power_suspend, ov9285_power_resume, NULL)
};

const struct v4l2_subdev_core_ops ov9285_core_ops = {
	.log_status = ov9285_log_status,
};

static const struct v4l2_subdev_video_ops ov9285_video_ops = {
	.s_stream = ov9285_set_stream,
};

static const struct v4l2_subdev_pad_ops ov9285_pad_ops = {
	.init_cfg = ov9285_entity_init_cfg,
	.enum_mbus_code = ov9285_enum_mbus_code,
	.enum_frame_size = ov9285_enum_frame_size,
	.get_selection = ov9285_get_selection,
	.get_fmt = ov9285_get_fmt,
	.set_fmt = ov9285_set_fmt,
};

static const struct v4l2_subdev_ops ov9285_subdev_ops = {
	.core = &ov9285_core_ops,
	.video = &ov9285_video_ops,
	.pad = &ov9285_pad_ops,
};

static const struct media_entity_operations ov9285_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov9285_ctrls_init(struct ov9285 *ov9285)
{
	int rtn = 0;

	v4l2_ctrl_handler_init(&ov9285->ctrls, 5);

	v4l2_ctrl_new_std(&ov9285->ctrls, &ov9285_ctrl_ops,
				V4L2_CID_GAIN, 0, 0xF0, 1, 0);
	v4l2_ctrl_new_std(&ov9285->ctrls, &ov9285_ctrl_ops,
				V4L2_CID_EXPOSURE, 0, 0xffff, 1, 0);

	ov9285->link_freq = v4l2_ctrl_new_int_menu(&ov9285->ctrls,
					       &ov9285_ctrl_ops,
					       V4L2_CID_LINK_FREQ,
					       ARRAY_SIZE(ov9285_link_freq) - 1,
					       0, ov9285_link_freq);
	if (ov9285->link_freq)
		ov9285->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov9285->pixel_rate = v4l2_ctrl_new_std(&ov9285->ctrls, &ov9285_ctrl_ops,
						V4L2_CID_PIXEL_RATE, 1,
						INT_MAX, 1,
						ov9285_modes[0].pixel_rate);

	ov9285->hblank = v4l2_ctrl_new_std(&ov9285->ctrls, &ov9285_ctrl_ops,
						V4L2_CID_HBLANK, 1,
						INT_MAX, 1,
						ov9285_modes[0].hblank);

	ov9285->sd.ctrl_handler = &ov9285->ctrls;

	if (ov9285->ctrls.error) {
		dev_err(ov9285->dev, "Control initialization error %d\n",
			ov9285->ctrls.error);
		rtn = ov9285->ctrls.error;
	}

	return rtn;
}

static int ov9285_parse_mclk(struct ov9285 *ov9285)
{
	int rtn = 0;
	u32 xclk_freq;

	ov9285->xclk = devm_clk_get(ov9285->dev, "mclk");
	if (IS_ERR(ov9285->xclk)) {
		dev_err(ov9285->dev, "Could not get xclk");
		rtn = PTR_ERR(ov9285->xclk);
		goto err_return;
	}

	rtn = fwnode_property_read_u32(dev_fwnode(ov9285->dev), "clock-frequency",
				       &xclk_freq);
	if (rtn) {
		dev_err(ov9285->dev, "Could not get xclk frequency\n");
		goto err_return;
	}

	if (xclk_freq != OV9285_MCLK) {
		dev_err(ov9285->dev, "External clock frequency %u is not supported\n",
			xclk_freq);
		rtn = -EINVAL;
		goto err_return;
	}

	rtn = clk_set_rate(ov9285->xclk, xclk_freq);
	if (rtn) {
		dev_err(ov9285->dev, "Could not set xclk frequency\n");
		goto err_return;
	}

err_return:
	return rtn;
}

static int ov9285_parse_power(struct ov9285 *ov9285)
{
	ov9285->pwdn_gpio = devm_gpiod_get_optional(ov9285->dev, "pwdn", GPIOD_OUT_LOW);

	return 0;
}

static int ov9285_parse_endpoint(struct ov9285 *ov9285)
{
	int rtn = 0;
	struct fwnode_handle *endpoint;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(ov9285->dev), NULL);
	if (!endpoint) {
		dev_err(ov9285->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	rtn = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ov9285->ep);
	fwnode_handle_put(endpoint);
	if (rtn) {
		dev_err(ov9285->dev, "Parsing endpoint node failed\n");
		rtn = -EINVAL;
		goto err_return;
	}

	if (!ov9285->ep.nr_of_link_frequencies) {
		dev_err(ov9285->dev, "link-frequency property not found in DT\n");
		rtn = -EINVAL;
		goto err_free;
	}

	if (ov9285->ep.link_frequencies[0] != OV9285_DEFAULT_LINK_FREQ) {
		dev_err(ov9285->dev, "Unsupported link frequency\n");
		rtn = -EINVAL;
		goto err_free;
	}

	/* Only CSI2 is supported for now */
	if (ov9285->ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(ov9285->dev, "Unsupported bus type, should be CSI2\n");
		rtn = -EINVAL;
		goto err_free;
	}

	return rtn;

err_free:
	v4l2_fwnode_endpoint_free(&ov9285->ep);
err_return:
	return rtn;
}


static int ov9285_register_subdev(struct ov9285 *ov9285)
{
	int rtn = 0;

	v4l2_i2c_subdev_init(&ov9285->sd, ov9285->client, &ov9285_subdev_ops);

	ov9285->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov9285->sd.dev = &ov9285->client->dev;
	ov9285->sd.entity.ops = &ov9285_subdev_entity_ops;
	ov9285->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ov9285->pad.flags = MEDIA_PAD_FL_SOURCE;
	rtn = media_entity_pads_init(&ov9285->sd.entity, 1, &ov9285->pad);
	if (rtn < 0) {
		dev_err(ov9285->dev, "Could not register media entity\n");
		goto err_return;
	}

	rtn = v4l2_async_register_subdev(&ov9285->sd);
	if (rtn < 0) {
		dev_err(ov9285->dev, "Could not register v4l2 device\n");
		goto err_return;
	}

err_return:
	return rtn;
}

static int ov9285_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov9285 *ov9285;
	int ret = -EINVAL;

	ov9285 = devm_kzalloc(dev, sizeof(*ov9285), GFP_KERNEL);
	if (!ov9285)
		return -ENOMEM;

	ov9285->dev = dev;
	ov9285->client = client;
	ov9285->regmap = devm_regmap_init_i2c(client, &ov9285_regmap_config);
	if (IS_ERR(ov9285->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	ret = ov9285_parse_endpoint(ov9285);
	if (ret) {
		dev_err(ov9285->dev, "Error parse endpoint\n");
		goto return_err;
	}

	/* Set default mode to max resolution */
	ov9285->current_mode = &ov9285_modes[0];

	/* get system clock (xclk) */
	ret = ov9285_parse_mclk(ov9285);
	if (ret) {
		dev_err(ov9285->dev, "Error parse mclk\n");
		goto free_err;
	}

	ret = ov9285_parse_power(ov9285);
	if (ret) {
		dev_err(ov9285->dev, "Error parse power ctrls\n");
		goto free_err;
	}

	mutex_init(&ov9285->lock);

	/* Power on the device to match runtime PM state below */
	ret = ov9285_power_on(ov9285);
	if (ret < 0) {
		dev_err(dev, "Could not power on the device\n");
		goto free_err;
	}

	ret = ov9285_get_id(ov9285);
	if (ret) {
		dev_err(dev, "Could not get id\n");
		ov9285_power_off(ov9285);
		goto free_err;
	}

	ret = ov9285_ctrls_init(ov9285);
	if (ret) {
		dev_err(ov9285->dev, "Error ctrls init\n");
		goto free_ctrl;
	}

	ret = ov9285_register_subdev(ov9285);
	if (ret) {
		dev_err(ov9285->dev, "Error register subdev\n");
		goto free_entity;
	}

	v4l2_fwnode_endpoint_free(&ov9285->ep);

	return 0;

free_entity:
	media_entity_cleanup(&ov9285->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&ov9285->ctrls);
	mutex_destroy(&ov9285->lock);
free_err:
	v4l2_fwnode_endpoint_free(&ov9285->ep);
return_err:
	return ret;
}

static int ov9285_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9285 *ov9285 = to_ov9285(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	mutex_destroy(&ov9285->lock);

	ov9285_power_off(ov9285);
	devm_clk_put(ov9285->dev, ov9285->xclk);
	if (ov9285->pwdn_gpio)
		devm_gpiod_put(ov9285->dev, ov9285->pwdn_gpio);

	return 0;
}

static const struct of_device_id ov9285_of_match[] = {
	{ .compatible = "omnivision, ov9285" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov9285_of_match);

static struct i2c_driver ov9285_i2c_driver = {
	.probe_new  = ov9285_probe,
	.remove = ov9285_remove,
	.driver = {
		.name  = "ov9285",
		.pm = &ov9285_pm_ops,
		.of_match_table = of_match_ptr(ov9285_of_match),
	},
};

module_i2c_driver(ov9285_i2c_driver);

MODULE_DESCRIPTION("Sony OV9285 CMOS Image Sensor Driver");
MODULE_AUTHOR("keke.li");
MODULE_AUTHOR("keke.li <keke.li@amlogic.com>");
MODULE_LICENSE("GPL v2");
