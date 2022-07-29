// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX290 CMOS Image Sensor Driver
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

#define IMX290_STANDBY    0x3000
#define IMX290_REGHOLD    0x3001
#define IMX290_XMSTA      0x3002
#define IMX290_GAIN       0x3014
#define IMX290_EXPOSURE   0x3020
#define IMX290_ID         0xb201

#define IMX290_BLKLEVEL_LOW  0x300a
#define IMX290_BLKLEVEL_HIGH 0x300b

#define IMX290_HMAX_LOW  0x301c
#define IMX290_HMAX_HIGH 0x301d

#define IMX290_PGCTRL     0x308c

#define IMX290_FR_FDG_SEL    0x3009
#define IMX290_PHY_LANE_NUM  0x3407
#define IMX290_CSI_LANE_MODE 0x3443

#define AML_SENSOR_NAME  "imx290-%u"


struct imx290_regval {
	u16 reg;
	u8 val;
};

struct imx290_mode {
	u32 width;
	u32 height;
	u32 hmax;
	u32 link_freq_index;

	const struct imx290_regval *data;
	u32 data_size;
};

struct imx290 {
	int index;
	struct device *dev;
	struct clk *xclk;
	struct regmap *regmap;
	u8 nlanes;
	u8 bpp;

	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct v4l2_fwnode_endpoint ep;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct imx290_mode *current_mode;

	struct gpio_desc *rst_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *power_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;

	int status;
	struct mutex lock;
};

struct imx290_pixfmt {
	u32 code;
	u32 min_width;
	u32 max_width;
	u32 min_height;
	u32 max_height;
	u8 bpp;
};

static const struct imx290_pixfmt imx290_formats[] = {
	{ MEDIA_BUS_FMT_SRGGB10_1X10, 1280, 1920, 720, 1080, 10 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, 1280, 1920, 720, 1080, 12 },
};

static const struct regmap_config imx290_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static const struct imx290_regval imx290_global_init_settings[] = {
	{ 0x3007, 0x00 },
	//{ 0x3009, 0x00 },//lane setting fr fdg sel
	{ 0x3018, 0x65 },
	{ 0x3019, 0x04 },
	{ 0x301a, 0x00 },
	//{ 0x3443, 0x03 },//lane setting
	{ 0x3444, 0x20 },
	{ 0x3445, 0x25 },
	//{ 0x3407, 0x03 },// lane setting
	{ 0x303a, 0x0c },
	{ 0x3040, 0x00 },
	{ 0x3041, 0x00 },
	{ 0x303c, 0x00 },
	{ 0x303d, 0x00 },
	{ 0x3042, 0x9c },
	{ 0x3043, 0x07 },
	{ 0x303e, 0x49 },
	{ 0x303f, 0x04 },
	{ 0x304b, 0x0a },
	{ 0x300f, 0x00 },
	{ 0x3010, 0x21 },
	{ 0x3012, 0x64 },
	{ 0x3016, 0x09 },
	{ 0x3070, 0x02 },
	{ 0x3071, 0x11 },
	{ 0x309b, 0x10 },
	{ 0x309c, 0x22 },
	{ 0x30a2, 0x02 },
	{ 0x30a6, 0x20 },
	{ 0x30a8, 0x20 },
	{ 0x30aa, 0x20 },
	{ 0x30ac, 0x20 },
	{ 0x30b0, 0x43 },
	{ 0x3119, 0x9e },
	{ 0x311c, 0x1e },
	{ 0x311e, 0x08 },
	{ 0x3128, 0x05 },
	{ 0x313d, 0x83 },
	{ 0x3150, 0x03 },
	{ 0x317e, 0x00 },
	{ 0x32b8, 0x50 },
	{ 0x32b9, 0x10 },
	{ 0x32ba, 0x00 },
	{ 0x32bb, 0x04 },
	{ 0x32c8, 0x50 },
	{ 0x32c9, 0x10 },
	{ 0x32ca, 0x00 },
	{ 0x32cb, 0x04 },
	{ 0x332c, 0xd3 },
	{ 0x332d, 0x10 },
	{ 0x332e, 0x0d },
	{ 0x3358, 0x06 },
	{ 0x3359, 0xe1 },
	{ 0x335a, 0x11 },
	{ 0x3360, 0x1e },
	{ 0x3361, 0x61 },
	{ 0x3362, 0x10 },
	{ 0x33b0, 0x50 },
	{ 0x33b2, 0x1a },
	{ 0x33b3, 0x04 },
};

static const struct imx290_regval imx290_1080p_settings[] = {
	/* mode settings */
	{ 0x3007, 0x00 },
	{ 0x303a, 0x0c },
	{ 0x3414, 0x0a },
	{ 0x3472, 0x80 },
	{ 0x3473, 0x07 },
	{ 0x3418, 0x38 },// vmax
	{ 0x3419, 0x04 },// vmax
	{ 0x3012, 0x64 },
	{ 0x3013, 0x00 },
	{ 0x305c, 0x18 },
	{ 0x305d, 0x03 },
	{ 0x305e, 0x20 },
	{ 0x305f, 0x01 },
	{ 0x315e, 0x1a },
	{ 0x3164, 0x1a },
	{ 0x3480, 0x49 },
	/* data rate settings */
	//{ 0x3009, 0x01 },// fr fdg sel lane related 60/50 fps
	{ 0x3405, 0x10 },
	{ 0x3446, 0x57 },
	{ 0x3447, 0x00 },
	{ 0x3448, 0x37 },
	{ 0x3449, 0x00 },
	{ 0x344a, 0x1f },
	{ 0x344b, 0x00 },
	{ 0x344c, 0x1f },
	{ 0x344d, 0x00 },
	{ 0x344e, 0x1f },
	{ 0x344f, 0x00 },
	{ 0x3450, 0x77 },
	{ 0x3451, 0x00 },
	{ 0x3452, 0x1f },
	{ 0x3453, 0x00 },
	{ 0x3454, 0x17 },
	{ 0x3455, 0x00 },
	//{ 0x301c, 0x98 },// hmax low
	//{ 0x301d, 0x08 },// hmax high
};

static const struct imx290_regval imx290_720p_settings[] = {
	/* mode settings */
	{ 0x3007, 0x10 },
	{ 0x303a, 0x06 },
	{ 0x3414, 0x04 },
	{ 0x3472, 0x00 },
	{ 0x3473, 0x05 },
	{ 0x3418, 0xd0 },
	{ 0x3419, 0x02 },
	{ 0x3012, 0x64 },
	{ 0x3013, 0x00 },
	{ 0x305c, 0x20 },
	{ 0x305d, 0x00 },
	{ 0x305e, 0x20 },
	{ 0x305f, 0x01 },
	{ 0x315e, 0x1a },
	{ 0x3164, 0x1a },
	{ 0x3480, 0x49 },
	/* data rate settings */
	{ 0x3009, 0x01 },
	{ 0x3405, 0x10 },
	{ 0x3446, 0x4f },
	{ 0x3447, 0x00 },
	{ 0x3448, 0x2f },
	{ 0x3449, 0x00 },
	{ 0x344a, 0x17 },
	{ 0x344b, 0x00 },
	{ 0x344c, 0x17 },
	{ 0x344d, 0x00 },
	{ 0x344e, 0x17 },
	{ 0x344f, 0x00 },
	{ 0x3450, 0x57 },
	{ 0x3451, 0x00 },
	{ 0x3452, 0x17 },
	{ 0x3453, 0x00 },
	{ 0x3454, 0x17 },
	{ 0x3455, 0x00 },
	{ 0x301c, 0xe4 },
	{ 0x301d, 0x0c },
};

static const struct imx290_regval imx290_10bit_settings[] = {
	{ 0x3005, 0x00},
	{ 0x3046, 0x00},
	{ 0x3129, 0x1d},
	{ 0x317c, 0x12},
	{ 0x31ec, 0x37},
	{ 0x3441, 0x0a},
	{ 0x3442, 0x0a},
	{ 0x300a, 0x3c},
	{ 0x300b, 0x00},
};


static const struct imx290_regval imx290_12bit_settings[] = {
	{ 0x3005, 0x01 },
	{ 0x3046, 0x01 },
	{ 0x3129, 0x00 },
	{ 0x317c, 0x00 },
	{ 0x31ec, 0x0e },
	{ 0x3441, 0x0c },
	{ 0x3442, 0x0c },
	{ 0x300a, 0xf0 },
	{ 0x300b, 0x00 },
};

/* supported link frequencies */
#define FREQ_INDEX_1080P	0
#define FREQ_INDEX_720P		1

/* supported link frequencies */
static const s64 imx290_link_freq_2lanes[] = {
	[FREQ_INDEX_1080P] = 445500000,
	[FREQ_INDEX_720P] = 297000000,
};


static const s64 imx290_link_freq_4lanes[] = {
	[FREQ_INDEX_1080P] = 222750000,
	[FREQ_INDEX_720P] = 148500000,
};


static inline const s64 *imx290_link_freqs_ptr(const struct imx290 *imx290)
{
	if (imx290->nlanes == 2)
		return imx290_link_freq_2lanes;
	else {
		return imx290_link_freq_4lanes;
	}
}

static inline int imx290_link_freqs_num(const struct imx290 *imx290)
{
	if (imx290->nlanes == 2)
		return ARRAY_SIZE(imx290_link_freq_2lanes);
	else
		return ARRAY_SIZE(imx290_link_freq_4lanes);
}

/* Mode configs */
static const struct imx290_mode imx290_modes_2lanes[] = {
	{
		.width = 1920,
		.height = 1080,
		.hmax  = 0x1130,
		.data = imx290_1080p_settings,
		.data_size = ARRAY_SIZE(imx290_1080p_settings),

		.link_freq_index = FREQ_INDEX_1080P,
	},
	{
		.width = 1280,
		.height = 720,
		.hmax = 0x19c8,
		.data = imx290_720p_settings,
		.data_size = ARRAY_SIZE(imx290_720p_settings),

		.link_freq_index = FREQ_INDEX_720P,
	},
};


static const struct imx290_mode imx290_modes_4lanes[] = {
	{
		.width = 1920,
		.height = 1080,
		.hmax = 0x0898,
		.link_freq_index = FREQ_INDEX_1080P,
		.data = imx290_1080p_settings,
		.data_size = ARRAY_SIZE(imx290_1080p_settings),
	},
	{
		.width = 1280,
		.height = 720,
		.hmax = 0x0ce4,
		.link_freq_index = FREQ_INDEX_720P,
		.data = imx290_720p_settings,
		.data_size = ARRAY_SIZE(imx290_720p_settings),
	},
};

static inline const struct imx290_mode *imx290_modes_ptr(const struct imx290 *imx290)
{
	if (imx290->nlanes == 2)
		return imx290_modes_2lanes;
	else
		return imx290_modes_4lanes;
}

static inline int imx290_modes_num(const struct imx290 *imx290)
{
	if (imx290->nlanes == 2)
		return ARRAY_SIZE(imx290_modes_2lanes);
	else
		return ARRAY_SIZE(imx290_modes_4lanes);
}

static inline struct imx290 *to_imx290(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx290, sd);
}

static inline int imx290_read_reg(struct imx290 *imx290, u16 addr, u8 *value)
{
	unsigned int regval;

	int i, ret;

	for (i = 0; i < 3; ++i) {
		ret = regmap_read(imx290->regmap, addr, &regval);
		if (0 == ret ) {
			break;
		}
	}

	if (ret)
		dev_err(imx290->dev, "I2C read with i2c transfer failed for addr: %x, ret %d\n", addr, ret);

	*value = regval & 0xff;
	return 0;
}

static int imx290_write_reg(struct imx290 *imx290, u16 addr, u8 value)
{
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_write(imx290->regmap, addr, value);
		if (0 == ret) {
			break;
		}
	}

	if (ret)
		dev_err(imx290->dev, "I2C write failed for addr: %x, ret %d\n", addr, ret);

	return ret;
}

static int imx290_get_id(struct imx290 *imx290)
{
	int rtn = -EINVAL;
	u32 id = 0;
	u8 val = 0;

	imx290_read_reg(imx290, 0x301e, &val);
	id |= (val << 8);

	imx290_read_reg(imx290, 0x301f, &val);
	id |= val;

	if (id != IMX290_ID) {
		dev_err(imx290->dev, "Failed to get imx290 id: 0x%x\n", id);
		return rtn;
	} else {
		dev_err(imx290->dev, "success get imx290 id 0x%x", id);
	}

	return 0;
}

static int imx290_set_register_array(struct imx290 *imx290,
				     const struct imx290_regval *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = imx290_write_reg(imx290, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int imx290_write_buffered_reg(struct imx290 *imx290, u16 address_low,
				     u8 nr_regs, u32 value)
{
	unsigned int i;
	int ret;

	ret = imx290_write_reg(imx290, IMX290_REGHOLD, 0x01);
	if (ret) {
		dev_err(imx290->dev, "Error setting hold register\n");
		return ret;
	}

	for (i = 0; i < nr_regs; i++) {
		ret = imx290_write_reg(imx290, address_low + i,
				       (u8)(value >> (i * 8)));
		if (ret) {
			dev_err(imx290->dev, "Error writing buffered registers\n");
			return ret;
		}
	}

	ret = imx290_write_reg(imx290, IMX290_REGHOLD, 0x00);
	if (ret) {
		dev_err(imx290->dev, "Error setting hold register\n");
		return ret;
	}

	return ret;
}

static int imx290_set_gain(struct imx290 *imx290, u32 value)
{
	int ret;

	ret = imx290_write_buffered_reg(imx290, IMX290_GAIN, 1, value);
	if (ret)
		dev_err(imx290->dev, "Unable to write gain\n");

	return ret;
}

static int imx290_set_exposure(struct imx290 *imx290, u32 value)
{
	int ret;

	ret = imx290_write_buffered_reg(imx290, IMX290_EXPOSURE, 2, value);
	if (ret)
		dev_err(imx290->dev, "Unable to write gain\n");

	return ret;
}


/* Stop streaming */
static int imx290_stop_streaming(struct imx290 *imx290)
{
	int ret;

	ret = imx290_write_reg(imx290, IMX290_STANDBY, 0x01);
	if (ret < 0)
		return ret;

	msleep(30);

	return imx290_write_reg(imx290, IMX290_XMSTA, 0x01);
}

static int imx290_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx290 *imx290 = container_of(ctrl->handler,
					     struct imx290, ctrls);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(imx290->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ret = imx290_set_gain(imx290, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx290_set_exposure(imx290, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		break;
	default:
		dev_err(imx290->dev, "Error ctrl->id %u, flag 0x%lx\n",
			ctrl->id, ctrl->flags);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx290->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx290_ctrl_ops = {
	.s_ctrl = imx290_set_ctrl,
};
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int imx290_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int imx290_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	if (code->index >= ARRAY_SIZE(imx290_formats))
		return -EINVAL;

	code->code = imx290_formats[code->index].code;

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int imx290_enum_frame_size(struct v4l2_subdev *sd,
			        struct v4l2_subdev_state *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
#else
static int imx290_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
#endif
{
	if (fse->index >= ARRAY_SIZE(imx290_formats))
		return -EINVAL;

	fse->min_width = imx290_formats[fse->index].min_width;
	fse->min_height = imx290_formats[fse->index].min_height;;
	fse->max_width = imx290_formats[fse->index].max_width;
	fse->max_height = imx290_formats[fse->index].max_height;

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int imx290_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *cfg,
			  struct v4l2_subdev_format *fmt)
#else
static int imx290_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
#endif
{
	struct imx290 *imx290 = to_imx290(sd);
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&imx290->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(&imx290->sd, cfg,
						      fmt->pad);
	else
		framefmt = &imx290->current_format;

	fmt->format = *framefmt;

	mutex_unlock(&imx290->lock);

	return 0;
}


static inline u8 imx290_get_link_freq_index(struct imx290 *imx290)
{
	return imx290->current_mode->link_freq_index;
}

static s64 imx290_get_link_freq(struct imx290 *imx290)
{
	u8 index = imx290_get_link_freq_index(imx290);

	return *(imx290_link_freqs_ptr(imx290) + index);
}

static u64 imx290_calc_pixel_rate(struct imx290 *imx290)
{
	s64 link_freq = imx290_get_link_freq(imx290);
	u8 nlanes = imx290->nlanes;
	u64 pixel_rate;

	/* pixel rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
	pixel_rate = link_freq * 2 * nlanes;
	do_div(pixel_rate, imx290->bpp);
	return pixel_rate;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int imx290_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *cfg,
			struct v4l2_subdev_format *fmt)
#else
static int imx290_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *fmt)
#endif
{
	struct imx290 *imx290 = to_imx290(sd);
	const struct imx290_mode *mode;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	mutex_lock(&imx290->lock);

	mode = v4l2_find_nearest_size(imx290_modes_ptr(imx290),
				 imx290_modes_num(imx290),
				width, height,
				fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;

	for (i = 0; i < ARRAY_SIZE(imx290_formats); i++) {
		if (imx290_formats[i].code == fmt->format.code) {
			dev_err(imx290->dev, " zzw find proper format \n");
			break;
		}
	}

	if (i >= ARRAY_SIZE(imx290_formats)) {
		i = 0;
		dev_err(imx290->dev, " zzw No format. reset i = 0 \n");
	}

	fmt->format.code = imx290_formats[i].code;
	fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		dev_err(imx290->dev, " zzw try format \n");
		format = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	} else {
		dev_err(imx290->dev, " zzw set format, w %d, h %d, code 0x%x \n",
            fmt->format.width, fmt->format.height,
            fmt->format.code);
		format = &imx290->current_format;
		imx290->current_mode = mode;
		imx290->bpp = imx290_formats[i].bpp;

		if (imx290->link_freq)
			__v4l2_ctrl_s_ctrl(imx290->link_freq, imx290_get_link_freq_index(imx290) );
		if (imx290->pixel_rate)
			__v4l2_ctrl_s_ctrl_int64(imx290->pixel_rate, imx290_calc_pixel_rate(imx290) );

	}

	*format = fmt->format;
	imx290->status = 0;

	mutex_unlock(&imx290->lock);

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
int imx290_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_selection *sel)
#else
int imx290_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
#endif
{
	int rtn = 0;
	struct imx290 *imx290 = to_imx290(sd);
	const struct imx290_mode *mode = imx290->current_mode;

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
		dev_err(imx290->dev, "Error support target: 0x%x\n", sel->target);
	break;
	}

	return rtn;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int imx290_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *cfg)
#else
static int imx290_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
#endif
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1920;
	fmt.format.height = 1080;

	imx290_set_fmt(subdev, cfg, &fmt);

	return 0;
}

static int imx290_write_current_format(struct imx290 *imx290,
				       struct v4l2_mbus_framefmt *format)
{
	int ret;

	switch (format->code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		ret = imx290_set_register_array(imx290, imx290_10bit_settings,
						ARRAY_SIZE(
							imx290_10bit_settings));
		if (ret < 0) {
			dev_err(imx290->dev, "Could not set format registers\n");
			return ret;
		}
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		ret = imx290_set_register_array(imx290, imx290_12bit_settings,
						ARRAY_SIZE(
							imx290_12bit_settings));
		if (ret < 0) {
			dev_err(imx290->dev, "Could not set format registers\n");
			return ret;
		}
		break;


	default:
		dev_err(imx290->dev, "Unknown pixel format, code 0x%x \n", format->code);
		return -EINVAL;
	}

	return 0;
}


static int imx290_set_hmax(struct imx290 *imx290, u32 val)
{
	int ret;

	ret = imx290_write_reg(imx290, IMX290_HMAX_LOW, (val & 0xff));
	if (ret) {
		dev_err(imx290->dev, "Error setting HMAX register\n");
		return ret;
	}

	ret = imx290_write_reg(imx290, IMX290_HMAX_HIGH, ((val >> 8) & 0xff));
	if (ret) {
		dev_err(imx290->dev, "Error setting HMAX register\n");
		return ret;
	}

	return 0;
}

/* Start streaming */
static int imx290_start_streaming(struct imx290 *imx290)
{
	int ret;

	/* Set init register settings */
	ret = imx290_set_register_array(imx290, imx290_global_init_settings,
				ARRAY_SIZE(imx290_global_init_settings));
	if (ret < 0) {
		dev_err(imx290->dev, "Could not set init registers\n");
		return ret;
	}

	/* Set current frame format */
	ret = imx290_write_current_format(imx290, &imx290->current_format);
	if (ret < 0) {
		dev_err(imx290->dev, "Could not set frame format\n");
		return ret;
	}

	/* Apply default values of current mode */
	ret = imx290_set_register_array(imx290, imx290->current_mode->data,
				imx290->current_mode->data_size);
	if (ret < 0) {
		dev_err(imx290->dev, "Could not set current mode\n");
		return ret;
	}

	ret = imx290_set_hmax(imx290, imx290->current_mode->hmax);
	if (ret < 0)
		return ret;

	/* Apply customized values from user */
	ret = v4l2_ctrl_handler_setup(imx290->sd.ctrl_handler);
	if (ret) {
		dev_err(imx290->dev, "Could not sync v4l2 controls\n");
		return ret;
	}

	ret = imx290_write_reg(imx290, IMX290_STANDBY, 0x00);
	if (ret < 0)
		return ret;

	msleep(30);

	/* Start streaming */
	return imx290_write_reg(imx290, IMX290_XMSTA, 0x00);
}

static int imx290_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx290 *imx290 = to_imx290(sd);
	int ret = 0;

	if (imx290->status == enable)
		return ret;
	else
		imx290->status = enable;

	if (enable) {
		ret = imx290_start_streaming(imx290);
		if (ret) {
			dev_err(imx290->dev, "Start stream failed\n");
			goto unlock_and_return;
		}

		dev_info(imx290->dev, "stream on\n");
	} else {
		imx290_stop_streaming(imx290);

		dev_info(imx290->dev, "stream off\n");
	}

unlock_and_return:

	return ret;
}


static int imx290_set_data_lanes(struct imx290 *imx290)
{
	int ret = 0, laneval, frsel;

	switch (imx290->nlanes) {
	case 2:
		laneval = 0x01;
		frsel = 0x02;
		break;
	case 4:
		laneval = 0x03;
		frsel = 0x01;
		break;
	default:
		/*
		 * We should never hit this since the data lane count is
		 * validated in probe itself
		 */
		dev_err(imx290->dev, "Lane configuration not supported\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = imx290_write_reg(imx290, IMX290_PHY_LANE_NUM, laneval);
	if (ret) {
		dev_err(imx290->dev, "Error setting Physical Lane number register\n");
		goto exit;
	}

	ret = imx290_write_reg(imx290, IMX290_CSI_LANE_MODE, laneval);
	if (ret) {
		dev_err(imx290->dev, "Error setting CSI Lane mode register\n");
		goto exit;
	}

	ret = imx290_write_reg(imx290, IMX290_FR_FDG_SEL, frsel);
	if (ret)
		dev_err(imx290->dev, "Error setting FR/FDG SEL register\n");

exit:
	return ret;
}

static int imx290_power_on(struct imx290 *imx290)
{
	int ret;

	gpiod_set_value_cansleep(imx290->pwdn_gpio, 0);

	gpiod_set_value_cansleep(imx290->rst_gpio, 1);

	ret = clk_prepare_enable(imx290->xclk);
	if (ret) {
		dev_err(imx290->dev, "Failed to enable clock\n");
		return ret;
	}

	// 30ms
	usleep_range(30000, 31000);

	/* Set data lane count */
	imx290_set_data_lanes(imx290);

	return 0;
}

static int imx290_power_off(struct imx290 *imx290)
{
	clk_disable_unprepare(imx290->xclk);

	gpiod_set_value_cansleep(imx290->rst_gpio, 0);
	gpiod_set_value_cansleep(imx290->pwdn_gpio, 1);

	return 0;
}

static int imx290_power_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx290 *imx290 = to_imx290(sd);

	gpiod_set_value_cansleep(imx290->rst_gpio, 0);
	gpiod_set_value_cansleep(imx290->pwdn_gpio, 1);

	return 0;
}

static int imx290_power_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx290 *imx290 = to_imx290(sd);

	gpiod_set_value_cansleep(imx290->rst_gpio, 0);
	gpiod_set_value_cansleep(imx290->pwdn_gpio, 1);

	return 0;
}

static int imx290_log_status(struct v4l2_subdev *sd)
{
	struct imx290 *imx290 = to_imx290(sd);

	dev_info(imx290->dev, "log status done\n");

	return 0;
}

static const struct dev_pm_ops imx290_pm_ops = {
	SET_RUNTIME_PM_OPS(imx290_power_suspend, imx290_power_resume, NULL)
};

const struct v4l2_subdev_core_ops imx290_core_ops = {
	.log_status = imx290_log_status,
};

static const struct v4l2_subdev_video_ops imx290_video_ops = {
	.s_stream = imx290_set_stream,
};

static const struct v4l2_subdev_pad_ops imx290_pad_ops = {
	.init_cfg = imx290_entity_init_cfg,
	.enum_mbus_code = imx290_enum_mbus_code,
	.enum_frame_size = imx290_enum_frame_size,
	.get_selection = imx290_get_selection,
	.get_fmt = imx290_get_fmt,
	.set_fmt = imx290_set_fmt,
};

static const struct v4l2_subdev_ops imx290_subdev_ops = {
	.core = &imx290_core_ops,
	.video = &imx290_video_ops,
	.pad = &imx290_pad_ops,
};

static const struct media_entity_operations imx290_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx290_ctrls_init(struct imx290 *imx290)
{
	int rtn = 0;

	v4l2_ctrl_handler_init(&imx290->ctrls, 4);

	v4l2_ctrl_new_std(&imx290->ctrls, &imx290_ctrl_ops,
				V4L2_CID_GAIN, 0, 0xF0, 1, 0);

	v4l2_ctrl_new_std(&imx290->ctrls, &imx290_ctrl_ops,
				V4L2_CID_EXPOSURE, 0, 0xffff, 1, 0);

	imx290->link_freq = v4l2_ctrl_new_int_menu(&imx290->ctrls,
					       &imx290_ctrl_ops,
					       V4L2_CID_LINK_FREQ,
					       imx290_link_freqs_num(imx290) - 1,
					       0, imx290_link_freqs_ptr(imx290) );

	if (imx290->link_freq)
		imx290->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx290->pixel_rate = v4l2_ctrl_new_std(&imx290->ctrls,
					       &imx290_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       1, INT_MAX, 1,
					       imx290_calc_pixel_rate(imx290));

	imx290->sd.ctrl_handler = &imx290->ctrls;

	if (imx290->ctrls.error) {
		dev_err(imx290->dev, "Control initialization a error  %d\n",
			imx290->ctrls.error);
		rtn = imx290->ctrls.error;
	}

	return rtn;
}

static int imx290_parse_mclk(struct imx290 *imx290)
{
	int rtn = 0;
	u32 xclk_freq;

	imx290->xclk = devm_clk_get(imx290->dev, "mclk");
	if (IS_ERR(imx290->xclk)) {
		dev_err(imx290->dev, "Could not get xclk");
		rtn = PTR_ERR(imx290->xclk);
		goto err_return;
	}

	rtn = fwnode_property_read_u32(dev_fwnode(imx290->dev), "clock-frequency",
				       &xclk_freq);
	if (rtn) {
		dev_err(imx290->dev, "Could not get xclk frequency\n");
		goto err_return;
	}

	/* external clock must be 37.125 MHz */
	if (xclk_freq != 37125000) {
		dev_err(imx290->dev, "External clock frequency %u is not supported\n",
			xclk_freq);
		rtn = -EINVAL;
		goto err_return;
	}

	rtn = clk_set_rate(imx290->xclk, xclk_freq);
	if (rtn) {
		dev_err(imx290->dev, "Could not set xclk frequency\n");
		goto err_return;
	}

err_return:
	return rtn;
}

static int imx290_parse_power(struct imx290 *imx290)
{
	int rtn = 0;

	imx290->rst_gpio = devm_gpiod_get_optional(imx290->dev,
						"reset",
						GPIOD_OUT_LOW);
	if (IS_ERR(imx290->rst_gpio)) {
		dev_err(imx290->dev, "Cannot get reset gpio\n");
		rtn = PTR_ERR(imx290->rst_gpio);
		goto err_return;
	}

	imx290->pwdn_gpio = devm_gpiod_get_optional(imx290->dev,
						"pwdn",
						GPIOD_OUT_LOW);
	if (IS_ERR(imx290->pwdn_gpio)) {
		dev_err(imx290->dev, "Cannot get pwdn gpio\n");
		rtn = PTR_ERR(imx290->pwdn_gpio);
		goto err_return;
	}

err_return:

	return rtn;
}

/*
 * Returns 0 if all link frequencies used by the driver for the given number
 * of MIPI data lanes are mentioned in the device tree, or the value of the
 * first missing frequency otherwise.
 */
static s64 imx290_check_link_freqs(const struct imx290 *imx290,
				   const struct v4l2_fwnode_endpoint *ep)
{
	int i, j;
	const s64 *freqs = imx290_link_freqs_ptr(imx290);
	int freqs_count = imx290_link_freqs_num(imx290);

	for (i = 0; i < freqs_count; i++) {
		for (j = 0; j < ep->nr_of_link_frequencies; j++) {
			if (freqs[i] == ep->link_frequencies[j]) {
				return 0;
			}
		}
		if (j == ep->nr_of_link_frequencies)
			return freqs[i];
	}
	return 0;
}

static int imx290_parse_endpoint(struct imx290 *imx290)
{
	int rtn = 0;
	s64 fq;
	struct fwnode_handle *endpoint;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(imx290->dev), NULL);
	if (!endpoint) {
		dev_err(imx290->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	rtn = v4l2_fwnode_endpoint_alloc_parse(endpoint, &imx290->ep);
	fwnode_handle_put(endpoint);
	if (rtn) {
		dev_err(imx290->dev, "Parsing endpoint node failed\n");
		rtn = -EINVAL;
		goto err_return;
	}

	/* Only CSI2 is supported for now */
	if (imx290->ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(imx290->dev, "Unsupported bus type, should be CSI2\n");
		rtn = -EINVAL;
		goto err_free;
	}

	imx290->nlanes = imx290->ep.bus.mipi_csi2.num_data_lanes;
	if (imx290->nlanes != 2 && imx290->nlanes != 4) {
		dev_err(imx290->dev, "Invalid data lanes: %d\n", imx290->nlanes);
		rtn = -EINVAL;
		goto err_free;
	}
	dev_info(imx290->dev, "Using %u data lanes\n", imx290->nlanes);

	if (!imx290->ep.nr_of_link_frequencies) {
		dev_err(imx290->dev, "link-frequency property not found in DT\n");
		rtn = -EINVAL;
		goto err_free;
	}

	/* Check that link frequences for all the modes are in device tree */
	fq = imx290_check_link_freqs(imx290, &imx290->ep);
	if (fq) {
		dev_err(imx290->dev, "Link frequency of %lld is not supported\n", fq);
		rtn = -EINVAL;
		goto err_free;
	}

	return rtn;

err_free:
	v4l2_fwnode_endpoint_free(&imx290->ep);
err_return:
	return rtn;
}


static int imx290_register_subdev(struct imx290 *imx290)
{
	int rtn = 0;

	v4l2_i2c_subdev_init(&imx290->sd, imx290->client, &imx290_subdev_ops);

	imx290->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx290->sd.dev = &imx290->client->dev;
	imx290->sd.entity.ops = &imx290_subdev_entity_ops;
	imx290->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	snprintf(imx290->sd.name, sizeof(imx290->sd.name), AML_SENSOR_NAME, imx290->index);

	imx290->pad.flags = MEDIA_PAD_FL_SOURCE;
	rtn = media_entity_pads_init(&imx290->sd.entity, 1, &imx290->pad);
	if (rtn < 0) {
		dev_err(imx290->dev, "Could not register media entity\n");
		goto err_return;
	}

	rtn = v4l2_async_register_subdev(&imx290->sd);
	if (rtn < 0) {
		dev_err(imx290->dev, "Could not register v4l2 device\n");
		goto err_return;
	}

err_return:
	return rtn;
}

static int imx290_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx290 *imx290;
	int ret = -EINVAL;


	imx290 = devm_kzalloc(dev, sizeof(*imx290), GFP_KERNEL);
	if (!imx290)
		return -ENOMEM;
	dev_err(dev, "i2c dev addr 0x%x, name %s \n", client->addr, client->name);

	imx290->dev = dev;
	imx290->client = client;

	imx290->regmap = devm_regmap_init_i2c(client, &imx290_regmap_config);
	if (IS_ERR(imx290->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	if (of_property_read_u32(dev->of_node, "index", &imx290->index)) {
		dev_err(dev, "Failed to read sensor index. default to 0\n");
		imx290->index = 0;
	}


	ret = imx290_parse_endpoint(imx290);
	if (ret) {
		dev_err(imx290->dev, "Error parse endpoint\n");
		goto return_err;
	}


	/* get system clock (xclk) */
	ret = imx290_parse_mclk(imx290);
	if (ret) {
		dev_err(imx290->dev, "Error parse mclk\n");
		goto free_err;
	}

	ret = imx290_parse_power(imx290);
	if (ret) {
		dev_err(imx290->dev, "Error parse power ctrls\n");
		goto free_err;
	}

	mutex_init(&imx290->lock);

	/* Power on the device to match runtime PM state below */
	dev_err(dev, "bef get id. pwdn -0, reset - 1\n");

	ret = imx290_power_on(imx290);
	if (ret < 0) {
		dev_err(dev, "Could not power on the device\n");
		goto free_err;
	}


	ret = imx290_get_id(imx290);
	if (ret) {
		dev_err(dev, "Could not get id\n");
		imx290_power_off(imx290);
		goto free_err;
	}

	/*
	 * Initialize the frame format. In particular, imx290->current_mode
	 * and imx290->bpp are set to defaults: imx290_calc_pixel_rate() call
	 * below in imx290_ctrls_init relies on these fields.
	 */
	imx290_entity_init_cfg(&imx290->sd, NULL);

	ret = imx290_ctrls_init(imx290);
	if (ret) {
		dev_err(imx290->dev, "Error ctrls init\n");
		goto free_ctrl;
	}

	ret = imx290_register_subdev(imx290);
	if (ret) {
		dev_err(imx290->dev, "Error register subdev\n");
		goto free_entity;
	}

	v4l2_fwnode_endpoint_free(&imx290->ep);

	dev_info(imx290->dev, "probe done \n");

	return 0;

free_entity:
	media_entity_cleanup(&imx290->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&imx290->ctrls);
	mutex_destroy(&imx290->lock);
free_err:
	v4l2_fwnode_endpoint_free(&imx290->ep);
return_err:
	return ret;
}

static int imx290_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx290 *imx290 = to_imx290(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	mutex_destroy(&imx290->lock);

	imx290_power_off(imx290);

	return 0;
}

static const struct of_device_id imx290_of_match[] = {
	{ .compatible = "sony, imx290" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx290_of_match);

static struct i2c_driver imx290_i2c_driver = {
	.probe_new  = imx290_probe,
	.remove = imx290_remove,
	.driver = {
		.name  = "imx290",
		.pm = &imx290_pm_ops,
		.of_match_table = of_match_ptr(imx290_of_match),
	},
};

module_i2c_driver(imx290_i2c_driver);

MODULE_DESCRIPTION("Sony IMX290 CMOS Image Sensor Driver");
MODULE_AUTHOR("keke.li");
MODULE_AUTHOR("keke.li <keke.li@amlogic.com>");
MODULE_LICENSE("GPL v2");
