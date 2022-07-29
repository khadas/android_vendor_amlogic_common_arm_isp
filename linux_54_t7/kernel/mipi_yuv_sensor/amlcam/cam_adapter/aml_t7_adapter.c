/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2020 Amlogic or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#define pr_fmt(fmt)  "aml-adap:%s:%d: " fmt, __func__, __LINE__

#include <linux/version.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>

#include "aml_t7_cam.h"

#define AML_ADAPTER_NAME "t7-adapter-%u"

static const struct aml_format adap_support_formats[] = {
	{0, 0, MEDIA_BUS_FMT_YUYV8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_UYVY8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_YVYU8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_VYUY8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_SBGGR8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SGBRG8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SGRBG8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SRGGB8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SBGGR10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SGBRG10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SGRBG10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SRGGB10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SBGGR12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SGBRG12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SGRBG12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SRGGB12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SBGGR14_1X14, 0, 1, 14},
	{0, 0, MEDIA_BUS_FMT_SGBRG14_1X14, 0, 1, 14},
	{0, 0, MEDIA_BUS_FMT_SGRBG14_1X14, 0, 1, 14},
	{0, 0, MEDIA_BUS_FMT_SRGGB14_1X14, 0, 1, 14},
};

static void __iomem *adap_ioremap_resource(void *a_dev, char *name)
{
	void __iomem *reg;
	struct resource *res;
	resource_size_t size;
	struct adapter_dev_t *adap_dev;
	struct device *dev;
	struct platform_device *pdev;

	if (!a_dev || !name) {
		pr_err("Error input param\n");
		return NULL;
	}

	adap_dev = a_dev;
	dev = adap_dev->dev;
	pdev = adap_dev->pdev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(dev, "Error %s res\n", name);
		return NULL;
	}

	size = resource_size(res);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	reg = devm_ioremap(dev, res->start, size);
#else
	reg = devm_ioremap_nocache(dev, res->start, size);
#endif
	if (!reg) {
		dev_err(dev, "Failed to ioremap %s %pR\n", name, res);
		reg = IOMEM_ERR_PTR(-ENOMEM);
	}

	return reg;
}

static void adap_iounmap_resource(void *a_dev)
{
	struct device *dev;
	struct adapter_dev_t *adap_dev;

	adap_dev = a_dev;
	dev = adap_dev->dev;

	if (adap_dev->adap) {
		devm_iounmap(dev, adap_dev->adap);
		adap_dev->adap = NULL;
	}
}

static int adap_of_parse_version(struct adapter_dev_t *adap_dev)
{
	int rtn = 0;
	struct device *dev = adap_dev->dev;

	switch (adap_dev->version) {
	case 0:
		adap_dev->ops = &adap_dev_hw_ops;
	break;
	default:
		rtn = -EINVAL;
		dev_err(dev, "Error invalid version num: %u\n", adap_dev->version);
	break;
	}

	return rtn;
}

static int adap_of_parse_dev(struct adapter_dev_t *adap_dev)
{
	int rtn = 0;

	adap_dev->adap = adap_ioremap_resource(adap_dev, "adapter");
	if (!adap_dev->adap) {
		dev_err(adap_dev->dev, "Failed to get adapter reg\n");
		rtn = -EINVAL;
		goto error_rtn;
	}

	adap_dev->isp_top = adap_ioremap_resource(adap_dev, "isp_top");
	if (!adap_dev->isp_top) {
		dev_err(adap_dev->dev, "Failed to get isp_top reg\n");
		rtn = -EINVAL;
		goto error_rtn;
	}

	adap_dev->irq = irq_of_parse_and_map(adap_dev->dev->of_node, 0);
	if (!adap_dev->irq) {
		dev_err(adap_dev->dev, "Error to parse irq\n");
		rtn = -EINVAL;
		goto error_rtn;
	}

	of_reserved_mem_device_init(adap_dev->dev);

	adap_dev->adap_clk = devm_clk_get(adap_dev->dev, "cts_mipi_isp_clk");
	if (IS_ERR(adap_dev->adap_clk)) {
		devm_iounmap(adap_dev->dev, adap_dev->adap);
		dev_err(adap_dev->dev, "Error to get adap_clk\n");
		return PTR_ERR(adap_dev->adap_clk);
	}

error_rtn:
	return rtn;
}

static irqreturn_t adap_interrupt_handler(int irq, void *dev)
{
	int id = 0;
	u32 status = 0;
	struct aml_video *video;
	struct adapter_dev_t *adap_dev = dev;
	status = adap_dev->ops->hw_interrupt_status(adap_dev);

	if (status & (0x1 << 18)) {
		video = &adap_dev->video[AML_ADAP_STREAM_RAW];
		if (video->ops->cap_irq_handler)
			video->ops->cap_irq_handler(video, status);
	}

	return IRQ_HANDLED;
}

static int adap_subdev_hw_start(struct adapter_dev_t *adap_dev)
{
	if (adap_dev->ops->hw_start)
		adap_dev->ops->hw_start(adap_dev);

	return 0;
}

static void adap_subdev_hw_stop(struct adapter_dev_t *adap_dev)
{
	if (adap_dev->ops->hw_stop)
		adap_dev->ops->hw_stop(adap_dev);
}

static int adap_subdev_set_stream(struct v4l2_subdev *sd, int enable)
{
	int rtn = 0;
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	if (enable)
		rtn = adap_subdev_hw_start(adap_dev);
	else
		adap_subdev_hw_stop(adap_dev);

	return rtn;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int adap_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int adap_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)

#endif
{
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	if (code->pad == AML_ADAP_PAD_SINK) {
		if (code->index >= adap_dev->fmt_cnt)
			return -EINVAL;

		code->code = adap_dev->formats[code->index].code;
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = adap_dev->formats[0].code;
	}

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static struct v4l2_mbus_framefmt *
adap_subdev_get_padfmt(struct adapter_dev_t *adap_dev,
				struct v4l2_subdev_state *cfg,
				struct v4l2_subdev_format *fmt)
#else
static struct v4l2_mbus_framefmt *
adap_subdev_get_padfmt(struct adapter_dev_t *adap_dev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)

#endif
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&adap_dev->sd, cfg, fmt->pad);

	return &adap_dev->pfmt[fmt->pad];
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int adap_subdev_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_format *fmt)
#else
static int adap_subdev_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
#endif
{
	struct v4l2_mbus_framefmt *format;
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	format = adap_subdev_get_padfmt(adap_dev, cfg, fmt);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int adap_subdev_convert_fmt(struct adapter_dev_t *adap_dev,
				struct v4l2_mbus_framefmt *format)
{
	int i = 0;
	int fmt = -1;
	const struct aml_format *am_fmt;

	for (i = 0; i < adap_dev->fmt_cnt; i++) {
		if (adap_dev->formats[i].code == format->code) {
			am_fmt = &adap_dev->formats[i];
			break;
		}
	}

	if (i == adap_dev->fmt_cnt)
		return fmt;

	switch (am_fmt->bpp) {
	case 8:
		fmt = ADAP_RAW8;
	break;
	case 10:
		fmt = ADAP_RAW10;
	break;
	case 12:
		fmt = ADAP_RAW12;
	break;
	case 14:
		fmt = ADAP_RAW14;
	break;
	case 16:
		fmt = ADAP_YUV422_8BIT;
	break;
	default:
		dev_err(adap_dev->dev, "Error support format\n");
	break;
	}

	return fmt;
}

static int adap_subdev_hw_init(struct adapter_dev_t *adap_dev,
				struct v4l2_mbus_framefmt *format)
{
	int rtn = 0;
	struct adapter_dev_param *param = &adap_dev->param;

	rtn = adap_subdev_convert_fmt(adap_dev, format);
	if (rtn < 0) {
		dev_err(adap_dev->dev, "Error to convert fmt\n");
		return rtn;
	}
	param->format = rtn;

	param->width = format->width;
	param->height = format->height;
	if (format->code == MEDIA_BUS_FMT_UYVY8_2X8 ) {
		param->mode = MODE_MIPI_YUV_SDR_DDR;
		param->dol_type = ADAP_DOL_YUV;
		param->offset.offset_x = 0;
		param->offset.offset_y = 0;
		param->offset.long_offset = 0x0;
		param->offset.short_offset = 0x0;
	} else {
		param->mode = MODE_MIPI_RAW_SDR_DDR;
		param->dol_type = ADAP_DOL_NONE;
		param->offset.offset_x = 12;
		param->offset.offset_y = 0;
		param->offset.long_offset = 0x8;
		param->offset.short_offset = 0x8;
	}

	if (adap_dev->ops->hw_reset)
		adap_dev->ops->hw_reset(adap_dev);

	if (adap_dev->ops->hw_init)
		adap_dev->ops->hw_init(adap_dev);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int adap_subdev_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_format *fmt)
#else
static int adap_subdev_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
#endif
{
	int rtn = 0;
	struct v4l2_mbus_framefmt *format;
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	format = adap_subdev_get_padfmt(adap_dev, cfg, fmt);
	if (format == NULL)
		return -EINVAL;

	*format = fmt->format;
	format->field = V4L2_FIELD_NONE;

	if (fmt->pad == AML_ADAP_PAD_SINK) {
		rtn = adap_subdev_hw_init(adap_dev, format);
	}

	return rtn;
}

static int adap_subdev_log_status(struct v4l2_subdev *sd)
{
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	dev_info(adap_dev->dev, "log status done\n");

	return 0;
}

const struct v4l2_subdev_core_ops adap_subdev_core_ops = {
	.log_status = adap_subdev_log_status,
};

static const struct v4l2_subdev_video_ops adap_subdev_video_ops = {
	.s_stream = adap_subdev_set_stream,
};

static const struct v4l2_subdev_pad_ops adap_subdev_pad_ops = {
	.enum_mbus_code = adap_subdev_enum_mbus_code,
	.get_fmt = adap_subdev_get_format,
	.set_fmt = adap_subdev_set_format,
};

static const struct v4l2_subdev_ops adap_subdev_v4l2_ops = {
	.core = &adap_subdev_core_ops,
	.video = &adap_subdev_video_ops,
	.pad = &adap_subdev_pad_ops,
};

static int adap_v4l2_subdev_link_validate(struct media_link *link)
{
	int rtn = 0;
	struct media_entity *src = link->source->entity;
	struct media_entity *sink = link->sink->entity;

	rtn = v4l2_subdev_link_validate(link);
	if (rtn)
		pr_err("Error: src->sink: %s-->%s, rtn %d\n",
			src->name, sink->name, rtn);

	return rtn;
}

static const struct media_entity_operations adap_subdev_media_ops = {
	.link_validate = adap_v4l2_subdev_link_validate,
};

int aml_adap_subdev_register(struct adapter_dev_t *adap_dev)
{
	int rtn = -1;
	struct device *dev = adap_dev->dev;
	struct v4l2_subdev *sd = &adap_dev->sd;
	struct media_pad *pads = adap_dev->pads;
	struct v4l2_device *v4l2_dev = adap_dev->v4l2_dev;

	adap_dev->formats = adap_support_formats;
	adap_dev->fmt_cnt = ARRAY_SIZE(adap_support_formats);

	v4l2_subdev_init(sd, &adap_subdev_v4l2_ops);
	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), AML_ADAPTER_NAME, adap_dev->index);

	v4l2_set_subdevdata(sd, adap_dev);

	pads[AML_ADAP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[AML_ADAP_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	sd->entity.ops = &adap_subdev_media_ops;

	rtn = media_entity_pads_init(&sd->entity, AML_ADAP_PAD_MAX, pads);
	if (rtn) {
		dev_err(dev, "Error init entity pads: %d\n", rtn);
		return rtn;
	}

	rtn = v4l2_device_register_subdev(v4l2_dev, sd);
	if (rtn < 0) {
		dev_err(dev, "Error to register subdev: %d\n", rtn);
		media_entity_cleanup(&sd->entity);
		return rtn;
	}

	dev_info(adap_dev->dev,
		"Success to register adapter-%u subdev\n", adap_dev->index);

	return rtn;
}

void aml_adap_subdev_unregister(struct adapter_dev_t *adap_dev)
{
	struct v4l2_subdev *sd = &adap_dev->sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
}

int aml_adap_subdev_init(void *c_dev)
{
	int rtn = -1;
	struct platform_device *pdev;
	struct adapter_dev_t *adap_dev;
	struct device_node *node;
	struct cam_device *cam_dev = c_dev;

	adap_dev = &cam_dev->adap_dev;

	node = of_parse_phandle(cam_dev->dev->of_node, "adapter", 0);
	if (!node) {
		pr_err("Failed to parse adapter handle\n");
		return rtn;
	}

	adap_dev->pdev = of_find_device_by_node(node);
	if (!adap_dev->pdev) {
		of_node_put(node);
		pr_err("Failed to find adapter platform device");
		return rtn;
	}
	of_node_put(node);

	pdev = adap_dev->pdev;
	adap_dev->dev = &pdev->dev;
	adap_dev->v4l2_dev = &cam_dev->v4l2_dev;
	adap_dev->index = cam_dev->index;
	adap_dev->bus_info = cam_dev->bus_info;
	platform_set_drvdata(pdev, adap_dev);

	rtn = adap_of_parse_version(adap_dev);
	if (rtn) {
		dev_err(adap_dev->dev, "Failed to parse version\n");
		return rtn;
	}

	rtn = adap_of_parse_dev(adap_dev);
	if (rtn) {
		dev_err(adap_dev->dev, "Failed to parse dev\n");
		return rtn;
	}

	rtn = devm_request_irq(adap_dev->dev, adap_dev->irq,
			adap_interrupt_handler,
			IRQF_SHARED, "adapter", adap_dev);
	if (rtn) {
		adap_iounmap_resource(adap_dev);
		return rtn;
	}

	dev_pm_domain_attach(adap_dev->dev, true);
	pm_runtime_enable(adap_dev->dev);
	pm_runtime_get_sync(adap_dev->dev);

	clk_set_rate(adap_dev->adap_clk, 666666667);
	rtn = clk_prepare_enable(adap_dev->adap_clk);
	if (rtn) {
		adap_iounmap_resource(adap_dev);
		return rtn;
	}

	dev_info(adap_dev->dev, "Success adapter-%u subdev init\n", adap_dev->index);

	return rtn;
}

void aml_adap_subdev_deinit(void *c_dev)
{
	struct adapter_dev_t *adap_dev;
	struct cam_device *cam_dev;

	cam_dev = c_dev;
	adap_dev = &cam_dev->adap_dev;

	devm_free_irq(adap_dev->dev, adap_dev->irq, adap_dev);

	clk_disable_unprepare(adap_dev->adap_clk);
	if (adap_dev->adap_clk != NULL) {
		devm_clk_put(adap_dev->dev, adap_dev->adap_clk);
		adap_dev->adap_clk = NULL;
	}

	adap_iounmap_resource(adap_dev);

	pm_runtime_put_sync(adap_dev->dev);
	pm_runtime_disable(adap_dev->dev);
	dev_pm_domain_detach(adap_dev->dev, true);

	dev_info(adap_dev->dev, "Success adapter-%u subdev deinit\n", adap_dev->index);
}
