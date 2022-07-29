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

#define pr_fmt(fmt)  "aml-csiphy:%s:%d: " fmt, __func__, __LINE__

#include <linux/version.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/clk/clk-conf.h>

#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "aml_t7_cam.h"

#define AML_CSIPHY_NAME "t7-csi2phy-%u"

static const struct aml_format csiphy_support_formats[] = {
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

static int csiphy_of_parse_endpoint_node(struct device_node *node,
					struct csiphy_async_subdev *c_asd)
{
	int rtn = -1;
	struct v4l2_fwnode_endpoint vep = { { 0 } };

	rtn = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &vep);
	if (rtn) {
		pr_err("Failed to parse csiphy endpoint\n");
		return rtn;
	}

	c_asd->phy_id = vep.base.id;
	c_asd->data_lanes = vep.bus.mipi_csi2.num_data_lanes;

	return 0;
}

static int csiphy_of_parse_ports(struct csiphy_dev_t *csiphy_dev)
{
	unsigned int rtn = 0;
	struct device_node *node = NULL;
	struct device_node *remote = NULL;
	struct csiphy_async_subdev *c_asd = NULL;
	struct v4l2_async_subdev *asd = NULL;
	struct v4l2_async_notifier *notifier = csiphy_dev->notifier;
	struct device *dev = csiphy_dev->dev;

	v4l2_async_notifier_init(notifier);

	for_each_endpoint_of_node(dev->of_node, node) {
		if (!of_device_is_available(node))
			continue;

		remote = of_graph_get_remote_port_parent(node);
		if (!remote) {
			dev_err(dev, "Cannot get remote parent\n");
			of_node_put(node);
			return -EINVAL;
		}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
		asd = v4l2_async_notifier_add_fwnode_subdev(notifier,
					of_fwnode_handle(remote),
					struct v4l2_async_subdev);
#else
		asd = v4l2_async_notifier_add_fwnode_subdev(notifier,
					of_fwnode_handle(remote),
					sizeof(*c_asd));
#endif
		if (IS_ERR(asd)) {
			dev_err(dev, "Failed to add subdev\n");
			of_node_put(node);
			return -EINVAL;
		}

		c_asd = container_of(asd, struct csiphy_async_subdev, asd);

		rtn = csiphy_of_parse_endpoint_node(node, c_asd);
		if (rtn < 0) {
			of_node_put(node);
			return rtn;
		}
	}

	of_node_put(node);

	return rtn;
}

static void csiphy_notifier_cleanup(void *c_dev)
{
	struct csiphy_dev_t *csiphy_dev;
	struct v4l2_async_notifier *notifier;

	csiphy_dev = c_dev;
	notifier = csiphy_dev->notifier;

	v4l2_async_notifier_cleanup(notifier);
}

static void __iomem *csiphy_ioremap_resource(void *c_dev, char *name)
{
	void __iomem *reg;
	struct resource *res;
	resource_size_t size;
	struct csiphy_dev_t *csiphy_dev;
	struct device *dev;
	struct platform_device *pdev;

	if (!c_dev || !name) {
		pr_err("Error input param\n");
		return NULL;
	}

	csiphy_dev = c_dev;
	dev = csiphy_dev->dev;
	pdev = csiphy_dev->pdev;

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

static void csiphy_iounmap_resource(void *c_dev)
{
	struct device *dev;
	struct csiphy_dev_t *csiphy_dev;

	csiphy_dev = c_dev;
	dev = csiphy_dev->dev;

	if (csiphy_dev->csi_phy) {
		devm_iounmap(dev, csiphy_dev->csi_phy);
		csiphy_dev->csi_phy = NULL;
	}

	if (csiphy_dev->csi_aphy) {
		devm_iounmap(dev, csiphy_dev->csi_aphy);
		csiphy_dev->csi_aphy = NULL;
	}

	if (csiphy_dev->csi_host) {
		devm_iounmap(dev, csiphy_dev->csi_host);
		csiphy_dev->csi_host = NULL;
	}
}

static int csiphy_of_parse_version(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = 0;
	u32 *version;
	struct device *dev;

	dev = csiphy_dev->dev;
	version = &csiphy_dev->version;

	switch (*version) {
	case 0:
		csiphy_dev->ops = &csiphy_dev_hw_ops;
	break;
	default:
		rtn = -EINVAL;
		dev_err(dev, "Error invalid version num: %u\n", *version);
	break;
	}

	return rtn;
}

static int csiphy_of_parse_dev(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = -1;

	csiphy_dev->csi_phy = csiphy_ioremap_resource(csiphy_dev, "csi_phy");
	if (!csiphy_dev->csi_phy) {
		rtn = -EINVAL;
		goto error_rtn;
	}

	csiphy_dev->csi_aphy = csiphy_ioremap_resource(csiphy_dev, "csi_aphy");
	if (!csiphy_dev->csi_aphy) {
		rtn = -EINVAL;
		goto error_rtn;
	}

	csiphy_dev->csi_host = csiphy_ioremap_resource(csiphy_dev, "csi_host");
	if (!csiphy_dev->csi_host) {
		rtn = -EINVAL;
		goto error_rtn;
	}

	csiphy_dev->mipi_clk0 = devm_clk_get(csiphy_dev->dev, "cts_mipi_csi_phy_clk0");
	if (IS_ERR(csiphy_dev->mipi_clk0)) {
		dev_err(csiphy_dev->dev, "Error to get mipi_clk0\n");
		return PTR_ERR(csiphy_dev->mipi_clk0);
	}

	rtn = 0;

error_rtn:
	return rtn;
}

static struct media_entity *csiphy_subdev_get_sensor_entity(struct media_entity *entity)
{
	struct media_pad *pad;
	struct media_pad *r_pad;

	pad = &entity->pads[AML_CSIPHY_PAD_SINK];
	if (!(pad->flags & MEDIA_PAD_FL_SINK))
		return NULL;

	r_pad = media_entity_remote_pad(pad);
	if (!r_pad || !is_media_entity_v4l2_subdev(r_pad->entity))
		return NULL;

	if (r_pad->entity->function == MEDIA_ENT_F_CAM_SENSOR)
		return r_pad->entity;
	else
		return NULL;
}

static int csiphy_subdev_get_link_freq(struct media_entity *entity, s64 *link_freq)
{
	int rtn = 0;
	struct v4l2_querymenu qm;
	struct media_entity *sensor;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl *ctrl;

	sensor = csiphy_subdev_get_sensor_entity(entity);
	if (!sensor) {
		pr_err("Failed to get sensor entity\n");
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(sensor);

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_LINK_FREQ);
	if (!ctrl) {
		pr_err("Failed to get link freq ctrl\n");
		return -EINVAL;
	}

	qm.id = V4L2_CID_LINK_FREQ;
	qm.index = ctrl->val;

	rtn = v4l2_querymenu(subdev->ctrl_handler, &qm);
	if (rtn) {
		pr_err("Failed to querymenu idx %d\n", qm.index);
		return rtn;
	}

	*link_freq = qm.value;

	return 0;
}

static int csiphy_subdev_get_casd(struct media_entity *entity, struct csiphy_async_subdev **casd)
{
	struct media_entity *sensor;
	struct v4l2_subdev *subdev;

	sensor = csiphy_subdev_get_sensor_entity(entity);
	if (!sensor) {
		pr_err("Failed to get sensor entity\n");
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(sensor);

	*casd = subdev->host_priv;

	return 0;
}

static int csiphy_subdev_stream_on(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = -1;
	s64 link_freq = 0;
	struct csiphy_async_subdev *casd;

	rtn = csiphy_subdev_get_link_freq(&csiphy_dev->sd.entity, &link_freq);
	if (rtn)
		return rtn;

	pr_info("get link freq from sensor: %lld", link_freq);

	rtn = csiphy_subdev_get_casd(&csiphy_dev->sd.entity, &casd);
	if (rtn || !casd)
		return rtn;

	return csiphy_dev->ops->hw_start(csiphy_dev, casd->data_lanes, link_freq);
}

static void csiphy_subdev_stream_off(struct csiphy_dev_t *csiphy_dev)
{
	csiphy_dev->ops->hw_stop(csiphy_dev);
}

static int csiphy_subdev_set_stream(struct v4l2_subdev *sd, int enable)
{
	int rtn = 0;
	struct csiphy_dev_t *csiphy_dev = v4l2_get_subdevdata(sd);

	if (enable)
		rtn = csiphy_subdev_stream_on(csiphy_dev);
	else
		csiphy_subdev_stream_off(csiphy_dev);

	return rtn;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int csiphy_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int csiphy_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct csiphy_dev_t *csiphy_dev = v4l2_get_subdevdata(sd);

	if (code->pad == AML_CSIPHY_PAD_SINK) {
		if (code->index >= csiphy_dev->fmt_cnt)
			return -EINVAL;

		code->code = csiphy_dev->formats[code->index].code;
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = csiphy_dev->formats[0].code;
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static struct v4l2_mbus_framefmt *
csiphy_subdev_get_padfmt(struct csiphy_dev_t *csiphy_dev,
				struct v4l2_subdev_state *cfg,
				struct v4l2_subdev_format *fmt)

#else
static struct v4l2_mbus_framefmt *
csiphy_subdev_get_padfmt(struct csiphy_dev_t *csiphy_dev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
#endif
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&csiphy_dev->sd, cfg, fmt->pad);

	return &csiphy_dev->pfmt[fmt->pad];
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int csiphy_subdev_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_format *fmt)
#else
static int csiphy_subdev_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
#endif
{
	struct v4l2_mbus_framefmt *format;
	struct csiphy_dev_t *csiphy_dev = v4l2_get_subdevdata(sd);

	format = csiphy_subdev_get_padfmt(csiphy_dev, cfg, fmt);
	if (format == NULL)
		return -EINVAL;

	*format = fmt->format;
	format->field = V4L2_FIELD_NONE;

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int csiphy_subdev_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_format *fmt)
#else
static int csiphy_subdev_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
#endif
{
	struct v4l2_mbus_framefmt *format;
	struct csiphy_dev_t *csiphy_dev = v4l2_get_subdevdata(sd);

	format = csiphy_subdev_get_padfmt(csiphy_dev, cfg, fmt);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int csiphy_subdev_log_status(struct v4l2_subdev *sd)
{
	struct csiphy_dev_t *csiphy_dev = v4l2_get_subdevdata(sd);

	dev_info(csiphy_dev->dev, "log status done\n");

	return 0;
}

const struct v4l2_subdev_core_ops csiphy_subdev_core_ops = {
	.log_status = csiphy_subdev_log_status,
};

static const struct v4l2_subdev_video_ops csiphy_subdev_video_ops = {
	.s_stream = csiphy_subdev_set_stream,
};

static const struct v4l2_subdev_pad_ops csiphy_subdev_pad_ops = {
	.enum_mbus_code = csiphy_subdev_enum_mbus_code,
	.get_fmt = csiphy_subdev_get_format,
	.set_fmt = csiphy_subdev_set_format,
};

static const struct v4l2_subdev_ops csiphy_subdev_v4l2_ops = {
	.core = &csiphy_subdev_core_ops,
	.video = &csiphy_subdev_video_ops,
	.pad = &csiphy_subdev_pad_ops,
};

static int csiphy_v4l2_subdev_link_validate(struct media_link *link)
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

static const struct media_entity_operations csiphy_subdev_media_ops = {
	.link_validate = csiphy_v4l2_subdev_link_validate,
};

int aml_csiphy_subdev_register(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = -1;
	struct device *dev = csiphy_dev->dev;
	struct v4l2_subdev *sd = &csiphy_dev->sd;
	struct media_pad *pads = csiphy_dev->pads;
	struct v4l2_device *v4l2_dev = csiphy_dev->v4l2_dev;

	csiphy_dev->formats = csiphy_support_formats;
	csiphy_dev->fmt_cnt = ARRAY_SIZE(csiphy_support_formats);

	v4l2_subdev_init(sd, &csiphy_subdev_v4l2_ops);
	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), AML_CSIPHY_NAME, csiphy_dev->index);

	v4l2_set_subdevdata(sd, csiphy_dev);

	pads[AML_CSIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[AML_CSIPHY_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	sd->entity.ops = &csiphy_subdev_media_ops;

	rtn = media_entity_pads_init(&sd->entity, AML_CSIPHY_PAD_MAX, pads);
	if (rtn) {
		dev_err(dev, "Error init entity pads: %d\n", rtn);
		return rtn;
	}

	rtn = v4l2_device_register_subdev(v4l2_dev, sd);
	if (rtn < 0) {
		dev_err(dev, "Error to register subdev: %d\n", rtn);
		media_entity_cleanup(&sd->entity);
	}

	dev_info(csiphy_dev->dev,
		"Success to register csiphy-%u subdev\n", csiphy_dev->index);

	return rtn;
}

void aml_csiphy_subdev_unregister(struct csiphy_dev_t *csiphy_dev)
{
	struct v4l2_subdev *sd = &csiphy_dev->sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
}

int aml_csiphy_subdev_init(void *c_dev)
{
	int rtn = -1;
	struct platform_device *pdev;
	struct csiphy_dev_t *csiphy_dev;
	struct device_node *node;
	struct cam_device *cam_dev = c_dev;

	csiphy_dev = &cam_dev->csiphy_dev;

	node = of_parse_phandle(cam_dev->dev->of_node, "csiphy", 0);
	if (!node) {
		pr_err("Failed to parse csiphy handle\n");
		return rtn;
	}

	csiphy_dev->pdev = of_find_device_by_node(node);
	if (!csiphy_dev->pdev) {
		of_node_put(node);
		pr_err("Failed to find csiphy platform device");
		return rtn;
	}
	of_node_put(node);

	pdev = csiphy_dev->pdev;
	csiphy_dev->dev = &pdev->dev;
	csiphy_dev->v4l2_dev = &cam_dev->v4l2_dev;
	csiphy_dev->notifier = &cam_dev->notifier;
	csiphy_dev->index = cam_dev->index;
	platform_set_drvdata(pdev, csiphy_dev);

	rtn = csiphy_of_parse_ports(csiphy_dev);
	if (rtn) {
		dev_err(csiphy_dev->dev, "Failed to parse port\n");
		return -ENODEV;
	}

	rtn = csiphy_of_parse_version(csiphy_dev);
	if (rtn) {
		dev_err(csiphy_dev->dev, "Failed to parse version\n");
		return rtn;
	}

	rtn = csiphy_of_parse_dev(csiphy_dev);
	if (rtn) {
		dev_err(csiphy_dev->dev, "Failed to parse dev\n");
		return rtn;
	}


	dev_pm_domain_attach(csiphy_dev->dev, true);
	pm_runtime_enable(csiphy_dev->dev);
	pm_runtime_get_sync(csiphy_dev->dev);

	of_clk_set_defaults(node, false);
	clk_set_rate(csiphy_dev->mipi_clk0, 200000000);
	rtn = clk_prepare_enable(csiphy_dev->mipi_clk0);
	if (rtn) {
		dev_err(csiphy_dev->dev, "Failed to enable csiphy-%u clk\n", csiphy_dev->index);
		csiphy_iounmap_resource(csiphy_dev);
		csiphy_notifier_cleanup(csiphy_dev);
		return rtn;
	}

	dev_info(csiphy_dev->dev, "Success csiphy-%u subdev init\n", csiphy_dev->index);

	return rtn;
}


void aml_csiphy_subdev_deinit(void *c_dev)
{
	struct csiphy_dev_t *csiphy_dev;
	struct cam_device *cam_dev = c_dev;

	csiphy_dev = &cam_dev->csiphy_dev;

	csiphy_iounmap_resource(csiphy_dev);

	csiphy_notifier_cleanup(csiphy_dev);

	clk_disable_unprepare(csiphy_dev->mipi_clk0);

	pm_runtime_put_sync(csiphy_dev->dev);
	pm_runtime_disable(csiphy_dev->dev);
	dev_pm_domain_detach(csiphy_dev->dev, true);

	dev_info(csiphy_dev->dev, "Success csiphy-%u subdev deinit\n", csiphy_dev->index);
}
