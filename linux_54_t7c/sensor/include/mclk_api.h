/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */

#ifndef __MCLK_API__
#define __MCLK_API__
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
#include "../../amlcam/cam_common/aml_misc.h"

static int mclk_enable(struct device *dev)
{
	int ret;
	struct clk *clk = NULL,*clk_pre = NULL,*clk_p = NULL,*clk_x = NULL;
	int clk_val;
	uint32_t rate = 0;

	clk = devm_clk_get(dev, "mclk");
	if (IS_ERR(clk)) {
		dev_err(dev,"cannot get %s clk\n", "mclk");
		clk = NULL;
		return -1;
	}
	ret = fwnode_property_read_u32(dev_fwnode(dev), "clock-frequency",
					&rate);
	if (ret) {
		dev_err(dev, "Could not get xclk frequency\n");
		return -1;
	}

	if (rate == 24000000) {
		clk_x = devm_clk_get(dev, "mclk_x");
		if (IS_ERR(clk_x) == 0)
			clk_set_parent(clk, clk_x);
		clk_pre = devm_clk_get(dev, "mclk_pre");
	} else {
		clk_p = devm_clk_get(dev, "mclk_p");
		if (IS_ERR(clk_p) == 0) {
			clk_set_parent(clk, clk_p);
			clk_set_rate(clk_p, rate * 2);
		}
	}

	ret = clk_set_rate(clk, rate);
	if (ret < 0)
		dev_err(dev, "clk_set_rate failed\n");
	udelay(30);

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		dev_err(dev, " clk_prepare_enable failed\n");

	if (IS_ERR(clk_pre) == 0) {
		clk_disable_unprepare(clk_pre);
	}

	clk_val = clk_get_rate(clk);
	dev_err(dev,"init mclk is %d MHZ\n",clk_val/1000000);

	return 0;
}

static int mclk_disable(struct device *dev)
{
    struct clk *clk;
    int clk_val;

    clk = devm_clk_get(dev, "mclk");
    if (IS_ERR(clk)) {
        pr_err("cannot get %s clk\n", "mclk");
        clk = NULL;
        return -1;
    }

    clk_val = clk_get_rate(clk);
    if ((clk_val != 12000000) && (clk_val != 24000000)) {
        clk_disable_unprepare(clk);
    }

    devm_clk_put(dev, clk);

    pr_info("Success disable mclk: %d\n", clk_val);

    return 0;
}

#endif