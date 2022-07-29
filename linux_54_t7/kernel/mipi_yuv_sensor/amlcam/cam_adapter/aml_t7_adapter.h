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

#ifndef __AML_T7_ADAPTER_H__
#define __AML_T7_ADAPTER_H__

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>

#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/media-entity.h>

#include "aml_t7_video.h"

#define ADAP_DDR_BUFF_CNT 4
#define ADAP_DOL_BUFF_CNT 2
#define ADAP_ALIGN(data, val) ((data + val - 1) & (~(val - 1)))

enum {
	MODE_MIPI_RAW_SDR_DDR = 0,
	MODE_MIPI_RAW_SDR_DIRCT,
	MODE_MIPI_RAW_HDR_DDR_DDR,
	MODE_MIPI_RAW_HDR_DDR_DIRCT,
	MODE_MIPI_YUV_SDR_DDR,
	MODE_MIPI_RGB_SDR_DDR,
	MODE_MIPI_YUV_SDR_DIRCT,
	MODE_MIPI_RGB_SDR_DIRCT
};

enum {
	AML_ADAP_PAD_SINK = 0,
	AML_ADAP_PAD_SRC,
	AML_ADAP_PAD_MAX,
};

enum {
	AML_ADAP_STREAM_RAW,
	AML_ADAP_STREAM_MAX
};

enum {
	ADAP_PATH0 = 0,
	ADAP_PATH1,
	ADAP_PATH_MAX
};

enum {
	ADAP_DDR_MODE = 0,
	ADAP_DIR_MODE,
	ADAP_DOL_MODE,
	ADAP_MODE_MAX
};

#define ADAP_YUV422_8BIT  0x1e
#define ADAP_YUV422_10BIT 0x1f
#define ADAP_RGB444       0x20
#define ADAP_RGB555       0x21
#define ADAP_RGB565       0x22
#define ADAP_RGB666       0x23
#define ADAP_RGB888       0x24
#define ADAP_RAW6         0x28
#define ADAP_RAW7         0x29
#define ADAP_RAW8         0x2a
#define ADAP_RAW10        0x2b
#define ADAP_RAW12        0x2c
#define ADAP_RAW14        0x2d

enum {
	ADAP_DOL_NONE = 0,
	ADAP_DOL_VC,
	ADAP_DOL_LINEINFO,
	ADAP_DOL_YUV,
	ADAP_DOL_MAX
};

struct adap_regval {
	u32 reg;
	u32 val;
};

struct adap_exp_offset {
	int long_offset;
	int short_offset;
	int offset_x;
	int offset_y;
};

struct fe_param_t{
   int fe_sel;//add for sel 7 mipi_isp_top
   int fe_cfg_ddr_max_bytes_other;
   int fe_dec_ctrl0;
   int fe_dec_ctrl1;
   int fe_dec_ctrl2;
   int fe_dec_ctrl3;
   int fe_dec_ctrl4;

   int fe_work_mode;
   int fe_mem_x_start;
   int fe_mem_x_end;
   int fe_mem_y_start;
   int fe_mem_y_end;
   int fe_isp_x_start;
   int fe_isp_x_end;
   int fe_isp_y_start;
   int fe_isp_y_end;
   int fe_mem_ping_addr;
   int fe_mem_pong_addr;
   int fe_mem_other_addr;
   int fe_mem_line_stride;
   int fe_mem_line_minbyte;
   int fe_int_mask;
};

struct adapter_dev_param {
	int path;
	int mode;
	u32 width;
	u32 height;
	int format;
	int dol_type;
	struct fe_param_t fe_param;
	struct adap_exp_offset offset;
};

struct adapter_dev_ops {
	int (*hw_init)(void *a_dev);
	void (*hw_reset)(void *a_dev);
	int (*hw_start)(void *a_dev);
	void (*hw_stop)(void *a_dev);
	u32 (*hw_interrupt_status)(void *a_dev);
	int (*hw_stream_set_fmt)(struct aml_video *video, struct aml_format *fmt);
	int (*hw_stream_cfg_buf)(struct aml_video *video, struct aml_buffer *buff);
	void (*hw_stream_on)(struct aml_video *video);
	void (*hw_stream_off)(struct aml_video *video);
};

struct adapter_dev_t {
	u32 index;
	u32 version;
	char *bus_info;
	struct device *dev;
	struct platform_device *pdev;

	int irq;
	void __iomem *adap;
	void __iomem *isp_top;
	struct clk *adap_clk;

	struct v4l2_subdev sd;
	struct media_pad pads[AML_ADAP_PAD_MAX];
	struct v4l2_mbus_framefmt pfmt[AML_ADAP_PAD_MAX];
	unsigned int fmt_cnt;
	const struct aml_format *formats;
	struct v4l2_device *v4l2_dev;
	struct media_pipeline pipe;

	struct adapter_dev_param param;
	const struct adapter_dev_ops *ops;

	struct aml_video video[AML_ADAP_STREAM_MAX];
};

int aml_adap_subdev_init(void *c_dev);
void aml_adap_subdev_deinit(void *c_dev);
int aml_adap_video_register(struct adapter_dev_t *adap_dev);
void aml_adap_video_unregister(struct adapter_dev_t *adap_dev);
int aml_adap_subdev_register(struct adapter_dev_t *adap_dev);
void aml_adap_subdev_unregister(struct adapter_dev_t *adap_dev);

extern const struct adapter_dev_ops adap_dev_hw_ops;

#endif /* __AML_P1_ADAPTER_H__ */
