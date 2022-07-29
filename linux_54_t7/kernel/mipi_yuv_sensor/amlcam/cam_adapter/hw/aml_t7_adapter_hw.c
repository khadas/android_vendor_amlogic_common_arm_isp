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
#include <linux/delay.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
#include <linux/dma-contiguous.h>
#endif
#include "../aml_t7_adapter.h"
#include "aml_t7_adapter_hw.h"
#include "aml_t7_misc.h"

static int ceil_upper(int val, int mod)
{
	int ret = -1;

	if ((val == 0) || (mod == 0)) {
		pr_err("Error input a invalid value\n");
		return ret;
	}

	ret = val % mod ? ((val / mod) + 1) : (val / mod);

	return ret;
}

static void __iomem *module_get_base(void *a_dev, int module)
{
	void __iomem *m_base = NULL;
	struct adapter_dev_t *adap_dev = a_dev;

	switch (module) {
	case FRONTEND_MD:
		m_base = adap_dev->adap;
	break;
	case ISPTOP_MD:
		m_base = adap_dev->isp_top;
	break;
	default:
		pr_err("Error module: %d\n", module);
	break;
	}

	return m_base;
}

static int module_reg_write(void *a_dev, int module, u32 addr, u32 val)
{
	int rtn = -1;
	void __iomem *m_base = NULL;

	m_base = module_get_base(a_dev, module);
	if (!m_base) {
		pr_err("Failed to get %d module base\n", module);
		return rtn;
	}

	writel(val, m_base + addr);

	return 0;
}

static int module_reg_read(void *a_dev, int module, u32 addr, u32 *val)
{
	int rtn = -1;
	void __iomem *m_base = NULL;

	m_base = module_get_base(a_dev, module);
	if (!m_base) {
		pr_err("Failed to get %d module base\n", module);
		return rtn;
	}

	*val = readl(m_base + addr);

	return 0;
}

static int module_update_bits(void *a_dev, int module,
				u32 addr, u32 val, u32 start, u32 len)
{
	int rtn = -1;
	u32 mask = 0;
	u32 orig = 0;
	u32 temp = 0;

	if (start + len > 32) {
		pr_err("Error input start and len\n");
		return 0;
	} else if (start == 0 && len == 32) {
		rtn = module_reg_write(a_dev, module, addr, val);
		return rtn;
	}

	rtn = module_reg_read(a_dev, module, addr, &orig);
	if (rtn) {
		pr_err("Error to read: addr 0x%x\n", addr);
		return rtn;
	}

	mask = ((1 << len) - 1) << start;
	val = (val << start);

	temp = orig & (~mask);
	temp |= val & mask;

	if (temp != orig)
		rtn = module_reg_write(a_dev, module, addr, temp);
	else
		rtn = 0;

	if (rtn)
		pr_err("Error update bits: module %d, addr 0x%08x, temp 0x%08x\n",
					module, addr, temp);

	return rtn;
}

static int adap_get_pixel_depth(void *a_param)
{
	int depth = -1;
	struct adapter_dev_param *param = a_param;

	switch (param->format) {
	case ADAP_RAW6:
		depth = 6;
	break;
	case ADAP_RAW7:
		depth = 7;
	break;
	case ADAP_RAW8:
		depth = 8;
	break;
	case ADAP_RAW10:
		depth = 10;
	break;
	case ADAP_RAW12:
		depth = 12;
	break;
	case ADAP_RAW14:
		depth = 14;
	break;
	case ADAP_YUV422_8BIT:
		depth = 16;
	break;

	default:
		pr_err("Error to support format\n");
	break;
	}

	return depth;
}

/* adapter frontend sub-module cfg */

static int adap_frontend_init(void *a_dev)
{
	int rtn = 0;
	u32 cfg_all_to_mem, pingpong_en;
	u32 cfg_isp2ddr_enable, cfg_isp2comb_enable, vc_mode;
	u32 cfg_line_sup_vs_en,cfg_line_sup_vs_sel,cfg_line_sup_sel;
	u32 reg_lbuf0_vs_sel;
	u32 reg_vfifo_vs_out_pre;
	int module = FRONTEND_MD;
	struct adapter_dev_t *adap_dev = a_dev;
	struct adapter_dev_param *param = &adap_dev->param;

	switch (param->mode) {
	case MODE_MIPI_RAW_SDR_DDR:
		cfg_all_to_mem      = 1;
		pingpong_en         = 0;
		cfg_isp2ddr_enable  = 0;
		cfg_isp2comb_enable = 0;
		vc_mode             = 0x11110000;
		cfg_line_sup_vs_en  = 1;
		cfg_line_sup_vs_sel = 1;
		cfg_line_sup_sel    = 1;
	break ;
	case MODE_MIPI_YUV_SDR_DDR:
		cfg_all_to_mem      = 1;
		pingpong_en         = 0; // no ping-pong mode
		cfg_isp2ddr_enable  = 0; // no isp2ddr
		cfg_isp2comb_enable = 0; // no isp2comb
		vc_mode             = 0x11110000;
		cfg_line_sup_vs_en  = 1; // line sup en 1. enable line supply
		cfg_line_sup_vs_sel = 1; // vsync sel 1 use mem2ddrvsync
		cfg_line_sup_sel    = 1; // line sup sel. 1 - line supply on mem2ddr path
	break;
	default:
		cfg_all_to_mem      = 1;
		pingpong_en         = 0;
		cfg_isp2ddr_enable  = 0;
		cfg_isp2comb_enable = 0;
		vc_mode             = 0x11110000;
		cfg_line_sup_vs_en  = 1;
		cfg_line_sup_vs_sel = 1;
		cfg_line_sup_sel    = 1;
	break;
	}

	module_reg_write(a_dev, module, CSI2_CLK_RESET, 0x6);

	module_reg_write(a_dev, module, CSI2_X_START_END_ISP,
				(param->fe_param.fe_isp_x_end << 16 |
				param->fe_param.fe_isp_x_start << 0));

	module_reg_write(a_dev, module, CSI2_Y_START_END_ISP,
				(param->fe_param.fe_isp_y_end << 16 |
				param->fe_param.fe_isp_y_start << 0));

	module_reg_write(a_dev, module, CSI2_X_START_END_MEM,
				(param->fe_param.fe_mem_x_end << 16 |
				param->fe_param.fe_mem_x_start << 0));

	module_reg_write(a_dev, module, CSI2_Y_START_END_MEM,
				(param->fe_param.fe_mem_y_end << 16 |
				param->fe_param.fe_mem_y_start << 0));

	module_reg_write(a_dev, module, CSI2_DDR_STRIDE_PIX,
				param->fe_param.fe_mem_line_stride << 4);
	module_reg_write(a_dev, module, CSI2_DDR_START_PIX, 0x00000000);
	module_reg_write(a_dev, module, CSI2_DDR_START_PIX_ALT, 0x00000000);


	module_reg_write(a_dev, module, CSI2_DDR_STRIDE_PIX_B,
				param->fe_param.fe_mem_line_stride << 4);
	module_reg_write(a_dev, module, CSI2_DDR_START_PIX_B, 0x00000000);
	module_reg_write(a_dev, module, CSI2_DDR_START_PIX_B_ALT, 0x00000000);

	module_reg_write(a_dev, module, CSI2_DDR_START_OTHER, 0x00000000);
	module_reg_write(a_dev, module, CSI2_DDR_START_OTHER_ALT, 0x00000000);

	module_reg_write(a_dev, module, CSI2_INTERRUPT_CTRL_STAT,
				param->fe_param.fe_int_mask);

	module_reg_write(a_dev, module, CSI2_LINE_SUP_CNTL0,
				cfg_line_sup_vs_sel << 15 |
				cfg_line_sup_vs_en << 16 |
				cfg_line_sup_sel << 17 |
				param->fe_param.fe_mem_line_minbyte << 0);

	module_reg_write(a_dev, module, CSI2_VC_MODE, vc_mode);

	module_reg_write(a_dev, module, CSI2_GEN_CTRL0,
				cfg_all_to_mem << 4 |
				pingpong_en << 5 |
				0x01 << 12 |
				0x07 << 16 | // allow receive raw yuv rgb data
				cfg_isp2ddr_enable << 25 |
				cfg_isp2comb_enable << 26);

	return rtn;
}

static u32 adap_frontend_status(void *a_dev)
{
	u32 val = 0;

	module_reg_read(a_dev, FRONTEND_MD, CSI2_INTERRUPT_CTRL_STAT, &val);
	module_reg_write(a_dev, FRONTEND_MD, CSI2_INTERRUPT_CTRL_STAT, val);

	return val;
}

static void adap_frontend_start(void *a_dev)
{
	module_update_bits(a_dev, FRONTEND_MD, CSI2_GEN_CTRL0, 0xf, 0, 4);
}

static void adap_frontend_stop(void *a_dev)
{
	module_update_bits(a_dev, FRONTEND_MD, CSI2_GEN_CTRL0, 0x0, 0, 4);
}

static void adap_module_reset(void *a_dev)
{
	module_update_bits(a_dev, FRONTEND_MD, CSI2_CLK_RESET, 0x1, 0, 1);

	udelay(10);

	module_update_bits(a_dev, FRONTEND_MD, CSI2_CLK_RESET, 0x0, 0, 1);
}

static void adap_module_top_pending_mask(void *a_dev, u32 val)
{
	struct adapter_dev_t *adap_dev = a_dev;

	switch (adap_dev->index) {
	case 0:
		module_update_bits(a_dev, ISPTOP_MD, MIPI_TOP_ISP_PENDING_MASK0, val, 28, 1);
	break;
	case 1:
		module_update_bits(a_dev, ISPTOP_MD, MIPI_TOP_ISP_PENDING_MASK1, val, 8, 1);
	break;
	}
}

static void adap_module_top_de_bypass(void *a_dev)
{
	module_update_bits(a_dev, ISPTOP_MD, MIPI_TOP_ADAPT_DE_CTRL0, 0x1, 3, 1);
	module_update_bits(a_dev, ISPTOP_MD, MIPI_TOP_ADAPT_DE_CTRL0, 0x1, 7, 1);
}

/* adapter hardware cfg interface */

static int adap_hw_init(void *a_dev)
{
	int rtn = 0;
	struct adapter_dev_t *adap_dev = a_dev;
	struct adapter_dev_param *param = &adap_dev->param;

	param->fe_param.fe_sel = adap_dev->index;
	param->fe_param.fe_work_mode = param->mode;
	param->fe_param.fe_mem_x_start = 0;
	param->fe_param.fe_mem_x_end = param->width - 1;
	param->fe_param.fe_mem_y_start = 0;
	param->fe_param.fe_mem_y_end = param->height - 1;
	param->fe_param.fe_isp_x_start = 0 ;
	param->fe_param.fe_isp_x_end = param->width - 1;
	param->fe_param.fe_isp_y_start = 0;
	param->fe_param.fe_isp_y_end = param->height - 1;

	param->fe_param.fe_mem_line_stride =
		ceil_upper((adap_get_pixel_depth(param) * param->width), (8 * 16));

	param->fe_param.fe_mem_line_minbyte =
		(adap_get_pixel_depth(param) * param->width + 7) >> 3;

	param->fe_param.fe_int_mask = 0x4; // 0xc;

	rtn = adap_frontend_init(a_dev);
	if (rtn)
		return rtn;

	pr_info("Success adap hw init.raw yuv rgb enabled \n");

	return rtn;
}

static int adap_stream_set_fmt(struct aml_video *video, struct aml_format *fmt)
{
	pr_err("adap stream set fmt, empty\n\n");
	return 0;
}

static int adap_stream_cfg_buf(struct aml_video *video, struct aml_buffer *buff)
{
	int module = FRONTEND_MD;
	u32 addr = buff->addr[AML_PLANE_A];

	module_reg_write(video->priv, module, CSI2_DDR_START_PIX, addr);
	module_reg_write(video->priv, module, CSI2_DDR_START_PIX_ALT, addr);

	return 0;
}

static void adap_hw_stream_on(struct aml_video *video)
{
	adap_frontend_start(video->priv);
}

static void adap_hw_stream_off(struct aml_video *video)
{
	adap_frontend_stop(video->priv);
}

static u32 adap_hw_interrupt_status(void *a_dev)
{
	return adap_frontend_status(a_dev);
}

static void adap_hw_reset(void *a_dev)
{

	adap_module_reset(a_dev);

	pr_info("Success adap hw reset\n");
}

static int adap_hw_start(void *a_dev)
{
	adap_module_top_de_bypass(a_dev);
	adap_module_top_pending_mask(a_dev, 1);

	pr_info("Success adap hw start\n");

	return 0;
}

static void adap_hw_stop(void *a_dev)
{
	adap_module_top_pending_mask(a_dev, 0);

	pr_info("Success adap hw stop\n");
}


const struct adapter_dev_ops adap_dev_hw_ops = {
	.hw_init = adap_hw_init,
	.hw_reset = adap_hw_reset,
	.hw_start = adap_hw_start,
	.hw_stop = adap_hw_stop,
	.hw_stream_cfg_buf = adap_stream_cfg_buf,
	.hw_stream_set_fmt = adap_stream_set_fmt,
	.hw_stream_on = adap_hw_stream_on,
	.hw_stream_off = adap_hw_stream_off,
	.hw_interrupt_status = adap_hw_interrupt_status,
};
