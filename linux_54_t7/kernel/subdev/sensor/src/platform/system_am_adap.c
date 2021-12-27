/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2018 Amlogic or its affiliates
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
#define pr_fmt(fmt) "AM_ADAP: " fmt

#include "system_am_adap.h"
#include <linux/irqreturn.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-contiguous.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_fdt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <uapi/linux/sched/types.h>
#include "acamera_command_api.h"
#include "acamera_logger.h"
#include "system_log.h"

#define AM_ADAPTER_NAME "amlogic, isp-adapter"
#define BOUNDRY             16
#define FRONTEDN_USED       4
#define RD_USED             1

struct am_adap_fsm_t {
    uint8_t cam_en;
    int control_flag;
    int wbuf_index;

    int next_buf_index;
    resource_size_t read_buf;
    resource_size_t buffer_start;
    resource_size_t ddr_buf[DDR_BUF_SIZE];
    struct kfifo adapt_fifo;

    struct page *cma_pages;

    struct am_adap_info para;
};

struct am_adap_fsm_t adap_fsm[CAMERA_NUM];
uint32_t camera_frame_fifo[CAMERA_QUEUE_NUM];

resource_size_t dol_buf[DOL_BUF_SIZE];

struct am_adap *g_adap = NULL;
struct adaptfe_param fe_param[FRONTEDN_USED];
struct adaptrd_param rd_param;
struct adaptpixel_param pixel_param;
struct adaptalig_param alig_param;

struct kfifo adapt_fifo;

/*we allocte from CMA*/

#define DEFAULT_ADAPTER_BUFFER_SIZE 24

static int ceil_upper(int val, int mod)
{
    int ret = 0;
    if ((val == 0) || (mod == 0)) {
        pr_info("input a invalid value.\n");
        return 0;
    } else {
        if ((val % mod) == 0) {
            ret = (val/mod);
        } else {
            ret = ((val/mod) + 1);
        }
    }
    return ret;
}

int write_index_to_file(char *path, char *buf, int index, int size)
{
    int ret = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t pos = 0;
    int nwrite = 0;
    int offset = 0;
    char file_name[150];

    sprintf(file_name, "%s%s%d%s", path, "adapt_img_", index, ".raw");
    /* change to KERNEL_DS address limit */
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    /* open file to write */
    fp = filp_open(file_name, O_WRONLY|O_CREAT, 0640);
    if (!fp) {
       printk("%s: open file error\n", __FUNCTION__);
       ret = -1;
       goto exit;
    }

    pos=(unsigned long)offset;

    /* Write buf to file */
    nwrite=vfs_write(fp, buf, size, &pos);
    offset +=nwrite;

    if (fp) {
        filp_close(fp, NULL);
    }

exit:
    set_fs(old_fs);
    return ret;
}

void inline update_wr_reg_bits(unsigned int reg,
                adap_io_type_t io_type, unsigned int mask,
                unsigned int val)
{
    unsigned int tmp, orig;
    void __iomem *base = NULL;
    void __iomem *reg_addr = NULL;
    switch (io_type) {
        case FRONTEND0_IO:
            base = g_adap->base_addr + FRONTEND0_BASE;
            break;
        case FRONTEND1_IO:
            base = g_adap->base_addr + FRONTEND1_BASE;
            break;
        case FRONTEND2_IO:
            base = g_adap->base_addr + FRONTEND2_BASE;
            break;
        case FRONTEND3_IO:
            base = g_adap->base_addr + FRONTEND3_BASE;
            break;
        case RD_IO:
            base = g_adap->base_addr + RD_BASE;
            break;
        case PIXEL_IO:
            base = g_adap->base_addr + PIXEL_BASE;
            break;
        case ALIGN_IO:
            base = g_adap->base_addr + ALIGN_BASE;
            break;
        case MISC_IO:
            base = g_adap->base_addr + MISC_BASE;
            break;
        case CMPR_CNTL_IO:
            base = g_adap->base_addr + CMPR_CNTL_BASE;
            break;
        case CMPR_IO:
            base = g_adap->base_addr + CMPR_BASE;
            break;
        default:
            pr_err("adapter error io type.\n");
            base = NULL;
            break;
    }
    if (base !=  NULL) {
        reg_addr = base + reg;
        if (io_type == CMPR_CNTL_IO)
            reg_addr = base + (reg - CMPR_CNTL_ADDR);
        if (io_type == CMPR_IO)
            reg_addr = base + (reg - CMPR_BASE_ADDR);
        orig = readl(reg_addr);
        tmp = orig & ~mask;
        tmp |= val & mask;
        writel(tmp, reg_addr);
    }
}

void inline adap_wr_bit(unsigned int adr,
        adap_io_type_t io_type, unsigned int val,
        unsigned int start, unsigned int len)
{
    unsigned long flags;

    spin_lock_irqsave(&g_adap->reg_lock, flags);

    update_wr_reg_bits(adr, io_type,
                   ((1<<len)-1)<<start, val<<start);
    spin_unlock_irqrestore(&g_adap->reg_lock, flags);
}

void inline adap_write(int addr, adap_io_type_t io_type, uint32_t val)
{
    void __iomem *base_reg_addr = NULL;
    void __iomem *reg_addr = NULL;
    unsigned long flags;

    spin_lock_irqsave(&g_adap->reg_lock, flags);

    switch (io_type) {
        case FRONTEND0_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND0_BASE;
            break;
        case FRONTEND1_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND1_BASE;
            break;
        case FRONTEND2_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND2_BASE;
            break;
        case FRONTEND3_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND3_BASE;
            break;
        case RD_IO:
            base_reg_addr = g_adap->base_addr + RD_BASE;
            break;
        case PIXEL_IO:
            base_reg_addr = g_adap->base_addr + PIXEL_BASE;
            break;
        case ALIGN_IO:
            base_reg_addr = g_adap->base_addr + ALIGN_BASE;
            break;
        case MISC_IO:
            base_reg_addr = g_adap->base_addr + MISC_BASE;
            break;
        case CMPR_CNTL_IO:
            base_reg_addr = g_adap->base_addr + CMPR_CNTL_BASE;
            break;
        case CMPR_IO:
            base_reg_addr = g_adap->base_addr + CMPR_BASE;
            break;
        default:
            pr_err("adapter error io type.\n");
            base_reg_addr = NULL;
            break;
    }
    if (base_reg_addr != NULL) {
        reg_addr = base_reg_addr + addr;
        if (io_type == CMPR_CNTL_IO)
            reg_addr = base_reg_addr + (addr - CMPR_CNTL_ADDR);
        if (io_type == CMPR_IO)
            reg_addr = base_reg_addr + (addr - CMPR_BASE_ADDR);
        writel(val, reg_addr);
    } else
        pr_err("mipi adapter write register failed.\n");

    spin_unlock_irqrestore(&g_adap->reg_lock, flags);
}

void inline adap_read(int addr, adap_io_type_t io_type, uint32_t *val)
{
    void __iomem *base_reg_addr = NULL;
    void __iomem *reg_addr = NULL;
    unsigned long flags;

    spin_lock_irqsave(&g_adap->reg_lock, flags);

    switch (io_type) {
        case FRONTEND0_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND0_BASE;
            break;
        case FRONTEND1_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND1_BASE;
            break;
        case FRONTEND2_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND2_BASE;
            break;
        case FRONTEND3_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND3_BASE;
            break;
        case RD_IO:
            base_reg_addr = g_adap->base_addr + RD_BASE;
            break;
        case PIXEL_IO:
            base_reg_addr = g_adap->base_addr + PIXEL_BASE;
            break;
        case ALIGN_IO:
            base_reg_addr = g_adap->base_addr + ALIGN_BASE;
            break;
        case MISC_IO:
            base_reg_addr = g_adap->base_addr + MISC_BASE;
            break;
        case CMPR_CNTL_IO:
            base_reg_addr = g_adap->base_addr + CMPR_CNTL_BASE;
            break;
        case CMPR_IO:
            base_reg_addr = g_adap->base_addr + CMPR_BASE;
            break;
        default:
            pr_err("%s, adapter error io type.\n", __func__);
            base_reg_addr = NULL;
            break;
    }
    if (base_reg_addr != NULL) {
        reg_addr = base_reg_addr + addr;
        if (io_type == CMPR_CNTL_IO)
            reg_addr = base_reg_addr + (addr - CMPR_CNTL_ADDR);
        if (io_type == CMPR_IO)
            reg_addr = base_reg_addr + (addr - CMPR_BASE_ADDR);
        *val = readl(reg_addr);
    } else
        pr_err("mipi adapter read register failed.\n");

    spin_unlock_irqrestore(&g_adap->reg_lock, flags);
}

void adap_read_ext(int addr, adap_io_type_t io_type, uint32_t *val)
{
    void __iomem *base_reg_addr = NULL;
    void __iomem *reg_addr = NULL;
    switch (io_type) {
        case FRONTEND0_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND0_BASE;
            break;
        case FRONTEND1_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND1_BASE;
            break;
        case FRONTEND2_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND2_BASE;
            break;
        case FRONTEND3_IO:
            base_reg_addr = g_adap->base_addr + FRONTEND3_BASE;
            break;
        case RD_IO:
            base_reg_addr = g_adap->base_addr + RD_BASE;
            break;
        case PIXEL_IO:
            base_reg_addr = g_adap->base_addr + PIXEL_BASE;
            break;
        case ALIGN_IO:
            base_reg_addr = g_adap->base_addr + ALIGN_BASE;
            break;
        case MISC_IO:
            base_reg_addr = g_adap->base_addr + MISC_BASE;
            break;
        case CMPR_CNTL_IO:
            base_reg_addr = g_adap->base_addr + CMPR_CNTL_BASE;
            break;
        default:
            pr_err("%s, adapter error io type.\n", __func__);
            base_reg_addr = NULL;
            break;
    }
    if (base_reg_addr != NULL) {
        reg_addr = base_reg_addr + addr;
        *val = readl(reg_addr);
    } else
        pr_err("mipi adapter read register failed.\n");

}

int am_adap_parse_dt(struct device_node *node)
{
    int rtn = -1;
    int irq = -1;
    int ret = 0;
    struct resource rs;
    struct am_adap *t_adap = NULL;

    if (node == NULL) {
        pr_err("%s: Error input param\n", __func__);
        return -1;
    }

    rtn = of_device_is_compatible(node, AM_ADAPTER_NAME);
    if (rtn == 0) {
        pr_err("%s: Error match compatible\n", __func__);
        return -1;
    }

    t_adap = kzalloc(sizeof(*t_adap), GFP_KERNEL);
    if (t_adap == NULL) {
        pr_err("%s: Failed to alloc adapter\n", __func__);
        return -1;
    }

    t_adap->of_node = node;
    t_adap->f_end_irq = 0;
    t_adap->f_fifo = 0;
    t_adap->f_adap = 0;

    rtn = of_address_to_resource(node, 0, &rs);
    if (rtn != 0) {
        pr_err("%s:Error get adap reg resource\n", __func__);
        goto reg_error;
    }

    //pr_info("%s: rs idx info: name: %s\n", __func__, rs.name);
    if (strcmp(rs.name, "adapter") == 0) {
        t_adap->reg = rs;
        t_adap->base_addr =
                ioremap_nocache(t_adap->reg.start, 0x5800);//resource_size(&t_adap->reg));
    }

    irq = irq_of_parse_and_map(node, 0);
    if (irq <= 0) {
        pr_err("%s:Error get adap irq\n", __func__);
        goto irq_error;
    }

    t_adap->rd_irq = irq;
    //pr_info("%s:adapt irq: %d\n", __func__, t_adap->rd_irq);

    t_adap->p_dev = of_find_device_by_node(node);
    ret = of_reserved_mem_device_init(&(t_adap->p_dev->dev));
    if (ret != 0) {
        pr_err("adapt reserved mem device init failed.\n");
        return ret;
    }

    ret = of_property_read_u32(t_adap->p_dev->dev.of_node, "mem_alloc",
        &(t_adap->adap_buf_size));
    pr_info("adapter alloc %dM memory\n", t_adap->adap_buf_size);
    if (ret != 0) {
        pr_err("failed to get adap-buf-size from dts, use default value\n");
        t_adap->adap_buf_size = DEFAULT_ADAPTER_BUFFER_SIZE;
    }

    t_adap->adap_buf_size = t_adap->adap_buf_size / CAMERA_NUM;

    system_dbg_create(&(t_adap->p_dev->dev));

    g_adap = t_adap;

    spin_lock_init(&g_adap->reg_lock);

    adap_fsm[0].cam_en = CAM_DIS;
    adap_fsm[1].cam_en = CAM_DIS;

    return 0;

irq_error:
    iounmap(t_adap->base_addr);
    t_adap->base_addr = NULL;


reg_error:
    if (t_adap != NULL)
        kfree(t_adap);
    return -1;
}

void am_adap_deinit_parse_dt(void)
{
    struct am_adap *t_adap = NULL;

    t_adap = g_adap;

    if (t_adap == NULL || t_adap->p_dev == NULL ||
                t_adap->base_addr == NULL) {
        pr_err("Error input param\n");
        return;
    }

    system_dbg_remove(&(t_adap->p_dev->dev));

    iounmap(t_adap->base_addr);
    t_adap->base_addr = NULL;

    kfree(t_adap);
    t_adap = NULL;
    g_adap = NULL;

    pr_info("Success deinit parse adap module\n");
}


void am_adap_set_info(struct am_adap_info *info)
{
    memset(&adap_fsm[info->path].para, 0, sizeof(struct am_adap_info));
    memcpy(&adap_fsm[info->path].para, info, sizeof(struct am_adap_info));
}

int am_adap_get_depth(uint8_t channel)
{
    int depth = 0;
    switch (adap_fsm[channel].para.fmt) {
        case MIPI_CSI_RAW6:
            depth = 6;
            break;
        case MIPI_CSI_RAW7:
            depth = 7;
            break;
        case MIPI_CSI_RAW8:
            depth = 8;
            break;
        case MIPI_CSI_RAW10:
            depth = 10;
            break;
        case MIPI_CSI_RAW12:
            depth = 12;
            break;
        case MIPI_CSI_RAW14:
            depth = 14;
            break;
        case MIPI_CSI_YUV422_8BIT:
            depth = 16;
            break;
        default:
            pr_err("Not supported data format.\n");
            break;
    }
    return depth;
}

int am_disable_irq(uint8_t channel)
{
    //disable irq mask
    //adap_write(CSI2_INTERRUPT_CTRL_STAT, FRONTEND0_IO, 0);
    if (channel == CAM0_ACT) {
        adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 0, FRONT0_WR_DONE, 1);
    } else if (channel == CAM1_ACT) {
        adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 0, FRONT2_WR_DONE, 1);
    } else {
        adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 0, FRONT3_WR_DONE, 1);
    }

    //adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 0, 9, 1 );//rd0 done

    return 0;
}

/*
 *========================AM ADAPTER FRONTEND INTERFACE========================
 */

void am_adap_frontend_start(uint8_t channel)
{
    int fe_io = 0;
    switch (fe_param[channel].fe_sel) {
        case 0:
            fe_io = FRONTEND0_IO;
            break;
        case 1:
            fe_io = FRONTEND1_IO;
            break;
        case 2:
            fe_io = FRONTEND2_IO;
            break;
        case 3:
            fe_io = FRONTEND3_IO;
            break;
        default:
            break;
    }
    adap_wr_bit(CSI2_GEN_CTRL0, fe_io, 0xf, 0, 4);
}


int am_adap_frontend_init(struct adaptfe_param* prm, uint8_t channel) {
    uint32_t fe_io = FRONTEND0_IO;
    switch (prm->fe_sel) {
        case 0:
            fe_io = FRONTEND0_IO;
            break;
        case 1:
            fe_io = FRONTEND1_IO;
            break;
        case 2:
            fe_io = FRONTEND2_IO;
            break;
        case 3:
            fe_io = FRONTEND3_IO;
            break;
        default:
            break;
    }

    uint32_t cfg_all_to_mem,pingpong_en,cfg_isp2ddr_enable,cfg_isp2comb_enable,vc_mode;
    uint32_t cfg_line_sup_vs_en,cfg_line_sup_vs_sel,cfg_line_sup_sel;
    uint32_t mem_addr0_a,mem_addr0_b,mem_addr1_a,mem_addr1_b;

    if (adap_fsm[channel].para.mode == DDR_MODE) {
        prm->fe_mem_ping_addr = adap_fsm[channel].ddr_buf[0];
        prm->fe_mem_pong_addr = adap_fsm[channel].ddr_buf[0];
        prm->fe_mem_other_addr= adap_fsm[channel].ddr_buf[1];
    } else if (adap_fsm[channel].para.mode == DOL_MODE) {
        prm->fe_mem_ping_addr = dol_buf[0];
        prm->fe_mem_pong_addr = dol_buf[1];
        prm->fe_mem_other_addr= dol_buf[1];
    } else {
        prm->fe_mem_ping_addr = adap_fsm[channel].ddr_buf[0];
        prm->fe_mem_pong_addr = adap_fsm[channel].ddr_buf[0];
        prm->fe_mem_other_addr= adap_fsm[channel].ddr_buf[1];
    }

    mem_addr0_a = prm->fe_mem_ping_addr;
    mem_addr0_b = prm->fe_mem_pong_addr;
    mem_addr1_a = prm->fe_mem_ping_addr;
    mem_addr1_b = prm->fe_mem_pong_addr;

    switch (prm->fe_work_mode) {
        case MODE_MIPI_RAW_SDR_DDR:
            cfg_all_to_mem        = 1;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 1;
            cfg_line_sup_vs_sel = 1;
            cfg_line_sup_sel    = 1;
            break;
        case MODE_MIPI_RAW_SDR_DIRCT:
            cfg_all_to_mem        = 0;
            pingpong_en            = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 0;
            cfg_line_sup_vs_sel = 0;
            cfg_line_sup_sel    = 0;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DDR:
            cfg_all_to_mem        = 0;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 1;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x111100f1;
            cfg_line_sup_vs_en  = 1;
            cfg_line_sup_vs_sel = 1;
            cfg_line_sup_sel    = 1;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DIRCT:
            cfg_all_to_mem        = 0;
            pingpong_en         = 1;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x111100f1;
            cfg_line_sup_vs_en  = 1;
            cfg_line_sup_vs_sel = 1;
            cfg_line_sup_sel    = 1;
            break;
        case MODE_MIPI_YUV_SDR_DDR:
            cfg_all_to_mem        = 1;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 0;
            cfg_line_sup_vs_sel = 0;
            cfg_line_sup_sel    = 0;
            break;
        case MODE_MIPI_RGB_SDR_DDR:
            cfg_all_to_mem        = 1;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 0;
            cfg_line_sup_vs_sel = 0;
            cfg_line_sup_sel    = 0;
            break;
        case MODE_MIPI_YUV_SDR_DIRCT:
            cfg_all_to_mem        = 0;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 1;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 0;
            cfg_line_sup_vs_sel = 0;
            cfg_line_sup_sel    = 0;
            break;
        case MODE_MIPI_RGB_SDR_DIRCT:
            cfg_all_to_mem        = 0;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 1;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 0;
            cfg_line_sup_vs_sel = 0;
            cfg_line_sup_sel    = 0;
            break;
        default:
            cfg_all_to_mem        = 0;
            pingpong_en         = 0;
            cfg_isp2ddr_enable  = 0;
            cfg_isp2comb_enable = 0;
            vc_mode             = 0x11110000;
            cfg_line_sup_vs_en  = 0;
            cfg_line_sup_vs_sel = 0;
            cfg_line_sup_sel    = 0;
            break;
    }

    adap_write(CSI2_CLK_RESET          , fe_io, 0x0);
    adap_write(CSI2_CLK_RESET          , fe_io, 0x6);
    adap_write(CSI2_X_START_END_ISP    , fe_io, (prm->fe_isp_x_end << 16 |
                                         prm->fe_isp_x_start << 0));
    adap_write(CSI2_Y_START_END_ISP    , fe_io, (prm->fe_isp_y_end << 16 |
                                         prm->fe_isp_y_start << 0));
    adap_write(CSI2_X_START_END_MEM    , fe_io, (prm->fe_mem_x_end << 16 |
                                         prm->fe_mem_x_start << 0));
    adap_write(CSI2_Y_START_END_MEM    , fe_io, (prm->fe_mem_y_end << 16 |
                                         prm->fe_mem_y_start << 0));
    adap_write(CSI2_DDR_START_PIX      , fe_io, mem_addr0_a);
    adap_write(CSI2_DDR_START_PIX_ALT  , fe_io, mem_addr0_b);
    adap_write(CSI2_DDR_START_PIX_B    , fe_io, mem_addr1_a);
    adap_write(CSI2_DDR_START_PIX_B_ALT, fe_io, mem_addr1_b);

    adap_write(CSI2_DDR_START_OTHER    , fe_io, prm->fe_mem_other_addr);
    adap_write(CSI2_DDR_START_OTHER_ALT, fe_io, prm->fe_mem_other_addr);
    adap_write(CSI2_DDR_STRIDE_PIX     , fe_io, prm->fe_mem_line_stride << 4);
    adap_write(CSI2_DDR_STRIDE_PIX_B   , fe_io, prm->fe_mem_line_stride << 4);
    adap_write(CSI2_INTERRUPT_CTRL_STAT, fe_io, prm->fe_int_mask);
    adap_write(CSI2_LINE_SUP_CNTL0     , fe_io, cfg_line_sup_vs_sel << 15 |
                                         cfg_line_sup_vs_en << 16 |
                                         cfg_line_sup_sel << 17 |
                                         prm->fe_mem_line_minbyte << 0);
    adap_write(CSI2_VC_MODE            , fe_io, vc_mode);
    adap_write(CSI2_GEN_CTRL0          , fe_io, cfg_all_to_mem  << 4 |
                                         pingpong_en            << 5 |
                                         0x01                    << 12 |
                                         0x03                    << 16 |
                                         cfg_isp2ddr_enable     << 25 |
                                         cfg_isp2comb_enable    << 26);

    if (prm->fe_work_mode == MODE_MIPI_RAW_HDR_DDR_DIRCT) {
        if (adap_fsm[channel].para.type == DOL_LINEINFO) {
            adap_write(CSI2_VC_MODE, fe_io, 0x11110052);
            if (adap_fsm[channel].para.fmt == MIPI_CSI_RAW10 ) {
                adap_write(CSI2_VC_MODE2_MATCH_MASK_A_L, fe_io, 0x6e6e6e6e);
                adap_write(CSI2_VC_MODE2_MATCH_MASK_A_H, fe_io, 0xffffff00);
                adap_write(CSI2_VC_MODE2_MATCH_A_L, fe_io, 0x90909090);
                adap_write(CSI2_VC_MODE2_MATCH_A_H, fe_io, 0x55);
                adap_write(CSI2_VC_MODE2_MATCH_MASK_B_L, fe_io, 0x6e6e6e6e);
                adap_write(CSI2_VC_MODE2_MATCH_MASK_B_H, fe_io, 0xffffff00);
                adap_write(CSI2_VC_MODE2_MATCH_B_L, fe_io, 0x90909090);
                adap_write(CSI2_VC_MODE2_MATCH_B_H, fe_io, 0xaa);
            } else if (adap_fsm[channel].para.fmt == MIPI_CSI_RAW12) {
                adap_write(CSI2_VC_MODE2_MATCH_MASK_A_L, fe_io, 0xff000101);
                adap_write(CSI2_VC_MODE2_MATCH_MASK_A_H, fe_io, 0xffffffff);
                adap_write(CSI2_VC_MODE2_MATCH_A_L, fe_io, 0x112424);
                adap_write(CSI2_VC_MODE2_MATCH_A_H, fe_io, 0x0);
                adap_write(CSI2_VC_MODE2_MATCH_MASK_B_L, fe_io, 0xff000101);
                adap_write(CSI2_VC_MODE2_MATCH_MASK_B_H, fe_io, 0xffffffff);
                adap_write(CSI2_VC_MODE2_MATCH_B_L, fe_io, 0x222424);
                adap_write(CSI2_VC_MODE2_MATCH_B_H, fe_io, 0x0);
            }
            int long_exp_offset = adap_fsm[channel].para.offset.long_offset;
            int short_exp_offset = adap_fsm[channel].para.offset.short_offset;
            adap_wr_bit(CSI2_X_START_END_MEM, fe_io, 0xc, 0, 16);
            adap_wr_bit(CSI2_X_START_END_MEM, fe_io,
                        0xc + adap_fsm[channel].para.img.width - 1, 16, 16);
            adap_wr_bit(CSI2_Y_START_END_MEM, fe_io,
                        long_exp_offset, 0, 16);
            adap_wr_bit(CSI2_Y_START_END_MEM, fe_io,
                        long_exp_offset + adap_fsm[channel].para.img.height - 1, 16, 16);
            //set short exposure offset
            adap_wr_bit(CSI2_X_START_END_ISP, fe_io, 0xc, 0, 16);
            adap_wr_bit(CSI2_X_START_END_ISP, fe_io,
                        0xc + adap_fsm[channel].para.img.width - 1, 16, 16);
            adap_wr_bit(CSI2_Y_START_END_ISP, fe_io,
                        short_exp_offset, 0, 16);
            adap_wr_bit(CSI2_Y_START_END_ISP, fe_io,
                        short_exp_offset + adap_fsm[channel].para.img.height - 1, 16, 16);
        }else if (adap_fsm[channel].para.type == DOL_VC) {
            adap_write(CSI2_VC_MODE, fe_io, 0x11220040);
        }
    }
    return 0;
}


/*
 *========================AM ADAPTER READER INTERFACE==========================
 */

void am_adap_reader_start(uint8_t channel)
{
    uint32_t data = 0;
    adap_read(MIPI_ADAPT_DDR_RD0_CNTL0,  RD_IO, &data);
    adap_write(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, data | (1 << 31));
    adap_read(MIPI_ADAPT_DDR_RD1_CNTL0,  RD_IO, &data);
    adap_write(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, data | (1 << 31));
}

int am_adap_reader_init(struct adaptrd_param* prm, uint8_t channel) {
    uint32_t dol_mode,pingpong_mode;
    uint32_t port_sel_0;
    uint32_t port_sel_1;
    uint32_t ddr_rden_0;
    uint32_t ddr_rden_1;
    uint32_t ddr_rd0_ping,ddr_rd0_pong;
    uint32_t ddr_rd1_ping,ddr_rd1_pong;
    uint32_t continue_mode = 0;
    uint32_t data = 0;

    //wire    direct_path_sel = port_sel == 2'b00;
    //wire    ddr_path_sel    = port_sel == 2'b01;
    //wire    comb_path_sel    = port_sel == 2'b10;
    if (adap_fsm[channel].para.mode == DDR_MODE) {
        prm->rd_mem_ping_addr = adap_fsm[channel].ddr_buf[0];
        prm->rd_mem_pong_addr = adap_fsm[channel].ddr_buf[0];
    } else if (adap_fsm[channel].para.mode == DOL_MODE) {
        prm->rd_mem_ping_addr = dol_buf[0];
        prm->rd_mem_pong_addr = dol_buf[1];
    } else {
        prm->rd_mem_ping_addr = adap_fsm[channel].ddr_buf[0];
        prm->rd_mem_pong_addr = adap_fsm[channel].ddr_buf[0];
    }

    ddr_rd0_ping  = prm->rd_mem_ping_addr;
    ddr_rd0_pong  = prm->rd_mem_pong_addr;
    ddr_rd1_ping  = prm->rd_mem_ping_addr;
    ddr_rd1_pong  = prm->rd_mem_pong_addr;

    switch (prm->rd_work_mode) {
        case MODE_MIPI_RAW_SDR_DDR:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 1;
            port_sel_1    = 1;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
        case MODE_MIPI_RAW_SDR_DIRCT:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 0;
            port_sel_1    = 0;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DDR:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 1;
            port_sel_1    = 1;
            ddr_rden_0    = 1;
            ddr_rden_1    = 1;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DIRCT:
            dol_mode      = 1;
            pingpong_mode = 0;
            continue_mode = 1;
            port_sel_0    = 0;
            port_sel_1    = 1;
            ddr_rden_0    = 1;
            ddr_rden_1    = 1;
            break;
        case MODE_MIPI_YUV_SDR_DDR:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 1;
            port_sel_1    = 0;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
        case MODE_MIPI_RGB_SDR_DDR:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 1;
            port_sel_1    = 0;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
        case MODE_MIPI_YUV_SDR_DIRCT:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 2;
            port_sel_1    = 0;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
        case MODE_MIPI_RGB_SDR_DIRCT:
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 2;
            port_sel_1    = 0;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
        default:   //SDR direct
            dol_mode      = 0;
            pingpong_mode = 0;
            port_sel_0    = 0;
            port_sel_1    = 0;
            ddr_rden_0    = 1;
            ddr_rden_1    = 0;
            break;
   }

    adap_write(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 3<<28 | //axi burst
                                         1<<26 | //reg_sample_sel
                                         continue_mode << 24 |
                                         dol_mode << 23 |
                                         pingpong_mode << 22 |
                                         prm->rd_mem_line_stride << 4);
    adap_write(MIPI_ADAPT_DDR_RD0_CNTL1, RD_IO, port_sel_0 << 30 |
                                         prm->rd_mem_line_number << 16 |
                                         prm->rd_mem_line_size << 0);
    adap_write(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, ddr_rd0_ping);
    adap_write(MIPI_ADAPT_DDR_RD0_CNTL3, RD_IO, ddr_rd0_pong);

    adap_write(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, 3<<28 | //axi burst
                                         1<<26 | //reg_sample_sel
                                         continue_mode << 24 |
                                         dol_mode << 23 |
                                         pingpong_mode << 22 |
                                         prm->rd_mem_line_stride << 4);
    adap_write(MIPI_ADAPT_DDR_RD1_CNTL1, RD_IO, port_sel_1 << 30 |
                                         prm->rd_mem_line_number  << 16 |
                                         prm->rd_mem_line_size << 0);
    adap_write(MIPI_ADAPT_DDR_RD1_CNTL2, RD_IO, ddr_rd1_ping);
    adap_write(MIPI_ADAPT_DDR_RD1_CNTL3, RD_IO, ddr_rd1_pong);
    adap_read( MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, &data);
    adap_write(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, data |
                                         ddr_rden_0    << 0);
    adap_read( MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, &data);
    adap_write(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, data |
                                         ddr_rden_1    << 0);
    return 0;
}

/*
 *========================AM ADAPTER PIXEL INTERFACE===========================
 */

void am_adap_pixel_start(uint8_t channel)
{
    uint32_t data = 0;
    adap_read( MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, &data);
    adap_write(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, data | pixel_param.pixel_en_0<< 31);
    adap_read( MIPI_ADAPT_PIXEL1_CNTL0, PIXEL_IO, &data);
    adap_write(MIPI_ADAPT_PIXEL1_CNTL0, PIXEL_IO, data | pixel_param.pixel_en_1 << 31);
}

int am_adap_pixel_init(struct adaptpixel_param* prm, uint8_t channel) {
    uint32_t data_mode_0;  // 0:raw from ddr 1:raw from direct 2:comb from ddr 3:comb from direct
    uint32_t data_mode_1;  // 0:raw from ddr 1:raw from direct 2:comb from ddr 3:comb from direct

    switch (prm->pixel_work_mode) {
        case MODE_MIPI_RAW_SDR_DDR:
            data_mode_0     = 0;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
        case MODE_MIPI_RAW_SDR_DIRCT:
            data_mode_0     = 1;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DDR:
            data_mode_0     = 0;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 1;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DIRCT:
            data_mode_0     = 1;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 1;
            break;
        case MODE_MIPI_YUV_SDR_DDR:
            data_mode_0     = 2;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
        case MODE_MIPI_RGB_SDR_DDR:
            data_mode_0     = 2;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
        case MODE_MIPI_YUV_SDR_DIRCT:
            data_mode_0     = 3;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
        case MODE_MIPI_RGB_SDR_DIRCT:
            data_mode_0     = 3;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
        default:
            data_mode_0     = 1;
            data_mode_1     = 0;
            prm->pixel_en_0 = 1;
            prm->pixel_en_1 = 0;
            break;
    }

    adap_write(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, prm->pixel_data_type << 20 |
                                        data_mode_0 << 16);
    adap_write(MIPI_ADAPT_PIXEL0_CNTL1, PIXEL_IO, prm->pixel_isp_x_start << 16 |
                                        prm->pixel_isp_x_end << 0);
    adap_write(MIPI_ADAPT_PIXEL0_CNTL2, PIXEL_IO, prm->pixel_pixel_number << 15 |
                                        prm->pixel_line_size << 0);

    adap_write(MIPI_ADAPT_PIXEL1_CNTL0, PIXEL_IO, prm->pixel_data_type << 20 |
                                        data_mode_1 << 16);
    adap_write(MIPI_ADAPT_PIXEL1_CNTL1, PIXEL_IO, prm->pixel_isp_x_start << 16 |
                                        prm->pixel_isp_x_end << 0);
    adap_write(MIPI_ADAPT_PIXEL1_CNTL2, PIXEL_IO, prm->pixel_pixel_number << 15 |
                                        prm->pixel_line_size << 0);

    return 0;
}

/*
 *========================AM ADAPTER ALIGNMENT INTERFACE=======================
 */

void am_adap_alig_start(uint8_t channel)
{
    int width, height, alig_width, alig_height;
    width = adap_fsm[channel].para.img.width;
    height = adap_fsm[channel].para.img.height;
    alig_width = width + 40; //hblank > 32 cycles
    alig_width = adap_fsm[channel].para.align_width;
    if (channel) {
        if (adap_fsm[channel - 1].para.align_width > adap_fsm[channel].para.align_width)
            alig_width = adap_fsm[channel - 1].para.align_width;
    } else {
        if (adap_fsm[channel + 1].para.align_width > adap_fsm[channel].para.align_width)
            alig_width = adap_fsm[channel + 1].para.align_width;
    }

    alig_height = height + 60; //vblank > 48 lines
    //val = width + 35; // width < val < alig_width
    adap_wr_bit(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, alig_width, 0, 13);
    adap_wr_bit(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, alig_height, 16, 13);
    adap_wr_bit(MIPI_ADAPT_ALIG_CNTL1, ALIGN_IO, width, 16, 13);
    adap_wr_bit(MIPI_ADAPT_ALIG_CNTL2, ALIGN_IO, height, 16, 13);
    //adap_wr_bit(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, val, 16, 13);
    adap_wr_bit(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 1, 31, 1);  // enable
}

int am_adap_alig_init(struct adaptalig_param* prm, uint8_t channel)
{
    uint32_t lane0_en      ; //  = mipi_alig_cntl6[0] ;  //lane enable
    uint32_t lane0_sel     ; //  = mipi_alig_cntl6[1] ;  //lane select
    uint32_t lane1_en      ; //  = mipi_alig_cntl6[2] ;  //lane enable
    uint32_t lane1_sel     ; //  = mipi_alig_cntl6[3] ;  //lane select
    uint32_t pix_datamode_0; //  = mipi_alig_cntl6[4] ;  //1 dir 0 ddr
    uint32_t pix_datamode_1; //  = mipi_alig_cntl6[5] ;  //
    uint32_t vdata0_sel    ; //  = mipi_alig_cntl6[8] ;  //vdata0 select
    uint32_t vdata1_sel    ; //  = mipi_alig_cntl6[9] ;  //vdata1 select
    uint32_t vdata2_sel    ; //  = mipi_alig_cntl6[10];  //vdata2 select
    uint32_t vdata3_sel    ; //  = mipi_alig_cntl6[11];  //vdata3 select
    uint32_t yuvrgb_mode   ; //  = mipi_alig_cntl6[31];
    switch (prm->alig_work_mode) {
        case MODE_MIPI_RAW_SDR_DDR:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 0;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 0;
            break;
        case MODE_MIPI_RAW_SDR_DIRCT:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 1;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 0;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DDR:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 1;
            lane1_sel      = 1;
            pix_datamode_0 = 0;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 1;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 0;
            break;
        case MODE_MIPI_RAW_HDR_DDR_DIRCT:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 1;
            lane1_sel      = 1;
            pix_datamode_0 = 1;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 1;
            vdata2_sel     = 1;
            vdata3_sel     = 1;
            yuvrgb_mode    = 0;
            break;
        case MODE_MIPI_YUV_SDR_DDR:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 0;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 1;
            break;
        case MODE_MIPI_RGB_SDR_DDR:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 0;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 1;
            break;
        case MODE_MIPI_YUV_SDR_DIRCT:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 1;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 1;
            break;
        case MODE_MIPI_RGB_SDR_DIRCT:
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 1;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 1;
            break;
        default:    //SDR direct
            lane0_en       = 1;
            lane0_sel      = 0;
            lane1_en       = 0;
            lane1_sel      = 0;
            pix_datamode_0 = 1;
            pix_datamode_1 = 0;
            vdata0_sel     = 0;
            vdata1_sel     = 0;
            vdata2_sel     = 0;
            vdata3_sel     = 0;
            yuvrgb_mode    = 0;
            break;
   }

   adap_write(MIPI_ADAPT_ALIG_CNTL0 , PIXEL_IO, (prm->alig_hsize + 64) << 0 |
                                                (prm->alig_vsize + 64) << 16);
   adap_write(MIPI_ADAPT_ALIG_CNTL1 , PIXEL_IO, prm->alig_hsize << 16 |
                                                0 << 0);
   adap_write(MIPI_ADAPT_ALIG_CNTL2 , PIXEL_IO, prm->alig_vsize << 16 |
                                                0 << 0);
   adap_write(MIPI_ADAPT_ALIG_CNTL3 , PIXEL_IO, prm->alig_hsize << 0  |
                                                0 << 16);
   adap_write(MIPI_ADAPT_ALIG_CNTL6 , PIXEL_IO, lane0_en << 0 |
                                                lane0_sel      << 1 |
                                                lane1_en       << 2 |
                                                lane1_sel      << 3 |
                                                pix_datamode_0 << 4 |
                                                pix_datamode_1 << 5 |
                                                vdata0_sel     << 8 |
                                                vdata1_sel     << 9 |
                                                vdata2_sel     << 10|
                                                vdata3_sel     << 11|
                                                0xf            << 12|
                                                yuvrgb_mode    << 31);
   adap_write(MIPI_ADAPT_ALIG_CNTL8,  PIXEL_IO, (1 << 12 ) | (1 << 5));
   return 0;
}

/*
 *========================AM ADAPTER INTERFACE==========================
 */

static int get_next_wr_buf_index(uint8_t channel)
{
    int index = 0;
    int total_size = DDR_BUF_SIZE;
    adap_fsm[channel].wbuf_index = adap_fsm[channel].wbuf_index + 1;

    index = adap_fsm[channel].wbuf_index % total_size;

    return index;
}

static irqreturn_t adpapter_isr(int irq, void *para)
{
    uint32_t data = 0;
    uint32_t data1 = 0;
    resource_size_t val = 0;

    adap_read(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, &data);
    adap_read(MIPI_TOP_ISP_PENDING1, CMPR_CNTL_IO, &data1);
    adap_write(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, data);
    adap_write(MIPI_TOP_ISP_PENDING1, CMPR_CNTL_IO, data1);

    if ((data & (1 << FRONT0_WR_DONE))) {
        if ((kfifo_len(&adap_fsm[ADAP0_PATH].adapt_fifo) / 8) < (DDR_BUF_SIZE - 2)) {
            kfifo_in(&adap_fsm[ADAP0_PATH].adapt_fifo, &adap_fsm[ADAP0_PATH].ddr_buf[adap_fsm[ADAP0_PATH].next_buf_index], sizeof(resource_size_t));
            camera_frame_fifo[g_adap->write_frame_ptr ++] = CAM0_ACT;
            if (g_adap->write_frame_ptr == CAMERA_QUEUE_NUM)
                g_adap->write_frame_ptr = 0;
        } else {
            LOG(LOG_INFO, "adapt fifo 0 is full .\n");
            return IRQ_HANDLED;
        }

        adap_fsm[ADAP0_PATH].next_buf_index = get_next_wr_buf_index(ADAP0_PATH);
        val = adap_fsm[ADAP0_PATH].ddr_buf[adap_fsm[ADAP0_PATH].next_buf_index];
        adap_write(CSI2_DDR_START_PIX, FRONTEND0_IO, val);
        LOG(LOG_INFO, "frontend cam0 wr start %x", val);
        g_adap->frame_state = FRAME_READY;
        wake_up_interruptible( &g_adap->frame_wq);
    }

    if (data & (1 << 9)) {
        adap_fsm[ADAP0_PATH].control_flag = 0;
        adap_fsm[ADAP1_PATH].control_flag = 0;
        adap_fsm[ADAP2_PATH].control_flag = 0;
        LOG(LOG_INFO, "align read done");
    }

    if (data1 & (1 << FRONT2_WR_DONE)) {
        if ((kfifo_len(&adap_fsm[ADAP1_PATH].adapt_fifo) / 8) < (DDR_BUF_SIZE - 2)) {
            kfifo_in(&adap_fsm[ADAP1_PATH].adapt_fifo, &adap_fsm[ADAP1_PATH].ddr_buf[adap_fsm[ADAP1_PATH].next_buf_index], sizeof(resource_size_t));
            camera_frame_fifo[g_adap->write_frame_ptr ++] = CAM1_ACT;
            if (g_adap->write_frame_ptr == CAMERA_QUEUE_NUM)
                g_adap->write_frame_ptr = 0;
        } else {
            LOG(LOG_INFO, "adapt fifo 1 is full .\n");
            return IRQ_HANDLED;
        }

        adap_fsm[ADAP1_PATH].next_buf_index = get_next_wr_buf_index(ADAP1_PATH);
        val = adap_fsm[ADAP1_PATH].ddr_buf[adap_fsm[ADAP1_PATH].next_buf_index];
        adap_write(CSI2_DDR_START_PIX, FRONTEND2_IO, val);
        LOG(LOG_INFO, "frontend cam1 wr start %x", val);
        g_adap->frame_state = FRAME_READY;
        wake_up_interruptible( &g_adap->frame_wq);
    }

    if (data1 & (1 << FRONT3_WR_DONE)) {
        if ((kfifo_len(&adap_fsm[ADAP2_PATH].adapt_fifo) / 8) < (DDR_BUF_SIZE - 2)) {
            kfifo_in(&adap_fsm[ADAP2_PATH].adapt_fifo, &adap_fsm[ADAP2_PATH].ddr_buf[adap_fsm[ADAP2_PATH].next_buf_index], sizeof(resource_size_t));
            camera_frame_fifo[g_adap->write_frame_ptr ++] = CAM2_ACT;
            if (g_adap->write_frame_ptr == CAMERA_QUEUE_NUM)
                g_adap->write_frame_ptr = 0;
        } else {
            LOG(LOG_INFO, "adapt fifo 2 is full .\n");
            return IRQ_HANDLED;
        }

        adap_fsm[ADAP2_PATH].next_buf_index = get_next_wr_buf_index(ADAP2_PATH);
        val = adap_fsm[ADAP2_PATH].ddr_buf[adap_fsm[ADAP2_PATH].next_buf_index];
        adap_write(CSI2_DDR_START_PIX, FRONTEND3_IO, val);
        LOG(LOG_INFO, "frontend cam2 wr start %x", val);
        g_adap->frame_state = FRAME_READY;
        wake_up_interruptible( &g_adap->frame_wq);
    }

    return IRQ_HANDLED;
}

static int am_adap_isp_check_status(uint8_t adapt_path)
{
    resource_size_t val = 0;
    int check_count = 0;
    int frame_state = 0;
    int kfifo_ret = -1;

    while (adap_fsm[CAM0_ACT].control_flag || adap_fsm[CAM1_ACT].control_flag || adap_fsm[CAM2_ACT].control_flag) {
        if (check_count ++ > 16) {
            pr_err("adapt read%d done timeout %d-%d-%d.\n",adapt_path, adap_fsm[CAM0_ACT].control_flag,adap_fsm[CAM1_ACT].control_flag,adap_fsm[CAM1_ACT].control_flag);
            adap_fsm[CAM0_ACT].control_flag = 0;
            adap_fsm[CAM1_ACT].control_flag = 0;
            adap_fsm[CAM2_ACT].control_flag = 0;
            frame_state = -1;
            break;
        } else
            mdelay(1);
    }

    check_count = 0;
    if (adap_fsm[CAM0_ACT].para.type != DOL_YUV && adap_fsm[CAM1_ACT].para.type != DOL_YUV && adap_fsm[CAM2_ACT].para.type != DOL_YUV) {
        while (kfifo_ret) {
            camera_notify(NOTIFY_GET_QUEUE_STATUS, &kfifo_ret);
            if (kfifo_ret == 0)
                break;
            if (check_count ++ > 16) {
                pr_err("isp %d busy now, kfifo_ret: %d\n", adapt_path, kfifo_ret);
                frame_state = -1;
                break;
            } else
                mdelay(1);
        }
    }

    check_count = 0;
    kfifo_ret = -1;
    while (kfifo_ret) {
        camera_notify(NOTIFY_GET_SC03_STATUS, &kfifo_ret);
        if (kfifo_ret == SCMIF_IDLE)
           break;
        if (check_count ++ > 16) {
           pr_err("sc %d busy now, kfifo_ret: %d\n", adapt_path, kfifo_ret);
           frame_state = -1;
           break;
        } else
           mdelay(1);
    }

    kfifo_ret = kfifo_out(&adap_fsm[adapt_path].adapt_fifo, &val, sizeof(val));
    adap_fsm[adapt_path].read_buf = val;
    adap_fsm[adapt_path].control_flag = 1;
    if (g_adap->read_frame_ptr > 0)
        adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, camera_frame_fifo[g_adap->read_frame_ptr - 1], CAM_LAST, 2);
    else
        adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, camera_frame_fifo[CAMERA_QUEUE_NUM - 1], CAM_LAST, 2);
    adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, camera_frame_fifo[g_adap->read_frame_ptr], CAM_CURRENT, 2);
    adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, camera_frame_fifo[(g_adap->read_frame_ptr + 1)%CAMERA_QUEUE_NUM], CAM_NEXT, 2);
    adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, camera_frame_fifo[(g_adap->read_frame_ptr + 2)%CAMERA_QUEUE_NUM], CAM_NEXT_NEXT, 2);

    g_adap->read_frame_ptr ++ ;
    g_adap->read_frame_ptr %= CAMERA_QUEUE_NUM;

    if (adap_fsm[adapt_path].para.type == DOL_YUV) {
        rd_param.rd_work_mode         = MODE_MIPI_YUV_SDR_DDR;
        pixel_param.pixel_work_mode  = MODE_MIPI_YUV_SDR_DDR;
        alig_param.alig_work_mode     = MODE_MIPI_YUV_SDR_DDR;
    } else {
        rd_param.rd_work_mode         = MODE_MIPI_RAW_SDR_DDR;
        pixel_param.pixel_work_mode  = MODE_MIPI_RAW_SDR_DDR;
        alig_param.alig_work_mode     = MODE_MIPI_RAW_SDR_DDR;
    }

    rd_param.rd_mem_line_stride     = (adap_fsm[adapt_path].para.img.width * am_adap_get_depth(adapt_path) + 127) >> 7;
    rd_param.rd_mem_line_size        = (adap_fsm[adapt_path].para.img.width * am_adap_get_depth(adapt_path) + 127) >> 7;
    rd_param.rd_mem_line_number     = adap_fsm[adapt_path].para.img.height;
    pixel_param.pixel_data_type     = adap_fsm[adapt_path].para.fmt;
    pixel_param.pixel_isp_x_start    = 0;
    pixel_param.pixel_isp_x_end     = adap_fsm[adapt_path].para.img.width - 1;
    pixel_param.pixel_line_size     = (adap_fsm[adapt_path].para.img.width * am_adap_get_depth(adapt_path) + 127) >> 7;
    pixel_param.pixel_pixel_number    = adap_fsm[adapt_path].para.img.width;
    alig_param.alig_hsize            = adap_fsm[adapt_path].para.img.width;
    alig_param.alig_vsize            = adap_fsm[adapt_path].para.img.height;
    am_adap_reader_init(&rd_param, adapt_path);
    am_adap_pixel_init(&pixel_param, adapt_path);
    am_adap_alig_init(&alig_param, adapt_path);

    kfifo_ret = SCMIF_BUSY;
    camera_notify(NOTIFY_SET_SC03_STATUS, &kfifo_ret);

    return frame_state;
}

static int adap_stream_copy_thread( void *data )
{
    uint8_t cam_mode = 0;
    uint8_t frame_num = 0;
    int8_t frame_state = 0;

    set_freezable();

    for ( ;; ) {
        try_to_freeze();

        if ( kthread_should_stop() )
            break;

        frame_state = 0;
        cam_mode = adap_fsm[ADAP0_PATH].cam_en + adap_fsm[ADAP1_PATH].cam_en + adap_fsm[ADAP2_PATH].cam_en;
        frame_num = kfifo_len(&adap_fsm[ADAP0_PATH].adapt_fifo) + kfifo_len(&adap_fsm[ADAP1_PATH].adapt_fifo) + kfifo_len(&adap_fsm[ADAP2_PATH].adapt_fifo);

        if ( wait_event_interruptible_timeout( g_adap->frame_wq,
            (g_adap->frame_state == FRAME_READY),
            msecs_to_jiffies( 5 ) ) < 0 ) {
            continue;
        }

        if (g_adap->frame_state == FRAME_READY)
            g_adap->frame_state = FRAME_NOREADY;

        if ((frame_num / 8 ) < 3)
            continue;

        if ((kfifo_len(&adap_fsm[ADAP0_PATH].adapt_fifo) > 0) && camera_frame_fifo[g_adap->read_frame_ptr] == CAM0_ACT) {
            frame_state = am_adap_isp_check_status(ADAP0_PATH);
            adap_write(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, adap_fsm[ADAP0_PATH].read_buf);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 25, 1);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0, 25, 1);

            if (frame_state == 0)
                adap_write(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80001C00 | (1 << 6) | (1 << 5));// 0<<10 no care isp req
            else
                adap_write(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80001000 | (1 << 6) | (1 << 5));// 3<<10 care isp req

            adap_wr_bit(MIPI_ADAPT_PIXEL0_CNTL0, RD_IO, 1, 31, 1);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 31, 1);
            LOG(LOG_INFO, "send cam 0 \n");
            continue;
        }

        if ((kfifo_len(&adap_fsm[ADAP1_PATH].adapt_fifo) > 0)  && camera_frame_fifo[g_adap->read_frame_ptr] == CAM1_ACT) {
            frame_state = am_adap_isp_check_status(ADAP1_PATH);
            adap_write(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, adap_fsm[ADAP1_PATH].read_buf);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 25, 1);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0, 25, 1);

            if (frame_state == 0)
                adap_write(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80001C00 | (1 << 6) | (1 << 5));
            else
                adap_write(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80001000 | (1 << 6) | (1 << 5));

            adap_wr_bit(MIPI_ADAPT_PIXEL0_CNTL0, RD_IO, 1, 31, 1);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 31, 1);
            LOG(LOG_INFO, "send cam 1 \n");
            continue;
        }

        if ((kfifo_len(&adap_fsm[ADAP2_PATH].adapt_fifo) > 0)  && camera_frame_fifo[g_adap->read_frame_ptr] == CAM2_ACT) {
            frame_state = am_adap_isp_check_status(ADAP2_PATH);
            adap_write(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, adap_fsm[ADAP2_PATH].read_buf);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 25, 1);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0, 25, 1);

            if (frame_state == 0)
                adap_write(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80001C00 | (1 << 6) | (1 << 5));
            else
                adap_write(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 0x80001000 | (1 << 6) | (1 << 5));

            adap_wr_bit(MIPI_ADAPT_PIXEL0_CNTL0, RD_IO, 1, 31, 1);
            adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 31, 1);
            LOG(LOG_INFO, "send cam 2 \n");
            continue;
        }
    }

    return 0;
}

int am_adap_alloc_mem(uint8_t channel)
{
    if (adap_fsm[channel].para.mode == DDR_MODE) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
        adap_fsm[channel].cma_pages = dma_alloc_from_contiguous(
                  &(g_adap->p_dev->dev),
                  (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0, false);
#else
        adap_fsm[channel].cma_pages = dma_alloc_from_contiguous(
                 &(g_adap->p_dev->dev),
                 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0);

#endif
        if (adap_fsm[channel].cma_pages) {
            adap_fsm[channel].buffer_start = page_to_phys(adap_fsm[channel].cma_pages);
        } else {
            pr_err("alloc cma pages failed.\n");
            return 0;
        }
    } else if (adap_fsm[channel].para.mode == DCAM_MODE) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
        adap_fsm[channel].cma_pages = dma_alloc_from_contiguous(
                  &(g_adap->p_dev->dev),
                  (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0, false);
#else
        adap_fsm[channel].cma_pages = dma_alloc_from_contiguous(
                 &(g_adap->p_dev->dev),
                 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0);

#endif
        if (adap_fsm[channel].cma_pages) {
            adap_fsm[channel].buffer_start = page_to_phys(adap_fsm[channel].cma_pages);
        } else {
            pr_err("alloc cma pages failed.\n");
            return 0;
        }
    } else if (adap_fsm[channel].para.mode == DOL_MODE) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
        adap_fsm[channel].cma_pages = dma_alloc_from_contiguous(
                  &(g_adap->p_dev->dev),
                  (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0, false);
#else
        adap_fsm[channel].cma_pages = dma_alloc_from_contiguous(
                  &(g_adap->p_dev->dev),
                  (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0);

#endif
        if (adap_fsm[channel].cma_pages) {
            adap_fsm[channel].buffer_start = page_to_phys(adap_fsm[channel].cma_pages);
        } else {
            pr_err("alloc dol cma pages failed.\n");
            return 0;
        }
    }
    return 0;
}

int am_adap_free_mem(uint8_t channel)
{
    if (adap_fsm[channel].para.mode == DDR_MODE) {
        if (adap_fsm[channel].cma_pages) {
            dma_release_from_contiguous(
                 &(g_adap->p_dev->dev),
                 adap_fsm[channel].cma_pages,
                 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT);
            adap_fsm[channel].cma_pages = NULL;
            adap_fsm[channel].buffer_start = 0;
            pr_info("release alloc CMA buffer. channel:%d\n",channel);
        }
    } else if (adap_fsm[channel].para.mode == DCAM_MODE) {
        if (adap_fsm[channel].cma_pages) {
            dma_release_from_contiguous(
                 &(g_adap->p_dev->dev),
                 adap_fsm[channel].cma_pages,
                 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT);
            adap_fsm[channel].cma_pages = NULL;
            adap_fsm[channel].buffer_start = 0;
            pr_info("release alloc CMA buffer. channel:%d\n",channel);
        }
    } else if (adap_fsm[channel].para.mode == DOL_MODE) {
        if (adap_fsm[channel].cma_pages) {
            dma_release_from_contiguous(
                 &(g_adap->p_dev->dev),
                 adap_fsm[channel].cma_pages,
                 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT);
            adap_fsm[channel].cma_pages = NULL;
            adap_fsm[channel].buffer_start = 0;
            pr_info("release alloc dol CMA buffer. channel:%d\n",channel);
        }
    }
    return 0;
}

int am_adap_init(uint8_t channel)
{
    int ret = 0;
    int depth;
    int i;
    int kfifo_ret = 0;
    resource_size_t temp_buf;
    char *buf = NULL;
    uint32_t stride;
    int buf_cnt;
    int temp_work_mode = DIR_MODE;
    adap_fsm[channel].control_flag = 0;
    adap_fsm[channel].wbuf_index = 0;
    adap_fsm[channel].next_buf_index = 0;

    if (adap_fsm[channel].cma_pages) {
        am_adap_free_mem(channel);
        adap_fsm[channel].cma_pages = NULL;
    }

    if ((adap_fsm[channel].para.mode == DDR_MODE) ||
        (adap_fsm[channel].para.mode == DCAM_MODE) ||
        (adap_fsm[channel].para.mode == DOL_MODE)) {
        am_adap_alloc_mem(channel);
        depth = am_adap_get_depth(channel);
        if ((adap_fsm[channel].cma_pages) && (adap_fsm[channel].para.mode == DDR_MODE)) {
            //note important : ddr_buf[0] and ddr_buf[1] address should alignment 16 byte
            stride = (adap_fsm[channel].para.img.width * depth)/8;
            stride = ((stride + (BOUNDRY - 1)) & (~(BOUNDRY - 1)));
            adap_fsm[channel].ddr_buf[0] = adap_fsm[channel].buffer_start;
            adap_fsm[channel].ddr_buf[0] = (adap_fsm[channel].ddr_buf[0] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
            temp_buf = adap_fsm[channel].ddr_buf[0];
            buf = phys_to_virt(adap_fsm[channel].ddr_buf[0]);
            memset(buf, 0x0, (stride * adap_fsm[channel].para.img.height));
            for (i = 1; i < DDR_BUF_SIZE; i++) {
                adap_fsm[channel].ddr_buf[i] = temp_buf + (stride * (adap_fsm[channel].para.img.height + 100));
                adap_fsm[channel].ddr_buf[i] = (adap_fsm[channel].ddr_buf[i] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
                temp_buf = adap_fsm[channel].ddr_buf[i];
                buf = phys_to_virt(adap_fsm[channel].ddr_buf[i]);
            }
        } else if ((adap_fsm[channel].cma_pages) && (adap_fsm[channel].para.mode == DCAM_MODE)) {
            //note important : ddr_buf[0] and ddr_buf[1] address should alignment 16 byte
            stride = (adap_fsm[channel].para.img.width * depth)/8;
            stride = ((stride + (BOUNDRY - 1)) & (~(BOUNDRY - 1)));
            adap_fsm[channel].ddr_buf[0] = adap_fsm[channel].buffer_start;
            adap_fsm[channel].ddr_buf[0] = (adap_fsm[channel].ddr_buf[0] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
            temp_buf = adap_fsm[channel].ddr_buf[0];
            buf = phys_to_virt(adap_fsm[channel].ddr_buf[0]);
            memset(buf, 0x0, (stride * adap_fsm[channel].para.img.height));
            for (i = 1; i < DDR_BUF_SIZE; i++) {
                adap_fsm[channel].ddr_buf[i] = temp_buf + (stride * (adap_fsm[channel].para.img.height + 100));
                adap_fsm[channel].ddr_buf[i] = (adap_fsm[channel].ddr_buf[i] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
                temp_buf = adap_fsm[channel].ddr_buf[i];
                buf = phys_to_virt(adap_fsm[channel].ddr_buf[i]);
                memset(buf, 0x0, (stride * adap_fsm[channel].para.img.height));
            }
        } else if ((adap_fsm[channel].cma_pages) && (adap_fsm[channel].para.mode == DOL_MODE)) {
            dol_buf[0] = adap_fsm[channel].buffer_start;
            dol_buf[0] = (dol_buf[0] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
            temp_buf = dol_buf[0];
            buf_cnt = 2;
            for (i = 1; i < buf_cnt; i++) {
                dol_buf[i] = temp_buf + ((adap_fsm[channel].para.img.width) * (adap_fsm[channel].para.img.height) * depth)/8;
                dol_buf[i] = (dol_buf[i] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
                temp_buf = dol_buf[i];
            }
            if ( buf_cnt == 2 )
                dol_buf[1] = dol_buf[0];
        }
    }

    if (adap_fsm[channel].para.mode == DDR_MODE && g_adap->f_end_irq == 0) {
        ret = request_irq(g_adap->rd_irq, &adpapter_isr, IRQF_SHARED | IRQF_TRIGGER_HIGH,
        "adapter-irq", (void *)g_adap);
        g_adap->f_end_irq = 1;
        pr_info("adapter irq = %d, ret = %d\n", g_adap->rd_irq, ret);
    } else if (adap_fsm[channel].para.mode == DCAM_MODE && g_adap->f_end_irq == 0) {
        ret = request_irq(g_adap->rd_irq, &adpapter_isr, IRQF_SHARED | IRQF_TRIGGER_HIGH,
        "adapter-irq", (void *)g_adap);
        g_adap->f_end_irq = 1;
        pr_info("adapter irq = %d, ret = %d\n", g_adap->rd_irq, ret);
        pr_info("am_adap_init: create irq dcam\n");
    }

    if (adap_fsm[channel].para.mode == DDR_MODE && g_adap->f_fifo == 0) {
        kfifo_ret = kfifo_alloc(&adap_fsm[CAM0_ACT].adapt_fifo, 8 * DDR_BUF_SIZE, GFP_KERNEL);
        kfifo_ret = kfifo_alloc(&adap_fsm[CAM1_ACT].adapt_fifo, 8 * DDR_BUF_SIZE, GFP_KERNEL);
        kfifo_ret = kfifo_alloc(&adap_fsm[CAM2_ACT].adapt_fifo, 8 * DDR_BUF_SIZE, GFP_KERNEL);
        if (kfifo_ret) {
            pr_info("alloc adapter fifo failed.\n");
            return kfifo_ret;
        }

        init_waitqueue_head( &g_adap->frame_wq );
        static struct sched_param param;
        param.sched_priority = 2;//struct sched_param param = { .sched_priority = 2 };
        g_adap->kadap_stream = kthread_run( adap_stream_copy_thread, NULL, "adap-stream");
        sched_setscheduler(g_adap->kadap_stream, SCHED_IDLE, &param);
        wake_up_process(g_adap->kadap_stream);
        g_adap->f_fifo = 1;
    } else if (adap_fsm[channel].para.mode == DCAM_MODE && g_adap->f_fifo == 0) {
        kfifo_ret = kfifo_alloc(&adap_fsm[CAM0_ACT].adapt_fifo, 8 * DDR_BUF_SIZE, GFP_KERNEL);
        kfifo_ret = kfifo_alloc(&adap_fsm[CAM1_ACT].adapt_fifo, 8 * DDR_BUF_SIZE, GFP_KERNEL);
        kfifo_ret = kfifo_alloc(&adap_fsm[CAM2_ACT].adapt_fifo, 8 * DDR_BUF_SIZE, GFP_KERNEL);
        if (kfifo_ret) {
            pr_info("alloc adapter fifo failed.\n");
            return kfifo_ret;
        }
        pr_info("am_adap_init: create thread dcam\n");

        init_waitqueue_head( &g_adap->frame_wq );
        static struct sched_param param;
        param.sched_priority = 2;   //struct sched_param param = { .sched_priority = 2 };
        g_adap->kadap_stream = kthread_run( adap_stream_copy_thread, NULL, "adap-stream");
        sched_setscheduler(g_adap->kadap_stream, SCHED_IDLE, &param);
        wake_up_process(g_adap->kadap_stream);
        g_adap->f_fifo = 1;
    }

    if (g_adap->f_adap == 0) {
        if (channel == CAM0_ACT)
            am_adap_reset(ADAP0_PATH);
        else if (channel == CAM1_ACT)
            am_adap_reset(ADAP2_PATH);
        else if (channel == CAM2_ACT)
            am_adap_reset(ADAP3_PATH);
    }

    switch (adap_fsm[channel].para.mode) {
        case DIR_MODE:
            rd_param.rd_work_mode        = MODE_MIPI_RAW_SDR_DIRCT;
            pixel_param.pixel_work_mode     = MODE_MIPI_RAW_SDR_DIRCT;
            alig_param.alig_work_mode    = MODE_MIPI_RAW_SDR_DIRCT;
            temp_work_mode               = MODE_MIPI_RAW_SDR_DIRCT;
            if ( adap_fsm[channel].para.type == DOL_YUV ) {
                rd_param.rd_work_mode         = MODE_MIPI_YUV_SDR_DIRCT;
                pixel_param.pixel_work_mode  = MODE_MIPI_YUV_SDR_DIRCT;
                alig_param.alig_work_mode     = MODE_MIPI_YUV_SDR_DIRCT;
                temp_work_mode                 = MODE_MIPI_YUV_SDR_DIRCT;
            }
            break;
        case DDR_MODE:
            rd_param.rd_work_mode        = MODE_MIPI_RAW_SDR_DDR;
            pixel_param.pixel_work_mode     = MODE_MIPI_RAW_SDR_DDR;
            alig_param.alig_work_mode    = MODE_MIPI_RAW_SDR_DDR;
            temp_work_mode               = MODE_MIPI_RAW_SDR_DDR;
            if ( adap_fsm[channel].para.type == DOL_YUV ) {
                rd_param.rd_work_mode         = MODE_MIPI_YUV_SDR_DDR;
                pixel_param.pixel_work_mode  = MODE_MIPI_YUV_SDR_DDR;
                alig_param.alig_work_mode     = MODE_MIPI_YUV_SDR_DDR;
                temp_work_mode                 = MODE_MIPI_YUV_SDR_DDR;
            }
            if (channel == 0)
                adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, FRONT0_WR_DONE, 1 );//fe0 wr done
            else if (channel == 1)
                adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 1, FRONT2_WR_DONE, 1 );//fe2 wr done
            else if (channel == 2)
                adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 1, FRONT3_WR_DONE, 1 );//fe3 wr done
            adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, 9, 1 );//rd0 done
            break;
        case DCAM_MODE:
            rd_param.rd_work_mode        = MODE_MIPI_RAW_SDR_DDR;
            pixel_param.pixel_work_mode     = MODE_MIPI_RAW_SDR_DDR;
            alig_param.alig_work_mode    = MODE_MIPI_RAW_SDR_DDR;
            temp_work_mode               = MODE_MIPI_RAW_SDR_DDR;
            if ( adap_fsm[channel].para.type == DOL_YUV ) {
                rd_param.rd_work_mode         = MODE_MIPI_YUV_SDR_DDR;
                pixel_param.pixel_work_mode  = MODE_MIPI_YUV_SDR_DDR;
                alig_param.alig_work_mode     = MODE_MIPI_YUV_SDR_DDR;
                temp_work_mode                 = MODE_MIPI_YUV_SDR_DDR;
            }
            if (channel == 0)
                adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, FRONT0_WR_DONE, 1 );//fe0 wr done
            else if (channel == 1)
                adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 1, FRONT2_WR_DONE, 1 );//fe2 wr done
            else if (channel == 2)
                adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 1, FRONT3_WR_DONE, 1 );//fe3 wr done
            adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, 9, 1 );//rd0 done
            break;
        case DOL_MODE:
            rd_param.rd_work_mode        = MODE_MIPI_RAW_HDR_DDR_DIRCT;
            pixel_param.pixel_work_mode     = MODE_MIPI_RAW_HDR_DDR_DIRCT;
            alig_param.alig_work_mode    = MODE_MIPI_RAW_HDR_DDR_DIRCT;
            temp_work_mode               = MODE_MIPI_RAW_HDR_DDR_DIRCT;
            break;
        default:
            pr_err("invalid adapt work mode!\n");
            break;
    }

    if (channel == CAM2_ACT)
        fe_param[channel].fe_sel              = ADAP3_PATH;
    else if (channel == CAM1_ACT)
        fe_param[channel].fe_sel              = ADAP2_PATH;
    else
        fe_param[channel].fe_sel              = ADAP0_PATH;
    fe_param[channel].fe_work_mode        = temp_work_mode;
    fe_param[channel].fe_mem_x_start      = 0;
    fe_param[channel].fe_mem_x_end        = adap_fsm[channel].para.img.width - 1;
    fe_param[channel].fe_mem_y_start      = 0;
    fe_param[channel].fe_mem_y_end        = adap_fsm[channel].para.img.height - 1;
    fe_param[channel].fe_isp_x_start      = 0;
    fe_param[channel].fe_isp_x_end        = adap_fsm[channel].para.img.width - 1;
    fe_param[channel].fe_isp_y_start      = 0x0;
    fe_param[channel].fe_isp_y_end        = adap_fsm[channel].para.img.height - 1;
    fe_param[channel].fe_mem_line_stride  = ceil_upper((am_adap_get_depth(channel) * adap_fsm[channel].para.img.width), (8 * 16));
    fe_param[channel].fe_mem_line_minbyte = (am_adap_get_depth(channel) * adap_fsm[channel].para.img.width + 7) >> 3;
    fe_param[channel].fe_int_mask         = 0x0;
    am_adap_frontend_init(&fe_param[channel], channel);

    if (g_adap->f_adap == 0) {
        rd_param.rd_mem_line_stride     = (adap_fsm[channel].para.img.width * am_adap_get_depth(channel) + 127) >> 7;
        rd_param.rd_mem_line_size       = (adap_fsm[channel].para.img.width * am_adap_get_depth(channel) + 127) >> 7;
        rd_param.rd_mem_line_number     = adap_fsm[channel].para.img.height;
        pixel_param.pixel_data_type        = adap_fsm[channel].para.fmt;
        pixel_param.pixel_isp_x_start   = 0;
        pixel_param.pixel_isp_x_end        = adap_fsm[channel].para.img.width - 1;
        pixel_param.pixel_line_size        = (adap_fsm[channel].para.img.width * am_adap_get_depth(channel) + 127) >> 7;
        pixel_param.pixel_pixel_number  = adap_fsm[channel].para.img.width;
        alig_param.alig_hsize            = adap_fsm[channel].para.img.width;
        alig_param.alig_vsize            = adap_fsm[channel].para.img.height;
        am_adap_reader_init(&rd_param, channel);
        am_adap_pixel_init(&pixel_param, channel);
        am_adap_alig_init(&alig_param, channel);

        //set tnr frame_vs
        //uint32_t data = 0;
        //adap_read(MIPI_ADAPT_FE_MUX_CTL4, MISC_IO, &data);
        //adap_write(MIPI_ADAPT_FE_MUX_CTL4, MISC_IO,(0<<8)  | (2<<14));//15:8  wr frame vs sel isp frame start
        //adap_write(MIPI_ADAPT_FE_MUX_CTL4, MISC_IO, data|(0<<16) | (2<<22));//23:16 rd frame vs sel isp frame start
        adap_wr_bit(MIPI_ADAPT_ALIG_CNTL10, PIXEL_IO, 0xf, 24, 4 );  //wait ds frame sync
        adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL1, MISC_IO, 0xf, 24, 4);
        adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL1, MISC_IO, 0xf, 20, 4);
        adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL2, MISC_IO, 0x1, 22, 2);    //ds0 see fe_vs
        adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL2, MISC_IO, 0x1, 30, 2);    //change see fe_vs
        adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL3, MISC_IO, 0x1,  6, 2);     //change see fe_vs
        adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL3, MISC_IO, 0x1, 14, 2);    //change see fe_vs

        adap_wr_bit(MIPI_ADAPT_ALIG_CNTL11, MISC_IO, 0x2, 16, 3);
        adap_wr_bit(MIPI_ADAPT_ALIG_CNTL12, MISC_IO, 0x2, 16, 3);
        adap_wr_bit(MIPI_ADAPT_ALIG_CNTL13, MISC_IO, 0x2, 16, 3);
        adap_wr_bit(MIPI_ADAPT_ALIG_CNTL14, MISC_IO, 0x2, 16, 3);
    }

    g_adap->f_adap = 1;

    if (adap_fsm[channel].para.mode != DCAM_MODE) {
        if (channel == CAM0_ACT) {
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_LAST, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_CURRENT, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_NEXT, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_NEXT_NEXT, 2);

            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 0, 24, 4);
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 0, 0, 2);
        } else if (channel == CAM1_ACT) {
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_LAST, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_CURRENT, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_NEXT, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_NEXT_NEXT, 2);

            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 2, 24, 4);
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 2, 0, 2);
        } else {
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_LAST, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_CURRENT, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_NEXT, 2);
            adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_NEXT_NEXT, 2);

            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 3, 24, 4);
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 3, 0, 2);
        }

        kfifo_ret = SCMIF_BUSY;
        camera_notify(NOTIFY_UPDATE_SC03_CAMID, &kfifo_ret);
    }

    if (adap_fsm[channel].para.mode != DIR_MODE) {
        if (channel == CAM0_ACT)
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 4, 24, 4); //interrput source for sc
        else if(channel == CAM1_ACT)
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 6, 24, 4);
        else if(channel == CAM2_ACT)
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 7, 24, 4);
    }

    return 0;
}

int am_adap_start(uint8_t channel, uint8_t dcam)
{
    adap_fsm[channel].cam_en = CAM_EN;
    if ( dcam ) {
        am_adap_frontend_start(channel);
        if ((adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_EN) {
            am_adap_alig_start(channel);
            am_adap_pixel_start(channel);
            am_adap_reader_start(channel);
            memset(camera_frame_fifo, 0, CAMERA_QUEUE_NUM * sizeof(uint32_t));
            g_adap->read_frame_ptr = 0;
            g_adap->write_frame_ptr = 0;
            if (channel == CAM0_ACT) {
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_LAST, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_CURRENT, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_NEXT, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM0_ACT, CAM_NEXT_NEXT, 2);
            } else if (channel == CAM1_ACT) {
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_LAST, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_CURRENT, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_NEXT, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM1_ACT, CAM_NEXT_NEXT, 2);
            } else {
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_LAST, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_CURRENT, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_NEXT, 2);
                adap_wr_bit(MIPI_OTHER_CNTL0, RD_IO, CAM2_ACT, CAM_NEXT_NEXT, 2);
            }
        }
    } else {
        am_adap_alig_start(channel);
        am_adap_pixel_start(channel);
        am_adap_reader_start(channel);
        am_adap_frontend_start(channel);
        if (channel == CAM0_ACT) {
            adap_fsm[1].cam_en = CAM_DIS;
            adap_fsm[2].cam_en = CAM_DIS;
        } else if (channel == CAM1_ACT) {
            adap_fsm[0].cam_en = CAM_DIS;
            adap_fsm[2].cam_en = CAM_DIS;
        } else if(channel == CAM2_ACT) {
            adap_fsm[0].cam_en = CAM_DIS;
            adap_fsm[1].cam_en = CAM_DIS;
        }
    }

    return 0;
}

int am_adap_reset(uint8_t channel)
{
    switch (channel) {
        case FRONTEND0_IO:
            adap_write(CSI2_GEN_CTRL0, FRONTEND0_IO, 0x00000000);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND0_IO, 1, 0, 1);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND0_IO, 0, 0, 1);
            break;
        case FRONTEND1_IO:
            adap_write(CSI2_GEN_CTRL0, FRONTEND1_IO, 0x00000000);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND1_IO, 1, 0, 1);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND1_IO, 0, 0, 1);
            break;
        case FRONTEND2_IO:
            adap_write(CSI2_GEN_CTRL0, FRONTEND2_IO, 0x00000000);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND2_IO, 1, 0, 1);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND2_IO, 0, 0, 1);
            break;
        case FRONTEND3_IO:
            adap_write(CSI2_GEN_CTRL0, FRONTEND3_IO, 0x00000000);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND3_IO, 1, 0, 1);
            adap_wr_bit(CSI2_CLK_RESET, FRONTEND3_IO, 0, 0, 1);
            break;
    }

    adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0, 0, 1);
    adap_wr_bit(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, 0, 0, 1);
    system_timer_usleep(1000);
    adap_wr_bit(MIPI_TOP_CSI2_CTRL0, CMPR_CNTL_IO, 1, 6, 1);
    adap_wr_bit(MIPI_TOP_CSI2_CTRL0, CMPR_CNTL_IO, 0, 6, 1);

    pr_err("am_adap_reset");
    return 0;
}

int am_adap_deinit(uint8_t channel)
{
    adap_fsm[channel].cam_en = CAM_DIS;

    if ( (adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_DIS ) {
        am_adap_reset(ADAP0_PATH);
        am_adap_reset(ADAP2_PATH);
        am_adap_reset(ADAP3_PATH);
    }

    if (adap_fsm[channel].para.mode == DDR_MODE) {
        am_disable_irq(channel);
        am_adap_free_mem(channel);
        if (g_adap->f_fifo && ((adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_DIS)) {
            adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 0, 9, 1);  //align rd done
            g_adap->f_fifo = 0;
            kfifo_free(&adap_fsm[CAM0_ACT].adapt_fifo);
            kfifo_free(&adap_fsm[CAM1_ACT].adapt_fifo);
            kfifo_free(&adap_fsm[CAM2_ACT].adapt_fifo);
        }
    } else if (adap_fsm[channel].para.mode == DCAM_MODE) {
        am_disable_irq(channel);
        am_adap_free_mem(channel);
        // switch sc interrput source
        if (channel == CAM0_ACT)
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 6, 24, 4);
        else if (channel == CAM1_ACT)
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 4, 24, 4);
        else if (channel == CAM2_ACT)
            adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 4, 24, 4);

        if (g_adap->f_fifo && ((adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_DIS)) {
            adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 0, 9, 1);  //align rd done
            g_adap->f_fifo = 0;
            kfifo_free(&adap_fsm[CAM0_ACT].adapt_fifo);
            kfifo_free(&adap_fsm[CAM1_ACT].adapt_fifo);
            kfifo_free(&adap_fsm[CAM2_ACT].adapt_fifo);
        }
    } else if (adap_fsm[channel].para.mode == DOL_MODE) {
        am_adap_free_mem(channel);
    }

    if (g_adap->f_end_irq && ((adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_DIS)) {
        g_adap->f_end_irq = 0;
        free_irq( g_adap->rd_irq, (void *)g_adap );
    }

    adap_fsm[channel].control_flag = 0;
    adap_fsm[channel].wbuf_index = 0;
    adap_fsm[channel].next_buf_index = 0;
    if ((adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_DIS)
        g_adap->f_adap = 0;

    if ( g_adap->kadap_stream != NULL && ((adap_fsm[CAM0_ACT].cam_en + adap_fsm[CAM1_ACT].cam_en + adap_fsm[CAM2_ACT].cam_en) == CAM_DIS)) {
        kthread_stop( g_adap->kadap_stream);
        g_adap->kadap_stream = NULL;
        g_adap->read_frame_ptr = 0;
        g_adap->write_frame_ptr = 0;
    }

    return 0;
}

