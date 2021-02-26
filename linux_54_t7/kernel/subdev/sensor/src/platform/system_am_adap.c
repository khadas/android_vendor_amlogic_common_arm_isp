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
#include "system_log.h"

#define AM_ADAPTER_NAME "amlogic, isp-adapter"
#define BOUNDRY             16
#define FRONTEDN_USED       3
#define RD_USED             1

resource_size_t ddr_buf[DDR_BUF_SIZE];
resource_size_t dol_buf[DOL_BUF_SIZE];

struct am_adap *g_adap = NULL;
struct am_adap_info para;
struct adaptfe_param fe_param[FRONTEDN_USED];
struct adaptrd_param rd_param;
struct adaptpixel_param pixel_param;
struct adaptalig_param alig_param;

struct kfifo adapt_fifo;

/*we allocte from CMA*/
static uint8_t *isp_cma_mem = NULL;
static struct page *cma_pages = NULL;
static resource_size_t buffer_start;

#define DEFAULT_ADAPTER_BUFFER_SIZE 24

static int dump_width;
static int dump_height;
static int dump_flag;
static int dump_cur_flag;
static int dump_buf_index;
static int irq_count;
static int cur_buf_index;
static int current_flag;
static int control_flag;
static int wbuf_index;

static int fte1_index;
static int fte0_index;
static int buffer_index;

static int dump_dol_frame;
static int fte_state;

static struct completion wakeupdump;
static unsigned int data_process_para;
static unsigned int frontend1_flag = 0;
static unsigned int virtcam_flag = 0;

module_param(data_process_para, uint, 0664);
MODULE_PARM_DESC(data_process_para, "\n control inject or dump data parameter from adapter\n");

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

static void parse_param(char *buf_orig, char **parm, int num)
{
	char *ps, *token;
	unsigned int n = 0;
	char delim1[3] = " \n";

	ps = buf_orig;

	token = strsep(&ps, delim1);
	while (token != NULL) {
		if (*token != '\0') {
			parm[n++] = token;
		}
		if (n > num - 1) {
			printk("string element larger than array.\n");
			break;
		}
		token = strsep(&ps, delim1);
	}
}

static long getulfromstr(char* input)
{
	long out = 0;
	if (!input)
		return -1;
	if (kstrtoul(input, 10, &out) < 0) {
		return -1;
	}
	return out;
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
	update_wr_reg_bits(adr, io_type,
		           ((1<<len)-1)<<start, val<<start);
}

void inline adap_write(int addr, adap_io_type_t io_type, uint32_t val)
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

}

void inline adap_read(int addr, adap_io_type_t io_type, uint32_t *val)
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

static ssize_t adapt_frame_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("adapt-read.\n");
	uint8_t buf1[100];

	return sprintf(buf1,"dump flag:%d", 0);
}


static ssize_t adapt_frame_write(struct device *dev,
	struct device_attribute *attr, char const *buf, size_t size)
{
	long val = 0;
	ssize_t ret = size;
	char *virt_buf = NULL;
	int depth;
	uint32_t stride;
	uint32_t frame_size;
	char *buf_orig, *parm[10] = {NULL};
	unsigned int frame_index = 0;
	resource_size_t dump_buf_addr;

	if (!buf)
		return ret;

	buf_orig = kstrdup(buf, GFP_KERNEL);
	if (!buf_orig)
		return ret;

	parse_param(buf_orig, (char **)&parm, sizeof(parm)/sizeof(parm[0]));

	if (!parm[0]) {
		ret = -EINVAL;
		goto Err;
	}

	if (!parm[1] || (kstrtoul(parm[1], 10, &val) < 0)) {
		ret = -EINVAL;
		goto Err;
	} else {
		dump_cur_flag = val;
	}

	if (!parm[2] || (kstrtoul(parm[2], 10, &val) < 0)) {
		ret = -EINVAL;
		goto Err;
	} else {
		cur_buf_index = val;
		if (cur_buf_index >= DDR_BUF_SIZE) {
			pr_info("dump current index is invalid.\n");
			ret = -EINVAL;
			goto Err;
		}
	}

	depth = am_adap_get_depth();
	stride = ((dump_width * depth)/8);
	stride = ((stride + (BOUNDRY - 1)) & (~(BOUNDRY - 1)));
	frame_index = ((data_process_para) & (0xfffffff));
	frame_size = stride * dump_height;
	pr_info("dump width = %d, height = %d, size = %d\n",
		dump_width, dump_height, frame_size);

	if (dump_cur_flag) {
		dump_buf_addr = ddr_buf[cur_buf_index];
		pr_info("dump current buffer index %d.\n", cur_buf_index);
		if (dump_buf_addr)
			virt_buf = phys_to_virt(dump_buf_addr);
		write_index_to_file(parm[0], virt_buf, cur_buf_index, frame_size);
		dump_cur_flag = 0;
		current_flag = 0;
	} else if (frame_index > 0) {
		pr_info("dump the buf_index = %d\n", dump_buf_index);
		dump_buf_addr = ddr_buf[dump_buf_index];
		if (dump_buf_addr)
			virt_buf = phys_to_virt(dump_buf_addr);
		write_index_to_file(parm[0], virt_buf, dump_buf_index, frame_size);
		dump_flag = 0;
	} else {
		pr_info("No match condition to dump file.\n");
	}

Err:
	kfree(buf_orig);
	return ret;

}

static DEVICE_ATTR(adapt_frame, S_IRUGO | S_IWUSR, adapt_frame_read, adapt_frame_write);

static ssize_t dol_frame_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("adapt-read.\n");
	uint8_t buf1[100];

	return sprintf(buf1,"dump flag:%d", 0);
}

static ssize_t dol_frame_write(struct device *dev,
	struct device_attribute *attr, char const *buf, size_t size)
{
	ssize_t ret = size;
	char *virt_buf = NULL;
	int depth;
	uint32_t stride;
	uint32_t frame_size;
	uint32_t frame_count = 0;
	char *buf_orig, *parm[10] = {NULL};
	resource_size_t dump_buf_addr;

	if (!buf)
		return ret;

	buf_orig = kstrdup(buf, GFP_KERNEL);
	if (!buf_orig)
		return ret;

	parse_param(buf_orig, (char **)&parm, sizeof(parm)/sizeof(parm[0]));

	if (!parm[0]) {
		ret = -EINVAL;
		goto Err;
	}

	frame_count = getulfromstr(parm[1]);
	depth = am_adap_get_depth();
	stride = ((dump_width * depth)/8);
	stride = ((stride + (BOUNDRY - 1)) & (~(BOUNDRY - 1)));
	frame_size = stride * dump_height;
	pr_info("dump width = %d, height = %d, size = %d\n",
		dump_width, dump_height, frame_size);

restart:
	dump_dol_frame = 1;

	if (!wait_for_completion_timeout(&wakeupdump, msecs_to_jiffies(100))) {
		pr_err("wait for same frame timeout.\n");
		dump_dol_frame = 0;
		return ret;
	}

	dump_buf_addr = dol_buf[(buffer_index - 1) % 2];
	pr_info("dump ft0/ft1 buffer index %d.\n", buffer_index);
	if (dump_buf_addr)
		virt_buf = phys_to_virt(dump_buf_addr);
	write_index_to_file(parm[0], virt_buf, frame_count * 2, frame_size);

	dump_buf_addr = dol_buf[(buffer_index - 1)% 2 + 2];
	if (dump_buf_addr)
		virt_buf = phys_to_virt(dump_buf_addr);
	write_index_to_file(parm[0], virt_buf, frame_count * 2 - 1, frame_size);

	dump_dol_frame = 0;

	if (frame_count > 0) {
		frame_count --;
		goto restart;
	}

	/*if (buffer_index % 2 == 1) {
		adap_write(CSI2_DDR_START_PIX, FRONTEND0_IO, dol_buf[0]);
		adap_write(CSI2_DDR_START_PIX + FTE1_OFFSET, FRONTEND0_IO, dol_buf[2]);
	} else {
		adap_write(CSI2_DDR_START_PIX_ALT, FRONTEND0_IO, dol_buf[1]);
		adap_write(CSI2_DDR_START_PIX_ALT + FTE1_OFFSET, FRONTEND0_IO, dol_buf[3]);
	}*/

Err:
	kfree(buf_orig);
	return ret;

}

static DEVICE_ATTR(dol_frame, S_IRUGO | S_IWUSR, dol_frame_read, dol_frame_write);

static int write_data_to_buf(char *path, char *buf, int size)
{
	int ret = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	unsigned int r_size = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_info("read error.\n");
		return -1;
	}
	r_size = vfs_read(fp, buf, size, &pos);
	pr_info("read r_size = %u, size = %u\n", r_size, size);

	vfs_fsync(fp, 0);
	filp_close(fp, NULL);
	set_fs(old_fs);

	return ret;
}

static const char *adapt_inject_usage_str = {
	"Usage:\n"
	"echo <src_path> <width> <height> <bit_depth> > /sys/devices/platform/ff650000.isp-adapter/inject_frame\n"
};

static ssize_t err_note(void) {
	uint8_t buf1[128];
	return sprintf(buf1, "%s\n", adapt_inject_usage_str);
}

static ssize_t inject_frame_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", adapt_inject_usage_str);
}


static ssize_t inject_frame_write(struct device *dev,
	struct device_attribute *attr, char const *buf, size_t size)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
	char *buf_orig, *parm[30] = {NULL};
#else
    char *buf_orig, *parm[100] = {NULL};
#endif
	long stride = 0;
	long frame_width = 0;
	long frame_height = 0;
	long bit_depth = 0;
	char *virt_buf = NULL;
	uint32_t file_size;
	ssize_t ret = size;

	if (!buf)
		return ret;

	buf_orig = kstrdup(buf, GFP_KERNEL);
	if (!buf_orig)
		return ret;

	parse_param(buf_orig, (char **)&parm, sizeof(parm)/sizeof(parm[0]));

	if (!parm[0]) {
		ret = -EINVAL;
		pr_err("error---path--->:%s\n", adapt_inject_usage_str);
		goto Err;
	}

	pr_info("file_path = %s\n", parm[0]);

	frame_width = getulfromstr(parm[1]);
	frame_height = getulfromstr(parm[2]);
	bit_depth = getulfromstr(parm[3]);
	if (frame_width < 0 || frame_height < 0 || bit_depth < 0) {
		ret = -EINVAL;
		goto Err;
	}

	stride = (frame_width * bit_depth)/8;
	stride = ((stride + (BOUNDRY - 1)) & (~(BOUNDRY - 1)));
	if (ddr_buf[DDR_BUF_SIZE - 1] != 0)
		virt_buf = phys_to_virt(ddr_buf[DDR_BUF_SIZE - 1]);
	file_size = stride * frame_height;
	pr_info("inject frame width = %ld, height = %ld, bitdepth = %ld, size = %d\n",
		frame_width, frame_height,
		bit_depth, file_size);
	write_data_to_buf(parm[0], virt_buf, file_size);

Err:
	if (ret < 0)
		err_note();
	kfree(buf_orig);
	return ret;
}

static DEVICE_ATTR(inject_frame, S_IRUGO | S_IWUSR, inject_frame_read, inject_frame_write);

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
	//t_adap->adap_buf_size = 48;

	device_create_file(&(t_adap->p_dev->dev), &dev_attr_adapt_frame);
	device_create_file(&(t_adap->p_dev->dev), &dev_attr_inject_frame);
	device_create_file(&(t_adap->p_dev->dev), &dev_attr_dol_frame);
	system_dbg_create(&(t_adap->p_dev->dev));

	g_adap = t_adap;

	bypass_init(); //bypass de_compress_TNR and de_compress_ADAPT

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

	device_remove_file(&(t_adap->p_dev->dev), &dev_attr_adapt_frame);
	device_remove_file(&(t_adap->p_dev->dev), &dev_attr_inject_frame);
	device_remove_file(&(t_adap->p_dev->dev), &dev_attr_dol_frame);
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
	int inject_data_flag;
	int dump_data_flag;

	memset(&para, 0, sizeof(struct am_adap_info));
	memcpy(&para, info, sizeof(struct am_adap_info));
	frontend1_flag = ((data_process_para >> 30) & 0x1);
	inject_data_flag = ((data_process_para >> 29) & 0x1);
	dump_data_flag = ((data_process_para >> 28) & 0x1);

	pr_info("inject_data_flag = %x, dump_data_flag = %x\n",
		 inject_data_flag, dump_data_flag);

	if ((inject_data_flag) || (dump_data_flag) || (virtcam_flag)) {
		para.mode = DDR_MODE;
	}
	dump_width = para.img.width;
	dump_height = para.img.height;
}

int get_fte1_flag(void)
{
	frontend1_flag = (data_process_para >> 30) & 0x1;
	return frontend1_flag;
}

int am_adap_get_depth(void)
{
	int depth = 0;
	switch (para.fmt) {
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

int am_disable_irq(void)
{
	//disable irq mask
	adap_write(CSI2_INTERRUPT_CTRL_STAT, FRONTEND0_IO, 0);
	adap_write(MIPI_ADAPT_IRQ_MASK0, ALIGN_IO, 0);

	return 0;
}

/*
 *========================AM ADAPTER FRONTEND INTERFACE========================
 */

void am_adap_frontend_start(int port)
{
	int fe_io = 0;
	switch (fe_param[port].fe_sel) {
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


int am_adap_frontend_init(struct adaptfe_param* prm) {
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

    if (para.mode == DDR_MODE) {
		prm->fe_mem_ping_addr = ddr_buf[0];
		prm->fe_mem_pong_addr = ddr_buf[0];
		prm->fe_mem_other_addr= ddr_buf[1];
	} else if (para.mode == DOL_MODE) {
		prm->fe_mem_ping_addr = dol_buf[0];
		prm->fe_mem_pong_addr = dol_buf[1];
		prm->fe_mem_other_addr= dol_buf[1];
	} else {
		prm->fe_mem_ping_addr = ddr_buf[0];
		prm->fe_mem_pong_addr = ddr_buf[0];
		prm->fe_mem_other_addr= ddr_buf[1];
	}

	mem_addr0_a = prm->fe_mem_ping_addr;
	mem_addr0_b = prm->fe_mem_pong_addr;
	mem_addr1_a = prm->fe_mem_ping_addr;
	mem_addr1_b = prm->fe_mem_pong_addr;

	switch (prm->fe_work_mode) {
		case MODE_MIPI_RAW_SDR_DDR:
			cfg_all_to_mem	    = 1;
			pingpong_en         = 0;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 0;
			vc_mode             = 0x11110000;
			cfg_line_sup_vs_en  = 1;
			cfg_line_sup_vs_sel = 1;
			cfg_line_sup_sel    = 1;
			break;
		case MODE_MIPI_RAW_SDR_DIRCT:
			cfg_all_to_mem	    = 0;
			pingpong_en		    = 0;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 0;
			vc_mode             = 0x11110000;
			cfg_line_sup_vs_en  = 0;
			cfg_line_sup_vs_sel = 0;
			cfg_line_sup_sel    = 0;
			break;
		case MODE_MIPI_RAW_HDR_DDR_DDR:
			cfg_all_to_mem	    = 0;
			pingpong_en         = 0;
			cfg_isp2ddr_enable  = 1;
			cfg_isp2comb_enable = 0;
			vc_mode             = 0x111100f1;
			cfg_line_sup_vs_en  = 1;
			cfg_line_sup_vs_sel = 1;
			cfg_line_sup_sel    = 1;
			break;
		case MODE_MIPI_RAW_HDR_DDR_DIRCT:
			cfg_all_to_mem	    = 0;
			pingpong_en         = 1;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 0;
			vc_mode             = 0x111100f1;
			cfg_line_sup_vs_en  = 1;
			cfg_line_sup_vs_sel = 1;
			cfg_line_sup_sel    = 1;
			break;
		case MODE_MIPI_YUV_SDR_DDR:
			cfg_all_to_mem	    = 1;
			pingpong_en         = 0;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 0;
			vc_mode             = 0x11110000;
			cfg_line_sup_vs_en  = 0;
			cfg_line_sup_vs_sel = 0;
			cfg_line_sup_sel    = 0;
			break;
		case MODE_MIPI_RGB_SDR_DDR:
			cfg_all_to_mem	    = 1;
			pingpong_en         = 0;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 0;
			vc_mode             = 0x11110000;
			cfg_line_sup_vs_en  = 0;
			cfg_line_sup_vs_sel = 0;
			cfg_line_sup_sel    = 0;
			break;
		case MODE_MIPI_YUV_SDR_DIRCT:
			cfg_all_to_mem	    = 0;
			pingpong_en         = 0;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 1;
			vc_mode             = 0x11110000;
			cfg_line_sup_vs_en  = 0;
			cfg_line_sup_vs_sel = 0;
			cfg_line_sup_sel    = 0;
			break;
		case MODE_MIPI_RGB_SDR_DIRCT:
			cfg_all_to_mem	    = 0;
			pingpong_en         = 0;
			cfg_isp2ddr_enable  = 0;
			cfg_isp2comb_enable = 1;
			vc_mode             = 0x11110000;
			cfg_line_sup_vs_en  = 0;
			cfg_line_sup_vs_sel = 0;
			cfg_line_sup_sel    = 0;
			break;
		default:
			cfg_all_to_mem	    = 0;
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
	if ( (fe_io == FRONTEND1_IO) && frontend1_flag ) {
		adap_write(CSI2_DDR_START_PIX      , fe_io, dol_buf[2]);
		adap_write(CSI2_DDR_START_PIX_ALT  , fe_io, dol_buf[3]);
	}
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
                                         pingpong_en		    << 5 |
                                         0x01	                << 12 |
                                         0x01				    << 16 |
                                         cfg_isp2ddr_enable     << 25 |
                                         cfg_isp2comb_enable    << 26);

	if (prm->fe_work_mode == MODE_MIPI_RAW_HDR_DDR_DIRCT) {
		if (para.type == DOL_LINEINFO) {
			adap_write(CSI2_VC_MODE, fe_io, 0x11110052);
			if ((fe_io == FRONTEND1_IO) && frontend1_flag )
				adap_write(CSI2_VC_MODE, fe_io, 0x11110046);   //ft1 vc_mode
			if (para.fmt == MIPI_CSI_RAW10 ) {
				adap_write(CSI2_VC_MODE2_MATCH_MASK_A_L, fe_io, 0x6e6e6e6e);
				adap_write(CSI2_VC_MODE2_MATCH_MASK_A_H, fe_io, 0xffffff00);
				adap_write(CSI2_VC_MODE2_MATCH_A_L, fe_io, 0x90909090);
				adap_write(CSI2_VC_MODE2_MATCH_A_H, fe_io, 0x55);
				adap_write(CSI2_VC_MODE2_MATCH_MASK_B_L, fe_io, 0x6e6e6e6e);
				adap_write(CSI2_VC_MODE2_MATCH_MASK_B_H, fe_io, 0xffffff00);
				adap_write(CSI2_VC_MODE2_MATCH_B_L, fe_io, 0x90909090);
				adap_write(CSI2_VC_MODE2_MATCH_B_H, fe_io, 0xaa);
			} else if (para.fmt == MIPI_CSI_RAW12) {
				adap_write(CSI2_VC_MODE2_MATCH_MASK_A_L, fe_io, 0xff000101);
				adap_write(CSI2_VC_MODE2_MATCH_MASK_A_H, fe_io, 0xffffffff);
				adap_write(CSI2_VC_MODE2_MATCH_A_L, fe_io, 0x112424);
				adap_write(CSI2_VC_MODE2_MATCH_A_H, fe_io, 0x0);
				adap_write(CSI2_VC_MODE2_MATCH_MASK_B_L, fe_io, 0xff000101);
				adap_write(CSI2_VC_MODE2_MATCH_MASK_B_H, fe_io, 0xffffffff);
				adap_write(CSI2_VC_MODE2_MATCH_B_L, fe_io, 0x222424);
				adap_write(CSI2_VC_MODE2_MATCH_B_H, fe_io, 0x0);
			}
			int long_exp_offset = para.offset.long_offset;
			int short_exp_offset = para.offset.short_offset;
			adap_wr_bit(CSI2_X_START_END_MEM, fe_io, 0xc, 0, 16);
			adap_wr_bit(CSI2_X_START_END_MEM, fe_io,
						0xc + para.img.width - 1, 16, 16);
			adap_wr_bit(CSI2_Y_START_END_MEM, fe_io,
						long_exp_offset, 0, 16);
			adap_wr_bit(CSI2_Y_START_END_MEM, fe_io,
						long_exp_offset + para.img.height - 1, 16, 16);
			//set short exposure offset
			adap_wr_bit(CSI2_X_START_END_ISP, fe_io, 0xc, 0, 16);
			adap_wr_bit(CSI2_X_START_END_ISP, fe_io,
						0xc + para.img.width - 1, 16, 16);
			adap_wr_bit(CSI2_Y_START_END_ISP, fe_io,
						short_exp_offset, 0, 16);
			adap_wr_bit(CSI2_Y_START_END_ISP, fe_io,
						short_exp_offset + para.img.height - 1, 16, 16);
		}else if (para.type == DOL_VC) {
			adap_write(CSI2_VC_MODE, fe_io, 0x11220040);
			if ((fe_io == FRONTEND1_IO) && frontend1_flag )
				adap_write(CSI2_VC_MODE, fe_io, 0x22110000);
		}
	}
	return 0;
}


/*
 *========================AM ADAPTER READER INTERFACE==========================
 */

void am_adap_reader_start(void)
{
	uint32_t data = 0;
	adap_read(MIPI_ADAPT_DDR_RD0_CNTL0,  RD_IO, &data);
    adap_write(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, data | (1 << 31));
	adap_read(MIPI_ADAPT_DDR_RD1_CNTL0,  RD_IO, &data);
    adap_write(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, data | (1 << 31));
}

int am_adap_reader_init(struct adaptrd_param* prm) {
	uint32_t dol_mode,pingpong_mode;
	uint32_t port_sel_0;
	uint32_t port_sel_1;
	uint32_t ddr_rden_0;
	uint32_t ddr_rden_1;
	uint32_t ddr_rd0_ping,ddr_rd0_pong;
	uint32_t ddr_rd1_ping,ddr_rd1_pong;
	uint32_t continue_mode = 0;
	uint32_t data = 0;

	//wire	direct_path_sel = port_sel == 2'b00;
	//wire	ddr_path_sel	= port_sel == 2'b01;
	//wire	comb_path_sel	= port_sel == 2'b10;
    if (para.mode == DDR_MODE) {
		prm->rd_mem_ping_addr = ddr_buf[0];
		prm->rd_mem_pong_addr = ddr_buf[0];
	} else if (para.mode == DOL_MODE) {
		prm->rd_mem_ping_addr = dol_buf[0];
		prm->rd_mem_pong_addr = dol_buf[1];
	} else {
		prm->rd_mem_ping_addr = ddr_buf[0];
		prm->rd_mem_pong_addr = ddr_buf[0];
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
                                         ddr_rden_0	<< 0);
	adap_read( MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, &data);
	adap_write(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, data |
                                         ddr_rden_1	<< 0);
	return 0;
}

/*
 *========================AM ADAPTER PIXEL INTERFACE===========================
 */

void am_adap_pixel_start(void)
{
	uint32_t data = 0;
	adap_read( MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, &data);
	adap_write(MIPI_ADAPT_PIXEL0_CNTL0, PIXEL_IO, data | pixel_param.pixel_en_0<< 31);
	adap_read( MIPI_ADAPT_PIXEL1_CNTL0, PIXEL_IO, &data);
	adap_write(MIPI_ADAPT_PIXEL1_CNTL0, PIXEL_IO, data | pixel_param.pixel_en_1 << 31);

}

int am_adap_pixel_init(struct adaptpixel_param* prm) {
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

void am_adap_alig_start(void)
{
	int width, height, alig_width, alig_height;
	width = para.img.width;
	height = para.img.height;
	if (virtcam_flag)
		alig_width = width + 2000; //hblank > 3840
	else
		alig_width = width + 40; //hblank > 32 cycles
	alig_height = height + 60; //vblank > 48 lines
	//val = width + 35; // width < val < alig_width
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, alig_width, 0, 13);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL0, ALIGN_IO, alig_height, 16, 13);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL1, ALIGN_IO, width, 16, 13);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL2, ALIGN_IO, height, 16, 13);
	//adap_wr_bit(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, val, 16, 13);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL8, ALIGN_IO, 1, 31, 1);  // enable
}

int am_adap_alig_init(struct adaptalig_param* prm)
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

static int get_next_wr_buf_index(int inject_flag)
{
	int index = 0;
	wbuf_index = wbuf_index + 1;

	if (inject_flag) {
		index = wbuf_index % (DDR_BUF_SIZE - 1);
	} else {
		if (dump_flag) {
			index = wbuf_index % DDR_BUF_SIZE;
			if (index == dump_buf_index) {
				wbuf_index = wbuf_index + 1;
				index = wbuf_index % DDR_BUF_SIZE;
			}
		} else if (current_flag){
			index = wbuf_index % DDR_BUF_SIZE;
			if (index == cur_buf_index) {
				wbuf_index = wbuf_index + 1;
				index = wbuf_index % DDR_BUF_SIZE;
			}
		} else {
			index = wbuf_index % DDR_BUF_SIZE;
		}
	}

	return index;
}

static resource_size_t read_buf;
static int next_buf_index;
static irqreturn_t adpapter_isr(int irq, void *para)
{
	uint32_t data = 0;
	resource_size_t val = 0;
	int kfifo_ret = 0;
	int inject_data_flag = ((data_process_para >> 29) & 0x1);
	int frame_index = ((data_process_para) & (0xfffffff));

	adap_read(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, &data);

	if (data & (1 << 28)) {
		adap_wr_bit(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, 1, 28, 1); //clear write done irq
		if ((dump_cur_flag) && (next_buf_index == cur_buf_index)) {
			current_flag = 1;
		}
		if (!control_flag) {
			if (!kfifo_is_full(&adapt_fifo)) {
				kfifo_in(&adapt_fifo, &ddr_buf[next_buf_index], sizeof(resource_size_t));
				irq_count = irq_count + 1;
				if (irq_count == frame_index) {
					dump_buf_index = next_buf_index;
					dump_flag = 1;
				}
			} else {
				pr_err("adapt fifo is full .\n");
			}

			next_buf_index = get_next_wr_buf_index(inject_data_flag);
			val = ddr_buf[next_buf_index];
			adap_write(CSI2_DDR_START_PIX, FRONTEND0_IO, val);
		}
		if ((!control_flag) && (kfifo_len(&adapt_fifo) > 0)) {
			adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 31, 1);
			kfifo_ret = kfifo_out(&adapt_fifo, &val, sizeof(val));
			read_buf = val;
			control_flag = 1;
		}

	}

	if (data & (1 << 21)) {
		adap_wr_bit(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, 1, 21, 1);
		if (inject_data_flag) {
			adap_write(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, ddr_buf[DDR_BUF_SIZE - 1]);
		} else {
			adap_write(MIPI_ADAPT_DDR_RD0_CNTL2, RD_IO, read_buf);
		}
		if (virtcam_flag)
			adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 31, 1);
		control_flag = 0;
	}

	return IRQ_HANDLED;
}

static irqreturn_t dol_isr(int irq, void *para)
{
	uint32_t pending0 = 0;
	uint32_t pending1 = 0;

	adap_read(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, &pending0);
	adap_read(MIPI_TOP_ISP_PENDING1, CMPR_CNTL_IO, &pending1);

	if (pending0 & (1 << 28)) {
		adap_wr_bit(MIPI_TOP_ISP_PENDING0, CMPR_CNTL_IO, 1, 28, 1); //clear write done irq
		fte0_index ++;
	}

	if (frontend1_flag) {
		if (pending1 & (1 << 2)) {
			adap_wr_bit(MIPI_TOP_ISP_PENDING1, CMPR_CNTL_IO, 1, 2, 1); //clear write done irq
			fte1_index ++;
		}
	}

	if (dump_dol_frame) {   //replace ping/pong buffer
		pr_info("frontend0 index:%d, frontend1 index:%d\n",fte0_index, fte1_index);
		if (fte0_index == fte1_index) {
			/*if (fte0_index % 2 == 1) {
				adap_write(CSI2_DDR_START_PIX, FRONTEND0_IO, dol_buf[4]);
				adap_write(CSI2_DDR_START_PIX + FTE1_OFFSET, FRONTEND0_IO, dol_buf[5]);
			} else {
				adap_write(CSI2_DDR_START_PIX_ALT, FRONTEND0_IO, dol_buf[4]);
				adap_write(CSI2_DDR_START_PIX_ALT + FTE1_OFFSET, FRONTEND0_IO, dol_buf[5]);
			}*/
			buffer_index = fte0_index;
			dump_dol_frame = 0;
			complete(&wakeupdump);
		} /*else if (fte0_index > fte1_index){
			if (fte0_index % 2 == 1) {
				adap_write(CSI2_DDR_START_PIX, FRONTEND0_IO, dol_buf[4]);
			} else {
				adap_write(CSI2_DDR_START_PIX_ALT, FRONTEND0_IO, dol_buf[4]);
			}
		} else {
			if (fte1_index % 2 == 1) {
				adap_write(CSI2_DDR_START_PIX + FTE1_OFFSET, FRONTEND0_IO, dol_buf[5]);
			} else {
				adap_write(CSI2_DDR_START_PIX_ALT + FTE1_OFFSET, FRONTEND0_IO, dol_buf[5]);
			}
		}*/
	}
	return IRQ_HANDLED;
}

int am_adap_alloc_mem(void)
{

	if (para.mode == DDR_MODE) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
		cma_pages = dma_alloc_from_contiguous(
				  &(g_adap->p_dev->dev),
				  (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0, false);
#else
        cma_pages = dma_alloc_from_contiguous(
		         &(g_adap->p_dev->dev),
		         (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0);

#endif
		if (cma_pages) {
			buffer_start = page_to_phys(cma_pages);
		} else {
			pr_err("alloc cma pages failed.\n");
			return 0;
		}
		isp_cma_mem = phys_to_virt(buffer_start);
	} else if (para.mode == DOL_MODE) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
		cma_pages = dma_alloc_from_contiguous(
				  &(g_adap->p_dev->dev),
				  (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0, false);
#else
        cma_pages = dma_alloc_from_contiguous(
		          &(g_adap->p_dev->dev),
		          (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT, 0);

#endif
		if (cma_pages) {
			buffer_start = page_to_phys(cma_pages);
		} else {
			pr_err("alloc dol cma pages failed.\n");
			return 0;
		}
	}
	return 0;
}

int am_adap_free_mem(void)
{
	if (para.mode == DDR_MODE) {
		if (cma_pages) {
			dma_release_from_contiguous(
				 &(g_adap->p_dev->dev),
				 cma_pages,
				 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT);
			cma_pages = NULL;
			buffer_start = 0;
			pr_info("release alloc CMA buffer.\n");
		}
	} else if (para.mode == DOL_MODE) {
		if (cma_pages) {
			dma_release_from_contiguous(
				 &(g_adap->p_dev->dev),
				 cma_pages,
				 (g_adap->adap_buf_size * SZ_1M) >> PAGE_SHIFT);
			cma_pages = NULL;
			buffer_start = 0;
			pr_info("release alloc dol CMA buffer.\n");
		}
	}
	return 0;
}

int am_adap_init(int idx)
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
	control_flag = 0;
	wbuf_index = 0;
	dump_flag = 0;
	dump_cur_flag = 0;
	dump_buf_index = 0;
	next_buf_index = 0;
	irq_count = 0;
	cur_buf_index = 0;
	current_flag = 0;

	if (frontend1_flag) {
		fte0_index = 0;
		fte1_index = 0;
		dump_dol_frame = 0;
		fte_state = FTE_DONE;
		init_completion(&wakeupdump);
	}
	if (cma_pages) {
		am_adap_free_mem();
		cma_pages = NULL;
	}

	if ((para.mode == DDR_MODE) ||
		(para.mode == DOL_MODE)) {
		am_adap_alloc_mem();
		depth = am_adap_get_depth();
		if ((cma_pages) && (para.mode == DDR_MODE)) {
			//note important : ddr_buf[0] and ddr_buf[1] address should alignment 16 byte
			stride = (para.img.width * depth)/8;
			stride = ((stride + (BOUNDRY - 1)) & (~(BOUNDRY - 1)));
			ddr_buf[0] = buffer_start;
			ddr_buf[0] = (ddr_buf[0] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
			temp_buf = ddr_buf[0];
			buf = phys_to_virt(ddr_buf[0]);
			memset(buf, 0x0, (stride * para.img.height));
			for (i = 1; i < DDR_BUF_SIZE; i++) {
				ddr_buf[i] = temp_buf + (stride * (para.img.height));
				ddr_buf[i] = (ddr_buf[i] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
				temp_buf = ddr_buf[i];
				buf = phys_to_virt(ddr_buf[i]);
				if (virtcam_flag == 0)
					memset(buf, 0x0, (stride * para.img.height));
			}
		} else if ((cma_pages) && (para.mode == DOL_MODE)) {
			dol_buf[0] = buffer_start;
			dol_buf[0] = (dol_buf[0] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
			temp_buf = dol_buf[0];
			buf_cnt = 2;
			if (frontend1_flag)
				buf_cnt = 4;
			for (i = 1; i < buf_cnt; i++) {
				dol_buf[i] = temp_buf + ((para.img.width) * (para.img.height) * depth)/8;
				dol_buf[i] = (dol_buf[i] + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
				temp_buf = dol_buf[i];
			}
			if ( buf_cnt == 2 )
				dol_buf[1] = dol_buf[0];
		}
	}

	if (para.mode == DOL_MODE && g_adap->f_end_irq == 0 && frontend1_flag) {
		ret = request_irq(g_adap->rd_irq, &dol_isr, IRQF_SHARED | IRQF_TRIGGER_HIGH,
		"adapter-irq", (void *)g_adap);
		g_adap->f_end_irq = 1;
		pr_err("adapter dol irq = %d, ret = %d\n", g_adap->rd_irq, ret);
	}

	if (para.mode == DDR_MODE && g_adap->f_end_irq == 0) {
		ret = request_irq(g_adap->rd_irq, &adpapter_isr, IRQF_SHARED | IRQF_TRIGGER_HIGH,
		"adapter-irq", (void *)g_adap);
		g_adap->f_end_irq = 1;
		pr_info("adapter irq = %d, ret = %d\n", g_adap->rd_irq, ret);
	}

	if (para.mode == DDR_MODE && g_adap->f_fifo == 0) {
		kfifo_ret = kfifo_alloc(&adapt_fifo, PAGE_SIZE, GFP_KERNEL);
		if (kfifo_ret) {
			pr_info("alloc adapter fifo failed.\n");
			return kfifo_ret;
		}
		g_adap->f_fifo = 1;
	}

	am_adap_reset(idx);
	if (frontend1_flag)
		am_adap_reset(idx+1);
	pr_err("adapt_reset success, mode:%d(0:ddr 1:dir 2:dol)\n", para.mode);

	switch (para.mode) {
		case DIR_MODE:
			rd_param.rd_work_mode        = MODE_MIPI_RAW_SDR_DIRCT;
			pixel_param.pixel_work_mode	 = MODE_MIPI_RAW_SDR_DIRCT;
			alig_param.alig_work_mode    = MODE_MIPI_RAW_SDR_DIRCT;
			temp_work_mode               = MODE_MIPI_RAW_SDR_DIRCT;
			if ( para.type == DOL_YUV ) {
				rd_param.rd_work_mode		 = MODE_MIPI_YUV_SDR_DIRCT;
				pixel_param.pixel_work_mode  = MODE_MIPI_YUV_SDR_DIRCT;
				alig_param.alig_work_mode	 = MODE_MIPI_YUV_SDR_DIRCT;
				temp_work_mode				 = MODE_MIPI_YUV_SDR_DIRCT;
			}
			break;
		case DDR_MODE:
			rd_param.rd_work_mode        = MODE_MIPI_RAW_SDR_DDR;
			pixel_param.pixel_work_mode	 = MODE_MIPI_RAW_SDR_DDR;
			alig_param.alig_work_mode    = MODE_MIPI_RAW_SDR_DDR;
			temp_work_mode               = MODE_MIPI_RAW_SDR_DDR;
			adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, 28, 1 );
			adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, 21, 1 );
			break;
		case DOL_MODE:
			rd_param.rd_work_mode        = MODE_MIPI_RAW_HDR_DDR_DIRCT;
			pixel_param.pixel_work_mode	 = MODE_MIPI_RAW_HDR_DDR_DIRCT;
			alig_param.alig_work_mode    = MODE_MIPI_RAW_HDR_DDR_DIRCT;
			temp_work_mode               = MODE_MIPI_RAW_HDR_DDR_DIRCT;
			if (frontend1_flag) {
				adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, 28, 1 );
				adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK0, CMPR_CNTL_IO, 1, 21, 1 );
				adap_wr_bit(MIPI_TOP_ISP_PENDING_MASK1, CMPR_CNTL_IO, 1, 2, 1 );
			}
			break;
		default:
			pr_err("invalid adapt work mode!\n");
			break;
	}
	int cnt = 1;
	if (frontend1_flag)
		cnt = 2;
	for (i = idx; ((i < FRONTEDN_USED) && (i < idx + cnt)); i++) {
		fe_param[i].fe_sel              = i; //0~3
		fe_param[i].fe_work_mode        = temp_work_mode;
		fe_param[i].fe_mem_x_start      = 0;
		fe_param[i].fe_mem_x_end        = para.img.width - 1;
		fe_param[i].fe_mem_y_start      = 0;
		fe_param[i].fe_mem_y_end        = para.img.height - 1;
		fe_param[i].fe_isp_x_start      = 0;
		fe_param[i].fe_isp_x_end        = para.img.width - 1;
		fe_param[i].fe_isp_y_start      = 0x0;
		fe_param[i].fe_isp_y_end        = para.img.height - 1;
		fe_param[i].fe_mem_line_stride  = ceil_upper((am_adap_get_depth() * para.img.width), (8 * 16));
		fe_param[i].fe_mem_line_minbyte = (am_adap_get_depth() * para.img.width + 7) >> 3;
		fe_param[i].fe_int_mask         = 0x0;
		pr_err("config--frontend[%d]", i);
		am_adap_frontend_init(&fe_param[i]);
	}
	{
	    rd_param.rd_mem_line_stride     = (para.img.width * am_adap_get_depth() + 127) >> 7;
	    rd_param.rd_mem_line_size       = (para.img.width * am_adap_get_depth() + 127) >> 7;
	    rd_param.rd_mem_line_number     = para.img.height;
		pixel_param.pixel_data_type	    = para.fmt;
		pixel_param.pixel_isp_x_start   = 0;
		pixel_param.pixel_isp_x_end	    = para.img.width - 1;
		pixel_param.pixel_line_size	    = (para.img.width * am_adap_get_depth() + 127) >> 7;
		pixel_param.pixel_pixel_number  = para.img.width;
		alig_param.alig_hsize	        = para.img.width;
		alig_param.alig_vsize	        = para.img.height;
		am_adap_reader_init(&rd_param);
		am_adap_pixel_init(&pixel_param);
		am_adap_alig_init(&alig_param);
	}

	//set tnr frame_vs
	//uint32_t data = 0;
	//adap_read(MIPI_ADAPT_FE_MUX_CTL4, MISC_IO, &data);
	//adap_write(MIPI_ADAPT_FE_MUX_CTL4, MISC_IO,(0<<8)  | (2<<14));//15:8  wr frame vs sel isp frame start
	//adap_write(MIPI_ADAPT_FE_MUX_CTL4, MISC_IO, data|(0<<16) | (2<<22));//23:16 rd frame vs sel isp frame start
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL10, PIXEL_IO, 0xf, 24, 4 );  //wait ds frame sync
	adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL1, MISC_IO, 0xf, 24, 4);
	adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL2, MISC_IO, 0x1, 22, 2);    //ds0 see fe_vs
	adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL2, MISC_IO, 0x1, 30, 2);    //change see fe_vs
	adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL3, MISC_IO, 0x1,  6, 2);     //change see fe_vs
	adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL3, MISC_IO, 0x1, 14, 2);    //change see fe_vs

	if (idx == 2) {//use adapter 2
		adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 2, 24, 4);
		adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 2, 0, 2);
		adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL0, MISC_IO, 2, 8, 2);
	}
	adap_wr_bit(MIPI_ADAPT_FE_MUX_CTL1, MISC_IO, 0xf, 20, 4);

	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL11, MISC_IO, 0x2, 16, 3);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL12, MISC_IO, 0x2, 16, 3);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL13, MISC_IO, 0x2, 16, 3);
	adap_wr_bit(MIPI_ADAPT_ALIG_CNTL14, MISC_IO, 0x2, 16, 3);

	return 0;
}

int am_adap_start(int idx)
{
#if ISP_HAS_CMPR_ADAPT
	if(para.mode == DOL_MODE)
		aml_adap_decmpr_init(1, para.img.width, para.img.height, para.fmt);
#else
	//bypass_init();
#endif

	am_adap_alig_start();
	am_adap_pixel_start();
	am_adap_reader_start();
	am_adap_frontend_start(idx);
	if (frontend1_flag)
		am_adap_frontend_start(idx+1);

	if (virtcam_flag) {
		read_buf = ddr_buf[0];
		adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 1, 31, 1);
	}

	return 0;
}

int am_adap_reset(int idx)
{
    switch (idx) {
		case FRONTEND0_IO:
			adap_write(CSI2_GEN_CTRL0, FRONTEND0_IO, 0x00000000);
			break;
		case FRONTEND1_IO:
			adap_write(CSI2_GEN_CTRL0, FRONTEND1_IO, 0x00000000);
			break;
		case FRONTEND2_IO:
			adap_write(CSI2_GEN_CTRL0, FRONTEND2_IO, 0x00000000);
			break;
		case FRONTEND3_IO:
			adap_write(CSI2_GEN_CTRL0, FRONTEND3_IO, 0x00000000);
			break;
	}
	adap_wr_bit(MIPI_ADAPT_DDR_RD0_CNTL0, RD_IO, 0, 0, 1);
	adap_wr_bit(MIPI_ADAPT_DDR_RD1_CNTL0, RD_IO, 0, 0, 1);
	system_timer_usleep(1000);
	adap_wr_bit(MIPI_TOP_CSI2_CTRL0, CMPR_CNTL_IO, 1, 6, 1);
	adap_wr_bit(MIPI_TOP_CSI2_CTRL0, CMPR_CNTL_IO, 0, 6, 1);

	return 0;
}

int am_adap_deinit(int idx)
{
	am_adap_reset(idx);
	if (frontend1_flag)
		am_adap_reset(idx+1);
	am_disable_irq();

	if (para.mode == DDR_MODE) {
		am_adap_free_mem();
		if (g_adap->f_fifo)
		{
			g_adap->f_fifo = 0;
			kfifo_free(&adapt_fifo);
		}
	} else if (para.mode == DOL_MODE) {
		am_adap_free_mem();
	}

	if (g_adap->f_end_irq)
	{
		g_adap->f_end_irq = 0;
		free_irq( g_adap->rd_irq, (void *)g_adap );
	}

	control_flag = 0;
	wbuf_index = 0;
	dump_flag = 0;
	dump_cur_flag = 0;
	dump_buf_index = 0;
	next_buf_index = 0;
	irq_count = 0;
	cur_buf_index = 0;
	current_flag = 0;
	virtcam_flag = 0;

	if (frontend1_flag) {
		fte0_index = 0;
		fte1_index = 0;
		dump_dol_frame = 0;
		fte_state = 0;
	}
	return 0;
}

void adapt_set_virtcam(void)
{
	virtcam_flag = 1;
}
