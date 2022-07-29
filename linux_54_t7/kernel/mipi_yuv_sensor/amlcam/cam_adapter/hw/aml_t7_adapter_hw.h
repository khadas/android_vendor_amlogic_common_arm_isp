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

#ifndef __AML_T7_ADAPTER_HW_H__
#define __AML_T7_ADAPTER_HW_H__

#define CSI2_CLK_RESET                       (0x00 << 2)
#define CSI2_GEN_CTRL0                       (0x01 << 2)
#define CSI2_GEN_CTRL1                       (0x02 << 2)
#define CSI2_X_START_END_ISP                 (0x03 << 2)
#define CSI2_Y_START_END_ISP                 (0x04 << 2)
#define CSI2_X_START_END_MEM                 (0x05 << 2)
#define CSI2_Y_START_END_MEM                 (0x06 << 2)
#define CSI2_VC_MODE                         (0x07 << 2)
#define CSI2_VC_MODE2_MATCH_MASK_A_L         (0x08 << 2)
#define CSI2_VC_MODE2_MATCH_MASK_A_H         (0x09 << 2)
#define CSI2_VC_MODE2_MATCH_A_L              (0x0a << 2)
#define CSI2_VC_MODE2_MATCH_A_H              (0x0b << 2)
#define CSI2_VC_MODE2_MATCH_B_L              (0x0c << 2)
#define CSI2_VC_MODE2_MATCH_B_H              (0x0d << 2)
#define CSI2_DDR_START_PIX                   (0x0e << 2)
#define CSI2_DDR_START_PIX_ALT               (0x0f << 2)
#define CSI2_DDR_STRIDE_PIX                  (0x10 << 2)
#define CSI2_DDR_START_OTHER                 (0x11 << 2)
#define CSI2_DDR_START_OTHER_ALT             (0x12 << 2)
#define CSI2_DDR_MAX_BYTES_OTHER             (0x13 << 2)
#define CSI2_INTERRUPT_CTRL_STAT             (0x14 << 2)
#define CSI2_VC_MODE2_MATCH_MASK_B_L         (0x15 << 2)
#define CSI2_VC_MODE2_MATCH_MASK_B_H         (0x16 << 2)
#define CSI2_GEN_STAT0                       (0x20 << 2)
#define CSI2_ERR_STAT0                       (0x21 << 2)
#define CSI2_PIC_SIZE_STAT                   (0x22 << 2) //0x88
#define CSI2_DDR_WPTR_STAT_PIX               (0x23 << 2)
#define CSI2_DDR_WPTR_STAT_OTHER             (0x24 << 2)
#define CSI2_STAT_MEM_0                      (0x25 << 2)
#define CSI2_STAT_MEM_1                      (0x26 << 2)
#define CSI2_STAT_MEM_2                      (0x27 << 2)
#define CSI2_STAT_GEN_SHORT_08               (0x28 << 2)
#define CSI2_STAT_GEN_SHORT_09               (0x29 << 2)
#define CSI2_STAT_GEN_SHORT_0A               (0x2a << 2)
#define CSI2_STAT_GEN_SHORT_0B               (0x2b << 2)
#define CSI2_STAT_GEN_SHORT_0C               (0x2c << 2)
#define CSI2_STAT_GEN_SHORT_0D               (0x2d << 2)
#define CSI2_STAT_GEN_SHORT_0E               (0x2e << 2)
#define CSI2_STAT_GEN_SHORT_0F               (0x2f << 2)
#define CSI2_STAT_TYPE_RCVD_L                (0x30 << 2)
#define CSI2_STAT_TYPE_RCVD_H                (0x31 << 2)
#define CSI2_DDR_START_PIX_B                 (0x32 << 2)
#define CSI2_DDR_START_PIX_B_ALT             (0x33 << 2)
#define CSI2_DDR_STRIDE_PIX_B                (0x34 << 2)
#define CSI2_DDR_WPTR_STAT_PIX_B             (0x35 << 2)
#define CSI2_STAT_MEM_3                      (0x36 << 2)
#define CSI2_AXI_QOS_CNTL0                   (0x37 << 2)
#define CSI2_AXI_QOS_CNTL1                   (0x38 << 2)
#define CSI2_AXI_UGT_CNTL0                   (0x39 << 2)
#define CSI2_AXI_QOS_ST                      (0x3a << 2)
#define CSI2_LINE_SUP_CNTL0                  (0x3b << 2)
#define CSI2_LINE_SUP_ST                     (0x3c << 2)

#define MIPI_TOP_ADAPT_DE_CTRL0              (0xc1 << 2)
#define MIPI_TOP_ISP_PENDING_MASK0           (0xf0 << 2)
#define MIPI_TOP_ISP_PENDING_MASK1           (0xf1 << 2)
#define MIPI_TOP_ISP_PENDING0                (0xf3 << 2)
#define MIPI_TOP_ISP_PENDING1                (0xf4 << 2)


enum {
	FRONTEND_MD = 0,
	ISPTOP_MD,
	MAX_MD,
};

#endif /* __AML_T7_ADAPTER_HW_H__ */
