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

#ifndef __AML_T7_CSIPHY_HW_H__
#define __AML_T7_CSIPHY_HW_H__

#define MIPI_PHY_CTRL                   (0x00 << 2) //0x00
#define MIPI_PHY_CLK_LANE_CTRL          (0x01 << 2) //0x04
#define MIPI_PHY_DATA_LANE_CTRL         (0x02 << 2) //0x08
#define MIPI_PHY_DATA_LANE_CTRL1        (0x03 << 2) //0x0c
#define MIPI_PHY_TCLK_MISS              (0x04 << 2) //0x10
#define MIPI_PHY_TCLK_SETTLE            (0x05 << 2) //0x14
#define MIPI_PHY_THS_EXIT               (0x06 << 2) //0x18
#define MIPI_PHY_THS_SKIP               (0x07 << 2) //0x1c
#define MIPI_PHY_THS_SETTLE             (0x08 << 2) //0x20
#define MIPI_PHY_TINIT                  (0x09 << 2) //0x24
#define MIPI_PHY_TULPS_C                (0x0a << 2) //0x28
#define MIPI_PHY_TULPS_S                (0x0b << 2) //0x2c
#define MIPI_PHY_TMBIAS                 (0x0c << 2) //0x30
#define MIPI_PHY_TLP_EN_W               (0x0d << 2) //0x34
#define MIPI_PHY_TLPOK                  (0x0e << 2) //0x38
#define MIPI_PHY_TWD_INIT               (0x0f << 2) //0x3c
#define MIPI_PHY_TWD_HS                 (0x10 << 2) //0x40
#define MIPI_PHY_AN_CTRL0               (0x11 << 2) //0x44
#define MIPI_PHY_AN_CTRL1               (0x12 << 2) //0x48
#define MIPI_PHY_AN_CTRL2               (0x13 << 2) //0x4c
#define MIPI_PHY_CLK_LANE_STS           (0x14 << 2) //0x50
#define MIPI_PHY_DATA_LANE0_STS	        (0x15 << 2) //0x54
#define MIPI_PHY_DATA_LANE1_STS	        (0x16 << 2) //0x58
#define MIPI_PHY_DATA_LANE2_STS	        (0x17 << 2) //0x5c
#define MIPI_PHY_DATA_LANE3_STS	        (0x18 << 2) //0x60
#define MIPI_PHY_ESC_CMD                (0x19 << 2) //0x64
#define MIPI_PHY_INT_CTRL               (0x1a << 2) //0x68
#define MIPI_PHY_INT_STS                (0x1b << 2) //0x6c

#define MIPI_PHY_MUX_CTRL0              (0xa1 << 2)
#define MIPI_PHY_MUX_CTRL1              (0xa2 << 2)

#define MIPI_CSI2_PHY0_BASE_ADDR        0xFE3BFC00
#define MIPI_CSI2_PHY1_BASE_ADDR        0xFE3BF800
#define MIPI_CSI2_PHY2_BASE_ADDR        0xFE3BF400
#define MIPI_CSI2_PHY3_BASE_ADDR        0xFE3BF000

#define MIPI_CSI2_HOST0_BASE_ADDR       0xFE3BEC00
#define MIPI_CSI2_HOST1_BASE_ADDR       0xFE3BE800
#define MIPI_CSI2_HOST2_BASE_ADDR       0xFE3BE400
#define MIPI_CSI2_HOST3_BASE_ADDR       0xFE3BE000

#define MIPI_ISP_BASE_ADDR              0xFA000000

// MIPI-CSI2 analog registers
 #define MIPI_CSI_PHY_CNTL0            (0x0000  << 2)
 #define MIPI_CSI_PHY_CNTL1            (0x0001  << 2)
 #define MIPI_CSI_PHY_CNTL2            (0x0002  << 2)
 #define MIPI_CSI_PHY_CNTL3            (0x0003  << 2)
 #define MIPI_CSI_PHY_CNTL4            (0x0004  << 2)
 #define MIPI_CSI_PHY_CNTL5            (0x0005  << 2)
 #define MIPI_CSI_PHY_CNTL6            (0x0006  << 2)
 #define MIPI_CSI_PHY_CNTL7            (0x0007  << 2)

// MIPI-CSI2 host registers (internal offset)
#define CSI2_HOST_VERSION              0x000
#define CSI2_HOST_N_LANES              0x004
#define CSI2_HOST_PHY_SHUTDOWNZ        0x008
#define CSI2_HOST_DPHY_RSTZ            0x00c
#define CSI2_HOST_CSI2_RESETN          0x010
#define CSI2_HOST_PHY_STATE            0x014
#define CSI2_HOST_DATA_IDS_1           0x018
#define CSI2_HOST_DATA_IDS_2           0x01c
#define CSI2_HOST_ERR1                 0x020
#define CSI2_HOST_ERR2                 0x024
#define CSI2_HOST_MASK1                0x028
#define CSI2_HOST_MASK2                0x02c
#define CSI2_HOST_PHY_TST_CTRL0        0x030
#define CSI2_HOST_PHY_TST_CTRL1        0x034

enum {
	DPHY_MD = 0,
	HOST_MD,
	APHY_MD
};

#endif /* __AML_T7_CSIPHY_HW_H__ */
