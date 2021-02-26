/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
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

#if !defined(__OV5640_SENSOR_H__)
#define __OV5640_SENSOR_H__


/*-----------------------------------------------------------------------------
Initialization sequence - do not edit
-----------------------------------------------------------------------------*/

#include "sensor_init.h"

static acam_reg_t setting_1920_1080_2lane_672m_30fps[] = {
	{0x3103, 0x11, 0xff, 1},
	{0x3008, 0x82, 0xff, 1},
	{0x3008, 0x42, 0xff, 1},
	{0x3103, 0x03, 0xff, 1},
	{0x3017, 0x00, 0xff, 1},
	{0x3018, 0x00, 0xff, 1},
	{0x3034, 0x18, 0xff, 1},
	{0x3035, 0x11, 0xff, 1},
	{0x3036, 0x54, 0xff, 1},
	{0x3037, 0x13, 0xff, 1},
	{0x3108, 0x01, 0xff, 1},
	{0x3630, 0x36, 0xff, 1},
	{0x3631, 0x0e, 0xff, 1},
	{0x3632, 0xe2, 0xff, 1},
	{0x3633, 0x12, 0xff, 1},
	{0x3621, 0xe0, 0xff, 1},
	{0x3704, 0xa0, 0xff, 1},
	{0x3703, 0x5a, 0xff, 1},
	{0x3715, 0x78, 0xff, 1},
	{0x3717, 0x01, 0xff, 1},
	{0x370b, 0x60, 0xff, 1},
	{0x3705, 0x1a, 0xff, 1},
	{0x3905, 0x02, 0xff, 1},
	{0x3906, 0x10, 0xff, 1},
	{0x3901, 0x0a, 0xff, 1},
	{0x3731, 0x12, 0xff, 1},
	{0x3600, 0x08, 0xff, 1},
	{0x3601, 0x33, 0xff, 1},
	{0x302d, 0x60, 0xff, 1},
	{0x3620, 0x52, 0xff, 1},
	{0x371b, 0x20, 0xff, 1},
	{0x471c, 0x50, 0xff, 1},
	{0x3a13, 0x43, 0xff, 1},
	{0x3a18, 0x00, 0xff, 1},
	{0x3a19, 0xf8, 0xff, 1},
	{0x3635, 0x13, 0xff, 1},
	{0x3636, 0x03, 0xff, 1},
	{0x3634, 0x40, 0xff, 1},
	{0x3622, 0x01, 0xff, 1},
	{0x3c01, 0x34, 0xff, 1},
	{0x3c04, 0x28, 0xff, 1},
	{0x3c05, 0x98, 0xff, 1},
	{0x3c06, 0x00, 0xff, 1},
	{0x3c07, 0x07, 0xff, 1},
	{0x3c08, 0x00, 0xff, 1},
	{0x3c09, 0x1c, 0xff, 1},
	{0x3c0a, 0x9c, 0xff, 1},
	{0x3c0b, 0x40, 0xff, 1},
	{0x3820, 0x40, 0xff, 1},
	{0x3821, 0x06, 0xff, 1},
	{0x3814, 0x11, 0xff, 1},
	{0x3815, 0x11, 0xff, 1},
	{0x3800, 0x01, 0xff, 1},
	{0x3801, 0x50, 0xff, 1},
	{0x3802, 0x01, 0xff, 1},
	{0x3803, 0xb2, 0xff, 1},
	{0x3804, 0x08, 0xff, 1},
	{0x3805, 0xef, 0xff, 1},
	{0x3806, 0x05, 0xff, 1},
	{0x3807, 0xf1, 0xff, 1},
	{0x3808, 0x07, 0xff, 1},
	{0x3809, 0x80, 0xff, 1},
	{0x380a, 0x04, 0xff, 1},
	{0x380b, 0x38, 0xff, 1},
	{0x380c, 0x09, 0xff, 1},
	{0x380d, 0xc4, 0xff, 1},
	{0x380e, 0x04, 0xff, 1},
	{0x380f, 0x60, 0xff, 1},
	{0x3810, 0x00, 0xff, 1},
	{0x3811, 0x10, 0xff, 1},
	{0x3812, 0x00, 0xff, 1},
	{0x3813, 0x04, 0xff, 1},
	{0x3618, 0x04, 0xff, 1},
	{0x3612, 0x2b, 0xff, 1},
	{0x3708, 0x63, 0xff, 1},
	{0x3709, 0x12, 0xff, 1},
	{0x370c, 0x00, 0xff, 1},
	{0x3a02, 0x04, 0xff, 1},
	{0x3a03, 0x60, 0xff, 1},
	{0x3a08, 0x01, 0xff, 1},
	{0x3a09, 0x50, 0xff, 1},
	{0x3a0a, 0x01, 0xff, 1},
	{0x3a0b, 0x18, 0xff, 1},
	{0x3a0e, 0x03, 0xff, 1},
	{0x3a0d, 0x04, 0xff, 1},
	{0x3a14, 0x04, 0xff, 1},
	{0x3a15, 0x60, 0xff, 1},
	{0x4001, 0x02, 0xff, 1},
	{0x4004, 0x06, 0xff, 1},
	{0x4050, 0x6e, 0xff, 1},
	{0x4051, 0x8f, 0xff, 1},
	{0x3000, 0x00, 0xff, 1},
	{0x3002, 0x1c, 0xff, 1},
	{0x3004, 0xff, 0xff, 1},
	{0x3006, 0xc3, 0xff, 1},
	{0x300e, 0x45, 0xff, 1},
	{0x302e, 0x08, 0xff, 1},
	{0x4300, 0x30, 0xff, 1},
	{0x501f, 0x00, 0xff, 1},
	{0x5684, 0x07, 0xff, 1},
	{0x5685, 0xa0, 0xff, 1},
	{0x5686, 0x04, 0xff, 1},
	{0x5687, 0x40, 0xff, 1},
	{0x4713, 0x02, 0xff, 1},
	{0x4407, 0x04, 0xff, 1},
	{0x440e, 0x00, 0xff, 1},
	{0x460b, 0x37, 0xff, 1},
	{0x460c, 0x20, 0xff, 1},
	{0x4837, 0x0a, 0xff, 1},
	{0x3824, 0x04, 0xff, 1},
	{0x5000, 0xa7, 0xff, 1},
	{0x5001, 0x83, 0xff, 1},
	{0x5180, 0xff, 0xff, 1},
	{0x5181, 0xf2, 0xff, 1},
	{0x5182, 0x00, 0xff, 1},
	{0x5183, 0x14, 0xff, 1},
	{0x5184, 0x25, 0xff, 1},
	{0x5185, 0x24, 0xff, 1},
	{0x5186, 0x09, 0xff, 1},
	{0x5187, 0x09, 0xff, 1},
	{0x5188, 0x09, 0xff, 1},
	{0x5189, 0x75, 0xff, 1},
	{0x518a, 0x54, 0xff, 1},
	{0x518b, 0xe0, 0xff, 1},
	{0x518c, 0xb2, 0xff, 1},
	{0x518d, 0x42, 0xff, 1},
	{0x518e, 0x3d, 0xff, 1},
	{0x518f, 0x56, 0xff, 1},
	{0x5190, 0x46, 0xff, 1},
	{0x5191, 0xf8, 0xff, 1},
	{0x5192, 0x04, 0xff, 1},
	{0x5193, 0x70, 0xff, 1},
	{0x5194, 0xf0, 0xff, 1},
	{0x5195, 0xf0, 0xff, 1},
	{0x5196, 0x03, 0xff, 1},
	{0x5197, 0x01, 0xff, 1},
	{0x5198, 0x04, 0xff, 1},
	{0x5199, 0x12, 0xff, 1},
	{0x519a, 0x04, 0xff, 1},
	{0x519b, 0x00, 0xff, 1},
	{0x519c, 0x06, 0xff, 1},
	{0x519d, 0x82, 0xff, 1},
	{0x519e, 0x38, 0xff, 1},
	{0x5381, 0x1e, 0xff, 1},
	{0x5382, 0x5b, 0xff, 1},
	{0x5383, 0x08, 0xff, 1},
	{0x5384, 0x0a, 0xff, 1},
	{0x5385, 0x7e, 0xff, 1},
	{0x5386, 0x88, 0xff, 1},
	{0x5387, 0x7c, 0xff, 1},
	{0x5388, 0x6c, 0xff, 1},
	{0x5389, 0x10, 0xff, 1},
	{0x538a, 0x01, 0xff, 1},
	{0x538b, 0x98, 0xff, 1},
	{0x5300, 0x08, 0xff, 1},
	{0x5301, 0x30, 0xff, 1},
	{0x5302, 0x10, 0xff, 1},
	{0x5303, 0x00, 0xff, 1},
	{0x5304, 0x08, 0xff, 1},
	{0x5305, 0x30, 0xff, 1},
	{0x5306, 0x08, 0xff, 1},
	{0x5307, 0x16, 0xff, 1},
	{0x5309, 0x08, 0xff, 1},
	{0x530a, 0x30, 0xff, 1},
	{0x530b, 0x04, 0xff, 1},
	{0x530c, 0x06, 0xff, 1},
	{0x5480, 0x01, 0xff, 1},
	{0x5481, 0x08, 0xff, 1},
	{0x5482, 0x14, 0xff, 1},
	{0x5483, 0x28, 0xff, 1},
	{0x5484, 0x51, 0xff, 1},
	{0x5485, 0x65, 0xff, 1},
	{0x5486, 0x71, 0xff, 1},
	{0x5487, 0x7d, 0xff, 1},
	{0x5488, 0x87, 0xff, 1},
	{0x5489, 0x91, 0xff, 1},
	{0x548a, 0x9a, 0xff, 1},
	{0x548b, 0xaa, 0xff, 1},
	{0x548c, 0xb8, 0xff, 1},
	{0x548d, 0xcd, 0xff, 1},
	{0x548e, 0xdd, 0xff, 1},
	{0x548f, 0xea, 0xff, 1},
	{0x5490, 0x1d, 0xff, 1},
	{0x5580, 0x02, 0xff, 1},
	{0x5583, 0x40, 0xff, 1},
	{0x5584, 0x10, 0xff, 1},
	{0x5589, 0x10, 0xff, 1},
	{0x558a, 0x00, 0xff, 1},
	{0x558b, 0xf8, 0xff, 1},
	{0x5800, 0x23, 0xff, 1},
	{0x5801, 0x14, 0xff, 1},
	{0x5802, 0x0f, 0xff, 1},
	{0x5803, 0x0f, 0xff, 1},
	{0x5804, 0x12, 0xff, 1},
	{0x5805, 0x26, 0xff, 1},
	{0x5806, 0x0c, 0xff, 1},
	{0x5807, 0x08, 0xff, 1},
	{0x5808, 0x05, 0xff, 1},
	{0x5809, 0x05, 0xff, 1},
	{0x580a, 0x08, 0xff, 1},
	{0x580b, 0x0d, 0xff, 1},
	{0x580c, 0x08, 0xff, 1},
	{0x580d, 0x03, 0xff, 1},
	{0x580e, 0x00, 0xff, 1},
	{0x580f, 0x00, 0xff, 1},
	{0x5810, 0x03, 0xff, 1},
	{0x5811, 0x09, 0xff, 1},
	{0x5812, 0x07, 0xff, 1},
	{0x5813, 0x03, 0xff, 1},
	{0x5814, 0x00, 0xff, 1},
	{0x5815, 0x01, 0xff, 1},
	{0x5816, 0x03, 0xff, 1},
	{0x5817, 0x08, 0xff, 1},
	{0x5818, 0x0d, 0xff, 1},
	{0x5819, 0x08, 0xff, 1},
	{0x581a, 0x05, 0xff, 1},
	{0x581b, 0x06, 0xff, 1},
	{0x581c, 0x08, 0xff, 1},
	{0x581d, 0x0e, 0xff, 1},
	{0x581e, 0x29, 0xff, 1},
	{0x581f, 0x17, 0xff, 1},
	{0x5820, 0x11, 0xff, 1},
	{0x5821, 0x11, 0xff, 1},
	{0x5822, 0x15, 0xff, 1},
	{0x5823, 0x28, 0xff, 1},
	{0x5824, 0x46, 0xff, 1},
	{0x5825, 0x26, 0xff, 1},
	{0x5826, 0x08, 0xff, 1},
	{0x5827, 0x26, 0xff, 1},
	{0x5828, 0x64, 0xff, 1},
	{0x5829, 0x26, 0xff, 1},
	{0x582a, 0x24, 0xff, 1},
	{0x582b, 0x22, 0xff, 1},
	{0x582c, 0x24, 0xff, 1},
	{0x582d, 0x24, 0xff, 1},
	{0x582e, 0x06, 0xff, 1},
	{0x582f, 0x22, 0xff, 1},
	{0x5830, 0x40, 0xff, 1},
	{0x5831, 0x42, 0xff, 1},
	{0x5832, 0x24, 0xff, 1},
	{0x5833, 0x26, 0xff, 1},
	{0x5834, 0x24, 0xff, 1},
	{0x5835, 0x22, 0xff, 1},
	{0x5836, 0x22, 0xff, 1},
	{0x5837, 0x26, 0xff, 1},
	{0x5838, 0x44, 0xff, 1},
	{0x5839, 0x24, 0xff, 1},
	{0x583a, 0x26, 0xff, 1},
	{0x583b, 0x28, 0xff, 1},
	{0x583c, 0x42, 0xff, 1},
	{0x583d, 0xce, 0xff, 1},
	{0x5025, 0x00, 0xff, 1},
	{0x3a0f, 0x30, 0xff, 1},
	{0x3a10, 0x28, 0xff, 1},
	{0x3a1b, 0x30, 0xff, 1},
	{0x3a1e, 0x26, 0xff, 1},
	{0x3a11, 0x60, 0xff, 1},
	{0x3a1f, 0x14, 0xff, 1},
	{0x3008, 0x02, 0xff, 1},

	{ 0x0000, 0x0000, 0x0000, 0x0000 },
};

static acam_reg_t setting_2592_1944_2lane_672m_30fps[] = {
	{0x3103, 0x11, 0xff, 1},
	{0x3008, 0x82, 0xff, 1},
	{0x3008, 0x42, 0xff, 1},
	{0x3103, 0x03, 0xff, 1},
	{0x3017, 0x00, 0xff, 1},
	{0x3018, 0x00, 0xff, 1},
	{0x3034, 0x18, 0xff, 1},
	{0x3035, 0x11, 0xff, 1},
	{0x3036, 0x54, 0xff, 1},
	{0x3037, 0x13, 0xff, 1},
	{0x3108, 0x01, 0xff, 1},
	{0x3630, 0x36, 0xff, 1},
	{0x3631, 0x0e, 0xff, 1},
	{0x3632, 0xe2, 0xff, 1},
	{0x3633, 0x12, 0xff, 1},
	{0x3621, 0xe0, 0xff, 1},
	{0x3704, 0xa0, 0xff, 1},
	{0x3703, 0x5a, 0xff, 1},
	{0x3715, 0x78, 0xff, 1},
	{0x3717, 0x01, 0xff, 1},
	{0x370b, 0x60, 0xff, 1},
	{0x3705, 0x1a, 0xff, 1},
	{0x3905, 0x02, 0xff, 1},
	{0x3906, 0x10, 0xff, 1},
	{0x3901, 0x0a, 0xff, 1},
	{0x3731, 0x12, 0xff, 1},
	{0x3600, 0x08, 0xff, 1},
	{0x3601, 0x33, 0xff, 1},
	{0x302d, 0x60, 0xff, 1},
	{0x3620, 0x52, 0xff, 1},
	{0x371b, 0x20, 0xff, 1},
	{0x471c, 0x50, 0xff, 1},
	{0x3a13, 0x43, 0xff, 1},
	{0x3a18, 0x00, 0xff, 1},
	{0x3a19, 0xf8, 0xff, 1},
	{0x3635, 0x13, 0xff, 1},
	{0x3636, 0x03, 0xff, 1},
	{0x3634, 0x40, 0xff, 1},
	{0x3622, 0x01, 0xff, 1},
	{0x3c01, 0x34, 0xff, 1},
	{0x3c04, 0x28, 0xff, 1},
	{0x3c05, 0x98, 0xff, 1},
	{0x3c06, 0x00, 0xff, 1},
	{0x3c07, 0x07, 0xff, 1},
	{0x3c08, 0x00, 0xff, 1},
	{0x3c09, 0x1c, 0xff, 1},
	{0x3c0a, 0x9c, 0xff, 1},
	{0x3c0b, 0x40, 0xff, 1},
	{0x3820, 0x40, 0xff, 1},
	{0x3821, 0x06, 0xff, 1},
	{0x3814, 0x11, 0xff, 1},
	{0x3815, 0x11, 0xff, 1},
	{0x3800, 0x00, 0xff, 1},
	{0x3801, 0x00, 0xff, 1},
	{0x3802, 0x00, 0xff, 1},
	{0x3803, 0x00, 0xff, 1},
	{0x3804, 0x0a, 0xff, 1},
	{0x3805, 0x3f, 0xff, 1},
	{0x3806, 0x07, 0xff, 1},
	{0x3807, 0x9f, 0xff, 1},
	{0x3808, 0x0a, 0xff, 1},
	{0x3809, 0x20, 0xff, 1},
	{0x380a, 0x07, 0xff, 1},
	{0x380b, 0x98, 0xff, 1},
	{0x380c, 0x0b, 0xff, 1},
	{0x380d, 0x1c, 0xff, 1},
	{0x380e, 0x07, 0xff, 1},
	{0x380f, 0xb0, 0xff, 1},
	{0x3810, 0x00, 0xff, 1},
	{0x3811, 0x10, 0xff, 1},
	{0x3812, 0x00, 0xff, 1},
	{0x3813, 0x04, 0xff, 1},
	{0x3618, 0x04, 0xff, 1},
	{0x3612, 0x2b, 0xff, 1},
	{0x3708, 0x63, 0xff, 1},
	{0x3709, 0x12, 0xff, 1},
	{0x370c, 0x00, 0xff, 1},
	{0x3a02, 0x07, 0xff, 1},
	{0x3a03, 0xb0, 0xff, 1},
	{0x3a08, 0x01, 0xff, 1},
	{0x3a09, 0x27, 0xff, 1},
	{0x3a0a, 0x00, 0xff, 1},
	{0x3a0b, 0xf6, 0xff, 1},
	{0x3a0e, 0x06, 0xff, 1},
	{0x3a0d, 0x08, 0xff, 1},
	{0x3a14, 0x07, 0xff, 1},
	{0x3a15, 0xb0, 0xff, 1},
	{0x4001, 0x02, 0xff, 1},
	{0x4004, 0x06, 0xff, 1},
	{0x4050, 0x6e, 0xff, 1},
	{0x4051, 0x8f, 0xff, 1},
	{0x3000, 0x00, 0xff, 1},
	{0x3002, 0x1c, 0xff, 1},
	{0x3004, 0xff, 0xff, 1},
	{0x3006, 0xc3, 0xff, 1},
	{0x300e, 0x45, 0xff, 1},
	{0x302e, 0x08, 0xff, 1},
	{0x4300, 0x30, 0xff, 1},
	{0x4837, 0x0a, 0xff, 1},
	{0x501f, 0x00, 0xff, 1},
	{0x5684, 0x0a, 0xff, 1},
	{0x5685, 0x20, 0xff, 1},
	{0x5686, 0x07, 0xff, 1},
	{0x5687, 0x98, 0xff, 1},
	{0x440e, 0x00, 0xff, 1},
	{0x5000, 0xa7, 0xff, 1},
	{0x5001, 0x83, 0xff, 1},
	{0x5180, 0xff, 0xff, 1},
	{0x5181, 0xf2, 0xff, 1},
	{0x5182, 0x00, 0xff, 1},
	{0x5183, 0x14, 0xff, 1},
	{0x5184, 0x25, 0xff, 1},
	{0x5185, 0x24, 0xff, 1},
	{0x5186, 0x09, 0xff, 1},
	{0x5187, 0x09, 0xff, 1},
	{0x5188, 0x09, 0xff, 1},
	{0x5189, 0x75, 0xff, 1},
	{0x518a, 0x54, 0xff, 1},
	{0x518b, 0xe0, 0xff, 1},
	{0x518c, 0xb2, 0xff, 1},
	{0x518d, 0x42, 0xff, 1},
	{0x518e, 0x3d, 0xff, 1},
	{0x518f, 0x56, 0xff, 1},
	{0x5190, 0x46, 0xff, 1},
	{0x5191, 0xf8, 0xff, 1},
	{0x5192, 0x04, 0xff, 1},
	{0x5193, 0x70, 0xff, 1},
	{0x5194, 0xf0, 0xff, 1},
	{0x5195, 0xf0, 0xff, 1},
	{0x5196, 0x03, 0xff, 1},
	{0x5197, 0x01, 0xff, 1},
	{0x5198, 0x04, 0xff, 1},
	{0x5199, 0x12, 0xff, 1},
	{0x519a, 0x04, 0xff, 1},
	{0x519b, 0x00, 0xff, 1},
	{0x519c, 0x06, 0xff, 1},
	{0x519d, 0x82, 0xff, 1},
	{0x519e, 0x38, 0xff, 1},
	{0x5381, 0x1e, 0xff, 1},
	{0x5382, 0x5b, 0xff, 1},
	{0x5383, 0x08, 0xff, 1},
	{0x5384, 0x0a, 0xff, 1},
	{0x5385, 0x7e, 0xff, 1},
	{0x5386, 0x88, 0xff, 1},
	{0x5387, 0x7c, 0xff, 1},
	{0x5388, 0x6c, 0xff, 1},
	{0x5389, 0x10, 0xff, 1},
	{0x538a, 0x01, 0xff, 1},
	{0x538b, 0x98, 0xff, 1},
	{0x5300, 0x08, 0xff, 1},
	{0x5301, 0x30, 0xff, 1},
	{0x5302, 0x10, 0xff, 1},
	{0x5303, 0x00, 0xff, 1},
	{0x5304, 0x08, 0xff, 1},
	{0x5305, 0x30, 0xff, 1},
	{0x5306, 0x08, 0xff, 1},
	{0x5307, 0x16, 0xff, 1},
	{0x5309, 0x08, 0xff, 1},
	{0x530a, 0x30, 0xff, 1},
	{0x530b, 0x04, 0xff, 1},
	{0x530c, 0x06, 0xff, 1},
	{0x5480, 0x01, 0xff, 1},
	{0x5481, 0x08, 0xff, 1},
	{0x5482, 0x14, 0xff, 1},
	{0x5483, 0x28, 0xff, 1},
	{0x5484, 0x51, 0xff, 1},
	{0x5485, 0x65, 0xff, 1},
	{0x5486, 0x71, 0xff, 1},
	{0x5487, 0x7d, 0xff, 1},
	{0x5488, 0x87, 0xff, 1},
	{0x5489, 0x91, 0xff, 1},
	{0x548a, 0x9a, 0xff, 1},
	{0x548b, 0xaa, 0xff, 1},
	{0x548c, 0xb8, 0xff, 1},
	{0x548d, 0xcd, 0xff, 1},
	{0x548e, 0xdd, 0xff, 1},
	{0x548f, 0xea, 0xff, 1},
	{0x5490, 0x1d, 0xff, 1},
	{0x5580, 0x02, 0xff, 1},
	{0x5583, 0x40, 0xff, 1},
	{0x5584, 0x10, 0xff, 1},
	{0x5589, 0x10, 0xff, 1},
	{0x558a, 0x00, 0xff, 1},
	{0x558b, 0xf8, 0xff, 1},
	{0x5800, 0x23, 0xff, 1},
	{0x5801, 0x14, 0xff, 1},
	{0x5802, 0x0f, 0xff, 1},
	{0x5803, 0x0f, 0xff, 1},
	{0x5804, 0x12, 0xff, 1},
	{0x5805, 0x26, 0xff, 1},
	{0x5806, 0x0c, 0xff, 1},
	{0x5807, 0x08, 0xff, 1},
	{0x5808, 0x05, 0xff, 1},
	{0x5809, 0x05, 0xff, 1},
	{0x580a, 0x08, 0xff, 1},
	{0x580b, 0x0d, 0xff, 1},
	{0x580c, 0x08, 0xff, 1},
	{0x580d, 0x03, 0xff, 1},
	{0x580e, 0x00, 0xff, 1},
	{0x580f, 0x00, 0xff, 1},
	{0x5810, 0x03, 0xff, 1},
	{0x5811, 0x09, 0xff, 1},
	{0x5812, 0x07, 0xff, 1},
	{0x5813, 0x03, 0xff, 1},
	{0x5814, 0x00, 0xff, 1},
	{0x5815, 0x01, 0xff, 1},
	{0x5816, 0x03, 0xff, 1},
	{0x5817, 0x08, 0xff, 1},
	{0x5818, 0x0d, 0xff, 1},
	{0x5819, 0x08, 0xff, 1},
	{0x581a, 0x05, 0xff, 1},
	{0x581b, 0x06, 0xff, 1},
	{0x581c, 0x08, 0xff, 1},
	{0x581d, 0x0e, 0xff, 1},
	{0x581e, 0x29, 0xff, 1},
	{0x581f, 0x17, 0xff, 1},
	{0x5820, 0x11, 0xff, 1},
	{0x5821, 0x11, 0xff, 1},
	{0x5822, 0x15, 0xff, 1},
	{0x5823, 0x28, 0xff, 1},
	{0x5824, 0x46, 0xff, 1},
	{0x5825, 0x26, 0xff, 1},
	{0x5826, 0x08, 0xff, 1},
	{0x5827, 0x26, 0xff, 1},
	{0x5828, 0x64, 0xff, 1},
	{0x5829, 0x26, 0xff, 1},
	{0x582a, 0x24, 0xff, 1},
	{0x582b, 0x22, 0xff, 1},
	{0x582c, 0x24, 0xff, 1},
	{0x582d, 0x24, 0xff, 1},
	{0x582e, 0x06, 0xff, 1},
	{0x582f, 0x22, 0xff, 1},
	{0x5830, 0x40, 0xff, 1},
	{0x5831, 0x42, 0xff, 1},
	{0x5832, 0x24, 0xff, 1},
	{0x5833, 0x26, 0xff, 1},
	{0x5834, 0x24, 0xff, 1},
	{0x5835, 0x22, 0xff, 1},
	{0x5836, 0x22, 0xff, 1},
	{0x5837, 0x26, 0xff, 1},
	{0x5838, 0x44, 0xff, 1},
	{0x5839, 0x24, 0xff, 1},
	{0x583a, 0x26, 0xff, 1},
	{0x583b, 0x28, 0xff, 1},
	{0x583c, 0x42, 0xff, 1},
	{0x583d, 0xce, 0xff, 1},
	{0x5025, 0x00, 0xff, 1},
	{0x3a0f, 0x30, 0xff, 1},
	{0x3a10, 0x28, 0xff, 1},
	{0x3a1b, 0x30, 0xff, 1},
	{0x3a1e, 0x26, 0xff, 1},
	{0x3a11, 0x60, 0xff, 1},
	{0x3a1f, 0x14, 0xff, 1},
	{0x3008, 0x02, 0xff, 1},

	{ 0x0000, 0x0000, 0x0000, 0x0000 },
};

static acam_reg_t settings_context_os5640[] = {
    //stop sequence - address is 0x0000
    { 0x0000, 0x0000, 0x0000, 0x0000 }
};

static const acam_reg_t *seq_table[] = {
    setting_1920_1080_2lane_672m_30fps,
    setting_2592_1944_2lane_672m_30fps,
};

static const acam_reg_t *isp_seq_table[] = {
    settings_context_os5640,
};


#define SENSOR__OV5640_SEQUENCE_DEFAULT seq_table
#define SENSOR__OV5640_ISP_SEQUENCE_DEFAULT seq_table



#define SENSOR_OV5640_SEQUENCE_DEFAULT_PREVIEW    0
#define SENSOR_OV5640_ISP_CONTEXT_SEQ   0


#endif /* __OV5640_SENSOR_H__ */