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

#ifndef __AML_T7_VIDEO_H__
#define __AML_T7_VIDEO_H__

#include <media/media-device.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-core.h>

enum aml_plane {
	AML_PLANE_A  = 0,
	AML_PLANE_B = 1,
	AML_PLANE_C = 2,
	AML_PLANE_MAX
};

enum {
	AML_OFF = 0,
	AML_ON,
};

struct aml_format {
	u32 width;
	u32 height;
	u32 code;
	u32 fourcc;
	u32 nplanes;
	u32 bpp;
};

struct aml_crop {
	u32 hstart;
	u32 vstart;
	u32 hsize;
	u32 vsize;
};

struct aml_buffer {
	struct vb2_v4l2_buffer vb;
	void *vaddr[AML_PLANE_MAX];
	dma_addr_t addr[AML_PLANE_MAX];
	int flag;
	struct list_head list;
};

struct aml_cap_ops {
	int (*cap_irq_handler)(void *video, int status);
	int (*cap_set_format)(void *video);
	int (*cap_cfg_buffer)(void *video, void *buff);
	void (*cap_stream_on)(void *video);
	void (*cap_stream_off)(void *video);
	void (*cap_flush_buffer)(void *video);
};

struct aml_video {
	int id;
	int belong_cam_idx;
	int status;
	char name[32];
	char *bus_info;
	struct device *dev;
	struct v4l2_device *v4l2_dev;
	struct media_pad pad;
	struct vb2_queue vb2_q;
	struct video_device vdev;
	const struct aml_format *format;
	u32 fmt_cnt;
	u32 frm_cnt;
	struct aml_crop acrop;
	struct aml_format afmt;
	struct v4l2_format f_current;
	struct aml_buffer *b_current;
	enum v4l2_buf_type type;
	struct media_pipeline *pipe;
	const struct aml_cap_ops *ops;
	spinlock_t buff_list_lock;
	struct mutex lock;
	struct mutex q_lock;
	void *priv;
	struct list_head head;
};

int aml_video_register(struct aml_video *video);
void aml_video_unregister(struct aml_video *video);

#endif /* __AML_P1_VIDEO_H__ */
