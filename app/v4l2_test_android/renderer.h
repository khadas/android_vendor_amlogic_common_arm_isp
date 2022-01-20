/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */

#ifndef _RENDERER_H_
#define _RENDERER_H_

/* renderer modes */
typedef enum render_mode {
    AFD_RENDER_MODE_CENTER,
    AFD_RENDER_MODE_LEFT_TOP,
    AFD_RENDER_MODE_MAX
} render_mode_t;

/* renderer image parameter */
typedef struct image_info {
    unsigned char *ptr;
    int width;
    int height;
    int bpp;
    uint32_t fmt;
} image_info_t;

/* renderer function */
int renderImage(unsigned char *dst, struct fb_var_screeninfo vinfo, struct fb_fix_screeninfo finfo,
    unsigned char *src, int width, int height, render_mode_t mode, int fb_fd, int fb_index);

#endif