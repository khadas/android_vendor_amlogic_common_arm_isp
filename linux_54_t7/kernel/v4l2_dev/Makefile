#
# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2011-2018 ARM or its affiliates
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; version 2.
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#

obj-m += iv009_isp.o

M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

ifeq ($(FW_SRC_OBJ),)
	FW_SRC := $(wildcard src/*.c src/*/*.c src/*/*/*.c app/*.c app/*/*.c)
	export FW_SRC_OBJ := $(FW_SRC:.c=.o)

	ifneq ($(wildcard src/fw_lib/libacamera_isp.a),)
		export FW_SRC_OBJ := $(FW_SRC_OBJ) src/fw_lib/libacamera_isp.a
	endif
endif

iv009_isp-objs += $(FW_SRC_OBJ)

LOCAL_INCLUDES := -I$(M_PATH)/app -I$(M_PATH)/inc -I$(M_PATH)/app/control \
		-I$(M_PATH)/inc/api -I$(M_PATH)/inc/isp -I$(M_PATH)/inc/sys \
		-I$(M_PATH)/src/platform -I$(M_PATH)/src/fw \
		-I$(M_PATH)/src/fw_lib -I$(M_PATH)/src/calibration \
		-I$(M_PATH)/src/driver/sensor -I$(M_PATH)/src/driver/lens

ifeq ($(SYSTEM_VERSION),)
    ccflags-y += -DCONFIG_ANDROID_OS
else
    ccflags-y += -DCONFIG_$(SYSTEM_VERSION)
endif

ifeq ($(PLATFORM_VERSION),)
    ccflags-y += -DCONFIG_T7
else
    ccflags-y += -DCONFIG_$(PLATFORM_VERSION)
endif

ifeq ($(SEAMLESS_SUPPORT),)
    ccflags-y += -DCONFIG_SEAMLESS_N
else
    ccflags-y += -DCONFIG_SEAMLESS_$(SEAMLESS_SUPPORT)
endif

ccflags-y += $(LOCAL_INCLUDES)
ccflags-y += -Wno-declaration-after-statement -Idrivers/staging/android/ \
		-Wno-error
EXTRA_CFLAGS += $(LOCAL_INCLUDES)

all:
	@$(MAKE) -C $(KERNEL_SRC) M=$(M)  modules
modules_install:
	@echo "$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install"
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib
	cp $(OUT_DIR)/$(M)/*.ko ${OUT_DIR}/../vendor_lib/modules/

clean:
	@$(MAKE) -C $(KERNEL_SRC) M=$(M) clean