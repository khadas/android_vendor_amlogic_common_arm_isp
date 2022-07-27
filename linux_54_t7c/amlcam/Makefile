
obj-m += amlcam.o

M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

ifeq ($(DRV_SRC_OBJ),)
	DRV_SRC := $(wildcard *.c cam_common/*.c cam_isp/*.c cam_isp/*/*.c cam_adapter/*.c cam_adapter/*/*.c cam_csiphy/*.c cam_csiphy/*/*.c cam_pattern/*.c)
	export DRV_SRC_OBJ := $(DRV_SRC:.c=.o)
endif

amlcam-objs += $(DRV_SRC_OBJ)

LOCAL_INCLUDES := -I$(M_PATH)/ -I$(M_PATH)/cam_common \
		-I$(M_PATH)/cam_isp -I$(M_PATH)/cam_isp/hw/ \
		-I$(M_PATH)/cam_adapter -I$(M_PATH)/cam_adapter/hw/ \
		-I$(M_PATH)/cam_csiphy -I$(M_PATH)/cam_csiphy/hw/ \
		-I$(M_PATH)/cam_pattern

ccflags-y += -DT7C_CHIP

ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable

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