M_PATH     := $(shell dirname $(lastword $(MAKEFILE_LIST)))
LOCAL_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

obj-m += amlcam_auto.o

DRV_SRC := $(wildcard $(LOCAL_PATH)/*.c) \
			$(wildcard $(LOCAL_PATH)/cam_common/*.c) \
			$(wildcard $(LOCAL_PATH)/cam_adapter/*.c) \
			$(wildcard $(LOCAL_PATH)/cam_adapter/*/*.c) \
			$(wildcard $(LOCAL_PATH)/cam_csiphy/*.c) \
			$(wildcard $(LOCAL_PATH)/cam_csiphy/*/*.c)

M_DRV_SRC := $(DRV_SRC:$(LOCAL_PATH)/%=%)
DRV_SRC_OBJ := $(M_DRV_SRC:.c=.o)

ccflags-y := -I$(M_PATH)/ -I$(M_PATH)/cam_common -Wno-implicit-fallthrough

ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable
ccflags-y += -Wno-declaration-after-statement

amlcam_auto-objs := $(DRV_SRC_OBJ)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M)  modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

modules_install:
	@echo "$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install"
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib
	cp $(OUT_DIR)/$(M)/*.ko ${OUT_DIR}/../vendor_lib/modules/
