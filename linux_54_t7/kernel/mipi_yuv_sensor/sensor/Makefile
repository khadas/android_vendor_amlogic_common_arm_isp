
LOCAL_ARCH := arm64
LOCAL_KDIR := /mnt/fileroot/zhiwei.zhang/r-aosp-t7-2022-6-4/out/android12-5.4/common/
LOCAL_CROSS_COMPILE := /opt/gcc-linaro-6.3.1-2017.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-

LOCAL_PATH := $(shell pwd)

define module_func
	make ARCH=$(LOCAL_ARCH) CROSS_COMPILE=$(LOCAL_CROSS_COMPILE) \
		-C $(LOCAL_KDIR) M=$(LOCAL_PATH)/$(1) modules
endef

define module_clean
	make ARCH=$(LOCAL_ARCH) CROSS_COMPILE=$(LOCAL_CROSS_COMPILE) \
		-C $(LOCAL_KDIR) M=$(LOCAL_PATH)/$(1) clean
endef

.PHONY: clean imx290 imx290_clean

all:
	$(call module_func,imx290)
	$(call module_func,ov9285)

clean:
	$(call module_clean,imx290)
	$(call module_clean,ov9285)

imx290:
	$(call module_func,imx290)

imx290_clean:
	$(call module_clean,imx290)

install:
	@find $(LOCAL_PATH) -name "*.ko" | xargs -i cp {} $(LOCAL_PATH)
