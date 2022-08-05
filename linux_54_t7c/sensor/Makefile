
define module_func
	@$(MAKE) -C $(KERNEL_SRC) M=$(M)/$(1) modules
endef

define module_clean
	@$(MAKE) -C $(KERNEL_SRC) M=$(M)/$(1) clean
endef

define module_install
	@echo "$(MAKE) INSTALL_MOD_STRIP=1 M=$(M)/$(1) -C $(KERNEL_SRC) modules_install"
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M)/$(1) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib
	cp $(OUT_DIR)/$(M)/$(1)/*.ko ${OUT_DIR}/../vendor_lib/modules/
endef

all:
	$(call module_func,imx290)
	$(call module_func,ov9285)
	$(call module_func,ov08a10)
	$(call module_func,imx415)

clean:
	$(call module_clean,imx290)
	$(call module_clean,ov9285)
	$(call module_clean,ov08a10)
	$(call module_clean,imx415)

modules_install:
	$(call module_install,imx290)
	$(call module_install,ov9285)
	$(call module_install,ov08a10)
	$(call module_install,imx415)
