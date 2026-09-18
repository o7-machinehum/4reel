BUILDROOT := $(CURDIR)/buildroot
BR2_EXTERNAL := $(CURDIR)
RV1103G_OUTPUT := $(CURDIR)/output/rv1103g
RV1103G_ARGS := O=$(RV1103G_OUTPUT) BR2_EXTERNAL=$(BR2_EXTERNAL)

.PHONY: rv1103g rv1103g-menuconfig rv1103g-clean

rv1103g: $(RV1103G_OUTPUT)/.config
	+$(MAKE) -C $(BUILDROOT) $(RV1103G_ARGS)

$(RV1103G_OUTPUT)/.config: $(CURDIR)/configs/rv1103g_defconfig
	+$(MAKE) -C $(BUILDROOT) $(RV1103G_ARGS) rv1103g_defconfig

rv1103g-menuconfig: $(RV1103G_OUTPUT)/.config
	+$(MAKE) -C $(BUILDROOT) $(RV1103G_ARGS) menuconfig

rv1103g-clean:
	+$(MAKE) -C $(BUILDROOT) $(RV1103G_ARGS) clean
