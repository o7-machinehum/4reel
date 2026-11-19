################################################################################
#
# rockchip-rknn
#
################################################################################

ROCKCHIP_RKNN_VERSION = 2c723177a94fac65cbc036d722907ddbc91980cf
ROCKCHIP_RKNN_SITE = https://raw.githubusercontent.com/airockchip/rknn-toolkit2/$(ROCKCHIP_RKNN_VERSION)/rknpu2/runtime/Linux/librknn_api/armhf-uclibc
ROCKCHIP_RKNN_SOURCE = librknnmrt.a
ROCKCHIP_RKNN_EXTRA_DOWNLOADS = \
	https://raw.githubusercontent.com/airockchip/rknn-toolkit2/$(ROCKCHIP_RKNN_VERSION)/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h \
	https://raw.githubusercontent.com/airockchip/rknn-toolkit2/$(ROCKCHIP_RKNN_VERSION)/LICENSE
ROCKCHIP_RKNN_LICENSE = Proprietary
ROCKCHIP_RKNN_LICENSE_FILES = LICENSE
ROCKCHIP_RKNN_REDISTRIBUTE = NO
ROCKCHIP_RKNN_INSTALL_STAGING = YES
ROCKCHIP_RKNN_INSTALL_TARGET = NO
ROCKCHIP_RKNN_DEPENDENCIES = libdrm

define ROCKCHIP_RKNN_EXTRACT_CMDS
	$(INSTALL) -m 0644 $(ROCKCHIP_RKNN_DL_DIR)/librknnmrt.a $(@D)/librknnmrt.a
	$(INSTALL) -m 0644 $(ROCKCHIP_RKNN_DL_DIR)/rknn_api.h $(@D)/rknn_api.h
	$(INSTALL) -m 0644 $(ROCKCHIP_RKNN_DL_DIR)/LICENSE $(@D)/LICENSE
endef

define ROCKCHIP_RKNN_BUILD_CMDS
	$(TARGET_OBJCOPY) --redefine-sym mmap=rknn_uclibc_mmap \
		--redefine-sym lseek=rknn_uclibc_lseek \
		--redefine-sym ftruncate=rknn_uclibc_ftruncate \
		$(@D)/librknnmrt.a $(@D)/librknnmrt-musl.a
	$(TARGET_CC) $(TARGET_CFLAGS) -std=c11 -Wall -Wextra -c \
		$(BR2_EXTERNAL_FOURREEL_PATH)/package/rockchip-rknn/musl_compat.c \
		-o $(@D)/musl_compat.o
	$(TARGET_AR) rcs $(@D)/librknncompat.a $(@D)/musl_compat.o
endef

define ROCKCHIP_RKNN_INSTALL_STAGING_CMDS
	$(INSTALL) -D -m 0644 $(@D)/librknnmrt-musl.a $(STAGING_DIR)/usr/lib/librknnmrt.a
	$(INSTALL) -D -m 0644 $(@D)/librknncompat.a $(STAGING_DIR)/usr/lib/librknncompat.a
	$(INSTALL) -D -m 0644 $(@D)/rknn_api.h $(STAGING_DIR)/usr/include/rknn_api.h
endef

$(eval $(generic-package))
