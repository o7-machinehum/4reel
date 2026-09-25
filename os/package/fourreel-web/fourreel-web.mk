################################################################################
#
# fourreel-web
#
################################################################################

FOURREEL_WEB_SITE = $(BR2_EXTERNAL_FOURREEL_PATH)/package/fourreel-web
FOURREEL_WEB_SITE_METHOD = local
FOURREEL_WEB_LICENSE = MIT
FOURREEL_WEB_LICENSE_FILES = LICENSE
FOURREEL_WEB_DEPENDENCIES = rockchip-rve

define FOURREEL_WEB_BUILD_CMDS
	$(TARGET_CC) $(TARGET_CFLAGS) -std=c11 -Wall -Wextra -DFOURREEL_RVE \
		$(@D)/server.c $(@D)/http.c $(@D)/depth.c $(@D)/rve_sad.c \
		$(@D)/rve_memory.c $(TARGET_LDFLAGS) -Wl,--gc-sections \
		-Wl,--wrap=rve_drm_init,--wrap=rve_drm_deinit \
		-Wl,--wrap=RVE_DRM_MmzAlloc,--wrap=RVE_DRM_MmzFree \
		-Wl,--wrap=RVE_DRM_MmzFlush_Start,--wrap=RVE_DRM_MmzFlush_End \
		-lrve -lm -lpthread -ldl -o $(@D)/4reel-web
endef

define FOURREEL_WEB_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/4reel-web $(TARGET_DIR)/usr/bin/4reel-web
	$(INSTALL) -D -m 0644 $(@D)/index.html $(TARGET_DIR)/usr/share/4reel-web/index.html
	$(INSTALL) -D -m 0755 $(@D)/S85fourreel-web $(TARGET_DIR)/etc/init.d/S85fourreel-web
endef

$(eval $(generic-package))
