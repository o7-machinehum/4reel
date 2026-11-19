# Heavy export/conversion is done on the host, never in the 64 MiB image.
FOURREEL_STEREO_SITE = $(BR2_EXTERNAL_FOURREEL_PATH)/package/fourreel-web
FOURREEL_STEREO_SITE_METHOD = local
FOURREEL_STEREO_LICENSE = MIT, Apache-2.0, LGPL-2.1+, RKNN-SDK
FOURREEL_STEREO_LICENSE_FILES = LICENSE
# Private development image only; do not distribute vendor blobs implicitly.
FOURREEL_STEREO_REDISTRIBUTE = NO
FOURREEL_STEREO_BUNDLE = $(BR2_EXTERNAL_FOURREEL_PATH)/../output/zipdepth-bundle

define FOURREEL_STEREO_BUILD_CMDS
	cd $(FOURREEL_STEREO_BUNDLE) && sha256sum -c $(BR2_EXTERNAL_FOURREEL_PATH)/package/fourreel-stereo/bundle.sha256
	$(TARGET_CC) $(TARGET_CFLAGS) -std=c11 -Wall -Wextra -Werror \
		$(@D)/stereo_main.c $(@D)/stereo_capture.c $(@D)/stereo_sparse.c \
		$(@D)/stereo_metric.c $(@D)/stereo_trace.c $(@D)/stereo_preview.c \
		$(TARGET_LDFLAGS) -pthread -lm -o $(@D)/4reel-stereo
	sh $(BR2_EXTERNAL_FOURREEL_PATH)/tools/zipdepth/build_runner.sh \
		"$(TARGET_CC)" "$(STAGING_DIR)/usr/include" "$(FOURREEL_STEREO_BUNDLE)" \
		"$(@D)/zipdepth_runner.c" "$(@D)/zipdepth-runner"
endef

define FOURREEL_STEREO_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/4reel-stereo $(TARGET_DIR)/usr/bin/4reel-stereo
	$(INSTALL) -D -m 0755 $(@D)/zipdepth-runner $(TARGET_DIR)/opt/4reel-rknn/bin/zipdepth-runner
	$(INSTALL) -d $(TARGET_DIR)/opt/4reel-rknn/lib $(TARGET_DIR)/usr/share/4reel-stereo/licenses
	for lib in libc.so.6 libm.so.6 libpthread.so.0 ld-linux-armhf.so.3; do \
		$(INSTALL) -m 0755 $(FOURREEL_STEREO_BUNDLE)/glibc/lib/$$lib $(TARGET_DIR)/opt/4reel-rknn/lib/; \
	done
	$(INSTALL) -m 0755 $(FOURREEL_STEREO_BUNDLE)/lib/*.so $(TARGET_DIR)/opt/4reel-rknn/lib/
	$(INSTALL) -m 0644 $(FOURREEL_STEREO_BUNDLE)/zipdepth-mono-rv1103.rknn $(TARGET_DIR)/usr/share/4reel-stereo/
	$(INSTALL) -m 0644 $(FOURREEL_STEREO_BUNDLE)/manifest.json $(TARGET_DIR)/usr/share/4reel-stereo/
	$(INSTALL) -m 0644 $(FOURREEL_STEREO_BUNDLE)/licenses/* $(TARGET_DIR)/usr/share/4reel-stereo/licenses/
	$(INSTALL) -m 0644 $(@D)/stereo_preview.html $(TARGET_DIR)/usr/share/4reel-stereo/preview.html
	$(INSTALL) -m 0644 $(BR2_EXTERNAL_FOURREEL_PATH)/board/rv1103g/STEREO_ZIPDEPTH.md $(TARGET_DIR)/usr/share/4reel-stereo/
endef

$(eval $(generic-package))
