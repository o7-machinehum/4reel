################################################################################
#
# fourreel-vo
#
################################################################################

FOURREEL_VO_VERSION = e5e8ece3061c074cfa2708779ce2986866e1fa98
FOURREEL_VO_SITE = https://github.com/srv/viso2.git
FOURREEL_VO_SITE_METHOD = git
# The git archive is generated locally; the full commit ID pins its contents.
BR_NO_CHECK_HASH_FOR += $(FOURREEL_VO_SOURCE)
FOURREEL_VO_EXTRA_DOWNLOADS = \
	https://raw.githubusercontent.com/DLTcollab/sse2neon/3cf69760cc6fdc45a5d70f0d42a8f079489f9867/sse2neon.h \
	https://raw.githubusercontent.com/DLTcollab/sse2neon/3cf69760cc6fdc45a5d70f0d42a8f079489f9867/LICENSE
FOURREEL_VO_LICENSE = GPL-3.0-or-later, MIT
# Upstream has no standalone COPYING/LICENCE file; source files carry grants.
FOURREEL_VO_LICENSE_FILES = COPYING.GPL-3.0 libviso2/libviso2/src/matcher.cpp libviso2/libviso2/src/filter.cpp sse2neon.LICENSE

define FOURREEL_VO_INSTALL_SSE2NEON
	$(SED) 's/\r$$//' $(@D)/libviso2/libviso2/src/matcher.h
	$(INSTALL) -m 0644 $(FOURREEL_VO_DL_DIR)/sse2neon.h $(@D)/libviso2/libviso2/src/sse2neon.h
	$(INSTALL) -m 0644 $(FOURREEL_VO_DL_DIR)/LICENSE $(@D)/sse2neon.LICENSE
	$(INSTALL) -m 0644 $(BR2_EXTERNAL_FOURREEL_PATH)/package/fourreel-vo/COPYING.GPL-3.0 $(@D)/COPYING.GPL-3.0
endef
FOURREEL_VO_POST_EXTRACT_HOOKS += FOURREEL_VO_INSTALL_SSE2NEON

FOURREEL_VO_SRCS = \
	$(@D)/libviso2/libviso2/src/filter.cpp \
	$(@D)/libviso2/libviso2/src/matcher.cpp \
	$(@D)/libviso2/libviso2/src/matrix.cpp \
	$(@D)/libviso2/libviso2/src/viso.cpp \
	$(@D)/libviso2/libviso2/src/viso_stereo.cpp

define FOURREEL_VO_BUILD_CMDS
	$(TARGET_CXX) $(TARGET_CXXFLAGS) -std=gnu++11 \
		-I$(@D)/libviso2/libviso2/src \
		$(BR2_EXTERNAL_FOURREEL_PATH)/package/fourreel-vo/main.cpp \
		$(FOURREEL_VO_SRCS) $(TARGET_LDFLAGS) -o $(@D)/4reel-vo
endef

define FOURREEL_VO_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/4reel-vo $(TARGET_DIR)/usr/bin/4reel-vo
	$(INSTALL) -D -m 0644 $(@D)/sse2neon.LICENSE $(TARGET_DIR)/usr/share/licenses/4reel-vo/sse2neon.LICENSE
	$(INSTALL) -D -m 0644 $(@D)/COPYING.GPL-3.0 $(TARGET_DIR)/usr/share/licenses/4reel-vo/COPYING.GPL-3.0
endef

$(eval $(generic-package))
