################################################################################
#
# iqtx
#
################################################################################

IQTX_VERSION = local
IQTX_SITE = $(BR2_EXTERNAL_FISHBALL_PATH)/../../../iqtx/sw
IQTX_SITE_METHOD = local
IQTX_LICENSE = GPL-2.0+

define IQTX_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) clean
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) $(TARGET_CONFIGURE_OPTS) iqtx
endef

define IQTX_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef

$(eval $(generic-package))
