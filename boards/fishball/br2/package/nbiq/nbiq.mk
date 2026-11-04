################################################################################
#
# nbiq
#
################################################################################

NBIQ_VERSION = local
NBIQ_SITE = $(BR2_EXTERNAL_FISHBALL_PATH)/../../../nbiq/sw
NBIQ_SITE_METHOD = local
NBIQ_LICENSE = GPL-2.0+

define NBIQ_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) clean
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) $(TARGET_CONFIGURE_OPTS) nbiq
endef

define NBIQ_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef

$(eval $(generic-package))
