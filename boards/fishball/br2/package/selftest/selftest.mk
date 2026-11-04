################################################################################
#
# selftest
#
################################################################################

SELFTEST_VERSION = local
SELFTEST_SITE = $(BR2_EXTERNAL_FISHBALL_PATH)/../../../awgn/sw
SELFTEST_SITE_METHOD = local
SELFTEST_LICENSE = GPL-2.0+

define SELFTEST_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) clean
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) $(TARGET_CONFIGURE_OPTS) all
endef

define SELFTEST_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef

$(eval $(generic-package))
