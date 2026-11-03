################################################################################
#
# dvbs2-tx
#
################################################################################

DVBS2_TX_VERSION = local
DVBS2_TX_SITE = $(BR2_EXTERNAL_FISHBALL_PATH)/../../../dvbs2/sw
DVBS2_TX_SITE_METHOD = local
DVBS2_TX_LICENSE = GPL-2.0+

define DVBS2_TX_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) $(TARGET_CONFIGURE_OPTS) dvbs2-tx dvbs2-rx
endef

define DVBS2_TX_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef

$(eval $(generic-package))
