################################################################################
#
# plutosat-station
#
################################################################################

# the staged build of boards/fishball/br2/build-station.sh (cargo, npm)
PLUTOSAT_STATION_VERSION = local
PLUTOSAT_STATION_SITE = $(BR2_EXTERNAL_FISHBALL_PATH)/../../../build/station
PLUTOSAT_STATION_SITE_METHOD = local
PLUTOSAT_STATION_LICENSE = MIT
PLUTOSAT_STATION_LICENSE_FILES = usr/share/doc/plutosatd/NOTICE.rade

PLUTOSAT_STATION_DEBIAN_TOML = $(BR2_EXTERNAL_FISHBALL_PATH)/../debian/plutosatd.toml

# The default configuration: the Debian package's (boards/fishball/debian/
# plutosatd.toml: HTTPS on 443, port 80 redirects, transmit disabled) with
# this image's header and a [recorder] section. S95plutosatd fills in the
# admin password and the recorder placeholders at the first start.
define PLUTOSAT_STATION_BUILD_CMDS
	test -x $(@D)/usr/bin/plutosatd || { echo "plutosat-station: no build/station: run boards/fishball/br2/build-station.sh"; exit 1; }
	grep -q '^admin_password = "ADMIN_PASSWORD"' $(PLUTOSAT_STATION_DEBIAN_TOML)
	test "$$(grep -c '^enabled = false' $(PLUTOSAT_STATION_DEBIAN_TOML))" -ge 2
	! grep -q '^\[recorder\]' $(PLUTOSAT_STATION_DEBIAN_TOML)
	{ cat $(PLUTOSAT_STATION_PKGDIR)/plutosatd-head.toml; \
	  sed '1,/^$$/d' $(PLUTOSAT_STATION_DEBIAN_TOML); \
	  cat $(PLUTOSAT_STATION_PKGDIR)/plutosatd-tail.toml; } > $(@D)/plutosatd.toml.default
endef

define PLUTOSAT_STATION_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/usr/bin/plutosatd $(TARGET_DIR)/usr/bin/plutosatd
	$(INSTALL) -D -m 0755 $(@D)/usr/bin/plutosat $(TARGET_DIR)/usr/bin/plutosat
	rm -rf $(TARGET_DIR)/usr/share/plutosat/web
	mkdir -p $(TARGET_DIR)/usr/share/plutosat
	cp -r $(@D)/usr/share/plutosat/web $(TARGET_DIR)/usr/share/plutosat/web
	$(INSTALL) -D -m 0644 $(@D)/VERSION $(TARGET_DIR)/usr/share/plutosatd/VERSION
	# api.md, NOTICE.rade (buildroot removes /usr/share/doc from the target)
	cp $(@D)/usr/share/doc/plutosatd/* $(TARGET_DIR)/usr/share/plutosatd/
	$(INSTALL) -D -m 0644 $(@D)/plutosatd.toml.default $(TARGET_DIR)/usr/share/plutosatd/plutosatd.toml.default
	$(INSTALL) -D -m 0755 $(PLUTOSAT_STATION_PKGDIR)/plutosatd-run $(TARGET_DIR)/usr/sbin/plutosatd-run
endef

define PLUTOSAT_STATION_INSTALL_INIT_SYSV
	$(INSTALL) -D -m 0755 $(PLUTOSAT_STATION_PKGDIR)/S95plutosatd $(TARGET_DIR)/etc/init.d/S95plutosatd
endef

$(eval $(generic-package))
