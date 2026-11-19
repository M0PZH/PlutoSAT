#!/bin/sh
# fishball post-build script, run after the Pluto board's post-build.sh
# (BR2_ROOTFS_POST_BUILD_SCRIPT, boards/fishball/buildroot/fishball_defconfig.sed)
#   $1  the target directory
set -e
TARGET_DIR=$1
S41=$TARGET_DIR/etc/init.d/S41network

# Pluto's S41network serves its info page (also on the USB drive as
# info.html) with busybox httpd on port 80. With the station, plutosatd's
# HTTP listener on port 80 redirects to its HTTPS web UI: move httpd to 8080.
if [ -x "$TARGET_DIR/usr/bin/plutosatd" ]; then
	if ! grep -q 'httpd -h /www -p 8080$' "$S41"; then
		grep -q 'httpd -h /www$' "$S41" || { echo "fishball post-build: no 'httpd -h /www' line in $S41" >&2; exit 1; }
		sed -i 's|httpd -h /www$|httpd -h /www -p 8080|' "$S41"
	fi
fi
