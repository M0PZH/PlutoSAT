#!/bin/sh
# Install the fishball-sdr-7020 board files into the linux, u-boot and
# br2-external submodules. Only adds untracked files; tracked files in the
# submodules are never modified. Safe to run repeatedly.
#
# usage: boards/fishball/install.sh <top-level repo dir>
set -e

TOP=$(cd "${1:-$(dirname "$0")/../..}" && pwd)
B=$TOP/boards/fishball

install_if_changed() { # src dst
	if ! cmp -s "$1" "$2"; then
		mkdir -p "$(dirname "$2")"
		cp "$1" "$2"
		echo "  installed $2"
	fi
}

gen_if_changed() { # dst (content on stdin)
	tmp=$(mktemp)
	cat > "$tmp"
	if ! cmp -s "$tmp" "$1"; then
		mv "$tmp" "$1"
		echo "  generated $1"
	else
		rm -f "$tmp"
	fi
}

echo "Installing fishball-sdr-7020 board files"

# Linux
install_if_changed "$B/linux/zynq-fishball-sdr-7020.dts" \
	"$TOP/linux/arch/arm/boot/dts/xilinx/zynq-fishball-sdr-7020.dts"
cat "$TOP/linux/arch/arm/configs/zynq_pluto_defconfig" "$B/linux/fishball.config" | \
	gen_if_changed "$TOP/linux/arch/arm/configs/zynq_fishball_defconfig"

# u-boot
install_if_changed "$B/u-boot/zynq-fishball-sdr-7020.dts" \
	"$TOP/u-boot/arch/arm/dts/zynq-fishball-sdr-7020.dts"
install_if_changed "$B/u-boot/zynq_fishball_sdr.h" \
	"$TOP/u-boot/include/configs/zynq_fishball_sdr.h"
cat "$TOP/u-boot/configs/zynq_pluto_defconfig" "$B/u-boot/fishball.config" | \
	gen_if_changed "$TOP/u-boot/configs/zynq_fishball_defconfig"

# Buildroot (br2-external): Pluto's board directory plus fishball overrides
BR=$TOP/br2-external
mkdir -p "$BR/board/adi/fishball"
rsync -a --exclude device_config --exclude motd --exclude VERSIONS --exclude msd/LICENSE.html --exclude msd/index.html \
	"$BR/board/adi/pluto/" "$BR/board/adi/fishball/"
for f in device_config motd; do
	install_if_changed "$B/buildroot/$f" "$BR/board/adi/fishball/$f"
done
sed -f "$B/buildroot/fishball_defconfig.sed" "$BR/configs/zynq_pluto_defconfig" | \
	gen_if_changed "$BR/configs/zynq_fishball_defconfig"
