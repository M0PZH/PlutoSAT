#!/bin/bash
# Build the PlutoSAT Debian packages for a fishball board running Debian 12
# (bookworm, armhf) with dpkg-deb, into build/debs/:
#
#   build-debs.sh [--no-station] [--no-tools] [--no-firmware] [--no-boot | --boot]
#
#   plutosat-firmware  the bitstream (build/system_top.bit, FPGA-manager format),
#                      plutosat-pl-load and plutosat-pl-load.service (loads it at
#                      every boot), cores.txt, pl-axi.txt, the device-tree notes
#   plutosat-tools     the C tools of dvbs2/sw, nbiq/sw, awgn/sw, iqtx/sw and the
#                      fft/sw scripts, cross-built against glibc 2.34 symbols
#   plutosatd          plutosatd + plutosat (static armv7 musl), the web UI, the
#                      systemd unit, /etc/plutosat/plutosatd.toml (conffile;
#                      transmit disabled), api.md
#   plutosat-boot      OPTIONAL and risky: BOOT.bin (FSBL + bitstream + u-boot)
#                      and plutosat-boot, which puts it, the patched device tree
#                      and uEnv.txt on the SD card's boot partition. Built when
#                      the XSA of the same build is there (or --boot: required).
#
# Versions: git describe of this repository (0.39+161.gf821df4), plutosatd
# also the station's commit (+st139.g5154eb4); "+dirty" for local changes.
# Every package Provides plutosat-core-NAME (= VERSION) for the cores of the
# bitstream (regs/gen/*_regs.h); plutosatd and plutosat-tools depend on the
# same major versions (the register layout they were built for).
#
# Environment:
#   BIT=build/system_top.bit      the bitstream
#   XSA=build/system_top.xsa      plutosat-boot: the XSA of the same build (its
#                                 bitstream must match BIT), FSBL built from it
#   FSBL_WORK=/tmp/plutosat-fsbl  where the FSBL is built (its path ends up in
#                                 the FSBL's assert strings: a fixed path keeps
#                                 BOOT.bin reproducible)
#   FSBL_ELF=                     use this FSBL instead (its ps7_init tables must
#                                 equal those of the one built from XSA)
#   UBOOT_ELF=                    the fishball u-boot (default: build u-boot/ with
#                                 zynq_fishball_defconfig); must be the u-boot
#                                 commit this repository pins
#   BOOT_REF=                     a known-good BOOT.bin: the new FSBL's ps7_init
#                                 tables must be in it; partitions compared
#   VIVADO_SETTINGS=/opt/Xilinx/2026.1/Vivado/settings64.sh  (bootgen; Vitis next to it)
#   OUT=build/debs
#   MAINTAINER="Name <email>"     (default: git config user.name / user.email)
# Needs: dpkg-deb, fakeroot, bootgen, arm-linux-gnueabihf-gcc; plutosatd:
# cargo (armv7-unknown-linux-musleabihf), npm, the RADE sources
# (fetch-sources.sh); plutosat-boot: sdtgen/empyro (Vitis), arm-none-eabi-gcc,
# dtc.
set -e
TOP=$(cd "$(dirname "$0")/../../.." && pwd)
HERE=$TOP/boards/fishball/debian
OUT=${OUT:-$TOP/build/debs}
BIT=${BIT:-$TOP/build/system_top.bit}
XSA=${XSA:-$TOP/build/system_top.xsa}
FSBL_WORK=${FSBL_WORK:-/tmp/plutosat-fsbl}
VIVADO_SETTINGS=${VIVADO_SETTINGS:-/opt/Xilinx/2026.1/Vivado/settings64.sh}
CROSS=arm-linux-gnueabihf-
ARCH=armhf
STATION=1 TOOLS=1 FIRMWARE=1 BOOT=auto
for a in "$@"; do
  case $a in
    --no-station) STATION=0 ;;
    --no-tools) TOOLS=0 ;;
    --no-firmware) FIRMWARE=0 ;;
    --no-boot) BOOT=0 ;;
    --boot) BOOT=1 ;;
    *) sed -n '2,/^set -e/s/^# \{0,1\}//p' "$0"; exit 2 ;;
  esac
done
die() { echo "build-debs.sh: $*" >&2; exit 1; }
note() { echo "build-debs.sh: $*"; }
for t in dpkg-deb fakeroot python3; do command -v $t > /dev/null || die "$t not found"; done
if ! command -v bootgen > /dev/null; then
  [ -f "$VIVADO_SETTINGS" ] || die "no bootgen and no $VIVADO_SETTINGS"
  . "$VIVADO_SETTINGS" > /dev/null
fi
[ -f "$BIT" ] || die "no $BIT (make TARGET=fishball, or BIT=...)"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$OUT"

# ---------------------------------------------------------------- versions
debver() {   # git describe -> Debian version: v0.39-161-gf821df4-dirty -> 0.39+161.gf821df4+dirty
  git -C "$1" describe --tags --long --dirty --match 'v[0-9]*' 2> /dev/null \
    | sed -e 's/^v//' -e 's/-\([0-9]*\)-g\([0-9a-f]*\)/+\1.g\2/' -e 's/-dirty$/+dirty/'
}
VER=$(debver "$TOP")
[ -n "$VER" ] || die "git describe found no v* tag"
SVER=""
if [ -e "$TOP/station/.git" ]; then
  SVER="st$(git -C "$TOP/station" rev-list --count HEAD).g$(git -C "$TOP/station" rev-parse --short=7 HEAD)"
  git -C "$TOP/station" diff --quiet HEAD -- 2> /dev/null || SVER="$SVER+dirty"
fi
MAINT=${MAINTAINER:-"$(git -C "$TOP" config user.name || echo PlutoSAT) <$(git -C "$TOP" config user.email || echo root@localhost)>"}
DATE=$(date -R)
note "version $VER${SVER:+, plutosatd $VER+$SVER}"

# the cores of the bitstream (built from this tree: regs/ is the source of truth)
CORES="dvbs2_tx dvbs2_rx nbiq rx_awgn fft iqtx"
corever() {   # NAME -> 0x00020000
  sed -n "s/^#define [A-Z0-9_]*_VERSION_VALUE \(0x[0-9a-fA-F]*\)u\{0,1\}$/\1/p" "$TOP/regs/gen/$1_regs.h"
}
dotted() { v=$(($1)); echo "$((v >> 16)).$(((v >> 8) & 255)).$((v & 255))"; }
PROVIDES=""
NEEDS=""
for c in $CORES; do
  v=$(corever $c)
  [ -n "$v" ] || die "no ${c}_VERSION_VALUE in regs/gen/${c}_regs.h"
  d=$(dotted "$v")
  maj=${d%%.*}
  p=plutosat-core-$(echo $c | tr _ -)
  PROVIDES="$PROVIDES${PROVIDES:+, }$p (= $d)"
  NEEDS="$NEEDS, $p (>= $maj), $p (<< $((maj + 1)))"
done

# ---------------------------------------------------------------- helpers
# pkg_dir NAME -> $TMP/NAME (empty tree)
pkg_dir() { rm -rf "$TMP/$1"; mkdir -p "$TMP/$1/DEBIAN"; echo "$TMP/$1"; }

# docs NAME DIR LICENCE-TEXT-FILE: copyright and changelog
docs() {
  d=$2/usr/share/doc/$1
  mkdir -p "$d"
  cp "$3" "$d/copyright"
  printf '%s (%s) bookworm; urgency=medium\n\n  * Built from %s.\n\n -- %s  %s\n' \
    "$1" "$4" "$(git -C "$TOP" describe --tags --always --dirty)" "$MAINT" "$DATE" | gzip -9n > "$d/changelog.gz"
  chmod 644 "$d/copyright" "$d/changelog.gz"
}

# build_deb NAME VERSION DIR "control fields"
build_deb() {
  n=$1 v=$2 d=$3
  find "$d" -type d -exec chmod 755 {} +
  (cd "$d" && find . -path ./DEBIAN -prune -o -type f -print | sed 's|^\./||' | sort | xargs -r md5sum > DEBIAN/md5sums)
  size=$(du -sk --exclude=DEBIAN "$d" | cut -f1)
  {
    echo "Package: $n"
    echo "Version: $v"
    echo "Architecture: $ARCH"
    echo "Maintainer: $MAINT"
    echo "Installed-Size: $size"
    printf '%s\n' "$4"
  } > "$d/DEBIAN/control"
  for s in preinst postinst prerm postrm; do
    [ -f "$HERE/pkg/$n/$s" ] && install -m 755 "$HERE/pkg/$n/$s" "$d/DEBIAN/$s"
  done
  [ -f "$HERE/pkg/$n/conffiles" ] && install -m 644 "$HERE/pkg/$n/conffiles" "$d/DEBIAN/conffiles"
  chmod 644 "$d/DEBIAN/control" "$d/DEBIAN/md5sums"
  rm -f "$OUT/${n}_"*"_$ARCH.deb"
  fakeroot dpkg-deb --root-owner-group -Zxz --build "$d" "$OUT/${n}_${v}_$ARCH.deb" > /dev/null
  note "$OUT/${n}_${v}_$ARCH.deb"
}

# bitstream -> FPGA manager format
tobin() {   # in.bit out.bit.bin
  rm -rf "$TMP/bg" && mkdir "$TMP/bg"
  cp "$1" "$TMP/bg/plutosat.bit"
  printf 'all:\n{\n  plutosat.bit\n}\n' > "$TMP/bg/bit.bif"
  (cd "$TMP/bg" && bootgen -image bit.bif -arch zynq -process_bitstream bin -w > bootgen.log)
  cp "$TMP/bg/plutosat.bit.bin" "$2"
}
tobin "$BIT" "$TMP/plutosat.bit.bin"

# is XSA the build of BIT? (the same configuration data; the .bit headers differ by date)
XSA_OK=0
if [ -f "$XSA" ]; then
  unzip -p "$XSA" system_top.bit > "$TMP/xsa.bit"
  tobin "$TMP/xsa.bit" "$TMP/xsa.bit.bin"
  if cmp -s "$TMP/xsa.bit.bin" "$TMP/plutosat.bit.bin"; then XSA_OK=1; else note "$XSA is not the build of $BIT"; fi
fi

# the bitstream's AXI address map: system_bd.tcl's ad_cpu_interconnect lines,
# the ranges from the XSA's hardware description when it is the same build
plaxi() {
  python3 - "$TOP/boards/fishball/hdl/system_bd.tcl" "$1" > "$TMP/pl-axi.txt" << 'EOF'
import re, sys, zipfile
tcl = {int(a, 16): n for a, n in re.findall(r"^ad_cpu_interconnect\s+(0x[0-9A-Fa-f]+)\s+(\S+)", open(sys.argv[1]).read(), re.M)}
rows = {a: (a, a + 0xFFFF, n) for a, n in tcl.items()}
src = "system_bd.tcl (64 KiB each)"
if sys.argv[2]:
    hwh = zipfile.ZipFile(sys.argv[2]).read("system.hwh").decode()
    got = {}
    for m in re.finditer(r"<MEMRANGE [^>]*>", hwh):
        t = m.group(0)
        f = dict(re.findall(r'(\w+)="([^"]*)"', t))
        if f.get("MASTERBUSINTERFACE") in ("M_AXI_GP0", "M_AXI_GP1") and f.get("INSTANCE") != "sys_ps7":
            got[int(f["BASEVALUE"], 16)] = (int(f["BASEVALUE"], 16), int(f["HIGHVALUE"], 16), f["INSTANCE"])
    if set(got) != set(tcl):
        sys.exit(f"pl-axi: system_bd.tcl and the XSA disagree: {sorted(hex(a) for a in set(got) ^ set(tcl))}")
    rows, src = got, "the XSA's system.hwh"
print(f"# AXI slaves of the PlutoSAT PL (M_AXI_GP0/GP1), from {src}: BASE HIGH NAME")
for a in sorted(rows):
    lo, hi, n = rows[a]
    print(f"0x{lo:08X} 0x{hi:08X} {n}")
EOF
}
if [ $XSA_OK = 1 ]; then plaxi "$XSA"; else plaxi ""; fi

{
  echo "# PL cores of plutosat.bit.bin (VERSION registers, regs/gen of $(git -C "$TOP" describe --tags --always --dirty))"
  echo "# bitstream sha256 $(sha256sum < "$TMP/plutosat.bit.bin" | cut -c1-64)"
  for c in $CORES; do v=$(corever $c); echo "$c $(dotted "$v") $v"; done
} > "$TMP/cores.txt"

# ---------------------------------------------------------------- plutosat-firmware
if [ $FIRMWARE = 1 ]; then
  D=$(pkg_dir plutosat-firmware)
  install -D -m 644 "$TMP/plutosat.bit.bin" "$D/lib/firmware/plutosat.bit.bin"
  install -D -m 755 "$HERE/pl-load.sh" "$D/usr/sbin/plutosat-pl-load"
  install -D -m 644 "$HERE/plutosat-pl-load.service" "$D/lib/systemd/system/plutosat-pl-load.service"
  install -D -m 644 "$TMP/cores.txt" "$D/usr/share/plutosat-firmware/cores.txt"
  install -D -m 644 "$TMP/pl-axi.txt" "$D/usr/share/plutosat-firmware/pl-axi.txt"
  install -D -m 644 "$HERE/README.firmware.md" "$D/usr/share/doc/plutosat-firmware/README.md"
  install -D -m 644 "$HERE/dtb-add-plutosat.py" "$D/usr/share/doc/plutosat-firmware/dtb-add-plutosat.py"
  cat > "$TMP/copyright.fw" << EOF
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: PlutoSAT (fishball-sdr-7020)
Source: https://github.com/M0PZH/PlutoSAT

Files: *
Copyright: PlutoSAT contributors
License: other
 The PlutoSAT PL cores (dvbs2/, nbiq/, iqtx/, fft/, awgn/) and scripts are
 part of the PlutoSAT repository; see its LICENSE.md and README.md. The
 bitstream also contains Analog Devices HDL library cores (hdl/, ADI's
 licences) and AMD/Xilinx IP (Vivado licence terms).
EOF
  docs plutosat-firmware "$D" "$TMP/copyright.fw" "$VER"
  build_deb plutosat-firmware "$VER" "$D" "Section: misc
Priority: optional
Provides: $PROVIDES
Recommends: plutosatd
Suggests: plutosat-tools
Description: PlutoSAT PL bitstream for the fishball-sdr-7020 board
 The FPGA design of the PlutoSAT QO-100 ground station (DVB-S2 TX/RX,
 narrowband IQ channels and transmitter, spectrum FFT) for the Zynq-7020 +
 AD9363 fishball board, loaded at every boot through the FPGA manager by
 plutosat-pl-load.service. It never changes the boot partition; the
 device tree must reserve the DMA memory and list the UIO nodes (see
 README.md; postinst checks and warns)."
fi

# ---------------------------------------------------------------- plutosat-tools
if [ $TOOLS = 1 ]; then
  command -v ${CROSS}gcc > /dev/null || die "${CROSS}gcc not found"
  D=$(pkg_dir plutosat-tools)
  ${CROSS}gcc -O2 -std=c11 -Wall -c -o "$TMP/glibc-compat.o" "$HERE/glibc-compat.c"
  for d in dvbs2/sw nbiq/sw awgn/sw iqtx/sw; do
    mkdir -p "$TMP/src/$d"
    cp -a "$TOP/$d/." "$TMP/src/$d/"
    make -s -C "$TMP/src/$d" clean
    make -s -C "$TMP/src/$d" CC=${CROSS}gcc CFLAGS="-O2 -g0" LDFLAGS="$TMP/glibc-compat.o -lm" all 2> "$TMP/cc.log" \
      || { cat "$TMP/cc.log"; die "build of $d failed"; }
    make -s -C "$TMP/src/$d" DESTDIR="$D" install
  done
  install -m 755 "$TOP/fft/sw/fft_spectrum.py" "$D/usr/bin/fft-spectrum"
  install -m 755 "$TOP/fft/sw/fft_dmatest.py" "$D/usr/bin/fft-dmatest"
  for f in "$D"/usr/bin/*; do
    if file -b "$f" | grep -q ELF; then
      ${CROSS}strip --strip-unneeded "$f"
      # loadable on Debian 12 (glibc 2.36)?
      new=$(${CROSS}objdump -T "$f" | grep -oE 'GLIBC_2\.[0-9]+' | sort -t. -k2n -u | awk -F. '$2 > 36' | tr '\n' ' ')
      [ -z "$new" ] || die "$(basename "$f") needs $new (newer than Debian 12's glibc 2.36): extend glibc-compat.c"
    fi
  done
  cat > "$TMP/copyright.tools" << EOF
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: PlutoSAT (fishball-sdr-7020)
Source: https://github.com/M0PZH/PlutoSAT

Files: *
Copyright: PlutoSAT contributors
License: other
 The tools (dvbs2/sw, nbiq/sw, awgn/sw, iqtx/sw, fft/sw) are part of the
 PlutoSAT repository; see its LICENSE.md and README.md.
EOF
  docs plutosat-tools "$D" "$TMP/copyright.tools" "$VER"
  build_deb plutosat-tools "$VER" "$D" "Section: hamradio
Priority: optional
Depends: libc6 (>= 2.34), plutosat-firmware$NEEDS
Recommends: python3, python3-numpy
Description: PlutoSAT command-line tools for the PL cores
 dvbs2-tx, dvbs2-rx, nbiq, iqtx, rx-awgn, ts-prbs, dvbs2-selftest and the
 fft-spectrum / fft-dmatest board tests: drive the PlutoSAT PL cores
 directly through UIO (stop plutosatd first: systemctl stop plutosatd)."
fi

# ---------------------------------------------------------------- plutosatd
if [ $STATION = 1 ]; then
  S=$TOP/station
  [ -d "$S/rust" ] || die "no station/ checkout"
  export PLUTOSAT_RADE_REQUIRED=${PLUTOSAT_RADE_REQUIRED:-1}
  if [ -f "$S/rust/crates/plutosat-rade/fetch-sources.sh" ]; then
    sh "$S/rust/crates/plutosat-rade/fetch-sources.sh" || [ "$PLUTOSAT_RADE_REQUIRED" = 0 ] \
      || die "RADE sources could not be fetched (PLUTOSAT_RADE_REQUIRED=0 builds without RADE)"
  fi
  (cd "$S/rust" && PLUTOSAT_REGS=$TOP/regs PLUTOSAT_FW=$TOP \
    cargo build --release --target armv7-unknown-linux-musleabihf -p plutosatd -p plutosat-cli)
  (cd "$S/web" && npm ci --silent && npm run build --silent)
  B=$S/rust/target/armv7-unknown-linux-musleabihf/release
  D=$(pkg_dir plutosatd)
  install -D -m 755 "$B/plutosatd" "$D/usr/bin/plutosatd"
  install -D -m 755 "$B/plutosat" "$D/usr/bin/plutosat"
  mkdir -p "$D/usr/share/plutosat/web"
  cp -r "$S/web/dist/." "$D/usr/share/plutosat/web/"
  find "$D/usr/share/plutosat/web" -type f -exec chmod 644 {} +
  install -D -m 644 "$HERE/plutosatd.service" "$D/lib/systemd/system/plutosatd.service"
  install -D -m 600 "$HERE/plutosatd.toml" "$D/etc/plutosat/plutosatd.toml"
  install -D -m 644 "$HERE/plutosatd.toml" "$D/usr/share/plutosatd/plutosatd.toml.default"
  install -D -m 644 "$S/docs/api.md" "$D/usr/share/doc/plutosatd/api.md"
  grep -q '^enabled = false' "$D/etc/plutosat/plutosatd.toml" || die "plutosatd.toml: transmit not disabled"
  cat > "$TMP/copyright.station" << EOF
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: PlutoSAT station (plutosatd, plutosat, web UI)
Source: https://github.com/M0PZH/PlutoSAT-station

Files: *
Copyright: PlutoSAT contributors
License: Expat
 Permission is hereby granted, free of charge, to any person obtaining a
 copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to permit
 persons to whom the Software is furnished to do so, subject to the
 following conditions:
 .
 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.
 .
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 USE OR OTHER DEALINGS IN THE SOFTWARE.
 .
 The binaries link Rust crates and, with RADE, rade_c (BSD-2-Clause) and
 opus (BSD-3-Clause) sources (station/rust/crates/plutosat-rade/NOTICE);
 the web UI bundles HamUI and npm packages under their own licences.
EOF
  [ -f "$S/rust/crates/plutosat-rade/NOTICE" ] && install -D -m 644 "$S/rust/crates/plutosat-rade/NOTICE" "$D/usr/share/doc/plutosatd/NOTICE.rade"
  docs plutosatd "$D" "$TMP/copyright.station" "$VER${SVER:++$SVER}"
  build_deb plutosatd "$VER${SVER:++$SVER}" "$D" "Section: hamradio
Priority: optional
Depends: plutosat-firmware$NEEDS
Suggests: plutosat-tools
Description: PlutoSAT QO-100 ground station daemon and web UI
 plutosatd drives the PlutoSAT PL cores of the fishball-sdr-7020 board:
 two DVB-S2 receive slots, DATV transmit from the browser, narrowband
 receive and transmit (SSB, CW, FT8 / FT4, RADE), spectrum and waterfall,
 the QO-100 beacon lock and a background recorder, with a web UI on
 https://BOARD/ (self-signed certificate). The CLI plutosat talks to it.
 Transmitting is disabled in the installed configuration
 (/etc/plutosat/plutosatd.toml, [tx] and [nbtx] enabled = false)."
fi

# ---------------------------------------------------------------- plutosat-boot (optional)
if [ $BOOT = auto ]; then
  if [ $XSA_OK = 1 ]; then BOOT=1; else BOOT=0; note "plutosat-boot skipped: no XSA of this bitstream ($XSA)"; fi
fi
if [ $BOOT = 1 ]; then
  [ $XSA_OK = 1 ] || die "plutosat-boot needs the XSA of the same build as $BIT (XSA=...)"
  command -v dtc > /dev/null || die "dtc not found"
  # 1. the PS setup of the XSA is the known-good one
  python3 "$HERE/ps7-writes.py" "$XSA" > "$TMP/ps7.txt"
  diff -u "$HERE/ps7_init.ref" "$TMP/ps7.txt" > "$TMP/ps7.diff" \
    || { cat "$TMP/ps7.diff"; die "the XSA's ps7_init differs from ps7_init.ref (the known-good PS setup): check it, then update ps7_init.ref"; }
  note "ps7_init: the same $(wc -l < "$TMP/ps7.txt") PS register operations as ps7_init.ref"
  # 2. the FSBL, from that XSA
  VITIS_SETTINGS=$(dirname "$VIVADO_SETTINGS")/../Vitis/settings64.sh
  mkdir -p "$(dirname "$FSBL_WORK")"
  (
    flock -w 3600 9 || exit 1
    bash -c ". '$VIVADO_SETTINGS' > /dev/null && . '$VITIS_SETTINGS' > /dev/null && \
      '$TOP/scripts/create_fsbl.sh' '$XSA' '$FSBL_WORK' '$TMP/fsbl-xsa.elf'" > "$TMP/fsbl.log" 2>&1 \
      || { tail -20 "$TMP/fsbl.log"; exit 1; }
  ) 9> "$FSBL_WORK.lock" || die "FSBL build failed"
  FSBL=$TMP/fsbl-xsa.elf
  if [ -n "$FSBL_ELF" ]; then cp "$FSBL_ELF" "$TMP/fsbl-given.elf"; FSBL=$TMP/fsbl-given.elf; fi
  # 3. u-boot
  UB_COMMIT=$(git -C "$TOP" ls-tree HEAD u-boot | awk '{print $3}' | cut -c1-12)
  if [ -z "$UBOOT_ELF" ]; then
    [ -f "$TOP/u-boot/Makefile" ] || die "no u-boot/ checkout: git submodule update --init u-boot && boards/fishball/install.sh, or UBOOT_ELF=..."
    sh "$TOP/boards/fishball/install.sh" "$TOP" > /dev/null
    SOURCE_DATE_EPOCH=$(git -C "$TOP/u-boot" log -1 --format=%ct) \
      make -s -C "$TOP/u-boot" ARCH=arm CROSS_COMPILE=$CROSS zynq_fishball_defconfig all > "$TMP/uboot.log" 2>&1 \
      || { tail -20 "$TMP/uboot.log"; die "u-boot build failed"; }
    UBOOT_ELF=$TOP/u-boot/u-boot
  fi
  strings "$UBOOT_ELF" | grep -q "^U-Boot .*-g$UB_COMMIT" || die "$UBOOT_ELF is not u-boot $UB_COMMIT (the commit this repository pins)"
  strings "$UBOOT_ELF" | grep -q "^fit_image=fishball.itb" || die "$UBOOT_ELF is not built with zynq_fishball_defconfig"
  # 4. BOOT.bin
  cp "$BIT" "$TMP/system_top.bit"
  cp "$FSBL" "$TMP/fsbl.elf"
  cp "$UBOOT_ELF" "$TMP/u-boot.elf"
  printf 'the_ROM_image:\n{\n  [bootloader] fsbl.elf\n  system_top.bit\n  u-boot.elf\n}\n' > "$TMP/boot.bif"
  (cd "$TMP" && bootgen -arch zynq -image boot.bif -w -o BOOT.bin > bootgen-boot.log)
  # 5. the FSBL's ps7_init tables: those of the FSBL built from the XSA, and
  #    (BOOT_REF) in the known-good BOOT.bin
  python3 - "$TMP/fsbl-xsa.elf" "$TMP/BOOT.bin" "${BOOT_REF:-}" << 'EOF' || die "ps7_init tables check failed"
import struct, subprocess, sys
elf, boot, ref = sys.argv[1:4]
def parts(path):
    """The partitions of a Zynq BOOT.bin: FSBL, bitstream, u-boot."""
    d = open(path, "rb").read()
    ph, = struct.unpack_from("<I", d, 0x9C)
    out = []
    for i in range(8):
        w = struct.unpack_from("<16I", d, ph + 64 * i)
        if w[0] == 0:
            break
        out.append(d[w[5] * 4: w[5] * 4 + w[0] * 4])
    return out
def symbol_bytes(path, names):
    """{symbol: its bytes} through the ELF's PT_LOAD segments."""
    e = open(path, "rb").read()
    phoff, = struct.unpack_from("<I", e, 0x1C)
    phentsize, phnum = struct.unpack_from("<HH", e, 0x2A)
    segs = []
    for i in range(phnum):
        t, off, va, pa, fsz, msz, fl, al = struct.unpack_from("<8I", e, phoff + i * phentsize)
        if t == 1:
            segs.append((va, off, fsz))
    out = {}
    for n, (a, size) in names.items():
        for va, off, fsz in segs:
            if va <= a and a + size <= va + fsz:
                out[n] = e[off + a - va: off + a - va + size]
    return out
syms = subprocess.run(["arm-none-eabi-nm", "-S", elf], capture_output=True, text=True, check=True).stdout
names = {}
for l in syms.splitlines():
    f = l.split()
    if len(f) == 4 and f[3].startswith("ps7_") and f[3][-4:-1] in ("_3_", "_2_", "_1_"):
        names[f[3]] = (int(f[0], 16), int(f[1], 16))
tables = symbol_bytes(elf, names)
if len(tables) < 15:
    sys.exit(f"only {len(tables)} ps7 tables found in {elf}")
for name, path in (("BOOT.bin", boot), ("BOOT_REF", ref)):
    if not path:
        continue
    fsbl = parts(path)[0]
    miss = [t for t, b in tables.items() if b not in fsbl]
    if miss:
        sys.exit(f"{name} {path}: its FSBL lacks the ps7_init tables of the FSBL built from the XSA: {miss}")
    print(f"build-debs.sh: {name}: FSBL has all {len(tables)} ps7_init tables of the FSBL built from the XSA")
if ref:
    a, b = parts(boot), parts(ref)
    for i, n in enumerate(["FSBL", "bitstream", "u-boot"]):
        same = i < len(a) and i < len(b) and a[i] == b[i]
        print(f"build-debs.sh: BOOT_REF: {n} partition {'identical' if same else 'DIFFERENT'}")
    print(f"build-debs.sh: BOOT_REF: BOOT.bin {'identical' if open(boot, 'rb').read() == open(ref, 'rb').read() else 'DIFFERENT'}")
EOF
  D=$(pkg_dir plutosat-boot)
  install -D -m 644 "$TMP/BOOT.bin" "$D/usr/share/plutosat-boot/BOOT.bin"
  install -D -m 644 "$TMP/pl-axi.txt" "$D/usr/share/plutosat-boot/pl-axi.txt"
  install -D -m 644 "$HERE/dtb-add-plutosat.py" "$D/usr/share/plutosat-boot/dtb-add-plutosat.py"
  grep '^uenvcmd=' "$HERE/README.md" > "$TMP/uenvcmd.txt"
  [ "$(wc -l < "$TMP/uenvcmd.txt")" = 1 ] || die "README.md: no single uenvcmd= line"
  install -D -m 644 "$TMP/uenvcmd.txt" "$D/usr/share/plutosat-boot/uenvcmd.txt"
  echo "$VER" > "$TMP/version"
  install -D -m 644 "$TMP/version" "$D/usr/share/plutosat-boot/version"
  {
    echo "BOOT.bin $(sha256sum < "$TMP/BOOT.bin" | cut -c1-64)"
    echo "fsbl.elf $(sha256sum < "$TMP/fsbl.elf" | cut -c1-64) (from $(basename "$XSA") of the bitstream build${FSBL_ELF:+; given: $FSBL_ELF})"
    echo "u-boot.elf $(sha256sum < "$TMP/u-boot.elf" | cut -c1-64) (u-boot $UB_COMMIT, zynq_fishball_defconfig)"
    echo "plutosat.bit.bin $(sha256sum < "$TMP/plutosat.bit.bin" | cut -c1-64)"
  } > "$TMP/manifest"
  install -D -m 644 "$TMP/manifest" "$D/usr/share/plutosat-boot/MANIFEST"
  install -D -m 755 "$HERE/plutosat-boot.py" "$D/usr/sbin/plutosat-boot"
  install -D -m 755 "$HERE/pkg/plutosat-boot/plutosat-boot-rollback" "$D/usr/sbin/plutosat-boot-rollback"
  install -D -m 644 "$HERE/README.boot.md" "$D/usr/share/doc/plutosat-boot/README.md"
  cat > "$TMP/copyright.boot" << EOF
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: PlutoSAT (fishball-sdr-7020)
Source: https://github.com/M0PZH/PlutoSAT

Files: *
Copyright: PlutoSAT contributors
License: other
 The scripts and the bitstream: see the PlutoSAT repository (LICENSE.md,
 README.md) and the plutosat-firmware package. BOOT.bin also holds the
 Zynq FSBL (AMD/Xilinx embeddedsw, MIT-style licence) and Das U-Boot
 (GPL-2.0+, source: the u-boot submodule of the PlutoSAT repository at
 commit $UB_COMMIT, with boards/fishball/u-boot).
EOF
  docs plutosat-boot "$D" "$TMP/copyright.boot" "$VER"
  build_deb plutosat-boot "$VER" "$D" "Section: admin
Priority: optional
Depends: python3, device-tree-compiler, plutosat-firmware (= $VER)
Description: PlutoSAT boot files for the fishball board (optional, risky)
 Writes BOOT.bin (FSBL + the PlutoSAT bitstream + the fishball u-boot), the
 board's device tree patched for PlutoSAT, and the Debian uenvcmd in
 uEnv.txt to the SD card's boot partition, after a preflight (board,
 partition, device tree against the bitstream's address map, the QSPI
 u-boot environment), with backups and plutosat-boot-rollback. A bad boot
 file needs someone at the SD card: nothing depends on this package
 (plutosat-firmware loads the bitstream at every boot anyway)."
fi
ls -la "$OUT"
