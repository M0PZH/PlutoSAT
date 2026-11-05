#!/bin/sh
# Deploy the PlutoSAT PL design and tools to a fishball board that runs its
# own Debian (kernel, u-boot and BOOT.bin stay as they are).
#
#   deploy.sh [--boot] [--station] root@HOST
#
# Always:   build/system_top.bit -> /lib/firmware/plutosat.bit.bin (the one
#           before kept as plutosat.bit.bin.prev),
#           plutosat-pl-load (run-time FPGA load), and the tools built on the
#           board from dvbs2/sw, nbiq/sw, awgn/sw into /usr/local/bin.
# --boot:   once per board (and again when dtb-add-plutosat.py changes):
#           devicetree.dtb + reserved memory and the UIO nodes, - the IIO
#           streaming DMAs (dtb-add-plutosat.py), uEnv.txt + uio_pdrv_genirq.of_id;
#           the originals stay on the card as *.pre-plutosat. Reboot after.
# --station: plutosatd and plutosat (cross-built here for armv7 musl), the
#           web UI (npm run build here) into /usr/share/plutosat/web, the
#           systemd service plutosatd (enabled, restarted), and
#           /etc/plutosat/plutosatd.toml if there is none yet (from
#           plutosatd.toml, with a random admin password, printed once).
#
# Then on the board: plutosat-pl-load (after every boot, until the
# bitstream goes into BOOT.bin), then e.g. dvbs2-selftest.
# Needs here: Vivado (bootgen), dtc, ssh/scp access to the board; with
# --station also cargo (target armv7-unknown-linux-musleabihf) and npm.
set -e
TOP=$(cd "$(dirname "$0")/../../.." && pwd)
HERE=$TOP/boards/fishball/debian
BOOT=0
STATION=0
while [ $# -gt 1 ]; do
  case $1 in
    --boot) BOOT=1 ;;
    --station) STATION=1 ;;
    *) break ;;
  esac
  shift
done
[ -n "$1" ] || { sed -n '2,24s/^# \{0,1\}//p' "$0"; exit 2; }
H=$1
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
ssh_() { ssh -o BatchMode=yes "$H" "$@" < /dev/null; }

# bitstream -> FPGA manager format
[ -f "$TOP/build/system_top.bit" ] || { echo "no build/system_top.bit (make TARGET=fishball)"; exit 1; }
cp "$TOP/build/system_top.bit" "$TMP/plutosat.bit"
printf 'all:\n{\n  plutosat.bit\n}\n' > "$TMP/bit.bif"
(cd "$TMP" && bootgen -image bit.bif -arch zynq -process_bitstream bin -w > bootgen.log)
ssh_ '[ ! -e /lib/firmware/plutosat.bit.bin ] || cp -p /lib/firmware/plutosat.bit.bin /lib/firmware/plutosat.bit.bin.prev'
scp -q "$TMP/plutosat.bit.bin" "$H:/lib/firmware/plutosat.bit.bin"
scp -q "$HERE/pl-load.sh" "$H:/usr/local/sbin/plutosat-pl-load"
ssh_ chmod 755 /usr/local/sbin/plutosat-pl-load

# tools, built on the board (its glibc)
tar czf "$TMP/src.tgz" -C "$TOP" dvbs2/sw nbiq/sw awgn/sw
scp -q "$TMP/src.tgz" "$H:/tmp/plutosat-src.tgz"
ssh_ 'set -e; command -v gcc >/dev/null || apt-get install -y gcc make;
  rm -rf /tmp/plutosat-src && mkdir /tmp/plutosat-src && tar xzf /tmp/plutosat-src.tgz -C /tmp/plutosat-src 2>/dev/null;
  for d in dvbs2/sw nbiq/sw awgn/sw; do
    make -s -C /tmp/plutosat-src/$d clean; make -s -C /tmp/plutosat-src/$d CFLAGS=-O2 all;
    make -s -C /tmp/plutosat-src/$d DESTDIR=/tmp/plutosat-inst install;
  done;
  install -m 755 /tmp/plutosat-inst/usr/bin/* /usr/local/bin/;
  rm -rf /tmp/plutosat-src /tmp/plutosat-inst /tmp/plutosat-src.tgz;
  echo "tools: $(ls /usr/local/bin | tr "\n" " ")"'

if [ $BOOT = 1 ]; then
  ssh_ 'mkdir -p /mnt/plutosat-boot && mount /dev/mmcblk0p1 /mnt/plutosat-boot'
  scp -q "$H:/mnt/plutosat-boot/devicetree.dtb" "$TMP/orig.dtb"
  scp -q "$H:/mnt/plutosat-boot/uEnv.txt" "$TMP/uEnv.orig"
  if python3 "$HERE/dtb-add-plutosat.py" "$TMP/orig.dtb" "$TMP/new.dtb"; then
    scp -q "$TMP/new.dtb" "$H:/tmp/plutosat.dtb"
    ssh_ 'cd /mnt/plutosat-boot && { [ -e devicetree.dtb.pre-plutosat ] || cp -p devicetree.dtb devicetree.dtb.pre-plutosat; } && cp /tmp/plutosat.dtb devicetree.dtb'
  fi
  if grep -q "rootwait rw &&" "$TMP/uEnv.orig" && ! grep -q "uio_pdrv_genirq" "$TMP/uEnv.orig"; then
    sed 's/rootwait rw &&/rootwait rw uio_pdrv_genirq.of_id=generic-uio \&\&/' "$TMP/uEnv.orig" > "$TMP/uEnv.new"
    scp -q "$TMP/uEnv.new" "$H:/tmp/plutosat-uEnv.txt"
    ssh_ 'cd /mnt/plutosat-boot && { [ -e uEnv.txt.pre-plutosat ] || cp -p uEnv.txt uEnv.txt.pre-plutosat; } && cp /tmp/plutosat-uEnv.txt uEnv.txt'
  else
    echo "uEnv.txt: bootargs not changed (already patched, or no 'rootwait rw &&' to extend)"
  fi
  ssh_ 'sync; umount /mnt/plutosat-boot; rm -f /tmp/plutosat.dtb /tmp/plutosat-uEnv.txt'
  echo "boot files updated: reboot the board, then run plutosat-pl-load"
fi

if [ $STATION = 1 ]; then
  S=$TOP/station
  (cd "$S/rust" && PLUTOSAT_REGS=$TOP/regs PLUTOSAT_FW=$TOP \
    cargo build --release --target armv7-unknown-linux-musleabihf -p plutosatd -p plutosat-cli)
  (cd "$S/web" && npm ci --silent && npm run build --silent)
  BIN=$S/rust/target/armv7-unknown-linux-musleabihf/release
  scp -q "$BIN/plutosatd" "$H:/usr/local/bin/plutosatd.new"
  scp -q "$BIN/plutosat" "$H:/usr/local/bin/plutosat.new"
  tar czf "$TMP/web.tgz" -C "$S/web/dist" .
  scp -q "$TMP/web.tgz" "$H:/tmp/plutosat-web.tgz"
  scp -q "$S/docs/api.md" "$HERE/plutosatd.service" "$HERE/plutosatd.toml" "$H:/tmp/"
  ssh_ 'set -e
    systemctl stop plutosatd 2>/dev/null || true
    mv /usr/local/bin/plutosatd.new /usr/local/bin/plutosatd
    mv /usr/local/bin/plutosat.new /usr/local/bin/plutosat
    chmod 755 /usr/local/bin/plutosatd /usr/local/bin/plutosat
    rm -rf /usr/share/plutosat/web.new && mkdir -p /usr/share/plutosat/web.new
    tar xzf /tmp/plutosat-web.tgz -C /usr/share/plutosat/web.new
    rm -rf /usr/share/plutosat/web && mv /usr/share/plutosat/web.new /usr/share/plutosat/web
    install -m 644 /tmp/api.md /usr/share/plutosat/api.md
    install -m 644 /tmp/plutosatd.service /etc/systemd/system/plutosatd.service
    if [ ! -e /etc/plutosat/plutosatd.toml ]; then
      mkdir -p /etc/plutosat
      PW=$(head -c 12 /dev/urandom | od -An -tx1 | tr -d " \n")
      sed "s/ADMIN_PASSWORD/$PW/" /tmp/plutosatd.toml > /etc/plutosat/plutosatd.toml
      chmod 600 /etc/plutosat/plutosatd.toml
      echo "created /etc/plutosat/plutosatd.toml, admin password: $PW"
    fi
    rm -f /tmp/plutosat-web.tgz /tmp/api.md /tmp/plutosatd.service /tmp/plutosatd.toml
    systemctl daemon-reload
    systemctl enable plutosatd
    systemctl restart plutosatd
    sleep 2; systemctl --no-pager --lines=5 status plutosatd || true'
fi
