#!/bin/sh
# Build the PlutoSAT station for the buildroot image and stage it for the
# br2 package plutosat-station (package/plutosat-station):
#
#   build-station.sh [TOP [OUT]]      (default: this repository, TOP/build/station)
#
# OUT/usr/bin/plutosatd, OUT/usr/bin/plutosat   static armv7 musl (rust-lld), with RADE
# OUT/usr/share/plutosat/web/                    the web UI (station/web, npm run build)
# OUT/usr/share/doc/plutosatd/                   api.md, NOTICE.rade
# OUT/VERSION                                    the station commit
#
# The same recipe as boards/fishball/debian/build-debs.sh (plutosatd
# package). The top-level Makefile runs it before buildroot (make
# TARGET=fishball ...; scripts/fishball.mk).
#
# Environment:
#   RADE_DL=DIR                 download cache for the RADE sources
#                               (fetch-sources.sh DIR; pinned SHA-256)
#   PLUTOSAT_RADE_REQUIRED=1    0: build plutosatd without RADE when the
#                               sources cannot be fetched
#   NODE_DIR=                   directory with node / npm (default: PATH, else
#                               ~/.local/share/nvm/v22.11.0/bin)
# Needs: cargo with the armv7-unknown-linux-musleabihf and wasm32 targets,
# arm-linux-gnueabihf-gcc (ring), node + npm.
set -e
TOP=$(cd "${1:-$(dirname "$0")/../../..}" && pwd)
OUT=${2:-$TOP/build/station}
S=$TOP/station
die() { echo "build-station.sh: $*" >&2; exit 1; }
[ -f "$S/rust/Cargo.toml" ] || die "no station/ checkout in $TOP (git submodule update --init station)"

if [ -n "$NODE_DIR" ]; then
  PATH=$NODE_DIR:$PATH
elif ! command -v npm > /dev/null && [ -x "$HOME/.local/share/nvm/v22.11.0/bin/npm" ]; then
  PATH=$HOME/.local/share/nvm/v22.11.0/bin:$PATH
fi
export PATH
for t in cargo npm; do command -v $t > /dev/null || die "$t not found"; done

export PLUTOSAT_RADE_REQUIRED=${PLUTOSAT_RADE_REQUIRED:-1}
if [ -f "$S/rust/crates/plutosat-rade/fetch-sources.sh" ]; then
  sh "$S/rust/crates/plutosat-rade/fetch-sources.sh" ${RADE_DL:+"$RADE_DL"} || [ "$PLUTOSAT_RADE_REQUIRED" = 0 ] \
    || die "RADE sources could not be fetched (PLUTOSAT_RADE_REQUIRED=0 builds without RADE)"
fi
export PLUTOSAT_REGS=$TOP/regs PLUTOSAT_FW=$TOP
(cd "$S/rust" && cargo build --release --target armv7-unknown-linux-musleabihf -p plutosatd -p plutosat-cli)
(cd "$S/web" && npm ci --no-audit --no-fund --prefer-offline && npm run build)

B=$S/rust/target/armv7-unknown-linux-musleabihf/release
rm -rf "${OUT:?}.tmp"
install -D -m 755 "$B/plutosatd" "$OUT.tmp/usr/bin/plutosatd"
install -D -m 755 "$B/plutosat" "$OUT.tmp/usr/bin/plutosat"
mkdir -p "$OUT.tmp/usr/share/plutosat/web"
cp -r "$S/web/dist/." "$OUT.tmp/usr/share/plutosat/web/"
find "$OUT.tmp/usr/share/plutosat/web" -type f -exec chmod 644 {} +
install -D -m 644 "$S/docs/api.md" "$OUT.tmp/usr/share/doc/plutosatd/api.md"
[ -f "$S/rust/crates/plutosat-rade/NOTICE" ] \
  && install -D -m 644 "$S/rust/crates/plutosat-rade/NOTICE" "$OUT.tmp/usr/share/doc/plutosatd/NOTICE.rade"
SV=$(git -C "$S" describe --tags --always --dirty 2> /dev/null || echo unknown)
echo "station $SV" > "$OUT.tmp/VERSION"
rm -rf "${OUT:?}"
mv "$OUT.tmp" "$OUT"
echo "build-station.sh: $OUT (station $SV)"
