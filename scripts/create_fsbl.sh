#!/bin/bash
# Build the Zynq-7000 FSBL from an .xsa without xsct (Vitis 2023.2+ SDT flow):
#   sdtgen  -> system device tree
#   empyro  -> standalone BSP + zynq_fsbl application, built with CMake/Ninja
#
# Needs the bare-metal ARM toolchain (arm-none-eabi-gcc) on PATH, either from
# the Vitis installer (gnu/aarch32) or e.g. Ubuntu's gcc-arm-none-eabi.
#
# usage: create_fsbl.sh <system_top.xsa> <work dir> <output fsbl.elf>
set -euo pipefail

XSA=$(readlink -f "$1")
WS=$(readlink -m "$2")
OUT=$(readlink -m "$3")

for t in sdtgen empyro arm-none-eabi-gcc; do
	command -v "$t" >/dev/null || { echo "create_fsbl.sh: '$t' not found in PATH" >&2; exit 1; }
done
if [ -z "${XILINX_VITIS:-}" ]; then
	echo "create_fsbl.sh: source Vitis settings64.sh first" >&2
	exit 1
fi

rm -rf "$WS"
mkdir -p "$WS"
cd "$WS"   # empyro drops its .repo.yaml cache in the current directory
sdtgen "$XILINX_VITIS/vitis-server/scripts/platformutil.tcl" "$XSA" "$WS/sdt"
empyro create_bsp -s "$WS/sdt/system-top.dts" -p ps7_cortexa9_0 -o standalone -t zynq_fsbl -w "$WS/bsp"
empyro create_app -t zynq_fsbl -d "$WS/bsp" -w "$WS/fsbl"
empyro build_app -w "$WS/fsbl"

ELF=$(find "$WS/fsbl" -name '*.elf' -path '*build*' | head -1)
[ -n "$ELF" ] || { echo "create_fsbl.sh: no FSBL elf produced" >&2; exit 1; }
mkdir -p "$(dirname "$OUT")"
cp "$ELF" "$OUT"
echo "create_fsbl.sh: $ELF -> $OUT"
