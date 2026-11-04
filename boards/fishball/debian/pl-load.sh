#!/bin/sh
# Load a PL bitstream at run time through the FPGA manager, with the ADI
# drivers that sit on the PL unbound around it (their registers vanish while
# the PL is reprogrammed). A reboot restores the boot bitstream.
#
#   plutosat-pl-load [FILE]     FILE in /lib/firmware, default plutosat.bit.bin
set -e
BIT=${1:-plutosat.bit.bin}
D=/sys/bus/platform/drivers
ADC=79020000.cf-ad9361-lpc
DDS=79024000.cf-ad9361-dds-core-lpc
[ -f /lib/firmware/$BIT ] || { echo "no /lib/firmware/$BIT" >&2; exit 1; }

unb() { if [ -e $D/$1/$2 ]; then echo $2 > $D/$1/unbind; echo "unbound $2"; fi; }
bnd() { echo $2 > $D/$1/bind; echo "bound $2"; }

sync
unb cf_axi_dds $DDS
unb cf_axi_adc $ADC
unb dma-axi-dmac 7c400000.dma
unb dma-axi-dmac 7c420000.dma
unb xiic-i2c 41600000.i2c
echo 0 > /sys/class/fpga_manager/fpga0/flags
echo -n $BIT > /sys/class/fpga_manager/fpga0/firmware
echo "fpga: $(cat /sys/class/fpga_manager/fpga0/state)"
bnd xiic-i2c 41600000.i2c
bnd dma-axi-dmac 7c400000.dma
bnd dma-axi-dmac 7c420000.dma
bnd cf_axi_adc $ADC
bnd cf_axi_dds $DDS
