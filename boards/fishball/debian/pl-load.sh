#!/bin/sh
# Load a PL bitstream at run time through the FPGA manager, with the ADI
# drivers that sit on the PL unbound around it (their registers vanish while
# the PL is reprogrammed). A reboot restores the boot bitstream.
#
#   plutosat-pl-load [--iio-dma] [FILE]   FILE in /lib/firmware, default plutosat.bit.bin
#
# The PlutoSAT PL has no IIO streaming DMAs (axi_ad9361_adc_dma at
# 0x7C400000, axi_ad9361_dac_dma at 0x7C420000): the device tree must not
# have them either (deploy.sh --boot), else binding their driver to the empty
# address range would hang the bus, and the ADC core driver would wait for
# its "rx" DMA forever. --iio-dma: FILE is an older bitstream that still has
# them, and the device tree still lists them: unbind and bind them as well.
set -e
IIO_DMA=0
[ "$1" = --iio-dma ] && { IIO_DMA=1; shift; }
BIT=${1:-plutosat.bit.bin}
D=/sys/bus/platform/drivers
DT=/proc/device-tree/fpga-axi@0
ADC=79020000.cf-ad9361-lpc
DDS=79024000.cf-ad9361-dds-core-lpc
DMAS="7c400000.dma 7c420000.dma"
[ -f /lib/firmware/$BIT ] || { echo "no /lib/firmware/$BIT" >&2; exit 1; }
if [ $IIO_DMA = 0 ] && { [ -e $DT/dma@7c400000 ] || [ -e $DT/dma@7c420000 ] || [ -e $DT/cf-ad9361-lpc@79020000/dmas ]; }; then
  echo "the device tree still has the IIO DMAs (dma@7c400000/dma@7c420000, dmas): the PlutoSAT PL has none." >&2
  echo "Update it (deploy.sh --boot) and reboot, or use --iio-dma for a bitstream that has them." >&2
  exit 1
fi
# the DVB-S2 receiver's LLR queue (LLRQ_CTRL, off by default) writes 2 MB at
# 0x3E800000 through S_AXI_HP1: that memory must be reserved
if [ ! -e /proc/device-tree/reserved-memory/dvbs2-llr-buffer@3e800000 ]; then
  echo "warning: no dvbs2-llr-buffer@3e800000 in the device tree (deploy.sh --boot): keep the RX LLR queue off" >&2
fi

unb() { if [ -e $D/$1/$2 ]; then echo $2 > $D/$1/unbind; echo "unbound $2"; fi; }
bnd() { echo $2 > $D/$1/bind; echo "bound $2"; }

# plutosatd must not touch the cores while the PL is reprogrammed (its
# register accesses would fault): stop the service, start it again after
SVC=0
if command -v systemctl > /dev/null && systemctl -q is-active plutosatd 2> /dev/null; then
  systemctl stop plutosatd && SVC=1 && echo "stopped plutosatd"
fi
restart_svc() { if [ $SVC = 1 ]; then systemctl start plutosatd && echo "started plutosatd"; fi; }
trap restart_svc EXIT

sync
unb cf_axi_dds $DDS
unb cf_axi_adc $ADC
if [ $IIO_DMA = 1 ]; then for d in $DMAS; do unb dma-axi-dmac $d; done; fi
unb xiic-i2c 41600000.i2c
echo 0 > /sys/class/fpga_manager/fpga0/flags
echo -n $BIT > /sys/class/fpga_manager/fpga0/firmware
echo "fpga: $(cat /sys/class/fpga_manager/fpga0/state)"
bnd xiic-i2c 41600000.i2c
if [ $IIO_DMA = 1 ]; then for d in $DMAS; do bnd dma-axi-dmac $d; done; fi
bnd cf_axi_adc $ADC
bnd cf_axi_dds $DDS
# the core drivers probe without DMAs (no IIO buffer); a failed probe leaves
# the AD9363 digital interface untuned and the cores in reset
for d in cf_axi_adc/$ADC cf_axi_dds/$DDS; do
  [ -e $D/$d ] || { echo "${d#*/}: driver not bound, see dmesg" >&2; exit 1; }
done
