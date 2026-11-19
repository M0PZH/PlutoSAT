# Target specific constants go here

TARGET_DTS_FILES:= zynq-fishball-sdr-7020.dtb
COMPLETE_NAME:=fishball-sdr-7020
ZIP_ARCHIVE_PREFIX:=fishball
# PlutoSDR-compatible USB IDs so libiio / existing Pluto tooling work unchanged
DEVICE_VID:=0x0456
DEVICE_PID:=0xb673

BOARD_DIR:=boards/fishball
HDL_PROJECT_DIR:=$(BOARD_DIR)/hdl
HDL_PROJECT_NAME:=fishball
BOARD_BR2_EXTERNAL:=$(CURDIR)/$(BOARD_DIR)/br2
# 7020 bitstream + kernel + gzip rootfs exceed the 14 MB QSPI FIT partition
ROOTFS_EXT:=xz
# the QSPI FIT partition (qspi-linux): no .frm for a larger itb
FIT_MAX_SIZE:=0xE00000
# make sdimg: also a uEnv.txt that boots fishball.itb with the PlutoSAT bootargs
SDIMG_FILES:=$(BOARD_DIR)/buildroot/uEnv.txt

# the station's commit in /opt/VERSIONS
VSUBDIRS += station
# local (source in this repository) br2 packages: rebuilt by every image build
BOARD_BR2_LOCAL_PKGS:=dvbs2-tx nbiq selftest iqtx plutosat-station

linux/arch/arm/boot/dts/xilinx/%.dts: $(BOARD_DIR)/linux/%.dts
	cp $< $@

### PlutoSAT station (plutosatd, plutosat, web UI) ###
# Built outside buildroot (cargo, npm) into build/station, which the br2
# package plutosat-station installs (boards/fishball/br2/build-station.sh).

.PHONY: build/station
build/station: | build
	$(BOARD_DIR)/br2/build-station.sh $(CURDIR) $(CURDIR)/build/station

$(BUILDROOT_DIR)/output/images/rootfs.cpio.$(ROOTFS_EXT): build/station
