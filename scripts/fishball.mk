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

linux/arch/arm/boot/dts/xilinx/%.dts: $(BOARD_DIR)/linux/%.dts
	cp $< $@
