/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for the fishball-sdr-7020 board (Zynq-7020 + AD9363).
 * Derived from include/configs/zynq_pluto_sdr.h (C) 2015-2026 Analog Devices Inc.
 *
 * Differences from Pluto:
 *  - 16 MB QSPI: the FIT image partition is 14 MB (0x200000..0x1000000)
 *  - LED on MIO0, DFU button on MIO10
 *  - single FIT configuration (no adi_hwref revision probe)
 *  - "sdboot": loads fishball.itb (same FIT as QSPI) from the SD card FAT partition
 *  - modeboot follows the boot-mode straps: SD -> sdboot, otherwise qspiboot
 */

#ifndef __CONFIG_ZYNQ_FISHBALL_SDR_H
#define __CONFIG_ZYNQ_FISHBALL_SDR_H

#define FISHBALL_FIT_MAX	"0xE00000"

#define DFU_ALT_INFO_RAM \
	"dfu_ram_info=" \
	"setenv dfu_alt_info " \
	"dummy.dfu ram 0 0\\\;" \
	"firmware.dfu ram ${fit_load_address} " FISHBALL_FIT_MAX "\0" \
	"dfu_ram=echo Entering DFU RAM mode ... && run dfu_ram_info && dfu 0 ram 0\0"

#define DFU_ALT_INFO_SF \
	"dfu_sf_info=" \
	"setenv dfu_alt_info " \
	"boot.dfu raw 0x0 0x100000\\\;" \
	"firmware.dfu raw 0x200000 " FISHBALL_FIT_MAX "\\\;" \
	"uboot-extra-env.dfu raw 0xFF000 0x1000\\\;" \
	"uboot-env.dfu raw 0x100000 0x20000\\\;" \
	"spare.dfu raw 0x120000 0xE0000\0" \
	"dfu_sf=gpio set 0;setenv silent;echo Entering DFU SF mode ... && run dfu_sf_info && dfu 0 sf 0:0:40000000:0 && if test -n ${dfu_alt_num} && test ${dfu_alt_num} = 1; " \
	"then setenv fit_size ${filesize} && setenv dfu_alt_num && env save; fi;gpio clear 0;\0"

#define FISHBALL_ENV_VERSION "1"

#define FISHBALL_BOOTARGS \
	"setenv bootargs console=ttyPS0,115200 maxcpus=${maxcpus} rootfstype=ramfs root=/dev/ram0 rw earlyprintk clk_ignore_unused uio_pdrv_genirq.of_id=generic-uio uboot=\"${ver}\" ${extra_bootargs}"

#define CFG_EXTRA_ENV_SETTINGS \
	"env_version=" FISHBALL_ENV_VERSION "\0" \
	"modeboot=qspiboot\0" \
	"ethaddr=00:0a:35:00:01:23\0" \
	"ipaddr=192.168.2.1\0" \
	"ipaddr_host=192.168.2.10\0" \
	"netmask=255.255.255.0\0" \
	"fit_image=fishball.itb\0" \
	"fit_load_address=0x2080000\0" \
	"fit_config=config@0\0" \
	"extraenv_load_address=0x207E000\0" \
	"fit_size=" FISHBALL_FIT_MAX "\0" \
	"fdt_high=0x20000000\0" \
	"initrd_high=0x20000000\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv_addr=0x2000000\0" \
	"maxcpus=2\0" \
	"clear_reset_cause=mw f8000008 df0d && mw f8000258 00400000 && mw f8000004 767b\0" \
	"mode=2r2t\0" \
	"fishball_loadvals=fdt addr ${fit_load_address} && fdt get value fdt_choosen /configurations/${fit_config}/ fdt && " \
		"fdt get addr fdtaddr /images/${fdt_choosen} data && fdt addr ${fdtaddr}; " \
		"if test -n \"${ad936x_ext_refclk_override}\"; then " \
			"fdt set /clocks/clock@0 clock-frequency ${ad936x_ext_refclk_override}; " \
		"fi; " \
		"if test -n \"${attr_name}\" && test -n \"${attr_val}\"; then " \
			"fdt set /amba/spi@e0006000/ad9361-phy@0 ${attr_name} ${attr_val} || fdt set /axi/spi@e0006000/ad9361-phy@0 ${attr_name} ${attr_val}; " \
		"fi; " \
		"if test \"${mode}\" = \"1r1t\"; then " \
			"fdt rm /amba/spi@e0006000/ad9361-phy@0 adi,2rx-2tx-mode-enable || fdt rm /axi/spi@e0006000/ad9361-phy@0 adi,2rx-2tx-mode-enable; " \
			"fdt set /fpga-axi/cf-ad9361-dds-core-lpc@79024000 compatible adi,axi-ad9364-dds-6.00.a; " \
		"fi; \0" \
	"qspiboot_extraenv=sf read ${extraenv_load_address} 0xFF000 0x1000 && " \
		"env import -c ${extraenv_load_address} 0x1000 || true \0" \
	"read_sf=sf probe && run qspiboot_extraenv && " \
		"sf read ${fit_load_address} 0x200000 ${fit_size} && " \
		"iminfo ${fit_load_address} || " \
		"sf read ${fit_load_address} 0x200000 " FISHBALL_FIT_MAX "; \0" \
	"loadbootenv=load mmc 0 ${loadbootenv_addr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from SD ...; env import -t ${loadbootenv_addr} ${filesize}\0" \
	"uenvboot=" \
		"if run loadbootenv; then " \
			"run importbootenv; " \
		"fi; " \
		"if test -n $uenvcmd; then " \
			"echo Running uenvcmd ...; " \
			"run uenvcmd; " \
		"fi\0" \
	"sdboot=echo Booting from SD card ...; mmc rescan && run uenvboot; " \
		"load mmc 0 ${fit_load_address} ${fit_image} && run fishball_loadvals; " \
		"" FISHBALL_BOOTARGS " && " \
		"bootm ${fit_load_address}#${fit_config} || echo SD boot failed, trying QSPI ... && run qspiboot_verbose\0" \
	"ramboot_verbose=echo Copying Linux from DFU to RAM... && " \
		"run dfu_ram;" \
		"run fishball_loadvals; " \
		"" FISHBALL_BOOTARGS " && " \
		"bootm ${fit_load_address}#${fit_config}\0" \
	"qspiboot_verbose=echo Copying Linux from QSPI flash to RAM... && " \
		"run read_sf && run fishball_loadvals; " \
		"" FISHBALL_BOOTARGS " && " \
		"bootm ${fit_load_address}#${fit_config} || echo BOOT failed entering DFU mode ... && run dfu_sf \0" \
	"qspiboot=gpio input 10 && setenv silent && sf probe && sf protect lock 0 100000 && run dfu_sf; " \
		"setenv silent;" \
		"itest *f8000258 == 480003 && run clear_reset_cause && run dfu_sf; " \
		"itest *f8000258 == 480007 && run clear_reset_cause && run ramboot_verbose; " \
		"itest *f8000258 == 480006 && run clear_reset_cause && run qspiboot_verbose; " \
		"itest *f8000258 == 480002 && run clear_reset_cause && exit; " \
		"run qspiboot_verbose\0" \
	"jtagboot=env default -a;sf probe && sf protect unlock 0 100000 && run dfu_sf; \0" \
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_SF

#include <configs/zynq-common.h>

#endif /* __CONFIG_ZYNQ_FISHBALL_SDR_H */
