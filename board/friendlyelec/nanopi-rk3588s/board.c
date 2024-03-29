/*
 * SPDX-License-Identifier:     GPL-2.0+
 *
 * Copyright (c) 2022 FriendlyElec Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
 *
 * (C) Copyright 2021 Rockchip Electronics Co., Ltd
 */

#include <common.h>
#include <adc.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <i2c.h>
#include <misc.h>
#include <net.h>
#include <asm/setup.h>
#include <usb.h>
#include <dwc3-uboot.h>

#include "hwrev.h"

DECLARE_GLOBAL_DATA_PTR;

int board_select_fdt_index(ulong dt_table_hdr, struct blk_desc *dev_desc)
{
	return (dev_desc ? dev_desc->devnum : 0);
}

static int board_check_supply(void)
{
	u32 adc_reading = 0;
	int mv = 5000;

	adc_channel_single_shot("saradc@fec10000", 2, &adc_reading);
	debug("%s: ADC reading %d\n", __func__, adc_reading);

	mv = adc_reading * 2475 / 512;

	printf("vdd_usbc %d mV\n", mv);

	return 0;
}

static int mac_read_from_generic_eeprom(u8 *addr)
{
	struct udevice *i2c_dev;
	int ret;

	/* Microchip 24AA02xxx EEPROMs with EUI-48 Node Identity */
	ret = i2c_get_chip_for_busnum(6, 0x53, 1, &i2c_dev);
	if (!ret)
		ret = dm_i2c_read(i2c_dev, 0xfa, addr, 6);
	else
		debug("%s: no EEPROMs found %d\n", __func__, ret);

	return ret;
}

static void __maybe_unused setup_macaddr(void)
{
	u8 mac_addr[6] = { 0 };
	int lockdown = 0;
	int ret;

#ifndef CONFIG_ENV_IS_NOWHERE
	lockdown = env_get_yesno("lockdown") == 1;
#endif
	if (lockdown && env_get("ethaddr"))
		return;

	ret = mac_read_from_generic_eeprom(mac_addr);
	if (!ret && is_valid_ethaddr(mac_addr)) {
		debug("MAC: %pM\n", mac_addr);
		eth_env_set_enetaddr("ethaddr", mac_addr);
	}
}

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	setup_macaddr();

	return 0;
}
#endif

#if defined(CONFIG_DISPLAY_BOARDINFO) || defined(CONFIG_DISPLAY_BOARDINFO_LATE)
int show_board_info(void)
{
	printf("Board: %s\n", get_board_name());

	return 0;
}
#endif

#ifdef CONFIG_REVISION_TAG
static void set_board_rev(void)
{
	char info[64] = {0, };

	snprintf(info, ARRAY_SIZE(info), "%02x", get_board_rev());
	env_set("board_rev", info);
}
#endif

const char* rk3588_nano="rk3588%s-nano%s.dtb";

void set_dtb_name(void)
{
	char info[64] = {0, };
#ifndef CONFIG_ENV_IS_NOWHERE
	if (env_get_yesno("lockdown") == 1 &&
		env_get("dtb_name"))
		return;
#endif
	switch (get_board_rev()) {
                case 0x01:
		       snprintf(info, ARRAY_SIZE(info), rk3588_nano, "",  "PC-T6");
			break;
                case 0x04:
                case 0x05:
		       snprintf(info, ARRAY_SIZE(info), rk3588_nano, "s", "pi-r6c");
			break;
                case 0x02:
                case 0x03:
		default:
		       snprintf(info, ARRAY_SIZE(info), rk3588_nano, "s", "pi-r6s");
			break;
        }
	env_set("dtb_name", info);
	printf("dtb_name: %s\n", info);
}

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	char *serial_string;
	u64 serial = 0;

	serial_string = env_get("serial#");

	if (serial_string)
		serial = simple_strtoull(serial_string, NULL, 16);

	serialnr->high = (u32)(serial >> 32);
	serialnr->low = (u32)(serial & 0xffffffff);
}
#endif

#ifdef CONFIG_BOARD_LATE_INIT
int rk_board_late_init(void)
{
	board_check_supply();

#ifdef CONFIG_REVISION_TAG
	set_board_rev();
#endif
	set_dtb_name();

#ifdef CONFIG_SILENT_CONSOLE
	gd->flags &= ~GD_FLG_SILENT;
#endif

	printf("\n");

	return 0;
}
#endif


#ifdef CONFIG_OF_BOARD_SETUP
int nanopi6_add_reserved_memory_fdt_nodes(void *new_blob)
{
	struct fdt_memory gap1 = {
		.start = 0x3fc000000,
		.end = 0x3fc4fffff,
	};
	struct fdt_memory gap2 = {
		.start = 0x3fff00000,
		.end = 0x3ffffffff,
	};
	unsigned long flags = FDTDEC_RESERVED_MEMORY_NO_MAP;
	unsigned int ret;

	/*
	 * Inject the reserved-memory nodes into the DTS
	 */
	ret = fdtdec_add_reserved_memory(new_blob, "gap1", &gap1,  NULL, 0,
					 NULL, flags);
	if (ret)
		return ret;

	return fdtdec_add_reserved_memory(new_blob, "gap2", &gap2,  NULL, 0,
					  NULL, flags);
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	return nanopi6_add_reserved_memory_fdt_nodes(blob);
}
#endif
