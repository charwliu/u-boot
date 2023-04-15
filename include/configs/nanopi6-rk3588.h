/*
 * SPDX-License-Identifier:     GPL-2.0+
 *
 * Copyright (C) Guangzhou FriendlyELEC Computer Tech. Co., Ltd.
 * (http://www.friendlyelec.com)
 *
 * Copyright (c) 2021 Rockchip Electronics Co., Ltd
 */

#ifndef __CONFIG_NANOPI6_H__
#define __CONFIG_NANOPI6_H__


#define CONFIG_SERIAL_TAG

#define ROCKCHIP_DEVICE_SETTINGS \
	"stdout=serial,vidconsole\0" \
	"stderr=serial,vidconsole\0"

#include <configs/rk3588_common.h>

#endif /* __CONFIG_NANOPI6_H__ */
