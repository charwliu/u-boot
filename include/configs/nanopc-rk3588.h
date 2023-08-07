/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2022 Collabora Ltd.
 */

#ifndef __NANOPC_RK3588_H
#define __NANOPC_RK3588_H

#define ROCKCHIP_DEVICE_SETTINGS \
                "stdout=serial,vidconsole\0" \
                "stderr=serial,vidconsole\0"

#define CONFIG_REVISION_TAG

#include <configs/rk3588_common.h>

#endif /* __NANOPC_RK3588_H */
