// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2022-2023 Sumit Garg <sumit.garg@linaro.org>
 *
 * Qcom DWMAC specific glue layer
 */

#define DEBUG
#include <common.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <netdev.h>
#include <phy.h>
#include <reset.h>
#include <syscon.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#include "dwc_eth_qos.h"

static int eqos_start_clks_rockchip(struct udevice *dev)
{
#ifdef CONFIG_CLK
        struct eqos_priv *eqos = dev_get_priv(dev);
        int ret;

        debug("%s(dev=%p):\n", __func__, dev);

        ret = clk_enable(&eqos->clk_master_bus);
        if (ret < 0) {
                pr_err("clk_enable(clk_master_bus) failed: %d", ret);
                goto err;
        }

        ret = clk_enable(&eqos->clk_rx);
        if (ret < 0) {
                pr_err("clk_enable(clk_rx) failed: %d", ret);
                goto err_disable_clk_master_bus;
        }

        ret = clk_enable(&eqos->clk_tx);
        if (ret < 0) {
                pr_err("clk_enable(clk_tx) failed: %d", ret);
                goto err_disable_clk_rx;
        }

        if (clk_valid(&eqos->clk_ck) && !eqos->clk_ck_enabled) {
                ret = clk_enable(&eqos->clk_ck);
                if (ret < 0) {
                        pr_err("clk_enable(clk_ck) failed: %d", ret);
                        goto err_disable_clk_tx;
                }
                eqos->clk_ck_enabled = true;
        }
#endif

	debug("%s: OK\n", __func__);
	return 0;

#ifdef CONFIG_CLK
err_disable_clk_tx:
        clk_disable(&eqos->clk_tx);
err_disable_clk_rx:
        clk_disable(&eqos->clk_rx);
err_disable_clk_master_bus:
        clk_disable(&eqos->clk_master_bus);
err:
        debug("%s: FAILED: %d\n", __func__, ret);
        return ret;
#endif
}

static int eqos_stop_clks_rockchip(struct udevice *dev)
{
#ifdef CONFIG_CLK
        struct eqos_priv *eqos = dev_get_priv(dev);
        
        debug("%s(dev=%p):\n", __func__, dev);
        
        clk_disable(&eqos->clk_tx);
        clk_disable(&eqos->clk_rx);
        clk_disable(&eqos->clk_master_bus);
#endif

	debug("%s: OK\n", __func__);
	return 0;
}

static ulong eqos_get_tick_clk_rate_rockchip(struct udevice *dev)
{
#ifdef CONFIG_CLK
        struct eqos_priv *eqos = dev_get_priv(dev);

        return clk_get_rate(&eqos->clk_master_bus);
#else
        return 0;
#endif
}

static int eqos_probe_resources_rockchip(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	int ret;
	phy_interface_t interface;

	debug("%s(dev=%p):\n", __func__, dev);

	interface = eqos->config->interface(dev);

	if (interface == PHY_INTERFACE_MODE_NA) {
		pr_err("Invalid PHY interface\n");
		return -EINVAL;
	}
	
	ret = board_interface_eth_init(dev, interface);
        if (ret)
                return -EINVAL;

	ret = reset_get_by_name(dev, "stmmaceth", &eqos->reset_ctl);
	if (ret) {
		pr_err("reset_get_by_name(rst) failed: %d\n", ret);
		goto err_probe;
	}

	ret = clk_get_by_name(dev, "mac-clk-rx", &eqos->clk_rx);
        if (ret) {
                pr_warn("clk_get_by_name(rx) failed: %d\n", ret);
        }

        ret = clk_get_by_name(dev, "mac-clk-tx", &eqos->clk_tx);
        if (ret) {
                pr_warn("clk_get_by_name(tx) failed: %d\n", ret);
        }

	/*  Get ETH_CLK clocks (optional) */
        ret = clk_get_by_name(dev, "eth-ck", &eqos->clk_ck);
        if (ret)
                pr_warn("No phy clock provided %d\n", ret);

	debug("%s: OK\n", __func__);
	return 0;
err_probe:
        debug("%s: returns %d\n", __func__, ret);
        return ret;
}

static int eqos_remove_resources_rockchip(struct udevice *dev)
{
	struct eqos_priv * __maybe_unused eqos = dev_get_priv(dev);

        debug("%s(dev=%p):\n", __func__, dev);

#ifdef CONFIG_CLK
        clk_free(&eqos->clk_tx);
        clk_free(&eqos->clk_rx);
        clk_free(&eqos->clk_master_bus);
        if (clk_valid(&eqos->clk_ck))
                clk_free(&eqos->clk_ck);
#endif

        debug("%s: OK\n", __func__);
        return 0;
}

static struct eqos_ops eqos_rockchip_ops = {
	.eqos_inval_desc = eqos_inval_desc_generic,
	.eqos_flush_desc = eqos_flush_desc_generic,
	.eqos_inval_buffer = eqos_inval_buffer_generic,
	.eqos_flush_buffer = eqos_flush_buffer_generic,
	.eqos_probe_resources = eqos_probe_resources_rockchip,
	.eqos_remove_resources = eqos_remove_resources_rockchip,
	.eqos_stop_resets = eqos_null_ops,
	.eqos_start_resets = eqos_null_ops,
	.eqos_stop_clks = eqos_stop_clks_rockchip,
	.eqos_start_clks = eqos_start_clks_rockchip,
	.eqos_calibrate_pads = eqos_null_ops,
	.eqos_disable_calibration = eqos_null_ops,
	.eqos_set_tx_clk_speed = eqos_null_ops,
	.eqos_get_enetaddr = eqos_null_ops,
       	.eqos_get_tick_clk_rate = eqos_get_tick_clk_rate_rockchip,
};

struct eqos_config __maybe_unused eqos_rockchip_config = {
	.reg_access_always_ok = false,
	.mdio_wait = 10000,
	.swr_wait = 200,
	.config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED,
        .config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_100_150,
	.axi_bus_width = EQOS_AXI_WIDTH_64,
	.interface = dev_read_phy_mode,
	.ops = &eqos_rockchip_ops
};
