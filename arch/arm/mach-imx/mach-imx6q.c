/*
 * Copyright 2011 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/memblock.h>
#include <linux/micrel_phy.h>
#include <asm/smp_twd.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/system_misc.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/viv_gpu.h>

#include <linux/android_pmem.h>

struct platform_device imx6q_pmem_device = {
	.name = "android_pmem",
	.id = 0,
};

static struct android_pmem_platform_data imx6q_pmem_data = {
	.name = "pmem_gpu",
	.size = SZ_32M,
};

static struct viv_gpu_platform_data gpu_pdata = {
	.reserved_mem_size = SZ_128M,
};

static const struct of_dev_auxdata imx6q_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("viv,galcore", 0x00130000, "galcore", &gpu_pdata),
};

void imx6q_restart(char mode, const char *cmd)
{
	struct device_node *np;
	void __iomem *wdog_base;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-wdt");
	wdog_base = of_iomap(np, 0);
	if (!wdog_base)
		goto soft;

	imx_src_prepare_restart();

	/* enable wdog */
	writew_relaxed(1 << 2, wdog_base);
	/* write twice to ensure the request will not get ignored */
	writew_relaxed(1 << 2, wdog_base);

	/* wait for reset to assert ... */
	mdelay(500);

	pr_err("Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

soft:
	/* we'll take a jump through zero as a poor second */
	soft_restart(0);
}

/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);

	return 0;
}

static void __init imx6q_init_machine(void)
{
	if (of_machine_is_compatible("fsl,imx6q-sabrelite"))
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
					   ksz9021rn_phy_fixup);

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6q_auxdata_lookup, NULL);

#ifdef CONFIG_ANDROID_PMEM
	imx6q_pmem_device.dev.platform_data = &imx6q_pmem_data;
	platform_device_register(&imx6q_pmem_device);
#endif

	imx6q_pm_init();

}

static void __init imx6q_map_io(void)
{
	imx_lluart_map_io();
	imx_scu_map_io();
	imx6q_clock_map_io();
}

static int __init imx6q_gpio_add_irq_domain(struct device_node *np,
				struct device_node *interrupt_parent)
{
	static int gpio_irq_base = MXC_GPIO_IRQ_START + ARCH_NR_GPIOS;

	gpio_irq_base -= 32;
	irq_domain_add_legacy(np, 32, gpio_irq_base, 0, &irq_domain_simple_ops,
			      NULL);

	return 0;
}

static const struct of_device_id imx6q_irq_match[] __initconst = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init, },
	{ .compatible = "fsl,imx6q-gpio", .data = imx6q_gpio_add_irq_domain, },
	{ /* sentinel */ }
};

static void __init imx6q_init_irq(void)
{
	l2x0_of_init(0, ~0UL);
	imx_src_init();
	imx_gpc_init();
	of_irq_init(imx6q_irq_match);
}

static void __init imx6q_timer_init(void)
{
	mx6q_clocks_init();
	twd_local_timer_of_register();
}

static struct sys_timer imx6q_timer = {
	.init = imx6q_timer_init,
};

static void __init imx6q_reserve(void)
{
	phys_addr_t phys;

	if (gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_free(phys, gpu_pdata.reserved_mem_size);
		memblock_remove(phys, gpu_pdata.reserved_mem_size);
		gpu_pdata.reserved_mem_base = phys;
	}
#ifdef CONFIG_ANDROID_PMEM
	if (imx6q_pmem_data.size) {
		phys = memblock_alloc(imx6q_pmem_data.size, SZ_4K);
		memblock_free(phys, imx6q_pmem_data.size);
		memblock_remove(phys, imx6q_pmem_data.size);
		imx6q_pmem_data.start = phys;
	}
#endif
};

static const char *imx6q_dt_compat[] __initdata = {
	"fsl,imx6q-arm2",
	"fsl,imx6q-sabrelite",
	"fsl,imx6q",
	NULL,
};

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad (Device Tree)")
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.handle_irq	= imx6q_handle_irq,
	.timer		= &imx6q_timer,
	.reserve 	= imx6q_reserve,
	.init_machine	= imx6q_init_machine,
	.dt_compat	= imx6q_dt_compat,
	.restart	= imx6q_restart,
	.reserve 	= imx6q_reserve,
MACHINE_END
