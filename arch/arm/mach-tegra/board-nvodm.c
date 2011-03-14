/*
 * arch/arm/mach-tegra/board-nvodm.c
 *
 * Converts data from ODM query library into platform data
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/machine.h>
#include <linux/lbee9qmb-rfkill.h>
#include <linux/gpio.h>
#include <linux/console.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kobject.h>

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
#include <linux/i2c/panjit_ts.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#include <linux/i2c/atmel_maxtouch.h>
#endif

#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/pinmux.h>
#include <mach/usb-hcd.h>
#include <mach/usb-otg.h>
#include <mach/serial.h>
#include <mach/sdhci.h>
#include <mach/nand.h>
#include <mach/regulator.h>
#include <mach/kbc.h>
#include <mach/i2c.h>
#include <mach/spi.h>
#include <mach/w1.h>

#include <mach/nvrm_linux.h>

#include "nvrm_gpio.h"
#include "nvodm_query.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvrm_pinmux.h"
#include "nvrm_module.h"
#include "nvodm_kbc.h"
#include "nvodm_query_kbc.h"
#include "nvodm_kbc_keymapping.h"
#include "gpio-names.h"
#include "power.h"
#include "board.h"
#include "nvrm_pmu.h"

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
#include <linux/proc_fs.h>      /* Necessary because we use the proc fs */
#include <asm/uaccess.h>        /* for copy_from_user */
#endif

#ifdef CONFIG_TEGRA_TMON_PROC
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#endif

# define BT_RESET 0
# define BT_SHUTDOWN 1

#if defined(CONFIG_KEYBOARD_GPIO)
#include "nvodm_query_gpio.h"
#include <linux/gpio_keys.h>
#include <linux/input.h>
#endif

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
#include <linux/earlysuspend.h>
#include "odm_kit/adaptations/pmu/tps6586x/nvodm_pmu_tps6586x_supply_info_table.h"
NvOdmServicesPmuHandle s_hPmuServices;

#include <linux/usb.h>
#include "../../../drivers/usb/core/hcd.h"
#include "../../../drivers/usb/core/usb.h"

enum udev_pm_level{udev_level_on , udev_level_auto ,udev_level_suspend};

static enum udev_pm_level s_camera_level=udev_level_on;
static enum udev_pm_level s_camera_usb_level=udev_level_auto;
static int s_camera_usb_autosuspend=2;
#endif

NvBool IsBoardTango(void) {
  return NV_FALSE;
}
NvRmGpioHandle s_hGpioGlobal;

struct debug_port_data {
	NvOdmDebugConsole port;
	const struct tegra_pingroup_config *pinmux;
	struct clk *clk_data;
	int nr_pins;
};

static u64 tegra_dma_mask = DMA_BIT_MASK(32);

static struct debug_port_data uart_debug_port = {
			.port = NvOdmDebugConsole_None,
};

extern const struct tegra_pingroup_config *tegra_pinmux_get(const char *dev_id,
	int config, int *len);


static struct plat_serial8250_port debug_uart_platform[] = {
	{
		/* Force the debug console UART port type to PORT_TEGRA.*/
		.flags = UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type = PORT_TEGRA,
		.iotype = UPIO_MEM,
		.regshift = 2,
	}, {
		.flags = 0,
	}
};
static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform,
	},
};

static void __init tegra_setup_debug_uart(void)
{
	NvOdmDebugConsole uart = NvOdmQueryDebugConsole();
	const struct tegra_pingroup_config *pinmux = NULL;
	const NvU32 *odm_table;
	struct clk *c = NULL;
	NvU32 odm_nr;
	int nr_pins;

	if (uart < NvOdmDebugConsole_UartA ||
	    uart > NvOdmDebugConsole_UartE)
		return;

	NvOdmQueryPinMux(NvOdmIoModule_Uart, &odm_table, &odm_nr);
	if (odm_nr <= (uart - NvOdmDebugConsole_UartA)) {
		pr_err("%s: ODM query configured improperly\n", __func__);
		WARN_ON(1);
		return;
	}

	odm_nr = odm_table[uart - NvOdmDebugConsole_UartA];

	if (uart == NvOdmDebugConsole_UartA) {
		pinmux = tegra_pinmux_get("tegra_uart.0", odm_nr, &nr_pins);
		c = clk_get_sys("uart.0", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTA_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTA_BASE;
		debug_uart_platform[0].irq = INT_UARTA;
	} else if (uart == NvOdmDebugConsole_UartB) {
		pinmux = tegra_pinmux_get("tegra_uart.1", odm_nr, &nr_pins);
		c = clk_get_sys("uart.1", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTB_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTB_BASE;
		debug_uart_platform[0].irq = INT_UARTB;
	} else if (uart == NvOdmDebugConsole_UartC) {
		pinmux = tegra_pinmux_get("tegra_uart.2", odm_nr, &nr_pins);
		c = clk_get_sys("uart.2", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTC_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTC_BASE;
		debug_uart_platform[0].irq = INT_UARTC;
	} else if (uart == NvOdmDebugConsole_UartD) {
		pinmux = tegra_pinmux_get("tegra_uart.3", odm_nr, &nr_pins);
		c = clk_get_sys("uart.3", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTD_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTD_BASE;
		debug_uart_platform[0].irq = INT_UARTD;
	} else if (uart == NvOdmDebugConsole_UartE) {
		pinmux = tegra_pinmux_get("tegra_uart.4", odm_nr, &nr_pins);
		c = clk_get_sys("uart.4", NULL);
		debug_uart_platform[0].membase = IO_ADDRESS(TEGRA_UARTE_BASE);
		debug_uart_platform[0].mapbase = TEGRA_UARTE_BASE;
		debug_uart_platform[0].irq = INT_UARTE;
	}

	if (!c || !pinmux || !nr_pins) {
		if (c)
			clk_put(c);
		return;
	}

	tegra_pinmux_config_tristate_table(pinmux, nr_pins, TEGRA_TRI_NORMAL);
	clk_set_rate(c, 115200*16);
	clk_enable(c);
	debug_uart_platform[0].uartclk = clk_get_rate(c);

	platform_device_register(&debug_uart);

	uart_debug_port.port = uart;
	uart_debug_port.pinmux = pinmux;
	uart_debug_port.nr_pins = nr_pins;
	uart_debug_port.clk_data = c;
}

static void tegra_debug_port_suspend(void)
{
	if (uart_debug_port.port == NvOdmDebugConsole_None)
		return;
	clk_disable(uart_debug_port.clk_data);
	tegra_pinmux_config_tristate_table(uart_debug_port.pinmux,
				uart_debug_port.nr_pins, TEGRA_TRI_TRISTATE);
}

static void tegra_debug_port_resume(void)
{
	if (uart_debug_port.port == NvOdmDebugConsole_None)
		return;
	clk_enable(uart_debug_port.clk_data);
	tegra_pinmux_config_tristate_table(uart_debug_port.pinmux,
				uart_debug_port.nr_pins, TEGRA_TRI_NORMAL);
}


#ifdef CONFIG_MMC_SDHCI_TEGRA
extern struct tegra_nand_platform tegra_nand_plat;
static struct tegra_sdhci_platform_data tegra_sdhci_platform[] = {
	[0] = {
		.bus_width = 4,
		.debounce = 5,
	},
	[1] = {
		.bus_width = 4,
		.debounce = 5,
	},
	[2] = {
		.bus_width = 4,
		.debounce = 5,
	},
	[3] = {
		.bus_width = 4,
		.debounce = 5,
	},
};
static struct resource tegra_sdhci_resources[][2] = {
	[0] = {
		[0] = {
			.start = TEGRA_SDMMC1_BASE,
			.end = TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC1,
			.end = INT_SDMMC1,
			.flags = IORESOURCE_IRQ,
		},
	},
	[1] = {
		[0] = {
			.start = TEGRA_SDMMC2_BASE,
			.end = TEGRA_SDMMC2_BASE + TEGRA_SDMMC2_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC2,
			.end = INT_SDMMC2,
			.flags = IORESOURCE_IRQ,
		},
	},
	[2] = {
		[0] = {
			.start = TEGRA_SDMMC3_BASE,
			.end = TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC3,
			.end = INT_SDMMC3,
			.flags = IORESOURCE_IRQ,
		},
	},
	[3] = {
		[0] = {
			.start = TEGRA_SDMMC4_BASE,
			.end = TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC4,
			.end = INT_SDMMC4,
			.flags = IORESOURCE_IRQ,
		},
	},
};
static struct platform_device tegra_sdhci_devices[] = {
	[0] = {
		.id = 0,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[0],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[0]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[1] = {
		.id = 1,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[1],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[1]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[2] = {
		.id = 2,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[2],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[2]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[3] = {
		.id = 3,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[3],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[3]),
		.dev = {
			.platform_data = &tegra_sdhci_platform[3],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
};

#define active_high(_pin) ((_pin)->activeState == NvOdmGpioPinActiveState_High ? 1 : 0)

static void __init tegra_setup_sdhci(void) {
	const NvOdmGpioPinInfo *gpio;
	struct tegra_sdhci_platform_data *plat;
	const NvU32 *clock_limits;
	const NvU32 *pinmux;
	NvU32 nr_pinmux;
	NvU32 clock_count;
	NvU32 gpio_count;
	NvRmModuleSdmmcInterfaceCaps caps;
	int i;

	NvOdmQueryClockLimits(NvOdmIoModule_Sdio, &clock_limits, &clock_count);
	NvOdmQueryPinMux(NvOdmIoModule_Sdio, &pinmux, &nr_pinmux);

#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	/* check if an "MBR" partition was parsed from the tegra partition
	 * command line, and store it in sdhci.3's offset field */
	for (i=0; i<tegra_nand_plat.nr_parts; i++) {
		plat = &tegra_sdhci_platform[3];
		if (strcmp("mbr", tegra_nand_plat.parts[i].name))
			continue;
		plat->offset = tegra_nand_plat.parts[i].offset;
	}
#endif

	for (i=ARRAY_SIZE(tegra_sdhci_platform)-1; i>=0; i--) {
		const NvOdmQuerySdioInterfaceProperty *prop;
		prop = NvOdmQueryGetSdioInterfaceProperty(i);
		if (!prop || prop->usage==NvOdmQuerySdioSlotUsage_unused)
			continue;

		plat = &tegra_sdhci_platform[i];
		gpio = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio,
			i, &gpio_count);

		plat->is_removable = prop->IsCardRemovable;
		plat->is_always_on = prop->AlwaysON;

#ifdef CONFIG_MACH_VENTANA
		if (prop->usage == NvOdmQuerySdioSlotUsage_wlan)
			plat->register_status_notify =
				ventana_wifi_status_register;
#endif

		if (!gpio)
			gpio_count = 0;
		switch (gpio_count) {
		case 2:
			plat->gpio_nr_wp = 8*gpio[1].Port + gpio[1].Pin;
			plat->gpio_nr_cd = 8*gpio[0].Port + gpio[0].Pin;
			plat->gpio_polarity_wp = active_high(&gpio[1]);
			plat->gpio_polarity_cd = active_high(&gpio[0]);
			break;
		case 1:
			plat->gpio_nr_wp = -1;
			plat->gpio_nr_cd = 8*gpio[0].Port + gpio[0].Pin;
			plat->gpio_polarity_cd = active_high(&gpio[0]);
			break;
		case 0:
			plat->gpio_nr_wp = -1;
			plat->gpio_nr_cd = -1;
			break;
		}

		if (NvRmGetModuleInterfaceCapabilities(s_hRmGlobal,
			NVRM_MODULE_ID(NvRmModuleID_Sdio, i),
			sizeof(caps), &caps)==NvSuccess)
			plat->bus_width = caps.MmcInterfaceWidth;

		if (clock_limits && i<clock_count)
			plat->max_clk = clock_limits[i] * 1000;

		if (pinmux && i<nr_pinmux) {
			char name[20];
			snprintf(name, sizeof(name), "tegra-sdhci.%d", i);
			plat->pinmux = tegra_pinmux_get(name,
				pinmux[i], &plat->nr_pins);
		}

		platform_device_register(&tegra_sdhci_devices[i]);
	}
}
#else
static void __init tegra_setup_sdhci(void) { }
#endif

#ifdef CONFIG_SERIAL_TEGRA
static struct tegra_serial_platform_data tegra_uart_platform[] = {
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTA_BASE),
			.mapbase = TEGRA_UARTA_BASE,
			.irq = INT_UARTA,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTB_BASE),
			.mapbase = TEGRA_UARTB_BASE,
			.irq = INT_UARTB,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTC_BASE),
			.mapbase = TEGRA_UARTC_BASE,
			.irq = INT_UARTC,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTD_BASE),
			.mapbase = TEGRA_UARTD_BASE,
			.irq = INT_UARTD,
		},
	},
	{
		.p = {
			.membase = IO_ADDRESS(TEGRA_UARTE_BASE),
			.mapbase = TEGRA_UARTE_BASE,
			.irq = INT_UARTE,
		},
	},
};
static struct platform_device tegra_uart[] = {
	{
		.name = "tegra_uart",
		.id = 0,
		.dev = {
			.platform_data = &tegra_uart_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 1,
		.dev = {
			.platform_data = &tegra_uart_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 2,
		.dev = {
			.platform_data = &tegra_uart_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 3,
		.dev = {
			.platform_data = &tegra_uart_platform[3],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	{
		.name = "tegra_uart",
		.id = 4,
		.dev = {
			.platform_data = &tegra_uart_platform[4],
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},

};
static void __init tegra_setup_hsuart(void)
{
	NvOdmDebugConsole uart = NvOdmQueryDebugConsole();
	int dbg_id = (int)uart - (int)NvOdmDebugConsole_UartA;
	const NvU32 *odm_table;
	NvU32 odm_nr;
	int i;

	NvOdmQueryPinMux(NvOdmIoModule_Uart, &odm_table, &odm_nr);

	for (i=0; i<ARRAY_SIZE(tegra_uart); i++) {
		struct tegra_serial_platform_data *plat;
		char name[16];

		if (i==dbg_id)
			continue;

		if (odm_table[i] == 0)
			continue;

		plat = &tegra_uart_platform[i];

		snprintf(name, sizeof(name), "%s.%d",
			 tegra_uart[i].name, tegra_uart[i].id);

		if (i < odm_nr) {
			plat->pinmux = tegra_pinmux_get(name,
				odm_table[i], &plat->nr_pins);
		} else {
			plat->pinmux = NULL;
			plat->nr_pins = 0;
		}

		if (platform_device_register(&tegra_uart[i])) {
			pr_err("%s: failed to register %s.%d\n",
			       __func__, tegra_uart[i].name, tegra_uart[i].id);
		}
	}
}
#else
static void __init tegra_setup_hsuart(void) { }
#endif

#ifdef CONFIG_USB_TEGRA_HCD
static struct tegra_hcd_platform_data tegra_hcd_platform[] = {
	[0] = {
		.instance = 0,
	},
	[1] = {
		.instance = 1,
	},
	[2] = {
		.instance = 2,
	},
};
static struct resource tegra_hcd_resources[][2] = {
	[0] = {
		[0] = {
			.flags = IORESOURCE_MEM,
			.start = TEGRA_USB_BASE,
			.end = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		},
		[1] = {
			.flags = IORESOURCE_IRQ,
			.start = INT_USB,
			.end = INT_USB,
		},
	},
	[1] = {
		[0] = {
			.flags = IORESOURCE_MEM,
			.start = TEGRA_USB1_BASE,
			.end = TEGRA_USB1_BASE + TEGRA_USB1_SIZE - 1,
		},
		[1] = {
			.flags = IORESOURCE_IRQ,
			.start = INT_USB2,
			.end = INT_USB2,
		},
	},
	[2] = {
		[0] = {
			.flags = IORESOURCE_MEM,
			.start = TEGRA_USB2_BASE,
			.end = TEGRA_USB2_BASE + TEGRA_USB2_SIZE - 1,
		},
		[1] = {
			.flags = IORESOURCE_IRQ,
			.start = INT_USB3,
			.end = INT_USB3,
		},
	},
};
/* EHCI transfers must be 32B aligned */
static u64 tegra_ehci_dma_mask = DMA_BIT_MASK(32) & ~0x1f;
static struct platform_device tegra_hcd[] = {
	[0] = {
		.name = "tegra-ehci",
		.id = 0,
		.dev = {
			.platform_data = &tegra_hcd_platform[0],
			.coherent_dma_mask = DMA_BIT_MASK(32) & ~0x1f,
			.dma_mask = &tegra_ehci_dma_mask,
		},
		.resource = tegra_hcd_resources[0],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[0]),
	},
	[1] = {
		.name = "tegra-ehci",
		.id = 1,
		.dev = {
			.platform_data = &tegra_hcd_platform[1],
			.coherent_dma_mask = DMA_BIT_MASK(32) & ~0x1f,
			.dma_mask = &tegra_ehci_dma_mask,
		},
		.resource = tegra_hcd_resources[1],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[1]),
	},
	[2] = {
		.name = "tegra-ehci",
		.id = 2,
		.dev = {
			.platform_data = &tegra_hcd_platform[2],
			.coherent_dma_mask = DMA_BIT_MASK(32) & ~0x1f,
			.dma_mask = &tegra_ehci_dma_mask,
		},
		.resource = tegra_hcd_resources[2],
		.num_resources = ARRAY_SIZE(tegra_hcd_resources[2]),
	},
};

#ifdef CONFIG_USB_TEGRA_OTG
#define otg_is_okay(_instance) ((_instance)==0)
static struct tegra_otg_platform_data tegra_otg_platform = {
	.instance = 0,
};
static struct resource tegra_otg_resources[] = {
	[0] = {
		.start = TEGRA_USB_BASE,
		.end = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_USB,
		.end = INT_USB,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device tegra_otg = {
	.name = "tegra-otg",
	.id = 0,
	.resource = tegra_otg_resources,
	.num_resources = ARRAY_SIZE(tegra_otg_resources),
	.dev = {
		.platform_data = &tegra_otg_platform,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &tegra_dma_mask,
	},
};
#else
#define otg_is_okay(_instance) (0)
#endif

static void __init tegra_setup_hcd(void)
{
	int i;

	for (i=0; i<ARRAY_SIZE(tegra_hcd_platform); i++) {
		const NvOdmUsbProperty *p;
		struct tegra_hcd_platform_data *plat = &tegra_hcd_platform[i];

		p = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, i);

		if ((p->UsbMode == NvOdmUsbModeType_Device) ||
		    (p->UsbMode == NvOdmUsbModeType_None))
			continue;

		plat->otg_mode = (p->UsbMode == NvOdmUsbModeType_OTG);
		if (plat->otg_mode && !otg_is_okay(i)) {
			pr_err("%s: OTG not enabled in kernel for USB "
			       "controller %d, but ODM kit specifes OTG\n",
			       __func__, i);
			continue;
		}
#ifdef CONFIG_USB_TEGRA_OTG
		if (plat->otg_mode && otg_is_okay(i)) {
			tegra_otg_platform.usb_property = p;
			platform_device_register(&tegra_otg);
		}
#endif
		if (p->IdPinDetectionType == NvOdmUsbIdPinType_Gpio) {
			const NvOdmGpioPinInfo *gpio;
			NvU32 count;

			gpio = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Usb,
						    i, &count);
			if (!gpio || (count<=NvOdmGpioPin_UsbCableId)) {
				pr_err("%s: invalid ODM query for controller "
				       "%d\n", __func__, i);
				WARN_ON(1);
				continue;
			}
			plat->id_detect = ID_PIN_GPIO;
			gpio += NvOdmGpioPin_UsbCableId;
			plat->gpio_nr = gpio->Port*8 + gpio->Pin;
		} else if (p->IdPinDetectionType == NvOdmUsbIdPinType_CableId) {
			plat->id_detect = ID_PIN_CABLE_ID;
		}
		platform_device_register(&tegra_hcd[i]);
	}
}
#else
static inline void tegra_setup_hcd(void) { }
#endif

#ifdef CONFIG_KEYBOARD_TEGRA
struct tegra_kbc_plat tegra_kbc_platform;

static noinline void __init tegra_setup_kbc(void)
{
	struct tegra_kbc_plat *pdata = &tegra_kbc_platform;
	const NvOdmPeripheralConnectivity *conn;
	NvOdmPeripheralSearch srch_attr = NvOdmPeripheralSearch_IoModule;
	const struct NvOdmKeyVirtTableDetail **vkeys;
	NvU32 srch_val = NvOdmIoModule_Kbd;
	NvU32 temp;
	NvU64 guid;
	NvU32 i, j, k;
	NvU32 cols=0;
	NvU32 rows=0;
	NvU32 *wake_row;
	NvU32 *wake_col;
	NvU32 wake_num;
	NvU32 vnum;

	pdata->keymap = kzalloc(sizeof(*pdata->keymap)*KBC_MAX_KEY, GFP_KERNEL);
	if (!pdata->keymap) {
		pr_err("%s: out of memory for key mapping\n", __func__);
		return;
	}
	pdata->wake_cnt = 0;
	if (NvOdmKbcIsSelectKeysWkUpEnabled(&wake_row, &wake_col, &wake_num)) {
		BUG_ON(!wake_num || wake_num>=KBC_MAX_KEY);
		pdata->wake_cfg = kzalloc(sizeof(*pdata->wake_cfg)*wake_num,
			GFP_KERNEL);
		if (pdata->wake_cfg) {
			pdata->wake_cnt = (int)wake_num;
			for (i=0; i<wake_num; i++) {
				pdata->wake_cfg[i].row=wake_row[i];
				pdata->wake_cfg[i].col=wake_col[i];
			}
		} else
			pr_err("disabling wakeup key filtering due to "
				"out-of-memory error\n");
	}

	NvOdmKbcGetParameter(NvOdmKbcParameter_DebounceTime, 1, &temp);

	/* debounce time is reported from ODM in terms of clock ticks. */
	pdata->debounce_cnt = temp;

	/* Get the scanning timeout in terms of MilliSeconds.*/
	temp = 0;
	NvOdmKbcGetParameter(NvOdmKbcParameter_KeyScanTimeout, 1, &temp);
	/* If value is 0 then set it to 5 second as default */
	if (!temp)
		temp = 5000;
	/* Convert Milliseconds to clock count of 32Kz */
	pdata->scan_timeout_cnt = temp*32;

	/* repeat cycle is reported from ODM in milliseconds,
	 * but needs to be specified in 32KHz ticks */
	temp = 0;
	NvOdmKbcGetParameter(NvOdmKbcParameter_RepeatCycleTime, 1, &temp);
	pdata->repeat_cnt = temp * 32;

	temp = NvOdmPeripheralEnumerate(&srch_attr, &srch_val, 1, &guid, 1);
	if (!temp) {
		kfree(pdata->keymap);
		pr_err("%s: failed to find keyboard module\n", __func__);
		return;
	}
	conn = NvOdmPeripheralGetGuid(guid);
	if (!conn) {
		kfree(pdata->keymap);
		pr_err("%s: failed to find keyboard\n", __func__);
		return;
	}

	for (i=0; i<conn->NumAddress; i++) {
		NvU32 addr = conn->AddressList[i].Address;

		if (conn->AddressList[i].Interface!=NvOdmIoModule_Kbd) continue;

		if (conn->AddressList[i].Instance) {
			pdata->pin_cfg[addr].num = cols++;
			pdata->pin_cfg[addr].is_col = true;
		} else {
			pdata->pin_cfg[addr].num = rows++;
			pdata->pin_cfg[addr].is_row = true;
		}
	}

	for (i=0; i<KBC_MAX_KEY; i++)
		pdata->keymap[i] = -1;

	vnum = NvOdmKbcKeyMappingGetVirtualKeyMappingList(&vkeys);

	for (i=0; i<rows; i++) {
		for (j=0; j<cols; j++) {
			NvU32 sc = NvOdmKbcGetKeyCode(i, j, rows, cols);
			for (k=0; k<vnum; k++) {
				if (sc >= vkeys[k]->StartScanCode &&
				    sc <= vkeys[k]->EndScanCode) {
					sc -= vkeys[k]->StartScanCode;
					sc = vkeys[k]->pVirtualKeyTable[sc];
					if (!sc) continue;
					pdata->keymap[kbc_indexof(i,j)]=sc;
				}

                        }
		}
	}
}
#else
static void tegra_setup_kbc(void) { }
#endif

#if defined(CONFIG_KEYBOARD_GPIO)
struct gpio_keys_platform_data tegra_button_data;
static char *gpio_key_names = "gpio_keys";
static noinline void __init tegra_setup_gpio_key(void)
{
	struct gpio_keys_button *tegra_buttons = NULL;
	int ngpiokeys = 0;
	const NvOdmGpioPinInfo *gpio_key_info;
	int i;
	NvOdmGpioPinKeyInfo *gpio_pin_info = NULL;

	gpio_key_info = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_keypadMisc, 0,
						 &ngpiokeys);

	if (!ngpiokeys) {
		pr_info("No gpio is configured as buttons\n");
		return;
	}

	tegra_buttons = kzalloc(ngpiokeys * sizeof(struct gpio_keys_button),
				 GFP_KERNEL);
	if (!tegra_buttons) {
		pr_err("Memory allocation failed for tegra_buttons\n");
		return;
	}

	for (i = 0; i < ngpiokeys; ++i) {
		tegra_buttons[i].gpio =
			(int)(gpio_key_info[i].Port*8 + gpio_key_info[i].Pin);
		gpio_pin_info = gpio_key_info[i].GpioPinSpecificData;
		tegra_buttons[i].code = (int)gpio_pin_info->Code;
		tegra_buttons[i].desc = gpio_key_names;

		if (gpio_key_info[i].activeState == NvOdmGpioPinActiveState_Low)
			tegra_buttons[i].active_low = 1;
		else
			tegra_buttons[i].active_low = 0;
		tegra_buttons[i].type = EV_KEY;
		tegra_buttons[i].wakeup = (gpio_pin_info->Wakeup)? 1: 0;
		tegra_buttons[i].debounce_interval =
				 gpio_pin_info->DebounceTimeMs;
	}

	tegra_button_data.buttons = tegra_buttons;
	tegra_button_data.nbuttons = ngpiokeys;
	return;
}
#else
static void tegra_setup_gpio_key(void) { }
#endif

#ifdef CONFIG_RTC_DRV_TEGRA
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};
#endif

#ifdef CONFIG_RTC_DRV_TEGRA_ODM
static struct platform_device tegra_rtc_odm_device = {
	.name = "tegra_rtc_odm",
	.id   = -1,
};
#endif

#ifdef CONFIG_TEGRA_NVEC
static struct platform_device tegra_nvec_device = {
	.name = "nvec",
	.id = -1,
};
#endif
#if defined(CONFIG_TEGRA_BATTERY_NVEC) || defined(CONFIG_TEGRA_BATTERY_ODM)
static struct platform_device tegra_battery_device = {
	.name = "tegra_battery",
	.id = -1,
};
#endif
#ifdef CONFIG_REGULATOR_TEGRA
static struct regulator_consumer_supply pex_clk_consumers[] = {
	[0] = {
		.supply = "pex_clk",
	},
};
static struct regulator_consumer_supply lbee9qmb_consumers[] = {
	[0] = {
		.supply = "Vdd",
		.dev_name = "lbee9qmb-rfkill.0",
	},
};
static struct regulator_consumer_supply tegra_soc_consumers[] = {
	[0] = {
		.supply = "soc_main",
	},
};
static struct regulator_consumer_supply tegra_vdd_bb_consumers[] = {
	[0] = {
		.supply   = "vddio bb",
	},
};
static struct regulator_consumer_supply tegra_vdd_lcd_consumers[] = {
	[0] = {
		.supply   = "vddio lcd",
	},
};
static struct regulator_consumer_supply tegra_vdd_vi_consumers[] = {
	[0] = {
		.supply   = "vddio vi",
	},
};
static struct regulator_consumer_supply tegra_vdd_uart_consumers[] = {
	[0] = {
		.supply   = "vddio uart",
	},
};
static struct regulator_consumer_supply tegra_vdd_ddr_consumers[] = {
	[0] = {
		.supply   = "vddio ddr",
	},
};
static struct regulator_consumer_supply tegra_vdd_nand_consumers[] = {
	[0] = {
		.supply   = "vddio nand",
	},
};
static struct regulator_consumer_supply tegra_vdd_sys_consumers[] = {
	[0] = {
		.supply   = "vddio sys",
	},
};
static struct regulator_consumer_supply tegra_vdd_audio_consumers[] = {
	[0] = {
		.supply   = "vddio audio",
	},
};
static struct regulator_consumer_supply tegra_vdd_sd_consumers[] = {
	[0] = {
		.supply   = "vddio sd",
	},
};

#ifdef CONFIG_TEGRA_USB_CHARGE
static struct regulator_consumer_supply tegra_vbus_consumers[] = {
	[0] = {
		.supply = "vbus_draw",
		.dev_name = "tegra-udc.0",
	},
};
#endif
static struct tegra_regulator_entry tegra_regulators[] = {
	[0] = {
		.guid = NV_VDD_PEX_CLK_ODM_ID,
		.name = "pex_clk",
		.id = 0,
		.consumers = pex_clk_consumers,
		.nr_consumers = ARRAY_SIZE(pex_clk_consumers),
	},
	[1] = {
		.guid = NV_ODM_GUID('b','c','m','_','4','3','2','9'),
		.name = "lbee9qmb_vdd",
		.id = 1,
		.consumers = lbee9qmb_consumers,
		.nr_consumers = ARRAY_SIZE(lbee9qmb_consumers),
	},
	[2] = {
		.guid = NV_VDD_SoC_ODM_ID,
		.name = "soc_main",
		.id = 2,
		.consumers = tegra_soc_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_soc_consumers),
	},
	[3] = {
		.guid = NV_VDD_BB_ODM_ID,
		.name = "vddio bb",
		.id = 3,
		.consumers = tegra_vdd_bb_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_bb_consumers),
	},
	[4] = {
		.guid = NV_VDD_LCD_ODM_ID,
		.name = "vddio lcd",
		.id = 4,
		.consumers = tegra_vdd_lcd_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_lcd_consumers),
	},
	[5] = {
		.guid = NV_VDD_VI_ODM_ID,
		.name = "vddio vi",
		.id = 5,
		.consumers = tegra_vdd_vi_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_vi_consumers),
	},
	[6] = {
		.guid = NV_VDD_UART_ODM_ID,
		.name = "vddio uart",
		.id = 6,
		.consumers = tegra_vdd_uart_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_uart_consumers),
	},
	[7] = {
		.guid = NV_VDD_DDR_ODM_ID,
		.name = "vddio ddr",
		.id = 7,
		.consumers = tegra_vdd_ddr_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_ddr_consumers),
	},
	[8] = {
		.guid = NV_VDD_NAND_ODM_ID,
		.name = "vddio nand",
		.id = 8,
		.consumers = tegra_vdd_nand_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_nand_consumers),
	},
	[9] = {
		.guid = NV_VDD_SYS_ODM_ID,
		.name = "vddio sys",
		.id = 9,
		.consumers = tegra_vdd_sys_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_sys_consumers),
	},
	[10] = {
		.guid = NV_VDD_AUD_ODM_ID,
		.name = "vddio audio",
		.id = 10,
		.consumers = tegra_vdd_audio_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_audio_consumers),
	},
	[11] = {
		.guid = NV_VDD_SDIO_ODM_ID,
		.name = "vddio sd",
		.id = 11,
		.consumers = tegra_vdd_sd_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vdd_sd_consumers),
	},
	[12] = {
		.guid = NV_ODM_GUID('l','b','e','e','9','q','m','b'),
		.name = "lbee9qmb_vdd",
		.id = 12,
		.consumers = lbee9qmb_consumers,
		.nr_consumers = ARRAY_SIZE(lbee9qmb_consumers),
	},
#ifdef CONFIG_TEGRA_USB_CHARGE
	[13] = {
		.charging_path = NvRmPmuChargingPath_UsbBus,
		.name = "vbus_draw",
		.id = 13,
		.consumers = tegra_vbus_consumers,
		.nr_consumers = ARRAY_SIZE(tegra_vbus_consumers),
		.is_charger = true,
	},
#endif
};
static struct tegra_regulator_platform_data tegra_regulator_platform = {
	.regs = tegra_regulators,
	.nr_regs = ARRAY_SIZE(tegra_regulators),
};
static struct platform_device tegra_regulator_device = {
	.name = "tegra_regulator",
	.id = -1,
	.dev = {
		.platform_data = &tegra_regulator_platform,
	},
};
#endif
#ifdef CONFIG_LBEE9QMB_RFKILL
static struct lbee9qmb_platform_data lbee9qmb_platform;
static struct platform_device lbee9qmb_device = {
	.name = "lbee9qmb-rfkill",
	.dev = {
		.platform_data = &lbee9qmb_platform,
	},
};
static noinline void __init tegra_setup_rfkill(void)
{
	const NvOdmPeripheralConnectivity *con;
	unsigned int i;
	lbee9qmb_platform.delay=5;
	lbee9qmb_platform.gpio_pwr=-1;
	if ((con = NvOdmPeripheralGetGuid(NV_ODM_GUID('l','b','e','e','9','q','m','b'))))
	{
		for (i=0; i<con->NumAddress; i++) {
			if (con->AddressList[i].Interface == NvOdmIoModule_Gpio
					&& con->AddressList[i].Purpose == BT_RESET ){
				int nr_gpio = con->AddressList[i].Instance * 8 +
					con->AddressList[i].Address;
				lbee9qmb_platform.gpio_reset = nr_gpio;
				if (platform_device_register(&lbee9qmb_device))
					pr_err("%s: registration failed\n", __func__);
				return;
			}
		}
	}
	else if ((con = NvOdmPeripheralGetGuid(NV_ODM_GUID('b','c','m','_','4','3','2','9'))))
	{
		int nr_gpio;
		for (i=0; i<con->NumAddress; i++) {
                        if (con->AddressList[i].Interface == NvOdmIoModule_Gpio
						&& con->AddressList[i].Purpose == BT_RESET){
					nr_gpio = con->AddressList[i].Instance * 8 +
						con->AddressList[i].Address;
					lbee9qmb_platform.gpio_reset = nr_gpio;
				}
			else if (con->AddressList[i].Interface == NvOdmIoModule_Gpio
						&& con->AddressList[i].Purpose == BT_SHUTDOWN ){
					nr_gpio = con->AddressList[i].Instance * 8 +
						 con->AddressList[i].Address;
					lbee9qmb_platform.gpio_pwr = nr_gpio;
				}
		}
		lbee9qmb_platform.delay=200;
                if (platform_device_register(&lbee9qmb_device))
			pr_err("%s: registration failed\n", __func__);
                return;
        }
        return;
}
#else
static void tegra_setup_rfkill(void) { }
#endif

#ifdef CONFIG_TOUCHSCREEN_TEGRA_ODM
static struct platform_device tegra_touch_device = {
	.name = "tegra_touch",
	.id = -1,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
static struct panjit_i2c_ts_platform_data panjit_data = {
	.gpio_reset = TEGRA_GPIO_PQ7,
};

static const struct i2c_board_info ventana_i2c_bus1_touch_info[] = {
	{
	 I2C_BOARD_INFO("panjit_touch", 0x3),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &panjit_data,
	 },
};

static int __init ventana_touch_init_panjit(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);	/* FIXME:  Ventana-specific GPIO assignment	*/
	tegra_gpio_enable(TEGRA_GPIO_PQ7);	/* FIXME:  Ventana-specific GPIO assignment	*/
	i2c_register_board_info(0, ventana_i2c_bus1_touch_info, 1);

	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
/* Atmel MaxTouch touchscreen              Driver data */
/*-----------------------------------------------------*/
/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */
static u8 read_chg()
{
	return gpio_get_value(TEGRA_GPIO_PV6);
}

static u8 valid_interrupt()
{
	return !read_chg();
}

static struct mxt_platform_data Atmel_mxt_info = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch = 10,
	// TODO: no need for any hw-specific things at init/exit?
	.init_platform_hw = NULL,
	.exit_platform_hw = NULL,
	.max_x = 1366,
	.max_y = 768,
	.valid_interrupt = &valid_interrupt,
	.read_chg = &read_chg,
};

static struct i2c_board_info __initdata i2c_info[] = {
	{
	 I2C_BOARD_INFO("maXTouch", MXT_I2C_ADDRESS),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &Atmel_mxt_info,
	 },
};

static int __init ventana_touch_init_atmel(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);	/* FIXME:  Ventana-specific GPIO assignment	*/
	tegra_gpio_enable(TEGRA_GPIO_PQ7);	/* FIXME:  Ventana-specific GPIO assignment	*/

	gpio_set_value(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, i2c_info, 1);

	return 0;
}
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_ACCEL
static struct platform_device tegra_accelerometer_device = {
	.name = "lsm303dlh_acc",
	.id   = -1,
};
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_SCROLL
static struct platform_device tegra_scrollwheel_device = {
	.name = "tegra_scrollwheel",
	.id   = -1,
};
#endif

/* Daniel20100805 */
#ifdef CONFIG_INPUT_TEGRA_ODM_ECOMPASS 
static struct platform_device tegra_ecompass_device =
{
    .name = "tegra_ecompass",
    .id   = -1,
};
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_DOCK
static struct platform_device tegra_dock_device =  
{
    .name = "tegra_dock",
    .id   = -1,
};
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_CAPSENSOR             
static struct platform_device tegra_capsensor_device =
{
    .name = "tegra_capsensor",
    .id   = -1,
};
#endif

#if 1
/* ATHENV */
static struct platform_device tegra_wlan_ar6000_pm_device = {
	.name		= "wlan_ar6000_pm",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
};
/* ATHENV */
#endif

static struct platform_device *nvodm_devices[] __initdata = {
#ifdef CONFIG_RTC_DRV_TEGRA
	&tegra_rtc_device,
#endif
#if 1
/* ATHENV */
    /* 
     * It is necessary to put here in order to support WoW.
     * Put it before MMC host controller in worst case 
     */
	&tegra_wlan_ar6000_pm_device,
/* ATHENV */
#endif
#ifdef CONFIG_RTC_DRV_TEGRA_ODM
	&tegra_rtc_odm_device,
#endif
#ifdef CONFIG_TEGRA_NVEC
	&tegra_nvec_device,
#endif
#if defined(CONFIG_TEGRA_BATTERY_NVEC) || defined(CONFIG_TEGRA_BATTERY_ODM)
	&tegra_battery_device,
#endif
#ifdef CONFIG_REGULATOR_TEGRA
	&tegra_regulator_device,
#endif
#ifdef CONFIG_TOUCHSCREEN_TEGRA_ODM
	&tegra_touch_device,
#endif
#ifdef CONFIG_INPUT_TEGRA_ODM_SCROLL
	&tegra_scrollwheel_device,
#endif
#ifdef CONFIG_INPUT_TEGRA_ODM_ACCEL
	&tegra_accelerometer_device,
#endif
#ifdef CONFIG_INPUT_TEGRA_ODM_CAPSENSOR
	&tegra_capsensor_device,
#endif
#ifdef CONFIG_INPUT_TEGRA_ODM_ECOMPASS 
        &tegra_ecompass_device,
#endif
#ifdef CONFIG_INPUT_TEGRA_ODM_DOCK
	&tegra_dock_device,
#endif

};

#ifdef CONFIG_SPI_TEGRA
static struct tegra_spi_platform_data tegra_spi_platform[] = {
	[0] = {
		.is_slink = true,
	},
	[1] = {
		.is_slink = true,
	},
	[2] = {
		.is_slink = true,
	},
	[3] = {
		.is_slink = true,
	},
	[4] = {
		.is_slink = false,
	},
};
static struct platform_device tegra_spi_devices[] = {
	[0] = {
		.name = "tegra_spi",
		.id = 0,
		.dev = {
			.platform_data = &tegra_spi_platform[0],
		},
	},
	[1] = {
		.name = "tegra_spi",
		.id = 1,
		.dev = {
			.platform_data = &tegra_spi_platform[1],
		},
	},
	[2] = {
		.name = "tegra_spi",
		.id = 2,
		.dev = {
			.platform_data = &tegra_spi_platform[2],
		},
	},
	[3] = {
		.name = "tegra_spi",
		.id = 3,
		.dev = {
			.platform_data = &tegra_spi_platform[3],
		},
	},
	[4] = {
		.name = "tegra_spi",
		.id = 4,
		.dev = {
			.platform_data = &tegra_spi_platform[4],
		},
	},
};
static noinline void __init tegra_setup_spi(void)
{
	const NvU32 *spi_mux;
	const NvU32 *sflash_mux;
	NvU32 spi_mux_nr;
	NvU32 sflash_mux_nr;
	int i;

	NvOdmQueryPinMux(NvOdmIoModule_Spi, &spi_mux, &spi_mux_nr);
	NvOdmQueryPinMux(NvOdmIoModule_Sflash, &sflash_mux, &sflash_mux_nr);

	for (i=0; i<ARRAY_SIZE(tegra_spi_devices); i++) {
		struct platform_device *pdev = &tegra_spi_devices[i];
		struct tegra_spi_platform_data *plat = &tegra_spi_platform[i];

		const NvOdmQuerySpiDeviceInfo *info = NULL;
		NvU32 mux = 0;
		int rc;

		if (plat->is_slink && pdev->id<spi_mux_nr)
			mux = spi_mux[pdev->id];
		else if (sflash_mux_nr && !plat->is_slink)
			mux = sflash_mux[0];

		if (!mux)
			continue;

		if (mux == NVODM_QUERY_PINMAP_MULTIPLEXED) {
			pr_err("%s: not registering multiplexed SPI master "
			       "%s.%d\n", __func__, pdev->name, pdev->id);
			WARN_ON(1);
			continue;
		}

		if (plat->is_slink) {
			info = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Spi,
							  pdev->id, 0);
		} else {
			info = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Sflash,
							  0, 0);
		}

		if (info && info->IsSlave) {
			pr_info("%s: not registering SPI slave %s.%d\n",
				__func__, pdev->name, pdev->id);
			continue;
		}

		rc = platform_device_register(pdev);
		if (rc) {
			pr_err("%s: registration of %s.%d failed\n",
			       __func__, pdev->name, pdev->id);
		}
	}
}
#else
static void tegra_setup_spi(void) { }
#endif

#ifdef CONFIG_I2C_TEGRA
#ifdef CONFIG_TEGRA_ODM_VENTANA
static struct tegra_i2c_plat_parms tegra_i2c_platform[] = {
	[0] = {
		.adapter_nr = 0,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 }, /* default to 100KHz */
		.is_dvc = false,
	},
	[1] = {
		.adapter_nr = 1,
		.bus_count = 2,
		.bus_mux = { NvOdmI2cPinMap_Config1, NvOdmI2cPinMap_Config2},
		.bus_clk = { 100000, 100000 },
		.is_dvc = false,
	},
	[2] = {
		.adapter_nr = 3,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = false,
	},
	[3] = {
		.adapter_nr = 4,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = true,
	},
};
#else
static struct tegra_i2c_plat_parms tegra_i2c_platform[] = {
	[0] = {
		.adapter_nr = 0,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 }, /* default to 100KHz */
		.is_dvc = false,
	},
	[1] = {
		.adapter_nr = 1,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = false,
	},
	[2] = {
		.adapter_nr = 2,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = false,
	},
	[3] = {
		.adapter_nr = 3,
		.bus_count = 1,
		.bus_mux = { 0, 0 },
		.bus_clk = { 100000, 0 },
		.is_dvc = true,
	},
};
#endif
static struct platform_device tegra_i2c_devices[] = {
	[0] = {
		.name = "tegra_i2c",
		.id = 0,
		.dev = {
			.platform_data = &tegra_i2c_platform[0],
		},
	},
	[1] = {
		.name = "tegra_i2c",
		.id = 1,
		.dev = {
			.platform_data = &tegra_i2c_platform[1],
		},
	},
	[2] = {
		.name = "tegra_i2c",
		.id = 2,
		.dev = {
			.platform_data = &tegra_i2c_platform[2],
		},
	},
	[3] = {
		.name = "tegra_i2c",
		.id = 3,
		.dev = {
			.platform_data = &tegra_i2c_platform[3],
		},
	},
};
static noinline void __init tegra_setup_i2c(void)
{
	const NvOdmPeripheralConnectivity *smbus;
	const NvOdmIoAddress *smbus_addr = NULL;
	const NvU32 *odm_mux_i2c = NULL;
	const NvU32 *odm_clk_i2c = NULL;
	const NvU32 *odm_mux_i2cp = NULL;
	const NvU32 *odm_clk_i2cp = NULL;
	NvU32 odm_mux_i2c_nr;
	NvU32 odm_clk_i2c_nr;
	NvU32 odm_mux_i2cp_nr;
	NvU32 odm_clk_i2cp_nr;
	int i;

	smbus = NvOdmPeripheralGetGuid(NV_ODM_GUID('I','2','c','S','m','B','u','s'));

	if (smbus) {
		unsigned int j;
		smbus_addr = smbus->AddressList;
		for (j=0; j<smbus->NumAddress; j++, smbus_addr++) {
			if ((smbus_addr->Interface == NvOdmIoModule_I2c) ||
			    (smbus_addr->Interface == NvOdmIoModule_I2c_Pmu))
				break;
		}
		if (j==smbus->NumAddress)
			smbus_addr = NULL;
	}

	NvOdmQueryPinMux(NvOdmIoModule_I2c, &odm_mux_i2c, &odm_mux_i2c_nr);
	NvOdmQueryPinMux(NvOdmIoModule_I2c_Pmu, &odm_mux_i2cp, &odm_mux_i2cp_nr);
	NvOdmQueryClockLimits(NvOdmIoModule_I2c, &odm_clk_i2c, &odm_clk_i2c_nr);
	NvOdmQueryClockLimits(NvOdmIoModule_I2c_Pmu, &odm_clk_i2cp, &odm_clk_i2cp_nr);

	for (i=0; i<ARRAY_SIZE(tegra_i2c_devices); i++) {

		struct platform_device *dev = &tegra_i2c_devices[i];
		struct tegra_i2c_plat_parms *plat = &tegra_i2c_platform[i];
		NvU32 mux, clk;

		if (smbus_addr) {
			if (smbus_addr->Interface == NvOdmIoModule_I2c &&
			    smbus_addr->Instance == dev->id && !plat->is_dvc) {
				pr_info("%s: skipping %s.%d (SMBUS)\n",
					__func__, dev->name, dev->id);
				continue;
			}
		}

		if (plat->is_dvc) {
			mux = (odm_mux_i2cp_nr) ? odm_mux_i2cp[0] : 0;
			clk = (odm_clk_i2cp_nr) ? odm_clk_i2cp[0] : 100;
		} else if (dev->id < odm_mux_i2c_nr) {
			mux = odm_mux_i2c[dev->id];
			clk = (dev->id < odm_clk_i2c_nr) ? odm_clk_i2c[dev->id] : 100;
		} else {
			mux = 0;
			clk = 0;
		}

		if (!mux)
			continue;

#ifndef CONFIG_TEGRA_ODM_VENTANA
		if (mux == NVODM_QUERY_PINMAP_MULTIPLEXED) {
			pr_err("%s: unable to register %s.%d (multiplexed)\n",
			       __func__, dev->name, dev->id);
			WARN_ON(1);
			continue;
		}
#endif

		if (clk)
			plat->bus_clk[0] = clk*1000;

		if (platform_device_register(dev))
			pr_err("%s: failed to register %s.%d\n",
			       __func__, dev->name, dev->id);
	}
}
#else
static void tegra_setup_i2c(void) { }
#endif

#ifdef CONFIG_W1_MASTER_TEGRA
static struct tegra_w1_platform_data tegra_w1_platform;
static struct platform_device tegra_w1_device = {
	.name = "tegra_w1",
	.id = 0,
	.dev = {
		.platform_data = &tegra_w1_platform,
	},
};
static noinline void __init tegra_setup_w1(void)
{
	const NvU32 *pinmux;
	NvU32 nr_pinmux;

	NvOdmQueryPinMux(NvOdmIoModule_OneWire, &pinmux, &nr_pinmux);
	if (!nr_pinmux || !pinmux[0]) {
		pr_info("%s: no one-wire device\n", __func__);
		return;
	}
	tegra_w1_platform.pinmux = pinmux[0];
	if (platform_device_register(&tegra_w1_device)) {
		pr_err("%s: failed to register %s.%d\n",
		       __func__, tegra_w1_device.name, tegra_w1_device.id);
	}
}
#else
static void tegra_setup_w1(void) { }
#endif

#ifdef CONFIG_TEGRA_PCI
extern void __init tegra_pcie_init(void);
static int tegra_setup_pcie(void)
{
	const struct tegra_pingroup_config *pinmux = NULL;
	const NvU32 *odmpinmux;
	NvU32 nr_configs;
	int nr_pins;

	NvOdmQueryPinMux(NvOdmIoModule_PciExpress, &odmpinmux, &nr_configs);
	if (!odmpinmux || !nr_configs) {
		pr_info("%s: PCIE not supported on platform\n", __func__);
		return 0;
	}
	pinmux = tegra_pinmux_get("tegra_pcie",	odmpinmux[0], &nr_pins);
	tegra_pinmux_config_tristate_table(pinmux, nr_pins, TEGRA_TRI_NORMAL);
	tegra_pcie_init();
	return 0;
}
late_initcall(tegra_setup_pcie);
#endif

static void tegra_system_power_off(void)
{
	struct regulator *regulator = regulator_get(NULL, "soc_main");

	if (!IS_ERR(regulator)) {
		int rc;
		regulator_enable(regulator);
		rc = regulator_disable(regulator);
		pr_err("%s: regulator_disable returned %d\n", __func__, rc);
	} else {
		pr_err("%s: regulator_get returned %ld\n", __func__,
		       PTR_ERR(regulator));
	}
	local_irq_disable();
	while (1) {
		dsb();
		__asm__ ("wfi");
	}
}

static struct tegra_suspend_platform_data tegra_suspend_platform = {
	.cpu_timer = 2000,
};

static void __init tegra_setup_suspend(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	const int wakepad_irq[] = {
		gpio_to_irq(TEGRA_GPIO_PO5), gpio_to_irq(TEGRA_GPIO_PV3),
		gpio_to_irq(TEGRA_GPIO_PL1), gpio_to_irq(TEGRA_GPIO_PB6),
		gpio_to_irq(TEGRA_GPIO_PN7), gpio_to_irq(TEGRA_GPIO_PA0),
		gpio_to_irq(TEGRA_GPIO_PU5), gpio_to_irq(TEGRA_GPIO_PU6),
		gpio_to_irq(TEGRA_GPIO_PC7), gpio_to_irq(TEGRA_GPIO_PS2),
		gpio_to_irq(TEGRA_GPIO_PAA1), gpio_to_irq(TEGRA_GPIO_PW3),
		/* FIXME: USB/SDMMC wake pad interrupt mapping may be wrong */
		gpio_to_irq(TEGRA_GPIO_PW2), INT_SDMMC1,
		gpio_to_irq(TEGRA_GPIO_PV6), gpio_to_irq(TEGRA_GPIO_PJ7),
		INT_RTC, INT_KBC, INT_EXTERNAL_PMU,
		INT_USB, INT_USB3, INT_USB, INT_USB3,
		gpio_to_irq(TEGRA_GPIO_PI5), gpio_to_irq(TEGRA_GPIO_PV2),
		gpio_to_irq(TEGRA_GPIO_PS4), gpio_to_irq(TEGRA_GPIO_PS5),
		INT_SDMMC2, gpio_to_irq(TEGRA_GPIO_PQ6),
		gpio_to_irq(TEGRA_GPIO_PQ7), gpio_to_irq(TEGRA_GPIO_PN2),
	};
#endif
	const NvOdmWakeupPadInfo *w;
	const NvOdmSocPowerStateInfo *lp;
	struct tegra_suspend_platform_data *plat = &tegra_suspend_platform;
	NvOdmPmuProperty pmu;
	NvBool has_pmu;
	NvU32 nr_wake;

	lp = NvOdmQueryLowestSocPowerState();
	w = NvOdmQueryGetWakeupPadTable(&nr_wake);
	has_pmu = NvOdmQueryGetPmuProperty(&pmu);

	if (!has_pmu) {
		pr_info("%s: no PMU property, ignoring all suspend state\n",
			__func__);
		goto do_register;
	}

	if (lp->LowestPowerState==NvOdmSocPowerState_Suspend) {
		plat->dram_suspend = true;
		plat->core_off = false;
	} else if (lp->LowestPowerState==NvOdmSocPowerState_DeepSleep) {
		plat->dram_suspend = true;
		plat->core_off = true;
	}

	if (has_pmu) {
		plat->cpu_timer = pmu.CpuPowerGoodUs;
		plat->cpu_off_timer = pmu.CpuPowerOffUs;
		plat->core_timer = pmu.PowerGoodCount;
		plat->core_off_timer = pmu.PowerOffCount;

		plat->separate_req = !pmu.CombinedPowerReq;
		plat->corereq_high =
			(pmu.CorePowerReqPolarity ==
			 NvOdmCorePowerReqPolarity_High);
		plat->sysclkreq_high =
			(pmu.SysClockReqPolarity ==
			 NvOdmCorePowerReqPolarity_High);
	}

	if (!w || !nr_wake)
		goto do_register;

	plat->wake_enb = 0;
	plat->wake_low = 0;
	plat->wake_high = 0;
	plat->wake_any = 0;

	while (nr_wake--) {
		unsigned int pad = w->WakeupPadNumber;
		if (pad < ARRAY_SIZE(wakepad_irq) && w->enable)
			enable_irq_wake(wakepad_irq[pad]);

		if (w->enable) {
			plat->wake_enb |= (1 << pad);

			if (w->Polarity == NvOdmWakeupPadPolarity_Low)
				plat->wake_low |= (1 << pad);
			else if (w->Polarity == NvOdmWakeupPadPolarity_High)
				plat->wake_high |= (1 << pad);
			else if (w->Polarity == NvOdmWakeupPadPolarity_AnyEdge)
				plat->wake_any |= (1 << pad);
		}
		w++;
	}

do_register:
	tegra_init_suspend(plat);
	tegra_init_idle(plat);
}

static int tegra_reboot_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		/* USB power rail must be enabled during boot */
		NvOdmEnableUsbPhyPowerRail(NV_TRUE);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_reboot_nb = {
	.notifier_call = tegra_reboot_notify,
	.next = NULL,
	.priority = 0
};

static void __init tegra_setup_reboot(void)
{
	int rc = register_reboot_notifier(&tegra_reboot_nb);
	if (rc)
		pr_err("%s: failed to regsiter platform reboot notifier\n",
			__func__);
}

static int __init tegra_setup_data(void)
{
	NvError e = NvSuccess;
	if (!s_hRmGlobal)
		e = NvRmOpenNew(&s_hRmGlobal);
	BUG_ON(e!=NvSuccess);
	platform_add_devices(nvodm_devices, ARRAY_SIZE(nvodm_devices));
	return 0;
}
postcore_initcall(tegra_setup_data);

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)

#define PROCFS_MAX_SIZE             512
#define PROCFS_SYSINFO			"sysinfo"
#define PROCFS_DMISTATUS		"dmi_status"
#define PROCFS_DMIINFO		"dmi_info"
#define PROCFS_ISMEDIAWORK		"isMediaWork"

#define PRO_CMD_HP_IN 			"hp_in"
#define PRO_CMD_HP_OUT 		"hp_out"

static struct proc_dir_entry *proc_sysinfo;
static struct proc_dir_entry *proc_dmistatus;
static struct proc_dir_entry *proc_dmiinfo;
static struct proc_dir_entry *proc_ismediawork;

int  headphone_plug = { 0 };
int  dmi_status = { 0 };
char dmi_info[1024] = { 0 };
int mediawork = { 0 };
/**
 * The buffer used to store character for this module
 *
 */
static char procfs_buffer[PROCFS_MAX_SIZE];

/**
 * The size of the buffer
 *
 */
static unsigned long procfs_buffer_size = 0;

/** 
 * This function is called then the /proc file is read
 *
 */
int 
procfile_sysinfo_read(char *buffer,
              char **buffer_location,
              off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	int len = 0;
	        
	if (offset > 0) {
	        /* we have finished to read, return 0 */
	        ret  = 0;
	} else {
	        /* fill the buffer, return the buffer size */
			len +=sprintf(procfs_buffer+len, "----System Info-----\n");
			len +=sprintf(procfs_buffer+len, "headphone plug :%d\n", headphone_plug);

			procfs_buffer_size = len;
			
			memcpy(buffer, procfs_buffer, procfs_buffer_size);
			ret = procfs_buffer_size;
	}

	return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int procfile_sysinfo_write(struct file *file, const char *buffer, unsigned long count,
                   void *data)
{
	/* get buffer size */
	procfs_buffer_size = count;
	if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
	        procfs_buffer_size = PROCFS_MAX_SIZE;
	}

	/* write data to the buffer */
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
	        return -EFAULT;
	}

	/* headphone plug in/out */
	if ( strncmp( procfs_buffer, PRO_CMD_HP_IN, sizeof(PRO_CMD_HP_IN)-1) == 0 ) {
		headphone_plug = 1;
	}else if ( strncmp( procfs_buffer, PRO_CMD_HP_OUT, sizeof(PRO_CMD_HP_OUT)-1) == 0 ){
		headphone_plug = 0;
	}

	
	return procfs_buffer_size;
}

	
int proc_sysinfo_init(void)
{
	/* /proc/pega/sysinfo */
	proc_sysinfo = create_proc_entry(PROCFS_SYSINFO, 0666, NULL);
	if (proc_sysinfo == NULL) {
	        printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
	                	PROCFS_SYSINFO);
	        return -ENOMEM;
	}
	proc_sysinfo->read_proc  = procfile_sysinfo_read;
	proc_sysinfo->write_proc = procfile_sysinfo_write;
	proc_sysinfo->mode         = S_IFREG |S_IRUGO|S_IWUGO;
	proc_sysinfo->uid            = 0;
	proc_sysinfo->gid            = 0;
	proc_sysinfo->size           = 128;
	printk(KERN_INFO "/proc/%s created\n", PROCFS_SYSINFO);

	return 0;       /* everything is ok */
}

/** 
 * This function is called then the /proc file is read
 *
 */
int 
procfile_dmistatus_read(char *buffer,
              char **buffer_location,
              off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	int len = 0;
	        
	if (offset > 0) {
	        /* we have finished to read, return 0 */
	        ret  = 0;
	} else {
		/* fill the buffer, return the buffer size */
        	ret = sprintf(buffer, "%d", dmi_status);
	}

	return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int procfile_dmistatus_write(struct file *file, const char *buffer, unsigned long count,
                   void *data)
{
    /* get buffer size */
    procfs_buffer_size = count;
    if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
		procfs_buffer_size = PROCFS_MAX_SIZE;
    }

    /* write data to the buffer */
    if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
            return -EFAULT;
    }

	dmi_status = *buffer - '0';
    return procfs_buffer_size;
}

int proc_dmistatus_init(void)
{	
	proc_dmistatus = create_proc_entry(PROCFS_DMISTATUS, 0666, NULL);
	if (proc_dmistatus == NULL) {
	        printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
	                	PROCFS_SYSINFO);
	        return -ENOMEM;
	}
	proc_dmistatus->read_proc  = procfile_dmistatus_read;
	proc_dmistatus->write_proc = procfile_dmistatus_write;
	proc_dmistatus->mode         = S_IFREG |S_IRUGO|S_IWUGO;
	proc_dmistatus->uid            = 0;
	proc_dmistatus->gid            = 0;
	proc_dmistatus->size           = 128;
	printk(KERN_INFO "/proc/%s created\n", PROCFS_DMISTATUS);

	return 0;       /* everything is ok */
}

/** 
 * This function is called then the /proc file is read
 *
 */
int 
procfile_dmiinfo_read(char *buffer,
              char **buffer_location,
              off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	int len = 0;
	        
	if (offset > 0) {
	        /* we have finished to read, return 0 */
	        ret  = 0;
	} else {
		/* fill the buffer, return the buffer size */
        	ret = sprintf(buffer, "%s", dmi_info);
	}

	return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int procfile_dmiinfo_write(struct file *file, const char *buffer, unsigned long count,
                   void *data)
{
    /* get buffer size */
    procfs_buffer_size = count;
    if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
		procfs_buffer_size = PROCFS_MAX_SIZE;
    }

    /* write data to the buffer */
    if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
            return -EFAULT;
    }

    strncpy(dmi_info, buffer, sizeof(dmi_info)-1);
    return procfs_buffer_size;
}

int proc_dmiinfo_init(void)
{	
	proc_dmiinfo = create_proc_entry(PROCFS_DMIINFO, 0666, NULL);
	if (proc_dmiinfo == NULL) {
	        printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
	                	PROCFS_SYSINFO);
	        return -ENOMEM;
	}
	proc_dmiinfo->read_proc  = procfile_dmiinfo_read;
	proc_dmiinfo->write_proc = procfile_dmiinfo_write;
	proc_dmiinfo->mode         = S_IFREG |S_IRUGO|S_IWUGO;
	proc_dmiinfo->uid            = 0;
	proc_dmiinfo->gid            = 0;
	proc_dmiinfo->size           = 128;
	printk(KERN_INFO "/proc/%s created\n", PROCFS_DMIINFO);

	return 0;       /* everything is ok */
}


int proc_dmicache_init(void)
{
	proc_dmistatus_init();
	proc_dmiinfo_init();

	return 0;       /* everything is ok */
}

procfile_ismediawork_read(char *buffer,
              char **buffer_location,
              off_t offset, int buffer_length, int *eof, void *data)
{
    int ret;
    int len = 0;

	if (offset > 0) {
        ret  = 0;
    } else {
        /* fill the buffer, return the buffer size */
        ret = sprintf(buffer, "%d", mediawork);
    }

    return ret;
}

int procfile_ismediawork_write(struct file *file, const char *buffer, unsigned long count,
                   void *data)
{
    /* get buffer size */
    procfs_buffer_size = count;
    if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
		procfs_buffer_size = PROCFS_MAX_SIZE;
    }

    /* write data to the buffer */
    if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
            return -EFAULT;
    }

	mediawork = *buffer - '0';
    return procfs_buffer_size;
}

int proc_ismediawork_init(void)
{
	proc_ismediawork = create_proc_entry(PROCFS_ISMEDIAWORK, 0666, NULL);
	if (proc_ismediawork == NULL) {
	        printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
	                	PROCFS_ISMEDIAWORK);
	        return -ENOMEM;
	}
	proc_ismediawork->read_proc  = procfile_ismediawork_read;
	proc_ismediawork->write_proc = procfile_ismediawork_write;
	proc_ismediawork->mode         = S_IFREG |S_IRUGO|S_IWUGO;
	proc_ismediawork->uid            = 0;
	proc_ismediawork->gid            = 0;
	proc_ismediawork->size           = 128;
	printk(KERN_INFO "/proc/%s created\n", PROCFS_ISMEDIAWORK);

	return 0;       /* everything is ok */
}
#endif

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
static int  TB2_PCB_BOARD_ID = 0;	
int tegra_board_nvodm_board_id(void)
{
    return TB2_PCB_BOARD_ID;
}

/*
    Test  for PCB ID
*/
static void __init tegra_config_pcb_id(void)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    static NvOdmGpioPinHandle s_hPCBPin1 = NULL;
    static NvOdmGpioPinHandle s_hPCBPin2 = NULL;
    static NvOdmServicesGpioHandle s_hGpio = NULL;
    int pinValue1,pinValue2;
    pConnectivity = (NvOdmPeripheralConnectivity *)NvOdmPeripheralGetGuid(NV_ODM_GUID('b','o','a','r','d','i','d',' '));
    if (!pConnectivity)
    {
       pr_err("ERROR Can not get pConnectivity\n");
       goto ErrorExit;
    }
    
    s_hGpio = NvOdmGpioOpen();
        
     if (!s_hGpio)
     {
        pr_err("ERROR configCameraGpio:s_hGpio=null, can not get NvOdmServicesGpioHandle\n");	
        goto ErrorExit;
     }
     
     s_hPCBPin1 = NvOdmGpioAcquirePinHandle(s_hGpio, 
        						pConnectivity->AddressList[0].Instance,
							pConnectivity->AddressList[0].Address);              

     NvOdmGpioConfig(s_hGpio,s_hPCBPin1, NvOdmGpioPinMode_InputData);  

     s_hPCBPin2 = NvOdmGpioAcquirePinHandle(s_hGpio, 
        						pConnectivity->AddressList[1].Instance,
							pConnectivity->AddressList[1].Address);              
     
     NvOdmGpioConfig(s_hGpio,s_hPCBPin2, NvOdmGpioPinMode_InputData);    
     
     NvOdmGpioGetState(s_hGpio,s_hPCBPin1, &pinValue1);
     NvOdmGpioGetState(s_hGpio,s_hPCBPin2, &pinValue2);
     TB2_PCB_BOARD_ID |= (pinValue1 <<1);
     TB2_PCB_BOARD_ID |=  pinValue2;
     pr_err(">>>>>>>>TB2 PCB Board ID %1d.%1d TB2_PCB_BOARD_ID = %d <<<<<<<<<<<\n",pinValue1,pinValue2, TB2_PCB_BOARD_ID);	
     NvOdmGpioReleasePinHandle(s_hGpio, s_hPCBPin1);
     NvOdmGpioReleasePinHandle(s_hGpio, s_hPCBPin2);     
     NvOdmGpioClose(s_hGpio);        
ErrorExit:
    return;
}

/*
 *   DESCRIPTION:
 *         Set camera's configuation including 'power/level' and 'power/autosuspend' and try to show their inforamtion
 *
 *   VERSION:
 *         2010/09/23 -- crated by doyle
 */

// be revised from 'drivers\usb\core\sysfs.c' [ show_level ]
// "struct device *dev --> struct usb_device *udev" : fixed by doyle , 2010/09/24
//
static enum udev_pm_level
udev_get_level(struct usb_device *udev)
{
	enum udev_pm_level p = udev_level_auto;

	if (udev->state == USB_STATE_SUSPENDED) {
		if (udev->autoresume_disabled)
			p = udev_level_suspend;
	} else {
		if (udev->autosuspend_disabled)
			p = udev_level_on;
	}

	return p;
}

// be revised from 'drivers\usb\core\sysfs.c' [ set_level ]
// "struct device *dev --> struct usb_device *udev" : fixed by doyle , 2010/09/24
//
static int
udev_set_level(struct usb_device *udev,
	enum udev_pm_level p)
{
	int rc = 0;
	int old_autosuspend_disabled, old_autoresume_disabled;

	usb_lock_device(udev);

	old_autosuspend_disabled = udev->autosuspend_disabled;
	old_autoresume_disabled = udev->autoresume_disabled;

	/* Setting the flags without calling usb_pm_lock is a subject to
	 * races, but who cares...
	 */
	if (p==udev_level_on) {
		udev->autosuspend_disabled = 1;
		udev->autoresume_disabled = 0;
		rc = usb_external_resume_device(udev, PMSG_USER_RESUME);

	} else if (p==udev_level_auto) {
		udev->autosuspend_disabled = 0;
		udev->autoresume_disabled = 0;
		rc = usb_external_resume_device(udev, PMSG_USER_RESUME);

	} else if (p==udev_level_suspend) {
		udev->autosuspend_disabled = 0;
		udev->autoresume_disabled = 1;
		rc = usb_external_suspend_device(udev, PMSG_USER_SUSPEND);

	} else
		rc = -EINVAL;

	if (rc) {
		udev->autosuspend_disabled = old_autosuspend_disabled;
		udev->autoresume_disabled = old_autoresume_disabled;
	}
	usb_unlock_device(udev);

	return rc;
}

// be revised from 'drivers\usb\core\sysfs.c' [ get_autosuspend ]
// "struct device *dev --> struct usb_device *udev" : fixed by doyle , 2010/09/24
//
static int
udev_get_autosuspend(struct usb_device *udev)
{
	return ( udev->autosuspend_delay / HZ );
}

// be revised from 'drivers\usb\core\sysfs.c' [ set_autosuspend ]
// "struct device *dev --> struct usb_device *udev" : fixed by doyle , 2010/09/24
//
static int
udev_set_autosuspend(struct usb_device *udev,
		int value)
{
	if ( value >= INT_MAX/HZ ||	value <= - INT_MAX/HZ)
		return -EINVAL;
	value *= HZ;

	udev->autosuspend_delay = value;
	if (value >= 0)
		usb_try_autosuspend_device(udev);
	else {
		if (usb_autoresume_device(udev) == 0)
			usb_autosuspend_device(udev);
	}

	return 0;
}

//Get usb device of camera and usb(hold controller)
//Input parameter: (1) const platform device ; (2) struct usb_device pointer of camera ; (3) struct usb_device pointer of usb(hold controller) ; 
//Output: void
//Creater: 2010/09/23 doyle
//
static void
CamGetUsbDevice(struct platform_device const *pdev ,struct usb_device **rhdev_camera , struct usb_device **rhdev_usb  )
{
	//Get hcd
	struct usb_hcd *hcd=(struct usb_hcd *) dev_get_drvdata( &pdev->dev );
	//Get rhdev
	*rhdev_usb=hcd->self.root_hub;
	*rhdev_camera=(*rhdev_usb)->children[0];
}

//As suspend or display false  happen , set camera's config including camera's level , camera usb's level ,and camera usb's autosuspend 
//Input parameter: (1) struct usb_device pointer of camera ; (2) struct usb_device pointer of usb(hold controller) ; 
//Output: void
//Creater: 2010/09/23 doyle
//
static void
CamSetSuspendConfig(struct usb_device *rhdev_usb ,  struct usb_device *rhdev_camera){

	enum udev_pm_level tmp_config;
	int tmp_auto_config;

	// camera's level set ,as suspend or display false  happen :
	//    (1) if current camera's level is not 'suspend' , the camera's level will set as  'suspend' , and record its config value
	//    (2) otherwise , it do nothing
	//
	tmp_config=udev_get_level( rhdev_camera);
	if ( tmp_config != udev_level_suspend)
	{
		udev_set_level( rhdev_camera , udev_level_suspend);
		s_camera_level=tmp_config;
	}

	// camera usb's level set ,as suspend or display false  happen :
	//    (1) if current camera usb's level is not 'suspend' , the camera usb's level will set as  'suspend' , and record its config value
	//    (2) otherwise , it do nothing
	//
	tmp_config=udev_get_level( rhdev_usb );
	if ( tmp_config != udev_level_suspend )
	{
		udev_set_level( rhdev_usb , udev_level_suspend);
		s_camera_usb_level=tmp_config;
	}

	// camera usb's autosuspend set ,as suspend or display false  happen :
	//    (1) if current camera usb's autosuspend is not '-1' , the camera usb's level will set as  '-1' , and record its config value
	//    (2) otherwise , it do nothing
	//
	tmp_auto_config=udev_get_autosuspend( rhdev_usb );
	if ( tmp_auto_config != -1 )
	{
		udev_set_autosuspend( rhdev_usb , tmp_auto_config);
		s_camera_usb_autosuspend=tmp_auto_config;
	}
}

//As resume  happen , set camera's config including camera's level , camera usb's level ,and camera usb's autosuspend 
//Input parameter: (1) struct usb_device pointer of camera ; (2) struct usb_device pointer of usb(hold controller) ; 
//Output: void
//Creater: 2010/09/23 doyle
//
static void
CamSetResumeConfig(struct usb_device *rhdev_usb ,  struct usb_device *rhdev_camera){

	enum udev_pm_level tmp_config;
	int tmp_auto_config;

	// camera usb's level set ,as resume  happen :
	//    (1) if current camera usb's level is 'suspend' , then:
	//            (1-1) the camera usb's level will set as  pre-value
	//            (1-2) inspect and set the camera's level
	//    (2) otherwise , it do nothing
	//
	tmp_config=udev_get_level( rhdev_usb );
	if ( tmp_config == udev_level_suspend )
	{
		udev_set_level( rhdev_usb , s_camera_usb_level);

		// camera's level set ,as resume  happen :
		//    (1) if current camera's level is 'suspend' , the camera's level will set as  pre-value
		//    (2) otherwise , it do nothing
		//
		tmp_config=udev_get_level( rhdev_camera );
		if ( tmp_config == udev_level_suspend )
		{
			udev_set_level( rhdev_camera , s_camera_level );
		}
	}

	// camera usb's level set ,as resume  happen :
	//    (1) if current camera usb's level is 'suspend' , the camera usb's level will set as  pre-value
	//    (2) otherwise , it do nothing
	//
	tmp_auto_config=udev_get_autosuspend( rhdev_usb);
	if ( tmp_auto_config == -1 )
	{
		udev_set_autosuspend( rhdev_usb , s_camera_usb_autosuspend);
	}
}

static void CamSuspend(struct early_suspend *h)
{
	struct usb_device *rhdev_usb = NULL;
	struct usb_device *rhdev_camera = NULL;

	CamGetUsbDevice( &tegra_hcd[1] , &rhdev_camera , &rhdev_usb );

	if (rhdev_usb && rhdev_camera)
	{
		CamSetSuspendConfig(rhdev_usb , rhdev_camera);
		NvOdmServicesPmuSetVoltage(s_hPmuServices, Ext_SWITCHPmuSupply_CamPwdn, NVODM_VOLTAGE_OFF, NULL);
	}
}

static void CamResume(struct early_suspend *h)
{
	struct usb_device *rhdev_usb = NULL;
	struct usb_device *rhdev_camera = NULL;

	NvOdmServicesPmuSetVoltage(s_hPmuServices, Ext_SWITCHPmuSupply_CamPwdn, 3300, NULL);

	CamGetUsbDevice( &tegra_hcd[1] , &rhdev_camera , &rhdev_usb );
	if (rhdev_usb && rhdev_camera)
	CamSetResumeConfig(rhdev_usb , rhdev_camera);
}

static struct early_suspend CamEarlySuspend = {
    .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
    .suspend = CamSuspend,
    .resume = CamResume,
};
#endif

#ifdef CONFIG_TEGRA_TMON_PROC /* temperature sensor */
NvU32 gTemperatureC = 0;
NvU32 localTemperatureC = 0;

static DEFINE_MUTEX(tmon_list_lock);
static int tegra_tmon_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	
	mutex_lock(&tmon_list_lock);
	len += snprintf(page+len, count-len,"%d", gTemperatureC);
	mutex_unlock(&tmon_list_lock);
	*eof = 1;

	return len;
}

static int tegra_localtmon_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	mutex_lock(&tmon_list_lock);
	len += snprintf(page+len, count-len,"%d", localTemperatureC);
	mutex_unlock(&tmon_list_lock);

	*eof = 1;
	return len;
}

static DEFINE_MUTEX(pcbid_list_lock);
static int tegra_pcbid_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	mutex_lock(&pcbid_list_lock);
	len += snprintf(page+len, count-len,"%d", tegra_board_nvodm_board_id());
	mutex_unlock(&pcbid_list_lock);

	*eof = 1;
	return len;
}
#endif 

#if defined(CONFIG_TEGRA_ODM_USB_HUB_INTR)	
/*
    USB HUB SUSPEND PIN handle for hotplug
*/
static NvOdmServicesGpioHandle s_hUsbHubGpio = NULL;
static NvOdmGpioPinHandle s_hUsbHubPin = NULL;
static NvOdmServicesGpioIntrHandle s_hUsbHubIntrHandle = NULL;


/* get GPIO_PW2 status  */
static void tegra_usb_hub_hotplug_intr_hdr(void *arg)
{
	NvU32 pinState = 0;
	int ret = -1;

	ret = send_usb_hub_uevent_sock();
	if(ret != 0) {
		pr_err("ERROR send_usb_hub_uevent_sock failed\n");
	}
	
	NvOdmGpioInterruptDone(s_hUsbHubIntrHandle);
}

static void __init tegra_config_usb_hub_hotplug(void)
{
	const NvOdmPeripheralConnectivity *pConnectivity = NULL;	
	NvOdmInterruptHandler IntrHandler = (NvOdmInterruptHandler)tegra_usb_hub_hotplug_intr_hdr;

	pConnectivity = (NvOdmPeripheralConnectivity *)NvOdmPeripheralGetGuid(NV_ODM_GUID('u','s','b','h','u','b','s','u'));
	if (!pConnectivity)
	{
		pr_err("ERROR Can not get pConnectivity\n");
		goto Exit;
	}

	s_hUsbHubGpio = NvOdmGpioOpen();

	if (!s_hUsbHubGpio)
	{
		pr_err("ERROR tegra_config_usb_hub_hotplug: NvOdmGpioOpen fail\n");	
		goto Exit;
	}

	s_hUsbHubPin = NvOdmGpioAcquirePinHandle(s_hUsbHubGpio, 
										pConnectivity->AddressList[0].Instance,
										pConnectivity->AddressList[0].Address);
	
	NvOdmGpioConfig(s_hUsbHubGpio, s_hUsbHubPin, NvOdmGpioPinMode_InputData);  

	/* register interrupt handler for GPIO_PW2 status */
	if (NvOdmGpioInterruptRegister(s_hUsbHubGpio, &s_hUsbHubIntrHandle,
									 s_hUsbHubPin, NvOdmGpioPinMode_InputInterruptFallingEdge,
									 IntrHandler, (void *)NULL, 0) == NV_FALSE)
	{
		NvOsDebugPrintf (("ERROR tegra_config_usb_hub_hotplug: NvOdmGpioInterruptRegister"));
	}

Exit:
    return;
}
#endif


#if 0
/* paul porting for 9.12.13 */
static void __init tegra_configCameraGpio(void)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    static NvOdmGpioPinHandle s_hCarmeraPin = NULL;
    static NvOdmServicesGpioHandle s_hGpio = NULL;

    pConnectivity = (NvOdmPeripheralConnectivity *)NvOdmPeripheralGetGuid(NV_ODM_GUID('c','a','m','e','r','a',' ',' '));

    s_hGpio = NvOdmGpioOpen();
    if (!s_hGpio)
    {
        pr_err("ERROR configCameraGpio:s_hGpio=null, can not get NvOdmServicesGpioHandle\n");	
        goto ErrorExit;
    }

    if (!s_hCarmeraPin)
        s_hCarmeraPin = NvOdmGpioAcquirePinHandle(s_hGpio, 
        						pConnectivity->AddressList[0].Instance,
							pConnectivity->AddressList[0].Address);

    if (!s_hCarmeraPin)
    {
        NvOdmGpioClose(s_hGpio);
        s_hGpio = NULL;
	 pr_err("ERROR configCameraGpio: ERROR NvOdmGpioAcquirePinHandle: Not able to Acq pinhandle\n");	
    }else{
        NvOdmGpioConfig(s_hGpio,s_hCarmeraPin, NvOdmGpioPinMode_Output);
        NvOdmGpioSetState(s_hGpio, s_hCarmeraPin, 0x0);
	 //pr_err("configCameraGpio: config GPIO_PV4 as output pin\n");		
    }
ErrorExit:
    return;
}
#endif
void __init tegra_setup_nvodm(bool standard_i2c, bool standard_spi)
{
#if	defined(CONFIG_TOUCHSCREEN_PANJIT_I2C) && \
	defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	NvOdmBoardInfo	BoardInfo;
#endif

	NvRmGpioOpen(s_hRmGlobal, &s_hGpioGlobal);
	tegra_setup_debug_uart();
	tegra_setup_hcd();
	tegra_setup_hsuart();
	tegra_setup_sdhci();
	tegra_setup_rfkill();
	tegra_setup_kbc();
#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)	
    tegra_config_pcb_id();    /* TB2 PCB ID */
//	tegra_configCameraGpio(); /*paul : porting for 9.12.13 */
#endif
#if defined(CONFIG_TEGRA_ODM_USB_HUB_INTR)	
	tegra_config_usb_hub_hotplug();
#endif
	tegra_setup_gpio_key();
	if (standard_i2c)
		tegra_setup_i2c();
	if (standard_spi)
		tegra_setup_spi();
#if	defined(CONFIG_TOUCHSCREEN_PANJIT_I2C) && \
	defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
#define NVODM_ATMEL_TOUCHSCREEN    0x0A00
#define NVODM_PANJIT_TOUCHSCREEN   0x0000
#define BOARD_VENTANA              0x024B
	if (NvOdmPeripheralGetBoardInfo(BOARD_VENTANA, &BoardInfo)) {
		/* Diagnostics print messages to print Board SKU
		   printk("\n\nRRC:  Board ID:  %04X\n\n", BoardInfo.SKU);
		 */
		switch (BoardInfo.SKU & 0xFF00) {
		case NVODM_ATMEL_TOUCHSCREEN:
			ventana_touch_init_atmel();
			break;

		default:
			ventana_touch_init_panjit();
			break;
		}
	} else
		ventana_touch_init_panjit();
#elif defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	ventana_touch_init_atmel();
#elif defined(CONFIG_TOUCHSCREEN_PANJIT_I2C)
	ventana_touch_init_panjit();
#endif
	tegra_setup_w1();
	pm_power_off = tegra_system_power_off;
	tegra_setup_suspend();
	tegra_setup_reboot();
#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)	
	proc_sysinfo_init();
	proc_dmicache_init();
	proc_ismediawork_init();
	s_hPmuServices = NvOdmServicesPmuOpen();
        NvOdmServicesPmuSetVoltage(s_hPmuServices, Ext_SWITCHPmuSupply_VSleep, 3300, NULL);
	NvOdmServicesPmuSetVoltage(s_hPmuServices, Ext_SWITCHPmuSupply_CamPwdn, 3300, NULL);
	register_early_suspend(&CamEarlySuspend);
#endif
#ifdef CONFIG_TEGRA_TMON_PROC 
	create_proc_read_entry("tmoninfo", S_IRUGO, NULL, tegra_tmon_read_proc, NULL);
	create_proc_read_entry("localtmon", S_IRUGO, NULL, tegra_localtmon_read_proc, NULL);
	create_proc_read_entry("pcbid", S_IRUGO, NULL, tegra_pcbid_read_proc, NULL);
#endif
}

void tegra_board_nvodm_suspend(void)
{
	if (console_suspend_enabled)
		tegra_debug_port_suspend();
#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
        NvOdmServicesPmuSetVoltage(s_hPmuServices, Ext_SWITCHPmuSupply_VSleep, NVODM_VOLTAGE_OFF, NULL);
#endif
}

void tegra_board_nvodm_resume(void)
{
#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
        NvOdmServicesPmuSetVoltage(s_hPmuServices, Ext_SWITCHPmuSupply_VSleep, 3300, NULL);
#endif
	if (console_suspend_enabled)
		tegra_debug_port_resume();
}
