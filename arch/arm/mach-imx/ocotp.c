/*
 * Copyright (C) 2016 PHYTEC Messtechnik GmbH,
 * Author: Stefan Christ <s.christ@phytec.de>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/system_info.h>

#include "common.h"

#define OCOTP_CFG0			0x410
#define OCOTP_CFG1			0x420

void __init imx_init_serial_from_ocotp(const char* ocotp_compat)
{
	struct device_node *np;
	void __iomem *base;

	np = of_find_compatible_node(NULL, NULL, ocotp_compat);
	if (!np) {
		pr_warn("failed to find ocotp node in dtb!\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	system_serial_high = readl_relaxed(base + OCOTP_CFG0);
	system_serial_low  = readl_relaxed(base + OCOTP_CFG1);

	iounmap(base);
put_node:
	of_node_put(np);
}
