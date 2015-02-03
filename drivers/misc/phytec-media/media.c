/*	--*- c -*--
 * Copyright (C) 2014 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 and/or (at your option) version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>

/* HACK: include for cpu_is_imx6dl() */
#include <../mach-imx/hardware.h>

#include <media/imx.h>

struct phytec_media_port {
	unsigned int		idx;
	struct gpio_desc	*gpio_enable;
	struct gpio_desc	*gpio_npwrdn;
	struct clk_hw		clk_hw;
	struct clk		*clk;
	bool			is_lvds;
};

struct phytec_media_system {
	struct device			*dev;
	struct phytec_media_port	port[2];
};

static struct clk_ops const		phytec_media_clk_ops = {
};

static struct clk *phytec_media_get_clk(struct of_phandle_args *args,
					void *media_)
{
	unsigned int			idx = args->args[0];
	struct phytec_media_system	*media = media_;

	if (idx >= ARRAY_SIZE(media->port) || !media->port[idx].clk) {
		dev_warn(media->dev, "bad clk index %u\n", idx);
		return ERR_PTR(-EINVAL);
	}

	return media->port[idx].clk;
}

static int phytec_media_init_port(struct phytec_media_system *media,
				  unsigned char idx)
{
	struct phytec_media_port	*port = &media->port[idx];
	char				*name;

	char		mode_name[sizeof "phytec,camX_serial" + 4];
	char		clk_name[sizeof "cameraX-clk" + 4];
	struct clk	*clk_cam = NULL;
	int		rc;

	char const	*clk_parents[1];
	struct clk_init_data	clk_init = {
		.ops		= &phytec_media_clk_ops,
		.num_parents	= 1,
		.parent_names	= clk_parents,
		.flags		= CLK_IS_BASIC | CLK_SET_RATE_PARENT,
	};

	/* check whether we are LVDS or parallel */
	sprintf(mode_name, "phytec,cam%u_serial", idx);
	port->is_lvds = of_property_read_bool(media->dev->of_node, mode_name);

	/* get and setup data-en gpio */
	name = devm_kasprintf(media->dev, GFP_KERNEL, "phytec,cam%u_data_en",
			      idx);
	if (!name)
		return -ENOMEM;

	port->gpio_enable = devm_gpiod_get(
		media->dev, name,
		port->is_lvds ? GPIOD_OUT_LOW : GPIOD_OUT_HIGH);

	rc = PTR_ERR_OR_ZERO(port->gpio_enable);
	if (rc == -EPROBE_DEFER)
		return rc;
	else if (rc < 0)
		/* ignore errors for now... */
		port->gpio_enable = NULL;

	/* get and setup npwrdn gpio */
	name = devm_kasprintf(media->dev, GFP_KERNEL, "phytec,cam%u_npwrdn",
			      idx);
	if (!name)
		return -ENOMEM;

	port->gpio_npwrdn = devm_gpiod_get(
		media->dev, name,
		port->is_lvds ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);

	rc = PTR_ERR_OR_ZERO(port->gpio_npwrdn);
	if (rc == -EPROBE_DEFER)
		return rc;
	else if (rc < 0)
		/* ignore errors for now... */
		port->gpio_npwrdn = NULL;

	port->idx = idx;

	/* try to get the base clock */
	sprintf(clk_name, "camera%u-clk", idx);
	clk_cam = clk_get(media->dev, clk_name);
	rc = PTR_ERR_OR_ZERO(clk_cam);
	if (rc == -EPROBE_DEFER) {
		goto out;
	} else if (rc == -ENOENT) {
		dev_dbg(media->dev, "skipping setup of %s clock\n", clk_name);
	} else if (rc < 0) {
		dev_err(media->dev, "failed to get %s clock: %d\n",
			clk_name, rc);
		goto out;
	}

	/* when we got the base clock, register a clk provider */
	if (clk_cam) {
		clk_init.name = clk_name;
		clk_parents[0] = __clk_get_name(clk_cam);

		port->clk_hw.init = &clk_init;

		port->clk = devm_clk_register(media->dev, &port->clk_hw);
		rc = PTR_ERR_OR_ZERO(port->clk);

		if (rc) {
			dev_err(media->dev, "failed to register %s clock: %d\n",
				clk_init.name, rc);
			goto out;
		}

		dev_dbg(media->dev, "registered clock %s\n", clk_init.name);
		port->clk_hw.init = NULL;
	}

	/* when debugging is enabled; export the gpios into the sysfs */
	if (IS_ENABLED(DEBUG) && port->gpio_enable)
		gpiod_export(port->gpio_enable, false);

	if (IS_ENABLED(DEBUG) && port->gpio_npwrdn)
		gpiod_export(port->gpio_npwrdn, false);

	dev_info(media->dev, "initialized port #%u (%s), GPIOs %d + %d\n", idx,
		 port->is_lvds ? "LVDS" : "parallel",
		 port->gpio_enable ? desc_to_gpio(port->gpio_enable) : -1,
		 port->gpio_npwrdn ? desc_to_gpio(port->gpio_npwrdn) : -1);

	rc = 0;

out:
	if (!IS_ERR_OR_NULL(clk_cam))
		clk_put(clk_cam);

	return rc;
}

static int phytec_media_drv_probe(struct platform_device *pdev)
{
	struct phytec_media_system	*media;
	int				rc;
	struct regmap			*gpr;
	size_t				i;
	bool				have_provider = false;

	media = devm_kzalloc(&pdev->dev, sizeof *media, GFP_KERNEL);
	if (!media)
		return -ENOMEM;

	media->dev = &pdev->dev;

	/* iterate through ports */
	rc = 0;
	for (i = 0; i < ARRAY_SIZE(media->port) && !rc; ++i)
		rc = phytec_media_init_port(media, i);

	if (rc) {
		dev_err(&pdev->dev, "failed to initialize port %zu: %d\n",
			i, rc);
		goto out;
	}

	/* allow our clk provider to be addressed in the device tree */
	rc = of_clk_add_provider(pdev->dev.of_node, phytec_media_get_clk,
				 media);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to add clock provider: %d\n", rc);
		goto out;
	}
	have_provider = true;

	/* create devices from child nodes */
	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to populate dtree: %d\n", rc);
		goto out;
	}

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(gpr)) {
		dev_warn(&pdev->dev, "failed to get IOMUX GPR registers\n");
	} else if (cpu_is_imx6dl()) {
		/* enable CSIx MUX */
                regmap_update_bits(gpr, IOMUXC_GPR13, 0x3f,
				   (4 << 3) | (4 << 0));
	} else if (cpu_is_imx6q()) {
		/* TODO */
                regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_MIPI_IPU1_MUX_MASK |
				   IMX6Q_GPR1_MIPI_IPU2_MUX_MASK,
				   IMX6Q_GPR1_MIPI_IPU1_MUX_IOMUX |
				   IMX6Q_GPR1_MIPI_IPU2_MUX_IOMUX);
                regmap_update_bits(gpr, IOMUXC_GPR3,
				   IMX6Q_GPR3_IPU_DIAG_MASK, 0);

	} else {
		WARN_ON(1);
	}

	rc = ipu_media_device_register(&pdev->dev);
	if (rc < 0) {
		dev_err(&pdev->dev, "ipu_media_device_register() failed: %d\n",
			rc);
		goto out;
	}

	rc = 0;

out:
	if (rc < 0) {
		if (have_provider)
			of_clk_del_provider(pdev->dev.of_node);
	}

	return rc;
}

static int phytec_media_drv_remove(struct platform_device *pdev)
{
	int		rc;

	of_clk_del_provider(pdev->dev.of_node);
	rc = ipu_media_device_unregister(&pdev->dev);
	WARN_ON(rc < 0);

	return rc;
}

static const struct of_device_id phytec_media_dt_ids[] = {
	{ .compatible = "phytec,media-bus", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phytec_media_dt_ids);

static struct platform_device_id phytec_media_id_table[] = {
	{ "phytec-media-bus", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, phytec_media_id_table);

static struct platform_driver	phytec_media_driver = {
	.probe	= phytec_media_drv_probe,
	.remove	= phytec_media_drv_remove,
	.id_table	= phytec_media_id_table,
	.driver		= {
		.of_match_table = phytec_media_dt_ids,
		.owner	= THIS_MODULE,
		.name	= "phytec-media-bus",
	},
};
module_platform_driver(phytec_media_driver);

MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_LICENSE("GPL");
