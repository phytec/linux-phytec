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
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>

/* HACK: include for cpu_is_imx6dl() */
#include <../mach-imx/hardware.h>

#include <media/imx.h>

struct phytec_media_system {
	struct device		*dev;
	struct gpio_desc	*gpio_enable[2];
	struct gpio_desc	*gpio_npwrdn[2];
};

static int phytec_media_drv_probe(struct platform_device *pdev)
{
	struct phytec_media_system	*media;
	int				rc;
	struct regmap			*gpr;

	media = devm_kzalloc(&pdev->dev, sizeof *media, GFP_KERNEL);
	if (!media)
		return -ENOMEM;

	media->gpio_enable[0] = devm_gpiod_get(&pdev->dev,
					       "phytec,cam0_data_en",
					       GPIOD_OUT_HIGH);
	media->gpio_enable[1] = devm_gpiod_get(&pdev->dev,
					       "phytec,cam1_data_en",
					       GPIOD_OUT_HIGH);
	media->gpio_npwrdn[0] = devm_gpiod_get(&pdev->dev,
					       "phytec,cam0_npwrdn",
					       GPIOD_OUT_LOW);
	media->gpio_npwrdn[1] = devm_gpiod_get(&pdev->dev,
					       "phytec,cam1_npwrdn",
					       GPIOD_OUT_LOW);

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
	return rc;
}

static int phytec_media_drv_remove(struct platform_device *pdev)
{
	int		rc;

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
