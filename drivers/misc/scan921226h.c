// SPDX-License-Identifier: GPL-2.0+
/*
 * TI SCAN921226H Video Deserialzer Driver
 *
 * Copyright (C) 2021 PHYTEC Messtechnik GmbH
 * Author: Stefan Riedmueller <s.riedmueller@phytec.de>
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/sysfs.h>

struct video_des {
	struct gpio_desc *pdown_gpio;
	struct gpio_desc *enable_gpio;
	struct mutex lock;
};

static void video_des_enable(struct video_des *vdes, bool enable)
{
	if (enable) {
		gpiod_set_value_cansleep(vdes->enable_gpio, 1);
		udelay(10);
		gpiod_set_value_cansleep(vdes->pdown_gpio, 1);
	} else {
		gpiod_set_value_cansleep(vdes->enable_gpio, 0);
		gpiod_set_value_cansleep(vdes->pdown_gpio, 0);
	}
};

static ssize_t enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct video_des *vdes = dev_get_drvdata(dev);
	int enabled = 0;

	mutex_lock(&vdes->lock);

	if (gpiod_get_value_cansleep(vdes->enable_gpio) &&
	    gpiod_get_value_cansleep(vdes->pdown_gpio))
		enabled = 1;

	mutex_unlock(&vdes->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct video_des *vdes = dev_get_drvdata(dev);
	bool enable;

	if (sysfs_streq(buf, "1"))
		enable = true;
	else if (sysfs_streq(buf, "0"))
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&vdes->lock);
	video_des_enable(vdes, enable);
	mutex_unlock(&vdes->lock);

	return count;
}

static DEVICE_ATTR_RW(enable);

static int video_des_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct video_des *vdes;
	int ret;

	vdes = devm_kzalloc(dev, sizeof(*vdes), GFP_KERNEL);
	if (!vdes)
		return -ENOMEM;

	platform_set_drvdata(pdev, vdes);

	vdes->pdown_gpio = devm_gpiod_get(dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR(vdes->pdown_gpio)) {
		ret = PTR_ERR(vdes->pdown_gpio);
		dev_err(dev, "Failed to get Powerdown GPIO (%d)\n", ret);
		return ret;
	}
	vdes->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(vdes->enable_gpio)) {
		ret = PTR_ERR(vdes->enable_gpio);
		dev_err(dev, "Failed to get Enable GPIO (%d)\n", ret);
		return ret;
	}

	if (of_property_read_bool(np, "ti,boot-on"))
		video_des_enable(vdes, true);

	mutex_init(&vdes->lock);
	ret = device_create_file(dev, &dev_attr_enable);
	return ret;
}

static int video_des_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_enable);
	return 0;
}

static const struct of_device_id video_des_dt_ids[] = {
	{ .compatible = "ti,scan921226h", .data = NULL },
	{ },
};
MODULE_DEVICE_TABLE(of, video_des_dt_ids);

static struct platform_driver video_des_driver = {
	.probe		= video_des_probe,
	.remove		= video_des_remove,
	.driver		= {
		.name	= "scan921226h",
		.of_match_table = of_match_ptr(video_des_dt_ids),
	},
};

module_platform_driver(video_des_driver);

MODULE_DESCRIPTION("SCAN921226H Video Deserializer");
MODULE_AUTHOR("Stefan Riedmueller <s.riedmueller@phytec.de>");
MODULE_LICENSE("GPL");
