/*
 * da9062_wdt.c - WDT device driver for DA9062
 * Copyright (C) 2015  Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/mfd/da9062/registers.h>
#include <linux/mfd/da9062/core.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/of.h>

static const unsigned int wdt_timeout[] = { 0, 2, 4, 8, 16, 32, 65, 131 };
#define DA9062_TWDSCALE_DISABLE		0
#define DA9062_TWDSCALE_MIN		1
#define DA9062_TWDSCALE_MAX		(ARRAY_SIZE(wdt_timeout) - 1)
#define DA9062_WDT_MIN_TIMEOUT		wdt_timeout[DA9062_TWDSCALE_MIN]
#define DA9062_WDT_MAX_TIMEOUT		wdt_timeout[DA9062_TWDSCALE_MAX]
#define DA9062_WDG_DEFAULT_TIMEOUT	wdt_timeout[DA9062_TWDSCALE_MAX-1]
#define DA9062_RESET_PROTECTION_MS	300

struct da9062_watchdog {
	struct da9062 *hw;
	struct watchdog_device wdtdev;
	unsigned long j_time_stamp;
	struct delayed_work ping_work;
	struct notifier_block restart_handler;
	struct notifier_block reboot_notifier;
};

static unsigned int da9062_get_wait_time(struct da9062_watchdog *wdt)
{
	unsigned long delay = msecs_to_jiffies(DA9062_RESET_PROTECTION_MS);
	unsigned long timeout = wdt->j_time_stamp + delay;
	unsigned long now = jiffies;

	/* if time-limit has not elapsed then wait for remainder */
	if (time_before(now, timeout)) {
		return timeout - now;
	}

	return 0; /* Watchdog can be pinged at once */
}

static void da9062_set_window_start(struct da9062_watchdog *wdt)
{
	wdt->j_time_stamp = jiffies;
}

static void da9062_apply_window_protection(struct da9062_watchdog *wdt)
{
	unsigned int diff_ms = jiffies_to_msecs(da9062_get_wait_time(wdt));

	/* if time-limit has not elapsed then wait for remainder */
	if (diff_ms) {
		dev_warn(wdt->hw->dev,
			 "Kicked too quickly. Delaying %u msecs\n", diff_ms);
		msleep(diff_ms);
	}
}

static unsigned int da9062_wdt_timeout_to_sel(unsigned int secs)
{
	unsigned int i;

	for (i = DA9062_TWDSCALE_MIN; i <= DA9062_TWDSCALE_MAX; i++) {
		if (wdt_timeout[i] >= secs)
			return i;
	}

	return DA9062_TWDSCALE_MAX;
}

static int da9062_reset_watchdog_timer(struct da9062_watchdog *wdt)
{
	int ret;

	da9062_apply_window_protection(wdt);

	ret = regmap_update_bits(wdt->hw->regmap,
			   DA9062AA_CONTROL_F,
			   DA9062AA_WATCHDOG_MASK,
			   DA9062AA_WATCHDOG_MASK);

	da9062_set_window_start(wdt);

	return ret;
}

static int da9062_wdt_update_timeout_register(struct da9062_watchdog *wdt,
					      unsigned int regval)
{
	struct da9062 *chip = wdt->hw;
	int ret;

	ret = da9062_reset_watchdog_timer(wdt);
	if (ret)
		return ret;

	return regmap_update_bits(chip->regmap,
				  DA9062AA_CONTROL_D,
				  DA9062AA_TWDSCALE_MASK,
				  regval);
}

static int da9062_wdt_start(struct watchdog_device *wdd)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int selector;
	int ret;

	/*
	 * Use da9062's SHUTDOWN mode instead of POWERDOWN for watchdog reset.
	 * On timeout the PMIC should reset the system, not powering it
	 * off.
	 */
	ret = regmap_update_bits(wdt->hw->regmap,
				 DA9062AA_CONFIG_I,
				 DA9062AA_WATCHDOG_SD_MASK,
				 DA9062AA_WATCHDOG_SD_MASK);
	if (ret)
		dev_err(wdt->hw->dev,
			"failed to set wdt reset mode. Expect poweroff on watchdog reset: %d\n",
			ret);

	selector = da9062_wdt_timeout_to_sel(wdt->wdtdev.timeout);
	ret = da9062_wdt_update_timeout_register(wdt, selector);
	if (ret)
		dev_err(wdt->hw->dev, "Watchdog failed to start (err = %d)\n",
			ret);

	return ret;
}

static int da9062_wdt_stop(struct watchdog_device *wdd)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	int ret;

	/*
	 * Cancel current ping request and wait for it to finished if it's
	 * currently running.
	 */
	cancel_delayed_work_sync(&wdt->ping_work);

	ret = da9062_reset_watchdog_timer(wdt);
	if (ret) {
		dev_err(wdt->hw->dev, "Failed to ping the watchdog (err = %d)\n",
			ret);
		return ret;
	}

	ret = regmap_update_bits(wdt->hw->regmap,
				 DA9062AA_CONTROL_D,
				 DA9062AA_TWDSCALE_MASK,
				 DA9062_TWDSCALE_DISABLE);
	if (ret)
		dev_err(wdt->hw->dev, "Watchdog failed to stop (err = %d)\n",
			ret);

	return ret;
}

static int da9062_wdt_ping(struct watchdog_device *wdd)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int delay = da9062_get_wait_time(wdt); /* in jiffies */

	schedule_delayed_work(&wdt->ping_work, delay);

	return 0;
}

static int _da9062_wdt_ping_work_now_helper(struct delayed_work *workd)
{
	struct da9062_watchdog *wdt = container_of(workd,
						struct da9062_watchdog,
						ping_work);
	int ret;

	da9062_apply_window_protection(wdt);

	ret = da9062_reset_watchdog_timer(wdt);
	if (ret)
		dev_err(wdt->hw->dev, "Failed to ping the watchdog (err = %d)\n",
			ret);

	da9062_set_window_start(wdt);

	return ret;
}

static void da9062_wdt_ping_work_now(struct work_struct *work)
{
	struct delayed_work *workd = container_of(work,
						struct delayed_work,
						work);

	_da9062_wdt_ping_work_now_helper(workd);
}

static int da9062_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int selector;
	int ret;

	selector = da9062_wdt_timeout_to_sel(timeout);
	ret = da9062_wdt_update_timeout_register(wdt, selector);
	if (ret)
		dev_err(wdt->hw->dev, "Failed to set watchdog timeout (err = %d)\n",
			ret);
	else
		wdd->timeout = wdt_timeout[selector];

	return ret;
}

/* See da9063_wdt_reboot_notifier for details */
static int da9062_wdt_reboot_notifier(struct notifier_block *this, unsigned long val, void *v)
{
	struct da9062_watchdog *wdt = container_of(this,
						   struct da9062_watchdog,
						   reboot_notifier);
	struct i2c_adapter *adap = wdt->hw->i2c->adapter;

	/*
	 * First block the I2C bus for other drivers. Other consumers are not
	 * allowed to access the bus now.
	 */
	adap->blocked = true;

	/*
	 * Then acquire adapter lock. This will wait until all other consumers
	 * have finished.  After that no more writes are possible for other
	 * drivers.
	 */
	i2c_lock_adapter(adap);

	/*
	 * Now the I2C adapter can be used in contexts that are not allowed to
	 * wait for locks or call schedule() because there is no other
	 * consumer.
	 */
	return NOTIFY_DONE;
}

/* See da9063_wdt_restart_handler for details */
static int da9062_wdt_restart_handler(struct notifier_block *this,
				      unsigned long mode, void *cmd)
{
	struct da9062_watchdog *wdt = container_of(this,
						   struct da9062_watchdog,
						   restart_handler);
	unsigned char data[3] = {DA9062AA_CONTROL_F, DA9062AA_SHUTDOWN_MASK, 0x0};
	struct i2c_client *client = wdt->hw->i2c;
	struct i2c_msg msgs[1] = {
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) | I2C_M_IRQLESS,
			.len = sizeof(data),
			.buf = data,
		}
	};
	int ret;

	/* TODO Maybe increase adapter->retries count. It is currently 0 for imx-driver. */

	ret = __i2c_transfer(client->adapter, msgs, sizeof(msgs));
	if (ret < 0)
		dev_alert(wdt->hw->dev, "Failed to shutdown (err = %d)\n",
			  ret);

	udelay(500); /* wait for reset to assert... */

	return NOTIFY_DONE;
}

static const struct watchdog_info da9062_watchdog_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "DA9062 WDT",
};

static const struct watchdog_ops da9062_watchdog_ops = {
	.owner = THIS_MODULE,
	.start = da9062_wdt_start,
	.stop = da9062_wdt_stop,
	.ping = da9062_wdt_ping,
	.set_timeout = da9062_wdt_set_timeout,
};

static int da9062_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct da9062 *chip;
	struct da9062_watchdog *wdt;

	chip = dev_get_drvdata(pdev->dev.parent);
	if (!chip)
		return -EINVAL;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->hw = chip;

	wdt->wdtdev.info = &da9062_watchdog_info;
	wdt->wdtdev.ops = &da9062_watchdog_ops;
	wdt->wdtdev.min_timeout = DA9062_WDT_MIN_TIMEOUT;
	wdt->wdtdev.max_timeout = DA9062_WDT_MAX_TIMEOUT;
	wdt->wdtdev.timeout = DA9062_WDG_DEFAULT_TIMEOUT;
	wdt->wdtdev.status = WATCHDOG_NOWAYOUT_INIT_STATUS;

	watchdog_set_drvdata(&wdt->wdtdev, wdt);
	dev_set_drvdata(&pdev->dev, wdt);

	ret = watchdog_register_device(&wdt->wdtdev);
	if (ret < 0) {
		dev_err(wdt->hw->dev,
			"watchdog registration failed (%d)\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&(wdt->ping_work), da9062_wdt_ping_work_now);

	da9062_set_window_start(wdt);

	ret = _da9062_wdt_ping_work_now_helper(&wdt->ping_work);
	if (ret < 0)
		watchdog_unregister_device(&wdt->wdtdev);

	wdt->reboot_notifier.notifier_call = da9062_wdt_reboot_notifier;
	wdt->reboot_notifier.priority = 0; /* be the last notifier */
	ret = register_reboot_notifier(&wdt->reboot_notifier);
	if (ret)
		dev_err(wdt->hw->dev,
			"Failed to register reboot notifier (err = %d)\n", ret);

	wdt->restart_handler.notifier_call = da9062_wdt_restart_handler;
	wdt->restart_handler.priority = 128;
	ret = register_restart_handler(&wdt->restart_handler);
	if (ret)
		dev_err(wdt->hw->dev,
			"Failed to register restart handler (err = %d)\n", ret);

	return ret;
}

static int da9062_wdt_remove(struct platform_device *pdev)
{
	struct da9062_watchdog *wdt = dev_get_drvdata(&pdev->dev);

	unregister_reboot_notifier(&wdt->reboot_notifier);

	unregister_restart_handler(&wdt->restart_handler);

	watchdog_unregister_device(&wdt->wdtdev);
	return 0;
}

static struct platform_driver da9062_wdt_driver = {
	.probe = da9062_wdt_probe,
	.remove = da9062_wdt_remove,
	.driver = {
		.name = "da9062-watchdog",
	},
};
module_platform_driver(da9062_wdt_driver);

MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("WDT device driver for Dialog DA9062");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:da9062-watchdog");
