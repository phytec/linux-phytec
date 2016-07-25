/*
 * Watchdog driver for DA9063 PMICs.
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *
 * Author: Mariusz Wojtasik <mariusz.wojtasik@diasemi.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/mfd/da9063/core.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

/*
 * Watchdog selector to timeout in seconds.
 *   0: WDT disabled;
 *   others: timeout = 2048 ms * 2^(TWDSCALE-1).
 */
static const unsigned int wdt_timeout[] = { 0, 2, 4, 8, 16, 32, 65, 131 };
#define DA9063_TWDSCALE_DISABLE		0
#define DA9063_TWDSCALE_MIN		1
#define DA9063_TWDSCALE_MAX		(ARRAY_SIZE(wdt_timeout) - 1)
#define DA9063_WDT_MIN_TIMEOUT		wdt_timeout[DA9063_TWDSCALE_MIN]
#define DA9063_WDT_MAX_TIMEOUT		wdt_timeout[DA9063_TWDSCALE_MAX]
#define DA9063_WDG_TIMEOUT		wdt_timeout[3]
#define DA9063_RESET_PROTECTION_MS	256

struct da9063_watchdog {
	struct da9063 *da9063;
	struct watchdog_device wdtdev;
	struct notifier_block restart_handler;
	struct notifier_block reboot_notifier;
	struct delayed_work ping_work;
	unsigned long j_time_stamp;
};

static void da9063_set_window_start(struct da9063_watchdog *wdt)
{
	wdt->j_time_stamp = jiffies;
}

static unsigned int da9063_get_wait_time(struct da9063_watchdog *wdt)
{
	unsigned long delay = msecs_to_jiffies(DA9063_RESET_PROTECTION_MS);
	unsigned long timeout = wdt->j_time_stamp + delay;
	unsigned long now = jiffies;

	/* if time-limit has not elapsed then wait for remainder */
	if (time_before(now, timeout)) {
		return timeout - now;
	}

	return 0; /* Watchdog can be pinged at once */
}

static void da9063_apply_window_protection(struct da9063_watchdog *wdt)
{
	unsigned int diff_ms = jiffies_to_msecs(da9063_get_wait_time(wdt));

	/* if time-limit has not elapsed then wait for remainder */
	if (diff_ms) {
		dev_dbg(wdt->da9063->dev,
			 "Kicked too quickly. Delaying %u msecs\n", diff_ms);
		msleep(diff_ms);
	}
}

static unsigned int da9063_wdt_timeout_to_sel(unsigned int secs)
{
	unsigned int i;

	for (i = DA9063_TWDSCALE_MIN; i <= DA9063_TWDSCALE_MAX; i++) {
		if (wdt_timeout[i] >= secs)
			return i;
	}

	return DA9063_TWDSCALE_MAX;
}

static int _da9063_wdt_set_timeout(struct da9063_watchdog *wdt, unsigned int regval)
{
	int ret;

	da9063_apply_window_protection(wdt);

	ret = regmap_update_bits(wdt->da9063->regmap, DA9063_REG_CONTROL_D,
				  DA9063_TWDSCALE_MASK, regval);

	da9063_set_window_start(wdt);

	return ret;
}

static int da9063_wdt_start(struct watchdog_device *wdd)
{
	struct da9063_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int selector;
	int ret;

	selector = da9063_wdt_timeout_to_sel(wdt->wdtdev.timeout);
	ret = _da9063_wdt_set_timeout(wdt, selector);
	if (ret)
		dev_err(wdt->da9063->dev, "Watchdog failed to start (err = %d)\n",
			ret);

	da9063_set_window_start(wdt);

	return ret;
}

static int da9063_wdt_stop(struct watchdog_device *wdd)
{
	struct da9063_watchdog *wdt = watchdog_get_drvdata(wdd);
	int ret;

	/*
	 * Cancel current ping request and wait for it to finished if it's
	 * currently running.
	 */
	cancel_delayed_work_sync(&wdt->ping_work);

	ret = regmap_update_bits(wdt->da9063->regmap, DA9063_REG_CONTROL_D,
				 DA9063_TWDSCALE_MASK, DA9063_TWDSCALE_DISABLE);
	if (ret)
		dev_alert(wdt->da9063->dev, "Watchdog failed to stop (err = %d)\n",
			  ret);

	return ret;
}

static void da9063_wdt_ping_work_now(struct work_struct *work)
{
	struct delayed_work *workd = container_of(work,
						struct delayed_work,
						work);
	struct da9063_watchdog *wdt = container_of(workd,
						struct da9063_watchdog,
						ping_work);
	int ret;

	da9063_apply_window_protection(wdt);

	ret = regmap_write(wdt->da9063->regmap, DA9063_REG_CONTROL_F,
			   DA9063_WATCHDOG);
	if (ret)
		dev_alert(wdt->da9063->dev, "Failed to ping the watchdog (err = %d)\n",
			  ret);

	da9063_set_window_start(wdt);
}

static int da9063_wdt_ping(struct watchdog_device *wdd)
{
	struct da9063_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int delay = da9063_get_wait_time(wdt); /* in jiffies */

	schedule_delayed_work(&wdt->ping_work, delay);

	return 0;
}

static int da9063_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct da9063_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int selector;
	int ret;

	selector = da9063_wdt_timeout_to_sel(timeout);
	ret = _da9063_wdt_set_timeout(wdt, selector);
	if (ret)
		dev_err(wdt->da9063->dev, "Failed to set watchdog timeout (err = %d)\n",
			ret);
	else
		wdd->timeout = wdt_timeout[selector];

	return ret;
}

static int da9063_wdt_reboot_notifier(struct notifier_block *this, unsigned long val, void *v)
{
	struct da9063_watchdog *wdt = container_of(this,
						   struct da9063_watchdog,
						   reboot_notifier);
	struct i2c_adapter *adap = wdt->da9063->i2c->adapter;

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

static int da9063_wdt_restart_handler(struct notifier_block *this,
				      unsigned long mode, void *cmd)
{
	struct da9063_watchdog *wdt = container_of(this,
						   struct da9063_watchdog,
						   restart_handler);
	unsigned char data[3] = {DA9063_REG_CONTROL_F, DA9063_SHUTDOWN, 0x0};
	struct i2c_client *client = wdt->da9063->i2c;
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

	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_alert(wdt->da9063->dev, "Failed to shutdown (err = %d)\n",
			  ret);

	udelay(500); /* wait for reset to assert... */

	return NOTIFY_DONE;
}

static const struct watchdog_info da9063_watchdog_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "DA9063 Watchdog",
};

static const struct watchdog_ops da9063_watchdog_ops = {
	.owner = THIS_MODULE,
	.start = da9063_wdt_start,
	.stop = da9063_wdt_stop,
	.ping = da9063_wdt_ping,
	.set_timeout = da9063_wdt_set_timeout,
};

static int da9063_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct da9063 *da9063;
	struct da9063_watchdog *wdt;

	if (!pdev->dev.parent)
		return -EINVAL;

	da9063 = dev_get_drvdata(pdev->dev.parent);
	if (!da9063)
		return -EINVAL;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->da9063 = da9063;
	da9063_set_window_start(wdt);

	wdt->wdtdev.info = &da9063_watchdog_info;
	wdt->wdtdev.ops = &da9063_watchdog_ops;
	wdt->wdtdev.min_timeout = DA9063_WDT_MIN_TIMEOUT;
	wdt->wdtdev.max_timeout = DA9063_WDT_MAX_TIMEOUT;
	wdt->wdtdev.timeout = DA9063_WDG_TIMEOUT;

	wdt->wdtdev.status = WATCHDOG_NOWAYOUT_INIT_STATUS;

	watchdog_set_drvdata(&wdt->wdtdev, wdt);
	dev_set_drvdata(&pdev->dev, wdt);

	ret = watchdog_register_device(&wdt->wdtdev);
	if (ret)
		return ret;

	wdt->reboot_notifier.notifier_call = da9063_wdt_reboot_notifier;
	wdt->reboot_notifier.priority = 0; /* be the last notifier */
	ret = register_reboot_notifier(&wdt->reboot_notifier);
	if (ret)
		dev_err(wdt->da9063->dev,
			"Failed to register reboot notifier (err = %d)\n", ret);

	wdt->restart_handler.notifier_call = da9063_wdt_restart_handler;
	wdt->restart_handler.priority = 128;
	ret = register_restart_handler(&wdt->restart_handler);
	if (ret)
		dev_err(wdt->da9063->dev,
			"Failed to register restart handler (err = %d)\n", ret);

	INIT_DELAYED_WORK(&(wdt->ping_work), da9063_wdt_ping_work_now);

	return 0;
}

static int da9063_wdt_remove(struct platform_device *pdev)
{
	struct da9063_watchdog *wdt = dev_get_drvdata(&pdev->dev);

	/* Wait for delayed worker to finish. */
	cancel_delayed_work_sync(&wdt->ping_work);

	unregister_reboot_notifier(&wdt->reboot_notifier);

	unregister_restart_handler(&wdt->restart_handler);

	watchdog_unregister_device(&wdt->wdtdev);

	return 0;
}

static struct platform_driver da9063_wdt_driver = {
	.probe = da9063_wdt_probe,
	.remove = da9063_wdt_remove,
	.driver = {
		.name = DA9063_DRVNAME_WATCHDOG,
	},
};
module_platform_driver(da9063_wdt_driver);

MODULE_AUTHOR("Mariusz Wojtasik <mariusz.wojtasik@diasemi.com>");
MODULE_DESCRIPTION("Watchdog driver for Dialog DA9063");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DA9063_DRVNAME_WATCHDOG);
