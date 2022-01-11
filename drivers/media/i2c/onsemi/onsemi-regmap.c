/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Regmap access drivers for ONSemi ARxxx sensor devices.
 *
 * Copyright (C) 2018 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 */

#include "onsemi-regmap.h"

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define I2C_BUFSZ		4

struct regmap_onsemi_ctx {
	struct i2c_client			*i2c;
	struct regmap				*regmap;

	struct mutex				lock;
	struct {
		unsigned char	wr[I2C_BUFSZ] ____cacheline_aligned;
		unsigned char	rd[I2C_BUFSZ] ____cacheline_aligned;
	}			xfer;
};

static bool _is_8bit(struct regmap_onsemi_ctx const *ctx,
		     unsigned int reg)
{
	return (reg & 0x8000u) != 0;
}

static int regmap_onsemi_reg_read(void *ctx_, unsigned int reg,
				  unsigned int *val)
{
	struct regmap_onsemi_ctx	*ctx = ctx_;
	struct i2c_client		*i2c = ctx->i2c;
	bool				is_8bit = _is_8bit(ctx, reg);
	struct i2c_msg			xfer[] = {
		[0] = {
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= ctx->xfer.wr,
		},
		[1] = {
			.addr	= i2c->addr,
			.flags	= I2C_M_RD,
			.len	= is_8bit ? 1 : 2,
			.buf	= ctx->xfer.rd,
		},
	};
	int				rc;
	uint16_t			res;

	mutex_lock(&ctx->lock);

	reg = ONSEMI_REGMAP_ADDR_TO_REG(reg);

	ctx->xfer.wr[0] = (reg >> 8) & 0xff;
	ctx->xfer.wr[1] = (reg >> 0) & 0xff;

	rc = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (rc == ARRAY_SIZE(xfer))
		rc = 0;
	else if (rc >= 0)
		rc = -EIO;

	if (rc < 0)
		goto out;

	res = ctx->xfer.rd[0];

	if (!is_8bit) {
		res <<= 8;
		res  |= ctx->xfer.rd[1];
	}

	*val = res;

out:
	mutex_unlock(&ctx->lock);

	return rc;
}

static int regmap_onsemi_reg_write(void *ctx_, unsigned int reg,
				   unsigned int val)
{
	struct regmap_onsemi_ctx	*ctx = ctx_;
	struct i2c_client		*i2c = ctx->i2c;
	bool				is_8bit = _is_8bit(ctx, reg);
	struct i2c_msg			xfer[] = {
		[0] = {
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= is_8bit ? 3 : 4,
			.buf	= ctx->xfer.wr,
		}
	};
	int				rc;

	if (WARN_ON(is_8bit && val > 0xff))
		return -EINVAL;

	if (WARN_ON(!is_8bit && reg % 2 != 0))
		return -EINVAL;

	mutex_lock(&ctx->lock);

	reg = ONSEMI_REGMAP_ADDR_TO_REG(reg);

	ctx->xfer.wr[0] = (reg >> 8) & 0xff;
	ctx->xfer.wr[1] = (reg >> 0) & 0xff;

	if (is_8bit) {
		ctx->xfer.wr[2] = val;
	} else {
		ctx->xfer.wr[2] = (val >> 8) & 0xff;
		ctx->xfer.wr[3] = (val >> 0) & 0xff;
	}

	rc = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (rc == ARRAY_SIZE(xfer))
		rc = 0;
	else if (rc >= 0)
		rc = -EIO;

	mutex_unlock(&ctx->lock);

	return rc;
}


static struct regmap_bus regmap_onsemi_bus = {
	.reg_write		= regmap_onsemi_reg_write,
	.reg_read		= regmap_onsemi_reg_read,
};

static void devm_regmap_onsemi_release(struct device *dev, void *ctx_)
{
	struct regmap_onsemi_ctx	*ctx = ctx_;

	regmap_exit(ctx->regmap);
}

struct regmap *__devm_regmap_init_onsemi(struct i2c_client *i2c,
					 struct regmap_config const *config,
					 struct lock_class_key *lock_key,
					 const char *lock_name)
{
	struct regmap_onsemi_ctx	*ctx;
	struct regmap			*regmap;

	ctx = devres_alloc(devm_regmap_onsemi_release, sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	regmap = __regmap_init(&i2c->dev, &regmap_onsemi_bus, ctx,
			       config, lock_key, lock_name);
	if (IS_ERR(regmap)) {
		devres_free(ctx);
		goto out;
	}

	*ctx = (struct regmap_onsemi_ctx) {
		.i2c		= i2c,
		.regmap		= regmap,
		.lock		= __MUTEX_INITIALIZER(ctx->lock),
	};

	devres_add(&i2c->dev, ctx);

out:
	return regmap;
}
EXPORT_SYMBOL_GPL(__devm_regmap_init_onsemi);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
