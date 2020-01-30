/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Regmap access drivers for ONSemi ARxxx sensor devices.
 *
 * Copyright (C) 2018 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 */

#ifndef H_LINUX_MEDIA_I2C_ONSEMI_REGMAP_H
#define H_LINUX_MEDIA_I2C_ONSEMI_REGMAP_H

#include <linux/regmap.h>

struct regmap *__devm_regmap_init_onsemi(struct i2c_client *i2c,
					 struct regmap_config const *config,
					 struct lock_class_key *lock_key,
					 const char *lock_name);

#define devm_regmap_init_onsemi(i2c, config)				\
	__regmap_lockdep_wrapper(__devm_regmap_init_onsemi, #config,	\
				 i2c, config)


#define ONSEMI_REGMAP_8BIT_REG_TO_ADDR(_reg) \
	(0x8000u | ((_reg) << 1))

#define ONSEMI_REGMAP_ADDR_TO_REG(_addr) \
	(((_addr) & 0x8000u) ? (((_addr) & ~0x8000u) >> 1) : (_addr))


#endif	/* H_LINUX_MEDIA_I2C_ONSEMI_REGMAP_H */
