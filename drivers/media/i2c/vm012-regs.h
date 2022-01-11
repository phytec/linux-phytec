/*	--*- c -*--
 * Copyright (C) 2016 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
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

#ifndef H_LINUX_KERNEL_DRIVERS_MEDIA_I2C_VM012_REGS_H
#define H_LINUX_KERNEL_DRIVERS_MEDIA_I2C_VM012_REGS_H

/* vita1300 selected register addresses as i2c address via i2c to spi brigde
 * on VM-012*/
#define VITA1300_CHIP_ID			0
#  define VITA1300_DTA_CHIP_ID		0x560d

#define VITA1300_CHIP_CONFIG		2
#define VITA1300_SOFT_RESET_PLL		8

#define VITA1300_SOFT_RESET_CGEN	9
#define VITA1300_SOFT_RESET_ANALOG	10
#define VITA1300_POWER_DOWN			16
#define VITA1300_I_O_CONFIG			20
#define VITA1300_CLOCK_GENERATOR	32
#  define VITA1300_FLD_CLOCK_GENERATOR_ENABLE_ANALOG	BIT(0)
#  define VITA1300_FLD_CLOCK_GENERATOR_ENABLE_LOGIC	BIT(1)
#  define VITA1300_FLD_CLOCK_GENERATOR_SELECT_PLL	BIT(2)
#  define VITA1300_FLD_CLOCK_GENERATOR_ADC_MODE_10	BIT(3)
#define VITA1300_GENERAL_LOGIC		34
#define VITA1300_IMAGE_CORE_01		40
#define VITA1300_IMAGE_CORE_02		41
#define VITA1300_AFE_POWER_DOWN		48
#define VITA1300_BIAS_POWER_DOWN	64
#define VITA1300_BIAS_CONFIG		65
#define VITA1300_BIAS_AFE_BIAS		66
#define VITA1300_BIAS_MUX_BIAS		67
#define VITA1300_BIAS_LVDS_BIAS		68
#define VITA1300_BIAS_AFE_REF		70
#define VITA1300_CHARGE_PUMP_CONF	72
#define VITA1300_SER_POWER_DOWN		112
#define VITA1300_DATA_BLOCK_BLACK	128
#define VITA1300_DATA_BLOCK_CONF	129

#define VM012_REG_FIRMWARE			0x97
#  define VM012_FLD_FIRMWARE_MINOR(_r)		((_r) << 0)
#  define VM012_FLD_FIRMWARE_MINOR_msk		VM012_FLD_FIRMWARE_MINOR(0xffu)
#  define VM012_FLD_FIRMWARE_MINOR_get(_reg)	(((_reg) & VM012_FLD_FIRMWARE_MINOR_msk) >> 0)
#  define VM012_FLD_FIRMWARE_MAJOR(_r)		((_r) << 8)
#  define VM012_FLD_FIRMWARE_MAJOR_msk		VM012_FLD_FIRMWARE_MAJOR(0xffu)
#  define VM012_FLD_FIRMWARE_MAJOR_get(_reg)	(((_reg) & VM012_FLD_FIRMWARE_MAJOR_msk) >> 8)

#define VM012_REG_CONTROL			0x98
#  define VM012_FLD_CONTROL_PAGE		BIT(0)
#  define VM012_FLD_CONTROL_AUTO_SENSOR_RESET	BIT(1)
#  define VM012_FLD_CONTROL_SENSOR_RESET	BIT(2)
#  define VM012_FLD_CONTROL_VM012_RESET		BIT(8)

#define VM012_REG_STATUS			0x99
#  define VM012_FLD_STATUS_SPI_READ		BIT(0)
#  define VM012_FLD_STATUS_SPI_WRITE		BIT(1)

#define VITA1300_AEC_REG176			176
#define VITA1300_AEC_REG180			180
#define VITA1300_AEC_REG181			181

#define VITA1300_SEQ_CONF			192
#  define VITA1300_FLD_SEQ_CONF_ENA_SEQ			BIT(0)
#  define VITA1300_FLD_SEQ_CONF_ENA_ROL_SHUT		BIT(1)
#  define VITA1300_FLD_SEQ_CONF_TRIGGERED_MODE		BIT(4)
#  define VITA1300_FLD_SEQ_CONF_SLAVE_MODE		BIT(5)
#  define VITA1300_FLD_SEQ_CONF_ENA_XSM_DELAY		BIT(6)
#  define VITA1300_FLD_SEQ_CONF_ENA_SUBSAMPLING		BIT(7)
#  define VITA1300_FLD_SEQ_CONF_ENA_BINNING		BIT(8)
#  define VITA1300_FLD_SEQ_CONF_ENA_ROI_AEC		BIT(10)
#  define VITA1300_FLD_SEQ_CONF_MONITOR_SEL(_s)		((_s) << 11)

#define VITA1300_REG_ROI_ACTIVE		195
#define VITA1300_SEQ_BLACK_LINES	197
#define VITA1300_SET_DB_GAIN		205

#define VITA1300_REG_ROIx_CFG0(_x)	(256 + (_x) * 3)
#define VITA1300_REG_ROIx_CFG1(_x)	(257 + (_x) * 3)
#define VITA1300_REG_ROIx_CFG2(_x)	(258 + (_x) * 3)

#  define VITA1300_FLD_ROI_CFG0_X_START(_x)		((_x) << 0)
#  define VITA1300_FLD_ROI_CFG0_X_START_msk		VITA1300_FLD_ROI_CFG0_X_START(0xffu)
#  define VITA1300_FLD_ROI_CFG0_X_START_get(_r)		(((_r) & VITA1300_FLD_ROI_CFG0_X_START_msk) >> 0)

#  define VITA1300_FLD_ROI_CFG0_X_END(_x)		((_x) << 8)
#  define VITA1300_FLD_ROI_CFG0_X_END_msk		VITA1300_FLD_ROI_CFG0_X_END(0xffu)
#  define VITA1300_FLD_ROI_CFG0_X_END_get(_r)		(((_r) & VITA1300_FLD_ROI_CFG0_X_END_msk) >> 8)

#  define VITA1300_FLD_ROI_CFG1_Y_START(_x)		((_x) << 0)
#  define VITA1300_FLD_ROI_CFG1_Y_START_msk		VITA1300_FLD_ROI_CFG1_Y_START(0x3ffu)
#  define VITA1300_FLD_ROI_CFG1_Y_START_get(_r)		(((_r) & VITA1300_FLD_ROI_CFG1_Y_START_msk) >> 0)

#  define VITA1300_FLD_ROI_CFG2_Y_END(_x)		((_x) << 0)
#  define VITA1300_FLD_ROI_CFG2_Y_END_msk		VITA1300_FLD_ROI_CFG2_Y_END(0x3ffu)
#  define VITA1300_FLD_ROI_CFG2_Y_END_get(_r)		(((_r) & VITA1300_FLD_ROI_CFG2_Y_END_msk) >> 0)


#define VITA1300_SEQ_REG387			387
#define VITA1300_SEQ_REG388			388
#define VITA1300_SEQ_REG389			389
#define VITA1300_SEQ_REG390			390
#define VITA1300_SEQ_REG391			391
#define VITA1300_SEQ_REG392			392
#define VITA1300_SEQ_REG431			431
#define VITA1300_SEQ_REG432			432
#define VITA1300_SEQ_REG433			433
#define VITA1300_SEQ_REG434			434
#define VITA1300_SEQ_REG435			435
#define VITA1300_SEQ_REG436			436
#define VITA1300_SEQ_REG437			437
#define VITA1300_SEQ_REG438			438
#define VITA1300_SEQ_REG439			439
#define VITA1300_SEQ_REG440			440
#define VITA1300_SEQ_REG441			441
#define VITA1300_SEQ_REG442			442
#define VITA1300_SEQ_REG443			443
#define VITA1300_SEQ_REG447			447
#define VITA1300_SEQ_REG448			448
#define VITA1300_SEQ_REG449			449
#define VITA1300_SEQ_REG450			450
#define VITA1300_SEQ_REG451			451
#define VITA1300_SEQ_REG452			452
#define VITA1300_SEQ_REG453			453
#define VITA1300_SEQ_REG454			454
#define VITA1300_SEQ_REG455			455
#define VITA1300_SEQ_REG456			456
#define VITA1300_SEQ_REG457			457
#define VITA1300_SEQ_REG458			458
#define VITA1300_SEQ_REG459			459
#define VITA1300_SEQ_REG460			460
#define VITA1300_SEQ_REG469			469
#define VITA1300_SEQ_REG472			472
#define VITA1300_SEQ_REG476			476
#define VITA1300_SEQ_REG480			480
#define VITA1300_SEQ_REG481			481
#define VITA1300_SEQ_REG484			484
#define VITA1300_SEQ_REG485			485
#define VITA1300_SEQ_REG489			489
#define VITA1300_SEQ_REG493			493
#define VITA1300_SEQ_REG496			496
#define VITA1300_SEQ_REG500			500
#define VITA1300_SEQ_REG504			504
#define VITA1300_SEQ_REG505			505
#define VITA1300_SEQ_REG508			508
#define VITA1300_SEQ_REG509			509


#define VITA1300_STATIC_INIT_ENABLE_CLOCK_MANAGEMENT_V2			\
	/* Configure clock management */				\
	REGSET(VITA1300_CLOCK_GENERATOR, 0x200c),			\
	/* Configure clock management */				\
	REGSET(VITA1300_I_O_CONFIG, 0),					\
	/* Configure PLL bypass mode */					\
	REGSET(VITA1300_POWER_DOWN, 0x0007),				\
	/* Release clock generator soft reset */			\
	REGSET(VITA1300_SOFT_RESET_CGEN, 0x0000),			\
	/* Enable logic clock */					\
	REGSET(VITA1300_CLOCK_GENERATOR, 0x200e),			\
	/* Enable logic blocks */					\
	REGSET(VITA1300_GENERAL_LOGIC, 0x0001)				\

#define VITA1300_STATIC_INIT_REQUIRED_REGISTERS				\
	/* Configure image core */					\
	REGSET(VITA1300_IMAGE_CORE_02, 0x085a),				\
	/* Configure 10-bit mode */					\
	REGSET(VITA1300_DATA_BLOCK_CONF, 0xc001),			\
	/* Configure CP biasing */					\
	REGSET(VITA1300_BIAS_CONFIG, 0x288b),				\
	/* Configure AFE biasing */					\
	REGSET(VITA1300_BIAS_AFE_BIAS, 0x53c5),				\
	/* Configure MUX biasing */					\
	REGSET(VITA1300_BIAS_MUX_BIAS, 0x0344),				\
	/* Configure LVDS biasing */					\
	REGSET(VITA1300_BIAS_LVDS_BIAS, 0x0085),			\
	/* Configure AFE biasing */					\
	REGSET(VITA1300_BIAS_AFE_REF, 0x4800),				\
	/* Configure black calibration */				\
	REGSET(VITA1300_DATA_BLOCK_BLACK, 0x4710),			\
	/* Configure black calibration */				\
	REGSET(VITA1300_SEQ_BLACK_LINES, 0x0103),			\
	/* Configure AEC */						\
	REGSET(VITA1300_AEC_REG176, 0x00f5),				\
	/* Configure AEC */						\
	REGSET(VITA1300_AEC_REG180, 0x00fd),				\
	/* Configure AEC */						\
	REGSET(VITA1300_AEC_REG181, 0x0144),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG387, 0x549f),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG388, 0x549f),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG389, 0x5091),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG390, 0x1011),				\
	/* Configure sequencer*/					\
	REGSET(VITA1300_SEQ_REG391, 0x111f),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG392, 0x1110),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG431, 0x0356),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG432, 0x0141),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG433, 0x214f),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG434, 0x214a),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG435, 0x2101),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG436, 0x0101),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG437, 0x0b85),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG438, 0x0381),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG439, 0x0181),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG440, 0x218f),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG441, 0x218a),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG442, 0x2101),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG443, 0x0100),				\
	/* Configure sequencer */					\
	REGSET(VITA1300_SEQ_REG447, 0x0b55),				\
	REGSET(VITA1300_SEQ_REG448, 0x0351),				\
	REGSET(VITA1300_SEQ_REG449, 0x0141),				\
	REGSET(VITA1300_SEQ_REG450, 0x214f),				\
	REGSET(VITA1300_SEQ_REG451, 0x214a),				\
	REGSET(VITA1300_SEQ_REG452, 0x2101),				\
	REGSET(VITA1300_SEQ_REG453, 0x0101),				\
	REGSET(VITA1300_SEQ_REG454, 0x0b85),				\
	REGSET(VITA1300_SEQ_REG455, 0x0381),				\
	REGSET(VITA1300_SEQ_REG456, 0x0181),				\
	REGSET(VITA1300_SEQ_REG457, 0x218f),				\
	REGSET(VITA1300_SEQ_REG458, 0x218a),				\
	REGSET(VITA1300_SEQ_REG459, 0x2101),				\
	REGSET(VITA1300_SEQ_REG460, 0x0100),				\
	REGSET(VITA1300_SEQ_REG469, 0x2184),				\
	REGSET(VITA1300_SEQ_REG472, 0x1347),				\
	REGSET(VITA1300_SEQ_REG476, 0x2144),				\
	REGSET(VITA1300_SEQ_REG480, 0x8d04),				\
	REGSET(VITA1300_SEQ_REG481, 0x8501),				\
	REGSET(VITA1300_SEQ_REG484, 0xcd04),				\
	REGSET(VITA1300_SEQ_REG485, 0xc501),				\
	REGSET(VITA1300_SEQ_REG489, 0x0be2),				\
	REGSET(VITA1300_SEQ_REG493, 0x2184),				\
	REGSET(VITA1300_SEQ_REG496, 0x1347),				\
	REGSET(VITA1300_SEQ_REG500, 0x2144),				\
	REGSET(VITA1300_SEQ_REG504, 0x8d04),				\
	REGSET(VITA1300_SEQ_REG505, 0x8501),				\
	REGSET(VITA1300_SEQ_REG508, 0xcd04),				\
	REGSET(VITA1300_SEQ_REG509, 0xc501)				\

	/* SOFT POWER UP REGISTER UPLOADS FOR MODE DEPENDENT REGISTERS
	 * V2-SN/SE 10-bit mode */
#define VITA1300_STATIC_INIT_MODE_REGISTERS_V2				\
	/* Enable analog clock distribution */				\
	REGSET(VITA1300_CLOCK_GENERATOR, 0x200f),			\
	/* Release soft reset state */					\
	REGSET(VITA1300_SOFT_RESET_ANALOG, 0x0000),			\
	/* Enable biasing block */					\
	REGSET(VITA1300_BIAS_POWER_DOWN, 0x0001),			\
	/* Enable charge pump */					\
	REGSET(VITA1300_CHARGE_PUMP_CONF, 0x0203),			\
	/* Enable column multiplexer */					\
	REGSET(VITA1300_IMAGE_CORE_01, 0x0003),				\
	/* Enable AFE */						\
	REGSET(VITA1300_AFE_POWER_DOWN, 0x0001),			\
	/* Configure I/O */						\
	REGSET(VITA1300_SER_POWER_DOWN, 0x0000)


#endif	/* H_LINUX_KERNEL_DRIVERS_MEDIA_I2C_VM012_REGS_H */
