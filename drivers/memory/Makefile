# SPDX-License-Identifier: GPL-2.0
#
# Makefile for memory devices
#

ifeq ($(CONFIG_DDR),y)
obj-$(CONFIG_OF)		+= of_memory.o
endif
obj-$(CONFIG_ARM_PL172_MPMC)	+= pl172.o
obj-$(CONFIG_ATMEL_SDRAMC)	+= atmel-sdramc.o
obj-$(CONFIG_ATMEL_EBI)		+= atmel-ebi.o
obj-$(CONFIG_TI_AEMIF)		+= ti-aemif.o
obj-$(CONFIG_TI_EMIF)		+= emif.o
obj-$(CONFIG_OMAP_GPMC)		+= omap-gpmc.o
obj-$(CONFIG_FSL_CORENET_CF)	+= fsl-corenet-cf.o
obj-$(CONFIG_FSL_IFC)		+= fsl_ifc.o
obj-$(CONFIG_MVEBU_DEVBUS)	+= mvebu-devbus.o
obj-$(CONFIG_TEGRA20_MC)	+= tegra20-mc.o
obj-$(CONFIG_JZ4780_NEMC)	+= jz4780-nemc.o
obj-$(CONFIG_MTK_SMI)		+= mtk-smi.o
obj-$(CONFIG_DA8XX_DDRCTL)	+= da8xx-ddrctl.o

obj-$(CONFIG_SAMSUNG_MC)	+= samsung/
obj-$(CONFIG_TEGRA_MC)		+= tegra/
obj-$(CONFIG_TI_EMIF_SRAM)	+= ti-emif-sram.o
ti-emif-sram-objs		:= ti-emif-pm.o ti-emif-sram-pm.o

AFLAGS_ti-emif-sram-pm.o	:=-Wa,-march=armv7-a

ifneq ($(filter y,$(CONFIG_SOC_AM33XX) $(CONFIG_SOC_AM43XX)),)
#Generate ASM friendly offset macros from C code
offsets-file := emif-asm-offsets.h

targets += $(obj)/$(offsets-file) drivers/memory/emif-asm-offsets.s

$(obj)/$(offsets-file): $(obj)/emif-asm-offsets.s FORCE
	$(call filechk,offsets,__TI_EMIF_ASM_OFFSETS_H__)
	cp $(obj)/$(offsets-file) arch/arm/include/generated/.

# We use internal kbuild rules to avoid the "is up to date" message from make
$(obj)/emif-asm-offsets.s: $(obj)/emif-asm-offsets.c FORCE
	$(call if_changed_dep,cc_s_c)

$(obj)/ti-emif-sram-pm.o: $(obj)/$(offsets-file)
endif
