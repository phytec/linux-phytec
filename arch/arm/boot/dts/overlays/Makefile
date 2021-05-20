# SPDX-License-Identifier: GPL-2.0

dtb-y += imx6-cam-0-lvds.dtbo
dtb-y += imx6-cam-1-lvds.dtbo
dtb-y += imx6-vm009-0.dtbo
dtb-y += imx6-vm009-1.dtbo
dtb-y += imx6-vm010-bw-0.dtbo
dtb-y += imx6-vm010-bw-1.dtbo
dtb-y += imx6-vm010-col-0.dtbo
dtb-y += imx6-vm010-col-1.dtbo
dtb-y += imx6-vm011-bw-0.dtbo
dtb-y += imx6-vm011-bw-1.dtbo
dtb-y += imx6-vm011-col-0.dtbo
dtb-y += imx6-vm011-col-1.dtbo
dtb-y += imx6-vm012-bw-0.dtbo
dtb-y += imx6-vm012-bw-1.dtbo
dtb-y += imx6-vm012-col-0.dtbo
dtb-y += imx6-vm012-col-1.dtbo
dtb-y += imx6-vm016-0.dtbo
dtb-y += imx6-vm016-1.dtbo
dtb-y += imx6-vm016-mipi-0.dtbo
dtb-y += imx6-vm050-0.dtbo
dtb-y += imx6-vm050-1.dtbo

DTC_FLAGS += -@

quiet_cmd_dtbo = DTBO     $@
cmd_dtbo = $(HOSTCC) -E $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ; \
	   $(DTC) -O dtb -o $@ -b 0 \
		$(addprefix -i,$(dir $<) $(DTC_INCLUDE)) $(DTC_FLAGS) \
		-d $(depfile).dtc.tmp $(dtc-tmp) ; \
	   cat $(depfile).pre.tmp $(depfile).dtc.tmp > $(depfile)

$(obj)/%.dtbo: $(src)/%.dtso $(DTC) FORCE
	$(call if_changed_dep,dtbo)
