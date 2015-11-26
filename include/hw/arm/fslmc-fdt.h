/*
 * Dynamic FSL mc-bus device tree node generation API
 *
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Authors:
 *  Bharat Bhushan <bharat.bhushan@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef HW_FSL_MC_FDT_H
#define HW_FSL_MC_FDT_H

#include "hw/arm/arm.h"
#include "qemu-common.h"
#include "hw/sysbus.h"

/*
 * struct that contains dimensioning parameters of the platform bus
 */
typedef struct {
    hwaddr fslmc_bus_base; /* start address of the bus */
    hwaddr fslmc_bus_size; /* size of the bus */
    int fslmc_bus_first_irq; /* first hwirq assigned to the bus */
    int fslmc_bus_num_irqs; /* number of hwirq assigned to the bus */
} FSLMCBusSystemParams;

/*
 * struct that contains all relevant info to build the fdt nodes of
 * platform bus and attached dynamic sysbus devices
 * in the future might be augmented with additional info
 * such as PHY, CLK handles ...
 */
typedef struct {
    const FSLMCBusSystemParams *system_params;
    struct arm_boot_info *binfo;
    const char *intc; /* parent interrupt controller name */
} FSLMCBusFDTParams;

/**
 * arm_register_platform_bus_fdt_creator - register a machine init done
 * notifier that creates the device tree nodes of the platform bus and
 * associated dynamic sysbus devices
 */
void fsl_register_mc_bus_fdt_creator(FSLMCBusFDTParams *fdt_params);

#endif
