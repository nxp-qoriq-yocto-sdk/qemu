/*
 * Freescale Management Complex (MC) Bus device tree generation helpers
 *
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Authors:
 *  Bharat Bhushan <bharat.bhushan@freescale.com>
 *
 * This file is derived from hw/arm/sysbus-fdt.c
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

#include "hw/arm/sysbus-fdt.h"
#include "qemu/error-report.h"
#include "sysemu/device_tree.h"
#include "sysemu/sysemu.h"
#include "hw/fsl-mc/fsl-mc.h"
#include "hw/vfio/vfio-fsl-mc.h"
#include "hw/arm/fslmc-fdt.h"

/*
 * internal struct that contains the information to create dynamic
 * sysbus device node
 */
typedef struct FslmcBusFDTData {
    void *fdt; /* device tree handle */
} FslmcBusFDTData;

/*
 * struct used when calling the machine init done notifier
 * that constructs the fdt nodes of mc bus devices
 */
typedef struct FSLMCBusFDTNotifierParams {
    Notifier notifier;
    FSLMCBusFDTParams *fdt_params;
} FSLMCBusFDTNotifierParams;

/* Device Specific Code */

/**
 * add_fsl_mc_bus_fdt_node - create all fsl mc bus node
 *
 * builds the root fsl mc bus node and map regions of other devices
 */
static void add_fsl_mc_bus_fdt_node(FSLMCBusFDTParams *fdt_params)
{
    const char compat[] = "fsl,qoriq-mc";
    gchar *node;
    hwaddr mc_p_addr, mc_p_size, qbman_p_addr, qbman_p_size;
    hwaddr mcaddr, mcsize;
    int dtb_size;
    struct arm_boot_info *info = fdt_params->binfo;
    const FSLMCBusSystemParams *params = fdt_params->system_params;
    void *fdt = info->get_dtb(info, &dtb_size);
    uint32_t *irq_attr;
    int irq_num, i;
    int ret;

    /*
     * If the user provided a dtb, we assume the dynamic sysbus nodes
     * already are integrated there. This corresponds to a use case where
     * the dynamic sysbus nodes are complex and their generation is not yet
     * supported. In that case the user can take charge of the guest dt
     * while qemu takes charge of the qom stuff.
     */
    if (info->dtb_filename) {
        return;
    }

    fsl_mc_get_portals_ranges(&mc_p_addr, &mc_p_size,
                              &qbman_p_addr, &qbman_p_size);
    fsl_mc_get_root_mcp_addr_range(&mcaddr, &mcsize);

    irq_attr = g_new(uint32_t, 64 * 3);
    for (i = 0; i < 64; i++) {
        irq_num = params->fslmc_bus_first_irq + i;
        irq_attr[3 * i] = cpu_to_be32(0);
        irq_attr[3 * i + 1] = cpu_to_be32(irq_num);
        irq_attr[3 * i + 2] = cpu_to_be32(1);
    }

    assert(fdt);

    node = g_strdup_printf("/fsl-mc@%"PRIx64, params->fslmc_bus_base);

    /* Create a /fsl-mc node that we can put all devices into */
    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop(fdt, node, "compatible", compat, sizeof(compat));
    qemu_fdt_setprop_cells(fdt, node, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt, node, "#address-cells", 3);
    qemu_fdt_setprop_cells(fdt, node, "ranges", 0x0, 0x0, 0x0,
                           mc_p_addr >> 32, mc_p_addr, mc_p_size,
                           0x1, 0x0, 0x0, qbman_p_addr >> 32,
                           qbman_p_addr, qbman_p_size);
    qemu_fdt_setprop_cells(fdt, node, "reg", mcaddr >> 32, mcaddr,
                           mcsize >> 32, mcsize);
    ret = qemu_fdt_setprop(fdt, node, "interrupts",
                           irq_attr, 64*3*sizeof(uint32_t));
    if (ret) {
        error_report("could not set interrupts property of node %s", node);
    }
    g_free(irq_attr);
    g_free(node);
}

static void fsl_mc_bus_fdt_notify(Notifier *notifier, void *data)
{
    FSLMCBusFDTNotifierParams *p = DO_UPCAST(FSLMCBusFDTNotifierParams,
                                                notifier, notifier);

    add_fsl_mc_bus_fdt_node(p->fdt_params);
    g_free(p->fdt_params);
    g_free(p);
}

void fsl_register_mc_bus_fdt_creator(FSLMCBusFDTParams *fdt_params)
{
    FSLMCBusFDTNotifierParams *p = g_new(FSLMCBusFDTNotifierParams, 1);

    p->fdt_params = fdt_params;
    p->notifier.notify = fsl_mc_bus_fdt_notify;
    qemu_add_machine_init_done_notifier(&p->notifier);
}
