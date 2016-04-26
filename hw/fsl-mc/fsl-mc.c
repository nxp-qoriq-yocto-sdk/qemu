/*
 * FSL Management Complex driver
 *
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Bharat Bhushan, <bharat.bhushan@freescale.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 * *****************************************************************
 *
 */

#include "fsl-mc.h"

static Property fsl_mc_props[] = {
    DEFINE_PROP_UINT64("mc_bus_base_addr", FslMcHostState, mc_bus_base_addr, 0),
    DEFINE_PROP_UINT32("mc_bus_num_irqs", FslMcHostState, mc_bus_num_irqs, 0),
    DEFINE_PROP_UINT64("mc_portals_range_offset", FslMcHostState,
                       mc_portals_range_offset, 0),
    DEFINE_PROP_UINT64("mc_portals_range_size", FslMcHostState,
                       mc_portals_range_size, 0),
    DEFINE_PROP_UINT64("qbman_portals_range_offset", FslMcHostState,
                       qbman_portals_range_offset, 0),
    DEFINE_PROP_UINT64("qbman_portals_range_size", FslMcHostState,
                       qbman_portals_range_size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

int fsl_mc_get_portals_ranges(hwaddr *mc_p_addr, hwaddr *mc_p_size,
                              hwaddr *qbman_p_addr, hwaddr *qbman_p_size)
{
    DeviceState *dev;
    FslMcHostState *host;

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_FSL_MC_HOST);
    host = FSL_MC_HOST(dev);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    *mc_p_addr = host->mc_bus_base_addr +  host->mc_portals_range_offset;
    *mc_p_size = host->mc_portals_range_size;
    *qbman_p_addr = host->mc_bus_base_addr + host->qbman_portals_range_offset;
    *qbman_p_size = host->qbman_portals_range_size;
    return 0;
}

static FslMcDeviceState *find_root_dprc_device(FslMcBusState *bus)
{
    FslMcDeviceState *mcdev = NULL;

    QLIST_FOREACH(mcdev, &bus->device_list, next) {
        if(mcdev->root_dprc == true) {
            return mcdev;
        }
    }
    return NULL;
}

int fsl_mc_get_root_mcp_addr_range(hwaddr *mc_p_addr, hwaddr *mc_p_size)
{
    DeviceState *dev;
    FslMcHostState *host;
    FslMcDeviceState *mcdev = NULL;
    hwaddr addr;

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_FSL_MC_HOST);
    host = FSL_MC_HOST(dev);

    mcdev = find_root_dprc_device(&host->bus);
    if (mcdev == NULL) {
	return -1;
    }

    /* Get to the Base of MC-Portal */
    addr = host->mc_bus_base_addr + host->mc_portals_range_offset;
    /* Add the Mc-portal device offset */
    addr += mcdev->offset;
    *mc_p_addr = addr;
    *mc_p_size = 0x40;
    return 0;
}

int fsl_mc_register_device(FslMcDeviceState *mcdev, int region_num,
                           MemoryRegion *mem, MemoryRegion *mmap_mem,
                           char *name, uint16_t id, off_t offset)
{
    FslMcBusState *bus;
    FslMcHostState *host;
    MemoryRegion *portal = NULL;
    static bool root_dprc_probed = false;
    FslMcDeviceState *tmp;
    bool found = false;

    bus = mcdev->bus;
    if (bus == NULL) {
        fprintf(stderr, "No FSL-MC Bus found\n");
        return -ENODEV;
    }

    host = FSL_MC_HOST(bus->qbus.parent);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    if (strncmp(name, "dprc", 10) == 0) {
        if (root_dprc_probed == false) {
            mcdev->root_dprc = true;
            root_dprc_probed = true;
            mcdev->dprc_id = id;
        } else {
            fprintf(stderr, "Only One Root DPRC can exists\n");
        }
    }

    /* Hack to calculate the device offset address */
    offset &= 0x00FFFFFF;
    mcdev->offset = offset;

    if (strncmp(name, "dprc", 10) == 0) {
        portal = &host->mc_portal;
	if (offset > host->mc_portals_range_size) {
            return -EINVAL;
        }
    } else if (strncmp(name, "dpmcp", 10) == 0) {
        portal = &host->mc_portal;
	if (offset > host->mc_portals_range_size) {
            return -EINVAL;
        }
    } else if (strncmp(name, "dpio", 10) == 0) {
        portal = &host->qbman_portal;
        if (region_num) {
            offset += host->qbman_portals_ce_offset;
        } else {
            offset += host->qbman_portals_ci_offset;
        }

	if (offset > host->qbman_portals_range_size) {
            return -EINVAL;
        }
    } else {
        fprintf(stderr, "%s: Error No Matching device(%s) found \n",
                __func__, name);
        return -EINVAL;
    }

    memory_region_add_subregion(portal, offset, mem);
    memory_region_set_enabled(mmap_mem, true);

    QLIST_FOREACH(tmp, &bus->device_list, next) {
        if (tmp == mcdev) {
            found = true;
            break;
         }
    }
    if (found == false) {
        QLIST_INSERT_HEAD(&bus->device_list, mcdev, next);
    }

    return 0;
}

int fsl_mc_connect_irq(FslMcDeviceState *mcdev, int irq_num,
                       char *name, uint16_t id)
{
    DeviceState *dev;
    FslMcBusState *bus;
    FslMcHostState *host;
    SysBusDevice *d = NULL;
    int max_irqs;
    int irq_index;

    bus = mcdev->bus;
    if (bus == NULL) {
        fprintf(stderr, "No FSL-MC Bus found\n");
        return -ENODEV;
    }

    host = FSL_MC_HOST(bus->qbus.parent);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_FSL_MC_HOST);
    d = SYS_BUS_DEVICE(dev);
    if (d == NULL) {
        fprintf(stderr, "sysbus device not found\n");
        return -ENODEV;
    }

    /* Get mapped device irq */
    irq_index = mcdev->irq_map[irq_num];
    if ((irq_index != 0) && test_bit(irq_index, host->used_irqs)) {
        /* IRQ is already mapped, nothing to do */
        fprintf(stderr, "sysbus irq already mapped\n");
        return 0;
    }

    max_irqs = host->mc_bus_num_irqs;
    irq_index = find_first_zero_bit(host->used_irqs, max_irqs);
    if (irq_index >= max_irqs) {
        hw_error("Platform Bus: Can not fit IRQ line");
        return -ENODEV;
    }

    set_bit(irq_index, host->used_irqs);
    /* Map device IRQ-Num to allocated irq index */
    mcdev->irq_map[irq_num] = 176 + irq_index;

    return 0;
}

int fsl_mc_assert_irq(FslMcDeviceState *mcdev, int irq_num)
{
    DeviceState *dev;
    SysBusDevice *d = NULL;
    FslMcBusState *bus;
    FslMcHostState *host;
    int irq_index;

    bus = mcdev->bus;
    if (bus == NULL) {
        fprintf(stderr, "No FSL-MC Bus found\n");
        return -ENODEV;
    }

    host = FSL_MC_HOST(bus->qbus.parent);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_FSL_MC_HOST);
    d = SYS_BUS_DEVICE(dev);
    if (d == NULL) {
        fprintf(stderr, "sysbus device not found\n");
        return -ENODEV;
    }

    /* Get mapped device irq */
    irq_index = mcdev->irq_map[irq_num];
    irq_index -= 176;

    if (!(test_bit(irq_index, host->used_irqs)) ||
        !sysbus_is_irq_connected(d, irq_index)) {
        return -ENODEV;
    }

    qemu_set_irq(host->irqs[irq_index], 1);
    return 0;
}

static int fsl_mc_qdev_init(DeviceState *qdev)
{
    FslMcDeviceState *mcdev = (FslMcDeviceState *)qdev;
    FslMcDeviceClass *mc_dc = FSL_MC_DEVICE_GET_CLASS(mcdev);
    int ret;

    if (mc_dc->init) {
        ret = mc_dc->init(mcdev);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

static void fsl_mc_dev_realize(DeviceState *qdev, Error **errp)
{
    FslMcDeviceState *mcdev = (FslMcDeviceState *)qdev;
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_GET_CLASS(mcdev);
    FslMcBusState *bus;
    Error *local_err = NULL;

    bus = FSL_MC_BUS(qdev_get_parent_bus(qdev));
    mcdev->bus = bus;

    if (mcdc->realize) {
        mcdc->realize(mcdev, &local_err);
        if (local_err) {
            return;
        }
    }
}

static void fsl_mc_default_realize(FslMcDeviceState *mcdev, Error **errp)
{
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_GET_CLASS(mcdev);

    if (mcdc->init) {
        if (mcdc->init(mcdev) < 0) {
            error_setg(errp, "Device initialization failed");
            return;
        }
    }
}

static void fsl_mc_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_CLASS(klass);

    dc->bus_type = TYPE_FSL_MC_BUS;
    dc->realize = fsl_mc_dev_realize;
    dc->init = fsl_mc_qdev_init;
//    dc->exit = fsl_mc_qdev_exit;
    mcdc->realize = fsl_mc_default_realize;
}

static const TypeInfo fsl_mc_device_info = {
    .name          = TYPE_FSL_MC_DEVICE,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(FslMcDeviceState),
    .class_size = sizeof(FslMcDeviceClass),
    .class_init    = fsl_mc_device_class_init,
};

static uint64_t fsl_mc_portal_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    fprintf(stderr, "%s: Addr = %lx, Size = %d\n", __func__, addr, size);
    return 0;
}
static void fsl_mc_portal_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned size)
{
    fprintf(stderr, "%s: Addr = %lx, Size = %d\n", __func__, addr, size);
}

static const MemoryRegionOps fsl_mc_portal_ops = {
    .read = fsl_mc_portal_read,
    .write = fsl_mc_portal_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void fsl_mc_host_initfn(Object *obj)
{
    FslMcHostState *s = FSL_MC_HOST(obj);
    DeviceState *ds = DEVICE(obj);

    qbus_create_inplace(&s->bus, sizeof(s->bus), TYPE_FSL_MC_BUS, ds, NULL);
    QLIST_INIT(&s->bus.device_list);
}

static void fsl_mc_host_realize(DeviceState *dev, Error **errp)
{
    FslMcHostState *s = FSL_MC_HOST(dev);
    SysBusDevice *d = SYS_BUS_DEVICE(dev);
    int i;

    if (s == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return;
    }

    s->qbman_portals_ci_offset = 0x0;
    s->qbman_portals_ce_offset = s->qbman_portals_ci_offset + 0x4000000;

    memory_region_init_io(&s->mc_portal, OBJECT(s), &fsl_mc_portal_ops, s,
                          "fsl_mc portal", s->mc_portals_range_size);
    sysbus_init_mmio(d, &s->mc_portal);

    memory_region_init_io(&s->qbman_portal, OBJECT(s), NULL, s,
                          "fsl_qbman portal", s->qbman_portals_range_size);
    sysbus_init_mmio(d, &s->qbman_portal);

    s->used_irqs = bitmap_new(s->mc_bus_num_irqs);
    s->irqs = g_new0(qemu_irq, s->mc_bus_num_irqs);
    for (i = 0; i < s->mc_bus_num_irqs; i++) {
        sysbus_init_irq(d, &s->irqs[i]);
    }
}

static void fsl_mc_host_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->props = fsl_mc_props;
    dc->realize = fsl_mc_host_realize;
}

static const TypeInfo fsl_mc_host_info = {
    .name          = TYPE_FSL_MC_HOST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FslMcHostState),
    .instance_init = fsl_mc_host_initfn,
    .class_init    = fsl_mc_host_class_init,
};

static const TypeInfo fsl_mc_bus_info = {
    .name = TYPE_FSL_MC_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(FslMcBusState),
};

static void fsl_mc_register_types(void)
{
    type_register_static(&fsl_mc_bus_info);
    type_register_static(&fsl_mc_host_info);
    type_register_static(&fsl_mc_device_info);
}

type_init(fsl_mc_register_types)
