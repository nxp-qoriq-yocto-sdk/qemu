/*
 * FSL Management Complex driver
 *
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Bharat Bhushan <bharat.bhushan@freescale.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 */

#if !defined(FSL_MC_FSL_MC_H)
#define FSL_MC_FSL_MC_H

#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"

/* Range within all MC portals fit in */
#define FSLMC_MC_PORTALS_RANGE_SIZE 0x4000000
/* Size of each MC Portal */
#define FSLMC_MC_PORTAL_SIZE 0x10000
/* Range within all QBMAN portals fit in */
#define FSLMC_QBMAN_PORTALS_RANGE_SIZE 0x8000000
/* Size of each QBMAN Portal */
#define FSLMC_QBMAN_PORTAL_SIZE 0x10000

struct FslMcBusState;

#define TYPE_FSL_MC_BUS "fsl-mc-bus"
#define FSL_MC_BUS(obj) OBJECT_CHECK(FslMcBusState, (obj), TYPE_FSL_MC_BUS)

struct FslMcBusState {
    BusState qbus;

    QLIST_HEAD(, FslMcDeviceState) device_list;
};
typedef struct FslMcBusState FslMcBusState;

#define TYPE_FSL_MC_HOST "fsl-mc-host"
#define FSL_MC_HOST(obj) OBJECT_CHECK(FslMcHostState, (obj), TYPE_FSL_MC_HOST)

typedef struct FslMcHostState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    FslMcBusState bus;
    MemoryRegion mc_portal;
    MemoryRegion qbman_portal;
    uint64_t mc_bus_base_addr;
    uint64_t mc_portals_range_offset;
    uint64_t mc_portals_range_size;
    uint64_t qbman_portals_range_offset;
    uint64_t qbman_portals_range_size;
    uint64_t qbman_portals_ce_offset;
    uint64_t qbman_portals_ci_offset;
} FslMcHostState;

typedef struct FslMcHostClass {
    DeviceClass parent_class;
} FslMcHostClass;

#define TYPE_FSL_MC_DEVICE "fsl-mc-device"
#define FSL_MC_DEVICE(obj) OBJECT_CHECK(FslMcDeviceState, (obj), TYPE_FSL_MC_DEVICE)
#define FSL_MC_DEVICE_CLASS(klass) \
         OBJECT_CLASS_CHECK(FslMcDeviceClass, (klass), TYPE_FSL_MC_DEVICE)
#define FSL_MC_DEVICE_GET_CLASS(obj) \
        OBJECT_GET_CLASS(FslMcDeviceClass, (obj), TYPE_FSL_MC_DEVICE)


typedef struct FslMcDeviceState {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/
    DeviceState qdev;
    FslMcBusState *bus;
    bool root_dprc;
    uint16_t dprc_id;
    QLIST_ENTRY(FslMcDeviceState) next;
} FslMcDeviceState;

typedef struct FslMcDeviceClass {
    DeviceClass parent_class;

    void (*realize)(FslMcDeviceState *dev, Error **errp);
    int (*init)(FslMcDeviceState *mcdev);
    int (*exit)(FslMcDeviceState *mcdev);
    uint16_t vendor_id;
    uint16_t device_id;
} FslMcDeviceClass;

int fsl_mc_register_device(FslMcDeviceState *mcdev, int region_num,
                           MemoryRegion *mem, MemoryRegion *mmap_mem,
                           char *name, uint16_t id);
int fsl_mc_get_portals_ranges(hwaddr *mc_p_addr, hwaddr *mc_p_size,
                              hwaddr *qbman_p_addr, hwaddr *qbman_p_size);
int fsl_mc_get_root_mcp_addr_range(hwaddr *mc_p_addr, hwaddr *mc_p_size);
#endif /* !defined(FSL_MC_FSL_MC_H) */
