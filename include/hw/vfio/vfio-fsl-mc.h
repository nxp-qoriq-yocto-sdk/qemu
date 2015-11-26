/*
 * vfio based device assignment support -Freescale Management Complex devices
 *
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Bharat Bhushan,     <Bharat.Bhushan@freescale.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Based on vfio based PCI device assignment support:
 *  Copyright Red Hat, Inc. 2012
 */

#ifndef HW_VFIO_VFIO_FSL_MC_H
#define HW_VFIO_VFIO_FSL_MC_H

#include "hw/sysbus.h"
#include "hw/vfio/vfio-common.h"

#define TYPE_VFIO_FSL_MC "vfio-fsl-mc"

typedef struct VFIOFslmcDevice {
    FslMcDeviceState mcdev;
    VFIODevice vbasedev; /* not a QOM object */
    VFIORegion **regions;
    char name[10];
    uint16_t id;
    int32_t bootindex;
} VFIOFslmcDevice;

typedef struct VFIOFslmcDeviceClass {
    /*< private >*/
    SysBusDeviceClass parent_class;
    /*< public >*/
} VFIOFslmcDeviceClass;

#define VFIO_FSL_MC_DEVICE(obj) \
     OBJECT_CHECK(VFIOFslmcDevice, (obj), TYPE_VFIO_FSL_MC)
#define VFIO_FSL_MC_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(VFIOFslmcDeviceClass, (klass), TYPE_VFIO_FSL_MC)
#define VFIO_FSL_MC_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(VFIOFslmcDeviceClass, (obj), TYPE_VFIO_FSL_MC)

#endif /*HW_VFIO_VFIO_FSL_MC_H*/
