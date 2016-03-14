/*
 * vfio based device assignment support -Freescale Management Complex devices
 *
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Bharat Bhushan, <Bharat.Bhushan@freescale.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Based on vfio based PCI device assignment support:
 *  Copyright Red Hat, Inc. 2012
 */

#include <linux/vfio.h>
#include <sys/ioctl.h>

#include "hw/fsl-mc/fsl-mc.h"
#include "hw/vfio/vfio-fsl-mc.h"
#include "qemu/error-report.h"
#include "qemu/range.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"
#include "qemu/queue.h"
#include "sysemu/kvm.h"

/* VFIO skeleton */

/* not implemented yet */
static void vfio_fsl_mc_compute_needs_reset(VFIODevice *vbasedev)
{
    vbasedev->needs_reset = false;
}

/* not implemented yet */
static int vfio_fsl_mc_hot_reset_multi(VFIODevice *vbasedev)
{
    return 0;
}

static VFIO_LINE_IRQ *vfio_init_line_irq(VFIODevice *vbasedev,
                                         struct vfio_irq_info info)
{
    int ret;
    VFIO_LINE_IRQ *line_irq;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);

    line_irq = g_malloc0(sizeof(*line_irq));
    line_irq->vdev = vdev;
    line_irq->pin = info.index;
    line_irq->flags = info.flags;

    ret = event_notifier_init(&line_irq->interrupt, 0);
    if (ret) {
        g_free(line_irq);
        error_report("vfio: Error: trigger event_notifier_init failed ");
        return NULL;
    }

    QLIST_INSERT_HEAD(&vdev->irq_list, line_irq, next);
    return line_irq;
}

/**
 * vfio_populate_device - Allocate and populate MMIO region
 * and IRQ structs according to driver returned information
 * @vbasedev: the VFIO device handle
 *
 */
static int vfio_populate_device(VFIODevice *vbasedev)
{
    int i, ret = -1;
    VFIO_LINE_IRQ *line_irq, *tmp;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);

    if (!(vbasedev->flags & VFIO_DEVICE_FLAGS_FSL_MC)) {
        error_report("vfio: Um, this isn't a fsl_mc device");
        return ret;
    }

    vdev->regions = g_malloc0_n(1,
                                sizeof(VFIORegion *) * vbasedev->num_regions);

    for (i = 0; i < vbasedev->num_regions; i++) {
        struct vfio_region_info reg_info = { .argsz = sizeof(reg_info) };
        VFIORegion *ptr;

        vdev->regions[i] = g_malloc0_n(1, sizeof(VFIORegion));
        ptr = vdev->regions[i];
        reg_info.index = i;
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_REGION_INFO, &reg_info);
        if (ret) {
            error_report("vfio: Error getting region %d info: %m", i);
            goto reg_error;
        }
        ptr->flags = reg_info.flags;
        ptr->size = reg_info.size;
        ptr->fd_offset = reg_info.offset;
        ptr->nr = i;
        ptr->vbasedev = vbasedev;
    }

    for (i = 0; i < vbasedev->num_irqs; i++) {
        struct vfio_irq_info irq = { .argsz = sizeof(irq) };

        irq.index = i;
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
        if (ret) {
            error_printf("vfio: error getting device %s irq info",
                         vbasedev->name);
            goto irq_err;
        }

        line_irq = vfio_init_line_irq(vbasedev, irq);
        if (!line_irq) {
            error_report("vfio: Error installing IRQ %d up", i);
            goto irq_err;
        }
    }

    return 0;
irq_err:
    QLIST_FOREACH_SAFE(line_irq, &vdev->irq_list, next, tmp) {
        QLIST_REMOVE(line_irq, next);
        g_free(line_irq);
    }

reg_error:
    for (i = 0; i < vbasedev->num_regions; i++) {
        g_free(vdev->regions[i]);
    }
    g_free(vdev->regions);
    return ret;
}

/* specialized functions for VFIO FSL-MC devices */
static VFIODeviceOps vfio_fsl_mc_ops = {
    .vfio_compute_needs_reset = vfio_fsl_mc_compute_needs_reset,
    .vfio_hot_reset_multi = vfio_fsl_mc_hot_reset_multi,
};

/**
 * vfio_base_device_init - perform preliminary VFIO setup
 * @vbasedev: the VFIO device handle
 *
 * Implement the VFIO command sequence that allows to discover
 * assigned device resources: group extraction, device
 * fd retrieval, resource query.
 * Precondition: the device name must be initialized
 */
static int vfio_base_device_init(VFIODevice *vbasedev)
{
    VFIOGroup *group;
    VFIODevice *vbasedev_iter;
    char path[PATH_MAX], iommu_group_path[PATH_MAX], *group_name;
    ssize_t len;
    struct stat st;
    int groupid;
    int ret;

    /* name must be set prior to the call */
    if (!vbasedev->name) {
        return -EINVAL;
    }

    /* Check that the host device exists */
    g_snprintf(path, sizeof(path), "/sys/bus/fsl-mc/devices/%s/",
               vbasedev->name);

    if (stat(path, &st) < 0) {
        error_report("vfio: error: no such host device: %s", path);
        return -errno;
    }

    g_strlcat(path, "iommu_group", sizeof(path));
    len = readlink(path, iommu_group_path, sizeof(iommu_group_path));
    if (len < 0) {
        error_report("vfio: error no iommu_group for device");
        return -errno;
    }

    iommu_group_path[len] = 0;
    group_name = basename(iommu_group_path);

    if (sscanf(group_name, "%d", &groupid) != 1) {
        error_report("vfio: error reading %s: %m", path);
        return -errno;
    }

    group = vfio_get_group(groupid, &address_space_memory);
    if (!group) {
        error_report("vfio: failed to get group %d", groupid);
        return -ENOENT;
    }

    g_snprintf(path, sizeof(path), "%s", vbasedev->name);

    QLIST_FOREACH(vbasedev_iter, &group->device_list, next) {
        if (strcmp(vbasedev_iter->name, vbasedev->name) == 0) {
            error_report("vfio: error: device %s is already attached", path);
            vfio_put_group(group);
            return -EBUSY;
        }
    }

    ret = vfio_get_device(group, path, vbasedev);
    if (ret) {
        error_report("vfio: failed to get device %s", path);
        vfio_put_group(group);
        return ret;
    }

    ret = vfio_populate_device(vbasedev);
    if (ret) {
        error_report("vfio: failed to populate device %s", path);
        vfio_put_group(group);
    }

    return ret;
}

/**
 * vfio_map_region - initialize the 2 memory regions for a given
 * MMIO region index
 * @vdev: the VFIO fsl_mc device handle
 * @nr: the index of the region
 *
 * Init the top memory region and the mmapped memory region beneath
 * VFIOFslmcDevice is used since VFIODevice is not a QOM Object
 * and could not be passed to memory region functions
*/
static void vfio_map_region(VFIOFslmcDevice *vdev, int nr)
{
    VFIORegion *region = vdev->regions[nr];
    unsigned size = region->size;
    char name[64];

    if (!size) {
        return;
    }

    g_snprintf(name, sizeof(name), "VFIO %s region %d",
               vdev->vbasedev.name, nr);

    /* A "slow" read/write mapping underlies all regions */
    memory_region_init_io(&region->mem, OBJECT(vdev), &vfio_region_ops,
                          region, name, size);

    g_strlcat(name, " mmap", sizeof(name));

    if (vfio_mmap_region(OBJECT(vdev), region, &region->mem,
                         &region->mmap_mem, &region->mmap, size, 0, name)) {
        error_report("%s unsupported. Performance may be slow", name);
    }
}

static void vfio_fsl_mc_reset(DeviceState *dev)
{
    VFIOFslmcDevice *vdev = VFIO_FSL_MC_DEVICE(dev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    int ret;

    if (vbasedev->reset_works) {
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_RESET);
        if (ret) {
            printf("fails to reset device err(%d)\n", ret);
        }
     }
}

static void vfio_fsl_mc_irq_handler(VFIO_LINE_IRQ *line_irq)
{
    int ret;
    VFIOFslmcDevice *vdev = line_irq->vdev;

    ret = event_notifier_test_and_clear(&line_irq->interrupt);
    if (!ret) {
        error_report("Error when clearing fd=%d (ret = %d)\n",
                     event_notifier_get_fd(&line_irq->interrupt), ret);
    }

    /* trigger the virtual IRQ */
    fsl_mc_assert_irq(&vdev->mcdev, line_irq->pin);
}

/**
 * vfio_set_trigger_eventfd - set VFIO eventfd handling
 *
 * Setup VFIO signaling and attach an optional user-side handler
 * to the eventfd
 */
static int vfio_set_trigger_eventfd(VFIO_LINE_IRQ *line_irq,
                                    eventfd_user_side_handler_t handler)
{
    VFIODevice *vbasedev = &line_irq->vdev->vbasedev;
    struct vfio_irq_set *irq_set;
    int argsz, ret;
    int32_t *pfd;

    argsz = sizeof(*irq_set) + sizeof(*pfd);
    irq_set = g_malloc0(argsz);
    irq_set->argsz = argsz;
    irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
    irq_set->index = line_irq->pin;
    irq_set->start = 0;
    irq_set->count = 1;
    irq_set->irq_num = line_irq->hw_irq_line;
    pfd = (int32_t *)&irq_set->data;

    *pfd = event_notifier_get_fd(&line_irq->interrupt);

    qemu_set_fd_handler(*pfd, (IOHandler *)handler, NULL, line_irq);

    ret = ioctl(vbasedev->fd, VFIO_DEVICE_SET_IRQS, irq_set);
    g_free(irq_set);
    if (ret < 0) {
        error_report("vfio: Failed to set trigger eventfd: %m");
        qemu_set_fd_handler(*pfd, NULL, NULL, NULL);
    }
    return ret;
}

static int vfio_set_kvm_irqfd(VFIOFslmcDevice *vdev, VFIO_LINE_IRQ *line_irq, int hwirq)
{
    struct kvm_irqfd irqfd = {
        .fd = event_notifier_get_fd(&line_irq->interrupt),
        .gsi = hwirq,
        .flags = 0,
    };

    if (!kvm_irqfds_enabled()) {
        printf("%s: irqfd not supported, kvm_irqfds_enabled = %d\n", __func__, kvm_irqfds_enabled());
        return 0;
    }

    qemu_set_fd_handler(irqfd.fd, NULL, NULL, vdev);

    vfio_mask_single_irqindex(&vdev->vbasedev, line_irq->pin);

   if (kvm_vm_ioctl(kvm_state, KVM_IRQFD, &irqfd)) {
        error_report("vfio: Error: Failed to setup resample irqfd: %m");
        return 0;
    }

    vfio_set_trigger_eventfd(line_irq, NULL);

    /* Let's resume injection with irqfd setup */
    vfio_unmask_single_irqindex(&vdev->vbasedev, line_irq->pin);

    return 0;
}

static int vfio_fsl_mc_initfn(FslMcDeviceState *mcdev)
{
    VFIOFslmcDevice *vdev = DO_UPCAST(VFIOFslmcDevice, mcdev, mcdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    VFIO_LINE_IRQ *line_irq;
    int i, ret;
    char *temp;

    vbasedev->type = VFIO_DEVICE_TYPE_FSL_MC;
    vbasedev->ops = &vfio_fsl_mc_ops;

    ret = vfio_base_device_init(vbasedev);
    if (ret) {
        return ret;
    }

    strncpy(vdev->name, vbasedev->name, 10);
    temp = strchr(vdev->name, '.');
    *temp = '\0';
    temp++;
    vdev->id = atoi(temp);

    for (i = 0; i < vbasedev->num_regions; i++) {
        vfio_map_region(vdev, i);
        ret = fsl_mc_register_device(mcdev, i, &vdev->regions[i]->mem,
                               &vdev->regions[i]->mmap_mem,
                               vdev->name, vdev->id,
                               vdev->regions[i]->fd_offset);
        if (ret) {
            return ret;
        }
    }

    for (i = 0; i < vbasedev->num_irqs; i++) {
        ret = fsl_mc_connect_irq(mcdev, i, vdev->name, vdev->id);
        if (ret) {
            printf("Failed to connect irq for device %s.%d\n",
                   vdev->name, vdev->id);
            return ret;
        }

        QLIST_FOREACH(line_irq, &vdev->irq_list, next) {
            if (line_irq->pin == i) {
                line_irq->hw_irq_line = mcdev->irq_map[i];

                ret = vfio_set_kvm_irqfd(vdev, line_irq, mcdev->irq_map[i]);
                if (ret) {
                    printf("Failed to setup irqfd for device  %s.%d\n",
                           vdev->name, vdev->id);
                    vfio_set_trigger_eventfd(line_irq, vfio_fsl_mc_irq_handler);
                }
            }
        }
    }

    return 0;
}

static void vfio_fsl_mc_instance_init(Object *obj)
{
    FslMcDeviceState *mcdev = FSL_MC_DEVICE(obj);
    VFIOFslmcDevice *vdev = DO_UPCAST(VFIOFslmcDevice, mcdev, mcdev);

    device_add_bootindex_property(obj, &vdev->bootindex,
                                  "bootindex", NULL,
                                  &mcdev->qdev, NULL);
}

static const VMStateDescription vfio_fsl_mc_vmstate = {
    .name = TYPE_VFIO_FSL_MC,
    .unmigratable = 1,
};

static Property vfio_fsl_mc_dev_properties[] = {
    DEFINE_PROP_STRING("host", VFIOFslmcDevice, vbasedev.name),
    DEFINE_PROP_BOOL("x-mmap", VFIOFslmcDevice, vbasedev.allow_mmap, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void vfio_fsl_mc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_CLASS(klass);

    /* Reset is called after _initfn() and we can not allow reset after
     * _initfn() as interrupts are setup in _initfn() but a reset to DPRC
     * will cleanup interrupt configuration in MC.
     */
//    dc->reset = vfio_fsl_mc_reset;
    dc->props = vfio_fsl_mc_dev_properties;
    dc->vmsd = &vfio_fsl_mc_vmstate;
    dc->desc = "VFIO-based fsl_mc device assignment";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
//    mcdc->exit = vfio_fsl_mc_exitfn;
    mcdc->init = vfio_fsl_mc_initfn;
}

static const TypeInfo vfio_fsl_mc_dev_info = {
    .name = TYPE_VFIO_FSL_MC,
    .parent = TYPE_FSL_MC_DEVICE,
    .instance_size = sizeof(VFIOFslmcDevice),
    .class_init = vfio_fsl_mc_class_init,
    .instance_init = vfio_fsl_mc_instance_init,
//    .class_size = sizeof(VFIOFslmcDeviceClass),
};

static void register_vfio_fsl_mc_dev_type(void)
{
    type_register_static(&vfio_fsl_mc_dev_info);
}

type_init(register_vfio_fsl_mc_dev_type)
