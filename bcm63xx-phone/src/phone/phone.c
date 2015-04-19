/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifdef __KERNEL__
#include <linux/delay.h>
#else // !__KERNEL__
#include <stdlib.h>
#endif // !__KERNEL__

#include <mpi.h>
#include <phone.h>
#include "zarlink/le88221/slic_le88221.h"
#include "zarlink/le88266/slic_le88266.h"

// Include after system files
#include <compile.h>

void phone_device_init(phone_device_t *t,
   const vtbl_phone_device_t *vtbl,
   const phone_desc_device_t *desc, __u8 tick_period)
{
   size_t i;

   bcm_pr_debug("phone_device_init()\n");

   bcm_assert((NULL != vtbl) && (NULL != desc)
      && (desc->line_count <= ARRAY_SIZE(t->lines)));

   t->vtbl = vtbl;
   t->desc = desc;
   t->tick_period = tick_period;
   t->started = false;
   t->country = BCMPH_COUNTRY_ETSI;
   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
      t->lines[i] = NULL;
   }
}

void phone_device_deinit(phone_device_t *t)
{
   bcm_pr_debug("phone_device_deinit()\n");
}

phone_device_t * __init phone_device_alloc(
   const phone_desc_device_t *dev_desc, __u8 tick_period)
{
   phone_device_t *ret = NULL;

   bcm_pr_debug("phone_device_alloc()\n");

   switch (dev_desc->type) {
#ifdef VP_CC_880_SERIES
      case BCMPH_VD_ZARLINK_88221: {
         phone_dev_le88221_t *dev = (phone_dev_le88221_t *)(kmalloc(sizeof(phone_dev_le88221_t), GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY));
         if (NULL == dev) {
            bcm_pr_err("Cannot allocate %lu bytes for device Zarlink Le88221\n",
               (unsigned long)(sizeof(phone_dev_le88221_t)));
         }
         else {
            bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(sizeof(phone_dev_le88221_t)), (unsigned long)(dev));
            if (phone_dev_le88221_init(dev, dev_desc, tick_period)) {
               kfree(dev);
            }
            else {
               ret = &(dev->ve880.vdz.vd);
               bcm_assert(((void *)(ret)) == ((void *)(dev)));
            }
         }
         break;
      }
      case BCMPH_VD_ZARLINK_88266: {
         phone_dev_le88266_t *dev = (phone_dev_le88266_t *)(kmalloc(sizeof(phone_dev_le88266_t), GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY));
         if (NULL == dev) {
            bcm_pr_err("Cannot allocate %lu bytes for device Zarlink Le88266\n",
               (unsigned long)(sizeof(phone_dev_le88266_t)));
         }
         else {
            bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(sizeof(phone_dev_le88266_t)), (unsigned long)(dev));
            if (phone_dev_le88266_init(dev, dev_desc, tick_period)) {
               kfree(dev);
            }
            else {
               ret = &(dev->ve880.vdz.vd);
               bcm_assert(((void *)(ret)) == ((void *)(dev)));
            }
         }
         break;
      }
#endif // VP_CC_880_SERIES
      default: {
         bcm_pr_err("Unknow phone device %d\n", (int)(dev_desc->type));
         break;
      }
   }

   return (ret);
}

void phone_device_free(phone_device_t *t)
{
   bcm_pr_debug("phone_device_free()\n");

   if (NULL != t) {
      void *p = (*(t->vtbl->deinit))(t);
      kfree(p);
   }
}

