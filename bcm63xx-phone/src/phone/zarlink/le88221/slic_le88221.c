/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#include "slic_le88221.h"

#ifdef VP_CC_880_SERIES

#include <bcm63xx_log.h>

// Include after system files
#include <compile.h>

static void *le88221_deinit(phone_device_t *d)
{
   size_t i;
   phone_dev_le88221_t *t = container_of(d, phone_dev_le88221_t, ve880.vdz.vd);

   bcm_pr_debug("le88221_deinit()\n");

   phone_dev_zarlink_ve880_stop(&(t->ve880.vdz.vd));

   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
      phone_dev_zarlink_deinit_line(&(t->ve880.vdz), i);
      phone_line_deinit(&(t->lines[i].vl));
   }

   phone_dev_zarlink_ve880_deinit(&(t->ve880));

   return (t);
}

static vtbl_phone_dev_zarlink_t vtbl_le88221 = {
   .vd = {
      .deinit = le88221_deinit,
      .start = phone_dev_zarlink_ve880_start,
      .stop = phone_dev_zarlink_ve880_stop,
      .tick = phone_dev_zarlink_tick,
      .update_line_state_asap = phone_dev_zarlink_update_line_state_asap
   },
};

int __init phone_dev_le88221_init(phone_dev_le88221_t *t,
   const phone_desc_device_t *dev_desc, __u8 tick_period)
{
   int ret = 0;
   size_t i;

   bcm_pr_debug("phone_dev_le88221_init()\n");

   bcm_assert(ARRAY_SIZE(t->lines) <= ARRAY_SIZE(t->ve880.vdz.lines));

   bcm_assert((NULL != dev_desc) && (NULL != dev_desc->parameters.zarlink));
   for (i = 0; (i < dev_desc->line_count); i += 1) {
      bcm_assert(NULL != dev_desc->lines[i].parameters.zarlink);
   }

   do { // Empty loop
      if (dev_desc->line_count > ARRAY_SIZE(t->lines)) {
         bcm_pr_err("Le88221 description can only have %lu lines at most\n", (unsigned long)(ARRAY_SIZE(t->lines)));
         ret = -ENODEV;
         break;
      }

      ret = phone_dev_zarlink_ve880_init(&(t->ve880), &(vtbl_le88221), dev_desc, tick_period);
      if (ret) {
         break;
      }

      for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
         phone_line_init(&(t->lines[i].vl));
         memset(&(t->lines[i].obj), 0, sizeof(t->lines[i].obj));
         memset(&(t->lines[i].ctx), 0, sizeof(t->lines[i].ctx));
         phone_dev_zarlink_init_line(&(t->ve880.vdz), i,
            &(t->lines[i].vl), &(t->lines[i].obj), &(t->lines[i].ctx));
      }
   }
   while (false);

   return (ret);
}

#endif // VP_CC_880_SERIES
