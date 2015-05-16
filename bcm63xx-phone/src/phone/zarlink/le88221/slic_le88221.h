/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __SLIC_LE88221_H__    /* support nested includes */
#define __SLIC_LE88221_H__

#include <config.h>

#include "../zarlink_common.h"

#ifdef VP_CC_880_SERIES

#define LE88221_NUM_LINES 2

typedef struct {
   phone_dev_zarlink_ve880_t ve880;
   struct {
      phone_line_t vl;
      VpLineCtxType ctx;
      Vp880LineObjectType obj;
   } lines[LE88221_NUM_LINES];
} phone_dev_le88221_t;

extern int phone_dev_le88221_init(phone_dev_le88221_t *dev,
   const phone_desc_device_t *dev_desc, __u8 tick_period);

#endif // VP_CC_880_SERIES

#endif   /* __SLIC_LE88221_H__ */
