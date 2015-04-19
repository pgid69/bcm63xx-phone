/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __SLIC_LE88266_H__    /* support nested includes */
#define __SLIC_LE88266_H__

#include <config.h>

#include "../zarlink_common.h"

#ifdef VP_CC_880_SERIES

#define LE88266_NUM_LINES 2

typedef struct {
   phone_dev_zarlink_ve880_t ve880;
   struct {
      phone_line_t vl;
      VpLineCtxType ctx;
      Vp880LineObjectType obj;
   } lines[LE88266_NUM_LINES];
} phone_dev_le88266_t;

extern int phone_dev_le88266_init(phone_dev_le88266_t *dev,
   const phone_desc_device_t *dev_desc, __u8 tick_period);

#endif // VP_CC_880_SERIES

#endif   /* __SLIC_LE88266_H__ */
