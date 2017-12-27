/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __PHONE_H__
#define __PHONE_H__

#include "config.h"

#include <extern/linux/atomic.h>
#include <extern/linux/stddef.h>

#include <vp_api_types.h>

#include "board.h"
#include "bcm63xx_log.h"

typedef struct {
   // Tell if line is enabled
   bool enable;
   // First timeslot used to transmit data to the line
   // if codec is linear, the line uses the next timeslot
   // if codec is linear16, the line also uses two other timeslots
   __u8 timeslot;
   // Line state
   bcm_phone_line_state_t line_state;
   // Flag to tell if line_state has changed
   bool line_state_changed;
   // New codec asked
   bcm_phone_codec_t new_codec;
   // New polarity
   bool new_rev_polarity;
   // New mode asked
   bcm_phone_line_mode_t new_mode;
   // New tone asked (eventually with times on and off)
   __u32 new_tone;
} phone_line_t;

static inline void phone_line_init(phone_line_t *t)
{
   d_bcm_pr_debug("%s()\n", __func__);
   t->enable = false;
   bcm_phone_line_state_init(&(t->line_state));
   t->line_state_changed = false;
   t->new_codec = t->line_state.codec;
   t->new_rev_polarity = t->line_state.rev_polarity;
   t->new_mode = t->line_state.mode;
   t->new_tone = bcm_phone_line_tone_code_index(t->line_state.tone);
}

static inline void phone_line_deinit(phone_line_t *t)
{
   d_bcm_pr_debug("%s()\n", __func__);
   t->enable = false;
   t->new_tone = bcm_phone_line_tone_code_index(t->line_state.tone);
   t->new_mode = t->line_state.mode;
   t->new_rev_polarity = t->line_state.rev_polarity;
   t->new_codec = t->line_state.codec;
   t->line_state_changed = false;
   bcm_phone_line_state_deinit(&(t->line_state));
}

static inline void phone_line_enable(phone_line_t *t,
   bcm_phone_codec_t codec, __u8 timeslot,
   bcm_phone_line_status_t initial_status,
   bcm_phone_line_mode_t initial_mode)
{
   d_bcm_pr_debug("%s(codec=%d, timeslot=%u)\n", __func__,
      (int)(codec), (unsigned int)(timeslot));
   t->timeslot = timeslot;
   bcm_phone_line_state_reset(&(t->line_state), initial_status, codec,
      false, initial_mode, BCMPH_TONE_NONE);
   t->new_codec = t->line_state.codec;
   t->new_rev_polarity = t->line_state.rev_polarity;
   t->new_mode = t->line_state.mode;
   t->new_tone = bcm_phone_line_tone_code_index(t->line_state.tone);
   t->line_state_changed = false;
   t->enable = true;
}

static inline void phone_line_disable(phone_line_t *t)
{
   d_bcm_pr_debug("%s()\n", __func__);
   t->enable = false;
   bcm_phone_line_state_reset(&(t->line_state), BCMPH_STATUS_UNSPECIFIED,
      BCMPH_CODEC_LINEAR, false, BCMPH_MODE_DISCONNECT, BCMPH_TONE_NONE);
   t->new_codec = t->line_state.codec;
   t->new_mode = t->line_state.mode;
   t->new_rev_polarity = t->line_state.rev_polarity;
   t->new_tone = bcm_phone_line_tone_code_index(t->line_state.tone);
   t->line_state_changed = false;
}

static inline bool phone_line_is_enabled(const phone_line_t *t)
{
   bool ret = t->enable;
   dd_bcm_pr_debug("%s() -> %d\n", __func__, (int)(ret));
   return (ret);
}

static inline void phone_line_line_state_changed(phone_line_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   t->line_state_changed = true;
}

static inline void phone_line_move_line_state(phone_line_t *t, bcm_phone_line_state_t *line_state)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_phone_line_state_move(&(t->line_state), line_state);
   t->line_state_changed = false;
}

typedef struct {
   __u8 enable;
   bcm_phone_codec_t codec;
   __u8 first_timeslot;
} phone_line_params_t;

typedef struct {
   const struct vtbl_phone_device *vtbl;
   const phone_desc_device_t *desc;
   __u8 tick_period;
   phone_line_t *lines[BCMPH_MAX_LINES_PER_DEV];
   bcmph_country_t country;
   bool started;
} phone_device_t;

typedef struct vtbl_phone_device {
   // Function deinit must return a pointer on the start of the object
   void *(*deinit)(phone_device_t *dev);
   int (*start)(phone_device_t *dev, bcmph_country_t country,
      const phone_line_params_t * const *line_params, size_t line_count);
   void (*stop)(phone_device_t *dev);
   void (*tick)(phone_device_t *dev);
   void (*update_line_state_asap)(phone_device_t *dev, size_t index);
} vtbl_phone_device_t;

extern phone_device_t *phone_device_alloc(
   const phone_desc_device_t *dev_desc, __u8 tick_period);

extern void phone_device_free(phone_device_t *d);

extern void phone_device_init(phone_device_t *t,
   const vtbl_phone_device_t *vtbl,
   const phone_desc_device_t *desc, __u8 tick_period);

extern void phone_device_deinit(phone_device_t *t);

static inline void phone_device_start(phone_device_t *t, bcmph_country_t country)
{
   d_bcm_pr_debug("%s(country=%d)\n", __func__, (int)(country));
   t->country = country;
   t->started = true;
}

static inline void phone_device_stop(phone_device_t *t)
{
   d_bcm_pr_debug("%s()\n", __func__);
   t->started = false;
}

static inline bool phone_device_is_started(const phone_device_t *t)
{
   bool ret = t->started;
   dd_bcm_pr_debug("%s() -> %d\n", __func__, (int)(ret));
   return (ret);
}

static inline bcmph_country_t phone_device_get_country(const phone_device_t *t)
{
   bcmph_country_t ret = t->country;
   d_bcm_pr_debug("%s() -> %d\n", __func__, (int)(ret));
   return (ret);
}

static inline const phone_desc_device_t *phone_device_get_desc(const phone_device_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   return (t->desc);
}

static inline __u8 phone_device_get_tick_period(const phone_device_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   return (t->tick_period);
}

static inline size_t phone_device_get_line_count(const phone_device_t *t)
{
   size_t ret = t->desc->line_count;
   dd_bcm_pr_debug("%s() -> %lu\n", __func__, (unsigned long)(ret));
   return (ret);
}

static inline const phone_desc_line_t *phone_device_get_line_desc(const phone_device_t *t, size_t index)
{
   dd_bcm_pr_debug("%s(index=%lu)\n", __func__, (unsigned long)(index));
   bcm_assert(index < t->desc->line_count);
   return (&(t->desc->lines[index]));
}

static inline phone_line_t *phone_device_get_line(const phone_device_t *t, size_t index)
{
   dd_bcm_pr_debug("%s(index=%lu)\n", __func__, (unsigned long)(index));
   bcm_assert((index < t->desc->line_count) && (index < ARRAY_SIZE(t->lines)));
   return (t->lines[index]);
}

static inline void phone_device_init_line(phone_device_t *t, size_t index, phone_line_t *vl)
{
   dd_bcm_pr_debug("%s(index=%lu)\n", __func__, (unsigned long)(index));
   bcm_assert((index < t->desc->line_count) && (index < ARRAY_SIZE(t->lines)));
   t->lines[index] = vl;
}

static inline void phone_device_deinit_line(phone_device_t *t, size_t index)
{
   dd_bcm_pr_debug("%s(index=%lu)\n", __func__, (unsigned long)(index));
   bcm_assert((index < t->desc->line_count) && (index < ARRAY_SIZE(t->lines)));
   t->lines[index] = NULL;
}

#endif // __PHONE_H__
