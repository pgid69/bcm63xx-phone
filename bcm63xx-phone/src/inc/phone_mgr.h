/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __PHONES_H__
#define __PHONES_H__

#include "config.h"

#ifdef __KERNEL__
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#endif // __KERNEL__

#include <phone.h>
#include <timer.h>
#include <wait_queue.h>

typedef struct {
   // Index of device into phone_devices[] of phone_mgr.
   size_t index_dev;
   // Index of line used to refer to the line with the phone device
   size_t index_line;

   // Direct pointer to the phone_line.
   // Avoid to ask to the device each time we need to access the phone_line
   phone_line_t *line;

   // All the fields inside the following structure are shared with
   // the timer function : the timer function access these fields
   // with the phone_mgr.lock acquired, so they can be accessed outside
   // the timer function only by acquiring the phone_mgr.lock too.
   struct {
      // Status of the line, reset by call to get_line_states.
      bcm_phone_line_state_t line_state;
      // Bits field to tell if codec or mode field must be changed
      // New value are in 'new_mode' and 'new_codec' below
      enum {
         NO_CHANGE_ASKED = 0,
         CHANGE_ASKED = 0x01,
         CHANGE_ASKED_ASAP = 0x02,
         CHANGE_PENDING = 0x04
      } change_status;
      bcm_phone_codec_t new_codec;
      bcm_phone_line_mode_t new_mode;
      __u32 new_tone;
      // A copy of line_state.codec but that we can access without getting the spinlock
      atomic_t current_codec;
      // A copy of line_state.mode but that we can access without getting the spinlock
      atomic_t current_mode;
      // A copy of line_state.tone but that we can access without getting the spinlock
      atomic_t current_tone;
   } shared;
} bcm_phone_mgr_line_t;

typedef struct bcm_phones {
   // Board description
   const board_desc_t *board_desc;
   // Wait queue used to block processes waiting for an event occuring in
   // timer function
   bcm_wait_queue_t eventq;
   // Timer used to periodically update the state of the phone lines
   bcm_timer_t timer;
   // Work and workqueue scheduled by the timer
   struct work_struct work;
   struct workqueue_struct *workqueue;
   // Callback called at the end of the timer function
   void (*timer_cb)(struct bcm_phones *t);
   // Lock used to arbitrate access to shared data
   spinlock_t lock;
   // Phone devices
   size_t phone_dev_count;
   struct {
      // The phone device
      phone_device_t *dev;
      // The number of lines enabled
      size_t line_enable_count;
   } phone_devices[BCMPH_MAX_DEVICES];
   // Phone lines
   size_t phone_line_count;
   bcm_phone_mgr_line_t phone_lines[BCMPH_MAX_LINES];
} bcm_phone_mgr_t;

extern int bcm_phone_mgr_init(bcm_phone_mgr_t *t, const board_desc_t *board_desc,
   void (*timer_cb)(struct bcm_phones *t));

extern void bcm_phone_mgr_deinit(bcm_phone_mgr_t *t);

static inline size_t bcm_phone_mgr_get_line_count(const bcm_phone_mgr_t *t)
{
   return (t->phone_line_count);
}

static inline bcm_wait_queue_t *bcm_phone_mgr_get_eventq(bcm_phone_mgr_t *t)
{
   return (&(t->eventq));
}

extern int bcm_phone_mgr_start(bcm_phone_mgr_t *t, bcmph_country_t country,
   const phone_line_params_t *lps, size_t lps_count);

extern void bcm_phone_mgr_stop(bcm_phone_mgr_t *t);

static inline bool bcm_phone_mgr_line_is_enabled(const bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

   ret = phone_line_is_enabled(t->phone_lines[line].line);
   dd_bcm_pr_debug("bcm_phone_mgr_line_is_enabled(line=%lu) => %d\n", (unsigned long)(line), (int)(ret));
   return (ret);
}

extern __u8 bcm_phone_mgr_get_first_timeslot_line(const bcm_phone_mgr_t *t, size_t line);

extern bool bcm_phone_mgr_line_supports_codec(const bcm_phone_mgr_t *t, size_t line, bcm_phone_codec_t codec);

extern bool bcm_phone_mgr_line_can_switch_to_codec(const bcm_phone_mgr_t *t, size_t line,
   bcm_phone_codec_t old_codec, bcm_phone_codec_t new_codec);

static inline bcm_phone_codec_t bcm_phone_mgr_get_current_codec(const bcm_phone_mgr_t *t, size_t line)
{
   bcm_phone_codec_t ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_codec_t)(atomic_read(&(t->phone_lines[line].shared.current_codec))));

   dd_bcm_pr_debug("bcm_phone_mgr_get_current_codec(line=%lu) => %d\n", (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bcm_phone_line_mode_t bcm_phone_mgr_get_current_mode(const bcm_phone_mgr_t *t, size_t line)
{
   bcm_phone_line_mode_t ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_line_mode_t)(atomic_read(&(t->phone_lines[line].shared.current_mode))));

   dd_bcm_pr_debug("bcm_phone_mgr_get_current_mode(line=%lu) => %d\n", (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bcm_phone_line_tone_t bcm_phone_mgr_get_current_tone(const bcm_phone_mgr_t *t, size_t line)
{
   bcm_phone_line_tone_t ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_line_tone_t)(atomic_read(&(t->phone_lines[line].shared.current_tone))));

   dd_bcm_pr_debug("bcm_phone_mgr_get_current_tone(line=%lu) => %d\n", (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_line_rx_use_pcm(const bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

   if (BCMPH_MODE_OFF_TALKING == bcm_phone_mgr_get_current_mode(t, line)) {
      bcm_assert(phone_line_is_enabled(t->phone_lines[line].line));
      ret = true;
   }
   else {
      ret = false;
   }
   dd_bcm_pr_debug("bcm_phone_mgr_line_use_pcm(line=%lu) => %d\n", (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_line_tx_use_pcm(const bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

   /*
    There's an amibiguity when generating tone : voice can't be heard
    but do we have to suspend sending voice data on the PCM bus ?
   */
   if ((BCMPH_MODE_OFF_TALKING == bcm_phone_mgr_get_current_mode(t, line))
       /* && (BCMPH_TONE_NONE == bcm_phone_mgr_get_current_tone(t, line)) */) {
      bcm_assert(phone_line_is_enabled(t->phone_lines[line].line));
      ret = true;
   }
   else {
      ret = false;
   }
   dd_bcm_pr_debug("bcm_phone_mgr_line_use_pcm(line=%lu) => %d\n", (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_line_has_change_pending(bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

   spin_lock_bh(&(t->lock));
   // When timer is stopped, change_status is reset to NO_CHANGED_ASKED
   // so no need to test that timer is active
   if (NO_CHANGE_ASKED == t->phone_lines[line].shared.change_status) {
      ret = true;
   }
   else {
      ret = false;
   }
   spin_unlock_bh(&(t->lock));

   return (ret);
}

extern void bcm_phone_mgr_get_line_states(bcm_phone_mgr_t *t,
   bcm_phone_line_state_t *line_states, size_t line_states_len);

#ifdef BCMPH_TEST_PCM
extern int bcm_phone_mgr_set_line_state(bcm_phone_mgr_t *t,
   const bcm_phone_set_line_state_t *set_line_state);
#endif // BCMPH_TEST_PCM

extern int bcm_phone_mgr_set_line_codec(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_codec_t codec, bcm_phone_line_mode_t mode,
   __u32 tone, int wait, bcmph_mutex_t *lock);

extern int bcm_phone_mgr_set_line_mode(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_line_mode_t mode, __u32 tone, int wait, bcmph_mutex_t *lock);

extern int bcm_phone_mgr_set_line_tone(bcm_phone_mgr_t *t, size_t line,
   __u32 tone, int wait, bcmph_mutex_t *lock);

#endif // __PHONES_H__
