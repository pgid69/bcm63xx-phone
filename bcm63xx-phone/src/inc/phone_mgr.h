/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __PHONES_H__
#define __PHONES_H__

#include "config.h"

#include <extern/linux/mutex.h>
#include <extern/linux/spinlock.h>
#include <extern/linux/wait.h>
#include <extern/linux/workqueue.h>

#include <phone.h>
#include <timer.h>
#include <wait_queue.h>

enum {
   NO_CHANGE_ASKED = 0,
   CHANGE_ASKED = 0x01,
   CHANGE_ASKED_ASAP = 0x02,
   CHANGE_PENDING = 0x04
};

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
      // New value are in 'new_mode' and 'new_codec'... below
      unsigned int change_status;
      bcm_phone_codec_t new_codec;
      bool new_rev_polarity;
      bcm_phone_line_mode_t new_mode;
      __u32 new_tone;
      // A copy of line_state.codec but that we can access without getting the spinlock
      atomic_t current_codec;
      // A copy of line_state.rev_polarity but that we can access without getting the spinlock
      atomic_t current_rev_polarity;
      // A copy of line_state.mode but that we can access without getting the spinlock
      atomic_t current_mode;
      // A copy of line_state.tone but that we can access without getting the spinlock
      atomic_t current_tone;
   } shared;
} bcm_phone_mgr_line_t;

typedef struct bcm_phones {
   // Board description
   const board_desc_t *board_desc;
   // Periodic timer used to schedule the periodic_work
   bcm_periodic_timer_t *periodic_timer;
   // Workqueue used to execute periodic_work
   struct workqueue_struct *wq;
   // Periodic work used to update line state...
   bcm_periodic_work_t periodic_work;
   // Callback called at the end of the periodic work
   void (*timer_cb)(struct bcm_phones *t);
#ifdef BCMPH_EXPORT_DEV_FILE
   struct {
      // Wait queue used to block processes waiting for an event occuring in
      // periodic_work
      bcm_wait_queue_t eventq;
      // Lock used to arbitrate access to shared data
      spinlock_t lock;
   } dev;
#endif /* BCMPH_EXPORT_DEV_FILE */
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

extern int bcm_phone_mgr_init(bcm_phone_mgr_t *t,
   const board_desc_t *board_desc,
   bcm_periodic_timer_t *periodic_timer, struct workqueue_struct *wq,
   void (*timer_cb)(struct bcm_phones *t));

extern void bcm_phone_mgr_deinit(bcm_phone_mgr_t *t);

static inline size_t bcm_phone_mgr_get_line_count(const bcm_phone_mgr_t *t)
{
   return (t->phone_line_count);
}

#ifdef BCMPH_EXPORT_DEV_FILE
static inline bcm_wait_queue_t *bcm_phone_mgr_get_eventq(bcm_phone_mgr_t *t)
{
   return (&(t->dev.eventq));
}
#endif /* BCMPH_EXPORT_DEV_FILE */

extern int bcm_phone_mgr_start(bcm_phone_mgr_t *t, bcmph_country_t country,
   const phone_line_params_t *lps, size_t lps_count);

extern void bcm_phone_mgr_stop(bcm_phone_mgr_t *t);

static inline bool bcm_phone_mgr_line_is_enabled(const bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

   ret = phone_line_is_enabled(t->phone_lines[line].line);
   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

extern __u8 bcm_phone_mgr_get_first_timeslot_line(const bcm_phone_mgr_t *t, size_t line);

extern bool bcm_phone_mgr_line_supports_codec(const bcm_phone_mgr_t *t, size_t line, bcm_phone_codec_t codec);

extern bool bcm_phone_mgr_line_can_switch_to_codec(const bcm_phone_mgr_t *t, size_t line,
   bcm_phone_codec_t old_codec, bcm_phone_codec_t new_codec);

static inline bcm_phone_codec_t bcm_phone_mgr_get_current_codec(bcm_phone_mgr_t *t, size_t line)
{
   bcm_phone_codec_t ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_codec_t)(atomic_read(&(t->phone_lines[line].shared.current_codec))));

   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_get_current_rev_polarity(bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_line_mode_t)(atomic_read(&(t->phone_lines[line].shared.current_rev_polarity))));

   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bcm_phone_line_mode_t bcm_phone_mgr_get_current_mode(bcm_phone_mgr_t *t, size_t line)
{
   bcm_phone_line_mode_t ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_line_mode_t)(atomic_read(&(t->phone_lines[line].shared.current_mode))));

   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bcm_phone_line_tone_t bcm_phone_mgr_get_current_tone(bcm_phone_mgr_t *t, size_t line)
{
   bcm_phone_line_tone_t ret;

   bcm_assert(line < t->phone_line_count);

   ret = ((bcm_phone_line_tone_t)(atomic_read(&(t->phone_lines[line].shared.current_tone))));

   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_line_rx_use_pcm(bcm_phone_mgr_t *t, size_t line)
{
   bool ret;
   bcm_phone_line_mode_t mode;

   bcm_assert(line < t->phone_line_count);

   mode = bcm_phone_mgr_get_current_mode(t, line);
   if (BCMPH_MODE_OFF_TALKING == mode) {
      bcm_assert(phone_line_is_enabled(t->phone_lines[line].line));
      ret = true;
   }
   else {
      ret = false;
   }
   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_line_tx_use_pcm(bcm_phone_mgr_t *t, size_t line)
{
   bool ret;
   bcm_phone_line_mode_t mode;

   bcm_assert(line < t->phone_line_count);

   /*
    There's an amibiguity when generating tone : voice can't be heard
    but do we have to suspend sending voice data on the PCM bus ?
   */
   mode = bcm_phone_mgr_get_current_mode(t, line);
   if ((BCMPH_MODE_OFF_TALKING == mode) || (BCMPH_MODE_ON_TALKING == mode)
       /* && (BCMPH_TONE_NONE == bcm_phone_mgr_get_current_tone(t, line)) */) {
      bcm_assert(phone_line_is_enabled(t->phone_lines[line].line));
      ret = true;
   }
   else {
      ret = false;
   }
   dd_bcm_pr_debug("%s(line=%lu) => %d\n", __func__, (unsigned long)(line), (int)(ret));
   return (ret);
}

static inline bool bcm_phone_mgr_line_has_change_pending(bcm_phone_mgr_t *t, size_t line)
{
   bool ret;

   bcm_assert(line < t->phone_line_count);

#ifdef BCMPH_EXPORT_DEV_FILE
   spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
   // When timer is stopped, change_status is reset to NO_CHANGED_ASKED
   // so no need to test that timer is active
   if (NO_CHANGE_ASKED == t->phone_lines[line].shared.change_status) {
      ret = true;
   }
   else {
      ret = false;
   }
#ifdef BCMPH_EXPORT_DEV_FILE
   spin_unlock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */

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
   int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern int bcm_phone_mgr_set_line_rev_polarity(bcm_phone_mgr_t *t, size_t line,
   bool rev_polarity
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern int bcm_phone_mgr_set_line_mode(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_line_mode_t mode, int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern int bcm_phone_mgr_set_line_tone(bcm_phone_mgr_t *t, size_t line,
   __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

#endif // __PHONES_H__
