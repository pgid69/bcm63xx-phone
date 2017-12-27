/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/barrier.h>
#include <extern/linux/errno.h>
#include <extern/linux/stddef.h>

#include <phone_mgr.h>

static inline void bcm_phone_mgr_update_from_ls(bcm_phone_mgr_line_t *t)
{
   t->shared.change_status = NO_CHANGE_ASKED;
   t->shared.new_codec = t->shared.line_state.codec;
   t->shared.new_rev_polarity = t->shared.line_state.rev_polarity;
   t->shared.new_mode = t->shared.line_state.mode;
   t->shared.new_tone = bcm_phone_line_tone_code_index(t->shared.line_state.tone);
   atomic_set(&(t->shared.current_codec), t->shared.line_state.codec);
   atomic_set(&(t->shared.current_rev_polarity), t->shared.line_state.rev_polarity);
   atomic_set(&(t->shared.current_mode), t->shared.line_state.mode);
   atomic_set(&(t->shared.current_tone), t->shared.line_state.tone);
}

static void bcm_phone_mgr_line_init(bcm_phone_mgr_line_t *t)
{
   t->line = NULL;
   bcm_phone_line_state_init(&(t->shared.line_state));
   bcm_phone_mgr_update_from_ls(t);
}

static void bcm_phone_mgr_line_deinit(bcm_phone_mgr_line_t *t)
{
   t->line = NULL;
   bcm_phone_line_state_deinit(&(t->shared.line_state));
   bcm_phone_mgr_update_from_ls(t);
}

static void bcm_phone_mgr_line_reset(bcm_phone_mgr_line_t *t)
{
   bcm_phone_line_state_reset(&(t->shared.line_state),
      BCMPH_STATUS_UNSPECIFIED, BCMPH_CODEC_LINEAR, false,
      BCMPH_MODE_DISCONNECT, BCMPH_TONE_NONE);
   bcm_phone_mgr_update_from_ls(t);
}

static void bcm_phone_mgr_line_init_from_line(bcm_phone_mgr_line_t *t)
{
   phone_line_move_line_state(t->line, &(t->shared.line_state));
   bcm_phone_mgr_update_from_ls(t);
}

static bool bcm_phone_mgr_mode_allow_tones(bcm_phone_line_mode_t mode)
{
   bool ret;
   if ((BCMPH_MODE_IDLE == mode) || (BCMPH_MODE_OFF_TALKING == mode)) {
      ret = true;
   }
   else {
      ret = false;
   }
   return (ret);
}

#ifdef BCMPH_EXPORT_DEV_FILE
static int bcm_phone_mgr_wait_for_change_completion(bcm_phone_mgr_t *t,
   const bcm_phone_mgr_line_t *pl, int wq_counter,
   int wait, bcmph_mutex_t *lock)
{
   int ret = 0;

   d_bcm_pr_debug("%s(wait=%d)\n", __func__, (int)(wait));

   if (wait < 0) {
      for (;;) {
         ret = bcm_wait_queue_wait_event_counter(&(t->dev.eventq),
            wq_counter, lock);
         if (ret < 0) {
            break;
         }
         barrier();
         // We update the counter before doing the verification
         // so we are sure not to loose an event between the verification
         // and when we start waiting for an event
         wq_counter = bcm_wait_queue_get_counter(&(t->dev.eventq));
         spin_lock_bh(&(t->dev.lock));
         // When timer is stopped, change_status is reset to NO_CHANGED_ASKED
         // so no need to test timer is active
         if (NO_CHANGE_ASKED == pl->shared.change_status) {
            spin_unlock_bh(&(t->dev.lock));
            ret = 0;
            break;
         }
         spin_unlock_bh(&(t->dev.lock));
      }
   }
   else {
      long timeout_in_jiffies = msecs_to_jiffies(wait);
      ret = 1;
      for (;;) {
         long remaining = bcm_wait_queue_wait_event_counter_timeout(&(t->dev.eventq),
            wq_counter, timeout_in_jiffies, lock);
         if (remaining < 0) {
            ret = remaining;
            break;
         }
         barrier();
         // We update the counter before doing the verification
         // so we are sure not to loose an event between the verification
         // and we wait for an event
         wq_counter = bcm_wait_queue_get_counter(&(t->dev.eventq));
         spin_lock_bh(&(t->dev.lock));
         // When timer is stopped, change_status is reset to NO_CHANGED_ASKED
         // so no need to test timer is active
         if (NO_CHANGE_ASKED == pl->shared.change_status) {
            spin_unlock_bh(&(t->dev.lock));
            ret = 0;
            break;
         }
         spin_unlock_bh(&(t->dev.lock));
         timeout_in_jiffies = remaining;
         if (timeout_in_jiffies <= 0) {
            break;
         }
      }
   }

   return (ret);
}
#endif /* BCMPH_EXPORT_DEV_FILE */

__u8 bcm_phone_mgr_get_first_timeslot_line(const bcm_phone_mgr_t *t, size_t line)
{
   const bcm_phone_mgr_line_t *pl;
   const phone_device_t *dev;
   const phone_desc_line_t *desc;

   bcm_pr_debug("%s(line=%lu)\n", __func__, (unsigned long)(line));

   bcm_assert(line < t->phone_line_count);

   pl = &(t->phone_lines[line]);
   dev = t->phone_devices[pl->index_dev].dev;
   desc = phone_device_get_line_desc(dev, pl->index_line);

   return (desc->first_timeslot);
}

bool bcm_phone_mgr_line_supports_codec(const bcm_phone_mgr_t *t, size_t line, bcm_phone_codec_t codec)
{
   bool ret = false;

   d_bcm_pr_debug("%s(line=%lu, codec=%d)\n", __func__, (unsigned long)(line), (int)(codec));

   bcm_assert(line < t->phone_line_count);

   do { // Empty loop
      const bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
      const phone_device_t *dev = t->phone_devices[pl->index_dev].dev;
      const phone_desc_device_t *desc = phone_device_get_desc(dev);

      if (BCMPH_CODEC_ALAW == codec) {
         // Check that device support ALAW codec
         if (0 == (desc->caps & BCMPH_CAPS_ALAW_CODEC)) {
            break;
         }
      }
      else if (BCMPH_CODEC_ULAW == codec) {
         // Check that device support ULAW codec
         if (0 == (desc->caps & BCMPH_CAPS_ULAW_CODEC)) {
            break;
         }
      }
      else if (BCMPH_CODEC_LINEAR == codec) {
         // Check that device support LINEAR codec
         if (0 == (desc->caps & BCMPH_CAPS_LINEAR_CODEC)) {
            break;
         }
      }
      else if (BCMPH_CODEC_LINEAR16 == codec) {
         // Check that device support LINEAR16 codec
         if (0 == (desc->caps & BCMPH_CAPS_LINEAR16_CODEC)) {
            break;
         }
      }
      else if (BCMPH_CODEC_ALAW16 == codec) {
         // Check that device support ALAW16 codec
         if (0 == (desc->caps & BCMPH_CAPS_ALAW16_CODEC)) {
            break;
         }
      }
      else if (BCMPH_CODEC_ULAW16 == codec) {
         // Check that device support ULAW16 codec
         if (0 == (desc->caps & BCMPH_CAPS_ULAW16_CODEC)) {
            break;
         }
      }
      else {
         // Unsupported codec
         break;
      }

      ret = true;
   } while (false);

   return (ret);
}

static inline bool bcm_phone_mgr_codec_is_wideband(bcm_phone_codec_t codec)
{
   if ((BCMPH_CODEC_LINEAR16 == codec) || (BCMPH_CODEC_ALAW16 == codec)
       || (BCMPH_CODEC_ULAW16 == codec)) {
      return (true);
   }
   else {
      return (false);
   }
}

bool bcm_phone_mgr_line_can_switch_to_codec(const bcm_phone_mgr_t *t,
   size_t line, bcm_phone_codec_t old_codec, bcm_phone_codec_t new_codec)
{
   bool ret = false;

   d_bcm_pr_debug("%s(line=%lu, old_codec=%d, new_codec=%d)\n", __func__,
      (unsigned long)(line), (int)(old_codec), (int)(new_codec));

   bcm_assert(line < t->phone_line_count);

   do { /* Empty loop */
      const bcm_phone_mgr_line_t *pl;
      const phone_device_t *dev;
      const phone_desc_device_t *desc;

      if (bcm_phone_mgr_codec_is_wideband(old_codec)) {
         if (bcm_phone_mgr_codec_is_wideband(new_codec)) {
            ret = true;
            break;
         }
      }
      else {
         if (!bcm_phone_mgr_codec_is_wideband(new_codec)) {
            ret = true;
            break;
         }
      }

      pl = &(t->phone_lines[line]);
      dev = t->phone_devices[pl->index_dev].dev;
      desc = phone_device_get_desc(dev);
      if ((desc->caps & BCMPH_CAPS_CAN_SWITCH_BETWEEN_NB_AND_WB)) {
         bcm_assert((desc->caps & BCMPH_CAPS_CAN_MIX_NB_AND_WB));
         ret = true;
         break;
      }
   } while (false);

   return (ret);
}

int bcm_phone_mgr_set_line_codec(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_codec_t codec, bcm_phone_line_mode_t mode,
   int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   d_bcm_pr_debug("%s(line=%lu, codec=%d, mode=%d, tone=0x%lx"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (int)(codec), (int)(mode), (unsigned long)(tone)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   bcm_assert((line < t->phone_line_count)
      && (bcm_phone_mgr_line_supports_codec(t, line, codec))
      && (phone_line_is_enabled(t->phone_lines[line].line)));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
      bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);
#ifdef BCMPH_EXPORT_DEV_FILE
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->dev.eventq));
#endif /* BCMPH_EXPORT_DEV_FILE */

#ifdef BCMPH_EXPORT_DEV_FILE
      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
      pl->shared.new_codec = codec;
      if (BCMPH_MODE_UNSPECIFIED != mode) {
         if (mode < BCMPH_MAX_MODES) {
            pl->shared.new_mode = mode;
         }
         else {
            bcm_pr_debug("Invalid mode %d\n", (int)(mode));
         }
      }
      if (reverse_polarity >= 0) {
         pl->shared.new_rev_polarity = (reverse_polarity ? true : false);
      }
      if (BCMPH_TONE_UNSPECIFIED != tone_index) {
         if (!bcm_phone_mgr_mode_allow_tones(pl->shared.new_mode)) {
            if (BCMPH_TONE_NONE != tone_index) {
               bcm_pr_debug("Mode %d does not allow tone emission. Tone changed to %lu\n",
                  (int)(mode), (unsigned long)(bcm_phone_line_tone_code_index(BCMPH_TONE_NONE)));
            }
            pl->shared.new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
         }
         else if (tone_index < BCMPH_MAX_TONES) {
            pl->shared.new_tone = tone;
         }
         else {
            bcm_pr_debug("Invalid tone %lu\n", (unsigned long)(tone));
         }
      }
#ifdef BCMPH_EXPORT_DEV_FILE
      if (wait) {
#endif /* BCMPH_EXPORT_DEV_FILE */
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
#ifdef BCMPH_EXPORT_DEV_FILE
      }
      else {
         pl->shared.change_status |= CHANGE_ASKED;
      }
      spin_unlock_bh(&(t->dev.lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
   while (false);

   return (ret);
}

int bcm_phone_mgr_set_line_rev_polarity(bcm_phone_mgr_t *t, size_t line,
   bool rev_polarity
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   d_bcm_pr_debug("%s(line=%lu, rev_polarity=%d"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (int)(rev_polarity)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   bcm_assert((line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[line].line)));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
#ifdef BCMPH_EXPORT_DEV_FILE
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->dev.eventq));

      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
      pl->shared.new_rev_polarity = rev_polarity;
#ifdef BCMPH_EXPORT_DEV_FILE
      if (wait) {
#endif /* BCMPH_EXPORT_DEV_FILE */
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
#ifdef BCMPH_EXPORT_DEV_FILE
      }
      else {
         pl->shared.change_status |= CHANGE_ASKED;
      }
      spin_unlock_bh(&(t->dev.lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
   while (false);

   return (ret);
}

int bcm_phone_mgr_set_line_mode(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_line_mode_t mode, int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   d_bcm_pr_debug("%s(line=%lu, mode=%d, reverse_polarity=%d, tone=0x%lx"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (int)(mode),
      (int)(reverse_polarity), (unsigned long)(tone)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   bcm_assert((line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[line].line))
      && (((mode != BCMPH_MODE_UNSPECIFIED) && (mode < BCMPH_MAX_MODES))
          || (reverse_polarity >= 0)));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
#ifdef BCMPH_EXPORT_DEV_FILE
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->dev.eventq));
#endif /* BCMPH_EXPORT_DEV_FILE */
      bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);

#ifdef BCMPH_EXPORT_DEV_FILE
      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
      if (BCMPH_MODE_UNSPECIFIED != mode) {
         if (mode < BCMPH_MAX_MODES) {
            pl->shared.new_mode = mode;
         }
         else {
            bcm_pr_debug("Invalid mode %lu\n", (unsigned long)(mode));
         }
      }
      if (reverse_polarity >= 0) {
         pl->shared.new_rev_polarity = (reverse_polarity ? true : false);
      }
      if (BCMPH_TONE_UNSPECIFIED != tone_index) {
         if (!bcm_phone_mgr_mode_allow_tones(pl->shared.new_mode)) {
            if (BCMPH_TONE_NONE != tone_index) {
               bcm_pr_debug("Mode %d does not allow tone emission. Tone changed to %lu\n",
                  (int)(mode), (unsigned long)(bcm_phone_line_tone_code_index(BCMPH_TONE_NONE)));
            }
            pl->shared.new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
         }
         else if (tone_index < BCMPH_MAX_TONES) {
            pl->shared.new_tone = tone;
         }
         else {
            bcm_pr_debug("Invalid tone %lu\n", (unsigned long)(tone));
         }
      }
#ifdef BCMPH_EXPORT_DEV_FILE
      if (wait) {
#endif /* BCMPH_EXPORT_DEV_FILE */
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
#ifdef BCMPH_EXPORT_DEV_FILE
      }
      else {
         pl->shared.change_status |= CHANGE_ASKED;
      }
      spin_unlock_bh(&(t->dev.lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
   while (false);

   return (ret);
}

int bcm_phone_mgr_set_line_tone(bcm_phone_mgr_t *t, size_t line,
   __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;
#ifdef BCMPH_DEBUG
   bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);
#endif // BCMPH_DEBUG

   d_bcm_pr_debug("%s(line=%lu, tone=0x%lx"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (unsigned long)(tone)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   bcm_assert((line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[line].line))
      && (BCMPH_TONE_UNSPECIFIED != tone_index)
      && (tone_index < BCMPH_MAX_TONES));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
#ifdef BCMPH_EXPORT_DEV_FILE
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->dev.eventq));
#endif /* BCMPH_EXPORT_DEV_FILE */

#ifdef BCMPH_EXPORT_DEV_FILE
      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
      pl->shared.new_tone = tone;
#ifdef BCMPH_EXPORT_DEV_FILE
      if (wait) {
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
      }
      else {
#endif /* BCMPH_EXPORT_DEV_FILE */
         pl->shared.change_status |= CHANGE_ASKED;
#ifdef BCMPH_EXPORT_DEV_FILE
      }
      spin_unlock_bh(&(t->dev.lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
   while (false);

   return (ret);
}

#ifdef BCMPH_TEST_PCM
int bcm_phone_mgr_set_line_state(bcm_phone_mgr_t *t,
   const bcm_phone_set_line_state_t *set_line_state)
{
   int ret = 0;

   d_bcm_pr_debug("%s()\n", __func__);

   bcm_assert((NULL != set_line_state)
      && (set_line_state->line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[set_line_state->line].line)));

   do { // Empty loop
#ifdef BCMPH_EXPORT_DEV_FILE
      bool send_event = false;
#endif /* BCMPH_EXPORT_DEV_FILE */
      size_t digit_idx;
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[set_line_state->line]);
      bcm_phone_line_state_t *line_state = &(pl->shared.line_state);

#ifdef BCMPH_EXPORT_DEV_FILE
      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */

      // Set status
      if ((BCMPH_STATUS_UNSPECIFIED != set_line_state->status) && (set_line_state->status < BCMPH_MAX_STATUS)) {
         if (line_state->status != set_line_state->status) {
            line_state->status = set_line_state->status;
            line_state->status_change_count += 1;
#ifdef BCMPH_EXPORT_DEV_FILE
            send_event = true;
#endif /* BCMPH_EXPORT_DEV_FILE */
         }
      }

      // Add digits
      for (digit_idx = 0; ((digit_idx < set_line_state->digits_count) && (line_state->digits_count < ARRAY_SIZE(line_state->digits))); digit_idx += 1) {
         line_state->digits[line_state->digits_count] = set_line_state->digits[digit_idx];
         line_state->digits_count += 1;
      }
#ifdef BCMPH_EXPORT_DEV_FILE
      if (digit_idx > 0) {
         send_event = true;
      }
#endif /* BCMPH_EXPORT_DEV_FILE */

      // Add flash count
      if (set_line_state->flash_count > 0) {
         line_state->flash_count += set_line_state->flash_count;
#ifdef BCMPH_EXPORT_DEV_FILE
         send_event = true;
#endif /* BCMPH_EXPORT_DEV_FILE */
      }

#ifdef BCMPH_EXPORT_DEV_FILE
      spin_unlock_bh(&(t->dev.lock));

      if (send_event) {
         bcm_wait_queue_wake_up(&(t->dev.eventq));
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
   while (false);

   return (ret);
}
#endif // BCMPH_TEST_PCM

void bcm_phone_mgr_get_line_states(bcm_phone_mgr_t *t,
   bcm_phone_line_state_t *line_states, size_t line_states_len)
{
   size_t line_idx;

   dd_bcm_pr_debug("%s()\n", __func__);

   if (line_states_len > t->phone_line_count) {
      line_states_len = t->phone_line_count;
   }
#ifdef BCMPH_EXPORT_DEV_FILE
   spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
   for (line_idx = 0; (line_idx < line_states_len); line_idx += 1) {
      bcm_phone_line_state_move(&(t->phone_lines[line_idx].shared.line_state), &(line_states[line_idx]));
   }
#ifdef BCMPH_EXPORT_DEV_FILE
   spin_unlock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
}

static void bcm_phone_mgr_timer_work_fn(struct work_struct *work)
{
   bcm_phone_mgr_t *t = container_of(work, bcm_phone_mgr_t, periodic_work.work);
   int tick_count = bcm_periodic_work_get_tick_count(&(t->periodic_work));

   bcm_periodic_work_dec_tick_count(&(t->periodic_work), tick_count);

   dd_bcm_pr_debug("%s()\n", __func__);

   while (tick_count > 0) {
      size_t dev_idx;
      size_t line_idx;
#ifdef BCMPH_EXPORT_DEV_FILE
      bool send_event;
#endif /* BCMPH_EXPORT_DEV_FILE */

      tick_count -= 1;

#ifdef BCMPH_EXPORT_DEV_FILE
      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
      for (line_idx = 0; (line_idx < t->phone_line_count); line_idx += 1) {
         bcm_phone_mgr_line_t *pl = &(t->phone_lines[line_idx]);
         phone_line_t *line = pl->line;
         if (phone_line_is_enabled(line)) {
#ifdef BCMPH_TEST_PCM
            if (line->line_state.status != pl->shared.line_state.status) {
               line->line_state.status = pl->shared.line_state.status;
               line->new_rev_polarity = false;
               line->new_mode = BCMPH_MODE_IDLE;
               line->new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
            }
#endif /* BCMPH_TEST_PCM */
            if ((pl->shared.change_status & (CHANGE_ASKED | CHANGE_ASKED_ASAP))) {
               line->new_codec = pl->shared.new_codec;
               line->new_rev_polarity = pl->shared.new_rev_polarity;
               line->new_mode = pl->shared.new_mode;
               line->new_tone = pl->shared.new_tone;
               if ((pl->shared.change_status & CHANGE_ASKED_ASAP)) {
                  phone_device_t *dev = t->phone_devices[pl->index_dev].dev;
                  // Tell the device to update line_state as soon as possible but don't
                  // update the line now because we have the spinlock and we don't
                  // want to hold it for too long
                  (*(dev->vtbl->update_line_state_asap))(dev, pl->index_line);
               }
               pl->shared.change_status &= (~(CHANGE_ASKED | CHANGE_ASKED_ASAP));
               pl->shared.change_status |= CHANGE_PENDING;
            }
         }
      }
#ifdef BCMPH_EXPORT_DEV_FILE
      spin_unlock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */

      for (dev_idx = 0; (dev_idx < t->phone_dev_count); dev_idx += 1) {
         if (t->phone_devices[dev_idx].line_enable_count > 0) {
            (*(t->phone_devices[dev_idx].dev->vtbl->tick))(t->phone_devices[dev_idx].dev);
         }
      }

#ifdef BCMPH_EXPORT_DEV_FILE
      send_event = false;
      spin_lock_bh(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */
      for (line_idx = 0; (line_idx < t->phone_line_count); line_idx += 1) {
         bcm_phone_mgr_line_t *pl = &(t->phone_lines[line_idx]);
         phone_line_t *line = pl->line;
         if (phone_line_is_enabled(line)) {
            if (line->line_state_changed) {
               // We transfer the changes in the structure line_state
               // written and read by ioctl()
               phone_line_move_line_state(line, &(pl->shared.line_state));
               bcm_assert(!line->line_state_changed);
               atomic_set(&(pl->shared.current_codec), pl->shared.line_state.codec);
               atomic_set(&(pl->shared.current_rev_polarity), pl->shared.line_state.rev_polarity);
               atomic_set(&(pl->shared.current_mode), pl->shared.line_state.mode);
               atomic_set(&(pl->shared.current_tone), pl->shared.line_state.tone);
               // And we set a flag to wake up processes blocked in wait queue eventq
               // to tell them that at least one line state has changed
#ifdef BCMPH_EXPORT_DEV_FILE
               send_event = true;
#endif /* BCMPH_EXPORT_DEV_FILE */
            }
            if ((line->new_codec == line->line_state.codec)
                 && (line->new_rev_polarity == line->line_state.rev_polarity)
                 && (line->new_mode == line->line_state.mode)
                 && (bcm_phone_line_tone_decode_index(line->new_tone) == line->line_state.tone)) {
               if ((pl->shared.change_status & CHANGE_PENDING)) {
                  pl->shared.change_status &= (~(CHANGE_PENDING));
#ifdef BCMPH_EXPORT_DEV_FILE
                  send_event = true;
#endif /* BCMPH_EXPORT_DEV_FILE */
               }
            }
         }
      }
#ifdef BCMPH_EXPORT_DEV_FILE
      spin_unlock_bh(&(t->dev.lock));

      if (send_event) {
         bcm_wait_queue_wake_up(&(t->dev.eventq));
      }
#endif /* BCMPH_EXPORT_DEV_FILE */

      (*(t->timer_cb))(t);
   }
}

void bcm_phone_mgr_stop(bcm_phone_mgr_t *t)
{
   size_t dev_idx;
   size_t line_idx;

   bcm_pr_debug("%s()\n", __func__);

   bcm_periodic_timer_del_work(t->periodic_timer, &(t->periodic_work));
   cancel_work_sync(&(t->periodic_work.work));

   for (dev_idx = 0; (dev_idx < t->phone_dev_count); dev_idx += 1) {
      (*(t->phone_devices[dev_idx].dev->vtbl->stop))(t->phone_devices[dev_idx].dev);
   }

   // Before wake up process waiting on eventq
   // we reset phone_line in particular change_status
   for (line_idx = 0; (line_idx < t->phone_line_count); line_idx += 1) {
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line_idx]);
      phone_line_disable(pl->line);
      bcm_phone_mgr_line_reset(pl);
      bcm_assert(NO_CHANGE_ASKED == pl->shared.change_status);
   }
#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_wait_queue_wake_up(&(t->dev.eventq));
#endif /* BCMPH_EXPORT_DEV_FILE */
}

int bcm_phone_mgr_start(bcm_phone_mgr_t *t, bcmph_country_t country,
   const phone_line_params_t *lps, size_t lps_count)
{
   int ret = 0;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(country < BCMPH_COUNTRY_MAX);

   do { // Empty loop
      size_t dev_idx;
      size_t line_idx;

      for (dev_idx = 0; (dev_idx < t->phone_dev_count); dev_idx += 1) {
         t->phone_devices[dev_idx].line_enable_count = 0;
      }

      for (line_idx = 0; (line_idx < t->phone_line_count); line_idx += 1) {
         bcm_phone_mgr_line_t *pl = &(t->phone_lines[line_idx]);
         phone_line_disable(pl->line);
         bcm_phone_mgr_line_reset(pl);
         bcm_assert(NO_CHANGE_ASKED == pl->shared.change_status);
      }

      for (line_idx = 0; (line_idx < lps_count); line_idx += 1) {
         if (!lps[line_idx].enable) {
            continue;
         }
         bcm_assert((line_idx < t->phone_line_count) && (bcm_phone_mgr_line_supports_codec(t, line_idx, lps[line_idx].codec)));
         t->phone_devices[t->phone_lines[line_idx].index_dev].line_enable_count += 1;
      }

      for (dev_idx = 0; (dev_idx < t->phone_dev_count); dev_idx += 1) {
         if (t->phone_devices[dev_idx].line_enable_count > 0) {
            const phone_line_params_t *line_params[BCMPH_MAX_LINES_PER_DEV];
            size_t line_enabled;
            size_t are_narrowband;
            size_t are_wideband;

            bcm_assert(phone_device_get_line_count(t->phone_devices[dev_idx].dev) <= ARRAY_SIZE(line_params));

            for (line_idx = 0; (line_idx < ARRAY_SIZE(line_params)); line_idx += 1) {
               line_params[line_idx] = NULL;
            }

            line_enabled = 0;
            for (line_idx = 0; (line_idx < lps_count); line_idx += 1) {
               if ((lps[line_idx].enable) && (t->phone_lines[line_idx].index_dev == dev_idx)) {
                  bcm_assert((t->phone_lines[line_idx].index_line < ARRAY_SIZE(line_params))
                     && (NULL == line_params[t->phone_lines[line_idx].index_line]));
                  line_params[t->phone_lines[line_idx].index_line] = &(lps[line_idx]);
                  line_enabled += 1;
               }
            }
            bcm_assert(line_enabled == t->phone_devices[dev_idx].line_enable_count);

            // Now we check that all lines are wideband or not
            are_narrowband = 0;
            are_wideband = 0;
            for (line_idx = 0; (line_idx < ARRAY_SIZE(line_params)); line_idx += 1) {
               if (NULL != line_params[line_idx]) {
                  if (bcm_phone_mgr_codec_is_wideband(line_params[line_idx]->codec)) {
                     are_wideband += 1;
                  }
                  else {
                     are_narrowband += 1;
                  }
               }
            }
            if ((0 != are_wideband) && (0 != are_narrowband)) {
               const phone_desc_device_t *desc = phone_device_get_desc(t->phone_devices[dev_idx].dev);
               if (!(desc->caps & BCMPH_CAPS_CAN_MIX_NB_AND_WB)) {
                  bcm_assert(!(desc->caps & BCMPH_CAPS_CAN_SWITCH_BETWEEN_NB_AND_WB));
                  bcm_pr_err("All or none of the lines of the same device, can use wideband codec\n");
                  ret = -EINVAL;
                  break;
               }
            }
            ret = (*(t->phone_devices[dev_idx].dev->vtbl->start))(
               t->phone_devices[dev_idx].dev, country, line_params,
               ARRAY_SIZE(line_params));
            if (ret) {
               bcm_pr_debug("Failed to configure device\n");
               while (dev_idx > 0) {
                  dev_idx -= 1;
                  (*(t->phone_devices[dev_idx].dev->vtbl->stop))(t->phone_devices[dev_idx].dev);
               }
               break;
            }
         }
      }
      if (!ret) {
         for (line_idx = 0; (line_idx < t->phone_line_count); line_idx += 1) {
            bcm_phone_mgr_line_t *pl = &(t->phone_lines[line_idx]);
            if (phone_line_is_enabled(pl->line)) {
               bcm_phone_mgr_line_init_from_line(pl);
            }
         }
         bcm_periodic_timer_add_work(t->periodic_timer,
            &(t->periodic_work), t->board_desc->phone_desc->tick_period * 1000);
      }
   } while (false);

   return (ret);
}

static int __init bcm_phone_mgr_devs_init(bcm_phone_mgr_t *t)
{
   int ret = 0;
   size_t line_idx;
   size_t dev_idx;

   bcm_pr_debug("%s()\n", __func__);

   t->phone_dev_count = 0;
   t->phone_line_count = 0;

   for (line_idx = 0; (line_idx < ARRAY_SIZE(t->phone_lines)); line_idx += 1) {
      bcm_phone_mgr_line_init(&(t->phone_lines[line_idx]));
   }

   ret = 0;
   for (dev_idx = 0; (dev_idx < t->board_desc->phone_desc->device_count); dev_idx += 1) {
      const phone_desc_device_t *desc = &(t->board_desc->phone_desc->devices[dev_idx]);
      phone_device_t *dev;
      size_t line_count;
      size_t fxs_count;

      if (t->phone_dev_count >= ARRAY_SIZE(t->phone_devices)) {
         bcm_assert(false);
         break;
      }

#ifdef BCMPH_DAHDI_DRIVER
# ifndef BCMPH_NOHW
      if (!(desc->caps & BCMPH_CAPS_ALAW_CODEC)) {
         bcm_assert(false);
         continue;
      }
# else /* BCMPH_NOHW */
      if (!(desc->caps & BCMPH_CAPS_LINEAR_CODEC)) {
         bcm_assert(false);
         continue;
      }
# endif /* BCMPH_NOHW */
#endif /* BCMPH_DAHDI_DRIVER */

      if (!(desc->caps & BCMPH_CAPS_CAN_MIX_NB_AND_WB)) {
         if ((desc->caps & BCMPH_CAPS_CAN_SWITCH_BETWEEN_NB_AND_WB)) {
            bcm_assert(false);
            continue;
         }
      }

      dev = phone_device_alloc(desc, t->board_desc->phone_desc->tick_period);
      if (NULL == dev) {
         ret = -ENOMEM;
         break;
      }

      t->phone_devices[t->phone_dev_count].dev = dev;
      t->phone_dev_count += 1;

      line_count = phone_device_get_line_count(dev);
      if ((line_count <= 0) || (line_count > BCMPH_MAX_LINES_PER_DEV)) {
         bcm_assert(false);
         t->phone_dev_count -= 1;
         t->phone_devices[t->phone_dev_count].dev = NULL;
         phone_device_free(dev);
         continue;
      }

      fxs_count = 0;
      for (line_idx = 0; (line_idx < line_count); line_idx += 1) {
         const phone_desc_line_t *line_desc = phone_device_get_line_desc(dev, line_idx);
         bcm_assert(NULL != line_desc);
         if (BCMPH_LIN_FXS == line_desc->type) {
            bcm_phone_mgr_line_t *pl;
            if (t->phone_line_count >= ARRAY_SIZE(t->phone_lines)) {
               bcm_assert(false);
               break;
            }
            pl = &(t->phone_lines[t->phone_line_count]);
            pl->index_dev = dev_idx;
            pl->index_line = line_idx;
            pl->line = phone_device_get_line(dev, line_idx);
            bcm_assert(NULL != pl->line);
            t->phone_line_count += 1;
            fxs_count += 1;
         }
         else {
            bcm_pr_warn("Driver handles only FXS lines\n");
         }
      }

      if (fxs_count <= 0) {
         bcm_assert(false);
         t->phone_dev_count -= 1;
         t->phone_devices[t->phone_dev_count].dev = NULL;
         phone_device_free(dev);
         continue;
      }
   }

   if ((!ret) && (t->phone_line_count <= 0)) {
      bcm_assert(0 == t->phone_dev_count);
      bcm_pr_err("No FXS line\n");
      ret = -ENODEV;
   }

   if (ret) {
      while (t->phone_dev_count > 0) {
         t->phone_dev_count -= 1;
         phone_device_free(t->phone_devices[t->phone_dev_count].dev);
         t->phone_devices[t->phone_dev_count].dev = NULL;
      }
      for (line_idx = 0; (line_idx < ARRAY_SIZE(t->phone_lines)); line_idx += 1) {
         bcm_phone_mgr_line_deinit(&(t->phone_lines[line_idx]));
      }
      t->phone_line_count = 0;
   }
   else {
      bcm_pr_info("%lu device(s) provides %lu FXS line(s)\n",
         (unsigned long)(t->phone_dev_count), (unsigned long)(t->phone_line_count));
   }

   return (ret);
}

static void bcm_phone_mgr_devs_deinit(bcm_phone_mgr_t *t)
{
   size_t line_idx;

   bcm_pr_debug("%s()\n", __func__);

   while (t->phone_dev_count > 0) {
      t->phone_dev_count -= 1;
      phone_device_free(t->phone_devices[t->phone_dev_count].dev);
      t->phone_devices[t->phone_dev_count].dev = NULL;
   }
   for (line_idx = 0; (line_idx < ARRAY_SIZE(t->phone_lines)); line_idx += 1) {
      bcm_phone_mgr_line_deinit(&(t->phone_lines[line_idx]));
   }
   t->phone_line_count = 0;
}

int __init bcm_phone_mgr_init(bcm_phone_mgr_t *t,
   const board_desc_t *board_desc,
   bcm_periodic_timer_t *periodic_timer, struct workqueue_struct *wq,
   void (*timer_cb)(struct bcm_phones *t))
{
   int ret = -1;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert((NULL != board_desc) && (NULL != timer_cb));

   t->board_desc = board_desc;
   t->periodic_timer = periodic_timer;
   t->wq = wq;
   t->timer_cb = timer_cb;

   if (jiffies_to_msecs(1) > t->board_desc->phone_desc->tick_period) {
      bcm_pr_err("Required timer period (%lu msecs) is below minimal period allowed by driver (%lu msecs)\n",
         (unsigned long)(t->board_desc->phone_desc->tick_period), (unsigned long)(jiffies_to_msecs(1)));
      ret = -1;
      goto fail_period;
   }

   bcm_periodic_work_init(&(t->periodic_work), t->wq, bcm_phone_mgr_timer_work_fn);

   ret = bcm_phone_mgr_devs_init(t);
   if (ret) {
      goto fail_phone_devs;
   }

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_pr_debug("Initializing event wait queue\n");
   bcm_wait_queue_init(&(t->dev.eventq));

   spin_lock_init(&(t->dev.lock));
#endif /* BCMPH_EXPORT_DEV_FILE */

   return (0);

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_wait_queue_deinit(&(t->dev.eventq));
#endif /* BCMPH_EXPORT_DEV_FILE */
   bcm_phone_mgr_devs_deinit(t);
fail_phone_devs:
   bcm_periodic_work_deinit(&(t->periodic_work));
fail_period:
   t->timer_cb = NULL;
   t->wq = NULL;
   t->periodic_timer = NULL;
   t->board_desc = NULL;
   return (ret);
}

void bcm_phone_mgr_deinit(bcm_phone_mgr_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_pr_debug("Cancelling work and destroying it\n");
   bcm_periodic_timer_del_work(t->periodic_timer, &(t->periodic_work));
   cancel_work_sync(&(t->periodic_work.work));
#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_wait_queue_deinit(&(t->dev.eventq));
#endif /* BCMPH_EXPORT_DEV_FILE */
   bcm_phone_mgr_devs_deinit(t);
   bcm_periodic_work_deinit(&(t->periodic_work));
   t->timer_cb = NULL;
   t->wq = NULL;
   t->periodic_timer = NULL;
   t->board_desc = NULL;
}
