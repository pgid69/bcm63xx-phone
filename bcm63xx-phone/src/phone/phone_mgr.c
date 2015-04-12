/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#include <phone_mgr.h>

static inline void bcm_phone_mgr_update_from_ls(bcm_phone_mgr_line_t *t)
{
   t->shared.change_status = NO_CHANGE_ASKED;
   t->shared.new_codec = t->shared.line_state.codec;
   t->shared.new_mode = t->shared.line_state.mode;
   t->shared.new_tone = bcm_phone_line_tone_code_index(t->shared.line_state.tone);
   atomic_set(&(t->shared.current_codec), t->shared.line_state.codec);
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
      BCMPH_STATUS_DISCONNECTED, BCMPH_CODEC_LINEAR,
      BCMPH_MODE_IDLE, BCMPH_TONE_NONE);
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

static int bcm_phone_mgr_wait_for_change_completion(bcm_phone_mgr_t *t,
   const bcm_phone_mgr_line_t *pl, int wq_counter,
   int wait, bcmph_mutex_t *lock)
{
   int ret = 0;

   d_bcm_pr_debug("bcm_phone_mgr_wait_for_change_completion(wait=%d)\n", (int)(wait));

   if (wait < 0) {
      for (;;) {
         ret = bcm_wait_queue_wait_event_counter(&(t->eventq),
            wq_counter, lock);
         if (ret < 0) {
            break;
         }
         barrier();
         // We update the counter before doing the verification
         // so we are sure not to loose an event between the verification
         // and when we start waiting for an event
         wq_counter = bcm_wait_queue_get_counter(&(t->eventq));
         spin_lock_bh(&(t->lock));
         // When timer is stopped, change_status is reset to NO_CHANGED_ASKED
         // so no need to test timer is active
         if (NO_CHANGE_ASKED == pl->shared.change_status) {
            spin_unlock_bh(&(t->lock));
            ret = 0;
            break;
         }
         spin_unlock_bh(&(t->lock));
      }
   }
   else {
      long timeout_in_jiffies = msecs_to_jiffies(wait);
      ret = 1;
      for (;;) {
         long remaining = bcm_wait_queue_wait_event_counter_timeout(&(t->eventq),
            wq_counter, timeout_in_jiffies, lock);
         if (remaining < 0) {
            ret = remaining;
            break;
         }
         barrier();
         // We update the counter before doing the verification
         // so we are sure not to loose an event between the verification
         // and we wait for an event
         wq_counter = bcm_wait_queue_get_counter(&(t->eventq));
         spin_lock_bh(&(t->lock));
         // When timer is stopped, change_status is reset to NO_CHANGED_ASKED
         // so no need to test timer is active
         if (NO_CHANGE_ASKED == pl->shared.change_status) {
            spin_unlock_bh(&(t->lock));
            ret = 0;
            break;
         }
         spin_unlock_bh(&(t->lock));
         timeout_in_jiffies = remaining;
         if (timeout_in_jiffies <= 0) {
            break;
         }
      }
   }

   return (ret);
}

__u8 bcm_phone_mgr_get_first_timeslot_line(const bcm_phone_mgr_t *t, size_t line)
{
   const bcm_phone_mgr_line_t *pl;
   const phone_device_t *dev;
   const phone_desc_line_t *desc;

   bcm_pr_debug("bcm_phone_mgr_get_first_timeslot_line(line=%lu)\n",
      (unsigned long)(line));

   bcm_assert(line < t->phone_line_count);

   pl = &(t->phone_lines[line]);
   dev = t->phone_devices[pl->index_dev].dev;
   desc = phone_device_get_line_desc(dev, pl->index_line);

   return (desc->first_timeslot);
}

bool bcm_phone_mgr_line_supports_codec(const bcm_phone_mgr_t *t, size_t line, bcm_phone_codec_t codec)
{
   bool ret = false;

   bcm_pr_debug("bcm_phone_mgr_line_supports_codec(line=%lu, codec=%d)\n",
      (unsigned long)(line), (int)(codec));

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
      else {
         // Unsupported codec
         break;
      }

      ret = true;
   } while (false);

   return (ret);
}

bool bcm_phone_mgr_line_can_switch_to_codec(const bcm_phone_mgr_t *t, size_t line,
   bcm_phone_codec_t old_codec, bcm_phone_codec_t new_codec)
{
   bool ret = false;

   bcm_pr_debug("bcm_phone_mgr_line_can_switch_to_codec(line=%lu, old_codec=%d, new_codec=%d)\n",
      (unsigned long)(line), (int)(old_codec), (int)(new_codec));

   bcm_assert(line < t->phone_line_count);

   // We forbids changing to or from BCMPH_CODEC_LINEAR16
   if (BCMPH_CODEC_LINEAR16 == old_codec) {
      if (BCMPH_CODEC_LINEAR16 == new_codec) {
         ret = true;
      }
   }
   else {
      if (BCMPH_CODEC_LINEAR16 != new_codec) {
         ret = true;
      }
   }

   return (ret);
}

int bcm_phone_mgr_set_line_codec(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_codec_t codec, bcm_phone_line_mode_t mode,
   __u32 tone, int wait, bcmph_mutex_t *lock)
{
   int ret = 0;

   bcm_pr_debug("bcm_phone_mgr_set_line_codec(line=%lu, codec=%d, mode=%d, tone=0x%lx, wait=%d)\n",
      (unsigned long)(line), (int)(codec), (int)(mode), (unsigned long)(tone), (int)(wait));

   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));

   bcm_assert((line < t->phone_line_count)
      && (bcm_phone_mgr_line_supports_codec(t, line, codec))
      && (phone_line_is_enabled(t->phone_lines[line].line)));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->eventq));

      spin_lock_bh(&(t->lock));
      pl->shared.new_codec = codec;
      if (BCMPH_MODE_UNSPECIFIED != mode) {
         if (mode < BCMPH_MAX_MODES) {
            pl->shared.new_mode = mode;
            if (!bcm_phone_mgr_mode_allow_tones(mode)) {
               pl->shared.new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
            }
            else {
               bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);
               if (BCMPH_TONE_UNSPECIFIED != tone_index) {
                  if (tone_index < BCMPH_MAX_TONES) {
                     pl->shared.new_tone = tone;
                  }
                  else {
                     bcm_pr_debug("Invalid tone %lu\n", (unsigned long)(tone));
                  }
               }
            }
         }
         else {
            bcm_pr_debug("Invalid mode %d\n", (int)(mode));
         }
      }
      if (wait) {
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
      }
      else {
         pl->shared.change_status |= CHANGE_ASKED;
      }
      spin_unlock_bh(&(t->lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
   }
   while (false);

   return (ret);
}

int bcm_phone_mgr_set_line_mode(bcm_phone_mgr_t *t, size_t line,
   bcm_phone_line_mode_t mode, __u32 tone,
   int wait, bcmph_mutex_t *lock)
{
   int ret = 0;

   bcm_pr_debug("bcm_phone_mgr_set_line_mode(line=%lu, mode=%d, tone=0x%lx, wait=%d)\n",
      (unsigned long)(line), (int)(mode), (unsigned long)(tone), (int)(wait));

   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));

   bcm_assert((line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[line].line))
      && (mode != BCMPH_MODE_UNSPECIFIED) && (mode < BCMPH_MAX_MODES));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->eventq));
      bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);

      spin_lock_bh(&(t->lock));
      pl->shared.new_mode = mode;
      if (BCMPH_TONE_UNSPECIFIED != tone_index) {
         if (!bcm_phone_mgr_mode_allow_tones(mode)) {
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
      if (wait) {
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
      }
      else {
         pl->shared.change_status |= CHANGE_ASKED;
      }
      spin_unlock_bh(&(t->lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
   }
   while (false);

   return (ret);
}

int bcm_phone_mgr_set_line_tone(bcm_phone_mgr_t *t, size_t line,
   __u32 tone, int wait, bcmph_mutex_t *lock)
{
   int ret = 0;
#ifdef BCMPH_DEBUG
   bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);
#endif // BCMPH_DEBUG

   bcm_pr_debug("bcm_phone_mgr_set_line_tone(line=%lu, tone=0x%lx, wait=%d)\n",
      (unsigned long)(line), (unsigned long)(tone), (int)(wait));

   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));

   bcm_assert((line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[line].line))
      && (BCMPH_TONE_UNSPECIFIED != tone_index)
      && (tone_index < BCMPH_MAX_TONES));

   do { // Empty loop
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[line]);
      // We init the counter before doing the change
      // so we are sure not to loose an event between the change
      // and when we start waiting for an event
      int wq_counter = bcm_wait_queue_get_counter(&(t->eventq));

      spin_lock_bh(&(t->lock));
      pl->shared.new_tone = tone;
      if (wait) {
         pl->shared.change_status |= CHANGE_ASKED_ASAP;
      }
      else {
         pl->shared.change_status |= CHANGE_ASKED;
      }
      spin_unlock_bh(&(t->lock));

      if (wait) {
         ret = bcm_phone_mgr_wait_for_change_completion(t, pl, wq_counter, wait, lock);
      }
   }
   while (false);

   return (ret);
}

#ifdef BCMPH_TEST_PCM
int bcm_phone_mgr_set_line_state(bcm_phone_mgr_t *t,
   const bcm_phone_set_line_state_t *set_line_state)
{
   int ret = 0;

   d_bcm_pr_debug("bcm_phone_mgr_set_line_state()\n");

   bcm_assert((NULL != set_line_state)
      && (set_line_state->line < t->phone_line_count)
      && (phone_line_is_enabled(t->phone_lines[set_line_state->line].line)));

   do { // Empty loop
      bool send_event = false;
      size_t i;
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[set_line_state->line]);
      bcm_phone_line_state_t *line_state = &(pl->shared.line_state);

      spin_lock_bh(&(t->lock));

      // Set status
      if ((BCMPH_STATUS_UNSPECIFIED != set_line_state->status) && (set_line_state->status < BCMPH_MAX_STATUS)) {
         if (line_state->status != set_line_state->status) {
            line_state->status = set_line_state->status;
            line_state->status_change_count += 1;
            send_event = true;
         }
      }

      // Add digits
      for (i = 0; ((i < set_line_state->digits_count) && (line_state->digits_count < ARRAY_SIZE(line_state->digits))); i += 1) {
         line_state->digits[line_state->digits_count] = set_line_state->digits[i];
         line_state->digits_count += 1;
      }
      if (i > 0) {
         send_event = true;
      }

      // Add flash count
      if (set_line_state->flash_count > 0) {
         line_state->flash_count += set_line_state->flash_count;
         send_event += 1;
      }

      spin_unlock_bh(&(t->lock));

      if (send_event) {
         bcm_wait_queue_wake_up(&(t->eventq));
      }
   }
   while (false);

   return (ret);
}
#endif // BCMPH_TEST_PCM

void bcm_phone_mgr_get_line_states(bcm_phone_mgr_t *t,
   bcm_phone_line_state_t *line_states, size_t line_states_len)
{
   size_t i;

   dd_bcm_pr_debug("bcm_phone_mgr_get_line_states()\n");

   if (line_states_len > t->phone_line_count) {
      line_states_len = t->phone_line_count;
   }
   spin_lock_bh(&(t->lock));
   for (i = 0; (i < line_states_len); i += 1) {
      bcm_phone_line_state_move(&(t->phone_lines[i].shared.line_state), &(line_states[i]));
   }
   spin_unlock_bh(&(t->lock));
}

static void bcm_phone_mgr_timer_cb(bcm_timer_t *timer)
{
   bcm_phone_mgr_t *t = container_of(timer, bcm_phone_mgr_t, timer);
   queue_work(t->workqueue, &(t->work));
}

static void bcm_phone_mgr_timer_work_fn(struct work_struct *work)
{
   bcm_phone_mgr_t *t = container_of(work, bcm_phone_mgr_t, work);

   dd_bcm_pr_debug("bcm_phone_mgr_timer_work_fn()\n");

   if (bcm_timer_is_active(&(t->timer))) {
      size_t i;
      bool send_event;

      spin_lock_bh(&(t->lock));
      for (i = 0; (i < t->phone_line_count); i += 1) {
         bcm_phone_mgr_line_t *pl = &(t->phone_lines[i]);
         phone_line_t *line = pl->line;
         if (phone_line_is_enabled(line)) {
#ifdef BCMPH_TEST_PCM
            if (line->line_state.status != pl->shared.line_state.status) {
               line->line_state.status = pl->shared.line_state.status;
               line->new_mode = BCMPH_MODE_IDLE;
               line->new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
            }
#endif // BCMPH_TEST_PCM
            if ((pl->shared.change_status & (CHANGE_ASKED | CHANGE_ASKED_ASAP))) {
               line->new_codec = pl->shared.new_codec;
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
      spin_unlock_bh(&(t->lock));

      for (i = 0; (i < t->phone_dev_count); i += 1) {
         if (t->phone_devices[i].line_enable_count > 0) {
            (*(t->phone_devices[i].dev->vtbl->tick))(t->phone_devices[i].dev);
         }
      }

      send_event = false;
      spin_lock_bh(&(t->lock));
      for (i = 0; (i < t->phone_line_count); i += 1) {
         bcm_phone_mgr_line_t *pl = &(t->phone_lines[i]);
         phone_line_t *line = pl->line;
         if (phone_line_is_enabled(line)) {
            if (line->line_state_changed) {
               // We transfer the changes in the structure line_state
               // written and read by ioctl()
               phone_line_move_line_state(line, &(pl->shared.line_state));
               atomic_set(&(pl->shared.current_codec), pl->shared.line_state.codec);
               atomic_set(&(pl->shared.current_mode), pl->shared.line_state.mode);
               atomic_set(&(pl->shared.current_tone), pl->shared.line_state.tone);
               // And we set a flag to wake up processes blocked in wait queue eventq
               // to tell them that at least one line state has changed
               send_event = true;
            }
            if ((line->new_codec == line->line_state.codec)
                 && (line->new_mode == line->line_state.mode)
                 && (bcm_phone_line_tone_decode_index(line->new_tone) == line->line_state.tone)) {
               if ((pl->shared.change_status & CHANGE_PENDING)) {
                  pl->shared.change_status &= (~(CHANGE_PENDING));
                  send_event = true;
               }
            }
         }
      }
      spin_unlock_bh(&(t->lock));

      if (send_event) {
         bcm_wait_queue_wake_up(&(t->eventq));
      }

      (*(t->timer_cb))(t);

      bcm_timer_reschedule(&(t->timer));
   }
}

void bcm_phone_mgr_stop(bcm_phone_mgr_t *t)
{
   size_t i;

   bcm_pr_debug("bcm_phone_mgr_stop()\n");

   bcm_timer_stop(&(t->timer));
   cancel_work_sync(&(t->work));

   for (i = 0; (i < t->phone_dev_count); i += 1) {
      (*(t->phone_devices[i].dev->vtbl->stop))(t->phone_devices[i].dev);
   }

   // Before wake up process waiting on eventq
   // we reset phone_line in particular change_status
   for (i = 0; (i < t->phone_line_count); i += 1) {
      bcm_phone_mgr_line_t *pl = &(t->phone_lines[i]);
      phone_line_disable(pl->line);
      bcm_phone_mgr_line_reset(pl);
      bcm_assert(NO_CHANGE_ASKED == pl->shared.change_status);
   }
   bcm_wait_queue_wake_up(&(t->eventq));
}

int bcm_phone_mgr_start(bcm_phone_mgr_t *t, bcmph_country_t country,
   const phone_line_params_t *lps, size_t lps_count)
{
   int ret = 0;

   bcm_pr_debug("bcm_phone_mgr_start()\n");

   bcm_assert((!bcm_timer_is_active(&(t->timer))) && (country < BCMPH_COUNTRY_MAX));

   do { // Empty loop
      size_t i;

      for (i = 0; (i < t->phone_dev_count); i += 1) {
         t->phone_devices[i].line_enable_count = 0;
      }

      for (i = 0; (i < t->phone_line_count); i += 1) {
         bcm_phone_mgr_line_t *pl = &(t->phone_lines[i]);
         phone_line_disable(pl->line);
         bcm_phone_mgr_line_reset(pl);
         bcm_assert(NO_CHANGE_ASKED == pl->shared.change_status);
      }

      for (i = 0; (i < lps_count); i += 1) {
         if (!lps[i].enable) {
            continue;
         }
         bcm_assert((i < t->phone_line_count) && (bcm_phone_mgr_line_supports_codec(t, i, lps[i].codec)));
         t->phone_devices[t->phone_lines[i].index_dev].line_enable_count += 1;
      }

      for (i = 0; (i < t->phone_dev_count); i += 1) {
         if (t->phone_devices[i].line_enable_count > 0) {
            const phone_line_params_t *line_params[BCMPH_MAX_LINES_PER_DEV];
            size_t j;
            size_t k;
            size_t are_not_linear16;
            size_t are_linear16;

            bcm_assert(phone_device_get_line_count(t->phone_devices[i].dev) <= ARRAY_SIZE(line_params));

            for (j = 0; (j < ARRAY_SIZE(line_params)); j += 1) {
               line_params[j] = NULL;
            }

            k = 0;
            for (j = 0; (j < lps_count); j += 1) {
               if ((lps[j].enable) && (t->phone_lines[j].index_dev == i)) {
                  bcm_assert((t->phone_lines[j].index_line < ARRAY_SIZE(line_params))
                     && (NULL == line_params[t->phone_lines[j].index_line]));
                  line_params[t->phone_lines[j].index_line] = &(lps[j]);
                  k += 1;
               }
            }
            bcm_assert(k == t->phone_devices[i].line_enable_count);

            // Now we check that all lines are LINEAR16 or not
            are_not_linear16 = 0;
            are_linear16 = 0;
            for (j = 0; (j < ARRAY_SIZE(line_params)); j += 1) {
               if (NULL != line_params[j]) {
                  if (BCMPH_CODEC_LINEAR16 == line_params[j]->codec) {
                     are_linear16 += 1;
                  }
                  else {
                     are_not_linear16 += 1;
                  }
               }
            }
            if ((0 != are_linear16) && (0 != are_not_linear16)) {
               bcm_pr_err("All or none of the lines of the same device, can use LINEAR16 codec\n");
               ret = -EINVAL;
               break;
            }
            ret = (*(t->phone_devices[i].dev->vtbl->start))(
               t->phone_devices[i].dev, country, line_params,
               ARRAY_SIZE(line_params));
            if (ret) {
               bcm_pr_debug("Failed to configure device\n");
               while (i > 0) {
                  i -= 1;
                  (*(t->phone_devices[i].dev->vtbl->stop))(t->phone_devices[i].dev);
               }
               break;
            }
         }
      }
      if (!ret) {
         for (i = 0; (i < t->phone_line_count); i += 1) {
            bcm_phone_mgr_line_t *pl = &(t->phone_lines[i]);
            if (phone_line_is_enabled(pl->line)) {
               bcm_phone_mgr_line_init_from_line(pl);
            }
         }
         bcm_timer_start(&(t->timer), t->board_desc->phone_desc->tick_period);
      }
   } while (false);

   return (ret);
}

static int __init bcm_phone_mgr_devs_init(bcm_phone_mgr_t *t)
{
   int ret = 0;
   size_t i;

   bcm_pr_debug("bcm_phone_mgr_devs_init()\n");

   t->phone_dev_count = 0;
   t->phone_line_count = 0;

   for (i = 0; (i < ARRAY_SIZE(t->phone_lines)); i += 1) {
      bcm_phone_mgr_line_init(&(t->phone_lines[i]));
   }

   ret = 0;
   for (i = 0; (i < t->board_desc->phone_desc->device_count); i += 1) {
      phone_device_t *dev;
      size_t j;
      size_t line_count;
      size_t fxs_count;

      if (t->phone_dev_count >= ARRAY_SIZE(t->phone_devices)) {
         bcm_assert(false);
         break;
      }

      dev = phone_device_alloc(&(t->board_desc->phone_desc->devices[i]), t->board_desc->phone_desc->tick_period);
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
      for (j = 0; (j < line_count); j += 1) {
         const phone_desc_line_t *line_desc = phone_device_get_line_desc(dev, j);
         bcm_assert(NULL != line_desc);
         if (BCMPH_LIN_FXS == line_desc->type) {
            bcm_phone_mgr_line_t *pl;
            if (t->phone_line_count >= ARRAY_SIZE(t->phone_lines)) {
               bcm_assert(false);
               break;
            }
            pl = &(t->phone_lines[t->phone_line_count]);
            pl->index_dev = i;
            pl->index_line = j;
            pl->line = phone_device_get_line(dev, j);
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
      for (i = 0; (i < ARRAY_SIZE(t->phone_lines)); i += 1) {
         bcm_phone_mgr_line_deinit(&(t->phone_lines[i]));
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
   size_t i;

   bcm_pr_debug("bcm_phone_mgr_devs_deinit()\n");

   while (t->phone_dev_count > 0) {
      t->phone_dev_count -= 1;
      phone_device_free(t->phone_devices[t->phone_dev_count].dev);
      t->phone_devices[t->phone_dev_count].dev = NULL;
   }
   for (i = 0; (i < ARRAY_SIZE(t->phone_lines)); i += 1) {
      bcm_phone_mgr_line_deinit(&(t->phone_lines[i]));
   }
   t->phone_line_count = 0;
}

int __init bcm_phone_mgr_init(bcm_phone_mgr_t *t,
   const board_desc_t *board_desc, void (*timer_cb)(struct bcm_phones *t))
{
   int ret;

   bcm_pr_debug("bcm_phone_mgr_init()\n");

   bcm_assert((NULL != board_desc) && (NULL != timer_cb));

   t->board_desc = board_desc;
   t->timer_cb = timer_cb;

   if (jiffies_to_msecs(1) > t->board_desc->phone_desc->tick_period) {
      bcm_pr_err("Required timer period (%lu usecs) is below minimal period allowed by the kernel (%lu usecs)\n", (unsigned long)(t->board_desc->phone_desc->tick_period), (unsigned long)(jiffies_to_usecs(1)));
      ret = -1;
      goto fail_period;
   }

   ret = bcm_phone_mgr_devs_init(t);
   if (ret) {
      goto fail_phone_devs;
   }

   bcm_pr_debug("Initializing event wait queue\n");
   bcm_wait_queue_init(&(t->eventq));

   t->workqueue = alloc_workqueue(driver_name, WQ_UNBOUND | WQ_FREEZABLE | WQ_HIGHPRI, 1);
   if (NULL == t->workqueue) {
      ret = -ENOMEM;
      goto fail_create_wq;
   }
   INIT_WORK(&(t->work), bcm_phone_mgr_timer_work_fn);

   spin_lock_init(&(t->lock));

   // Init time work
   ret = bcm_timer_init(&(t->timer), bcm_phone_mgr_timer_cb);
   if (ret) {
      goto fail_timer;
   }

   return (0);

   bcm_timer_deinit(&(t->timer));
fail_timer:
   bcm_pr_debug("Flushing timer workqueue and destroying it\n");
   cancel_work_sync(&(t->work));
   flush_workqueue(t->workqueue);
   destroy_workqueue(t->workqueue);
   t->workqueue = NULL;
fail_create_wq:
   bcm_wait_queue_deinit(&(t->eventq));
   bcm_phone_mgr_devs_deinit(t);
fail_phone_devs:
fail_period:
   return (ret);
}

void bcm_phone_mgr_deinit(bcm_phone_mgr_t *t)
{
   bcm_pr_debug("bcm_phone_mgr_deinit()\n");

   bcm_timer_deinit(&(t->timer));
   bcm_pr_debug("Flushing timer workqueue and destroying it\n");
   cancel_work_sync(&(t->work));
   flush_workqueue(t->workqueue);
   destroy_workqueue(t->workqueue);
   t->workqueue = NULL;
   bcm_wait_queue_deinit(&(t->eventq));
   bcm_phone_mgr_devs_deinit(t);
}
