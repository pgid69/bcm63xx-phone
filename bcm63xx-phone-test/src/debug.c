/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <ncurses.h>
#include <sys/ioctl.h>

#define ALSA_PCM_NEW_HW_PARAMS_API
#define ALSA_PCM_NEW_SW_PARAMS_API
#include <alsa/asoundlib.h>

//#define BCMPH_DAHDI_DRIVER
#define BCMPH_EXPORT_DEV_FILE
#include <bcm63xx_phone.h>
#include <macros.h>

#include "../../bcm63xx-ast-chan/src/tone_generation.h"

#define SAMPLE_SIZE 2

typedef struct {
   WINDOW *win;
} bpd_console_t;

static void bpd_console_init(bpd_console_t *t, WINDOW *win)
{
   t->win = win;
   scrollok(t->win, TRUE);
   idlok(t->win, TRUE);
}

static void bpd_console_deinit(bpd_console_t *t)
{
   delwin(t->win);
   t->win = NULL;
}

static bpd_console_t *bpd_console = NULL;

#ifdef bcm_pr_debug
# undef bcm_pr_debug
# define bcm_pr_debug(fmt, args...) if (NULL == bpd_console) { \
      fprintf(stderr, "Debug: %s() %lu: " fmt, __func__, (unsigned long)(__LINE__), ## args); \
   } \
   else { \
      wprintw(bpd_console->win, "Debug: %s() %lu: " fmt, __func__, (unsigned long)(__LINE__), ## args); \
      wrefresh(bpd_console->win); \
   }
#endif
#ifdef bcm_pr_info
# undef bcm_pr_info
# define bcm_pr_info(fmt, args...) if (NULL == bpd_console) { \
      fprintf(stderr, "Info: " fmt, ## args); \
   } \
   else { \
      wprintw(bpd_console->win, "Info: " fmt, ## args); \
      wrefresh(bpd_console->win); \
   }
#endif
#ifdef bcm_pr_warn
# undef bcm_pr_warn
# define bcm_pr_warn(fmt, args...) if (NULL == bpd_console) { \
      fprintf(stderr, "Warning: " fmt, ## args); \
   } \
   else { \
      wprintw(bpd_console->win, "Warning: " fmt, ## args); \
      wrefresh(bpd_console->win); \
   }
#endif
#ifdef bcm_pr_err
# undef bcm_pr_err
# define bcm_pr_err(fmt, args...) if (NULL == bpd_console) { \
      fprintf(stderr, "Error: " fmt, ## args); \
   } \
   else { \
      wprintw(bpd_console->win, "Error: " fmt, ## args); \
      wrefresh(bpd_console->win); \
   }
#endif

#if (2 != SAMPLE_SIZE)
#error "SAMPLE_SIZE must be equal to 2"
#endif
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define SND_PCM_SAMPLE_FORMAT SND_PCM_FORMAT_S16_LE
#else
#define SND_PCM_SAMPLE_FORMAT SND_PCM_FORMAT_S16_BE
#endif
/* Let's use a 30 ms period */
#define PERIOD_SIZE_IN_FRAMES (30 * BCMPH_SAMPLES_PER_MS)

#define DTMF_ON_DURATION 100
#define DTMF_OFF_DURATION 100

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_0_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 941,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_1_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 697,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_2_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 697,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_3_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 697,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_4_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 770,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_5_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 770,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_6_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 770,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_7_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 852,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_8_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 852,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_9_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 852,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_aster_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 941,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_pound_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 941,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_A_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 697,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_B_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 770,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_C_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 852,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_raw_def_t bpd_tone_dtmf_D_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 941,
      .duration = DTMF_ON_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   },
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = DTMF_OFF_DURATION,
      .modulation_percent = 0,
      .midinote = 0,
   }
};

static bcmph_dual_tone_sequence_t bpd_tone_dtmf_0;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_1;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_2;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_3;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_4;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_5;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_6;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_7;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_8;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_9;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_aster;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_pound;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_A;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_B;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_C;
static bcmph_dual_tone_sequence_t bpd_tone_dtmf_D;

#if (2 != SAMPLE_SIZE)
#error "SAMPLE_SIZE must be equal to 2"
#endif

static void bpd_init_tones(void)
{
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_0), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_0_parts, ARRAY_SIZE(bpd_tone_dtmf_0_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_1), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_1_parts, ARRAY_SIZE(bpd_tone_dtmf_1_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_2), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_2_parts, ARRAY_SIZE(bpd_tone_dtmf_2_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_3), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_3_parts, ARRAY_SIZE(bpd_tone_dtmf_3_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_4), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_4_parts, ARRAY_SIZE(bpd_tone_dtmf_4_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_5), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_5_parts, ARRAY_SIZE(bpd_tone_dtmf_5_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_6), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_6_parts, ARRAY_SIZE(bpd_tone_dtmf_6_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_7), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_7_parts, ARRAY_SIZE(bpd_tone_dtmf_7_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_8), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_8_parts, ARRAY_SIZE(bpd_tone_dtmf_8_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_9), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_9_parts, ARRAY_SIZE(bpd_tone_dtmf_9_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_aster), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_aster_parts, ARRAY_SIZE(bpd_tone_dtmf_aster_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_pound), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_pound_parts, ARRAY_SIZE(bpd_tone_dtmf_pound_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_A), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_A_parts, ARRAY_SIZE(bpd_tone_dtmf_A_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_B), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_B_parts, ARRAY_SIZE(bpd_tone_dtmf_B_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_C), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_C_parts, ARRAY_SIZE(bpd_tone_dtmf_C_parts), -1);
   bcmph_dual_tone_sequence_init(&(bpd_tone_dtmf_D), BCMPH_SAMPLE_RATE, bpd_tone_dtmf_D_parts, ARRAY_SIZE(bpd_tone_dtmf_D_parts), -1);
}

static const char *bcm63xx_phone_dev_name = "/dev/bcm63xx_phone_debug";

static int bpd_do_ioctl(int fd, unsigned long request, unsigned long arg, int retry)
{
   int ret = ioctl(fd, request, arg);
   if ((retry) && (ret) && ((EAGAIN == errno) || (EINTR == errno))) {
      ret = ioctl(fd, request, arg);
   }
   if (ret < 0) {
      bcm_pr_err("ioctl(%lu) failed. ret=%ld, errno=%ld\n", (unsigned long)(request), (long)(ret), (long)(errno));
   }
   return (ret);
}

typedef struct {
   snd_pcm_t *card;
   snd_pcm_hw_params_t *hw_params;
   snd_pcm_sw_params_t *sw_params;
   bool started;
} bpd_snd_card_t;

static int bpd_snd_card_init(bpd_snd_card_t *t, const char *dev,
   snd_pcm_stream_t stream)
{
   int ret = -1;
   snd_pcm_t *handle = NULL;
   snd_pcm_hw_params_t *hw_params = NULL;
   snd_pcm_sw_params_t *sw_params = NULL;

   memset(t, 0, sizeof(*t));
   t->started = false;

   do { /* Empty loop */
      int err;
      int direction;
      snd_pcm_uframes_t period_size;
      snd_pcm_uframes_t buffer_size = 0;
      unsigned int rate;
      snd_pcm_uframes_t start_threshold;
      snd_pcm_uframes_t stop_threshold;

      err = snd_pcm_open(&(handle), dev, stream, SND_PCM_NONBLOCK);
      if (err) {
         bcm_pr_err("snd_pcm_open() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         ret = err;
         break;
      }
      bcm_pr_debug("Opening device '%s' in %s mode\n", dev, (stream == SND_PCM_STREAM_CAPTURE) ? "read" : "write");

      hw_params = NULL;
      err = snd_pcm_hw_params_malloc(&(hw_params));
      if ((err < 0) || (NULL == hw_params)) {
         hw_params = NULL;
         bcm_pr_err("Failed to allocate hw_params structure for device '%s'\n", dev);
         break;
      }

      err = snd_pcm_hw_params_any(handle, hw_params);
      if (err < 0) {
         ret = err;
         bcm_pr_err("snd_pcm_hw_params_any() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      err = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
      if (err < 0) {
         ret = err;
         bcm_pr_err("snd_pcm_hw_params_set_access() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      err = snd_pcm_hw_params_set_format(handle, hw_params, SND_PCM_SAMPLE_FORMAT);
      if (err < 0) {
         ret = err;
         bcm_pr_err("snd_pcm_hw_params_set_format() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      err = snd_pcm_hw_params_set_channels(handle, hw_params, 1);
      if (err < 0) {
         ret = err;
         bcm_pr_err("snd_pcm_hw_params_set_channels() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      direction = 0;
      rate = BCMPH_SAMPLE_RATE;
      err = snd_pcm_hw_params_set_rate_near(handle, hw_params, &(rate), &(direction));
      if (err < 0) {
         ret = err;
         bcm_pr_err("snd_pcm_hw_params_set_rate_near() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }
      if (rate != BCMPH_SAMPLE_RATE) {
         bcm_pr_warn("Can't set rate for device '%s', requested %u, got %u\n",
            dev, (unsigned int)(BCMPH_SAMPLE_RATE), (unsigned int)(rate));
         break;
      }

      direction = 0;
      period_size = PERIOD_SIZE_IN_FRAMES;
      err = snd_pcm_hw_params_set_period_size_near(handle, hw_params, &(period_size), &(direction));
      if (err < 0) {
         bcm_pr_err("snd_pcm_hw_params_set_period_size_near() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }
      bcm_pr_debug("Period size is %lu\n", (unsigned long)(period_size));

      buffer_size = period_size * 16;
      err = snd_pcm_hw_params_set_buffer_size_near(handle, hw_params, &(buffer_size));
      if (err < 0) {
         bcm_pr_warn("snd_pcm_hw_params_set_buffer_size_near() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }
      bcm_pr_debug("Buffer size is set to %lu frames\n", (unsigned long)(buffer_size));

      err = snd_pcm_hw_params(handle, hw_params);
      if (err < 0) {
         ret = err;
         bcm_pr_err("Couldn't set the new hw params for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      sw_params = NULL;
      err = snd_pcm_sw_params_malloc(&(sw_params));
      if ((err < 0) || (NULL == sw_params)) {
         bcm_pr_err("Failed to allocate sw_params structure for device '%s'\n", dev);
         break;
      }
      err = snd_pcm_sw_params_current(handle, sw_params);
      if (err < 0) {
         ret = err;
         bcm_pr_err("snd_pcm_sw_params_current() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      if (stream == SND_PCM_STREAM_PLAYBACK) {
         start_threshold = period_size;
      }
      else {
         start_threshold = 1;
      }
      err = snd_pcm_sw_params_set_start_threshold(handle, sw_params, start_threshold);
      if (err < 0) {
         bcm_pr_err("snd_pcm_sw_params_set_start_threshold() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      if (stream == SND_PCM_STREAM_PLAYBACK) {
         /*
          At first we were considering disabling automatic stop and enabling
          silence generation.
          To disable automatic stop we must set stop threshold to
          snd_pcm_sw_params_get_boundary() : in this mode ALSA loops in the ring
          buffer even if there's no new data.
          To fill buffer with silence without overwriting data, we must set
          silence size to boundary and silence threshold to 0, so each time
          ALSA read a sample in ring buffer it replaces it with silence.

          BUT IT DOES'NT WORK : as soon as an underrun occurs, on some devices
          (eg default), once ALSA played silence instead of underrunning one
          time, it never would play anything again.
          It's explained in this forum thread :
          https://sourceforge.net/p/alsa/mailman/alsa-user/thread/4FEB7437.4010307@ischo.com/

          We don't use snd_pcm_avail() and snd_pcm_forward() because
          snd_pcm_forward() does not work with all devices

          So we just disable silence generation and set stop threshold to buffer
          size, and we handle under run.
         */
         stop_threshold = buffer_size;
      }
      else {
         stop_threshold = buffer_size;
      }
      err = snd_pcm_sw_params_set_stop_threshold(handle, sw_params, stop_threshold);
      if (err < 0) {
         bcm_pr_err("snd_pcm_sw_params_set_stop_threshold() failed for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      err = snd_pcm_sw_params(handle, sw_params);
      if (err < 0) {
         bcm_pr_err("Couldn't set the new sw params for device '%s': '%s'\n", dev, snd_strerror(err));
         break;
      }

      t->card = handle;
      handle = NULL;
      t->hw_params = hw_params;
      hw_params = NULL;
      t->sw_params = sw_params;
      sw_params = NULL;

      ret = 0;
   }
   while (false);

   if (NULL != handle) {
      bcm_assert(NULL == t->card);
      snd_pcm_close(handle);
      handle = NULL;
   }

   if (NULL != sw_params) {
      bcm_assert(NULL == t->sw_params);
      snd_pcm_sw_params_free(sw_params);
      sw_params = NULL;
   }

   if (NULL != hw_params) {
      bcm_assert(NULL == t->hw_params);
      snd_pcm_hw_params_free(hw_params);
      hw_params = NULL;
   }

   return (ret);
}

static void bpd_snd_card_deinit(bpd_snd_card_t *t)
{
   if (NULL != t->card) {
      snd_pcm_close(t->card);
      t->card = NULL;
      snd_pcm_hw_params_free(t->hw_params);
      t->hw_params = NULL;
      snd_pcm_sw_params_free(t->sw_params);
      t->sw_params = NULL;
   }
}

static bool bpd_snd_card_handle_error(bpd_snd_card_t *t, snd_pcm_sframes_t error, const char *function)
{
   bool ret = false;

   bcm_assert(error < 0);

   do { /* Empty loop */
      if (-EAGAIN == error) {
          /*  Nothing todo */
         break;
      }
      if (-EPIPE == error) {
         snd_pcm_state_t state = snd_pcm_state(t->card);
         if (state != SND_PCM_STATE_XRUN) {
            int err = snd_pcm_prepare(t->card);
            if (err) {
               bcm_pr_err("snd_pcm_prepare() failed: '%s'. Cannot recover from xrun\n", snd_strerror(err));
            }
         }
         else {
            /*
             Nothing to do, XRUN handled next time we called
             bpd_snd_card_read() or bpd_snd_card_write()
             when we check that stream is in correct state.
            */
         }
         break;
      }
      bcm_pr_debug("%s() failed with error %ld : '%s'\n",
         function, (long)(error), snd_strerror(error));
      if (-ESTRPIPE == error) {
         /* Device is in suspend state */
         int err = 0;
         for (;;) {
            int err = snd_pcm_resume(t->card);
            if (-EAGAIN != err) {
               break;
            }
            usleep(4000);
         }
         if (err) {
            err = snd_pcm_prepare(t->card);
            if (err) {
               bcm_pr_err("snd_pcm_prepare() failed: '%s'.Cannot from suspend\n", snd_strerror(err));
            }
         }
         break;
      }
      if ((-ENODEV == error) || (-ENOTTY == error)) {
         ret = true;
         break;
      }
   } while (false);

   return (ret);
}

static void bpd_snd_card_start(bpd_snd_card_t *t)
{
   int err = snd_pcm_prepare(t->card);
   if (err) {
      bcm_pr_debug("snd_pcm_prepare() failed: '%s'\n", snd_strerror(err));
   }
   else {
      err = snd_pcm_start(t->card);
      if (err) {
         bcm_pr_debug("snd_pcm_start() failed: '%s'\n", snd_strerror(err));
      }
      else {
         t->started = true;
      }
   }
}

static void bpd_snd_card_stop(bpd_snd_card_t *t)
{
   int err = snd_pcm_drop(t->card);
   if (err) {
      bcm_pr_debug("snd_pcm_drop() failed: '%s'\n", snd_strerror(err));
   }
   else {
      t->started = false;
   }
}

static inline bool bpd_snd_card_is_started(bpd_snd_card_t *t)
{
   return (t->started);
}

static void bpd_snd_card_drain(bpd_snd_card_t *t)
{
   int err = snd_pcm_drain(t->card);
   if (err) {
      bcm_pr_debug("snd_pcm_drain() failed: '%s'\n", snd_strerror(err));
   }
}

static int bpd_snd_card_read(bpd_snd_card_t *t, __u8 *buf, size_t len)
{
   snd_pcm_state_t state;
   snd_pcm_sframes_t read;
   snd_pcm_uframes_t to_read;

   bcm_assert((NULL != buf) && (len > 0));

   state = snd_pcm_state(t->card);
   if ((state != SND_PCM_STATE_PREPARED) && (state != SND_PCM_STATE_RUNNING)) {
      int err = snd_pcm_prepare(t->card);
      if (err) {
         bcm_pr_debug("snd_pcm_prepare() failed: '%s'\n", snd_strerror(err));
      }
   }

   to_read = len / SAMPLE_SIZE;
   read = snd_pcm_readi(t->card, buf, to_read);
   if (read < 0) {
      if (!bpd_snd_card_handle_error(t, read, "snd_pcm_readi")) {
         read = 0;
      }
   }
   else {
      read *= SAMPLE_SIZE;
   }

   return (read);
}

static int bpd_snd_card_write(bpd_snd_card_t *t, const __u8 *buf, size_t len)
{
   snd_pcm_state_t state;
   snd_pcm_sframes_t written;
   snd_pcm_uframes_t to_write;

   bcm_assert((NULL != buf) && (len > 0));

   /* We test if playback card is started */
   state = snd_pcm_state(t->card);
   if ((state != SND_PCM_STATE_PREPARED) && (state != SND_PCM_STATE_RUNNING)) {
      int err = snd_pcm_prepare(t->card);
      if (err) {
         bcm_pr_debug("snd_pcm_prepare() failed: '%s'\n", snd_strerror(err));
      }
   }
   to_write = len / SAMPLE_SIZE;
   written = snd_pcm_writei(t->card, buf, to_write);
   if (written < 0) {
      if (!bpd_snd_card_handle_error(t, written, "snd_pcm_writei")) {
         written = 0;
      }
   }
   else {
      written *= SAMPLE_SIZE;
   }
   return (written);
}

typedef struct {
   bool off_hook;
   char digits[64];
   size_t index_digit;
   size_t digits_count;

   /* Handle of sound capture device */
   bpd_snd_card_t snd_capture;
   __u8 *buf_capture;
   size_t buf_capture_len_max;
   size_t buf_capture_len;
   size_t buf_capture_offset;
   /* Handle of sound playback device */
   bpd_snd_card_t snd_playback;
   __u8 *buf_playback;
   size_t buf_playback_len_max;
   size_t buf_playback_len;
   size_t buf_playback_offset;

   /* File descriptor to device */
   int fd_dev;

   /* Tone currently playing */
   char tone;
   bcmph_dual_tone_generator_t tone_generator;

   struct {
      size_t dev_read;
      size_t dev_written;
      size_t snd_read;
      size_t snd_written;
   } stats;
} bpd_t;

static bool bpd_is_sending_tone(const bpd_t *t)
{
   return (('\0' != t->tone) ? true : false);
}

static void bpd_set_line_tone(bpd_t *t, char tone)
{
   static const int vol = 7219; /* Default to -8db */

   bcm_pr_debug("%s(tone=%c)\n", __func__,
      (int)(('\0' != tone) ? tone : ' '));

   if ('\0' != t->tone) {
      (*(t->tone_generator.sound_generator.vtbl->deinit))(&(t->tone_generator.sound_generator));
      t->tone = '\0';
   }

   switch (tone) {
      case '\0': {
         break;
      }
      case '0': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_0), vol);
         break;
      }
      case '1': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_1), vol);
         break;
      }
      case '2': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_2), vol);
         break;
      }
      case '3': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_3), vol);
         break;
      }
      case '4': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_4), vol);
         break;
      }
      case '5': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_5), vol);
         break;
      }
      case '6': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_6), vol);
         break;
      }
      case '7': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_7), vol);
         break;
      }
      case '8': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_8), vol);
         break;
      }
      case '9': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_9), vol);
         break;
      }
      case '*': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_aster), vol);
         break;
      }
      case '#': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_pound), vol);
         break;
      }
      case 'A': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_A), vol);
         break;
      }
      case 'B': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_B), vol);
         break;
      }
      case 'C': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_C), vol);
         break;
      }
      case 'D': {
         bcmph_dual_tone_generator_init(&(t->tone_generator), &(bpd_tone_dtmf_D), vol);
         break;
      }
      default: {
         bcm_assert(false);
         tone = '\0';
         break;
      }
   }
   t->tone = tone;
}

static size_t bpd_write_tone_data(bpd_t *t)
{
   size_t ret = 0;

   /* bcm_pr_debug("%s()\n", __func__); */

   bcm_assert('\0' != t->tone);

   for (;;) {
      size_t needed = (t->buf_capture_len_max - t->buf_capture_len) / SAMPLE_SIZE;
      if (needed > 0) {
         bcm_assert(0 == (t->buf_capture_len % SAMPLE_SIZE));
         __s16 *ptr = (__s16 *)(t->buf_capture + t->buf_capture_len);
         size_t generated;

         generated = (*(t->tone_generator.sound_generator.vtbl->write))(&(t->tone_generator.sound_generator), ptr, needed);
         if (generated > 0) {
            bcm_assert(0 == (generated % SAMPLE_SIZE));
            t->buf_capture_len += (generated * SAMPLE_SIZE);
            ret += generated;
         }
         if (generated < needed) {
            bcm_assert(t->index_digit < t->digits_count);
            t->index_digit += 1;
            if (t->index_digit < t->digits_count) {
               bpd_set_line_tone(t, t->digits[t->index_digit]);
            }
            else {
               bpd_set_line_tone(t, '\0');
               break;
            }
         }
         else {
            break;
         }
      }
      else {
         break;
      }
   }
   return (ret * SAMPLE_SIZE);
}

static inline bool bpd_is_off_hook(const bpd_t *t)
{
   return (t->off_hook);
}

static void bpd_transfer_data(bpd_t *t)
{
   if (bpd_is_off_hook(t)) {
      if (bpd_is_sending_tone(t)) {
         if (bpd_snd_card_is_started(&(t->snd_capture))) {
            bpd_snd_card_stop(&(t->snd_capture));
         }

         /* Generate tone data and put it in ring buffer */
         bpd_write_tone_data(t);
      }
      else {
         if (!bpd_snd_card_is_started(&(t->snd_capture))) {
            bpd_snd_card_start(&(t->snd_capture));
         }

         /* Then we fill ring buffer with data from capture device */
         if (t->buf_capture_len < t->buf_capture_len_max) {
            int read = bpd_snd_card_read(&(t->snd_capture), t->buf_capture + t->buf_capture_len, t->buf_capture_len_max - t->buf_capture_len);
            if (read >= 0) {
               bcm_assert(0 == (read % SAMPLE_SIZE));
               t->buf_capture_len += read;
               t->stats.snd_read += read;
            }
         }
      }
      /* And write data to dev */
      if (t->buf_capture_offset < t->buf_capture_len) {
         int written = write(t->fd_dev, t->buf_capture + t->buf_capture_offset, t->buf_capture_len - t->buf_capture_offset);
         if (written >= 0) {
            t->buf_capture_offset += written;
            if (t->buf_capture_offset >= t->buf_capture_len) {
               t->buf_capture_offset = 0;
               t->buf_capture_len = 0;
            }
            t->stats.dev_written += written;
         }
         else {
            if ((EAGAIN != errno) && (EWOULDBLOCK != errno) && (EINTR != errno)) {
               bcm_pr_err("Error writing dev device : %d\n", errno);
            }
         }
      }
   }

   /* Read data from dev */
   if (t->buf_playback_len < t->buf_playback_len_max) {
      int readd = read(t->fd_dev, t->buf_playback + t->buf_playback_len, t->buf_playback_len_max - t->buf_playback_len);
      if (readd >= 0) {
         t->buf_playback_len += readd;
         t->stats.dev_read += readd;
      }
      else {
         if ((EAGAIN != errno) && (EWOULDBLOCK != errno) && (EINTR != errno)) {
            bcm_pr_err("Error reading dev device : %d\n", errno);
         }
      }
   }

   /* And write to playback dev */
   if (t->buf_playback_offset < t->buf_playback_len) {
      int written = bpd_snd_card_write(&(t->snd_playback), t->buf_playback + t->buf_playback_offset, t->buf_playback_len - t->buf_playback_offset);
      if (written >= 0) {
         t->buf_playback_offset += written;
         if (t->buf_playback_offset >= t->buf_playback_len) {
            t->buf_playback_offset = 0;
            t->buf_playback_len = 0;
         }
         t->stats.snd_written += written;
      }
   }
}

static void bpd_hook_off(bpd_t *t)
{
   int ret;
   bcm_phone_set_line_state_t set_line_state;

   bcm_pr_debug("%s()\n", __func__);

   memset(&(set_line_state), 0, sizeof(set_line_state));
   set_line_state.status = BCMPH_STATUS_OFF_HOOK;
   ret = bpd_do_ioctl(t->fd_dev, BCMPH_IOCTL_SET_LINE_STATE, (unsigned long)(&(set_line_state)), true);
   if (!ret) {
      t->off_hook = true;
      if (t->index_digit < t->digits_count) {
         /* We start sending DTMF tones */
         bpd_set_line_tone(t, t->digits[t->index_digit]);
      }
      else {
         bpd_snd_card_start(&(t->snd_capture));
      }
   }
}

static void bpd_hook_on(bpd_t *t)
{
   int ret;
   bcm_phone_set_line_state_t set_line_state;

   bcm_pr_debug("%s()\n", __func__);

   memset(&(set_line_state), 0, sizeof(set_line_state));
   set_line_state.status = BCMPH_STATUS_ON_HOOK;
   ret = bpd_do_ioctl(t->fd_dev, BCMPH_IOCTL_SET_LINE_STATE, (unsigned long)(&(set_line_state)), true);
   if (!ret) {
      bpd_set_line_tone(t, '\0');
      t->digits_count = 0;
      t->index_digit = 0;
      bpd_snd_card_stop(&(t->snd_capture));
      t->off_hook = false;
   }
}


static inline void bpd_toggle_hook_state(bpd_t *t)
{
   if (bpd_is_off_hook(t)) {
      bpd_hook_on(t);
   }
   else {
      bpd_hook_off(t);
   }
}

static void bpd_add_ch(bpd_t *t, char c)
{
   if ('\0' != c) {
      if ((t->index_digit > 0) && (t->digits_count >= ARRAY_SIZE(t->digits))) {
         memmove(t->digits, &(t->digits[1]), (t->digits_count - 1) * sizeof(t->digits[0]));
         t->index_digit -= 1;
         t->digits_count -= 1;
      }
      if (t->digits_count < ARRAY_SIZE(t->digits)) {
         t->digits[t->digits_count] = c;
         t->digits_count += 1;
         if ((bpd_is_off_hook(t)) && (!bpd_is_sending_tone(t))) {
            bpd_set_line_tone(t, t->digits[t->index_digit]);
         }
      }
   }
}

static void bpd_remove_last_ch(bpd_t *t)
{
   if (t->digits_count > 0) {
      if (((t->index_digit + 1) < t->digits_count)
          || (((t->index_digit + 1) == t->digits_count) && (!bpd_is_sending_tone(t)))) {
         t->digits_count -= 1;
      }
   }
}

static int bpd_init(bpd_t *t,
   const char *snd_capture_dev_name, const char *snd_playback_dev_name,
   const char *dev_name)
{
   int ret = 0;
   size_t size;

   memset(t, 0, sizeof(*t));
#ifndef BCMPH_EXPORT_DEV_FILE
   t->off_hook = false;
#else /* BCMPH_EXPORT_DEV_FILE */
   t->off_hook = true;
#endif /* BCMPH_EXPORT_DEV_FILE */
   t->fd_dev = -1;

   /* Alloc memory for buffers */
   size = PERIOD_SIZE_IN_FRAMES * SAMPLE_SIZE;
   t->buf_capture = malloc(size * 2);
   if (NULL == t->buf_capture) {
      goto fail_alloc_buffer;
   }
   t->buf_capture_len_max = size;
   t->buf_capture_len = 0;
   t->buf_capture_offset = 0;
   t->buf_playback = t->buf_capture + size;
   t->buf_playback_len_max = size;
   t->buf_playback_len = 0;
   t->buf_playback_offset = 0;

   /* Init sound cards */
   ret = bpd_snd_card_init(&(t->snd_capture), snd_capture_dev_name,
      SND_PCM_STREAM_CAPTURE);
   if (ret) {
      bcm_pr_err("Problem opening ALSA capture device '%s'\n", snd_capture_dev_name);
      goto fail_open_captude_dev;
   }

   ret = bpd_snd_card_init(&(t->snd_playback), snd_playback_dev_name,
      SND_PCM_STREAM_PLAYBACK);
   if (ret) {
      bcm_pr_err("Problem opening ALSA playback device '%s'\n", snd_playback_dev_name);
      goto fail_open_playback_dev;
   }

   /* Open file to access device */
   t->fd_dev = open(dev_name, O_RDWR | O_NONBLOCK);
   if (-1 == t->fd_dev) {
      ret = errno;
      perror("");
      bcm_pr_err("Error opening device '%s'\n", dev_name);
      goto fail_open_dev;
   }

   t->tone = '\0';

   return (ret);

   close(t->fd_dev);
   t->fd_dev = -1;
fail_open_dev:
   bpd_snd_card_deinit(&(t->snd_playback));
fail_open_playback_dev:
   bpd_snd_card_deinit(&(t->snd_capture));
fail_open_captude_dev:
   free(t->buf_capture);
   t->buf_capture = NULL;
   t->buf_capture_len_max = 0;
   t->buf_capture_len = 0;
   t->buf_capture_offset = 0;
   t->buf_playback = NULL;
   t->buf_playback_len_max = 0;
   t->buf_playback_len = 0;
   t->buf_playback_offset = 0;
fail_alloc_buffer:
   return (ret);
}

static void bpd_deinit(bpd_t *t)
{
   if (bpd_is_off_hook(t)) {
      bpd_hook_on(t);
   }

   /* Close file allowing access to device */
   close(t->fd_dev);
   t->fd_dev = -1;

   /* Deinit sound cards */
   bpd_snd_card_deinit(&(t->snd_playback));
   bpd_snd_card_deinit(&(t->snd_capture));

   /* Free memory allocated for buffers */
   free(t->buf_capture);
   t->buf_capture = NULL;
   t->buf_capture_len_max = 0;
   t->buf_capture_len = 0;
   t->buf_capture_offset = 0;
   t->buf_playback = NULL;
   t->buf_playback_len_max = 0;
   t->buf_playback_len = 0;
   t->buf_playback_offset = 0;


   memset(t, 0, sizeof(*t));
   t->fd_dev = -1;
}

typedef struct {
   const bpd_t *model;
   WINDOW *win;
   int win_lines;
   int win_cols;
} bpd_ui_t;

#define WIN_HOOK_1_ST_LINE 1
#define WIN_HOOK_1_ST_COL 2
#define WIN_HOOK_WIDTH 14
#define WIN_HOOK_HEIGHT 3

#define WIN_STATS_SND_1_ST_LINE 1
#define WIN_STATS_SND_1_ST_COL ((WIN_HOOK_1_ST_COL) + (WIN_HOOK_WIDTH) + 2)
#define WIN_STATS_SND_WIDTH 25
#define WIN_STATS_SND_HEIGHT 3

#define WIN_STATS_DEV_1_ST_LINE 1
#define WIN_STATS_DEV_1_ST_COL ((WIN_STATS_SND_1_ST_COL) + (WIN_STATS_SND_WIDTH) + 2)
#define WIN_STATS_DEV_WIDTH 25
#define WIN_STATS_DEV_HEIGHT 3

#define WIN_DIGITS_1_ST_LINE ((WIN_HOOK_1_ST_LINE) + (WIN_HOOK_HEIGHT) + 1)
#define WIN_DIGITS_1_ST_COL 2
#define WIN_DIGITS_HEIGHT 3

#define WIN_CONSOLE_1_ST_LINE ((WIN_DIGITS_1_ST_LINE) + (WIN_DIGITS_HEIGHT) + 1)
#define WIN_CONSOLE_1_ST_COL 2
#define WIN_CONSOLE_WIDTH 76
#define WIN_CONSOLE_HEIGHT 14

static inline int bpd_ui_getch(bpd_ui_t *t)
{
   return (wgetch(t->win));
}

static inline void bpd_ui_wrefresh(bpd_ui_t *t)
{
   wrefresh(t->win);
}

static void bpd_ui_refresh_digits(bpd_ui_t *t)
{
   size_t i;
   int line = WIN_DIGITS_1_ST_LINE + 1;
   int col = WIN_DIGITS_1_ST_COL + 2;
   wmove(t->win, line, col);
   for (i = 0; (i < t->model->digits_count); i += 1) {
      if ((bpd_is_sending_tone(t->model)) && (i == t->model->index_digit)) {
         wattr_on(t->win, WA_BOLD, NULL);
         waddch(t->win, t->model->digits[i]);
         wattr_off(t->win, WA_BOLD, NULL);
      }
      else {
         waddch(t->win, t->model->digits[i]);
      }
   }
   for (; (i < ARRAY_SIZE(t->model->digits)); i += 1) {
      waddch(t->win, ' ');
   }
   bpd_ui_wrefresh(t);
}

static void bpd_ui_refresh_hook_state(bpd_ui_t *t)
{
   int line = WIN_HOOK_1_ST_LINE + 1;
   int col = WIN_HOOK_1_ST_COL + 2;
   wmove(t->win, line, col);
   if (bpd_is_off_hook(t->model)) {
      waddstr(t->win, "OFF");
   }
   else {
      waddstr(t->win, "ON ");
   }
   bpd_ui_wrefresh(t);
}

static void bpd_ui_refresh_stats(bpd_ui_t *t)
{
   int line = WIN_STATS_SND_1_ST_LINE + 1;
   int col = WIN_STATS_SND_1_ST_COL + 2;
   wmove(t->win, line, col);
   if (bpd_is_sending_tone(t->model)) {
      wprintw(t->win, "%10u*%10u", t->model->stats.snd_read, t->model->stats.snd_written);
   }
   else {
      wprintw(t->win, "%10u/%10u", t->model->stats.snd_read, t->model->stats.snd_written);
   }

   line = WIN_STATS_DEV_1_ST_LINE + 1;
   col = WIN_STATS_DEV_1_ST_COL + 2;
   wmove(t->win, line, col);
   wprintw(t->win, "%10u/%10u", t->model->stats.dev_read, t->model->stats.dev_written);
   bpd_ui_wrefresh(t);
}

static void bpd_win_draw_frame(WINDOW *win, int line, int col, int width, int height, const char *title)
{
   int i;
   int iMax;
   bcm_assert((width >= 4) && (height >= 3))
   wmove(win, line, col);
   waddch(win, ACS_ULCORNER);
   waddch(win, ' ');
   waddstr(win, title);
   waddch(win, ' ');
   iMax = width - 1;
   for (i = 3 + strlen(title); (i < iMax); i += 1) {
      waddch(win, ACS_HLINE);
   }
   waddch(win, ACS_URCORNER);
   iMax = height - 1;
   for (i = 1; (i < iMax); i += 1) {
      wmove(win, line + i, col);
      waddch(win, ACS_VLINE);
      wmove(win, line + i, col + width - 1);
      waddch(win, ACS_VLINE);
   }
   wmove(win, line + height - 1, col);
   waddch(win, ACS_LLCORNER);
   iMax = width - 1;
   for (i = 1; (i < iMax); i += 1) {
      waddch(win, ACS_HLINE);
   }
   waddch(win, ACS_LRCORNER);
   wrefresh(win);
}

static void bpd_ui_refresh(bpd_ui_t *t)
{
   getmaxyx(t->win, t->win_lines, t->win_cols);
   wclear(t->win);
   bpd_win_draw_frame(t->win, WIN_HOOK_1_ST_LINE, WIN_HOOK_1_ST_COL, WIN_HOOK_WIDTH, WIN_HOOK_HEIGHT, "Hook state");
   bpd_win_draw_frame(t->win, WIN_STATS_SND_1_ST_LINE, WIN_STATS_SND_1_ST_COL, WIN_STATS_SND_WIDTH, WIN_STATS_SND_HEIGHT, "Sound card read/write");
   bpd_win_draw_frame(t->win, WIN_STATS_DEV_1_ST_LINE, WIN_STATS_DEV_1_ST_COL, WIN_STATS_DEV_WIDTH, WIN_STATS_DEV_HEIGHT, "Driver read/write");
   bpd_win_draw_frame(t->win, WIN_DIGITS_1_ST_LINE, WIN_DIGITS_1_ST_COL, 4 + ARRAY_SIZE(t->model->digits), WIN_DIGITS_HEIGHT, "Digits");
   bpd_ui_refresh_hook_state(t);
   bpd_ui_refresh_stats(t);
   bpd_ui_refresh_digits(t);
}

static void bpd_ui_init(bpd_ui_t *t, bpd_t *model)
{
   memset(t, 0, sizeof(*t));
   t->win = initscr();
   cbreak();
   noecho();
   nl();
   curs_set(0);
   ESCDELAY = 0;
   intrflush(t->win, FALSE);
   keypad(t->win, TRUE);
   nodelay(t->win, TRUE);
   t->model = model;
   bpd_ui_refresh(t);
}

static void bpd_ui_deinit(bpd_ui_t *t)
{
   endwin();
}

typedef struct {
   bpd_t *model;
   bpd_ui_t view;
} bpd_ctrl_t;


static void bpd_ctrl_init(bpd_ctrl_t *t, bpd_t *model)
{
   t->model = model;
   bpd_ui_init(&(t->view), t->model);
}

static void bpd_ctrl_deinit(bpd_ctrl_t *t)
{
   bpd_ui_deinit(&(t->view));
   t->model = NULL;
}

#define USLEEP_TIME 8000 /* 8 ms */
#define GETCH_PERIOD (1000000 / (USLEEP_TIME * 10))
#define REFRESH_STATS_PERIOD (1000000 / (USLEEP_TIME * 2))
#define REFRESH_DIGITS_PERIOD (1000000 / (USLEEP_TIME * 20))

static void bpd_ctrl_run(bpd_ctrl_t *t)
{
   int getch_count = GETCH_PERIOD;
   int refresh_stats_count = REFRESH_STATS_PERIOD;
   int refresh_digits_count = REFRESH_DIGITS_PERIOD;

   for (;;) {
      getch_count -= 1;
      if (getch_count <= 0) {
         int ch;
         bool exit_loop = false;

         getch_count = GETCH_PERIOD;
         ch = bpd_ui_getch(&(t->view));
         if (ERR != ch) {
            switch (ch) {
               case 'q':
               case 'Q': {
                  /* Exit from the program. If off hook, toggle hook state before exiting */
                  exit_loop = true;
                  break;
               }
               case 'a':
               case 'A':
               case 'b':
               case 'B':
               case 'c':
               case 'C':
               case 'd':
               case 'D': {
                  bpd_add_ch(t->model, toupper(ch));
                  bpd_ui_refresh_digits(&(t->view));
                  break;
               }
               case '*':
               case '#':
               case '0':
               case '1':
               case '2':
               case '3':
               case '4':
               case '5':
               case '6':
               case '7':
               case '8':
               case '9': {
                  bpd_add_ch(t->model, ch);
                  bpd_ui_refresh_digits(&(t->view));
                  break;
               }
               case KEY_LEFT:
               case KEY_BACKSPACE: {
                  /* Erase last digit entered if on hook */
                  bpd_remove_last_ch(t->model);
                  bpd_ui_refresh_digits(&(t->view));
                  break;
               }
               case 'h':
               case 'H': {
                  /* Toggle hook state */
                  bpd_toggle_hook_state(t->model);
                  bpd_ui_refresh_hook_state(&(t->view));
                  bpd_ui_refresh_digits(&(t->view));
                  break;
               }
               case KEY_RESIZE: {
                  bpd_ui_refresh(&(t->view));
                  break;
               }
            }
            refresh_digits_count = REFRESH_DIGITS_PERIOD;
         }
         if (exit_loop) {
            break;
         }
      }
      usleep(USLEEP_TIME);
      bpd_transfer_data(t->model);
      refresh_stats_count -= 1;
      if (refresh_stats_count <= 0) {
         refresh_stats_count = REFRESH_STATS_PERIOD;
         bpd_ui_refresh_stats(&(t->view));
      }
      if (bpd_is_sending_tone(t->model)) {
         refresh_digits_count -= 1;
         if (refresh_digits_count <= 0) {
            refresh_digits_count = REFRESH_DIGITS_PERIOD;
            bpd_ui_refresh_digits(&(t->view));
         }
      }
   }
}

static void parse_option(const char *option, const char **name, size_t *name_len, const char **value)
{
   const char *tmp;

   while (isspace(*option)) {
      option += 1;
   }
   *name = option;

   tmp = strchr(*name, '=');
   if (NULL != tmp) {
      *value = tmp + 1;
   }
   else {
      *value = NULL;
      tmp = *name;
      tmp += strlen(tmp);
   }
   *name_len = tmp - *name;
}

static const char prm_name_capture[] = "capture";
static const char prm_name_playback[] = "playback";
static const char prm_name_test_alsa[] = "test_alsa";

static void print_usage(void)
{
   fprintf(stdout, "Usage : bcm63xx-phone-debug arg1 arg2 arg3... where arguments are one of\n");
   fprintf(stdout, " %s=no default value : the name of the capture sound card, mandatory argument\n", prm_name_capture);
   fprintf(stdout, " %s=no default value : the name of the playback sound card, mandatory argument\n", prm_name_playback);
}

static void do_test_alsa(bpd_t *t)
{
   bpd_add_ch(t, '0');
   bpd_add_ch(t, '1');
   bpd_add_ch(t, '2');
   bpd_add_ch(t, '3');
   bpd_set_line_tone(t, t->digits[t->index_digit]);
   while (bpd_is_sending_tone(t)) {
      bpd_write_tone_data(t);
      if (t->buf_capture_offset < t->buf_capture_len) {
         int written = bpd_snd_card_write(&(t->snd_playback), t->buf_capture + t->buf_capture_offset, t->buf_capture_len - t->buf_capture_offset);
         if (written >= 0) {
            t->buf_capture_offset += written;
            if (t->buf_capture_offset >= t->buf_capture_len) {
               t->buf_capture_offset = 0;
               t->buf_capture_len = 0;
            }
         }
         else {
            break;
         }
         usleep(20000);
      }
      else {
         break;
      }
   }
   bpd_snd_card_stop(&(t->snd_playback));
}

int main(int argc, char **argv)
{
   int ret = 0;
   const char *snd_capture_dev_name = NULL;
   const char *snd_playback_dev_name = NULL;
   bool test_alsa = false;
   bpd_t model;
   bpd_ctrl_t controler;

   if (argc > 1) {
      int i;
      const char *name;
      size_t name_len;
      const char *value;
      for (i = 1; (i < argc); i += 1) {
         parse_option(argv[i], &(name), &(name_len), &(value));
         if ((NULL == value) || ('\0' == value[0])) {
            bcm_pr_info("Arg '%s' is ignored because it has no value\n", argv[i]);
            continue;
         }
         if (((ARRAY_SIZE(prm_name_capture) - 1) == name_len) && (0 == strncasecmp(name, prm_name_capture, name_len))) {
            snd_capture_dev_name = value;
            continue;
         }
         if (((ARRAY_SIZE(prm_name_playback) - 1) == name_len) && (0 == strncasecmp(name, prm_name_playback, name_len))) {
            snd_playback_dev_name = value;
            continue;
         }
         if (((ARRAY_SIZE(prm_name_test_alsa) - 1) == name_len) && (0 == strncasecmp(name, prm_name_test_alsa, name_len))) {
            test_alsa = true;
            continue;
         }
         bcm_pr_info("Arg '%s' is ignored because it's unknown\n", argv[i]);
      }
   }
   if ((NULL == snd_capture_dev_name) || ('\0' == snd_capture_dev_name[0])) {
      bcm_pr_err(" Missing value for parameter '%s'\n", prm_name_capture);
      print_usage();
      return (-1);
   }
   if ((NULL == snd_playback_dev_name) || ('\0' == snd_playback_dev_name[0])) {
      bcm_pr_err(" Missing value for parameter '%s'\n", prm_name_playback);
      print_usage();
      return (-1);
   }

   bpd_init_tones();
   if (!test_alsa) {
      if (!bpd_init(&(model), snd_capture_dev_name, snd_playback_dev_name, bcm63xx_phone_dev_name)) {
         bpd_console_t console;
         WINDOW *win_console;

         bpd_ctrl_init(&(controler), &(model));

         win_console = newwin(WIN_CONSOLE_HEIGHT - 2, WIN_CONSOLE_WIDTH - 4, WIN_CONSOLE_1_ST_LINE + 1, WIN_CONSOLE_1_ST_COL + 2);
         if (NULL != win_console) {
            bpd_win_draw_frame(controler.view.win, WIN_CONSOLE_1_ST_LINE, WIN_CONSOLE_1_ST_COL, WIN_CONSOLE_WIDTH, WIN_CONSOLE_HEIGHT, "Console");
            bpd_console_init(&(console), win_console);
            bpd_console = &(console);
         }

         bpd_ctrl_run(&(controler));

         if (NULL != win_console) {
            bpd_console = NULL;
            bpd_console_deinit(&(console));
         }

         bpd_ctrl_deinit(&(controler));
         bpd_deinit(&(model));
      }
   }
   else {
      if (!bpd_init(&(model), snd_capture_dev_name, snd_playback_dev_name, "/dev/null")) {
         do_test_alsa(&(model));
         bpd_deinit(&(model));
      }
   }

   return (ret);
}
