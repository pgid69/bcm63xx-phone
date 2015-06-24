/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

/* Documentation about API of Asterisk 1.11 is available
   http://doxygen.asterisk.org/trunk/ */

/* To be included first */
#include <asterisk.h>

ASTERISK_FILE_VERSION(__FILE__, "$Revision: 1 $")

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>

#include <bcm63xx_phone.h>

#include <asterisk/ast_version.h>
#include <asterisk/channel.h>
#include <asterisk/callerid.h>
#include <asterisk/causes.h>
#ifdef BCMPH_NOHW
#include <asterisk/cli.h>
#endif /* BCMPH_NOHW */
#include <asterisk/config.h>
#include <asterisk/dsp.h>
#include <asterisk/format_cap.h>
#include <asterisk/frame.h>
#include <asterisk/linkedlists.h>
#include <asterisk/lock.h>
#include <asterisk/logger.h>
#include <asterisk/module.h>
#include <asterisk/musiconhold.h>
#include <asterisk/pbx.h>
#include <asterisk/strings.h>
#include <asterisk/utils.h>

#define DEFAULT_CALLER_ID "Unknown"

/* The two following values are from the Asterisk code */
#define MIN_DTMF_DURATION 100
#define MIN_TIME_BETWEEN_DTMF 45

/*

 D'apres ce que je comprends le deroulement est le suivant
 * Si l'appel est initie par l'utilisateur :
 celui ci decroche le telephone, ce qui est detecte par le moniteur,
 ce dernier attend ou pas qu'une extension soit tape, appelle bcmph_new()
 ce qui cree un channel dans l'etat AST_STATE_RING, et appelle ast_pbx_start()
 pour la creation d'un thread pour le channel precedemment cree.
 Asterisk appelle ensuite bcmph_chan_answer() ce qui fait passer le channel
 dans l'etat AST_STATE_UP et passe la ligne en mode conversation.
 En fin de conversation hangup() est appelle : le channel est detache
 de la ligne.
 * Si l'appel est recu de l'exterieur :
 Asterisk appelle bcmph_chan_request() qui appelle bcmph_new() ce qui
 cree un channel dans l'etat AST_STATE_DOWN. Pas de thread demarre, c'est
 Asterisk qui s'en chargera plus tard.
 Asterisk appelle ensuite bcmph_chan_call() pour faire sonner le telephone.
 Lorsque l'utilisateur decroche la ligne passe en mode conversation.
 En fin de conversation hangup() est appelle : le channel est detache de
 la ligne.
*/

/* #undef BCMPH_DEBUG */

#undef bcm_assert
#undef bcm_pr_debug
#ifdef BCMPH_DEBUG
#define bcm_assert(cond) if (!(cond)) { ast_log(AST_LOG_DEBUG, "condition '%s' is false\n", #cond); }
#define bcm_pr_debug(fmt, args...) ast_log(AST_LOG_DEBUG, fmt, ## args)
#else /* !BCMPH_DEBUG */
#define bcm_assert(cond)
#define bcm_pr_debug(fmt, args...)
#endif /* !BCMPH_DEBUG */

#if !defined(__cplusplus) && !defined(c_plusplus)
typedef int bool;

#define false 0
#define true 1
#endif /* __cplusplus */

#define AST_MODULE bcmph_chan_type

#include <bcm63xx_line_state.c>
#include <bcm63xx_ring_buf.c>

static const char bcmph_device[] = "/dev/bcm63xx-phone";
static const char bcmph_chan_type[] = "Bcm63xxPhone";
static const char bcmph_chan_desc[] = "Broadcom 63xx Telephony Driver";
static const char bcmph_cfg_file[] = "bcm63xx_phone.conf";

static const char bcmph_default_extension[] = "s";
/*
 Constant short_timeout is used for example when we fail to lock a
 mutex : we set a short timeout for ioctl BCMPH_IOCTL_GET_LINE_STATES
 to retry quicly
*/
static const int bcmph_monitor_short_timeout = 5 /* ms */;
/*
 When no line are in conversation mode, we set a timeout on ioctl
 BCMPH_IOCTL_GET_LINE_STATES of several seconds (but remember that
 this timeout will delay the unloading of the module, more
 specifically, when we stop the monitor)
*/
static const int bcmph_monitor_idle_timeout = 10000; /* ms */

typedef struct {
   bool enable;
   struct ast_format_cap *capabilities;
   bool monitor_dialing;
   int dialing_timeout;
   enum {
      DETECT_DTMF_NEVER = 0,
      DETECT_DTMF_WHEN_DIALING,
      DETECT_DTMF_WHEN_CONNECTED,
      DETECT_DTMF_ALWAYS,
   } detect_dtmf;
   char context[AST_MAX_CONTEXT];
   char cid_name[AST_MAX_EXTENSION];
   char cid_num[AST_MAX_EXTENSION];
   char moh_interpret[MAX_MUSICCLASS];
} bcmph_line_config_t;

typedef struct {
   const char *language;
   bcmph_country_t country;
   size_t line_count;
   bcmph_line_config_t line_cfgs[BCMPH_MAX_LINES];
   int monitor_busy_period;
} bcmph_chan_config_t;

/*
- The events that can occur on a line
-*/
typedef enum {
   /* The user hooks off the phone */
   BCMPH_EV_OFF_HOOK,
   /* The user hooks on the phone */
   BCMPH_EV_ON_HOOK,
   /* The phone is disconnected */
   BCMPH_EV_DISCONNECTED,
   /* An extension has been found */
   BCMPH_EV_EXT_FOUND,
   /* No extension can be found */
   BCMPH_EV_NO_EXT_CAN_BE_FOUND,
   /* Internal error */
   BCMPH_EV_INTERNAL_ERROR,
   /* Asterisk calls bcmph_chan_request() */
   BCMPH_EV_AST_REQUEST,
   /* Asterisk calls bcmph_chan_call() */
   BCMPH_EV_AST_CALL,
   /* Asterisk calls bcmph_chan_answer() */
   BCMPH_EV_AST_ANSWER,
   /* Asterisk calls bcmph_chan_hangup() */
   BCMPH_EV_AST_HANGUP,
} bcmph_event_t;

/*
 The logical states of a line
*/
typedef enum {
   /* Phone is disconnected */
   BCMPH_ST_DISCONNECTED,
   /* Phone is on hook */
   BCMPH_ST_ON_IDLE,
   /* Phone is on hook and Asterisk calls bcmph_chan_request() */
   BCMPH_ST_ON_PRE_RINGING,
   /*
    Phone is on hook and while we were in state BCMPH_ST_ON_PRE_RINGING,
    Asterisk calls bcmph_chan_call()
   */
   BCMPH_ST_ON_RINGING,
   /*
    Phone is off hook and while we were in state BCMPH_ST_ON_IDLE,
    the user hook off the phone. This state is entered only if
    line_cfg->monitor_dialing is true
   */
   BCMPH_ST_OFF_DIALING,
   /*
    Phone is off hook.
    We can switch to this state from the state BCMPH_ST_OFF_DIALING, if
    we have found a valid extension, or from the state BCMPH_ST_ON_IDLE if the
    user hook off the phone and line_cfg->monitor_dialing is false
   */
   BCMPH_ST_OFF_WAITING_ANSWER,
   /*
    Phone is off hook and we are sending and receiving audio.
    We can switch to this state from the state BCMPH_ST_OFF_WAITING_ANSWER when
    Asterisk calls bcmph_chan_answer(), or from the state BCMPH_ST_ON_RINGING
    if the user hook off the phone
   */
   BCMPH_ST_OFF_TALKING,
   /*
    Phone is off hook and we are waiting the user to hook on the phone.
    We can switch to this state when Asterisk calls bcmph_chan_hangup()
    or if there's an internal error
   */
   BCMPH_ST_OFF_NO_SERVICE,
} bcmph_state_t;

/*
 BCMPH_MIN_SIZE_RING_BUFFER is the minimum size of the ring buffer of the
 drivers containing slin16 encoded data.
 That represents (BCMPH_MIN_SIZE_RING_BUFFER / (8 * 4)) msecs
 BCMPH_MAX_BUF is halved to represent the same time but for slin encoded data
*/
#define BCMPH_MAX_BUF (BCMPH_MIN_SIZE_RING_BUFFER / 2)

typedef struct bcmph_pvt
{
   AST_LIST_ENTRY(bcmph_pvt) list;
   struct bcmph_chan *channel;
   /* Index of the line in array channel->config.line_cfgs */
   size_t index_line;
   /* Configuration of the line */
   const bcmph_line_config_t *line_cfg;

   /*
    Asterisk channel linked to this line :
    any change of this field is protected by the monitor's lock and the channel
    lock at the same time, so read of this field is safe whenever one of the
    lock is acquired.
   */
   struct ast_channel *owner;
#ifdef BCMPH_DEBUG
   int owner_lock_count;
#endif /* BCMPH_DEBUG */

   /*
    The following fields are used by the monitor.
    If the monitor is running they must be accessed under the protection of
    the monitor's lock
   */
   struct {
      /*
       Store and accumulate the state of line states in case we can't lock the
       channel in the monitor loop
      */
      bcm_phone_line_state_t line_state;
   } monitor;

   /*
    The following fields must be accessed under the protection of the
    ast_channel's lock (if an ast_channel is associated with this line)
   */
   struct {
      bcm_phone_line_status_t current_status;
      bcm_phone_line_mode_t current_mode;
      bcm_phone_line_tone_t current_tone;
      /* DSP processor used to detect DTMF digits in data read from the driver */
      struct ast_dsp *dsp;
      /* Logical state */
      bcmph_state_t state;
      /*
       Used to accumulate digits dialed to recognize an extension when
       dialing, or to send dtmfs when in conversation (because we can't send
       DTMFs too quickly, we must use a buffer to store them)
      */
      char digits[AST_MAX_EXTENSION + 1];
      size_t digits_len;
      /*
       When dialing flag set to true when a new digit is dialed, meaning we can
       search for an extension.
       Flag reset to false when search has been done
      */
      bool search_extension;
      /*
       When in conversation flag set to NONE, ON or OFF to handle
       sending of DTMF.
      */
      enum {
         NONE, ON, OFF
      } dtmf_sent;
      /*
       Used to wait a minimum delay :
       - when dialing between last digit dialed and the search for
       an extension
       - when in conversation, between the queuing of two DTMFs
      */
      struct timeval tv_last_digit;
      /* Last output format */
      struct ast_format current_format;
      /* Size of sample */
      size_t bytes_per_sample;
      /* Size of frames (in bytes) read from the driver and sent to Asterisk */
      size_t frame_size_read;
      /* Frame used when calling ast_queue_frame(), with its buffer */
      struct ast_frame frame_to_queue;
      __u8 buf_fr_to_queue[BCMPH_MAX_BUF];
      /* Frame used in bcmph_chan_read(), with its buffer */
      struct ast_frame frame;
      __u8 buf_fr[BCMPH_MAX_BUF + AST_FRIENDLY_OFFSET];
      /*
       Buffer used to hold an incomplete sample in order to only write complete
       sample to the driver
      */
      __u8 bytes_not_written[4];
      size_t bytes_not_written_len;

      /*
       The following fields must be accessed under the protection of the
       ast_channel's lock (if an ast_channel is associated with this line)
       AND also under the protection of the mmap's lock
      */
      struct {
         bcm_ring_buf_desc_t *dev_rx_ring_buf_desc;
         /* RX ring buffer containing the last received bytes */
         bcm_ring_buf_t dev_rx_ring_buf;
         bcm_ring_buf_desc_t *dev_tx_ring_buf_desc;
         /* TX ring buffer containing the bytes to send */
         bcm_ring_buf_t dev_tx_ring_buf;
      } mmap;
   } ast_channel;
} bcmph_pvt_t;

typedef struct {
   bool channel_is_locked;
   int timeout;
} bcmph_monitor_prms_t;

typedef struct bcmph_chan
{
   /* Driver config */
   bcmph_chan_config_t config;
   struct ast_channel_tech chan_tech;
   bool channel_registered;
   AST_LIST_HEAD_NOLOCK(pvt_list, bcmph_pvt) pvt_list;

   struct
   {
      /* Flag set to false to stop the monitor */
      volatile bool run;
      /* This is the thread for the monitor which checks for input on the lines
         which are not currently in use.  */
      pthread_t thread;
      ast_mutex_t lock;
#ifdef BCMPH_DEBUG
      int lock_count;
#endif /* BCMPH_DEBUG */
      /* Count the number of lines off hook */
      __u32 line_off_hook_count;
   } monitor;

   /* File descriptor on the kernel driver */
   int dev_fd;

   struct
   {
      /* Description of the mmap region allowing communication with the kernel
       driver */
      bcm_phone_get_mmap_desc_t desc;

      __u8 *dev_start_addr;
      bcm_phone_ioctl_param_t *dev_ioctl_param;

      /* Mutex to manage access to mmap */
      ast_mutex_t lock;
#ifdef BCMPH_DEBUG
      int lock_count;
#endif /* BCMPH_DEBUG */
   } mmap;
} bcmph_chan_t;

/*
 Compare parameter version with Asterisk version returned by
 ast_get_version_num().
 version must respects same format as the value returned by
 ast_get_version_num().
 Returns :
 - a negative value if version < Asterisk version
 - 0 if version == Asterisk version
 - a positive value if version > Asterisk version
*/
static int bcmph_cmp_with_ast_version(const char *version)
{
   int ret;
   const char *ast_version = ast_get_version_num();
   size_t len_ast;
   size_t len;

   bcm_assert((NULL != version) && (NULL != ast_version));
   while (*version == '0') {
      version += 1;
   }
   while (*ast_version == '0') {
      ast_version += 1;
   }
   len = strlen(version);
   len_ast = strlen(ast_version);

   if (len < len_ast) {
      bcm_pr_debug("'%s' is less than Asterisk version ('%s')\n",
         version, ast_version);
      ret = -1;
   }
   else if (len > len_ast) {
      bcm_pr_debug("'%s' is greater than Asterisk version ('%s')\n",
         version, ast_version);
      ret = 1;
   }
   else {
      ret = strcmp(version, ast_version);
#ifdef BCMPH_DEBUG
      if (ret < 0) {
         bcm_pr_debug("'%s' is less than Asterisk version ('%s')\n",
            version, ast_version);
      }
      else if (ret > 0) {
         bcm_pr_debug("'%s' is greater than Asterisk version ('%s')\n",
            version, ast_version);
      }
      else {
         bcm_pr_debug("'%s' is equal to Asterisk version ('%s')\n",
            version, ast_version);
      }
#endif
   }
   return (ret);
}

static int bcmph_do_ioctl(const bcmph_chan_t *t, unsigned long request,
   unsigned long arg, bool retry)
{
   int ret = ioctl(t->dev_fd, request, arg);
   if ((retry) && (ret) && ((EAGAIN == errno) || (EINTR == errno))) {
      ret = ioctl(t->dev_fd, request, arg);
   }
   if (ret < 0) {
      ast_log(AST_LOG_ERROR, "ioctl(%lu) failed. errno=%ld\n",
         (unsigned long)(request), (long)(errno));
   }
   return (ret);
}

/* Must be called with mmap.lock locked */
static void bcmph_update_ring_bufs_desc(bcmph_chan_t *t)
{
   bcmph_pvt_t *pvt;

   bcm_assert(t->mmap.lock_count > 0);

   AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
      bcm_assert(pvt->ast_channel.mmap.dev_rx_ring_buf_desc->len == pvt->ast_channel.mmap.dev_rx_ring_buf.desc.len);
      bcm_ring_buf_desc_copy(pvt->ast_channel.mmap.dev_rx_ring_buf_desc, &(pvt->ast_channel.mmap.dev_rx_ring_buf.desc));
      bcm_assert(pvt->ast_channel.mmap.dev_tx_ring_buf_desc->len == pvt->ast_channel.mmap.dev_tx_ring_buf.desc.len);
      bcm_ring_buf_desc_copy(pvt->ast_channel.mmap.dev_tx_ring_buf_desc, &(pvt->ast_channel.mmap.dev_tx_ring_buf.desc));
      /* bcm_pr_debug("RX : size = %lu, TX : size = %lu\n",
         (unsigned long)(bcm_ring_buf_get_size(&(pvt->ast_channel.mmap.dev_rx_ring_buf))),
         (unsigned long)(bcm_ring_buf_get_size(&(pvt->ast_channel.mmap.dev_tx_ring_buf)))); */
   }
}

/* Must be called with mmap.lock locked */
static inline void bcmph_force_update_ring_bufs_desc(bcmph_chan_t *t)
{
   bcm_assert(t->mmap.lock_count > 0);
   bcmph_do_ioctl(t, BCMPH_IOCTL_UPDATE_RBS, 0, false);
   bcmph_update_ring_bufs_desc(t);
}

static int bcmph_mmap_trylock(bcmph_chan_t *t)
{
   int ret = ast_mutex_trylock(&(t->mmap.lock));
   if (!ret) {
#ifdef BCMPH_DEBUG
      t->mmap.lock_count += 1;
      bcm_assert(t->mmap.lock_count > 0);
#endif /* BCMPH_DEBUG */
      bcmph_force_update_ring_bufs_desc(t);
   }
   return (ret);
}

static void bcmph_mmap_lock(bcmph_chan_t *t)
{
   ast_mutex_lock(&(t->mmap.lock));
#ifdef BCMPH_DEBUG
   t->mmap.lock_count += 1;
   bcm_assert(t->mmap.lock_count > 0);
#endif /* BCMPH_DEBUG */
   bcmph_force_update_ring_bufs_desc(t);
}

static void bcmph_mmap_unlock(bcmph_chan_t *t)
{
#ifdef BCMPH_DEBUG
   bcm_assert(t->mmap.lock_count > 0);
   t->mmap.lock_count -= 1;
#endif /* BCMPH_DEBUG */
   ast_mutex_unlock(&(t->mmap.lock));
}

/* Must be called with monitor.lock locked */
static inline int bcmph_start_pcm(const bcmph_chan_t *t)
{
   int ret;

   bcm_pr_debug("bcmph_start_pcm()\n");

   bcm_assert(t->monitor.lock_count > 0);

   ret = bcmph_do_ioctl(t, BCMPH_IOCTL_START_PCM, 0, true);
   if (ret) {
      ast_log(AST_LOG_ERROR, "Unable to start PCM transfers\n");
   }
   return (ret);
}

/* Must be called with monitor.lock locked */
static inline int bcmph_stop_pcm(const bcmph_chan_t *t)
{
   int ret;

   bcm_pr_debug("bcmph_stop_pcm()\n");

   bcm_assert(t->monitor.lock_count > 0);

   ret = bcmph_do_ioctl(t, BCMPH_IOCTL_STOP_PCM, 0, true);
   if (ret) {
      ast_log(AST_LOG_ERROR, "Unable to stop PCM transfers\n");
   }
   return (ret);
}

/* Must be called with pvt->owner locked */
static int bcmph_set_line_codec(bcmph_pvt_t *pvt,
   const struct ast_format *format,
   bcm_phone_line_mode_t mode, bcm_phone_line_tone_t tone)
{
   int ret;
   bcm_phone_set_line_codec_t set_line_codec;
   size_t bytes_per_sample = 1;
   size_t frame_size_read = (BCMPH_MAX_BUF / 2);

   bcm_assert((NULL != format) && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0)));

   memset(&(set_line_codec), 0, sizeof(set_line_codec));
   set_line_codec.line = pvt->index_line;
   switch (format->id) {
      case AST_FORMAT_ALAW:
         set_line_codec.codec = BCMPH_CODEC_ALAW;
         bytes_per_sample = 1;
         frame_size_read = (BCMPH_MAX_BUF / 2);
         break;
      case AST_FORMAT_ULAW:
         set_line_codec.codec = BCMPH_CODEC_ULAW;
         bytes_per_sample = 1;
         frame_size_read = (BCMPH_MAX_BUF / 2);
         break;
      case AST_FORMAT_SLINEAR:
         set_line_codec.codec = BCMPH_CODEC_LINEAR;
         bytes_per_sample = 2;
         frame_size_read = BCMPH_MAX_BUF;
         break;
      case AST_FORMAT_SLINEAR16:
         set_line_codec.codec = BCMPH_CODEC_LINEAR16;
         /*
          With codec LINEAR16 we receive 16 samples of 2 bytes per ms
          which is equivalent to 8 samples of 4 bytes per ms
         */
         bytes_per_sample = 4;
         frame_size_read = BCMPH_MAX_BUF;
         break;
      default:
         bcm_assert(false);
         break;
   }
   set_line_codec.mode = mode;
   set_line_codec.tone = bcm_phone_line_tone_code_index(tone);

   bcm_pr_debug("bcmph_set_line_codec(codec=%d, mode=%d, tone=%d)\n",
      (int)(set_line_codec.codec), (int)(set_line_codec.mode), (int)(set_line_codec.tone));

   ret = bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_SET_LINE_CODEC, (unsigned long)(&(set_line_codec)), true);
   if (ret) {
      ast_log(AST_LOG_ERROR, "Unable to set line codec %d, for line %lu -> %d\n",
         (int)(set_line_codec.codec), (unsigned long)(pvt->index_line + 1), (int)(ret));
   }
   else {
      frame_size_read -= (frame_size_read % bytes_per_sample);
      ast_format_copy(&(pvt->ast_channel.current_format), format);
      pvt->ast_channel.bytes_per_sample = bytes_per_sample;
      pvt->ast_channel.frame_size_read = frame_size_read;
      bcm_assert(pvt->ast_channel.bytes_per_sample <= ARRAY_SIZE(pvt->ast_channel.bytes_not_written));
      pvt->ast_channel.bytes_not_written_len = 0;
   }

   return (ret);
}

/* Must be called with pvt->owner locked */
static int bcmph_set_line_mode(bcmph_pvt_t *pvt,
   bcm_phone_line_mode_t mode, bcm_phone_line_tone_t tone, int wait)
{
   int ret;
   bcm_phone_set_line_mode_t set_line_mode;

   bcm_pr_debug("bcmph_set_line_mode(mode=%d, tone=%d, wait=%d)\n",
      (int)(mode), (int)(tone), (int)(wait));

   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   memset(&(set_line_mode), 0, sizeof(set_line_mode));
   set_line_mode.line = pvt->index_line;
   set_line_mode.mode = mode;
   set_line_mode.tone = bcm_phone_line_tone_code_index(tone);
   set_line_mode.wait = wait;
   ret = bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_SET_LINE_MODE, (unsigned long)(&(set_line_mode)), true);
   if (ret < 0) {
      ast_log(AST_LOG_ERROR, "Unable to set line mode %d, for line %lu -> %d\n",
         (int)(mode), (unsigned long)(pvt->index_line + 1), (int)(ret));
   }

   return (ret);
}

/* Must be called with pvt->owner locked */
static int bcmph_set_line_tone(bcmph_pvt_t *pvt,
   __u32 new_tone, int wait)
{
   int ret;
   bcm_phone_set_line_tone_t set_line_tone;

   bcm_pr_debug("bcmph_set_line_tone(new_tone=%lu, wait=%d)\n",
      (unsigned long)(new_tone), (int)(wait));

   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   memset(&(set_line_tone), 0, sizeof(set_line_tone));
   set_line_tone.line = pvt->index_line;
   set_line_tone.tone = new_tone;
   set_line_tone.wait = wait;
   ret = bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_SET_LINE_TONE, (unsigned long)(&(set_line_tone)), true);
   if (ret < 0) {
      ast_log(AST_LOG_ERROR, "Unable to set line tone %lu, for line %lu -> %d\n",
         (unsigned long)(new_tone), (unsigned long)(pvt->index_line + 1), (int)(ret));
   }

   return (ret);
}

/* Must be called with pvt->owner locked */
static inline void bcmph_reset_pvt_monitor_state(bcmph_pvt_t *pvt)
{
   bcm_pr_debug("bcmph_reset_pvt_monitor_state()\n");

   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   pvt->ast_channel.digits[0] = '\0';
   pvt->ast_channel.digits_len = 0;
   pvt->ast_channel.search_extension = false;
   pvt->ast_channel.dtmf_sent = NONE;
   if (NULL != pvt->ast_channel.dsp) {
      ast_dsp_digitreset(pvt->ast_channel.dsp);
   }
}

/* Must be called with pvt->owner locked */
static void bcmph_set_new_state(bcmph_pvt_t *pvt,
   bcmph_state_t new_state, bcmph_event_t cause)
{
   bcm_pr_debug("bcmph_set_new_state(new_state=%d, cause=%d)\n",
      (int)(new_state), (int)(cause));

   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   switch (new_state) {
      case BCMPH_ST_DISCONNECTED: {
         bcm_assert((BCMPH_STATUS_DISCONNECTED == pvt->ast_channel.current_status)
            && (NULL == pvt->owner));
         bcm_assert(BCMPH_EV_DISCONNECTED == cause);
         bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_NONE, 0);
         break;
      }
      case BCMPH_ST_ON_IDLE: {
         bcm_assert((BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)
            && (NULL == pvt->owner));
         bcm_assert((BCMPH_EV_AST_HANGUP == cause) || (BCMPH_EV_INTERNAL_ERROR == cause) || (BCMPH_EV_ON_HOOK == cause));
         /* We reset the mode to BCMPH_MODE_IDLE */
         bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_NONE, 0);
         bcmph_reset_pvt_monitor_state(pvt);
         pvt->ast_channel.bytes_not_written_len = 0;
         break;
      }
      case BCMPH_ST_ON_PRE_RINGING: {
         bcm_assert((BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)
            && (NULL != pvt->owner));
         bcm_assert((BCMPH_ST_ON_IDLE == pvt->ast_channel.state)
            && (BCMPH_EV_AST_REQUEST == cause));
         break;
      }
      case BCMPH_ST_ON_RINGING: {
         bcm_assert((BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)
            && (NULL != pvt->owner));
         bcm_assert((BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state)
            && (BCMPH_EV_AST_CALL == cause));
         break;
      }
      case BCMPH_ST_OFF_DIALING: {
         bcm_assert((BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status)
            && (NULL == pvt->owner) && (pvt->line_cfg->monitor_dialing));
         bcm_assert((BCMPH_ST_ON_IDLE == pvt->ast_channel.state)
            && (BCMPH_EV_OFF_HOOK == cause));
         bcmph_reset_pvt_monitor_state(pvt);
         /*
          We start searching bcmph_default_extension ('s') after
          pvt->line_cfg->dialing_timeout milliseconds
         */
         pvt->ast_channel.search_extension = true;
         pvt->ast_channel.tv_last_digit = ast_tvnow();
         break;
      }
      case BCMPH_ST_OFF_WAITING_ANSWER: {
         bcm_assert((BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status)
            && (NULL != pvt->owner));
         bcm_assert(((BCMPH_ST_ON_IDLE == pvt->ast_channel.state)
            && (BCMPH_EV_OFF_HOOK == cause) && (!pvt->line_cfg->monitor_dialing))
                    || ((BCMPH_ST_OFF_DIALING == pvt->ast_channel.state)
            && (BCMPH_EV_EXT_FOUND == cause)));
         /*
          We return to mode IDLE in case we were in mode
          BCMPH_MODE_OFF_TALKING (if we have to detect DTMF) and we
          also stop tone.
         */
         bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_NONE, 0);
         /*
          We forget all digits dialed
         */
         bcmph_reset_pvt_monitor_state(pvt);
         break;
      }
      case BCMPH_ST_OFF_TALKING: {
         bcm_assert((BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status)
            && (NULL != pvt->owner));
         bcm_assert(((BCMPH_ST_ON_RINGING == pvt->ast_channel.state)
            && (BCMPH_EV_OFF_HOOK == cause))
                    || ((BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)
            && (BCMPH_EV_AST_ANSWER == cause)));
         break;
      }
      case BCMPH_ST_OFF_NO_SERVICE: {
         bcm_assert((BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status)
            && (NULL == pvt->owner));
         bcm_assert(((BCMPH_ST_OFF_DIALING == pvt->ast_channel.state) && (BCMPH_EV_NO_EXT_CAN_BE_FOUND == cause))
            || (BCMPH_EV_AST_HANGUP == cause)
            || (BCMPH_EV_INTERNAL_ERROR == cause));
         if (BCMPH_EV_AST_HANGUP == cause) {
            bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_BUSY, 0);
         }
         else {
            bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_INVALID, 0);
         }
         bcmph_reset_pvt_monitor_state(pvt);
         pvt->ast_channel.bytes_not_written_len = 0;
         break;
      }
   }
   pvt->ast_channel.state = new_state;
}

/* Must be called with pvt->owner and monitor.lock locked */
static void bcmph_unlink_from_ast_channel(bcmph_pvt_t *pvt,
   bcmph_event_t cause, bool unlock_ast_channel)
{
   struct ast_channel *ast = pvt->owner;

   bcm_pr_debug("bcmph_unlink_from_ast_channel(cause=%d)\n", (int)(cause));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0));


   ast_format_clear(&(pvt->ast_channel.current_format));
   ast_channel_tech_pvt_set(ast, NULL);
   pvt->owner = NULL;
   ast_setstate(ast, AST_STATE_DOWN);
   /* Set state before unlocking the channel */
   if (BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status) {
      bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, cause);
   }
   else if (BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status) {
      bcmph_set_new_state(pvt, BCMPH_ST_ON_IDLE, cause);
   }
   else {
      bcm_assert(BCMPH_STATUS_DISCONNECTED == pvt->ast_channel.current_status);
      bcmph_set_new_state(pvt, BCMPH_ST_DISCONNECTED, BCMPH_EV_DISCONNECTED);
   }
   bcm_pr_debug("Line %lu unlink from channel '%s'.\n",
      (unsigned long)(pvt->index_line + 1), ast_channel_name(ast));
   ast_module_unref(ast_module_info->self);
   if (unlock_ast_channel) {
#ifdef BCMPH_DEBUG
       bcm_assert(pvt->owner_lock_count > 0);
       pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
      ast_channel_unlock(ast);
   }
}

/* Must be called with monitor.lock locked and pvt->owner set to NULL */
static void bcmph_new(bcmph_pvt_t *pvt,
   bcmph_state_t state, bcmph_event_t cause,
   const char *linkedid, bool *try_to_lock_ast_channel)
{
   struct ast_channel *tmp;
   enum ast_channel_state ast_state;

   bcm_pr_debug("bcmph_new(state=%d, cause=%d)\n",
      (int)(state), (int)(cause));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL == pvt->owner));

   switch (state) {
      case BCMPH_ST_OFF_WAITING_ANSWER: {
         /*
          We don't set AST_STATE_UP right now, because it makes Asterisk
          believe that the call is already answered : in function
          ast_channel_alloc(), Asterisk call ast_cdr_init() and test if state
          is AST_STATE_UP to pass AST_CDR_ANSWERED or AST_CDR_NOANSWER
         */
         ast_state = AST_STATE_RING;
         bcm_assert((cause == BCMPH_EV_OFF_HOOK) || (cause == BCMPH_EV_EXT_FOUND));
         break;
      }
      case BCMPH_ST_ON_PRE_RINGING: {
         ast_state = AST_STATE_DOWN;
         bcm_assert(cause == BCMPH_EV_AST_REQUEST);
         break;
      }
      default: {
         bcm_assert(false);
         ast_state = AST_STATE_DOWN;
         break;
      }
   }
   bcm_assert(pvt->ast_channel.digits_len < ARRAY_LEN(pvt->ast_channel.digits));
   if (pvt->ast_channel.digits_len <= 0) {
      bcm_assert(ARRAY_LEN(bcmph_default_extension) <= ARRAY_LEN(pvt->ast_channel.digits));
      strcpy(pvt->ast_channel.digits, bcmph_default_extension);
   }
   else {
      pvt->ast_channel.digits[pvt->ast_channel.digits_len] = '\0';
   }
   tmp = ast_channel_alloc(1, ast_state,
      ('\0' != pvt->line_cfg->cid_num[0]) ? pvt->line_cfg->cid_num : NULL,
      ('\0' != pvt->line_cfg->cid_name[0]) ? pvt->line_cfg->cid_name : NULL,
      "" /* acctcode */, pvt->ast_channel.digits, pvt->line_cfg->context, linkedid, 0,
      "%s/%lu", pvt->channel->chan_tech.type, (unsigned long)(pvt->index_line + 1));
   if (NULL != tmp) {
      struct ast_format tmpfmt;
      bool hangup = false;

      /* From version 12.0, ast_channel_alloc() returned an ast_channel locked */
      if (bcmph_cmp_with_ast_version("120000") > 0) {
         /*
          We can safely lock channel (without fear of a deadlock because
          monitor.lock is locked), for 2 reasons :
          - channel_tech is not set
          - pvt->owner is still NULL
         */
         ast_channel_lock(tmp);
      }
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */

      /* Set channel tech */
      ast_channel_tech_set(tmp, &(pvt->channel->chan_tech));

      /* No file descriptor to poll (polling done in monitor thread) */
      ast_channel_set_fd(tmp, 0, -1);
      ast_format_cap_copy(ast_channel_nativeformats(tmp), pvt->line_cfg->capabilities);
      if ((DETECT_DTMF_ALWAYS == pvt->line_cfg->detect_dtmf)
          || (DETECT_DTMF_WHEN_CONNECTED == pvt->line_cfg->detect_dtmf)) {
         ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0);
         if (!ast_format_cap_iscompatible(pvt->line_cfg->capabilities, &(tmpfmt))) {
            ast_best_codec(ast_channel_nativeformats(tmp), &(tmpfmt));
         }
      }
      else {
         ast_best_codec(ast_channel_nativeformats(tmp), &(tmpfmt));
      }
      ast_format_copy(ast_channel_rawreadformat(tmp), &(tmpfmt));
      ast_format_copy(ast_channel_rawwriteformat(tmp), &(tmpfmt));
      /* no need to call ast_setstate: the channel_alloc already did its job */
      ast_channel_exten_set(tmp, pvt->ast_channel.digits);
      if (!ast_strlen_zero(pvt->channel->config.language)) {
         ast_channel_language_set(tmp, pvt->channel->config.language);
      }
      if (AST_STATE_RING == ast_state) {
         ast_channel_rings_set(tmp, 1);
      }

      ast_channel_tech_pvt_set(tmp, pvt);
      ast_module_ref(ast_module_info->self);
      pvt->owner = tmp;

      /*
       Set state before starting channel's thread
      */
      bcmph_set_new_state(pvt, state, cause);

      /* Unset pvt->owner before unlocking channel (see below) */
      pvt->owner = NULL;
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
      ast_channel_unlock(tmp);

      if ((!hangup) && (AST_STATE_DOWN != ast_state) && (ast_pbx_start(tmp))) {
         ast_log(AST_LOG_ERROR, "Unable to start PBX on '%s'\n", ast_channel_name(tmp));
         hangup = true;
      }
      /*
       Here we can still safely lock channel (without fear of a deadlock
       because monitor.lock is locked), as pvt->owner is still NULL
      */
      bcm_assert(NULL == pvt->owner);
      ast_channel_lock(tmp);
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      pvt->owner = tmp;
      if (hangup) {
         bcmph_unlink_from_ast_channel(pvt, BCMPH_EV_INTERNAL_ERROR, true);
         bcm_assert(NULL == pvt->owner);
         /* ast_channel_unlock(tmp) is done in bcmph_unlink_from_ast_channel() */
         /*
          Calling ast_hangup() means that we call indirectly
          bcmph_chan_hangup().
          As the association between the ast_channel and the tech_pvt
          is already removed bcmph_chan_hangup() will do nothing.
          Of course in bcmph_chan_hangup(), we must check that the
          ast_channel has a valid tech_pvt
         */
         ast_hangup(tmp);
         tmp = NULL;
      }
      else {
         if ((NULL == try_to_lock_ast_channel) || (!(*try_to_lock_ast_channel))) {
#ifdef BCMPH_DEBUG
            bcm_assert(pvt->owner_lock_count > 0);
            pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
            ast_channel_unlock(pvt->owner);
         }
      }
   }
   else {
      ast_log(AST_LOG_ERROR, "Unable to allocate channel structure\n");
      if (AST_STATE_DOWN != ast_state) {
         bcm_assert(BCMPH_ST_OFF_WAITING_ANSWER == state);
         /* We signals the user that there's a problem */
         bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_INVALID, 0);
         bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, BCMPH_EV_INTERNAL_ERROR);
      }
      else {
         bcm_assert(BCMPH_ST_ON_PRE_RINGING == state);
         bcmph_set_new_state(pvt, BCMPH_ST_ON_IDLE, BCMPH_EV_INTERNAL_ERROR);
      }
   }
}

/*
 Must be call with pvt->owner locked.
 Beware that it can take several milliseconds to execute
*/
static int bcmph_setup(bcmph_pvt_t *pvt,
   const struct ast_format *format, enum ast_channel_state ast_state)
{
   int ret = 0;

   bcm_pr_debug("bcmph_setup(ast_state=%d)\n", (int)(ast_state));

   bcm_assert((NULL != pvt->owner) && (pvt->owner_lock_count > 0));

   do { /* Empty loop */
      if (NULL == format) {
         ast_log(AST_LOG_WARNING, "No format specified\n");
         ret = -1;
         break;
      }
      if (!ast_format_cap_iscompatible(pvt->line_cfg->capabilities, format)) {
         ast_log(AST_LOG_WARNING, "Can't do format '%s'\n", ast_getformatname(format));
         ret = -1;
         break;
      }
      if (bcmph_set_line_codec(pvt, format, BCMPH_MODE_OFF_TALKING, BCMPH_TONE_NONE)) {
         ret = -1;
         break;
      }
      if (NULL != pvt->ast_channel.dsp) {
         ast_dsp_digitreset(pvt->ast_channel.dsp);
      }
      ast_setstate(pvt->owner, ast_state);
   } while (false);

   return (ret);
}

/*
 Must be called with pvt->owner and mmap.lock locked.
 Return the frame that has been read or NULL
*/
static struct ast_frame *bcmph_read_frame(bcmph_pvt_t *pvt, bool detect_dtmf, bool *unlock_mmap)
{
   struct ast_frame *ret = NULL;

   /* bcm_pr_debug("bcmph_read_frame()\n"); */

   bcm_assert((NULL != unlock_mmap) && (*unlock_mmap)
      && (pvt->channel->mmap.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0)));

   do { /* Empty loop */
      size_t len;

      /* Check that there are data in the ring buffer */
      len = bcm_ring_buf_get_size(&(pvt->ast_channel.mmap.dev_rx_ring_buf));
      if (len >= pvt->ast_channel.frame_size_read) {
         len = pvt->ast_channel.frame_size_read;
         pvt->ast_channel.frame.data.ptr = &(pvt->ast_channel.buf_fr[AST_FRIENDLY_OFFSET]);
         pvt->ast_channel.frame.datalen = len;
         pvt->ast_channel.frame.samples = len / pvt->ast_channel.bytes_per_sample;
         pvt->ast_channel.frame.frametype = AST_FRAME_VOICE;
         ast_format_copy(&(pvt->ast_channel.frame.subclass.format), &(pvt->ast_channel.current_format));
         pvt->ast_channel.frame.src = bcmph_chan_type;
         pvt->ast_channel.frame.offset = AST_FRIENDLY_OFFSET;
         pvt->ast_channel.frame.mallocd = 0;
         pvt->ast_channel.frame.delivery = ast_tv(0,0);
         bcm_ring_buf_remove(&(pvt->ast_channel.mmap.dev_rx_ring_buf), pvt->ast_channel.frame.data.ptr, len);
         if (bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_READ_MM, (unsigned long)((len << 8) | pvt->index_line), false)) {
            ast_log(AST_LOG_ERROR, "Can't remove data in RX buffer of line %lu\n", (unsigned long)(pvt->index_line + 1));
            break;
         }
         bcmph_update_ring_bufs_desc(pvt->channel);
         bcmph_mmap_unlock(pvt->channel);
         *unlock_mmap = false;
         ret = &(pvt->ast_channel.frame);
         if (detect_dtmf) {
            ret = ast_dsp_process(pvt->owner, pvt->ast_channel.dsp, ret);
         }
      }
   } while (false);

   return (ret);
}

/*
 Must be called with pvt->owner and mmap.lock locked.
 Return true if at least one frame has been queued
*/
static bool bcmph_queue_read(bcmph_pvt_t *pvt, bool *unlock_mmap)
{
   bool ret = false;

   /* bcm_pr_debug("bcmph_queue_read()\n"); */

   bcm_assert((NULL != unlock_mmap) && (*unlock_mmap)
      && (pvt->channel->mmap.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0)
      && (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state));

   do { /* Empty loop */
      size_t len;
      bool detect_dtmf = ((DETECT_DTMF_ALWAYS == pvt->line_cfg->detect_dtmf)
         || (DETECT_DTMF_WHEN_CONNECTED == pvt->line_cfg->detect_dtmf));
      bool queue_frames = (AST_STATE_UP == ast_channel_state(pvt->owner));

      /* Check that there are data in the ring buffer */
      len = bcm_ring_buf_get_size(&(pvt->ast_channel.mmap.dev_rx_ring_buf));
      while (len >= pvt->ast_channel.frame_size_read) {
         len = pvt->ast_channel.frame_size_read;
         if (!queue_frames) {
            bcm_ring_buf_remove_len(&(pvt->ast_channel.mmap.dev_rx_ring_buf), len);
            if (bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_READ_MM, (unsigned long)((len << 8) | pvt->index_line), false)) {
               ast_log(AST_LOG_ERROR, "Can't remove data in RX buffer of line %lu\n", (unsigned long)(pvt->index_line + 1));
               break;
            }
            bcmph_update_ring_bufs_desc(pvt->channel);
         }
         else {
            pvt->ast_channel.frame_to_queue.datalen = len;
            pvt->ast_channel.frame_to_queue.samples = len / pvt->ast_channel.bytes_per_sample;
            pvt->ast_channel.frame_to_queue.frametype = AST_FRAME_VOICE;
            ast_format_copy(&(pvt->ast_channel.frame_to_queue.subclass.format), &(pvt->ast_channel.current_format));
            pvt->ast_channel.frame_to_queue.src = bcmph_chan_type;
            pvt->ast_channel.frame_to_queue.offset = 0;
            pvt->ast_channel.frame_to_queue.mallocd = 0;
            pvt->ast_channel.frame_to_queue.delivery = ast_tv(0,0);
            if (!detect_dtmf) {
               size_t len_direct;
               /*
                Here it's valid to set data.ptr to point directly in the ring
                buffer, because ast_queue_frame() copy the data pointed to by data.ptr
                and we call BCMPH_IOCTL_READ_MM after ast_queue_frame()
               */
               pvt->ast_channel.frame_to_queue.data.ptr = bcm_ring_buf_get_ptr_peek(&(pvt->ast_channel.mmap.dev_rx_ring_buf), &(len_direct));
               if (len_direct < len) {
                  pvt->ast_channel.frame_to_queue.data.ptr = pvt->ast_channel.buf_fr_to_queue;
                  bcm_ring_buf_peek(&(pvt->ast_channel.mmap.dev_rx_ring_buf), pvt->ast_channel.frame_to_queue.data.ptr, len);
               }
               if (ast_queue_frame(pvt->owner, &(pvt->ast_channel.frame_to_queue))) {
                  ast_frfree(&(pvt->ast_channel.frame_to_queue));
                  ast_log(AST_LOG_WARNING, "Can't queue voice frame for line %lu\n", (unsigned long)(pvt->index_line + 1));
                  break;
               }
               ast_frfree(&(pvt->ast_channel.frame_to_queue));
               bcm_ring_buf_remove_len(&(pvt->ast_channel.mmap.dev_rx_ring_buf), len);
               if (bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_READ_MM, (unsigned long)((len << 8) | pvt->index_line), false)) {
                  ast_log(AST_LOG_ERROR, "Can't remove data in RX buffer of line %lu\n", (unsigned long)(pvt->index_line + 1));
                  break;
               }
               bcmph_update_ring_bufs_desc(pvt->channel);
            }
            else {
               struct ast_frame *fr;

               pvt->ast_channel.frame_to_queue.data.ptr = pvt->ast_channel.buf_fr_to_queue;
               bcm_ring_buf_remove(&(pvt->ast_channel.mmap.dev_rx_ring_buf), pvt->ast_channel.frame_to_queue.data.ptr, len);
               if (bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_READ_MM, (unsigned long)((len << 8) | pvt->index_line), false)) {
                  ast_frfree(&(pvt->ast_channel.frame_to_queue));
                  ast_log(AST_LOG_ERROR, "Can't remove data in RX buffer of line %lu\n", (unsigned long)(pvt->index_line + 1));
                  break;
               }
               bcmph_update_ring_bufs_desc(pvt->channel);
               bcmph_mmap_unlock(pvt->channel);
               *unlock_mmap = false;
               fr = ast_dsp_process(pvt->owner, pvt->ast_channel.dsp, &(pvt->ast_channel.frame_to_queue));
               if (NULL != fr) {
                  if (ast_queue_frame(pvt->owner, fr)) {
                     ast_frfree(fr);
                     ast_log(AST_LOG_WARNING, "Can't queue frame for line %lu\n", (unsigned long)(pvt->index_line + 1));
                     break;
                  }
                  ast_frfree(fr);
               }
               else {
                  ast_frfree(&(pvt->ast_channel.frame_to_queue));
               }
               bcmph_mmap_lock(pvt->channel);
               *unlock_mmap = true;
            }
         }
         ret = true;
         len = bcm_ring_buf_get_size(&(pvt->ast_channel.mmap.dev_rx_ring_buf));
      }
   } while (false);

   return (ret);
}

/*
 Must be called with pvt->owner and monitor.lock locked.
 When exiting pvt->owner has been unlocked and pvt->owner is set to NULL
*/
static void bcmph_queue_hangup(bcmph_pvt_t *pvt, bcmph_event_t cause)
{
   struct ast_channel *ast = pvt->owner;

   bcm_pr_debug("bcmph_queue_hangup(cause=%d)\n",
      (int)(cause));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0));

   bcmph_unlink_from_ast_channel(pvt, cause, true);
   bcm_assert(NULL == pvt->owner);
   if (ast_queue_hangup(ast)) {
      ast_log(AST_LOG_WARNING, "Unable to queue hangup on line '%s'\n", ast_channel_name(ast));
   }
}

static inline void bcmph_change_monitor_timeout(
   bcmph_monitor_prms_t *monitor_prms, int timeout)
{
   /* bcm_pr_debug("bcmph_change_monitor_timeout(timeout=%d)\n",
      (int)(timeout)); */

   bcm_assert((timeout > 0) && (NULL != monitor_prms));
   if (timeout < monitor_prms->timeout) {
      monitor_prms->timeout = timeout;
   }
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_status_change(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bcm_phone_line_status_t new_status)
{
   bcm_pr_debug("bcmph_handle_status_change(new_status=%d)\n",
      (int)(new_status));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   pvt->ast_channel.current_status = new_status;

   /* Handle status change */
   if (BCMPH_STATUS_OFF_HOOK == new_status) {
      if ((BCMPH_ST_ON_RINGING == pvt->ast_channel.state)
          || (BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state)) {
         bcm_assert(NULL != pvt->owner);
         /*
          bcmph_chan_request() has created the channel.
          Either bcmph_chan_call() has been called so the phone
          is ringing and the user picked up the phone to answer
          the call or bcmph_chan_call() has not been called and
          the user picked up the phone to make a call.
          In this latter case there's a conflict, so we hang
          up the channel created by bcmph_chan_request()
          and emit the tone busy to the user.
         */
         bool hangup = true;
         if (BCMPH_ST_ON_RINGING == pvt->ast_channel.state) {
            bcm_assert(AST_STATE_RINGING == ast_channel_state(pvt->owner));
            /* The user wants to answer the call */
            if (bcmph_setup(pvt, ast_channel_rawreadformat(pvt->owner), AST_STATE_UP)) {
               ast_log(AST_LOG_ERROR, "Unable to answer the call on '%s'\n", ast_channel_name(pvt->owner));
            }
            else {
               if (ast_queue_control(pvt->owner, AST_CONTROL_ANSWER)) {
                  ast_log(AST_LOG_ERROR, "Unable to answer the call on '%s'\n", ast_channel_name(pvt->owner));
               }
               else {
                  bcm_pr_debug("Call answered on '%s'\n", ast_channel_name(pvt->owner));
                  hangup = false;
                  bcmph_set_new_state(pvt, BCMPH_ST_OFF_TALKING, BCMPH_EV_OFF_HOOK);
                  bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.monitor_busy_period);
               }
            }
         }
         else {
            bcm_assert(BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state);
            /*
             The user wants to make a call but that's not possible as
             there's an incoming call pending
            */
         }
         if (hangup) {
            bcmph_queue_hangup(pvt, BCMPH_EV_INTERNAL_ERROR);
            /* ast_channel_unlock(pvt->owner) is done in bcmph_queue_hangup() */
            bcm_assert((NULL == pvt->owner) && (BCMPH_ST_OFF_NO_SERVICE == pvt->ast_channel.state));
         }
      }
      else {
         bcm_assert(
            ((BCMPH_ST_ON_IDLE == pvt->ast_channel.state)
             || (BCMPH_ST_DISCONNECTED == pvt->ast_channel.state))
            && (NULL == pvt->owner));
         /* The user has picked up the phone to make a call */
         if (pvt->line_cfg->monitor_dialing) {
            bcmph_state_t new_state = BCMPH_ST_OFF_DIALING;
            bcmph_event_t cause = BCMPH_EV_OFF_HOOK;
            /*
             We do not create an ast_channel now
             We just change the tone to BCMPH_TONE_WAITING_DIAL, and
             wait for digits dialed by the user.
             When the digits will form a valid extension then we will
             create the ast_channel dedicated to handle the call
            */
            if ((DETECT_DTMF_WHEN_DIALING != pvt->line_cfg->detect_dtmf)
                && (DETECT_DTMF_ALWAYS != pvt->line_cfg->detect_dtmf)) {
               bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_WAITING_DIAL, 0);
            }
            else {
               /*
                We set the mode to BCMPH_MODE_OFF_TALKING to receive
                audio data in which we try to detect DTMF tones
               */
               struct ast_format tmpfmt;

               /*
                AST_FORMAT_SLINEAR is the recommended codec for frames
                processed by dsp
               */
               ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0);
               if (!ast_format_cap_iscompatible(pvt->line_cfg->capabilities, &(tmpfmt))) {
                  /*
                   AST_FORMAT_SLINEAR is not supported by the line,
                   so we fall back to ALAW or ULAW
                  */
                  ast_format_set(&(tmpfmt), AST_FORMAT_ALAW, 0);
                  if (!ast_format_cap_iscompatible(pvt->line_cfg->capabilities, &(tmpfmt))) {
                     ast_format_set(&(tmpfmt), AST_FORMAT_ULAW, 0);
                     bcm_assert(ast_format_cap_iscompatible(pvt->line_cfg->capabilities, &(tmpfmt)));
                  }
               }
               if (bcmph_set_line_codec(pvt, &(tmpfmt), BCMPH_MODE_OFF_TALKING, BCMPH_TONE_WAITING_DIAL)) {
                  new_state = BCMPH_ST_OFF_NO_SERVICE;
                  cause = BCMPH_EV_INTERNAL_ERROR;
               }
               else {
                  bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.monitor_busy_period);
               }
            }
            bcmph_set_new_state(pvt, new_state, cause);
         }
         else { /* (!pvt->line_cfg->monitor_dialing) */
            /*
             We create an ast_channel now.
            */
            pvt->ast_channel.digits_len = 0;
            bcmph_new(pvt, BCMPH_ST_OFF_WAITING_ANSWER, BCMPH_EV_OFF_HOOK,
               NULL, &(monitor_prms->channel_is_locked));
         }
      }
   }
   else { /* (BCMPH_STATUS_OFF_HOOK != new_status) */
      bcmph_state_t new_state;
      bcmph_event_t cause;
      if (BCMPH_STATUS_ON_HOOK == new_status) {
         /*
          At least one off hook to on hook transition
         */
         new_state = BCMPH_ST_ON_IDLE;
         cause = BCMPH_EV_ON_HOOK;
      }
      else {
         bcm_assert(BCMPH_STATUS_DISCONNECTED == new_status);
         ast_log(AST_LOG_WARNING, "Line %lu has been disconnected\n", (unsigned long)(pvt->index_line + 1));
         new_state = BCMPH_ST_DISCONNECTED;
         cause = BCMPH_EV_DISCONNECTED;
      }
      /* Gone on hook or disconnected, so notify hangup */
      if (NULL != pvt->owner) {
         bcmph_queue_hangup(pvt, cause);
         /* ast_channel_unlock(pvt->owner) is done in bcmph_queue_hangup() */
         bcm_assert(NULL == pvt->owner);
      }
      else {
         bcmph_set_new_state(pvt, new_state, cause);
      }
   }
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_mode_change(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bcm_phone_line_mode_t new_mode)
{
   bcm_pr_debug("bcmph_handle_mode_change(new_mode=%d)\n",
      (int)(new_mode));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   pvt->ast_channel.current_mode = new_mode;
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_tone_change(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bcm_phone_line_tone_t new_tone)
{
   bcm_pr_debug("bcmph_handle_tone_change(new_tone=%d)\n",
      (int)(new_tone));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   pvt->ast_channel.current_tone = new_tone;
}

/*
 Must be called with monitor.lock locked and pvt->owner set to NULL
*/
static void bcmph_search_extension(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   /* bcm_pr_debug("bcmph_search_extension()\n"); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL == pvt->owner) && (pvt->line_cfg->monitor_dialing)
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && (BCMPH_ST_OFF_DIALING == pvt->ast_channel.state));

   do { /* Empty loop */
      if (!pvt->ast_channel.search_extension) {
         break;
      }
      if (pvt->line_cfg->dialing_timeout > 0) {
         struct timeval now = ast_tvnow();
         int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_last_digit);
         if (tvdiff < pvt->line_cfg->dialing_timeout) {
            bcmph_change_monitor_timeout(monitor_prms, (int)(pvt->line_cfg->dialing_timeout - tvdiff));
            break;
         }
      }
      pvt->ast_channel.search_extension = false;
      /* We test if, with these numbers, the extension is valid */
      bcm_assert(pvt->ast_channel.digits_len < ARRAY_LEN(pvt->ast_channel.digits));
      if (pvt->ast_channel.digits_len <= 0) {
         bcm_assert(ARRAY_LEN(bcmph_default_extension) <= ARRAY_LEN(pvt->ast_channel.digits));
         strcpy(pvt->ast_channel.digits, bcmph_default_extension);
      }
      else {
         pvt->ast_channel.digits[pvt->ast_channel.digits_len] = '\0';
      }
      bcm_pr_debug("Searching for extension '%s' in context '%s'\n",
         pvt->ast_channel.digits, pvt->line_cfg->context);
      /*
       Reset flag search_extension to false to stop
       searching for an extension while no new digit is
       dialed
      */
      if (ast_exists_extension(NULL, pvt->line_cfg->context, pvt->ast_channel.digits, 1, pvt->line_cfg->cid_num)) {
         /* It's a valid extension in its context, get moving! */
         bcm_pr_debug("Extension '%s' found in context '%s'\n", pvt->ast_channel.digits, pvt->line_cfg->context);
         bcmph_new(pvt, BCMPH_ST_OFF_WAITING_ANSWER, BCMPH_EV_EXT_FOUND,
            NULL, &(monitor_prms->channel_is_locked));
      }
      else {
         if (!ast_canmatch_extension(NULL, pvt->line_cfg->context, pvt->ast_channel.digits, 1, pvt->line_cfg->cid_num)) {
            /* It's not a valid extension */
            bcm_pr_debug("Extension '%s' can't match anything in '%s'\n",
               pvt->ast_channel.digits, pvt->line_cfg->context);
            /* We signals the user that there's a problem */
            bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_INVALID, 0);
            bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, BCMPH_EV_NO_EXT_CAN_BE_FOUND);
         }
         else if (pvt->ast_channel.digits_len >= (ARRAY_LEN(pvt->ast_channel.digits) - 1)) {
            /*
             We overrun maximum length of extension without
             matching anything
            */
            bcm_pr_debug("Extension '%s' can't match anything in '%s' and reaches maximum length\n",
               pvt->ast_channel.digits, pvt->line_cfg->context);
            /* We signals the user that there's a problem */
            bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, BCMPH_TONE_INVALID, 0);
            bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, BCMPH_EV_NO_EXT_CAN_BE_FOUND);
         }
      }
   } while (false);
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_try_to_send_dtmf(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   bool send_a_null_frame = false;

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0)
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state));

   if (NONE != pvt->ast_channel.dtmf_sent) {
      struct timeval now = ast_tvnow();
      int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_last_digit);
      if (ON == pvt->ast_channel.dtmf_sent) {
         if (tvdiff < MIN_DTMF_DURATION) {
            bcmph_change_monitor_timeout(monitor_prms, (int)(MIN_DTMF_DURATION - tvdiff));
         }
         else {
            pvt->ast_channel.dtmf_sent = OFF;
            pvt->ast_channel.tv_last_digit = ast_tvnow();
            bcmph_change_monitor_timeout(monitor_prms, MIN_TIME_BETWEEN_DTMF);
            send_a_null_frame = true;
         }
      }
      else {
         bcm_assert(OFF == pvt->ast_channel.dtmf_sent);
         if (tvdiff < MIN_TIME_BETWEEN_DTMF) {
            bcmph_change_monitor_timeout(monitor_prms, (int)(MIN_TIME_BETWEEN_DTMF - tvdiff));
         }
         else {
            pvt->ast_channel.dtmf_sent = NONE;
            send_a_null_frame = true;
         }
      }
   }
   if (send_a_null_frame) {
      /*
       As we only send AST_FRAME_DTMF, we send a null frame
       for Asterisk to properly generate AST_FRAME_DTMF_BEGIN
       and AST_FRAME_DTMF_END (it does it only when processing
       a frame)
      */
      if (AST_STATE_UP == ast_channel_state(pvt->owner)) {
         if (ast_queue_frame(pvt->owner, &(ast_null_frame))) {
            ast_log(AST_LOG_WARNING, "Can't queue null frame for line %lu\n", (unsigned long)(pvt->index_line + 1));
         }
      }
   }
   if ((NONE == pvt->ast_channel.dtmf_sent) && (pvt->ast_channel.digits_len > 0)) {
      int ret;
      pvt->ast_channel.frame_to_queue.datalen = 0;
      pvt->ast_channel.frame_to_queue.samples = 0;
      pvt->ast_channel.frame_to_queue.data.ptr =  NULL;
      pvt->ast_channel.frame_to_queue.src = bcmph_chan_type;
      pvt->ast_channel.frame_to_queue.offset = 0;
      pvt->ast_channel.frame_to_queue.mallocd = 0;
      pvt->ast_channel.frame_to_queue.delivery = ast_tv(0,0);
      pvt->ast_channel.frame_to_queue.frametype = AST_FRAME_DTMF;
      pvt->ast_channel.frame_to_queue.subclass.integer = pvt->ast_channel.digits[0];
      ret = ast_queue_frame(pvt->owner, &(pvt->ast_channel.frame_to_queue));
      ast_frfree(&(pvt->ast_channel.frame_to_queue));
      if (ret) {
         ast_log(AST_LOG_WARNING, "Unable to queue a digit on line %lu\n", (unsigned long)(pvt->index_line + 1));
      }
      else {
         bcm_pr_debug("DTMF '%c' sent\n", (int)(pvt->ast_channel.frame_to_queue.subclass.integer));
         pvt->ast_channel.dtmf_sent = ON;
         pvt->ast_channel.tv_last_digit = ast_tvnow();
         bcmph_change_monitor_timeout(monitor_prms, MIN_DTMF_DURATION);
         pvt->ast_channel.digits_len -= 1;
         if (pvt->ast_channel.digits_len > 0) {
            memmove(pvt->ast_channel.digits, &(pvt->ast_channel.digits[1]), pvt->ast_channel.digits_len);
         }
      }
   }
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_digits(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, const char *digits, size_t digits_len)
{
   bcm_pr_debug("bcmph_handle_digits(digits_len=%lu)\n",
      (unsigned long)(digits_len));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && (NULL != digits) && (digits_len > 0));

   if (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state) {
      bcm_assert(NULL != pvt->owner);
      if ((pvt->ast_channel.digits_len + digits_len) > ARRAY_LEN(pvt->ast_channel.digits)) {
         digits_len = ARRAY_LEN(pvt->ast_channel.digits) - pvt->ast_channel.digits_len;
      }
      if (digits_len > 0) {
         memcpy(&(pvt->ast_channel.digits[pvt->ast_channel.digits_len]), digits, digits_len);
         pvt->ast_channel.digits_len += digits_len;
      }
   }
   else if (BCMPH_ST_OFF_DIALING == pvt->ast_channel.state) {
      bcm_assert((NULL == pvt->owner) && (pvt->line_cfg->monitor_dialing));
      do { /* Empty loop */
         size_t i = 0;
         /* We search for an extension before adding new digit */
         bcmph_search_extension(pvt, monitor_prms);
         if ((BCMPH_ST_OFF_DIALING != pvt->ast_channel.state) || (!monitor_prms->channel_is_locked)) {
            break;
         }
         while ((i < digits_len) && (pvt->ast_channel.digits_len < (ARRAY_LEN(pvt->ast_channel.digits) - 1))) {
            pvt->ast_channel.digits[pvt->ast_channel.digits_len] = digits[i];
            i += 1;
            if (0 == pvt->ast_channel.digits_len) {
               /* We switch off the BCMPH_TONE_WAITING_DIAL tone */
               bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_NONE), 0);
            }
            pvt->ast_channel.digits_len += 1;
            if (pvt->line_cfg->dialing_timeout <= 0) {
               pvt->ast_channel.search_extension = true;
               bcmph_search_extension(pvt, monitor_prms);
               bcm_assert(!pvt->ast_channel.search_extension);
               if ((BCMPH_ST_OFF_DIALING != pvt->ast_channel.state) || (!monitor_prms->channel_is_locked)) {
                  break;
               }
            }
         }
         if ((BCMPH_ST_OFF_DIALING != pvt->ast_channel.state) || (!monitor_prms->channel_is_locked)) {
            break;
         }
         if (i > 0) {
            /* New digits appended to array pvt->ast_channel.digits */
            if (pvt->line_cfg->dialing_timeout > 0) {
               /*
                We set the flag saying we can search for an extension
                and we update the time last digit was dialed
               */
               pvt->ast_channel.search_extension = true;
               pvt->ast_channel.tv_last_digit = ast_tvnow();
               bcmph_change_monitor_timeout(monitor_prms, pvt->line_cfg->dialing_timeout);
            }
         }
      } while (false);
   }
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_flashes(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, size_t flash_count)
{
#ifdef BCMPH_DEBUG
   size_t i;
   char digit;
#endif /* BCMPH_DEBUG */

   bcm_pr_debug("bcmph_handle_flashes(flash_count=%lu)\n",
      (unsigned long)(flash_count));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

#ifdef BCMPH_DEBUG
   digit = '#';
   for (i = 0; (i < flash_count); i += 1) {
      bcmph_handle_digits(pvt, monitor_prms, &(digit), 1);
      if (!monitor_prms->channel_is_locked) {
         break;
      }
   }
#else /* BCMPH_DEBUG */
   if (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state) {
      while (flash_count > 0) {
         if (ast_queue_control(pvt->owner, AST_CONTROL_FLASH)) {
            ast_log(AST_LOG_WARNING, "Unable to queue flash event on line %lu\n", (unsigned long)(pvt->index_line + 1));
            break;
         }
         flash_count -= 1;
      }
   }
#endif /* !BCMPH_DEBUG */
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_monitor_pvt(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   /* bcm_pr_debug("bcmph_monitor_pvt()\n"); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   if (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state) {
      bcm_assert(NULL != pvt->owner);
      bcmph_try_to_send_dtmf(pvt, monitor_prms);
      if ((monitor_prms->channel_is_locked)
          && (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)) {
         /* We read as much data as possible coming from the driver
          and queue ast_frame */
         if (!bcmph_mmap_trylock(pvt->channel)) {
            bool unlock_mmap = true;
            bcmph_queue_read(pvt, &(unlock_mmap));
            if (unlock_mmap) {
               bcmph_mmap_unlock(pvt->channel);
            }
            bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.monitor_busy_period);
         }
         else {
            bcmph_change_monitor_timeout(monitor_prms, bcmph_monitor_short_timeout);
         }
      }
   }
   else if (BCMPH_ST_OFF_DIALING == pvt->ast_channel.state) {
      bool new_digit_dialed = false;
      bcm_assert((NULL == pvt->owner) && (pvt->line_cfg->monitor_dialing));
      if ((DETECT_DTMF_WHEN_DIALING == pvt->line_cfg->detect_dtmf)
          || (DETECT_DTMF_ALWAYS == pvt->line_cfg->detect_dtmf)) {
         bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.monitor_busy_period);
         for (;;) {
            bool unlock_mmap;
            struct ast_frame *fr;
            /*
             We try to read a frame and pass it to dsp.
             If we detect a digit, we add it to extension
             and search for a valid extension
            */
            if (bcmph_mmap_trylock(pvt->channel)) {
               /*
                Can't lock the mmap to read a frame, we retry later
               */
               bcmph_change_monitor_timeout(monitor_prms, bcmph_monitor_short_timeout);
               break;
            }
            unlock_mmap = true;
            fr = bcmph_read_frame(pvt, true, &(unlock_mmap));
            if (unlock_mmap) {
               bcmph_mmap_unlock(pvt->channel);
               unlock_mmap = false;
            }
            if (NULL == fr) {
               break;
            }
            if (fr->frametype == AST_FRAME_DTMF) {
               char digit = fr->subclass.integer;
               bcmph_handle_digits(pvt, monitor_prms, &(digit), 1);
               new_digit_dialed = true;
               if ((BCMPH_ST_OFF_DIALING != pvt->ast_channel.state)
                   || (!monitor_prms->channel_is_locked)) {
                  ast_frfree(fr);
                  break;
               }
            }
            ast_frfree(fr);
         }
      }
      if ((monitor_prms->channel_is_locked)
          && (BCMPH_ST_OFF_DIALING == pvt->ast_channel.state)
          && (!new_digit_dialed)) {
         bcmph_search_extension(pvt, monitor_prms);
      }
   }
}

static inline void bcmph_monitor_lock(bcmph_chan_t *t)
{
   ast_mutex_lock(&(t->monitor.lock));
#ifdef BCMPH_DEBUG
   bcm_assert(0 == t->monitor.lock_count);
   t->monitor.lock_count += 1;
#endif /* BCMPH_DEBUG */
}

static inline void bcmph_monitor_unlock(bcmph_chan_t *t)
{
#ifdef BCMPH_DEBUG
   t->monitor.lock_count -= 1;
   bcm_assert(0 == t->monitor.lock_count);
#endif /* BCMPH_DEBUG */
   ast_mutex_unlock(&(t->monitor.lock));
}

static inline void bcmph_prepare_start_monitor(bcmph_chan_t *t)
{
   t->monitor.run = true;
}

static inline void bcmph_prepare_stop_monitor(bcmph_chan_t *t)
{
   t->monitor.run = false;
}

static void *bcmph_do_monitor(void *data)
{
   bcmph_chan_t *t = (bcmph_chan_t *)(data);
   bcmph_monitor_prms_t monitor_prms;
#ifdef BCMPH_DEBUG
   /* static int last_timeout; */
#endif /* BCMPH_DEBUG */

   monitor_prms.timeout = bcmph_monitor_short_timeout;
#ifdef BCMPH_DEBUG
   /* last_timeout = monitor_prms.timeout; */
#endif /* BCMPH_DEBUG */

   bcm_pr_debug("Monitor thread starting\n");

   while (t->monitor.run) {
      bcm_phone_get_line_states_t get_line_states;
      bcmph_pvt_t *pvt;
      bool line_states_are_valid = false;
      int res;

#ifdef BCMPH_DEBUG
      /*
      if (monitor_prms.timeout != last_timeout) {
         bcm_pr_debug("Monitor timeout changed from %d to %d\n",
            (int)(last_timeout), (int)(monitor_prms.timeout));
         last_timeout = monitor_prms.timeout;
      }
      */
#endif /* BCMPH_DEBUG */
      /*
       Asks the driver line states. Blocks until a line state changes
       or timeout expires
      */
      memset(&(get_line_states), 0, sizeof(get_line_states));
      /*
       Timeout is not the same if at least one line is transmitting
       audio
      */
      get_line_states.wait = monitor_prms.timeout;
      /*
       Do not use BCMPH_IOCTL_GET_LINE_STATES_MM
       because it could block the access to mmap by channel threads
       when writing frames
      */
      res = bcmph_do_ioctl(t, BCMPH_IOCTL_GET_LINE_STATES, (unsigned long)(&(get_line_states)), false);
      if (res) {
         line_states_are_valid = false;
         if ((EINTR != errno) && (EAGAIN != errno)) {
            ast_log(AST_LOG_ERROR, "Unable to get line states -> %d. errno = %d\n", (int)(res), (int)(errno));
            break;
         }
      }
      else {
         line_states_are_valid = true;
      }

      monitor_prms.timeout = bcmph_monitor_idle_timeout;

      bcmph_monitor_lock(t);

      AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
         bcm_phone_line_state_t *line_state = &(pvt->monitor.line_state);

         monitor_prms.channel_is_locked = false;

         if (line_states_are_valid) {
            /* We accumulate the line_state in case we can't lock the channel */
            bcm_phone_line_state_move(&(get_line_states.line_state[pvt->index_line]), line_state);
            if (BCMPH_STATUS_OFF_HOOK != line_state->status) {
               /*
                Discards digits and flash events that do not
                occured off hook, so that when we get off hook we handle
                only digits and flash events that really occured off hook
               */
               bcm_phone_line_state_reset_digits(line_state);
               bcm_phone_line_state_reset_flash_count(line_state);
            }
         }

         if (NULL != pvt->owner) {
            if (ast_channel_trylock(pvt->owner)) {
               /*
                Because the channel is locked in another thread,
                we can't handle the line state change(s)
                So we set a short timeout to handle the line state change
                in a few milliseconds
               */
               bcmph_change_monitor_timeout(&(monitor_prms), bcmph_monitor_short_timeout);
               continue;
            }
#ifdef BCMPH_DEBUG
            bcm_assert(0 == pvt->owner_lock_count);
            pvt->owner_lock_count += 1;
#endif /* BCMPH_DEBUG */
         }
         monitor_prms.channel_is_locked = true;

         /*
          Handle codec
          Nothing to do, as codec change can occured only if asked
          and handled in bcmph_set_line_codec()
         */
         bcm_phone_line_state_reset_codec_change_count(line_state);

         /* Handle mode */
         if ((pvt->ast_channel.current_mode != line_state->mode)
             || (line_state->mode_change_count > 0)) {

            bcm_pr_debug("Line %lu, mode changes %lu times. Was %d and is now %d\n",
               (unsigned long)(pvt->index_line + 1),
               (unsigned long)(line_state->mode_change_count),
               (int)(pvt->ast_channel.current_mode),
               (int)(line_state->mode));

            bcmph_handle_mode_change(pvt, &(monitor_prms), line_state->mode);
            bcm_phone_line_state_reset_mode_change_count(line_state);
            if (!monitor_prms.channel_is_locked) {
               continue;
            }
         }

         /* Handle tone */
         if ((pvt->ast_channel.current_tone != line_state->tone)
             || (line_state->tone_change_count > 0)) {

            bcm_pr_debug("Line %lu, tone changes %lu times. Was %d and is now %d\n",
               (unsigned long)(pvt->index_line + 1),
               (unsigned long)(line_state->tone_change_count),
               (int)(pvt->ast_channel.current_tone),
               (int)(line_state->tone));

            bcmph_handle_tone_change(pvt, &(monitor_prms), line_state->tone);
            bcm_phone_line_state_reset_tone_change_count(line_state);
            if (!monitor_prms.channel_is_locked) {
               continue;
            }
         }

         /* Handle status */
         if ((line_state->status != pvt->ast_channel.current_status)
             || (line_state->status_change_count > 0)) {

            bcm_pr_debug("Line %lu, hookstate changes %lu times. Was %d and is now %d\n",
               (unsigned long)(pvt->index_line + 1), (unsigned long)(line_state->status_change_count),
               (int)(pvt->ast_channel.current_status), (int)(line_state->status));

            bcm_assert(((line_state->status == pvt->ast_channel.current_status)
                        && (0 == (line_state->status_change_count % 2)))
                       || ((line_state->status != pvt->ast_channel.current_status)
                           && (1 == (line_state->status_change_count % 2))));

            if ((line_state->status_change_count > 1) && (BCMPH_STATUS_OFF_HOOK == line_state->status)) {
               /*
                Several transitions between on hook and off hook :
                it means that if there was a call, it has been hung up
               */
               bcmph_handle_status_change(pvt, &(monitor_prms), BCMPH_STATUS_ON_HOOK);
               bcm_assert(monitor_prms.channel_is_locked);
            }

            /* We test if we must start or stop PCM */
            if (line_state->status != pvt->ast_channel.current_status) {
               if (BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status) {
                  if (t->monitor.line_off_hook_count > 0) {
                     t->monitor.line_off_hook_count -= 1;
                     if (0 == t->monitor.line_off_hook_count) {
                        bcmph_stop_pcm(t);
                     }
                  }
                  else {
                     bcm_assert(false);
                  }
               }
               else if (BCMPH_STATUS_OFF_HOOK == line_state->status) {
                  if (0 == t->monitor.line_off_hook_count) {
                     bcmph_start_pcm(t);
                  }
                  t->monitor.line_off_hook_count += 1;
               }
            }

            bcmph_handle_status_change(pvt, &(monitor_prms), line_state->status);
            bcm_phone_line_state_reset_status_change_count(line_state);
            if (!monitor_prms.channel_is_locked) {
               continue;
            }
         }

         /* Handle digits */
         if (line_state->digits_count > 0) {
            bcmph_handle_digits(pvt, &(monitor_prms), line_state->digits, line_state->digits_count);
            bcm_phone_line_state_reset_digits(line_state);
            if (!monitor_prms.channel_is_locked) {
               continue;
            }
         }

         /* Handle flash events */
         if (line_state->flash_count > 0) {
            bcmph_handle_flashes(pvt, &(monitor_prms), line_state->flash_count);
            bcm_phone_line_state_reset_flash_count(line_state);
            if (!monitor_prms.channel_is_locked) {
               continue;
            }
         }

         /* Do periodic tasks */
         bcmph_monitor_pvt(pvt, &(monitor_prms));
         if (!monitor_prms.channel_is_locked) {
            continue;
         }

         if (NULL != pvt->owner) {
#ifdef BCMPH_DEBUG
            pvt->owner_lock_count -= 1;
            bcm_assert(0 == pvt->owner_lock_count);
#endif /* BCMPH_DEBUG */
            ast_channel_unlock(pvt->owner);
         }
         monitor_prms.channel_is_locked = false;
      }
      bcmph_monitor_unlock(t);
   }

   bcm_pr_debug("Monitor thread stopping\n");

   return (NULL);
}

static int bcmph_stop_monitor(bcmph_chan_t *t)
{
   int ret = 0;

   bcm_pr_debug("Stopping the monitor\n");

   do { /* Empty loop */
      if (t->monitor.thread == pthread_self()) {
         ast_log(AST_LOG_ERROR, "Cannot kill myself\n");
         ret = -1;
         break;
      }
      if ((AST_PTHREADT_NULL != t->monitor.thread) && (AST_PTHREADT_STOP != t->monitor.thread)) {
         /* Stops the monitor thread */
         bcmph_prepare_stop_monitor(t);
         /*
          Don't send signal SIGURG with pthread_kill() because
          BCMPH_IOCTL_GET_LINE_STATES called by monitor thread,
          if interrupted by signal, could end returning -ERESTARTSYS.
          As the signal handler in Asterisk has the flags SA_RESTART,
          the syscall would be retried and the monitor will stay
          blocked on BCMPH_IOCTL_GET_LINE_STATES
         */
         bcm_pr_debug("Calling pthread_join()\n");
         ret = pthread_join(t->monitor.thread, NULL);
         if (ret) {
            ast_log(AST_LOG_ERROR, "pthread_join() failed : %d, %d\n", ret, errno);
         }
         else {
            bcm_pr_debug("Monitor stopped\n");
         }
         t->monitor.thread = AST_PTHREADT_NULL;
         ret = 0;
      }
      else {
         bcm_pr_debug("Monitor not running\n");
      }
   } while (false);

   return (ret);
}

static int bcmph_start_monitor(bcmph_chan_t *t)
{
   int ret = 0;

   bcm_pr_debug("Starting the monitor\n");

   do { /* Empty loop */
      /* If we're supposed to be stopped -- stay stopped */
      if (AST_PTHREADT_STOP == t->monitor.thread) {
         ret = 0;
         break;
      }
      bcm_assert(AST_PTHREADT_NULL == t->monitor.thread);
      bcmph_prepare_start_monitor(t);
      /* Start a new monitor */
      if (ast_pthread_create_background(&(t->monitor.thread), NULL, bcmph_do_monitor, t) < 0) {
         ast_log(AST_LOG_ERROR, "Unable to start monitor thread.\n");
         t->monitor.thread = AST_PTHREADT_NULL;
         ret = -1;
         break;
      }
      bcm_assert((AST_PTHREADT_STOP != t->monitor.thread) && (AST_PTHREADT_NULL != t->monitor.thread));
      ret = 0;
   } while (false);

   return (ret);
}

static void bcmph_close_device(bcmph_chan_t *t)
{
   if (-1 != t->dev_fd) {
      if (NULL != t->mmap.dev_start_addr) {
         munmap(t->mmap.dev_start_addr, t->mmap.desc.mmap_size);
         t->mmap.dev_start_addr = NULL;
      }
      /*
       No need to call BCMPH_IOCTL_STOP.. as all cleanup is done
       when file descriptor is closed
      */
      close(t->dev_fd);
      t->dev_fd = -1;
   }
}

static int bcmph_open_device(bcmph_chan_t *t)
{
   int ret = AST_MODULE_LOAD_SUCCESS;

   bcm_assert((-1 == t->dev_fd) && (!AST_LIST_EMPTY(&(t->pvt_list))));

   do { /* Empty loop */
      size_t i;
      bcm_phone_cfg_params_t *phone_cfg;
      bcm_phone_get_mmap_rbs_location_t *mmap_rbs_location;
      bcm_phone_get_line_states_t *get_line_states;
      bcmph_pvt_t *pvt;
      struct ast_format tmpfmt;

      bcm_pr_debug("Opening device '%s'\n", bcmph_device);

      /* Open the device */
      t->dev_fd = open(bcmph_device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
      if (-1 == t->dev_fd) {
         ast_log(AST_LOG_ERROR, "Unable to open device '%s'.  Aborting.\n", bcmph_device);
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }
      t->mmap.dev_start_addr = NULL;
      memset(&(t->mmap.desc), 0, sizeof(t->mmap.desc));

      bcm_pr_debug("Getting mmap description\n");

      /* Get the description of shared memory */
      ret = bcmph_do_ioctl(t, BCMPH_IOCTL_GET_MMAP_DESC, (unsigned long)(&(t->mmap.desc)), true);
      if (ret) {
         ast_log(AST_LOG_ERROR, "Error reading mmap description.\n");
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }

      /* Check that alignement of structures are coherent */
      if ((sizeof(bcm_phone_ioctl_param_t) != t->mmap.desc.ioctl_param_size)
          || ((t->mmap.desc.ioctl_param_off + t->mmap.desc.ioctl_param_size) > t->mmap.desc.mmap_size)) {
         ast_log(AST_LOG_ERROR, "Structures shared with the driver are not of the expected size\n");
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }

      bcm_pr_debug("Mapping memory region (size = %lu)\n", (unsigned long)(t->mmap.desc.mmap_size));

      /* Call mmap */
      t->mmap.dev_start_addr = mmap(NULL, t->mmap.desc.mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, t->dev_fd, 0);
      if (NULL == t->mmap.dev_start_addr) {
         ast_log(AST_LOG_ERROR, "mmap() failed.\n");
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }

      bcm_pr_debug("Configuring the device\n");

      /* Init pointer to shared zone used by ioctl() */
      t->mmap.dev_ioctl_param = (bcm_phone_ioctl_param_t *)(t->mmap.dev_start_addr + t->mmap.desc.ioctl_param_off);

      /* Initialize the driver */
      phone_cfg = &(t->mmap.dev_ioctl_param->p.start);
      memset(phone_cfg, 0, sizeof(*phone_cfg));
      for (i = 0; (i < ARRAY_LEN(phone_cfg->line_params)); i += 1) {
         phone_cfg->line_params[i].enable = 0;
      }
      phone_cfg->country = t->config.country;
      phone_cfg->pcm_use_16bits_timeslot = false;
      for (pvt = AST_LIST_FIRST(&(t->pvt_list)); (NULL != pvt); pvt = AST_LIST_NEXT(pvt, list)) {
         i = pvt->index_line;
         bcm_assert(pvt->line_cfg->enable);
         phone_cfg->line_params[i].enable = 1;
         /*
          For initial config we choose the codec that uses the less timeslots
          Alaw is told to be simpler to process that ulaw so we give it precedence over ulaw
         */
         if (ast_format_cap_iscompatible(pvt->line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_ALAW, 0))) {
            phone_cfg->line_params[i].codec = BCMPH_CODEC_ALAW;
         }
         else if (ast_format_cap_iscompatible(pvt->line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_ULAW, 0))) {
            phone_cfg->line_params[i].codec = BCMPH_CODEC_ULAW;
         }
         else if (ast_format_cap_iscompatible(pvt->line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0))) {
            phone_cfg->line_params[i].codec = BCMPH_CODEC_LINEAR;
            phone_cfg->pcm_use_16bits_timeslot = true;
         }
         else if (ast_format_cap_iscompatible(pvt->line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR16, 0))) {
            phone_cfg->line_params[i].codec = BCMPH_CODEC_LINEAR16;
            phone_cfg->pcm_use_16bits_timeslot = true;
         }
      }

      t->mmap.dev_ioctl_param->size = sizeof(*phone_cfg);
      if (bcmph_do_ioctl(t, BCMPH_IOCTL_START_MM, 0, true)) {
         ast_log(AST_LOG_ERROR, "Unable to start device\n");
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }

      /*
       Now that driver is configured, get the zones in shared memory
       assigned to line ring buffer
      */
      mmap_rbs_location = &(t->mmap.dev_ioctl_param->p.get_mmap_rbs_location);
      memset(mmap_rbs_location, 0, sizeof(*mmap_rbs_location));
      t->mmap.dev_ioctl_param->size = sizeof(*mmap_rbs_location);
      if (bcmph_do_ioctl(t, BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM, 0, true)) {
         ast_log(AST_LOG_ERROR, "Unable to get ring buffer addresses of lines\n");
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }
      AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
         i = pvt->index_line;
         bcm_assert(i < ARRAY_LEN(mmap_rbs_location->rbs));
         if ((sizeof(bcm_ring_buf_desc_t) != mmap_rbs_location->rbs[i].rx_ring_buf_desc_size)
             || (sizeof(bcm_ring_buf_desc_t) != mmap_rbs_location->rbs[i].tx_ring_buf_desc_size)) {
            ast_log(AST_LOG_ERROR, "Structures shared with the driver are not of the expected size\n");
            ret = AST_MODULE_LOAD_FAILURE;
            break;
         }
         pvt->ast_channel.mmap.dev_rx_ring_buf_desc = (bcm_ring_buf_desc_t *)(t->mmap.dev_start_addr + mmap_rbs_location->rbs[i].rx_ring_buf_desc_off);
         pvt->ast_channel.mmap.dev_tx_ring_buf_desc = (bcm_ring_buf_desc_t *)(t->mmap.dev_start_addr + mmap_rbs_location->rbs[i].tx_ring_buf_desc_off);
         bcm_ring_buf_init(&(pvt->ast_channel.mmap.dev_rx_ring_buf),
            t->mmap.dev_start_addr + mmap_rbs_location->rbs[i].rx_buffer_offset, mmap_rbs_location->rbs[i].rx_buffer_size);
         bcm_ring_buf_init(&(pvt->ast_channel.mmap.dev_tx_ring_buf),
            t->mmap.dev_start_addr + mmap_rbs_location->rbs[i].tx_buffer_offset, mmap_rbs_location->rbs[i].tx_buffer_size);
      }
      if (AST_MODULE_LOAD_SUCCESS != ret) {
         break;
      }

      bcm_pr_debug("Getting initial line states\n");

      /* Finally init the state of the lines */
      get_line_states = &(t->mmap.dev_ioctl_param->p.get_line_states);
      memset(get_line_states, 0, sizeof(*get_line_states));
      t->mmap.dev_ioctl_param->size = sizeof(*get_line_states);
      get_line_states->wait = 0;
      if (bcmph_do_ioctl(t, BCMPH_IOCTL_GET_LINE_STATES_MM, 0, true)) {
         ast_log(AST_LOG_ERROR, "Unable to get line states\n");
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }
      bcmph_monitor_lock(t);
      t->monitor.line_off_hook_count = 0;
      AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
         bcm_phone_line_state_t *line_state;
         bcm_phone_line_mode_t mode_wanted;
         bcm_phone_line_tone_t tone_wanted;
         i = pvt->index_line;
         bcm_assert(i < ARRAY_LEN(get_line_states->line_state));
         line_state = &(get_line_states->line_state[i]);
         switch (line_state->codec) {
            case BCMPH_CODEC_ALAW:
               ast_format_set(&(pvt->ast_channel.current_format), AST_FORMAT_ALAW, 0);
               pvt->ast_channel.bytes_per_sample = 1;
               pvt->ast_channel.frame_size_read = (BCMPH_MAX_BUF / 2);
               break;
            case BCMPH_CODEC_ULAW:
               ast_format_set(&(pvt->ast_channel.current_format), AST_FORMAT_ULAW, 0);
               pvt->ast_channel.bytes_per_sample = 1;
               pvt->ast_channel.frame_size_read = (BCMPH_MAX_BUF / 2);
               break;
            case BCMPH_CODEC_LINEAR:
               ast_format_set(&(pvt->ast_channel.current_format), AST_FORMAT_SLINEAR, 0);
               pvt->ast_channel.bytes_per_sample = 2;
               pvt->ast_channel.frame_size_read = BCMPH_MAX_BUF;
               break;
            case BCMPH_CODEC_LINEAR16:
               ast_format_set(&(pvt->ast_channel.current_format), AST_FORMAT_SLINEAR16, 0);
               pvt->ast_channel.bytes_per_sample = 4;
               pvt->ast_channel.frame_size_read = BCMPH_MAX_BUF;
               break;
            default:
               bcm_assert(false);
               break;
         }
         pvt->ast_channel.frame_size_read -= (pvt->ast_channel.frame_size_read % pvt->ast_channel.bytes_per_sample);
         pvt->ast_channel.bytes_not_written_len = 0;
         pvt->ast_channel.current_mode = line_state->mode;
         pvt->ast_channel.current_tone = line_state->tone ;
         pvt->ast_channel.current_status = line_state->status;

         mode_wanted = BCMPH_MODE_IDLE;
         tone_wanted = BCMPH_TONE_NONE;
         if (BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status) {
            pvt->ast_channel.state = BCMPH_ST_OFF_NO_SERVICE;
            tone_wanted = BCMPH_TONE_INVALID;
            t->monitor.line_off_hook_count += 1;
         }
         else if (BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status) {
            pvt->ast_channel.state = BCMPH_ST_ON_IDLE;
         }
         else {
            bcm_assert(BCMPH_STATUS_DISCONNECTED == pvt->ast_channel.current_status);
            pvt->ast_channel.state = BCMPH_ST_DISCONNECTED;
         }
         if (line_state->mode != mode_wanted) {
            bcmph_set_line_mode(pvt, mode_wanted, tone_wanted, 0);
         }
         else if (line_state->tone != tone_wanted) {
            bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(tone_wanted), 0);
         }
         bcm_phone_line_state_reset(&(pvt->monitor.line_state), line_state->status,
            line_state->codec, line_state->mode, line_state->tone);
      }

      if ((AST_MODULE_LOAD_SUCCESS == ret)
          && (t->monitor.line_off_hook_count > 0)
          && (bcmph_start_pcm(t))) {
         bcmph_monitor_unlock(t);
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }
      bcmph_monitor_unlock(t);
   } while (false);

   if (AST_MODULE_LOAD_SUCCESS != ret) {
      bcmph_close_device(t);
   }

   return (ret);
}

static inline bcmph_pvt_t *bcmph_get_pvt(struct ast_channel *ast)
{
   bcmph_pvt_t *pvt = ast_channel_tech_pvt(ast);

   if ((NULL == pvt) || (pvt->owner != ast)) {
      ast_log(AST_LOG_WARNING, "Channel '%s' unlink to a line or linked to another line\n", ast_channel_name(ast));
      pvt = NULL;
   }
   return (pvt);
}

/*!
 * \brief Make a call
 * \note The channel is locked when this function gets called.
 * \param ast which channel to make the call on
 * \param addr destination of the call
 * \param timeout time to wait on for connect (Doesn't seem to be used.)
 * \retval 0 on success
 * \retval -1 on failure
 */
static int bcmph_chan_call(struct ast_channel *ast, const char *addr, int timeout)
{
   int ret = -1;
   bcmph_pvt_t *pvt = bcmph_get_pvt(ast);

   bcm_pr_debug("bcmph_chan_call(ast='%s', addr='%s', timeout=%d)\n", ast_channel_name(ast), addr, (int)(timeout));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      do { /* Empty loop */
         /* If phone is off hook, call() is not allowed */
         if (BCMPH_STATUS_ON_HOOK != pvt->ast_channel.current_status) {
            ast_log(AST_LOG_NOTICE, "'%s' is busy\n", ast_channel_name(ast));
            ast_setstate(ast, AST_STATE_BUSY);
            ast_queue_control(ast, AST_CONTROL_BUSY);
         }
         else {
            enum ast_channel_state ast_state = ast_channel_state(ast);
            if ((AST_STATE_DOWN != ast_state) && (AST_STATE_RESERVED != ast_state)) {
               ast_log(AST_LOG_WARNING, "bcmph_chan_call() called on '%s', neither down nor reserved\n",
                  ast_channel_name(ast));
            }
            else {
               struct timeval utc_time = ast_tvnow();
               struct ast_tm tm;
               bcm_phone_cid_t cid;

               /*
                bcmph_chan_call() can't be called if
                (BCMPH_ST_ON_IDLE == pvt->ast_channel.state) because
                pvt->owner is NULL, or if
                (BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state)
                because ast_channel_state(ast) is AST_STATE_RING
                So, as (BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)
                (BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state)
               */
               bcm_assert(BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state);

               ast_localtime(&(utc_time), &(tm), NULL);

               memset(&(cid), 0, sizeof(cid));
               snprintf(cid.month, sizeof(cid.month), "%02d", (int)(tm.tm_mon + 1));
               snprintf(cid.day, sizeof(cid.day),     "%02d", (int)(tm.tm_mday));
               snprintf(cid.hour, sizeof(cid.hour),   "%02d", (int)(tm.tm_hour));
               snprintf(cid.min, sizeof(cid.min),     "%02d", (int)(tm.tm_min));
               /* the standard format of ast->callerid is:  "name" <number>, but not always complete */
               if ((!ast_channel_connected(ast)->id.name.valid)
                   || (ast_strlen_zero(ast_channel_connected(ast)->id.name.str))) {
                  strcpy(cid.name, DEFAULT_CALLER_ID);
               }
               else {
                  ast_copy_string(cid.name, ast_channel_connected(ast)->id.name.str, sizeof(cid.name));
               }

               if ((ast_channel_connected(ast)->id.number.valid) && (ast_channel_connected(ast)->id.number.str)) {
                  ast_copy_string(cid.number, ast_channel_connected(ast)->id.number.str, sizeof(cid.number));
               }

               bcm_pr_debug("Ringing '%s' on '%s' (with CID '%s', '%s')\n",
                  addr, ast_channel_name(ast), cid.number, cid.name);

               /* TODO : handle CID when driver allows it */

               /*
                Here it's important to handle the return code of bcmph_set_line_mode()
                because if the user can't hear the phone ringing, there's
                little chance that he hooks off the phone
               */
               if (bcmph_set_line_mode(pvt, BCMPH_MODE_ON_RINGING, BCMPH_TONE_NONE, 0)) {
                  break;
               }

               ast_setstate(ast, AST_STATE_RINGING);
               ast_queue_control(ast, AST_CONTROL_RINGING);
               bcmph_set_new_state(pvt, BCMPH_ST_ON_RINGING, BCMPH_EV_AST_CALL);
               ret = 0;
            }
         }
      } while (false);
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }

   return (ret);
}

/*!
 * \brief Answer the channel
 *
 * \note The channel is locked when this function gets called.
 */
static int bcmph_chan_answer(struct ast_channel *ast)
{
   int ret = -1;
   bcmph_pvt_t *pvt = bcmph_get_pvt(ast);

   /* Remote end has answered the call */
   bcm_pr_debug("bcmph_chan_answer(ast='%s')\n", ast_channel_name(ast));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      /* If phone not off hook, answer() is not allowed */
      if (BCMPH_STATUS_OFF_HOOK != pvt->ast_channel.current_status) {
         ast_log(AST_LOG_WARNING, "Channel '%s' can't answer now, because not off hook\n", ast_channel_name(ast));
      }
      else {
         /* We accept that answer() can be called if BCMPH_ST_OFF_TALKING == pvt->ast_channel.state */
         if (BCMPH_ST_OFF_TALKING != pvt->ast_channel.state) {
            bcm_assert(BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state);
            ret = bcmph_setup(pvt, ast_channel_rawreadformat(pvt->owner), AST_STATE_UP);
            if (!ret) {
               ast_channel_rings_set(pvt->owner, 0);
               bcmph_set_new_state(pvt, BCMPH_ST_OFF_TALKING, BCMPH_EV_AST_ANSWER);
            }
         }
      }
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }
   return (ret);
}

/*!
 * \brief Fix up a channel:  If a channel is consumed, this is called.  Basically update any ->owner links
 *
 * \note The channel is locked when this function gets called.
 */
static int bcmph_chan_fixup(struct ast_channel *old, struct ast_channel *new)
{
   bcmph_pvt_t *pvt = bcmph_get_pvt(old);

   bcm_pr_debug("bcmph_chan_fixup(old='%s', new='%s')\n", ast_channel_name(old), ast_channel_name(new));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      /*
       Here the channel is locked and we get the monitor lock.
       In the monitor we get the monitor lock and then only TRY to lock the channel
       IMHO no deadlock can occur
      */
      bcmph_monitor_lock(pvt->channel);
      pvt->owner = new;
      bcmph_monitor_unlock(pvt->channel);
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }

   return (0);
}

/*!
 * \brief Start sending a literal DTMF digit
 *
 * \note The channel is not locked when this function gets called.
 */
static int bcmph_chan_digit_begin(struct ast_channel *ast, char digit)
{
   bcm_pr_debug("bcmph_chan_digit_begin(ast='%s', digit='%c')\n", ast_channel_name(ast), (char)(digit));

   /* Done in bcmph_chan_digit_end() */

   return (0);
}

/*!
 * \brief Stop sending a literal DTMF digit
 *
 * \note The channel is not locked when this function gets called.
 */
static int bcmph_chan_digit_end(struct ast_channel *ast, char digit, unsigned int duration)
{
   int ret = 0;
   bcmph_pvt_t *pvt;
   bcm_phone_line_tone_t tone;

   bcm_pr_debug("bcmph_chan_digit_end(ast='%s', digit='%c', duration=%u)\n",
      ast_channel_name(ast), (char)(digit), (unsigned int)(duration));

   ast_channel_lock(ast);

   pvt = bcmph_get_pvt(ast);
   if (NULL == pvt) {
      ret = -1;
   }
   else {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      if (BCMPH_STATUS_OFF_HOOK != pvt->ast_channel.current_status) {
         ast_log(AST_LOG_WARNING, "Can't send a DTMF while not off_hook\n");
         ret = -1;
      }
      else {
         bcm_assert((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
            || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state));
         switch (digit) {
            case '0': {
               tone = BCMPH_TONE_DTMF_0;
               break;
            }
            case '1': {
               tone = BCMPH_TONE_DTMF_1;
               break;
            }
            case '2': {
               tone = BCMPH_TONE_DTMF_2;
               break;
            }
            case '3': {
               tone = BCMPH_TONE_DTMF_3;
               break;
            }
            case '4': {
               tone = BCMPH_TONE_DTMF_4;
               break;
            }
            case '5': {
               tone = BCMPH_TONE_DTMF_5;
               break;
            }
            case '6': {
               tone = BCMPH_TONE_DTMF_6;
               break;
            }
            case '7': {
               tone = BCMPH_TONE_DTMF_7;
               break;
            }
            case '8': {
               tone = BCMPH_TONE_DTMF_8;
               break;
            }
            case '9': {
               tone = BCMPH_TONE_DTMF_9;
               break;
            }
            case '*': {
               tone = BCMPH_TONE_DTMF_ASTER;
               break;
            }
            case '#': {
               tone = BCMPH_TONE_DTMF_POUND;
               break;
            }
            case 'A': {
               tone = BCMPH_TONE_DTMF_A;
               break;
            }
            case 'B': {
               tone = BCMPH_TONE_DTMF_B;
               break;
            }
            case 'C': {
               tone = BCMPH_TONE_DTMF_C;
               break;
            }
            case 'D': {
               tone = BCMPH_TONE_DTMF_D;
               break;
            }
            default: {
               ast_log(AST_LOG_WARNING, "Unknown digit '%c'\n", (char)(digit));
               ret = -1;
               break;
            }
         }
         if (!ret) {
            bcm_pr_debug("Playing DTMF digit '%c'\n", (char)(digit));
            if (duration > ((unsigned int)(BCMPH_TONE_OFF_TIME_MASK >> BCMPH_TONE_OFF_TIME_SHIFT))) {
               duration = (BCMPH_TONE_OFF_TIME_MASK >> BCMPH_TONE_OFF_TIME_SHIFT);
            }
            bcmph_set_line_tone(pvt, bcm_phone_line_tone_code(tone, 0, duration), 1);
         }
      }
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }

   ast_channel_unlock(ast);

   return (ret);
}

/*!
 * \brief Indicate a particular condition (e.g. AST_CONTROL_BUSY or AST_CONTROL_RINGING or AST_CONTROL_CONGESTION
 *
 * \note The channel is locked when this function gets called.
 */
static int bcmph_chan_indicate(struct ast_channel *ast, int condition, const void *data, size_t datalen)
{
   int ret = -1;
   bcmph_pvt_t *pvt = bcmph_get_pvt(ast);

   bcm_pr_debug("bcmph_chan_indicate(ast='%s', condition=%d)\n", ast_channel_name(ast), (int)(condition));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      if (BCMPH_STATUS_OFF_HOOK != pvt->ast_channel.current_status) {
         ast_log(AST_LOG_WARNING, "Can't indicate a condition while not off_hook\n");
      }
      else {
         bcm_assert((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
            || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state));
         switch (condition) {
            case -1: {
               bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_NONE), 1);
               ret = 0;
               break;
            }
            case AST_CONTROL_CONGESTION:
            case AST_CONTROL_BUSY: {
               bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_BUSY), 0);
               ret = 0;
               break;
            }
            case AST_CONTROL_RINGING: {
               bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_RINGBACK), 0);
               ret = 0;
               break;
            }
            case AST_CONTROL_HOLD: {
               ast_moh_start(ast, data, ('\0' != pvt->line_cfg->moh_interpret[0]) ? pvt->line_cfg->moh_interpret : NULL);
               ret = 0;
               break;
            }
            case AST_CONTROL_UNHOLD: {
               ast_moh_stop(ast);
               ret = 0;
               break;
            }
            case AST_CONTROL_PROGRESS:
            case AST_CONTROL_PROCEEDING:
            case AST_CONTROL_VIDUPDATE:
            case AST_CONTROL_SRCUPDATE: {
               ret = 0;
               break;
            }
            case AST_CONTROL_INCOMPLETE:
            case AST_CONTROL_PVT_CAUSE_CODE: {
               break;
            }
            default: {
               /* -1 lets asterisk generate the tone */
               ast_log(AST_LOG_WARNING, "Condition %d is not supported on channel '%s'\n", (int)(condition), ast_channel_name(ast));
               break;
            }
         }
      }
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }
   return (ret);
}

/*!
 * \brief Read a frame, in standard format (see frame.h)
 *
 * \note The channel is locked when this function gets called.
 * As we don't associate a file descriptor to a channel, in theory this function
 * will be called only if there's a jitter buffer
 */
static struct ast_frame *bcmph_chan_read(struct ast_channel *ast)
{
   struct ast_frame *ret = &(ast_null_frame);
   bcmph_pvt_t *pvt = bcmph_get_pvt(ast);

   /* bcm_pr_debug("bcmph_chan_read(ast='%s')\n", ast_channel_name(ast)); */

   if (NULL != pvt) {
      bool do_unlock = false;

#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      do { /* Empty loop */
         if ((BCMPH_STATUS_OFF_HOOK != pvt->ast_channel.current_status)
             || (AST_STATE_UP != ast_channel_state(ast))) {
            /* Don't try to send audio on-hook */
            ast_log(AST_LOG_WARNING, "Trying to receive audio while not off hook or not in the correct state\n");
            break;
         }

         bcm_assert(BCMPH_ST_OFF_TALKING == pvt->ast_channel.state);

         bcmph_mmap_lock(pvt->channel);
         do_unlock = true;

         ret = bcmph_read_frame(pvt,
            ((DETECT_DTMF_ALWAYS == pvt->line_cfg->detect_dtmf)
             || (DETECT_DTMF_WHEN_CONNECTED == pvt->line_cfg->detect_dtmf)),
            &(do_unlock));
         if (NULL == ret) {
            ret = &(ast_null_frame);
         }
      } while (false);
      if (do_unlock) {
         bcmph_mmap_unlock(pvt->channel);
      }
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }

   /* Reading is also done by the monitor thread */
   return (ret);
}

/*!
 * \brief Write a frame, in standard format (see frame.h)
 *
 * \note The channel is locked when this function gets called.
 */
static int bcmph_chan_write(struct ast_channel *ast, struct ast_frame *frame)
{
   int ret = 0;
   bcmph_pvt_t *pvt = bcmph_get_pvt(ast);

   /* bcm_pr_debug("bcmph_chan_write(ast='%s')\n", ast_channel_name(ast)); */

   if (NULL != pvt) {
      bool do_unlock = false;

#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      do { /* Empty loop */
         __u8 *pos;
         size_t todo;

         /* Write a frame of (presumably voice) data */
         if (AST_FRAME_VOICE != frame->frametype) {
            ast_log(AST_LOG_WARNING, "Don't know what to do with frame type %d\n", (int)(frame->frametype));
            break;
         }

         if (frame->datalen <= 0) {
            bcm_pr_debug("Void frame\n");
            break;
         }

         if (!ast_format_cap_iscompatible(pvt->line_cfg->capabilities, &(frame->subclass.format))) {
            ast_log(AST_LOG_WARNING, "Cannot handle frames in '%s' format\n", ast_getformatname(&(frame->subclass.format)));
            ret = -1;
            break;
         }

         if ((BCMPH_STATUS_OFF_HOOK != pvt->ast_channel.current_status)
             || (BCMPH_ST_OFF_DIALING == pvt->ast_channel.state)
             /* || (AST_STATE_UP != ast_channel_state(pvt->owner)) */) {
            /* Don't try to send audio on-hook or not in the correct state */
            ast_log(AST_LOG_WARNING, "Trying to send audio while not off hook or not in the correct state\n");
            break;
         }

         bcm_assert((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
            || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state));

         if (pvt->ast_channel.current_format.id != frame->subclass.format.id) {
            if (bcmph_set_line_codec(pvt, &(frame->subclass.format), BCMPH_MODE_UNSPECIFIED, BCMPH_TONE_UNSPECIFIED)) {
               ret = -1;
               break;
            }
         }

         bcmph_mmap_lock(pvt->channel);
         do_unlock = true;
         pos = frame->data.ptr;
         todo = frame->datalen;
         bcm_assert(ARRAY_LEN(pvt->ast_channel.bytes_not_written) >= pvt->ast_channel.bytes_per_sample);
         for (;;) {
            size_t free_space_in_samples = bcm_ring_buf_get_free_space(&(pvt->ast_channel.mmap.dev_tx_ring_buf)) / pvt->ast_channel.bytes_per_sample;
            size_t tmp;

            /* Do we have enough space to write one sample ? */
            if (free_space_in_samples >= 1) {
               /* Yes */
               size_t len_to_write = 0;

               /* Are there some bytes not written ? */
               if (pvt->ast_channel.bytes_not_written_len > 0) {
                  /* Yes */
                  if ((pvt->ast_channel.bytes_not_written_len + todo) < pvt->ast_channel.bytes_per_sample) {
                     /*
                      Not enough byte to make a complete sample, we just
                      add bytes to buffer pvt->ast_channel.bytes_not_written
                     */
                     memcpy(&(pvt->ast_channel.bytes_not_written[pvt->ast_channel.bytes_not_written_len]), pos, todo);
                     pvt->ast_channel.bytes_not_written_len += todo;
                     pos += todo;
                     todo = 0;
                     break;
                  }
                  bcm_assert(pvt->ast_channel.bytes_not_written_len <= pvt->ast_channel.bytes_per_sample);
                  tmp = pvt->ast_channel.bytes_per_sample - pvt->ast_channel.bytes_not_written_len;
                  bcm_assert(tmp <= todo);
                  /* We add bytes to buffer pvt->ast_channel.bytes_not_written to make a complete sample */
                  if (tmp > 0) {
                     memcpy(&(pvt->ast_channel.bytes_not_written[pvt->ast_channel.bytes_not_written_len]), pos, tmp);
                     pvt->ast_channel.bytes_not_written_len += tmp;
                     pos += tmp;
                     todo -= tmp;
                  }
                  /* We copy in the ring buffer, the bytes not previously written */
                  bcm_ring_buf_add(&(pvt->ast_channel.mmap.dev_tx_ring_buf),
                     pvt->ast_channel.bytes_not_written,
                     pvt->ast_channel.bytes_not_written_len);
                  len_to_write += pvt->ast_channel.bytes_not_written_len;
                  free_space_in_samples -= 1;
               }
               /* We copy the maximum samples that we can in the ring buffer */
               tmp = (todo / pvt->ast_channel.bytes_per_sample);
               if (tmp > free_space_in_samples) {
                  tmp = free_space_in_samples;
               }
               if (tmp > 0) {
                  tmp *= pvt->ast_channel.bytes_per_sample;
                  bcm_ring_buf_add(&(pvt->ast_channel.mmap.dev_tx_ring_buf), pos, tmp);
                  pos += tmp;
                  todo -= tmp;
                  len_to_write += tmp;
                  free_space_in_samples -= (tmp / pvt->ast_channel.bytes_per_sample);
               }
               if (len_to_write > 0) {
                  if (bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_WRITE_MM, (unsigned long)((len_to_write << 8) | pvt->index_line), true)) {
                     ast_log(AST_LOG_ERROR, "Can't add data in TX buffer of line %lu\n", (unsigned long)(pvt->index_line + 1));
                     ret = -1;
                     break;
                  }
                  bcmph_update_ring_bufs_desc(pvt->channel);
                  pvt->ast_channel.bytes_not_written_len = 0;
                  if (todo > pvt->ast_channel.bytes_per_sample) {
                     continue;
                  }
               }
            }
            if (todo <= 0) {
               break;
            }
            /*
             Not enough free space to write a sample or not enough bytes to write
             a sample
            */
            tmp = (pvt->ast_channel.bytes_not_written_len + todo) % pvt->ast_channel.bytes_per_sample;
            if (tmp > 0) {
               if (tmp <= todo) {
                  pos += (todo - tmp);
                  memcpy(pvt->ast_channel.bytes_not_written, pos, tmp);
                  pos += tmp;
                  todo -= tmp;
               }
               else {
                  /* tmp > todo */
                  memmove(pvt->ast_channel.bytes_not_written,
                     &(pvt->ast_channel.bytes_not_written[pvt->ast_channel.bytes_not_written_len + todo - tmp]),
                     tmp - todo);
                  memcpy(&(pvt->ast_channel.bytes_not_written[tmp - todo]), pos, todo);
                  pos += todo;
                  todo = 0;
               }
            }
            pvt->ast_channel.bytes_not_written_len = tmp;
            if (todo > 0) {
               ast_log(AST_LOG_WARNING, "Only wrote %lu of %lu bytes of audio data to line %lu\n",
                  (unsigned long)(frame->datalen - todo), (unsigned long)(frame->datalen),
                  (unsigned long)(pvt->index_line + 1));
            }
            break;
         }
      } while (false);

      if (do_unlock) {
         if (BCMPH_ST_OFF_TALKING == pvt->ast_channel.state) {
            bcmph_queue_read(pvt, &(do_unlock));
         }
         if (do_unlock) {
            bcmph_mmap_unlock(pvt->channel);
            do_unlock = false;
         }
      }
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }

   return (ret);
}

/*!
 * \brief Hangup (and possibly destroy) the channel
 *
 * \note The channel is locked when this function gets called.
 */
static int bcmph_chan_hangup(struct ast_channel *ast)
{
   bcmph_pvt_t *pvt = ast_channel_tech_pvt(ast);

   bcm_pr_debug("bcmph_chan_hangup(ast='%s')\n", ast_channel_name(ast));

   if ((NULL == pvt) || (pvt->owner != ast)) {
      ast_log(AST_LOG_DEBUG, "Channel '%s' unlink or link to another line\n", ast_channel_name(ast));
      if ((NULL != pvt) && (NULL == pvt->owner)) {
         /* Can happen if bcmph_chan_hangup() called while we are still in bcmph_new() */
         ast_channel_tech_pvt_set(ast, NULL);
      }
   }
   else {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      /*
       Here the channel is locked and we get the monitor lock.
       In the monitor we get the monitor lock and then only TRY to lock the channel
       IMHO no deadlock can occur
      */
      bcmph_monitor_lock(pvt->channel);
      bcmph_unlink_from_ast_channel(pvt, BCMPH_EV_AST_HANGUP, false);
      bcm_assert(NULL == pvt->owner);
      bcmph_monitor_unlock(pvt->channel);
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
   }
   bcm_pr_debug("'%s' hung up\n", ast_channel_name(ast));
   ast_setstate(ast, AST_STATE_DOWN);

   return (0);
}

/******************************************************************************/

static struct ast_channel *bcmph_chan_request(const char *type,
   struct ast_format_cap *cap, const struct ast_channel *requestor,
   const char *addr, int *cause);

static bcmph_chan_t bcmph_chan = {
   .chan_tech = {
      .type = bcmph_chan_type,
      .description = bcmph_chan_desc,
      .requester = bcmph_chan_request,
      .send_digit_begin = bcmph_chan_digit_begin,
      .send_digit_end = bcmph_chan_digit_end,
      .call = bcmph_chan_call,
      .hangup = bcmph_chan_hangup,
      .answer = bcmph_chan_answer,
      .read = bcmph_chan_read,
      .write = bcmph_chan_write,
      .indicate = bcmph_chan_indicate,
      .fixup = bcmph_chan_fixup
   },
   .channel_registered = false,
   .pvt_list = AST_LIST_HEAD_NOLOCK_INIT_VALUE,
   .dev_fd = -1,
   .mmap = {
      .dev_start_addr = NULL,
   },
   .monitor = {
      .run = false,
      .thread = AST_PTHREADT_NULL,
   },
   /*
    We don't set properties because we don't want that bridge between this channel
    and another requires jitter buffers.
    Even if we know that when we change codec or set
    mode BCMPH_OFF_TALKING of one line, read for other lines is blocked for a
    few milliseconds (in theory less than 20 ms).
    If this is a problem (only when two or more lines are in conversation
    simultaneously) we can use the function jitterbuffer in dialplan.
   */
   /* .properties = AST_CHAN_TP_WANTSJITTER | AST_CHAN_TP_CREATESJITTER, */
};

/*!
 * \brief Requester - to set up call data structures (pvt's)
 *
 * \param type type of channel to request
 * \param cap Format capabilities for requested channel
 * \param requestor channel asking for data
 * \param addr destination of the call
 * \param cause Cause of failure
 *
 * \details
 * Request a channel of a given type, with addr as optional information used
 * by the low level module
 *
 * \retval NULL failure
 * \retval non-NULL channel on success
 */
static struct ast_channel *bcmph_chan_request(const char *type,
   struct ast_format_cap *cap, const struct ast_channel *requestor,
   const char *addr, int *cause)
{
   struct ast_channel *ret = NULL;
   bcmph_chan_t *t = &(bcmph_chan);
   bcmph_pvt_t *pvt;

   bcm_pr_debug("bcmph_chan_request(type='%s', addr='%s')\n", type, addr);

   if (ast_strlen_zero(addr)) {
      ast_log(AST_LOG_WARNING, "Unable to create channel with empty destination.\n");
      *cause = AST_CAUSE_CHANNEL_UNACCEPTABLE;
   }
   else {
      /* Search for an unowned channel */
      bcmph_monitor_lock(t);
      *cause = AST_CAUSE_CHANNEL_UNACCEPTABLE;
      AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
         char tmp[16];
         size_t length;
         sprintf(tmp, "%lu", (unsigned long)(pvt->index_line + 1));
         length = strlen(tmp);
         if ((0 == strncmp(addr, tmp, length)) && (!isalnum(addr[length]))) {
            if (ast_format_cap_has_joint(cap, pvt->channel->chan_tech.capabilities)) {
               if ((NULL == pvt->owner) && (BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)) {
                  bcm_assert(BCMPH_ST_ON_IDLE == pvt->ast_channel.state);
                  bcmph_new(pvt, BCMPH_ST_ON_PRE_RINGING, BCMPH_EV_AST_REQUEST,
                     ((NULL != requestor) ? ast_channel_linkedid(requestor) : NULL), NULL);
                  ret = pvt->owner;
               }
               else {
                  bcm_pr_debug("Channel is busy\n");
                  *cause = AST_CAUSE_BUSY;
               }
            }
            else {
               char buf[256];
               ast_log(AST_LOG_WARNING, "Asked to get a channel of unsupported format '%s'\n", ast_getformatname_multiple(buf, sizeof(buf), cap));
            }
            break;
         }
      }
      bcmph_monitor_unlock(t);
   }
   return (ret);
}

#ifdef BCMPH_NOHW
static char *bcmph_cli_set(struct ast_cli_entry *e, int cmd, struct ast_cli_args *a)
{
   char *ret = CLI_SUCCESS;
   bcmph_chan_t *t = &(bcmph_chan);

   switch (cmd) {
      case CLI_INIT: {
         e->command = "bcmph set";
         e->usage =
            "Usage: bcmph set line [on|off|disc]\n"
            "       Set status for line 'line' :\n"
            "       - 'on' line is on hook\n"
            "       - 'off' line is off hook\n"
            "       - 'disc' line is disconnected\n";
         return (NULL);
      }
      case CLI_GENERATE: {
         return (NULL);
      }
   }

   do { /* Empty loop */
      int tmp;
      bcm_phone_set_line_state_t set_line_state;
      char str_status[64];
      char *f;

      if (4 != a->argc) {
         ret = CLI_SHOWUSAGE;
         break;
      }

      memset(&(set_line_state), 0, sizeof(set_line_state));
      set_line_state.digits_count = 0;
      set_line_state.flash_count = 0;

      /* We parse the line number */
      if ((1 != sscanf(a->argv[2], " %10d ", &(tmp))) || (tmp <= 0) || (tmp > BCMPH_MAX_LINES)) {
         ast_cli(a->fd, "Invalid line '%s'\n", a->argv[2]);
         ret = CLI_FAILURE;
         break;
      }
      tmp -= 1;
      set_line_state.line = (size_t)(tmp);

      /* We parse the status */
      ast_copy_string(str_status, a->argv[3], ARRAY_LEN(str_status));
      f = ast_strip(str_status);
      if (!strcasecmp(f, "on")) {
         set_line_state.status = BCMPH_STATUS_ON_HOOK;
      }
      else if (!strcasecmp(f, "off")) {
         set_line_state.status = BCMPH_STATUS_OFF_HOOK;
      }
      else if (!strcasecmp(f, "disc")) {
         set_line_state.status = BCMPH_STATUS_DISCONNECTED;
      }
      else {
         ast_cli(a->fd, "Invalid status '%s'\n", f);
         ret = CLI_FAILURE;
         break;
      }

      /* We change the status of the line */
      if (bcmph_do_ioctl(t, BCMPH_IOCTL_SET_LINE_STATE, (unsigned long)(&(set_line_state)), true)) {
         ret = CLI_FAILURE;
         break;
      }
   } while (false);

   return (ret);
}

static char *bcmph_cli_dial(struct ast_cli_entry *e, int cmd, struct ast_cli_args *a)
{
   char *ret = CLI_SUCCESS;
   bcmph_chan_t *t = &(bcmph_chan);

   switch (cmd) {
      case CLI_INIT: {
         e->command = "bcmph dial";
         e->usage =
            "Usage: bcmph dial line [digits]\n"
            "       Dials the digits\n";
         return (NULL);
      }
      case CLI_GENERATE: {
         return (NULL);
      }
   }

   do { /* Empty loop */
      int tmp;
      bcm_phone_set_line_state_t set_line_state;
      const char *f;

      if (4 != a->argc) {
         ret = CLI_SHOWUSAGE;
         break;
      }

      memset(&(set_line_state), 0, sizeof(set_line_state));
      set_line_state.status = BCMPH_STATUS_UNSPECIFIED;
      set_line_state.digits_count = 0;
      set_line_state.flash_count = 0;

      /* We parse the line number */
      if ((1 != sscanf(a->argv[2], " %10d ", &(tmp))) || (tmp <= 0) || (tmp > BCMPH_MAX_LINES)) {
         ast_cli(a->fd, "Invalid line '%s'\n", a->argv[2]);
         ret = CLI_FAILURE;
         break;
      }
      tmp -= 1;
      set_line_state.line = (size_t)(tmp);

      /* We parse the digits */
      f = a->argv[3];
      for (f = a->argv[3]; (('\0' != *f) && (set_line_state.digits_count < ARRAY_LEN(set_line_state.digits))); f += 1) {
         if (isspace(*f)) {
            continue;
         }
         if ((*f >= '0') && (*f <= '9')) {
            set_line_state.digits[set_line_state.digits_count] = *f;
            set_line_state.digits_count += 1;
            continue;
         }
         if (('*' == *f) || ('#' == *f)) {
            set_line_state.digits[set_line_state.digits_count] = *f;
            set_line_state.digits_count += 1;
            continue;
         }
         if (('A' == *f) || ('a' == *f)) {
            set_line_state.digits[set_line_state.digits_count] = 'A';
            set_line_state.digits_count += 1;
            continue;
         }
         if (('B' == *f) || ('b' == *f)) {
            set_line_state.digits[set_line_state.digits_count] = 'B';
            set_line_state.digits_count += 1;
            continue;
         }
         if (('C' == *f) || ('c' == *f)) {
            set_line_state.digits[set_line_state.digits_count] = 'C';
            set_line_state.digits_count += 1;
            continue;
         }
         if (('D' == *f) || ('d' == *f)) {
            set_line_state.digits[set_line_state.digits_count] = 'D';
            set_line_state.digits_count += 1;
            continue;
         }
         break;
      }
      if ('\0' != *f) {
         if (set_line_state.digits_count >= ARRAY_LEN(set_line_state.digits)) {
            ast_cli(a->fd, "Too many digits\n");
         }
         else {
            ast_cli(a->fd, "Invalid digit '%c'\n", (char)(*f));
         }
         ret = CLI_FAILURE;
         break;
      }

      /* We change the status of the line */
      if (bcmph_do_ioctl(t, BCMPH_IOCTL_SET_LINE_STATE, (unsigned long)(&(set_line_state)), true)) {
         ret = CLI_FAILURE;
         break;
      }
   } while (false);

   return (ret);
}

static struct ast_cli_entry cli_bcmph[] = {
   AST_CLI_DEFINE(bcmph_cli_set, "Change status of a line"),
   AST_CLI_DEFINE(bcmph_cli_dial, "Dial digits for a line"),
};
#endif /* BCMPH_NOHW */

static void bcmph_hangup_all_lines(bcmph_chan_t *t, bool monitor_is_really_stopped)
{
   bcmph_pvt_t *pvt;
   /* We hangup all lines if they have an owner */
   bcm_pr_debug("Hanging up all the lines\n");
   bcm_assert(!t->monitor.run);
   bcmph_monitor_lock(t);
   AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
      struct ast_channel *ast = pvt->owner;
      if (NULL != ast) {
         if (!monitor_is_really_stopped) {
            if (ast_channel_trylock(ast)) {
               continue;
            }
         }
         else {
            ast_channel_lock(ast);
         }
#ifdef BCMPH_DEBUG
         bcm_assert(0 == pvt->owner_lock_count);
         pvt->owner_lock_count += 1;
#endif /* BCMPH_DEBUG */
         bcmph_queue_hangup(pvt, BCMPH_EV_AST_HANGUP);
         /* ast_channel_unlock(ast) is done in bcmph_queue_hangup() */
         bcm_assert((NULL == pvt->owner) && (0 == pvt->owner_lock_count));
      }
   }
   bcmph_monitor_unlock(t);
}

static int __unload_module(void)
{
   int ret = 0;
   bcmph_chan_t *t = &(bcmph_chan);
   bcmph_pvt_t *p;
   size_t i;
#ifdef BCMPH_NOHW
   bool unregister_cli = t->channel_registered;
#endif

   do { /* Empty loop */
      /* Ask the monitor thread to stop */
      bcmph_prepare_stop_monitor(t);

      /* Take us out of the channel loop : no new ast_channel can be created for
      incoming calls (by bcmph_chan_request()) */
      bcm_pr_debug("Unregistering channel\n");
      if (t->channel_registered) {
         ast_channel_unregister(&(t->chan_tech));
         t->channel_registered = false;
      }

      /* Hangup all lines */
      bcmph_hangup_all_lines(t, false);

      /* We stop the monitor thread : no new ast_channel can be created
       for outgoing call because the user hooks off the phone */
      if (bcmph_stop_monitor(t)) {
         ast_log(AST_LOG_ERROR, "Unable to stop the monitor\n");
         ret = -1;
         break;
      }

      /* Once again hangup all lines that could have been created by monitor
       before it stops */
      bcmph_hangup_all_lines(t, true);

#ifdef BCMPH_NOHW
      if (unregister_cli) {
         ast_cli_unregister_multiple(cli_bcmph, ARRAY_LEN(cli_bcmph));
      }
#endif /* BCMPH_NOHW */

      /* We close the device */
      bcmph_close_device(t);

      /* We destroy all the interfaces and free their memory */
      bcm_pr_debug("Destroying all the lines\n");
      p = AST_LIST_FIRST(&(t->pvt_list));
      while (NULL != p) {
         bcmph_pvt_t *pl = p;
         p = AST_LIST_NEXT(p, list);
         bcm_phone_line_state_deinit(&(pl->monitor.line_state));
         if (NULL != pl->ast_channel.dsp) {
            ast_dsp_free(pl->ast_channel.dsp);
            pl->ast_channel.dsp = NULL;
         }
         ast_free(pl);
      }

      /* We free structures allocated for the configuration */
      for (i = 0; (i < ARRAY_LEN(t->config.line_cfgs)); i += 1) {
         if (NULL != t->config.line_cfgs[i].capabilities) {
            t->config.line_cfgs[i].capabilities = ast_format_cap_destroy(t->config.line_cfgs[i].capabilities);
         }
      }

      if (NULL != t->chan_tech.capabilities) {
         t->chan_tech.capabilities = ast_format_cap_destroy(t->chan_tech.capabilities);
      }

      ast_mutex_destroy(&(t->monitor.lock));
      ast_mutex_destroy(&(t->mmap.lock));

      ret = 0;
   } while (false);

   return (ret);
}

static int unload_module(void)
{
   return (__unload_module());
}

static bcmph_pvt_t *bcmph_add_pvt(bcmph_chan_t *t, size_t index_line)
{
   /* Make a phone_pvt structure for this interface */
   bcmph_pvt_t *tmp;

   bcm_assert(NULL != t);

   bcmph_monitor_lock(t);
   do { /* Empty loop */
      tmp = ast_calloc(1, sizeof(*tmp));
      if (NULL == tmp) {
         ast_log(AST_LOG_ERROR, "Unable to allocate memory for line\n");
         break;
      }

      memset(tmp, 0, sizeof(*tmp));

      tmp->channel = t;
      tmp->index_line = index_line;
      tmp->line_cfg = &(t->config.line_cfgs[index_line]);
      tmp->owner = NULL;
#ifdef BCMPH_DEBUG
      tmp->owner_lock_count = 0;
#endif /* BCMPH_DEBUG */
      bcm_phone_line_state_init(&(tmp->monitor.line_state));
      bcmph_reset_pvt_monitor_state(tmp);
      tmp->ast_channel.current_status = BCMPH_STATUS_DISCONNECTED;
      tmp->ast_channel.current_mode = BCMPH_MODE_IDLE;
      tmp->ast_channel.current_tone = BCMPH_TONE_NONE;
      if (DETECT_DTMF_NEVER != tmp->line_cfg->detect_dtmf) {
         tmp->ast_channel.dsp = ast_dsp_new();
         if (NULL != tmp->ast_channel.dsp) {
            ast_dsp_set_features(tmp->ast_channel.dsp, DSP_FEATURE_DIGIT_DETECT);
            ast_dsp_set_digitmode(tmp->ast_channel.dsp, DSP_DIGITMODE_DTMF | DSP_DIGITMODE_RELAXDTMF);
         }
         else {
            ast_log(AST_LOG_ERROR, "Error setting up dsp for DTMF detection\n");
            ast_free(tmp);
            tmp = NULL;
            break;
         }
      }
      tmp->ast_channel.state = BCMPH_ST_DISCONNECTED;
      /*
       Here it does not matter if line handles AST_FORMAT_ALAW or not, because
       the following fields are initialized in bcmph_set_line_codec() when we
       switch the line to conversation mode (state BCMPH_ST_OFF_TALKING)
       (bcmph_setup())
      */
      ast_format_set(&(tmp->ast_channel.current_format), AST_FORMAT_ALAW, 0);
      tmp->ast_channel.bytes_per_sample = 1;
      tmp->ast_channel.frame_size_read = (BCMPH_MAX_BUF / 2);
      tmp->ast_channel.bytes_not_written_len = 0;
      AST_LIST_INSERT_TAIL(&(t->pvt_list), tmp, list);
   } while (false);
   bcmph_monitor_unlock(t);

   return (tmp);
}

static int load_module(void)
{
   int ret = AST_MODULE_LOAD_SUCCESS;
   bcmph_chan_t *t = &(bcmph_chan);
   struct ast_config *cfg = CONFIG_STATUS_FILEINVALID;
   size_t i;

   memset(&(t->config), 0, sizeof(t->config));
   t->config.line_count = 0;
   t->channel_registered = false;
   t->pvt_list.first = NULL;
   t->pvt_list.last = NULL;
   t->dev_fd = -1;
   t->mmap.dev_start_addr = NULL;
   ast_mutex_init(&(t->mmap.lock));
#ifdef BCMPH_DEBUG
   t->mmap.lock_count = 0;
#endif /* BCMPH_DEBUG */
   t->monitor.run = false;
   t->monitor.thread = AST_PTHREADT_NULL;
   ast_mutex_init(&(t->monitor.lock));
#ifdef BCMPH_DEBUG
   t->monitor.lock_count = 0;
#endif /* BCMPH_DEBUG */
   t->monitor.line_off_hook_count = 0;
   t->chan_tech.capabilities = NULL;
   for (i = 0; (i < ARRAY_LEN(t->config.line_cfgs)); i += 1) {
      t->config.line_cfgs[i].capabilities = NULL;
      t->config.line_cfgs[i].detect_dtmf = DETECT_DTMF_NEVER;
   }

   do { /* Empty loop */
      struct ast_variable *v;
      struct ast_flags config_flags = { 0 };
      struct ast_format tmpfmt;
      size_t max_bytes_per_sample;

      t->chan_tech.capabilities = ast_format_cap_alloc();
      if (NULL == t->chan_tech.capabilities) {
         ret = AST_MODULE_LOAD_DECLINE;
         break;
      }
      ast_format_cap_add(t->chan_tech.capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_ULAW, 0));
      ast_format_cap_add(t->chan_tech.capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_ALAW, 0));
      ast_format_cap_add(t->chan_tech.capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0));
      ast_format_cap_add(t->chan_tech.capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR16, 0));

      bcm_pr_debug("Reading configuration file '%s'\n", bcmph_cfg_file);

      cfg = ast_config_load2(bcmph_cfg_file, bcmph_chan_type, config_flags);
      if (CONFIG_STATUS_FILEINVALID == cfg) {
         ast_log(AST_LOG_ERROR, "Config file '%s' is in an invalid format.  Aborting.\n", bcmph_cfg_file);
         ret = AST_MODULE_LOAD_DECLINE;
         break;
      }

      /* We *must* have a config file otherwise stop immediately */
      if (CONFIG_STATUS_FILEMISSING == cfg) {
         cfg = CONFIG_STATUS_FILEINVALID;
         ast_log(AST_LOG_ERROR, "Unable to load config file '%s'\n", bcmph_cfg_file);
         ret = AST_MODULE_LOAD_DECLINE;
         break;
      }

      for (v = ast_variable_browse(cfg, "interfaces"); (NULL != v); v = v->next) {
         if (!strcasecmp(v->name, "lines")) {
            int tmp;
            if ((1 != sscanf(v->value, " %10d ", &(tmp))) || (tmp <= 0) || (((size_t)(tmp)) > ARRAY_LEN(t->config.line_cfgs))) {
               ast_log(AST_LOG_ERROR, "Invalid value for variable 'lines' in section 'interfaces' of config file '%s'\n",
                  bcmph_cfg_file);
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }
            t->config.line_count = (size_t)(tmp);
         }
         else if (!strcasecmp(v->name, "country")) {
            char country[64];
            char *f;

            ast_copy_string(country, v->value, ARRAY_LEN(country));
            f = ast_strip(country);
            if (!strcasecmp(f, "etsi")) {
               t->config.language = "en_GB";
               t->config.country = BCMPH_COUNTRY_ETSI;
            }
            else if (!strcasecmp(f, "gr57")) {
               t->config.language = "en_GB";
               t->config.country = BCMPH_COUNTRY_GR57;
            }
            else if (!strcasecmp(f, "australia")) {
               t->config.language = "en_AU";
               t->config.country = BCMPH_COUNTRY_AU;
            }
            else if (!strcasecmp(f, "austria")) {
               t->config.language = "de_AT";
               t->config.country = BCMPH_COUNTRY_AT;
            }
            else if (!strcasecmp(f, "belgium")) {
               t->config.language = "fr_BE";
               t->config.country = BCMPH_COUNTRY_BE;
            }
            else if (!strcasecmp(f, "brazil")) {
               t->config.language = "pt_BR";
               t->config.country = BCMPH_COUNTRY_BR;
            }
            else if (!strcasecmp(f, "bulgaria")) {
               t->config.language = "bg_BG";
               t->config.country = BCMPH_COUNTRY_BG;
            }
            else if (!strcasecmp(f, "canada")) {
               t->config.language = "en_CA";
               t->config.country = BCMPH_COUNTRY_CA;
            }
            else if (!strcasecmp(f, "china")) {
               t->config.language = "zh_CN";
               t->config.country = BCMPH_COUNTRY_CN;
            }
            else if (!strcasecmp(f, "denmark")) {
               t->config.language = "da_DK";
               t->config.country = BCMPH_COUNTRY_DK;
            }
            else if (!strcasecmp(f, "finland")) {
               t->config.language = "fi_FI";
               t->config.country = BCMPH_COUNTRY_FI;
            }
            else if (!strcasecmp(f, "france")) {
               t->config.language = "fr_FR";
               t->config.country = BCMPH_COUNTRY_FR;
            }
            else if (!strcasecmp(f, "germany")) {
               t->config.language = "de_DE";
               t->config.country = BCMPH_COUNTRY_DE;
            }
            else if (!strcasecmp(f, "great_britain")) {
               t->config.language = "en_GB";
               t->config.country = BCMPH_COUNTRY_GB;
            }
            else if (!strcasecmp(f, "greece")) {
               t->config.language = "el_GR";
               t->config.country = BCMPH_COUNTRY_GR;
            }
            else if (!strcasecmp(f, "honk_kong")) {
               t->config.language = "zh_HK";
               t->config.country = BCMPH_COUNTRY_HK;
            }
            else if (!strcasecmp(f, "hungary")) {
               t->config.language = "hu_HU";
               t->config.country = BCMPH_COUNTRY_HU;
            }
            else if (!strcasecmp(f, "iceland")) {
               t->config.language = "is_IS";
               t->config.country = BCMPH_COUNTRY_IS;
            }
            else if (!strcasecmp(f, "ireland")) {
               t->config.language = "en_IE";
               t->config.country = BCMPH_COUNTRY_IE;
            }
            else if (!strcasecmp(f, "israel")) {
               t->config.language = "he_IL";
               t->config.country = BCMPH_COUNTRY_IL;
            }
            else if (!strcasecmp(f, "italy")) {
               t->config.language = "it_IT";
               t->config.country = BCMPH_COUNTRY_IT;
            }
            else if (!strcasecmp(f, "japan")) {
               t->config.language = "ja_JP";
               t->config.country = BCMPH_COUNTRY_JP;
            }
            else if (!strcasecmp(f, "korea")) {
               t->config.language = "ko_KR";
               t->config.country = BCMPH_COUNTRY_KR;
            }
            else if (!strcasecmp(f, "netherlands")) {
               t->config.language = "nl_NL";
               t->config.country = BCMPH_COUNTRY_NL;
            }
            else if (!strcasecmp(f, "new_zealand")) {
               t->config.language = "en_NZ";
               t->config.country = BCMPH_COUNTRY_NZ;
            }
            else if (!strcasecmp(f, "norway")) {
               t->config.language = "no_NO";
               t->config.country = BCMPH_COUNTRY_NO;
            }
            else if (!strcasecmp(f, "portugal")) {
               t->config.language = "pt_PT";
               t->config.country = BCMPH_COUNTRY_PT;
            }
            else if (!strcasecmp(f, "russia")) {
               t->config.language = "ru_RU";
               t->config.country = BCMPH_COUNTRY_RU;
            }
            else if (!strcasecmp(f, "singapore")) {
               t->config.language = "en_SG";
               t->config.country = BCMPH_COUNTRY_SG;
            }
            else if (!strcasecmp(f, "south_africa")) {
               t->config.language = "en_ZA";
               t->config.country = BCMPH_COUNTRY_ZA;
            }
            else if (!strcasecmp(f, "spain")) {
               t->config.language = "es_ES";
               t->config.country = BCMPH_COUNTRY_ES;
            }
            else if (!strcasecmp(f, "sweden")) {
               t->config.language = "sv_SE";
               t->config.country = BCMPH_COUNTRY_SE;
            }
            else if (!strcasecmp(f, "switzerland")) {
               t->config.language = "de_CH";
               t->config.country = BCMPH_COUNTRY_CH;
            }
            else if (!strcasecmp(f, "taiwan")) {
               t->config.language = "zh_TW";
               t->config.country = BCMPH_COUNTRY_TW;
            }
            else if (!strcasecmp(f, "turkey")) {
               t->config.language = "tr_TR";
               t->config.country = BCMPH_COUNTRY_TK;
            }
            else if (!strcasecmp(f, "united_states")) {
               t->config.language = "en_US";
               t->config.country = BCMPH_COUNTRY_US;
            }
            else {
               ast_log(AST_LOG_ERROR, "Invalid value for variable 'country' in section 'interfaces' of config file '%s'\n",
                  bcmph_cfg_file);
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }
         }
         else {
            ast_log(AST_LOG_WARNING, "Unknown variable '%s' in section 'interfaces' of config_file '%s'\n",
               v->name, bcmph_cfg_file);
         }
      }
      if (AST_MODULE_LOAD_SUCCESS != ret) {
         break;
      }

      /*
       Even if we use codec AST_FORMAT_SLINEAR and AST_FORMAT_SLINEAR16
       we always use the PCM in 8 bits mode because it's reduce the DMA
       transfers
      */
      max_bytes_per_sample = 1;
      for (i = 0; (i < t->config.line_count); i += 1) {
         char section[64];
         bcmph_line_config_t *line_cfg = &(t->config.line_cfgs[i]);

         snprintf(section, ARRAY_LEN(section), "line%d", (int)(i + 1));
         for (v = ast_variable_browse(cfg, section); (NULL != v); v = v->next) {
            if (!strcasecmp(v->name, "enable")) {
               if (ast_true(v->value)) {
                  line_cfg->enable = true;
               }
               else {
                  line_cfg->enable = false;
                  /* Stop parsing section */
                  break;
               }
            }
            else if (!strcasecmp(v->name, "codecs")) {
               /* List of codecs used */
               char codecs[256];
               char *f;
               char *save_ptr = NULL;

               if (NULL == line_cfg->capabilities) {
                  line_cfg->capabilities = ast_format_cap_alloc();
                  if (NULL == line_cfg->capabilities) {
                     ret = AST_MODULE_LOAD_DECLINE;
                     break;
                  }
               }
               ast_copy_string(codecs, v->value, ARRAY_LEN(codecs));
               f = strtok_r(codecs, ",", &(save_ptr));
               while (NULL != f)
               {
                  const char *format_name = ast_strip(f);
                  if (0 == strcasecmp(format_name, ast_getformatname(ast_format_set(&(tmpfmt), AST_FORMAT_ULAW, 0)))) {
                     ast_format_set(&(tmpfmt), AST_FORMAT_ULAW, 0);
                     ast_format_cap_add(line_cfg->capabilities, &(tmpfmt));
                  }
                  else if (0 == strcasecmp(format_name, ast_getformatname(ast_format_set(&(tmpfmt), AST_FORMAT_ALAW, 0)))) {
                     ast_format_set(&(tmpfmt), AST_FORMAT_ALAW, 0);
                     ast_format_cap_add(line_cfg->capabilities, &(tmpfmt));
                  }
                  else if (0 == strcasecmp(format_name, ast_getformatname(ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0)))) {
                     ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0);
                     ast_format_cap_add(line_cfg->capabilities, &(tmpfmt));
                     if (max_bytes_per_sample < 2) {
                        max_bytes_per_sample = 2;
                     }
                  }
                  else if (0 == strcasecmp(format_name, ast_getformatname(ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR16, 0)))) {
                     ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR16, 0);
                     ast_format_cap_add(line_cfg->capabilities, &(tmpfmt));
                     if (max_bytes_per_sample < 4) {
                        max_bytes_per_sample = 4;
                     }
                  }
                  else {
                     ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable 'codecs' in section '%s' of config file '%s'\n",
                        f, section, bcmph_cfg_file);
                     ret = AST_MODULE_LOAD_DECLINE;
                     break;
                  }
                  f = strtok_r(NULL, ",", &(save_ptr));
               }
               if (AST_MODULE_LOAD_SUCCESS != ret) {
                  break;
               }
            }
            else if (!strcasecmp(v->name, "context")) {
               ast_copy_string(line_cfg->context, v->value, ARRAY_LEN(line_cfg->context));
            }
            else if (!strcasecmp(v->name, "monitor_dialing")) {
               if (ast_true(v->value)) {
                  line_cfg->monitor_dialing = true;
               }
               else {
                  line_cfg->monitor_dialing = false;
               }
            }
            else if (!strcasecmp(v->name, "dialing_timeout")) {
               int tmp;
               if ((1 != sscanf(v->value, " %10d ", &(tmp))) || (tmp < 0)) {
                  ast_log(AST_LOG_ERROR, "Invalid value for variable 'dialing_timeout' in section '%s' of config file '%s'\n",
                     section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               line_cfg->dialing_timeout = tmp;
            }
            else if (!strcasecmp(v->name, "detect_dtmf")) {
               char value[32];
               char *f;

               ast_copy_string(value, v->value, ARRAY_LEN(value));
               f = ast_strip(value);
               if (0 == strcasecmp(f, "always")) {
                  line_cfg->detect_dtmf = DETECT_DTMF_ALWAYS;
               }
               else if (0 == strcasecmp(f, "dialing")) {
                  line_cfg->detect_dtmf = DETECT_DTMF_WHEN_DIALING;
               }
               else if (0 == strcasecmp(f, "connected")) {
                  line_cfg->detect_dtmf = DETECT_DTMF_WHEN_CONNECTED;
               }
               else if (0 == strcasecmp(f, "never")) {
                  line_cfg->detect_dtmf = DETECT_DTMF_NEVER;
               }
               else {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable 'detect_dtmf' in section '%s' of config file '%s'\n",
                     v->value, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
            }
            else if (!strcasecmp(v->name, "caller_id")) {
               ast_callerid_split(v->value, line_cfg->cid_name,
                  ARRAY_LEN(line_cfg->cid_name),
                  line_cfg->cid_num, ARRAY_LEN(line_cfg->cid_num));
            }
            else if (!strcasecmp(v->name, "moh_interpret")) {
               ast_copy_string(line_cfg->moh_interpret, v->value, ARRAY_LEN(line_cfg->moh_interpret));
            }
            else {
               ast_log(AST_LOG_WARNING, "Unknown variable '%s' in section '%s' of config file '%s'\n",
                  v->name, section, bcmph_cfg_file);
            }
         }
         if (AST_MODULE_LOAD_SUCCESS != ret) {
            break;
         }
         if (line_cfg->enable) {
            if (NULL == line_cfg->capabilities) {
               ast_log(AST_LOG_ERROR, "No codec specified for line %lu in config file '%s'\n",
                  (unsigned long)(i + 1), bcmph_cfg_file);
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }
            if (DETECT_DTMF_NEVER != line_cfg->detect_dtmf) {
               if ((!ast_format_cap_iscompatible(line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_SLINEAR, 0)))
                   && (!ast_format_cap_iscompatible(line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_ALAW, 0)))
                   && (!ast_format_cap_iscompatible(line_cfg->capabilities, ast_format_set(&(tmpfmt), AST_FORMAT_ULAW, 0)))) {
                  ast_log(AST_LOG_ERROR, "Line %lu does not support codec alaw, ulaw or slin, so DTMF detection is not possible\n",
                     (unsigned long)(i + 1));
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               if ((DETECT_DTMF_ALWAYS == line_cfg->detect_dtmf)
                   || (DETECT_DTMF_WHEN_CONNECTED == line_cfg->detect_dtmf)) {
                  ast_format_cap_iter_start(line_cfg->capabilities);
                  while (!ast_format_cap_iter_next(line_cfg->capabilities, &(tmpfmt))) {
                     if ((AST_FORMAT_SLINEAR != tmpfmt.id)
                         && (AST_FORMAT_ALAW != tmpfmt.id)
                         && (AST_FORMAT_ULAW != tmpfmt.id)) {
                        ast_log(AST_LOG_ERROR, "Line %lu supports codec '%s', but it's not compatible with DTMF detection\n",
                           (unsigned long)(i + 1), ast_getformatname(&(tmpfmt)));
                        ret = AST_MODULE_LOAD_DECLINE;
                        break;
                     }
                  }
                  ast_format_cap_iter_end(line_cfg->capabilities);
                  if (AST_MODULE_LOAD_SUCCESS != ret) {
                     break;
                  }
               }
            }
            if (!line_cfg->context[0]) {
               ast_copy_string(line_cfg->context, "default", sizeof(line_cfg->context));
            }
            if (NULL == bcmph_add_pvt(t, i)) {
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }
         }
      }
      if (AST_MODULE_LOAD_SUCCESS != ret) {
         break;
      }

      /*
       When at least one line is in conversation mode, as monitor thread is
       in charge of queuing data received, we set a period equal to the time
       needed to fill a frame (considering that we receive 8 samples per ms)
      */
      if (max_bytes_per_sample > 2) {
         t->config.monitor_busy_period = (BCMPH_MAX_BUF / (max_bytes_per_sample * 8));
      }
      else {
         t->config.monitor_busy_period = (BCMPH_MAX_BUF / (2 * 8));
      }

      if (AST_LIST_EMPTY(&(t->pvt_list))) {
         ast_log(AST_LOG_ERROR, "No active line in config file '%s'\n", bcmph_cfg_file);
         ret = AST_MODULE_LOAD_DECLINE;
         break;
      }

      ast_config_destroy(cfg);
      cfg = CONFIG_STATUS_FILEINVALID;

      ret = bcmph_open_device(t);
      if (AST_MODULE_LOAD_SUCCESS != ret) {
         break;
      }

      bcm_pr_debug("Registering channel\n");

      /*
       Now that all internal structures are initialized we can register
       the channel and start the monitor
      */
      if (ast_channel_register(&(t->chan_tech))) {
         ast_log(AST_LOG_ERROR, "Unable to register channel class '%s'\n", bcmph_chan_desc);
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }
#ifdef BCMPH_NOHW
      ast_log(AST_LOG_VERBOSE, "Registering commands\n");

      if (ast_cli_register_multiple(cli_bcmph, ARRAY_LEN(cli_bcmph))) {
         ast_log(AST_LOG_ERROR, "Unable to register commands\n");
         ast_channel_unregister(&(t->chan_tech));
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }
#endif /* BCMPH_NOHW */
      t->channel_registered = true;

      if (bcmph_start_monitor(t)) {
         ret = AST_MODULE_LOAD_FAILURE;
         break;
      }

      ret = AST_MODULE_LOAD_SUCCESS;
   }
   while (false);

   if (CONFIG_STATUS_FILEINVALID != cfg) {
      ast_config_destroy(cfg);
   }

   if (AST_MODULE_LOAD_SUCCESS != ret) {
      __unload_module();
   }

   return (ret);
}

AST_MODULE_INFO_STANDARD(ASTERISK_GPL_KEY, "Broadcom 63xx Phone Support");
