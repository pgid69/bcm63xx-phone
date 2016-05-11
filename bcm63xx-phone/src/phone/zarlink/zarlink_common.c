/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifdef __KERNEL__
#include <linux/delay.h>
#else // !__KERNEL__
#include <stdlib.h>
#endif // !__KERNEL__

#include <vp_api_int.h>
#include "zarlink_common.h"

#include <board.h>
#include <bcm63xx_log.h>
#include <utils.h>

// Include after system files
#include <compile.h>

/* Constants declarations */
/*
 Time in msecs we wait for zarlink events related to
 initialization and calibration
*/
#define INIT_TIMEOUT 5000

/* Period in msecs between 2 calls to phone_dev_zarlink_update_line() */
#define LINE_UPDATE_PERIOD 79

#ifdef BCMPH_VP_DECODE_PULSE
static char zarlink_convert_digit_type_to_char(VpDigitType digit)
{
   char ret = 0;

   dd_bcm_pr_debug("zarlink_convert_digit_type_to_char(digit=%d)\n", (int)(digit));
   if (VP_DIG_NONE != digit) {
      switch (digit) {
         case VP_DIG_ZERO: {
            ret = '0';
            break;
         }
         case VP_DIG_A: {
            ret = 'A';
            break;
         }
         case VP_DIG_B: {
            ret = 'B';
            break;
         }
         case VP_DIG_C: {
            ret = 'C';
            break;
         }
         case VP_DIG_D: {
            ret = 'D';
            break;
         }
         case VP_DIG_ASTER: {
            ret = '*';
            break;
         }
         case VP_DIG_POUND: {
            ret = '#';
            break;
         }
         default: {
            ret = (char)(digit + 48);
            break;
         }
      }
   }

   return (ret);
}
#endif // BCMPH_VP_DECODE_PULSE

static VpDigitType zarlink_convert_tone_to_digit_type(bcm_phone_line_tone_t tone)
{
   VpDigitType ret = VP_DIG_NONE;

   dd_bcm_pr_debug("zarlink_convert_tone_to_digit_type(tone=%d)\n", (int)(tone));
   switch (tone) {
      case BCMPH_TONE_DTMF_0: {
         ret = VP_DIG_ZERO;
         break;
      }
      case BCMPH_TONE_DTMF_1: {
         ret = VP_DIG_1;
         break;
      }
      case BCMPH_TONE_DTMF_2: {
         ret = VP_DIG_2;
         break;
      }
      case BCMPH_TONE_DTMF_3: {
         ret = VP_DIG_3;
         break;
      }
      case BCMPH_TONE_DTMF_4: {
         ret = VP_DIG_4;
         break;
      }
      case BCMPH_TONE_DTMF_5: {
         ret = VP_DIG_5;
         break;
      }
      case BCMPH_TONE_DTMF_6: {
         ret = VP_DIG_6;
         break;
      }
      case BCMPH_TONE_DTMF_7: {
         ret = VP_DIG_7;
         break;
      }
      case BCMPH_TONE_DTMF_8: {
         ret = VP_DIG_8;
         break;
      }
      case BCMPH_TONE_DTMF_9: {
         ret = VP_DIG_9;
         break;
      }
      case BCMPH_TONE_DTMF_ASTER: {
         ret = VP_DIG_ASTER;
         break;
      }
      case BCMPH_TONE_DTMF_POUND: {
         ret = VP_DIG_POUND;
         break;
      }
      case BCMPH_TONE_DTMF_A: {
         ret = VP_DIG_A;
         break;
      }
      case BCMPH_TONE_DTMF_B: {
         ret = VP_DIG_B;
         break;
      }
      case BCMPH_TONE_DTMF_C: {
         ret = VP_DIG_C;
         break;
      }
      case BCMPH_TONE_DTMF_D: {
         ret = VP_DIG_D;
         break;
      }
      default: {
         bcm_assert(false);
         break;
      }
   }

   return (ret);
}

static void zarlink_reset(uint16 gpio_num)
{
   bcm_pr_debug("zarlink_reset(gpio_num=0x%x)\n", (unsigned int)(gpio_num));
   if (GPIO_IS_INVALID != gpio_num) {
      board_set_gpio(gpio_num, 0);
      msleep(10);
      board_set_gpio(gpio_num, 1);
      msleep(10);
      board_set_gpio(gpio_num, 0);
      msleep(5);
      if ((gpio_num & GPIO_IS_ACTIVE_LOW)) {
         d_bcm_pr_debug("Reset applied on GPIO pin %d (active low) \n", (int)(gpio_num & (~(GPIO_IS_ACTIVE_LOW))));
      }
      else {
         d_bcm_pr_debug("Reset applied on GPIO pin %d\n", (int)(gpio_num));
      }
   }
}

static void zarlink_shutdown(uint16 gpio_num)
{
   bcm_pr_debug("zarlink_shutdown(gpio_num=0x%x)\n", (unsigned int)(gpio_num));
   if (GPIO_IS_INVALID != gpio_num) {
      board_set_gpio(gpio_num, 1);
   }
}

static VpOptionCodecType zarlink_get_codec_type(bcm_phone_codec_t codec)
{
   VpOptionCodecType codec_type = VP_OPTION_LINEAR;

   d_bcm_pr_debug("zarlink_get_codec_type(codec=%d)\n", (int)(codec));
   switch (codec)
   {
      case BCMPH_CODEC_ALAW: {
         codec_type = VP_OPTION_ALAW;
         break;
      }
      case BCMPH_CODEC_ULAW: {
         codec_type = VP_OPTION_MLAW;
         break;
      }
      case BCMPH_CODEC_LINEAR: {
         codec_type = VP_OPTION_LINEAR;
         break;
      }
      case BCMPH_CODEC_LINEAR16: {
         codec_type = VP_OPTION_LINEAR_WIDEBAND;
         break;
      }
      case BCMPH_CODEC_ALAW16: {
         codec_type = VP_OPTION_ALAW_WIDEBAND;
         break;
      }
      case BCMPH_CODEC_ULAW16: {
         codec_type = VP_OPTION_MLAW_WIDEBAND;
         break;
      }
      default: {
         bcm_pr_err("PCM mode not specified !!! \n");
         bcm_assert(0);
         break;
      }
   }

   return (codec_type);
}

#ifdef BCMPH_DEBUG
static void zarlink_mpi_pr_regs(bcm_mpi_t *mpi,
   const zarlink_cmd_desc_t *cmds, size_t cmd_len, int line_id)
{
#ifdef __KERNEL__
   size_t i;
   size_t j;
   size_t k;
   int l;
   __u8 cmd[4];
   __u8 data[32];
   char message[256];

   d_bcm_pr_debug("zarlink_mpi_pr_regs()\n");
   bcm_assert((NULL != mpi) && (NULL != cmds) && (cmd_len > 0));
   i = 0;
   if (line_id >= 0) {
      cmd[i] = 0x4A;
      i += 1;
      cmd[i] = (__u8)(line_id);
      i += 1;
   }
   for (k = 0; (k < cmd_len); k += 1) {
      j = i;
      cmd[j] = cmds[k].cmd;
      j += 1;
      bcm_assert(cmds[k].data_len <= ARRAY_SIZE(data));
      bcm_mpi_read(mpi, data, cmds[k].data_len, cmd, j);
      l = snprintf(message, ARRAY_SIZE(message), KERN_INFO "0x%02X -> [ 0x%02X", (unsigned int)(cmds[k].cmd), (unsigned int)(data[0]));
      for (j = 1; (j < cmds[k].data_len); j += 1) {
         l += snprintf(&(message[l]), ARRAY_SIZE(message) - l, ", 0x%02X", (unsigned int)(data[j]));
      }
      l += snprintf(&(message[l]), ARRAY_SIZE(message) - l, " ]\n");
      printk(message);
   }
#endif // !__KERNEL__
}
#endif // BCMPH_DEBUG

static VpStatusType zarlink_set_timeslot_and_codec(VpLineCtxType *line_ctx, VpOptionCodecType codec_type, uint8 ts)
{
   VpStatusType status;
   VpOptionTimeslotType timeslot;

   bcm_pr_debug("zarlink_set_timeslot_and_codec(codec_type=%u, ts=%d)\n", (unsigned int)(codec_type), (int)(ts));
   bcm_assert(NULL != line_ctx);
   timeslot.tx = ts;
   timeslot.rx = ts;
   /* Don't give device context (just give NULL) if you are configuring any parameters for a specific line */
   status = VpSetOption(line_ctx, VP_NULL, VP_OPTION_ID_TIMESLOT, (void *)(&(timeslot)));
   if (VP_STATUS_SUCCESS != status)
   {
      bcm_pr_err("VpSetOption() for timeslot failed (%d) \n", status);
   }
   else
   {
      status = VpSetOption(line_ctx, VP_NULL, VP_OPTION_ID_CODEC, (void *)(&(codec_type)));
      if (VP_STATUS_SUCCESS != status)
      {
          bcm_pr_err("VpSetOption() for codec type failed (%d)\n", status);
      }
   }
   return (status);
}

/*
*****************************************************************************
** FUNCTION:   zarlink_wait_for_event
**
** PURPOSE:    Waiting for specific event category and event Id on device.
**
** PARAMETERS: deviceNum - Device number of SLIC
**             category  - Event Category to wait for
**             event     - Event Id to wait for
**
** RETURNS:    TRUE  - Event occurred.
**             FALSE - Event did not occur.
**
*****************************************************************************
*/
static bool zarlink_wait_for_event(VpDevCtxType *dev_ctx, __u8 tick_period,
   VpEventCategoryType evt_category, uint16 evt_id)
{
   uint16 i;
   VpStatusType status;
   bool event_pending = FALSE;

   bcm_pr_debug("zarlink_wait_for_event(evt_category=%d, evt_id=%u)\n",
      (int)(evt_category), (unsigned int)(evt_id));
   bcm_assert(NULL != dev_ctx);
   i = 0;
   do {
      status = VpApiTick(dev_ctx, &(event_pending));
      if ((VP_STATUS_SUCCESS == status) && (TRUE == event_pending))
      {
         VpEventType event;
         while (VpGetEvent(dev_ctx, &(event)))
         {
            switch (event.eventCategory)
            {
               /* Add more categories as needed */
               case VP_EVCAT_RESPONSE:
                  switch(event.eventId)
                  {
                     /* Add more events as needed */
                     case VP_DEV_EVID_DEV_INIT_CMP:
                        bcm_pr_debug("Received VP_DEV_EVID_DEV_INIT_CMP event (i = %d)\n", i);
                        break;
                     case VP_LINE_EVID_LINE_INIT_CMP:
                        bcm_pr_debug("Received VP_LINE_EVID_LINE_INIT_CMP event (i = %d)\n", i);
                        break;
                     case VP_EVID_CAL_CMP:
                        bcm_pr_debug("Received VP_EVID_CAL_CMP event (i = %d)\n", i);
                        break;
                     default:
                        /* Do nothing */
                        bcm_pr_err("ERROR Unexpected Event %d came from the SLIC.\n", event.eventId);
                        break;
                  }
                  break;
               default:
                  break;
            }
            /* As soon as it finds the event we are looking for, return */
            if ((event.eventCategory == evt_category) && (event.eventId == evt_id))
            {
               bcm_pr_debug("Found event: pEvent.eventCategory = %d pEvent.eventId = %d \n", event.eventCategory, event.eventId);
               return (TRUE);
            }
            else
            {
               bcm_pr_err("Unexpected event: pEvent.eventCategory = %d pEvent.eventId = %d \n", event.eventCategory, event.eventId);
            }
         } /* while */
      } /* if */
      msleep(tick_period);
      i += tick_period;
   } while (i < INIT_TIMEOUT);

   /* Could not find any events if we reach here */
   return (FALSE);
}

static inline const VpProfileDataType *zarlink_profiles_get_dev(const zarlink_profiles_t *t)
{
   d_bcm_pr_debug("zarlink_profiles_get_dev()\n");
   bcm_assert((NULL != t->dev_profile) && (VP_PRFWZ_PROFILE_DEVICE == t->dev_profile[VP_PROFILE_TYPE_LSB]));
   return (t->dev_profile);
}

static inline const VpProfileDataType *zarlink_profiles_get_default_AC(const zarlink_profiles_t *t,
   VpOptionCodecType codec)
{
   d_bcm_pr_debug("zarlink_profiles_get_default_AC(codec=%d)\n", (int)(codec));
   if (VP_OPTION_WIDEBAND != codec) {
      bcm_assert((NULL != t->default_AC_profile_NB) && (VP_PRFWZ_PROFILE_AC == t->default_AC_profile_NB[VP_PROFILE_TYPE_LSB]));
      return (t->default_AC_profile_NB);
   }
   else {
      bcm_assert((NULL != t->default_AC_profile_WB) && (VP_PRFWZ_PROFILE_AC == t->default_AC_profile_WB[VP_PROFILE_TYPE_LSB]));
      return (t->default_AC_profile_WB);
   }
}

static inline const VpProfileDataType *zarlink_profiles_get_AC(const zarlink_profiles_t *t,
   bcmph_country_t country, VpOptionCodecType codec)
{
   const VpProfileDataType *ret;

   d_bcm_pr_debug("zarlink_profiles_get_AC(country=%d, codec=%d)\n",
      (int)(country), (int)(codec));
   bcm_assert(/* (country >= 0) && */(country < BCMPH_COUNTRY_MAX));
   if (VP_OPTION_WIDEBAND != codec) {
      ret = t->AC_profiles_NB[country];
   }
   else {
      ret = t->AC_profiles_WB[country];
   }
   if ((NULL == ret) || (VP_PRFWZ_PROFILE_AC != ret[VP_PROFILE_TYPE_LSB])) {
      ret = zarlink_profiles_get_default_AC(t, codec);
   }
   return (ret);
}

static inline const VpProfileDataType *zarlink_profiles_get_default_DC(const zarlink_profiles_t *t)
{
   d_bcm_pr_debug("zarlink_profiles_get_default_DC()\n");
   bcm_assert((NULL != t->default_DC_profile) && (VP_PRFWZ_PROFILE_DC == t->default_DC_profile[VP_PROFILE_TYPE_LSB]));
   return (t->default_DC_profile);
}

static inline const VpProfileDataType *zarlink_profiles_get_DC(const zarlink_profiles_t *t,
   bcmph_country_t country)
{
   const VpProfileDataType *ret;

   d_bcm_pr_debug("zarlink_profiles_get_DC(country=%d)\n", (int)(country));
   bcm_assert(/* (country >= 0) && */(country < BCMPH_COUNTRY_MAX));
   ret = t->DC_profiles[country];
   if ((NULL == ret) || (VP_PRFWZ_PROFILE_DC != ret[VP_PROFILE_TYPE_LSB])) {
      ret = zarlink_profiles_get_default_DC(t);
   }
   return (ret);
}

static inline const VpProfileDataType *zarlink_profiles_get_default_ring(const zarlink_profiles_t *t)
{
   d_bcm_pr_debug("zarlink_profiles_get_default_ring()\n");
   bcm_assert((NULL != t->default_ring_profile) && (VP_PRFWZ_PROFILE_RING == t->default_ring_profile[VP_PROFILE_TYPE_LSB]));
   return (t->default_ring_profile);
}

static inline const VpProfileDataType *zarlink_profiles_get_ring(const zarlink_profiles_t *t,
   bcmph_country_t country)
{
   const VpProfileDataType *ret;

   d_bcm_pr_debug("zarlink_profiles_get_ring(country=%d)\n", (int)(country));
   bcm_assert(/* (country >= 0) && */(country < BCMPH_COUNTRY_MAX));
   ret = t->ring_profiles[country];
   if ((NULL == ret) || (VP_PRFWZ_PROFILE_RING != ret[VP_PROFILE_TYPE_LSB])) {
      ret = zarlink_profiles_get_default_ring(t);
   }
   return (ret);
}

static inline void zarlink_device_id_init(zarlink_device_id_t *t, bcm_mpi_t *mpi)
{
   bcm_pr_debug("zarlink_device_id_init()\n");
   bcm_assert(NULL != mpi);
   t->mpi = mpi;
}

static inline void zarlink_device_id_deinit(zarlink_device_id_t *t)
{
   bcm_pr_debug("zarlink_device_id_deinit()\n");
}

static VpStatusType zarlink_dev_make(void *dev_obj, VpDevCtxType *dev_ctx,
   VpDeviceType device_type, VpDeviceIdType device_id)
{
   VpStatusType status;

   bcm_pr_debug("zarlink_dev_make(deviceType=%d)\n",
      (int)(device_type));
   bcm_assert((NULL != dev_obj) && (NULL != dev_ctx));
   do // Empty loop
   {
      status = VpMakeDeviceObject(device_type, device_id, dev_ctx, dev_obj);
      if (VP_STATUS_SUCCESS != status)
      {
         bcm_pr_err("VpMakeDeviceObject() failed (%d)\n", status);
         break;
      }
   } while (0);

   return (status);
}

static void zarlink_dev_free(VpDevCtxType *dev_ctx)
{
   bcm_pr_debug("zarlink_dev_free()\n");
   bcm_assert(NULL != dev_ctx);
}

static VpStatusType zarlink_dev_init(VpDevCtxType *dev_ctx, __u8 tick_period,
   const zarlink_profiles_t *profiles, bcmph_country_t country,
   bcm_phone_codec_t codec)
{
   VpStatusType status;

   bcm_pr_debug("zarlink_dev_init(country=%d)\n", (int)(country));
   bcm_assert((NULL != dev_ctx) && (NULL != profiles));
   do // Empty loop
   {
      VpOptionCodecType codec_type = zarlink_get_codec_type(codec);

      status = VpInitDevice(dev_ctx,
         zarlink_profiles_get_dev(profiles),
         zarlink_profiles_get_AC(profiles, country, codec_type),
         zarlink_profiles_get_DC(profiles, country),
         zarlink_profiles_get_ring(profiles, country),
         VP_PTABLE_NULL, VP_PTABLE_NULL);
      if (status != VP_STATUS_SUCCESS)
      {
         bcm_pr_err("VpInitDevice() failed (%d)\n", status);
         break;
      }

      if (TRUE != zarlink_wait_for_event(dev_ctx, tick_period, VP_EVCAT_RESPONSE, VP_DEV_EVID_DEV_INIT_CMP))
      {
         bcm_pr_err("ERROR: Device initialization was not completed\n");
         status = VP_STATUS_FAILURE;
         break;
      }
   } while (0);

   return (status);
}

static void zarlink_dev_deinit(VpDevCtxType *dev_ctx)
{
   bcm_pr_debug("zarlink_dev_deinit()\n");
   bcm_assert(NULL != dev_ctx);
}

static VpStatusType zarlink_slic_line_make(VpDevCtxType *dev_ctx,
   void *line_obj, VpLineCtxType *line_ctx, VpTermType term_type,
   uint8 internal_channel_id, uint8 external_channel_id)
{
   VpStatusType status;

   bcm_pr_debug("zarlink_slic_line_make(term_type=%d, internal_channel_id=%u, external_channel_id=%u)\n",
      (int)(term_type),
      (unsigned int)(internal_channel_id), (unsigned int)(external_channel_id));
   bcm_assert((NULL != dev_ctx) && (NULL != line_obj) && (NULL != line_ctx)
      && ((VP_TERM_FXS_GENERIC == term_type) || (VP_TERM_FXS_LOW_PWR == term_type)));

   /* Create the FXS line */
   status = VpMakeLineObject(term_type, internal_channel_id,
      line_ctx, line_obj, dev_ctx);
   if (VP_STATUS_SUCCESS == status)
   {
      status = VpMapLineId(line_ctx, external_channel_id);
      if (VP_STATUS_SUCCESS != status)
      {
         VpFreeLineCtx(line_ctx);
         bcm_pr_err("VpMapLineId() failed (%d)\n", status);
      }
   }
   else
   {
      bcm_pr_err("VpMakeLineObject() failed (%d)\n", status);
   }

   return (status);
}

static void zarlink_slic_line_free(VpLineCtxType *line_ctx)
{
   bcm_pr_debug("zarlink_slic_line_free()\n");
   bcm_assert(NULL != line_ctx);
   VpSetLineTone(line_ctx, VP_PTABLE_NULL, VP_PTABLE_NULL, VP_NULL);
   VpSetLineState(line_ctx, VP_LINE_DISCONNECT);
   VpFreeLineCtx(line_ctx);
}

static VpStatusType zarlink_slic_line_init(
   VpDevCtxType *dev_ctx, __u8 tick_period,
   VpLineCtxType *line_ctx, VpLineStateType initial_state,
   const zarlink_profiles_t *profiles, bcmph_country_t country,
   bcm_phone_codec_t codec, uint8 timeslot)
{
   VpStatusType status;

   bcm_pr_debug("zarlink_slic_line_init(country=%d, codec=%d, timeslot=%u)\n",
      (int)(country), (int)(codec), (unsigned int)(timeslot));
   bcm_assert((NULL != dev_ctx) && (NULL != line_ctx) && (NULL != profiles));
   do // Empty loop
   {
      VpOptionEventMaskType event_mask;
      VpOptionCodecType codec_type;

      /* Initialize the line with proper profile settings */
      codec_type = zarlink_get_codec_type(codec);
      status = VpInitLine(line_ctx,
         zarlink_profiles_get_AC(profiles, country, codec_type),
         zarlink_profiles_get_DC(profiles, country),
         zarlink_profiles_get_ring(profiles, country));
      if (VP_STATUS_SUCCESS != status)
      {
         bcm_pr_err("VpInitLine() failed (%d)\n", status);
         break;
      }

      /* Check if VP_LINE_EVID_LINE_INIT_CMP occurred */
      if (TRUE != zarlink_wait_for_event(dev_ctx, tick_period, VP_EVCAT_RESPONSE, VP_LINE_EVID_LINE_INIT_CMP)) {
         bcm_pr_err("SLIC: ERROR: Line initialization was not completed\n");
         status = VP_STATUS_FAILURE;
         break;
      }

      /* Set the initial line state */
      status = VpSetLineState(line_ctx, initial_state);
      if (VP_STATUS_SUCCESS != status) {
         bcm_pr_err("VpSetLineState() failed (%d)\n", status);
         break;
      }

      /* Unmask events related to calibration */
      memset(&(event_mask), 0xFF, sizeof(event_mask));
      event_mask.response = (~(VP_EVID_CAL_CMP | VP_EVID_CAL_BUSY));
      status = VpSetOption(line_ctx, NULL, VP_OPTION_ID_EVENT_MASK,
         &(event_mask));
      if (VP_STATUS_SUCCESS != status) {
         bcm_pr_err("VpSetOption() failed to set event mask (%d)\n", status);
         break;
      }

      /* Calibrate the line */
      status = VpCalLine(line_ctx);
      if ((VP_STATUS_SUCCESS != status) && (VP_STATUS_FUNC_NOT_SUPPORTED != status)) {
         bcm_pr_err("VpCalLine() failed (%d)\n", status);
         break;
      }
      else if (VP_STATUS_SUCCESS == status) {
         /* Check if VP_EVID_CAL_CMP occurred */
         if (TRUE != zarlink_wait_for_event(dev_ctx, tick_period, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP)) {
            bcm_pr_err("SLIC: ERROR: Line calibration was not completed\n");
            status = VP_STATUS_FAILURE;
            break;
         }
      }

      bcm_assert(/*(timeslot >= 0) && */(timeslot <= 128));
      status = zarlink_set_timeslot_and_codec(line_ctx, codec_type, timeslot);
   }
   while (0);

   if (VP_STATUS_SUCCESS == status) {
      bcm_pr_debug("FXS line initialized OK\n");
   }
   else {
      VpSetLineState(line_ctx, VP_LINE_DISCONNECT);
   }

   return (status);
}

static void zarlink_slic_line_deinit(VpLineCtxType *line_ctx)
{
   bcm_pr_debug("zarlink_slic_line_deinit()\n");
   bcm_assert(NULL != line_ctx);
   VpSetLineTone(line_ctx, VP_PTABLE_NULL, VP_PTABLE_NULL, VP_NULL);
   VpSetLineState(line_ctx, VP_LINE_DISCONNECT);
}

void __init phone_dev_zarlink_init(phone_dev_zarlink_t *t,
   const vtbl_phone_dev_zarlink_t *vtbl,
   const phone_desc_device_t *dev_desc, __u8 tick_period,
   void *dev_obj, bcm_mpi_t *mpi)
{
   size_t i;

   bcm_pr_debug("phone_dev_zarlink_init()\n");
   bcm_assert(ARRAY_SIZE(t->lines) <= ARRAY_SIZE(t->vd.lines));
   bcm_assert((NULL != vtbl) && (NULL != dev_desc) && (NULL != dev_obj)
      && (NULL != mpi));
   if ((dev_desc->caps & BCMPH_CAPS_REQUIRES_RESET)) {
      zarlink_reset(dev_desc->reset_gpio);
   }
   phone_device_init(&(t->vd), &(vtbl->vd), dev_desc, tick_period);
   t->vtbl = vtbl;
   zarlink_device_id_init(&(t->device_id), mpi);
   t->dev.obj = dev_obj;
   memset(&(t->dev.ctx), 0, sizeof(t->dev.ctx));
   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
      t->lines[i].obj = NULL;
      t->lines[i].ctx = NULL;
   }
   t->line_update_timer = 0;
}

void phone_dev_zarlink_deinit(phone_dev_zarlink_t *t)
{
   size_t i;
   const phone_desc_device_t *desc = phone_device_get_desc(&(t->vd));
   __u16 reset_gpio = GPIO_IS_INVALID;

   bcm_pr_debug("phone_dev_zarlink_deinit()\n");
   if ((desc->caps & BCMPH_CAPS_REQUIRES_RESET)) {
      reset_gpio = desc->reset_gpio;
   }
   t->dev.obj = NULL;
   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
      t->lines[i].obj = NULL;
      t->lines[i].ctx = NULL;
   }
   t->dev.obj = NULL;
   zarlink_device_id_deinit(&(t->device_id));
   phone_device_deinit(&(t->vd));
   if (GPIO_IS_INVALID != reset_gpio) {
      zarlink_reset(reset_gpio);
   }
}

static VpStatusType phone_dev_zarlink_init_vp_objects(phone_dev_zarlink_t *t,
   bcmph_country_t country)
{
   VpStatusType status;
   size_t i;
#ifndef BCMPH_TEST_PCM
   __u8 tick_period = phone_device_get_tick_period(&(t->vd));
#endif // BCMPH_TEST_PCM
   const phone_desc_device_t *desc = phone_device_get_desc(&(t->vd));
   const phone_line_t *first_line = NULL;

   bcm_pr_debug("phone_dev_zarlink_init_vp_objects(country=%d)\n",
      (int)(country));
   bcm_assert((NULL != phone_dev_zarlink_get_dev_obj(t))
      && (NULL != phone_dev_zarlink_get_device_id(t)->mpi));

   status = zarlink_dev_make(phone_dev_zarlink_get_dev_obj(t),
      phone_dev_zarlink_get_dev_ctx(t),
      desc->parameters.zarlink->type,
      phone_dev_zarlink_get_device_id(t));
   if (VP_STATUS_SUCCESS != status)
   {
      goto failMakeDev;
   }

   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1)
   {
      const phone_line_t *vl = phone_device_get_line(&(t->vd), i);
      if (NULL != vl) {
         bcm_assert((NULL != phone_dev_zarlink_get_line_obj(t, i))
            && (NULL != phone_dev_zarlink_get_line_ctx(t, i)));
         status = zarlink_slic_line_make(phone_dev_zarlink_get_dev_ctx(t),
            phone_dev_zarlink_get_line_obj(t, i),
            phone_dev_zarlink_get_line_ctx(t, i),
            desc->lines[i].parameters.zarlink->type,
            desc->lines[i].parameters.zarlink->id, i);
         if (VP_STATUS_SUCCESS != status)
         {
            while (i > 0)
            {
               i -= 1;
               vl = phone_device_get_line(&(t->vd), i);
               if (NULL != vl) {
                  zarlink_slic_line_free(phone_dev_zarlink_get_line_ctx(t, i));
               }
            }
            goto failMakeLine;
         }
         if ((phone_line_is_enabled(vl)) && (NULL == first_line)) {
            first_line = vl;
         }
      }
   }

#ifndef BCMPH_TEST_PCM
   status = zarlink_dev_init(phone_dev_zarlink_get_dev_ctx(t), tick_period,
      &(desc->parameters.zarlink->profiles), country,
      ((NULL != first_line) ? first_line->line_state.codec : BCMPH_CODEC_ALAW));
   if (VP_STATUS_SUCCESS != status)
   {
      goto failInitDev;
   }

   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1)
   {
      const phone_line_t *vl = phone_device_get_line(&(t->vd), i);
      if ((NULL != vl) && (phone_line_is_enabled(vl))) {
         status = zarlink_slic_line_init(phone_dev_zarlink_get_dev_ctx(t),
            tick_period, phone_dev_zarlink_get_line_ctx(t, i),
            desc->parameters.zarlink->modes.on_hook_idle,
            &(desc->parameters.zarlink->profiles), country,
            vl->line_state.codec, vl->timeslot);
         if (VP_STATUS_SUCCESS != status)
         {
            while (i > 0)
            {
               i -= 1;
               vl = phone_device_get_line(&(t->vd), i);
               if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                  zarlink_slic_line_deinit(phone_dev_zarlink_get_line_ctx(t, i));
               }
            }
            goto failInitLine;
         }
      }
   }
#endif // BCMPH_TEST_PCM

   return (status);

#ifndef BCMPH_TEST_PCM
   i = ARRAY_SIZE(t->lines);
   do
   {
      const phone_line_t *vl;
      i -= 1;
      vl = phone_device_get_line(&(t->vd), i);
      if ((NULL != vl) && (phone_line_is_enabled(vl))) {
         zarlink_slic_line_deinit(phone_dev_zarlink_get_line_ctx(t, i));
      }
   } while (i > 0);
failInitLine:
   zarlink_dev_deinit(phone_dev_zarlink_get_dev_ctx(t));
failInitDev:
#endif // BCMPH_TEST_PCM
   i = ARRAY_SIZE(t->lines);
   do
   {
      const phone_line_t *vl;
      i -= 1;
      vl = phone_device_get_line(&(t->vd), i);
      if (NULL != vl) {
         zarlink_slic_line_free(phone_dev_zarlink_get_line_ctx(t, i));
      }
   } while (i > 0);
failMakeLine:
   zarlink_dev_free(phone_dev_zarlink_get_dev_ctx(t));
failMakeDev:
   return(status);
}

/*
*****************************************************************************
*/
static void phone_dev_zarlink_deinit_vp_objects(phone_dev_zarlink_t *t)
{
   size_t i;

   bcm_pr_debug("phone_dev_zarlink_deinit_vp_objects()\n");

#ifndef BCMPH_TEST_PCM
   i = ARRAY_SIZE(t->lines);
   do
   {
      const phone_line_t *vl;
      i -= 1;
      vl = phone_device_get_line(&(t->vd), i);
      if ((NULL != vl) && (phone_line_is_enabled(vl))) {
         zarlink_slic_line_deinit(phone_dev_zarlink_get_line_ctx(t, i));
      }
   } while (i > 0);

   zarlink_dev_deinit(phone_dev_zarlink_get_dev_ctx(t));
#endif // BCMPH_TEST_PCM

   i = ARRAY_SIZE(t->lines);
   do
   {
      const phone_line_t *vl;
      i -= 1;
      vl = phone_device_get_line(&(t->vd), i);
      if (NULL != vl) {
         zarlink_slic_line_free(phone_dev_zarlink_get_line_ctx(t, i));
      }
   } while (i > 0);

   zarlink_dev_free(phone_dev_zarlink_get_dev_ctx(t));
}

static void phone_dev_zarlink_stop(phone_dev_zarlink_t *t)
{
   bcm_pr_debug("phone_dev_zarlink_stop()\n");

   if (phone_device_is_started(&(t->vd))) {
      size_t i;

      phone_dev_zarlink_deinit_vp_objects(t);
      for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
         phone_line_t *vl = phone_device_get_line(&(t->vd), i);
         if ((NULL != vl) && (phone_line_is_enabled(vl))) {
            phone_line_disable(vl);
         }
      }
      phone_device_stop(&(t->vd));
   }
}

static int phone_dev_zarlink_start(phone_dev_zarlink_t *t,
   bcmph_country_t country,
   const phone_line_params_t * const *line_params, size_t line_count)
{
   int ret = 0;
   size_t i;
   VpStatusType status;

   bcm_pr_debug("phone_dev_zarlink_start(country=%d)\n", (int)(country));
   bcm_assert((NULL != line_params) && (line_count <= ARRAY_SIZE(t->lines)));

   phone_dev_zarlink_stop(t);
   for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
      if ((i < line_count) && (NULL != line_params[i]) && (line_params[i]->enable)) {
         phone_line_t *vl = phone_device_get_line(&(t->vd), i);
         bcm_assert((NULL != vl) && (NULL != phone_dev_zarlink_get_line_ctx(t, i)));
         phone_line_enable(vl, line_params[i]->codec, line_params[i]->first_timeslot);
      }
      else {
         phone_line_t *vl = phone_device_get_line(&(t->vd), i);
         if (NULL != vl) {
            phone_line_disable(vl);
         }
      }
   }

   status = phone_dev_zarlink_init_vp_objects(t, country);
   if (VP_STATUS_SUCCESS == status) {
#ifndef BCMPH_TEST_PCM
#ifdef BCMPH_VP_DECODE_PULSE
      VpOptionPulseModeType pulse_mode;
#endif // BCMPH_VP_DECODE_PULSE
      VpOptionEventMaskType event_mask;
      bool hook_status;

      memset(&(event_mask), 0xFF, sizeof(event_mask));
      event_mask.faults = (~(VP_DEV_EVID_CLK_FLT | VP_DEV_EVID_BAT_FLT
         | VP_LINE_EVID_THERM_FLT | VP_LINE_EVID_DC_FLT | VP_LINE_EVID_AC_FLT));
      event_mask.signaling = (~(VP_LINE_EVID_HOOK_OFF | VP_LINE_EVID_HOOK_ON));
#ifdef BCMPH_VP_DECODE_PULSE
      event_mask.signaling &= (~(VP_LINE_EVID_PULSE_DIG
         | VP_LINE_EVID_FLASH | VP_LINE_EVID_EXTD_FLASH));
      pulse_mode = VP_OPTION_PULSE_DECODE_ON;
#endif // BCMPH_VP_DECODE_PULSE
      for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
         phone_line_t *vl = phone_device_get_line(&(t->vd), i);
         if ((NULL != vl) && (phone_line_is_enabled(vl))) {
            status = VpSetOption(phone_dev_zarlink_get_line_ctx(t, i),
               NULL, VP_OPTION_ID_EVENT_MASK, &(event_mask));
            if (VP_STATUS_SUCCESS != status) {
               bcm_pr_err("Failed to set event mask (%d)\n", status);
               break;
            }
#ifdef BCMPH_VP_DECODE_PULSE
            status = VpSetOption(phone_dev_zarlink_get_line_ctx(t, i),
               NULL, VP_OPTION_ID_PULSE_MODE, &(pulse_mode));
            if (VP_STATUS_SUCCESS != status) {
               bcm_pr_err("Failed to set pulse mode (%d)\n", status);
               break;
            }
#endif // BCMPH_VP_DECODE_PULSE
            status = VpGetLineStatus(phone_dev_zarlink_get_line_ctx(t, i),
               VP_INPUT_HOOK, &(hook_status));
            if (VP_STATUS_SUCCESS != status) {
               bcm_pr_err("Failed to get hook status (%d)\n", status);
               break;
            }
            if (hook_status) {
               bcm_pr_debug("Line %lu is off hook\n", (unsigned long)(i));
               vl->line_state.status = BCMPH_STATUS_OFF_HOOK;
            }
            else {
               bcm_pr_debug("Line %lu is on hook\n", (unsigned long)(i));
               vl->line_state.status = BCMPH_STATUS_ON_HOOK;
            }
         }
         t->lines[i].tone_cadencer.off_time = 0;
         t->lines[i].mode_cadencer.off_time = 0;
      }
      if (i < ARRAY_SIZE(t->lines)) {
         ret = -1;
         for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
            phone_line_t *vl = phone_device_get_line(&(t->vd), i);
            if ((NULL != vl) && (phone_line_is_enabled(vl))) {
               phone_line_disable(vl);
            }
         }
         phone_dev_zarlink_deinit_vp_objects(t);
      }
      else {
#endif // !BCMPH_TEST_PCM
         phone_device_start(&(t->vd), country);
         t->line_update_timer = phone_device_get_tick_period(&(t->vd));
#ifndef BCMPH_TEST_PCM
      }
#endif // !BCMPH_TEST_PCM
   }
   else {
      bcm_pr_err("Failed to configure zarlink device\n");
      ret = -1;
      for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
         phone_line_t *vl = phone_device_get_line(&(t->vd), i);
         if ((NULL != vl) && (phone_line_is_enabled(vl))) {
            phone_line_disable(vl);
         }
      }
   }

   return (ret);
}

static inline bool phone_dev_zarlink_is_mode_valid(
   bcm_phone_line_status_t status, bcm_phone_line_mode_t mode)
{
   bool ret = true;
   dd_bcm_pr_debug("phone_dev_zarlink_is_mode_valid(status=%d, mode=%d)\n",
      (int)(status), (int)(mode));
   if (BCMPH_MODE_IDLE != mode) {
      if (BCMPH_STATUS_DISCONNECTED == status) {
         ret = false;
      }
      else if (BCMPH_STATUS_ON_HOOK == status) {
         if (BCMPH_MODE_ON_RINGING != mode) {
            ret = false;
         }
      }
      else {
         bcm_assert(BCMPH_STATUS_OFF_HOOK == status);
         if (BCMPH_MODE_ON_RINGING == mode) {
            ret = false;
         }
      }
   }
   return (ret);
}

static inline bool phone_dev_zarlink_is_tone_valid(
   bcm_phone_line_status_t status, bcm_phone_line_mode_t mode,
   bcm_phone_line_tone_t tone_index)
{
   bool ret = true;
   dd_bcm_pr_debug("phone_dev_zarlink_is_tone_valid(status=%d, mode=%d, tone=%d)\n",
      (int)(status), (int)(mode), (int)(tone));
   if (BCMPH_TONE_NONE != tone_index) {
      if (BCMPH_STATUS_OFF_HOOK != status) {
         ret = false;
      }
      else if ((BCMPH_MODE_IDLE != mode) && (BCMPH_MODE_OFF_TALKING != mode)) {
         ret = false;
      }
   }
   return (ret);
}

static inline VpStatusType phone_dev_zarlink_set_tone_on(
   phone_dev_zarlink_t *t, size_t line,
   const VpProfileDataType *z_profile, VpDigitType z_tone_id,
   __u16 time)
{
   VpStatusType ret;

   d_bcm_pr_debug("phone_dev_zarlink_set_tone_on(line=%lu, z_profile=0x%lx, z_tone_id=%d, time=%u)\n",
      (unsigned long)(line), (unsigned long)(z_profile), (int)(z_tone_id),
      (unsigned)(time));
   bcm_assert(line < ARRAY_SIZE(t->lines));
#ifndef BCMPH_TEST_PCM
   if (NULL != z_profile) {
      ret = VpSetLineTone(t->lines[line].ctx, z_profile, VP_PTABLE_NULL, VP_NULL);
   }
   else {
      VpDtmfToneGenType dtmf_control;
      dtmf_control.toneId = z_tone_id;
      dtmf_control.dir = VP_DIRECTION_DS;
      ret = VpSetLineTone(t->lines[line].ctx, VP_PTABLE_NULL, VP_PTABLE_NULL, &(dtmf_control));
   }
#else // BCMPH_TEST_PCM
   ret = VP_STATUS_SUCCESS;
#endif // BCMPH_TEST_PCM
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].tone_cadencer.timer = time;
      t->lines[line].tone_cadencer.is_on = true;
   }
   else {
      bcm_pr_err("VpSetLineTone() failed (%d)\n", ret);
   }
   return (ret);
}

static inline VpStatusType phone_dev_zarlink_set_tone_off(
   phone_dev_zarlink_t *t, size_t line, __u16 time)
{
   VpStatusType ret;

   d_bcm_pr_debug("phone_dev_zarlink_set_tone_off(line=%lu, time=%u)\n",
      (unsigned long)(line), (unsigned)(time));
   bcm_assert(line < ARRAY_SIZE(t->lines));
#ifndef BCMPH_TEST_PCM
   ret = VpSetLineTone(t->lines[line].ctx, VP_PTABLE_NULL, VP_PTABLE_NULL, VP_NULL);
#else // BCMPH_TEST_PCM
   ret = VP_STATUS_SUCCESS;
#endif // BCMPH_TEST_PCM
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].tone_cadencer.timer = time;
      t->lines[line].tone_cadencer.is_on = false;
   }
   else {
      bcm_pr_err("VpSetLineTone() failed (%d)\n", ret);
   }
   return (ret);
}

static VpStatusType phone_dev_zarlink_tone_start(phone_dev_zarlink_t *t,
   size_t line, __u32 tone)
{
   VpStatusType ret;
   const phone_desc_device_t *phone_desc = phone_device_get_desc(&(t->vd));
   const zarlink_tone_t *z_tone = NULL;
   const VpProfileDataType *z_profile = NULL;
   VpDigitType z_tone_id = VP_DIG_NONE;
   __u8 tick_period = phone_device_get_tick_period(&(t->vd));
   bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);
   __u16 on_time = bcm_phone_line_tone_decode_on_time(tone);
   __u16 off_time = bcm_phone_line_tone_decode_off_time(tone);

   d_bcm_pr_debug("phone_dev_zarlink_tone_start(line=%lu, tone=0x%lx)\n",
      (unsigned long)(line), (unsigned long)(tone));

   switch (tone_index) {
      case BCMPH_TONE_WAITING_DIAL: {
         z_tone = &(phone_desc->parameters.zarlink->tones.waiting_dial);
         break;
      }
      case BCMPH_TONE_RINGBACK: {
         z_tone = &(phone_desc->parameters.zarlink->tones.ringback);
         break;
      }
      case BCMPH_TONE_INVALID: {
         z_tone = &(phone_desc->parameters.zarlink->tones.invalid);
         break;
      }
      case BCMPH_TONE_BUSY: {
         z_tone = &(phone_desc->parameters.zarlink->tones.busy);
         break;
      }
      case BCMPH_TONE_DISCONNECT: {
         z_tone = &(phone_desc->parameters.zarlink->tones.disconnect);
         break;
      }
      case BCMPH_TONE_DTMF_0:
      case BCMPH_TONE_DTMF_1:
      case BCMPH_TONE_DTMF_2:
      case BCMPH_TONE_DTMF_3:
      case BCMPH_TONE_DTMF_4:
      case BCMPH_TONE_DTMF_5:
      case BCMPH_TONE_DTMF_6:
      case BCMPH_TONE_DTMF_7:
      case BCMPH_TONE_DTMF_8:
      case BCMPH_TONE_DTMF_9:
      case BCMPH_TONE_DTMF_ASTER:
      case BCMPH_TONE_DTMF_POUND:
      case BCMPH_TONE_DTMF_A:
      case BCMPH_TONE_DTMF_B:
      case BCMPH_TONE_DTMF_C:
      case BCMPH_TONE_DTMF_D: {
         z_tone_id = zarlink_convert_tone_to_digit_type(tone_index);
         break;
      }
      default: {
         if (BCMPH_TONE_NONE != tone_index) {
            bcm_pr_debug("Unknown tone %lu\n", (unsigned long)(tone_index));
         }
         break;
      }
   }
   bcm_assert(line < ARRAY_SIZE(t->lines));
   if (NULL != z_tone) {
      bcm_assert(VP_DIG_NONE == z_tone_id);
      z_profile = z_tone->profile;
      if (NULL != z_profile) {
         if ((on_time <= 0) && (off_time <= 0)) {
            // We use default values
            on_time = z_tone->on_time;
            off_time = z_tone->off_time;
            bcm_assert((on_time > 0) || (off_time > 0));
         }
      }
      else {
         // We stop playing any tone
         bcm_pr_debug("No profile for tone %d\n", (int)(bcm_phone_line_tone_decode_index(tone)));
         on_time = 0;
         off_time = 0;
      }
   }
   else if (VP_DIG_NONE != z_tone_id) {
      bcm_assert(NULL == z_tone);
      if ((on_time <= 0) && (off_time <= 0)) {
         // We use default values : we play a DTMF for 100 msecs one time
         on_time = 0;
         off_time = 100;
      }
   }
   else {
      // We stop playing any tone
      on_time = 0;
      off_time = 0;
   }
   on_time = round_up_generic(on_time, tick_period);
   off_time = round_up_generic(off_time, tick_period);
   if (on_time > 0) {
      // Continuous (if off_time == 0) or repeating tone
      ret = phone_dev_zarlink_set_tone_on(t, line, z_profile, z_tone_id, on_time);
   }
   else if (off_time > 0) {
      // One shot tone
      ret = phone_dev_zarlink_set_tone_on(t, line, z_profile, z_tone_id, off_time);
   }
   else {
      // No tone
      ret = phone_dev_zarlink_set_tone_off(t, line, 0);
   }
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].tone_cadencer.tone = tone_index;
      t->lines[line].tone_cadencer.z_profile = z_profile;
      t->lines[line].tone_cadencer.z_tone_id = z_tone_id;
      t->lines[line].tone_cadencer.on_time = on_time;
      t->lines[line].tone_cadencer.off_time = off_time;
   }
   return (ret);
}

static VpStatusType phone_dev_zarlink_tone_stop(phone_dev_zarlink_t *t, size_t line)
{
   VpStatusType ret;
   d_bcm_pr_debug("phone_dev_zarlink_tone_stop(line=%lu)\n",
      (unsigned long)(line));
   ret = phone_dev_zarlink_set_tone_off(t, line, 0);
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].tone_cadencer.off_time = 0;
   }
   return (ret);
}

static inline VpStatusType phone_dev_zarlink_set_mode_on(
   phone_dev_zarlink_t *t, size_t line, VpLineStateType mode_on,
   __u16 time)
{
   VpStatusType ret;

   d_bcm_pr_debug("phone_dev_zarlink_set_mode_on(line=%lu, mode_on=%d, time=%u)\n",
      (unsigned long)(line), (int)(mode_on), (unsigned)(time));
   bcm_assert(line < ARRAY_SIZE(t->lines));
#ifndef BCMPH_TEST_PCM
   ret = VpSetLineState(t->lines[line].ctx, mode_on);
#else // BCMPH_TEST_PCM
   ret = VP_STATUS_SUCCESS;
#endif // BCMPH_TEST_PCM
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].mode_cadencer.timer = time;
      t->lines[line].mode_cadencer.is_on = true;
   }
   else {
      bcm_pr_err("VpSetLineState() failed (%d)\n", ret);
   }
   return (ret);
}

static inline VpStatusType phone_dev_zarlink_set_mode_off(
   phone_dev_zarlink_t *t, size_t line, VpLineStateType mode_off,
   __u16 time)
{
   VpStatusType ret;

   d_bcm_pr_debug("phone_dev_zarlink_set_mode_off(line=%lu, mode_off=%d, time=%u)\n",
      (unsigned long)(line), (int)(mode_off), (unsigned)(time));
   bcm_assert(line < ARRAY_SIZE(t->lines));
#ifndef BCMPH_TEST_PCM
   ret = VpSetLineState(t->lines[line].ctx, mode_off);
#else // BCMPH_TEST_PCM
   ret = VP_STATUS_SUCCESS;
#endif // BCMPH_TEST_PCM
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].mode_cadencer.timer = time;
      t->lines[line].mode_cadencer.is_on = false;
   }
   else {
      bcm_pr_err("VpSetLineState() failed (%d)\n", ret);
   }
   return (ret);
}

static VpStatusType phone_dev_zarlink_mode_start(phone_dev_zarlink_t *t,
   size_t line, bcm_phone_line_mode_t mode)
{
   VpStatusType ret;
   bool is_disconnected = false;
   const phone_desc_device_t *phone_desc = phone_device_get_desc(&(t->vd));
   bcm_phone_line_mode_t mode_off = mode;
   VpLineStateType z_mode_on;
   VpLineStateType z_mode_off;
   __u16 on_time = 0x8000;
   __u16 off_time = 0;

   d_bcm_pr_debug("phone_dev_zarlink_mode_start(line=%lu, mode=%d)\n",
      (unsigned long)(line), (int)(mode));
   bcm_assert(line < ARRAY_SIZE(t->lines));

   switch (mode) {
      case BCMPH_MODE_ON_RINGING: {
         z_mode_on = phone_desc->parameters.zarlink->modes.on_hook_ringing;
         z_mode_off = phone_desc->parameters.zarlink->modes.on_hook_idle;
         mode_off = BCMPH_MODE_IDLE;
         on_time = phone_desc->parameters.zarlink->modes.ring_cadence.on_time;
         off_time = phone_desc->parameters.zarlink->modes.ring_cadence.off_time;
         bcm_assert((on_time > 0) || (off_time > 0));
         break;
      }
      case BCMPH_MODE_OFF_TALKING: {
         z_mode_on = phone_desc->parameters.zarlink->modes.off_hook_talking;
         z_mode_off = z_mode_on;
         break;
      }
      default: {
         phone_line_t *vl = phone_device_get_line(&(t->vd), line);
         bcm_phone_line_state_t *ls = &(vl->line_state);
         if (BCMPH_MODE_IDLE != mode) {
            bcm_pr_debug("Unknown mode %d\n", (int)(mode));
         }
         // If disconnected do not change the line state
         if (BCMPH_STATUS_DISCONNECTED != ls->status) {
            if (BCMPH_STATUS_OFF_HOOK == ls->status) {
               z_mode_on = phone_desc->parameters.zarlink->modes.off_hook_idle;
            }
            else {
               z_mode_on = phone_desc->parameters.zarlink->modes.on_hook_idle;
            }
            z_mode_off = z_mode_on;
         }
         else {
            is_disconnected = true;
         }
         break;
      }
   }

   if (is_disconnected) {
      ret = VP_STATUS_SUCCESS;
   }
   else {
      if (z_mode_on == z_mode_off) {
         // Same mode, so we switch to mode_off and disable cadencing
         on_time = 0;
         off_time = 0;
      }
      else {
         // We convert msecs to tick counts
         __u8 tick_period = phone_device_get_tick_period(&(t->vd));
         on_time = round_up_generic(on_time, tick_period);
         off_time = round_up_generic(off_time, tick_period);
      }
      if (on_time > 0) {
         // Continuous (if off_time == 0) or repeating mode
         ret = phone_dev_zarlink_set_mode_on(t, line, z_mode_on, on_time);
      }
      else if (off_time > 0) {
         // One shot mode
         ret = phone_dev_zarlink_set_mode_on(t, line, z_mode_on, off_time);
      }
      else {
         // No mode
         ret = phone_dev_zarlink_set_mode_off(t, line, z_mode_off, 0);
      }
      if (VP_STATUS_SUCCESS == ret) {
         t->lines[line].mode_cadencer.mode_on = mode;
         t->lines[line].mode_cadencer.mode_off = mode_off;
         t->lines[line].mode_cadencer.z_mode_on = z_mode_on;
         t->lines[line].mode_cadencer.z_mode_off = z_mode_off;
         t->lines[line].mode_cadencer.on_time = on_time;
         t->lines[line].mode_cadencer.off_time = off_time;
      }
   }
   return (ret);
}

static VpStatusType phone_dev_zarlink_mode_stop(phone_dev_zarlink_t *t,
   size_t line)
{
   VpStatusType ret;
   d_bcm_pr_debug("phone_dev_zarlink_mode_stop(line=%lu)\n",
      (unsigned long)(line));
   ret = phone_dev_zarlink_set_mode_off(t, line,
      t->lines[line].mode_cadencer.z_mode_off, 0);
   if (VP_STATUS_SUCCESS == ret) {
      t->lines[line].mode_cadencer.off_time = 0;
   }
   return (ret);
}

static void phone_dev_zarlink_update_line(phone_dev_zarlink_t *t, size_t line)
{
   phone_line_t *vl = phone_device_get_line(&(t->vd), line);
   bcm_phone_line_state_t *ls;
   bcm_phone_line_tone_t new_tone_index;
   VpStatusType status;

   dd_bcm_pr_debug("phone_dev_zarlink_update_line(line=%lu)\n",
      (unsigned long)(line));
   bcm_assert((NULL != vl) && (phone_line_is_enabled(vl)));

   ls = &(vl->line_state);
   if (vl->new_codec != ls->codec) {
#ifndef BCMPH_TEST_PCM
      VpOptionCodecType codec_type = zarlink_get_codec_type(vl->new_codec);
      VpLineCtxType *line_ctx = phone_dev_zarlink_get_line_ctx(t, line);
      bcm_assert(NULL != line_ctx);
#endif // !BCMPH_TEST_PCM
      d_bcm_pr_debug("Changing to codec %d\n", (int)(vl->new_codec));
#ifndef BCMPH_TEST_PCM
      status = VpSetOption(line_ctx, VP_NULL, VP_OPTION_ID_CODEC, (void *)(&(codec_type)));
      if (VP_STATUS_SUCCESS != status) {
         bcm_pr_err("VpSetOption() for codec type failed (%d)\n", status);
         // Abort codec change
         vl->new_codec = ls->codec;
      }
      else {
#endif // !BCMPH_TEST_PCM
         ls->codec = vl->new_codec;
         ls->codec_change_count += 1;
         phone_line_line_state_changed(vl);
#ifndef BCMPH_TEST_PCM
      }
#endif // !BCMPH_TEST_PCM
   }
   if (!phone_dev_zarlink_is_mode_valid(ls->status, vl->new_mode)) {
      bcm_pr_debug("Mode %d is invalid with status %d. Mode changed to %d\n",
         (int)(vl->new_mode), (int)(ls->status), (int)(BCMPH_MODE_IDLE));
      // Force new_mode to BCMPH_MODE_IDLE
      vl->new_mode = BCMPH_MODE_IDLE;
   }
   if ((vl->new_mode != ls->mode) && (BCMPH_TONE_NONE != ls->tone)) {
      // Stop emission of tone before changing mode
      status = phone_dev_zarlink_tone_stop(t, line);
      if (VP_STATUS_SUCCESS != status) {
         bcm_pr_err("Failed to disable line tone (%d)\n", status);
         // Abort mode change
         vl->new_mode = ls->mode;
      }
      else {
         ls->tone = BCMPH_TONE_NONE;
         ls->tone_change_count += 1;
         phone_line_line_state_changed(vl);
      }
   }
   if (vl->new_mode != ls->mode) {
      d_bcm_pr_debug("Changing to mode %d\n", (int)(vl->new_mode));
      status = phone_dev_zarlink_mode_start(t, line, vl->new_mode);
      if (VP_STATUS_SUCCESS != status) {
         bcm_pr_err("Failed to set line mode (%d)\n", status);
         // Abort mode change
         vl->new_mode = ls->mode;
      }
      else {
         ls->mode = vl->new_mode;
         ls->mode_change_count += 1;
         phone_line_line_state_changed(vl);
      }
   }
   new_tone_index = bcm_phone_line_tone_decode_index(vl->new_tone);
   if (!phone_dev_zarlink_is_tone_valid(ls->status, ls->mode, new_tone_index)) {
      // Force new_tone to BCMPH_TONE_NONE
      new_tone_index = BCMPH_TONE_NONE;
      bcm_pr_debug("Tone 0x%lx is invalid with mode %d. Tone changed to 0x%lx\n",
         (unsigned long)(vl->new_tone), (int)(vl->new_mode),
         (unsigned long)(bcm_phone_line_tone_code_index(new_tone_index)));
      vl->new_tone = bcm_phone_line_tone_code_index(new_tone_index);
   }
   if (new_tone_index != ls->tone) {
      d_bcm_pr_debug("Changing to tone 0x%lx\n", (unsigned long)(vl->new_tone));
      status = phone_dev_zarlink_tone_start(t, line, vl->new_tone);
      if (VP_STATUS_SUCCESS != status) {
         bcm_pr_err("Failed to set line tone (%d)\n", status);
         // Abort tone change
         vl->new_tone = bcm_phone_line_tone_code_index(ls->tone);
      }
      else {
         ls->tone = new_tone_index;
         ls->tone_change_count += 1;
         phone_line_line_state_changed(vl);
      }
   }
}

static inline void phone_dev_zarlink_update_tone(phone_dev_zarlink_t *t,
   size_t line, bcm_phone_line_tone_t initial_tone,
   bcm_phone_line_tone_t actual_tone)
{
   phone_line_t *vl = phone_device_get_line(&(t->vd), line);
   bcm_phone_line_state_t *ls = &(vl->line_state);

   d_bcm_pr_debug("phone_dev_zarlink_update_tone(line=%lu, initial_tone=%d, actual_tone=%d)\n",
      (unsigned long)(line), (int)(initial_tone), (int)(actual_tone));
   /*
    We check before changing new_tone that it has the value asked in
    phone_dev_zarlink_tone_start() else it means that the user asked another
    tone
   */
   if (initial_tone == bcm_phone_line_tone_decode_index(vl->new_tone)) {
      vl->new_tone = bcm_phone_line_tone_code_index(actual_tone);
   }
   if (actual_tone != ls->tone) {
      ls->tone = actual_tone;
      ls->tone_change_count += 1;
      phone_line_line_state_changed(vl);
   }
}

static void phone_dev_zarlink_tone_tick(phone_dev_zarlink_t *t,
   size_t line)
{
   __u8 tick_period = phone_device_get_tick_period(&(t->vd));

   d_bcm_pr_debug("phone_dev_zarlink_tone_tick(line=%lu)\n",
      (unsigned long)(line));
   bcm_assert((line < ARRAY_SIZE(t->lines))
      && ((NULL != t->lines[line].tone_cadencer.z_profile)
          || (VP_DIG_NONE != t->lines[line].tone_cadencer.z_tone_id))
      && (t->lines[line].tone_cadencer.off_time > 0));

   if (t->lines[line].tone_cadencer.timer < tick_period) {
      VpStatusType ret;
      if (t->lines[line].tone_cadencer.is_on) {
         if (t->lines[line].tone_cadencer.on_time <= 0) {
            // One shot tone
            ret = phone_dev_zarlink_tone_stop(t, line);
            t->lines[line].tone_cadencer.off_time = 0;
            if (VP_STATUS_SUCCESS != ret) {
               phone_dev_zarlink_update_tone(t, line,
                  t->lines[line].tone_cadencer.tone,
                  t->lines[line].tone_cadencer.tone);
            }
            else {
               phone_dev_zarlink_update_tone(t, line,
                  t->lines[line].tone_cadencer.tone,
                  BCMPH_TONE_NONE);
            }
         }
         else {
            // Repeating tone
            ret = phone_dev_zarlink_set_tone_off(t, line,
               t->lines[line].tone_cadencer.off_time);
            if (VP_STATUS_SUCCESS != ret) {
               t->lines[line].tone_cadencer.off_time = 0;
               phone_dev_zarlink_update_tone(t, line,
                  t->lines[line].tone_cadencer.tone,
                  t->lines[line].tone_cadencer.tone);
            }
         }
      }
      else {
         // Repeating tone
         ret = phone_dev_zarlink_set_tone_on(t, line,
            t->lines[line].tone_cadencer.z_profile,
            t->lines[line].tone_cadencer.z_tone_id,
            t->lines[line].tone_cadencer.on_time);
         if (VP_STATUS_SUCCESS != ret) {
            t->lines[line].tone_cadencer.off_time = 0;
            phone_dev_zarlink_update_tone(t, line,
               t->lines[line].tone_cadencer.tone,
               BCMPH_TONE_NONE);
         }
      }
   }
   else {
      t->lines[line].tone_cadencer.timer -= tick_period;
   }
}

static inline void phone_dev_zarlink_update_mode(phone_dev_zarlink_t *t,
   size_t line, bcm_phone_line_mode_t initial_mode,
   bcm_phone_line_mode_t actual_mode)
{
   phone_line_t *vl = phone_device_get_line(&(t->vd), line);
   bcm_phone_line_state_t *ls = &(vl->line_state);

   d_bcm_pr_debug("phone_dev_zarlink_update_tone(line=%lu, initial_mode=%d, actual_mode=%d)\n",
      (unsigned long)(line), (int)(initial_mode), (int)(actual_mode));
   /*
    We check before changing new_mode that it has the value asked in
    phone_dev_zarlink_mode_start() else it means that the user asked another
    mode
   */
   if (initial_mode == vl->new_mode) {
      vl->new_mode = actual_mode;
   }
   if (actual_mode != ls->mode) {
      ls->mode = actual_mode;
      ls->mode_change_count += 1;
      phone_line_line_state_changed(vl);
   }
}

static void phone_dev_zarlink_mode_tick(phone_dev_zarlink_t *t,
   size_t line)
{
   __u8 tick_period = phone_device_get_tick_period(&(t->vd));

   d_bcm_pr_debug("phone_dev_zarlink_ring_tick(line=%lu)\n",
      (unsigned long)(line));
   bcm_assert((line < ARRAY_SIZE(t->lines))
      && (t->lines[line].mode_cadencer.off_time > 0));

   if (t->lines[line].mode_cadencer.timer < tick_period) {
      VpStatusType ret;
      if (t->lines[line].mode_cadencer.is_on) {
         if (t->lines[line].mode_cadencer.on_time <= 0) {
            // One shot mode
            ret = phone_dev_zarlink_mode_stop(t, line);
            t->lines[line].mode_cadencer.off_time = 0;
            if (VP_STATUS_SUCCESS != ret) {
               phone_dev_zarlink_update_mode(t, line,
                  t->lines[line].mode_cadencer.mode_on,
                  t->lines[line].mode_cadencer.mode_on);
            }
            else {
               phone_dev_zarlink_update_mode(t, line,
                  t->lines[line].mode_cadencer.mode_on,
                  t->lines[line].mode_cadencer.mode_off);
            }
         }
         else {
            // Repeating mode
            ret = phone_dev_zarlink_set_mode_off(t, line,
               t->lines[line].mode_cadencer.z_mode_off,
               t->lines[line].mode_cadencer.off_time);
            if (VP_STATUS_SUCCESS != ret) {
               t->lines[line].mode_cadencer.off_time = 0;
               phone_dev_zarlink_update_mode(t, line,
                  t->lines[line].mode_cadencer.mode_on,
                  t->lines[line].mode_cadencer.mode_on);
            }
         }
      }
      else {
         // Repeating mode
         ret = phone_dev_zarlink_set_mode_on(t, line,
            t->lines[line].mode_cadencer.z_mode_on,
            t->lines[line].mode_cadencer.on_time);
         if (VP_STATUS_SUCCESS != ret) {
            t->lines[line].mode_cadencer.off_time = 0;
            phone_dev_zarlink_update_mode(t, line,
               t->lines[line].mode_cadencer.mode_on,
               t->lines[line].mode_cadencer.mode_off);
         }
      }
   }
   else {
      t->lines[line].mode_cadencer.timer -= tick_period;
   }
}

static void phone_dev_zarlink_disconnect_line(phone_dev_zarlink_t *t, size_t line)
{
   phone_line_t *vl = phone_device_get_line(&(t->vd), line);
   bcm_phone_line_state_t *ls;
   VpStatusType status;

   bcm_pr_debug("phone_dev_zarlink_disconnect_line(line=%lu)\n",
      (unsigned long)(line));
   bcm_assert((NULL != vl) && (NULL != phone_dev_zarlink_get_line_ctx(t, line)));

   ls = &(vl->line_state);
   // Set the status to DISCONNECTED and line_mode to NONE
   if (BCMPH_STATUS_DISCONNECTED != ls->status) {
      ls->status = BCMPH_STATUS_DISCONNECTED;
      ls->status_change_count += 1;
      phone_line_line_state_changed(vl);
   }
   vl->new_mode = BCMPH_MODE_IDLE;
   vl->new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
   // Update the line immediately and disconnect it
   phone_dev_zarlink_update_line(t, line);
   t->lines[line].tone_cadencer.off_time = 0;
   t->lines[line].mode_cadencer.off_time = 0;
   status = VpSetLineState(phone_dev_zarlink_get_line_ctx(t, line),
      VP_LINE_DISCONNECT);
   if (VP_STATUS_SUCCESS != status) {
      bcm_pr_err("Failed to disconnect the line %lu (%d)\n",
         (unsigned long)(line), (int)(status));
   }
}

void phone_dev_zarlink_tick(phone_device_t *d)
{
   phone_dev_zarlink_t *t = container_of(d, phone_dev_zarlink_t, vd);

   dd_bcm_pr_debug("phone_dev_zarlink_tick()\n");
#ifndef BCMPH_TEST_PCM
   if (phone_device_is_started(&(t->vd))) {
      VpStatusType status;
      bool event_pending = FALSE;

      status = VpApiTick(phone_dev_zarlink_get_dev_ctx(t), &(event_pending));
      if ((VP_STATUS_SUCCESS == status) && (TRUE == event_pending)) {
         VpEventType event;
         while (VpGetEvent(phone_dev_zarlink_get_dev_ctx(t), &(event))) {
            bool unexpected = true;
            if (VP_STATUS_SUCCESS != event.status) {
               continue;
            }
            switch (event.eventCategory) {
               case VP_EVCAT_FAULT: {
                  switch (event.eventId) {
                     case VP_DEV_EVID_BAT_FLT: {
                        bcm_pr_debug("Received VP_DEV_EVID_BAT_FLT event\n");
                        unexpected = false;

                        if (VP_BAT_FLT_NONE != event.eventData) {
                           size_t i;

                           bcm_pr_err("Battery fault detected on Zarlink device. Disconnecting its lines\n");

                           for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
                              phone_line_t *line = phone_device_get_line(&(t->vd), i);
                              if ((NULL != line) && (phone_line_is_enabled(line))) {
                                 phone_dev_zarlink_disconnect_line(t, i);
                              }
                           }
                        }
                        break;
                     }
                     case VP_DEV_EVID_CLK_FLT: {
                        bcm_pr_debug("Received VP_LINE_EVID_CLK_FLT event\n");
                        unexpected = false;

                        if ((event.eventData & 0x01)) {
                           size_t i;

                           bcm_pr_err("Clock fault detected on Zarlink device. Disconnecting its lines.\n");

                           for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
                              phone_line_t *line = phone_device_get_line(&(t->vd), i);
                              if ((NULL != line) && (phone_line_is_enabled(line))) {
                                 phone_dev_zarlink_disconnect_line(t, i);
                              }
                           }
                        }
                        break;
                     }
                     case VP_LINE_EVID_AC_FLT:
                     case VP_LINE_EVID_DC_FLT:
                     case VP_LINE_EVID_THERM_FLT: {
                        size_t line = event.lineId;
                        if (/* (line >= 0) && */(line <= ARRAY_SIZE(t->lines))) {
                           const phone_line_t *vl = phone_device_get_line(&(t->vd), line);
                           if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                              bcm_pr_debug("Received VP_LINE_EVID_xxx_FLT (%d) event for line %lu\n",
                                 (int)(event.eventId), (unsigned long)(line));
                              unexpected = false;

                              if ((event.eventData & 0x01)) {
                                 bcm_pr_err("Fault detected on line %lu of Zarlink device. Disconnecting the line.\n", (unsigned long)(line));
                                 phone_dev_zarlink_disconnect_line(t, line);
                              }
                           }
                        }
                        break;
                     }
                     default: {
                        break;
                     }
                  }
                  break;
               }
               case VP_EVCAT_SIGNALING: {
                  switch (event.eventId) {
                     case VP_LINE_EVID_HOOK_OFF: {
                        size_t line = event.lineId;
                        if (/* (line >= 0) && */(line <= ARRAY_SIZE(t->lines))) {
                           phone_line_t *vl = phone_device_get_line(&(t->vd), line);
                           if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                              bcm_phone_line_state_t *ls = &(vl->line_state);
                              d_bcm_pr_debug("Received VP_LINE_EVID_HOOK_OFF event for line %lu\n", (unsigned long)(line));
                              unexpected = false;

                              if (BCMPH_STATUS_OFF_HOOK != ls->status) {
                                 ls->status = BCMPH_STATUS_OFF_HOOK;
                                 ls->status_change_count += 1;
                                 phone_line_line_state_changed(vl);
                              }
                              // We force a mode change
                              ls->mode = BCMPH_MODE_UNSPECIFIED;
                              vl->new_mode = BCMPH_MODE_IDLE;
                              vl->new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
                              (*(d->vtbl->update_line_state_asap))(d, line);
                           }
                        }
                        break;
                     }
                     case VP_LINE_EVID_HOOK_ON: {
                        size_t line = event.lineId;
                        if (/* (line >= 0) && */(line <= ARRAY_SIZE(t->lines))) {
                           phone_line_t *vl = phone_device_get_line(&(t->vd), line);
                           if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                              bcm_phone_line_state_t *ls = &(vl->line_state);
                              d_bcm_pr_debug("Received VP_LINE_EVID_HOOK_ON event for line %lu\n", (unsigned long)(line));
                              unexpected = false;

                              if (BCMPH_STATUS_ON_HOOK != ls->status) {
                                 ls->status = BCMPH_STATUS_ON_HOOK;
                                 ls->status_change_count += 1;
                                 phone_line_line_state_changed(vl);
                              }
                              // We force a mode change
                              ls->mode = BCMPH_MODE_UNSPECIFIED;
                              vl->new_mode = BCMPH_MODE_IDLE;
                              vl->new_tone = bcm_phone_line_tone_code_index(BCMPH_TONE_NONE);
                              (*(d->vtbl->update_line_state_asap))(d, line);
                           }
                        }
                        break;
                     }
#ifdef BCMPH_VP_DECODE_PULSE
                     case VP_LINE_EVID_PULSE_DIG: {
                        size_t line = event.lineId;
                        if (/* (line >= 0) && */(line <= ARRAY_SIZE(t->lines))) {
                           phone_line_t *vl = phone_device_get_line(&(t->vd), line);
                           if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                              bcm_phone_line_state_t *ls = &(vl->line_state);
                              // Push the digit in the buffer
                              __u8 digit = event.eventData & 0xF;
                              char c = zarlink_convert_digit_type_to_char(digit);

                              d_bcm_pr_debug("Received VP_LINE_EVID_PULSE_DIG event for line %lu (digit is %c)\n", (unsigned long)(line), (char)(c));
                              unexpected = false;

                              if ((0 != c) && (ls->digits_count < ARRAY_SIZE(ls->digits))) {
                                 ls->digits[ls->digits_count] = c;
                                 ls->digits_count += 1;
                                 phone_line_line_state_changed(vl);
                              }
                           }
                        }
                        break;
                     }
                     case VP_LINE_EVID_FLASH: {
                        size_t line = event.lineId;
                        if (/* (line >= 0) && */(line <= ARRAY_SIZE(t->lines))) {
                           phone_line_t *vl = phone_device_get_line(&(t->vd), line);
                           if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                              bcm_phone_line_state_t *ls = &(vl->line_state);
                              d_bcm_pr_debug("Received VP_LINE_EVID_FLASH event for line %lu\n", (unsigned long)(line));
                              unexpected = false;

                              // We just increment the number of flash event
                              ls->flash_count += 1;
                              phone_line_line_state_changed(vl);
                           }
                        }
                        break;
                     }
                     case VP_LINE_EVID_EXTD_FLASH: {
                        size_t line = event.lineId;
                        if (/* (line >= 0) && */(line <= ARRAY_SIZE(t->lines))) {
                           phone_line_t *vl = phone_device_get_line(&(t->vd), line);
                           if ((NULL != vl) && (phone_line_is_enabled(vl))) {
                              bcm_phone_line_state_t *ls = &(vl->line_state);
                              d_bcm_pr_debug("Received VP_LINE_EVID_EXTD_FLASH event for line %lu\n", (unsigned long)(line));
                              unexpected = false;

                              // It means that we were OFF_HOOK and we briefly change to ON_HOOK before reverting OFF_HOOK again
                              // We just increment status_change_count without verifying that we are in OFF_HOOK state
                              ls->status_change_count += 2;
                              phone_line_line_state_changed(vl);
                           }
                        }
                        break;
                     }
#endif // BCMPH_VP_DECODE_PULSE
                     default: {
                        break;
                     }
                  }
                  break;
               }
               default: {
                  break;
               }
            }
            if (unexpected) {
               /* Do nothing */
               bcm_pr_debug("Unexpected event (%d, %d).\n", (int)(event.eventCategory), (int)(event.eventId));
            }
            if (event.hasResults) {
               __u8 tmp[sizeof(VpResultsType)];
               VpGetResults(&(event), tmp);
            }
         }
      }
#endif // !BCMPH_TEST_PCM

      if (t->line_update_timer > LINE_UPDATE_PERIOD) {
         t->line_update_timer = 0;
      }

      {
         size_t i;

         for (i = 0; (i < ARRAY_SIZE(t->lines)); i += 1) {
            phone_line_t *vl = phone_device_get_line(&(t->vd), i);
            if ((NULL != vl) && (phone_line_is_enabled(vl))) {
               if (t->lines[i].tone_cadencer.off_time > 0) {
                  phone_dev_zarlink_tone_tick(t, i);
               }
               if (t->lines[i].mode_cadencer.off_time > 0) {
                  phone_dev_zarlink_mode_tick(t, i);
               }
               /*
                We must update the line after having called
                phone_dev_zarlink_tone_tick() and phone_dev_zarlink_mode_tick()
                because these two functions could have changed new_mode and
                new_tone.
               */
               if (t->line_update_timer <= 0) {
                  phone_dev_zarlink_update_line(t, i);
               }
            }
         }

      }

      t->line_update_timer += phone_device_get_tick_period(&(t->vd));
#ifndef BCMPH_TEST_PCM
   }
#endif // !BCMPH_TEST_PCM
}

void phone_dev_zarlink_update_line_state_asap(phone_device_t *d, size_t index_line)
{
   phone_dev_zarlink_t *t = container_of(d, phone_dev_zarlink_t, vd);
   t->line_update_timer = 0;
}

#ifdef VP_CC_880_SERIES

#include <vp880_api_int.h>

#ifdef BCMPH_TEST_PCM
extern const char *bcm_drv_param_mpi;
#endif // BCMPH_TEST_PCM

int __init phone_dev_zarlink_ve880_init(phone_dev_zarlink_ve880_t *t,
   const vtbl_phone_dev_zarlink_t *vtbl,
   const phone_desc_device_t *dev_desc, __u8 tick_period)
{
   int ret = 0;

   bcm_pr_debug("phone_dev_zarlink_ve880_init()\n");
   bcm_assert((NULL != vtbl) && (NULL != dev_desc)
      && (VP_DEV_880_SERIES == dev_desc->parameters.zarlink->type));

   ret = bcm_mpi_init(&(t->mpi), &(dev_desc->mpi_params));
   if (!ret) {
      memset(&(t->dev_obj), 0, sizeof(t->dev_obj));
      phone_dev_zarlink_init(&(t->vdz), vtbl, dev_desc, tick_period,
      &(t->dev_obj), &(t->mpi));
#ifdef BCMPH_TEST_PCM
      {
         // Reset SLIC
         __u8 buf[32];

         buf[0] = 0x02;
         bcm_mpi_write(&(t->mpi), buf, 1);
         msleep(10);

         if (NULL != bcm_drv_param_mpi) {
            const char *s = bcm_drv_param_mpi;
            char *endptr;

            bcm_pr_debug("init_bytes = %s\n", s);

            // Parse init_bytes
            for (;;) {
               size_t i = 0;
               do {
                  long l;
                  while ((' ' == *s) || ('\t' == *s)) {
                     s += 1;
                  }
                  if ('\0' == *s) {
                     break;
                  }
                  l = simple_strtol(s, &(endptr), 0);
                  if ((s == endptr) || (l < 0) || (l > 255)) {
                     bcm_pr_debug("Error parsing param mpi ('%s')\n", s);
                     break;
                  }
                  buf[i] = (__u8)(l);
                  i += 1;
                  s = endptr;
                  if ('\0' == *s) {
                     break;
                  }
                  s += 1;
               } while (i < ARRAY_SIZE(buf));
               if (i > 0) {
                  bcm_mpi_write(&(t->mpi), buf, i);
               }
               else {
                  break;
               }
            }
         }
         else {
            bcm_pr_debug("Module param mpi is NULL\n");
         }
      }
#endif // BCMPH_TEST_PCM
   }
   return (ret);
}

void phone_dev_zarlink_ve880_deinit(phone_dev_zarlink_ve880_t *t)
{
   bcm_pr_debug("phone_dev_zarlink_ve880_deinit()\n");
   phone_dev_zarlink_deinit(&(t->vdz));
   bcm_mpi_deinit(&(t->mpi));
}

#ifdef BCMPH_DEBUG
static zarlink_cmd_desc_t ve880_read_dev_registers[] = {
   { VP880_XR_CS_RD, VP880_XR_CS_LEN }, /* 0x45, 1 */
   { VP880_MCLK_CNT_RD, VP880_MCLK_CNT_LEN }, /* 0x47, 1 */
   { VP880_NO_UL_SIGREG_RD, VP880_NO_UL_SIGREG_LEN }, /* 0x4D, 2 */
   { VP880_INT_MASK_RD, VP880_INT_MASK_LEN }, /* 0x6D, 2 */
   { VP880_RCN_PCN_RD, VP880_RCN_PCN_LEN }, /* 0x73, 2 */
};

static zarlink_cmd_desc_t ve880_read_line_registers[] = {
   { VP880_TX_TS_RD, VP880_TX_TS_LEN }, /* 0x41, 1 */
   { VP880_RX_TS_RD, VP880_RX_TS_LEN }, /* 0x43, 1 */
   { VP880_VP_GAIN_RD, VP880_VP_GAIN_LEN }, /* 0x51, 1 */
   { VP880_IODATA_REG_RD, VP880_IODATA_REG_LEN}, /* 0x53, 1 */
   { VP880_IODIR_REG_RD, VP880_IODIR_REG_LEN }, /* 0x55, 1 */
   { VP880_SYS_STATE_RD, VP880_SYS_STATE_LEN }, /* 0x57, 1 */
   { VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN }, /* 0x61, 1 */
   { VP880_SS_CONFIG_RD, VP880_SS_CONFIG_LEN }, /* 0x69, 1 */
   { VP880_OP_COND_RD, VP880_OP_COND_LEN }, /* 0x71, 1 */
   { VP880_GX_GAIN_RD, VP880_GX_GAIN_LEN }, /* 0x81, 2 */
   { VP880_GR_GAIN_RD, VP880_GR_GAIN_LEN }, /* 0x83, 2 */
   { VP880_B1_FILTER_RD, VP880_B1_FILTER_LEN }, /* 0x87, 14 */
   { VP880_X_FILTER_RD, VP880_X_FILTER_LEN }, /* 0x89, 12 */
   { VP880_R_FILTER_RD, VP880_R_FILTER_LEN }, /* 0x8B, 14 */
   { VP880_B2_FILTER_RD, VP880_B2_FILTER_LEN }, /* 0x97, 2 */
   { VP880_Z1_FILTER_RD, VP880_Z1_FILTER_LEN }, /* 0x99, 10 */
   { VP880_Z2_FILTER_RD, VP880_Z2_FILTER_LEN }, /* 0x9B, 5 */
   { VP880_CONV_CFG_RD, VP880_CONV_CFG_LEN }, /* 0xA7, 1 */
   { VP880_LOOP_SUP_RD, VP880_LOOP_SUP_LEN }, /* 0xC3, 4 */
   { VP880_DC_FEED_RD, VP880_DC_FEED_LEN }, /* 0xC7, 2 */
   { VP880_DISN_RD, VP880_DISN_LEN }, /* 0xCB, 1 */
   { VP880_TX_PCM_DATA_RD, VP880_TX_PCM_DATA_LEN }, /* 0xCD, 2 */
   { VP880_METERING_PARAM_RD, VP880_METERING_PARAM_LEN }, /* 0xD1, 4 */
   { VP880_SIGA_PARAMS_RD, VP880_SIGA_PARAMS_LEN }, /* 0xD3, 11 */
   { VP880_SIGCD_PARAMS_RD, VP880_SIGCD_PARAMS_LEN }, /* 0xD5, 8 */
   { VP880_GEN_CTRL_RD, VP880_GEN_CTRL_LEN }, /* 0xDF, 1 */
   { VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN }, /* 0xE1, 4 */
   { VP880_CID_DATA_RD, VP880_CID_DATA_LEN }, /* 0xE3, 1 */
   { VP880_REGULATOR_PARAM_RD, VP880_REGULATOR_PARAM_LEN }, /* 0xE5, 3 */
   { VP880_REGULATOR_CTRL_RD, VP880_REGULATOR_CTRL_LEN }, /* 0xE7, 1 */
   { VP880_CID_PARAM_RD, VP880_CID_PARAM_LEN }, /* 0xEB, 1 */
};

void phone_dev_zarlink_ve880_pr_regs(phone_dev_zarlink_ve880_t *t)
{
#ifndef BCMPH_NOHW
   const phone_desc_device_t *desc = phone_device_get_desc(&(t->vdz.vd));
   printk(KERN_INFO "VE880 device registers :\n");
   zarlink_mpi_pr_regs(&(t->mpi), ve880_read_dev_registers, ARRAY_SIZE(ve880_read_dev_registers), -1);
   printk(KERN_INFO "VE880 line 1 registers :\n");
   zarlink_mpi_pr_regs(&(t->mpi), ve880_read_line_registers, ARRAY_SIZE(ve880_read_line_registers), VP880_EC_CH1);
   if (desc->line_count > 1) {
      printk(KERN_INFO "VE880 line 2 registers :\n");
      zarlink_mpi_pr_regs(&(t->mpi), ve880_read_line_registers, ARRAY_SIZE(ve880_read_line_registers), VP880_EC_CH2);
   }
#endif // !BCMPH_NOHW
}
#endif // BCMPH_DEBUG

int phone_dev_zarlink_ve880_start(phone_device_t *d,
   bcmph_country_t country,
   const phone_line_params_t * const *line_params, size_t line_count)
{
   int ret = 0;
   phone_dev_zarlink_ve880_t *t = container_of(d, phone_dev_zarlink_ve880_t, vdz.vd);

   bcm_pr_debug("phone_dev_zarlink_ve880_start(country=%d, line_count=%lu)\n",
      (int)(country), (unsigned long)(line_count));
   bcm_assert((NULL != line_params) && (line_count > 0));
   phone_dev_zarlink_ve880_stop(d);
   ret = phone_dev_zarlink_start(&(t->vdz), country, line_params, line_count);
   if (!ret) {
#ifdef BCMPH_DEBUG_MPI
      bcm_mpi_dump_and_reset_trace(&(t->mpi));
#endif
   }

   return (ret);
}

void phone_dev_zarlink_ve880_stop(phone_device_t *d)
{
   bcm_pr_debug("phone_dev_zarlink_ve880_stop()\n");
   if (phone_device_is_started(d)) {
      phone_dev_zarlink_ve880_t *t = container_of(d, phone_dev_zarlink_ve880_t, vdz.vd);
#ifdef BCMPH_DEBUG_MPI
      bcm_mpi_dump_and_reset_trace(&(t->mpi));
#endif
      phone_dev_zarlink_stop(&(t->vdz));
#ifdef BCMPH_DEBUG_MPI
      bcm_mpi_dump_and_reset_trace(&(t->mpi));
#endif
   }
}

#endif // VP_CC_880_SERIES
