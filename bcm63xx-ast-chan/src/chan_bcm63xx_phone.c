/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

/* Documentation about API of Asterisk 1.11 is available
   http://doxygen.asterisk.org/trunk/ */

#if ((!defined AST_VERSION) || ((18 != AST_VERSION) && (110 != AST_VERSION) && (130 != AST_VERSION) && (160 != AST_VERSION)))
#error "Preprocessor define AST_VERSION not defined or not equal to 18, 110, 130 or 160"
#endif

/* To be included first */
#include <asterisk.h>

#if (AST_VERSION < 160)
ASTERISK_FILE_VERSION(__FILE__, "$Revision: 1 $")
#endif

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mman.h>

#include <bcm63xx_phone.h>

#include <asterisk/alaw.h>
#include <asterisk/ast_version.h>
#include <asterisk/channel.h>
#include <asterisk/callerid.h>
#include <asterisk/causes.h>
#ifdef BCMPH_NOHW
#include <asterisk/cli.h>
#endif /* BCMPH_NOHW */
#include <asterisk/config.h>
#include <asterisk/dsp.h>
#if (AST_VERSION > 110)
#include <asterisk/format_cache.h>
#endif /* (AST_VERSION > 110) */
#if (AST_VERSION >= 110)
#include <asterisk/format_cap.h>
#endif /* (AST_VERSION >= 110) */
#include <asterisk/frame.h>
#include <asterisk/indications.h>
#include <asterisk/linkedlists.h>
#include <asterisk/lock.h>
#include <asterisk/logger.h>
#include <asterisk/module.h>
#include <asterisk/musiconhold.h>
#include <asterisk/pbx.h>
#include <asterisk/strings.h>
#include <asterisk/ulaw.h>
#include <asterisk/utils.h>

#include "logger.h"

// The following source files are included instead of being compiled separately
// because otherwise the compilation fails with Asterisk 1.8 and GCC >= 5
// with multiple defined Asterisk functions in asterisk/lock.h, asterisk/string.h...
#include "callerid.c"
#include "tone_generation.c"

#if (AST_VERSION < 110)
typedef format_t bcmph_ast_format;
#else /* (AST_VERSION >= 110) */
typedef struct ast_format bcmph_ast_format;
#endif /* (AST_VERSION >= 110) */

#if (AST_VERSION < 110)
typedef format_t bcmph_ast_format_cap;
#else /* (AST_VERSION >= 110) */
typedef struct ast_format_cap bcmph_ast_format_cap;
#endif /* (AST_VERSION >= 110) */

#if (AST_VERSION <= 110)
/*
 If a new format is added, functions bcmph_init_cache_ast_format must
 be updated
 */
static bcmph_ast_format *ast_format_ulaw;
static bcmph_ast_format *ast_format_alaw;
static bcmph_ast_format *ast_format_slin;
static bcmph_ast_format *ast_format_slin16;
#endif /* (AST_VERSION <= 110) */

static void bcmph_init_cache_ast_format(void)
{
#if (AST_VERSION < 110)
   static format_t format_ulaw = AST_FORMAT_ULAW;
   static format_t format_alaw = AST_FORMAT_ALAW;
   static format_t format_slin = AST_FORMAT_SLINEAR;
   static format_t format_slin16 = AST_FORMAT_SLINEAR16;
   ast_format_ulaw = &(format_ulaw);
   ast_format_alaw = &(format_alaw);
   ast_format_slin = &(format_slin);
   ast_format_slin16 = &(format_slin16);
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   static struct ast_format format_ulaw;
   static struct ast_format format_alaw;
   static struct ast_format format_slin;
   static struct ast_format format_slin16;
   ast_format_ulaw = ast_getformatbyname("ulaw", &(format_ulaw));
   bcm_assert(NULL != ast_format_ulaw);
   ast_format_alaw = ast_getformatbyname("alaw", &(format_alaw));
   bcm_assert(NULL != ast_format_alaw);
   ast_format_slin = ast_getformatbyname("slin", &(format_slin));
   bcm_assert(NULL != ast_format_slin);
   ast_format_slin16 = ast_getformatbyname("slin16", &(format_slin16));
   bcm_assert(NULL != ast_format_slin16);
#endif /* (110 == AST_VERSION) */
}

static inline void bcmph_ao2_ref_format(bcmph_ast_format *format, int count)
{
   bcm_assert((NULL != format) && ((1 == count) || (-1 == count)));
#if (AST_VERSION > 110)
   ao2_ref(format, count);
#endif /* (AST_VERSION > 110) */
}

static inline const char *bcmph_ast_format_get_name(
   const bcmph_ast_format *format)
{
   bcm_assert(NULL != format);
#if (AST_VERSION < 110)
   return (ast_getformatname(*format));
#endif /* (110 == AST_VERSION) */
#if (110 == AST_VERSION)
   return (ast_getformatname(format));
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   return (ast_format_get_name(format));
#endif /* (AST_VERSION > 110) */
}

static inline int bcmph_ast_formats_are_equal(
   const bcmph_ast_format *format1, const bcmph_ast_format *format2)
{
   bcm_assert((NULL != format1) && (NULL != format2));
#if (AST_VERSION < 110)
   return (*format1 == *format2);
#else /* (AST_VERSION >= 110) */
   return (AST_FORMAT_CMP_EQUAL == ast_format_cmp(format1, format2));
#endif /* (AST_VERSION > =110) */
}

static inline bcmph_ast_format_cap *bcmph_ast_format_cap_alloc(void)
{
#if (AST_VERSION < 110)
   bcm_assert(false);
   return (NULL);
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   return (ast_format_cap_alloc());
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   return (ast_format_cap_alloc(AST_FORMAT_CAP_FLAG_DEFAULT));
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_format_cap_destroy(bcmph_ast_format_cap *cap)
{
   bcm_assert(NULL != cap);
#if (AST_VERSION < 110)
   bcm_assert(false);
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_cap_destroy(cap);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ao2_ref(cap, -1);
#endif /* (AST_VERSION > 110) */
}

#if (AST_VERSION <= 110)
enum ast_media_type {
   AST_MEDIA_TYPE_UNKNOWN,
   AST_MEDIA_TYPE_AUDIO,
};
#endif /* (AST_VERSION <= 110) */

static inline void bcmph_ast_format_cap_remove_by_type(
   bcmph_ast_format_cap *cap, enum ast_media_type type)
{
   bcm_assert((NULL != cap) &&
      ((AST_MEDIA_TYPE_AUDIO == type) || (AST_MEDIA_TYPE_UNKNOWN == type)));
   if (AST_MEDIA_TYPE_AUDIO == type) {
#if (AST_VERSION < 110)
      *cap &= (~(AST_FORMAT_AUDIO_MASK));
#endif /* (110 == AST_VERSION) */
#if (110 == AST_VERSION)
      ast_format_cap_remove_bytype(cap, AST_FORMAT_TYPE_AUDIO);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
      ast_format_cap_remove_by_type(cap, type);
#endif /* (AST_VERSION > 110) */
   }
   else if (AST_MEDIA_TYPE_UNKNOWN == type) {
#if (AST_VERSION < 110)
      *cap = 0;
#endif /* (110 == AST_VERSION) */
#if (110 == AST_VERSION)
      ast_format_cap_remove_all(cap);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
      ast_format_cap_remove_by_type(cap, type);
#endif /* (AST_VERSION > 110) */
   }
   else {
      bcm_assert(false);
   }
}

static inline void bcmph_ast_format_cap_append_format(
   bcmph_ast_format_cap *cap, bcmph_ast_format *format)
{
   bcm_assert((NULL != cap) && (NULL != format));
#if (AST_VERSION < 110)
   *cap |= *format;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_cap_add(cap, format);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ast_format_cap_append(cap, format, 0);
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_format_cap_remove(
   bcmph_ast_format_cap *cap, bcmph_ast_format *format)
{
   bcm_assert((NULL != cap) && (NULL != format));
#if (AST_VERSION < 110)
   *cap &= (~(*format));
#else /* (AST_VERSION >= 110) */
   ast_format_cap_remove(cap, format);
#endif /* (AST_VERSION > 110) */
}

static inline int bcmph_ast_format_cap_iscompatible_cap(
   const bcmph_ast_format_cap *cap1, const bcmph_ast_format_cap *cap2)
{
   bcm_assert((NULL != cap1) && (NULL != cap2));
#if (AST_VERSION < 110)
   return (*cap1 & *cap2);
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   return (ast_format_cap_has_joint(cap1, cap2));
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   return (ast_format_cap_iscompatible(cap1, cap2));
#endif /* (AST_VERSION > 110) */
}

static inline int bcmph_ast_format_cap_iscompatible_format(
   const bcmph_ast_format_cap *cap, const bcmph_ast_format *format)
{
   bcm_assert((NULL != cap) && (NULL != format));
#if (AST_VERSION < 110)
   return (*cap & *format);
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   return (ast_format_cap_iscompatible(cap, format));
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   return (AST_FORMAT_CMP_EQUAL == ast_format_cap_iscompatible_format(cap, format));
#endif /* (AST_VERSION > 110) */
}

static const char *bcmph_ast_channel_name(const struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (chan->name);
#else /* (AST_VERSION >= 110) */
   return (ast_channel_name(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline enum ast_channel_state bcmph_ast_channel_state(const struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (chan->_state);
#else /* (AST_VERSION >= 110) */
   return (ast_channel_state(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline const char *bcmph_ast_channel_linkedid(const struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (chan->linkedid);
#else /* (AST_VERSION >= 110) */
   return (ast_channel_linkedid(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline struct ast_party_caller *bcmph_ast_channel_caller(struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (&(chan->caller));
#else /* (AST_VERSION >= 110) */
   return (ast_channel_caller(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline struct ast_party_connected_line *bcmph_ast_channel_connected(struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (&(chan->connected));
#else /* (AST_VERSION >= 110) */
   return (ast_channel_connected(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_tech_set(struct ast_channel *chan,
   const struct ast_channel_tech *chan_tech)
{
#if (AST_VERSION < 110)
   chan->tech = chan_tech;
#else /* (AST_VERSION >= 110) */
   ast_channel_tech_set(chan, chan_tech);
#endif /* (AST_VERSION >= 110) */
}

static inline void *bcmph_ast_channel_tech_pvt(struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (chan->tech_pvt);
#else /* (AST_VERSION >= 110) */
   return (ast_channel_tech_pvt(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_tech_pvt_set(
   struct ast_channel *chan, void *value)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   chan->tech_pvt = value;
#else /* (AST_VERSION >= 110) */
   ast_channel_tech_pvt_set(chan, value);
#endif /* (AST_VERSION >= 110) */
}

/* Beware : value must not be allocated on the stack */
static inline void bcmph_ast_channel_nativeformats_set(
   struct ast_channel *chan, bcmph_ast_format_cap *value)
{
   bcm_assert((NULL != chan) && (NULL != value));
#if (AST_VERSION < 110)
   chan->nativeformats = *value;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   struct ast_format_cap *tmpcap = ast_channel_nativeformats(chan);
   if (NULL == tmpcap) {
      tmpcap = ast_format_cap_alloc();
   }
   if (NULL != tmpcap) {
      ast_format_cap_remove_all(tmpcap);
      ast_format_cap_copy(tmpcap, value);
   }
   else {
      bcm_assert(false);
   }
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ast_channel_nativeformats_set(chan, value);
#endif /* (AST_VERSION > 110) */
}

static bcmph_ast_format *bcmph_ast_channel_rawreadformat(
   struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (&(chan->rawreadformat));
#else /* (AST_VERSION >= 110) */
   return (ast_channel_rawreadformat(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_set_rawreadformat(
   struct ast_channel *chan, bcmph_ast_format *format)
{
   bcm_assert((NULL != chan) && (NULL != format));
#if (AST_VERSION < 110)
   chan->rawreadformat = *format;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_copy(ast_channel_rawreadformat(chan), format);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ast_channel_set_rawreadformat(chan, format);
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_channel_set_rawwriteformat(
   struct ast_channel *chan, bcmph_ast_format *format)
{
   bcm_assert((NULL != chan) && (NULL != format));
#if (AST_VERSION < 110)
   chan->rawwriteformat = *format;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_copy(ast_channel_rawwriteformat(chan), format);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ast_channel_set_rawwriteformat(chan, format);
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_channel_set_readformat(
   struct ast_channel *chan, bcmph_ast_format *format)
{
   bcm_assert((NULL != chan) && (NULL != format));
#if (AST_VERSION < 110)
   chan->readformat = *format;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_copy(ast_channel_readformat(chan), format);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ast_channel_set_readformat(chan, format);
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_channel_set_writeformat(
   struct ast_channel *chan, bcmph_ast_format *format)
{
   bcm_assert((NULL != chan) && (NULL != format));
#if (AST_VERSION < 110)
   chan->writeformat = *format;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_copy(ast_channel_writeformat(chan), format);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   ast_channel_set_writeformat(chan, format);
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_channel_context_set(
   struct ast_channel *chan, const char *value)
{
   bcm_assert((NULL != chan) && (NULL != value));
#if (AST_VERSION < 110)
   ast_copy_string(chan->context, value, sizeof(chan->context));
#else /* (AST_VERSION >= 110) */
   ast_channel_context_set(chan, value);
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_exten_set(
   struct ast_channel *chan, const char *value)
{
   bcm_assert((NULL != chan) && (NULL != value));
#if (AST_VERSION < 110)
   ast_copy_string(chan->exten, value, sizeof(chan->exten));
#else /* (AST_VERSION >= 110) */
   ast_channel_exten_set(chan, value);
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_language_set(
   struct ast_channel *chan, const char *value)
{
   bcm_assert((NULL != chan) && (NULL != value));
#if (AST_VERSION < 110)
   ast_string_field_set(chan, language, value);
#else /* (AST_VERSION >= 110) */
   ast_channel_language_set(chan, value);
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_rings_set(
   struct ast_channel *chan, int value)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   chan->rings = value;
#else /* (AST_VERSION >= 110) */
   ast_channel_rings_set(chan, value);
#endif /* (AST_VERSION >= 110) */
}

static inline bcmph_ast_format_cap *bcmph_get_chan_tech_cap(
   struct ast_channel_tech *chan_tech)
{
   bcm_assert(NULL != chan_tech);
#if (AST_VERSION < 110)
   return (&(chan_tech->capabilities));
#else /* (AST_VERSION >= 110) */
   return (chan_tech->capabilities);
#endif /* (AST_VERSION >= 110) */
}

static inline const bcmph_ast_format *bcmph_ast_get_frame_format(
   const struct ast_frame *frame)
{
   bcm_assert(NULL != frame);
#if (AST_VERSION < 110)
   return (&(frame->subclass.codec));
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   return (&(frame->subclass.format));
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   return (frame->subclass.format);
#endif /* (AST_VERSION > 110) */
}

static inline void bcmph_ast_set_frame_format(
   struct ast_frame *frame, bcmph_ast_format *format)
{
   bcm_assert((NULL != frame) && (NULL != format));
#if (AST_VERSION < 110)
   frame->subclass.codec = *format;
#endif /* (AST_VERSION < 110) */
#if (110 == AST_VERSION)
   ast_format_copy(&(frame->subclass.format), format);
#endif /* (110 == AST_VERSION) */
#if (AST_VERSION > 110)
   frame->subclass.format = format;
#endif /* (AST_VERSION > 110) */
}

static inline struct ast_tone_zone *bcmph_ast_channel_zone(const struct ast_channel *chan)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   return (chan->zone);
#else /* (AST_VERSION >= 110) */
   return (ast_channel_zone(chan));
#endif /* (AST_VERSION >= 110) */
}

static inline void bcmph_ast_channel_zone_set(struct ast_channel *chan, struct ast_tone_zone *zone)
{
   bcm_assert(NULL != chan);
#if (AST_VERSION < 110)
   chan->zone = zone;
#else /* (AST_VERSION >= 110) */
   ast_channel_zone_set(chan, zone);
#endif /* (AST_VERSION >= 110) */
}


#define DEFAULT_CALLER_ID "Unknown"

/* The two following values are from the Asterisk code */
#define MIN_DTMF_DURATION 100
#define MIN_TIME_BETWEEN_DTMF 45

/*
 D'apres ce que je comprends le deroulement est le suivant
 * Si l'appel est initie par l'utilisateur :
 celui ci decroche le telephone, ce qui est detecte par le moniteur.
 Ce dernier attend ou pas qu'une extension soit tape, appelle bcmph_new()
 ce qui cree un channel dans l'etat AST_STATE_RING, et appelle ast_pbx_start()
 pour la creation d'un thread pour le channel precedemment cree.
 Asterisk appelle ensuite bcmph_chan_answer() ce qui fait passer le
 channel dans l'etat AST_STATE_UP et passe la ligne en mode conversation.
 En fin de conversation bcmph_chan_hangup() est appelle : le channel est
 detache de la ligne.
 * Si l'appel est recu de l'exterieur :
 Asterisk appelle bcmph_chan_request() qui appelle bcmph_new() ce qui
 cree un channel dans l'etat AST_STATE_DOWN. Pas de thread demarre,
 c'est Asterisk qui s'en chargera plus tard.
 Asterisk appelle ensuite bcmph_chan_call() pour faire sonner le telephone.
 Lorsque l'utilisateur decroche la ligne passe en mode conversation.
 En fin de conversation bcmph_chan_hangup() est appelle : le channel est
 detache de la ligne.
*/

/* #undef BCMPH_DEBUG */


#if (AST_VERSION < 160)
#define AST_MODULE bcmph_chan_type
#else
#define AST_MODULE "Bcm63xxPhone"
#endif

#include <bcm63xx_line_state.c>
#include <bcm63xx_ring_buf.c>

static const char bcmph_device[] = "/dev/bcm63xx_phone";
static const char bcmph_chan_type[] = "Bcm63xxPhone";
static const char bcmph_chan_desc[] = "Broadcom 63xx Telephony Driver";
static const char bcmph_cfg_file[] = "bcm63xx_phone.conf";
static struct {
   const char *name;
   const char *language;
   const char *default_zone;
   bcmph_country_t country;
} bcmph_countries[] = {
   {
      .name = "etsi",
      .language = "en_GB",
      .default_zone = "uk",
      .country = BCMPH_COUNTRY_ETSI,
   },
   {
      /*
       GR-57 refers to a document from Telcordia.
       But not sure of the values we must use
      */
      .name = "gr57",
      .language = "en_US",
      .default_zone = "us",
      .country = BCMPH_COUNTRY_GR57,
   },
   {
      .name = "australia",
      .language = "en_AU",
      .default_zone = "au",
      .country = BCMPH_COUNTRY_AU,
   },
   {
      .name = "austria",
      .language = "de_AT",
      .default_zone = "at",
      .country = BCMPH_COUNTRY_AT,
   },
   {
      .name = "belgium",
      .language = "fr_BE",
      .default_zone = "be",
      .country = BCMPH_COUNTRY_BE,
   },
   {
      .name = "brazil",
      .language = "pt_BR",
      .default_zone = "br",
      .country = BCMPH_COUNTRY_BR,
   },
   {
      .name = "bulgaria",
      .language = "bg_BG",
      .default_zone = "bg",
      .country = BCMPH_COUNTRY_BG,
   },
   {
      .name = "canada",
      .language = "en_CA",
      .default_zone = "us",
      .country = BCMPH_COUNTRY_CA,
   },
   {
      .name = "china",
      .language = "zh_CN",
      .default_zone = "cn",
      .country = BCMPH_COUNTRY_CN,
   },
   {
      .name = "denmark",
      .language = "da_DK",
      .default_zone = "dk",
      .country = BCMPH_COUNTRY_DK,
   },
   {
      .name = "finland",
      .language = "fi_FI",
      .default_zone = "fi",
      .country = BCMPH_COUNTRY_FI,
   },
   {
      .name = "france",
      .language = "fr_FR",
      .default_zone = "fr",
      .country = BCMPH_COUNTRY_FR,
   },
   {
      .name = "germany",
      .language = "de_DE",
      .default_zone = "de",
      .country = BCMPH_COUNTRY_DE,
   },
   {
      .name = "great_britain",
      .language = "en_GB",
      .default_zone = "uk",
      .country = BCMPH_COUNTRY_GB,
   },
   {
      .name = "greece",
      .language = "el_GR",
      .default_zone = "gr",
      .country = BCMPH_COUNTRY_GR,
   },
   {
      .name = "honk_kong",
      .language = "zh_HK",
      .default_zone = "cn",
      .country = BCMPH_COUNTRY_HK,
   },
   {
      .name = "hungary",
      .language = "hu_HU",
      .default_zone = "hu",
      .country = BCMPH_COUNTRY_HU,
   },
   {
      .name = "iceland",
      .language = "is_IS",
      .default_zone = NULL,
      .country = BCMPH_COUNTRY_IS,
   },
   {
      .name = "ireland",
      .language = "en_IE",
      .default_zone = "uk",
      .country = BCMPH_COUNTRY_IE,
   },
   {
      .name = "israel",
      .language = "he_IL",
      .default_zone = "il",
      .country = BCMPH_COUNTRY_IL,
   },
   {
      .name = "italy",
      .language = "it_IT",
      .default_zone = "it",
      .country = BCMPH_COUNTRY_IT,
   },
   {
      .name = "japan",
      .language = "ja_JP",
      .default_zone = "jp",
      .country = BCMPH_COUNTRY_JP,
   },
   {
      .name = "korea",
      .language = "ko_KR",
      .default_zone = NULL,
      .country = BCMPH_COUNTRY_KR,
   },
   {
      .name = "netherlands",
      .language = "nl_NL",
      .default_zone = "nl",
      .country = BCMPH_COUNTRY_NL,
   },
   {
      .name = "new_zealand",
      .language = "en_NZ",
      .default_zone = "nz",
      .country = BCMPH_COUNTRY_NZ,
   },
   {
      .name = "norway",
      .language = "no_NO",
      .default_zone = "no",
      .country = BCMPH_COUNTRY_NO,
   },
   {
      .name = "portugal",
      .language = "pt_PT",
      .default_zone = "pt",
      .country = BCMPH_COUNTRY_PT,
   },
   {
      .name = "russia",
      .language = "ru_RU",
      .default_zone = "ru",
      .country = BCMPH_COUNTRY_RU,
   },
   {
      .name = "singapore",
      .language = "en_SG",
      .default_zone = "sg",
      .country = BCMPH_COUNTRY_SG,
   },
   {
      .name = "south_africa",
      .language = "en_ZA",
      .default_zone = "za",
      .country = BCMPH_COUNTRY_ZA,
   },
   {
      .name = "spain",
      .language = "es_ES",
      .default_zone = "es",
      .country = BCMPH_COUNTRY_ES,
   },
   {
      .name = "sweden",
      .language = "sv_SE",
      .default_zone = "se",
      .country = BCMPH_COUNTRY_SE,
   },
   {
      .name = "switzerland",
      .language = "de_CH",
      .default_zone = "ch",
      .country = BCMPH_COUNTRY_CH,
   },
   {
      .name = "taiwan",
      .language = "zh_TW",
      .default_zone = "tw",
      .country = BCMPH_COUNTRY_TW,
   },
   {
      .name = "turkey",
      .language = "tr_TR",
      .default_zone = NULL,
      .country = BCMPH_COUNTRY_TK,
   },
   {
      .name = "united_states",
      .language = "en_US",
      .default_zone = "us",
      .country = BCMPH_COUNTRY_US,
   },
};

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

enum {
   DETECT_DTMF_WHEN_DIALING = 0x01,
   /*
    Before connection means before function bcmph_setup() is
    called
   */
   DETECT_DTMF_BEFORE_CONNECTION = 0x02,
   /*
    When connected means when in conversation
   */
   DETECT_DTMF_WHEN_CONNECTED = 0x04,
};

typedef struct {
   /* See file bcm63xx-phone.conf */
   bool enable;
   bcmph_cid_signalling_t cid_signalling;
   bcmph_cid_start_t cid_start;
#if (AST_VERSION < 110)
   bcmph_ast_format_cap capabilities;
#else /* (AST_VERSION >= 110) */
   bcmph_ast_format_cap *capabilities;
#endif /* (AST_VERSION >= 110) */
   bool monitor_dialing;
   char search_extension_trigger;
   int dialing_timeout_1st_digit;
   int dialing_timeout;
   int detect_dtmf;
   char context[AST_MAX_CONTEXT];
   char cid_name[AST_MAX_EXTENSION];
   char cid_num[AST_MAX_EXTENSION];
   char moh_interpret[MAX_MUSICCLASS];
   int reverse_polarity;
   int echo_cancel_tap_length;
} bcmph_line_config_t;

static inline bcmph_ast_format_cap *bcmph_get_line_cfg_cap(bcmph_line_config_t *line_cfg)
{
#if (AST_VERSION < 110)
   return (&(line_cfg->capabilities));
#else /* (AST_VERSION >= 110) */
   return (line_cfg->capabilities);
#endif /* (AST_VERSION >= 110) */
}

typedef struct {
   const char *language;
   bcmph_country_t country;
   char zone[16];
   size_t line_count;
   bcmph_line_config_t line_cfgs[BCMPH_MAX_LINES];
   int monitor_busy_period;
   struct {
      const int *cadence;
      size_t cadence_len;
   } ring;
   /*
    Asterisk tone zone corresponding to zone, or default zone
    if zone is empty or is not found in Asterisk config file
    indications.conf
   */
   struct ast_tone_zone *tone_zone;
   /* Definitions of tones extracted from tone_zone */
   bcmph_dual_tone_sequence_t tone_dial;
   bcmph_dual_tone_sequence_t tone_busy;
   bcmph_dual_tone_sequence_t tone_congestion;
} bcmph_chan_config_t;

/*
- The events that can occur on a line
-*/
typedef enum {
   /* Pseudo event initialization */
   BCMPH_EV_INIT,
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
    Phone was on hook and we were in state BCMPH_ST_ON_IDLE, then
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
 That represents (BCMPH_MIN_SIZE_RING_BUFFER / (BCMPH_SAMPLES_PER_MS * 4)) msecs
 BCMPH_MAX_BUF is halved to represent the same time but for slin encoded data
*/
#define BCMPH_MAX_BUF (BCMPH_MIN_SIZE_RING_BUFFER / 2)

/*
 Ringing cadence
*/
#define BCMPH_RING_PULSE_DELAY 250 /* From ETSI specs, should be between 200 and 300ms */

static const int bcmph_default_ring_cadence[] = { 4000, 2000 };

#define BCMPH_RING_STAGE_RING_PULSE -2
#define BCMPH_RING_STAGE_SEND_CID_BEFORE_RING (BCMPH_RING_STAGE_RING_PULSE + 1)
#define BCMPH_RING_STAGE_SEND_CID_AFTER_RING 1

#define BCMPH_VOL_TONE 7219

typedef struct bcmph_pvt
{
   AST_LIST_ENTRY(bcmph_pvt) list;
   /* Pointer to bcmph_chan owning the line */
   struct bcmph_chan *channel;
   /* Index of the line in array channel->config.line_cfgs */
   size_t index_line;
   /* Configuration of the line */
   bcmph_line_config_t *line_cfg;

   /*
    Asterisk channel linked to this line :
    any change of this field must be protected by the monitor's lock and
    the channel lock at the same time, so read of this field is safe
    whenever one of the locks is acquired.
   */
   struct ast_channel *owner;
#ifdef BCMPH_DEBUG
   int owner_lock_count;
#endif /* BCMPH_DEBUG */

   /*
    The following fields are used by the monitor.
    If the monitor is running they must be accessed under the protection
    of the monitor's lock
   */
   struct {
      /*
       Store and accumulate the state of line states in case we can't
       lock the channel in the monitor loop
      */
      bcm_phone_line_state_t line_state;
   } monitor;

   /*
    The following fields must be accessed under the protection of the
    ast_channel's lock (if an ast_channel is associated with this line)
   */
   struct {
      /*
       Current status, mode and tone of the line. Used to detect
       changes and handle events produced by these changes
      */
      bcm_phone_line_status_t current_status;
      bcm_phone_line_mode_t current_mode;
      bcm_phone_line_tone_t current_tone;
      /*
       Last output format set and size of one sample : valid if
       ((BCMPH_MODE_ON_TALKING == current_mode)
       || (BCMPH_MODE_OFF_TALKING == current_mode))
      */
      bcmph_ast_format *current_format;
      /* Size of one sample */
      size_t bytes_per_sample;

      /* Logical state */
      bcmph_state_t state;

      /*
       DSP processor used to detect DTMF digits in data read from the
       driver
      */
      struct ast_dsp *dsp;

      /*
       Ring stage count : valid if (BCMPH_ST_ON_RINGING == state).
       If it's even, phone should be ringing, if it's odd,
       phone should be idle
      */
      int ring_stage;
      /*
       Used to wait a minimum delay :
       - when ringing to toggle the ring on and off
       - when dialing between last digit dialed and the search for
       an extension
       - when in conversation, between the queuing of two DTMFs
      */
      struct timeval tv_wait;

      /*
       Sound generator used to "play" Caller ID or tones before or after
       call is established.
       When call is established, we let Asterisk generates sound
       (cf function bcmph_chan_indicate())
      */
      struct {
         bcmph_sound_generator_t *worker;
         /* The codec used to encode the samples */
         bcmph_ast_format *format;
         /*
          If codec is not slin, the function used to convert slin samples
          generated by sound generator to the codec 'format'
         */
         void (*post_process_fn)(__s16 *buffer, size_t len);
      } sound_generator;
      /* Caller ID generator */
      bcmph_callerid_generator_t cid_generator;
      /* Tone generator for playing tones dial, busy or congestion */
      bcmph_dual_tone_generator_t tone_generator;

      /*
       Used to accumulate digits dialed to recognize an extension when
       dialing, or to send dtmfs when in conversation (because we can't
       send DTMFs too quickly, we must use a buffer to store them)
      */
      char digits[AST_MAX_EXTENSION + 1];
      size_t digits_len;
      /*
       When dialing flag set to true when a new digit is dialed, meaning
       we can search for an extension.
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

      /* Size of frames (in bytes) read from the driver and sent to Asterisk */
      size_t frame_size_read;
      /* Frame used when calling ast_queue_frame(), with its buffer */
      struct ast_frame frame_to_queue;
      __u8 buf_fr_to_queue[BCMPH_MAX_BUF];
      /* Frame used in bcmph_chan_read(), with its buffer */
      struct ast_frame frame;
      __u8 buf_fr[BCMPH_MAX_BUF + AST_FRIENDLY_OFFSET];
      /*
       Buffer used to hold an incomplete sample in order to only write
       complete sample to the driver
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
   /* Tell if Asterisk channel is still locked */
   bool channel_is_locked;
   /* Timeout to use by the monitor at next call to get_line_states() */
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
      /*
       This is the thread for the monitor which checks for input on the
       lines which are not currently in use.
      */
      pthread_t thread;
      ast_mutex_t lock;
#ifdef BCMPH_DEBUG
      int lock_count;
#endif /* BCMPH_DEBUG */
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

static struct {
   bcmph_ast_format *ast_format;
   bcm_phone_codec_t codec;
   size_t bytes_per_sample;
} bcmph_supported_codecs[] = {
   /*
    Order of codecs is important because used by
    bcmph_ast_format_cap_get_preferred()
    We choose the codec that uses the less timeslots
    Even if alaw is told to be simpler to process that ulaw,
    ulaw (which is american) is more "standard" than alaw
    (which is only european)
   */
   {
      .codec = BCMPH_CODEC_ULAW,
      .bytes_per_sample = 1,
   },
   {
      .codec = BCMPH_CODEC_ALAW,
      .bytes_per_sample = 1,
   },
   {
      .codec = BCMPH_CODEC_LINEAR,
      .bytes_per_sample = 2,
   },
   {
      .codec = BCMPH_CODEC_LINEAR16,
      /*
       With codec LINEAR16 we receive (BCMPH_SAMPLES_PER_MS * 2)
       samples of 2 bytes per ms which is equivalent to
       BCMPH_SAMPLES_PER_MS samples of 4 bytes per ms
      */
      .bytes_per_sample = 4,
   },
};

static void bcmph_init_supported_codecs(void)
{
   size_t codec_idx;

   bcmph_init_cache_ast_format();

   for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
      switch (bcmph_supported_codecs[codec_idx].codec) {
         case BCMPH_CODEC_ALAW: {
            bcmph_supported_codecs[codec_idx].ast_format = ast_format_alaw;
            break;
         }
         case BCMPH_CODEC_ULAW: {
            bcmph_supported_codecs[codec_idx].ast_format = ast_format_ulaw;
            break;
         }
         case BCMPH_CODEC_LINEAR: {
            bcmph_supported_codecs[codec_idx].ast_format = ast_format_slin;
            break;
         }
         case BCMPH_CODEC_LINEAR16: {
            bcmph_supported_codecs[codec_idx].ast_format = ast_format_slin16;
            break;
         }
         default: {
            bcm_assert(false);
            bcmph_supported_codecs[codec_idx].ast_format = NULL;
            break;
         }
      }
   }
}

/* The format returned (if any) must be released with bcmph_ao2_ref_format(f, -1) */
static inline bcmph_ast_format *bcmph_ast_format_cap_get_preferred(
   const bcmph_ast_format_cap *cap)
{
   bcmph_ast_format *ret = NULL;
   size_t codec_idx;

   bcm_assert(NULL != cap);

   for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
      if (bcmph_ast_format_cap_iscompatible_format(cap, bcmph_supported_codecs[codec_idx].ast_format)) {
         ret = bcmph_supported_codecs[codec_idx].ast_format;
         bcmph_ao2_ref_format(ret, 1);
         break;
      }
   }
   bcm_assert(codec_idx < ARRAY_SIZE(bcmph_supported_codecs));

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
   int ret = 0;

   bcm_pr_debug("%s()\n", __func__);

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
   int ret = 0;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(t->monitor.lock_count > 0);

   ret = bcmph_do_ioctl(t, BCMPH_IOCTL_STOP_PCM, 0, true);
   if (ret) {
      ast_log(AST_LOG_ERROR, "Unable to stop PCM transfers\n");
   }

   return (ret);
}

/* Must be called with pvt->owner locked */
static int _bcmph_set_line_codec(bcmph_pvt_t *pvt,
   const bcmph_ast_format *ast_format, bcm_phone_line_mode_t mode,
   int echo_cancellation, int reverse_polarity,
   bcm_phone_line_tone_t tone)
{
   int ret;
   bcm_phone_set_line_codec_t set_line_codec;
   size_t codec_idx;
   size_t bytes_per_sample = 1;
   size_t frame_size_read = (BCMPH_MAX_BUF / 2);
   bcmph_ast_format *new_format = NULL;

   bcm_assert(((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != ast_format));

   memset(&(set_line_codec), 0, sizeof(set_line_codec));
   set_line_codec.line = pvt->index_line;
   for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
      if (bcmph_ast_formats_are_equal(bcmph_supported_codecs[codec_idx].ast_format, ast_format)) {
         new_format = bcmph_supported_codecs[codec_idx].ast_format;
         set_line_codec.codec = bcmph_supported_codecs[codec_idx].codec;
         bytes_per_sample = bcmph_supported_codecs[codec_idx].bytes_per_sample;
         if (bytes_per_sample < 2) {
            bcm_assert(1 == bytes_per_sample);
            frame_size_read = (BCMPH_MAX_BUF / 2);
         }
         else {
            frame_size_read = BCMPH_MAX_BUF;
         }
         break;
      }
   }
   bcm_assert(codec_idx < ARRAY_SIZE(bcmph_supported_codecs));
   set_line_codec.mode = mode;
   set_line_codec.echo_cancellation = echo_cancellation;
   set_line_codec.reverse_polarity = reverse_polarity;
   set_line_codec.tone = bcm_phone_line_tone_code_index(tone);

   bcm_pr_debug("%s(codec=%d, mode=%d, echo_cancellation=%d, reverse_polarity=%d, tone=%d)\n",
      __func__, (int)(set_line_codec.codec), (int)(set_line_codec.mode),
      (int)(echo_cancellation), (int)(reverse_polarity),
      (int)(set_line_codec.tone));

   ret = bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_SET_LINE_CODEC, (unsigned long)(&(set_line_codec)), true);
   if (ret) {
      ast_log(AST_LOG_ERROR, "Unable to set line codec %d, for line %lu -> %d\n",
         (int)(set_line_codec.codec), (unsigned long)(pvt->index_line + 1), (int)(ret));
   }
   else {
      /*
       Call succeeded and as we wait for the change to take effect,
       we consider that codec, mode and tone are what we asked
       If mode or tone are not what we want, the change will be detected
       in the monitor
      */
      frame_size_read -= (frame_size_read % bytes_per_sample);
      bcm_assert(NULL != new_format);
      bcmph_ao2_ref_format(new_format, 1);
      if (NULL != pvt->ast_channel.current_format) {
         bcmph_ao2_ref_format(pvt->ast_channel.current_format, -1);
      }
      pvt->ast_channel.current_format = new_format;
      pvt->ast_channel.bytes_per_sample = bytes_per_sample;
      pvt->ast_channel.frame_size_read = frame_size_read;
      bcm_assert(pvt->ast_channel.bytes_per_sample <= ARRAY_SIZE(pvt->ast_channel.bytes_not_written));
      pvt->ast_channel.bytes_not_written_len = 0;
      if (BCMPH_MODE_UNSPECIFIED != mode) {
         pvt->ast_channel.current_mode = mode;
      }
      if (BCMPH_TONE_UNSPECIFIED != tone) {
         pvt->ast_channel.current_tone = tone;
      }
   }

   return (ret);
}

/* Must be called with pvt->owner locked */
static int bcmph_set_line_mode(bcmph_pvt_t *pvt,
   bcm_phone_line_mode_t mode, int echo_cancellation,
   int reverse_polarity, bcm_phone_line_tone_t tone, int wait)
{
   int ret;
   bcm_phone_set_line_mode_t set_line_mode;

   bcm_pr_debug("%s(mode=%d, echo_cancellation=%d, reverse_polarity=%d, tone=%d, wait=%d)\n",
      __func__, (int)(mode), (int)(echo_cancellation),
      (int)(reverse_polarity), (int)(tone), (int)(wait));

   bcm_assert(((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && ((BCMPH_MODE_UNSPECIFIED != mode) || (reverse_polarity >= 0)));

   memset(&(set_line_mode), 0, sizeof(set_line_mode));
   set_line_mode.line = pvt->index_line;
   set_line_mode.mode = mode;
   set_line_mode.echo_cancellation = echo_cancellation;
   set_line_mode.reverse_polarity = reverse_polarity;
   set_line_mode.tone = bcm_phone_line_tone_code_index(tone);
   set_line_mode.wait = wait;
   ret = bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_SET_LINE_MODE, (unsigned long)(&(set_line_mode)), true);
   if (ret < 0) {
      ast_log(AST_LOG_ERROR, "Unable to set line mode %d, for line %lu -> %d\n",
         (int)(mode), (unsigned long)(pvt->index_line + 1), (int)(ret));
   }
   else if (wait < 0) {
      /*
       Call succeeded and as we wait for the change to take effect,
       we consider that mode and tone are what we asked
       If mode or tone are not what we want, the change will be detected
       in the monitor
      */
      if (BCMPH_MODE_UNSPECIFIED != mode) {
         pvt->ast_channel.current_mode = mode;
      }
      if (BCMPH_TONE_UNSPECIFIED != tone) {
         pvt->ast_channel.current_tone = tone;
      }
   }

   return (ret);
}

/* Must be called with pvt->owner locked */
static int bcmph_set_line_tone(bcmph_pvt_t *pvt,
   __u32 tone, int wait)
{
   int ret;
   bcm_phone_set_line_tone_t set_line_tone;

   bcm_pr_debug("%s(tone=%lu, wait=%d)\n", __func__,
      (unsigned long)(tone), (int)(wait));

   bcm_assert(((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (BCMPH_TONE_UNSPECIFIED != bcm_phone_line_tone_decode_index(tone)));

   memset(&(set_line_tone), 0, sizeof(set_line_tone));
   set_line_tone.line = pvt->index_line;
   set_line_tone.tone = tone;
   set_line_tone.wait = wait;
   ret = bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_SET_LINE_TONE, (unsigned long)(&(set_line_tone)), true);
   if (ret < 0) {
      ast_log(AST_LOG_ERROR, "Unable to set line tone %lu, for line %lu -> %d\n",
         (unsigned long)(tone), (unsigned long)(pvt->index_line + 1), (int)(ret));
   }
   else if (wait < 0) {
      /*
       Call succeeded and as we wait for the change to take effect,
       we consider that tone is what we asked
       If tone is not what we want, the change will be detected
       in the monitor
      */
      pvt->ast_channel.current_tone = bcm_phone_line_tone_decode_index(tone);
   }

   return (ret);
}

static inline void bcmph_change_monitor_timeout(
   bcmph_monitor_prms_t *monitor_prms, int timeout)
{
   /* bcm_pr_debug("%s(timeout=%d)\n", __func__, (int)(timeout)); */

   bcm_assert((timeout > 0) && (NULL != monitor_prms));
   if (timeout < monitor_prms->timeout) {
      monitor_prms->timeout = timeout;
   }
}

static void bcmph_convert_slin_to_alaw_in_place(__s16 *buffer, size_t len)
{
   size_t buffer_idx;
   __u8 *ptr = (__u8 *)(buffer);
   for (buffer_idx = 0; (buffer_idx < len); buffer_idx += 1, buffer += 1, ptr += 1) {
      *ptr = AST_LIN2A(*buffer);
   }
}

static void bcmph_convert_slin_to_ulaw_in_place(__s16 *buffer, size_t len)
{
   size_t buffer_idx;
   __u8 *ptr = (__u8 *)(buffer);
   for (buffer_idx = 0; (buffer_idx < len); buffer_idx += 1, buffer += 1, ptr += 1) {
      *ptr = AST_LIN2MU(*buffer);
   }
}

/* Must be called with pvt->owner */
static inline bool bcmph_is_sound_generator_ready(const bcmph_pvt_t *pvt)
{
   return ((NULL != pvt->ast_channel.sound_generator.worker) ? true : false);
}

/* Must be called with pvt->owner */
static void bcmph_unset_sound_generator(bcmph_pvt_t *pvt)
{
   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   if (NULL != pvt->ast_channel.sound_generator.worker) {
      bcm_pr_debug("%s()\n", __func__);
      (*(pvt->ast_channel.sound_generator.worker->vtbl->deinit))(pvt->ast_channel.sound_generator.worker);
      pvt->ast_channel.sound_generator.worker = NULL;
   }
   if (NULL != pvt->ast_channel.sound_generator.format) {
      bcmph_ao2_ref_format(pvt->ast_channel.sound_generator.format, -1);
      pvt->ast_channel.sound_generator.format = NULL;
   }
   pvt->ast_channel.sound_generator.post_process_fn = NULL;
}

/* Must be called with pvt->owner locked */
static bool bcmph_set_sound_generator(bcmph_pvt_t *pvt,
   bcmph_sound_generator_t *sound_generator)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != sound_generator));

   bcmph_unset_sound_generator(pvt);

   bcm_assert((NULL == pvt->ast_channel.sound_generator.worker)
      && (NULL == pvt->ast_channel.sound_generator.post_process_fn)
      && (NULL == pvt->ast_channel.sound_generator.format));

   /* slin is the preferred codec as sound generators produce slin samples */
   if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_slin)) {
      pvt->ast_channel.sound_generator.format = ast_format_slin;
      pvt->ast_channel.sound_generator.post_process_fn = NULL;
   }
   else if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_ulaw)) {
      pvt->ast_channel.sound_generator.format = ast_format_ulaw;
      pvt->ast_channel.sound_generator.post_process_fn = bcmph_convert_slin_to_ulaw_in_place;
   }
   else if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_alaw)) {
      pvt->ast_channel.sound_generator.format = ast_format_alaw;
      pvt->ast_channel.sound_generator.post_process_fn = bcmph_convert_slin_to_alaw_in_place;
   }
   if (NULL != pvt->ast_channel.sound_generator.format) {
      bcmph_ao2_ref_format(pvt->ast_channel.sound_generator.format, 1);
      pvt->ast_channel.sound_generator.worker = sound_generator;
      return (true);
   }
   else {
      (*(sound_generator->vtbl->deinit))(sound_generator);
      bcm_pr_debug("%s() failed\n", __func__);
      return (false);
   }
}

/* Must be called with pvt->owner locked */
static inline void bcmph_unset_tone_generator(bcmph_pvt_t *pvt)
{
   bcm_pr_debug("%s()\n", __func__);

   if (pvt->ast_channel.sound_generator.worker == &(pvt->ast_channel.tone_generator.sound_generator)) {
      bcmph_unset_sound_generator(pvt);
   }
   else {
      bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_NONE), 1);
   }
}

/* Must be called with pvt->owner locked */
static bool bcmph_set_tone_generator(bcmph_pvt_t *pvt, bcmph_dual_tone_sequence_t *tone_def)
{
   bool ret;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != tone_def));

   if (tone_def->part_count > 0) {
      bcmph_dual_tone_generator_init(&(pvt->ast_channel.tone_generator), tone_def, BCMPH_VOL_TONE);
      ret = bcmph_set_sound_generator(pvt, &(pvt->ast_channel.tone_generator.sound_generator));
   }
   else {
      bcm_pr_debug("%s() failed, tone sequence is empty\n", __func__);
      bcmph_unset_sound_generator(pvt);
      ret = false;
   }
   return (ret);
}

/* Must be called with pvt->owner locked. */
static void bcmph_send_sound_generator_data(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   /*
    If possible when we exit we set a timeout that correspond to the
    number of samples in the ring buffer still to be sent, minus
    (2 * bcmph_monitor_short_timeout)
   */
   int timeout;

   bcm_assert(
      (bcmph_is_sound_generator_ready(pvt))
      && (NULL != pvt->ast_channel.sound_generator.worker)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && ((BCMPH_MODE_OFF_TALKING == pvt->ast_channel.current_mode)
          || (BCMPH_MODE_ON_TALKING == pvt->ast_channel.current_mode)));

   if (!bcmph_mmap_trylock(pvt->channel)) {
      bcm_ring_buf_t *rb = &(pvt->ast_channel.mmap.dev_tx_ring_buf);
      size_t free_space_in_samples = bcm_ring_buf_get_free_space(rb) / pvt->ast_channel.bytes_per_sample;
      size_t min_buffer_len = (*(pvt->ast_channel.sound_generator.worker->vtbl->get_min_buffer_len))(pvt->ast_channel.sound_generator.worker);
      size_t total_written = 0;
      /* We set an upper limit to the number of samples generated */
      if (free_space_in_samples > (50 * BCMPH_SAMPLES_PER_MS)) {
         free_space_in_samples = (50 * BCMPH_SAMPLES_PER_MS);
      }
      while (free_space_in_samples >= min_buffer_len) {
         size_t len;
         __u8 *ptr = bcm_ring_buf_get_ptr_store(rb, &(len));
         len /= pvt->ast_channel.bytes_per_sample;
         if (len > free_space_in_samples) {
            len = free_space_in_samples;
         }
         if ((len < min_buffer_len)
             || (NULL != pvt->ast_channel.sound_generator.post_process_fn)) {
            __s16 tmp[384];
            len = free_space_in_samples;
            if (len > ARRAY_SIZE(tmp)) {
               bcm_assert(ARRAY_SIZE(tmp) >= min_buffer_len);
               len = ARRAY_SIZE(tmp);
            }
            len = (*(pvt->ast_channel.sound_generator.worker->vtbl->write))(
               pvt->ast_channel.sound_generator.worker, tmp, len);
            if (0 == len) {
               /* Sound generation completed */
               bcmph_unset_sound_generator(pvt);
               break;
            }
            if (NULL != pvt->ast_channel.sound_generator.post_process_fn) {
               bcm_assert(sizeof(__s16) >= pvt->ast_channel.bytes_per_sample);
               (*(pvt->ast_channel.sound_generator.post_process_fn))(tmp, len);
            }
            else {
               bcm_assert(sizeof(__s16) == pvt->ast_channel.bytes_per_sample);
            }
            bcm_ring_buf_add(rb, (__u8 *)(tmp), len * pvt->ast_channel.bytes_per_sample);
         }
         else {
            /*
             ptr can't be unaligned with _s16 because we (the producer)
             puts (n * (pvt->ast_channel.bytes_per_sample)) bytes and
             the driver (the consumer) removes
             (n * (pvt->ast_channel.bytes_per_sample)) to fill DMA
             buffers
            */
            bcm_assert((sizeof(__s16) == pvt->ast_channel.bytes_per_sample)
               && (0 == (((uintptr_t)((void *)(ptr))) & 0x01)));
            len = (*(pvt->ast_channel.sound_generator.worker->vtbl->write))(
               pvt->ast_channel.sound_generator.worker, (__s16 *)(ptr), len);
            if (0 == len) {
               /* Sound generation completed */
               bcmph_unset_sound_generator(pvt);
               break;
            }
            bcm_ring_buf_add_len(rb, len * pvt->ast_channel.bytes_per_sample);
         }
         total_written += len;
         free_space_in_samples -= len;
      }
      if (total_written > 0) {
         total_written *= pvt->ast_channel.bytes_per_sample;
         if (bcmph_do_ioctl(pvt->channel, BCMPH_IOCTL_WRITE_MM, (unsigned long)((total_written << 8) | pvt->index_line), true)) {
            bcmph_mmap_unlock(pvt->channel);
            ast_log(AST_LOG_ERROR, "Can't add data in TX buffer of line %lu\n", (unsigned long)(pvt->index_line + 1));
            timeout = bcmph_monitor_short_timeout;
         }
         else {
            bcmph_update_ring_bufs_desc(pvt->channel);
            timeout = (int)(bcm_ring_buf_get_size(rb) / (pvt->ast_channel.bytes_per_sample * BCMPH_SAMPLES_PER_MS));
            bcmph_mmap_unlock(pvt->channel);
         }
      }
      else {
         timeout = (int)(bcm_ring_buf_get_size(rb) / (pvt->ast_channel.bytes_per_sample * BCMPH_SAMPLES_PER_MS));
         bcmph_mmap_unlock(pvt->channel);
      }
      if (timeout > (2 * bcmph_monitor_short_timeout)) {
         timeout -= (2 * bcmph_monitor_short_timeout);
      }
      if (timeout < bcmph_monitor_short_timeout) {
         timeout = bcmph_monitor_short_timeout;
      }
   }
   else {
      timeout = bcmph_monitor_short_timeout;
   }

   bcmph_change_monitor_timeout(monitor_prms, timeout);
}

/* Must be called with pvt->owner locked */
static inline int bcmph_set_line_codec_for_sound_generator(
   bcmph_pvt_t *pvt, bcm_phone_line_mode_t mode, int reverse_polarity)
{
   int ret;
   int echo_cancellation = -1;

   bcm_assert((bcmph_is_sound_generator_ready(pvt))
      && ((BCMPH_MODE_OFF_TALKING == mode) || (BCMPH_MODE_ON_TALKING == mode)));

   if ((BCMPH_MODE_OFF_TALKING == mode) && (NULL != pvt->owner)) {
      echo_cancellation = 1;
   }
   else {
      echo_cancellation = 0;
   }
   ret = _bcmph_set_line_codec(pvt,
      pvt->ast_channel.sound_generator.format, mode, echo_cancellation,
      reverse_polarity, BCMPH_TONE_NONE);
   if (ret) {
      bcmph_unset_sound_generator(pvt);
   }
   return (ret);
}

/* Must be called with pvt->owner locked */
static inline int bcmph_set_line_codec(bcmph_pvt_t *pvt,
   const bcmph_ast_format *format, bcm_phone_line_mode_t mode,
   int echo_cancellation, int reverse_polarity,
   bcm_phone_line_tone_t tone)
{
   /* We must cancel sound generator as we change codec */
   bcmph_unset_sound_generator(pvt);
   return (_bcmph_set_line_codec(pvt, format, mode, echo_cancellation,
      reverse_polarity, tone));
}

/* Must be called with pvt->owner locked */
static inline void bcmph_reset_pvt_monitor_state(bcmph_pvt_t *pvt)
{
   bcm_pr_debug("%s()\n", __func__);

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
   bcm_pr_debug("%s(new_state=%d, cause=%d)\n", __func__,
      (int)(new_state), (int)(cause));

   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   switch (new_state) {
      case BCMPH_ST_DISCONNECTED: {
         bcm_assert(NULL == pvt->owner);
         bcm_assert(BCMPH_EV_DISCONNECTED == cause);
         bcmph_set_line_mode(pvt, BCMPH_MODE_DISCONNECT, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE, 0);
         break;
      }
      case BCMPH_ST_ON_IDLE: {
         bcm_assert((BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)
            && (NULL == pvt->owner));
         bcm_assert((BCMPH_EV_INIT == cause) || (BCMPH_EV_AST_HANGUP == cause)
            || (BCMPH_EV_INTERNAL_ERROR == cause) || (BCMPH_EV_ON_HOOK == cause));
         bcmph_unset_sound_generator(pvt);
         bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE, 0);
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
          pvt->line_cfg->dialing_timeout_1st_digit milliseconds
         */
         pvt->ast_channel.search_extension = true;
         pvt->ast_channel.tv_wait = ast_tvnow();
         break;
      }
      case BCMPH_ST_OFF_WAITING_ANSWER: {
         bcm_assert((BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status)
            && (NULL != pvt->owner));
         bcm_assert(((BCMPH_ST_ON_IDLE == pvt->ast_channel.state)
            && (BCMPH_EV_OFF_HOOK == cause) && (!pvt->line_cfg->monitor_dialing))
                    || ((BCMPH_ST_OFF_DIALING == pvt->ast_channel.state)
            && (BCMPH_EV_EXT_FOUND == cause)));
         bcmph_unset_sound_generator(pvt);
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
         bcmph_unset_sound_generator(pvt);
         break;
      }
      case BCMPH_ST_OFF_NO_SERVICE: {
         bcm_phone_line_tone_t tone;
         bcm_assert((BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status)
            && (NULL == pvt->owner));
         bcm_assert(((BCMPH_ST_OFF_DIALING == pvt->ast_channel.state) && (BCMPH_EV_NO_EXT_CAN_BE_FOUND == cause))
            || (BCMPH_EV_INIT == cause)
            || (BCMPH_EV_AST_HANGUP == cause)
            || (BCMPH_EV_INTERNAL_ERROR == cause));
         if (BCMPH_EV_AST_HANGUP == cause) {
            tone = BCMPH_TONE_BUSY;
            bcmph_set_tone_generator(pvt, &(pvt->channel->config.tone_busy));
         }
         else {
            tone = BCMPH_TONE_INVALID;
            bcmph_set_tone_generator(pvt, &(pvt->channel->config.tone_congestion));
         }
         if ((!bcmph_is_sound_generator_ready(pvt))
             || (bcmph_set_line_codec_for_sound_generator(pvt,
                  BCMPH_MODE_OFF_TALKING, pvt->line_cfg->reverse_polarity))) {
            bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, 0, pvt->line_cfg->reverse_polarity, tone, 0);
         }
         bcmph_reset_pvt_monitor_state(pvt);
         pvt->ast_channel.bytes_not_written_len = 0;
         break;
      }
   }
   pvt->ast_channel.state = new_state;
}

/* Must be called with pvt->owner locked (and even with NULL == pvt->owner) */
static bool bcmph_set_new_state_off_dialing(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bcmph_event_t cause)
{
   bool ret = false;
   bcmph_state_t new_state = BCMPH_ST_OFF_DIALING;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert((NULL == pvt->owner)
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   bcmph_set_tone_generator(pvt, &(pvt->channel->config.tone_dial));

   if (!(pvt->line_cfg->detect_dtmf & DETECT_DTMF_WHEN_DIALING)) {
      /*
       We generate tone BCMPH_TONE_WAITING_DIAL and just wait that the driver
       transmit some digits that we use to search for an extension
      */
      if (bcmph_is_sound_generator_ready(pvt)) {
         if (bcmph_set_line_codec_for_sound_generator(pvt,
                BCMPH_MODE_OFF_TALKING, pvt->line_cfg->reverse_polarity)) {
            new_state = BCMPH_ST_OFF_NO_SERVICE;
            cause = BCMPH_EV_INTERNAL_ERROR;
         }
      }
      else {
         bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_WAITING_DIAL, 0);
      }
   }
   else {
      /*
       We generate tone BCMPH_TONE_WAITING_DIAL and set the mode to
       BCMPH_MODE_OFF_TALKING to receive audio data in which we try to detect
       DTMF tones that code the digits we use to search for an extension
      */
      if (bcmph_is_sound_generator_ready(pvt)) {
         /* Codec already fixed by sound generator */
         if (bcmph_set_line_codec_for_sound_generator(pvt,
               BCMPH_MODE_OFF_TALKING, pvt->line_cfg->reverse_polarity)) {
            new_state = BCMPH_ST_OFF_NO_SERVICE;
            cause = BCMPH_EV_INTERNAL_ERROR;
         }
      }
      else {
         /*
          slin is the recommended codec for frames processed by dsp
          (because samples are directly used without conversion)
         */
         bcmph_ast_format *format;
         if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_slin)) {
            format = ast_format_slin;
         }
         else if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_ulaw)) {
            format = ast_format_ulaw;
         }
         else {
            bcm_assert(bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_alaw));
            format = ast_format_alaw;
         }
         if (bcmph_set_line_codec(pvt, format, BCMPH_MODE_OFF_TALKING, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_WAITING_DIAL)) {
            new_state = BCMPH_ST_OFF_NO_SERVICE;
            cause = BCMPH_EV_INTERNAL_ERROR;
         }
      }
   }
   if (BCMPH_ST_OFF_DIALING == new_state) {
      bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.monitor_busy_period);
      ret = true;
   }
   bcmph_set_new_state(pvt, new_state, cause);

   return (ret);
}

static inline bool bcmph_is_ring_stage_really_ringing(
   int ring_stage)
{
   return ((ring_stage % 2) ? false : true);
}

static inline bool bcmph_is_ring_stage_used_to_send_cid_data(
   const bcmph_pvt_t *pvt, int ring_stage)
{
   if ((bcmph_is_sound_generator_ready(pvt))
       && ((BCMPH_RING_STAGE_SEND_CID_BEFORE_RING == ring_stage)
           || (BCMPH_RING_STAGE_SEND_CID_AFTER_RING == ring_stage))) {
      bcm_assert(pvt->ast_channel.sound_generator.worker == &(pvt->ast_channel.cid_generator.sound_generator));
      return (true);
   }
   else {
      return (false);
   }
}

/* Must be called with pvt->owner locked. */
static bool bcmph_start_sending_cid_data(bcmph_pvt_t *pvt)
{
   bool ret = false;
   int reverse_polarity = pvt->line_cfg->reverse_polarity;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert((NULL == pvt->owner) || (pvt->owner_lock_count > 0));

   if ((BCMPH_CID_START_POLARITY == pvt->line_cfg->cid_start)
       || (BCMPH_CID_START_POLARITY_DTMF == pvt->line_cfg->cid_start)) {
      reverse_polarity ^= 1;
   }
   if (!bcmph_set_line_codec_for_sound_generator(pvt, BCMPH_MODE_ON_TALKING, reverse_polarity)) {
      bcm_pr_debug("Start sending CID data\n");
      ret = true;
   }
   return (ret);
}

/* Must be called with pvt->owner locked. */
static bool bcmph_set_new_state_on_ringing(bcmph_pvt_t *pvt,
   const char *addr, const char *cid_name, const char *cid_number)
{
   bool ret = false;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert((NULL != pvt->owner) && (pvt->owner_lock_count > 0));

   do { /* Empty loop */
      bcm_assert((NULL != pvt->channel->config.ring.cadence)
         && (pvt->channel->config.ring.cadence_len > 0)
         && (bcmph_is_ring_stage_really_ringing(pvt->channel->config.ring.cadence_len)));

      pvt->ast_channel.ring_stage = 0;
      if (BCMPH_NO_CID != pvt->line_cfg->cid_signalling) {
         bcmph_callerid_generator_init(&(pvt->ast_channel.cid_generator),
            cid_name, cid_number, pvt->line_cfg->cid_signalling,
            pvt->line_cfg->cid_start, 0);
         bcmph_set_sound_generator(pvt, &(pvt->ast_channel.cid_generator.sound_generator));
         if (bcmph_is_sound_generator_ready(pvt)) {
            if (BCMPH_CID_START_RING_PULSE == pvt->line_cfg->cid_start) {
               pvt->ast_channel.ring_stage = BCMPH_RING_STAGE_RING_PULSE;
            }
            else if (BCMPH_CID_START_RING != pvt->line_cfg->cid_start) {
               pvt->ast_channel.ring_stage = BCMPH_RING_STAGE_SEND_CID_BEFORE_RING;
            }
         }
      }

      bcm_pr_debug("Ringing '%s' on '%s' (with CID \"%s\" <%s>)\n",
         addr, bcmph_ast_channel_name(pvt->owner), cid_name, cid_number);

      if (bcmph_is_ring_stage_used_to_send_cid_data(pvt,
            pvt->ast_channel.ring_stage)) {
         bcm_assert(!bcmph_is_ring_stage_really_ringing(pvt->ast_channel.ring_stage));
         if (!bcmph_start_sending_cid_data(pvt)) {
            break;
         }
         else {
            bcmph_monitor_prms_t monitor_prms;
            memset(&(monitor_prms), 0, sizeof(monitor_prms));
            monitor_prms.channel_is_locked = true;
            monitor_prms.timeout = 0;
            bcmph_send_sound_generator_data(pvt, &(monitor_prms));
         }
      }
      else {
         bcm_assert(bcmph_is_ring_stage_really_ringing(pvt->ast_channel.ring_stage));
         /*
          Here it's important to handle the return code of bcmph_set_line_mode()
          because if the user can't hear the phone ringing, there's a very
          very little chance that he hooks off the phone
         */
         if (bcmph_set_line_mode(pvt, BCMPH_MODE_ON_RINGING, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE, -1)) {
            /* Cancel sound generator because it's useless */
            bcmph_unset_sound_generator(pvt);
            break;
         }
      }

      /*
       Init time to calculate delay between switches from ring on
       to ring off, or the reverse
      */
      pvt->ast_channel.tv_wait = ast_tvnow();

      bcmph_set_new_state(pvt, BCMPH_ST_ON_RINGING, BCMPH_EV_AST_CALL);

      ret = true;
   } while (false);

   return (ret);
}

/* Must be called with pvt->owner and monitor.lock locked */
static void bcmph_unlink_from_ast_channel(bcmph_pvt_t *pvt,
   bcmph_event_t cause, bool unlock_ast_channel)
{
   struct ast_channel *ast = pvt->owner;

   bcm_pr_debug("%s(cause=%d)\n", __func__, (int)(cause));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0));

   if (NULL != pvt->ast_channel.current_format) {
      bcmph_ao2_ref_format(pvt->ast_channel.current_format, -1);
      pvt->ast_channel.current_format = NULL;
   }
   bcmph_ast_channel_tech_pvt_set(ast, NULL);
   pvt->owner = NULL;
   ast_setstate(ast, AST_STATE_DOWN);
   /* Set state before unlocking the channel */
   if (BCMPH_EV_DISCONNECTED == cause) {
      bcmph_set_new_state(pvt, BCMPH_ST_DISCONNECTED, BCMPH_EV_DISCONNECTED);
   }
   else if (BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status) {
      bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, cause);
   }
   else {
      bcm_assert(BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status);
      bcmph_set_new_state(pvt, BCMPH_ST_ON_IDLE, cause);
   }
   bcm_pr_debug("Line %lu unlink from channel '%s'.\n",
      (unsigned long)(pvt->index_line + 1), bcmph_ast_channel_name(ast));
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
#if (AST_VERSION <= 110)
   const char *linkedid,
#else /* (AST_VERSION > 110) */
   const struct ast_assigned_ids *assigned_ids,
   const struct ast_channel *requestor,
#endif /* (AST_VERSION > 110) */
   bool *try_to_lock_ast_channel)
{
   struct ast_channel *tmp;
   enum ast_channel_state ast_state;

   bcm_pr_debug("%s(state=%d, cause=%d)\n", __func__,
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
   /*
    Because we provide no file descriptor to poll (for ast_waitfor()), it's
    important to set parameter needqueue to true.
    That way if Asterisk can't create a timer (eg because there's no timing
    interfaces) it will at least create an "alert" pipe whose read end is polled
    in ast_waitfor() and whose write end is written each time we queue a frame
    (in ast_queue_frame()). read end of the pipe is read in ast_read().
    If Asterisk can create a timer and if its name is "timerfd", it will not
    create the "alert" pipe.
    Asterisk uses the timer by setting it in continuous mode in
    ast_queue_frame() and unsetting from continuous mode in ast_read()
    The file descriptor provided by the timer is used in ast_waitfor().
   */
   tmp = ast_channel_alloc(1, ast_state,
      ('\0' != pvt->line_cfg->cid_num[0]) ? pvt->line_cfg->cid_num : NULL,
      ('\0' != pvt->line_cfg->cid_name[0]) ? pvt->line_cfg->cid_name : NULL,
      "" /* acctcode */, pvt->ast_channel.digits, pvt->line_cfg->context,
#if (AST_VERSION <= 110)
      linkedid, 0,
#else /* (AST_VERSION > 110) */
      assigned_ids, requestor, AST_AMA_NONE,
#endif /* (AST_VERSION > 110) */
      "%s/%lu", pvt->channel->chan_tech.type, (unsigned long)(pvt->index_line + 1));
   if (NULL != tmp) {
      bcmph_ast_format *tmpfmt;
      bool hangup = false;

      /* From version 12.0, ast_channel_alloc() returned an ast_channel locked */
#if (AST_VERSION <= 110)
      /*
       We can safely lock channel (without fear of a deadlock because
       monitor.lock is locked), for 2 reasons :
       - channel_tech is not set
       - pvt->owner is still NULL
      */
      ast_channel_lock(tmp);
#endif /* (AST_VERSION <= 110) */
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */

      /* Set channel tech */
      bcmph_ast_channel_tech_set(tmp, &(pvt->channel->chan_tech));

      /* Set tone zone */
      if (NULL != pvt->channel->config.tone_zone) {
         /* When channel is destroyed by Asterisk, tone_zone is unref automatically */
         bcmph_ast_channel_zone_set(tmp, ast_tone_zone_ref(pvt->channel->config.tone_zone));
      }

      bcmph_ast_channel_nativeformats_set(tmp, bcmph_get_line_cfg_cap(pvt->line_cfg));
      if ((pvt->line_cfg->detect_dtmf & (DETECT_DTMF_BEFORE_CONNECTION | DETECT_DTMF_WHEN_CONNECTED))) {
         if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_slin)) {
            tmpfmt = ast_format_slin;
            bcmph_ao2_ref_format(tmpfmt, 1);
         }
         else {
            tmpfmt = bcmph_ast_format_cap_get_preferred(bcmph_get_line_cfg_cap(pvt->line_cfg));
         }
      }
      else {
         tmpfmt = bcmph_ast_format_cap_get_preferred(bcmph_get_line_cfg_cap(pvt->line_cfg));
      }
      bcmph_ast_channel_set_rawreadformat(tmp, tmpfmt);
      bcmph_ast_channel_set_rawwriteformat(tmp, tmpfmt);
      bcmph_ast_channel_set_readformat(tmp, tmpfmt);
      bcmph_ast_channel_set_writeformat(tmp, tmpfmt);
      /* no need to call ast_setstate: the channel_alloc already did its job */
      bcmph_ast_channel_context_set(tmp, pvt->line_cfg->context);
      bcmph_ast_channel_exten_set(tmp, pvt->ast_channel.digits);
      if (!ast_strlen_zero(pvt->channel->config.language)) {
         bcmph_ast_channel_language_set(tmp, pvt->channel->config.language);
      }
      if (AST_STATE_RING == ast_state) {
         bcmph_ast_channel_rings_set(tmp, 1);
      }
      /* Don't use ast_set_callerid() here because it will
       * generate a NewCallerID event before the NewChannel event */
      if (!ast_strlen_zero(pvt->line_cfg->cid_num)) {
         bcmph_ast_channel_caller(tmp)->ani.number.valid = 1;
         ast_free(bcmph_ast_channel_caller(tmp)->ani.number.str);
         bcmph_ast_channel_caller(tmp)->ani.number.str = ast_strdup(pvt->line_cfg->cid_num);
      }

      bcmph_ast_channel_tech_pvt_set(tmp, pvt);
      ast_module_ref(ast_module_info->self);
      pvt->owner = tmp;

      /*
       Set internal state before starting channel's thread
      */
      bcmph_set_new_state(pvt, state, cause);

      /* Unset pvt->owner before unlocking channel (see below) */
      pvt->owner = NULL;
#ifdef BCMPH_DEBUG
      bcm_assert(pvt->owner_lock_count > 0);
      pvt->owner_lock_count -= 1;
#endif /* BCMPH_DEBUG */
      ast_channel_unlock(tmp);

      if (AST_STATE_DOWN != ast_state) {
         bcm_assert(BCMPH_ST_OFF_WAITING_ANSWER == state);
         /*
          Set line codec, switch to mode BCMPH_MODE_OFF_TALKING and stop
          tone. We turn on echo canceller because we don't know what
          Asterisk does with the frames we read from device and send to
          it
         */
         if (bcmph_set_line_codec(pvt, tmpfmt, BCMPH_MODE_OFF_TALKING, 1, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE)) {
            hangup = true;
         }
         /* Start channel's thread */
         else if (ast_pbx_start(tmp)) {
            ast_log(AST_LOG_ERROR, "Unable to start PBX on '%s'\n", bcmph_ast_channel_name(tmp));
            hangup = true;
         }
      }
      bcmph_ao2_ref_format(tmpfmt, -1);

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
   const bcmph_ast_format *format,
   enum ast_channel_state ast_state)
{
   int ret = 0;

   bcm_pr_debug("%s(ast_state=%d)\n", __func__, (int)(ast_state));

   bcm_assert((NULL != pvt->owner) && (pvt->owner_lock_count > 0));

   do { /* Empty loop */
      if (NULL == format) {
         ast_log(AST_LOG_WARNING, "No format specified\n");
         ret = -1;
         break;
      }
      if (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), format)) {
         ast_log(AST_LOG_WARNING, "Can't do format '%s'\n", bcmph_ast_format_get_name(format));
         ret = -1;
         break;
      }
      if (bcmph_set_line_codec(pvt, format, BCMPH_MODE_OFF_TALKING, 1, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE)) {
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

   /* bcm_pr_debug("%s()\n", __func__); */

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
         bcm_assert(NULL != pvt->ast_channel.current_format);
         bcmph_ast_set_frame_format(&(pvt->ast_channel.frame), pvt->ast_channel.current_format);
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

   /* bcm_pr_debug("%s()\n", __func__); */

   bcm_assert((NULL != unlock_mmap) && (*unlock_mmap)
      && (pvt->channel->mmap.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0)
      && ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
          || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)));

   do { /* Empty loop */
      size_t len;
      bool detect_dtmf = (
         ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state) && ((pvt->line_cfg->detect_dtmf & DETECT_DTMF_WHEN_CONNECTED)))
         || ((BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state) && ((pvt->line_cfg->detect_dtmf & DETECT_DTMF_BEFORE_CONNECTION))));

      /* Check that there are data in the ring buffer */
      len = bcm_ring_buf_get_size(&(pvt->ast_channel.mmap.dev_rx_ring_buf));
      while (len >= pvt->ast_channel.frame_size_read) {
         len = pvt->ast_channel.frame_size_read;
         pvt->ast_channel.frame_to_queue.datalen = len;
         pvt->ast_channel.frame_to_queue.samples = len / pvt->ast_channel.bytes_per_sample;
         pvt->ast_channel.frame_to_queue.frametype = AST_FRAME_VOICE;
         bcm_assert(NULL != pvt->ast_channel.current_format);
         bcmph_ast_set_frame_format(&(pvt->ast_channel.frame_to_queue), pvt->ast_channel.current_format);
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
            pvt->ast_channel.frame_to_queue.data.ptr = (void *)(bcm_ring_buf_get_ptr_load(&(pvt->ast_channel.mmap.dev_rx_ring_buf), 0, &(len_direct)));
            if (len_direct < len) {
               pvt->ast_channel.frame_to_queue.data.ptr = pvt->ast_channel.buf_fr_to_queue;
               bcm_ring_buf_load(&(pvt->ast_channel.mmap.dev_rx_ring_buf), 0, pvt->ast_channel.frame_to_queue.data.ptr, len);
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

   bcm_pr_debug("%s(cause=%d)\n", __func__, (int)(cause));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL != pvt->owner) && (pvt->owner_lock_count > 0));

   bcmph_unlink_from_ast_channel(pvt, cause, true);
   bcm_assert(NULL == pvt->owner);
   if (ast_queue_hangup(ast)) {
      ast_log(AST_LOG_WARNING, "Unable to queue hangup on line '%s'\n", bcmph_ast_channel_name(ast));
   }
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_status_change(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bcm_phone_line_status_t new_status)
{
   /* bcm_pr_debug("%s(new_status=%d)\n", __func__, (int)(new_status)); */

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
            bcm_assert(AST_STATE_RINGING == bcmph_ast_channel_state(pvt->owner));

            /* The user wants to answer the call */
            if (bcmph_setup(pvt, bcmph_ast_channel_rawreadformat(pvt->owner), AST_STATE_UP)) {
               ast_log(AST_LOG_ERROR, "Unable to answer the call on '%s'\n", bcmph_ast_channel_name(pvt->owner));
            }
            else {
               if (ast_queue_control(pvt->owner, AST_CONTROL_ANSWER)) {
                  ast_log(AST_LOG_ERROR, "Unable to answer the call on '%s'\n", bcmph_ast_channel_name(pvt->owner));
               }
               else {
                  bcm_pr_debug("Call answered on '%s'\n", bcmph_ast_channel_name(pvt->owner));
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
            bcm_assert((NULL == pvt->owner) && (0 == pvt->owner_lock_count) && (BCMPH_ST_OFF_NO_SERVICE == pvt->ast_channel.state));
         }
      }
      else {
         bcm_assert(
            ((BCMPH_ST_ON_IDLE == pvt->ast_channel.state)
             || (BCMPH_ST_DISCONNECTED == pvt->ast_channel.state))
            && (NULL == pvt->owner));
         /* The user has picked up the phone to make a call */
         if (pvt->line_cfg->monitor_dialing) {
            /*
             We do not create an ast_channel now
             We just change the tone to BCMPH_TONE_WAITING_DIAL, and
             wait for digits dialed by the user.
             When the digits will form a valid extension then we will
             create the ast_channel dedicated to handle the call
            */
            bcmph_set_new_state_off_dialing(pvt, monitor_prms, BCMPH_EV_OFF_HOOK);
         }
         else { /* (!pvt->line_cfg->monitor_dialing) */
            /*
             We create an ast_channel now.
            */
            pvt->ast_channel.digits_len = 0;
            bcmph_new(pvt, BCMPH_ST_OFF_WAITING_ANSWER, BCMPH_EV_OFF_HOOK,
#if (AST_VERSION > 110)
               NULL,
#endif /* (AST_VERSION > 110) */
               NULL, &(monitor_prms->channel_is_locked));
         }
      }
   }
   else { /* (BCMPH_STATUS_OFF_HOOK != new_status) */
      bcmph_state_t new_state = BCMPH_ST_ON_IDLE;
      bcmph_event_t cause = BCMPH_EV_ON_HOOK;
      bcm_assert(BCMPH_STATUS_ON_HOOK == new_status);
      /*
       At least one off hook to on hook transition
      */
      if (NULL != pvt->owner) {
         /* Gone on hook so notify hangup */
         bcmph_queue_hangup(pvt, cause);
         /* ast_channel_unlock(pvt->owner) is done in bcmph_queue_hangup() */
         bcm_assert((NULL == pvt->owner) && (0 == pvt->owner_lock_count) && (new_state == pvt->ast_channel.state));
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
   /* bcm_pr_debug("%s(new_mode=%d)\n", __func__, (int)(new_mode)); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   if (pvt->ast_channel.current_mode != new_mode) {
      pvt->ast_channel.current_mode = new_mode;
      if (BCMPH_MODE_DISCONNECT == new_mode) {
         bcmph_state_t new_state = BCMPH_ST_DISCONNECTED;
         bcmph_event_t cause = BCMPH_EV_DISCONNECTED;
         ast_log(AST_LOG_WARNING, "Line %lu has been disconnected\n", (unsigned long)(pvt->index_line + 1));
         if (NULL != pvt->owner) {
            /* Gone disconnected, so notify hangup */
            bcmph_queue_hangup(pvt, cause);
            /* ast_channel_unlock(pvt->owner) is done in bcmph_queue_hangup() */
            bcm_assert((NULL == pvt->owner) && (0 == pvt->owner_lock_count) && (new_state == pvt->ast_channel.state));
         }
         else {
            bcmph_set_new_state(pvt, new_state, cause);
         }
      }
   }
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_handle_tone_change(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bcm_phone_line_tone_t new_tone)
{
   /* bcm_pr_debug("%s(new_tone=%d)\n", __func__, (int)(new_tone)); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   pvt->ast_channel.current_tone = new_tone;
}

/*
 Must be called with monitor.lock locked and pvt->owner set to NULL
*/
static void bcmph_search_extension(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms, bool ignore_timeout)
{
   /* bcm_pr_debug("%s(ignore_timeout=%d)\n", __func__, (int)(ignore_timeout)); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && (NULL == pvt->owner) && (pvt->line_cfg->monitor_dialing)
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && (BCMPH_ST_OFF_DIALING == pvt->ast_channel.state));

   do { /* Empty loop */
      if (!pvt->ast_channel.search_extension) {
         break;
      }
      bcm_assert(pvt->ast_channel.digits_len < ARRAY_LEN(pvt->ast_channel.digits));
      if (pvt->ast_channel.digits_len <= 0) {
         if ((!ignore_timeout) && (pvt->line_cfg->dialing_timeout_1st_digit > 0)) {
            struct timeval now = ast_tvnow();
            int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_wait);
            if (tvdiff < pvt->line_cfg->dialing_timeout_1st_digit) {
               bcmph_change_monitor_timeout(monitor_prms, (int)(pvt->line_cfg->dialing_timeout_1st_digit - tvdiff));
               break;
            }
            bcm_pr_debug("Timeout %d reached : searching for default extension\n", (int)(pvt->line_cfg->dialing_timeout_1st_digit));
         }
         bcm_assert(ARRAY_LEN(bcmph_default_extension) <= ARRAY_LEN(pvt->ast_channel.digits));
         strcpy(pvt->ast_channel.digits, bcmph_default_extension);
      }
      else {
         if ((!ignore_timeout) && (pvt->line_cfg->dialing_timeout > 0)) {
            struct timeval now = ast_tvnow();
            int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_wait);
            if (tvdiff < pvt->line_cfg->dialing_timeout) {
               bcmph_change_monitor_timeout(monitor_prms, (int)(pvt->line_cfg->dialing_timeout - tvdiff));
               break;
            }
            bcm_pr_debug("Timeout %d reached : searching for extension\n", (int)(pvt->line_cfg->dialing_timeout));
         }
         pvt->ast_channel.digits[pvt->ast_channel.digits_len] = '\0';
      }
      pvt->ast_channel.search_extension = false;
      /* We test if, with these numbers, the extension is valid */
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
#if (AST_VERSION > 110)
            NULL,
#endif /* (AST_VERSION > 110) */
            NULL, &(monitor_prms->channel_is_locked));
      }
      else {
         if (!ast_canmatch_extension(NULL, pvt->line_cfg->context, pvt->ast_channel.digits, 1, pvt->line_cfg->cid_num)) {
            /* It's not a valid extension */
            bcm_pr_debug("Extension '%s' can't match anything in '%s'\n",
               pvt->ast_channel.digits, pvt->line_cfg->context);
            /* We signals the user that there's a problem */
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
            bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, BCMPH_EV_NO_EXT_CAN_BE_FOUND);
         }
      }
   } while (false);
}

/*
 Must be called with pvt->owner locked.
*/
static void bcmph_try_to_send_dtmf(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   bool send_a_null_frame = false;

   bcm_assert((NULL != pvt->owner) && (pvt->owner_lock_count > 0)
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
          || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)));

   if (NONE != pvt->ast_channel.dtmf_sent) {
      struct timeval now = ast_tvnow();
      int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_wait);
      if (ON == pvt->ast_channel.dtmf_sent) {
         if (tvdiff < MIN_DTMF_DURATION) {
            bcmph_change_monitor_timeout(monitor_prms, (int)(MIN_DTMF_DURATION - tvdiff));
         }
         else {
            pvt->ast_channel.dtmf_sent = OFF;
            pvt->ast_channel.tv_wait = ast_tvnow();
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
      if (ast_queue_frame(pvt->owner, &(ast_null_frame))) {
         ast_log(AST_LOG_WARNING, "Can't queue null frame for line %lu\n", (unsigned long)(pvt->index_line + 1));
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
         pvt->ast_channel.tv_wait = ast_tvnow();
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
   bcm_pr_debug("%s(digits_len=%lu)\n", __func__, (unsigned long)(digits_len));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && (NULL != digits) && (digits_len > 0));

   if ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
       || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)) {
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
         bcmph_search_extension(pvt, monitor_prms, false);
         if ((BCMPH_ST_OFF_DIALING != pvt->ast_channel.state) || (!monitor_prms->channel_is_locked)) {
            break;
         }
         while ((i < digits_len) && (pvt->ast_channel.digits_len < (ARRAY_LEN(pvt->ast_channel.digits) - 1))) {
            if (0 == pvt->ast_channel.digits_len) {
               /* We switch off the BCMPH_TONE_WAITING_DIAL tone */
               bcmph_unset_tone_generator(pvt);
            }
            /*
             If the digit is equal to pvt->line_cfg->search_extension_trigger,
             we search for an extension now, without adding the digit
            */
            bcm_assert('\0' != digits[i]);
            if (/* ('\0' != pvt->line_cfg->search_extension_trigger)
                && */(digits[i] == pvt->line_cfg->search_extension_trigger)) {
               bcm_pr_debug("Digit %c dialed : searching extension immediately\n", (char)(digits[i]));
               pvt->ast_channel.search_extension = true;
               bcmph_search_extension(pvt, monitor_prms, true);
               bcm_assert(!pvt->ast_channel.search_extension);
               if ((BCMPH_ST_OFF_DIALING != pvt->ast_channel.state) || (!monitor_prms->channel_is_locked)) {
                  break;
               }
            }
            pvt->ast_channel.digits[pvt->ast_channel.digits_len] = digits[i];
            i += 1;
            pvt->ast_channel.digits_len += 1;
            /* If there's no timeout, we search for an extension now */
            if (pvt->line_cfg->dialing_timeout <= 0) {
               pvt->ast_channel.search_extension = true;
               bcmph_search_extension(pvt, monitor_prms, true);
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
               pvt->ast_channel.tv_wait = ast_tvnow();
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
   size_t flash_idx;
   char digit;
#endif /* BCMPH_DEBUG */

   bcm_pr_debug("%s(flash_count=%lu)\n", __func__, (unsigned long)(flash_count));

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

#ifdef BCMPH_DEBUG
   digit = '#';
   for (flash_idx = 0; (flash_idx < flash_count); flash_idx += 1) {
      bcmph_handle_digits(pvt, monitor_prms, &(digit), 1);
      if (!monitor_prms->channel_is_locked) {
         break;
      }
   }
#else /* BCMPH_DEBUG */
   if ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
       || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)) {
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
static void bcmph_monitor_pvt_ring_state(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   /* bcm_pr_debug("%s()\n", __func__); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked)
      && (BCMPH_ST_ON_RINGING == pvt->ast_channel.state));

   do { /* Empty loop */
      /*
       We can't test pvt->ast_channel.current_mode because it's not set
       immediately (only at next iteration of bcmph_do_monitor())
       Instead we test pvt->ast_channel.ring_sate, if it's even or odd
      */
      if (bcmph_is_ring_stage_really_ringing(pvt->ast_channel.ring_stage)) {
         /* Phone should be ringing */
         struct timeval now = ast_tvnow();
         int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_wait);
         if (pvt->ast_channel.ring_stage < 0) {
            /* Ring pulse used as alert signal before sending CID data */
            bcm_assert(BCMPH_RING_STAGE_RING_PULSE == pvt->ast_channel.ring_stage);
            if (tvdiff < BCMPH_RING_PULSE_DELAY) {
               bcmph_change_monitor_timeout(monitor_prms, (int)(BCMPH_RING_PULSE_DELAY - tvdiff));
               break;
            }
         }
         else {
            int ring_index = pvt->ast_channel.ring_stage % pvt->channel->config.ring.cadence_len;
            if (tvdiff < pvt->channel->config.ring.cadence[ring_index]) {
               bcmph_change_monitor_timeout(monitor_prms, (int)(pvt->channel->config.ring.cadence[ring_index] - tvdiff));
               break;
            }
         }

         /* Change to mode ON_TALKING if we need to send CID data, else change to ON_IDLE */
         if (bcmph_is_ring_stage_used_to_send_cid_data(pvt, pvt->ast_channel.ring_stage + 1)) {
            if (bcmph_start_sending_cid_data(pvt)) {
               pvt->ast_channel.ring_stage += 1;
               bcm_assert(!bcmph_is_ring_stage_really_ringing(pvt->ast_channel.ring_stage));
               pvt->ast_channel.tv_wait = ast_tvnow();
               /*
                Fill RX ring buffer with CID data
                (bcmph_send_sound_generator_data() sets the timeout)
               */
               bcmph_send_sound_generator_data(pvt, monitor_prms);
            }
            else {
               bcmph_queue_hangup(pvt, BCMPH_EV_INTERNAL_ERROR);
               bcm_assert((NULL == pvt->owner) && (0 == pvt->owner_lock_count) && (BCMPH_ST_ON_IDLE == pvt->ast_channel.state));
            }
         }
         else {
            bcm_assert(pvt->ast_channel.ring_stage >= 0);
            if (!bcmph_set_line_mode(pvt, BCMPH_MODE_IDLE, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE, 0)) {
               pvt->ast_channel.ring_stage += 1;
               bcm_assert(!bcmph_is_ring_stage_really_ringing(pvt->ast_channel.ring_stage));
               pvt->ast_channel.tv_wait = ast_tvnow();
               bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.ring.cadence[pvt->ast_channel.ring_stage % pvt->channel->config.ring.cadence_len]);
            }
            else {
               /*
                We ignore failure to set the mode and continue ringing.
                We just omit to increment ring_stage
               */
               bcmph_change_monitor_timeout(monitor_prms, bcmph_monitor_short_timeout);
            }
         }
      }
      else {
         /* Phone should not be ringing */
         if (bcmph_is_ring_stage_used_to_send_cid_data(pvt, pvt->ast_channel.ring_stage)) {
            /*
             If there are still CID data to send, we send them and
             ignore any timeout.
             And even if after call to bcmph_send_cid_data() there's no
             more data to send, we do not switch to mode ON_RINGING now,
             but only the next time this function is called, in order to
             let CID data be actually sent on the PCM bus.
            */
            bcmph_send_sound_generator_data(pvt, monitor_prms);
            break;
         }
         /* No more CID data to send, should the phone continues ringing ? */
         if (pvt->ast_channel.ring_stage >= 0) {
            struct timeval now = ast_tvnow();
            int64_t tvdiff = ast_tvdiff_ms(now, pvt->ast_channel.tv_wait);
            int ring_index = pvt->ast_channel.ring_stage % pvt->channel->config.ring.cadence_len;
            if (tvdiff < pvt->channel->config.ring.cadence[ring_index]) {
               bcmph_change_monitor_timeout(monitor_prms, (int)(pvt->channel->config.ring.cadence[ring_index] - tvdiff));
               break;
            }
         }
         else {
            bcm_assert((pvt->ast_channel.ring_stage + 1) >= 0);
         }
         /* Don't forget to restore polarity */
         if (!bcmph_set_line_mode(pvt, BCMPH_MODE_ON_RINGING, 0, pvt->line_cfg->reverse_polarity, BCMPH_TONE_NONE, 0)) {
            pvt->ast_channel.ring_stage += 1;
            bcm_assert(bcmph_is_ring_stage_really_ringing(pvt->ast_channel.ring_stage));
            pvt->ast_channel.tv_wait = ast_tvnow();
            bcmph_change_monitor_timeout(monitor_prms, pvt->channel->config.ring.cadence[pvt->ast_channel.ring_stage % pvt->channel->config.ring.cadence_len]);
         }
         else {
            /*
             We ignore failure to set the mode.
             We just omit to increment ring_stage
            */
            bcmph_change_monitor_timeout(monitor_prms, bcmph_monitor_short_timeout);
         }
      }
   } while (false);
}

/*
 Must be called with pvt->owner and monitor.lock locked.
*/
static void bcmph_monitor_pvt(bcmph_pvt_t *pvt,
   bcmph_monitor_prms_t *monitor_prms)
{
   /* bcm_pr_debug("%s()\n", __func__); */

   bcm_assert((pvt->channel->monitor.lock_count > 0)
      && ((NULL == pvt->owner) || (pvt->owner_lock_count > 0))
      && (NULL != monitor_prms) && (monitor_prms->channel_is_locked));

   if ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
       || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)) {
      bcm_assert(NULL != pvt->owner);
      bcmph_try_to_send_dtmf(pvt, monitor_prms);
      if (monitor_prms->channel_is_locked) {
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
      if (bcmph_is_sound_generator_ready(pvt)) {
         bcmph_send_sound_generator_data(pvt, monitor_prms);
      }
      if ((pvt->line_cfg->detect_dtmf & DETECT_DTMF_WHEN_DIALING)) {
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
         bcmph_search_extension(pvt, monitor_prms, false);
      }
   }
   else if (BCMPH_ST_OFF_NO_SERVICE == pvt->ast_channel.state) {
      if (bcmph_is_sound_generator_ready(pvt)) {
         bcmph_send_sound_generator_data(pvt, monitor_prms);
      }
   }
   else if (BCMPH_ST_ON_RINGING == pvt->ast_channel.state) {
      bcmph_monitor_pvt_ring_state(pvt, monitor_prms);
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
      size_t lines_using_pcm_bus;
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

      lines_using_pcm_bus = 0;
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

            /*
             We count the line that uses the PCM bus, so that at the
             end of the loop we can start or stop the PCM
            */
            if ((BCMPH_MODE_OFF_TALKING == line_state->mode)
                || (BCMPH_MODE_ON_TALKING == line_state->mode)) {
               lines_using_pcm_bus += 1;
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

         /* Clear rev_polarity change count */
         if (line_state->rev_polarity_change_count > 0) {
            bcm_pr_debug("Line %lu, rev polarity changes %lu times. Is now %d\n",
               (unsigned long)(pvt->index_line + 1),
               (unsigned long)(line_state->rev_polarity_change_count),
               (int)(line_state->rev_polarity));
            bcm_phone_line_state_reset_rev_polarity_change_count(line_state);
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

      /* Start or stop PCM if needed */
      if (line_states_are_valid) {
         if (lines_using_pcm_bus > 0) {
            if (!get_line_states.pcm_is_started) {
               bcmph_start_pcm(t);
            }
         }
         else {
            if (get_line_states.pcm_is_started) {
               bcmph_stop_pcm(t);
            }
         }
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
   bcm_pr_debug("%s()\n", __func__);

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
      size_t line_idx;
      bcm_phone_cfg_params_t *phone_cfg;
      bcm_phone_get_mmap_rbs_location_t *mmap_rbs_location;
      bcm_phone_get_line_states_t *get_line_states;
      bcmph_pvt_t *pvt;

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
      for (line_idx = 0; (line_idx < ARRAY_LEN(phone_cfg->line_params)); line_idx += 1) {
         phone_cfg->line_params[line_idx].enable = 0;
      }
      phone_cfg->country = t->config.country;
      phone_cfg->pcm_use_16bits_timeslot = false;
      for (pvt = AST_LIST_FIRST(&(t->pvt_list)); (NULL != pvt); pvt = AST_LIST_NEXT(pvt, list)) {
         line_idx = pvt->index_line;
         bcm_assert(pvt->line_cfg->enable);
         phone_cfg->line_params[line_idx].enable = 1;

         if ((pvt->line_cfg->detect_dtmf & (DETECT_DTMF_BEFORE_CONNECTION | DETECT_DTMF_WHEN_CONNECTED))
             && (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), ast_format_slin))) {
            phone_cfg->line_params[line_idx].codec = BCMPH_CODEC_LINEAR;
            phone_cfg->pcm_use_16bits_timeslot = true;
         }
         else {
            size_t codec_idx;
            for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
               if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), bcmph_supported_codecs[codec_idx].ast_format)) {
                  phone_cfg->line_params[line_idx].codec = bcmph_supported_codecs[codec_idx].codec;
                  if (bcmph_supported_codecs[codec_idx].bytes_per_sample > 1) {
                     phone_cfg->pcm_use_16bits_timeslot = true;
                  }
                  break;
               }
            }
         }
         phone_cfg->line_params[line_idx].echo_cancel_tap_length =
            pvt->line_cfg->echo_cancel_tap_length;
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
         line_idx = pvt->index_line;
         bcm_assert(line_idx < ARRAY_LEN(mmap_rbs_location->rbs));
         if ((sizeof(bcm_ring_buf_desc_t) != mmap_rbs_location->rbs[line_idx].rx_ring_buf_desc_size)
             || (sizeof(bcm_ring_buf_desc_t) != mmap_rbs_location->rbs[line_idx].tx_ring_buf_desc_size)) {
            ast_log(AST_LOG_ERROR, "Structures shared with the driver are not of the expected size\n");
            ret = AST_MODULE_LOAD_FAILURE;
            break;
         }
         pvt->ast_channel.mmap.dev_rx_ring_buf_desc = (bcm_ring_buf_desc_t *)(t->mmap.dev_start_addr + mmap_rbs_location->rbs[line_idx].rx_ring_buf_desc_off);
         pvt->ast_channel.mmap.dev_tx_ring_buf_desc = (bcm_ring_buf_desc_t *)(t->mmap.dev_start_addr + mmap_rbs_location->rbs[line_idx].tx_ring_buf_desc_off);
         bcm_ring_buf_init(&(pvt->ast_channel.mmap.dev_rx_ring_buf),
            t->mmap.dev_start_addr + mmap_rbs_location->rbs[line_idx].rx_buffer_offset, mmap_rbs_location->rbs[line_idx].rx_buffer_size);
         bcm_ring_buf_init(&(pvt->ast_channel.mmap.dev_tx_ring_buf),
            t->mmap.dev_start_addr + mmap_rbs_location->rbs[line_idx].tx_buffer_offset, mmap_rbs_location->rbs[line_idx].tx_buffer_size);
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
      AST_LIST_TRAVERSE(&(t->pvt_list), pvt, list) {
         size_t codec_idx;
         bcm_phone_line_state_t *line_state;
         if (NULL != pvt->ast_channel.current_format) {
            bcmph_ao2_ref_format(pvt->ast_channel.current_format, -1);
            pvt->ast_channel.current_format = NULL;
         }
         line_idx = pvt->index_line;
         bcm_assert(line_idx < ARRAY_LEN(get_line_states->line_state));
         line_state = &(get_line_states->line_state[line_idx]);
         for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
            if (bcmph_supported_codecs[codec_idx].codec == line_state->codec) {
               pvt->ast_channel.current_format = bcmph_supported_codecs[codec_idx].ast_format;
               pvt->ast_channel.bytes_per_sample = bcmph_supported_codecs[codec_idx].bytes_per_sample;
               if (pvt->ast_channel.bytes_per_sample < 2) {
                  bcm_assert(1 == pvt->ast_channel.bytes_per_sample);
                  pvt->ast_channel.frame_size_read = (BCMPH_MAX_BUF / 2);
               }
               else {
                  pvt->ast_channel.frame_size_read = BCMPH_MAX_BUF;
               }
               break;
            }
         }
         bcm_assert(codec_idx < ARRAY_SIZE(bcmph_supported_codecs));
         if (NULL != pvt->ast_channel.current_format) {
            bcmph_ao2_ref_format(pvt->ast_channel.current_format, 1);
         }
         pvt->ast_channel.frame_size_read -= (pvt->ast_channel.frame_size_read % pvt->ast_channel.bytes_per_sample);
         pvt->ast_channel.bytes_not_written_len = 0;
         pvt->ast_channel.current_mode = line_state->mode;
         pvt->ast_channel.current_tone = line_state->tone;
         pvt->ast_channel.current_status = line_state->status;

         bcm_phone_line_state_reset(&(pvt->monitor.line_state), line_state->status, false,
            line_state->codec, line_state->mode, line_state->tone);

         if (BCMPH_STATUS_OFF_HOOK == pvt->ast_channel.current_status) {
            bcmph_set_new_state(pvt, BCMPH_ST_OFF_NO_SERVICE, BCMPH_EV_INIT);
         }
         else {
            bcmph_set_new_state(pvt, BCMPH_ST_ON_IDLE, BCMPH_EV_INIT);
         }
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
   bcmph_pvt_t *pvt = bcmph_ast_channel_tech_pvt(ast);

   if ((NULL == pvt) || (pvt->owner != ast)) {
      /*
       * As we unlink the line from its channel as soon as we detect phone
       * transitions from off hook to on hook, it's possible
       * Asterisk calls some functions (that call this one), passing
       * a channel with a pvt that's NULL.
       * If pvt is not NULL, we check that pvt is linked to this channel,
       * just in case
       */
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
static int bcmph_chan_call(struct ast_channel *ast,
#if (AST_VERSION < 110)
   char *addr,
#else /* (AST_VERSION >= 110) */
   const char *addr,
#endif /* (AST_VERSION >= 110)*/
   int timeout)
{
   int ret = -1;
   bcmph_pvt_t *pvt = bcmph_get_pvt(ast);

   bcm_pr_debug("%s(ast='%s', addr='%s', timeout=%d)\n", __func__,
      bcmph_ast_channel_name(ast), addr, (int)(timeout));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      do { /* Empty loop */
         /* If phone is off hook, call() is not allowed */
         if (BCMPH_STATUS_ON_HOOK != pvt->ast_channel.current_status) {
            ast_log(AST_LOG_NOTICE, "'%s' is busy\n", bcmph_ast_channel_name(ast));
            ast_setstate(ast, AST_STATE_BUSY);
            ast_queue_control(ast, AST_CONTROL_BUSY);
         }
         else {
            enum ast_channel_state ast_state = bcmph_ast_channel_state(ast);
            if ((AST_STATE_DOWN != ast_state) && (AST_STATE_RESERVED != ast_state)) {
               ast_log(AST_LOG_WARNING, "bcmph_chan_call() called on '%s', neither down nor reserved\n",
                  bcmph_ast_channel_name(ast));
            }
            else {
               char cid_name[64];
               char cid_number[64];

               /*
                bcmph_chan_call() can't be called if
                (BCMPH_ST_ON_IDLE == pvt->ast_channel.state) because
                pvt->owner is NULL, or if
                (BCMPH_ST_ON_RINGING == pvt->ast_channel.state)
                because bcmph_ast_channel_state(ast) is AST_STATE_RINGING
                So, as (BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)
                (BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state)
               */
               bcm_assert(BCMPH_ST_ON_PRE_RINGING == pvt->ast_channel.state);

               cid_name[0] = '\0';
               if ((!bcmph_ast_channel_connected(ast)->id.name.valid)
                   || (ast_strlen_zero(bcmph_ast_channel_connected(ast)->id.name.str))) {
                  ast_copy_string(cid_name, DEFAULT_CALLER_ID, sizeof(cid_name));
               }
               else {
                  ast_copy_string(cid_name, bcmph_ast_channel_connected(ast)->id.name.str, sizeof(cid_name));
               }

               cid_number[0] = '\0';
               if ((bcmph_ast_channel_connected(ast)->id.number.valid)
                   && (!ast_strlen_zero(bcmph_ast_channel_connected(ast)->id.number.str))) {
                  ast_copy_string(cid_number, bcmph_ast_channel_connected(ast)->id.number.str, sizeof(cid_number));
               }

               if (!bcmph_set_new_state_on_ringing(pvt, addr, cid_name, cid_number)) {
                  break;
               }

               ast_setstate(ast, AST_STATE_RINGING);
               ast_queue_control(ast, AST_CONTROL_RINGING);
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
   bcm_pr_debug("%s(ast='%s')\n", __func__, bcmph_ast_channel_name(ast));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      /* If phone not off hook, answer() is not allowed */
      if ((BCMPH_ST_OFF_TALKING != pvt->ast_channel.state)
          && (BCMPH_ST_OFF_WAITING_ANSWER != pvt->ast_channel.state)) {
         ast_log(AST_LOG_WARNING, "Channel '%s' can't answer now, because not off hook or not in the correct state\n", bcmph_ast_channel_name(ast));
      }
      else {
         /* We accept that answer() can be called if BCMPH_ST_OFF_TALKING == pvt->ast_channel.state */
         if (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state) {
            ret = bcmph_setup(pvt, bcmph_ast_channel_rawreadformat(pvt->owner), AST_STATE_UP);
            if (!ret) {
               bcmph_ast_channel_rings_set(pvt->owner, 0);
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

   bcm_pr_debug("%s(old='%s', new='%s')\n", __func__,
      bcmph_ast_channel_name(old), bcmph_ast_channel_name(new));

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
   bcm_pr_debug("%s(ast='%s', digit='%c')\n", __func__,
      bcmph_ast_channel_name(ast), (char)(digit));

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

   bcm_pr_debug("%s(ast='%s', digit='%c', duration=%u)\n", __func__,
      bcmph_ast_channel_name(ast), (char)(digit), (unsigned int)(duration));

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
      if ((BCMPH_ST_OFF_TALKING != pvt->ast_channel.state)
          && (BCMPH_ST_OFF_WAITING_ANSWER != pvt->ast_channel.state)) {
         ast_log(AST_LOG_WARNING, "Can't send a DTMF while not off hook or not in the correct state\n");
         ret = -1;
      }
      else {
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

   bcm_pr_debug("%s(ast='%s', condition=%d)\n", __func__,
      bcmph_ast_channel_name(ast), (int)(condition));

   if (NULL != pvt) {
#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      if ((BCMPH_ST_OFF_TALKING != pvt->ast_channel.state)
          && (BCMPH_ST_OFF_WAITING_ANSWER != pvt->ast_channel.state)) {
         ast_log(AST_LOG_WARNING, "Can't indicate a condition while not off hook or not in the correct state\n");
      }
      else {
         switch (condition) {
            case -1: {
               bcmph_unset_tone_generator(pvt);
               ret = 0;
               break;
            }
            case AST_CONTROL_CONGESTION: {
               if (NULL == bcmph_ast_channel_zone(ast)) {
                  bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_INVALID), 0);
                  ret = 0;
               }
               break;
            }
            case AST_CONTROL_BUSY: {
               if (NULL == bcmph_ast_channel_zone(ast)) {
                  bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_BUSY), 0);
                  ret = 0;
               }
               break;
            }
            case AST_CONTROL_RINGING: {
               if (NULL == bcmph_ast_channel_zone(ast)) {
                  bcmph_set_line_tone(pvt, bcm_phone_line_tone_code_index(BCMPH_TONE_RINGBACK), 0);
                  ret = 0;
               }
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
#if (AST_VERSION >= 110)
            case AST_CONTROL_PVT_CAUSE_CODE:
#endif /* (AST_VERSION >= 110) */
               {
               break;
            }
            default: {
               /* -1 lets asterisk generate the tone */
               ast_log(AST_LOG_WARNING, "Condition %d is not supported on channel '%s'\n", (int)(condition), bcmph_ast_channel_name(ast));
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

   /* bcm_pr_debug("%s(ast='%s')\n", __func__, bcmph_ast_channel_name(ast)); */

   if (NULL != pvt) {
      bool do_unlock = false;

#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      do { /* Empty loop */
         bool detect_dtmf;

         if ((BCMPH_ST_OFF_TALKING != pvt->ast_channel.state)
              && (BCMPH_ST_OFF_WAITING_ANSWER != pvt->ast_channel.state)) {
            /* Don't try to receive audio on-hook or not in the correct state */
            ast_log(AST_LOG_WARNING, "Trying to receive audio while not off hook or not in the correct state\n");
            break;
         }

         bcmph_mmap_lock(pvt->channel);
         do_unlock = true;

         detect_dtmf = (((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state) && ((pvt->line_cfg->detect_dtmf & DETECT_DTMF_WHEN_CONNECTED)))
            || ((BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state) && ((pvt->line_cfg->detect_dtmf & DETECT_DTMF_BEFORE_CONNECTION))));
         ret = bcmph_read_frame(pvt, detect_dtmf, &(do_unlock));
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

   /* bcm_pr_debug("%s(ast='%s')\n", __func__, bcmph_ast_channel_name(ast)); */

   if (NULL != pvt) {
      bool do_unlock = false;

#ifdef BCMPH_DEBUG
      pvt->owner_lock_count += 1;
      bcm_assert(pvt->owner_lock_count > 0);
#endif /* BCMPH_DEBUG */
      do { /* Empty loop */
         __u8 *pos;
         size_t todo;
         const bcmph_ast_format *format;

         /* Write a frame of (presumably voice) data */
         if (AST_FRAME_VOICE != frame->frametype) {
            ast_log(AST_LOG_WARNING, "Don't know what to do with frame type %d\n", (int)(frame->frametype));
            break;
         }

         if (frame->datalen <= 0) {
            bcm_pr_debug("Void frame\n");
            break;
         }

         format = bcmph_ast_get_frame_format(frame);

         if (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(pvt->line_cfg), format)) {
            ast_log(AST_LOG_WARNING, "Cannot handle frames in '%s' format\n", bcmph_ast_format_get_name(format));
            ret = -1;
            break;
         }

         if ((BCMPH_ST_OFF_TALKING != pvt->ast_channel.state)
              && (BCMPH_ST_OFF_WAITING_ANSWER != pvt->ast_channel.state)) {
            /* Don't try to send audio on-hook or not in the correct state */
            ast_log(AST_LOG_WARNING, "Trying to send audio while not off hook or not in the correct state\n");
            break;
         }

         if ((NULL == pvt->ast_channel.current_format)
             || (!bcmph_ast_formats_are_equal(pvt->ast_channel.current_format, format))) {
#ifdef BCMPH_DEBUG
            /*
            if (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state) {
               bcm_pr_debug("%s() : changing codec and switching to mode off talking, before call is answered\n", __func__);
            }
            */
#endif /* BCMPH_DEBUG */
            if (bcmph_set_line_codec(pvt, format, BCMPH_MODE_OFF_TALKING, 1, pvt->line_cfg->reverse_polarity, BCMPH_TONE_UNSPECIFIED)) {
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
         if ((BCMPH_ST_OFF_TALKING == pvt->ast_channel.state)
             || (BCMPH_ST_OFF_WAITING_ANSWER == pvt->ast_channel.state)) {
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
   bcmph_pvt_t *pvt = bcmph_ast_channel_tech_pvt(ast);

   bcm_pr_debug("%s(ast='%s')\n", __func__, bcmph_ast_channel_name(ast));

   if ((NULL == pvt) || (pvt->owner != ast)) {
      ast_log(AST_LOG_DEBUG, "Channel '%s' unlink or link to another line\n", bcmph_ast_channel_name(ast));
      if ((NULL != pvt) && (NULL == pvt->owner)) {
         /* Can happen if bcmph_chan_hangup() called while we are still in bcmph_new() */
         bcmph_ast_channel_tech_pvt_set(ast, NULL);
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
   bcm_pr_debug("'%s' hung up\n", bcmph_ast_channel_name(ast));
   ast_setstate(ast, AST_STATE_DOWN);

   return (0);
}

/******************************************************************************/

static struct ast_channel *bcmph_chan_request(const char *type,
#if (AST_VERSION < 110)
   format_t cap,
#else /* (AST_VERSION >= 110) */
   bcmph_ast_format_cap *cap,
#endif /* (AST_VERSION >= 110)*/
#if (AST_VERSION > 110)
   const struct ast_assigned_ids *assigned_ids,
#endif /* (AST_VERSION > 110) */
   const struct ast_channel *requestor,
#if (AST_VERSION < 110)
   void *addr,
#else /* (AST_VERSION >= 110) */
   const char *addr,
#endif /* (AST_VERSION >= 110)*/
   int *cause);

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
#if (AST_VERSION < 110)
   format_t cap,
#else /* (AST_VERSION >= 110) */
   bcmph_ast_format_cap *cap,
#endif /* (AST_VERSION >= 110) */
#if (AST_VERSION > 110)
   const struct ast_assigned_ids *assigned_ids,
#endif /* (AST_VERSION > 110) */
   const struct ast_channel *requestor,
#if (AST_VERSION < 110)
   void *_addr,
#else /* (AST_VERSION >= 110) */
   const char *addr,
#endif /* (AST_VERSION >= 110)*/
   int *cause)
{
   struct ast_channel *ret = NULL;
   bcmph_chan_t *t = &(bcmph_chan);
   bcmph_pvt_t *pvt;
#if (AST_VERSION < 110)
   const char *addr = _addr;
#endif /* (AST_VERSION < 110) */

   bcm_pr_debug("%s(type='%s', addr='%s')\n", __func__, type, addr);

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
#if (AST_VERSION < 110)
            if (bcmph_ast_format_cap_iscompatible_cap(&(cap), bcmph_get_line_cfg_cap(pvt->line_cfg))) {
#else /* (AST_VERSION >= 110) */
            if (bcmph_ast_format_cap_iscompatible_cap(cap, bcmph_get_line_cfg_cap(pvt->line_cfg))) {
#endif /* (AST_VERSION >= 110)*/
               if ((NULL == pvt->owner) && (BCMPH_STATUS_ON_HOOK == pvt->ast_channel.current_status)) {
                  bcm_assert(BCMPH_ST_ON_IDLE == pvt->ast_channel.state);
                  bcmph_new(pvt, BCMPH_ST_ON_PRE_RINGING, BCMPH_EV_AST_REQUEST,
#if (AST_VERSION <= 110)
                     ((NULL != requestor) ? bcmph_ast_channel_linkedid(requestor) : NULL),
#else /* (AST_VERSION > 110) */
                     assigned_ids, requestor,
#endif /* (AST_VERSION > 110) */
                     NULL);
                  ret = pvt->owner;
               }
               else {
                  bcm_pr_debug("Channel is busy\n");
                  *cause = AST_CAUSE_BUSY;
               }
            }
            else {
#if (AST_VERSION <= 110)
               char buf[256];
               ast_log(AST_LOG_WARNING, "Asked to get a channel of unsupported format '%s'\n", ast_getformatname_multiple(buf, sizeof(buf), cap));
#else /* (AST_VERSION > 110) */
               struct ast_str *buf = ast_str_alloca(256);
               ast_log(AST_LOG_WARNING, "Asked to get a channel of unsupported format '%s'\n", ast_format_cap_get_names(cap, &(buf)));
#endif /* (AST_VERSION > 110) */
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
   bcm_pr_debug("%s()\n", __func__);
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
         bcm_assert((NULL == pvt->owner) && (0 == pvt->owner_lock_count) &&
            ((BCMPH_ST_OFF_NO_SERVICE == pvt->ast_channel.state) || (BCMPH_ST_ON_IDLE == pvt->ast_channel.state)));
      }
   }
   bcmph_monitor_unlock(t);
}

static int __unload_module(void)
{
   int ret = 0;
   bcmph_chan_t *t = &(bcmph_chan);
   bcmph_pvt_t *p;
   size_t line_idx;
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
         bcmph_unset_sound_generator(pl);
         bcm_phone_line_state_deinit(&(pl->monitor.line_state));
         if (NULL != pl->ast_channel.current_format) {
            bcmph_ao2_ref_format(pl->ast_channel.current_format, -1);
            pl->ast_channel.current_format = NULL;
         }
         if (NULL != pl->ast_channel.dsp) {
            ast_dsp_free(pl->ast_channel.dsp);
            pl->ast_channel.dsp = NULL;
         }
         ast_free(pl);
      }

      /* We free structures allocated for the configuration */
      for (line_idx = 0; (line_idx < ARRAY_LEN(t->config.line_cfgs)); line_idx += 1) {
#if (AST_VERSION >= 110)
         if (NULL != t->config.line_cfgs[line_idx].capabilities) {
            bcmph_ast_format_cap_destroy(t->config.line_cfgs[line_idx].capabilities);
            t->config.line_cfgs[line_idx].capabilities = NULL;
         }
#endif /* (AST_VERSION >= 110) */
      }

#if (AST_VERSION >= 110)
      if (NULL != t->chan_tech.capabilities) {
         bcmph_ast_format_cap_destroy(t->chan_tech.capabilities);
         t->chan_tech.capabilities = NULL;
      }
#endif /* (AST_VERSION >= 110) */

      t->config.ring.cadence = NULL;
      t->config.ring.cadence_len = 0;
      if (NULL != t->config.tone_zone) {
         t->config.tone_zone = ast_tone_zone_unref(t->config.tone_zone);
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
      tmp = ast_malloc(sizeof(*tmp));
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
      tmp->ast_channel.current_status = tmp->monitor.line_state.status;
      tmp->ast_channel.current_mode = tmp->monitor.line_state.mode;
      tmp->ast_channel.current_tone = tmp->monitor.line_state.tone;
      if ((tmp->line_cfg->detect_dtmf)) {
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
      tmp->ast_channel.sound_generator.worker = NULL;
      tmp->ast_channel.sound_generator.post_process_fn = NULL;
      tmp->ast_channel.sound_generator.format = NULL;
      tmp->ast_channel.state = BCMPH_ST_DISCONNECTED;
      tmp->ast_channel.ring_stage = 0;
      /*
       Here it does not matter if line handles AST_FORMAT_ULAW or not, because
       the following fields are initialized in bcmph_open_device()
      */
      tmp->ast_channel.current_format = ast_format_ulaw;
      bcmph_ao2_ref_format(tmp->ast_channel.current_format, 1);
      tmp->ast_channel.bytes_per_sample = 1;
      if (tmp->ast_channel.bytes_per_sample < 2) {
         bcm_assert(1 == tmp->ast_channel.bytes_per_sample);
         tmp->ast_channel.frame_size_read = (BCMPH_MAX_BUF / 2);
      }
      else {
         tmp->ast_channel.frame_size_read = BCMPH_MAX_BUF;
      }
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
   size_t line_idx;

   bcmph_init_supported_codecs();

   bcmph_init_tones();

   memset(&(t->config), 0, sizeof(t->config));
   t->config.line_count = 0;
   t->config.country = 0;
   t->config.language = NULL;
   t->config.zone[0] = '\0';
   t->config.ring.cadence = bcmph_default_ring_cadence;
   t->config.ring.cadence_len = ARRAY_SIZE(bcmph_default_ring_cadence);
   t->config.tone_zone = NULL;
   bcmph_dual_tone_sequence_init(&(t->config.tone_dial), BCMPH_SAMPLE_RATE, NULL, 0, -1);
   bcmph_dual_tone_sequence_init(&(t->config.tone_busy), BCMPH_SAMPLE_RATE, NULL, 0, -1);
   bcmph_dual_tone_sequence_init(&(t->config.tone_congestion), BCMPH_SAMPLE_RATE, NULL, 0, -1);
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
#if (AST_VERSION < 110)
   t->chan_tech.capabilities = 0;
#else /* (AST_VERSION >= 110) */
   t->chan_tech.capabilities = NULL;
#endif /* (AST_VERSION >= 110)*/
   for (line_idx = 0; (line_idx < ARRAY_LEN(t->config.line_cfgs)); line_idx += 1) {
      bcmph_line_config_t *line_cfg = &(t->config.line_cfgs[line_idx]);
      line_cfg->enable = false;
#if (AST_VERSION < 110)
      line_cfg->capabilities = 0;
      bcmph_ast_format_cap_remove_by_type(bcmph_get_line_cfg_cap(line_cfg), AST_MEDIA_TYPE_UNKNOWN);
#else /* (AST_VERSION >= 110) */
      line_cfg->capabilities = NULL;
#endif /* (AST_VERSION >= 110)*/
      line_cfg->monitor_dialing = false;
      line_cfg->search_extension_trigger = '\0';
      line_cfg->dialing_timeout_1st_digit = 5000;
      line_cfg->dialing_timeout = 3000;
      line_cfg->detect_dtmf = 0;
      sprintf(line_cfg->context, "bcmph-line-%d", (int)(line_idx + 1));
      sprintf(line_cfg->cid_name, "line%d", (int)(line_idx + 1));
      sprintf(line_cfg->cid_num, "00-00-00-%02d", (int)(line_idx + 1));
      ast_copy_string(line_cfg->moh_interpret, "default", ARRAY_LEN(line_cfg->moh_interpret));
      line_cfg->cid_signalling = BCMPH_NO_CID;
      line_cfg->cid_start = BCMPH_CID_START_RING;
      line_cfg->reverse_polarity = 0;
      line_cfg->echo_cancel_tap_length = 0;
   }

   do { /* Empty loop */
      struct ast_variable *v;
      struct ast_flags config_flags = { 0 };
      size_t max_bytes_per_sample;

#if (AST_VERSION >= 110)
      t->chan_tech.capabilities = bcmph_ast_format_cap_alloc();
      if (NULL == t->chan_tech.capabilities) {
         ret = AST_MODULE_LOAD_DECLINE;
         break;
      }
#endif /* (AST_VERSION >= 110) */
      bcmph_ast_format_cap_remove_by_type(bcmph_get_chan_tech_cap(&(t->chan_tech)), AST_MEDIA_TYPE_UNKNOWN);
      /*
       * Formats are added when reading configuration of lines, as a union of
       * all formats supported by each line
      */

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
               ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section 'interfaces' of config file '%s'\n",
                  v->value, v->name, bcmph_cfg_file);
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }
            t->config.line_count = (size_t)(tmp);
         }
         else if (!strcasecmp(v->name, "country")) {
            char country[64];
            char *f;
            size_t country_idx;

            ast_copy_string(country, v->value, ARRAY_LEN(country));
            f = ast_strip(country);

            for (country_idx = 0; (country_idx < ARRAY_SIZE(bcmph_countries)); country_idx += 1) {
               if (!strcasecmp(f, bcmph_countries[country_idx].name)) {
                  t->config.language = bcmph_countries[country_idx].language;
                  if ((NULL != bcmph_countries[country_idx].default_zone)
                      && ('\0' == t->config.zone[0])) {
                     ast_copy_string(t->config.zone,
                        bcmph_countries[country_idx].default_zone,
                        ARRAY_LEN(t->config.zone));
                  }
                  t->config.country = bcmph_countries[country_idx].country;
                  break;
               }
            }
            if (country_idx >= ARRAY_SIZE(bcmph_countries)) {
               ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section 'interfaces' of config file '%s'\n",
                  f, v->name, bcmph_cfg_file);
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }
         }
         else if (!strcasecmp(v->name, "zone")) {
            char zone[64];
            char *f;

            ast_copy_string(zone, v->value, ARRAY_LEN(zone));
            f = ast_strip(zone);
            ast_copy_string(t->config.zone, f, ARRAY_LEN(t->config.zone));
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
      for (line_idx = 0; (line_idx < t->config.line_count); line_idx += 1) {
         char section[64];
         bcmph_line_config_t *line_cfg = &(t->config.line_cfgs[line_idx]);

         snprintf(section, ARRAY_LEN(section), "line%d", (int)(line_idx + 1));
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

               ast_copy_string(codecs, v->value, ARRAY_LEN(codecs));
               f = strtok_r(codecs, ",", &(save_ptr));
               while (NULL != f)
               {
                  const char *format_name = ast_strip(f);
                  size_t codec_idx;
                  for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
                     if (0 == strcasecmp(format_name, bcmph_ast_format_get_name(bcmph_supported_codecs[codec_idx].ast_format))) {
#if (AST_VERSION >= 110)
                        if (NULL == line_cfg->capabilities) {
                           line_cfg->capabilities = bcmph_ast_format_cap_alloc();
                           if (NULL == line_cfg->capabilities) {
                              break;
                           }
                           bcmph_ast_format_cap_remove_by_type(bcmph_get_line_cfg_cap(line_cfg), AST_MEDIA_TYPE_UNKNOWN);
                        }
#endif /* (AST_VERSION >= 110) */
                        bcmph_ast_format_cap_append_format(bcmph_get_line_cfg_cap(line_cfg), bcmph_supported_codecs[codec_idx].ast_format);
                        break;
                     }
                  }
                  if (codec_idx >= ARRAY_SIZE(bcmph_supported_codecs)) {
                     ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                        v->value, v->name, section, bcmph_cfg_file);
                     ret = AST_MODULE_LOAD_DECLINE;
                     break;
                  }
#if (AST_VERSION >= 110)
                  if (NULL == line_cfg->capabilities) {
                     ret = AST_MODULE_LOAD_DECLINE;
                     break;
                  }
#endif /* (AST_VERSION >= 110) */
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
            else if (!strcasecmp(v->name, "search_extension_trigger")) {
               char value[32];
               char *f;

               ast_copy_string(value, v->value, ARRAY_LEN(value));
               f = ast_strip(value);
               if (('\0' == f[0]) || ('\0' == f[1])) {
                  line_cfg->search_extension_trigger = f[0];
               }
               else {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                     f, v->name, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
            }
            else if (!strcasecmp(v->name, "dialing_timeout_1st_digit")) {
               int tmp;
               if ((1 != sscanf(v->value, " %10d ", &(tmp))) || (tmp < 0)) {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                     v->value, v->name, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               line_cfg->dialing_timeout_1st_digit = tmp;
            }
            else if (!strcasecmp(v->name, "dialing_timeout")) {
               int tmp;
               if ((1 != sscanf(v->value, " %10d ", &(tmp))) || (tmp < 0)) {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                     v->value, v->name, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               line_cfg->dialing_timeout = tmp;
            }
            else if (!strcasecmp(v->name, "detect_dtmf")) {
               char values[256];
               char *f;
               char *save_ptr = NULL;

               line_cfg->detect_dtmf = 0;
               ast_copy_string(values, v->value, ARRAY_LEN(values));
               f = strtok_r(values, ",", &(save_ptr));
               while (NULL != f)
               {
                  const char *val = ast_strip(f);
                  if (0 == strcasecmp(val, "always")) {
                     line_cfg->detect_dtmf |= DETECT_DTMF_WHEN_DIALING | DETECT_DTMF_BEFORE_CONNECTION | DETECT_DTMF_WHEN_CONNECTED;
                  }
                  else if (0 == strcasecmp(val, "dialing")) {
                     line_cfg->detect_dtmf |= DETECT_DTMF_WHEN_DIALING;
                  }
                  else if (0 == strcasecmp(val, "before_connection")) {
                     line_cfg->detect_dtmf |= DETECT_DTMF_BEFORE_CONNECTION;
                  }
                  else if (0 == strcasecmp(val, "connected")) {
                     line_cfg->detect_dtmf |= DETECT_DTMF_WHEN_CONNECTED;
                  }
                  else if (0 == strcasecmp(val, "never")) {
                     /* Ignored, for compatibility */
                  }
                  else {
                     ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                        v->value, v->name, section, bcmph_cfg_file);
                     ret = AST_MODULE_LOAD_DECLINE;
                     break;
                  }
                  f = strtok_r(NULL, ",", &(save_ptr));
               }
               if (AST_MODULE_LOAD_SUCCESS != ret) {
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
            else if (!strcasecmp(v->name, "cid_signalling")) {
               char cid_signalling[64];
               char *f;

               ast_copy_string(cid_signalling, v->value, ARRAY_LEN(cid_signalling));
               f = ast_strip(cid_signalling);
               if (!strcasecmp(f, "none")) {
                  line_cfg->cid_signalling = BCMPH_NO_CID;
               }
               else if (!strcasecmp(f, "bell")) {
                  line_cfg->cid_signalling = BCMPH_CID_SIG_BELL;
               }
               else if (!strcasecmp(f, "v23")) {
                  line_cfg->cid_signalling = BCMPH_CID_SIG_V23;
               }
               else if (!strcasecmp(f, "dtmf")) {
                  line_cfg->cid_signalling = BCMPH_CID_SIG_DTMF;
               }
               else {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                     f, v->name, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
            }
            else if (!strcasecmp(v->name, "cid_start")) {
               char cid_start[64];
               char *f;

               ast_copy_string(cid_start, v->value, ARRAY_LEN(cid_start));
               f = ast_strip(cid_start);
               if (!strcasecmp(f, "ring")) {
                  line_cfg->cid_start = BCMPH_CID_START_RING;
               }
               else if (!strcasecmp(f, "ring_pulse")) {
                  line_cfg->cid_start = BCMPH_CID_START_RING_PULSE;
               }
               else if (!strcasecmp(f, "polarity")) {
                  line_cfg->cid_start = BCMPH_CID_START_POLARITY;
               }
               else if (!strcasecmp(f, "dtmf")) {
                  line_cfg->cid_start = BCMPH_CID_START_DTMF;
               }
               else if (!strcasecmp(f, "polarity_and_dtmf")) {
                  line_cfg->cid_start = BCMPH_CID_START_POLARITY_DTMF;
               }
               else {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                     f, v->name, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
            }
            else if (!strcasecmp(v->name, "echo_cancel_tap_length")) {
               int tmp;
               if ((1 != sscanf(v->value, " %10d ", &(tmp)))
                   || (tmp < 0)
                   || ((0 != tmp) && (32 != tmp)
                       && (64 != tmp) && (128 != tmp)
                       && (256 != tmp) && (512 != tmp)
                       && (1024 != tmp))) {
                  ast_log(AST_LOG_ERROR, "Invalid value '%s' for variable '%s' in section '%s' of config file '%s'\n",
                     v->value, v->name, section, bcmph_cfg_file);
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               line_cfg->echo_cancel_tap_length = tmp;
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
            size_t codec_idx;
#if (AST_VERSION < 110)
            if (0 == line_cfg->capabilities) {
#else /* (AST_VERSION >= 110) */
            if (NULL == line_cfg->capabilities) {
#endif /* (AST_VERSION >= 110)*/
               ast_log(AST_LOG_ERROR, "No codec specified for line %lu in config file '%s'\n",
                  (unsigned long)(line_idx + 1), bcmph_cfg_file);
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }

            if (line_cfg->echo_cancel_tap_length > 0) {
               if (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_slin)) {
                  ast_log(AST_LOG_ERROR, "Line %lu does not supports codec slin, so echo cancellation is not possible\n",
                     (unsigned long)(line_idx + 1));
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               /* Remove codecs not compatible with echo canceller */
               for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
                  bcmph_ast_format *tmpfmt = bcmph_supported_codecs[codec_idx].ast_format;
                  if (!bcmph_ast_formats_are_equal(tmpfmt, ast_format_slin)) {
                     if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), tmpfmt)) {
                        bcmph_ast_format_cap_remove(bcmph_get_line_cfg_cap(line_cfg), tmpfmt);
                     }
                  }
               }
            }

            if (!line_cfg->monitor_dialing) {
               line_cfg->detect_dtmf &= (~(DETECT_DTMF_WHEN_DIALING));
            }
            if (line_cfg->detect_dtmf) {
               if ((!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_slin))
                   && (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_ulaw))
                   && (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_alaw))) {
                  ast_log(AST_LOG_ERROR, "Line %lu does not support codec ulaw, alaw or slin, so DTMF detection is not possible\n",
                     (unsigned long)(line_idx + 1));
                  ret = AST_MODULE_LOAD_DECLINE;
                  break;
               }
               if ((line_cfg->detect_dtmf & (DETECT_DTMF_BEFORE_CONNECTION | DETECT_DTMF_WHEN_CONNECTED))) {
                  /*
                   We check that line supports only codecs compatible
                   with DTMF detection, because if line supports an
                   incompatible codec, Asterisk could choose it and it
                   will prevent DTMF detection.
                  */
                  for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
                     const bcmph_ast_format *tmpfmt = bcmph_supported_codecs[codec_idx].ast_format;
                     if ((bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), tmpfmt))
                         && (!bcmph_ast_formats_are_equal(tmpfmt, ast_format_slin))
                         && (!bcmph_ast_formats_are_equal(tmpfmt, ast_format_ulaw))
                         && (!bcmph_ast_formats_are_equal(tmpfmt, ast_format_alaw))) {
                        break;
                     }
                  }
                  if (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)) {
                     ast_log(AST_LOG_ERROR, "Line %lu supports codec '%s', but it's not compatible with DTMF detection\n",
                        (unsigned long)(line_idx + 1), bcmph_ast_format_get_name(bcmph_supported_codecs[codec_idx].ast_format));
                     ret = AST_MODULE_LOAD_DECLINE;
                     break;
                  }
               }
            }

            if ((BCMPH_NO_CID != line_cfg->cid_signalling)
                && (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_slin))
                && (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_ulaw))
                && (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), ast_format_alaw))) {
               ast_log(AST_LOG_ERROR, "Line %lu does not supports codec ulaw, alaw or slin, so caller ID transmission is not possible\n",
                  (unsigned long)(line_idx + 1));
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }

            if ((BCMPH_CID_SIG_DTMF == line_cfg->cid_signalling)
                && ((BCMPH_CID_START_DTMF == line_cfg->cid_start)
                    || (BCMPH_CID_START_POLARITY_DTMF == line_cfg->cid_start))) {
               ast_log(AST_LOG_ERROR, "Line %lu, cid_signalling 'dtmf' can't be used with cid_start 'dtmf' or 'polarity_dtmf'\n",
                  (unsigned long)(line_idx + 1));
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }

            if (!line_cfg->context[0]) {
               ast_copy_string(line_cfg->context, "default", sizeof(line_cfg->context));
            }

            if (NULL == bcmph_add_pvt(t, line_idx)) {
               ret = AST_MODULE_LOAD_DECLINE;
               break;
            }

            for (codec_idx = 0; (codec_idx < ARRAY_SIZE(bcmph_supported_codecs)); codec_idx += 1) {
               if (bcmph_ast_format_cap_iscompatible_format(bcmph_get_line_cfg_cap(line_cfg), bcmph_supported_codecs[codec_idx].ast_format)) {
                  if (max_bytes_per_sample < bcmph_supported_codecs[codec_idx].bytes_per_sample) {
                     max_bytes_per_sample = bcmph_supported_codecs[codec_idx].bytes_per_sample;
                  }
                  if (!bcmph_ast_format_cap_iscompatible_format(bcmph_get_chan_tech_cap(&(t->chan_tech)), bcmph_supported_codecs[codec_idx].ast_format)) {
                     bcmph_ast_format_cap_append_format(bcmph_get_chan_tech_cap(&(t->chan_tech)), bcmph_supported_codecs[codec_idx].ast_format);
                  }
               }
            }
         }
      }
      if (AST_MODULE_LOAD_SUCCESS != ret) {
         break;
      }

      /*
       When at least one line is in conversation mode, as monitor thread is
       in charge of queuing data received, we set a period equal to the time
       needed to fill a frame (considering that we receive BCMPH_SAMPLES_PER_MS
       samples per ms)
      */
      if (max_bytes_per_sample > 2) {
         t->config.monitor_busy_period = (BCMPH_MAX_BUF / (max_bytes_per_sample * BCMPH_SAMPLES_PER_MS));
      }
      else {
         t->config.monitor_busy_period = (BCMPH_MAX_BUF / (2 * BCMPH_SAMPLES_PER_MS));
      }

      if (AST_LIST_EMPTY(&(t->pvt_list))) {
         ast_log(AST_LOG_ERROR, "No active line in config file '%s'\n", bcmph_cfg_file);
         ret = AST_MODULE_LOAD_DECLINE;
         break;
      }

      ast_config_destroy(cfg);
      cfg = CONFIG_STATUS_FILEINVALID;

      t->config.tone_zone = ast_get_indication_zone(t->config.zone);
      if (NULL == t->config.tone_zone) {
         t->config.tone_zone = ast_get_indication_zone(NULL);
      }
      if (NULL != t->config.tone_zone) {
         struct ast_tone_zone_sound *tone_sound;
         t->config.tone_zone = ast_tone_zone_ref(t->config.tone_zone);

         tone_sound = ast_get_indication_tone(t->config.tone_zone, "dial");
         if (NULL != tone_sound) {
            bcmph_dual_tone_sequence_init_from_ast_def(&(t->config.tone_dial), BCMPH_SAMPLE_RATE, tone_sound->data);
         }

         tone_sound = ast_get_indication_tone(t->config.tone_zone, "busy");
         if (NULL != tone_sound) {
            bcmph_dual_tone_sequence_init_from_ast_def(&(t->config.tone_busy), BCMPH_SAMPLE_RATE, tone_sound->data);
         }

         tone_sound = ast_get_indication_tone(t->config.tone_zone, "congestion");
         if (NULL != tone_sound) {
            bcmph_dual_tone_sequence_init_from_ast_def(&(t->config.tone_congestion), BCMPH_SAMPLE_RATE, tone_sound->data);
         }

         t->config.ring.cadence = t->config.tone_zone->ringcadence;
         /* We force ring.cadence_len to be even */
         t->config.ring.cadence_len = (t->config.tone_zone->nrringcadence >> 1) << 1;
      }
      if (t->config.tone_dial.part_count <= 0) {
         bcmph_dual_tone_sequence_init_from_ast_def(&(t->config.tone_dial), BCMPH_SAMPLE_RATE, "350+440");
         bcm_assert(t->config.tone_dial.part_count > 0);
      }
      if (t->config.tone_busy.part_count <= 0) {
         bcmph_dual_tone_sequence_init_from_ast_def(&(t->config.tone_busy), BCMPH_SAMPLE_RATE, "480+620/500,0/500");
         bcm_assert(t->config.tone_busy.part_count > 0);
      }
      if (t->config.tone_congestion.part_count <= 0) {
         bcmph_dual_tone_sequence_init_from_ast_def(&(t->config.tone_congestion), BCMPH_SAMPLE_RATE, "480+620/250,0/250");
         bcm_assert(t->config.tone_congestion.part_count > 0);
      }
      if ((NULL == t->config.ring.cadence) || (t->config.ring.cadence_len <= 0)) {
         t->config.ring.cadence = bcmph_default_ring_cadence;
         t->config.ring.cadence_len = ARRAY_SIZE(bcmph_default_ring_cadence);
      }

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
