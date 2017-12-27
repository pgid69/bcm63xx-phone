/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <ctype.h>
#include <linux/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <bcm63xx_phone.h>
#include <macros.h>
#ifdef AST_VERSION
#include <asterisk.h>
#include <asterisk/indications.h>
#endif /* AST_VERSION */


#include "logger.h"
#include "tone_generation.h"

static const int midi_tohz[128] = {
   8,     8,     9,     9,     10,    10,    11,    12,    12,    13,
   14,    15,    16,    17,    18,    19,    20,    21,    23,    24,
   25,    27,    29,    30,    32,    34,    36,    38,    41,    43,
   46,    48,    51,    55,    58,    61,    65,    69,    73,    77,
   82,    87,    92,    97,    103,   110,   116,   123,   130,   138,
   146,   155,   164,   174,   184,   195,   207,   220,   233,   246,
   261,   277,   293,   311,   329,   349,   369,   391,   415,   440,
   466,   493,   523,   554,   587,   622,   659,   698,   739,   783,
   830,   880,   932,   987,   1046,  1108,  1174,  1244,  1318,  1396,
   1479,  1567,  1661,  1760,  1864,  1975,  2093,  2217,  2349,  2489,
   2637,  2793,  2959,  3135,  3322,  3520,  3729,  3951,  4186,  4434,
   4698,  4978,  5274,  5587,  5919,  6271,  6644,  7040,  7458,  7902,
   8372,  8869,  9397,  9956,  10548, 11175, 11839, 12543
};

void bcmph_goer_tone_def_init(bcmph_goer_tone_def_t *t, float freq, float sample_freq)
{
   double angle = (2.0 * M_PI * freq) / sample_freq;
   t->fac = 2.0 * cos(angle) * ((double)(1 << LOG2_FACTOR_FLOAT_TO_INT));
   t->init_v = sin(-angle) * ((double)(1 << LOG2_FACTOR_FLOAT_TO_INT));
   t->init_v_minus_1 = sin(-(2.0 * angle)) * ((double)(1 << LOG2_FACTOR_FLOAT_TO_INT));
}

#ifdef AST_VERSION
static void bcmph_dual_tone_raw_def_init_from_tone_zone_part(
   bcmph_dual_tone_raw_def_t *t,
   const struct ast_tone_zone_part *tone_data)
{
   t->freq1 = tone_data->freq1;
   t->freq2 = tone_data->freq2;
   if (tone_data->modulate) {
      t->modulation_percent = 90;
   }
   else {
      t->modulation_percent = 0;
   }
   t->duration = tone_data->time;
   t->midinote = tone_data->midinote;
}
#endif /* AST_VERSION */

void bcmph_dual_tone_def_init(bcmph_dual_tone_def_t *t,
   const bcmph_dual_tone_raw_def_t *def, int sample_freq)
{
   unsigned int freq1;
   unsigned int freq2;
   if (def->midinote) {
      if (def->freq1 <= 127) {
         freq1 = midi_tohz[def->freq1];
      }
      else {
         freq1 = 0;
      }
      if (def->freq2 <= 127) {
         freq2 = midi_tohz[def->freq2];
      }
      else {
         freq2 = 0;
      }
   }
   else {
      freq1 = def->freq1;
      freq2 = def->freq2;
   }
   bcmph_goer_tone_def_init(&(t->core1), freq1, sample_freq);
   bcmph_goer_tone_def_init(&(t->core2), freq2, sample_freq);
   t->duration = def->duration;
   t->modulation_percent = def->modulation_percent;
}

void bcmph_dual_tone_sequence_init(bcmph_dual_tone_sequence_t *t,
   int sample_freq, const bcmph_dual_tone_raw_def_t *parts, size_t part_count,
   int reppos)
{
   size_t part_idx;

   bcm_assert(((NULL == parts) && (part_count <= 0))
      || ((NULL != parts) && (part_count > 0) && (part_count <= ARRAY_SIZE(t->parts))));
   t->sample_freq = sample_freq;
   t->part_count = part_count;
   t->reppos = reppos;
   for (part_idx = 0; (part_idx < part_count); part_idx += 1) {
      bcmph_dual_tone_def_init(&(t->parts[part_idx]), &(parts[part_idx]), sample_freq);
   }
}

#ifdef AST_VERSION
void bcmph_dual_tone_sequence_init_from_ast_def(
   bcmph_dual_tone_sequence_t *t, int sample_freq, const char *def)
{
   t->sample_freq = sample_freq;
   t->part_count = 0;
   t->reppos = -1;

   if ((NULL != def) && ('\0' != def[0])) {
      const char *separator;
      char *stringp = ast_strdupa(def);
      char *s;

      /* check if the data is separated with '|' or with ',' by default */
      if (strchr(def,'|')) {
         separator = "|";
      }
      else {
         separator = ",";
      }

      while ((s = strsep(&(stringp), separator)) && (!ast_strlen_zero(s))) {
         bcmph_dual_tone_raw_def_t raw_def;
         struct ast_tone_zone_part tone_data;

         s = ast_strip(s);
         if ('!' == s[0]) {
            s += 1;
         }
         else if (t->reppos < 0) {
            t->reppos = t->part_count;
         }

         memset(&(tone_data), 0, sizeof(tone_data));
         if (ast_tone_zone_part_parse(s, &(tone_data))) {
            bcm_pr_err("Ignoring part '%s' from tone defined by '%s' because its parsing failed\n",
               s, def);
            continue;
         }

         if (t->part_count >= ARRAY_SIZE(t->parts)) {
            bcm_pr_err("Tone defined by '%s' contains too many parts (max allowed %u)\n",
               def, (unsigned int)(ARRAY_SIZE(t->parts)));
            break;
         }

         bcmph_dual_tone_raw_def_init_from_tone_zone_part(&(raw_def), &(tone_data));

         bcmph_dual_tone_def_init(&(t->parts[t->part_count]), &(raw_def), sample_freq);
         t->part_count += 1;
      }
   }
}
#endif /* AST_VERSION */

static size_t bcmph_dual_tone_generator_get_min_buffer_len(
   const bcmph_sound_generator_t *_t)
{
   return (1);
}

static size_t bcmph_dual_tone_generator_write(bcmph_sound_generator_t *_t, __s16 *buffer, size_t len)
{
   bcmph_dual_tone_generator_t *t = container_of(_t, bcmph_dual_tone_generator_t, sound_generator);
   size_t ret = 0;

   /* bcm_pr_debug("%s(len=%lu)\n", __func__
      (unsigned long)(len)); */

   bcm_assert((NULL != buffer) && (len > 0));

   if (t->npos < t->def->part_count) {

      /* we need to prepare a frame with 16 * timelen samples as we're
       * generating SLIN audio */
      memset(buffer, 0, len);

      for (;;) {
         const bcmph_dual_tone_def_t *pi = &(t->def->parts[t->npos]);
         size_t sample_count;
         size_t x;
         size_t duration_in_samples;

         if (pi->duration > 0) {
            duration_in_samples = (pi->duration * t->def->sample_freq) / 1000;
         }
         else {
            duration_in_samples = 0;
         }

         if (t->oldnpos != t->npos) {
            /* Load new parameters */
            bcmph_goer_tone_init(&(t->core1), &(pi->core1), t->vol);
            bcmph_goer_tone_init(&(t->core2), &(pi->core2), t->vol);
            t->pos = 0;
            t->oldnpos = t->npos;
         }

         sample_count = len;
         if ((pi->duration > 0)
             && ((t->pos + sample_count) > duration_in_samples)) {
            sample_count = duration_in_samples - t->pos;
         }

         for (x = 0; (x < sample_count); x += 1) {
            int s1 = bcmph_goer_tone_next_val(&(t->core1));
            int s2 = bcmph_goer_tone_next_val(&(t->core2));
            int sample;
            if (pi->modulation_percent) {
               int p = s2 - 32768;
               if (p < 0) {
                  p = -p;
               }
               p = ((p * pi->modulation_percent) / 100) + 1;
               sample = (s1 * p) >> 15;
            } else {
               sample = s1 + s2;
            }
            *buffer = sample;
            buffer += 1;
         }

         t->pos += x;
         ret += x;
         len -= x;

         if ((pi->duration > 0)
             && (t->pos >= duration_in_samples)) { /* item finished? */
            t->npos += 1;
            if (t->npos >= t->def->part_count) { /* last item? */
               if (t->def->reppos >= 0) {     /* repeat set? */
                  t->npos = t->def->reppos;  /* redo from top */
               }
            }
         }

         if (len <= 0) {
            break;
         }
      }
   }

   return (ret);
}

static void bcmph_dual_tone_generator_deinit(bcmph_sound_generator_t *_t)
{
}

static bcmph_sound_generator_ops_t ops_dual_tone_generator = {
   .get_min_buffer_len = bcmph_dual_tone_generator_get_min_buffer_len,
   .write = bcmph_dual_tone_generator_write,
   .deinit = bcmph_dual_tone_generator_deinit,
};

void bcmph_dual_tone_generator_init(bcmph_dual_tone_generator_t *t,
   const bcmph_dual_tone_sequence_t *def, __s16 vol)
{
   t->sound_generator.vtbl = &(ops_dual_tone_generator);
   t->def = def;
   t->vol = vol;
   t->pos = 0;
   t->npos = 0;
   bcm_assert(t->def->part_count > 0);
   t->oldnpos = t->def->part_count;
}

size_t bcmph_write_silence(__s16 *buffer, size_t len)
{
   size_t buffer_idx;

   for (buffer_idx = 0; (buffer_idx < len); buffer_idx += 1) {
      *buffer = 0;
      buffer += 1;
   }

   return (len);
}

static size_t bcmph_silence_generator_get_min_buffer_len(
   const bcmph_sound_generator_t *_t)
{
   return (1);
}

static size_t bcmph_silence_generator_write(bcmph_sound_generator_t *_t, __s16 *buffer, size_t len)
{
   bcmph_silence_generator_t *t = container_of(_t, bcmph_silence_generator_t, sound_generator);
   size_t ret = 0;
   if (t->count < t->max) {
      size_t i_max = t->max - t->count;
      if (i_max > len) {
         i_max = len;
      }
      ret = bcmph_write_silence(buffer, i_max);
      t->count += ret;
   }
   return (ret);
}

static void bcmph_silence_generator_deinit(bcmph_sound_generator_t *_t)
{
}

static bcmph_sound_generator_ops_t ops_silence_generator = {
   .get_min_buffer_len = bcmph_silence_generator_get_min_buffer_len,
   .write = bcmph_silence_generator_write,
   .deinit = bcmph_silence_generator_deinit,
};

void bcmph_silence_generator_init(bcmph_silence_generator_t *t, size_t max)
{
   t->sound_generator.vtbl = &(ops_silence_generator);
   t->max = max;
   t->count = 0;
}

static bcmph_dual_tone_raw_def_t bcmph_dtmf_0_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 941,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_1_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 697,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_2_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 697,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_3_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 697,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_4_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 770,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_5_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 770,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_6_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 770,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_7_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 852,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_8_parts[] = {
   {
      .freq1 = 1336,
      .freq2 = 852,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_9_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 852,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_aster_parts[] = {
   {
      .freq1 = 1209,
      .freq2 = 941,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_pound_parts[] = {
   {
      .freq1 = 1477,
      .freq2 = 941,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_A_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 697,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_B_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 770,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_C_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 852,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_D_parts[] = {
   {
      .freq1 = 1633,
      .freq2 = 941,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_dtas_parts[] = {
   {
      .freq1 = 2130,
      .freq2 = 2750,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

static bcmph_dual_tone_raw_def_t bcmph_dtmf_null_parts[] = {
   {
      .freq1 = 0,
      .freq2 = 0,
      .duration = 0,
      .modulation_percent = 0,
      .midinote = 0,
   },
};

bcmph_dual_tone_sequence_t bcmph_dtmf_0 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_1 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_2 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_3 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_4 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_5 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_6 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_7 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_8 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_9 = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_aster = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_pound = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_A = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_B = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_C = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_D = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_dtas = {
   .part_count = 0
};
bcmph_dual_tone_sequence_t bcmph_dtmf_null = {
   .part_count = 0
};

void bcmph_init_tones(void)
{
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_0), BCMPH_SAMPLE_RATE, bcmph_dtmf_0_parts, ARRAY_SIZE(bcmph_dtmf_0_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_1), BCMPH_SAMPLE_RATE, bcmph_dtmf_1_parts, ARRAY_SIZE(bcmph_dtmf_1_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_2), BCMPH_SAMPLE_RATE, bcmph_dtmf_2_parts, ARRAY_SIZE(bcmph_dtmf_2_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_3), BCMPH_SAMPLE_RATE, bcmph_dtmf_3_parts, ARRAY_SIZE(bcmph_dtmf_3_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_4), BCMPH_SAMPLE_RATE, bcmph_dtmf_4_parts, ARRAY_SIZE(bcmph_dtmf_4_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_5), BCMPH_SAMPLE_RATE, bcmph_dtmf_5_parts, ARRAY_SIZE(bcmph_dtmf_5_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_6), BCMPH_SAMPLE_RATE, bcmph_dtmf_6_parts, ARRAY_SIZE(bcmph_dtmf_6_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_7), BCMPH_SAMPLE_RATE, bcmph_dtmf_7_parts, ARRAY_SIZE(bcmph_dtmf_7_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_8), BCMPH_SAMPLE_RATE, bcmph_dtmf_8_parts, ARRAY_SIZE(bcmph_dtmf_8_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_9), BCMPH_SAMPLE_RATE, bcmph_dtmf_9_parts, ARRAY_SIZE(bcmph_dtmf_9_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_aster), BCMPH_SAMPLE_RATE, bcmph_dtmf_aster_parts, ARRAY_SIZE(bcmph_dtmf_aster_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_pound), BCMPH_SAMPLE_RATE, bcmph_dtmf_pound_parts, ARRAY_SIZE(bcmph_dtmf_pound_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_A), BCMPH_SAMPLE_RATE, bcmph_dtmf_A_parts, ARRAY_SIZE(bcmph_dtmf_A_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_B), BCMPH_SAMPLE_RATE, bcmph_dtmf_B_parts, ARRAY_SIZE(bcmph_dtmf_B_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_C), BCMPH_SAMPLE_RATE, bcmph_dtmf_C_parts, ARRAY_SIZE(bcmph_dtmf_C_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_D), BCMPH_SAMPLE_RATE, bcmph_dtmf_D_parts, ARRAY_SIZE(bcmph_dtmf_D_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_dtas), BCMPH_SAMPLE_RATE, bcmph_dtmf_dtas_parts, ARRAY_SIZE(bcmph_dtmf_dtas_parts), -1);
   bcmph_dual_tone_sequence_init(&(bcmph_dtmf_null), BCMPH_SAMPLE_RATE, bcmph_dtmf_null_parts, ARRAY_SIZE(bcmph_dtmf_null_parts), -1);
}

const bcmph_dual_tone_sequence_t *bcmph_get_tone_sequence_for_dtmf(char tone)
{
   const bcmph_dual_tone_sequence_t *ret = &(bcmph_dtmf_null);

   switch (tone) {
      case '0': {
         ret = &(bcmph_dtmf_0);
         break;
      }
      case '1': {
         ret = &(bcmph_dtmf_1);
         break;
      }
      case '2': {
         ret = &(bcmph_dtmf_2);
         break;
      }
      case '3': {
         ret = &(bcmph_dtmf_3);
         break;
      }
      case '4': {
         ret = &(bcmph_dtmf_4);
         break;
      }
      case '5': {
         ret = &(bcmph_dtmf_5);
         break;
      }
      case '6': {
         ret = &(bcmph_dtmf_6);
         break;
      }
      case '7': {
         ret = &(bcmph_dtmf_7);
         break;
      }
      case '8': {
         ret = &(bcmph_dtmf_8);
         break;
      }
      case '9': {
         ret = &(bcmph_dtmf_9);
         break;
      }
      case '*': {
         ret = &(bcmph_dtmf_aster);
         break;
      }
      case '#': {
         ret = &(bcmph_dtmf_pound);
         break;
      }
      case 'a':
      case 'A': {
         ret = &(bcmph_dtmf_A);
         break;
      }
      case 'b':
      case 'B': {
         ret = &(bcmph_dtmf_B);
         break;
      }
      case 'c':
      case 'C': {
         ret = &(bcmph_dtmf_C);
         break;
      }
      case 'd':
      case 'D': {
         ret = &(bcmph_dtmf_D);
         break;
      }
      default: {
         bcm_assert(false);
         break;
      }
   }

   return (ret);
}

size_t bcmph_write_dtmf(char tone, __s16 *buffer, size_t len)
{
   static const __s16 vol = 7219; /* Values come from Asterisk code. Default to -8db. */
   size_t ret;
   bcmph_dual_tone_generator_t tone_generator;

   bcmph_dual_tone_generator_init(&(tone_generator), bcmph_get_tone_sequence_for_dtmf(tone), vol);
   ret = bcmph_dual_tone_generator_write(&(tone_generator.sound_generator), buffer, len);
   bcmph_dual_tone_generator_deinit(&(tone_generator.sound_generator));

   return (ret);
}

void bcmph_ident_tone_def_init(bcmph_ident_tone_def_t *t, float freq,
   float sample_freq)
{
   double angle = freq * 2.0 * M_PI / sample_freq;
   t->cosDelta = cos(angle) * ((double)(1 << LOG2_FACTOR_FLOAT_TO_INT));
   t->sinDelta = sin(angle) * ((double)(1 << LOG2_FACTOR_FLOAT_TO_INT));
}

void bcmph_ident_tone_init(bcmph_ident_tone_t *t,
   const bcmph_ident_tone_def_t *def, __s16 vol)
{
   t->def = def;
   t->vol = vol;
   t->cos = (1 << LOG2_FACTOR_FLOAT_TO_INT);
   t->sin = 0;
}

int bcmph_ident_tone_next_val(bcmph_ident_tone_t *t)
{
   int ret = ((t->sin * t->vol) >> LOG2_FACTOR_FLOAT_TO_INT);
   int temp = (((t->cos * t->def->cosDelta) - (t->sin * t->def->sinDelta)) >> LOG2_FACTOR_FLOAT_TO_INT);
   t->sin = (((t->sin * t->def->cosDelta) + (t->cos * t->def->sinDelta)) >> LOG2_FACTOR_FLOAT_TO_INT);
   t->cos = temp;
   /* Reduce round off error (code comes from Asterisk callerid.c) */
   temp = (2 << LOG2_FACTOR_FLOAT_TO_INT) - (((t->cos * t->cos) + (t->sin * t->sin)) >> LOG2_FACTOR_FLOAT_TO_INT);
   t->cos = ((temp * t->cos) >> LOG2_FACTOR_FLOAT_TO_INT);
   t->sin = ((temp * t->sin) >> LOG2_FACTOR_FLOAT_TO_INT);;
   return (ret);
}

/** >>>>> TRIGINT */

/* This code comes from trigint library https://bitbucket.org/ddribin/trigint/ */

/*
 * Copyright (c) 2010 Dave Dribin
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 How angle is decomposed :
 BCMPH_SINE_INDEX_WIDTH is the number of bits used in angle
 as an index into bcmph_trigint_sin16_table[] table.
 BCMPH_SINE_INTERP_WIDTH is the number of bits used in angle
 for computing interpolation
 */
#define BCMPH_SINE_INDEX_WIDTH 8
#define BCMPH_SINE_INTERP_WIDTH (BCMPH_LOG2_TRIGINT_ANGLES_PER_CYCLE - BCMPH_SINE_INDEX_WIDTH)

/*
 Table of the sin values converted to int.
 Size is + 1 to store the value of sin(360°),
 hence we're storing sin(0) <= sin(degrees) <= sin(360).
 This is needed when we compute sin(BCMPH_TRIGINT_ANGLE_MAX),
 because we used bcmph_trigint_sin16_table[BCMPH_TRIGINT_ANGLE_MAX >> BCMPH_SINE_INTERP_WIDTH]
 and bcmph_trigint_sin16_table[(BCMPH_TRIGINT_ANGLE_MAX >> BCMPH_SINE_INTERP_WIDTH) + 1]
 for interpolation
*/
static int bcmph_trigint_sin16_table[(1 << BCMPH_SINE_INDEX_WIDTH) + 1] = {
        0,   1608,   3216,   4821,   6424,   8022,   9616,  11204,
    12785,  14359,  15924,  17479,  19024,  20557,  22078,  23586,
    25080,  26558,  28020,  29466,  30893,  32303,  33692,  35062,
    36410,  37736,  39040,  40320,  41576,  42806,  44011,  45190,
    46341,  47464,  48559,  49624,  50660,  51665,  52639,  53581,
    54491,  55368,  56212,  57022,  57798,  58538,  59244,  59914,
    60547,  61145,  61705,  62228,  62714,  63162,  63572,  63944,
    64277,  64571,  64827,  65043,  65220,  65358,  65457,  65516,
    65536,  65516,  65457,  65358,  65220,  65043,  64827,  64571,
    64277,  63944,  63572,  63162,  62714,  62228,  61705,  61145,
    60547,  59914,  59244,  58538,  57798,  57022,  56212,  55368,
    54491,  53581,  52639,  51665,  50660,  49624,  48559,  47464,
    46341,  45190,  44011,  42806,  41576,  40320,  39040,  37736,
    36410,  35062,  33692,  32303,  30893,  29466,  28020,  26558,
    25080,  23586,  22078,  20557,  19024,  17479,  15924,  14359,
    12785,  11204,   9616,   8022,   6424,   4821,   3216,   1608,
        0,  -1608,  -3216,  -4821,  -6424,  -8022,  -9616, -11204,
   -12785, -14359, -15924, -17479, -19024, -20557, -22078, -23586,
   -25080, -26558, -28020, -29466, -30893, -32303, -33692, -35062,
   -36410, -37736, -39040, -40320, -41576, -42806, -44011, -45190,
   -46341, -47464, -48559, -49624, -50660, -51665, -52639, -53581,
   -54491, -55368, -56212, -57022, -57798, -58538, -59244, -59914,
   -60547, -61145, -61705, -62228, -62714, -63162, -63572, -63944,
   -64277, -64571, -64827, -65043, -65220, -65358, -65457, -65516,
   -65536, -65516, -65457, -65358, -65220, -65043, -64827, -64571,
   -64277, -63944, -63572, -63162, -62714, -62228, -61705, -61145,
   -60547, -59914, -59244, -58538, -57798, -57022, -56212, -55368,
   -54491, -53581, -52639, -51665, -50660, -49624, -48559, -47464,
   -46341, -45190, -44011, -42806, -41576, -40320, -39040, -37736,
   -36410, -35062, -33692, -32303, -30893, -29466, -28020, -26558,
   -25080, -23586, -22078, -20557, -19024, -17479, -15924, -14359,
   -12785, -11204,  -9616,  -8022,  -6424,  -4821,  -3216,  -1608,
        0
};

#define BITS(_VALUE_, _WIDTH_, _SHIFT_) (((_VALUE_) >> (_SHIFT_)) & ((1 << (_WIDTH_)) - 1))

int bcmph_trigint_sin16(bcmph_trigint_angle_t angle)
{
   int interp;
   __u16 index;
   int x1;
   int x2;
   int approximation;

   interp = angle & ((1 << BCMPH_SINE_INTERP_WIDTH) - 1);
   index = ((angle >> BCMPH_SINE_INTERP_WIDTH) & ((1 << BCMPH_SINE_INDEX_WIDTH) - 1));

   // Do calculations with 32 bits since the multiplication can overflow 16 bits
   x1 = bcmph_trigint_sin16_table[index];
   x2 = bcmph_trigint_sin16_table[index + 1];
   approximation = ((x2 - x1) * interp) >> BCMPH_SINE_INTERP_WIDTH;

   return (x1 + approximation);
}

/** <<<<< TRIGINT */

void bcmph_trigint_tone_init(bcmph_trigint_tone_t *t, const bcmph_trigint_tone_def_t *def, __s16 vol)
{
   t->vol = vol;
   bcmph_fraction_seq_init(&(t->counter), 0,
      def->frequency * BCMPH_TRIGINT_ANGLES_PER_CYCLE, def->sample_freq);
   t->counter.current.quot &= BCMPH_TRIGINT_ANGLE_MAX;
}

int bcmph_trigint_tone_next_val(bcmph_trigint_tone_t *t)
{
   int ret = bcmph_fraction_seq_get_val(&(t->counter));
   bcmph_fraction_seq_next_val(&(t->counter));
   t->counter.current.quot &= BCMPH_TRIGINT_ANGLE_MAX;
   return ((bcmph_trigint_sin16(ret) * t->vol) >> BCMPH_TRIGINT_LOG2_FACTOR_FLOAT_TO_INT);
}

#ifdef BCMPH_NOHW

static void bcmph_trigint_sin16_table_gen(void)
{
   size_t table_idx;
   char *sep = "";
   for (table_idx = 0; (table_idx < ARRAY_SIZE(bcmph_trigint_sin16_table)); table_idx += 1) {
      double radians = table_idx * 2 * M_PI / (ARRAY_SIZE(bcmph_trigint_sin16_table) - 1);
      double sinValue = ((double)(1 << BCMPH_TRIGINT_LOG2_FACTOR_FLOAT_TO_INT)) * sin(radians);
      int tableValue = round(sinValue);
      bcmph_trigint_sin16_table[table_idx] = tableValue;
      printf("%s%6d", sep, tableValue);
        if (((table_idx + 1) % 8) == 0) {
            sep = ",\n";
        } else {
            sep = ", ";
        }
   }
   printf("\n");
}

void do_test_tone(void)
{
   size_t f;
   static const int common_frequencies[] = { 440, 697, 770, 852, 941, 1200, 1209, 1336, 1477, 1633, 2130, 2200, 2750, 0 };
   static const int volume = 32767;

   bcmph_trigint_sin16_table_gen();

   for (f = 0; (f < ARRAY_SIZE(common_frequencies)); f += 1) {
      int frequency = common_frequencies[f];
      const double delta = (2 * M_PI * frequency) / BCMPH_SAMPLE_RATE;
      size_t i;
      size_t iMax;
      bcmph_goer_tone_def_t goer_tone_def;
      bcmph_goer_tone_t goer_tone;
      bcmph_ident_tone_def_t ident_tone_def;
      bcmph_ident_tone_t ident_tone;
      bcmph_trigint_tone_def_t trigint_tone_def;
      bcmph_trigint_tone_t trigint_tone;
      int max_delta_goer;
      int max_delta_ident;
      int max_delta_trigint;

      bcmph_goer_tone_def_init(&(goer_tone_def), frequency, BCMPH_SAMPLE_RATE);
      bcmph_goer_tone_init(&(goer_tone), &(goer_tone_def), volume);

      bcmph_ident_tone_def_init(&(ident_tone_def), frequency, BCMPH_SAMPLE_RATE);
      bcmph_ident_tone_init(&(ident_tone), &(ident_tone_def), volume);

      bcmph_trigint_tone_def_init(&(trigint_tone_def), frequency, BCMPH_SAMPLE_RATE);
      bcmph_trigint_tone_init(&(trigint_tone), &(trigint_tone_def), volume);

      max_delta_goer = 0;
      max_delta_ident = 0;
      max_delta_trigint = 0;
      iMax = (30000 * BCMPH_SAMPLE_RATE) / 1000;
      for (i = 0; (i < iMax); i += 1) {
         int goer = bcmph_goer_tone_next_val(&(goer_tone));
         int ident = bcmph_ident_tone_next_val(&(ident_tone));
         int trigint = bcmph_trigint_tone_next_val(&(trigint_tone));;
         int ref = sin(delta * i) * volume;
         int d;
         d = abs(ref - goer);
         if (max_delta_goer < d) {
            max_delta_goer = d;
         }
         d = abs(ref - ident);
         if (max_delta_ident < d) {
            max_delta_ident = d;
         }
         d = abs(ref - trigint);
         if (max_delta_trigint < d) {
            max_delta_trigint = d;
         }
      }
      printf("frequency = %d, volume = %d, max_delta_goer = %d (%g %%), max_delta_ident = %d (%g %%), max_delta_trigint = %d (%g %%)\n",
         frequency, volume,
         max_delta_goer, ((double)(max_delta_goer * 100)) / ((double)(volume)),
         max_delta_ident, ((double)(max_delta_ident * 100)) / ((double)(volume)),
         max_delta_trigint, ((double)(max_delta_trigint * 100)) / ((double)(volume)));
   }
}

#endif /* BCMPH_NOHW */
