/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __CALLERID_H__
#define __CALLERID_H__

#include "tone_generation.h"

typedef enum {
   BCMPH_NO_CID, /* Do not use CallerID */
   BCMPH_CID_SIG_BELL, /* Use FSK modulation Telcordia GR-30 */
   BCMPH_CID_SIG_V23, /* Use FSK modulation V23 */
   BCMPH_CID_SIG_DTMF, /* Use DTMF */
} bcmph_cid_signalling_t;

typedef enum {
   BCMPH_CID_START_RING, /* CID information sent after first ring */
   BCMPH_CID_START_RING_PULSE, /* CID information sent after short ring */
   BCMPH_CID_START_POLARITY, /* CID information sent before first ring. Start is signalled by polarity reversal */
   BCMPH_CID_START_DTMF, /* CID information sent before first ring. Start is signalled by special DTMF */
   BCMPH_CID_START_POLARITY_DTMF, /* CID information sent before first ring. Start is signalled by polarity reversal followed by special DTMF */
} bcmph_cid_start_t;

enum {
   BCMPH_CID_UNKNOWN_NUMBER = 0x01,
   BCMPH_CID_PRIVATE_NUMBER = 0x02,
   BCMPH_CID_UNKNOWN_NAME = 0x04,
   BCMPH_CID_PRIVATE_NAME = 0x08,
};

extern size_t bcmph_callerid_generate(const char *name, const char *number,
   bcmph_cid_signalling_t cid_signalling, bcmph_cid_start_t cid_start,
   int flags, __s16 *buffer, size_t buffer_len);

typedef struct {
   int sample_rate;
   div_t current;
   div_t inc[2];
   bcmph_fraction_seq_t counter;
   __s16 vol;
} bcmph_fsk_modulator_t;

typedef struct bcmph_callerid_generator {
   bcmph_sound_generator_t sound_generator;
   __u8 msg[64];
   size_t msg_len;
   bool generate_dtmf;
   size_t silence_in_samples_before_dtas;
   size_t samples_dtas;
   size_t silence_in_samples_before_data;
   size_t silence_in_samples_after_data;
   bcmph_dual_tone_generator_t dual_tone_generator;
   union {
      struct {
         bcmph_fsk_modulator_t modulator;
         size_t seizure_bytes;
         size_t mark_bits_before;
         size_t mark_bits_after;
      } fsk;
      struct {
         size_t samples_remaining;
      } dtmf;
   } u;
   size_t (*write)(struct bcmph_callerid_generator *t, __s16 *buffer, size_t buffer_len);
   size_t pos;
} bcmph_callerid_generator_t;

extern void bcmph_callerid_generator_init(bcmph_callerid_generator_t *t,
   const char *name, const char *number,
   bcmph_cid_signalling_t cid_signalling, bcmph_cid_start_t cid_start,
   int flags);

#endif /* __CALLERID_H__ */
