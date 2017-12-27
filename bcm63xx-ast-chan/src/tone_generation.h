/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __TONE_GENERATION_H__
#define __TONE_GENERATION_H__

#include <math.h>
#include <stdlib.h>

/*
 http://web.archive.org/web/20140723015339/http://www.dattalo.com/technical/theory/sinewave.html

4. Trigonometric Identity 1
---------------------------

Suppose you need a consecutive sequence of evenly spaced sine values like:

sin(1), sin(1.1), sin(1.2), ..., sin(9.9)

Furthermore, suppose you don't feel like creating a table to store them. Consider the trigonometric identities:

sin(a+b) = sin(a)cos(b) + cos(a)sin(b)

cos(a+b) = cos(a)cos(b) - sin(a)sin(b)

We can define the starting angle as a and the step in angles as b, and precompute the following:

s1 = sin(a); c1 = cos(a);
sd = sin(b); cd = cos(b);

For the first iteration, we have

sin(a+b) = s1 * cd + c1 * sd;
cos(a+b) = c1 * cd - s1 * sd;

Now, let s1 = sin(a+b) and c1 = cos(a+b).

For the second iteration:

sin(a+2*b) = sin((a+b) + b) = sin(a+b)*cd + cos(a+b)*sd = s1 * sd + c1 * sd;
cos(a+2*b) = cos((a+b) + b) = cos(a+b)*cd + sin(a+b)*sd = c1 * cd - s1 * sd;

Again, let s1 = sin(a+2*b) and c1 = cos(a+2*b). The third and successive steps are similar. These steps can be collected into a simple program:

s1 = sin(a); c1 = cos(a);
sd = sin(b); cd = cos(b);
for(i=0;i<N;i++) {
  temp = s1 * cd + c1 * sd;
  c1 = c1 * cd - s1 * sd;
  s1 = temp;
}

Each iteration requires four multiplications and two additions. While this is a substantial savings in terms of computing sines from scratch, the method shown below can optimize this approach one step further and cut the computation in half.

In additon, notice that we have obtained the cosine values for free (a matter of perspective; could be a burden if you don't need them). There are cases when having both the sine and cosine values is handy (e.g. to draw a circle). However, the next algorithm can produce sines only or sines and cosines more efficiently then this one.
*
5. Trigonometric Identity 2: Goertzel Algorithm
-----------------------------------------------

The previous section required both sine and cosines to generate a series of equally spaced (in frequency) sine values. However, if you only want sines (or cosines) there is a more efficient way to obtain them.

Using the nomenclature of the previous section, we can define the starting angle as a and the step in angles as b. We wish to compute:

sin(a), sin(a + b), sin(a + 2*b), sin(a + 3*b), ..., sin(a + n*b)

The goal is to recursively compute sin(a+n*b) using the two previous values in the sine series, sin(a + (n-1)*b) and sin(a + (n-2)*b):

sin(a + n*b) = x * sin(a + (n-2)*b) + y * sin(a + (n-1)*b)

Here, x and y are two constants that need to be determined. Note that the next value or the n'th value depends on the previous two values, n-2 and n-1. Re-arranging and simplifying:

sin(a + n*b) = x * sin(a + n*b - 2*b) + y * sin(a + n*b - 1*b)
sin(a + n*b) = x * [ sin(a + n*b) * cos(2*b) - cos(a + n*b) * sin(2*b)] +
               y * [ sin(a + n*b) * cos(b) - cos(a + n*b) * sin(b)]
sin(a + n*b) =  [x * cos(2*b) + y * cos(b)] * sin(a + n*b) -
                [x * sin(2*b) + y * sin(b)] * cos(a + n*b)

For this to be true for all n, we must have the two expressions in brackets satisfy:

[x * cos(2*b) + y * cos(b)] = 1

[x * sin(2*b) + y * sin(b)] = 0

which, when solved yields

x = -1

y = 2*cos(b)

Finally, substituting into the original equation we get:

sin(a + n*b) = -sin(a + (n-2)*b) + 2*cos(b) * sin(a + (n-1)*b)

The fortuitous "x=-1" reduces a multiplication to a simple subtraction. Consequently, we only have to do one multiplication and one addition (subtraction) per iteration.

Here's a simple program to implement the algorithm:

c = 2* cos(b);
snm2 = sin(a + b);
snm1 = sin(a + 2*b);

for(i=0;i<N;i++) {
  s = c * snm1 - snm2;
  snm2 = snm1;
  snm1 = s;
}

Incidently, the cosine function has an identical recursive relationship:

cos(a + n*b) = -cos(a + (n-2)*b) + 2*cos(b) * cos(a + (n-1)*b)

Putting this into the program yields:

cb = 2* cos(b);
snm2 = sin(a + b);  snm1 = sin(a + 2*b);
cnm2 = cos(a + b);  cnm1 = cos(a + 2*b);
for(i=0;i<N;i++) {
  s = cb * snm1 - snm2;
  c = cb * cnm1 - cnm2;
  snm2 = snm1;  cnm2 = cnm1;
  snm1 = s;  cnm1 = c;
}

Two multiplications and two additions are required at each step. This is twice as fast as the previous algorithm.

When using this algorithm (or the previous one), be aware that round off errors will accumulate. This may not be too serious of a problem for a series of say 50 terms when floating point arithmetic is used. However, if we want 8 bit accuracy and start off with one bit error, the accumulated error rapidly deteriorates the useful dynamic range.
*/

#define LOG2_FACTOR_FLOAT_TO_INT 15

/*
 Tone generation using Goertzel algorithm (see comment above)
 Very efficient but can't be used for long period because round off
 errors accumulate
*/
typedef struct {
   int fac;
   int init_v;
   int init_v_minus_1;
} bcmph_goer_tone_def_t;

extern void bcmph_goer_tone_def_init(bcmph_goer_tone_def_t *t, float freq, float sample_freq);

typedef struct {
   const bcmph_goer_tone_def_t *def;
   int vol;
   int v;
   int v_minus_1;
   int v_minus_2;
} bcmph_goer_tone_t;

static inline void bcmph_goer_tone_reset(bcmph_goer_tone_t *t)
{
   t->v = t->def->init_v;
   t->v_minus_1 = t->def->init_v_minus_1;
   t->v_minus_2 = 0;
}

static inline void bcmph_goer_tone_init(bcmph_goer_tone_t *t, const bcmph_goer_tone_def_t *def, __s16 vol)
{
   t->def = def;
   t->vol = vol;
   bcmph_goer_tone_reset(t);
}

static inline int bcmph_goer_tone_next_val(bcmph_goer_tone_t *t)
{
   t->v_minus_2 = t->v_minus_1;
   t->v_minus_1 = t->v;
   t->v = ((t->def->fac * t->v_minus_1) >> LOG2_FACTOR_FLOAT_TO_INT) - t->v_minus_2;
   return ((t->v * t->vol) >> LOG2_FACTOR_FLOAT_TO_INT);
}

struct bcmph_sound_generator;

typedef struct  {
   void (*deinit)(struct bcmph_sound_generator *t);
   /*
    Return the minimal lenght of buffer needed for write() function :
    if buffer is below this lenght, write() could return 0 even if
    sound generation is not completed
   */
   size_t (*get_min_buffer_len)(const struct bcmph_sound_generator *t);
   /*
    Generate at most len samples of sound in buffer.
    Return the number of samples written in the buffer.
    Return 0 if sound generation is completed or len is below the
    minimal buffer len required for this generator
   */
   size_t (*write)(struct bcmph_sound_generator *t, __s16 *buffer, size_t len);
} bcmph_sound_generator_ops_t;

typedef struct bcmph_sound_generator {
   const bcmph_sound_generator_ops_t *vtbl;
} bcmph_sound_generator_t;

typedef struct {
   unsigned int freq1;
   unsigned int freq2;
   int modulation_percent;
   unsigned int duration;
   unsigned int midinote:1;
} bcmph_dual_tone_raw_def_t;

typedef struct {
   bcmph_goer_tone_def_t core1;
   bcmph_goer_tone_def_t core2;
   int modulation_percent;
   unsigned int duration;
} bcmph_dual_tone_def_t;

extern void bcmph_dual_tone_def_init(bcmph_dual_tone_def_t *t,
   const bcmph_dual_tone_raw_def_t *def, int sample_freq);

/*
 Must be sufficient to parse every busy, congestion or dial tone in
 Asterisk config file indications.conf
*/
#define MAX_PARTS_PER_PLAYTONE 7

typedef struct {
   int reppos;
   size_t part_count;
   int sample_freq;
   bcmph_dual_tone_def_t parts[MAX_PARTS_PER_PLAYTONE];
} bcmph_dual_tone_sequence_t;

extern void bcmph_dual_tone_sequence_init(bcmph_dual_tone_sequence_t *t,
   int sample_freq, const bcmph_dual_tone_raw_def_t *parts, size_t part_count, int reppos);

#ifdef AST_VERSION
extern void bcmph_dual_tone_sequence_init_from_ast_def(
   bcmph_dual_tone_sequence_t *t, int sample_freq, const char *def);
#endif /* AST_VERSION */

extern bcmph_dual_tone_sequence_t bcmph_dtmf_silence;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_0;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_1;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_2;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_3;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_4;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_5;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_6;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_7;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_8;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_9;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_aster;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_pound;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_A;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_B;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_C;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_D;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_dtas;
extern bcmph_dual_tone_sequence_t bcmph_dtmf_null;

extern const bcmph_dual_tone_sequence_t *bcmph_get_tone_sequence_for_dtmf(char tone);

typedef struct {
   bcmph_sound_generator_t sound_generator;
   const bcmph_dual_tone_sequence_t *def;
   bcmph_goer_tone_t core1;
   bcmph_goer_tone_t core2;
   int vol;
   size_t npos;
   size_t oldnpos;
   size_t pos;
} bcmph_dual_tone_generator_t;

extern void bcmph_dual_tone_generator_init(bcmph_dual_tone_generator_t *t,
   const bcmph_dual_tone_sequence_t *def, __s16 vol);

typedef struct {
   bcmph_sound_generator_t sound_generator;
   size_t max;
   size_t count;
} bcmph_silence_generator_t;

extern void bcmph_silence_generator_init(bcmph_silence_generator_t *t,
   size_t max);

/*
 Tone generation using trigonometric identity 1 (see comment above)
 and adapted to reduce round off errors.
 Depending upon frequencies is more or less accurate that Goertzel
 algorithm, but it's clearly less efficient
*/
typedef struct {
   int cosDelta;
   int sinDelta;
} bcmph_ident_tone_def_t;

extern void bcmph_ident_tone_def_init(bcmph_ident_tone_def_t *t,
   float freq, float sample_freq);

typedef struct {
   const bcmph_ident_tone_def_t *def;
   int vol;
   int cos;
   int sin;
} bcmph_ident_tone_t;

extern void bcmph_ident_tone_init(bcmph_ident_tone_t *t,
   const bcmph_ident_tone_def_t *def, __s16 vol);

extern int bcmph_ident_tone_next_val(bcmph_ident_tone_t *t);

typedef struct {
   div_t current;
   div_t inc;
   int divider;
} bcmph_fraction_seq_t;

static inline void bcmph_fraction_seq_init(bcmph_fraction_seq_t *t, int initial_value, int increment, int divider)
{
   t->divider = divider;
   t->current = div(initial_value, divider);
   t->inc = div(increment, divider);
}

static inline int bcmph_fraction_seq_get_val(bcmph_fraction_seq_t *t)
{
   return (t->current.quot);
}

static inline int bcmph_fraction_seq_next_val(bcmph_fraction_seq_t *t)
{
   t->current.quot += t->inc.quot;
   t->current.rem += t->inc.rem;
   if (t->current.rem >= t->divider) {
      t->current.quot += 1;
      t->current.rem -= t->divider;
   }
   return (bcmph_fraction_seq_get_val(t));
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
/**
 * The number of angle units per sine wave cycle.  In other words, the number
 * of angle units in a circle. Must be a power of two.
 */
#define BCMPH_LOG2_TRIGINT_ANGLES_PER_CYCLE 15
#define BCMPH_TRIGINT_ANGLES_PER_CYCLE (1 << BCMPH_LOG2_TRIGINT_ANGLES_PER_CYCLE)

/**
 * The maximum angle value, before rolling over to the next cycle.  Can
 * be used as a mask to keep the angle within the nominal values:
 *
 * @code
 * trigint_angle_t angle = ... ;
 * angle += phaseOffset;
 * angle &= BCMPH_TRIGINT_ANGLE_MAX;
 * @endcode
 */
#define BCMPH_TRIGINT_ANGLE_MAX (BCMPH_TRIGINT_ANGLES_PER_CYCLE - 1)

/**
 * A BCMPH_LOG2_TRIGINT_ANGLES_PER_CYCLE -bit angle.
 * If BCMPH_LOG2_TRIGINT_ANGLES_PER_CYCLE is 15, this divides the circle
 * into 32768 angle units, instead of the standard 360 degrees.
 * Thus:
 *   - 1 angle unit =~ 360/16384 =~ 0.01098632 degrees
 *   - 1 angle unit =~ 2*M_PI/16384 =~ 0.0001917 radians
 */
typedef __u16 bcmph_trigint_angle_t;

/**
 bcmph_trigint_sin16_table[] stores int instead of float
 in order to make all computations with integers
 This is the multiplier used to convert float value to int
*/
#define BCMPH_TRIGINT_LOG2_FACTOR_FLOAT_TO_INT 16

extern int bcmph_trigint_sin16(bcmph_trigint_angle_t angle);

static inline int bcmph_trigint_cos16(bcmph_trigint_angle_t angle)
{
   return (bcmph_trigint_sin16(angle + 3 * (BCMPH_TRIGINT_ANGLES_PER_CYCLE >> 2)));
}

/** <<<<< TRIGINT */

/*
 Tone generation using classic trigonometry but with integers
 It's clearly the less efficient but by far the most accurate
 and can be used indefinitely as rounding errors don't
 accumulate over time
*/
typedef struct {
   int frequency;
   int sample_freq;
} bcmph_trigint_tone_def_t;

static inline void bcmph_trigint_tone_def_init(bcmph_trigint_tone_def_t *t, int frequency, int sample_freq)
{
   t->frequency = frequency;
   t->sample_freq = sample_freq;
}

typedef struct {
   bcmph_fraction_seq_t counter;
   __s16 vol;
} bcmph_trigint_tone_t;

extern void bcmph_trigint_tone_init(bcmph_trigint_tone_t *t, const bcmph_trigint_tone_def_t *def, __s16 vol);

extern int bcmph_trigint_tone_next_val(bcmph_trigint_tone_t *t);

extern void bcmph_init_tones(void);

extern size_t bcmph_write_dtmf(char tone, __s16 *buffer, size_t len);

extern size_t bcmph_write_silence(__s16 *buffer, size_t len);


#endif /* __TONE_GENERATION_H__ */
