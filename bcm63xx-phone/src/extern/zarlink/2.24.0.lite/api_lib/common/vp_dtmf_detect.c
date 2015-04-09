/** \file vp_dtmf_detect.c
 * vp_dtmf_detect.c
 *
 * This file contains the VP DTMF Detect Module C code.
 *
 * Goertzel algorithm implementation based on tone_detect.c by Stephen
 * Underwood:
 *              tone_detect.c - General telephony tone detection, and specific
 *                              detection of DTMF.
 *
 *              Copyright (C) 2001  Steve Underwood <steveu@coppice.org>
 *
 *              Despite my general liking of the GPL, I place this code in the
 *              public domain for the benefit of all mankind - even the slimy
 *              ones who might try to proprietize my work and use it to my
 *              detriment.
 *
 * Copyright (c) 2013, Microsemi Corporation
 *
 * $Revision: 10975 $
 * $LastChangedDate: 2013-05-21 18:04:34 -0500 (Tue, 21 May 2013) $
 */

#include "vp_api_cfg.h"

#if defined(VP886_INCLUDE_DTMF_DETECT)

#include "vp_dtmf_detect.h"
#include "vp_debug.h"

static VpDigitType DigitMap[4][4] = {
    {VP_DIG_1, VP_DIG_2, VP_DIG_3, VP_DIG_A},
    {VP_DIG_4, VP_DIG_5, VP_DIG_6, VP_DIG_B},
    {VP_DIG_7, VP_DIG_8, VP_DIG_9, VP_DIG_C},
    {VP_DIG_ASTER, VP_DIG_ZERO, VP_DIG_POUND, VP_DIG_D}
};

static void
VpDtmfDetectGoertzelUpdate(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    VpDtmfDetectGoertzelType *pGoertzel,
    int32 sample);

static uint32
VpDtmfDetectGoertzelEnergy(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    VpDtmfDetectGoertzelType *pGoertzel);


/**  VpDtmfDetectGoertzelUpdate()
  Update one Goertzel filter with a new sample.  This will update the filter
  variables as such:
    v1 = v2
    v2 = v3
    v3 = v2*coeff - v1 + sample
*/
void
VpDtmfDetectGoertzelUpdate(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    VpDtmfDetectGoertzelType *pGoertzel,
    int32 sample)
{
    int32 v1;

    if (pDtmfData->currentSample >= pGoertzel->size) {
        /* Ignore samples beyond the filter size */
        return;
    }

    sample >>= pGoertzel->scale;

    v1 = pGoertzel->v2;

    pGoertzel->v2 = pGoertzel->v3;

    pGoertzel->v3 = (pGoertzel->v2 * pGoertzel->coeff) >> 15;
    pGoertzel->v3 -= v1;
    pGoertzel->v3 += sample;
    
    /* Scale down the results if necessary to prevent overflow.  Remember each
       time we scale down so that the final energy calculation can be scaled
       correspondingly. */
    if (ABS(pGoertzel->v3) >= 32768) {
        pGoertzel->v3 >>= 1;
        pGoertzel->v2 >>= 1;
        pGoertzel->scale++;
    }
}


/**  VpDtmfDetectGoertzelEnergy()
  Compute the final magnitude-squared energy result of a Goertzel filter.
    energy = v3*v3 + v2*v2 - v2*v3*coeff
  The final result will be scaled down by 16 bits from what the full result
  (with no scaling during sampling) would be.  The downscaling performed during
  sampling will be reversed by subtracting it from this 16 bit shift.
*/
uint32
VpDtmfDetectGoertzelEnergy(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    VpDtmfDetectGoertzelType *pGoertzel)
{
    int32 t1;
    int32 t2;
    int32 t3;
    uint32 energy;
    int8 scale;

    /* Compute terms t3, t2, and t1 to simplify the operation to:
         t3 + t2 - t1
       Scale each term down by 2 bits to prevent overflow when adding them */
    scale = 2;
    t3 = (pGoertzel->v3 * pGoertzel->v3) >> 2;
    t2 = (pGoertzel->v2 * pGoertzel->v2) >> 2;
    t1 = (pGoertzel->v3 * pGoertzel->coeff) >> 15;
    t1 = (t1 * pGoertzel->v2) >> 2;

    if (t1 <= t3 + t2) {
        energy = t3 + t2 - t1;
    } else {
        energy = 0;
        return energy;
    }
    
    /* Scale down to apply gains without worrying about overflow */
    while (energy >= 0x10000) {
        energy >>= 1;
        scale++;
    }

    /* Normalize filters of different sizes.  Smaller filters produce
       smaller results.  This should normalize the different frequency results,
       and also results at different sampling rates since they use different
       filter sizes.
       Since we are calculating magnitude squared, the gain is also squared. */
    energy = (energy * VP_DTMF_NORMAL_BLOCKSIZE) / pGoertzel->size;
    energy = (energy * VP_DTMF_NORMAL_BLOCKSIZE) / pGoertzel->size;
    
    while (energy >= 0x10000) {
        energy >>= 1;
        scale++;
    }

    energy = (energy * pGoertzel->gainSqd) / 1000;

    /* Aim to shift all results down by 16 from the original value, minus the
       dynamic scaling value. 'scale' will be positive if we need to continue
       shifting down, or negative if we've shifted down too far already. */
    scale = VP_DTMF_NORMAL_ENERGY_SCALE - scale - (2 * pGoertzel->scale);

    /* Positive: shift down the rest of the way */
    if (scale > 0) {
        energy >>= scale;
    }

    /* Negative: shift up one bit at a time until we either reach the target
       scale or energy reaches the uint32 maximum. */
    while (scale < 0) {
        if (energy >= 0x80000000) {
            /* Maximum */
            energy = 0xFFFFFFFF;
            return energy;
        }
        energy <<= 1;
        scale++;
    }
    
    return energy;
}


/** VpDtmfDetectReset()
  Reset all of the Goertzel filters.  This should be done between each
  detection block.
*/
void
VpDtmfDetectReset(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    bool freshStart)
{
    uint8 i;
    for (i = 0; i < 4; i++) {
        pDtmfData->grtzlRow[i].v2 = 0;
        pDtmfData->grtzlRow[i].v3 = 0;
        pDtmfData->grtzlRow[i].scale = 0;
        pDtmfData->grtzlCol[i].v2 = 0;
        pDtmfData->grtzlCol[i].v3 = 0;
        pDtmfData->grtzlCol[i].scale = 0;
        pDtmfData->grtzl2ndRow[i].v2 = 0;
        pDtmfData->grtzl2ndRow[i].v3 = 0;
        pDtmfData->grtzl2ndRow[i].scale = 0;
        pDtmfData->grtzl2ndCol[i].v2 = 0;
        pDtmfData->grtzl2ndCol[i].v3 = 0;
        pDtmfData->grtzl2ndCol[i].scale = 0;
    }
    pDtmfData->grtzlLowGap.v2 = 0;
    pDtmfData->grtzlLowGap.v3 = 0;
    pDtmfData->grtzlLowGap.scale = 0;
    pDtmfData->grtzlMidGap.v2 = 0;
    pDtmfData->grtzlMidGap.v3 = 0;
    pDtmfData->grtzlMidGap.scale = 0;
    pDtmfData->grtzlHighGap.v2 = 0;
    pDtmfData->grtzlHighGap.v3 = 0;
    pDtmfData->grtzlHighGap.scale = 0;
    pDtmfData->currentSample = 0;
    pDtmfData->totalEnergy = 0;
    
    if (freshStart) {
        pDtmfData->prevHit = VP_DIG_NONE;
    }
}


/** VpDtmfDetectInit()
  Initialize Goertzel coefficients, block size, and other parameters.  This
  only needs to be performed once per VpDtmfDetectDataType() object unless the
  sampling rate or gain correction factors change.
*/
void
VpDtmfDetectInit(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData)
{
    VpDtmfDetectParamsType *pParams = &pDtmfData->params;

    switch (pParams->sampleRate) {
        /* Coefficients:
            32768 * 2 * cos(2 * PI * freq / sampleRate)
           The 32768 factor is to allow the coeff to be used without floating
           point, and will require a ">> 15" bitshift after each time it
           is multiplied with another number */
        case VP_DTMF_SAMPLE_RATE_4KHZ:
            /* Primary coefficients */
            pDtmfData->grtzlRow[0].coeff =  30028; /* 697 Hz */
            pDtmfData->grtzlRow[1].coeff =  23165; /* 770 Hz */
            pDtmfData->grtzlRow[2].coeff =  15099; /* 852 Hz */
            pDtmfData->grtzlRow[3].coeff =   6065; /* 941 Hz */
            pDtmfData->grtzlCol[0].coeff = -21131; /* 1209 Hz */
            pDtmfData->grtzlCol[1].coeff = -33005; /* 1336 Hz */
            pDtmfData->grtzlCol[2].coeff = -44637; /* 1477 Hz */
            pDtmfData->grtzlCol[3].coeff = -54944; /* 1633 Hz */

            /* Second harmonic coefficients */
            pDtmfData->grtzl2ndRow[0].coeff = -38020; /* 1394 Hz */
            pDtmfData->grtzl2ndRow[1].coeff = -49159; /* 1540 Hz */
            pDtmfData->grtzl2ndRow[2].coeff = -58579; /* 1704 Hz */
            pDtmfData->grtzl2ndRow[3].coeff = -64413; /* 1882 Hz */
            pDtmfData->grtzl2ndCol[0].coeff = -51910; /* 2418 Hz */
            pDtmfData->grtzl2ndCol[1].coeff = -32291; /* 2672 Hz */
            pDtmfData->grtzl2ndCol[2].coeff =  -4731; /* 2964 Hz */
            pDtmfData->grtzl2ndCol[3].coeff =  26593; /* 3266 Hz */

            /* Gap frequency detectors, used to reject signals just below the
               lowest column frequency, between the row and column ranges, and
               just above the highest column frequency. */
            pDtmfData->grtzlLowGap.coeff = 37261; /* 615 Hz */
            pDtmfData->grtzlMidGap.coeff = -7703; /* 1075 Hz */
            pDtmfData->grtzlHighGap.coeff = -62934; /* 1820 Hz */

            /* Individual filter sizes */
            pDtmfData->grtzlRow[0].size = pDtmfData->grtzl2ndRow[0].size = VP_DTMF_BLOCK_SIZE_4KHZ_697;
            pDtmfData->grtzlRow[1].size = pDtmfData->grtzl2ndRow[1].size = VP_DTMF_BLOCK_SIZE_4KHZ_770;
            pDtmfData->grtzlRow[2].size = pDtmfData->grtzl2ndRow[2].size = VP_DTMF_BLOCK_SIZE_4KHZ_852;
            pDtmfData->grtzlRow[3].size = pDtmfData->grtzl2ndRow[3].size = VP_DTMF_BLOCK_SIZE_4KHZ_941;
            pDtmfData->grtzlCol[0].size = pDtmfData->grtzl2ndCol[0].size = VP_DTMF_BLOCK_SIZE_4KHZ_1209;
            pDtmfData->grtzlCol[1].size = pDtmfData->grtzl2ndCol[1].size = VP_DTMF_BLOCK_SIZE_4KHZ_1336;
            pDtmfData->grtzlCol[2].size = pDtmfData->grtzl2ndCol[2].size = VP_DTMF_BLOCK_SIZE_4KHZ_1477;
            pDtmfData->grtzlCol[3].size = pDtmfData->grtzl2ndCol[3].size = VP_DTMF_BLOCK_SIZE_4KHZ_1633;
            pDtmfData->grtzlLowGap.size = VP_DTMF_BLOCK_SIZE_4KHZ_615;
            pDtmfData->grtzlMidGap.size = VP_DTMF_BLOCK_SIZE_4KHZ_1075;
            pDtmfData->grtzlHighGap.size = VP_DTMF_BLOCK_SIZE_4KHZ_1820;

            /* Total block size (max of filter sizes) */
            pDtmfData->blockSize = VP_DTMF_BLOCK_SIZE_4KHZ_MAX;
            
            break;
        case VP_DTMF_SAMPLE_RATE_8KHZ:
            /* Primary coefficients */
            pDtmfData->grtzlRow[0].coeff = 55959; /* 697 Hz */
            pDtmfData->grtzlRow[1].coeff = 53913; /* 770 Hz */
            pDtmfData->grtzlRow[2].coeff = 51403; /* 852 Hz */
            pDtmfData->grtzlRow[3].coeff = 48438; /* 941 Hz */
            pDtmfData->grtzlCol[0].coeff = 38145; /* 1209 Hz */
            pDtmfData->grtzlCol[1].coeff = 32649; /* 1336 Hz */
            pDtmfData->grtzlCol[2].coeff = 26169; /* 1477 Hz */
            pDtmfData->grtzlCol[3].coeff = 18630; /* 1633 Hz */

            /* Second harmonic coefficients */
            pDtmfData->grtzl2ndRow[0].coeff =  30028; /* 1394 Hz */
            pDtmfData->grtzl2ndRow[1].coeff =  23165; /* 1540 Hz */
            pDtmfData->grtzl2ndRow[2].coeff =  15099; /* 1704 Hz */
            pDtmfData->grtzl2ndRow[3].coeff =   6065; /* 1882 Hz */
            pDtmfData->grtzl2ndCol[0].coeff = -21131; /* 2418 Hz */
            pDtmfData->grtzl2ndCol[1].coeff = -33005; /* 2672 Hz */
            pDtmfData->grtzl2ndCol[2].coeff = -44637; /* 2964 Hz */
            pDtmfData->grtzl2ndCol[3].coeff = -54944; /* 3266 Hz */

            /* Gap frequency detectors, used to reject signals just below the
               lowest column frequency, between the row and column ranges, and
               just above the highest column frequency. */
            pDtmfData->grtzlLowGap.coeff = 58038; /* 615 Hz */
            pDtmfData->grtzlMidGap.coeff = 43532; /* 1075 Hz */
            pDtmfData->grtzlHighGap.coeff = 9234; /* 1820 Hz */

            /* Individual filter sizes */
            pDtmfData->grtzlRow[0].size = pDtmfData->grtzl2ndRow[0].size = VP_DTMF_BLOCK_SIZE_8KHZ_697;
            pDtmfData->grtzlRow[1].size = pDtmfData->grtzl2ndRow[1].size = VP_DTMF_BLOCK_SIZE_8KHZ_770;
            pDtmfData->grtzlRow[2].size = pDtmfData->grtzl2ndRow[2].size = VP_DTMF_BLOCK_SIZE_8KHZ_852;
            pDtmfData->grtzlRow[3].size = pDtmfData->grtzl2ndRow[3].size = VP_DTMF_BLOCK_SIZE_8KHZ_941;
            pDtmfData->grtzlCol[0].size = pDtmfData->grtzl2ndCol[0].size = VP_DTMF_BLOCK_SIZE_8KHZ_1209;
            pDtmfData->grtzlCol[1].size = pDtmfData->grtzl2ndCol[1].size = VP_DTMF_BLOCK_SIZE_8KHZ_1336;
            pDtmfData->grtzlCol[2].size = pDtmfData->grtzl2ndCol[2].size = VP_DTMF_BLOCK_SIZE_8KHZ_1477;
            pDtmfData->grtzlCol[3].size = pDtmfData->grtzl2ndCol[3].size = VP_DTMF_BLOCK_SIZE_8KHZ_1633;
            pDtmfData->grtzlLowGap.size = VP_DTMF_BLOCK_SIZE_8KHZ_615;
            pDtmfData->grtzlMidGap.size = VP_DTMF_BLOCK_SIZE_8KHZ_1075;
            pDtmfData->grtzlHighGap.size = VP_DTMF_BLOCK_SIZE_8KHZ_1820;

            /* Total block size (max of filter sizes) */
            pDtmfData->blockSize = VP_DTMF_BLOCK_SIZE_8KHZ_MAX;
            
            break;
        default:
            VP_WARNING(VpLineCtxType, pLineCtx, ("VpDtmfDetectInit: Invalid sampleRate %d",
                pParams->sampleRate));
            break;
    }
    
    /* Initialize device-specific gain factors */
    pDtmfData->grtzlRow[0].gainSqd = pParams->gainRow[0];
    pDtmfData->grtzlRow[1].gainSqd = pParams->gainRow[1];
    pDtmfData->grtzlRow[2].gainSqd = pParams->gainRow[2];
    pDtmfData->grtzlRow[3].gainSqd = pParams->gainRow[3];
    pDtmfData->grtzlCol[0].gainSqd = pParams->gainCol[0];
    pDtmfData->grtzlCol[1].gainSqd = pParams->gainCol[1];
    pDtmfData->grtzlCol[2].gainSqd = pParams->gainCol[2];
    pDtmfData->grtzlCol[3].gainSqd = pParams->gainCol[3];
    pDtmfData->grtzl2ndRow[0].gainSqd = pParams->gain2ndRow[0];
    pDtmfData->grtzl2ndRow[1].gainSqd = pParams->gain2ndRow[1];
    pDtmfData->grtzl2ndRow[2].gainSqd = pParams->gain2ndRow[2];
    pDtmfData->grtzl2ndRow[3].gainSqd = pParams->gain2ndRow[3];
    pDtmfData->grtzl2ndCol[0].gainSqd = pParams->gain2ndCol[0];
    pDtmfData->grtzl2ndCol[1].gainSqd = pParams->gain2ndCol[1];
    pDtmfData->grtzl2ndCol[2].gainSqd = pParams->gain2ndCol[2];
    pDtmfData->grtzl2ndCol[3].gainSqd = pParams->gain2ndCol[3];
    pDtmfData->grtzlLowGap.gainSqd = pParams->gainLowGap;
    pDtmfData->grtzlMidGap.gainSqd = pParams->gainMidGap;
    pDtmfData->grtzlHighGap.gainSqd = pParams->gainHighGap;
    
    pDtmfData->digitOutput = VP_DIG_NONE;
    VpDtmfDetectReset(pLineCtx, pDtmfData, TRUE);
}


/** VpDtmfDetectProcess()
  Processes a set of new samples.  This includes updating the Goertzel filters
  for each sample, calculating the energy at the end of each block, translating
  the filter results into a digit code, and weeding out false detections.
  
  Returns the current digit being detected.  During a digit on-time, this will
  return that digit value every time until the digit ends, not just when it is
  first detected.  When no digit is detected, the return value is VP_DIG_NONE.
  The upper level should generate a make or break event when this returned digit
  value is different from the previous value.
*/
VpDigitType
VpDtmfDetectProcess(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    int16 *pSamples,
    uint8 numSamples)
{
    VpDtmfDetectParamsType *pParams = &pDtmfData->params;
    uint8 i = 0;
    uint8 j = 0;
    uint32 energyRow[4];
    uint32 energyCol[4];
    uint32 peakEnergyRow = 0;
    uint32 peakEnergyCol = 0;
    uint32 energyLowGap = 0;
    uint32 energyMidGap = 0;
    uint32 energyHighGap = 0;
    uint32 harmonicRow = 0;
    uint32 harmonicCol = 0;
    uint8 bestRow = 0;
    uint8 bestCol = 0;
    VpDigitType hit = VP_DIG_NONE;
    
    /* Process samples until we run out */
    while (i < numSamples) {
        /* Process samples until we run out OR we reach the full block size */
        while (i < numSamples && pDtmfData->currentSample < pDtmfData->blockSize) {
            int32 sample = pSamples[i];

            /* Apply normalization factor */
            sample = (sample * pDtmfData->params.normalizeGain) / 1000;

            /* Update all of the Goertzel filters */
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlRow[0], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlRow[1], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlRow[2], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlRow[3], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlCol[0], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlCol[1], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlCol[2], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlCol[3], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndRow[0], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndRow[1], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndRow[2], sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndRow[3], sample);
            if (pParams->sampleRate != VP_DTMF_SAMPLE_RATE_4KHZ) {
                /* Can only check the column second harmonics at 8kHz sampling or
                   higher because the harmonics are all above 2kHz */
                VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndCol[0], sample);
                VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndCol[1], sample);
                VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndCol[2], sample);
                VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndCol[3], sample);
            }
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlLowGap, sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlMidGap, sample);
            VpDtmfDetectGoertzelUpdate(pLineCtx, pDtmfData, &pDtmfData->grtzlHighGap, sample);
            
            pDtmfData->totalEnergy += ((sample * sample) >> VP_DTMF_NORMAL_ENERGY_SCALE);

            i++;
            pDtmfData->currentSample++;
        }
        if (pDtmfData->currentSample < pDtmfData->blockSize) {
            /* Incomplete block, need more samples to continue */
            break;
        }

        /* We've reached the end of a block.  Process the Goertzel results */
        
        /* Find the peak row and peak column */
        energyRow[0] = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlRow[0]);
        bestRow = 0;
        energyCol[0] = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlCol[0]);
        bestCol = 0;
        for (j = 1; j < 4; j++) {
            energyRow[j] = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlRow[j]);
            if (energyRow[j] > energyRow[bestRow]) {
                bestRow = j;
            }
            energyCol[j] = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlCol[j]);
            if (energyCol[j] > energyCol[bestCol]) {
                bestCol = j;
            }
        }
        peakEnergyRow = energyRow[bestRow];
        peakEnergyCol = energyCol[bestCol];

        /* Calculate the energies of the harmonic and gap frequencies */
        harmonicRow = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndRow[bestRow]);
        if (pParams->sampleRate != VP_DTMF_SAMPLE_RATE_4KHZ) {
            harmonicCol = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzl2ndCol[bestCol]);
        }
        energyLowGap = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlLowGap);
        energyMidGap = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlMidGap);
        energyHighGap = VpDtmfDetectGoertzelEnergy(pLineCtx, pDtmfData, &pDtmfData->grtzlHighGap);
        /* If the peak frequencies detected are adjacent to the gaps, expect
           them to bleed over a little and reduce the gap levels by 1/2.  In
           particular, we saw problems with the mid gap check and the *,0,#,D
           digits. */
        if (bestRow == 0) {
            energyLowGap /= 2;
        }
        if (bestRow == 3) {
            energyMidGap /= 2;
        }
        if (bestCol == 0) {
            energyMidGap /= 2;
        }
        if (bestCol == 3) {
            energyHighGap /= 2;
        }
        
        /* Normalize the sum-of-squares total energy relative to block size. */
        pDtmfData->totalEnergy = (pDtmfData->totalEnergy * VP_DTMF_NORMAL_BLOCKSIZE) / pDtmfData->blockSize;
        
        /* Assume a hit by default.  Following checks can clear the hit. */
        hit = DigitMap[bestRow][bestCol];
        
        /* Test against basic threshold */
        if (peakEnergyRow < pParams->basicThreshold ||
            peakEnergyCol < pParams->basicThreshold)
        {
            hit = VP_DIG_NONE;
        }

        if (hit != VP_DIG_NONE || pDtmfData->prevHit != VP_DIG_NONE || pDtmfData->digitOutput != VP_DIG_NONE) {
            VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, (
                "Block finished:\n"
                "energyRows: %ld %ld %ld %ld\n"
                "energyCols: %ld %ld %ld %ld\n"
                "peakRow %d: %ld - peakCol %d: %ld\n"
                "energyLowGap %ld - energyMidGap %ld - energyHighGap %ld\n"
                "harmonicRow %ld - harmonicCol %ld - totalEnergy %lu\n"
                "prevHit %d, hit %d\n"
                "threshold %ld",
                energyRow[0], energyRow[1], energyRow[2], energyRow[3],
                energyCol[0], energyCol[1], energyCol[2], energyCol[3],
                bestRow, peakEnergyRow, bestCol, peakEnergyCol,
                energyLowGap, energyMidGap, energyHighGap,
                harmonicRow, harmonicCol, pDtmfData->totalEnergy,
                pDtmfData->prevHit, hit, pParams->basicThreshold
                ));
        }

        if (hit != VP_DIG_NONE) {
            /* Twist check */
            if ((peakEnergyRow * pParams->reverseTwistFactor) / 10 < peakEnergyCol ||
                (peakEnergyCol * pParams->normalTwistFactor) / 10 < peakEnergyRow)
            {
                hit = VP_DIG_NONE;
                VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed twist check"));
            }


            /* Relative peak check */
            for (j = 0; j < 4 && hit != VP_DIG_NONE; j++) {
                if (j != bestRow) {
                    if ((energyRow[j] * pParams->relPeakRowFactor) / 10 > peakEnergyRow) {
                        hit = VP_DIG_NONE;
                        VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed row relative peak check"));
                    }
                }
                if (j != bestCol) {
                    if ((energyCol[j] * pParams->relPeakColFactor) / 10 > peakEnergyCol) {
                        hit = VP_DIG_NONE;
                        VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed col relative peak check"));
                    }
                }
            }
        
            /* Also perform relative peak checks against the gap frequencies.
               Low gap is below the lowest row frequency, mid gap is between
               the row and column ranges, and high gap is above the highest
               row frequency. */
            if ((energyLowGap * pParams->relPeakRowFactor) / 10 > peakEnergyRow ||
                (energyMidGap * pParams->relPeakRowFactor) / 10 > peakEnergyRow)
            {
                hit = VP_DIG_NONE;
                VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed row gap check"));
            }
            if ((energyMidGap * pParams->relPeakColFactor) / 10 > peakEnergyCol ||
                (energyHighGap * pParams->relPeakColFactor) / 10 > peakEnergyCol)
            {
                hit = VP_DIG_NONE;
                VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed col gap check"));
            }


            /* Total energy check */
            if (peakEnergyRow + peakEnergyCol < (pDtmfData->totalEnergy * pParams->totalEnergyFactor) / 10) {
                hit = VP_DIG_NONE;
                VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed total energy check"));
            }


            /* Second harmonic check */
            /* The column frequencies for the digits 2, 6, and C are very near
               to the row 2nd harmonic frequencies, which means the row 2nd
               harmonic level will always be high for these digits, likely
               causing false rejections.  For these digits, subtract the peak
               column level from the row 2nd harmonic level. */
            if (hit == VP_DIG_2 || hit == VP_DIG_6 || hit == VP_DIG_C) {
                if (harmonicRow <= peakEnergyCol) {
                    harmonicRow = 0;
                } else {
                    harmonicRow -= peakEnergyCol;
                }
            }
        
            if (peakEnergyRow < (harmonicRow * pParams->secondHarmRowFactor) / 10) {
                hit = VP_DIG_NONE;
                VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed row second harmonic"));
            }

            /* Can only check the column second harmonics at 8kHz sampling or
               higher because the harmonics are all above 2kHz */
            if (pParams->sampleRate != VP_DTMF_SAMPLE_RATE_4KHZ) {
                if (peakEnergyCol < (harmonicCol * pParams->secondHarmColFactor) / 10) {
                    hit = VP_DIG_NONE;
                    VP_DTMF_DETAIL(VpLineCtxType, pLineCtx, ("failed column second harmonic"));
                }
            }
        }

        /* If two blocks in a row give the same result, update the output */
        if (hit == pDtmfData->prevHit) {
            pDtmfData->digitOutput = hit;
        }
        pDtmfData->prevHit = hit;

        /* Store the detected row and column energy so that the upper level can
           use them. */
        pDtmfData->peakEnergyRow = peakEnergyRow;
        pDtmfData->peakEnergyCol = peakEnergyCol;

        /* Reset the Goertzel filters and currentSample counter */
        VpDtmfDetectReset(pLineCtx, pDtmfData, FALSE);

    }
    
    return pDtmfData->digitOutput;
}

#endif /* defined(VP886_INCLUDE_DTMF_DETECT) */
