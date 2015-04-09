/** \file vp_pulse_decode.h
 * vp_pulse_decode.h
 *
 * Header file for the VP DTMF Detect Module c file.
 *
 * Copyright (c) 2013, Microsemi Corporation
 *
 * $Revision: 9734 $
 * $LastChangedDate: 2012-03-30 16:37:31 -0500 (Fri, 30 Mar 2012) $
 */

#ifndef VP_DTMF_DETECT_H
#define VP_DTMF_DETECT_H

#include "vp_api.h"
#include "vp_api_types.h"

/* Use different block sizes to get near-integer k for each target frequency,
   where k = (target / (sampleRate / blockSize)) */
#define VP_DTMF_BLOCK_SIZE_4KHZ_MAX     53
#define VP_DTMF_BLOCK_SIZE_4KHZ_697     52
#define VP_DTMF_BLOCK_SIZE_4KHZ_770     52
#define VP_DTMF_BLOCK_SIZE_4KHZ_852     52
#define VP_DTMF_BLOCK_SIZE_4KHZ_941     51
#define VP_DTMF_BLOCK_SIZE_4KHZ_1209    53
#define VP_DTMF_BLOCK_SIZE_4KHZ_1336    51
#define VP_DTMF_BLOCK_SIZE_4KHZ_1477    51
#define VP_DTMF_BLOCK_SIZE_4KHZ_1633    51
#define VP_DTMF_BLOCK_SIZE_4KHZ_615     52
#define VP_DTMF_BLOCK_SIZE_4KHZ_1075    52
#define VP_DTMF_BLOCK_SIZE_4KHZ_1820    53

#define VP_DTMF_BLOCK_SIZE_8KHZ_MAX     106
#define VP_DTMF_BLOCK_SIZE_8KHZ_697     103
#define VP_DTMF_BLOCK_SIZE_8KHZ_770     104
#define VP_DTMF_BLOCK_SIZE_8KHZ_852     103
#define VP_DTMF_BLOCK_SIZE_8KHZ_941     102
#define VP_DTMF_BLOCK_SIZE_8KHZ_1209    106
#define VP_DTMF_BLOCK_SIZE_8KHZ_1336    102
#define VP_DTMF_BLOCK_SIZE_8KHZ_1477    103
#define VP_DTMF_BLOCK_SIZE_8KHZ_1633    103
#define VP_DTMF_BLOCK_SIZE_8KHZ_615     104
#define VP_DTMF_BLOCK_SIZE_8KHZ_1075    104
#define VP_DTMF_BLOCK_SIZE_8KHZ_1820    105

/* Block size to normalize results against */
#define VP_DTMF_NORMAL_BLOCKSIZE        102

/* Scale energy results down by this many bits */
#define VP_DTMF_NORMAL_ENERGY_SCALE     16


#ifdef VP886_INCLUDE_DTMF_DETECT

void
VpDtmfDetectReset(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    bool freshStart);

void
VpDtmfDetectInit(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData);

VpDigitType
VpDtmfDetectProcess(
    VpLineCtxType *pLineCtx,
    VpDtmfDetectDataType *pDtmfData,
    int16 *pSamples,
    uint8 numSamples);

#endif /* VP886_INCLUDE_DTMF_DETECT */

#endif /* VP_DTMF_DETECT_H */
