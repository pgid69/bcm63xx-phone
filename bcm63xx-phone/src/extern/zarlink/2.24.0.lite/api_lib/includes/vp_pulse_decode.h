/** \file vp_pulse_decode.h
 * vp_pulse_decode.h
 *
 * Header file for the VP Pulse Decoder Module c file.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9734 $
 * $LastChangedDate: 2012-03-30 16:37:31 -0500 (Fri, 30 Mar 2012) $
 */

#ifndef VP_PULSE_DECODE_H
#define VP_PULSE_DECODE_H

#include "vp_api.h"
#include "vp_api_types.h"
#include "sys_service.h"

EXTERN void
VpPulseDecodeInit(
    VpPulseDecodeDataType *pPulseData);

EXTERN uint16
VpPulseDecodeRun(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp);

#endif /* VP_PULSE_DECODE_H */

