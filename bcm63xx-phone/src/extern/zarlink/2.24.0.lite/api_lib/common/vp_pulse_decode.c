/** \file vp_pulse_decode.c
 * vp_pulse_decode.c
 *
 * This file contains the VP Pulse Decoder Module C code.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 10975 $
 * $LastChangedDate: 2013-05-21 18:04:34 -0500 (Tue, 21 May 2013) $
 */

#include "vp_pulse_decode.h"
#include "vp_debug.h"

#if defined (VP_CC_886_SERIES)
#include "vp886_api_int.h"

/* Prototypes for static internal functions */
static uint16
VpPulseDecodeStateIdle(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp);

static uint16
VpPulseDecodeStateOffhookPrequal(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp);

static uint16
VpPulseDecodeStateBreak(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp);

static uint16
VpPulseDecodeStateMake(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp);

static void
VpPulseDecodeResetDigit(
    VpPulseDecodeDataType *pPulseData);

static void
VpPulseDecodeReportDigit(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData);

static void
VpPulseDecodePushEvent(
    VpLineCtxType *pLineCtx,
    uint16 eventId,
    uint16 eventData,
    uint16 parmHandle);

static bool
VpPulseDecodeGetSpecs(
    VpLineCtxType *pLineCtx,
    VpOptionPulseType **pSpec1,
    VpOptionPulseType **pSpec2);


/**  VpPulseDecodeInit()
  Initializes or resets the pulse decoding data structure.
*/
void
VpPulseDecodeInit(
    VpPulseDecodeDataType *pPulseData)
{
    pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
    /* All other data is reset upon exiting the IDLE state */
    return;
}

/**  VpPulseDecodeRun()
  Processes an input into the pulse decoding state machine.

  The input and current timestamp will be processed based on the current state.
  Events may be generated, the state may be changed, and the current timestamp
  will be saved as the new prevTimestamp.

  return:
    Timer that the above layer should set to send back
    VP_PULSE_DECODE_INPUT_TIMER, in 0.5ms units. A value of 0 indicates that
    no timer is needed.
*/
uint16
VpPulseDecodeRun(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp)
{
    uint16 timer = 0;

    switch (pPulseData->state) {
        case VP_PULSE_DECODE_STATE_IDLE: {
            timer = VpPulseDecodeStateIdle(pLineCtx, pPulseData, input, timestamp);
            break;
        }
        case VP_PULSE_DECODE_STATE_OFFHOOK_PREQUAL: {
            timer = VpPulseDecodeStateOffhookPrequal(pLineCtx, pPulseData, input, timestamp);
            break;
        }
        case VP_PULSE_DECODE_STATE_BREAK: {
            timer = VpPulseDecodeStateBreak(pLineCtx, pPulseData, input, timestamp);
            break;
        }
        case VP_PULSE_DECODE_STATE_MAKE: {
            timer = VpPulseDecodeStateMake(pLineCtx, pPulseData, input, timestamp);
            break;
        }
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("VpPulseDecodeRun: invalid state %d", pPulseData->state));
            break;
    }

    pPulseData->prevTimestamp = timestamp;

    /* Scale timer back to 0.5ms timestamp units */
    return timer / 4;
}


/**  VpPulseDecodeStateIdle()
  Processes an input into the VP_PULSE_DECODE_STATE_IDLE state.

  The idle state covers two actual possible states:
   - Onhook, after the onhook-min time elapsed
   - Offhook, before any pulse activity starts

  return:
    Required timer in 0.5ms units
    Onhook timeout if an onhook is processed,
    offhook prequal timer if an offhook is processed and offHookMin != 0,
    0 otherwise.
*/
uint16
VpPulseDecodeStateIdle(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp)
{
    uint16 timer = 0;
    VpOptionPulseType *pSpec1, *pSpec2;
    uint16 onHookMin;
    uint16 offHookMin;

    /* Retrieve pulse parameters */
    if (!VpPulseDecodeGetSpecs(pLineCtx, &pSpec1, &pSpec2)) {
        return 0;
    }
    onHookMin = pSpec1->flashMax;
    #ifdef EXTENDED_FLASH_HOOK
    onHookMin = pSpec1->onHookMin;
    #endif /* EXTENDED_FLASH_HOOK */

    offHookMin = 0;
    #ifdef VP_ENABLE_OFFHOOK_MIN
    offHookMin = pSpec1->offHookMin;
    #endif

    switch (input) {
        case VP_PULSE_DECODE_INPUT_OFF_HOOK: {
            if (offHookMin == 0) {
                VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_OFF, 0, timestamp);
                /* Remain in the idle state, waiting for a break */
            } else {
                VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_PREQUAL, VP_HOOK_PREQUAL_START, timestamp);
                timer = offHookMin;
                pPulseData->state = VP_PULSE_DECODE_STATE_OFFHOOK_PREQUAL;
            }
            break;
        }
        case VP_PULSE_DECODE_INPUT_ON_HOOK: {
            VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_STARTPULSE, 0, timestamp);
            VpPulseDecodeResetDigit(pPulseData);
            timer = onHookMin;
            pPulseData->state = VP_PULSE_DECODE_STATE_BREAK;
            break;
        }
        case VP_PULSE_DECODE_INPUT_TIMER:
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("VpPulseDecodeStateIdle: invalid input %d", input));
            break;
    }

    return timer;
}

/**  VpPulseDecodeStateOffhookPrequal()
  Processes an input into the VP_PULSE_DECODE_STATE_OFFHOOK_PREQUAL state.

  This is the state where an offhook has been detected, and a HOOK_PREQUAL event
  has been generated, but we need to wait before generating a HOOK_OFF event.

  return:
    Required timer in 0.5ms units
    This state only leads back to Idle so the timer should always be 0.
*/
uint16
VpPulseDecodeStateOffhookPrequal(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp)
{
    uint16 timer = 0;

    switch (input) {
        case VP_PULSE_DECODE_INPUT_ON_HOOK: {
            VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_PREQUAL, VP_HOOK_PREQUAL_ABORT, timestamp);
            pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
            break;
        }
        case VP_PULSE_DECODE_INPUT_TIMER: {
            VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_OFF, 0, timestamp);
            pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
            break;
        }
        case VP_PULSE_DECODE_INPUT_OFF_HOOK:
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("VpPulseDecodeStateOffhookPrequal: invalid input %d", input));
            break;
    }

    return timer;
}

/**  VpPulseDecodeStateBreak()
  Processes an input into the VP_PULSE_DECODE_STATE_BREAK state.

  This is the state when the line is onhook and the onhook-min time has not
  expired yet.  This could mean the break period of a pulse digit, the middle
  of a hook flash, or the beginning of a true onhook.

  return:
    Required timer in 0.5ms units
    Interdigit timeout if an offhook is processed, 0 otherwise.
*/
uint16
VpPulseDecodeStateBreak(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp)
{
    uint16 timer = 0;
    uint32 elapsed;
    VpOptionPulseType *pSpec1, *pSpec2;
    uint16 onHookMin;

    /* Calculate elapsed time and scale to 0.125ms units to match the pulse
       parameters */
    elapsed = (uint16)(timestamp - pPulseData->prevTimestamp);
    elapsed *= 4;

    VP_HOOK(VpLineCtxType, pLineCtx, ("VpPulseDecodeStateBreak: elapsed %lu.%lums", elapsed / 8, (elapsed % 8)*125));

    /* Retrieve pulse parameters */
    if (!VpPulseDecodeGetSpecs(pLineCtx, &pSpec1, &pSpec2)) {
        return 0;
    }
    onHookMin = pSpec1->flashMax;
    #ifdef EXTENDED_FLASH_HOOK
    onHookMin = pSpec1->onHookMin;
    #endif /* EXTENDED_FLASH_HOOK */

    switch (input) {
        case VP_PULSE_DECODE_INPUT_OFF_HOOK: {
            /* Check for rare onhook->offhook case */
            if (elapsed >= onHookMin) {
                /* Onhook time passed and then the line went back offhook
                   before the onhook timer was serviced.  Generate full onhook
                   event, followed by offhook.  Subtract 1 from the onhook
                   timestamp for ordering */
                VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_ON, 0, timestamp - 1);
                VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_OFF, 0, timestamp);
                pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
                break;
            }

            /* Only check for flash and extd flash if no digits have been
               accumulated yet. */
            if (pPulseData->digitCount == 0) {
                #ifdef EXTENDED_FLASH_HOOK
                if (elapsed > pSpec1->flashMax) {
                    VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_EXTD_FLASH, 0, timestamp);
                    pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
                    break;
                }
                #endif /* EXTENDED_FLASH_HOOK */

                if (elapsed >= pSpec1->flashMin) {
                    VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_FLASH, 0, timestamp);
                    pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
                    break;
                }
            }

            /* Check the duration of the break period against the two pulse
               specs and invalidate if necessary */
            if (elapsed > pSpec1->breakMax || elapsed < pSpec1->breakMin) {
                pPulseData->digitValidSpec1 = FALSE;
            }
            if (elapsed > pSpec2->breakMax || elapsed < pSpec2->breakMin) {
                pPulseData->digitValidSpec2 = FALSE;
            }
            pPulseData->digitCount++;

            /* Offhook now, so set the timer for the interdigit time */
            timer = pSpec1->interDigitMin;
            pPulseData->state = VP_PULSE_DECODE_STATE_MAKE;
            break;
        }
        case VP_PULSE_DECODE_INPUT_TIMER: {
            /* Onhook min timer expired */
            VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_HOOK_ON, 0, timestamp);
            pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
            break;
        }
        case VP_PULSE_DECODE_INPUT_ON_HOOK:
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("VpPulseDecodeStateBreak: invalid input %d", input));
            break;
    }

    return timer;
}

/**  VpPulseDecodeStateMake()
  Processes an input into the VP_PULSE_DECODE_STATE_BREAK state.

  This is the state when the line is offhook during pulse activity.  This could
  be in the make period of a pulse digit or at the end of a digit before the
  interdigit time elapses.

  return:
    Required timer in 0.5ms units
    Onhook timeout if an onhook is processed, 0 otherwise.
*/
uint16
VpPulseDecodeStateMake(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData,
    VpPulseDecodeInputType input,
    uint16 timestamp)
{
    uint16 timer = 0;
    uint32 elapsed;
    VpOptionPulseType *pSpec1, *pSpec2;
    uint16 onHookMin;

    /* Calculate elapsed time and scale to 0.125ms units to match the pulse
       parameters */
    elapsed = (uint16)(timestamp - pPulseData->prevTimestamp);
    elapsed *= 4;

    VP_HOOK(VpLineCtxType, pLineCtx, ("VpPulseDecodeStateMake: elapsed %lu.%lums", elapsed / 8, (elapsed % 8)*125));

    /* Retrieve pulse parameters */
    if (!VpPulseDecodeGetSpecs(pLineCtx, &pSpec1, &pSpec2)) {
        return 0;
    }
    onHookMin = pSpec1->flashMax;
    #ifdef EXTENDED_FLASH_HOOK
    onHookMin = pSpec1->onHookMin;
    #endif /* EXTENDED_FLASH_HOOK */

    switch (input) {
        case VP_PULSE_DECODE_INPUT_ON_HOOK: {
            /* Check for rare digit->startpulse case */
            if (elapsed >= pSpec1->interDigitMin) {
                /* Interdigit time passed and then the line went back onhook
                   before the interdigit timer was serviced.  Generate digit
                   event, followed by startpulse. */
                VpPulseDecodeReportDigit(pLineCtx, pPulseData);
                VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_STARTPULSE, 0, timestamp);
                VpPulseDecodeResetDigit(pPulseData);
                timer = onHookMin;
                pPulseData->state = VP_PULSE_DECODE_STATE_BREAK;
                break;
            }

            /* Check the duration of the make period against the two pulse
               specs and invalidate if necessary */
            if (elapsed > pSpec1->makeMax || elapsed < pSpec1->makeMin) {
                pPulseData->digitValidSpec1 = FALSE;
            }
            if (elapsed > pSpec2->makeMax || elapsed < pSpec2->makeMin) {
                pPulseData->digitValidSpec2 = FALSE;
            }

            /* Onhook now, so set the timer for the onHookMin time */
            timer = onHookMin;
            pPulseData->state = VP_PULSE_DECODE_STATE_BREAK;
            break;
        }
        case VP_PULSE_DECODE_INPUT_TIMER: {
            /* Interdigit timer expired */
            VpPulseDecodeReportDigit(pLineCtx, pPulseData);
            pPulseData->state = VP_PULSE_DECODE_STATE_IDLE;
            break;
        }
        case VP_PULSE_DECODE_INPUT_OFF_HOOK:
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("VpPulseDecodeStateMake: invalid input %d", input));
            break;
    }

    return timer;
}

/**  VpPulseDecodeResetDigit()
  Resets digit count and per-spec validation flags.
*/
static void
VpPulseDecodeResetDigit(
    VpPulseDecodeDataType *pPulseData)
{
    pPulseData->digitCount = 0;
    pPulseData->digitValidSpec1 = TRUE;
    pPulseData->digitValidSpec2 = TRUE;
}

/**  VpPulseDecodeReportDigit()
  Encodes event data based on pulse count and parmHandle based on which spec
  the digit met, then pushes out the digit event.
*/
static void
VpPulseDecodeReportDigit(
    VpLineCtxType *pLineCtx,
    VpPulseDecodeDataType *pPulseData)
{
    uint16 parmHandle;
    uint16 eventData;

    eventData = pPulseData->digitCount;

    if (pPulseData->digitValidSpec1) {
        parmHandle = 0;
    } else if (pPulseData->digitValidSpec2) {
        parmHandle = 1;
    } else {
        parmHandle = 0;
        eventData = VP_DIG_NONE;
    }

    if (eventData == 0 || eventData > VP_DIG_15) {
        eventData = VP_DIG_NONE;
    }

    VpPulseDecodePushEvent(pLineCtx, VP_LINE_EVID_PULSE_DIG, eventData, parmHandle);
}

/**  VpPulseDecodePushEvent()
  Pushes an event to the upper device-specific layer.
*/
static void
VpPulseDecodePushEvent(
    VpLineCtxType *pLineCtx,
    uint16 eventId,
    uint16 eventData,
    uint16 parmHandle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;

    switch (pDevCtx->deviceType) {
#if defined (VP_CC_886_SERIES)
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES: {
            Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
            uint8 channelId = pLineObj->channelId;
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, eventId, eventData, parmHandle, FALSE);
            break;
        }
#endif /* defined (VP_CC_886_SERIES) */
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("VpPulseDecodePushEvent: invalid deviceType %d", pDevCtx->deviceType));
            break;
    }

    return;
}

/**  VpPulseDecodePushEvent()
  Retrieves pulse specifications from the upper device-specific layer.

  Note: If a device type supports only one set of specifications, pSpec2 should
  be set equal to pSpec1, never NULL.
*/
static bool
VpPulseDecodeGetSpecs(
    VpLineCtxType *pLineCtx,
    VpOptionPulseType **pSpec1,
    VpOptionPulseType **pSpec2)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;

    switch (pDevCtx->deviceType) {
#if defined (VP_CC_886_SERIES)
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES: {
            Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
            *pSpec1 = &pDevObj->options.pulse;
            *pSpec2 = &pDevObj->options.pulse2;
            break;
        }
#endif /* defined (VP_CC_886_SERIES) */
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("VpPulseDecodeGetSpecs: invalid deviceType %d", pDevCtx->deviceType));
            return FALSE;
    }

    return TRUE;
}

#endif

