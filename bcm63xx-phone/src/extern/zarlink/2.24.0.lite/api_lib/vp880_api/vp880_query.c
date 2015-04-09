/** \file vp880_query.c
 * vp880_query.c
 *
 *  This file contains the query functions used in the Vp880 device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11380 $
 * $LastChangedDate: 2014-04-10 12:55:59 -0500 (Thu, 10 Apr 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES)

/* Project Includes */
#include "vp_api_types.h"
#include "sys_service.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"

#ifdef VP880_INCLUDE_TESTLINE_CODE
#include "vp_api_test.h"
#endif

/* Private Functions */
static uint16 Vp880CheckLineEvent(uint16 event, uint16 eventMask,
    VpEventCategoryType eventCat, Vp880LineObjectType *pLineObj);
static uint16 Vp880CheckDevEvent(uint16 event, uint16 eventMask,
    VpEventCategoryType eventCat, Vp880DeviceObjectType *pDevObj);

#ifdef VP880_ABS_SUPPORT
static void Vp880DevRingExitTimerHandler(VpDevCtxType *pDevCtx);
#endif

#ifdef VP880_FXS_SUPPORT
static void Vp880ServiceFxsTimers(VpLineCtxType *pLineCtx);

#ifdef VP880_LP_SUPPORT
static void Vp880UpdateHookInfo(Vp880LineObjectType *pLineObj, Vp880DeviceObjectType *pDevObj);
static void Vp880ServiceLpChangeTimer(VpDevCtxType *pDevCtx);
#endif /* VP880_LP_SUPPORT */

static void Vp880ServiceGroundStartTimer(VpLineCtxType *pLineCtx);
#endif /* VP880_FXS_SUPPORT */

#ifdef VP880_FXO_SUPPORT
static void Vp880ServiceFxoTimers(VpLineCtxType *pLineCtx);
#endif /* VP880_FXO_SUPPORT */

static VpStatusType
Vp880GetDeviceOption(VpDevCtxType *pDevCtx, VpOptionIdType option,
    uint16 handle);

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
static void
VpPrint880CalLineData(
    Vp880CalLineData *calLineData);
#endif

/**
 * Vp880FindSoftwareInterrupts()
 *  This function checks for active non-masked device and line events.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  Returns true if there is an active, non-masked event on either the device
 * or on a line associated with the device.
 */
bool
Vp880FindSoftwareInterrupts(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj;
    VpLineCtxType *pLineCtx;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

    VpOptionEventMaskType eventsMask = pDevObj->deviceEventsMask;
    VpOptionEventMaskType *pEvents = &(pDevObj->deviceEvents);

    /* First clear all device events that are masked */
    pEvents->faults &= ~(eventsMask.faults);
    pEvents->signaling &= ~(eventsMask.signaling);
    pEvents->response &= ~(eventsMask.response);
    pEvents->process &= ~(eventsMask.process);
    pEvents->test &= ~(eventsMask.test);
    pEvents->fxo &= ~(eventsMask.fxo);

    /* Evaluate if any events remain */
    if((pEvents->faults) || (pEvents->signaling) || (pEvents->response)
    || (pEvents->process) || (pEvents->test) || (pEvents->fxo)) {
        return TRUE;
    }

    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if(pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            eventsMask = pLineObj->lineEventsMask;
            pEvents = &(pLineObj->lineEvents);

            /* Clear the line events that are masked */
            pEvents->faults &= ~(eventsMask.faults);
            pEvents->signaling &= ~(eventsMask.signaling);
            pEvents->response &= ~(eventsMask.response);
            pEvents->process &= ~(eventsMask.process);
            pEvents->test &= ~(eventsMask.test);
            pEvents->fxo &= ~(eventsMask.fxo);

            /* Evaluate if any events remain */
            if(pEvents->faults || pEvents->signaling || pEvents->response
            || pEvents->process || pEvents->test || pEvents->fxo) {
                return TRUE;
            }
        }
    }

    return FALSE;
}

/**
 * Vp880GetEvent()
 *  This function reports new events that occured on the device. This function
 * returns one event for each call to it. It should be called repeatedly until
 * no more events are reported for a specific device.  This function does not
 * access the device, it returns status from the phantom registers that are
 * maintained by the API tick routine.
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  Returns true if there is an active event for the device.
 */
bool
Vp880GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)    /**< Pointer to the results event structure */
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj;
    VpLineCtxType *pLineCtx;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 i, eventCatLoop;
    uint8 chan, chanNum;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

#if defined (VP880_INCLUDE_TESTLINE_CODE) && defined (VP880_FXO_SUPPORT)
    #define EVENT_ARRAY_SIZE 6
#elif defined (VP880_INCLUDE_TESTLINE_CODE) || defined (VP880_FXO_SUPPORT)
    #define EVENT_ARRAY_SIZE 5
#else
    #define EVENT_ARRAY_SIZE 4
#endif

    uint16 eventArray[EVENT_ARRAY_SIZE];
    uint16 eventMaskArray[EVENT_ARRAY_SIZE];
    VpEventCategoryType eventCat[EVENT_ARRAY_SIZE] = {
        VP_EVCAT_FAULT,
        VP_EVCAT_SIGNALING,
        VP_EVCAT_RESPONSE,
        VP_EVCAT_PROCESS
#ifdef VP880_FXO_SUPPORT
        ,VP_EVCAT_FXO
#endif
#ifdef VP880_INCLUDE_TESTLINE_CODE
       ,VP_EVCAT_TEST
#endif
    };

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetEvent+"));

    pEvent->status = VP_STATUS_SUCCESS;
    pEvent->hasResults = FALSE;

    /* Initialize the arrays for device events */
    for (i = 0; i < EVENT_ARRAY_SIZE; i++) {
        switch(eventCat[i]) {
            case VP_EVCAT_FAULT:
                eventArray[i] = pDevObj->deviceEvents.faults;
                eventMaskArray[i] = pDevObj->deviceEventsMask.faults;
                break;

            case VP_EVCAT_SIGNALING:
                eventArray[i] = pDevObj->deviceEvents.signaling;
                eventMaskArray[i] = pDevObj->deviceEventsMask.signaling;
                break;

            case VP_EVCAT_RESPONSE:
                eventArray[i] = pDevObj->deviceEvents.response;
                eventMaskArray[i] = pDevObj->deviceEventsMask.response;
                break;

            case VP_EVCAT_PROCESS:
                eventArray[i] = pDevObj->deviceEvents.process;
                eventMaskArray[i] = pDevObj->deviceEventsMask.process;
                break;

#ifdef VP880_FXO_SUPPORT
            case VP_EVCAT_FXO:
                eventArray[i] = pDevObj->deviceEvents.fxo;
                eventMaskArray[i] = pDevObj->deviceEventsMask.fxo;
                break;
#endif

#ifdef VP880_INCLUDE_TESTLINE_CODE
            case VP_EVCAT_TEST:
                eventArray[i] = pDevObj->deviceEvents.test;
                eventMaskArray[i] = pDevObj->deviceEventsMask.test;
                break;
#endif

            default:
                /*
                 * This can only occur if there's a bug in this code. Get out
                 * since we don't know how to handle it.
                 */
                return FALSE;
        }
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Look for active device events first */
    for (eventCatLoop = 0; eventCatLoop < EVENT_ARRAY_SIZE; eventCatLoop++) {
        pEvent->eventId = Vp880CheckDevEvent(eventArray[eventCatLoop],
            eventMaskArray[eventCatLoop], eventCat[eventCatLoop], pDevObj);
        if (pEvent->eventId != 0x0000) {
            pEvent->deviceId = deviceId;
            pEvent->channelId = 0;
            pEvent->eventCategory = eventCat[eventCatLoop];
            pEvent->pDevCtx = pDevCtx;
            pEvent->pLineCtx = VP_NULL;
            pEvent->parmHandle = pDevObj->eventHandle;
            pEvent->hasResults = FALSE;

            if (pEvent->eventCategory == VP_EVCAT_RESPONSE) {
                /*
                 * For the events that require a read operation, set the has
                 * results indicator in the event structure
                 */

                switch (pEvent->eventId) {
                    case VP_LINE_EVID_RD_OPTION:
                        pEvent->channelId = pDevObj->getResultsOption.chanId;
                        pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
                        if (pEvent->pLineCtx != VP_NULL) {
                            Vp880LineObjectType *pLineObjLocal = pEvent->pLineCtx->pLineObj;
                            pEvent->lineId = pLineObjLocal->lineId;
                        }
                        pEvent->hasResults = TRUE;
                        pEvent->eventData = pDevObj->getResultsOption.optionType;
                        break;

                    case VP_DEV_EVID_IO_ACCESS_CMP:
                        pEvent->eventData =
                            (uint16)(pDevObj->getResultsOption.optionData.deviceIoData.accessType);
                        if (pEvent->eventData == VP_DEVICE_IO_READ) {
                            pEvent->hasResults = TRUE;
                        } else {
                            pEvent->hasResults = FALSE;
                        }
                        break;

                    case VP_DEV_EVID_DEV_INIT_CMP:
                        pEvent->eventData = 1;
                        break;

                    case VP_EVID_CAL_CMP:
                        pEvent->eventData = (uint16)pDevObj->responseData;
                        break;

                    default:
                        break;
                }
            }
            if (pEvent->eventCategory == VP_EVCAT_FAULT) {
                switch(pEvent->eventId) {
                    case VP_DEV_EVID_CLK_FLT:
                        pEvent->eventData =
                            (pDevObj->dynamicInfo.clkFault ? TRUE : FALSE);
                        break;

                    case VP_DEV_EVID_BAT_FLT:
                        if ((pDevObj->dynamicInfo.bat1Fault == TRUE)
                         || (pDevObj->dynamicInfo.bat2Fault == TRUE)
                         || (pDevObj->dynamicInfo.bat3Fault == TRUE)) {
                            pEvent->eventData = TRUE;
                        } else {
                            pEvent->eventData = FALSE;
                        }
                        break;

                    default:
                        break;
                }
            }
            /*
             * "Some" device event needs to be generated. Don't proceed with
             * line event processing.
             */
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return TRUE;
        }
    }

    /*
     * No device events, now look for Line events -- but make sure the line
     * context is valid before looking for a line object
     */
    if (pDevObj->dynamicInfo.lastChan >= maxChan) {
        pDevObj->dynamicInfo.lastChan = 0;
    }
    chanNum = pDevObj->dynamicInfo.lastChan;

    for(chan = 0; chan < maxChan; chan++) {
        pLineCtx = pDevCtx->pLineCtx[chanNum];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            /* The line context is valid, create a line object and initialize
             * the event arrays for this line
             */
            for (i = 0; i < EVENT_ARRAY_SIZE; i++) {
                switch(eventCat[i]) {
                    case VP_EVCAT_FAULT:
                        eventArray[i] = pLineObj->lineEvents.faults;
                        eventMaskArray[i] = pLineObj->lineEventsMask.faults;
                        break;

                    case VP_EVCAT_SIGNALING:
                        eventArray[i] = pLineObj->lineEvents.signaling;
                        eventMaskArray[i] = pLineObj->lineEventsMask.signaling;
                        break;

                    case VP_EVCAT_RESPONSE:
                        eventArray[i] = pLineObj->lineEvents.response;
                        eventMaskArray[i] = pLineObj->lineEventsMask.response;
                        break;

                    case VP_EVCAT_PROCESS:
                        eventArray[i] = pLineObj->lineEvents.process;
                        eventMaskArray[i] = pLineObj->lineEventsMask.process;
                        break;

#ifdef VP880_FXO_SUPPORT
                    case VP_EVCAT_FXO:
                        eventArray[i] = pLineObj->lineEvents.fxo;
                        eventMaskArray[i] = pLineObj->lineEventsMask.fxo;
                        break;
#endif

#ifdef VP880_INCLUDE_TESTLINE_CODE
                    case VP_EVCAT_TEST:
                        eventArray[i] = pLineObj->lineEvents.test;
                        eventMaskArray[i] = pLineObj->lineEventsMask.test;
                        break;
#endif
                    default:
                        /*
                         * This can only occur if there's a bug in this code. Get out
                         * since we don't know how to handle it.
                         */
                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        return FALSE;
                }
            }

            /* Check this line events */
            for (eventCatLoop = 0;
                 eventCatLoop < EVENT_ARRAY_SIZE;
                 eventCatLoop++) {
                pEvent->eventId = Vp880CheckLineEvent(eventArray[eventCatLoop],
                    eventMaskArray[eventCatLoop], eventCat[eventCatLoop],
                    pLineObj);

                if (pEvent->eventId != 0x0000) {
                    pEvent->deviceId = deviceId;
                    pEvent->channelId = chanNum;
                    pEvent->pLineCtx = pDevCtx->pLineCtx[chanNum];
                    pEvent->pDevCtx = pDevCtx;
                    pEvent->eventCategory = eventCat[eventCatLoop];
                    pEvent->parmHandle = pLineObj->lineEventHandle;
                    pEvent->lineId = pLineObj->lineId;
                    pEvent->hasResults = FALSE;

                    switch(pEvent->eventCategory) {
                        case VP_EVCAT_RESPONSE:
                            pEvent->eventData = (uint16)pLineObj->responseData;
                            switch(pEvent->eventId) {
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
                                case VP_LINE_EVID_LLCMD_RX_CMP:
                                    pEvent->eventData = pDevObj->mpiLen;
#endif
                                case VP_LINE_EVID_GAIN_CMP:
                                case VP_LINE_EVID_RD_LOOP:
                                    pEvent->hasResults = TRUE;
                                    break;

                                case VP_EVID_CAL_CMP:
                                    if (pLineObj->responseData == (uint8)VP_CAL_GET_SYSTEM_COEFF) {
                                        pEvent->eventData = pDevObj->mpiLen;
                                        pEvent->hasResults = TRUE;
                                        /*
                                         * Prevent future cal complete events from being
                                         * indicated as having results data.
                                         */
                                        pLineObj->responseData = (uint8)VP_CAL_ENUM_SIZE;
                                    }
                                    break;

                                default:
                                    break;
                            }
                            break;

                        case VP_EVCAT_SIGNALING:
                            if (pEvent->eventId == VP_LINE_EVID_DTMF_DIG) {
                                pEvent->eventData = pLineObj->dtmfDigitSense;
                                pEvent->parmHandle = pDevObj->timeStamp;
                            } else {
                                pEvent->eventData = pLineObj->signalingData;

                                if (pEvent->eventId == VP_LINE_EVID_HOOK_OFF) {
                                    pLineObj->status |= VP880_PREVIOUS_HOOK;
                                } else if (pEvent->eventId == VP_LINE_EVID_HOOK_ON) {
                                    pLineObj->status &= ~VP880_PREVIOUS_HOOK;
                                }
                            }
                            break;

#ifdef VP880_FXO_SUPPORT
                        case VP_EVCAT_FXO:
                            pEvent->eventData = pLineObj->fxoData;
                            break;
#endif

                        case VP_EVCAT_PROCESS:
                            pEvent->eventData = pLineObj->processData;
                            if (pEvent->eventId != VP_LINE_EVID_SIGNAL_CMP) {
                                pEvent->parmHandle = pDevObj->timeStamp;
                            }
                            break;

                        case VP_EVCAT_FAULT:
                            if (pEvent->eventId == VP_LINE_EVID_THERM_FLT) {
                                if ((pDevObj->intReg2[chanNum] & VP880_TEMPA1_MASK) !=
                                    (pDevObj->intReg[chanNum] & VP880_TEMPA1_MASK)) {

                                    pEvent->eventData = (pDevObj->intReg2[chanNum] & VP880_TEMPA1_MASK)
                                        ? TRUE : FALSE;

                                    pLineObj->lineEvents.faults |= VP_LINE_EVID_THERM_FLT;
                                    pDevObj->intReg2[chanNum] &= ~VP880_TEMPA1_MASK;
                                    pDevObj->intReg2[chanNum] |= (pDevObj->intReg[chanNum] & VP880_TEMPA1_MASK);
                                } else {
                                    pEvent->eventData = (pDevObj->intReg[chanNum] & VP880_TEMPA1_MASK)
                                        ? TRUE : FALSE;
                                }
                            }
                            break;

#if defined (VP880_INCLUDE_TESTLINE_CODE)
                        case VP_EVCAT_TEST:
                            if ( VP_LINE_EVID_TEST_CMP == pEvent->eventId) {
                                pEvent->eventData = pDevObj->testResults.testId;
                                pEvent->hasResults = TRUE;
                            }
                            break;
#endif
                        default:
                            /*
                             * This can only occur if there's a bug in this code. Get out
                             * since we don't know how to handle it.
                             */
                            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                            VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetEvent Error - Unknown Line Event Category"));
                            return FALSE;
                    }

                    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                    return TRUE;
                }
            }
        }
        /* We're done with this channel, start on next */
        chanNum = ((chanNum == 0) ? 1 : 0);
        pDevObj->dynamicInfo.lastChan = chanNum;
    }

    /* Actually, should never reach here. Just cleanup and exit quietly. */
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetEvent-"));

    return FALSE;
} /* End Vp880GetEvent */

/**
 * Vp880CheckDevEvent()
 *  This function performs a check on active device events and compares the
 * event with the event mask.  The event is cleared, and if the event is
 * unmasked it gets returned to the calling function via the return value.
 *
 * Preconditions:
 *  None. This is an internal API function call only and it is assumed all error
 * checking necessary is performed by higher level functions.
 *
 * Postconditions:
 *  If the returned value is other than 0x0000, the event being returned is
 * cleared in the device object.
 */
uint16
Vp880CheckDevEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp880DeviceObjectType *pDevObj)
{
    uint8 i;
    uint16 mask;

    VP_API_FUNC_INT(None, NULL, ("Vp880CheckDevEvent+"));

    for (i = 0, mask = 0x0001; i < 16; i++, (mask = mask << 1)) {
        /* Check to see if an event MAY be reported */
        if ((mask & event) != 0) {
            /*
             * Have to clear the device event so we don't report this event
             * again
             */
            switch(eventCat) {
                case VP_EVCAT_FAULT:
                    pDevObj->deviceEvents.faults &= (~mask);
                    break;

                case VP_EVCAT_SIGNALING:
                    pDevObj->deviceEvents.signaling &= (~mask);
                    break;

                case VP_EVCAT_RESPONSE:
                    pDevObj->deviceEvents.response &= (~mask);
                    break;

                case VP_EVCAT_PROCESS:
                    pDevObj->deviceEvents.process &= (~mask);
                    break;

                case VP_EVCAT_FXO:
                    pDevObj->deviceEvents.fxo &= (~mask);
                    break;

#ifdef VP880_INCLUDE_TESTLINE_CODE
                case VP_EVCAT_TEST:
                    pDevObj->deviceEvents.test &= (~mask);
                    break;
#endif
                default:
                    break;
            }

            /* If the event is not masked, return the event */
            if ((mask & eventMask) == 0) {
                VP_API_FUNC_INT(None, NULL, ("Vp880CheckDevEvent-"));
                return mask;
            }
        }
    }
    VP_API_FUNC_INT(None, NULL, ("Vp880CheckDevEvent-"));
    return 0x0000;
}

/**
 * Vp880CheckLineEvent()
 *  This function performs a check on active line events and compares the
 * event with the event mask.  The event is cleared, and if the event is
 * unmasked it gets returned to the calling function via the return value.
 *
 * Preconditions:
 *  None. This is an internal API function call only and it is assumed all error
 * checking necessary is performed by higher level functions.
 *
 * Postconditions:
 *  If the returned value is other than 0x0000, the event being returned is
 * cleared in the line object.
 */
uint16
Vp880CheckLineEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp880LineObjectType *pLineObj)
{
    uint8 i;
    uint16 mask;

    VP_API_FUNC_INT(None, NULL, ("Vp880CheckLineEvent+"));

    for (i = 0, mask = 0x0001; i < 16; i++, (mask = mask << 1)) {
        /* Check to see if an event MAY be reported */
        if ((mask & event) != 0) {
            /*
             * Have to clear the line event so we don't report this event
             * again
             */
            switch(eventCat) {
                case VP_EVCAT_FAULT:
                    pLineObj->lineEvents.faults &= (~mask);
                    break;

                case VP_EVCAT_SIGNALING:
                    pLineObj->lineEvents.signaling &= (~mask);
                    break;

                case VP_EVCAT_RESPONSE:
                    pLineObj->lineEvents.response &= (~mask);
                    break;

                case VP_EVCAT_PROCESS:
                    pLineObj->lineEvents.process &= (~mask);
                    break;

                case VP_EVCAT_FXO:
                    pLineObj->lineEvents.fxo &= (~mask);
                    break;

#ifdef VP880_INCLUDE_TESTLINE_CODE
               case VP_EVCAT_TEST:
                    pLineObj->lineEvents.test &= (~mask);
                    break;
#endif

                default:
                    break;
            }

            /* If the event is not masked, return the event */
            if ((mask & eventMask) == 0) {
                VP_API_FUNC_INT(None, NULL, ("Vp880CheckLineEvent-"));
                return mask;
            }
        }
    }
    VP_API_FUNC_INT(None, NULL, ("Vp880CheckLineEvent-"));
    return 0x0000;
}

/**
 * Vp880GetOption()
 *  This function accesses the option being requested, fills the device object
 * with the data to be returned, and sets the Read Option complete event.
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  The device object is filled with the results of the option type being
 * requested and the Read Option Event flag is set.  This function returns the
 * success code if the option type being requested is supported.
 */
VpStatusType
Vp880GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle)
{
    Vp880LineObjectType *pLineObj;
    Vp880DeviceObjectType *pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    VpGetResultsOptionsDataType *pOptionData;

    uint8 channelId, txSlot, rxSlot, pcn;
    VpDeviceIdType deviceId;
#ifdef VP880_FXS_SUPPORT
    uint8 tempSysConfig;
#endif
    uint8 ecVal;

    if (pLineCtx != VP_NULL) {
        VpDevCtxType *pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        pcn = pDevObj->staticInfo.rcnPcn[VP880_PCN_LOCATION];
        deviceId = pDevObj->deviceId;
        pLineObj = pLineCtx->pLineObj;
        ecVal = pLineObj->ecVal;
        channelId = pLineObj->channelId;
        pOptionData = &(pDevObj->getResultsOption.optionData);

        VP_API_FUNC(None, NULL, ("Vp880GetOption (Line)+"));

        if (pDevObj->deviceEvents.response & VP880_READ_RESPONSE_MASK) {
            VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880GetOption (Line) Error - VP_STATUS_DEVICE_BUSY"));
            return VP_STATUS_DEVICE_BUSY;
        }

        /* Do not allow FXS specific options on an FXO line */
        if (pLineObj->status & VP880_IS_FXO) {
            switch(option) {
                case VP_OPTION_ID_ZERO_CROSS:
                case VP_OPTION_ID_PULSE_MODE:
                case VP_OPTION_ID_LINE_STATE:
                case VP_OPTION_ID_RING_CNTRL:
                    VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880GetOption (Line) Error - VP_STATUS_INVALID_ARG"));
                    return VP_STATUS_INVALID_ARG;
                default:
                    break;
            }
        }

        /*
         * If this function can be executed, we will either access the MPI
         * and/or shared data. So it is best to label the entire function as
         * Code Critical so the data being accessed cannot be changed while
         * trying to be accessed
         */
        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

        pDevObj->getResultsOption.chanId = channelId;

        switch (option) {
            /* Line Options */
#ifdef CSLAC_GAIN_ABS
            case VP_OPTION_ID_ABS_GAIN:
                pOptionData->absGain.gain_AToD = pLineObj->gain.absGxGain;
                pOptionData->absGain.gain_DToA = pLineObj->gain.absGrGain;
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Posting absGxGain = 0x%02X, absGrGain = 0x%02X",
                    (uint8)pOptionData->absGain.gain_AToD, (uint8)pOptionData->absGain.gain_DToA));
                break;
#endif

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_PULSE_MODE:
                pOptionData->pulseModeOption = pLineObj->pulseMode;
                break;
#endif

            case VP_OPTION_ID_TIMESLOT:
                VpMpiCmdWrapper(deviceId, ecVal, VP880_TX_TS_RD,
                    VP880_TX_TS_LEN, &txSlot);

                VpMpiCmdWrapper(deviceId, ecVal, VP880_RX_TS_RD,
                    VP880_RX_TS_LEN, &rxSlot);

                pOptionData->timeSlotOption.tx = (txSlot & VP880_TX_TS_MASK);

                if ((pcn == VP880_DEV_PCN_88536) || (pcn == VP880_DEV_PCN_88264)) {
                    pOptionData->timeSlotOption.tx++;
                }

                pOptionData->timeSlotOption.rx = (rxSlot & VP880_RX_TS_MASK);
                break;

            case VP_OPTION_ID_CODEC:
                pOptionData->codecOption = pLineObj->codec;
                break;

            case VP_OPTION_ID_PCM_HWY:
                pOptionData->pcmHwyOption = VP_OPTION_HWY_A;
                break;

            case VP_OPTION_ID_LOOPBACK:
                /* Timeslot loopback via loopback register */
                if ((pLineObj->opCond[0] & VP880_INTERFACE_LOOPBACK_EN) ==
                     VP880_INTERFACE_LOOPBACK_EN) {
                    pOptionData->loopBackOption = VP_OPTION_LB_TIMESLOT;
                } else {
                    pOptionData->loopBackOption = VP_OPTION_LB_OFF;
                }
                break;

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_LINE_STATE:
                /* Battery control is automatic, so force it */
                pOptionData->lineStateOption.bat = VP_OPTION_BAT_AUTO;

                /* Smooth/Abrupt PolRev is controlled in the device */
                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                    VP880_SS_CONFIG_LEN, &tempSysConfig);

                if (tempSysConfig & VP880_SMOOTH_PR_EN) {
                    pOptionData->lineStateOption.battRev = FALSE;
                } else {
                    pOptionData->lineStateOption.battRev = TRUE;
                }
                break;
#endif

            case VP_OPTION_ID_EVENT_MASK:
                /*
                 * In SetOption(), we force all line-specific bits in the
                 * deviceEventsMask to zero.  Likewise, we force all device-
                 * specific bits in the lineEventsMask to zero.  This allows
                 * us to simply OR the two together here.
                 */
                pOptionData->eventMaskOption.faults =
                    pLineObj->lineEventsMask.faults |
                    pDevObj->deviceEventsMask.faults;
                pOptionData->eventMaskOption.signaling =
                    pLineObj->lineEventsMask.signaling |
                    pDevObj->deviceEventsMask.signaling;
                pOptionData->eventMaskOption.response =
                    pLineObj->lineEventsMask.response |
                    pDevObj->deviceEventsMask.response;
                pOptionData->eventMaskOption.test =
                    pLineObj->lineEventsMask.test |
                    pDevObj->deviceEventsMask.test;
                pOptionData->eventMaskOption.process =
                    pLineObj->lineEventsMask.process |
                    pDevObj->deviceEventsMask.process;
                pOptionData->eventMaskOption.fxo =
                    pLineObj->lineEventsMask.fxo |
                    pDevObj->deviceEventsMask.fxo;
                break;

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_ZERO_CROSS:
                pOptionData->zeroCross = pLineObj->ringCtrl.zeroCross;
                break;

            case VP_OPTION_ID_RING_CNTRL:
                pOptionData->ringControlOption = pLineObj->ringCtrl;
                break;
#endif
            case VP_OPTION_ID_PCM_TXRX_CNTRL:
                pOptionData->pcmTxRxCtrl = pLineObj->pcmTxRxCtrl;
                break;

#ifdef VP880_FXS_SUPPORT
            case VP_DEVICE_OPTION_ID_PULSE:
            case VP_DEVICE_OPTION_ID_PULSE2:
            case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
#endif
            case VP_DEVICE_OPTION_ID_DEVICE_IO:
                status = Vp880GetDeviceOption(pDevCtxLocal, option, handle);
                break;

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_DCFEED_PARAMS:
                /* Set the bits for the information we're returning */
                pOptionData->dcFeedParams.validMask =
                    (VP_OPTION_CFG_VOC | VP_OPTION_CFG_ILA | VP_OPTION_CFG_HOOK_THRESHOLD |
                    VP_OPTION_CFG_GKEY_THRESHOLD);

                /* Retrieve the user specified target VOC */
                pOptionData->dcFeedParams.voc =
                    ((pLineObj->calLineData.dcFeedRef[VP880_VOC_INDEX] & VP880_VOC_MASK) >> 2) * 3;
                /* Add 12V if low range, 36V if high range */
                pOptionData->dcFeedParams.voc +=
                    ((pLineObj->calLineData.dcFeedRef[VP880_VOC_INDEX] & VP880_VOC_LOW_RANGE) 
                    ? 12 : 36);
                /* Convert to mV */
                pOptionData->dcFeedParams.voc *= 1000;

                /* Retrieve the user specified target ILA */
                pOptionData->dcFeedParams.ila =
                    (pLineObj->calLineData.dcFeedRef[VP880_ILA_INDEX] & VP880_ILA_MASK) + 18;
                /* Convert to uA */
                pOptionData->dcFeedParams.ila *= 1000;

                /* Retreive the Hook Switch Threshold */
                pOptionData->dcFeedParams.hookThreshold =
                    ((pLineObj->calLineData.loopSup[VP880_LOOP_SUP_THRESH_BYTE] 
                      & VP880_SWHOOK_THRESH_MASK) + 8);
                /* Convert to uA */
                pOptionData->dcFeedParams.hookThreshold *= 1000;

                /* Retreive the Ground Key Threshold */
                pOptionData->dcFeedParams.gkeyThreshold =
                    ((pLineObj->calLineData.loopSup[VP880_LOOP_SUP_THRESH_BYTE] 
                      & VP880_GKEY_THRESH_MASK) >> 3) * 6;
                /* Convert to uA */
                pOptionData->dcFeedParams.gkeyThreshold *= 1000;

                /* Retreive the Floor Voltage */
                if ((pDevObj->stateInt & VP880_IS_ABS) != VP880_IS_ABS) {
                    pOptionData->dcFeedParams.validMask |= VP_OPTION_CFG_BATT_FLOOR;
                    pOptionData->dcFeedParams.battFloor =
                        (((pDevObj->swParams[VP880_FLOOR_VOLTAGE_BYTE] 
                         & VP880_FLOOR_VOLTAGE_MASK) * 5) + 5);
                    pOptionData->dcFeedParams.battFloor *= 1000;
                }
                break;

            case VP_OPTION_ID_RINGING_PARAMS: {
                uint8 ringingValues[VP880_RINGING_PARAMS_LEN];
                pOptionData->ringingParams.validMask =
                    (VP_OPTION_CFG_FREQUENCY | VP_OPTION_CFG_AMPLITUDE | VP_OPTION_CFG_DC_BIAS | 
                     VP_OPTION_CFG_RINGTRIP_THRESHOLD | VP_OPTION_CFG_RING_CURRENT_LIMIT | 
                     VP_OPTION_CFG_TRAP_RISE_TIME);
                VpMemCpy(ringingValues, pLineObj->ringingParamsRef, VP880_RINGER_PARAMS_LEN);

                /*
                 * DC Bias is a 16-bit signed value (+/-154.4V) for steps of ~4.71mV. The result
                 * is a 32-bit signed value because it needs to represent 48000mV
                 */
                pOptionData->ringingParams.dcBias =
                    (int32)((((uint16)(ringingValues[VP880_SIGA_BIAS_MSB]) << 8) & 0xFF00) |
                             ((uint16)(ringingValues[VP880_SIGA_BIAS_LSB]) & 0x00FF));
                pOptionData->ringingParams.dcBias *= 4712; /* VP880_RINGING_BIAS_SCALE; */
                pOptionData->ringingParams.dcBias /= 1000;
                
                /* Compute the Frequency/Rise Time. Eq. depends on Sine or Trapezoidal */
                if ((ringingValues[0] & VP880_SIGGEN1_SINTRAP_MASK)  == VP880_SIGGEN1_TRAP) {
                    /* Trapezopidal */
                    pOptionData->ringingParams.frequency =
                        (int32)((((uint16)ringingValues[VP880_SIGB_FREQ_MSB] << 8) & 0xFF00) |
                                (((uint16)ringingValues[VP880_SIGB_FREQ_LSB]) &0x00FF));
                    /* Convert per Command Set then into mHz */            
                    pOptionData->ringingParams.frequency =
                        1000 * (8000 / pOptionData->ringingParams.frequency);                                

                    /* Rise time is in Frequency A location IF Trapezoidal */         
                    pOptionData->ringingParams.trapRiseTime = 
                        (int32)((((uint16)ringingValues[VP880_SIGA_FREQ_MSB] << 8) & 0xFF00) |
                                (((uint16)ringingValues[VP880_SIGA_FREQ_LSB]) &0x00FF));
                    /* Protect Divide by 0 error */            
                    if (pOptionData->ringingParams.trapRiseTime == 0) {
                        pOptionData->ringingParams.trapRiseTime = 1;
                    }
                    /* Convert per Command Set and into uS (trise = 2.73 / FREQA) in seconds */
                    pOptionData->ringingParams.trapRiseTime =
                        (2730000 / pOptionData->ringingParams.trapRiseTime);
                } else {
                    /* Sine */
                    pOptionData->ringingParams.frequency =
                        (int32)((((uint16)ringingValues[VP880_SIGA_FREQ_MSB] << 8) & 0xFF00) |
                                (((uint16)ringingValues[VP880_SIGA_FREQ_LSB]) &0xFF));
                    /* Frequency is in 0.3662Hz. Multiply by 3662 / 10 gives mHz */
                    pOptionData->ringingParams.frequency *= 3662; /* VP880_GENA_FREQ_STEPSIZE; */
                    pOptionData->ringingParams.frequency /= 10;
                    
                    pOptionData->ringingParams.trapRiseTime = 0;
                }
                
                /*
                 * Amplitude is a 16-bit signed value (+/-154.4V) for steps of ~4.71mV. The result
                 * is a 32-bit signed value.
                 */
                pOptionData->ringingParams.amplitude = 
                    (int32)((((uint16)ringingValues[VP880_SIGA_AMP_MSB] << 8) & 0xFF00) |
                            (((uint16)ringingValues[VP880_SIGA_AMP_LSB]) &0x00FF));
                pOptionData->ringingParams.amplitude *= 4730; /* VP880_RINGING_AMP_SCALE; */
                pOptionData->ringingParams.amplitude /= 1000;

                /* Compute the Ring Trip Threshold and convert to uA. It's in 500uA steps. */
                pOptionData->ringingParams.ringTripThreshold =                 
                    ((int32)(pLineObj->calLineData.loopSup[VP880_LOOP_SUP_RT_MODE_BYTE] 
                      & VP880_LOOP_SUP_RT_MODE_THRSHLD) * 500);
                
                /* Compute the Ringing Current Limit :: 50-112mA, step = 2mA */
                pOptionData->ringingParams.ringCurrentLimit = 
                    ((pLineObj->calLineData.loopSup[VP880_LOOP_SUP_RING_LIM_BYTE] 
                      & VP880_LOOP_SUP_RING_LIM_MASK) + 25); /* Add 25 = 50mA / 2mA */                     
                /* Convert to uA */
                pOptionData->ringingParams.ringCurrentLimit *= 2000;
                }
                break;
#endif /* VP880_FXS_SUPPORT */

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880GetOption (Line)-"));
    } else {
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
        status = Vp880GetDeviceOption(pDevCtx, option, handle);
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    }

    if (status == VP_STATUS_SUCCESS) {
        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

        pDevObj->getResultsOption.optionType = option;
        pDevObj->deviceEvents.response |= VP_LINE_EVID_RD_OPTION;
        pDevObj->eventHandle = handle;

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    }

    return status;
}

/**
 * Vp880GetDeviceOption()
 *  This function accesses the option being requested, fills the device object
 * with the data to be returned, and sets the Read Option complete event.
 *
 * Functions calling this function have to make sure Enter/Exit critical are
 * called around this function.
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  The device object is filled with the results of the option type being
 * requested and the Read Option Event flag is set.  This function returns the
 * success code if the option type being requested is supported.
 */
VpStatusType
Vp880GetDeviceOption(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle)
{
    VpStatusType status = VP_STATUS_SUCCESS;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpGetResultsOptionsDataType *pOptionData;
    VpDeviceIdType deviceId;

    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    uint8 channelId;

    uint8 ecVal = pDevObj->ecVal;
    uint8 ioDirection[2] = {0x00, 0x00};

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetOption (Device)+"));

    /*
     * Upper layer checks to be sure that either device context or line
     * context pointers are not null -- so the device context is not null
     * in this case.
     */
    pDevObj = pDevCtx->pDevObj;
    deviceId = pDevObj->deviceId;
    pOptionData = &(pDevObj->getResultsOption.optionData);

    if (pDevObj->deviceEvents.response & VP880_READ_RESPONSE_MASK) {
        VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetOption (Device) Error - VP_STATUS_DEVICE_BUSY"));
        return VP_STATUS_DEVICE_BUSY;
    }

    switch (option) {
#ifdef VP880_FXS_SUPPORT
        case VP_DEVICE_OPTION_ID_PULSE:
            pOptionData->pulseTypeOption = pDevObj->pulseSpecs;
            break;

        case VP_DEVICE_OPTION_ID_PULSE2:
            pOptionData->pulseTypeOption = pDevObj->pulseSpecs2;
            break;

        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
            pOptionData->criticalFaultOption = pDevObj->criticalFault;
            break;
#endif
        case VP_DEVICE_OPTION_ID_DEVICE_IO: {
                uint8 pinCnt;
                uint16 bitMask;
                uint8 ecValMod[] = {VP880_EC_CH1, VP880_EC_CH2};

                uint8 regMask[VP880_MAX_PINS_PER_LINE] = {0x00,
                    VP880_IODIR_IO2_OUTPUT, /* 0x04 */
                    VP880_IODIR_IO3_OUTPUT, /* 0x08 */
                    VP880_IODIR_IO4_OUTPUT, /* 0x10 */
                    VP880_IODIR_IO5_OUTPUT, /* 0x20 */
                    VP880_IODIR_IO6_OUTPUT, /* 0x40 */
                };

                ecVal = pDevObj->ecVal;

               /*
                 * Preclear so only 'OR' operation for output and open drain
                 * indications are set.
                 */
                pOptionData->deviceIo.outputTypePins_63_32 = 0;
                pOptionData->deviceIo.directionPins_63_32 = 0;
                pOptionData->deviceIo.outputTypePins_31_0 = 0;
                pOptionData->deviceIo.directionPins_31_0 = 0;

                /* Get the current device IO control information */
                for (channelId = 0; channelId < maxChan; channelId++) {
                    VpMpiCmdWrapper(deviceId, (ecVal | ecValMod[channelId]),
                        VP880_IODIR_REG_RD, VP880_IODIR_REG_LEN,
                        &ioDirection[channelId]);
                }

                for (channelId = 0; channelId < maxChan; channelId++) {
                    for (pinCnt = 0; pinCnt < VP880_MAX_PINS_PER_LINE; pinCnt++) {
                        bitMask = (1 << (channelId + 2 * pinCnt));

                        if (pinCnt == 0) {
                            /* I/O-1 has a "type" of output to be determined. */
                            if (ioDirection[channelId] & VP880_IODIR_IO1_OPEN_DRAIN) {
                                pOptionData->deviceIo.outputTypePins_31_0 |= bitMask;
                                pOptionData->deviceIo.directionPins_31_0 |= bitMask;
                            } else if (ioDirection[channelId] & VP880_IODIR_IO1_OUTPUT) {
                                pOptionData->deviceIo.directionPins_31_0 |= bitMask;
                            }
                        } else {
                            /*
                             * All other pins are driven output only. Just need
                             * to determine IF they are configured for output.
                             */
                            if (ioDirection[channelId] & regMask[pinCnt]) {
                                pOptionData->deviceIo.directionPins_31_0 |= bitMask;
                            }
                        }
                    }
                }
            }
            break;

#ifdef VP880_FXS_SUPPORT
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_ZERO_CROSS:
        case VP_OPTION_ID_RING_CNTRL:
#endif
        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_PCM_HWY:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
        case VP_OPTION_ID_EVENT_MASK:
#ifdef CSLAC_GAIN_ABS
        case VP_OPTION_ID_ABS_GAIN:
#endif
            status = VP_STATUS_INVALID_ARG;
            VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetOption (Device) Error - VP_STATUS_INVALID_ARG"));
            break;

       default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetOption (Device) Error - VP_STATUS_OPTION_NOT_SUPPORTED"));
            break;
    }
    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetOption (Device)-"));
    return status;
}

/**
 * Vp880GetDeviceStatus()
 *  This function returns the status of all lines on a device for the type being
 * requested.
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  The location pointed to by the uint32 pointer passed is set (on a per line
 * basis) to either '1' if the status if TRUE on the given line, or '0' if the
 * status is FALSE on the given line for the status being requested.
 */
VpStatusType
Vp880GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    uint8 channelId;
    bool status = FALSE;
    VpLineCtxType *pLineCtx;

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetDeviceStatus+"));

    *pDeviceStatus = 0;

    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];

        if(pLineCtx != VP_NULL) {
            VpCSLACGetLineStatus(pLineCtx, input, &status);
        } else {
            status = FALSE;
        }
        *pDeviceStatus |= (((status == TRUE) ? 1 : 0) << channelId);
    }
    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetDeviceStatus-"));
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880FlushEvents()
 *  This function clears out all events on the device and all events on all
 * lines associated with the device passed.
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  All active device events are cleared, and all active line events associated
 * with this device are cleared.
 */
VpStatusType
Vp880FlushEvents(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VpLineCtxType *pLineCtx;
    Vp880LineObjectType *pLineObj;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880FlushEvents+"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VpMemSet(&pDevObj->deviceEvents, 0, sizeof(VpOptionEventMaskType));

    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if(pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            VpMemSet(&pLineObj->lineEvents, 0, sizeof(VpOptionEventMaskType));
        }
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880FlushEvents"));
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880GetResults()
 *  This function fills the results structure passed with the results data found
 * from the event that caused new results.
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  If the event structure passed provides the event catagory and ID for a valid
 * results type to read, then the structure passed is filled with the results
 * data.  This function returns the success code if the event catagory and ID is
 * supported by the device.
 */
VpStatusType
Vp880GetResults(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    uint8 mpiDataLen = pDevObj->mpiLen;
#endif
    uint8 commandByte;
    uint8 *pMpiData;

    VpGetResultsOptionsDataType *pOptionData =
        &(pDevObj->getResultsOption.optionData);

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetResults+"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    switch(pEvent->eventCategory) {
        case VP_EVCAT_RESPONSE:
            switch (pEvent->eventId) {
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
                case VP_LINE_EVID_LLCMD_RX_CMP:
                    pMpiData = (uint8 *)pResults;
                    VpMemCpy(pMpiData, pDevObj->mpiData, mpiDataLen);
                    break;
#endif

                case VP_LINE_EVID_GAIN_CMP:
                    *(VpRelGainResultsType *)pResults =
                        pDevObj->relGainResults;
                    break;

                case VP_DEV_EVID_IO_ACCESS_CMP:
                    *((VpDeviceIoAccessDataType *)pResults) =
                        pOptionData->deviceIoData;
                    break;

                case VP_LINE_EVID_RD_OPTION:
                    switch(pDevObj->getResultsOption.optionType) {
                        case VP_DEVICE_OPTION_ID_PULSE:
                            *(VpOptionPulseType *)pResults =
                                pDevObj->pulseSpecs;
                            break;

                        case VP_DEVICE_OPTION_ID_PULSE2:
                            *(VpOptionPulseType *)pResults =
                                pDevObj->pulseSpecs2;
                            break;

                        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
                            *(VpOptionCriticalFltType *)pResults =
                                pOptionData->criticalFaultOption;
                            break;

                        case VP_DEVICE_OPTION_ID_DEVICE_IO:
                            *(VpOptionDeviceIoType *)pResults =
                                pOptionData->deviceIo;
                            break;

                        case VP_OPTION_ID_RING_CNTRL:
                            *(VpOptionRingControlType *)pResults =
                                pOptionData->ringControlOption;
                            break;

                        case VP_OPTION_ID_ZERO_CROSS:
                            *(VpOptionZeroCrossType *)pResults =
                                pOptionData->zeroCross;
                            break;

                        case VP_OPTION_ID_PULSE_MODE:
                            *((VpOptionPulseModeType *)pResults) =
                                pOptionData->pulseModeOption;
                            break;

                        case VP_OPTION_ID_TIMESLOT:
                            *(VpOptionTimeslotType *)pResults =
                                pOptionData->timeSlotOption;
                            break;

                        case VP_OPTION_ID_CODEC:
                            *((VpOptionCodecType *)pResults) =
                                pOptionData->codecOption;
                            break;

                        case VP_OPTION_ID_PCM_HWY:
                            *((VpOptionPcmHwyType *)pResults) =
                                pOptionData->pcmHwyOption;
                            break;

                        case VP_OPTION_ID_LOOPBACK:
                            *((VpOptionLoopbackType *)pResults) =
                                pOptionData->loopBackOption;
                            break;

                        case VP_OPTION_ID_LINE_STATE:
                            *((VpOptionLineStateType *)pResults) =
                                pOptionData->lineStateOption;
                            break;

                        case VP_OPTION_ID_EVENT_MASK:
                            *((VpOptionEventMaskType *)pResults) =
                                pOptionData->eventMaskOption;
                            break;

                        case VP_OPTION_ID_PCM_TXRX_CNTRL:
                            *((VpOptionPcmTxRxCntrlType *)pResults) =
                                pOptionData->pcmTxRxCtrl;
                            break;
#ifdef CSLAC_GAIN_ABS
                        case VP_OPTION_ID_ABS_GAIN:
                            *(VpOptionAbsGainType *)pResults =
                                pOptionData->absGain;
                            break;
#endif
                        case VP_OPTION_ID_DCFEED_PARAMS:
                            *(VpOptionDcFeedParamsType *)pResults =
                                pOptionData->dcFeedParams;
                            break;

                        case VP_OPTION_ID_RINGING_PARAMS: 
                            *(VpOptionRingingParamsType *)pResults =
                                pOptionData->ringingParams;
                            break;    

                        default:
                            status = VP_STATUS_INVALID_ARG;
                            break;
                    }
                    break;

                case VP_EVID_CAL_CMP:
                    if (pResults == VP_NULL) {
                        status = VP_STATUS_INVALID_ARG;
                    } else {
                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880GetResults - VP_EVID_CAL_CMP"));

                        pMpiData = (uint8 *)pResults;
                        pMpiData[VP_PROFILE_TYPE_MSB] = VP_DEV_880_SERIES;
                        pMpiData[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_CAL;
                        pMpiData[VP_PROFILE_INDEX] = 0;
                        pMpiData[VP_PROFILE_LENGTH] = VP880_CAL_STRUCT_SIZE + 2;
                        pMpiData[VP_PROFILE_VERSION] = 0;
                        pMpiData[VP_PROFILE_MPI_LEN] = 0;
                        commandByte = VP_PROFILE_DATA_START;

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Y Error %d Size %lu",
                            pDevObj->vp880SysCalData.abvError[0], (uint32)sizeof(pDevObj->vp880SysCalData.abvError[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.abvError[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.abvError[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Z Error %d Size %lu",
                            pDevObj->vp880SysCalData.abvError[1], (uint32)sizeof(pDevObj->vp880SysCalData.abvError[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.abvError[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.abvError[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vocOffset[0][0], (uint32)sizeof(pDevObj->vp880SysCalData.vocOffset[0][0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[0][0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[0][0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vocError[0][0], (uint32)sizeof(pDevObj->vp880SysCalData.vocError[0][0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[0][0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[0][0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Rev Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vocOffset[0][1], (uint32)sizeof(pDevObj->vp880SysCalData.vocOffset[0][1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[0][1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[0][1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Rev Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vocError[0][1], (uint32)sizeof(pDevObj->vp880SysCalData.vocError[0][1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[0][1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[0][1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vocOffset[1][0], (uint32)sizeof(pDevObj->vp880SysCalData.vocOffset[1][0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[1][0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[1][0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vocError[1][0], (uint32)sizeof(pDevObj->vp880SysCalData.vocError[1][0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[1][0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[1][0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Rev Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vocOffset[1][1], (uint32)sizeof(pDevObj->vp880SysCalData.vocOffset[1][1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[1][1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocOffset[1][1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Rev Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vocError[1][1], (uint32)sizeof(pDevObj->vp880SysCalData.vocError[1][1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[1][1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vocError[1][1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Norm Error Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.sigGenAError[0][0], (uint32)sizeof(pDevObj->vp880SysCalData.sigGenAError[0][0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[0][0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[0][0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Rev Error Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.sigGenAError[0][1], (uint32)sizeof(pDevObj->vp880SysCalData.sigGenAError[0][1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[0][1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[0][1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Norm Error Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.sigGenAError[1][0], (uint32)sizeof(pDevObj->vp880SysCalData.sigGenAError[1][0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[1][0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[1][0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Rev Error Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.sigGenAError[1][1], (uint32)sizeof(pDevObj->vp880SysCalData.sigGenAError[1][1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[1][1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.sigGenAError[1][1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-20mA Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.ila20[0], (uint32)sizeof(pDevObj->vp880SysCalData.ila20[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila20[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila20[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-20mA Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.ila20[1], (uint32)sizeof(pDevObj->vp880SysCalData.ila20[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila20[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila20[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-25mA Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.ila25[0], (uint32)sizeof(pDevObj->vp880SysCalData.ila25[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila25[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila25[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-25mA Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.ila25[1], (uint32)sizeof(pDevObj->vp880SysCalData.ila25[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila25[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila25[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-32mA Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.ila32[0], (uint32)sizeof(pDevObj->vp880SysCalData.ila32[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila32[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila32[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-32mA Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.ila32[1], (uint32)sizeof(pDevObj->vp880SysCalData.ila32[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila32[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila32[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-40mA Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.ila40[0], (uint32)sizeof(pDevObj->vp880SysCalData.ila40[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila40[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila40[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-40mA Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.ila40[1], (uint32)sizeof(pDevObj->vp880SysCalData.ila40[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila40[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ila40[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA Offset Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.ilaOffsetNorm[0], (uint32)sizeof(pDevObj->vp880SysCalData.ilaOffsetNorm[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilaOffsetNorm[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilaOffsetNorm[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA Offset Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.ilaOffsetNorm[1], (uint32)sizeof(pDevObj->vp880SysCalData.ilaOffsetNorm[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilaOffsetNorm[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilaOffsetNorm[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILG Offset Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.ilgOffsetNorm[0], (uint32)sizeof(pDevObj->vp880SysCalData.ilgOffsetNorm[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilgOffsetNorm[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilgOffsetNorm[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILG Offset Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.ilgOffsetNorm[1], (uint32)sizeof(pDevObj->vp880SysCalData.ilgOffsetNorm[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilgOffsetNorm[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ilgOffsetNorm[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vas[0][0], (uint32)sizeof(pDevObj->vp880SysCalData.vas[0][0])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.vas[0][0];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Rev Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vas[0][1], (uint32)sizeof(pDevObj->vp880SysCalData.vas[0][1])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.vas[0][1];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vas[1][0], (uint32)sizeof(pDevObj->vp880SysCalData.vas[1][0])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.vas[1][0];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Rev Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vas[1][1], (uint32)sizeof(pDevObj->vp880SysCalData.vas[1][1])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.vas[1][1];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vagOffsetNorm[0], (uint32)sizeof(pDevObj->vp880SysCalData.vagOffsetNorm[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetNorm[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetNorm[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vagOffsetNorm[1], (uint32)sizeof(pDevObj->vp880SysCalData.vagOffsetNorm[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetNorm[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetNorm[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Rev Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vagOffsetRev[0], (uint32)sizeof(pDevObj->vp880SysCalData.vagOffsetRev[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetRev[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetRev[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Rev Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vagOffsetRev[1], (uint32)sizeof(pDevObj->vp880SysCalData.vagOffsetRev[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetRev[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vagOffsetRev[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Norm Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vbgOffsetNorm[0], (uint32)sizeof(pDevObj->vp880SysCalData.vbgOffsetNorm[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetNorm[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetNorm[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Norm Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vbgOffsetNorm[1], (uint32)sizeof(pDevObj->vp880SysCalData.vbgOffsetNorm[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetNorm[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetNorm[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Rev Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.vbgOffsetRev[0], (uint32)sizeof(pDevObj->vp880SysCalData.vbgOffsetRev[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetRev[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetRev[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Rev Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.vbgOffsetRev[1], (uint32)sizeof(pDevObj->vp880SysCalData.vbgOffsetRev[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetRev[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.vbgOffsetRev[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Normal Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.absNormCal[0], (uint32)sizeof(pDevObj->vp880SysCalData.absNormCal[0])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.absNormCal[0];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Rev Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.absPolRevCal[0], (uint32)sizeof(pDevObj->vp880SysCalData.absPolRevCal[0])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.absPolRevCal[0];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Normal Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.absNormCal[1], (uint32)sizeof(pDevObj->vp880SysCalData.absNormCal[1])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.absNormCal[1];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Rev Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.absPolRevCal[1], (uint32)sizeof(pDevObj->vp880SysCalData.absPolRevCal[1])));
                        pMpiData[commandByte++] = pDevObj->vp880SysCalData.absPolRevCal[1];

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWY Offset Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.swyOffset[0], (uint32)sizeof(pDevObj->vp880SysCalData.swyOffset[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swyOffset[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swyOffset[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWY Offset Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.swyOffset[1], (uint32)sizeof(pDevObj->vp880SysCalData.swyOffset[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swyOffset[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swyOffset[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWZ Offset Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.swzOffset[0], (uint32)sizeof(pDevObj->vp880SysCalData.swzOffset[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swzOffset[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swzOffset[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWZ Offset Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.swzOffset[1], (uint32)sizeof(pDevObj->vp880SysCalData.swzOffset[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swzOffset[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swzOffset[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("XB Offset Ch 0 %d Size %lu",
                            pDevObj->vp880SysCalData.swxbOffset[0], (uint32)sizeof(pDevObj->vp880SysCalData.swxbOffset[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swxbOffset[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swxbOffset[0]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("XB Offset Ch 1 %d Size %lu",
                            pDevObj->vp880SysCalData.swxbOffset[1], (uint32)sizeof(pDevObj->vp880SysCalData.swxbOffset[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swxbOffset[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.swxbOffset[1]) & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Tip Cap Ch 0 %li Size %lu",
                            pDevObj->vp880SysCalData.tipCapCal[0], (uint32)sizeof(pDevObj->vp880SysCalData.tipCapCal[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.tipCapCal[0] >> 24) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.tipCapCal[0] >> 16) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.tipCapCal[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)(pDevObj->vp880SysCalData.tipCapCal[0] & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Tip Cap Ch 1 %li Size %lu",
                            pDevObj->vp880SysCalData.tipCapCal[1], (uint32)sizeof(pDevObj->vp880SysCalData.tipCapCal[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.tipCapCal[1] >> 24) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.tipCapCal[1] >> 16) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.tipCapCal[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)(pDevObj->vp880SysCalData.tipCapCal[1] & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ring Cap Ch 0 %li Size %lu",
                            pDevObj->vp880SysCalData.ringCapCal[0], (uint32)sizeof(pDevObj->vp880SysCalData.ringCapCal[0])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ringCapCal[0] >> 24) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ringCapCal[0] >> 16) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ringCapCal[0] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)(pDevObj->vp880SysCalData.ringCapCal[0] & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ring Cap Ch 1 %li Size %lu",
                            pDevObj->vp880SysCalData.ringCapCal[1], (uint32)sizeof(pDevObj->vp880SysCalData.ringCapCal[1])));
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ringCapCal[1] >> 24) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ringCapCal[1] >> 16) & 0xFF);
                        pMpiData[commandByte++] = (uint8)((pDevObj->vp880SysCalData.ringCapCal[1] >> 8) & 0xFF);
                        pMpiData[commandByte++] = (uint8)(pDevObj->vp880SysCalData.ringCapCal[1] & 0xFF);

                        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Final Command Byte Value %d", commandByte));
                    }
                    break;

                default:
                    status = VP_STATUS_INVALID_ARG;
                    break;
            }
            break;

#ifdef VP880_INCLUDE_TESTLINE_CODE
        case VP_EVCAT_TEST:
            switch (pEvent->eventId) {
                case VP_LINE_EVID_TEST_CMP:
                    *((VpTestResultType *)pResults) = pDevObj->testResults;
                    break;

                 default:
                    break;
            }
#endif
            break;
        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880GetResults-"));
    return status;
}

/**
 * Vp880ServiceTimers()
 *  This function services active API timers on all channels of deviceId.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  All Active Timers have been serviced.
 */
bool
Vp880ServiceTimers(
    VpDevCtxType *pDevCtx)
{
    VpLineCtxType *pLineCtx;
    Vp880LineObjectType *pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 tempTimer;
    uint8 devTimerType;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    bool retFlag = FALSE;

    for (devTimerType = 0; devTimerType < VP_DEV_TIMER_LAST; devTimerType++) {
        if (pDevObj->devTimer[devTimerType] & VP_ACTIVATE_TIMER) {

            /* get the bits associated with the timer into a temp variable */
            tempTimer = (pDevObj->devTimer[devTimerType] & VP_TIMER_TIME_MASK);

            /* decrement the timer */
            if (tempTimer > 0) {
                tempTimer--;
            }

            /* reset the device timer to the new value*/
            pDevObj->devTimer[devTimerType] = tempTimer;

            /* if the timer has expired then run the timer code */
            if (pDevObj->devTimer[devTimerType] == 0) {
                switch(devTimerType) {
#ifdef VP880_ABS_SUPPORT
                    case VP_DEV_TIMER_EXIT_RINGING:
                        Vp880DevRingExitTimerHandler(pDevCtx);
                        break;
#endif  /* VP880_ABS_SUPPORT */

#ifdef VP880_LP_SUPPORT
                    case VP_DEV_TIMER_LP_CHANGE:
                        VP_LINE_STATE(VpDevCtxType, pDevCtx, ("VP_DEV_TIMER_LP_CHANGE Expired at %d",
                            pDevObj->timeStamp));
                        Vp880ServiceLpChangeTimer(pDevCtx);
                        break;
#endif  /* VP880_LP_SUPPORT */

#ifdef VP880_INCLUDE_TESTLINE_CODE
                    case VP_DEV_TIMER_TESTLINE:
                        {
                            const void *pTestArgs =
                                (const void*)&pDevObj->currentTest.pTestHeap->testArgs;
                            uint8 testChanId = pDevObj->currentTest.channelId;
                            VpTestLineIntFuncPtrType testline =
                                pDevCtx->funPtrsToApiFuncs.TestLineInt;

                            VP_LINE_STATE(VpDevCtxType, pDevCtx,
                                ("VP_DEV_TIMER_TESTLINE Expired at %d", pDevObj->timeStamp));

                            if ((testline != VP_NULL) && (pDevObj->currentTest.testState != -1)) {
                                /*
                                 * if the TestLineInt function exists and the current test state
                                 * has not been set back to -1 by test conclude before the timer
                                 * expired then run the call back
                                 */
                                testline(
                                    pDevCtx->pLineCtx[testChanId],
                                    pDevObj->currentTest.testId,
                                    pTestArgs,
                                    pDevObj->currentTest.handle,
                                    TRUE);
                            }
                        }
                        break;

#endif /* VP880_INCLUDE_TESTLINE_CODE */

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
                    case VP_DEV_TIMER_ABV_CAL:
                        VP_LINE_STATE(VpDevCtxType, pDevCtx, ("VP_DEV_TIMER_ABV_CAL Expired at %d",
                            pDevObj->timeStamp));

                        if (pDevObj->stateInt & VP880_IS_ABS) {
#ifdef VP880_ABS_SUPPORT
                            Vp880CalAbvAbsDev(pDevCtx);
#endif  /* VP880_ABS_SUPPORT */
                        } else {
#ifdef VP880_TRACKER_SUPPORT
                            Vp880CalAbv(pDevCtx);
#endif  /* VP880_TRACKER_SUPPORT */
                        }
                        break;

#ifdef VP880_ABS_SUPPORT
                    case VP_DEV_TIMER_ABSCAL:
                        VP_LINE_STATE(VpDevCtxType, pDevCtx, ("VP_DEV_TIMER_ABSCAL Expired at %d",
                            pDevObj->timeStamp));
                        Vp880AbsCalibration(pDevCtx);
                        break;
#endif  /* VP880_ABS_SUPPORT */
#endif  /* VP_CSLAC_RUNTIME_CAL_ENABLED */

#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
                    case VP_DEV_TIMER_ENTER_RINGING:
                        VP_LINE_STATE(VpDevCtxType, pDevCtx, ("VP_DEV_TIMER_ENTER_RINGING Expired at %d",
                            pDevObj->timeStamp));
                        Vp880LimitInRushCurrent(pDevObj, pDevObj->ecVal, TRUE);
                        break;
#endif

                    default:
                        break;
                } /* Switch (timerType) */

                /*
                 * This is a check to make sure that one of the call back
                 * functions has not reset the value of the devTimer. If
                 * the call back function has not then just clear the timer bits
                 * if it has then we need to enable the activate mask.
                 */
                if (pDevObj->devTimer[devTimerType] == 0) {
                    pDevObj->devTimer[devTimerType] &= ~(VP_ACTIVATE_TIMER);
                } else {
                    pDevObj->devTimer[devTimerType] |= VP_ACTIVATE_TIMER;
                }
            } else { /* If timer has not expired */
                pDevObj->devTimer[devTimerType] |= VP_ACTIVATE_TIMER;
            }
        } /* if timerType is active     */
    } /* Loop through all device timers */

    /* Iterate through the channels until all timers are serviced */
    for(channelId=0; channelId < maxChan; channelId++ ) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

            if (!(pLineObj->status & VP880_IS_FXO)) {
#ifdef VP880_FXS_SUPPORT
                Vp880ServiceFxsTimers(pLineCtx);
#endif
            } else {
#ifdef VP880_FXO_SUPPORT
                Vp880ServiceFxoTimers(pLineCtx);
#endif
            }
        } /* Line Context Check */
    } /* Loop through channels until no more tests */

    return retFlag;
} /* Vp880ServiceTimers() */

#ifdef VP880_ABS_SUPPORT
/**
 * Vp880DevRingExitTimerHandler()
 *  This function services the Device Level Ringing Exit Timer. It is used only for ABS.
 *
 * Preconditions:
 *  This Function should be called only when VP_DEV_TIMER_EXIT_RINGING exipres. The device must
 *  already be in HP state when entering this function since this function does not set it to HP
 *  mode if it determines the device cannot (yet) be taken out of HP mode.
 *
 * Postconditions:
 */
void
Vp880DevRingExitTimerHandler(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

    VpLineCtxType *pLineCtx = VP_NULL;
    Vp880LineObjectType *pLineObj = VP_NULL;

    uint8 channelId;
    uint8 lineState[VP880_SYS_STATE_LEN];

    /*
     * Default is to keep the supply in high powered mode because setting it to medium power has
     * some negative consequences if incorrect. So we want to design the algorithm to change to
     * medium power for only those conditions we're currently aware of. If new conditions later
     * exist that would also allow transition to low power mode and this algorithm is not updated,
     * this will still work correctly - but perhaps not optimally.
     */
    bool disableHpm = FALSE;

    VP_LINE_STATE(VpDevCtxType, pDevCtx,
        ("VP_DEV_TIMER_EXIT_RINGING Expired at %d", pDevObj->timeStamp));

    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

            /*
             * Since FXO is not supported on ABS devices, this check should not be needed. But
             * we just want to be 100% certain not to incorrectly mis-interpret register content
             * from an FXO line type
             */
            if (!(pLineObj->status & VP880_IS_FXO)) {   /* Line type is FXS */
                switch (pLineObj->slicValueCache & VP880_SS_LINE_FEED_MASK) {
                    case  VP880_SS_FEED_BALANCED_RINGING:
                    case  VP880_SS_FEED_UNBALANCED_RINGING:
                        break;

                    default:
                        /*
                         * The SLIC is not in Ringing at least. But since the SLIC waits for zero
                         * crossing to disable ringing, we now have to check it's sub-states. If
                         * the sub-states ALSO indicate the line is not in Ringing (Ringing Exit),
                         * then we can start to consider disabling High Power Mode
                         */
                        VpMpiCmdWrapper(deviceId, pLineObj->ecVal,
                            VP880_SYS_STATE_RD, VP880_SYS_STATE_LEN, lineState);

                        /* We're completely out of Ringing if the Ring Exit bit is cleared */
                        if (!(lineState[0] & VP880_SS_RING_EXIT_MASK)) {
                            disableHpm = TRUE;
                        }
                        break;
                }
            }
        }
    }
    if (disableHpm == TRUE) {
        uint8 regCtrl = VP880_SWY_MP | VP880_SWZ_MP;
        VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_CTRL_WRT,
            VP880_REGULATOR_CTRL_LEN, &regCtrl);
    } else {
        pDevObj->devTimer[VP_DEV_TIMER_EXIT_RINGING] =
            (MS_TO_TICKRATE(VP_DEV_TIMER_EXIT_RINGING_SAMPLE,
            pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
    }
}
#endif

#if defined (VP880_FXS_SUPPORT) && defined (VP880_LP_SUPPORT)
/**
 * Vp880ServiceLpChangeTimer()
 *  This function services the active Low Power Change Device timer.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  The LP Change timer has been serviced.
 */
void
Vp880ServiceLpChangeTimer(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp880LineObjectType *pLineObj;
    VpLineCtxType *pLineCtx;

    uint8 ecVal;

    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            bool failureMode = FALSE;
            pLineObj = pLineCtx->pLineObj;
            ecVal = pLineObj->ecVal;

            if ((pLineObj->lineState.currentState != VP_LINE_DISCONNECT) &&
                (pLineObj->lineState.currentState != VP_LINE_TIP_OPEN) &&
                (pLineObj->lineState.currentState != VP_LINE_RINGING) &&
                (!(pLineObj->status & VP880_LINE_IN_CAL))) {

                VP_HOOK(VpLineCtxType, pLineCtx, ("Signaling 0x%02X 0x%02X",
                    pDevObj->intReg[0], pDevObj->intReg[1]));

                VP_HOOK(VpLineCtxType, pLineCtx,
                        ("Last Hook State on line %d = %d LP Mode %d UserState %d",
                         channelId, (pLineObj->lineState.condition & VP_CSLAC_HOOK),
                         (pLineObj->status & VP880_LOW_POWER_EN), pLineObj->lineState.usrCurrent));

                if (pLineObj->status & VP880_LOW_POWER_EN) {
                    /*
                     * If we're in LP Mode, then the line should be detecting
                     * on-hook. All other conditions mean there could be a leaky
                     * line.
                     */
                    if ((pLineObj->lineState.condition & VP_CSLAC_HOOK)
                      && (!(pDevObj->intReg[channelId]) & VP880_HOOK1_MASK)) {
                        failureMode = TRUE;
                    }
                } else {
                    /*
                     * If we're not in LP Mode, then the line should be
                     * detecting off-hook unless just recovered from Disconnect.
                     * If not just from Disconnect, and looking for off-hook,
                     * the signaling bit should be high. Otherwise, error.
                     */
                    if ((pLineObj->lineState.condition & VP_CSLAC_HOOK)
                      && (!(pDevObj->intReg[channelId]) & VP880_HOOK1_MASK)) {
                        failureMode = TRUE;
                        pLineObj->leakyLineCnt++;
                    }
                }
            }

            /*
             * If the line was last seen off-hook and is now on-hook as a result
             * of exiting LP Mode, it could be a leaky line.
             */
            if (failureMode == TRUE) {
                if (pLineObj->leakyLineCnt >= VE8XX_LPM_LEAKY_LINE_CNT) {
                    /*
                     * After determining the line is leaky the lines are taken
                     * out of LPM causing this timer to complete (one last time)
                     * so if the Leaky Line Flag/Event is already complete, we
                     * can skip this step and just clean up.
                     */
                    if (!(pLineObj->status & VP880_LINE_LEAK)) {
                        VP_HOOK(VpLineCtxType, pLineCtx,
                                ("Flag Channel %d for Leaky Line at time %d Signaling 0x%02X LineState %d",
                                 channelId, pDevObj->timeStamp,
                                 (pDevObj->intReg[channelId] & VP880_HOOK1_MASK),
                                 pLineObj->lineState.usrCurrent));

                        pLineObj->status |= VP880_LINE_LEAK;
                        pDevObj->stateInt &=
                            ((channelId == 0) ? ~VP880_LINE0_LP : ~VP880_LINE1_LP);
                        pLineObj->lineEvents.faults |= VP_LINE_EVID_RES_LEAK_FLT;

                        /* Leak test is complete */
                        pLineObj->lineState.condition &= ~VP_CSLAC_LINE_LEAK_TEST;
                    }
                } else {
                    uint16 deviceTimer;

                    VP_HOOK(VpLineCtxType, pLineCtx,
                            ("Potential Leaky Line %d at time %d count (%d) ...retry",
                             channelId, pDevObj->timeStamp, pLineObj->leakyLineCnt));

                    /* Leak test is still in progress */
                    /*
                     * Make sure timer is restarted. This may occur as a result
                     * of SetLineState(), but it may not.
                     */
                    if (pDevObj->swParams[VP880_REGULATOR_TRACK_INDEX] & VP880_REGULATOR_FIXED_RING_SWY) {
                        /*
                         * Longest is using "fixed" ringing mode because the external
                         * capacitors are generally very large.
                         */
                        deviceTimer = VP880_PWR_SWITCH_DEBOUNCE_FXT;
                    } else {
                        deviceTimer = VP880_PWR_SWITCH_DEBOUNCE_FUT;
                    }
                    VpCSLACSetTimer(&pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE],
                                    (MS_TO_TICKRATE(deviceTimer, pDevObj->devProfileData.tickRate)));
                    VP_LINE_STATE(VpDevCtxType, pDevCtx,
                        ("Vp880ServiceLpChangeTimer() Channel %d: Starting VP_DEV_TIMER_LP_CHANGE with (%d) ms at time %d",
                        pLineObj->channelId, deviceTimer, pDevObj->timeStamp));
                }

                /* Update the line state */
                for (channelId = 0;
                     channelId < pDevObj->staticInfo.maxChannels;
                     channelId++) {
                    Vp880LineObjectType *pLineObjInt;

                    pLineCtx = pDevCtx->pLineCtx[channelId];
                    if (pLineCtx != VP_NULL) {
                        pLineObjInt = pLineCtx->pLineObj;
                        VP_HOOK(VpLineCtxType, pLineCtx,
                                ("1. Channel %d Current Linestate %d Current User Linestate %d",
                                 channelId, pLineObjInt->lineState.currentState,
                                 pLineObjInt->lineState.usrCurrent));

                        if (pLineObj->status & VP880_LOW_POWER_EN) {
                            /* Force line to feed state */
                            pLineObj->lineState.currentState = VP_LINE_OHT;
                            pDevObj->stateInt &= ~((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);
                        } else {
                            Vp880SetLineStateInt(pLineCtx, pLineObjInt->lineState.usrCurrent);
                        }
                    }
                }
            } else {
                /*
                 * No failure. Recover all hook status, line states, and clear
                 * Leaky Line Test Flag
                 */

                /* Leak test is complete */
                pLineObj->lineState.condition &= ~VP_CSLAC_LINE_LEAK_TEST;

                /*
                 * Low Power Mode exit simply sets the line to Active in order
                 * to maintain smooth line transients. This step is done to
                 * put the line into the user (API-II) defined state.
                 */
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                              ("LPM Timer: Current %d, User Current %d Channel %d",
                               pLineObj->lineState.currentState, pLineObj->lineState.usrCurrent,
                               channelId));

                if ((pLineObj->lineState.usrCurrent == VP_LINE_STANDBY)
                  && (!(pLineObj->status & VP880_LOW_POWER_EN))
                  && (pLineObj->calLineData.calDone == TRUE)) {     /* Must not occur during the calibration */
                    uint8 lineState[1] = {VP880_SS_IDLE};

#ifdef VP_CSLAC_SEQ_EN
                    if ((pLineObj->cadence.status & (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_SENDSIG)) !=
                        (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_SENDSIG)) {
#endif
                        Vp880UpdateBufferChanSel(pDevObj, channelId, lineState[0], TRUE);
                        if (pLineObj->slicValueCache != lineState[0]) {
                            pLineObj->slicValueCache = lineState[0];
                            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                          ("Setting Channel %d to 0x%02X State at time %d",
                                           channelId, lineState[0], pDevObj->timeStamp));
                            VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                                VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
                        }
#ifdef VP_CSLAC_SEQ_EN
                    }
#endif
                }

                if ((pLineObj->lineState.condition & VP_CSLAC_HOOK)
                    && (pDevObj->intReg[channelId]) & VP880_HOOK1_MASK) {
                    if ((pLineObj->lineState.condition & VP_CSLAC_HOOK) &&
                        (pLineObj->status & VP880_LOW_POWER_EN)) {
                        /* Valid on-hook */
                        VP_HOOK(VpLineCtxType, pLineCtx, ("Valid On-Hook on line %d at time %d",
                            channelId, pDevObj->timeStamp));

                        pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                        Vp880UpdateHookInfo(pLineObj, pDevObj);
                    } else {
                        /* Valid off-hook */
                        VP_HOOK(VpLineCtxType, pLineCtx, ("Valid Off-Hook on line %d at time %d",
                            channelId, pDevObj->timeStamp));

                        pLineObj->leakyLineCnt = 0;
                        pLineObj->status &= ~VP880_LINE_LEAK;

                        pLineObj->lineState.condition |= VP_CSLAC_HOOK;
                        Vp880UpdateHookInfo(pLineObj, pDevObj);
                    }
                }
            }
        }
    }
}
#endif  /* defined (VP880_FXS_SUPPORT) && defined (VP880_LP_SUPPORT) */

#ifdef VP880_FXS_SUPPORT
/**
 * Vp880ServiceFxsTimers()
 *  This function services active FXS API timers.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  All Active FXS Timers for the current line have been serviced.
 */
void
Vp880ServiceFxsTimers(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

#ifdef VP880_LP_SUPPORT
    uint8 channelId = pLineObj->channelId;
#endif

    uint8 ecVal = pLineObj->ecVal;

    uint8 lineTimerType;
    uint16 tempTimer;
    VpCslacTimers *fxsTimers = &pLineObj->lineTimers.timers;

    for (lineTimerType = 0; lineTimerType < VP_LINE_TIMER_LAST;  lineTimerType++) {

        if (fxsTimers->timer[lineTimerType] & VP_ACTIVATE_TIMER) {

            tempTimer = (fxsTimers->timer[lineTimerType] & VP_TIMER_TIME_MASK);

            if (tempTimer > 0) {
                tempTimer--;
            }

            fxsTimers->timer[lineTimerType] = tempTimer;

            if (tempTimer == 0) {
                fxsTimers->timer[lineTimerType] &= ~(VP_ACTIVATE_TIMER);

                /* Perform appropriate action based on timerType */
                switch (lineTimerType) {
                    case VP_LINE_RING_EXIT_PROCESS: {
                        /*
                         * There are two reasons to delay ringing exit - 1. in LPM to correct for
                         * the initial delay when entering ringing, and 2. to delay a polarity
                         * reversal change when exiting ringing to prevent the polarity reversal
                         * from occurring during the ringing cycle.
                         */
                        uint8 ringExitSt[VP880_SYS_STATE_LEN];
                        uint8 slicState = (pLineObj->slicValueCache & VP880_SS_LINE_FEED_MASK);
                        bool exitRinging = FALSE;

                        VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_RD,
                            VP880_SYS_STATE_LEN, ringExitSt);

                        /*
                         * Check to see if we're completely out of ringing. Although the SLIC state
                         * should have already been modified prior to this point and therefore NOT
                         * indicate Ringing State, it's possible in theory that the time between
                         * changing the SLIC to a non-Ringing state to the point the register is
                         * read could be too fast for the silicon state machine to update. It's
                         * just theory...but we want to be 100% sure not to change register settings
                         * until we're really out of Ringing.
                         */
                        if ((ringExitSt[0] & VP880_SS_RING_EXIT_MASK) ||
                            (slicState ==  VP880_SS_FEED_BALANCED_RINGING) ||
                            (slicState ==  VP880_SS_FEED_UNBALANCED_RINGING)) {
                            /*
                             * If any of the conditions are TRUE, it means Ringing is still on
                             * Tip/Ring. Ringing should be removed in <= 100ms from the time the
                             * new SLIC state is programmed, so keep track how long we're in this
                             * condition to eventually force stop it
                             */
                            if (fxsTimers->trackingTime < VP_CSLAC_RINGING_EXIT_MAX) {
                                /*
                                 * We haven't exceeded our maximum allowable ringing exit time,
                                 * so just restart this timer to run as soon as possible and update
                                 * the trackingTime (note: tracking time is in ms).
                                 */
                                fxsTimers->timer[VP_LINE_RING_EXIT_PROCESS] =
                                    (1 | VP_ACTIVATE_TIMER);
                                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                    ("Ringing still present on the line. RING_EXIT_TIMER restart Ch %d at time %d",
                                    pLineObj->channelId, pDevObj->timeStamp));

                                /*
                                 * Keep track of how long we've been in this condition so we don't
                                 * repeat this forever.
                                 */
                                fxsTimers->trackingTime +=
                                    TICKS_TO_MS(1, pDevObj->devProfileData.tickRate);
                            } else {
                                /*
                                 * Ringing is still on the line, but we've run out of patience.
                                 * Time to just force ringing removal
                                 */
                                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                              ("Forced Ring Exit after (%d) on Ch %d at time %d",
                                               fxsTimers->trackingTime, pLineObj->channelId,
                                               pDevObj->timeStamp));
                                exitRinging = TRUE;
                            }
                        } else {
                            /*
                             * Ringing is totally removed from the line. It's now ok to change to
                             * the final line state and re-disable the Auto-State Transitions of
                             * the silicon.
                             */
                            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                          ("Normal Ring Exit after (%d) on Ch %d at time %d",
                                           fxsTimers->trackingTime, pLineObj->channelId,
                                           pDevObj->timeStamp));
                            exitRinging = TRUE;
                        }

                        if (exitRinging) {
#ifdef VP_CSLAC_SEQ_EN
                            bool resetTimer = FALSE;
                            uint16 timeRemain = 0;
#endif

                            /*
                             * Even though the MPI read will initialize this value, some compilers
                             * may not be that smart and can generate a warning if this isn't done.
                             */
                            uint8 ssCfg[VP880_SS_CONFIG_LEN] = {0x00};

                            /*
                             * Before doing anything, disable the silicon auto-system state control
                             * mechanism. This can interfere with Low Power Mode and/or other
                             * API line state managed changes
                             */
                            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                                VP880_SS_CONFIG_LEN, ssCfg);
                            ssCfg[0] |= VP880_AUTO_SSC_DIS;
                            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT,
                                VP880_SS_CONFIG_LEN, ssCfg);

#ifdef VP_CSLAC_SEQ_EN
                            /*
                             * Calling Vp880SetLineStateInt() will reset the timer if in the proces
                             * of Ringing Cadence. Save the timeRemain value and write it back after
                             * calling Vp880SetLineStateInt() if necessary.
                             */
                            if (pLineObj->cadence.status & VP_CADENCE_STATUS_MID_TIMER) {
                                resetTimer = TRUE;
                                timeRemain = pLineObj->cadence.timeRemain;
                            }
#endif /* VP_CSLAC_SEQ_EN */

                            /*
                             * If performing a polarity reversal this will start the Hook Freeze
                             * timer based on polarity reversal hook activity. So we won't need to
                             * account for this again when modifying the same timer for ring trip
                             * or user specified ringing exit
                             */
                            Vp880SetLineStateInt(pLineCtx, pLineObj->lineState.currentState);
#ifdef VP_CSLAC_SEQ_EN
                            /*
                             * Restore the cadence timer status and timer if set prior to calling
                             * Vp880SetLineStateInt(). This occurs during Ringing Cadence.
                             */
                            if (resetTimer) {
                                pLineObj->cadence.status |= VP_CADENCE_STATUS_MID_TIMER;
                                pLineObj->cadence.timeRemain = timeRemain;
                            }
#endif /* VP_CSLAC_SEQ_EN */

                            /*
                             * Start the hook switch debounce timer based on conditions of ringing
                             * exit - either debounce for internally known continued hook activity
                             * up a ring trip or debounce for normal ringing exit if on-hook. For
                             * polarity reversal, Vp880SetLineState() will set the Hook Freeze
                             * timer to the correct value. The VpCSLACSetTimer() just updates the
                             * timer if the new value is longer than the previously set value.
                             */
                            if (pLineObj->status & VP_CSLAC_HOOK) { /* Ring Trip */
                                VpCSLACSetTimer(&fxsTimers->timer[VP_LINE_HOOK_FREEZE],
                                    MS_TO_TICKRATE(VP880_RING_TRIP_DEBOUNCE,
                                                   pDevObj->devProfileData.tickRate));
                            } else {    /* On-Hook Ringing Exit */
#ifdef VP_CSLAC_SEQ_EN
                                /*
                                 * Generate the Ringing Cadence Off Event now that ringing is
                                 * removed from the line or now that we've given up on trying. Only
                                 * do this if Cadence is supported in the API and if we're exiting
                                 * ringing while running a ringing cadence.
                                 */
                                if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                                    VpProfilePtrType pProfile = pLineObj->cadence.pActiveCadence;
                                    if ((pProfile != VP_NULL) && (pProfile[VP_PROFILE_TYPE_LSB]
                                                                   == VP_PRFWZ_PROFILE_RINGCAD)) {
                                        pLineObj->lineEvents.process |= VP_LINE_EVID_RING_CAD;
                                        pLineObj->processData = VP_RING_CAD_BREAK;
                                    }
                                }
#endif  /* VP_CSLAC_SEQ_EN */
                                /* Note the "/8" conversion from 125us to ms */
                                VpCSLACSetTimer(&fxsTimers->timer[VP_LINE_HOOK_FREEZE],
                                    MS_TO_TICKRATE((pLineObj->ringCtrl.ringExitDbncDur / 8),
                                                   pDevObj->devProfileData.tickRate));
                            }
                        }
                        }
                        /* May not be ready for this, but it doesn't hurt */
                        pDevObj->state |= VP_DEV_PENDING_INT;
                        break;

                    case VP_LINE_HOOK_FREEZE:
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                      ("VP_LINE_HOOK_FREEZE Expired on Ch %d at time %d",
                                       pLineObj->channelId, pDevObj->timeStamp));
                        pDevObj->state |= VP_DEV_PENDING_INT;
                        break;

                    case VP_LINE_DISCONNECT_EXIT:
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                      ("VP_LINE_DISCONNECT_EXIT Expired on Ch %d at time %d",
                                       pLineObj->channelId, pDevObj->timeStamp));

                        /*
                         * If we're in calibration, don't do anything. The device/line has to first
                         * be restored from calibration conditions before proceeding with normal
                         * control. Note that this flag can only be set and not cleared. That's
                         * because the sequence of starting disconnect and calibration occurs in
                         * VpInitDevice() (i.e., normal) but then once calibration is in process,
                         * additional line state changes by the application are not allowed.
                         */
                        if (pDevObj->state & VP_DEV_IN_CAL) {
                            pDevObj->state |= VP_DEV_DISC_PENDING;
                        } else {
                            /*
                             * The Service Disconnect Exit Timer management function will determine
                             * if the device interrupt should be read. So don't assume that to be
                             * the case here.
                             */
                            Vp880ServiceDiscExitTimer(pLineCtx);
                        }
                        break;

                    case VP_LINE_GND_START_TIMER:
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                      ("VP_LINE_GND_START_TIMER Expired on Ch %d at time %d",
                                       pLineObj->channelId, pDevObj->timeStamp));
                        Vp880ServiceGroundStartTimer(pLineCtx);
                        break;

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
                    case VP_LINE_CAL_LINE_TIMER:
                        VP_CALIBRATION(VpLineCtxType, pLineCtx,
                                       ("VP_LINE_CAL_LINE_TIMER Expired on Ch %d at time %d",
                                        pLineObj->channelId, pDevObj->timeStamp));
                        Vp880CalLineInt(pLineCtx);
                        break;
#endif

                   case VP_LINE_PING_TIMER: /* Set only for VC silicon */
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                      ("VP_LINE_PING_TIMER Expired on Ch %d at time %d",
                                       pLineObj->channelId, pDevObj->timeStamp));

                        if (((pDevObj->stateInt & (VP880_LINE0_LP | VP880_LINE1_LP))
                            == (VP880_LINE0_LP | VP880_LINE1_LP))
                         && (!(pLineObj->status & VP880_LINE_LEAK))
                         && (pLineObj->lineState.usrCurrent == VP_LINE_STANDBY)) {
                            pLineObj->nextSlicValue = VP880_SS_DISCONNECT;
                        }

                        if (pLineObj->slicValueCache != pLineObj->nextSlicValue) {
                            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                ("VP_LINE_PING_TIMER: Setting Chan %d to Value 0x%02X at Time %d",
                            pLineObj->channelId, pLineObj->nextSlicValue, pDevObj->timeStamp));
                            pLineObj->slicValueCache = pLineObj->nextSlicValue;
                            VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                                VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
                        }
                        break;

#ifdef VP880_LP_SUPPORT
                    case VP_LINE_TRACKER_DISABLE:
                        if (!(pLineObj->lineState.condition & VP_CSLAC_LINE_LEAK_TEST)) {
                            uint8 sysState[VP880_SYS_STATE_LEN] = {VP880_SS_DISCONNECT};
                            uint8 icr2Temp[VP880_ICR2_LEN];

                            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                ("VP_LINE_TRACKER_DISABLE Expired on Ch %d at time %d",
                                pLineObj->channelId, pDevObj->timeStamp));

                            Vp880UpdateBufferChanSel(pDevObj, channelId, sysState[0], TRUE);
                            if (pLineObj->slicValueCache != sysState[0]) {
                                /* Set line to Disconnect if not already there */
                                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                    ("Tracker Disable: Setting Ch %d to State 0x%02X at time %d",
                                    pLineObj->channelId, sysState[0], pDevObj->timeStamp));
                                pLineObj->slicValueCache = sysState[0];
                                VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
                            }

                            VpMemCpy(icr2Temp, pLineObj->icr2Values, VP880_ICR2_LEN);

                            pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX] |= VP880_ICR2_ILA_DAC;
                            pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX] &= ~VP880_ICR2_VOC_DAC_SENSE;
                            pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX+1] &= ~VP880_ICR2_ILA_DAC;

                            if ((icr2Temp[VP880_ICR2_VOC_DAC_INDEX] != pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX]) ||
                                (icr2Temp[VP880_ICR2_VOC_DAC_INDEX+1] != pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX+1])) {
                                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                    ("Tracker Disable Update Required: ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                                pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                                    pLineObj->channelId, pDevObj->timeStamp));

                                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT,
                                    VP880_ICR2_LEN, pLineObj->icr2Values);
                            }
                        }
                        break;
#endif

                    case VP_LINE_INTERNAL_TESTTERM_TIMER: {
                        /* Apply new bias settings to keep tip/ring near battery. */

                        /* While the internal test termination is applied, the
                         * line object ICR1 cache is used to keep track of what
                         * ICR1 needs to be once the internal test termination
                         * is removed.  It will not match the actual register
                         * value.  This is part of the internal test termination
                         * ICR1 override, so don't copy it to the line object */
                        uint8 icr1Reg[VP880_ICR1_LEN];
                        icr1Reg[0] = 0xFF;
                        icr1Reg[1] = 0x18;
                        icr1Reg[2] = 0xFF;
                        icr1Reg[3] = 0x04;
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_INTERNAL_TESTTERM_TIMER Expired on Ch %d at time %d ICR1 Write: 0x%02X 0x%02X 0x%02X 0x%02X",
                            pLineObj->channelId, pDevObj->timeStamp,
                            icr1Reg[0], icr1Reg[1], icr1Reg[2], icr1Reg[3]));
                        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR1_WRT, VP880_ICR1_LEN, icr1Reg);
                        break;
                    }

#ifdef VP880_LP_SUPPORT
                    case VP_LINE_SPEEDUP_RECOVERY_TIMER:
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_SPEEDUP_RECOVERY_TIMER Expired on Ch %d at time %d",
                            pLineObj->channelId, pDevObj->timeStamp));

                        pLineObj->icr2Values[VP880_ICR2_SPEEDUP_INDEX] &=
                            (uint8)(~(VP880_ICR2_MET_SPEED_CTRL | VP880_ICR2_BAT_SPEED_CTRL));

                        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT,
                            VP880_ICR2_LEN, pLineObj->icr2Values);

                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("ICR2_WRT 0x%02X 0x%02X 0x%02X 0x%02X",
                            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                            pLineObj->icr2Values[2], pLineObj->icr2Values[3]));
                        break;
#endif

                    default:
                        /*
                         * If we don't know what the timer is most likely it was masking hook or
                         * some other line detect activity. It can't hurt to force a signaling
                         * register read. If there are pending masks, the device specific API will
                         * freeze whatever status is necessary regardless of whether the signaling
                         * register is read or not.
                         */
                        pDevObj->state |= VP_DEV_PENDING_INT;
                        break;
                } /* Switch (timerType) */
            } else { /* If timer has not expired, keep it going */
                fxsTimers->timer[lineTimerType] |= VP_ACTIVATE_TIMER;
            }
        } /* if timerType is active     */
    } /* Loop through all timerTypes for chanID */

    return;
}

/**
 * Vp880ServiceDiscExitTimer()
 *  This function services active Disconnect Exit API timers.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  Active Disconnect Exit Timers for the current line have been serviced.
 */
void
Vp880ServiceDiscExitTimer(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;
#ifdef VP880_LP_SUPPORT
    uint8 channelId = pLineObj->channelId;
#endif
    uint8 mpiBuffer[6 + VP880_REGULATOR_PARAM_LEN + VP880_ICR2_LEN + VP880_ICR3_LEN
                      + VP880_ICR1_LEN + VP880_SYS_STATE_LEN + VP880_ICR4_LEN];
    uint8 mpiIndex = 0;
    uint8 speedUpByte;
    bool stateChange = FALSE;

    if (pLineObj->status & VP880_IS_FXO) {
        return;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("Disconnect Exit Timer State %d: Line Status 0x%04X on Ch %d time %d",
        pLineObj->discTimerExitState, pLineObj->status, pLineObj->channelId, pDevObj->timeStamp));

    if (pLineObj->discTimerExitState == 0) {
        if (VpIsLowPowerTermType(pLineObj->termType)) {
#ifdef VP880_LP_SUPPORT
            bool lpExit = FALSE;

#define ICR1_WRT_MASK       (0x1)
#define ICR2_WRT_MASK       (0x2)
#define ICR3_WRT_MASK       (0x4)
#define ICR4_WRT_MASK       (0x8)
#define SYS_STATE_WRT_MASK  (0x10)
            uint8 mpiCmdBitMask = ICR2_WRT_MASK;    /* No matter what, ICR2 is being modified */

            if(pLineObj->lineState.calType != VP_CSLAC_CAL_NONE) {
                pLineObj->nextSlicValue &= VP880_SS_POLARITY_MASK;
                pLineObj->nextSlicValue |= VP880_SS_ACTIVE;
            }

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("LPM Disconnect Timer at time %d: currentState %d nextSlicValue 0x%02X usrState %d",
                pDevObj->timeStamp, pLineObj->lineState.currentState,
                pLineObj->nextSlicValue, pLineObj->lineState.usrCurrent));

            /*
             * If we're in Disconnect when this timer expires (entering Disconnect)
             * need to correct ICR bits set to force feed toward ground and minimize
             * power.
             */
            if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
                /* Entering VP_LINE_DISCONNECT... */
                pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX] |= VP880_ICR2_ILA_DAC;
                pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX] &= ~VP880_ICR2_VOC_DAC_SENSE;
                pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX+1] &= ~VP880_ICR2_ILA_DAC;

                /* Disable Line Control if in Disconnect */
                pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX+1] &= ~VP880_ICR3_LINE_CTRL;
                mpiCmdBitMask |= (ICR2_WRT_MASK | ICR3_WRT_MASK);
            } else if (pLineObj->nextSlicValue == VP880_SS_DISCONNECT) {
                /* Entering Low Power Mode VP_LINE_STANDBY... */
                if ((pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] & 0x0D) != 0x0D) {
                    pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] &= ~VP880_FLOOR_VOLTAGE_MASK;
                    pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] |= 0x0D;   /* 70V */

                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_REGULATOR_PARAM_WRT,
                        VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);

                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Line Entering LPM Standby Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                        pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                        pDevObj->timeStamp));
                }

                pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] =
                    (VP880_ICR2_TIP_SENSE | VP880_ICR2_RING_SENSE |
                     VP880_ICR2_ILA_DAC | VP880_ICR2_FEED_SENSE);
                pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] =
                    (VP880_ICR2_TIP_SENSE | VP880_ICR2_RING_SENSE | VP880_ICR2_FEED_SENSE);
                mpiCmdBitMask |= ICR2_WRT_MASK;
            } else {
                /*
                 * Entering a non-LPM state. Make sure all ICR values are correct. These are
                 * slightly modified below to handle Disconnect Exit (Tip, Ring, Line Bias). But
                 * most important here is to reconfingure the SLIC to provide battery and work in
                 * (normal) current monitoring mode.
                 */
                 Vp880SetLPRegisters(pLineObj, FALSE);
                 mpiCmdBitMask |= (ICR1_WRT_MASK | ICR2_WRT_MASK | ICR3_WRT_MASK | ICR4_WRT_MASK);
                 lpExit = TRUE;

                /*
                 * The settings from calling Vp880SetLPRegisters(pLineObj, FALSE) are:
                 *
                 *   pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX] &= ~VP880_ICR3_LINE_CTRL;
                 *
                 *   pLineObj->icr4Values[VP880_ICR4_SUP_INDEX] &=
                 *       (uint8)(~(VP880_ICR4_SUP_DAC_CTRL | VP880_ICR4_SUP_DET_CTRL |
                 *                 VP880_ICR4_SUP_POL_CTRL));
                 *
                 *   - Remove previously set SW control of ICR1 -
                 *   pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] &=
                 *       ~VP880_ICR1_LINE_BIAS_OVERRIDE;
                 *
                 *   pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &=
                 *       (uint8)(~(VP880_ICR2_RING_SENSE | VP880_ICR2_TIP_SENSE |
                 *                 VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE));
                 */
            }

            if (pLineObj->nextSlicValue == VP880_SS_TIP_OPEN) {
                /*
                 * This already occurred if we exited into Tip Open from Disconnect
                 * when the other line was in a LPM state (because setting to Tip
                 * Open takes the lines out of LPM). Lower level code will check
                 * to make sure this isn't redundant.
                 */
                Vp880GroundStartProc(TRUE, pLineCtx, 0x00, pLineObj->nextSlicValue);
            } else {
                /*
                 * Release Tip Bias Override and clear Line Bias values. If we're in a normal
                 * feed state (see next "if ()" condition) the Line Bias will be set back to
                 * normal values.
                 */
                pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] &=
                    ~VP880_ICR1_TIP_BIAS_OVERRIDE;
                pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
                    (uint8)(~(VP880_ICR1_TIP_BIAS_OVERRIDE | VP880_ICR1_LINE_BIAS_OVERRIDE));

                /*
                 * Restore Normal SLIC Bias if NOT in Disconnect. Otherwise the SLIC bias remains
                 * set to drive T/R near 0V
                 */
                if (pLineObj->lineState.currentState != VP_LINE_DISCONNECT) {
                    pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
                        VP880_ICR1_LINE_BIAS_OVERRIDE_NORM;
                }

                /* Release Ring Bias Override. */
                pLineObj->icr1Values[VP880_ICR1_RING_BIAS_OVERRIDE_LOCATION] &=
                    ~VP880_ICR1_RING_BIAS_OVERRIDE;

                mpiCmdBitMask |= ICR1_WRT_MASK;

                Vp880UpdateBufferChanSel(pDevObj, channelId, pLineObj->nextSlicValue, TRUE);
                if (pLineObj->slicValueCache != pLineObj->nextSlicValue) {
                    mpiCmdBitMask |= SYS_STATE_WRT_MASK;
                    stateChange = TRUE;
                }
            }

            /* Check if the sequence order is LPM Exit or same as previous API Releases */
            if (lpExit) {
                /* This just writes ICR[1:4] and the SLIC State Register in a specific order that
                 * works well for LPM Exit. Otherwise, there's nothing special about this function.
                 * We're only checking for lpExit because don't want to write values that have not
                 * been set above.
                 */
                Vp880WriteLPExitRegisters(pLineCtx, deviceId, ecVal, &pLineObj->nextSlicValue);
            } else {
                if (mpiCmdBitMask & ICR2_WRT_MASK) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Disconnect Exit Timer: ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                    pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                    pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                        pLineObj->channelId, pDevObj->timeStamp));

                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                        VP880_ICR2_LEN, pLineObj->icr2Values);
                }
                if (mpiCmdBitMask & ICR1_WRT_MASK) {
                    mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);
                }
                if (mpiCmdBitMask & ICR3_WRT_MASK) {
                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
                        VP880_ICR3_LEN, pLineObj->icr3Values);

                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Disconnect Exit Timer: ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                        pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                        pLineObj->icr3Values[2], pLineObj->icr3Values[3],
                        pLineObj->channelId, pDevObj->timeStamp));
                }
                if (mpiCmdBitMask & SYS_STATE_WRT_MASK) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("LPM Disconnect Exit Timer: Setting Ch %d to State 0x%02X from 0x%02X at time %d",
                        channelId, pLineObj->nextSlicValue, pLineObj->slicValueCache, pDevObj->timeStamp));

                    pLineObj->slicValueCache = pLineObj->nextSlicValue;
                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                        VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
                }
                /*
                 * Technically possible that mpiIndex = 0, but it shouldn't happen. This is
                 * just a safety net to prevent sending garbage to the device.
                 */
                if (mpiIndex > 0) {
                    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
                }
            }
#endif  /* VP880_LP_SUPPORT */
        } else {
            /* Non-LPM Disconnect Enter/Exit Additional Handling */
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Disconnect Exit Timer for Non-LPM Termination Type Ch %d at time %d",
                pLineObj->channelId, pDevObj->timeStamp));

            /*
             * Battery Speedup Control makes sense only on ABS and could be
             * damaging if set.
             */
            if (pDevObj->stateInt & VP880_IS_ABS) { /* ABS */
                speedUpByte = VP880_ICR2_MET_SPEED_CTRL;
            } else {    /* other = Tracker */
                speedUpByte = (VP880_ICR2_MET_SPEED_CTRL | VP880_ICR2_BAT_SPEED_CTRL);
            }

            if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
                /* Line is in Disconnect */
                /*
                 * Line State is already set, need to disable SLIC Bias in ICR1 and
                 * disable DC Feed/Battery Hold Speed in preperation for Disconnect
                 * exit.
                 */

                pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
                    ~(VP880_ICR1_LINE_BIAS_OVERRIDE);

                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Non-LPM Disconnect Exit Timer: ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d at time %d",
                    pLineObj->icr1Values[0], pLineObj->icr1Values[1],
                    pLineObj->icr1Values[2], pLineObj->icr1Values[3],
                    pLineObj->channelId, pDevObj->timeStamp));

                mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

                pLineObj->icr2Values[VP880_ICR2_SPEEDUP_INDEX] |= speedUpByte;
                pLineObj->icr2Values[VP880_ICR2_SPEEDUP_INDEX+1] |= speedUpByte;
            } else {
                /* Line is exiting Disconnect */
                /* Line State needs to be corrected. SLIC Bias is already set */
                if (pLineObj->slicValueCache != pLineObj->nextSlicValue) {
                    pLineObj->slicValueCache = pLineObj->nextSlicValue;
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Non-LPM Disconnect Exit Timer: Setting Ch %d to State 0x%02X at time %d",
                        pLineObj->channelId, pLineObj->slicValueCache, pDevObj->timeStamp));

                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                        VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
                    stateChange = TRUE;
                }

                /* Restore DC Feed and Battery Speedup Hold Times */
                pLineObj->icr2Values[VP880_ICR2_SPEEDUP_INDEX] &= ~speedUpByte;
            }
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Non-LPM Disconnect Exit Timer: ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d at time %d",
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                pLineObj->channelId, pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                VP880_ICR2_LEN, pLineObj->icr2Values);

            VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
        }
    } else { /* Disconnect Exit State = 1 (all state changes done, end of debounce time */
        /*
         * Reset Disconnect Exit State for next use, although this should be set in the API where
         * the Disconnect Timer is being set.
         */
        pLineObj->discTimerExitState = 0;
    }

#ifdef VP880_LP_SUPPORT
    if(stateChange) {
        /*
         * If a state change was just made, we need to re-enable the Disconnect Hook/Groundkey
         * debounce mask because line conditions are not immediatly stable.
         */
        pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
            MS_TO_TICKRATE(VP_DISCONNECT_RECOVERY_TIME,
                pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
        pLineObj->discTimerExitState = 1;
    } else {
        if (VpIsLowPowerTermType(pLineObj->termType)) {
            VpCslacLineCondType tempHookSt =
                (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK);

            /*
             * Resolve hook state to avoid false leaky line test. Normal termination
             * types ok since they don't run leaky line test.
             */

            if ((pLineObj->lineState.currentState != VP_LINE_DISCONNECT)
             && (pLineObj->lineState.currentState != VP_LINE_TIP_OPEN)) {
                if (!(pLineObj->status & VP880_LOW_POWER_EN)) {
                    if (!(pDevObj->intReg[channelId]) & VP880_HOOK1_MASK) {
                        if (tempHookSt != 0) {
                            VP_HOOK(VpLineCtxType, pLineCtx,
                                ("Ch %d updating to On-Hook in non-LPM State at time %d",
                                channelId, pDevObj->timeStamp));
                            pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                            Vp880UpdateHookInfo(pLineObj, pDevObj);
                        }
                    } else {
                        if (tempHookSt != VP_CSLAC_HOOK) {
                            VP_HOOK(VpLineCtxType, pLineCtx,
                                ("Ch %d updating to Off-Hook in non-LPM State at time %d",
                                channelId, pDevObj->timeStamp));
                            pLineObj->lineState.condition |= VP_CSLAC_HOOK;
                            Vp880UpdateHookInfo(pLineObj, pDevObj);
                        }
                    }
                } else {
                    if (pDevObj->intReg[channelId] & VP880_HOOK1_MASK) {
                        if (tempHookSt != 0) {
                            VP_HOOK(VpLineCtxType, pLineCtx,
                                ("Ch %d updating to On-Hook in LPM State at time %d",
                                channelId, pDevObj->timeStamp));
                            pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                            Vp880UpdateHookInfo(pLineObj, pDevObj);
                        }
                    } else {
                        if (tempHookSt != VP_CSLAC_HOOK) {
                            VP_HOOK(VpLineCtxType, pLineCtx,
                                ("Ch %d updating to Off-Hook in LPM State at time %d",
                                channelId, pDevObj->timeStamp));
                            pLineObj->lineState.condition |= VP_CSLAC_HOOK;
                            Vp880UpdateHookInfo(pLineObj, pDevObj);
                        }
                    }
                }
            }
        }
    }
#endif  /* VP880_LP_SUPPORT */
    /*
     * Force an interrupt read in all cases, just to update the signaling information in the
     * device object. If we don't do this for polling methods other than Simple Polled, the
     * interrupt and signaling information will not get properly serviced.
     */
    pDevObj->state |= VP_DEV_PENDING_INT;
}

#ifdef VP880_LP_SUPPORT
/**
 * Vp880UpdateHookInfo()
 *  This function updates the line object event and dial pulse data based on
 * hook and DP Detection status. It is a helper function only for the VP880
 * API used at the end of Disconnect Exit and Low Power Mode changes. It is not
 * used in mid-DP detection.
 *
 * Preconditions:
 *  None. Helper function only.
 *
 * Postconditions:
 *  Line object data is updated. Event is generated IF Dial Pulse detection is
 * disabled. If DP Detection is enabled, Dial Pulse state machine is updated.
 */
void
Vp880UpdateHookInfo(
    Vp880LineObjectType *pLineObj,
    Vp880DeviceObjectType *pDevObj)
{
    VP_HOOK(None, VP_NULL, ("Vp880UpdateHookInfo()+"));

    if (pLineObj->lineState.condition & VP_CSLAC_HOOK) {
        /*
         * Only post the off-hook event if it was NOT the last event posted
         * w.r.t. on-hook.
         *
         * Generate the event whether running API DP Detection or Not (because
         * initial off-hook is not a DP dependant parameter)
         */
        if (!(pLineObj->status & VP880_PREVIOUS_HOOK)) {
            VP_HOOK(None, VP_NULL,
                ("Generating Off-Hook Event Ch %d at time %d",
                pLineObj->channelId, pDevObj->timeStamp));
            pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_OFF;
        }

        /*
         * If using the VP-API-II Dial Pulse Detection, need to also update
         * the Dial Pulse State machine.
         */
        if (pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) {
            VP_HOOK(None, VP_NULL,
                ("Feeding Off-Hook information Ch %d to Dial Pulse State machine at time %d",
                pLineObj->channelId, pDevObj->timeStamp));

            pLineObj->dpStruct.hookSt = TRUE;
            pLineObj->dpStruct2.hookSt = TRUE;

            /*
             * Init prevents DP detection in this pass, but this update is
             * happening because the hook state is different after a major
             * transition. So not in the middle if DP detection.
             */
            VpInitDP(&pLineObj->dpStruct);
            VpInitDP(&pLineObj->dpStruct);

            pLineObj->lineEventHandle = VP_DP_PARAM1;
        }
    } else {
        if (pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
            /* VP-API-II NOT doing Dial Pulse Detection, generate event here. */
            VP_HOOK(None, VP_NULL,
                ("Generating On-Hook Event Ch %d at time %d",
                pLineObj->channelId, pDevObj->timeStamp));

            /*
             * Only post the on-hook event if it was NOT the last event posted
             * w.r.t. off-hook
             */
            if (pLineObj->status & VP880_PREVIOUS_HOOK) {
                pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_ON;
            }
        } else {
            /*
             * VP-API-II doing Dial Pulse Detection, update the Dial Pulse State
             *  machines and generate the Start Pulse Event. The normal API FXS
             * processing and DP State Machine will continue to handle on-hook
             * for the on-hook event.
             */
            VP_HOOK(None, VP_NULL,
                ("Feeding On-Hook information Ch %d to Dial Pulse State machine at time %d",
                pLineObj->channelId, pDevObj->timeStamp));

            /*
             * Note that this is not treated the same way as off-hook, because
             * in this case we want to generate the "Start Pulse" event first
             * before the on-hook event. State Machines using VP-API-II DP
             * Detection will generally be looking for both events.
             */
            pLineObj->dpStruct.hookSt = FALSE;
            pLineObj->dpStruct.lo_time = 0;
            pLineObj->dpStruct.lc_time = 0xFFFF;

            pLineObj->dpStruct2.hookSt = FALSE;
            pLineObj->dpStruct2.lo_time = 0;
            pLineObj->dpStruct2.lc_time = 0xFFFF;

            pLineObj->lineEventHandle = VP_DP_PARAM1;

            /* This event should not be generated if we just completed a Leaky Line Test and
             * "simply" determined that the line is still on-hook. This can occur for leaky line
             * conditions that cause long detection of loop close in the LPM condition. The
             * last hook event reported will have been on-hook, so we can easily check for this
             */
            if (pLineObj->status & VP880_PREVIOUS_HOOK) {   /* Last reported event was off-hook */
                pLineObj->lineEvents.signaling |= VP_LINE_EVID_STARTPULSE;
            }
        }
    }

    pLineObj->lineEventHandle = pDevObj->timeStamp;
}
#endif

/**
 * Vp880ServiceGroundStartTimer()
 *  This function services active Ground Start API timers.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  Active Ground Start Timers for the current line have been serviced.
 */
void
Vp880ServiceGroundStartTimer(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pLineObj->ecVal;

    uint8 mpiBuffer[2 * (2 + VP880_ICR3_LEN + VP880_ICR4_LEN)
                  + 3 + VP880_ICR1_LEN + VP880_ICR2_LEN + VP880_SYS_STATE_LEN];
    uint8 mpiIndex = 0;

    if (pLineObj->lineState.currentState != VP_LINE_TIP_OPEN) {
        if (pLineObj->gsTimerExitState == 0) {
            pLineObj->icr3Values[VP880_ICR3_LONG_LOOP_CTRL_LOCATION] &=
                ~VP880_ICR3_LONG_LOOP_CONTROL;

            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ground Key Timer: Write ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d Time %d",
                pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                pLineObj->icr3Values[2], pLineObj->icr3Values[3],
                pLineObj->channelId, pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
                VP880_ICR3_LEN, pLineObj->icr3Values);

            pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION] &= ~VP880_ICR4_GKEY_DET;

            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ground Key Timer: Write ICR4 0x%02X 0x%02X 0x%02X 0x%02X Ch %d Time %d",
                pLineObj->icr4Values[0], pLineObj->icr4Values[1],
                pLineObj->icr4Values[2], pLineObj->icr4Values[3],
                pLineObj->channelId, pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
                VP880_ICR4_LEN, pLineObj->icr4Values);

            if ((pLineObj->lineState.currentState == VP_LINE_RINGING)
             || (pLineObj->lineState.currentState == VP_LINE_RINGING_POLREV)) {
                Vp880UpdateBufferChanSel(pDevObj, pLineObj->channelId,
                    pLineObj->nextSlicValue, TRUE);
                pLineObj->slicValueCache = pLineObj->nextSlicValue;
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Ground Key Timer: Setting Ch %d to State 0x%02X at time %d",
                    pLineObj->channelId, pLineObj->nextSlicValue, pDevObj->timeStamp));
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }

            VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

            /* Final debounce to prevent spurious ground key events. */
            pLineObj->lineTimers.timers.timer[VP_LINE_GND_START_TIMER] =
                (MS_TO_TICKRATE(VP880_GND_START_DEBOUNCE,
                    pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

            /* Advance to next ground start exit timer state */
            pLineObj->gsTimerExitState = 1;

        } else {
            /*
             * Reset ground start exit timer state. This step should be unnecessary - done by
             * functions that start the Ground Start Exit timer
             */
            pLineObj->gsTimerExitState = 0;
        }
    } else {
        /*
         *  Line State is TIP_OPEN. Set ICR registers accordingly (NOTE: Some
         * will have been set prior to this point, but doesn't hurt to be
         * sure).
         */
        /*
         * These bits are set for LPM Standby and Disconnect. Clear just in case
         * we're coming from either of those states.
         */
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] &=
            ~(VP880_ICR1_LINE_BIAS_OVERRIDE);
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT,
            VP880_ICR1_LEN, pLineObj->icr1Values);

        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &=
            ~(VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE |
              VP880_ICR2_TIP_SENSE | VP880_ICR2_RING_SENSE);
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
            VP880_ICR2_LEN, pLineObj->icr2Values);

        pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX] &= ~VP880_ICR3_LINE_CTRL;
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
            VP880_ICR3_LEN, pLineObj->icr3Values);

        pLineObj->icr4Values[VP880_ICR4_SUP_INDEX] &=
            ~(VP880_ICR4_SUP_DAC_CTRL | VP880_ICR4_SUP_DET_CTRL | VP880_ICR4_SUP_POL_CTRL);
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
            VP880_ICR4_LEN, pLineObj->icr4Values);

        VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
    }
}
#endif

#ifdef VP880_FXO_SUPPORT
/**
 * Vp880ServiceFxoTimers()
 *  This function services active FXO API timers.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  All Active FXO Timers for the current line have been serviced.
 */
void
Vp880ServiceFxoTimers(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpFXOTimerType *pFxoTimer = &pLineObj->lineTimers.timers.fxoTimer;
    uint16 tickAdder =
        pDevObj->devProfileData.tickRate / (VP_CSLAC_TICKSTEP_0_5MS / 2);

    /* Increment the time since polrev was observed */
    if (pFxoTimer->timeLastPolRev  < (0x7FFF - tickAdder)) {
        /*
         * The time is in 0.25mS increments, but the device
         * tickrate is something else. Increment by the scaled
         * amount.
         */
        pFxoTimer->timeLastPolRev += tickAdder;
    } else {
        /* Max limit the value of last polrev value */
        pFxoTimer->timeLastPolRev = 0x7FFF;
    }

    /* Set tick adder for 1ms increments */
    tickAdder =
        pDevObj->devProfileData.tickRate / (VP_CSLAC_TICKSTEP_0_5MS * 2);

    if (((uint16)pFxoTimer->lastStateChange + tickAdder) > 255) {
        pFxoTimer->lastStateChange = 255;
    } else {
        pFxoTimer->lastStateChange += tickAdder;
    }

    if (((uint16)pFxoTimer->lastNotLiu - tickAdder) <= 0) {
        pFxoTimer->lastNotLiu = 0;
    } else {
        pFxoTimer->lastNotLiu-=tickAdder;
    }

    if (((uint16)pFxoTimer->fxoDiscIO2Change - tickAdder) <= 0) {
        pFxoTimer->fxoDiscIO2Change = 0;
    } else {
        pFxoTimer->fxoDiscIO2Change -= tickAdder;
    }

    if (pFxoTimer->disconnectDebounce >= tickAdder) {
        pFxoTimer->disconnectDebounce -= tickAdder;

        if (pFxoTimer->disconnectDebounce ==0) {
            if (pLineObj->preDisconnect ==
                (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_RAW_DISC)) {
                /*
                 * There is a change that persisted longer than the
                 * debounce interval. Evaluate and generate event
                 */
                pLineObj->lineEventHandle = pDevObj->timeStamp;

                switch(pLineObj->lineState.usrCurrent) {
                    case VP_LINE_FXO_TALK:
                    case VP_LINE_FXO_LOOP_CLOSE:
                        if (pLineObj->preDisconnect == VP_CSLAC_RAW_DISC) {
                            if (!(pLineObj->lineState.condition & VP_CSLAC_DISC)) {
                                pLineObj->lineState.condition |= VP_CSLAC_DISC;
                                pLineObj->lineEvents.fxo |= VP_LINE_EVID_DISCONNECT;
                            }
                        } else {
                            if (pLineObj->lineState.condition & VP_CSLAC_DISC) {
                                pLineObj->lineState.condition &= ~VP_CSLAC_DISC;
                                pLineObj->lineEvents.fxo |= VP_LINE_EVID_RECONNECT;
                            }
                        }
                        break;

                    default:
                        if (pLineObj->preDisconnect == VP_CSLAC_RAW_DISC) {
                            if (!(pLineObj->lineState.condition & VP_CSLAC_DISC)) {
                                pLineObj->lineState.condition |= VP_CSLAC_DISC;
                                pLineObj->lineEvents.fxo |= VP_LINE_EVID_FEED_DIS;
                            }
                        } else {
                            if (pLineObj->lineState.condition & VP_CSLAC_DISC) {
                                pLineObj->lineState.condition &= ~VP_CSLAC_DISC;
                                pLineObj->lineEvents.fxo |= VP_LINE_EVID_FEED_EN;
                            }
                        }
                        break;
                }
            }
        }
    }
}
#endif

#ifdef VP880_INCLUDE_TESTLINE_CODE
/**
 * Vp880GetRelayState()
 *  This function returns the current relay state of VP880 device.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized.
 *
 * Postconditions:
 *  The indicated relay state is returned for the given line.
 */
VpStatusType
Vp880GetRelayState(
    VpLineCtxType           *pLineCtx,
    VpRelayControlType      *pRstate)
{
    VpDevCtxType            *pDevCtx;
    Vp880DeviceObjectType   *pDevObj;
    Vp880LineObjectType     *pLineObj;
    VpDeviceIdType          deviceId;

    if(pLineCtx == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }

    pDevCtx = pLineCtx->pDevCtx;
    pLineObj = pLineCtx->pLineObj;
    pDevObj = pDevCtx->pDevObj;
    deviceId = pDevObj->deviceId;

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    *pRstate = pLineObj->relayState;
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}
#endif

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
/**
 * Vp880RegisterDump()
 *  Dump all known 880 device and line specific registers (for debug purposes).
 *
 * Returns:
 */
VpStatusType
Vp880RegisterDump(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId, ecVal, registerIndex, registerNumber;

    /* Sufficient size to hold a single MPI Read is all that is needed. */
    uint8 registerBuffer[20];

#define VP880_DEVICE_REGISTER_COUNT    13
    uint8 deviceRegs[VP880_DEVICE_REGISTER_COUNT][2] = {
        {VP880_DEVTYPE_RD, VP880_DEVTYPE_LEN},
        {VP880_DEV_MODE_RD, VP880_DEV_MODE_LEN},
        {VP880_TEST_REG1_RD, VP880_TEST_REG1_LEN},
        {VP880_TEST_REG2_RD, VP880_TEST_REG2_LEN},
        {VP880_DCR_RD, VP880_DCR_LEN},
        {VP880_OP_MODE_RD, VP880_OP_MODE_LEN},
        {VP880_XR_CS_RD, VP880_XR_CS_LEN},
        {VP880_NO_UL_SIGREG_RD, VP880_NO_UL_SIGREG_LEN},
        {VP880_INT_MASK_RD, VP880_INT_MASK_LEN},
        {VP880_REV_INFO_RD, VP880_REV_INFO_LEN},
        {VP880_REGULATOR_PARAM_RD, VP880_REGULATOR_PARAM_LEN},
        {VP880_REGULATOR_CTRL_RD, VP880_REGULATOR_CTRL_LEN},
        {VP880_INT_SWREG_PARAM_RD, VP880_INT_SWREG_PARAM_LEN}
    };

    char *deviceRegsName[VP880_DEVICE_REGISTER_COUNT] = {
        "VP880_DEVTYPE_RD",     "VP880_DEV_MODE_RD",        "VP880_TEST_REG1_RD",
        "VP880_TEST_REG2_RD",   "VP880_DCR_RD",             "VP880_OP_MODE_RD",
        "VP880_XR_CS_RD",       "VP880_NO_UL_SIGREG_RD",    "VP880_INT_MASK_RD",
        "VP880_REV_INFO_RD",    "VP880_REGULATOR_PARAM_RD", "VP880_REGULATOR_CTRL_RD",
        "VP880_INT_SWREG_PARAM_RD"
    };

    VpSysDebugPrintf("\n\rDevice Registers:\n\r");
    ecVal = pDevObj->ecVal;

    for (registerNumber = 0; registerNumber < VP880_DEVICE_REGISTER_COUNT; registerNumber++) {
        VpMpiCmdWrapper(deviceId, ecVal,
            deviceRegs[registerNumber][0], deviceRegs[registerNumber][1],
            registerBuffer);

        VpSysDebugPrintf("\n\r%s (0x%02X) ",
            deviceRegsName[registerNumber], deviceRegs[registerNumber][0]);
        for (registerIndex = 0;
             registerIndex < deviceRegs[registerNumber][1];
             registerIndex++) {
            VpSysDebugPrintf("0x%02X ", registerBuffer[registerIndex]);
        }
    }

    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
#define VP880_CHANNEL_REGISTER_COUNT    33
        uint8 channelRegs[VP880_CHANNEL_REGISTER_COUNT][2] = {
            {VP880_SYS_STATE_RD, VP880_SYS_STATE_LEN},
            {VP880_SS_CONFIG_RD, VP880_SS_CONFIG_LEN},
            {VP880_CONV_CFG_RD, VP880_CONV_CFG_LEN},
            {VP880_ICR1_RD, VP880_ICR1_LEN},
            {VP880_ICR2_RD, VP880_ICR2_LEN},
            {VP880_ICR3_RD, VP880_ICR3_LEN},
            {VP880_ICR4_RD, VP880_ICR4_LEN},
            {VP880_ICR5_RD, VP880_ICR5_LEN},
            {VP880_ICR6_RD, VP880_ICR6_LEN},
            {VP880_DISN_RD, VP880_DISN_LEN},
            {VP880_VP_GAIN_RD, VP880_VP_GAIN_LEN},
            {VP880_GR_GAIN_RD, VP880_GR_GAIN_LEN},
            {VP880_GX_GAIN_RD, VP880_GX_GAIN_LEN},
            {VP880_X_FILTER_RD, VP880_X_FILTER_LEN},
            {VP880_R_FILTER_RD, VP880_R_FILTER_LEN},
            {VP880_B1_FILTER_RD, VP880_B1_FILTER_LEN},
            {VP880_B2_FILTER_RD, VP880_B2_FILTER_LEN},
            {VP880_Z1_FILTER_RD, VP880_Z1_FILTER_LEN},
            {VP880_Z2_FILTER_RD, VP880_Z2_FILTER_LEN},
            {VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN},
            {VP880_OP_COND_RD, VP880_OP_COND_LEN},
            {VP880_IODATA_REG_RD, VP880_IODATA_REG_LEN},
            {VP880_IODIR_REG_RD, VP880_IODIR_REG_LEN},
            {VP880_BAT_CALIBRATION_RD, VP880_BAT_CALIBRATION_LEN},
            {VP880_TX_TS_RD, VP880_TX_TS_LEN},
            {VP880_RX_TS_RD, VP880_RX_TS_LEN},
            {VP880_DC_FEED_RD, VP880_DC_FEED_LEN},
            {VP880_LOOP_SUP_RD, VP880_LOOP_SUP_LEN},
            {VP880_SIGA_PARAMS_RD, VP880_SIGA_PARAMS_LEN},
            {VP880_SIGCD_PARAMS_RD, VP880_SIGCD_PARAMS_LEN},
            {VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN},
            {VP880_CID_PARAM_RD, VP880_CID_PARAM_LEN},
            {VP880_METERING_PARAM_RD, VP880_METERING_PARAM_LEN}
        };

        char *registerName[VP880_CHANNEL_REGISTER_COUNT] = {
            "VP880_SYS_STATE_RD",       "VP880_SS_CONFIG_RD",   "VP880_CONV_CFG_RD",
            "VP880_ICR1_RD",            "VP880_ICR2_RD",        "VP880_ICR3_RD",
            "VP880_ICR4_RD",            "VP880_ICR5_RD",        "VP880_ICR6_RD",
            "VP880_DISN_RD",            "VP880_VP_GAIN_RD",     "VP880_GR_GAIN_RD",
            "VP880_GX_GAIN_RD",         "VP880_X_FILTER_RD",    "VP880_R_FILTER_RD",
            "VP880_B1_FILTER_RD",       "VP880_B2_FILTER_RD",   "VP880_Z1_FILTER_RD",
            "VP880_Z2_FILTER_RD",       "VP880_OP_FUNC_RD",     "VP880_OP_COND_RD",
            "VP880_IODATA_REG_RD",      "VP880_IODIR_REG_RD",   "VP880_BAT_CALIBRATION_RD",
            "VP880_TX_TS_RD",           "VP880_RX_TS_RD",       "VP880_DC_FEED_RD",
            "VP880_LOOP_SUP_RD",        "VP880_SIGA_PARAMS_RD", "VP880_SIGCD_PARAMS_RD",
            "VP880_CADENCE_TIMER_RD",   "VP880_CID_PARAM_RD",   "VP880_METERING_PARAM_RD"
        };

        ecVal = (channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2;
        ecVal |= pDevObj->ecVal;

        VpSysDebugPrintf("\n\rCHANNEL %d", channelId);
        for (registerNumber = 0; registerNumber < VP880_CHANNEL_REGISTER_COUNT; registerNumber++) {
            VpMpiCmdWrapper(deviceId, ecVal,
                channelRegs[registerNumber][0], channelRegs[registerNumber][1],
                registerBuffer);

            VpSysDebugPrintf("\n\r%s (0x%02X) ",
                registerName[registerNumber], channelRegs[registerNumber][0]);
            for (registerIndex = 0;
                 registerIndex < channelRegs[registerNumber][1];
                 registerIndex++) {
                VpSysDebugPrintf("0x%02X ", registerBuffer[registerIndex]);
            }
        }
    }

    VpSysDebugPrintf("\n\r");

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880ObjectDump()
 *  Dump 880 device and line object data. Upper level calls may limit this to
 * device object OR line object, but there's no reason we need to enforce that.
 *
 * This function is for SW Apps debug purposes only. Not to be documented for
 * customer purposes. It will significantly increase the driver size, implement
 * so as totally removed when Debug Error is not enabled.
 *
 * Returns: VP_STATUS_SUCCESS
 */
VpStatusType
Vp880ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx)
{
    if (pDevCtx != VP_NULL) {
        Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

        VP_PRINT_DEVICE_ID(pDevObj->deviceId);

        VpPrintStaticInfoStruct(&pDevObj->staticInfo);
        VpPrintDynamicInfoStruct(&pDevObj->dynamicInfo);
        VpPrintStateInformation(pDevObj->state);

        VpSysDebugPrintf("\n\rpDevObj->stateInt = 0x%08lX", pDevObj->stateInt);

        VpPrintDeviceProfileStruct(VP_DEV_880_SERIES, &pDevObj->devProfileData);

        VpPrintEventMaskStruct(TRUE, TRUE, &pDevObj->deviceEventsMask);
        VpPrintEventMaskStruct(TRUE, FALSE, &pDevObj->deviceEvents);

        VpPrintEventHandle(pDevObj->eventHandle);
        VpPrintTimeStamp(pDevObj->timeStamp);

        VpPrintGetResultsOptionStruct(&pDevObj->getResultsOption);
        VpPrintCriticalFltStruct(&pDevObj->criticalFault);
        VpPrintRelGainResultsStruct(&pDevObj->relGainResults);

#ifdef VP880_FXS_SUPPORT
        VpSysDebugPrintf("\n\n\rpDevObj->swParams = 0x%02X 0x%02X 0x%02X",
            pDevObj->swParams[0], pDevObj->swParams[1], pDevObj->swParams[2]);
        VpSysDebugPrintf("\n\n\rpDevObj->swParamsCache = 0x%02X 0x%02X 0x%02X",
            pDevObj->swParamsCache[0], pDevObj->swParamsCache[1],
            pDevObj->swParamsCache[2]);
        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->intSwParams =");
            for (byteCount = 0; byteCount < VP880_INT_SWREG_PARAM_LEN; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->intSwParams[byteCount]);
            }
        }
        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->intSwParamsFR =");
            for (byteCount = 0; byteCount < VP880_INT_SWREG_PARAM_LEN; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->intSwParamsFR[byteCount]);
            }
        }
#endif

        VpSysDebugPrintf("\n\n\rpDevObj->devMode = 0x%02X", pDevObj->devMode[0]);

        VpPrintDeviceTimers(pDevObj->devTimer);
        VpPrintDeviceProfileTable(&pDevObj->devProfileTable);
        VpPrintProfileTableEntry(&pDevObj->profEntry);

        VpSysDebugPrintf("\n\n\rpDevObj->intReg = 0x%02X 0x%02X",
            pDevObj->intReg[0], pDevObj->intReg[1]);
        VpSysDebugPrintf("\n\rpDevObj->intReg2 = 0x%02X 0x%02X",
            pDevObj->intReg2[0], pDevObj->intReg2[1]);

        VpPrintResponseData(pDevObj->responseData);
        VpPrintTxBufferRate(pDevObj->txBufferDataRate);

        VpSysDebugPrintf("\n\n\rpDevObj->mpiLen = %d", pDevObj->mpiLen);
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->mpiData =");
            for (byteCount = 0; byteCount < VP880_MAX_MPI_DATA; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->mpiData[byteCount]);
            }
        }
#endif

#ifdef VP880_FXS_SUPPORT
        VpPrintPulseSpecs(0, &pDevObj->pulseSpecs);
        VpPrintPulseSpecs(1, &pDevObj->pulseSpecs2);
#endif

        VpSysDebugPrintf("\n\n\rpDevObj->debugSelectMask = 0x%08lX",
            pDevObj->debugSelectMask);

        VpSysDebugPrintf("\n\n\rpDevObj->ecVal = 0x%02X", pDevObj->ecVal);

        VpSysDebugPrintf("\n\n\rpDevObj->lastCodecChange = %d",
            pDevObj->lastCodecChange);

    }

    if (pLineCtx != VP_NULL) {
        Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;

        VP_PRINT_LINE_ID(pLineObj->lineId);

        VpSysDebugPrintf("\n\n\rpLineObj->channelId = %d\npLineObj->ecVal = 0x%02X",
            pLineObj->channelId, pLineObj->ecVal);

        VpPrintTermType(pLineObj->termType);

        VpSysDebugPrintf("\n\rpLineObj->status = 0x%04X ", pLineObj->status);
        VpSysDebugPrintf("\n\rpLineObj->debugSelectMask = 0x%08lX ", pLineObj->debugSelectMask);

        VpPrintEventMaskStruct(FALSE, TRUE, &pLineObj->lineEventsMask);
        VpPrintEventMaskStruct(FALSE, FALSE, &pLineObj->lineEvents);

#ifdef VP880_FXS_SUPPORT
        VpSysDebugPrintf("\n\n\rpLineObj->pulseMode = %s",
            ((pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) ?
            "VP_OPTION_PULSE_DECODE_ON" : "VP_OPTION_PULSE_DECODE_OFF"));

        VpPrintDPStateMachine(0, &pLineObj->dpStruct);
        VpPrintDPStateMachine(1, &pLineObj->dpStruct2);
#endif
        VpSysDebugPrintf("\n\n\rpLineObj->slicValueCache = 0x%02X", pLineObj->slicValueCache);
        VpSysDebugPrintf("\n\rpLineObj->nextSlicValue = 0x%02X", pLineObj->nextSlicValue);
        VpPrintApiIntLineState(&pLineObj->lineState);
        VpSysDebugPrintf("\n\rpLineObj->opCond = 0x%02X", pLineObj->opCond[0]);
        VpSysDebugPrintf("\n\rpLineObj->loopSup = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->loopSup[0], pLineObj->loopSup[1],
            pLineObj->loopSup[2], pLineObj->loopSup[3]);

#ifdef VP880_FXS_SUPPORT
        VpSysDebugPrintf("\n\n\rpLineObj->icr1 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr1Values[0], pLineObj->icr1Values[1],
            pLineObj->icr1Values[2], pLineObj->icr1Values[3]);
        VpSysDebugPrintf("\n\rpLineObj->icr2 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
            pLineObj->icr2Values[2], pLineObj->icr2Values[3]);
        VpSysDebugPrintf("\n\rpLineObj->icr3 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr3Values[0], pLineObj->icr3Values[1],
            pLineObj->icr3Values[2], pLineObj->icr3Values[3]);
        VpSysDebugPrintf("\n\rpLineObj->icr4 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr4Values[0], pLineObj->icr4Values[1],
            pLineObj->icr4Values[2], pLineObj->icr4Values[3]);
        VpSysDebugPrintf("\n\rpLineObj->icr6 (DC Cal) = 0x%02X 0x%02X",
            pLineObj->icr6Values[0], pLineObj->icr6Values[1]);
#endif
        VpSysDebugPrintf("\n\rpLineObj->sigGenCtrl = 0x%02X", pLineObj->sigGenCtrl[0]);

        VpSysDebugPrintf("\n\n\rpLineObj->gain.gxInt = 0x%04X", pLineObj->gain.gxInt);
        VpSysDebugPrintf("\n\rpLineObj->gain.grInt = 0x%04X", pLineObj->gain.grInt);
        VpSysDebugPrintf("\n\rpLineObj->gain.absGxGain = %d", pLineObj->gain.absGxGain);
        VpSysDebugPrintf("\n\rpLineObj->gain.absGrGain = %d", pLineObj->gain.absGrGain);

        VpSysDebugPrintf("\n\n\rpLineObj->lineEventHandle = 0x%02X", pLineObj->lineEventHandle);
        VpSysDebugPrintf("\n\rpLineObj->signaling1 = 0x%04X", pLineObj->signaling1);
        VpSysDebugPrintf("\n\rpLineObj->signaling2 = 0x%04X", pLineObj->signaling2);
        VpSysDebugPrintf("\n\rpLineObj->signalingData = 0x%02X", pLineObj->signalingData);
        VpSysDebugPrintf("\n\rpLineObj->processData = 0x%04X", pLineObj->processData);
        VpSysDebugPrintf("\n\rpLineObj->responseData = 0x%04X", pLineObj->responseData);
        VpSysDebugPrintf("\n\rpLineObj->dtmfDigitSense = 0x%04X", pLineObj->dtmfDigitSense);

        VpPrintOptionCodecType(pLineObj->codec);
        VpPrintOptionPcmTxRxCntrlType(pLineObj->pcmTxRxCtrl);

#ifdef VP_CSLAC_SEQ_EN
        {
            uint8 intSeqData;
            VpSysDebugPrintf("\npLineObj->intSequence =");
            for (intSeqData = 0; intSeqData < VP880_INT_SEQ_LEN; intSeqData++) {
                VpSysDebugPrintf(" 0x%02X", pLineObj->intSequence[intSeqData]);
            }
        }
        VpPrintSeqDataType(&pLineObj->cadence);
#endif
        VpPrint880CalLineData(&pLineObj->calLineData);
        VpPrintVpCslacTimerStruct(&pLineObj->lineTimers);

#ifdef VP880_FXS_SUPPORT
        VpSysDebugPrintf("\n\nFXS ONLY DATA:");
#ifdef VP880_LP_SUPPORT
         VpSysDebugPrintf("\n\rpLineObj->leakyLineCnt = %d", pLineObj->leakyLineCnt);
#endif
        VpSysDebugPrintf("\n\rpLineObj->ringingParams = 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->ringingParams[0], pLineObj->ringingParams[1],
            pLineObj->ringingParams[2], pLineObj->ringingParams[3],
            pLineObj->ringingParams[4], pLineObj->ringingParams[5],
            pLineObj->ringingParams[6], pLineObj->ringingParams[7],
            pLineObj->ringingParams[8], pLineObj->ringingParams[9],
            pLineObj->ringingParams[10]);
        VpSysDebugPrintf("\n\rpLineObj->hookHysteresis = %d", pLineObj->hookHysteresis);
        VpSysDebugPrintf("\n\rpLineObj->internalTestTermApplied = %s",
            ((pLineObj->internalTestTermApplied == TRUE) ? "TRUE" : "FALSE"));
        VpPrintOptionRingControlType(&pLineObj->ringCtrl);
        VpPrintRelayControlType(pLineObj->relayState);

#ifdef VP_CSLAC_SEQ_EN
        VpSysDebugPrintf("\n\rpLineObj->pRingingCadence = %p", pLineObj->pRingingCadence);
        VpSysDebugPrintf("\n\rpLineObj->pCidProfileType1 = %p", pLineObj->pCidProfileType1);
        VpSysDebugPrintf("\n\rpLineObj->suspendCid = %s",
            ((pLineObj->suspendCid == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpLineObj->tickBeginState = 0x%02X ", pLineObj->tickBeginState[0]);
        VpPrintCallerIdType(&pLineObj->callerId);
        VpPrintCidSeqDataType(&pLineObj->cidSeq);
#endif
#endif
    }

    return VP_STATUS_SUCCESS;
}

void
VpPrint880CalLineData(
    Vp880CalLineData *calLineData)
{
    VpSysDebugPrintf("\n\n\rpLineObj->calLineData.calDone = %s",
        ((calLineData->calDone == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpLineObj->calLineData.reversePol = %s",
        ((calLineData->reversePol == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpLineObj->calLineData.forceCalDataWrite = %s",
        ((calLineData->forceCalDataWrite == TRUE) ? "TRUE" : "FALSE"));

    {
        uint8 dcFeedIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.dcFeedRef =");
        for (dcFeedIndex = 0; dcFeedIndex < VP880_DC_FEED_LEN; dcFeedIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->dcFeedRef[dcFeedIndex]);
        }
        VpSysDebugPrintf("\npLineObj->calLineData.dcFeed =");
        for (dcFeedIndex = 0; dcFeedIndex < VP880_DC_FEED_LEN; dcFeedIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->dcFeed[dcFeedIndex]);
        }
        VpSysDebugPrintf("\npLineObj->calLineData.dcFeedPr =");
        for (dcFeedIndex = 0; dcFeedIndex < VP880_DC_FEED_LEN; dcFeedIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->dcFeedPr[dcFeedIndex]);
        }
    }
    {
        uint8 icr2Index;
        VpSysDebugPrintf("\npLineObj->calLineData.icr2 =");
        for (icr2Index = 0; icr2Index < VP880_ICR2_LEN; icr2Index++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->icr2[icr2Index]);
        }
    }
    {
        uint8 disnIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.disnVal =");
        for (disnIndex = 0; disnIndex < VP880_DISN_LEN; disnIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->disnVal[disnIndex]);
        }
    }
    {
        uint8 vpGainIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.vpGain =");
        for (vpGainIndex = 0; vpGainIndex < VP880_VP_GAIN_LEN; vpGainIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->vpGain[vpGainIndex]);
        }
    }
    {
        uint8 loopSupIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.loopSup =");
        for (loopSupIndex = 0; loopSupIndex < VP880_LOOP_SUP_LEN; loopSupIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->loopSup[loopSupIndex]);
        }
    }
    {
        uint8 sigGenAIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.sigGenA =");
        for (sigGenAIndex = 0; sigGenAIndex < VP880_SIGA_PARAMS_LEN; sigGenAIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->sigGenA[sigGenAIndex]);
        }
    }
    {
        uint8 calRegIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.calReg =");
        for (calRegIndex = 0; calRegIndex < VP880_ICR6_LEN; calRegIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->calReg[calRegIndex]);
        }
    }
    {
        uint8 typeDataIndex;
        uint8 *pTypeData = (uint8 *)(&calLineData->typeData);
        uint8 typeDataSize = sizeof(Vp880CalTypeData);
        VpSysDebugPrintf("\npLineObj->calLineData.typeData =");
        for (typeDataIndex = 0; typeDataIndex < typeDataSize; typeDataIndex++) {
            if (!(typeDataIndex % 10)) {
                VpSysDebugPrintf("\n\t");
            }
            VpSysDebugPrintf(" 0x%02X", *pTypeData);
            pTypeData++;
        }
    }

    VpSysDebugPrintf("\npLineObj->calLineData.codecReg = 0x%02X", calLineData->codecReg);
    VpSysDebugPrintf("\npLineObj->calLineData.calState = 0x%04X", calLineData->calLineState);
    VpSysDebugPrintf("\npLineObj->calLineData.sysState = 0x%02X", calLineData->sysState);
    VpSysDebugPrintf("\npLineObj->calLineData.vasStart = %d", calLineData->vasStart);
    VpSysDebugPrintf("\npLineObj->calLineData.minVas = %d\n", calLineData->minVas);
}

#endif
#endif

