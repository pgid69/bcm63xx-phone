/** \file vp890_query.c
 * vp890_query.c
 *
 *  This file contains the implementation of the VP-API 890 Series
 *  Status and Query Functions.
 *
 * Copyright (c) 2012, Microsemi
 *
 * $Revision: 11388 $
 * $LastChangedDate: 2014-04-29 15:01:35 -0500 (Tue, 29 Apr 2014) $
 */

/* INCLUDES */
#include    "vp_api.h"

#if defined (VP_CC_890_SERIES)  /* Compile only if required */

#include    "vp_api_int.h"
#include    "vp890_api_int.h"
#include    "sys_service.h"

/* ========================
    Local Type Definitions
   ======================== */

typedef enum Vp890EventIndexType {
    VP890_EVT_IDX_FAULT = 0,
    VP890_EVT_IDX_SIGNALING,
    VP890_EVT_IDX_RESPONSE,
    VP890_EVT_IDX_PROCESS,
    VP890_EVT_IDX_FXO,
    VP890_EVT_IDX_TEST,
    VP890_EVT_IDX_MAX,
    VP890_EVT_IDX_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req. */
} Vp890EventIndexType;


/* =================================
    Prototypes for Static Functions
   ================================= */

static uint16
GetEventId(
    uint16                  eventIdx,
    VpEventCategoryType     *pEventCat,
    VpOptionEventMaskType   *pObjEvents,
    VpOptionEventMaskType   *pObjEventMask);

static uint16
ParseEventMask(
    uint16                  *pEvent,
    uint16                  *pEventMask);

static void
FillRestOfpEvent(
    VpDevCtxType            *pDevCtx,
    VpLineCtxType           *pLineCtx,
    VpEventType             *pEvent,
    bool                    devEvents);

static VpStatusType
GetDeviceOption(
    VpDevCtxType            *pDevCtx,
    VpOptionIdType          option,
    uint16                  handle);

static VpStatusType
GetLineOption(
    VpLineCtxType           *pLineCtx,
    VpOptionIdType          option,
    uint16                  handle);

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
static void
VpPrint890CalLineData(
    Vp890CalLineData *calLineData);
#endif

/* ============================
    Status and Query Functions
   ============================ */

/*******************************************************************************
 * Vp890GetEvent()
 *  This function reports new events that occured on the device. This function
 * returns one event for each call to it. It should be called repeatedly until
 * no more events are reported for a specific device.  This function does not
 * access the device, it returns status from the phantom registers that are
 * maintained by the API tick routine.
 *
 * Arguments:
 *  pDevCtx -
 *  pEvent  - Pointer to the results event structure
 *
 * Preconditions:
 *  None. All error checking required is assumed to exist in common interface
 * file.
 *
 * Postconditions:
 *  Returns true if there is an active event for the device.
 ******************************************************************************/
bool
Vp890GetEvent(
    VpDevCtxType            *pDevCtx,
    VpEventType             *pEvent)
{
    Vp890DeviceObjectType   *pDevObj        = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId        = pDevObj->deviceId;
    Vp890LineObjectType     *pLineObj;
    VpLineCtxType           *pLineCtx;
    VpEventCategoryType     eventCatType;
    uint8                   chan;
    uint16                  eventCatIdx;

    pEvent->status = VP_STATUS_SUCCESS;
    pEvent->hasResults = FALSE;

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Look for active device events first */
    for (eventCatIdx = VP890_EVT_IDX_FAULT; eventCatIdx < VP_NUM_EVCATS; eventCatIdx++) {
        eventCatType = VP_NUM_EVCATS;

        /* Determine the event Id */
        pEvent->eventId = GetEventId(eventCatIdx, &eventCatType,
                        &pDevObj->deviceEvents, &pDevObj->deviceEventsMask);

        /*
         * if the event id is 0 then no events are present for the
         * type of event currently being checked and we can continue
         * to the next type of event in the loop.
         */
        if (pEvent->eventId == 0x0000) {
            continue;
        }

        pEvent->deviceId        = deviceId;
        pEvent->channelId       = 0;
        pEvent->eventCategory   = eventCatType;
        pEvent->pDevCtx         = pDevCtx;
        pEvent->pLineCtx        = VP_NULL;
        pEvent->parmHandle      = pDevObj->eventHandle;

        FillRestOfpEvent(pDevCtx, VP_NULL, pEvent, TRUE);
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return TRUE;
    }
    /*
     * No device events, now look for Line events -- but make sure the line
     * context is valid before looking for a line object
     */
    for(chan = pDevObj->dynamicInfo.lastChan; chan < VP890_MAX_NUM_CHANNELS; chan++) {
        pLineCtx = pDevCtx->pLineCtx[chan];

        /* skip line if null */
        if (pLineCtx == VP_NULL) {
            continue;
        }

        /* The line context is valid, create a line object and initialize
         * the event arrays for this line
         */
        pLineObj = pLineCtx->pLineObj;

        /* Check this line events */
        for (eventCatIdx = VP890_EVT_IDX_FAULT; eventCatIdx < VP_NUM_EVCATS; eventCatIdx++) {
            eventCatType = VP_NUM_EVCATS;

            /* Determine the event Id */
            pEvent->eventId = GetEventId(eventCatIdx, &eventCatType,
                            &pLineObj->lineEvents, &pLineObj->lineEventsMask);

            /* move to the next channel if none were found on the current one */
            if (pEvent->eventId == 0x0000) {
                continue;
            }

            pEvent->deviceId        = deviceId;
            pEvent->channelId       = chan;
            pEvent->eventCategory   = eventCatType;
            pEvent->pDevCtx         = pDevCtx;
            pEvent->pLineCtx        = pDevCtx->pLineCtx[chan];
            pEvent->parmHandle      = pLineObj->lineEventHandle;
            pEvent->lineId          = pLineObj->lineId;

            FillRestOfpEvent(pDevCtx, pLineCtx, pEvent, FALSE);

            /*
             * We're returning, so update the device last channel that
             * was checked so we start at the next channel
             */
            pDevObj->dynamicInfo.lastChan = chan + 1;
            if (pDevObj->dynamicInfo.lastChan >= VP890_MAX_NUM_CHANNELS) {
                pDevObj->dynamicInfo.lastChan = 0;
            }
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return TRUE;
        }

    }
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return FALSE;
} /* Vp890GetEvent() */

/*******************************************************************************
 * GetEventId()
 *  This function maps the local VP890_EVT_IDX_xyz types to the corresponding
 *  VP-API Event Categories, then calls into the ParseEventMask.
 *
 * Preconditions:
 *  None. This is an internal API function call only and it is assumed all error
 * checking necessary is performed by higher level functions.
 *
 * Postconditions:
 *
 ******************************************************************************/
static uint16
GetEventId(
    uint16                  eventCatIdx,
    VpEventCategoryType     *pEventCat,
    VpOptionEventMaskType   *pObjEvents,
    VpOptionEventMaskType   *pObjEventMask)
{
    switch(eventCatIdx) {
        case VP890_EVT_IDX_FAULT:
            *pEventCat = VP_EVCAT_FAULT;
            return ParseEventMask(&pObjEvents->faults, &pObjEventMask->faults);

        case VP890_EVT_IDX_SIGNALING:
            *pEventCat = VP_EVCAT_SIGNALING;
            return ParseEventMask(&pObjEvents->signaling, &pObjEventMask->signaling);

        case VP890_EVT_IDX_RESPONSE:
            *pEventCat = VP_EVCAT_RESPONSE;
            return ParseEventMask(&pObjEvents->response, &pObjEventMask->response);

        case VP890_EVT_IDX_PROCESS:
            *pEventCat = VP_EVCAT_PROCESS;
            return ParseEventMask(&pObjEvents->process, &pObjEventMask->process);

        case VP890_EVT_IDX_FXO:
            *pEventCat = VP_EVCAT_FXO;
            return ParseEventMask(&pObjEvents->fxo, &pObjEventMask->fxo);

        case VP890_EVT_IDX_TEST:
            *pEventCat = VP_EVCAT_TEST;
            return ParseEventMask(&pObjEvents->test, &pObjEventMask->test);

        default:
            break;
    }
    return 0x0000;
} /* GetEventId() */

/*******************************************************************************
 * ParseEventMask()
 *  This function performs a check on active device/line events and compares the
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
 ******************************************************************************/
static uint16
ParseEventMask(
    uint16 *pEvent,
    uint16 *pEventMask)
{
    uint8   i;
    uint16  mask;

    for (i = 0, mask = 0x0001; i < 16; i++, (mask = mask << 1)) {
        /* Check to see if an event MAY be reported */
        if ((mask & *pEvent) == 0) {
            continue;
        }
        *pEvent    &= (~mask);

        /* If the event is not masked, return the event */
        if ((mask & *pEventMask) == 0) {
            return mask;
        }
    }
    return 0x0000;
} /* ParseEventMask() */

/*******************************************************************************
 * FillRestOfpEvent()
 *  This function fills out the remainder of the pEvent structure
 *  (eventData and hasResults) based on the an event catagory and event id.
 *
 * Preconditions:
 *  None. This is an internal API function call only and it is assumed all error
 * checking necessary is performed by higher level functions.
 *
 * Postconditions:
 *
 ******************************************************************************/
static void
FillRestOfpEvent(
    VpDevCtxType            *pDevCtx,
    VpLineCtxType           *pLineCtx,
    VpEventType             *pEvent,
    bool                    devEvents)

{
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    Vp890LineObjectType     *pLineObj;
    uint8                   channelId = 0;

    if(pLineCtx != VP_NULL){
        pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    } else {
        pLineObj = VP_NULL;
    }

    if (VP_EVCAT_FAULT == pEvent->eventCategory) {
        if (devEvents) {
            if (VP_DEV_EVID_CLK_FLT == pEvent->eventId) {
                pEvent->eventData = (pDevObj->dynamicInfo.clkFault ? TRUE : FALSE);

            } else if (VP_DEV_EVID_BAT_FLT == pEvent->eventId) {

                if ((pDevObj->dynamicInfo.bat1Fault == TRUE) ||
                    (pDevObj->dynamicInfo.bat2Fault == TRUE) ||
                    (pDevObj->dynamicInfo.bat3Fault == TRUE))
                {
                    pEvent->eventData = TRUE;
                } else {
                    pEvent->eventData = FALSE;
                }
            }
        } else { /* Line Event */

            if (VP_LINE_EVID_THERM_FLT == pEvent->eventId) {
                if (pEvent->eventId == VP_LINE_EVID_THERM_FLT) {
                    if ((pDevObj->intReg2[channelId] & VP890_TEMPA_MASK) !=
                        (pDevObj->intReg[channelId] & VP890_TEMPA_MASK)) {

                        pEvent->eventData = (pDevObj->intReg2[channelId] & VP890_TEMPA_MASK)
                            ? TRUE : FALSE;

                        pLineObj->lineEvents.faults |= VP_LINE_EVID_THERM_FLT;
                        pDevObj->intReg2[channelId] &= ~VP890_TEMPA_MASK;
                        pDevObj->intReg2[channelId] |= (pDevObj->intReg[channelId] & VP890_TEMPA_MASK);
                    } else {
                        pEvent->eventData = (pDevObj->intReg[channelId] & VP890_TEMPA_MASK)
                            ? TRUE : FALSE;
                    }
                }
            } else if (VP_LINE_EVID_DC_FLT == pEvent->eventId) {
                pEvent->eventData =
                    (pLineObj->lineState.condition & VP_CSLAC_DC_FLT) ? TRUE : FALSE;
            }
        }

    } else if (VP_EVCAT_RESPONSE == pEvent->eventCategory) {
        if (devEvents) {
            if (VP_LINE_EVID_RD_OPTION == pEvent->eventId) {

                pEvent->channelId = pDevObj->getResultsOption.chanId;
                pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];

                if (pEvent->pLineCtx != VP_NULL) {
                    Vp890LineObjectType *pLineObjLocal = pEvent->pLineCtx->pLineObj;
                    pEvent->lineId = pLineObjLocal->lineId;
                }
                pEvent->hasResults = TRUE;
                pEvent->eventData = pDevObj->getResultsOption.optionType;

            } else if (VP_DEV_EVID_IO_ACCESS_CMP == pEvent->eventId) {
                pEvent->eventData =
                    pDevObj->getResultsOption.optionData.deviceIoData.accessType;
                if (pEvent->eventData == VP_DEVICE_IO_READ) {
                    pEvent->hasResults = TRUE;
                } else {
                    pEvent->hasResults = FALSE;
                }
            } else if (VP_DEV_EVID_DEV_INIT_CMP == pEvent->eventId) {
                pEvent->eventData = 1;
            } else if (VP_EVID_CAL_CMP == pEvent->eventId) {
                pEvent->eventData = pDevObj->responseData;
            }
        } else { /* Line Event */
            pEvent->eventData = pLineObj->responseData;

            if ((VP_LINE_EVID_LLCMD_RX_CMP == pEvent->eventId) ||
                (VP_LINE_EVID_GAIN_CMP == pEvent->eventId) ||
                (VP_LINE_EVID_RD_LOOP == pEvent->eventId)) {
                pEvent->hasResults = TRUE;
            }

            if (pLineObj->responseData == (uint8)VP_CAL_GET_SYSTEM_COEFF) {
                pEvent->eventData = pDevObj->mpiLen;
                pEvent->hasResults = TRUE;
                /*
                 * Prevent future cal complete events from being
                 * indicated as having results data.
                 */
                pLineObj->responseData = (uint8)VP_CAL_ENUM_SIZE;
            }
        }
    } else if (!devEvents && (VP_EVCAT_SIGNALING == pEvent->eventCategory)) {
        if (VP_LINE_EVID_DTMF_DIG == pEvent->eventId ) {
            pEvent->eventData = pLineObj->dtmfDigitSense;
            pEvent->parmHandle = pDevObj->timeStamp;
        } else {
            pEvent->eventData = pLineObj->signalingData;
        }
#ifdef VP890_INCLUDE_TESTLINE_CODE
    } else if (!devEvents && (VP_EVCAT_TEST == pEvent->eventCategory)) {

        /* testId is an enum type representing a primitive name */
        if ( VP_LINE_EVID_TEST_CMP == pEvent->eventId) {
            pEvent->eventData = pDevObj->testResults.testId;
            pEvent->hasResults = TRUE;
        }
#endif
    } else if (!devEvents && (VP_EVCAT_PROCESS == pEvent->eventCategory)) {

        pEvent->eventData = pLineObj->processData;

    } else if (!devEvents && (VP_EVCAT_FXO == pEvent->eventCategory)) {
#ifdef VP890_FXO_SUPPORT
        pEvent->eventData = pLineObj->fxoData;
#else
        pEvent->eventData = 0;
#endif
    }
    return;
} /* FillRestOfpEvent() */

/*******************************************************************************
 * Vp890GetDeviceStatus()
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
 ******************************************************************************/
VpStatusType
Vp890GetDeviceStatus(
    VpDevCtxType            *pDevCtx,
    VpInputType             input,
    uint32                  *pDeviceStatus)
{
    uint8                   channelId;
    bool                    status      = FALSE;

    VpStatusType returnStatus        = VP_STATUS_SUCCESS;
    VpStatusType returnStatusTemp    = VP_STATUS_SUCCESS;

    VpLineCtxType           *pLineCtx;

    *pDeviceStatus = 0;

    for (channelId = 0; channelId < VP890_MAX_NUM_CHANNELS; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];

        if(pLineCtx != VP_NULL) {
            returnStatusTemp = VpCSLACGetLineStatus(pLineCtx, input, &status);
            if (returnStatusTemp != VP_STATUS_SUCCESS) {
                returnStatus = returnStatusTemp;
            }
        } else {
            status = FALSE;
        }
        *pDeviceStatus |= (((status == TRUE) ? 1 : 0) << channelId);
    }
    return returnStatus;
} /* Vp890GetDeviceStatus() */

/*******************************************************************************
 * Vp890GetOption()
 * This function ...
 *
 * Arguments:
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
VpStatusType
Vp890GetOption(
    VpLineCtxType       *pLineCtx,
    VpDevCtxType        *pDevCtx,
    VpOptionIdType      option,
    uint16              handle)
{
    VpStatusType status = VP_STATUS_INVALID_ARG;

    if(pLineCtx != VP_NULL){
        Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
        /* Do not allow FXS specific options on an FXO line */
        if (pLineObj->status & VP890_IS_FXO) {
            switch(option) {
                case VP_OPTION_ID_ZERO_CROSS:
                case VP_OPTION_ID_PULSE_MODE:
                case VP_OPTION_ID_LINE_STATE:
                case VP_OPTION_ID_RING_CNTRL:
                case VP_OPTION_ID_DCFEED_PARAMS:
                case VP_OPTION_ID_RINGING_PARAMS:
                    return VP_STATUS_INVALID_ARG;
                default:
                    status = GetLineOption(pLineCtx, option, handle);
            }
        } else {
            status = GetLineOption(pLineCtx, option, handle);
        }
        if (status != VP_STATUS_SUCCESS) {
            VpDevCtxType *pDevCtxLocal = pLineCtx->pDevCtx;
            status = GetDeviceOption(pDevCtxLocal, option, handle);
        }
        return status;
    } else {
        switch(option) {
            case VP_OPTION_ID_PULSE_MODE:
            case VP_OPTION_ID_TIMESLOT:
            case VP_OPTION_ID_CODEC:
            case VP_OPTION_ID_PCM_HWY:
            case VP_OPTION_ID_LOOPBACK:
            case VP_OPTION_ID_LINE_STATE:
            case VP_OPTION_ID_EVENT_MASK:
            case VP_OPTION_ID_ZERO_CROSS:
            case VP_OPTION_ID_RING_CNTRL:
            case VP_OPTION_ID_PCM_TXRX_CNTRL:
            case VP_OPTION_ID_SWITCHER_CTRL:
#ifdef CSLAC_GAIN_ABS
            case VP_OPTION_ID_ABS_GAIN:
#endif
            case VP_OPTION_ID_DCFEED_PARAMS:
            case VP_OPTION_ID_RINGING_PARAMS:
                return VP_STATUS_INVALID_ARG;

            default:
                return GetDeviceOption(pDevCtx, option, handle);
        }
    }
} /* Vp890GetOption() */

/*******************************************************************************
 * GetDeviceOption()
 * This function ...
 *
 * Arguments:
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
static VpStatusType
GetDeviceOption(
    VpDevCtxType        *pDevCtx,
    VpOptionIdType      option,
    uint16              handle)
{
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    Vp890LineObjectType *pLineObj;
    VpDeviceIdType deviceId         = pDevObj->deviceId;

    VpStatusType status             = VP_STATUS_SUCCESS;
    VpGetResultsOptionsDataType
            *pOptionData            = &(pDevObj->getResultsOption.optionData);
    VpOptionDeviceIoType *ioOption  = &(pOptionData->deviceIo);
    uint8 ecVal = 0;
    uint8 chanId;
    uint8 ioDirection;

    if (pDevObj->deviceEvents.response & VP890_READ_RESPONSE_MASK) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("GetDeviceOption() - Waiting to clear previous read"));
        return VP_STATUS_DEVICE_BUSY;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    switch (option) {
#ifdef VP890_FXS_SUPPORT
        case VP_DEVICE_OPTION_ID_PULSE:
            pOptionData->pulseTypeOption = pDevObj->pulseSpecs;
            break;

        case VP_DEVICE_OPTION_ID_PULSE2:
            pOptionData->pulseTypeOption = pDevObj->pulseSpecs2;
            break;
#endif

        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
            pOptionData->criticalFaultOption = pDevObj->criticalFault;
            break;

        case VP_DEVICE_OPTION_ID_DEVICE_IO:
            for(chanId=0; chanId < VP890_MAX_NUM_CHANNELS; chanId++ ) {
                pLineCtx = pDevCtx->pLineCtx[chanId];
                if (pLineCtx == VP_NULL) {
                    continue;
                }
                pLineObj = pLineCtx->pLineObj;
                ecVal |= pLineObj->ecVal;
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP890_IODIR_REG_RD, VP890_IODIR_REG_LEN,
                &ioDirection);
            /* Bits 2:4 of the register correspond to bits 1:3 of the option */
            ioOption->directionPins_31_0 =
                (ioDirection & ~VP890_IODIR_IO1_MASK) >> 1;
            if ((ioDirection & VP890_IODIR_IO1_MASK) == VP890_IODIR_IO1_INPUT) {
                ioOption->directionPins_31_0 |= VP_IO_INPUT_PIN;
            } else {
                ioOption->directionPins_31_0 |= VP_IO_OUTPUT_PIN;

                if ((ioDirection & VP890_IODIR_IO1_MASK) ==
                    VP890_IODIR_IO1_OUTPUT) {
                    ioOption->outputTypePins_31_0 = VP_OUTPUT_DRIVEN_PIN;
                } else if ((ioDirection & VP890_IODIR_IO1_MASK) ==
                            VP890_IODIR_IO1_OPEN_DRAIN) {
                    status = VP_STATUS_INVALID_ARG;
                }
            }
            break;

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            VP_ERROR(VpDevCtxType, pDevCtx, ("GetDeviceOption() - Device option not supported"));
            break;
    }

    if (status == VP_STATUS_SUCCESS) {
        pDevObj->getResultsOption.optionType = option;
        pDevObj->deviceEvents.response |= VP_LINE_EVID_RD_OPTION;
        pDevObj->eventHandle = handle;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
} /* GetDeviceOption() */

/*******************************************************************************
 * GetLineOption()
 * This function ...
 *
 * Arguments:
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
static VpStatusType
GetLineOption(
    VpLineCtxType           *pLineCtx,
    VpOptionIdType          option,
    uint16                  handle)
{
    VpDevCtxType            *pDevCtxLocal   = pLineCtx->pDevCtx;
    Vp890LineObjectType     *pLineObj       = pLineCtx->pLineObj;
    Vp890DeviceObjectType   *pDevObj        = pDevCtxLocal->pDevObj;

    VpGetResultsOptionsDataType
                            *pOptionData    = &(pDevObj->getResultsOption.optionData);
    VpStatusType             status         = VP_STATUS_SUCCESS;

    VpDeviceIdType          deviceId        = pDevObj->deviceId;
    uint8                   ecVal           = pLineObj->ecVal;
    uint8                   tempLoopBack[VP890_LOOPBACK_LEN];
    uint8                   txSlot, rxSlot;

    if (pDevObj->deviceEvents.response & VP890_READ_RESPONSE_MASK) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("GetLineOption() - Waiting to clear previous read"));
        return VP_STATUS_DEVICE_BUSY;
    }

    /*
     * If this function can be executed, we will either access the MPI
     * and/or shared data. So it is best to label the entire function as
     * Code Critical so the data being accessed cannot be changed while
     * trying to be accessed
     */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pDevObj->getResultsOption.chanId = pLineObj->channelId;

    switch (option) {
        /* Line Options */
#ifdef CSLAC_GAIN_ABS
        case VP_OPTION_ID_ABS_GAIN:
            pOptionData->absGain.gain_AToD = pLineObj->absGxGain;
            pOptionData->absGain.gain_DToA = pLineObj->absGrGain;
            break;
#endif

#ifdef VP890_FXS_SUPPORT
        case VP_OPTION_ID_PULSE_MODE:
            pOptionData->pulseModeOption = pLineObj->pulseMode;
            break;

        case VP_OPTION_ID_LINE_STATE: {
                uint8 tempSysConfig;

                /* Battery control is automatic, so force it */
                pOptionData->lineStateOption.bat = VP_OPTION_BAT_AUTO;

                /* Smooth/Abrupt PolRev is controlled in the device */
                VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD,
                    VP890_SS_CONFIG_LEN, &tempSysConfig);

                if (tempSysConfig & VP890_SMOOTH_PR_EN) {
                    pOptionData->lineStateOption.battRev = FALSE;
                } else {
                    pOptionData->lineStateOption.battRev = TRUE;
                }
            }
            break;

        case VP_OPTION_ID_ZERO_CROSS:
            pOptionData->zeroCross = pLineObj->ringCtrl.zeroCross;
            break;

        case VP_OPTION_ID_RING_CNTRL:
            pOptionData->ringControlOption = pLineObj->ringCtrl;
            break;

        case VP_OPTION_ID_SWITCHER_CTRL: {
                uint8 ssConfig[VP890_SS_CONFIG_LEN];

                VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD, VP890_SS_CONFIG_LEN,
                    ssConfig);

                if (ssConfig[0] & VP890_AUTO_BAT_SHUTDOWN_EN) {
                    pOptionData->autoShutdownEn = TRUE;
                } else {
                    pOptionData->autoShutdownEn = FALSE;
                }
            }
            break;
#endif

        case VP_OPTION_ID_TIMESLOT:
            VpMpiCmdWrapper(deviceId, ecVal, VP890_TX_TS_RD,
                VP890_TX_TS_LEN, &txSlot);

            VpMpiCmdWrapper(deviceId, ecVal, VP890_RX_TS_RD,
                VP890_RX_TS_LEN, &rxSlot);

            pOptionData->timeSlotOption.tx = (txSlot & VP890_TX_TS_MASK);

            pOptionData->timeSlotOption.rx = (rxSlot & VP890_RX_TS_MASK);
            break;

        case VP_OPTION_ID_CODEC:
            pOptionData->codecOption = pLineObj->codec;
            break;

        case VP_OPTION_ID_PCM_HWY:
            pOptionData->pcmHwyOption = VP_OPTION_HWY_A;
            break;

        case VP_OPTION_ID_LOOPBACK:
            /* Timeslot loopback via loopback register */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOPBACK_RD,
                VP890_LOOPBACK_LEN, tempLoopBack);

            if ((tempLoopBack[0] & VP890_INTERFACE_LOOPBACK_EN) ==
                 VP890_INTERFACE_LOOPBACK_EN) {
                pOptionData->loopBackOption = VP_OPTION_LB_TIMESLOT;
            } else {
                if((pDevObj->devMode[0] & (VP890_DEV_MODE_CH21PT | VP890_DEV_MODE_CH12PT)) ==
                    (VP890_DEV_MODE_CH21PT | VP890_DEV_MODE_CH12PT)) {
                    pOptionData->loopBackOption = VP_OPTION_LB_CHANNELS;
                } else {
                    pOptionData->loopBackOption = VP_OPTION_LB_OFF;
                }
            }
            break;

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

        case VP_OPTION_ID_PCM_TXRX_CNTRL:
            pOptionData->pcmTxRxCtrl = pLineObj->pcmTxRxCtrl;
            break;

#ifdef VP890_FXS_SUPPORT
        case VP_OPTION_ID_DCFEED_PARAMS: {
            uint8 voc;
            uint8 ila;
            uint8 tsh;
            uint8 tgk;
            uint8 battFloor;

/* Set the bits for the information we're returning */
            pOptionData->dcFeedParams.validMask =
                (VP_OPTION_CFG_VOC | VP_OPTION_CFG_ILA | VP_OPTION_CFG_HOOK_THRESHOLD | \
                 VP_OPTION_CFG_GKEY_THRESHOLD | VP_OPTION_CFG_BATT_FLOOR);

            /* VOC is a 3-bit field with step size of 3 V and a range of either
               12-33 or 36-57 V depending on the VOCSFT bit. */
            voc = ((pLineObj->calLineData.dcFeedRef[VP890_VOC_INDEX] & VP890_VOC_MASK) >> 2);
            if (pLineObj->calLineData.dcFeedRef[VP890_VOC_INDEX] & VP890_VOC_LOW_RANGE) {
                pOptionData->dcFeedParams.voc = voc * VP890_VOC_STEP + VP890_VOC_OFFSET;
            } else {
                pOptionData->dcFeedParams.voc = voc * VP890_VOC_STEP + VP890_VOC_HIGH_OFFSET;
            }

            /* ILA is a 5-bit value with a 18-49 mA scale (1 mA per step + 18 mA
               offset) */
            ila = (pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] & VP890_ILA_MASK);
            pOptionData->dcFeedParams.ila = ila * VP890_ILA_STEP + VP890_ILA_OFFSET;

            /* TSH is a 3-bit value with a 8-15 mA scale (1 mA per step + 8 mA
               offset) */
            tsh = (pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE] & VP890_SWHOOK_THRESH_MASK);
            pOptionData->dcFeedParams.hookThreshold = tsh * VP890_TSH_STEP + VP890_TSH_OFFSET;

            /* TGK is a 3-bit value with a 0-42 mA scale (6 mA per step) */
            tgk = (pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE] & VP890_GKEY_THRESH_MASK) >> 3;
            pOptionData->dcFeedParams.gkeyThreshold = tgk * VP890_TGK_STEP + VP890_TGK_OFFSET;

            /* Battery floor voltage is in -5V steps with a -5V offset */
            battFloor = (pDevObj->devProfileData.swParams[VP890_FLOOR_VOLTAGE_BYTE] & VP890_FLOOR_VOLTAGE_MASK);
            pOptionData->dcFeedParams.battFloor = battFloor * VP890_BATTFLR_STEP + VP890_BATTFLR_OFFSET;
        } break;

        case VP_OPTION_ID_RINGING_PARAMS: {
            uint8 ringingValues[VP890_RINGING_PARAMS_LEN];
            int16 frequency;
            int16 amplitude;
            int16 bias;
            bool trapezoidal;
            uint8 rtth;
            uint8 ilr;
            int16 riseTime;

            pOptionData->ringingParams.validMask = 0;

            VpMemCpy(ringingValues, pLineObj->ringingParamsRef, VP890_RINGER_PARAMS_LEN);

            if ((ringingValues[0] & VP890_SIGGEN1_SINTRAP_MASK) == VP890_SIGGEN1_TRAP) {
                trapezoidal = TRUE;
            } else {
                trapezoidal = FALSE;
            }

            pOptionData->ringingParams.validMask |= VP_OPTION_CFG_FREQUENCY;
            if (!trapezoidal) {
                /* Sinusoidal ringing frequency is a 15-bit value with a 0-12000 Hz
                   range.  12000000 mHz per 0x8000 reduces to 46875 mHz per 0x80. */
                frequency = ringingValues[VP890_SIGA_FREQ_MSB] << 8;
                frequency |= ringingValues[VP890_SIGA_FREQ_LSB];
                pOptionData->ringingParams.frequency = VpRoundedDivide(frequency * VP890_FREQ_STEP_NUM, VP890_FREQ_STEP_DEN);
            } else {
                /* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
                   This means the maximum is 8000 Hz (8000000 mHz), and the minimum
                   is 8000Hz / 0x7FFF, or 244.1 mHz.  Frequency can be calculated by 
                   8000000mHz / FRQB. */
                frequency = ringingValues[VP890_SIGB_FREQ_MSB] << 8;
                frequency |= ringingValues[VP890_SIGB_FREQ_LSB];
                pOptionData->ringingParams.frequency = VpRoundedDivide(VP890_TRAPFREQ_MAX, frequency);
            }

            /* Amplitude is a 16-bit value with a +/- 155V range.  This
               means 155000 mV per 0x8000, which reduces to 19375 mV per 0x1000. */
            pOptionData->ringingParams.validMask |= VP_OPTION_CFG_AMPLITUDE;
            amplitude = ringingValues[VP890_SIGA_AMP_MSB] << 8;
            amplitude |= ringingValues[VP890_SIGA_AMP_LSB];
            pOptionData->ringingParams.amplitude = VpRoundedDivide(amplitude * VP890_AMP_STEP_NUM, VP890_AMP_STEP_DEN); 

            /* DC Bias is a 16-bit value with a +/- 154.4V range.  This
               means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
            pOptionData->ringingParams.validMask |= VP_OPTION_CFG_DC_BIAS;
            bias = ringingValues[VP890_SIGA_BIAS_MSB] << 8;
            bias |= ringingValues[VP890_SIGA_BIAS_LSB];
            pOptionData->ringingParams.dcBias = VpRoundedDivide(bias * VP890_BIAS_STEP_NUM, VP890_BIAS_STEP_DEN); 

            /* Ring trip threshold is a 7-bit value with a 0-63.5 mA scale (0.5 mA
               per step).  Use the value saved in the line object, since the 
               register value may be modified for the low ILR workaround. */
            pOptionData->ringingParams.validMask |= VP_OPTION_CFG_RINGTRIP_THRESHOLD;
            rtth = (pLineObj->loopSup[VP890_LOOP_SUP_RT_MODE_BYTE] & VP890_RINGTRIP_THRESH_MASK);
            pOptionData->ringingParams.ringTripThreshold = rtth * VP890_RTTH_STEP + VP890_RTTH_OFFSET;

            /* Ring current limit is a 5-bit value with a 50-112 mA scale (2 mA per
               step + 50 mA offset). */
            pOptionData->ringingParams.validMask |= VP_OPTION_CFG_RING_CURRENT_LIMIT;
            ilr = (pLineObj->loopSup[VP890_LOOP_SUP_ILR_BYTE] & VP890_LOOP_SUP_RING_LIM_MASK);
            pOptionData->ringingParams.ringCurrentLimit = ilr * VP890_ILR_STEP + VP890_ILR_OFFSET;
            
            if (trapezoidal) {
                /* Trapezoidal rise time is defined as 2.7307sec / FRQA. This means the
                   maximum is 2.7307 sec (2730700 usec), and the minimum
                   is 2.7307sec / 0x7FFF, or 83.3 usec.  Rise time can be calculated by 
                   2730700usec / FRQA. */
                pOptionData->ringingParams.validMask |= VP_OPTION_CFG_TRAP_RISE_TIME;
                riseTime = ringingValues[VP890_SIGA_FREQ_MSB] << 8;
                riseTime |= ringingValues[VP890_SIGA_FREQ_LSB];
                pOptionData->ringingParams.trapRiseTime = VpRoundedDivide(VP890_TRAPRISE_MAX, riseTime);
            }
        } break;
#endif /* VP890_FXS_SUPPORT */

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            VP_ERROR(VpLineCtxType, pLineCtx, ("GetLineOption() - Line option not supported"));
            break;
    }

    if (status == VP_STATUS_SUCCESS) {
        pDevObj->getResultsOption.optionType = option;
        pDevObj->deviceEvents.response |= VP_LINE_EVID_RD_OPTION;
        pDevObj->eventHandle = handle;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
} /* GetLineOption() */

/*******************************************************************************
 * Vp890FlushEvents()
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
 ******************************************************************************/
VpStatusType
Vp890FlushEvents(
    VpDevCtxType            *pDevCtx)
{
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;

    VpLineCtxType           *pLineCtx;
    Vp890LineObjectType     *pLineObj;
    uint8                   channelId;

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VpMemSet(&pDevObj->deviceEvents, 0, sizeof(VpOptionEventMaskType));

    for (channelId = 0; channelId < VP890_MAX_NUM_CHANNELS; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if(pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            VpMemSet(&pLineObj->lineEvents, 0, sizeof(VpOptionEventMaskType));
        }
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp890FlushEvents() */

/*******************************************************************************
 * Vp890GetResults()
 * This function ...
 *
 * Arguments:
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
VpStatusType
Vp890GetResults(
    VpEventType             *pEvent,
    void                    *pResults)
{
    VpDevCtxType            *pDevCtx    = pEvent->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    VpStatusType            status      = VP_STATUS_SUCCESS;

    VpGetResultsOptionsDataType *pOptionData =
        &(pDevObj->getResultsOption.optionData);

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (VP_EVCAT_RESPONSE == pEvent->eventCategory) {
        if (VP_LINE_EVID_LLCMD_RX_CMP == pEvent->eventId) {
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
            uint8 cmdByte;
            for (cmdByte = 0; cmdByte < pDevObj->mpiLen; cmdByte++) {
                ((uint8 *)pResults)[cmdByte] = pDevObj->mpiData[cmdByte];
            }
#endif
        } else if (VP_LINE_EVID_GAIN_CMP == pEvent->eventId) {

            *(VpRelGainResultsType *)pResults =  pDevObj->relGainResults;

        } else if (VP_DEV_EVID_IO_ACCESS_CMP == pEvent->eventId) {

            *((VpDeviceIoAccessDataType *)pResults) = pOptionData->deviceIoData;

        } else if (VP_LINE_EVID_RD_OPTION == pEvent->eventId) {

            switch(pDevObj->getResultsOption.optionType) {
#ifdef VP890_FXS_SUPPORT
                case VP_DEVICE_OPTION_ID_PULSE:
                    *(VpOptionPulseType *)pResults =
                        pDevObj->pulseSpecs;
                    break;

                case VP_DEVICE_OPTION_ID_PULSE2:
                    *(VpOptionPulseType *)pResults =
                        pDevObj->pulseSpecs2;
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

                case VP_OPTION_ID_LINE_STATE:
                    *((VpOptionLineStateType *)pResults) =
                        pOptionData->lineStateOption;
                    break;

                case VP_OPTION_ID_SWITCHER_CTRL:
                    *((bool *)pResults) =
                        pOptionData->autoShutdownEn;
                    break;
#endif

                case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
                    *(VpOptionCriticalFltType *)pResults =
                        pOptionData->criticalFaultOption;
                    break;

                case VP_DEVICE_OPTION_ID_DEVICE_IO:
                    *(VpOptionDeviceIoType *)pResults =
                        pOptionData->deviceIo;
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
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp890GetResults() - Invalid option type"));
                    break;
            }
        } else if (VP_EVID_CAL_CMP == pEvent->eventId) {
            if (pResults == VP_NULL) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                uint8 *pMpiData;
                uint8 commandByte;

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp890GetResults - VP_EVID_CAL_CMP"));

                pMpiData = (uint8 *)pResults;
                pMpiData[VP_PROFILE_TYPE_MSB] = VP_DEV_890_SERIES;
                pMpiData[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_CAL;
                pMpiData[VP_PROFILE_INDEX] = 0;
                pMpiData[VP_PROFILE_LENGTH] = VP890_CAL_STRUCT_SIZE + 2;
                pMpiData[VP_PROFILE_VERSION] = 0;
                pMpiData[VP_PROFILE_MPI_LEN] = 0;
                commandByte = VP_PROFILE_DATA_START;

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Error Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.abvError[0], (uint32)sizeof(pDevObj->vp890SysCalData.abvError[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.abvError[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.abvError[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vocOffset[0][0], (uint32)sizeof(pDevObj->vp890SysCalData.vocOffset[0][0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocOffset[0][0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocOffset[0][0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vocError[0][0], (uint32)sizeof(pDevObj->vp890SysCalData.vocError[0][0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocError[0][0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocError[0][0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Rev Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vocOffset[0][1], (uint32)sizeof(pDevObj->vp890SysCalData.vocOffset[0][1])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocOffset[0][1] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocOffset[0][1]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Rev Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vocError[0][1], (uint32)sizeof(pDevObj->vp890SysCalData.vocError[0][1])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocError[0][1] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vocError[0][1]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Error Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.sigGenAError[0][0], (uint32)sizeof(pDevObj->vp890SysCalData.sigGenAError[0][0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.sigGenAError[0][0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.sigGenAError[0][0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Error Rev Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.sigGenAError[0][1], (uint32)sizeof(pDevObj->vp890SysCalData.sigGenAError[0][1])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.sigGenAError[0][1] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.sigGenAError[0][1]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-20mA Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.ila20[0], (uint32)sizeof(pDevObj->vp890SysCalData.ila20[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila20[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila20[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-25mA Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.ila25[0], (uint32)sizeof(pDevObj->vp890SysCalData.ila25[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila25[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila25[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-32mA Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.ila32[0], (uint32)sizeof(pDevObj->vp890SysCalData.ila32[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila32[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila32[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA-40mA Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.ila40[0], (uint32)sizeof(pDevObj->vp890SysCalData.ila40[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila40[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ila40[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA Offset Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.ilaOffsetNorm[0], (uint32)sizeof(pDevObj->vp890SysCalData.ilaOffsetNorm[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ilaOffsetNorm[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ilaOffsetNorm[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILG Offset Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.ilgOffsetNorm[0], (uint32)sizeof(pDevObj->vp890SysCalData.ilgOffsetNorm[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ilgOffsetNorm[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ilgOffsetNorm[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vas[0][0], (uint32)sizeof(pDevObj->vp890SysCalData.vas[0][0])));
                pMpiData[commandByte++] = pDevObj->vp890SysCalData.vas[0][0];

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Rev Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vas[0][1], (uint32)sizeof(pDevObj->vp890SysCalData.vas[0][1])));
                pMpiData[commandByte++] = pDevObj->vp890SysCalData.vas[0][1];

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vagOffsetNorm[0], (uint32)sizeof(pDevObj->vp890SysCalData.vagOffsetNorm[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vagOffsetNorm[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vagOffsetNorm[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Rev Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vagOffsetRev[0], (uint32)sizeof(pDevObj->vp890SysCalData.vagOffsetRev[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vagOffsetRev[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vagOffsetRev[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Norm Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vbgOffsetNorm[0], (uint32)sizeof(pDevObj->vp890SysCalData.vbgOffsetNorm[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vbgOffsetNorm[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vbgOffsetNorm[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Rev Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.vbgOffsetRev[0], (uint32)sizeof(pDevObj->vp890SysCalData.vbgOffsetRev[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vbgOffsetRev[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.vbgOffsetRev[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWY Offset Ch 0 %d Size %lu",
                    pDevObj->vp890SysCalData.swyOffset[0], (uint32)sizeof(pDevObj->vp890SysCalData.swyOffset[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.swyOffset[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.swyOffset[0]) & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Tip Cap Ch 0 %li Size %lu",
                    pDevObj->vp890SysCalData.tipCapCal[0], (uint32)sizeof(pDevObj->vp890SysCalData.tipCapCal[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.tipCapCal[0] >> 24) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.tipCapCal[0] >> 16) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.tipCapCal[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)(pDevObj->vp890SysCalData.tipCapCal[0] & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ring Cap Ch 0 %li Size %lu",
                    pDevObj->vp890SysCalData.ringCapCal[0], (uint32)sizeof(pDevObj->vp890SysCalData.ringCapCal[0])));
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ringCapCal[0] >> 24) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ringCapCal[0] >> 16) & 0xFF);
                pMpiData[commandByte++] = (uint8)((pDevObj->vp890SysCalData.ringCapCal[0] >> 8) & 0xFF);
                pMpiData[commandByte++] = (uint8)(pDevObj->vp890SysCalData.ringCapCal[0] & 0xFF);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Final Command Byte Value %d", commandByte));
            }
        } else {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp890GetResults() - Invalid event ID"));
            status = VP_STATUS_INVALID_ARG;
        }
#ifdef VP890_INCLUDE_TESTLINE_CODE
    } else if (VP_EVCAT_TEST == pEvent->eventCategory) {

        if (VP_LINE_EVID_TEST_CMP == pEvent->eventId) {
            *((VpTestResultType *)pResults) = pDevObj->testResults;
        } else {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp890GetResults() - Invalid event ID"));
            status = VP_STATUS_INVALID_ARG;
        }
#endif
    } else {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp890GetResults() - Invalid event category"));
        status = VP_STATUS_INVALID_ARG;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
} /* Vp890GetResults() */

#ifdef VP890_INCLUDE_TESTLINE_CODE
/**
 * Vp890GetRelayState()
 *  This function returns the current relay state of VP890 device.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized.
 *
 * Postconditions:
 *  The indicated relay state is returned for the given line.
 */
VpStatusType
Vp890GetRelayState(
    VpLineCtxType           *pLineCtx,
    VpRelayControlType      *pRstate)
{
    VpDevCtxType            *pDevCtx;
    Vp890DeviceObjectType   *pDevObj;
    Vp890LineObjectType     *pLineObj;
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

} /* Vp890SetRelayState() */
#endif /* VP890_FXS_SUPPORT */



#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
/**
 * Vp890RegisterDump()
 *  Dump all known 890 device and line specific registers (for debug purposes).
 *
 * Returns:
 */
VpStatusType
Vp890RegisterDump(
    VpDevCtxType *pDevCtx)
{
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId, ecVal, registerIndex, registerNumber, maxChannelNum;

    /* Sufficient size to hold a single MPI Read is all that is needed. */
    uint8 registerBuffer[20];

#define VP890_DEVICE_REGISTER_COUNT    18
    uint8 deviceRegs[VP890_DEVICE_REGISTER_COUNT][2] = {
        {VP890_TEST_REG1_RD, VP890_TEST_REG1_LEN},
        {VP890_FUSE_CTRL_REG_RD, VP890_FUSE_CTRL_REG_LEN},
        {VP890_FUSE1_REG_RD, VP890_FUSE1_REG_LEN},
        {VP890_FUSE5_REG_RD, VP890_FUSE5_REG_LEN},
        {VP890_TX_RX_CSLOT_RD, VP890_TX_RX_CSLOT_LEN},
        {VP890_DCR_RD, VP890_DCR_LEN},
        {VP890_OP_MODE_RD, VP890_OP_MODE_LEN},
        {VP890_NO_UL_SIGREG_RD, VP890_NO_UL_SIGREG_LEN},
        {VP890_IODATA_REG_RD, VP890_IODATA_REG_LEN},
        {VP890_IODIR_REG_RD, VP890_IODIR_REG_LEN},
        {VP890_DEV_MODE_RD, VP890_DEV_MODE_LEN},
        {VP890_INT_MASK_RD, VP890_INT_MASK_LEN},
        {VP890_DEVTYPE_RD, VP890_DEVTYPE_LEN},
        {VP890_REV_INFO_RD, VP890_REV_INFO_LEN},
        {VP890_TEST_DATA_RD, VP890_TEST_DATA_LEN},
        {VP890_FUSE6_REG_RD, VP890_FUSE6_REG_LEN},
        {VP890_REGULATOR_TIMING_RD, VP890_REGULATOR_TIMING_LEN},
        {VP890_FUSE7_REG_RD, VP890_FUSE7_REG_LEN}
    };

    char *deviceRegsName[VP890_DEVICE_REGISTER_COUNT] = {
        "VP890_TEST_REG1_RD", "VP890_FUSE_CTRL_REG_RD", "VP890_FUSE1_REG_RD",
        "VP890_FUSE5_REG_RD", "VP890_TX_RX_CSLOT_RD", "VP890_DCR_RD", "VP890_OP_MODE_RD",
        "VP890_NO_UL_SIGREG_RD", "VP890_IODATA_REG_RD", "VP890_IODIR_REG_RD",
        "VP890_DEV_MODE_RD", "VP890_INT_MASK_RD", "VP890_DEVTYPE_RD", "VP890_REV_INFO_RD",
        "VP890_TEST_DATA_RD", "VP890_FUSE6_REG_RD", "VP890_REGULATOR_TIMING_RD",
        "VP890_FUSE7_REG_RD"
    };

    VpSysDebugPrintf("\n\rDevice Registers:\n\r");
    ecVal = pDevObj->ecVal;

    for (registerNumber = 0; registerNumber < VP890_DEVICE_REGISTER_COUNT; registerNumber++) {
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

    switch(pDevObj->staticInfo.rcnPcn[1]) {
#ifdef VP890_FXS_SUPPORT
        case VP890_DEV_PCN_89116:   /**< FXS - Wideband */
        case VP890_DEV_PCN_89136:   /**< HV FXS - Wideband */
            channelId = 0;
            maxChannelNum = 1;
            break;
#endif

        case VP890_DEV_PCN_89316:   /**< FXO/FXS-Tracker - Wideband */
        case VP890_DEV_PCN_89336:   /**< FXO/FXS-Tracker - Wideband */
            channelId = 0;
            maxChannelNum = 2;
            break;

#ifdef VP890_FXO_SUPPORT
        case VP890_DEV_PCN_89010:   /**< Single Channel FXO */
            channelId = 1;
            maxChannelNum = 2;
            break;
#endif

        default:
            channelId = 0;
            maxChannelNum = 0;
            break;
    }

    for (; channelId < maxChannelNum; channelId++) {
        if (channelId == 0) {   /* Always FXS line if exists */
#define VP890_CHANNEL_REGISTER_COUNT    35
            uint8 channelRegs[VP890_CHANNEL_REGISTER_COUNT][2] = {
                {VP890_TX_TS_RD, VP890_TX_TS_LEN},
                {VP890_RX_TS_RD, VP890_RX_TS_LEN},
                {VP890_VP_GAIN_RD, VP890_VP_GAIN_LEN},
                {VP890_SYS_STATE_RD, VP890_SYS_STATE_LEN},
                {VP890_OP_FUNC_RD, VP890_OP_FUNC_LEN},
                {VP890_SS_CONFIG_RD, VP890_SS_CONFIG_LEN},
                {VP890_OP_COND_RD, VP890_OP_COND_LEN},
                {VP890_GX_GAIN_RD, VP890_GX_GAIN_LEN},
                {VP890_GR_GAIN_RD, VP890_GR_GAIN_LEN},
                {VP890_B1_FILTER_RD, VP890_B1_FILTER_LEN},
                {VP890_X_FILTER_RD, VP890_X_FILTER_LEN},
                {VP890_R_FILTER_RD, VP890_R_FILTER_LEN},
                {VP890_B2_FILTER_RD, VP890_B2_FILTER_LEN},
                {VP890_Z1_FILTER_RD, VP890_Z1_FILTER_LEN},
                {VP890_Z2_FILTER_RD, VP890_Z2_FILTER_LEN},
                {VP890_CONV_CFG_RD, VP890_CONV_CFG_LEN},
                {VP890_LOOP_SUP_RD, VP890_LOOP_SUP_LEN},
                {VP890_DC_FEED_RD, VP890_DC_FEED_LEN},
                {VP890_DISN_RD, VP890_DISN_LEN},
                {VP890_TX_PCM_DATA_RD, VP890_TX_PCM_DATA_LEN},
                {VP890_SIGA_PARAMS_RD, VP890_SIGA_PARAMS_LEN},
                {VP890_SIGCD_PARAMS_RD, VP890_SIGCD_PARAMS_LEN},
                {VP890_GEN_CTRL_RD, VP890_GEN_CTRL_LEN},
                {VP890_CADENCE_TIMER_RD, VP890_CADENCE_TIMER_LEN},
                {VP890_CID_DATA_RD, VP890_CID_DATA_LEN},
                {VP890_REGULATOR_PARAM_RD, VP890_REGULATOR_PARAM_LEN},
                {VP890_REGULATOR_CTRL_RD, VP890_REGULATOR_CTRL_LEN},
                {VP890_BAT_CALIBRATION_RD, VP890_BAT_CALIBRATION_LEN},
                {VP890_CID_PARAM_RD, VP890_CID_PARAM_LEN},
                {VP890_ICR1_RD, VP890_ICR1_LEN},
                {VP890_ICR2_RD, VP890_ICR2_LEN},
                {VP890_ICR3_RD, VP890_ICR3_LEN},
                {VP890_ICR4_RD, VP890_ICR4_LEN},
                {VP890_ICR5_RD, VP890_ICR5_LEN},
                {VP890_DC_CAL_REG_RD, VP890_DC_CAL_REG_LEN}
            };

            char *registerName[VP890_CHANNEL_REGISTER_COUNT] = {
                "VP890_TX_TS_RD",           "VP890_RX_TS_RD",           "VP890_VP_GAIN_RD",
                "VP890_SYS_STATE_RD",       "VP890_OP_FUNC_RD",         "VP890_SS_CONFIG_RD",
                "VP890_OP_COND_RD",         "VP890_GX_GAIN_RD",         "VP890_GR_GAIN_RD",
                "VP890_B1_FILTER_RD",       "VP890_X_FILTER_RD",        "VP890_R_FILTER_RD",
                "VP890_B2_FILTER_RD",       "VP890_Z1_FILTER_RD",       "VP890_Z2_FILTER_RD",
                "VP890_CONV_CFG_RD",        "VP890_LOOP_SUP_RD",        "VP890_DC_FEED_RD",
                "VP890_DISN_RD",            "VP890_TX_PCM_DATA_RD",     "VP890_SIGA_PARAMS_RD",
                "VP890_SIGCD_PARAMS_RD",    "VP890_GEN_CTRL_RD",        "VP890_CADENCE_TIMER_RD",
                "VP890_CID_DATA_RD",        "VP890_REGULATOR_PARAM_RD", "VP890_REGULATOR_CTRL_RD",
                "VP890_BAT_CALIBRATION_RD", "VP890_CID_PARAM_RD",       "VP890_ICR1_RD",
                "VP890_ICR2_RD",            "VP890_ICR3_RD",            "VP890_ICR4_RD",
                "VP890_ICR5_RD",            "VP890_DC_CAL_REG_RD"
            };

            ecVal = (VP890_EC_CH1 | pDevObj->ecVal);
            VpSysDebugPrintf("\n\rCHANNEL 0 (FXS)");
            for (registerNumber = 0; registerNumber < VP890_CHANNEL_REGISTER_COUNT; registerNumber++) {
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
        } else if (channelId == 1) {    /* Always FXO line if exists */
#define VP890_FXO_CHANNEL_REGISTER_COUNT    26
            uint8 channelRegs[VP890_FXO_CHANNEL_REGISTER_COUNT][2] = {
                {VP890_PERIOD_DET_RD, VP890_PERIOD_DET_LEN},
                {VP890_TX_TS_RD, VP890_TX_TS_LEN},
                {VP890_RX_TS_RD, VP890_RX_TS_LEN},
                {VP890_VP_GAIN_RD, VP890_VP_GAIN_LEN},
                {VP890_SYS_STATE_RD, VP890_SYS_STATE_LEN},
                {VP890_OP_FUNC_RD, VP890_OP_FUNC_LEN},
                {VP890_SS_CONFIG_RD, VP890_SS_CONFIG_LEN},
                {VP890_OP_COND_RD, VP890_OP_COND_LEN},
                {VP890_LSD_CTL_RD, VP890_LSD_CTL_LEN},
                {VP890_LSD_STAT_RD, VP890_LSD_STAT_LEN},
                {VP890_GX_GAIN_RD, VP890_GX_GAIN_LEN},
                {VP890_GR_GAIN_RD, VP890_GR_GAIN_LEN},
                {VP890_B1_FILTER_RD, VP890_B1_FILTER_LEN},
                {VP890_X_FILTER_RD, VP890_X_FILTER_LEN},
                {VP890_R_FILTER_RD, VP890_R_FILTER_LEN},
                {VP890_B2_FILTER_RD, VP890_B2_FILTER_LEN},
                {VP890_Z1_FILTER_RD, VP890_Z1_FILTER_LEN},
                {VP890_Z2_FILTER_RD, VP890_Z2_FILTER_LEN},
                {VP890_CONV_CFG_RD, VP890_CONV_CFG_LEN},
                {VP890_LOOP_SUP_RD, VP890_LOOP_SUP_LEN},
                {VP890_DISN_RD, VP890_DISN_LEN},
                {VP890_TX_PCM_DATA_RD, VP890_TX_PCM_DATA_LEN},
                {VP890_SIGCD_PARAMS_RD, VP890_SIGCD_PARAMS_LEN},
                {VP890_GEN_CTRL_RD, VP890_GEN_CTRL_LEN},
                {VP890_CADENCE_TIMER_RD, VP890_CADENCE_TIMER_LEN},
                {VP890_ICR1_RD, VP890_ICR1_LEN}
            };

            char *registerName[VP890_CHANNEL_REGISTER_COUNT] = {
                "VP890_PERIOD_DET_RD",      "VP890_TX_TS_RD",           "VP890_RX_TS_RD",
                "VP890_VP_GAIN_RD",         "VP890_SYS_STATE_RD",       "VP890_OP_FUNC_RD",
                "VP890_SS_CONFIG_RD",       "VP890_OP_COND_RD",         "VP890_LSD_CTL_RD",
                "VP890_LSD_STAT_RD",        "VP890_GX_GAIN_RD",         "VP890_GR_GAIN_RD",
                "VP890_B1_FILTER_RD",       "VP890_X_FILTER_RD",        "VP890_R_FILTER_RD",
                "VP890_B2_FILTER_RD",       "VP890_Z1_FILTER_RD",       "VP890_Z2_FILTER_RD",
                "VP890_CONV_CFG_RD",        "VP890_LOOP_SUP_RD",        "VP890_DISN_RD",
                "VP890_TX_PCM_DATA_RD",     "VP890_SIGCD_PARAMS_RD",    "VP890_GEN_CTRL_RD",
                "VP890_CADENCE_TIMER_RD",   "VP890_ICR1_RD"
            };

            ecVal = (VP890_EC_CH2 | pDevObj->ecVal);
            VpSysDebugPrintf("\n\rCHANNEL 1 (FXO)");
            for (registerNumber = 0; registerNumber < VP890_FXO_CHANNEL_REGISTER_COUNT; registerNumber++) {
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
    }

    VpSysDebugPrintf("\n\r");

    return VP_STATUS_SUCCESS;
}

/**
 * Vp890ObjectDump()
 *  Dump 890 device and line object data. Upper level calls may limit this to
 * device object OR line object, but there's no reason we need to enforce that.
 *
 * This function is for SW Apps debug purposes only. Not to be documented for
 * customer purposes. It will significantly increase the driver size, implement
 * so as totally removed when Debug Error is not enabled.
 *
 * Returns: VP_STATUS_SUCCESS
 */
VpStatusType
Vp890ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx)
{
    if (pDevCtx != VP_NULL) {
        Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;

        VP_PRINT_DEVICE_ID(pDevObj->deviceId);

        VpPrintStaticInfoStruct(&pDevObj->staticInfo);
        VpPrintDynamicInfoStruct(&pDevObj->dynamicInfo);
        VpPrintStateInformation(pDevObj->state);

        VpSysDebugPrintf("\n\rpDevObj->stateInt = 0x%04X", pDevObj->stateInt);

        VpPrintDeviceProfileStruct(VP_DEV_890_SERIES, &pDevObj->devProfileData);

        VpPrintEventMaskStruct(TRUE, TRUE, &pDevObj->deviceEventsMask);
        VpPrintEventMaskStruct(TRUE, FALSE, &pDevObj->deviceEvents);

        VpPrintEventHandle(pDevObj->eventHandle);
        VpPrintTimeStamp(pDevObj->timeStamp);

        VpPrintGetResultsOptionStruct(&pDevObj->getResultsOption);
        VpPrintCriticalFltStruct(&pDevObj->criticalFault);
        VpPrintRelGainResultsStruct(&pDevObj->relGainResults);

#ifdef VP890_FXS_SUPPORT
        VpSysDebugPrintf("\n\n\rpDevObj->swParamsCache = 0x%02X 0x%02X 0x%02X",
            pDevObj->swParamsCache[0], pDevObj->swParamsCache[1],
            pDevObj->swParamsCache[2]);
        VpSysDebugPrintf("\n\rpDevObj->switchCtrl = 0x%02X",
            pDevObj->switchCtrl[0]);
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
#if !defined(VP_REDUCED_API_IF) || defined(VP_CC_KWRAP)
        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->mpiData =");
            for (byteCount = 0; byteCount < VP890_MAX_MPI_DATA; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->mpiData[byteCount]);
            }
        }
#endif

#ifdef VP890_FXS_SUPPORT
        VpPrintPulseSpecs(0, &pDevObj->pulseSpecs);
        VpPrintPulseSpecs(1, &pDevObj->pulseSpecs2);
#endif
        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->testDataBuffer =");
            for (byteCount = 0; byteCount < VP890_TEST_DATA_LEN; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->testDataBuffer[byteCount]);
            }
        }

#ifdef VP890_INCLUDE_TESTLINE_CODE
        VpSysDebugPrintf("\n\n\rpDevObj->testResults.testId = 0x%04X",
            pDevObj->testResults.testId);
        VpSysDebugPrintf("\n\rpDevObj->testResults.errorCode = 0x%04X",
            pDevObj->testResults.errorCode);
        {
            uint8 byteCount;
            uint8 *pTestData = (uint8 *)(&pDevObj->testResults.result);
            uint8 testDataSize = sizeof(VpTestResultsUnionType);

            VpSysDebugPrintf("\n\rpDevObj->testResults.result =");
            for (byteCount = 0; byteCount < testDataSize; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", *pTestData);
                pTestData++;
            }
        }

        VpSysDebugPrintf("\n\n\rpDevObj->currentTest.pTestHeap = %p",
            (void *)pDevObj->currentTest.pTestHeap);
        VpSysDebugPrintf("\n\rpDevObj->currentTest.testHeapId = %d",
            pDevObj->currentTest.testHeapId);
        VpSysDebugPrintf("\n\rpDevObj->currentTest.channelId = %d",
            pDevObj->currentTest.channelId);
        VpSysDebugPrintf("\n\rpDevObj->currentTest.prepared = %s",
            ((pDevObj->currentTest.prepared == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpDevObj->currentTest.preparing = %s",
            ((pDevObj->currentTest.preparing == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpDevObj->currentTest.concluding = %s",
            ((pDevObj->currentTest.concluding == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpDevObj->currentTest.nonIntrusiveTest = %s",
            ((pDevObj->currentTest.nonIntrusiveTest == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpDevObj->currentTest.testId = 0x%04X",
            pDevObj->currentTest.testId);
        VpSysDebugPrintf("\n\rpDevObj->currentTest.testState = %d",
            pDevObj->currentTest.testState);
        VpSysDebugPrintf("\n\rpDevObj->currentTest.handle = %d",
            pDevObj->currentTest.handle);

        {
            uint8 chanNum;
            for (chanNum = 0; chanNum < VP890_MAX_NUM_CHANNELS; chanNum++) {
                VpSysDebugPrintf("\n\n\rpDevObj->calOffsets[%d].nullOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].nullOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].vabOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].vabOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].vahOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].vahOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].valOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].valOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].vbhOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].vbhOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].vblOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].vblOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].batOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].batOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].imtOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].imtOffset);
                VpSysDebugPrintf("\n\rpDevObj->calOffsets[%d].ilgOffset = 0x%04X",
                    chanNum, pDevObj->calOffsets[chanNum].ilgOffset);
            }
        }
#endif

        VpSysDebugPrintf("\n\n\rpDevObj->debugSelectMask = 0x%08lX",
            pDevObj->debugSelectMask);

        VpSysDebugPrintf("\n\n\rpDevObj->ecVal = 0x%02X", pDevObj->ecVal);

        VpSysDebugPrintf("\n\n\rpDevObj->lastCodecChange = %d",
            pDevObj->lastCodecChange);

#if defined (VP_CC_890_SERIES) || defined (VP_CC_KWRAP)
        {
            uint8 chan, polarity;

            VpSysDebugPrintf("\n\n\rpDevObj->vp890SysCalData.abvError[0] = %d",
                pDevObj->vp890SysCalData.abvError[0]);

            for (chan = 0; chan < VP890_SYS_CAL_CHANNEL_LENGTH; chan++) {
                VpSysDebugPrintf("\n");

                for (polarity = 0; polarity < VP890_SYS_CAL_POLARITY_LENGTH; polarity++) {
                    VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vocOffset[%d][%d] = %d",
                        chan, polarity, pDevObj->vp890SysCalData.vocOffset[chan][polarity]);
                }
                for (polarity = 0; polarity < VP890_SYS_CAL_POLARITY_LENGTH; polarity++) {
                    VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vocError[%d][%d] = %d",
                        chan, polarity, pDevObj->vp890SysCalData.vocError[chan][polarity]);
                }
                for (polarity = 0; polarity < VP890_SYS_CAL_POLARITY_LENGTH; polarity++) {
                    VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.sigGenAError[%d][%d] = %d",
                        chan, polarity, pDevObj->vp890SysCalData.sigGenAError[chan][polarity]);
                }

                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ila20[%d] = %d",
                    chan, pDevObj->vp890SysCalData.ila20[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ila25[%d] = %d",
                    chan, pDevObj->vp890SysCalData.ila25[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ila32[%d] = %d",
                    chan, pDevObj->vp890SysCalData.ila32[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ila40[%d] = %d",
                    chan, pDevObj->vp890SysCalData.ila40[chan]);

                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ilaOffsetNorm[%d] = %d",
                    chan, pDevObj->vp890SysCalData.ilaOffsetNorm[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ilgOffsetNorm[%d] = %d",
                    chan, pDevObj->vp890SysCalData.ilgOffsetNorm[chan]);

                for (polarity = 0; polarity < VP890_SYS_CAL_POLARITY_LENGTH; polarity++) {
                    VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vas[%d][%d] = 0x%02X",
                        chan, polarity, pDevObj->vp890SysCalData.vas[chan][polarity]);
                }

                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vagOffsetNorm[%d] = %d",
                    chan, pDevObj->vp890SysCalData.vagOffsetNorm[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vagOffsetRev[%d] = %d",
                    chan, pDevObj->vp890SysCalData.vagOffsetRev[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vbgOffsetNorm[%d] = %d",
                    chan, pDevObj->vp890SysCalData.vbgOffsetNorm[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.vbgOffsetRev[%d] = %d",
                    chan, pDevObj->vp890SysCalData.vbgOffsetRev[chan]);

                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.swyOffset[%d] = %d",
                    chan, pDevObj->vp890SysCalData.swyOffset[chan]);

                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.tipCapCal[%d] = %ld",
                    chan, pDevObj->vp890SysCalData.tipCapCal[chan]);
                VpSysDebugPrintf("\n\rpDevObj->vp890SysCalData.ringCapCal[%d] = %ld",
                    chan, pDevObj->vp890SysCalData.ringCapCal[chan]);
            }
        }
#endif
    }

    if (pLineCtx != VP_NULL) {
        Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;

        VP_PRINT_LINE_ID(pLineObj->lineId);

        VpSysDebugPrintf("\n\n\rpLineObj->channelId = %d\npLineObj->ecVal = 0x%02X",
            pLineObj->channelId, pLineObj->ecVal);

        VpPrintTermType(pLineObj->termType);

        VpSysDebugPrintf("\n\rpLineObj->status = 0x%04X ", pLineObj->status);
        VpSysDebugPrintf("\n\rpLineObj->debugSelectMask = 0x%08lX ", pLineObj->debugSelectMask);

        VpPrintEventMaskStruct(FALSE, TRUE, &pLineObj->lineEventsMask);
        VpPrintEventMaskStruct(FALSE, FALSE, &pLineObj->lineEvents);

#ifdef VP890_FXS_SUPPORT
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

        VpSysDebugPrintf("\n\n\rpLineObj->icr1 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr1Values[0], pLineObj->icr1Values[1],
            pLineObj->icr1Values[2], pLineObj->icr1Values[3]);

#ifdef VP890_FXS_SUPPORT
        VpSysDebugPrintf("\n\rpLineObj->icr2 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
            pLineObj->icr2Values[2], pLineObj->icr2Values[3]);
        VpSysDebugPrintf("\n\rpLineObj->icr3 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr3Values[0], pLineObj->icr3Values[1],
            pLineObj->icr3Values[2], pLineObj->icr3Values[3]);
        VpSysDebugPrintf("\n\rpLineObj->icr4 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->icr4Values[0], pLineObj->icr4Values[1],
            pLineObj->icr4Values[2], pLineObj->icr4Values[3]);
#endif
        VpSysDebugPrintf("\n\rpLineObj->sigGenCtrl = 0x%02X", pLineObj->sigGenCtrl[0]);

        VpSysDebugPrintf("\n\n\rpLineObj->gxBase = 0x%04X", pLineObj->gxBase);
        VpSysDebugPrintf("\n\rpLineObj->gxUserLevel = 0x%04X", pLineObj->gxUserLevel);
        VpSysDebugPrintf("\n\rpLineObj->gxCidLevel = 0x%04X", pLineObj->gxCidLevel);
        VpSysDebugPrintf("\n\rpLineObj->grBase = 0x%04X", pLineObj->grBase);
        VpSysDebugPrintf("\n\rpLineObj->grUserLevel = 0x%04X", pLineObj->grUserLevel);
        VpSysDebugPrintf("\n\rpLineObj->absGxGain = %d", pLineObj->absGxGain);
        VpSysDebugPrintf("\n\rpLineObj->absGrGain = %d", pLineObj->absGrGain);

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
            for (intSeqData = 0; intSeqData < VP890_INT_SEQ_LEN; intSeqData++) {
                VpSysDebugPrintf(" 0x%02X", pLineObj->intSequence[intSeqData]);
            }
        }
        VpPrintSeqDataType(&pLineObj->cadence);
#endif
        VpPrint890CalLineData(&pLineObj->calLineData);
        VpPrintVpCslacTimerStruct(&pLineObj->lineTimers);

#ifdef VP890_FXO_SUPPORT
        VpSysDebugPrintf("\n\n\rFXO ONLY DATA:");
        VpSysDebugPrintf("\n\rpLineObj->fxoData = 0x%02X", pLineObj->fxoData);
        VpSysDebugPrintf("\n\rpLineObj->fxoRingStateFlag = 0x%02X", pLineObj->fxoRingStateFlag);
        VpSysDebugPrintf("\n\rpLineObj->preDisconnect = 0x%02X", pLineObj->preDisconnect);
        VpSysDebugPrintf("\n\rpLineObj->ringDetMax = %d", pLineObj->ringDetMax);
        VpSysDebugPrintf("\n\rpLineObj->ringDetMin = %d", pLineObj->ringDetMin);
        VpSysDebugPrintf("\n\rpLineObj->dPoh = %d", pLineObj->dPoh);
        VpSysDebugPrintf("\n\rpLineObj->userDtg = %d", pLineObj->userDtg);
        VpSysDebugPrintf("\n\rpLineObj->cidDtg = %d", pLineObj->cidDtg);
        VpSysDebugPrintf("\n\rpLineObj->cidCorrectionSample = %d", pLineObj->cidCorrectionSample);
        VpSysDebugPrintf("\n\rpLineObj->cidCorrectionCtr = %d", pLineObj->cidCorrectionCtr);
        VpPrintLineStateType(pLineObj->fxoRingState, "pLineObj->fxoRingState");

        VpSysDebugPrintf("\n");
        VpPrintDigitGenDataType(&pLineObj->digitGenStruct);

        VpSysDebugPrintf("\n\n\rpLineObj->lowVoltageDetection.enabled = %s",
            ((pLineObj->lowVoltageDetection.enabled == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpLineObj->lowVoltageDetection.numDisc = %d",
            pLineObj->lowVoltageDetection.numDisc);
        VpSysDebugPrintf("\n\rpLineObj->lowVoltageDetection.numNotDisc = %d",
            pLineObj->lowVoltageDetection.numNotDisc);

        VpSysDebugPrintf("\n\n\rpLineObj->pllRecoveryState = %d",
            pLineObj->pllRecoveryState);
        VpSysDebugPrintf("\n\rpLineObj->pllRecoverAttempts = %d", pLineObj->pllRecoverAttempts);

        VpSysDebugPrintf("\n\n\rpLineObj->currentMonitor.stateValue = %d",
            pLineObj->currentMonitor.stateValue);
        VpSysDebugPrintf("\n\rpLineObj->currentMonitor.currentOffset = %d %d %d %d %d",
            pLineObj->currentMonitor.currentBuffer[0], pLineObj->currentMonitor.currentBuffer[1],
            pLineObj->currentMonitor.currentBuffer[2], pLineObj->currentMonitor.currentBuffer[3],
            pLineObj->currentMonitor.currentBuffer[4]);
        VpSysDebugPrintf("\n\rpLineObj->currentMonitor.currentOffset = %d",
            pLineObj->currentMonitor.currentOffset);
        VpSysDebugPrintf("\n\rpLineObj->currentMonitor.offsetMeasurements = %d",
            pLineObj->currentMonitor.offsetMeasurements);
        VpSysDebugPrintf("\n\rpLineObj->currentMonitor.steadyStateAverage = %li",
            pLineObj->currentMonitor.steadyStateAverage);
        VpSysDebugPrintf("\n\rpLineObj->currentMonitor.invalidData = %s",
            ((pLineObj->currentMonitor.invalidData == TRUE) ? "TRUE" : "FALSE"));
#endif

#ifdef VP890_FXS_SUPPORT
        VpSysDebugPrintf("\n\nFXS ONLY DATA:");
#ifdef VP890_LP_SUPPORT
         VpSysDebugPrintf("\n\rpLineObj->leakyLineCnt = %d", pLineObj->leakyLineCnt);
#endif
        VpSysDebugPrintf("\n\rpLineObj->ringingParams = 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->ringingParams[0], pLineObj->ringingParams[1],
            pLineObj->ringingParams[2], pLineObj->ringingParams[3],
            pLineObj->ringingParams[4], pLineObj->ringingParams[5],
            pLineObj->ringingParams[6], pLineObj->ringingParams[7],
            pLineObj->ringingParams[8], pLineObj->ringingParams[9],
            pLineObj->ringingParams[10]);
        VpSysDebugPrintf("\n\rpLineObj->dcCalValues = 0x%02X 0x%02X",
            pLineObj->dcCalValues[0], pLineObj->dcCalValues[1]);
        VpSysDebugPrintf("\n\rpLineObj->hookHysteresis = %d", pLineObj->hookHysteresis);
        VpSysDebugPrintf("\n\rpLineObj->internalTestTermApplied = %s",
            ((pLineObj->internalTestTermApplied == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpLineObj->pRingingCadence = %p", pLineObj->pRingingCadence);
        VpSysDebugPrintf("\n\rpLineObj->pCidProfileType1 = %p", pLineObj->pCidProfileType1);
        VpPrintOptionRingControlType(&pLineObj->ringCtrl);
        VpPrintRelayControlType(pLineObj->relayState);
#ifdef VP_CSLAC_SEQ_EN
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
VpPrint890CalLineData(
    Vp890CalLineData *calLineData)
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
        for (dcFeedIndex = 0; dcFeedIndex < VP890_DC_FEED_LEN; dcFeedIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->dcFeedRef[dcFeedIndex]);
        }
        VpSysDebugPrintf("\npLineObj->calLineData.dcFeed =");
        for (dcFeedIndex = 0; dcFeedIndex < VP890_DC_FEED_LEN; dcFeedIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->dcFeed[dcFeedIndex]);
        }
        VpSysDebugPrintf("\npLineObj->calLineData.dcFeedPr =");
        for (dcFeedIndex = 0; dcFeedIndex < VP890_DC_FEED_LEN; dcFeedIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->dcFeedPr[dcFeedIndex]);
        }
    }
    {
        uint8 icr2Index;
        VpSysDebugPrintf("\npLineObj->calLineData.icr2 =");
        for (icr2Index = 0; icr2Index < VP890_ICR2_LEN; icr2Index++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->icr2[icr2Index]);
        }
    }
    {
        uint8 disnIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.disnVal =");
        for (disnIndex = 0; disnIndex < VP890_DISN_LEN; disnIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->disnVal[disnIndex]);
        }
    }
    {
        uint8 vpGainIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.vpGain =");
        for (vpGainIndex = 0; vpGainIndex < VP890_VP_GAIN_LEN; vpGainIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->vpGain[vpGainIndex]);
        }
    }
    {
        uint8 loopSupIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.loopSup =");
        for (loopSupIndex = 0; loopSupIndex < VP890_LOOP_SUP_LEN; loopSupIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->loopSup[loopSupIndex]);
        }
    }
    {
        uint8 sigGenAIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.sigGenA =");
        for (sigGenAIndex = 0; sigGenAIndex < VP890_SIGA_PARAMS_LEN; sigGenAIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->sigGenA[sigGenAIndex]);
        }
    }
    {
        uint8 calRegIndex;
        VpSysDebugPrintf("\npLineObj->calLineData.calReg =");
        for (calRegIndex = 0; calRegIndex < VP890_DC_CAL_REG_LEN; calRegIndex++) {
            VpSysDebugPrintf(" 0x%02X", calLineData->calReg[calRegIndex]);
        }
    }
    {
        uint8 typeDataIndex;
        uint8 *pTypeData = (uint8 *)(&calLineData->typeData);
        uint8 typeDataSize = sizeof(Vp890CalTypeData);
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
    VpSysDebugPrintf("\npLineObj->calLineData.calState = 0x%04X", calLineData->calState);
    VpSysDebugPrintf("\npLineObj->calLineData.sysState = 0x%02X", calLineData->sysState);
    VpSysDebugPrintf("\npLineObj->calLineData.vasStart = %d", calLineData->vasStart);
    VpSysDebugPrintf("\npLineObj->calLineData.minVas = %d\n", calLineData->minVas);
}
#endif

#endif /* VP_CC_890_SERIES */
