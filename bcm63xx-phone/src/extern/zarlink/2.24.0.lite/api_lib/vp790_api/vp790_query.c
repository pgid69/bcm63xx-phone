/** \file vp790_query.c
 * vp790_query.c
 *
 *  This file contains the query functions used in the Vp790 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 10850 $
 * $LastChangedDate: 2013-03-05 10:34:01 -0600 (Tue, 05 Mar 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_790_SERIES)

/* Project Includes */
#include "vp_api_types.h"
#include "vp_hal.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp790_api.h"
#include "vp790_api_int.h"
#include "sys_service.h"

/* Private Functions */
static uint16
Vp790CheckLineEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp790LineObjectType *pLineObj);

static uint16
Vp790CheckDevEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp790DeviceObjectType *pDevObj);

static bool
Vp790ServiceFaultTimer(
    VpLineCtxType *pLineCtx);

static bool
Vp790ServiceHookInt(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType hookSt);

static bool
Vp790ServiceGkeyInt(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType gkeySt);

/**
 * Vp790FindSoftwareInterrupts()
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
Vp790FindSoftwareInterrupts(
    VpDevCtxType *pDevCtx)
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp790LineObjectType *pLineObj;
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

    /* Evaluate if any events remain */
    if(pEvents->faults || pEvents->signaling || pEvents->response
    || pEvents->process) {
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

            /* Evaluate if any events remain */
            if(pEvents->faults || pEvents->signaling || pEvents->response
            || pEvents->process) {
                return TRUE;
            }
        }
    }
    return FALSE;
}

/**
 * Vp790GetEvent()
 *  This function reports new events that occured on the device. This function
 * returns one event for each call to it. It should be called repeatedly until
 * no more events are reported for a specific device. This function does not
 * access the device, it returns status from the phantom registers that are
 * maintained by the API tick routine.
 *
 * Preconditions:
 *
 * Postconditions:
 *  Returns true if the device tested has changed state. The event type is
 * stored in the VpEventType structure whos location is passed as a pointer.
 */
bool
Vp790GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)    /* Pointer to the results event structure */
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp790LineObjectType *pLineObj;
    VpLineCtxType *pLineCtx;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 i, eventCatLoop;
    uint8 chan;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

#define VP790_EVENT_ARRAY_SIZE  4
    uint16 eventArray[VP790_EVENT_ARRAY_SIZE];
    uint16 eventMaskArray[VP790_EVENT_ARRAY_SIZE];
    VpEventCategoryType eventCat[] = {
        VP_EVCAT_FAULT,
        VP_EVCAT_SIGNALING,
        VP_EVCAT_RESPONSE,
        VP_EVCAT_PROCESS
    };

    pEvent->status = VP_STATUS_SUCCESS;
    pEvent->hasResults = FALSE;

    /* Initialize the arrays for device events */
    for (i = 0; i < VP790_EVENT_ARRAY_SIZE; i++) {
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

            default:
                return VP_STATUS_INVALID_ARG;
        }
    }

    /* Look for active device events first */
    for (eventCatLoop = 0;
         eventCatLoop < VP790_EVENT_ARRAY_SIZE;
         eventCatLoop++) {
        pEvent->eventId = Vp790CheckDevEvent(eventArray[eventCatLoop],
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
                    /*
                     * Read Options is treated by the device event flag because
                     * it can be either a line read or device read. Get the
                     * channel information for line read, otherwise those values
                     * are set to 0 and VP_NULL.
                     */
                    case VP_LINE_EVID_RD_OPTION:
                        pEvent->channelId = pDevObj->getResultsOption.chanId;
                        pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
                        if (pEvent->pLineCtx != VP_NULL) {
                            Vp790LineObjectType *pLineObjLocal = pEvent->pLineCtx->pLineObj;
                            pEvent->lineId = pLineObjLocal->lineId;
                        }
                        pEvent->hasResults = TRUE;
                        pEvent->eventData = pDevObj->getResultsOption.optionType;
                        break;

                    case VP_DEV_EVID_IO_ACCESS_CMP:
                        pEvent->eventData =
                            pDevObj->getResultsOption.optionData.deviceIoData.accessType;
                        if (pEvent->eventData == VP_DEVICE_IO_READ) {
                            pEvent->hasResults = TRUE;
                        } else {
                            pEvent->hasResults = FALSE;
                        }
                        break;

                    case VP_DEV_EVID_DEV_INIT_CMP:
                        pEvent->eventData = 1;
                        break;

                    default:
                        break;
                }
            }

            if (pEvent->eventCategory == VP_EVCAT_FAULT) {
                switch(pEvent->eventId) {
                    case VP_DEV_EVID_CLK_FLT:
                        pEvent->eventData = pDevObj->dynamicInfo.clkFault;
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
            return TRUE;
        }
    }

    /* No device events, now look for Line events -- but make sure the line
     * context is valid before looking for a line object
     */

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    for(chan = pDevObj->dynamicInfo.lastChan; chan < maxChan; chan++) {
        pLineCtx = pDevCtx->pLineCtx[chan];
        if (pLineCtx != VP_NULL) {
            /* The line context is valid, create a line object and initialize
             * the event arrays for this line
             */
            pLineObj = pLineCtx->pLineObj;
            for (i = 0; i < VP790_EVENT_ARRAY_SIZE; i++) {
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

                    default:
                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        return VP_STATUS_INVALID_ARG;
                }
            }

            /* Check this line events */
            for (eventCatLoop = 0;
                 eventCatLoop < VP790_EVENT_ARRAY_SIZE;
                 eventCatLoop++) {
                pEvent->eventId = Vp790CheckLineEvent(eventArray[eventCatLoop],
                    eventMaskArray[eventCatLoop], eventCat[eventCatLoop],
                    pLineObj);

                if (pEvent->eventId != 0x0000) {
                    pEvent->deviceId = deviceId;
                    pEvent->channelId = chan;
                    pEvent->pLineCtx = pDevCtx->pLineCtx[chan];
                    pEvent->pDevCtx = pDevCtx;
                    pEvent->eventCategory = eventCat[eventCatLoop];
                    pEvent->parmHandle = pLineObj->lineEventHandle;
                    pEvent->hasResults = FALSE;
                    pEvent->lineId = pLineObj->lineId;

                    switch(pEvent->eventCategory) {
                        case VP_EVCAT_RESPONSE:
                            switch(pEvent->eventId) {
                                case VP_LINE_EVID_LLCMD_RX_CMP:
                                case VP_LINE_EVID_RD_LOOP:
                                    pEvent->hasResults = TRUE;
                                    break;

                                default:
                                    break;
                            }
                            break;

                        case VP_EVCAT_SIGNALING:
                            if (pEvent->eventId == VP_LINE_EVID_DTMF_DIG) {
                                /*
                                 * Upper bits are used for the timestamp.
                                 * Lower bits are used for the digit and the
                                 * make/break bit.
                                 */
                                pEvent->eventData = (pDevObj->timeStamp << 5)
                                    | pLineObj->dtmfDigitSense;
                            } else {
                                pEvent->eventData = pLineObj->signalingData;
                            }
                            break;

                        case VP_EVCAT_PROCESS:
                            pEvent->eventData = pLineObj->processData;
                            break;

                        case VP_EVCAT_FAULT:
                            switch(pEvent->eventId) {
                                case VP_LINE_EVID_THERM_FLT:
                                    pEvent->eventData =
                                        (pLineObj->lineState.condition
                                       & VP_CSLAC_THERM_FLT) ? TRUE : FALSE;
                                    break;

                                case VP_LINE_EVID_DC_FLT:
                                    pEvent->eventData =
                                        (pLineObj->lineState.condition
                                       & VP_CSLAC_DC_FLT) ? TRUE : FALSE;
                                    break;

                                case VP_LINE_EVID_AC_FLT:
                                    pEvent->eventData =
                                        (pLineObj->lineState.condition
                                       & VP_CSLAC_AC_FLT) ? TRUE : FALSE;
                                    break;

                                default:
                                    break;
                            }
                            break;

                        default:
                            break;
                    }

                    /*
                     * We're returning, so update the device last channel that
                     * was checked so we start at the next channel
                     */
                    pDevObj->dynamicInfo.lastChan = chan + 1;
                    if (pDevObj->dynamicInfo.lastChan >= maxChan) {
                        pDevObj->dynamicInfo.lastChan = 0;
                    }
                    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                    return TRUE;
                }
            }
        }
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return FALSE;
} /* End Vp790GetEvent */

/**
 * Vp790CheckDevEvent()
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
Vp790CheckDevEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp790DeviceObjectType *pDevObj)
{
    uint8 i;
    uint16 mask;

    for (i = 0, mask = 0x0001; i < 16; i++, (mask = mask << 1)) {
        /* Check to see if an event MAY be reported */
        if ((mask & event) != 0) {
            /* Have to clear the device event so we don't report this event
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

                default:
                    return VP_STATUS_INVALID_ARG;
            }

            /* If the event is not masked, return the event */
            if ((mask & eventMask) == 0) {
                return mask;
            }
        }
    }
    return 0x0000;
}

/**
 * Vp790CheckLineEvent()
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
Vp790CheckLineEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp790LineObjectType *pLineObj)
{
    uint8 i;
    uint16 mask;

    for (i = 0, mask = 0x0001; i < 16; i++, (mask = mask << 1)) {
        /* Check to see if an event MAY be reported */
        if ((mask & event) != 0) {
            /* Have to clear the device event so we don't report this event
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

                default:
                    return VP_STATUS_INVALID_ARG;
            }

            /* If the event is not masked, return the event */
            if ((mask & eventMask) == 0) {
                return mask;
            }
        }
    }
    return 0x0000;
}

/**
 * Vp790GetOption()
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
Vp790GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle)
{
    VpDevCtxType *pDevCtxLocal;
    Vp790LineObjectType *pLineObj;
    Vp790DeviceObjectType *pDevObj;

    VpStatusType status = VP_STATUS_SUCCESS;
    VpGetResultsOptionsDataType *pOptionData;

    uint8 channelId, txSlot, rxSlot;
    VpDeviceIdType deviceId;
    uint8 tempLoopBack, codecReg;
    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint8 gioDir[VP790_GIO_DIR_LEN];

    if (pLineCtx != VP_NULL) {
        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        deviceId = pDevObj->deviceId;
        pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
        pOptionData = &(pDevObj->getResultsOption.optionData);

        /* Proceed if initialized or in progress, and not calibrating */
        if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
            if (pDevObj->status.state & VP_DEV_IN_CAL) {
                return VP_STATUS_DEVICE_BUSY;
            }
        } else {
            return VP_STATUS_DEV_NOT_INITIALIZED;
        }

        if (pDevObj->deviceEvents.response & VP790_READ_RESPONSE_MASK) {
            return VP_STATUS_DEVICE_BUSY;
        }

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

        pDevObj->getResultsOption.chanId = channelId;
        switch (option) {
            /* Line Options */
            case VP_OPTION_ID_PULSE_MODE:
                pOptionData->pulseModeOption = pLineObj->pulseMode;
                break;

            case VP_OPTION_ID_TIMESLOT:
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_TX_TS_RD,
                    VP790_TX_TS_LEN, &txSlot);
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_RX_TS_RD,
                    VP790_RX_TS_LEN, &rxSlot);

                pOptionData->timeSlotOption.tx = (txSlot & VP790_TX_TS_MASK);
                pOptionData->timeSlotOption.rx = (rxSlot & VP790_RX_TS_MASK);
                break;

            case VP_OPTION_ID_CODEC:
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_CODEC_REG_RD,
                    VP790_CODEC_REG_LEN, &codecReg);

                switch(codecReg & VP790_CODEC_COMPRESSION_MASK) {
                    case (VP790_LINEAR_CODEC | VP790_ALAW_CODEC):
                    case (VP790_LINEAR_CODEC | VP790_ULAW_CODEC):
                        pOptionData->codecOption = VP_OPTION_LINEAR;
                        break;

                    case VP790_ALAW_CODEC:
                        pOptionData->codecOption = VP_OPTION_ALAW;
                        break;

                    default:
                        pOptionData->codecOption = VP_OPTION_MLAW;
                        break;
                }
                break;

            case VP_OPTION_ID_PCM_HWY:
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_TX_TS_RD,
                    VP790_TX_TS_LEN, &txSlot);

                if (txSlot & VP790_TX_HWY_B) {
                    pOptionData->pcmHwyOption = VP_OPTION_HWY_B;
                } else {
                    pOptionData->pcmHwyOption = VP_OPTION_HWY_A;
                }
                break;

            case VP_OPTION_ID_LOOPBACK:
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_LOOPBACK_RD,
                    VP790_LOOPBACK_LEN, &tempLoopBack);
                if (tempLoopBack & VP790_PCM_LOOPBACK ) {
                    pOptionData->loopBackOption = VP_OPTION_LB_TIMESLOT;
                } else if (tempLoopBack & VP790_FULL_LOOPBACK) {
                    pOptionData->loopBackOption = VP_OPTION_LB_DIGITAL;
                } else {
                    pOptionData->loopBackOption = VP_OPTION_LB_OFF;
                }
                break;

            case VP_OPTION_ID_LINE_STATE:
                pOptionData->lineStateOption.bat =
                    pLineObj->lineStateBatOption.bat;
                break;

            case VP_OPTION_ID_EVENT_MASK:
                pOptionData->eventMaskOption = pLineObj->lineEventsMask;
                break;

            case VP_OPTION_ID_ZERO_CROSS:
                pOptionData->zeroCross = pLineObj->zeroCross;
                break;

            case VP_OPTION_ID_RING_CNTRL:
                pOptionData->ringControlOption = pLineObj->ringCtrl;
                break;

            case VP_OPTION_ID_PCM_TXRX_CNTRL:
                pOptionData->pcmTxRxCtrl = pLineObj->pcmTxRxCtrl;
                break;

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
    } else {
        /*
         * Upper layer checks to be sure that either device context or line
         * context pointers are not null -- so the device context is not null
         * in this case.
         */
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;
        pOptionData = &(pDevObj->getResultsOption.optionData);

        if (pDevObj->deviceEvents.response & VP790_READ_RESPONSE_MASK) {
            return VP_STATUS_DEVICE_BUSY;
        }

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

        switch (option) {
            case VP_DEVICE_OPTION_ID_PULSE:
                pOptionData->pulseTypeOption = pDevObj->pulseSpecs;
                break;

            case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
                pOptionData->criticalFaultOption = pDevObj->criticalFault;
                break;

            case VP_OPTION_ID_EVENT_MASK:
                pOptionData->eventMaskOption = pDevObj->deviceEventsMask;
                break;

            case VP_DEVICE_OPTION_ID_DEVICE_IO:
                /*
                 * Device does not support than 4 (up to 8) pins, and all
                 * output pins are driven
                 */
                pOptionData->deviceIo.outputTypePins_63_32 = 0x00000000;
                pOptionData->deviceIo.outputTypePins_31_0 = 0x00000000;
                pOptionData->deviceIo.directionPins_63_32 = 0x00000000;

                /* Read the current I/O pin direction */
                VpMpiCmdWrapper(deviceId, ecVal[0], VP790_GIO_DIR_RD,
                    VP790_GIO_DIR_LEN, gioDir);

                pOptionData->deviceIo.directionPins_31_0 =
                    (gioDir[0] & VP790_GIO_DIR_MASK);
                break;

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
    }

    if (status == VP_STATUS_SUCCESS) {
        pDevObj->getResultsOption.optionType = option;
        pDevObj->deviceEvents.response |= VP_LINE_EVID_RD_OPTION;
        pDevObj->eventHandle = handle;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
}

/**
 * Vp790GetDeviceStatus()
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
Vp790GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus)
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    uint8 channelId;
    bool status;
    VpLineCtxType *pLineCtx;

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
    return VP_STATUS_SUCCESS;
}

/**
 * Vp790FlushEvents()
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
Vp790FlushEvents(
    VpDevCtxType *pDevCtx)
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VpLineCtxType *pLineCtx;
    Vp790LineObjectType *pLineObj;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

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
    return VP_STATUS_SUCCESS;
}

/**
 * Vp790GetResults()
 *  This function fills the results structure passed with the results data
 * found from the event that caused new results.
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
Vp790GetResults(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    uint8 commandByte;
    uint8 *pMpiData;
    uint8 mpiDataLen = pDevObj->mpiLen;
#endif

    VpGetResultsOptionsDataType *pOptionData =
        &(pDevObj->getResultsOption.optionData);

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    switch(pEvent->eventCategory) {
        case VP_EVCAT_RESPONSE:
            switch (pEvent->eventId) {
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
                case VP_LINE_EVID_LLCMD_RX_CMP:
                    pMpiData = (uint8 *)pResults;
                    for (commandByte = 0;
                         commandByte < mpiDataLen;
                         commandByte++) {
                        pMpiData[commandByte] = pDevObj->mpiData[commandByte];
                    }
                    break;
#endif

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

                        default:
                            status = VP_STATUS_INVALID_ARG;
                            break;
                    }
                    break;

                case VP_LINE_EVID_RD_LOOP:
                    *((VpLoopCondResultsType *)pResults) =
                        pOptionData->loopCond;
                    break;

                default:
                    status = VP_STATUS_INVALID_ARG;
                    break;
            }
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
}

/**
 * Vp790GetLoopCond()
 *  This function is used to obtain the telephone loop condition for a given
 * line. See VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  This function starts the necessary actions to obtain the loop condition.
 */
VpStatusType
Vp790GetLoopCond(
    VpLineCtxType *pLineCtx,
    uint16 handle)
{
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpLoopCondResultsType results;

    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    uint8 vsabData[VP790_VAB_LEN];
    uint8 vimtData[VP790_VIMT_LEN];
    uint8 vilgData[VP790_VILG_LEN];
    uint8 rloopData[VP790_VRLOOP_RD];
    uint8 meteringData[VP790_METERING_PEAK_LEN];
    uint8 vblData[VP790_LOW_BATT_LEN];
    uint8 vbhData[VP790_HIGH_BATT_LEN];
    uint8 vbpData[VP790_POS_BATT_LEN];
    uint8 meterType[VP790_CCR5_LEN];
    uint8 battSel[VP790_SLIC_STATE_LEN];
    uint8 sigRegArray[VP790_NO_UL_SIGREG_LEN];

    /* Proceed if initialized or in progress, and not calibrating */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
        if (pDevObj->status.state & VP_DEV_IN_CAL) {
            return VP_STATUS_DEVICE_BUSY;
        }
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pDevObj->deviceEvents.response & VP790_READ_RESPONSE_MASK) {
        return VP_STATUS_DEVICE_BUSY;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Device state is ready for loop conditions read */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_VAB_RD, VP790_VAB_LEN, vsabData);
    results.vsab = (vsabData[0] << 8);
    results.vsab |= vsabData[1];

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_VIMT_RD, VP790_VIMT_LEN,
        vimtData);
    results.imt = (vimtData[0] << 8);
    results.imt |= vimtData[1];

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_VILG_RD, VP790_VILG_LEN,
        vilgData);
    results.ilg = (vilgData[0] << 8);
    results.ilg |= vilgData[1];

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_VRLOOP_RD, VP790_VRLOOP_LEN,
        rloopData);
    results.rloop = (rloopData[0] << 8);
    results.rloop |= rloopData[1];

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_CCR5_RD, VP790_CCR5_LEN,
        meterType);

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_METERING_PEAK_RD,
        VP790_METERING_PEAK_LEN, meteringData);
    results.mspl = (meteringData[0] << 8);
    results.mspl |= meteringData[1];

    if ((meterType[0] & VP790_MTR_FREQ_16KHZ) != VP790_MTR_FREQ_16KHZ) {
        results.mspl = (results.mspl << 3);
        results.mspl /= 5;
    }

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_LOW_BATT_RD, VP790_LOW_BATT_LEN,
        vblData);
    results.vbat1  = (vblData[0] << 8);
    results.vbat1 |= vblData[1];
    results.vbat1 = -results.vbat1;

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_HIGH_BATT_RD,
        VP790_HIGH_BATT_LEN, vbhData);
    results.vbat2 = (vbhData[0] << 8);
    results.vbat2 |= vbhData[1];
    results.vbat2 = -results.vbat2;

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_POS_BATT_RD, VP790_POS_BATT_LEN,
        vbpData);
    results.vbat3 = (vbpData[0] << 8);
    results.vbat3 |= vbpData[1];
    results.vbat3 = -results.vbat3;

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_SLIC_STATE_RD,
        VP790_SLIC_STATE_LEN, battSel);

    if (battSel[0] & VP790_SLIC_HIGH_BAT_SELECT) {
        results.selectedBat = VP_BATTERY_2;
    } else {
        results.selectedBat = VP_BATTERY_1;
    }

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_NO_UL_SIGREG_RD,
        VP790_NO_UL_SIGREG_LEN, sigRegArray);
    if (sigRegArray[0] & VP790_SIGREG_AST) {
        results.dcFeedReg = VP_DF_ANTI_SAT_REG;
    } else if (sigRegArray[0] & VP790_SIGREG_ICON) {
        results.dcFeedReg = VP_DF_CNST_CUR_REG;
    } else {
        results.dcFeedReg = VP_DF_RES_FEED_REG;
    }

    pLineObj->lineEventHandle = handle;
    pLineObj->lineEvents.response |= VP_LINE_EVID_RD_LOOP;
    pDevObj->getResultsOption.optionData.loopCond = results;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp790ServiceTimers()
 *  This function services active API timers on all channels of deviceId.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function.
 *
 * Postconditions:
 *  All Active Timers have been serviced.
 */
bool
Vp790ServiceTimers(
    VpDevCtxType *pDevCtx)
{
    VpLineCtxType *pLineCtx;
    Vp790LineObjectType *pLineObj;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint16 tempTimer;

    uint8 lineTimerType;
    uint8 devTimerType;

    VpCslacLineCondType tempHookSt;

    uint8 channelId, data, maxChan;
    bool retFlag = FALSE;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    for (devTimerType = 0; devTimerType < VP_DEV_TIMER_LAST; devTimerType++) {
        if (pDevObj->devTimer[devTimerType] & VP_ACTIVATE_TIMER) {
            /* get the bits associated with the timer into a temp varaible*/
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
                    case VP_DEV_TIMER_CLKFAIL:
                        /*
                         * Re-enable clock interrupt to see if condition is
                         * still true
                         */
                        VpMpiCmdWrapper(deviceId, ecVal[0], VP790_GD_MASK_RD,
                            VP790_GD_MASK_LEN, &data);
                        data &= ~VP790_GD_STAT_CFAIL;
                        VpMpiCmdWrapper(deviceId, ecVal[0], VP790_GD_MASK_WRT,
                            VP790_GD_MASK_LEN, &data);
                        break;

                    default:
                        break;
                }
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
        }
    }

    /* Iterate through the channels until all timers are serviced */
    maxChan = pDevObj->staticInfo.maxChannels;
    for(channelId=0; channelId < maxChan; channelId++ ) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

#ifdef VP_CSLAC_SEQ_EN
            /*
             * If we're in the middle of caller ID and the debounce timer has
             * been set but the timer is not yet activated, copy the timer to
             * the line debounce timer and activate.
             */
            if ((pLineObj->callerId.cliDebounceTime != 0)
             && (pLineObj->callerId.status & VP_CID_IN_PROGRESS)
             && (!(pLineObj->lineTimers.timers.timer[VP_LINE_CID_DEBOUNCE]
                  & VP_ACTIVATE_TIMER))) {

                pLineObj->lineTimers.timers.timer[VP_LINE_CID_DEBOUNCE] =
                    pLineObj->callerId.cliDebounceTime;

                pLineObj->lineTimers.timers.timer[VP_LINE_CID_DEBOUNCE] |=
                    VP_ACTIVATE_TIMER;
            }
#endif
            for (lineTimerType = 0; lineTimerType < VP_LINE_TIMER_LAST;
                 lineTimerType++) {

                if (pLineObj->lineTimers.timers.timer[lineTimerType]
                  & VP_ACTIVATE_TIMER) {

                    tempTimer = (pLineObj->lineTimers.timers.timer[lineTimerType]
                        & VP_TIMER_TIME_MASK);

                    if (tempTimer > 0) {
                        tempTimer--;
                    }

                    pLineObj->lineTimers.timers.timer[lineTimerType] = tempTimer;

                    if (tempTimer == 0) {
                        /* Perform appropriate action based on timerType */
                        pLineObj->lineTimers.timers.timer[lineTimerType]
                            &= ~(VP_ACTIVATE_TIMER);

                        switch (lineTimerType) {
                            case VP_LINE_RING_EXIT_PROCESS:
                                /*
                                 * This timer is set due to a ring exit state
                                 * that may or may not be part of Caller ID. CID
                                 * uses a seperate timer for known CID activity
                                 * on the line that may also require debounce
                                 */
                                VpMpiCmdWrapper(deviceId, ecVal[channelId],
                                    VP790_GSUP_RD, VP790_GSUP_LEN, &data);
                                data = (data >> (channelId * 2 + 1)) & 0x01;
                                tempHookSt = (data ? VP_CSLAC_HOOK : VP_CSLAC_STATUS_INVALID);

                                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK)
                                    != tempHookSt) {
                                    pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                                    pLineObj->lineState.condition |= tempHookSt;

                                    pLineObj->dpStruct.hookSt =
                                        ((tempHookSt == VP_CSLAC_HOOK) ? TRUE : FALSE);
                                    Vp790ServiceHookInt(pLineCtx, tempHookSt);
                                }

#ifdef VP_CSLAC_SEQ_EN
                                if ((tempHookSt == VP_CSLAC_HOOK) &&
                                    (pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
                                    VpCliStopCli(pLineCtx);
                                }
#endif
                                break;
#ifdef VP_CSLAC_SEQ_EN
                            case VP_LINE_CID_DEBOUNCE:
#endif
                            case VP_LINE_HOOK_FREEZE:
                                VpMpiCmdWrapper(deviceId, ecVal[channelId],
                                    VP790_GSUP_RD, VP790_GSUP_LEN, &data);

                                data = (data >> (channelId * 2 + 1)) & 0x01;
#ifdef VP_CSLAC_SEQ_EN
                                /* Hook event is fully debounced and ready to go */
                                if (lineTimerType == VP_LINE_CID_DEBOUNCE) {
                                    pLineObj->callerId.status |= VP_CID_IS_DEBOUNCE;
                                }
#endif
                                /*
                                 * If the hook has changed since before the
                                 * debounce, then service the hook
                                 */
                                tempHookSt = (data ? VP_CSLAC_HOOK : VP_CSLAC_STATUS_INVALID);

                                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK)
                                    != tempHookSt) {
                                    pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                                    pLineObj->lineState.condition |= tempHookSt;
                                    pLineObj->dpStruct.hookSt = tempHookSt ? TRUE : FALSE;
                                    Vp790ServiceHookInt(pLineCtx, tempHookSt);
                                }
                                break;

                            case VP_LINE_TIMER_FAULT:
                                retFlag = Vp790ServiceFaultTimer(pLineCtx);
                                break;

                            default:
                                break;
                        } /* Switch (timerType) */
                    } else { /* If timer has not expired */
                        pLineObj->lineTimers.timers.timer[lineTimerType]
                            |= VP_ACTIVATE_TIMER;
                    }
                } /* if timerType is active     */
            } /* Loop through all timerTypes for chanID */
        } /* Line Context Check */
    } /* Loop through channels until no more tests */
    return retFlag;
} /* Vp790ServiceTimers() */

/**
 * Vp790ContinueCalibrate()
 *  This function performs the necessary API Tick Actions when the device is
 *  calibrating
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function, if the device is
 *  calibrating.
 *
 * Postconditions:
 *  Returns TRUE, if calibration is complete, FALSE otherwise. Device Interrupts
 *  are re-enabled, if applicable to the Interrupt Mode
 */
bool
Vp790ContinueCalibrate(
    VpDevCtxType *pDevCtx)
{
    VpLineCtxType *pLineCtx;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp790LineObjectType *pLineObj;

#if defined (VP790_INTERRUPT_LEVTRIG_MODE)
    VpDeviceIdType deviceId = pDevObj->deviceId;
#endif

    uint8 channelId, maxChan;

    if (pDevObj->status.calibrating > 0) {
        pDevObj->status.calibrating--;
    }

    /* Has Calibration Completed? */
    if(pDevObj->status.calibrating == 0) {
        /* Flag a Calibration Complete Event */
        if(pDevObj->deviceEventsMask.response & VP_EVID_CAL_CMP) {
        } else {
            pDevObj->deviceEvents.response |= VP_EVID_CAL_CMP;
        }

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Clearing Calibration Flag at Time %d",
            pDevObj->timeStamp));

        /* Indicate calibration is complete for remainder of API */
        pDevObj->status.state &= ~(VP_DEV_IN_CAL);

        /*
         * Set a Device Init Complete Event if we're here because of a device
         * init function call
         */
        if (!(pDevObj->status.state & VP_DEV_INIT_CMP)) {
            pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
            pDevObj->status.state |= VP_DEV_INIT_CMP;

            maxChan = pDevObj->staticInfo.maxChannels;
            for (channelId = 0; channelId < maxChan; channelId++) {
                pLineCtx = pDevCtx->pLineCtx[channelId];

                if (pLineCtx != VP_NULL) {
                    Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
                }
            }
        } else {
            /* Restore lines to their previous states */
            for (channelId = 0;
                 channelId < pDevObj->staticInfo.maxChannels;
                 channelId++) {
                pLineCtx = pDevCtx->pLineCtx[channelId];
                if (pLineCtx != VP_NULL) {
                    pLineObj = pLineCtx->pLineObj;
                    Vp790SetLineStateInt(pLineCtx, pLineObj->lineState.previous);
                }
            }
        }

#if defined (VP790_INTERRUPT_LEVTRIG_MODE)
        VpSysEnableInt(deviceId);
#endif
        return TRUE;
    }
#if defined (VP790_INTERRUPT_LEVTRIG_MODE)
    VpSysEnableInt(deviceId);
#endif
    return FALSE;
} /* Vp790ContinueCalibrate() */


/**
 * Vp790ServiceFaultTimer()
 *  This function services an active Fault Timer.
 * Preconditions:
 *  This Function must be called from the ApiTick function, when a Fault timer
 *  is active on deviceId, chanID.
 * Postconditions:
 *  If a fault is active, then an event is triggered
 */
bool
Vp790ServiceFaultTimer(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint8 channelId = pLineObj->channelId;
    VpCslacLineCondType tempAcFault, tempDcFault, tempThermFault;

    uint8 activeMSB;
    bool retFlag = FALSE;
    uint8 sigRegArray[VP790_NO_UL_SIGREG_LEN];

    /* Read and don't unlock the signaling register */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_NO_UL_SIGREG_RD,
        VP790_NO_UL_SIGREG_LEN, sigRegArray);
    pLineObj->virtualDeviceReg.sigRegMSB = sigRegArray[0];
    pLineObj->virtualDeviceReg.sigRegLSB = sigRegArray[1];

    /* Get the non masked, active signals */
    activeMSB = (sigRegArray[0] & ~pLineObj->virtualDeviceReg.iMaskMSB);

    /* Is a fault condition active? */
    tempThermFault = (activeMSB & VP790_SIGREG_TEMPA) ? VP_CSLAC_THERM_FLT : VP_CSLAC_STATUS_INVALID;
    if (tempThermFault != (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_THERM_FLT)) {
        /* Trip a fault interrupt */
        pLineObj->lineState.condition &= ~(VP_CSLAC_THERM_FLT);
        pLineObj->lineState.condition |= tempThermFault;

        pLineObj->lineEvents.faults |= VP_LINE_EVID_THERM_FLT;
        if ((tempThermFault == VP_CSLAC_THERM_FLT)
         && (pDevObj->criticalFault.thermFltDiscEn == TRUE)) {
            Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
        }
        retFlag = TRUE;
    }

    tempDcFault = (activeMSB & VP790_SIGREG_DCFAULT) ? VP_CSLAC_DC_FLT : 0;
    if (tempDcFault != (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_DC_FLT)) {
        /* Trip a fault interrupt */
        pLineObj->lineState.condition &= ~(VP_CSLAC_DC_FLT);
        pLineObj->lineState.condition |= tempDcFault;

        pLineObj->lineEvents.faults |= VP_LINE_EVID_DC_FLT;
        if ((tempDcFault == VP_CSLAC_DC_FLT)
         && (pDevObj->criticalFault.dcFltDiscEn == TRUE)) {
            Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
        }
        retFlag = TRUE;
    }

    tempAcFault = (activeMSB & VP790_SIGREG_ACFAULT) ? VP_CSLAC_AC_FLT : VP_CSLAC_STATUS_INVALID;
    if (tempAcFault != (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_AC_FLT)) {
        /* Trip a fault interrupt */
        pLineObj->lineState.condition &= ~(VP_CSLAC_AC_FLT);
        pLineObj->lineState.condition |= tempAcFault;

        pLineObj->lineEvents.faults |= VP_LINE_EVID_AC_FLT;
        if ((tempAcFault == VP_CSLAC_AC_FLT)
         && (pDevObj->criticalFault.acFltDiscEn == TRUE)) {
            Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
        }
        retFlag = TRUE;
    }
    return retFlag;
} /* Vp790ServiceFaultTimer() */

/**
 * Vp790ServiceType1Int()
 *  This function services all active Type 1 (Device Status) Interrupts.
 * Preconditions:
 *  This Function must be called from the ApiTick function, when the device
 *  status register has changed. The active parameter should be the XOR between
 *  the current Device Status Register and the last time it was read.
 * Postconditions:
 *  This function returns TRUE, if there is an event to report.
 */
bool
Vp790ServiceType1Int(
    VpDevCtxType *pDevCtx)
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};

    bool retFlag = FALSE;
    uint8 data, active, globStatReg;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    /* Read the Device Device Status Register */
    VpMpiCmdWrapper(deviceId, ecVal[0], VP790_GD_STAT_RD, VP790_GD_STAT_LEN,
        &globStatReg);

    /* Determine the unmasked deltas from the previous call */
    active = (globStatReg ^ pDevObj->status.globStatReg);

    /* Do we return a new-event indication? */
    if(active != 0) {
        retFlag = TRUE;
    }
    /* Was the event a clock failure event ? */
    if(active & VP790_GD_STAT_CFAIL){
        /* Update the clock fail state */
        if(globStatReg & VP790_GD_STAT_CFAIL) {
            if (!(pDevObj->deviceEventsMask.faults & VP_DEV_EVID_CLK_FLT)) {
                pDevObj->deviceEvents.faults |= VP_DEV_EVID_CLK_FLT;
            }
        } else {
            pDevObj->deviceEvents.faults &= ~VP_DEV_EVID_CLK_FLT;
        }

        /* Mask the clockfail to prevent flood of interrupts */
        data = VP790_GD_STAT_CFAIL;
        VpMpiCmdWrapper(deviceId, ecVal[0], VP790_GD_MASK_WRT, VP790_GD_MASK_LEN,
            &data);

        /* Start timer to check clockfail again */
        if (pDevObj->devTimer[VP_DEV_TIMER_CLKFAIL] == 0) {
            pDevObj->devTimer[VP_DEV_TIMER_CLKFAIL] =
                MS_TO_TICKRATE(1000, pDevObj->devProfileData.tickRate);
            pDevObj->devTimer[VP_DEV_TIMER_CLKFAIL] |= VP_ACTIVATE_TIMER;
        }
    /* Was the event a battery failure event? */
    } else if (active
            & (VP790_GD_STAT_PINT | VP790_GD_STAT_LINT | VP790_GD_STAT_HINT)) {

        /* Update the battery failure state */
        if (globStatReg
         & (VP790_GD_STAT_PINT | VP790_GD_STAT_LINT | VP790_GD_STAT_HINT)) {
            if (!(pDevObj->deviceEventsMask.faults & VP_DEV_EVID_BAT_FLT)) {
                pDevObj->deviceEvents.faults |= VP_DEV_EVID_BAT_FLT;
            }
        } else {
            pDevObj->deviceEvents.faults &= ~VP_DEV_EVID_BAT_FLT;
        }
    }
    /* Update virtual device status register */
    pDevObj->status.globStatReg = globStatReg;
    return retFlag;
} /* Vp790ServiceType1Int() */

/**
 * Vp790ServiceType2Int()
 *  This function services all active Type 2 (Device Supervision) Interrupts.
 * Preconditions:
 *  This Function must be called from the ApiTick function, when the device
 *  supervision register has changed. The active parameter should be the
 *  XOR between the current Device Supervision Register and the last time it was
 *  read.
 * Postconditions:
 *  This function returns TRUE, if there is an event to report.
 */
bool
Vp790ServiceType2Int(
    VpLineCtxType *pLineCtx,
    uint8 intReg)            /* A copy of the current interrupt register read */
{
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    bool retFlag = FALSE;

    VpCslacLineCondType tempHookSt =
        (intReg & VP790_INTR_HOOK) ? VP_CSLAC_HOOK : VP_CSLAC_STATUS_INVALID;

    VpCslacLineCondType tempGnkSt =
        (intReg & VP790_INTR_GNK) ? VP_CSLAC_GKEY : VP_CSLAC_STATUS_INVALID;

    /*******************************************************
     *              SERVICE HOOK INTERRUPTS                *
     *******************************************************/
    if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK) != tempHookSt) {
        pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
        pLineObj->lineState.condition |= tempHookSt;

        if (
#ifdef VP_CSLAC_SEQ_EN
            (pLineObj->lineTimers.timers.timer[VP_LINE_CID_DEBOUNCE] & VP_ACTIVATE_TIMER) ||
#endif
            (pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] & VP_ACTIVATE_TIMER) ||
            (pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE] & VP_ACTIVATE_TIMER)) {
            /* Don't update the dial pulse hook state if debouncing */
        } else {
            pLineObj->dpStruct.hookSt = tempHookSt ? TRUE : FALSE;
        }
        retFlag |= Vp790ServiceHookInt(pLineCtx, tempHookSt);
    }

    /*******************************************************
     *           SERVICE GROUND-KEY INTERRUPTS             *
     *******************************************************/
    if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_GKEY) != tempGnkSt) {
        pLineObj->lineState.condition &= ~VP_CSLAC_GKEY;
        pLineObj->lineState.condition |= tempGnkSt;
        retFlag |= Vp790ServiceGkeyInt(pLineCtx, tempGnkSt);
    }

    return retFlag;
} /* Vp790ServiceType2Int() */

/**
 * Vp790ServiceType3Int()
 *  This function services all active Type 3 (Signalling) Interrupts.
 * Preconditions:
 *  This Function must be called from the ApiTick function, when a type III
 *  interrupt has occured.
 * Postconditions:
 *  This function returns TRUE, if there is an event to report.
 */
bool
Vp790ServiceType3Int(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    bool retFlag = FALSE;
    uint8 activeMSB, activeLSB;
    uint8 sigRegArray[2];

    /* Read and unlock the signaling register */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_UL_SIGREG_RD,
        VP790_UL_SIGREG_LEN, sigRegArray);

    /* Determine if anything unmasked changed from previous call */
    activeMSB = (sigRegArray[0] ^ pLineObj->virtualDeviceReg.sigRegMSB) &
                ~pLineObj->virtualDeviceReg.iMaskMSB;
    activeLSB = (sigRegArray[1] ^ pLineObj->virtualDeviceReg.sigRegLSB) &
                ~pLineObj->virtualDeviceReg.iMaskLSB;

    /* Was the event a fault? */
    if ((activeMSB & VP790_SIGREG_TEMPA)
     || (activeLSB & VP790_SIGREG_DCFAULT)
     || (activeLSB & VP790_SIGREG_ACFAULT)) {
        /* Is the event a new fault condition? */
        if ((sigRegArray[0] & VP790_SIGREG_TEMPA)
         || (sigRegArray[1] & VP790_SIGREG_DCFAULT)
         || (sigRegArray[1] & VP790_SIGREG_ACFAULT)) {

            /*
             * Evaluate the specific fault and set line to disconnect if it
             * should
             */
            if (sigRegArray[0] & VP790_SIGREG_TEMPA) {
                pLineObj->lineEvents.faults |= VP_LINE_EVID_THERM_FLT;
                pLineObj->lineState.condition |= VP_CSLAC_THERM_FLT;
                if (pDevObj->criticalFault.thermFltDiscEn == TRUE) {
                    Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
                }
            }

            if (sigRegArray[1] & VP790_SIGREG_DCFAULT) {
                pLineObj->lineEvents.faults |= VP_LINE_EVID_DC_FLT;
                pLineObj->lineState.condition |= VP_CSLAC_DC_FLT;

                if (pDevObj->criticalFault.dcFltDiscEn == TRUE) {
                    Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
                }
            }

            if (sigRegArray[1] & VP790_SIGREG_ACFAULT) {
                pLineObj->lineEvents.faults |= VP_LINE_EVID_AC_FLT;
                pLineObj->lineState.condition |= VP_CSLAC_AC_FLT;

                if (pDevObj->criticalFault.acFltDiscEn == TRUE) {
                    Vp790SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
                }
            }
        /* Otherwise the fault event indicates leaving fault condition */
        } else {
            /* If not a new Fault Event, than the fault has been removed */
            pLineObj->lineState.condition &=
                ~(VP_CSLAC_THERM_FLT | VP_CSLAC_AC_FLT | VP_CSLAC_DC_FLT);

            pLineObj->virtualDeviceReg.sigRegMSB = sigRegArray[0];
            pLineObj->virtualDeviceReg.sigRegLSB = sigRegArray[1];
            activeMSB = activeMSB & ~VP_LINE_EVID_THERM_FLT;
            activeLSB = activeLSB & ~VP_LINE_EVID_DC_FLT & ~VP_LINE_EVID_AC_FLT;
            pLineObj->lineEvents.faults = 0x0000;
        }
    }

    /* Indicate new signaling register event */
    if ((activeMSB) || (activeLSB)) {
        /* Update the virtual signaling register */
        pLineObj->virtualDeviceReg.sigRegMSB = sigRegArray[0];
        pLineObj->virtualDeviceReg.sigRegLSB = sigRegArray[1];

        retFlag = TRUE;
    } /* end if(temp16) */
    return retFlag;
} /* Vp790ServiceType3Int() */

/**
 * Vp790ServiceHookInt()
 *  This function services all Hook (On-Off) Interrupts.
 *
 * Preconditions:
 *  This Function must be called from (or indirectly from) the ApiTick function,
 * when a type II Hook interrupt has occured.
 *
 * Postconditions:
 *  This function returns TRUE if there is a hook event to report.
 */
bool
Vp790ServiceHookInt(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType hookSt)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    bool retFlag = FALSE;

    /* If debouncing, ignore hook. Otherwise process. */
    if (
#ifdef VP_CSLAC_SEQ_EN
        (pLineObj->lineTimers.timers.timer[VP_LINE_CID_DEBOUNCE] & VP_ACTIVATE_TIMER) ||
#endif
        (pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] & VP_ACTIVATE_TIMER))  {
    } else {
#ifdef VP_CSLAC_SEQ_EN
        /*
         * There was a sufficient hook activity to stop the active CID -- CID
         * debounce timer is expired.
         */
        if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
            VpCliStopCli(pLineCtx);
        }
#endif
        /* Now check the hook state */
        if (hookSt & VP_CSLAC_HOOK) {    /* Off-hook */
            /* Flag the event for Vp790GetEvent */
            if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
                pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_OFF;
                pLineObj->lineEvents.signaling &= ~(VP_LINE_EVID_HOOK_ON);
            }
            /*
             * Force ring-trip if in user ringing state. Note that this includes
             * time in silent interval.
             */
            if ((pLineObj->lineState.usrCurrent == VP_LINE_RINGING)
             || (pLineObj->lineState.usrCurrent == VP_LINE_RINGING_POLREV)) {
                /* If set for Auto Ringing Removal, then set to Ring Trip Exit State */
                if ((pLineObj->ringCtrl.ringTripExitSt == VP_LINE_RINGING)
                 || (pLineObj->ringCtrl.ringTripExitSt == VP_LINE_RINGING_POLREV)) {
                    Vp790SetLineState(pLineCtx, pLineObj->ringCtrl.ringTripExitSt);

                    /*
                     * Stop all currently running tones just in case tones were part of the
                     * ringing cadence. Not typical, but possible. If we don't do this and
                     * tones are part of the ringing cadence, then tones will remain enabled
                     * when a ring trip occurs.
                     */
                    Vp790SetLineTone(pLineCtx, VP_NULL, VP_NULL, VP_NULL);
                }
            }
            pLineObj->virtualDeviceReg.sigRegMSB |= VP790_SIGREG_HOOK;
        } else {    /* On-Hook */
            /* Flag the event for Vp790GetEvent */
            if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
                pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_ON;
                pLineObj->lineEvents.signaling &= ~(VP_LINE_EVID_HOOK_OFF);
            }
            /* Update Virtual signaling register */
            pLineObj->virtualDeviceReg.sigRegMSB &= ~(VP790_SIGREG_HOOK);
        }
        pLineObj->lineEventHandle = pDevObj->timeStamp;
        retFlag = TRUE;
    }

    return retFlag;
} /* Vp790ServiceHookInt() */

/**
 * Vp790ServiceGkeyInt()
 *  This function services all Ground-Key Interrupts.
 * Preconditions:
 *  This Function must be called from (or indirectly from) the ApiTick function,
 *  when a type II GroundKey interrupt has occured.
 * Postconditions:
 *  This function returns TRUE, if there is a GroundKey event to report.
 */
bool
Vp790ServiceGkeyInt(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType gkeySt
    )
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    bool retFlag = FALSE;
    uint8 masked =
        (pLineObj->virtualDeviceReg.iMaskMSB & VP790_SIGREG_GNK) >> 6;

    if ((!masked)
#ifdef VP_CSLAC_SEQ_EN
     && (pLineObj->lineTimers.timers.timer[VP_LINE_CID_DEBOUNCE] == 0)
#endif
    ) {
        if (gkeySt & VP_CSLAC_GKEY) {
            /* Update virtual signaling register - Gnkey active     */
            pLineObj->lineEvents.signaling |= VP_LINE_EVID_GKEY_DET;
            pLineObj->virtualDeviceReg.sigRegMSB |= VP790_SIGREG_GNK;
            /*
             * Force ring-trip if in user ringing state. Note that this includes
             * time in silent interval.
             */
            if ((pLineObj->lineState.usrCurrent == VP_LINE_RINGING) ||
                (pLineObj->lineState.usrCurrent == VP_LINE_RINGING_POLREV)) {
                /*
                 * If set for Auto Ringing Removal, then set to Ring Trip
                 * Exit State
                 */
                if ((pLineObj->ringCtrl.ringTripExitSt == VP_LINE_RINGING)
                 || (pLineObj->ringCtrl.ringTripExitSt == VP_LINE_RINGING_POLREV)) {
                    Vp790SetLineState(pLineCtx, pLineObj->ringCtrl.ringTripExitSt);
                }
            }
        } else {
            /* Update virtual signaling register - Gnkey not active */
            pLineObj->lineEvents.signaling |= VP_LINE_EVID_GKEY_REL;
            pLineObj->virtualDeviceReg.sigRegMSB &= ~VP790_SIGREG_GNK;
        }
        pLineObj->lineEventHandle = pDevObj->timeStamp;
        retFlag = TRUE;
    }
    return retFlag;
} /* Vp790ServiceGkeyInt() */

#endif






