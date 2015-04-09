/** \file vp580_query.c
 * vp580_query.c
 *
 *  This file contains the query functions used in the Vp580 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9215 $
 * $LastChangedDate: 2011-12-06 10:19:05 -0600 (Tue, 06 Dec 2011) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_580_SERIES)

/* Project Includes */
#include "vp_api_types.h"
#include "sys_service.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp580_api.h"
#include "vp580_api_int.h"

/* Private Functions */
static uint16 Vp580CheckLineEvent(uint16 event, uint16 eventMask,
    VpEventCategoryType eventCat, Vp580LineObjectType *pLineObj);
static uint16 Vp580CheckDevEvent(uint16 event, uint16 eventMask,
    VpEventCategoryType eventCat, Vp580DeviceObjectType *pDevObj);

/**
 * Vp580FindSoftwareInterrupts()
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
Vp580FindSoftwareInterrupts(
    VpDevCtxType *pDevCtx)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp580LineObjectType *pLineObj;
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
 * Vp580GetEvent()
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
Vp580GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)    /**< Pointer to the results event structure */
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp580LineObjectType *pLineObj;
    VpLineCtxType *pLineCtx;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 i, eventCatLoop;
    uint8 chan;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;

#define EVENT_ARRAY_SIZE 5

    uint16 eventArray[EVENT_ARRAY_SIZE];
    uint16 eventMaskArray[EVENT_ARRAY_SIZE];
    VpEventCategoryType eventCat[] = {
        VP_EVCAT_FAULT,
        VP_EVCAT_SIGNALING,
        VP_EVCAT_RESPONSE,
        VP_EVCAT_PROCESS,
        VP_EVCAT_FXO
    };

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

            case VP_EVCAT_FXO:
                eventArray[i] = pDevObj->deviceEvents.fxo;
                eventMaskArray[i] = pDevObj->deviceEventsMask.fxo;
                break;

            default:
                /* This can only occur if there's a bug in this code */
                break;
        }
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Look for active device events first */
    for (eventCatLoop = 0; eventCatLoop < EVENT_ARRAY_SIZE; eventCatLoop++) {
        pEvent->eventId = Vp580CheckDevEvent(eventArray[eventCatLoop],
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
                            Vp580LineObjectType *pLineObjLocal = pEvent->pLineCtx->pLineObj;
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

                    default:
                        break;
                }
            }
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return TRUE;
        }
    }

    /*
     * No device events, now look for Line events -- but make sure the line
     * context is valid before looking for a line object
     */
    for(chan = pDevObj->dynamicInfo.lastChan; chan < maxChan; chan++) {
        pLineCtx = pDevCtx->pLineCtx[chan];
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

                    case VP_EVCAT_FXO:
                        eventArray[i] = pLineObj->lineEvents.fxo;
                        eventMaskArray[i] = pLineObj->lineEventsMask.fxo;
                        break;

                    default:
                        /* This can only occur if there's a bug in this code */
                        break;

                }
            }

            /* Check this line events */
            for (eventCatLoop = 0;
                 eventCatLoop < EVENT_ARRAY_SIZE;
                 eventCatLoop++) {
                pEvent->eventId = Vp580CheckLineEvent(eventArray[eventCatLoop],
                    eventMaskArray[eventCatLoop], eventCat[eventCatLoop],
                    pLineObj);

                if (pEvent->eventId != 0x0000) {
                    pEvent->deviceId = deviceId;
                    pEvent->channelId = chan;
                    pEvent->pLineCtx = pDevCtx->pLineCtx[chan];
                    pEvent->pDevCtx = pDevCtx;
                    pEvent->eventCategory = eventCat[eventCatLoop];
                    pEvent->parmHandle = pLineObj->lineEventHandle;
                    pEvent->lineId = pLineObj->lineId;

                    switch(pEvent->eventCategory) {
                        case VP_EVCAT_RESPONSE:
                            switch(pEvent->eventId) {
                                case VP_LINE_EVID_LLCMD_RX_CMP:
                                case VP_LINE_EVID_GAIN_CMP:
                                    pEvent->hasResults = TRUE;
                                    break;
                                default:
                                    break;
                            }
                            break;

                        case VP_EVCAT_SIGNALING:
                            pEvent->eventData = pLineObj->signalingData;
                            break;

                        case VP_EVCAT_PROCESS:
                            pEvent->eventData = pLineObj->processData;
                            break;

                        case VP_EVCAT_FXO:
                            pEvent->eventData = pLineObj->fxoData;
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
} /* End Vp580GetEvent */

/**
 * Vp580CheckDevEvent()
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
Vp580CheckDevEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp580DeviceObjectType *pDevObj)
{
    uint8 i;
    uint16 mask;

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

                default:
                    break;
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
 * Vp580CheckLineEvent()
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
Vp580CheckLineEvent(
    uint16 event,
    uint16 eventMask,
    VpEventCategoryType eventCat,
    Vp580LineObjectType *pLineObj)
{
    uint8 i;
    uint16 mask;

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

                default:
                    break;
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
 * Vp580GetOption()
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
Vp580GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle)
{
    VpDevCtxType *pDevCtxLocal;
    Vp580LineObjectType *pLineObj;
    Vp580DeviceObjectType *pDevObj;

    VpStatusType status = VP_STATUS_SUCCESS;
    VpGetResultsOptionsDataType *pOptionData;

    uint8 channelId, txSlot, rxSlot, maxChan;
    VpDeviceIdType deviceId;
    uint8 tempLoopBack[VP580_LOOPBACK_LEN];
    uint8 codecReg;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 ioDirection[4];   /* One element for each channel */

    if (pLineCtx != VP_NULL) {
        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        deviceId = pDevObj->deviceId;
        pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
        pOptionData = &(pDevObj->getResultsOption.optionData);

        if (pDevObj->deviceEvents.response & VP580_READ_RESPONSE_MASK) {
            return VP_STATUS_DEVICE_BUSY;
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
            case VP_OPTION_ID_PULSE_MODE:
                pOptionData->pulseModeOption = pLineObj->pulseMode;
                break;

            case VP_OPTION_ID_TIMESLOT:
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_TX_TS_RD,
                    VP580_TX_TS_LEN, &txSlot);

                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_RX_TS_RD,
                    VP580_RX_TS_LEN, &rxSlot);

                pOptionData->timeSlotOption.tx = (txSlot & VP580_TX_TS_MASK);

                pOptionData->timeSlotOption.rx = (rxSlot & VP580_RX_TS_MASK);
                break;

            case VP_OPTION_ID_CODEC:
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_CODEC_REG_RD,
                    VP580_CODEC_REG_LEN, &codecReg);

                switch(codecReg & VP580_CODEC_COMPRESSION_MASK) {
                    case (VP580_LINEAR_CODEC | VP580_ALAW_CODEC):
                    case (VP580_LINEAR_CODEC | VP580_ULAW_CODEC):
                        pOptionData->codecOption = VP_OPTION_LINEAR;
                        break;

                    case VP580_ALAW_CODEC:
                        pOptionData->codecOption = VP_OPTION_ALAW;
                        break;

                    default:
                        pOptionData->codecOption = VP_OPTION_MLAW;
                        break;
                }
                break;

            case VP_OPTION_ID_PCM_HWY:
                pOptionData->pcmHwyOption = VP_OPTION_HWY_A;
                break;

            case VP_OPTION_ID_LOOPBACK:
                /* Timeslot loopback via loopback register */
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_LOOPBACK_RD,
                    VP580_LOOPBACK_LEN, tempLoopBack);

                if ((tempLoopBack[0] & VP580_INTERFACE_LOOPBACK_EN) ==
                     VP580_INTERFACE_LOOPBACK_EN) {
                    pOptionData->loopBackOption = VP_OPTION_LB_TIMESLOT;
                } else if ((tempLoopBack[0] & VP580_INTERFACE_LOOPBACK_EN) ==
                     VP580_FULL_DIGL_LOOPBACK_EN) {
                    pOptionData->loopBackOption = VP_OPTION_LB_DIGITAL;
                } else {
                    pOptionData->loopBackOption = VP_OPTION_LB_OFF;
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

            case VP_OPTION_ID_ZERO_CROSS:
                pOptionData->zeroCross = pLineObj->ringCtrl.zeroCross;
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

        if (pDevObj->deviceEvents.response & VP580_READ_RESPONSE_MASK) {
            return VP_STATUS_DEVICE_BUSY;
        }

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

        switch (option) {
            case VP_DEVICE_OPTION_ID_PULSE:
                pOptionData->pulseTypeOption = pDevObj->pulseSpecs;
                break;

            case VP_OPTION_ID_EVENT_MASK:
                pOptionData->eventMaskOption = pDevObj->deviceEventsMask;
                break;

            case VP_DEVICE_OPTION_ID_DEVICE_IO:
                maxChan = pDevObj->staticInfo.maxChannels;

                /* Default to input, or in those bits set to output */
                pOptionData->deviceIo.directionPins_31_0 = 0;

                /* Get the current device IO control information */
                for (channelId = 0; channelId < maxChan; channelId++) {
                    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_RD,
                        VP580_IODIR_REG_LEN, &ioDirection[channelId]);
                    ioDirection[channelId] &= VP580_IODIR_CTRL_BITS;
                    pOptionData->deviceIo.directionPins_31_0 |=
                        ((uint32)ioDirection[channelId] << (8 * channelId));
                }

                /* Output type is always driven */
                pOptionData->deviceIo.outputTypePins_31_0 = 0x00000000;
                pOptionData->deviceIo.outputTypePins_63_32 = 0x00000000;

                /* These direction bits are not used -- default to input */
                pOptionData->deviceIo.directionPins_63_32 = 0x00000000;
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
 * Vp580GetDeviceStatus()
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
Vp580GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
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
 * Vp580FlushEvents()
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
Vp580FlushEvents(
    VpDevCtxType *pDevCtx)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VpLineCtxType *pLineCtx;
    Vp580LineObjectType *pLineObj;
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
 * Vp580GetResults()
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
Vp580GetResults(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
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

                        case VP_DEVICE_OPTION_ID_DEVICE_IO:
                            *(VpOptionDeviceIoType *)pResults =
                                pOptionData->deviceIo;
                            break;

                        case VP_OPTION_ID_RING_CNTRL:
                            *(VpOptionRingControlType *)pResults =
                                pOptionData->ringControlOption;
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
                            *((VpOptionCodecType *)pResults) =
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

                        default:
                            status = VP_STATUS_INVALID_ARG;
                            break;
                    }
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
 * Vp580ServiceTimers()
 *  This function services active API timers on all channels of deviceId.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  All Active Timers have been serviced.
 */
bool
Vp580ServiceTimers(
    VpDevCtxType *pDevCtx)
{
    VpLineCtxType *pLineCtx;
    Vp580LineObjectType *pLineObj;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 tempTimer;

    uint8 lineTimerType;

    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    bool retFlag = FALSE;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    /* Iterate through the channels until all timers are serviced */
    for(channelId=0; channelId < maxChan; channelId++ ) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {

            pLineObj = pLineCtx->pLineObj;
            if (!(pLineObj->status & VP580_IS_FXO)) {
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
                            switch (lineTimerType) {
                                case VP_LINE_RING_EXIT_PROCESS:
                                    VpMpiCmdWrapper(deviceId, VP580_EC_CH1,
                                        VP580_UL_SIGREG_RD, VP580_UL_SIGREG_LEN,
                                        pDevObj->intReg);
                                    Vp580ServiceInterrupts(pDevCtx);
                                    break;

                                case VP_LINE_GPIO_CLKOUT_TIMER:
                                    Vp580RingSigGen(pLineCtx, VP580_RING_SIG_GEN_UPDATE, VP_NULL);
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
            }
        } /* Line Context Check */
    } /* Loop through channels until no more tests */

    return retFlag;
} /* Vp580ServiceTimers() */

#endif




