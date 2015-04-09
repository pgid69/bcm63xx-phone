/** \file vp_api_control.c
 * vp_api_control.c
 *
 *  This file contains the implementation of top level VoicePath API-II
 * Control procedures.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 6827 $
 * $LastChangedDate: 2010-04-01 17:51:38 -0500 (Thu, 01 Apr 2010) $
 */

/* INCLUDES */
#include "vp_api.h"     /* Typedefs and function prototypes for API */

#include "vp_hal.h"
#include "vp_api_int.h" /* Device specific typedefs and function prototypes */
#include "sys_service.h"

#if defined (VP_CC_880_SERIES)
#include "vp880_api_int.h"
#endif

/******************************************************************************
 *                        CONTROL FUNCTIONS                                   *
 ******************************************************************************/
#ifdef VP_CC_SET_LINE_STATE
/**
 * VpSetLineState()
 *  This function sets a given line to indicated state. See VP-API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The indicated line is set to indicated line state.
 */
VpStatusType
VpSetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SetLineState");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SetLineState, (pLineCtx, state));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SetLineState", status);
    return status;
} /* VpSetLineState() */
#endif /* VP_CC_SET_LINE_STATE */

#ifdef VP_CC_SET_LINE_TONE
/**
 * VpSetLineTone()
 *  This function is used to set a tone on a given line. See VP-API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Starts/Stops tone generation for a given line.
 */
VpStatusType
VpSetLineTone(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pCadProfile,
    VpDtmfToneGenType *pDtmfControl)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SetLineTone");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SetLineTone, (pLineCtx, pToneProfile, pCadProfile, pDtmfControl));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SetLineTone", status);
    return status;
} /* VpSetLineTone() */
#endif /* VP_CC_SET_LINE_TONE */

#ifdef VP_CC_SET_RELAY_STATE
/**
 * VpSetRelayState()
 *  This function controls the state of VTD controlled relays. See VP-API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The indicated relay state is set for the given line.
 */
VpStatusType
VpSetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SetRelayState");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SetRelayState, (pLineCtx, rState));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SetRelayState", status);
    return status;
} /* VpSetRelayState() */
#endif /* VP_CC_SET_RELAY_STATE */

#ifdef VP_CC_SET_CAL_RELAY_STATE
/**
 * VpSetCalRelayState()
 *  This function controls the calibration relays. See VP-API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The indicated relay state is set for the calibration bus.
 */
VpStatusType
VpSetCalRelayState(
    VpDevCtxType *pDevCtx,
    VpCalRelayControlType rState)
{
    VpStatusType status;
    VP_API_ENTER(VpDevCtxType, pDevCtx, "SetCalRelayState");

    /* Basic argument checking */
    if (pDevCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        status = VP_CALL_DEV_FUNC(SetCalRelayState, (pDevCtx, rState));
    }

    VP_API_EXIT(VpDevCtxType, pDevCtx, "SetCalRelayState", status);
    return status;
} /* VpCalSetRelayState() */
#endif /* VP_CC_SET_CAL_RELAY_STATE */

#ifdef VP_CC_SET_REL_GAIN
/**
 * VpSetRelGain()
 *  This function adjusts the transmit and receive path relative gains. See
 * VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The requested relative gains will be applied.
 */
VpStatusType
VpSetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SetRelGain");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SetRelGain, (pLineCtx, txLevel, rxLevel, handle));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SetRelGain", status);
    return status;
} /* VpSetRelGain() */
#endif /* VP_CC_SET_REL_GAIN */

#ifdef VP_CC_SEND_SIGNAL
/**
 * VpSendSignal()
 *  This function is used to send a signal on a line. The signal type is
 * specified by the type parameter and the parameters associated with the signal
 * type are specified by the structure pointer passed.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Applies a signal to the line.
 */
VpStatusType
VpSendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType signalType,
    void *pSignalData)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SendSignal");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SendSignal, (pLineCtx, signalType, pSignalData));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SendSignal", status);
    return status;
} /* VpSendSignal() */
#endif /* VP_CC_SEND_SIGNAL */

#ifdef VP_CC_SEND_CID
/**
 * VpSendCid()
 *  This function may be used to send Caller ID information on-demand. See
 * VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 * Caller ID information is transmitted on the line.
 */
VpStatusType
VpSendCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    VpProfilePtrType pCidProfile,
    uint8p pCidData)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SendCid");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SendCid, (pLineCtx, length, pCidProfile, pCidData));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SendCid", status);
    return status;
} /* VpSendCid() */
#endif /* VP_CC_SEND_CID */

#ifdef VP_CC_CONTINUE_CID
/**
 * VpContinueCid()
 *  This function is called to provide more caller ID data (in response to
 * Caller ID data event from the VP-API). See VP-API-II  documentation
 * for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Continues to transmit Caller ID information on the line.
 */
VpStatusType
VpContinueCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "ContinueCid");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(ContinueCid, (pLineCtx, length, pCidData));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "ContinueCid", status);
    return status;
} /* VpContinueCid() */
#endif /* VP_CC_CONTINUE_CID */

#ifdef VP_CC_START_METER
/**
 * VpStartMeter()
 *  This function starts(can also abort) metering pulses on the line. See
 * VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Metering pulses are transmitted on the line.
 */
VpStatusType
VpStartMeter(
    VpLineCtxType *pLineCtx,
    uint16 onTime,
    uint16 offTime,
    uint16 numMeters)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "StartMeter");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(StartMeter, (pLineCtx, onTime, offTime, numMeters));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "StartMeter", status);
    return status;
} /* VpStartMeter() */
#endif /* VP_CC_START_METER */

#ifdef VP_CC_GEN_TIMER_CTRL
/**
 * This function provides generic timer functionality.  Please see VP-API
 * documentation for more information.
 *
 * Preconditions:
 * This function assumes the passed line context is created and initialized.
 *
 * Postconditions:
 * Starts or cancels a timer.
 */
VpStatusType
VpGenTimerCtrl(
    VpLineCtxType *pLineCtx,
    VpGenTimerCtrlType timerCtrl,
    uint32 duration,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "GenTimerCtrl");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(GenTimerCtrl, (pLineCtx, timerCtrl, duration, handle));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "GenTimerCtrl", status);
    return status;
} /* VpGenTimerCtrl() */
#endif /* VP_CC_GEN_TIMER_CTRL */

#ifdef VP_CC_START_METER_32Q
/**
 * VpStartMeter32Q()
 *  This function starts(can also abort) metering pulses on the line. See
 * VP-API-II documentation for more information about this function.  This
 * version of the function supports 32-bit minDelay, onTime, and offTime
 * parameters at 1ms increments.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Metering pulses are transmitted on the line.
 */
VpStatusType
VpStartMeter32Q(
    VpLineCtxType *pLineCtx,
    uint32 minDelay,
    uint32 onTime,
    uint32 offTime,
    uint16 numMeters,
    uint16 eventRate)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "StartMeter32Q");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(StartMeter32Q, (pLineCtx, minDelay, onTime, offTime, numMeters, eventRate));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "StartMeter32Q", status);
    return status;
} /* VpStartMeter32Q() */
#endif /* VP_CC_START_METER_32Q*/

#ifdef VP_CC_READ_CAL_FLAG
/* undocumented function used only by the Test Library: */
bool
VpReadCalFlag(
    VpLineCtxType *pLineCtx)
{
    bool retval;

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        retval = FALSE;
    } else switch (pLineCtx->pDevCtx->deviceType) {

#ifdef VP_CC_VCP2_SERIES
        case VP_DEV_VCP2_SERIES:
            retval = Vcp2ReadCalFlag(pLineCtx);
            break;
#endif

#ifdef VP_CC_MELT_SERIES
        case VP_DEV_MELT_SERIES:
            retval = MeltReadCalFlag(pLineCtx);
            break;
#endif

#if defined (VP_CC_KWRAP)
        case VP_DEV_KWRAP:
            retval = KWrapReadCalFlag(pLineCtx);
            break;
#endif

        default:
            retval = FALSE;
    }

    return retval;
}
#endif /* VP_CC_READ_CAL_FLAG */

#ifdef VP_CC_SET_CAL_FLAG
/* undocumented function used only by the Test Library: */
void
VpSetCalFlag(
    VpLineCtxType *pLineCtx,
    bool value)
{
    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        return;
    } else switch (pLineCtx->pDevCtx->deviceType) {

#ifdef VP_CC_VCP2_SERIES
        case VP_DEV_VCP2_SERIES:
            Vcp2SetCalFlag(pLineCtx, value);
            break;
#endif

#ifdef VP_CC_MELT_SERIES
        case VP_DEV_MELT_SERIES:
            MeltSetCalFlag(pLineCtx, value);
            break;
#endif

#if defined (VP_CC_KWRAP)
        case VP_DEV_KWRAP:
            KWrapSetCalFlag(pLineCtx, value);
            break;
#endif
        default:
            break;
    }

    return;
}
#endif /* VP_CC_SET_CAL_FLAG */

#ifdef VP_CC_ASSOC_DSL_LINE
/**
 * VpAssocDslLine()
 *  This function associates a DSL line to a MeLT test resource.  See
 * VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The relay matrix will be controlled to allow the test resource to have
 *  access to Tip/Ring of the requested DSL line.
 */
VpStatusType
VpAssocDslLine(
    VpLineCtxType *pLineCtx,
    bool connect,
    uint8 line)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "AssocDslLine");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(AssocDslLine, (pLineCtx, connect, line));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "AssocDslLine", status);
    return status;
} /* VpAssocDslLine() */
#endif /* VP_CC_ASSOC_DSL_LINE */

#ifdef VP_CC_SET_SEAL_CUR
/**
 * VpSetSealCur()
 *  This function sets the sealing current control for a MeLT test resource.
 *
 * Preconditions:
 *  Device context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The sealing current will be applied to the lines designated by pSealArray.
 */
VpStatusType
VpSetSealCur(
    VpDevCtxType *pDevCtx,
    uint16 sealApplyTime,
    uint16 sealCycleTime,
    uint16 maxCurrent,
    uint16 minCurrent,
    uint16 *pSealArray,
    uint16 batteryOffset
)
{
    VpStatusType status;
    VP_API_ENTER(VpDevCtxType, pDevCtx, "SetSealCurrent");

    /* Basic argument checking */
    if (pDevCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        status = VP_CALL_DEV_FUNC(SetSealCur, (pDevCtx, sealApplyTime, sealCycleTime, maxCurrent, minCurrent,
            pSealArray, batteryOffset));
    }

    VP_API_EXIT(VpDevCtxType, pDevCtx, "SetSealCurrent", status);
    return status;
} /* VpSetSealCur() */
#endif /* VP_CC_SET_SEAL_CUR */

#ifdef VP_CC_SET_OPTION
/**
 * VpSetOption()
 *  This function is used to set various options that are supported by VP-API.
 * For a detailed description of how this function can be used to set device,
 * line specific, device specific options and to various types of options
 * please see VP-API user guide.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  This function sets the requested option.
 */
VpStatusType
VpSetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtxParam,
    VpOptionIdType optionId,
    void *value)
{
    VpDevCtxType *pDevCtx;
    VpStatusType status = VP_STATUS_SUCCESS;
    bool singleLine = (pLineCtx != VP_NULL);

    /* Basic argument checking */
    if (singleLine) {
        VP_API_FUNC(VpLineCtxType, pLineCtx, ("VpSetOption(%s):", VpGetString_VpOptionIdType(optionId)));
        if (pDevCtxParam != VP_NULL) {
            status = VP_STATUS_INVALID_ARG;
        }
        pDevCtx = pLineCtx->pDevCtx;
    } else {
        pDevCtx = pDevCtxParam;
        if (pDevCtx == VP_NULL) {

#ifdef VP_DEBUG
            /* Special case: We need a way of modifying this global variable. */
            if (optionId == VP_OPTION_ID_DEBUG_SELECT) {
                VP_API_FUNC(None, VP_NULL, ("VpSetOption(%s):", VpGetString_VpOptionIdType(VP_OPTION_ID_DEBUG_SELECT)));
#ifdef VP_CC_KWRAP
                KWrapSetOption(VP_NULL, VP_NULL, VP_OPTION_ID_DEBUG_SELECT, value);
#endif
                vpDebugSelectMask = *(uint32 *)value;
                VP_API_EXIT(None, VP_NULL, "SetOption", VP_STATUS_SUCCESS);
                return VP_STATUS_SUCCESS;
            }
#endif

            status = VP_STATUS_INVALID_ARG;
        }
        VP_API_FUNC(VpDevCtxType, pDevCtxParam, ("VpSetOption(%s):", VpGetString_VpOptionIdType(optionId)));
    }

    if (status == VP_STATUS_SUCCESS) {
        status = VP_CALL_DEV_FUNC(SetOption, (pLineCtx, pDevCtxParam, optionId, value));
    }

#if (VP_CC_DEBUG_SELECT & VP_DBG_API_FUNC)
    if (singleLine) {
        VP_API_EXIT(VpLineCtxType, pLineCtx, "SetOption", status);
    } else {
        VP_API_EXIT(VpDevCtxType, pDevCtx, "SetOption", status);
    }
#endif

    return status;
} /* VpSetOption() */
#endif /* VP_CC_SET_OPTION */

#ifdef VP_CC_DEVICE_IO_ACCESS
/**
 * VpDeviceIoAccess()
 *  This function is used to access device IO pins of the VTD. See VP-API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Reads/Writes from device IO pins.
 */
VpStatusType
VpDeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData)
{
    VpStatusType status;
    VP_API_ENTER(VpDevCtxType, pDevCtx, "DeviceIoAccess");

    /* Basic argument checking */
    if (pDevCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        status = VP_CALL_DEV_FUNC(DeviceIoAccess, (pDevCtx, pDeviceIoData));
    }

    VP_API_EXIT(VpDevCtxType, pDevCtx, "DeviceIoAccess", status);
    return status;
} /* VpDeviceIoAccess() */
#endif /* VP_CC_DEVICE_IO_ACCESS */

#ifdef VP_CC_DEVICE_IO_ACCESS_EXT
/**
 * VpDeviceIoAccessExt()
 *  This function is used to access device IO pins of the VTD. See VP-API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Reads/Writes from device IO pins.
 */
VpStatusType
VpDeviceIoAccessExt(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessExtType *pDeviceIoAccess)
{
    VpStatusType status;
    VP_API_ENTER(VpDevCtxType, pDevCtx, "DeviceIoAccessExt");

    /* Basic argument checking */
    if ((pDevCtx == VP_NULL) || (pDeviceIoAccess == VP_NULL)) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        status = VP_CALL_DEV_FUNC(DeviceIoAccessExt, (pDevCtx, pDeviceIoAccess));
    }

    VP_API_EXIT(VpDevCtxType, pDevCtx, "DeviceIoAccessExt", status);
    return status;
} /* VpDeviceIoAccessExt() */
#endif /* VP_CC_DEVICE_IO_ACCESS_EXT */

#ifdef VP_CC_LINE_IO_ACCESS
/**
 * VpLineIoAccess()
 *  This function is used to access the IO pins of the VTD associated with a
 * particular line. See VP-API-II documentation for more information about
 * this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Reads/Writes from line IO pins.
 */
VpStatusType
VpLineIoAccess(
    VpLineCtxType *pLineCtx,
    VpLineIoAccessType *pLineIoAccess,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "LineIoAccess");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(LineIoAccess, (pLineCtx, pLineIoAccess, handle));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "LineIoAccess", status);
    return status;
} /* VpLineIoAccess() */
#endif /* VP_CC_LINE_IO_ACCESS */

#ifdef VP_CC_VIRTUAL_ISR
/**
 * VpVirtualISR()
 *  This function should be called from the interrupt context for CSLAC devices
 * upon a CSLAC device interrupt. See VP-API-II documentation for more
 * information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Updates API internal variables to indicate an interrupt has occured.
 */
VpStatusType
VpVirtualISR(
    VpDevCtxType *pDevCtx)
{
    VpVirtualISRFuncPtrType VirtualISR;

    if (pDevCtx == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }

    VirtualISR = pDevCtx->funPtrsToApiFuncs.VirtualISR;

    if (VirtualISR == VP_NULL) {
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    } else {
        return VirtualISR(pDevCtx);
    }
} /* VpVirtualISR() */
#endif /* VP_CC_VIRTUAL_ISR */

#ifdef VP_CC_API_TICK
/**
 * VpApiTick()
 *  This function should be called at regular intervals of time for CSLAC
 * devices. This function call is used to update timing related aspects of
 * VP-API. See VP-API-II documentation for more information.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized.
 *
 * Postconditions:
 *  Updates necessary internal variables.
 */
VpStatusType
VpApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus)
{
    VpApiTickFuncPtrType ApiTick;

    if (pDevCtx == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }

    ApiTick = pDevCtx->funPtrsToApiFuncs.ApiTick;

    if (ApiTick == VP_NULL) {
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    } else {
        return ApiTick(pDevCtx, pEventStatus);
    }
} /* VpApiTick() */
#endif /* VP_CC_API_TICK */

#ifdef VP_CC_SHUTDOWN_DEVICE
/**
 * VpShutdownDevice()
 *  This function will shut down all device activity.
 *
 * Preconditions:
 *  Device context should be created and initialized.
 *
 * Postconditions:
 *  Device will be shut down, requiring reinitialization before using again.
 */
VpStatusType
VpShutdownDevice(
    VpDevCtxType *pDevCtx)
{
    VpShutdownDeviceFuncPtrType ShutdownDevice;

    if (pDevCtx == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }

    ShutdownDevice = pDevCtx->funPtrsToApiFuncs.ShutdownDevice;

    if (ShutdownDevice == VP_NULL) {
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    } else {
        return ShutdownDevice(pDevCtx);
    }
} /* VpShutdownDevice() */
#endif /* VP_CC_SHUTDOWN_DEVICE */

#ifdef VP_CC_LOW_LEVEL_CMD
/**
 * VpLowLevelCmd()
 *  This function provides a by-pass mechanism for the VP-API. THE USE
 * OF THIS FUNCTION BY THE APPLICATION CODE IS STRONGLY DISCOURAGED. THIS
 * FUNCTION CALL BREAKS THE SYNCHRONIZATION BETWEEN THE VP-API AND THE
 * VTD. See VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Performs the indicated low-level command access.
 */
VpStatusType
VpLowLevelCmd(
    VpLineCtxType *pLineCtx,
    uint8 *pCmdData,
    uint8 len,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "LowLevelCmd");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(LowLevelCmd, (pLineCtx, pCmdData, len, handle));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "LowLevelCmd", status);
    return status;
} /* VpLowLevelCmd() */
#endif /* VP_CC_LOW_LEVEL_CMD */

#ifdef VP_CC_LOW_LEVEL_CMD_16
/**
 * VpLowLevelCmd16()
 *  This function provides a by-pass mechanism for the VP-API. THE USE
 * OF THIS FUNCTION BY THE APPLICATION CODE IS STRONGLY DISCOURAGED. THIS
 * FUNCTION CALL BREAKS THE SYNCHRONIZATION BETWEEN THE VP-API AND THE
 * VTD. See VP-API-II documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Performs the indicated low-level command access.
 */
VpStatusType
VpLowLevelCmd16(
    VpLineCtxType *pLineCtx,
    VpLowLevelCmdType cmdType,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "LowLevelCmd16");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(LowLevelCmd16, (pLineCtx, cmdType, writeWords, numWriteWords, numReadWords, handle));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "LowLevelCmd16", status);
    return status;
} /* VpLowLevelCmd16() */
#endif /* VP_CC_LOW_LEVEL_CMD_16 */

