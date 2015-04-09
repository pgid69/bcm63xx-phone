/** \file vp_debug.c
 * vp_debug.c
 *
 *  This file contains the implementation of VP_DEBUG MACRO.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11616 $
 * $LastChangedDate: 2014-10-21 15:19:04 -0500 (Tue, 21 Oct 2014) $
 */

#include "vp_api_cfg.h"
#include "vp_api.h"
#include "vp_debug.h"

#if defined (VP_CC_VCP2_SERIES) || defined (VP_CC_MELT_SERIES)
#include "hbi_common.h"
#endif

uint32 vpDebugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;

#ifdef VP_CC_REGISTER_DUMP
/**
 * VpRegisterDump()
 *  This function is used to dump the content of all device registers.
 *
 * Preconditions:
 *  Device context should be created.
 *
 * Postconditions:
 *  Debug output of "all" registers.
 */
VpStatusType
VpRegisterDump(
    VpDevCtxType *pDevCtx)
{
    VpStatusType status;
    VP_API_ENTER(VpDevCtxType, pDevCtx, "RegisterDump");

    /* Basic argument checking */
    if (pDevCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        status = VP_CALL_DEV_FUNC(RegisterDump, (pDevCtx));
    }

    VP_API_EXIT(VpDevCtxType, pDevCtx, "RegisterDump", status);
    return status;
} /* VpRegisterDump() */
#endif /* VP_CC_REGISTER_DUMP */

#ifdef VP_CC_OBJECT_DUMP
/**
 * VpObjectDump()
 *  This function is used to dump the contents of a Device Context and Device
 * Object (if pDevCtx != VP_NULL) and/or the contents of a Line Context and Line
 * Object (if pLineCtx != VP_NULL).  Either pDevCtx or pLineCtx must be VP_NULL,
 * but not both.
 *
 * Preconditions:
 *  VP_CC_DEBUG_SELECT must include VP_DBG_ERROR for this function to be
 * compiled in.
 *
 * Postconditions:
 *  Debug output of "all" registers.
 */
VpStatusType
VpObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx)
{
    VpStatusType status = VP_STATUS_FUNC_NOT_SUPPORTED;

#if (VP_CC_DEBUG_SELECT & (VP_DBG_API_FUNC | VP_DBG_INFO))
    bool singleLine = (pLineCtx != VP_NULL);
#endif

#if (VP_CC_DEBUG_SELECT & VP_DBG_API_FUNC)
    if (singleLine) {
        VP_API_ENTER(VpLineCtxType, pLineCtx, "ObjectDump");
    } else {
        VP_API_ENTER(VpDevCtxType, pDevCtx, "ObjectDump");
    }
#endif

    /* Basic argument checking */
    if (
        ((pDevCtx == VP_NULL) && (pLineCtx == VP_NULL)) ||
        ((pDevCtx != VP_NULL) && (pLineCtx != VP_NULL))
    ) {
        status = VP_STATUS_INVALID_ARG;
    } else {

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
        VpSysDebugPrintf("\nVP-API version = %d.%d.%d.%d\n",
            VP_API_VERSION_MAJOR_NUM, VP_API_VERSION_MINOR_NUM,
            VP_API_VERSION_MINI_NUM, VP_API_VERSION_PATCH_NUM);

        VpSysDebugPrintf("\npDevCtx = %lu\n", (unsigned long)pDevCtx);
        VpSysDebugPrintf("pLineCtx = %lu\n", (unsigned long)pLineCtx);

        /* Contexts are device-independent, so code for displaying them is
           included here. */
        if (singleLine) {
            VpSysDebugPrintf("\nDumping Line Context:\n");
            VpSysDebugPrintf("\tpLineCtx->pDevCtx = %lu\n", (unsigned long)(pLineCtx->pDevCtx));
            VpSysDebugPrintf("\tpLineCtx->pLineObj = %lu\n", (unsigned long)(pLineCtx->pLineObj));
            pDevCtx = pLineCtx->pDevCtx;
            status = VP_CALL_DEV_FUNC(ObjectDump, (pLineCtx, VP_NULL));
        } else {
            int line;
            VpSysDebugPrintf("\nDumping Device Context:\n");
            VpSysDebugPrintf("\tpDevCtx->deviceType = %d\n", (int)(pDevCtx->deviceType));
            VpSysDebugPrintf("\tpDevCtx->pDevObj = %lu\n", (unsigned long)(pDevCtx->pDevObj));
            VpSysDebugPrintf("\tpDevCtx->pLineCtx[%d] = {", VP_MAX_LINES_PER_DEVICE);
            for (line = 0; line < VP_MAX_LINES_PER_DEVICE; line++) {
                VpSysDebugPrintf(" %lu", (unsigned long)(pDevCtx->pLineCtx[line]));
            }
            VpSysDebugPrintf(" }\n");
            status = VP_CALL_DEV_FUNC(ObjectDump, (VP_NULL, pDevCtx));
        }
#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_INFO) */

    }

#if (VP_CC_DEBUG_SELECT & VP_DBG_API_FUNC)
    if (singleLine) {
        VP_API_EXIT(VpLineCtxType, pLineCtx, "ObjectDump", status);
    } else {
        VP_API_EXIT(VpDevCtxType, pDevCtx, "ObjectDump", status);
    }
#endif

    return status;
} /* VpObjectDump() */
#endif /* VP_CC_OBJECT_DUMP */

#ifdef VP_DEBUG

static bool
DeviceDebugEnabled(
    uint32 debugSelectMask,
    char *msgTypeStr,
    VpDeviceIdType deviceId)
{
    if (!debugSelectMask)
        return FALSE;

    /* Display the message type and device ID. */
    VpSysDebugPrintf(msgTypeStr);
    VP_PRINT_DEVICE_ID(deviceId);
    VpSysDebugPrintf(": ");
    return TRUE;
}

static bool
LineDebugEnabled(
    uint32 debugSelectMask,
    char *msgTypeStr,
    VpLineIdType lineId)
{
    if (!debugSelectMask)
        return FALSE;

    /* Display the message type and line ID. */
    VpSysDebugPrintf(msgTypeStr);
    VP_PRINT_LINE_ID((lineId));
    VpSysDebugPrintf(": ");
    return TRUE;
}

/* This function is a "fallback" for the case where an object was expected, but
   VP_NULL was passed. */
static bool
NULLDebugEnabled(
    uint32 msgType,
    char *errorMsg,
    char *msgTypeStr)
{
    if (msgType & vpDebugSelectMask) {
        VpSysDebugPrintf(color_fg(red) "%s", errorMsg);
    }
    return VpDebugEnabled_None(msgType, msgTypeStr, VP_NULL);
}

#if defined (VP_CC_VCP2_SERIES)
bool
VpDebugEnabled_VpVcp2DeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    VpVcp2DeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->debugSelectMask;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

bool
VpDebugEnabled_VpVcp2LineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    VpVcp2LineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->debugSelectMask;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_VCP2_SERIES*/

#if defined (VP_CC_MELT_SERIES)
bool
VpDebugEnabled_VpMeltDeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    VpMeltDeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->debugSelectMask;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

bool
VpDebugEnabled_VpMeltLineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    VpMeltLineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->debugSelectMask;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_MELT_SERIES */

#ifdef VP_CC_890_SERIES
bool
VpDebugEnabled_Vp890DeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp890DeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->debugSelectMask;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

bool
VpDebugEnabled_Vp890LineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp890LineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->debugSelectMask;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_890_SERIES */

#ifdef VP_CC_880_SERIES
bool
VpDebugEnabled_Vp880DeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp880DeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->debugSelectMask;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

bool
VpDebugEnabled_Vp880LineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp880LineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->debugSelectMask;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_880_SERIES */

#ifdef VP_CC_886_SERIES
bool
VpDebugEnabled_Vp886DeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp886DeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->debugSelectMask;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

bool
VpDebugEnabled_Vp886LineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp886LineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->debugSelectMask;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_886_SERIES */

#ifdef VP_CC_580_SERIES
bool
VpDebugEnabled_Vp580DeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp580DeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->debugSelectMask;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

bool
VpDebugEnabled_Vp580LineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp580LineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->debugSelectMask;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_580_SERIES */

#ifdef VP_CC_792_SERIES
static bool
VpDebugEnabled_Vp792DeviceObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp792DeviceObjectType *pDevObj)
{
    uint32 debugSelectMask;
    VpDeviceIdType deviceId;

    if (pDevObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pDevObj->options.debugSelect;
    deviceId = pDevObj->deviceId;
    VP_ASSERT(pDevObj != VP_NULL);
    return DeviceDebugEnabled(msgType & debugSelectMask, msgTypeStr, deviceId);
}

static bool
VpDebugEnabled_Vp792LineObjectType(
    uint32 msgType,
    char *msgTypeStr,
    Vp792LineObjectType *pLineObj)
{
    uint32 debugSelectMask;
    VpLineIdType lineId;

    if (pLineObj == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineObj == VP_NULL) ", msgTypeStr);
    }

    debugSelectMask = pLineObj->options.debugSelect;
    lineId = pLineObj->lineId;
    VP_ASSERT(pLineObj != VP_NULL);
    return LineDebugEnabled(msgType & debugSelectMask, msgTypeStr, lineId);
}
#endif /* VP_CC_792_SERIES */

bool
VpDebugEnabled_VpDeviceIdType(
    uint32 msgType,
    char *msgTypeStr,
    VpDeviceIdType *pDeviceId)
{
    VP_ASSERT(pDeviceId != VP_NULL);
    return DeviceDebugEnabled(msgType & vpDebugSelectMask, msgTypeStr, *pDeviceId);
}

bool
VpDebugEnabled_VpLineIdType(
    uint32 msgType,
    char *msgTypeStr,
    VpLineIdType *pLineId)
{
    VP_ASSERT(pLineId != VP_NULL);
    return LineDebugEnabled(msgType & vpDebugSelectMask, msgTypeStr, *pLineId);
}


/*
  * This function returns TRUE if the specified message type is enabled for the specified device.
  * As a side effect, it displays the message type and device ID.
  */
bool
VpDebugEnabled_VpDevCtxType(
    uint32 msgType,
    char *msgTypeStr,
    VpDevCtxType *pDevCtx)
{
    if (pDevCtx == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pDevCtx == VP_NULL) ", msgTypeStr);
    }

    switch (pDevCtx->deviceType) {

#ifdef VP_CC_VCP2_SERIES
        case VP_DEV_VCP2_SERIES:
            return VpDebugEnabled_VpVcp2DeviceObjectType(msgType, msgTypeStr, pDevCtx->pDevObj);
#endif
#ifdef VP_CC_MELT_SERIES
        case VP_DEV_MELT_SERIES:
            return VpDebugEnabled_VpMeltDeviceObjectType(msgType, msgTypeStr, pDevCtx->pDevObj);
#endif
#ifdef VP_CC_890_SERIES
        case VP_DEV_890_SERIES:
            return VpDebugEnabled_Vp890DeviceObjectType(msgType, msgTypeStr, pDevCtx->pDevObj);
#endif
#ifdef VP_CC_880_SERIES
        case VP_DEV_880_SERIES:
            return VpDebugEnabled_Vp880DeviceObjectType(msgType, msgTypeStr, pDevCtx->pDevObj);
#endif
#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            return VpDebugEnabled_Vp886DeviceObjectType(msgType, msgTypeStr, pDevCtx->pDevObj);
#endif
#ifdef VP_CC_792_SERIES
        case VP_DEV_792_SERIES:
            return VpDebugEnabled_Vp792DeviceObjectType(msgType, msgTypeStr, pDevCtx->pDevObj);
#endif
        default:
            /* For other device types (for which we are still using VP_DOUT)
               use the global debug select mask. */
            return VpDebugEnabled_None(msgType, msgTypeStr, (void *)0);
        }
}

/*
  * This function returns TRUE if the specified message type is enabled for the specified line.
  * As a side effect, it displays the message type and line ID.
  */
bool
VpDebugEnabled_VpLineCtxType(
    uint32 msgType,
    char *msgTypeStr,
    VpLineCtxType *pLineCtx)
{
    if (pLineCtx == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineCtx == VP_NULL) ", msgTypeStr);
    }

    if (pLineCtx->pDevCtx == VP_NULL) {
        return NULLDebugEnabled(msgType, "(pLineCtx->pDevCtx == VP_NULL) ", msgTypeStr);
    }

    switch (pLineCtx->pDevCtx->deviceType) {

#ifdef VP_CC_VCP2_SERIES
        case VP_DEV_VCP2_SERIES:
            return VpDebugEnabled_VpVcp2LineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
#ifdef VP_CC_MELT_SERIES
        case VP_DEV_MELT_SERIES:
            return VpDebugEnabled_VpMeltLineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
#ifdef VP_CC_890_SERIES
        case VP_DEV_890_SERIES:
            return VpDebugEnabled_Vp890LineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
#ifdef VP_CC_880_SERIES
        case VP_DEV_880_SERIES:
            return VpDebugEnabled_Vp880LineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            return VpDebugEnabled_Vp886LineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
#ifdef VP_CC_580_SERIES
        case VP_DEV_580_SERIES:
            return VpDebugEnabled_Vp580LineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
#ifdef VP_CC_792_SERIES
        case VP_DEV_792_SERIES:
            return VpDebugEnabled_Vp792LineObjectType(msgType, msgTypeStr, pLineCtx->pLineObj);
#endif
        default:
            /* For other device types (for which we are still using VP_DOUT)
               use the global debug select mask. */
            return VpDebugEnabled_None(msgType, msgTypeStr, (void *)0);
    }
}

bool
VpDebugEnabled_None(
    uint32 msgType,
    char *msgTypeStr,
    void *nothing)
{
    if (!(msgType & vpDebugSelectMask))
        return FALSE;

    VpSysDebugPrintf(msgTypeStr);
    VpSysDebugPrintf(": ");
    return TRUE;
}

#ifdef VP_DEBUG
static const char *
VpGetEnumString(
    const char *strTable[],
    int numStrings,
    int value)
{
    static char buf[7] = "0x0000"; /* not reentrant */
    static const char hex[16] = "0123456789ABCD";

    if ((value < 0) || (value >= numStrings) || (strTable[value] == VP_NULL)) {

        /* Can't find a string for the requested value. */
        buf[5] = hex[value & 0xF];
        value >>= 4;
        buf[4] = hex[value & 0xF];
        value >>= 4;
        buf[3] = hex[value & 0xF];
        value >>= 4;
        buf[2] = hex[value & 0xF];
        return buf;
    }

    /* Found the requested string in the table. */
    return strTable[value];
}
#endif /* VP_DEBUG */

#ifdef VP_DEBUG
const char *
VpGetString_VpStatusType(
    VpStatusType status)
{
    static const char *strTable[VP_STATUS_NUM_TYPES] = {
        "SUCCESS",
        "FAILURE",
        "FUNC_NOT_SUPPORTED",
        "INVALID_ARG",
        "MAILBOX_BUSY",
        "ERR_VTD_CODE",
        "OPTION_NOT_SUPPORTED",
        "ERR_VERIFY",
        "DEVICE_BUSY",
        "MAILBOX_EMPTY",
        "ERR_MAILBOX_DATA",
        "ERR_HBI",
        "ERR_IMAGE",
        "IN_CRTCL_SECTN",
        "DEV_NOT_INITIALIZED",
        "ERR_PROFILE",
        "16",
        "CUSTOM_TERM_NOT_CFG",
        "DEDICATED_PINS",
        "INVALID_LINE",
        "LINE_NOT_CONFIG",
        "ERR_SPI",
        "INPUT_PARAM_OOR"
    };

    /* Make sure we update the above table when we update VpStatusType: */
    VP_ASSERT(VP_STATUS_NUM_TYPES == 23);

    return VpGetEnumString(strTable, VP_STATUS_NUM_TYPES, (int)status);
}
#endif /* VP_DEBUG */

#ifdef VP_DEBUG
const char *
VpGetString_VpOptionIdType(
    VpOptionIdType optionId)
{
    static const char *strTable[VP_NUM_OPTION_IDS] = {
        "VP_DEVICE_OPTION_ID_PULSE", /* only differs by prefix */
        "CRITICAL_FLT",
        "ZERO_CROSS",
        "RAMP2STBY",
        "PULSE_MODE",
        "TIMESLOT",
        "CODEC",
        "PCM_HWY",
        "LOOPBACK",
        "LINE_STATE",
        "EVENT_MASK",
        VP_NULL,
        "RING_CNTRL",
        VP_NULL,
        "DTMF_MODE",
        "DEVICE_IO",
        VP_NULL,
        "PCM_TXRX_CNTRL",
        "PULSE2",
        "LINE_IO_CFG",
        "DEV_IO_CFG",
        "DTMF_SPEC",
        "PARK_MODE",
        "DCFEED_SLOPE",
        "SWITCHER_CTRL",
        "HOOK_DETECT_MODE",
        "AUTO_LOOP_COND",
        "DCFEED_PARAMS",
        "RINGING_PARAMS",
        "GND_FLT_PROTECTION",
        "ADAPTIVE_RINGING",
        "RINGTRIP_CONFIRM",
        "RING_PHASE_SYNC",
        "HIGHPASS_FILTER",
        "FSYNC_RATE",
        "DTMF_PARAMS",
        "VP_OPTION_ID_PULSE",        /* only differs by prefix */
        "DEBUG_SELECT",
        "ABS_GAIN",
        "PCM_SIG_CTL",
        "LINESTATE_CTL_MODE"
    };

    /* Make sure we update the above table when we update VpOptionIdType: */
    VP_ASSERT(VP_NUM_OPTION_IDS == 0x29);

    return VpGetEnumString(strTable, VP_NUM_OPTION_IDS, (int)optionId);
}
#endif /* VP_DEBUG */

#ifdef VP_DEBUG
const char *
VpGetString_VpProfileType(
    VpProfileType profType)
{
    static const char *strTable[VP_NUM_PROFILE_TYPES] = {
        "DEVICE",
        "AC",
        "DC",
        "RING",
        "RINGCAD",
        "TONE",
        "METER",
        "CID",
        "TONECAD",
        "FXO_CONFIG",
        "CUSTOM_TERM",
        "CAL"
    };

    /* Make sure we update the above table when we update VpProfileType: */
    VP_ASSERT(VP_NUM_PROFILE_TYPES == 12);

    return VpGetEnumString(strTable, VP_NUM_PROFILE_TYPES, (int)profType);
}
#endif /* VP_DEBUG */

#if (VP_CC_DEBUG_SELECT & VP_DBG_EVENT)
void VpDisplayEvent(
    VpEventType *pEvent)
{
    typedef struct {
        VpEventCategoryType category;
        uint16 id;
        char *nameStr;
        bool hasEventData;
        bool hasParmHandle;
    } VpEventInfo;

    static const VpEventInfo eventInfoTable[] = {
        { VP_EVCAT_FAULT,      VP_DEV_EVID_BAT_FLT,            "BAT_FLT",            TRUE,  TRUE },
        { VP_EVCAT_FAULT,      VP_DEV_EVID_CLK_FLT,            "CLK_FLT",            TRUE,  TRUE },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_THERM_FLT,          "THERM_FLT",          TRUE,  TRUE },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_DC_FLT,             "DC_FLT",             TRUE,  TRUE },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_AC_FLT,             "AC_FLT",             TRUE,  FALSE },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_SYNC_FLT,           "SYNC_FLT",           FALSE, FALSE },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_RES_LEAK_FLT,       "RES_LEAK_FLT",       TRUE,  TRUE  },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_SEAL_CUR_FLT,       "SEAL_CURRENT_FLT",   TRUE,  FALSE },
        { VP_EVCAT_FAULT,     VP_LINE_EVID_GND_FLT,            "GND_FLT",            TRUE,  TRUE },
        { VP_EVCAT_FAULT,      VP_DEV_EVID_WDT_FLT,            "WDT_FLT",            FALSE, FALSE },
        { VP_EVCAT_FAULT,      VP_DEV_EVID_EVQ_OFL_FLT,        "EVQ_OFL_FLT",        FALSE, TRUE },
        { VP_EVCAT_FAULT,      VP_DEV_EVID_SYSTEM_FLT,         "SYSTEM_FLT",         TRUE,  FALSE },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_HOOK_OFF,           "HOOK_OFF",           FALSE, TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_HOOK_ON,            "HOOK_ON",            FALSE, TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_GKEY_DET,           "GKEY_DET",           FALSE, TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_GKEY_REL,           "GKEY_REL",           FALSE, FALSE },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_FLASH,              "FLASH",              FALSE, TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_STARTPULSE,         "STARTPULSE",         FALSE, TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_DTMF_DIG,           "DTMF_DIG",           TRUE,  TRUE },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_PULSE_DIG,          "PULSE_DIG",          TRUE,  TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_MTONE,              "MTONE",              FALSE, FALSE },
        { VP_EVCAT_SIGNALING,  VP_DEV_EVID_TS_ROLLOVER,        "TS_ROLLOVER",        FALSE, FALSE },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_US_TONE_DETECT,     "US_TONE_DETECT",     TRUE,  TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_DS_TONE_DETECT,     "DS_TONE_DETECT",     TRUE,  TRUE  },
        { VP_EVCAT_SIGNALING,  VP_DEV_EVID_SEQUENCER,          "SEQUENCER",          TRUE,  TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_BREAK_MAX,          "BREAK_MAX",          TRUE,  TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_EXTD_FLASH,         "EXTD_FLASH",         FALSE, TRUE  },
        { VP_EVCAT_SIGNALING, VP_LINE_EVID_HOOK_PREQUAL,       "HOOK_PREQUAL",       TRUE,  TRUE  },
        { VP_EVCAT_RESPONSE,   VP_DEV_EVID_BOOT_CMP,           "BOOT_CMP",           TRUE,  FALSE },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_LLCMD_TX_CMP,       "LLCMD_TX_CMP",       FALSE, FALSE },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_LLCMD_RX_CMP,       "LLCMD_RX_CMP",       FALSE, TRUE  },
        { VP_EVCAT_RESPONSE,   VP_DEV_EVID_DNSTR_MBOX,         "DNSTR_MBOX",         FALSE, FALSE },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_RD_OPTION,          "RD_OPTION",          TRUE,  TRUE  },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_RD_LOOP,            "RD_LOOP",            FALSE, TRUE  },
        { VP_EVCAT_RESPONSE,       VP_EVID_CAL_CMP,            "CAL_CMP",            FALSE, FALSE },
        { VP_EVCAT_RESPONSE,       VP_EVID_CAL_BUSY,           "CAL_BUSY",           FALSE, FALSE },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_GAIN_CMP,           "GAIN_CMP",           FALSE, TRUE  },
        { VP_EVCAT_RESPONSE,   VP_DEV_EVID_DEV_INIT_CMP,       "DEV_INIT_CMP",       TRUE,  FALSE },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_LINE_INIT_CMP,      "LINE_INIT_CMP",      FALSE, FALSE },
        { VP_EVCAT_RESPONSE,   VP_DEV_EVID_IO_ACCESS_CMP,      "IO_ACCESS_CMP",      TRUE,  FALSE },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_LINE_IO_RD_CMP,     "LINE_IO_RD_CMP",     FALSE, TRUE  },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_LINE_IO_WR_CMP,     "LINE_IO_WR_CMP",     FALSE, TRUE  },
        { VP_EVCAT_RESPONSE,  VP_LINE_EVID_SLAC_INIT_CMP,      "SLAC_INIT_CMP",      TRUE,  FALSE },
        { VP_EVCAT_TEST,      VP_LINE_EVID_TEST_CMP,           "TEST_CMP",           FALSE, TRUE  },
        { VP_EVCAT_TEST,      VP_LINE_EVID_DTONE_DET,          "DTONE_DET",          TRUE,  FALSE },
        { VP_EVCAT_TEST,      VP_LINE_EVID_DTONE_LOSS,         "DTONE_LOSS",         TRUE,  FALSE },
        { VP_EVCAT_TEST,       VP_DEV_EVID_STEST_CMP,          "STEST_CMP",          TRUE,  FALSE },
        { VP_EVCAT_TEST,       VP_DEV_EVID_CHKSUM,             "CHKSUM",             FALSE, TRUE  },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_MTR_CMP,            "MTR_CMP",            FALSE, FALSE },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_MTR_ABORT,          "MTR_ABORT",          TRUE,  FALSE },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_CID_DATA,           "CID_DATA",           TRUE,  TRUE },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_RING_CAD,           "RING_CAD",           TRUE,  TRUE },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_SIGNAL_CMP,         "SIGNAL_CMP",         TRUE,  TRUE  },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_MTR_CAD,            "MTR_CAD",            TRUE,  FALSE },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_TONE_CAD,           "TONE_CAD",           FALSE, TRUE },
        { VP_EVCAT_PROCESS,   VP_LINE_EVID_MTR_ROLLOVER,       "MTR_ROLLOVER",       TRUE,  TRUE  },
		{ VP_EVCAT_PROCESS,   VP_LINE_EVID_GEN_TIMER,          "VP_LINE_EVID_GEN_TIMER",      TRUE, TRUE },
		{ VP_EVCAT_PROCESS,   VP_LINE_EVID_USER,               "VP_LINE_EVID_USER",  TRUE,  FALSE },
		{ VP_EVCAT_PROCESS,   VP_LINE_EVID_AUTO_LOOP_COND,     "VP_LINE_EVID_AUTO_LOOP_COND", TRUE, TRUE },
        { VP_EVCAT_FXO,       VP_LINE_EVID_RING_ON,            "RING_ON",            FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_LIU,                "LIU",                FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_LNIU,               "LNIU",               FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_FEED_DIS,           "FEED_DIS",           FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_FEED_EN,            "FEED_EN",            FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_DISCONNECT,         "DISCONNECT",         FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_RECONNECT,          "RECONNECT",          FALSE, TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_POLREV,             "POLREV",             TRUE,  TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_POH,                "POH",                TRUE,  TRUE  },
        { VP_EVCAT_FXO,       VP_LINE_EVID_PNOH,               "PNOH",               TRUE,  TRUE  },
        { (VpEventCategoryType)0, 0, "", TRUE, TRUE }
    };
    const VpEventInfo *pEventInfo = eventInfoTable;
    char eventDataStr[16] = "";
    char parmHandleStr[16] = "";
    char eventNameStr[32] = "";
    bool lineSpecific = (pEvent->pLineCtx != VP_NULL);

    if (pEvent->status != VP_STATUS_SUCCESS) {
        VP_DOUT_(EVENT, VP_DBG_EVENT_COLOR, VpDevCtxType, pEvent->pDevCtx, ("status = %s", VpGetString_VpStatusType(pEvent->status)));
        return;
    }

    /* Find the event in the table. */
    while (
        (pEventInfo->id != 0) &&
        ((pEventInfo->category != pEvent->eventCategory) || (pEventInfo->id != pEvent->eventId))
    ) {
        pEventInfo++;
    }

    /* Build display strings. */
    if (pEventInfo->id == 0) {
        sprintf(eventNameStr, "cat=0x%4.4X id=0x%4.4X", pEvent->eventCategory, pEvent->eventId);
    } else {
        sprintf(eventNameStr, "id=%s", pEventInfo->nameStr);
    }
    if (pEventInfo->hasEventData) {
        sprintf(eventDataStr, " data=0x%4.4X", pEvent->eventData);
    }
    if (pEventInfo->hasParmHandle) {
        sprintf(parmHandleStr, " parm=0x%4.4X", pEvent->parmHandle);
    }

#define PRINTF_VPEVENT \
    ("%s%s%s%s", eventNameStr, eventDataStr, parmHandleStr, (pEvent->hasResults ? " +results" : ""))

    if (lineSpecific) {
        VP_DOUT_(EVENT, VP_DBG_EVENT_COLOR, VpLineCtxType, pEvent->pLineCtx, PRINTF_VPEVENT);
    } else {
        VP_DOUT_(EVENT, VP_DBG_EVENT_COLOR, VpDevCtxType, pEvent->pDevCtx, PRINTF_VPEVENT);
    }
}

void VpDisplayResults(
    VpEventType *pEvent,
    void *pResult)
{
    char outbuf[800];
    char *pOut = outbuf + sprintf(outbuf, "results={");
    bool lineSpecific = (pEvent->pLineCtx != VP_NULL);
    VpEventCategoryType cat = pEvent->eventCategory;
    uint16 id = pEvent->eventId;

    if (!pEvent->hasResults) {
        return;
    }

    if (
        ((cat == VP_EVCAT_RESPONSE) && (id == VP_DEV_EVID_BOOT_CMP)) ||
        ((cat == VP_EVCAT_TEST) && (id == VP_DEV_EVID_CHKSUM))
    ) {
        VpChkSumType *pChkSum = (VpChkSumType *)pResult;
        pOut += sprintf(pOut, "loadChecksum=0x%8.8X ", (unsigned)pChkSum->loadChecksum);
        pOut += sprintf(pOut, "vInfo={");
        pOut += sprintf(pOut, "vtdRevCode=0x%4.4X ",   pChkSum->vInfo.vtdRevCode);
        pOut += sprintf(pOut, "swProductId=0x%2.2X ",  pChkSum->vInfo.swProductId);
        pOut += sprintf(pOut, "swVerMajor=0x%2.2X ",   pChkSum->vInfo.swVerMajor);
        pOut += sprintf(pOut, "swVerMinor=0x%2.2X}",   pChkSum->vInfo.swVerMinor);
    } else if ((cat == VP_EVCAT_TEST) && (id == VP_LINE_EVID_TEST_CMP)) {
        /* To do */
        pOut += sprintf(pOut, "(test result not displayed)");
    } else if (cat == VP_EVCAT_RESPONSE) {
        switch (pEvent->eventId) {
            case VP_LINE_EVID_LLCMD_RX_CMP: {
                uint8 *pData = (uint8 *)pResult;
                uint16 bytes = pEvent->eventData;
                while (bytes--) {
                    pOut += sprintf(pOut, " %2.2X", *(pData++));
                }
                break;
            }
            case VP_LINE_EVID_RD_OPTION:
                /* To do */
                pOut += sprintf(pOut, "(option value not displayed)");
                break;
            case VP_LINE_EVID_RD_LOOP: {
                VpLoopCondResultsType *pLoopCond = (VpLoopCondResultsType *)pResult;
                pOut += sprintf(pOut, "rloop=%d ",       pLoopCond->rloop);
                pOut += sprintf(pOut, "ilg=%d ",         pLoopCond->ilg);
                pOut += sprintf(pOut, "imt=%d ",         pLoopCond->imt);
                pOut += sprintf(pOut, "vbat1=%d ",       pLoopCond->vbat1);
                pOut += sprintf(pOut, "vsab=%d ",        pLoopCond->vsab);
                pOut += sprintf(pOut, "vbat2=%d ",       pLoopCond->vbat2);
                pOut += sprintf(pOut, "mspl=%d ",        pLoopCond->mspl);
                pOut += sprintf(pOut, "vbat3=%d ",       pLoopCond->vbat3);
                pOut += sprintf(pOut, "selectedBat=%d ", pLoopCond->selectedBat);
                pOut += sprintf(pOut, "dcFeedReg=%d",    pLoopCond->dcFeedReg);
                break;
            }
            case VP_LINE_EVID_GAIN_CMP: {
                VpRelGainResultsType *pRelGain = (VpRelGainResultsType *)pResult;
                pOut += sprintf(pOut, "gResult=%d ",    pRelGain->gResult);
                pOut += sprintf(pOut, "gxValue=%4.4X ", pRelGain->gxValue);
                pOut += sprintf(pOut, "grValue=%4.4X",  pRelGain->grValue);
                break;
            }
            case VP_DEV_EVID_IO_ACCESS_CMP:
                /* We can't distinguish between VpDeviceIoAccessDataType and
                   VpDeviceIoAccessExtType here. */
                pOut += sprintf(pOut, "(cannot display result)");
                break;
            case VP_LINE_EVID_LINE_IO_RD_CMP: {
                VpLineIoAccessType *pLineIoAccess = (VpLineIoAccessType *)pResult;
                pOut += sprintf(pOut, "direction=%d ", pLineIoAccess->direction);
                pOut += sprintf(pOut, "ioBits={");
                pOut += sprintf(pOut, "mask=%2.2X ", pLineIoAccess->ioBits.mask);
                pOut += sprintf(pOut, "data=%2.2X}", pLineIoAccess->ioBits.data);
                break;
            }
            case VP_LINE_EVID_QUERY_CMP: {
                VpQueryResultsType *pQueryResults = (VpQueryResultsType *)pResult;
                int i;
                switch (pEvent->eventData) {
                    case VP_QUERY_ID_TEMPERATURE:
                        pOut += sprintf(pOut, "temp=%d", pQueryResults->temp);
                        break;
                    case VP_QUERY_ID_METER_COUNT:
                        pOut += sprintf(pOut, "meterCount=%u", pQueryResults->meterCount);
                        break;
                    case VP_QUERY_ID_LOOP_RES:
                        pOut += sprintf(pOut, "rloop=%u", pQueryResults->rloop);
                        break;
                    case VP_QUERY_ID_SEAL_CUR:
                        pOut += sprintf(pOut, "sealCur={");
                        pOut += sprintf(pOut, "sealApplyTime=%u ", pQueryResults->sealCur.sealApplyTime);
                        pOut += sprintf(pOut, "sealCycleTime=%u ", pQueryResults->sealCur.sealCycleTime);
                        pOut += sprintf(pOut, "maxCurrent=%u ", pQueryResults->sealCur.maxCurrent);
                        pOut += sprintf(pOut, "minCurrent=%u ", pQueryResults->sealCur.minCurrent);
                        pOut += sprintf(pOut, "sealMaskArray[]={");
                        for (i = 0; i < VP_SEAL_CURRENT_ARRAY_SIZE; i++) {
                            pOut += sprintf(pOut, "0x%4.4X ", pQueryResults->sealCur.sealMaskArray[i]);
                        }
                        pOut += sprintf(pOut, "} }");
                        break;
                    case VP_QUERY_ID_DEV_TRAFFIC:
                        pOut += sprintf(pOut, "trafficBytes=%lu", pQueryResults->trafficBytes);
                        break;
                    case VP_QUERY_ID_LINE_CAL_COEFF:
                        break;
                    case VP_QUERY_ID_TIMESTAMP:
                        pOut += sprintf(pOut, "timestamp=0x%08lX", pQueryResults->timestamp);
                        break;
                    case VP_QUERY_ID_LINE_TOPOLOGY:
                        pOut += sprintf(pOut, "rInsideDcSense=%u ", pQueryResults->lineTopology.rInsideDcSense);
                        pOut += sprintf(pOut, "rOutsideDcSense=%u ", pQueryResults->lineTopology.rOutsideDcSense);
                        break;
                    default:
                        /* Unknown Query ID. */
                        VP_ERROR(VpDevCtxType, pEvent->pDevCtx, ("Unknown query ID %d", pEvent->eventData));
                        return;
                }
                break;
            }
            case VP_EVID_CAL_CMP: {
                break;
            }
            default:
                VP_ERROR(VpDevCtxType, pEvent->pDevCtx, ("Unknown event results type (eventId=0x%4.4X)", pEvent->eventId));
                return;
        }
    }

    sprintf(pOut, "}");
    if (lineSpecific) {
        VP_DOUT_(EVENT, VP_DBG_EVENT_COLOR, VpLineCtxType, pEvent->pLineCtx, ("%s", outbuf));
    } else {
        VP_DOUT_(EVENT, VP_DBG_EVENT_COLOR, VpDevCtxType, pEvent->pDevCtx, ("%s", outbuf));
    }
}

#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_EVENT) */

#if ((VP_CC_DEBUG_SELECT & VP_DBG_HAL) && (defined(VP_CC_VCP2_SERIES) || defined(VP_CC_MELT_SERIES) || \
                                            defined(VP_CC_VCP_SERIES) || defined(VP_CC_792_SERIES)))
void
VpDisplayHbiAccess(
    VpDeviceType deviceType,
    VpDeviceIdType deviceId,
    unsigned bufLen,
    uint16 *buf)
{
    VP_HAL(VpDeviceIdType, &deviceId, ("%u-word opaque HBI block {", bufLen));

    while (bufLen > 0) {
        uint16 cmd = *buf++;
        unsigned dataLength = VpDisplayHbiCmd(deviceType, deviceId, cmd);
        bufLen--;
        if (dataLength > 0) {
            if (dataLength > bufLen) {
                VP_ERROR(VpDeviceIdType, &deviceId, ("Not enough data for HAL Access!"));
                dataLength = bufLen;
            }
            VpDisplayHbiData(deviceId, dataLength, buf);
            buf += dataLength;
            bufLen -= dataLength;
        }
    }

    VP_HAL(VpDeviceIdType, &deviceId, ("} end of block"));
}

#define HBI_WORDS_PER_LINE 8
#define HBI_WORD_FILLER "---- "
#define HBI_COMMAND_LINE_FILLER " = "

unsigned
VpDisplayHbiCmd(
    VpDeviceType deviceType,
    VpDeviceIdType deviceId,
    uint16 cmd)
{
    char output[128];
    unsigned dataLength = 0;

    switch (deviceType) {

#if (defined(VP_CC_VCP2_SERIES) || defined(VP_CC_MELT_SERIES) || defined(VP_CC_VCP_SERIES))
        case VP_DEV_VCP_SERIES:
        case VP_DEV_VCP2_SERIES:
        case VP_DEV_MELT_SERIES:
            if (cmd == 0xFFFF) {
                sprintf(output, "%4.4X%s%s", (int)cmd, HBI_COMMAND_LINE_FILLER, "No Op");
            } else if ((cmd & 0xFF00) == 0xFE00) {
                sprintf(output, "%4.4X%s%s 0x%2.2X", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Select Page", (int)(cmd & 0xFF));
            } else if ((cmd & 0xFF00) == (0xFD00)) {
                sprintf(output, "%4.4X%s%s 0x%2.2X", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Config Interface", (int)(cmd & 0xFF));
            } else if ((cmd & 0xFE00) == (0xFA00)) {
                dataLength = (cmd & 0xFF) + 1;
                sprintf(output, "%4.4X%s%s (%u word%s)", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Continue Paged Access", dataLength, (dataLength > 1 ? "s" : ""));
            } else if ((cmd & 0xFE00) == (0xF800)) {
                dataLength = (cmd & 0xFF) + 1;
                sprintf(output, "%4.4X%s%s (%u word%s)", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Start Paged Access", dataLength, (dataLength > 1 ? "s" : ""));
            } else {
                unsigned offset = (cmd & 0x7F00) >> 8;
                bool direct = ((cmd & 0x8000) != 0);
                bool write = ((cmd & 0x0080) != 0);
                dataLength = (cmd & 0x007F) + 1;
                sprintf(output, "%4.4X%s%s Offset %s (%u word%s at offset 0x%2.2X)", (int)cmd,
                    HBI_COMMAND_LINE_FILLER, (direct ? "Direct" : "Paged"),
                    (write ? "Write" : "Read"), dataLength, (dataLength > 1 ? "s" : ""),
                    offset);
            }
            break;
#endif /* (defined(VP_CC_VCP2_SERIES) || defined(VP_CC_VCP_SERIES) || defined(VP_CC_MELT_SERIES)) */

#ifdef VP_CC_792_SERIES
        case VP_DEV_792_SERIES:
            if (cmd == 0xFFFF) {
                sprintf(output, "%4.4X%s         %s", (int)cmd, HBI_COMMAND_LINE_FILLER, "No Op");
            } else if (cmd == 0xF701) {
                dataLength = 2;
                sprintf(output, "%4.4X%s         %s", (int)cmd, HBI_COMMAND_LINE_FILLER, "Read Shared Interrupt Regs");
            } else if ((cmd & 0xFF00) == (0xFD00)) {
                sprintf(output, "%4.4X%s         %s 0x%2.2X", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Config Interface", (int)(cmd & 0xFF));
            } else if ((cmd & 0xFF00) == 0xFE00) {
                if ((cmd & 0x00FF) == 0x00FF) {
                    sprintf(output, "%4.4X%s         %s", (int)cmd, HBI_COMMAND_LINE_FILLER,
                        "Code Load Page Broadcast");
                } else if ((cmd & 0x008F) == 0x0080) {
                    unsigned slacId = (cmd & 0x0070) >> 4;
                    sprintf(output, "%4.4X%s(slac %u) %s", (int)cmd, HBI_COMMAND_LINE_FILLER, slacId,
                        "Select Code Load Page");
                } else if ((cmd & 0x0080) == 0x0000) {
                    unsigned slacId = (cmd & 0x0070) >> 4;
                    unsigned page = cmd & 0x000F;
                    sprintf(output, "%4.4X%s(slac %u) %s %u", (int)cmd, HBI_COMMAND_LINE_FILLER, slacId,
                        "Select Page", page);
                } else {
                    sprintf(output, "%4.4X%s%s", (int)cmd, HBI_COMMAND_LINE_FILLER,
                        "INVALID COMMAND WORD!");
                }
            } else if ((cmd & 0xFF00) == (0xFD00)) {
                sprintf(output, "%4.4X%s         %s 0x%2.2X", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Config Interface", (int)(cmd & 0xFF));
            } else if ((cmd & 0xFF00) == (0xFC00)) {
                unsigned offset = (cmd & 0x00F0) >> 4;
                dataLength = (cmd & 0x000F) + 1;
                sprintf(output, "%4.4X%s         %s (%u word%s at offset 0x%2.2X)", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Direct Page Broadcast", dataLength, (dataLength > 1 ? "s" : ""), offset);
            } else if ((cmd & 0xFE00) == (0xFA00)) {
                dataLength = (cmd & 0xFF) + 1;
                sprintf(output, "%4.4X%s         %s (%u word%s)", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Continue Mailbox Access", dataLength, (dataLength > 1 ? "s" : ""));
            } else if ((cmd & 0xFE00) == (0xF800)) {
                dataLength = (cmd & 0xFF) + 1;
                sprintf(output, "%4.4X%s         %s (%u word%s)", (int)cmd, HBI_COMMAND_LINE_FILLER,
                    "Start Mailbox Access", dataLength, (dataLength > 1 ? "s" : ""));
            } else if (cmd & 0x8000) {
                unsigned offset = (cmd & 0x7F00) >> 8;
                bool write = (cmd & 0x0080);
                unsigned slacId = (cmd & 0x0070) >> 4;
                dataLength = (cmd & 0x000F) + 1;
                sprintf(output, "%4.4X%s(slac %u) %s %s (%u word%s at offset 0x%2.2X)",
                    (int)cmd, HBI_COMMAND_LINE_FILLER, slacId,
                    "Direct Page", (write ? "Write" : "Read"), dataLength,
                    (dataLength > 1 ? "s" : ""), offset);
            } else {
                unsigned offset = (cmd & 0x7F00) >> 8;
                bool write = ((cmd & 0x0080) != 0);
                dataLength = (cmd & 0x007F) + 1;
                sprintf(output, "%4.4X%s         Paged %s (%u word%s at offset 0x%2.2X)", (int)cmd,
                    HBI_COMMAND_LINE_FILLER, (write ? "Write" : "Read"), dataLength,
                    (dataLength > 1 ? "s" : ""), offset);
            }
            break;
#endif /* VP_CC_792_SERIES */

        default:
            sprintf(output, "%4.4X (cmd)", (int)cmd);
    }

    VP_HAL(VpDeviceIdType, &deviceId, ("%s", output));
    return dataLength;
}

void
VpDisplayHbiData(
    VpDeviceIdType deviceId,
    unsigned bufLen,
    uint16 *buf)
{
    char output[128] = "       ";
    char *pOut = &output[7];
    unsigned disp = 0;

    if (buf == VP_NULL) {
        VP_HAL(VpDeviceIdType, &deviceId, ("       (%u words of zeroes)", bufLen));
        return;
    }

    while (bufLen--) {
        pOut += sprintf(pOut, "%4.4X ", *buf++);
        disp++;
        if ((bufLen == 0) || ((disp % HBI_WORDS_PER_LINE) == 0)) {
            VP_HAL(VpDeviceIdType, &deviceId, ("%s", output));
            pOut = &output[7];
        }
    }
}

bool
VpHalHbiReadWrapper(
    VpDeviceType deviceType,
    VpDeviceIdType deviceId,
    uint16 cmd,
    uint8 numwords,
    uint16p data)
{
    bool status = VpHalHbiRead(deviceId, cmd, numwords, data);
    VpDisplayHbiCmd(deviceType, deviceId, cmd);
    VpDisplayHbiData(deviceId, numwords + 1, data);
    return status;
}

#endif /* ((VP_CC_DEBUG_SELECT & VP_DBG_HAL) && (defined(VP_CC_VCP2_SERIES) || defined(VP_CC_MELT_SERIES) || defined(VP_CC_VCP_SERIES))) */

#if (VP_CC_DEBUG_SELECT & VP_DBG_SSL)

const char *
VpGetString_VpCriticalSecType(
    VpCriticalSecType criticalSecType)
{
    static const char *strTable[VP_NUM_CRITICAL_SEC_TYPES] = {
        "VP_MPI_CRITICAL_SEC",
        "VP_HBI_CRITICAL_SEC",
        "VP_CODE_CRITICAL_SEC"
    };

    VP_ASSERT(VP_NUM_CRITICAL_SEC_TYPES == VP_CODE_CRITICAL_SEC + 1);

    return VpGetEnumString(strTable, VP_NUM_CRITICAL_SEC_TYPES, (int)criticalSecType);
}

uint8
VpSysEnterCriticalWrapper(
    VpDeviceIdType deviceId,
    VpCriticalSecType criticalSecType)
{
    uint8 depth = VpSysEnterCritical(deviceId, criticalSecType);
    VP_SSL(VpDeviceIdType, &(deviceId), ("VpSysEnterCritical(%s) = %u", VpGetString_VpCriticalSecType(criticalSecType), (unsigned)depth));
    return depth;
}

uint8
VpSysExitCriticalWrapper(
    VpDeviceIdType deviceId,
    VpCriticalSecType criticalSecType)
{
    uint8 depth = VpSysExitCritical(deviceId, criticalSecType);
    VP_SSL(VpDeviceIdType, &(deviceId), ("VpSysExitCritical(%s) = %u", VpGetString_VpCriticalSecType(criticalSecType), (unsigned)depth));
    return depth;
}

#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_SSL) */

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
void
VpPrintVpApiVersion(
    void)
{
    if (VP_API_VERSION_PATCH_NUM == 0) {
        VP_INFO(None, NULL, ("VP-API-II Version %d.%d.%d",
            VP_API_VERSION_MAJOR_NUM, VP_API_VERSION_MINOR_NUM,
            VP_API_VERSION_MINI_NUM));
    } else {
        VP_INFO(None, NULL, ("VP-API-II Version %d.%d.%d.%d",
            VP_API_VERSION_MAJOR_NUM, VP_API_VERSION_MINOR_NUM,
            VP_API_VERSION_MINI_NUM, VP_API_VERSION_PATCH_NUM));
    }
}

#if defined (VP_CC_880_SERIES) || defined (VP_CC_890_SERIES) || defined (VP_CC_886_SERIES)

void
VpPrintTermType(
    VpTermType termType)
{
#define VP_SUPPORT_TERM_SIZE    (8)
    uint8 termIndex;
    VpTermType termSupported[VP_SUPPORT_TERM_SIZE] = {
        VP_TERM_FXS_GENERIC, VP_TERM_FXS_ISOLATE, VP_TERM_FXS_SPLITTER,
        VP_TERM_FXS_LOW_PWR, VP_TERM_FXS_SPLITTER_LP, VP_TERM_FXS_ISOLATE_LP,
        VP_TERM_FXO_GENERIC, VP_TERM_FXO_DISC
    };
    char *termString[VP_SUPPORT_TERM_SIZE] = {
        "VP_TERM_FXS_GENERIC", "VP_TERM_FXS_ISOLATE", "VP_TERM_FXS_SPLITTER",
        "VP_TERM_FXS_LOW_PWR", "VP_TERM_FXS_SPLITTER_LP", "VP_TERM_FXS_ISOLATE_LP",
        "VP_TERM_FXO_GENERIC", "VP_TERM_FXO_DISC"
    };

    for (termIndex = 0; termIndex < VP_SUPPORT_TERM_SIZE; termIndex++) {
        if (termType == termSupported[termIndex]) {
            VpSysDebugPrintf("\n\n\rpLineObj->termType = %d : %s",
                termType, termString[termIndex]);
            return;
        }
    }
    VpSysDebugPrintf("\n\n\rpLineObj->termType = %d : UNKNOWN TERM TYPE",
        termType);
}

void
VpPrintLineStateType(
    VpLineStateType lineState,
    char *strLineState)
{
#define VP_SUPPORT_STATE_SIZE    (21)
    uint8 stateIndex;

    VpLineStateType lineStateArray[VP_SUPPORT_STATE_SIZE] = {
        VP_LINE_STANDBY,            VP_LINE_TIP_OPEN,           VP_LINE_ACTIVE,
        VP_LINE_ACTIVE_POLREV,      VP_LINE_TALK,               VP_LINE_TALK_POLREV,
        VP_LINE_OHT,                VP_LINE_OHT_POLREV,         VP_LINE_DISCONNECT,
        VP_LINE_RINGING,            VP_LINE_RINGING_POLREV,     VP_LINE_STANDBY_POLREV,
        VP_LINE_RING_OPEN,          VP_LINE_HOWLER,             VP_LINE_HOWLER_POLREV,
        VP_LINE_FXO_OHT,            VP_LINE_FXO_LOOP_OPEN,      VP_LINE_FXO_LOOP_CLOSE,
        VP_LINE_FXO_TALK,           VP_LINE_FXO_RING_GND,       VP_LINE_DISABLED
    };
    char *lineStateStr[VP_SUPPORT_STATE_SIZE] = {
        "VP_LINE_STANDBY",          "VP_LINE_TIP_OPEN",         "VP_LINE_ACTIVE",
        "VP_LINE_ACTIVE_POLREV",    "VP_LINE_TALK",             "VP_LINE_TALK_POLREV",
        "VP_LINE_OHT",              "VP_LINE_OHT_POLREV",       "VP_LINE_DISCONNECT",
        "VP_LINE_RINGING",          "VP_LINE_RINGING_POLREV",   "VP_LINE_STANDBY_POLREV",
        "VP_LINE_RING_OPEN",        "VP_LINE_HOWLER",           "VP_LINE_HOWLER_POLREV",
        "VP_LINE_FXO_OHT",          "VP_LINE_FXO_LOOP_OPEN",    "VP_LINE_FXO_LOOP_CLOSE",
        "VP_LINE_FXO_TALK",         "VP_LINE_FXO_RING_GND",     "VP_LINE_DISABLED"
    };

    for (stateIndex = 0; stateIndex < VP_SUPPORT_STATE_SIZE; stateIndex++) {
        if (lineState == lineStateArray[stateIndex]) {
            VpSysDebugPrintf("\n\r%s = %d : %s", strLineState, lineState, lineStateStr[stateIndex]);
            return;
        }
    }
    VpSysDebugPrintf("\n\r%s = %d : UNKNOWN LINE STATE", strLineState, lineState);
}

void
VpPrintOptionRingControlType(
    VpOptionRingControlType *ringCtrl)
{
    VpPrintOptionZeroCrossType(ringCtrl->zeroCross);
    VpSysDebugPrintf("\npLineObj->ringCtrl.ringExitDbncDur = %d", ringCtrl->ringExitDbncDur);
    VpPrintLineStateType(ringCtrl->ringTripExitSt, "pLineObj->ringCtrl.ringTripExitSt");

}

void
VpPrintOptionZeroCrossType(
    VpOptionZeroCrossType zeroCross)
{
    switch(zeroCross) {
        case VP_OPTION_ZC_M4B:
            VpSysDebugPrintf("\npLineObj->ringCtrl.zeroCross = VP_OPTION_ZC_M4B");
            break;

        case  VP_OPTION_ZC_B4M:
            VpSysDebugPrintf("\npLineObj->ringCtrl.zeroCross = VP_OPTION_ZC_B4M");
            break;

        case  VP_OPTION_ZC_NONE:
            VpSysDebugPrintf("\npLineObj->ringCtrl.zeroCross = VP_OPTION_ZC_NONE");
            break;

        default:
            VpSysDebugPrintf("\npLineObj->ringCtrl.zeroCross = UNKNOWN (%d)",
                zeroCross);
    }
}

void
VpPrintRelayControlType(
    VpRelayControlType relayState)
{
    switch(relayState) {
        case VP_RELAY_NORMAL:
            VpSysDebugPrintf("\npLineObj->relayState = VP_RELAY_NORMAL");
            break;

        case  VP_RELAY_RESET:
            VpSysDebugPrintf("\npLineObj->relayState = VP_RELAY_RESET");
            break;

        case  VP_RELAY_BRIDGED_TEST:
            VpSysDebugPrintf("\npLineObj->relayState = VP_RELAY_BRIDGED_TEST");
            break;

        default:
            VpSysDebugPrintf("\npLineObj->relayState = UNKNOWN (%d)",
                relayState);
    }
}

void
VpPrintOptionCodecType(
    VpOptionCodecType codec)
{
    char *codecStr[VP_NUM_OPTION_CODEC_TYPE_IDS] = {
        "VP_OPTION_ALAW",           "VP_OPTION_MLAW",               "VP_OPTION_LINEAR",
        "VP_OPTION_WIDEBAND",       "VP_OPTION_ALAW_WIDEBAND",      "VP_OPTION_MLAW_WIDEBAND"
    };

    if (codec < VP_NUM_OPTION_CODEC_TYPE_IDS) {
        VpSysDebugPrintf("\npLineObj->codec = %s", codecStr[codec]);
    } else {
        VpSysDebugPrintf("\npLineObj->codec = UNKNOWN (%d)", codec);
    }
}

void
VpPrintOptionPcmTxRxCntrlType(
    VpOptionPcmTxRxCntrlType pcmTxRxCtrl)
{
    char *pcmCtrlStr[] = {
        "VP_OPTION_PCM_BOTH", "VP_OPTION_PCM_RX_ONLY", "VP_OPTION_PCM_TX_ONLY",
        "VP_OPTION_PCM_ALWAYS_ON", "VP_OPTION_PCM_OFF"
    };

    if (pcmTxRxCtrl <= 3) {
        VpSysDebugPrintf("\npLineObj->pcmTxRxCtrl = %s", pcmCtrlStr[pcmTxRxCtrl]);
    } else {
        VpSysDebugPrintf("\npLineObj->pcmTxRxCtrl = UNKNOWN (%d)", pcmTxRxCtrl);
    }
}

void
VpPrintDigitType(
    VpDigitType digit,
    char *name)
{
    uint8 arrayLen;
    uint8 digitIndex;

    VpDigitType digitArray[] = {
        VP_DIG_1,       VP_DIG_2,       VP_DIG_3,       VP_DIG_A,
        VP_DIG_4,       VP_DIG_5,       VP_DIG_6,       VP_DIG_B,
        VP_DIG_7,       VP_DIG_8,       VP_DIG_9,       VP_DIG_C,
        VP_DIG_ASTER,   VP_DIG_ZERO,    VP_DIG_POUND,   VP_DIG_D,
        VP_DIG_NONE
    };
    char *digitStr[] = {
        "VP_DIG_1",     "VP_DIG_2",     "VP_DIG_3",     "VP_DIG_A",
        "VP_DIG_4",     "VP_DIG_5",     "VP_DIG_6",     "VP_DIG_B",
        "VP_DIG_7",     "VP_DIG_8",     "VP_DIG_9",     "VP_DIG_C",
        "VP_DIG_ASTER", "VP_DIG_ZERO",  "VP_DIG_POUND", "VP_DIG_D",
        "VP_DIG_NONE"
    };

    arrayLen = sizeof(digitArray) / sizeof(VpDigitType);

    for (digitIndex = 0; digitIndex < arrayLen; digitIndex++) {
        if (digit == digitArray[digitIndex]) {
            VpSysDebugPrintf("\n\r%s = 0x%02X : %s", name, digit, digitStr[digitIndex]);
            return;
        }
    }
    VpSysDebugPrintf("\n\r%s = 0x%02X", name, digit);
}

#if (VP_CC_DEBUG_SELECT & (VP_DBG_CID | VP_DBG_INFO))
void
VpPrintCallerIdType(
    VpCallerIdType *callerId)
{
    uint8 msgIndex;

    VpSysDebugPrintf("\n\npLineObj->callerId.status = 0x%08lX", callerId->status);
    VpSysDebugPrintf("\npLineObj->callerId.dtmfStatus = 0x%04X", callerId->dtmfStatus);
    VpSysDebugPrintf("\npLineObj->callerId.digitDet = 0x%04X", callerId->digitDet);
    VpSysDebugPrintf("\npLineObj->callerId.cliTimer = %d", callerId->cliTimer);
    VpSysDebugPrintf("\npLineObj->callerId.currentData = 0x%02X", callerId->currentData);
    VpSysDebugPrintf("\npLineObj->callerId.cidCheckSum = 0x%04X", callerId->cidCheckSum);
    VpSysDebugPrintf("\npLineObj->callerId.cliDebounceTime = %d", callerId->cliDebounceTime);
    VpSysDebugPrintf("\npLineObj->callerId.pCliProfile = %p", callerId->pCliProfile);
    VpSysDebugPrintf("\npLineObj->callerId.cliDetectTone1 = 0x%04X", callerId->cliDetectTone1);
    VpSysDebugPrintf("\npLineObj->callerId.cliDetectTone2 = 0x%04X", callerId->cliDetectTone2);
    VpSysDebugPrintf("\npLineObj->callerId.cliMPIndex = %d", callerId->cliMPIndex);
    VpSysDebugPrintf("\npLineObj->callerId.cliMSIndex = %d", callerId->cliMSIndex);
    VpSysDebugPrintf("\npLineObj->callerId.cliIndex = %d", callerId->cliIndex);

    VpSysDebugPrintf("\npLineObj->callerId.primaryMsgLen = %d", callerId->primaryMsgLen);
    VpSysDebugPrintf("\npLineObj->callerId.primaryBuffer =");
    for (msgIndex = 0; msgIndex < VP_SIZEOF_CID_MSG_BUFFER; msgIndex++) {
        if (!(msgIndex % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" 0x%02X", callerId->primaryBuffer[msgIndex]);
    }

    VpSysDebugPrintf("\npLineObj->callerId.secondaryMsgLen = %d", callerId->secondaryMsgLen);
    VpSysDebugPrintf("\npLineObj->callerId.secondaryBuffer =");
    for (msgIndex = 0; msgIndex < VP_SIZEOF_CID_MSG_BUFFER; msgIndex++) {
        if (!(msgIndex % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" 0x%02X", callerId->secondaryBuffer[msgIndex]);
    }
}
#endif

void
VpPrintVpCslacTimerStruct(
    VpCslacTimerStruct *lineTimers)
{
    if (lineTimers->type == VP_CSLAC_FXO_TIMER) {
#if (defined(VP_CC_880_SERIES) && defined(VP880_FXO_SUPPORT)) \
 || (defined(VP_CC_890_SERIES) && defined(VP890_FXO_SUPPORT))
        VpSysDebugPrintf("\npLineObj->lineTimers.type = VP_CSLAC_FXO_TIMER");
        VpSysDebugPrintf("\n\ttimers.fxoTimer.highToLowTime = %d",
            lineTimers->timers.fxoTimer.highToLowTime);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.prevHighToLowTime = %d",
            lineTimers->timers.fxoTimer.prevHighToLowTime);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.timeLastPolRev = %d",
            lineTimers->timers.fxoTimer.timeLastPolRev);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.timePrevPolRev = %d",
            lineTimers->timers.fxoTimer.timePrevPolRev);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.maxPeriod = %d",
            lineTimers->timers.fxoTimer.maxPeriod);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.lastStateChange = %d",
            lineTimers->timers.fxoTimer.lastStateChange);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.lastNotLiu = %d",
            lineTimers->timers.fxoTimer.lastNotLiu);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.disconnectDebounce = %d",
            lineTimers->timers.fxoTimer.disconnectDebounce);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.disconnectDuration = %d",
            lineTimers->timers.fxoTimer.disconnectDuration);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.liuDebounce = %d",
            lineTimers->timers.fxoTimer.liuDebounce);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.ringOffDebounce = %d",
            lineTimers->timers.fxoTimer.ringOffDebounce);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.ringTimer = %d",
            lineTimers->timers.fxoTimer.ringTimer);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.cidCorrectionTimer = %d",
            lineTimers->timers.fxoTimer.cidCorrectionTimer);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.bCalTimer = %d",
            lineTimers->timers.fxoTimer.bCalTimer);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.fxoDiscIO2Change = %d",
            lineTimers->timers.fxoTimer.fxoDiscIO2Change);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.pllRecovery = %d",
            lineTimers->timers.fxoTimer.pllRecovery);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.currentMonitorTimer = %d",
            lineTimers->timers.fxoTimer.currentMonitorTimer);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.measureBFilterTimer = %d",
            lineTimers->timers.fxoTimer.measureBFilterTimer);
        VpSysDebugPrintf("\n\ttimers.fxoTimer.lowVoltageTimer = %d",
            lineTimers->timers.fxoTimer.lowVoltageTimer);
        VpSysDebugPrintf("\n\rpLineObj->cadence.noCount = %s",
            ((lineTimers->timers.fxoTimer.noCount == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpLineObj->cadence.lastState = %s",
            ((lineTimers->timers.fxoTimer.lastState == TRUE) ? "TRUE" : "FALSE"));
#endif
    } else {
        uint8 timerIndex;
        char *timerNames[VP_LINE_TIMER_LAST] = {
            "VP_LINE_CID_DEBOUNCE", "VP_LINE_TIMER_CID_DTMF",
            "VP_LINE_TIMER_FAULT", "VP_LINE_RING_EXIT_PROCESS",
            "VP_LINE_HOOK_FREEZE", "VP_LINE_DISCONNECT_EXIT",
            "VP_LINE_GND_START_TIMER", "VP_LINE_CAL_LINE_TIMER",
            "VP_LINE_PING_TIMER", "VP_LINE_TRACKER_DISABLE",
            "VP_LINE_GPIO_CLKOUT_TIMER", "VP_LINE_INTERNAL_TESTTERM_TIMER",
            "VP_LINE_SPEEDUP_RECOVERY_TIMER"
        };

        VpSysDebugPrintf("\npLineObj->lineTimers.type = VP_CSLAC_FXS_TIMER");
        VpSysDebugPrintf("\npLineObj->lineTimers.trackingTime = %d",
                         lineTimers->timers.trackingTime);
        VpSysDebugPrintf("\npLineObj->lineTimers.timer =");
        for (timerIndex = 0; timerIndex < VP_LINE_TIMER_LAST; timerIndex++) {
            if (!(timerIndex % 10)) {
                VpSysDebugPrintf("\n\t");
            }
            VpSysDebugPrintf(" 0x%04X", lineTimers->timers.timer[timerIndex]);
            if (lineTimers->timers.timer[timerIndex] & VP_ACTIVATE_TIMER) {
                VpSysDebugPrintf(" - %s is Active\n\t", timerNames[timerIndex]);
            }
        }
    }
}

void
VpPrintSeqDataType(
    VpSeqDataType *cadence)
{
    VpSysDebugPrintf("\n\npLineObj->cadence.pActiveCadence = %p", cadence->pActiveCadence);
    VpSysDebugPrintf("\npLineObj->cadence.pCurrentPos = %p", cadence->pCurrentPos);
    VpSysDebugPrintf("\npLineObj->cadence.status = 0x%04X", cadence->status);
    VpSysDebugPrintf("\npLineObj->cadence.index = %d", cadence->index);
    VpSysDebugPrintf("\npLineObj->cadence.length = %d", cadence->length);
    {
        uint8 branchDepth;
        VpSysDebugPrintf("\npLineObj->cadence.count =");
        for (branchDepth = 0; branchDepth < VP_CSLAC_MAX_BRANCH_DEPTH; branchDepth++) {
            VpSysDebugPrintf(" 0x%02X", cadence->count[branchDepth]);
        }
    }
    {
        uint8 regDataIndex;
        VpSysDebugPrintf("\npLineObj->cadence.regData =");
        for (regDataIndex = 0; regDataIndex < VPCSLAC_MAX_GENERATOR_DATA; regDataIndex++) {
            if (!(regDataIndex % 10)) {
                VpSysDebugPrintf("\n\t");
            }
            VpSysDebugPrintf(" 0x%02X", cadence->regData[regDataIndex]);
        }
    }
    VpSysDebugPrintf("\npLineObj->cadence.timeRemain = %d", cadence->timeRemain);
    VpSysDebugPrintf("\npLineObj->cadence.branchAt = %d", cadence->branchAt);
    VpSysDebugPrintf("\npLineObj->cadence.meteringBurst = %d", cadence->meteringBurst);
    VpSysDebugPrintf("\n\rpLineObj->cadence.meterPendingAbort = %s",
        ((cadence->meterPendingAbort == TRUE) ? "TRUE" : "FALSE"));
    VpPrintLineStateType(cadence->meterAbortLineState, "pLineObj->cadence.meterAbortLineState");

    VpSysDebugPrintf("\npLineObj->cadence.startFreq = 0x%04X", cadence->startFreq);
    VpSysDebugPrintf("\npLineObj->cadence.stopFreq = 0x%04X", cadence->stopFreq);
    VpSysDebugPrintf("\npLineObj->cadence.freqStep = 0x%04X", cadence->freqStep);
    VpSysDebugPrintf("\n\rpLineObj->cadence.isFreqIncrease = %s",
        ((cadence->isFreqIncrease == TRUE) ? "TRUE" : "FALSE"));

    VpSysDebugPrintf("\npLineObj->cadence.startLevel = 0x%04X", cadence->startLevel);
    VpSysDebugPrintf("\npLineObj->cadence.stopLevel = 0x%04X", cadence->stopLevel);
    VpSysDebugPrintf("\npLineObj->cadence.levelStep = 0x%04X", cadence->levelStep);
}

void
VpPrintCidSeqDataType(
    VpCidSeqDataType *cidSeq)
{
    VpSysDebugPrintf("\n\npLineObj->cidSeq.pActiveCadence = %p", cidSeq->pActiveCadence);
    VpSysDebugPrintf("\npLineObj->cidSeq.pCurrentPos = %p", cidSeq->pCurrentPos);
    VpSysDebugPrintf("\npLineObj->cidSeq.index = %d", cidSeq->index);
    VpSysDebugPrintf("\npLineObj->cidSeq.timeRemain = %d", cidSeq->timeRemain);
}

void
VpPrintApiIntLineState(
    VpApiIntLineStateType *lineState)
{
    VpPrintLineStateType(lineState->currentState, "pLineObj->lineState.currentState");
    VpPrintLineStateType(lineState->previous, "pLineObj->lineState.previous");
    VpPrintLineStateType(lineState->usrCurrent, "pLineObj->lineState.usrCurrent");
    VpSysDebugPrintf("\n\rpLineObj->lineState.condition = 0x%04X", lineState->condition);
    VpSysDebugPrintf("\n\rpLineObj->lineState.calType = %d", lineState->calType);
}

void
VpPrintDigitGenDataType(
    VpDigitGenerationDataType *digitGenStruct)
{
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.dtmfOnTime = %d",
        digitGenStruct->dtmfOnTime);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.dtmfOffTime = %d",
        digitGenStruct->dtmfOffTime);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.breakTime = %d",
        digitGenStruct->breakTime);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.makeTime = %d",
        digitGenStruct->makeTime);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.flashTime = %d",
        digitGenStruct->flashTime);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.dpInterDigitTime = %d",
        digitGenStruct->dpInterDigitTime);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.dtmfHighFreqLevel = %d %d",
        digitGenStruct->dtmfHighFreqLevel[0], digitGenStruct->dtmfHighFreqLevel[1]);
    VpSysDebugPrintf("\n\rpLineObj->digitGenStruct.dtmfLowFreqLevel = %d %d",
        digitGenStruct->dtmfLowFreqLevel[0], digitGenStruct->dtmfLowFreqLevel[1]);
}

void
VpPrintDynamicInfoStruct(
    VpDeviceDynamicInfoType *dynamicInfo)
{
    VpSysDebugPrintf("\n\n\rpDevObj->dynamicInfo.lastChan = %d",
        dynamicInfo->lastChan);
    VpSysDebugPrintf("\n\rpDevObj->dynamicInfo.clkFault = %s",
        ((dynamicInfo->clkFault == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpDevObj->dynamicInfo.bat1Fault = %s",
        ((dynamicInfo->bat1Fault == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpDevObj->dynamicInfo.bat2Fault = %s",
        ((dynamicInfo->bat2Fault == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpDevObj->dynamicInfo.bat3Fault = %s",
        ((dynamicInfo->bat3Fault == TRUE) ? "TRUE" : "FALSE"));
}

void
VpPrintStaticInfoStruct(
    VpDeviceStaticInfoType *staticInfo)
{
    VpSysDebugPrintf("\n\rpDevObj->staticInfo.rcnPcn = 0x%02X 0x%02X",
        staticInfo->rcnPcn[0], staticInfo->rcnPcn[1]);
    VpSysDebugPrintf("\n\rpDevObj->staticInfo.maxChannels = %d",
        staticInfo->maxChannels);
}

/* Bit-wise values from VpDeviceBusyFlagsType */
void
VpPrintStateInformation(
    uint16 state)
{
    VpSysDebugPrintf("\n\n\rpDevObj->state = 0x%04X", state);
}

void
VpPrintDeviceProfileStruct(
    VpDeviceType deviceType,
    void *devProfileData)
{
    if (deviceType == VP_DEV_880_SERIES) {
#ifdef VP_CC_880_SERIES
        VpSysDebugPrintf("\n\n\rpDevObj->devProfileData.profVersion = %d",
            ((Vp880DeviceProfileType *)devProfileData)->profVersion);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.pcmClkRate = %d",
            ((Vp880DeviceProfileType *)devProfileData)->pcmClkRate);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.tickRate = 0x%04X",
            ((Vp880DeviceProfileType *)devProfileData)->tickRate);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.devCfg1 = 0x%02X",
            ((Vp880DeviceProfileType *)devProfileData)->devCfg1);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.clockSlot = 0x%02X",
            ((Vp880DeviceProfileType *)devProfileData)->clockSlot);
#endif
    } else if ((deviceType == VP_DEV_886_SERIES) || (deviceType == VP_DEV_887_SERIES)) {
#ifdef VP_CC_886_SERIES
        VpSysDebugPrintf("\n\n\rpDevObj->devProfileData.profVersion = %d",
            ((Vp886DeviceProfileType *)devProfileData)->profVersion);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.pcmClkRate = %d",
            ((Vp886DeviceProfileType *)devProfileData)->pcmClkRate);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.devCfg = 0x%02X",
            ((Vp886DeviceProfileType *)devProfileData)->devCfg);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.devMode = 0x%02X 0x%02X",
            ((Vp886DeviceProfileType *)devProfileData)->devMode[0],
            ((Vp886DeviceProfileType *)devProfileData)->devMode[1]);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swyv = %d",
            ((Vp886DeviceProfileType *)devProfileData)->swyv);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swzv = %d",
            ((Vp886DeviceProfileType *)devProfileData)->swzv);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swyLimit = %d",
            ((Vp886DeviceProfileType *)devProfileData)->swyLimit);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swzLimit = %d",
            ((Vp886DeviceProfileType *)devProfileData)->swzLimit);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.io2Use = 0x%02X",
            ((Vp886DeviceProfileType *)devProfileData)->io2Use);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.lowVoltOverride = %s",
            ((Vp886DeviceProfileType *)devProfileData)->lowVoltOverride ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.dialPulseCorrection = %d",
            ((Vp886DeviceProfileType *)devProfileData)->dialPulseCorrection);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swCfg = %d",
            ((Vp886DeviceProfileType *)devProfileData)->swCfg);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swOCC = %02x %02x",
            ((Vp886DeviceProfileType *)devProfileData)->swOCC[0],
            ((Vp886DeviceProfileType *)devProfileData)->swOCC[1]);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.blanking = %d",
            ((Vp886DeviceProfileType *)devProfileData)->blanking);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.absSuppCfg = 0x%01X",
            (((Vp886DeviceProfileType *)devProfileData)->absSuppCfg >> 4));
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.adaptiveRingingMaxPower = %d",
            ((Vp886DeviceProfileType *)devProfileData)->adaptiveRingingMaxPower);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.cpEnable = %s",
            ((Vp886DeviceProfileType *)devProfileData)->cpEnable ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.cpProtection = %d",
            ((Vp886DeviceProfileType *)devProfileData)->cpProtection);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.vsw = %d (%d mV)",
            ((Vp886DeviceProfileType *)devProfileData)->vsw,
            (uint16)(((Vp886DeviceProfileType *)devProfileData)->vsw) * 200);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.vbhOffset = %d",
            ((Vp886DeviceProfileType *)devProfileData)->vbhOffset);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.vbhOverhead = %d",
            ((Vp886DeviceProfileType *)devProfileData)->vbhOverhead);
#endif
    } else if (deviceType == VP_DEV_890_SERIES) {
#ifdef VP_CC_890_SERIES
        VpSysDebugPrintf("\n\n\rpDevObj->devProfileData.profVersion = %d",
            ((Vp890DeviceProfileType *)devProfileData)->profVersion);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.pcmClkRate = %d",
            ((Vp890DeviceProfileType *)devProfileData)->pcmClkRate);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.mClkMask = 0x%02X",
            ((Vp890DeviceProfileType *)devProfileData)->mClkMask);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.tickRate = 0x%04X",
            ((Vp890DeviceProfileType *)devProfileData)->tickRate);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.devCfg1 = 0x%02X",
            ((Vp890DeviceProfileType *)devProfileData)->devCfg1);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.clockSlot = 0x%02X",
            ((Vp890DeviceProfileType *)devProfileData)->clockSlot);

#ifdef VP890_FXS_SUPPORT
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.swParams = 0x%02X 0x%02X 0x%02X",
            ((Vp890DeviceProfileType *)devProfileData)->swParams[0],
            ((Vp890DeviceProfileType *)devProfileData)->swParams[1],
            ((Vp890DeviceProfileType *)devProfileData)->swParams[2]);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.timingParams = 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            ((Vp890DeviceProfileType *)devProfileData)->timingParams[0],
            ((Vp890DeviceProfileType *)devProfileData)->timingParams[1],
            ((Vp890DeviceProfileType *)devProfileData)->timingParams[2],
            ((Vp890DeviceProfileType *)devProfileData)->timingParams[3],
            ((Vp890DeviceProfileType *)devProfileData)->timingParams[4],
            ((Vp890DeviceProfileType *)devProfileData)->timingParams[5]);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.timingParamsFR = 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            ((Vp890DeviceProfileType *)devProfileData)->timingParamsFR[0],
            ((Vp890DeviceProfileType *)devProfileData)->timingParamsFR[1],
            ((Vp890DeviceProfileType *)devProfileData)->timingParamsFR[2],
            ((Vp890DeviceProfileType *)devProfileData)->timingParamsFR[3],
            ((Vp890DeviceProfileType *)devProfileData)->timingParamsFR[4],
            ((Vp890DeviceProfileType *)devProfileData)->timingParamsFR[5]);
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.peakManagement = %s",
            ((((Vp890DeviceProfileType *)devProfileData)->peakManagement == TRUE) ? "TRUE" : "FALSE"));
        VpSysDebugPrintf("\n\rpDevObj->devProfileData.lowVoltOverride = %s",
            ((((Vp890DeviceProfileType *)devProfileData)->lowVoltOverride == TRUE) ? "TRUE" : "FALSE"));
#endif  /* VP890_FXS_SUPPORT */
#endif  /* VP_CC_890_SERIES */
    }
}

void
VpPrintEventMaskStruct(
    bool isDeviceInfo,
    bool isMask,
    VpOptionEventMaskType *eventMask)
{
    char objectString[30];
    char lineOrDevString[30];

    if (isMask) {
        VpMemCpy(objectString, "EventsMask", sizeof("EventsMask"));
    } else {
        VpMemCpy(objectString, "Events", sizeof("Events"));
    }

    if (isDeviceInfo) {
        VpMemCpy(lineOrDevString, "pDevObj->device", sizeof("pDevObj->device"));
    } else {
        VpMemCpy(lineOrDevString, "pLineObj->line", sizeof("pLineObj->line"));
    }

    VpSysDebugPrintf("\n\n\r%s%s.faults = 0x%04X", lineOrDevString, objectString,
        eventMask->faults);
    VpSysDebugPrintf("\n\r%s%s.signaling = 0x%04X", lineOrDevString, objectString,
        eventMask->signaling);
    VpSysDebugPrintf("\n\r%s%s.response = 0x%04X", lineOrDevString, objectString,
        eventMask->response);
    VpSysDebugPrintf("\n\r%s%s.test = 0x%04X", lineOrDevString, objectString,
        eventMask->test);
    VpSysDebugPrintf("\n\r%s%s.process = 0x%04X", lineOrDevString, objectString,
        eventMask->process);
    VpSysDebugPrintf("\n\r%s%s.fxo = 0x%04X", lineOrDevString, objectString,
        eventMask->fxo);
    VpSysDebugPrintf("\n\r%s%s.packet = 0x%04X", lineOrDevString, objectString,
        eventMask->packet);
}

void
VpPrintCriticalFltStruct(
    VpOptionCriticalFltType *criticalFault)
{
    VpSysDebugPrintf("\n\n\rpDevObj->criticalFault.acFltDiscEn = %s",
        ((criticalFault->acFltDiscEn == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpDevObj->criticalFault.dcFltDiscEn = %s",
        ((criticalFault->dcFltDiscEn == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpDevObj->criticalFault.thermFltDiscEn = %s",
        ((criticalFault->thermFltDiscEn == TRUE) ? "TRUE" : "FALSE"));
}

void
VpPrintGetResultsOptionStruct(
    VpGetResultsOptionsType *getResultsOption)
{
    VpSysDebugPrintf("\n\n\rpDevObj->getResultsOption.chanId = %d",
        getResultsOption->chanId);
    VpSysDebugPrintf("\n\rpDevObj->getResultsOption.optionType = 0x%04X",
        getResultsOption->optionType);
    {
        uint8 byteCount;
        uint8 *pOptionData = (uint8 *)(&getResultsOption->optionData);
        uint8 optionSize = sizeof(VpGetResultsOptionsDataType);

        VpSysDebugPrintf("\n\rpDevObj->getResultsOption.optionData =");
        for (byteCount = 0; byteCount < optionSize; byteCount++) {
            if (!(byteCount % 10)) {
                VpSysDebugPrintf("\n\t");
            }
            VpSysDebugPrintf(" 0x%02X", *pOptionData);
            pOptionData++;
        }
    }
}

void
VpPrintRelGainResultsStruct(
    VpRelGainResultsType *relGainResults)
{
    VpSysDebugPrintf("\n\n\rpDevObj->relGainResults.gResult = %d",
        relGainResults->gResult);
    VpSysDebugPrintf("\n\rpDevObj->relGainResults.gxValue = 0x%04X",
        relGainResults->gxValue);
    VpSysDebugPrintf("\n\rpDevObj->relGainResults.grValue = 0x%04X",
        relGainResults->grValue);
}

void
VpPrintDeviceProfileTable(
    VpCSLACDeviceProfileTableType *devProfileTable)
{
    uint8 byteCount;

    VpSysDebugPrintf("\n\n\rpDevObj->devProfileTable.pDevProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_DEV_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pDevProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pAcProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_AC_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pAcProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pDcProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_DC_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pDcProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pRingingProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_RINGING_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pRingingProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pToneCadProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pToneCadProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pToneProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_TONE_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pToneProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pRingingCadProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pRingingCadProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pMeteringProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_METERING_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pMeteringProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pCallerIdProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_CALLERID_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pCallerIdProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pFxoConfigProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pFxoConfigProfileTable[byteCount]);
    }

    VpSysDebugPrintf("\n\rpDevObj->devProfileTable.pCustomTermProfileTable =");
    for (byteCount = 0; byteCount < VP_CSLAC_CUSTOM_TERM_PROF_TABLE_SIZE; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" %p", devProfileTable->pCustomTermProfileTable[byteCount]);
    }
}

void
VpPrintProfileTableEntry(
    VpCSLACProfileTableEntryType *profEntry)
{
    VpSysDebugPrintf("\n\n\rpDevObj->profEntry.devProfEntry = 0x%04X",
        profEntry->devProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.acProfEntry = 0x%04X",
        profEntry->acProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.dcProfEntry = 0x%04X",
        profEntry->dcProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.ringingProfEntry = 0x%04X",
        profEntry->ringingProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.ringCadProfEntry = 0x%04X",
        profEntry->ringCadProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.toneProfEntry = 0x%04X",
        profEntry->toneProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.toneCadProfEntry = 0x%04X",
        profEntry->toneCadProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.meterProfEntry = 0x%04X",
        profEntry->meterProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.cidCadProfEntry = 0x%04X",
        profEntry->cidCadProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.fxoConfigProfEntry = 0x%04X",
        profEntry->fxoConfigProfEntry);
    VpSysDebugPrintf("\n\rpDevObj->profEntry.customTermProfEntry = 0x%04X",
        profEntry->customTermProfEntry);
}

#if defined (VP890_FXS_SUPPORT) || defined (VP880_FXS_SUPPORT)
void
VpPrintPulseSpecs(
    uint8 specNumber,
    VpOptionPulseType *pulseSpecs)
{
    char specString[2] = {'\0', '\0'};

    if (specNumber == 1) {
        specString[0] = '2';
    }

    VpSysDebugPrintf("\n\n\rpDevObj->pulseSpecs%s.breakMin = %d",
        specString, pulseSpecs->breakMin);
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.breakMax = %d",
        specString, pulseSpecs->breakMax);
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.makeMin = %d",
        specString, pulseSpecs->makeMin);
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.makeMax = %d",
        specString, pulseSpecs->makeMax);
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.interDigitMin = %d",
        specString, pulseSpecs->interDigitMin);
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.flashMin = %d",
        specString, pulseSpecs->flashMin);
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.flashMax = %d",
        specString, pulseSpecs->flashMax);
#ifdef EXTENDED_FLASH_HOOK
    VpSysDebugPrintf("\n\rpDevObj->pulseSpecs%s.onHookMin = %d",
        specString, pulseSpecs->onHookMin);
#endif

}

void
VpPrintDPStateMachine(
    uint8 stateMachineNum,
    VpDialPulseDetectType  *dpStruct)
{
    char specString[2] = {'\0', '\0'};
    char *dpState[4] = {
        "VP_DP_DETECT_STATE_LOOP_OPEN",
        "VP_DP_DETECT_STATE_LOOP_CLOSE",
        "VP_DP_DETECT_STATE_IDLE",
        "UNKNOWN"
    };
    uint8 stateIndex = (dpStruct->state < 3) ? dpStruct->state : 3;

    if (stateMachineNum == 1) {
        specString[0] = '2';
    }

    VpSysDebugPrintf("\n\n\rpLineObj->dpStruct%s.digits = %d",
        specString, dpStruct->digits);
    VpSysDebugPrintf("\n\rpLineObj->dpStruct%s.lo_time = %d",
        specString, dpStruct->lo_time);
    VpSysDebugPrintf("\n\rpLineObj->dpStruct%s.lc_time = %d",
        specString, dpStruct->lc_time);
    VpSysDebugPrintf("\n\rpLineObj->dpStruct%s.state = %d:%s",
        specString, dpStruct->state, dpState[stateIndex]);
    VpSysDebugPrintf("\n\rpLineObj->dpStruct%s.hookSt = %s",
        specString, ((dpStruct->hookSt == TRUE) ? "TRUE" : "FALSE"));
    VpSysDebugPrintf("\n\rpLineObj->dpStruct%s.signalingData = %d\n\r",
        specString, dpStruct->signalingData);
}

#endif  /* defined (VP890_FXS_SUPPORT) || defined (VP880_FXS_SUPPORT) */

void
VpPrintDeviceTimers(
    uint16 devTimer[VP_DEV_TIMER_LAST])
{
    uint8 byteCount;
    char *timerNames[VP_DEV_TIMER_LAST] = {
        "VP_DEV_TIMER_TESTLINE", "VP_DEV_TIMER_CLKFAIL", "VP_DEV_TIMER_ABSCAL",
        "VP_DEV_TIMER_LP_CHANGE", "VP_DEV_TIMER_ABV_CAL", "VP_DEV_TIMER_ENTER_RINGING",
        "VP_DEV_TIMER_EXIT_RINGING", "VP_DEV_TIMER_WB_MODE_CHANGE"
    };

    VpSysDebugPrintf("\n\rpDevObj->devTimer =");
    for (byteCount = 0; byteCount < VP_DEV_TIMER_LAST; byteCount++) {
        if (!(byteCount % 10)) {
            VpSysDebugPrintf("\n\t");
        }
        VpSysDebugPrintf(" 0x%04X", devTimer[byteCount]);
        if (devTimer[byteCount] & VP_ACTIVATE_TIMER) {
            VpSysDebugPrintf(" - %s is Active\n\t", timerNames[byteCount]);
        }
    }
}

#if defined (VP_CC_886_SERIES)
void
VpPrintTimerQueue(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes)
{
    int16 idx;

    VpSysDebugPrintf("\n\n\rpDevObj->timerQueue (front = %d, timestamp = %lu):", pInfo->index, pInfo->timestamp);
    for (idx = 0; idx < pInfo->numNodes; idx++) {
        VpSysDebugPrintf("\n\r\t%2d: active %d, next %2d, id %4d, ch %d, dur %8lu, hndl %lu",
                idx, pNodes[idx].active, pNodes[idx].next, pNodes[idx].id, pNodes[idx].channelId, pNodes[idx].duration, pNodes[idx].handle);
    }
}

void
VpPrintPulseDecodeData(
    VpPulseDecodeDataType *pPulseDecode)
{
    char *stateNames[3] = {
        "VP_PULSE_DECODE_STATE_IDLE",
        "VP_PULSE_DECODE_STATE_BREAK",
        "VP_PULSE_DECODE_STATE_MAKE"
    };
    VpSysDebugPrintf("\n\n\rpLineObj->pulseDecodeData.state = %s", stateNames[pPulseDecode->state]);
    if (pPulseDecode->state != VP_PULSE_DECODE_STATE_IDLE) {
        VpSysDebugPrintf("\n\rpLineObj->pulseDecodeData.prevTimestamp = %d", pPulseDecode->prevTimestamp);
        VpSysDebugPrintf("\n\rpLineObj->pulseDecodeData.digitCount = %d", pPulseDecode->digitCount);
        VpSysDebugPrintf("\n\rpLineObj->pulseDecodeData.digitValidSpec1 = %s", pPulseDecode->digitValidSpec1 ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->pulseDecodeData.digitValidSpec2 = %s", pPulseDecode->digitValidSpec2 ? "TRUE" : "FALSE");
    }
}

void
VpPrintSlacBufData(
    VpSlacBufDataType *pSlacBuf)
{
    VpSysDebugPrintf("\n\n\rpDevObj->slacBufData.buffering = %s", pSlacBuf->buffering ? "TRUE" : "FALSE");
    VpSysDebugPrintf("\n\rpDevObj->slacBufData.wrtLen = %d", pSlacBuf->wrtLen);
    if (pSlacBuf->wrtLen > 0) {
        uint8 idx;
        VpSysDebugPrintf("\n\rpDevObj->slacBufData.wrtBuf = {");
        for (idx = 0; idx < pSlacBuf->wrtLen; idx++) {
            if (idx % 12 == 0) {
                VpSysDebugPrintf("\n\r%8s", "");
            }
            VpSysDebugPrintf("0x%02X ", pSlacBuf->wrtBuf[idx]);
        }
        VpSysDebugPrintf("\n\r%4s}", "");
    } else {
        VpSysDebugPrintf("\n\rpDevObj->slacBufData.wrtBuf = { }");
    }
    VpSysDebugPrintf("\n\rpDevObj->slacBufData.firstEc = 0x%02X", pSlacBuf->firstEc);
    VpSysDebugPrintf("\n\rpDevObj->slacBufData.currentEc = 0x%02X", pSlacBuf->currentEc);
}
#endif /* defined (VP_CC_886_SERIES) */

#endif  /* defined (VP_CC_880_SERIES) || defined (VP_CC_890_SERIES) || defined (VP_CC_886_SERIES) */
#endif  /* VP_CC_DEBUG_SELECT & VP_DBG_INFO */

#endif /* VP_DEBUG */

