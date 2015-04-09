/** \file vp886_init.c
 * vp886_init.c
 *
 * This file contains the line and device initialization functions for
 * the Vp886 device API. This includes the Vp886 and Vp887 series.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11615 $
 * $LastChangedDate: 2014-10-21 13:27:40 -0500 (Tue, 21 Oct 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"


static const uint8 VP886_INTERNAL_DEFAULT_AC_PROFILE[] = {
    /* AC FXS RF100 600R Normal Coefficients */
    0xAD, 0x00, 0xAC, 0x4C, 0x01, 0x49, 0xCA, 0xEA, 0x98, 0xBA, 0xEB, 0x2A,
    0x2C, 0xB5, 0x25, 0xAA, 0x24, 0x2C, 0x3D, 0x9A, 0xAA, 0xBA, 0x27, 0x9F,
    0x01, 0x8A, 0x2D, 0x01, 0x2B, 0xB0, 0x5A, 0x33, 0x24, 0x5C, 0x35, 0xA4,
    0x5A, 0x3D, 0x33, 0xB6, 0x88, 0x3A, 0x10, 0x3D, 0x3D, 0xB2, 0xA7, 0x6B,
    0xA5, 0x2A, 0xCE, 0x2A, 0x8F, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
    0x00, 0x86, 0x2A, 0x42, 0x22, 0x4B, 0x1C, 0xA3, 0xA8, 0xFF, 0x8F, 0xAA,
    0xF5, 0x9F, 0xBA, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

static const uint8 VP887_INTERNAL_DEFAULT_AC_PROFILE[] = {
    /* AC FXS RF100 600R Normal Coefficients */
    0xAD, 0x00, 0xAC, 0x4C, 0x01, 0x49, 0xCA, 0xEA, 0x98, 0xBA, 0xEB, 0x2A,
    0x2C, 0xB5, 0x25, 0xAA, 0x24, 0x2C, 0x3D, 0x9A, 0xAA, 0xBA, 0x27, 0x9F,
    0x01, 0x8A, 0x2D, 0x01, 0x2B, 0xB0, 0x5A, 0x33, 0x24, 0x5C, 0x35, 0xA4,
    0x5A, 0x3D, 0x33, 0xB6, 0x88, 0x3A, 0x10, 0x3D, 0x3D, 0xB2, 0xA7, 0x6B,
    0xA5, 0x2A, 0xCE, 0x2A, 0x8F, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
    0x00, 0x86, 0x2A, 0x42, 0x22, 0x4B, 0x1C, 0xA3, 0xA8, 0xFF, 0x8F, 0xAA,
    0xF5, 0x9F, 0xBA, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

static const uint8 VP886_INTERNAL_DEFAULT_DC_PROFILE[] = {
/* DC FXS Parameters - 20mA Current Feed */
/* Profile Header ---------------------------------------------------------- */
    0x0C, 0x01, 0x00,       /* DC Profile for ZL886xx device */
    0x0B,                   /* Profile length = 11 + 4 = 15 bytes */
    0x01,                   /* Profile format version = 1 */
    0x03,                   /* MPI section length = 3 bytes */
/* Raw MPI Section --------------------------------------------------------- */
    0xC6, 0x8D, 0xE2,       /* DC Feed Parameters: ir_overhead = 100 ohm; */
                            /* VOC = 45 V; LI = 50 ohm; ILA = 20 mA */
                            /* Battery Switch Offset Voltage = 4.02 V */
/* Formatted Parameters Section -------------------------------------------- */
    0x1C,                   /* Switch Hook Threshold = 11 mA */
                            /* Ground-Key Threshold = 18 mA */
    0x86,                   /* Switch Hook Debounce = 12 ms */
                            /* Ground-Key Debounce = 16 ms */
    0x58,                   /* Low-Power Switch Hook Hysteresis = 2 V */
                            /* Ground-Key Hysteresis = 6 mA */
                            /* Switch Hook Hysteresis = 2 mA */
    0x80,                   /* Low-Power Switch Hook Threshold = 22 V */
    0x98,                   /* Reserved */
    0x00                    /* RPTC = 0 ohms */
};

static const uint8 VP887_INTERNAL_DEFAULT_DC_PROFILE[] = {
/* DC FXS Parameters - 20mA Current Feed */
/* Profile Header ---------------------------------------------------------- */
    0x0D, 0x01, 0x00,       /* DC Profile for ZL887xx device */
    0x0C,                   /* Profile length = 12 + 4 = 16 bytes */
    0x01,                   /* Profile format version = 1 */
    0x03,                   /* MPI section length = 3 bytes */
/* Raw MPI Section --------------------------------------------------------- */
    0xC6, 0xCE, 0x02,       /* DC Feed Parameters: ir_overhead = 200 ohm; */
                            /* VOC = 45 V; LI = 100 ohm; VAS = 8.78 V; ILA = 20 mA */
                            /* Maximum Voice Amplitude = 2.93 V */
/* Formatted Parameters Section -------------------------------------------- */
    0x1A,                   /* Switch Hook Threshold = 9 mA */
                            /* Ground-Key Threshold = 18 mA */
    0x86,                   /* Switch Hook Debounce = 12 ms */
                            /* Ground-Key Debounce = 16 ms */
    0x58,                   /* Low-Power Switch Hook Hysteresis = 2 V */
                            /* Ground-Key Hysteresis = 6 mA */
                            /* Switch Hook Hysteresis = 2 mA */
    0x40,                   /* Low-Power Switch Hook Threshold = 18 V */
    0x04,                   /* Floor Voltage = -25 V */
    0x98,                   /* Reserved */
    0x32                    /* RPTC = 50 ohms */
};

static const uint8 VP886_INTERNAL_DEFAULT_RING_PROFILE[] = {
/* 45vrms 25Hz Fixed Ringing, AC trip */
/* Profile Header ---------------------------------------------------------- */
    0x0C, 0x04, 0x00,       /* Ringing Profile for ZL88602 device */
    0x12,                   /* Profile length = 18 + 4 = 22 bytes */
    0x01,                   /* Profile format version = 1 */
    0x0C,                   /* MPI section length = 12 bytes */
/* Raw MPI Section --------------------------------------------------------- */
    0xC0, 0x08, 0x00, 0x00, /* Ringing Generator Parameters: */
          0x00, 0x44, 0x34, /* 24.9 Hz Sine; 1.41 CF; 63.00 Vpk; 0.00 V bias */
          0x3A, 0x00, 0x00, /* Ring trip cycles = 1; RTHALFCYC = 0 */
          0x00, 0x00,
/* Formatted Parameters Section -------------------------------------------- */
    0xA8,                   /* Ring Trip Method = AC; Threshold = 20.0 mA */
    0x00,                   /* Ringing Current Limit = 50 mA */
    0x4C,                   /* Fixed; Max Ringing Supply = 65 V */
    0x00                    /* Balanced; Ring Cadence Control Disabled */
};

static const uint8 VP887_INTERNAL_DEFAULT_RING_PROFILE[] = {
/* Ringing 25Hz 45Vrms Fixed, AC Trip */
/* Profile Header ---------------------------------------------------------- */
    0x0D, 0x04, 0x00,       /* Ringing Profile for ZL887xx device */
    0x12,                   /* Profile length = 18 + 4 = 22 bytes */
    0x01,                   /* Profile format version = 1 */
    0x0C,                   /* MPI section length = 12 bytes */
/* Raw MPI Section --------------------------------------------------------- */
    0xC0, 0x08, 0x00, 0x00, /* Ringing Generator Parameters: */
          0x00, 0x44, 0x34, /* 24.9 Hz Sine; 1.41 CF; 63.00 Vpk; 0.00 V bias */
          0x3A, 0x00, 0x00, /* Ring trip cycles = 1; RTHALFCYC = 0 */
          0x00, 0x00,
/* Formatted Parameters Section -------------------------------------------- */
    0xAA,                   /* Ring Trip Method = AC; Threshold = 21.0 mA */
    0x00,                   /* Ringing Current Limit = 50 mA */
    0x4C,                   /* Fixed; Max Ringing Supply = 65 V */
    0x00                    /* Balanced; Ring Cadence Control Disabled */
};


/**< Vp886 Initalization Function Prototypes */
static VpStatusType
Vp886MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pLineObj,
    VpDevCtxType *pDevCtx);

static VpStatusType
Vp886MakeLineObjectInt(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    Vp886LineObjectType *pLineObj,
    VpDevCtxType *pDevCtx);

static VpStatusType
Vp886InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile);

static void
Vp886InitDeviceData(
    VpDevCtxType *pDevCtx);

static VpStatusType
Vp886InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp886InitLineInt(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile,
    bool initOptions);

static void
Vp886InitLineRegisters(
    VpLineCtxType *pLineCtx,
    bool initDevice);

static VpStatusType
Vp886InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile);

static VpStatusType
Vp886ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp886ConfigLineInt(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile,
    bool initDevice);

static VpStatusType
Vp886ConfigLineFxs(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    bool initDevice);

static VpStatusType
Vp886FreeRun(
    VpDevCtxType *pDevCtx,
    VpFreeRunModeType freeRunMode);

static bool
Vp886InitDeviceAlarmCheck(
    VpDevCtxType *pDevCtx,
    uint16* eventData);

static void
Vp886SubDefaultProfiles(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType *ppAcProfile,
    VpProfilePtrType *ppDcProfile,
    VpProfilePtrType *ppRingProfile);

void
Vp886SetSwTimingParams(
    VpDevCtxType *pDevCtx);

static void
Vp886ChangeDeviceType(
    VpDevCtxType *pDevCtx);

#ifdef VP886_INCLUDE_MPI_QUICK_TEST
static VpStatusType
Vp886QuickMpiTest(
    VpDevCtxType *pDevCtx);

static VpStatusType
Vp886QuickMpiTest1Ch(
    VpDevCtxType *pDevCtx);
#endif /* VP886_INCLUDE_MPI_QUICK_TEST */


/** VpMakeVp886DeviceObject()
  Implements VpMakeDeviceObject() to initialize a Vp886 device object and device
  context.

  This function must be run before any other API function for each device.

  See the VP-API-II Reference Guide for more details on VpMakeDeviceObject().
*/
VpStatusType
VpMakeVp886DeviceObject(
    VpDevCtxType *pDevCtx,
    Vp886DeviceObjectType *pDevObj)
{
    VpMemSet(pDevObj, 0, sizeof(Vp886DeviceObjectType));

    pDevObj->staticInfo.maxChannels = VP886_MAX_NUM_CHANNELS;

    Vp886SetDefaultCal(pDevObj, 0);
    Vp886SetDefaultCal(pDevObj, 1);
    pDevObj->calData[0].valid = FALSE;
    pDevObj->calData[1].valid = FALSE;
    pDevObj->isPowerLimited[0] = FALSE;
    pDevObj->isPowerLimited[1] = FALSE;

    pDevObj->ecVal = VP886_EC_GLOBAL;

    pDevObj->slacBufData.buffering = FALSE;

#ifdef VP886_TIMER_INTERRUPT_OVERRIDE
    pDevObj->timerOverride = TRUE;
#else
    pDevObj->timerOverride = FALSE;
#endif

#ifdef VP_DEBUG
    pDevObj->debugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;
#endif

    /* Initialize the device context and link it to this device object. */
    return VpMakeVp886DeviceCtx(pDevCtx, pDevObj);

}   /* VpMakeVp886DeviceObject() */


/** VpMakeVp886DeviceCtx()
  Implements VpMakeDeviceCtx() to initialize a Vp886 device context. This is
  also used internally by VpMakeVp886DeviceObject().

  Links the device context to an already initialized device object (pDevObj) and
  sets up the function pointer table in the device context.

  See the VP-API-II Reference Guide for more details on VpMakeDeviceCtx().
*/
VpStatusType
VpMakeVp886DeviceCtx(
    VpDevCtxType *pDevCtx,
    Vp886DeviceObjectType *pDevObj)
{
    uint8 channelCount, maxChan;
    ApiFunctions *pFunc;

    if((pDevCtx == VP_NULL) || (pDevObj == VP_NULL)) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Initialize Device context */
    pDevCtx->pDevObj = pDevObj;

    /* Initialize all of the line context pointers to null in the device context */
    maxChan = pDevObj->staticInfo.maxChannels;
    for (channelCount = 0; channelCount < maxChan; channelCount++) {
        pDevCtx->pLineCtx[channelCount] = VP_NULL;
    }

    pFunc = &pDevCtx->funPtrsToApiFuncs;

    /* Init functions (vp886_init.c) */
    pFunc->MakeLineObject = Vp886MakeLineObject;
    pFunc->InitDevice = Vp886InitDevice;
    pFunc->InitLine = Vp886InitLine;
    pFunc->ConfigLine = Vp886ConfigLine;
    pFunc->InitProfile = Vp886InitProfile;
    pFunc->FreeRun = Vp886FreeRun;
#if defined (VP_CSLAC_SEQ_EN)
    pFunc->InitRing = Vp886InitRing;
    pFunc->InitCid = Vp886InitCid;
    pFunc->InitMeter = Vp886InitMeter;
#endif /* defined (VP_CSLAC_SEQ_EN) */

    /* Control functions (vp886_control.c) */
    pFunc->SetLineState = Vp886SetLineState;
    pFunc->SetLineTone = Vp886SetLineTone;
#ifdef CSLAC_GAIN_RELATIVE
    pFunc->SetRelGain = Vp886SetRelGain;
#endif /* CSLAC_GAIN_RELATIVE */
    pFunc->SetOption = Vp886SetOption;
    pFunc->DeviceIoAccess = Vp886DeviceIoAccess;
    pFunc->LineIoAccess = Vp886LineIoAccess;
    pFunc->SetRelayState = Vp886SetRelayState;
    pFunc->DtmfDigitDetected = Vp886DtmfDigitDetected;
#if (VP886_USER_TIMERS > 0)
    pFunc->GenTimerCtrl = Vp886GenTimerCtrl;
#endif
    pFunc->ShutdownDevice = Vp886ShutdownDevice;

#if defined (VP_CSLAC_SEQ_EN)
    /* Sequencer-dependent functions (vp886_seq.c) */
    pFunc->SendSignal = Vp886SendSignal;
    pFunc->SendCid = Vp886SendCid;
    pFunc->ContinueCid = Vp886ContinueCid;
    pFunc->StartMeter = Vp886StartMeter;
#endif /* defined (VP_CSLAC_SEQ_EN) */

    /* Query functions (vp886_query.c) */
    pFunc->GetLineStatus = VpCSLACGetLineStatus;
    pFunc->GetDeviceStatus = Vp886GetDeviceStatus;
    pFunc->GetOption = Vp886GetOption;
    pFunc->GetOptionImmediate = Vp886GetOptionImmediate;
    pFunc->GetLoopCond = Vp886GetLoopCond;
    pFunc->Query = Vp886Query;
    pFunc->QueryImmediate = Vp886QueryImmediate;
#ifdef VP886_INCLUDE_TESTLINE_CODE
    /* Technically a "Query" function, but used only for Line Test purposes */
    pFunc->GetRelayState = Vp886GetRelayState;
#endif /* VP886_INCLUDE_TESTLINE_CODE */

    /* Event functions (vp886_event.c) */
    pFunc->ApiTick = Vp886ApiTick;
#ifndef VP886_SIMPLE_POLLED_MODE
    pFunc->VirtualISR = Vp886VirtualISR;
#endif /* VP886_SIMPLE_POLLED_MODE */
    pFunc->GetEvent = Vp886GetEvent;
    pFunc->FlushEvents = Vp886FlushEvents;
    pFunc->GetResults = Vp886GetResults;

#ifdef VP886_INCLUDE_TESTLINE_CODE
    /* Linetest functions (vp886_testline.c) */
    pFunc->TestLine = Vp886TestLine;
#endif /* VP886_INCLUDE_TESTLINE_CODE */

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
    /* Debug functions */
    pFunc->RegisterDump = Vp886RegisterDump;
    pFunc->ObjectDump = Vp886ObjectDump;
#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_INFO) */

    /* Calibration functions (vp886_calibration_common.c) */
#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
    pFunc->CalLine = Vp886CalLine;
#if !defined(VP_REDUCED_API_IF) || defined (VP_ENABLE_PROD_TEST)
    pFunc->CalCodec = Vp886CalCodec;
#endif /* !defined(VP_REDUCED_API_IF) || defined (VP_ENABLE_PROD_TEST) */
#endif /* VP_CSLAC_RUNTIME_CAL_ENABLED */
    pFunc->Cal = Vp886Cal;

    /* HAL Wrapper functions (vp886_slac.c) */
    pFunc->SlacBufStart = Vp886SlacBufStart;
    pFunc->SlacBufSend = Vp886SlacBufSend;
    pFunc->SlacRegWrite = Vp886SlacRegWrite;
    pFunc->SlacRegRead = Vp886SlacRegRead;

    return VP_STATUS_SUCCESS;
}


/** Vp886MakeLineObject()
  Implements VpMakeLineObject() to initialize a Vp886 line object and line
  context, and link them to a device context.

  This function must be run before any other API function for each line.

  See the VP-API-II Reference Guide for more details on VpMakeLineObject().
*/
VpStatusType
Vp886MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pVoidLineObj,
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pVoidLineObj;
    VpStatusType status;

    if (channelId >= pDevObj->staticInfo.maxChannels) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid channelId %d. Max channels on this device: %d",
            channelId, pDevObj->staticInfo.maxChannels));
        return VP_STATUS_INVALID_LINE;
    }

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886MakeLineObject");

    status = Vp886MakeLineObjectInt(termType, channelId, pLineCtx, pLineObj, pDevCtx);
#ifdef VP_DEBUG
    pLineObj->debugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;
#endif

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886MakeLineObject");
    return status;
}


/** Vp886MakeLineObjectInt()
  Used by Vp886MakeLineObject() and Vp886InitLineInt() to reset and initialize
  a line object.
*/
VpStatusType
Vp886MakeLineObjectInt(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    Vp886LineObjectType *pLineObj,
    VpDevCtxType *pDevCtx)
{
#ifdef VP_DEBUG
    uint32 debugSelectMask;
#endif

#ifdef VP_DEBUG
    /* Save the debug select mask so that it is not reset by VpInitLine() */
    debugSelectMask = pLineObj->debugSelectMask;
#endif

    VpMemSet(pLineObj, 0, sizeof(Vp886LineObjectType));

#ifdef VP_DEBUG
    pLineObj->debugSelectMask = debugSelectMask;
#endif

    switch (termType) {
        case VP_TERM_FXS_GENERIC:
            pLineObj->isFxs = TRUE;
            break;
        case VP_TERM_FXS_LOW_PWR:
            pLineObj->isFxs = TRUE;
            break;
        default:
            return VP_STATUS_ERR_VTD_CODE;
    }

    pLineCtx->pLineObj = pLineObj;
    pLineCtx->pDevCtx = pDevCtx;

    pDevCtx->pLineCtx[channelId] = pLineCtx;
    pLineObj->channelId = channelId;
    pLineObj->termType = termType;
    pLineObj->ecVal = ((channelId == 0) ? VP886_EC_1 : VP886_EC_2);

    return VP_STATUS_SUCCESS;
}

/** Vp886InitDevice()
  Begins the initialization of the device and all lines associated with this
  device.

  This function performs error checking and as much immediate initialization as
  possible.  The profiles are mostly applied immediately, along with some initial
  register settings and device object values.  The device bring-up steps and
  calibration are then performed by a timer- and measurement-driven state machine.
  At the end of the initialization and calibration sequence, the state machine
  will generate a VP_DEV_EVID_DEV_INIT_CMP event.

  The device profile is required.  Other profiles will be substituted with safe
  defaults if not provided.
*/
VpStatusType
Vp886InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpProfilePtrType pProf;
    uint8 mpiLen;
    uint8 shutdownState = VP886_R_STATE_SS_SHUTDOWN;
    VpStatusType status;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886InitDevice");

    /* Device profile is required. */
    if (pDevProfile == VP_PTABLE_NULL) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Device profile is required."));
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
        return VP_STATUS_ERR_PROFILE;
    }

    /* Reset device object variables */
    Vp886InitDeviceData(pDevCtx);

    /* Clear the MPI buffer before trying to access the device, in case there
       was a command issued earlier that is still expecting data.  Sending a
       stream of no-ops will satisfy the device's need for more data, and leave
       it in a state where it is ready to accept a command.  We're calling the
       common ClearMPIBuffer function twice because the longest command length
       for ZL880 is longer than it was for other CSLAC devices. */
    VpCSLACClearMPIBuffer(pDevObj->deviceId);
    VpCSLACClearMPIBuffer(pDevObj->deviceId);

    /* Hardware Reset the part, but first make sure the switchers are shut down
       to avoid catching a switcher duty cycle on with the reset signal. */
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, &shutdownState);
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_NOOP_WRT, 0, VP_NULL);
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_NOOP_WRT, 0, VP_NULL);
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_NOOP_WRT, 0, VP_NULL);
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_HWRESET_WRT, 0, VP_NULL);

    /* Read and verify the revision and product codes.  Need to know this now
       so we can verify that the profiles are targeted for the correct device. */
    status = Vp886InitDevicePcnRcn(pDevCtx);
    if (status != VP_STATUS_SUCCESS) {
        pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
        return status;
    }

#ifdef VP886_INCLUDE_MPI_QUICK_TEST
    /* Run a quick test of the MPI interface and HAL layer code */
    status = Vp886QuickMpiTest(pDevCtx);
    if (status != VP_STATUS_SUCCESS) {
        pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
        return status;
    }
#endif /* VP886_INCLUDE_MPI_QUICK_TEST */

    /* Check for valid profile arguments. */
    if ((Vp886GetProfileArg(pDevCtx, VP_PROFILE_DEVICE, &pDevProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_AC, &pAcProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_DC, &pDcProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_RING, &pRingProfile) != VP_STATUS_SUCCESS)
        /* Assuming no FXO support. So don't check the FXO related profiles */
    ) {
        pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
        return VP_STATUS_ERR_PROFILE;
    }

    /* Make sure the Device Ctxt Device Type, and the Device Profile Device Type
       are in agreement */
    if (pDevProfile[VP_PROFILE_TYPE_MSB] != pDevCtx->deviceType) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Device profile device type %02X does not match specified device type %02X",
            pDevProfile[VP_PROFILE_TYPE_MSB], pDevCtx->deviceType));
        pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
        return VP_STATUS_ERR_PROFILE;
    }

    /* Validate and store the PCM Clock rate from the device profile */
    pDevObj->devProfileData.devCfg = pDevProfile[VP886_DEV_PROFILE_DEV_CFG_DATA0];
    switch(pDevObj->devProfileData.devCfg & VP886_R_DEVCFG_CLKSEL) {
        case VP886_R_DEVCFG_CLKSEL_1536:
            pDevObj->devProfileData.pcmClkRate = 1536;
            break;
        case VP886_R_DEVCFG_CLKSEL_1544:
            pDevObj->devProfileData.pcmClkRate = 1544;
            break;
        case VP886_R_DEVCFG_CLKSEL_2048:
            pDevObj->devProfileData.pcmClkRate = 2048;
            break;
        case VP886_R_DEVCFG_CLKSEL_1024:
            pDevObj->devProfileData.pcmClkRate = 1024;
            break;
        case VP886_R_DEVCFG_CLKSEL_3072:
            pDevObj->devProfileData.pcmClkRate = 3072;
            break;
        case VP886_R_DEVCFG_CLKSEL_3088:
            pDevObj->devProfileData.pcmClkRate = 3088;
            break;
        case VP886_R_DEVCFG_CLKSEL_4096:
            pDevObj->devProfileData.pcmClkRate = 4096;
            break;
        case VP886_R_DEVCFG_CLKSEL_6144:
            pDevObj->devProfileData.pcmClkRate = 6144;
            break;
        case VP886_R_DEVCFG_CLKSEL_6176:
            pDevObj->devProfileData.pcmClkRate = 6176;
            break;
        case VP886_R_DEVCFG_CLKSEL_8192:
            pDevObj->devProfileData.pcmClkRate = 8192;
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid PCM Clock Rate selection in device profile, 0x%02X",
                pDevObj->devProfileData.devCfg));
            pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
            return VP_STATUS_ERR_PROFILE;
    }
    if (VP886_IS_SF(pDevObj)) {
        if (pDevObj->devProfileData.pcmClkRate % 512 != 0) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid PCM Clock Rate selection in device profile, %dkHz",
                pDevObj->devProfileData.pcmClkRate));
            pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
            return VP_STATUS_ERR_PROFILE;
        }
    }
    
    /* The device profile is good. Save a pointer to it. This pointer
       is used only in the first state of the Vp886InitDevice() state
       machine. The pointer cannot be guaranteed to be valid after that
       and should not be used elsewhere. */
    pDevObj->pDevProfile = pDevProfile;

    /* Substitute in default profiles for any that are not provided */
    Vp886SubDefaultProfiles(pDevCtx, &pAcProfile, &pDcProfile, &pRingProfile);

    /* Save a pointer to the AC profile. This pointer
       is used only in the first state of the Vp886InitDevice() state
       machine. The pointer cannot be guaranteed to be valid after that
       and should not be used elsewhere. */
    pDevObj->pAcProfile = pAcProfile;

    /* Save a pointer to the DC profile. This pointer
       is used only in the first state of the Vp886InitDevice() state
       machine. The pointer cannot be guaranteed to be valid after that
       and should not be used elsewhere. */
    pDevObj->pDcProfile = pDcProfile;

    /* Save a pointer to the Ring profile. This pointer
       is used only in the first state of the Vp886InitDevice() state
       machine. The pointer cannot be guaranteed to be valid after that
       and should not be used elsewhere. */
    pDevObj->pRingProfile = pRingProfile;


    /* Store configuration and registers from device profile */
    mpiLen = pDevProfile[VP_PROFILE_MPI_LEN];
    pProf = pDevProfile + VP_PROFILE_MPI_LEN + mpiLen + 1;

    /* Store the I/O 2 use from the Dev profile */
    pDevObj->devProfileData.io2Use = *pProf;
    pProf += 1;

    /* Store dial pulse correction value from the Dev profile */
    pDevObj->devProfileData.dialPulseCorrection = *pProf;
    pDevObj->devProfileData.dialPulseCorrection &= VP886_DEV_PROFILE_PD_CORR_MASK;
    pDevObj->devProfileData.dialPulseCorrection >>= 4;
    /* Determine low voltage override from the Dev profile */
    pDevObj->devProfileData.swCfg = (*pProf & VP886_DEV_PROFILE_SW_CONF_MASK);
    if (VP886_IS_TRACKER(pDevObj)) {
        switch (pDevObj->devProfileData.swCfg) {
            case VP886_DEV_PROFILE_SW_CONF_BB:
            case VP886_DEV_PROFILE_SW_CONF_FB100:
            case VP886_DEV_PROFILE_SW_CONF_IB100:
            case VP886_DEV_PROFILE_SW_CONF_IB100_5IN:
                pDevObj->devProfileData.lowVoltOverride = TRUE;
                break;
            case VP886_DEV_PROFILE_SW_CONF_FB150:
            case VP886_DEV_PROFILE_SW_CONF_IB150:
                pDevObj->devProfileData.lowVoltOverride = FALSE;
                break;
            default:
                VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid switcher configuration 0x%02X in device profile", pDevObj->devProfileData.swCfg));
                pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
                Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
                return VP_STATUS_ERR_PROFILE;
        }
    } else {
        switch (pDevObj->devProfileData.swCfg) {
            case VP886_DEV_PROFILE_SW_CONF_BB:
            case VP886_DEV_PROFILE_SW_CONF_ABS100:
            case VP886_DEV_PROFILE_SW_CONF_ABS120:
                pDevObj->devProfileData.lowVoltOverride = TRUE;
                break;
            default:
                VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid switcher configuration 0x%02X in device profile", pDevObj->devProfileData.swCfg));
                pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
                Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
                return VP_STATUS_ERR_PROFILE;
        }
    }
    pProf += 1;

    /* Save Switcher Over-Current Count */
    pDevObj->devProfileData.swOCC[0] = pProf[0];
    pDevObj->devProfileData.swOCC[1] = pProf[1];
    pProf += 2;

    /* Save ABS Supply Configuration */
    pDevObj->devProfileData.absSuppCfg = *pProf;
    /* Correct a problem from Profile Wizard P2.3.2 and earlier where both supply
       bits can be set to slave mode without the main bit being set for slave
       mode. */
    if ((pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Y_SLAVE) &&
        (pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Z_SLAVE))
    {
        pDevObj->devProfileData.absSuppCfg &= ~VP886_DEV_PROFILE_ABS_SUPP_CFG_MODE;
        pDevObj->devProfileData.absSuppCfg |= VP886_DEV_PROFILE_ABS_SUPP_CFG_SLAVE;
    }
    switch (pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_MODE) {
        case VP886_DEV_PROFILE_ABS_SUPP_CFG_SINGLE:
        case VP886_DEV_PROFILE_ABS_SUPP_CFG_SLAVE:
        case VP886_DEV_PROFILE_ABS_SUPP_CFG_MASTER:
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Illegal ABS Supply Configuration"));
            pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
            return VP_STATUS_ERR_PROFILE;
    }
    /* Save Leading Edge Blanking Window */
    pDevObj->devProfileData.blanking = (*pProf & VP886_DEV_PROFILE_BLANKING_USE_BITS);
    pProf += 1;

    /* Grab the Free Run Switching regulator timing parameters from the Dev profile */
    VpMemCpy(pDevObj->swTimingParamsFR, pProf, VP886_R_SWTIMING_LEN);
    pProf += VP886_R_SWTIMING_LEN;

    if (VP886_IS_ABS(pDevObj)) {
        /* Store the SWY Voltage from the Dev profile. Applies to ABS only */
        pDevObj->devProfileData.swyv = *pProf;
        pDevObj->swyv = *pProf;
        pProf += 1;

        /* Store the SWZ Voltage from the Dev profile. Applies to ABS only */
        pDevObj->devProfileData.swzv = *pProf;
        pDevObj->swzv = *pProf;
        pProf += 1;
    }

    /* Disable Adaptive Ringing by default (override whatever the profile says) */
    pDevObj->devProfileData.adaptiveRingingMaxPower = VP_ADAPTIVE_RINGING_DISABLED;
    pProf += 1;

    /* Save the low power switcher timings (version 2+) */
    if (pDevProfile[VP_PROFILE_VERSION] < 2) {
        pDevObj->useLowPowerTiming = FALSE;
    } else {
        pDevObj->useLowPowerTiming = TRUE;
        pDevObj->swTimingParamsLP[0] = pProf[0];
        pDevObj->swTimingParamsLP[1] = pProf[1];
        pProf += 2;
    }

    /* Version 3+ */
    if (pDevProfile[VP_PROFILE_VERSION] < 3) {
        /* Defaults */
        pDevObj->devProfileData.swyLimit = 0;
        pDevObj->devProfileData.swzLimit = 0;
        pDevObj->devProfileData.cpEnable = TRUE;
        pDevObj->devProfileData.cpProtection = VP886_CP_PROT_CYCLE_SKIP;
        if (pDevObj->stateInt & VP886_SHARED_SUPPLY) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Shared supply device detected, but device profile does not support shared supply."));
            pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
            return VP_STATUS_ERR_PROFILE;
        }
    } else {
        /* Save the SWY and SWZ limit voltages */
        pDevObj->devProfileData.swyLimit = pProf[0];
        pDevObj->devProfileData.swzLimit = pProf[1];
        pProf += 2;

        /* Save charge pump enable/disable setting */
        pDevObj->devProfileData.cpEnable = ((*pProf) & VP886_DEV_PROFILE_CP_ENABLE_MASK);
        /* Save the charge pump protection mode */
        pDevObj->devProfileData.cpProtection = ((*pProf) & VP886_DEV_PROFILE_CP_PROTECTION_MASK) >> 1;
        /* If we detected a shared supply device, make sure the device profile
           specifies shared supply. */
        if ((pDevObj->stateInt & VP886_SHARED_SUPPLY) && !(*pProf & VP886_DEV_PROFILE_SHARED_SUPPLY)) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Shared supply device detected, but device profile does not specify shared supply."));
            pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
            return VP_STATUS_ERR_PROFILE;
        }
        /* If the device profile specifies shared supply, make sure we detected a
           shared supply device. */
        if ((*pProf & VP886_DEV_PROFILE_SHARED_SUPPLY) && !(pDevObj->stateInt & VP886_SHARED_SUPPLY)) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Device profile specifies shared supply, but shared supply device not detected."));
            pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");
            return VP_STATUS_ERR_PROFILE;
        }
        pProf += 1;
    }

    /* Version 4+ */
    if (pDevProfile[VP_PROFILE_VERSION] < 4) {
        /* Defaults */
        pDevObj->devProfileData.vsw = 0;
        pDevObj->devProfileData.vbhOffset = 0;
        pDevObj->devProfileData.vbhOverhead = 0;
    } else {
        pDevObj->devProfileData.vsw = *pProf;
        pProf += 1;
        if (VP886_IS_ABS(pDevObj)) {
            pDevObj->devProfileData.vbhOffset = *pProf;
            pProf += 1;
        }
    }

    /* Version 5+ */
    if (pDevProfile[VP_PROFILE_VERSION] < 5) {
        /* Defaults */
        pDevObj->devProfileData.vbhOverhead = 0;
    } else {
        if (VP886_IS_ABS(pDevObj)) {
            pDevObj->devProfileData.vbhOverhead = *pProf;
            pProf += 1;
        }
    }

    /* If this flag is set, it means that the device is capable of being
       reprogrammed as either tracker or ABS, and the provided deviceType does
       not match the device's default PCN. */
    if (pDevObj->stateInt & VP886_CONVERT_DEVICE_TYPE) {
        Vp886ChangeDeviceType(pDevCtx);
    }

    /* Unmask all device interrupts except for those we know we will not use.
       - IO2 interrupts may be unmasked in Vp886InitLineInt if the pins are
         specified as interrupt inputs.
       - Mask hook and gkey interrupts during initialization.  We ignore them
         anyway, so masking them reduces unnecessary interrupt handling
       - Mask second-channel interrupts for single-channel devices */
    pDevObj->registers.intMask[0] = VP886_R_INTMASK_IO2 | VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK;
    pDevObj->registers.intMask[1] = VP886_R_INTMASK_IO2 | VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK;
    pDevObj->registers.intMask[2] = 0x00;
    pDevObj->registers.intMask[3] = 0x00;
    if (pDevObj->staticInfo.maxChannels == 1) {
        pDevObj->registers.intMask[1] |= VP886_R_INTMASK_OCALM | VP886_R_INTMASK_TEMPA |
            VP886_R_INTMASK_CAD | VP886_R_INTMASK_CID;
        pDevObj->registers.intMask[3] |= VP886_R_INTMASK_OVALM;
    }
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);
    
    /* Apply device profile MPI section */
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_NOOP, pDevProfile[VP_PROFILE_MPI_LEN],
        &pDevProfile[VP_PROFILE_DATA_START]);

    /* Read back data from the profile MPI section that we'll need later */
    VpSlacRegRead(pDevCtx, NULL, VP886_R_SWTIMING_RD, VP886_R_SWTIMING_LEN, pDevObj->swTimingParams);
    VpSlacRegRead(pDevCtx, VP_NULL, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN, pDevObj->swParams);
    VpSlacRegRead(pDevCtx, NULL, VP886_R_DEVMODE_RD, VP886_R_DEVMODE_LEN, pDevObj->devProfileData.devMode);
    VpSlacRegRead(pDevCtx, NULL, VP886_R_SWCTRL_RD, VP886_R_SWCTRL_LEN, pDevObj->registers.swCtrl);
    pDevObj->registers.devCfg[0] = pDevObj->devProfileData.devCfg;

    /* Defaulting the VpInitDevice state variable to the first state */
    pDevObj->initDeviceState = VP886_INIT_DEVICE_SWITCHER_PREP;

    /* Call into the state machine for the first time to set things off */
    Vp886InitDeviceSM(pDevCtx);

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitDevice");

    return VP_STATUS_SUCCESS;

} /* Vp886InitDevice */


/** Vp886InitDeviceData()
  Resets device object data during VpInitDevice().  This does not simply set
  all of the device object data to 0 because there are many variables that do
  not need to be reset, and others that must not be reset.

  Another possibility for this function would be to use memset to clear all of
  the device object data, while caching and restoring the parts that must not
  be reset.
*/
void
Vp886InitDeviceData(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    /* Reset timestamp data */
    pDevObj->timestamp = 0;
    pDevObj->rolloverCount = 0;
    pDevObj->rolloverBuffer = 0;

    /* Reset API device state flags */
    if (pDevObj->busyFlags & VP_DEV_INIT_CMP) {
        pDevObj->busyFlags = (VP_DEV_WARM_REBOOT | VP_DEV_INIT_IN_PROGRESS);
    } else {
        pDevObj->busyFlags = VP_DEV_INIT_IN_PROGRESS;
    }
    pDevObj->stateInt &= (VP886_SYS_CAL_COMPLETE | VP886_DEVICE_CAL_COMPLETE);
    pDevObj->absPowerReq[0] = pDevObj->absPowerReq[1] = VP886_ABS_POWER_REQ_LOW;
    pDevObj->spiError = FALSE;
    pDevObj->absPowerReq[0] = VP886_ABS_POWER_REQ_LOW;
    pDevObj->absPowerReq[1] = VP886_ABS_POWER_REQ_LOW;
    pDevObj->absRingingBattRequired[0] = FALSE;
    pDevObj->absRingingBattRequired[1] = FALSE;
    pDevObj->absRingingPeak[0] = 0;
    pDevObj->absRingingPeak[1] = 0;
    pDevObj->samplingTimerReq[0] = 0;
    pDevObj->samplingTimerReq[1] = 0;
    pDevObj->samplingTimerCurrent = 0;
    pDevObj->battFltStatus = 0;

    /* Reset the options cache */
    VpMemSet(&pDevObj->options, 0, sizeof(Vp886DevOptionsCacheType));

    /* Reset timer data */
    Vp886InitTimerQueue(pDevCtx);
#if (VP886_USER_TIMERS > 0)
    pDevObj->userTimers = 0;
#endif

    /* Reset event data */
    Vp886InitEventQueue(pDevCtx);
    pDevObj->getResultsRequired = FALSE;
}


/** Vp886InitDeviceSM()
  Implements the 886 device initialization state machine.

  Steps summary:
   - Device bring-up
   - Calibration
   - Switcher bring-up
   - Initialize lines
   - Generate VP_DEV_EVID_DEV_INIT_CMP
*/
void
Vp886InitDeviceSM(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status;
    bool runAnotherState = TRUE;
    static uint8 clkTestCount;
    uint16 eventData = VP_DEV_INIT_CMP_SUCCESS;
    uint32 timerDuration = 0;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDeviceSM+"));
    /* Keep looping throught the main switch until it's time to set a timer and
       leave.  Each state should either start a timer or measurement that will
       generate an interrupt OR set runAnotherState = TRUE. */
    while (runAnotherState) {
        runAnotherState = FALSE;

        VP_INFO(VpDevCtxType, pDevCtx, ("Vp886InitDeviceSM: Running state %d\n", pDevObj->initDeviceState));

        /* Switch on the state variable to enter the appropriate state (case) */
        switch (pDevObj->initDeviceState) {
            case VP886_INIT_DEVICE_SWITCHER_PREP: {
                uint8 mpiCmdData[6];
                uint8 mpiLen = pDevObj->pDevProfile[VP_PROFILE_MPI_LEN];
                uint8 channel;

                /* Program the profile switcher timing parameters with the low
                   power parameters included.  This will be overwritten later
                   in InitLine if any line is non-lowpower.
                   TODO: While this is slightly more efficient than programming
                   the params for each InitLine call, it is still redundant with
                   the MPI section write that we just did.  However, we do want
                   the low power params set by default, before we know about
                   the lines.  It still seems like there's a better way to do
                   this. */
                Vp886SetSwTimingParams(pDevCtx);

                /* Turn off the switchers' external circuits in the calibration control reg
                   by setting ZBATIN and YBATIN to 01 --> Disconnected.
                   This needs to be done for both lines.
                   Nothing about this in the Device Profile */
                mpiCmdData[0] = 0x50; mpiCmdData[1] = 0x00; mpiCmdData[2] = 0x00;
                /* For both channels */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, mpiCmdData);

                /* Increase the Switcher Over-current thresholds

                   This can be retrieved from the Switcher Over-Current
                   Threshold field of the Device Profile

                   The blanking parameter is also obtained from the
                   Device Profile */
                mpiCmdData[0] = 0x00; mpiCmdData[1] = 0x00;
                mpiCmdData[2] = pDevObj->pDevProfile[mpiLen+VP886_DEV_PROFILE_OC_CNT0_OFFSET];
                mpiCmdData[3] = pDevObj->pDevProfile[mpiLen+VP886_DEV_PROFILE_OC_CNT1_OFFSET];
                mpiCmdData[4] = pDevObj->pDevProfile[mpiLen+VP886_DEV_PROFILE_BLANKING_OFFSET];

                /* For both channels */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_SW_OC_WRT, VP886_R_SW_OC_LEN, mpiCmdData);

                /* Configure for auto-shutdown upon:
                   switcher over-current
                   switcher over-voltage
                   charge pump under-voltage

                   This probably isn't necessary since this is the power up default. Leaving
                   it in for completeness. Consider removing.
                   Nothing about this in the Device Profile.

                   The over voltage/current and undervoltage will by default be turned off by
                   the SWITCHER_CTRL option default value. This comes later. */
                mpiCmdData[0] = VP886_R_SSCFG_AUTO_SYSSTATE;
                mpiCmdData[1] = VP886_R_SSCFG_AUTO_OVERCURRENT | VP886_R_SSCFG_AUTO_OVERVOLTAGE | VP886_R_SSCFG_AUTO_CP_UV;
                /* For both channels */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, mpiCmdData);

                /* Enable over voltage detection

                   This is accomplished by applying the Switching Regulator Parameters
                   field of the Device Profile and making sure that the SWOVP is set. */
                pDevObj->swParams[1] |= VP886_R_SWPARAM_SWOVP;

                for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                    uint8 swParamBuf[VP886_R_SWPARAM_LEN];

                    /* The SWP reg needs to be written even if the LineCtx
                       for that line is NULL. This needs to be done to ensure that the
                       batteries come up */
                    swParamBuf[0] = pDevObj->swParams[0];
                    swParamBuf[1] = pDevObj->swParams[1];
                    swParamBuf[2] = pDevObj->swParams[2];

                    /* If this is an ABS part, swap in the SWY or SWZ voltage
                       as specified in the Dev Prof */
                    if (VP886_IS_ABS(pDevObj)) {

                        uint8 swv;
                        swParamBuf[0] &= ~VP886_R_SWPARAM_ABS_V;
                        if (channel == 0) {
                            swv = VpRoundedDivide(pDevObj->devProfileData.swyv, 5) - 1;
                        } else {
                            swv = VpRoundedDivide(pDevObj->devProfileData.swzv, 5) - 1;
                        }
                        if (swv <= 0) {
                            swv = 0;
                        }
                        swParamBuf[0] |= swv;
                    }

                    /* Send the SWR write command to the channel.*/
                    pDevObj->ecVal = (channel == 0) ? VP886_EC_1 : VP886_EC_2;
                    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, swParamBuf);
                    pDevObj->ecVal = VP886_EC_GLOBAL;
                }

                pDevObj->initDeviceState = VP886_INIT_DEVICE_CFAIL_CHECK_PREP;
                runAnotherState = TRUE;
                break;
            }
            case VP886_INIT_DEVICE_CFAIL_CHECK_PREP: {
                uint8 channel;

                /* Call Vp886ConfigLineInt now to blast down the AC, DC, and Ring profiles.
                   This way these profiles do not have to be stored in the device object */
                for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channel];
                    Vp886LineObjectType *pLineObj;
                    VpLineIdType lineId;
                    if (pLineCtx == VP_NULL) {
                        continue;
                    }
                    pLineObj = pLineCtx->pLineObj;

                    /* Initialize the line object variables, without disturbing lineId.
                       Do this now instead of during the later InitLine calls so that
                       data stored from the profiles is not lost. */
                    lineId = pLineObj->lineId;
                    Vp886MakeLineObjectInt(pLineObj->termType, pLineObj->channelId, pLineCtx, pLineObj, pDevCtx);
                    pLineObj->lineId = lineId;

                    Vp886ConfigLineInt(pLineCtx, pDevObj->pAcProfile, pDevObj->pDcProfile,
                                       pDevObj->pRingProfile, TRUE);
                }

                /* Initialize CFAIL failure count */
                clkTestCount = 10;

                /* Set a timer to give the clock some time to settle */
                timerDuration = VP886_CFAIL_WAIT_TIME;
                pDevObj->initDeviceState = VP886_INIT_DEVICE_CFAIL_CHECK;
                break;
            }
            case VP886_INIT_DEVICE_CFAIL_CHECK: {
                uint8 clkNotStable;

                /* Check for CFAIL */
                clkNotStable = pDevObj->registers.sigreg[0] & VP886_R_SIGREG_CFAIL;

                /* If clock fault is present and there are still some more tries allowed... */
                if (clkNotStable && (--clkTestCount)) {
                    /* Set a timer to return to this state in 10ms and check again */
                    timerDuration = VP886_CFAIL_WAIT_TIME;
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_CFAIL_CHECK;
                    break;
                }

                if (clkNotStable) {
                    /* If clock fault has persisted for too long */

                    if (VP886_IS_SF1(pDevObj) && pDevObj->registers.devCfg[0] == pDevObj->devProfileData.devCfg) {
                        /* SF1 supports 16kHz framesync.  If the device profile
                           was set for 8kHz see if 16 works, or vice-versa. */
                        pDevObj->registers.devCfg[0] ^= VP886_R_DEVCFG_FS_16K;
                        VpSlacRegWrite(pDevCtx, NULL, VP886_R_DEVCFG_WRT, VP886_R_DEVCFG_LEN, pDevObj->registers.devCfg);
                        clkTestCount = 10;
                        timerDuration = VP886_CFAIL_WAIT_TIME;
                        pDevObj->initDeviceState = VP886_INIT_DEVICE_CFAIL_CHECK;
                        break;
                    }

                    /* Send a Device Init Complete event with the CFAIL flag.
                       Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Clock Fault"));
                    eventData = VP_DEV_INIT_CMP_CFAIL;
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                    runAnotherState = TRUE;

                } else {
                    /* Clock fault cleared, proceed */
                    if (VP886_IS_SF1(pDevObj)) {
                        if (pDevObj->registers.devCfg[0] & VP886_R_DEVCFG_FS_16K) {
                            pDevObj->options.fsyncRate = VP_FSYNC_RATE_16KHZ;
                        } else {
                            pDevObj->options.fsyncRate = VP_FSYNC_RATE_8KHZ;
                        }
                    }
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_ZSI_DETECT;
                    runAnotherState = TRUE;
                }
                break;
            }
            case VP886_INIT_DEVICE_ZSI_DETECT: {
                uint8 csRd, csWrt;

                /* Check for ZSI mode by flipping the XE bit and verifying that it took */
                VpSlacRegRead(pDevCtx, NULL, VP886_R_CLKSLOTS_RD, VP886_R_CLKSLOTS_LEN, &csRd);
                csWrt = csRd ^ VP886_R_CLKSLOTS_EDGE;
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CLKSLOTS_WRT, VP886_R_CLKSLOTS_LEN, &csWrt);
                VpSlacRegRead(pDevCtx, NULL, VP886_R_CLKSLOTS_RD, VP886_R_CLKSLOTS_LEN, &csRd);

                if (csWrt == csRd) {
                    /* If the XE bit flip took, this is NOT ZSI mode.
                       Set the flag in the Device Object and put the
                       XE bit back to its original value. */
                    csWrt ^= VP886_R_CLKSLOTS_EDGE;
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_CLKSLOTS_WRT, VP886_R_CLKSLOTS_LEN, &csWrt);
                    pDevObj->stateInt &= ~VP886_ZSI_DETECTED;
                    VP_INFO(VpDevCtxType, pDevCtx, ("ZSI not detected"));
                } else {
                    /* The XE bit flip did not change, this is ZSI mode.
                       Set the flag in the device object. */
                    pDevObj->stateInt |= VP886_ZSI_DETECTED;
                    VP_INFO(VpDevCtxType, pDevCtx, ("ZSI detected"));
                }

                pDevObj->initDeviceState = VP886_INIT_DEVICE_ENABLE_VREF;
                runAnotherState = TRUE;
                break;
            }
            case VP886_INIT_DEVICE_ENABLE_VREF: {
                uint8 icr3Data[4];

                /* Enable Vref in ICR3 */
                icr3Data[0] = VP886_R_ICR3_VREF_EN; icr3Data[1] = VP886_R_ICR3_VREF_EN;
                icr3Data[2] = 0x00; icr3Data[3] = 0x00;
                /* For both channels */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, icr3Data);

                /* Set a timer to allow Vref to come up */
                timerDuration = VP886_VREF_WAIT_TIME;
                pDevObj->initDeviceState = VP886_INIT_DEVICE_VREF_CHECK;
                break;
            }
            case VP886_INIT_DEVICE_VREF_CHECK: {
                uint8 icr6Data[4];

                /* Read ICR6 to see if Vref came up */
                VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR6_RD, VP886_R_ICR6_LEN, icr6Data);

                /* If it didn't .... */
                if ((icr6Data[0] & 0x10) == 0x00) {

                    /* Send a Device Init Complete event with the VREF_FAIL flag.
                       Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Vref did not come up successfully"));
                    eventData = VP_DEV_INIT_CMP_VREF_FAIL;

                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

                    runAnotherState = TRUE;
                    break;

                }

                /* If it did come up ... give it a little bit more timer to settle */
                timerDuration = VP886_VREF_WAIT_TIME;
                if (pDevObj->devProfileData.cpEnable && !VP886_IS_SF(pDevObj)) {
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_CP_ENABLE;
                } else {
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_CH1_CH2_DISC;
                }

                break;
            }
            case VP886_INIT_DEVICE_CP_ENABLE: {
                uint8 dmData[2];

                /* Re-enable the Charge Pump
                   This is done by applying the Device Mode field of the Device
                   Profile, with CPEN = 1 */
                dmData[0] = pDevObj->devProfileData.devMode[0] | VP886_R_DEVMODE_CHARGEPUMP_EN;
                dmData[1] = pDevObj->devProfileData.devMode[1];
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_DEVMODE_WRT, VP886_R_DEVMODE_LEN, dmData);

                /* Give the charge pump some time to come up */
                timerDuration = VP886_CP_WAIT_TIME;
                pDevObj->initDeviceState = VP886_INIT_DEVICE_CH1_CH2_DISC;

                break;
            }

            case VP886_INIT_DEVICE_CH1_CH2_DISC: {
                uint8 ssData = VP886_R_STATE_SS_DISCONNECT;

                /* If ABS, put switchers into low power mode without actually turning them on */
                if (VP886_IS_ABS(pDevObj)) {
                    pDevObj->registers.swCtrl[0] &= ~(VP886_R_SWCTRL_MODE_Z | VP886_R_SWCTRL_MODE_Y);
                    pDevObj->registers.swCtrl[0] |= (VP886_R_SWCTRL_MODE_Y_LP | VP886_R_SWCTRL_MODE_Z_LP);
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWCTRL_WRT, VP886_R_SWCTRL_LEN, pDevObj->registers.swCtrl);
                }

                /* Put both channels into disconnect */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, &ssData);

                /* Set a timer to delay for a bit */
                timerDuration = VP886_DISC_WAIT_TIME;
                pDevObj->initDeviceState = VP886_INIT_DEVICE_CP_CHECK;
                break;
            }
            case VP886_INIT_DEVICE_CP_CHECK: {
                uint8 ssData1, ssData2;

                /* If there was an under-volt on the charge pump ... */
                if ((pDevObj->registers.sigreg[2] & VP886_R_SIGREG_CPUVLO) == VP886_R_SIGREG_CPUVLO) {

                    /* Send a Device Init Complete event with the CP_FAIL flag.
                       Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Charge Pump Undervoltage"));
                    eventData = VP_DEV_INIT_CMP_CP_FAIL;

                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

                    runAnotherState = TRUE;
                    break;

                }

                /* Verify that the channels are still in disconnect */
                pDevObj->ecVal = VP886_EC_1;
                VpSlacRegRead(pDevCtx, NULL, VP886_R_STATE_RD, VP886_R_STATE_LEN, &ssData1);
                pDevObj->ecVal = VP886_EC_1;
                VpSlacRegRead(pDevCtx, NULL, VP886_R_STATE_RD, VP886_R_STATE_LEN, &ssData2);
                pDevObj->ecVal = VP886_EC_GLOBAL;

                if ((ssData1 == 0x0F) || (ssData2 == 0x0F)) {

                    /* If channel 1 went back into shutdown ... */
                    if (ssData1 == 0x0F) {
                        /* Send a Device Init Complete event with the CH1_SD flag.
                           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
                        VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Ch1 Shutdown after CP bringup"));
                        eventData |= VP_DEV_INIT_CMP_CH1_SD;
                        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                        runAnotherState = TRUE;
                        break;
                    }

                    /* If channel 2 went back into shutdown ... */
                    if (ssData2 == 0x0F) {
                        /* Send a Device Init Complete event with the CH2_SD flag.
                           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
                        VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Ch2 Shutdown after CP bringup"));
                        eventData |= VP_DEV_INIT_CMP_CH2_SD;
                        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                        runAnotherState = TRUE;
                        break;
                    }

                }

                pDevObj->initDevSwitcherMode = (VP886_R_SWCTRL_MODE_Z_LP | VP886_R_SWCTRL_MODE_Y_LP);
                pDevObj->initDeviceState = VP886_INIT_DEVICE_CAL_CODEC_START;
                runAnotherState = TRUE;
                break;
            }
            case VP886_INIT_DEVICE_CAL_CODEC_START: {

                /* Set the first step of calCodec */
                pDevObj->calCodecState[0] = VP886_CAL_CODEC_START;

                pDevObj->initDeviceState = VP886_INIT_DEVICE_CAL_CODEC;
                runAnotherState = TRUE;
                break;
            }
            case VP886_INIT_DEVICE_CAL_CODEC: {

                Vp886CalCodecHandler(pDevCtx);

                if (pDevObj->calCodecState[0] == VP886_CAL_CODEC_COMPLETE) {
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_SW_ENABLE;
                    runAnotherState = TRUE;
                } else if (pDevObj->calCodecState[0] == VP886_CAL_CODEC_FAILED) {
                    eventData |= VP_DEV_INIT_CMP_CAL_FAIL;
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDevice(): Device calibration failed."));
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                    runAnotherState = TRUE;
                }
                /* else calibration is ongoing. */
                break;
            }
            case VP886_INIT_DEVICE_SW_ENABLE: {
                uint8 calCtrl[VP886_R_CALCTRL_LEN];

                /* Turn on both switchers
                   This is accomplished by writing the Switching Regulator Control
                   Register value provided in the Device Profile forced to a
                   certain power mode */
                pDevObj->registers.swCtrl[0] &= ~(VP886_R_SWCTRL_MODE_Z | VP886_R_SWCTRL_MODE_Y);
                pDevObj->registers.swCtrl[0] |= pDevObj->initDevSwitcherMode;

                VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWCTRL_WRT, VP886_R_SWCTRL_LEN, pDevObj->registers.swCtrl);

                /* Turn the switchers external circuits back on */
                calCtrl[0] = calCtrl[1] = calCtrl[2] = 0;
                /* For both channels */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calCtrl);

                /* Wait a few ms. Come back and check for over current again */
                timerDuration = VP886_SW_EN_WAIT_TIME;
                pDevObj->initDeviceState = VP886_INIT_DEVICE_ALARM_CHECK;
                break;
            }
            case VP886_INIT_DEVICE_ALARM_CHECK: {
                bool alarm;

                /* Check for Switcher Overcurrent/Overvoltage, and CP Undervoltage */
                alarm = Vp886InitDeviceAlarmCheck(pDevCtx, &eventData);

                /* If there was an alarm ... */
                if (alarm) {
                    /* Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                    runAnotherState = TRUE;
                    break;
                }

                /* At this point, the charge pump is enabled, over voltage protection is enabled
                   and the switchers are on, regardless of what is specified
                   in the device profile. */

                /* If this is a tracker device */
                if (VP886_IS_TRACKER(pDevObj)) {

                    /* Start the calibration */
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_INIT_LINES;

                    runAnotherState = TRUE;
                    break;
                }

                /* This is an ABS device */

                if ((pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_MODE) ==
                    VP886_DEV_PROFILE_ABS_SUPP_CFG_SLAVE) {
                    /* In slave mode, leave switchers at low power and move on to
                       calibration */
                   pDevObj->initDeviceState = VP886_INIT_DEVICE_INIT_LINES;

                } else if ((pDevObj->initDevSwitcherMode & VP886_R_SWCTRL_MODE_Y) == VP886_R_SWCTRL_MODE_Y_LP &&
                           (pDevObj->initDevSwitcherMode & VP886_R_SWCTRL_MODE_Z) == VP886_R_SWCTRL_MODE_Z_LP) {
                    /* From low power, go back to VP886_INIT_DEVICE_SW_ENABLE and change
                       to medium power */
                    if (!(pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Y_SLAVE)) {
                        pDevObj->initDevSwitcherMode &= ~VP886_R_SWCTRL_MODE_Y;
                        pDevObj->initDevSwitcherMode |= VP886_R_SWCTRL_MODE_Y_MP;
                    }
                    if (!(pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Z_SLAVE)) {
                        pDevObj->initDevSwitcherMode &= ~VP886_R_SWCTRL_MODE_Z;
                        pDevObj->initDevSwitcherMode |= VP886_R_SWCTRL_MODE_Z_MP;
                    }
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_SW_ENABLE;

                } else if ((pDevObj->initDevSwitcherMode & VP886_R_SWCTRL_MODE_Y) == VP886_R_SWCTRL_MODE_Y_MP ||
                           (pDevObj->initDevSwitcherMode & VP886_R_SWCTRL_MODE_Z) == VP886_R_SWCTRL_MODE_Z_MP) {
                    /* From medium power, go back to VP886_INIT_DEVICE_SW_ENABLE and change
                       to high power */
                    if (!(pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Y_SLAVE)) {
                        pDevObj->initDevSwitcherMode &= ~VP886_R_SWCTRL_MODE_Y;
                        pDevObj->initDevSwitcherMode |= VP886_R_SWCTRL_MODE_Y_HP;
                    }
                    if (!(pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Z_SLAVE)) {
                        pDevObj->initDevSwitcherMode &= ~VP886_R_SWCTRL_MODE_Z;
                        pDevObj->initDevSwitcherMode |= VP886_R_SWCTRL_MODE_Z_HP;
                    }
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_SW_ENABLE;

                } else {
                    /* After reaching high power, move on to init line */
                   pDevObj->initDeviceState = VP886_INIT_DEVICE_INIT_LINES;
                }

                runAnotherState = TRUE;
                break;
            }
            case VP886_INIT_DEVICE_INIT_LINES: {
                uint8 channel;

                /* Call Vp886InitLineInt() once for each channel. */
                for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channel];
                    if (pLineCtx == VP_NULL) {
                        continue;
                    }

                    status = Vp886InitLineInt(pLineCtx, VP_NULL, VP_NULL, VP_NULL, TRUE);
                    if (status != VP_STATUS_SUCCESS) {
                        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDevice(): Channel %d initialization failed!", channel));
                        eventData |= VP_DEV_INIT_CMP_LINE_FAIL;
                    }
                }

                /* Initialize options. */
                if (eventData == VP_DEV_INIT_CMP_SUCCESS) {
                    pDevObj->busyFlags |= VP_TEMP_IGNORE_ALL_BUSY_FLAGS;
                    pDevObj->busyFlags |= VP_DEV_IMPL_DEFAULT_OPTIONS;
                    status = VpImplementDefaultSettings(pDevCtx, VP_NULL);
                    pDevObj->busyFlags &= ~VP_DEV_IMPL_DEFAULT_OPTIONS;
                    pDevObj->busyFlags &= ~VP_TEMP_IGNORE_ALL_BUSY_FLAGS;
                    if (status != VP_STATUS_SUCCESS) {
                        eventData |= VP_DEV_INIT_CMP_FAIL;
                    }
                }

                if (eventData != VP_DEV_INIT_CMP_SUCCESS) {
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                    runAnotherState = TRUE;
                    break;
                }

                /* Override the API default for adaptive ringing power with the
                   profile value */
                pDevObj->options.adaptiveRinging.power = pDevObj->devProfileData.adaptiveRingingMaxPower;

                if (VP886_IS_ABS(pDevObj) || pDevObj->calData[0].valid) {
                    /* Calibration factors already set */
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                } else {
                    pDevObj->busyFlags |= VP_DEV_IN_CAL;

                    for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                        pDevObj->calCodecState[channel] = VP886_CAL_CODEC_LONGITUDINAL;
                        pDevObj->calCodecSubState[channel] = VP886_LONG_INIT;
                        pDevObj->channelCalBack[channel] = TRUE;
                    }

                    pDevObj->initDeviceState = VP886_INIT_DEVICE_LONG_CAL;
                }

                runAnotherState = TRUE;
                break;
            }
            case VP886_INIT_DEVICE_LONG_CAL: {
                uint8 channel, runChan = 0, queueSize = 0;
                uint8 otherChan = pDevObj->staticInfo.maxChannels - 1;

                for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                    /* Check which channels need to be precessed */
                    if (pDevObj->channelCalBack[channel]) {
                        if (queueSize++ == 0) {
                            runChan = channel;
                        }
                    }
                }

                if (queueSize > 0) {
                    pDevObj->channelCalBack[runChan] = FALSE;
                    Vp886LongitudinalCalibration(pDevCtx, runChan);
                    /* Check if another channel needs to be processes (max two channels per device) */
                    if (queueSize > 1) {
                        /* The first channel is always processed first if both channels are waiting,
                            if (queueSize > 1), it's always the second channel */
                        pDevObj->channelCalBack[1] = FALSE;
                        Vp886LongitudinalCalibration(pDevCtx, 1);
                    }
                } else {
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Unexpected longitudinal calibration error"));
                }

                /* If longitudinal calibration is done generate a complete event */
                if ((pDevObj->calCodecState[0] == VP886_CAL_CODEC_SYNC) &&
                    (pDevObj->calCodecState[otherChan] == VP886_CAL_CODEC_SYNC)) {
                    pDevObj->busyFlags &= ~VP_DEV_IN_CAL;
                    pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;
                    runAnotherState = TRUE;
                }
                break;
            }
            case VP886_INIT_DEVICE_GEN_EVENT: {
                uint8 channel;

                /* Apply the cal if it was loaded from a calibration profile */
                for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channel];

                    if (pLineCtx == VP_NULL) {
                        continue;
                    }

                    if (pDevObj->calData[channel].valid) {
                        /* Apply calibration for the line */
                        Vp886ApplyCalGeneral(pLineCtx);
                        Vp886ApplyCalDC(pLineCtx);
                        Vp886ApplyCalRing(pLineCtx);
                        Vp886ProgramCalRegisters(pLineCtx);
                    }
                }

                /* Return the second channel to shutdown if this is a single
                   channel tracking device */
                if (VP886_IS_TRACKER(pDevObj) && pDevObj->staticInfo.maxChannels == 1) {
                    uint8 ssData = VP886_R_STATE_SS_SHUTDOWN;
                    pDevObj->ecVal = VP886_EC_2;
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, &ssData);
                    pDevObj->ecVal = VP886_EC_GLOBAL;
                }

                /* For devices without both I/O pins, set the I/Os to output low for safety and
                   power saving. */
                switch (pDevObj->ioCapability) {
                    case VP886_IO_CAPABILITY_TWO_PER_CH:
                        if (pDevObj->staticInfo.maxChannels == 1) {
                            /* For single-channel parts, set the second channel IOs. */
                            uint8 ioDir = VP886_R_IODIR_IOD1_DIG_OUTPUT | VP886_R_IODIR_IOD2_OUTPUT;
                            uint8 ioData = 0x00;
                            pDevObj->ecVal = VP886_EC_2;
                            VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDir);
                            VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODATA_WRT, VP886_R_IODATA_LEN, &ioData);
                            pDevObj->ecVal = VP886_EC_GLOBAL;
                        }
                        break;
                    case VP886_IO_CAPABILITY_NONE: {
                        uint8 ioDir = VP886_R_IODIR_IOD1_DIG_OUTPUT | VP886_R_IODIR_IOD2_OUTPUT;
                    uint8 ioData = 0x00;
                    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDir);
                    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODATA_WRT, VP886_R_IODATA_LEN, &ioData);
                        break;
                    }
                    case VP886_IO_CAPABILITY_CH0_IO2: {
                        /* Set IO1 on the first channel and both IOs on the second channel */
                        uint8 ioDir;
                    uint8 ioData = 0x00;
                        pDevObj->ecVal = VP886_EC_1;
                        VpSlacRegRead(pDevCtx, VP_NULL, VP886_R_IODIR_RD, VP886_R_IODIR_LEN, &ioDir);
                        ioDir |= VP886_R_IODIR_IOD1_DIG_OUTPUT;
                        VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDir);
                    pDevObj->ecVal = VP886_EC_2;
                        ioDir = VP886_R_IODIR_IOD1_DIG_OUTPUT | VP886_R_IODIR_IOD2_OUTPUT;
                    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDir);
                        pDevObj->ecVal = VP886_EC_GLOBAL;
                    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_IODATA_WRT, VP886_R_IODATA_LEN, &ioData);
                }
                    default:
                        break;
                }

                /* Unmask hook and gkey interrupts */
                pDevObj->registers.intMask[0] &= ~(VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK);
                if (pDevObj->staticInfo.maxChannels == 2) {
                    pDevObj->registers.intMask[1] &= ~(VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK);
                }
                VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);

                pDevObj->busyFlags &= ~(VP_DEV_WARM_REBOOT | VP_DEV_INIT_IN_PROGRESS);
                if (eventData == VP_DEV_INIT_CMP_SUCCESS) {
                    /* Mark device as initialized to allow other functions to execute. */
                    pDevObj->busyFlags |= VP_DEV_INIT_CMP;
                } else {
                    /* Set global failure flag. */
                    eventData |= VP_DEV_INIT_CMP_FAIL;
                }
                Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_RESPONSE, VP_DEV_EVID_DEV_INIT_CMP,
                    eventData, Vp886GetTimestamp(pDevCtx), FALSE);
                pDevObj->initDeviceState = VP886_INIT_DEVICE_COMPLETE;
                break;
            }
            default:
                break;
        }

    }

    /* Set a timer, if requested */
    if (timerDuration != 0) {
        Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_INIT_DEVICE, timerDuration, 0, 0);
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDeviceSM-"));

} /* Vp886InitDeviceSM */


/** Vp886InitDevicePcnRcn()
  Flush the MPI buffer and read the RCN and PCN info into the device object.
*/
VpStatusType
Vp886InitDevicePcnRcn(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 devicePcn, deviceRcn;
    uint8 fuse8[VP886_R_FUSE_REG_8_LEN];

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDevicePcnRcn+"));

    /* If Device has already been succesfully initialized, don't need to do
       anything here. */
    if (pDevObj->busyFlags & VP_DEV_INIT_CMP) {
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDevicePcnRcn-"));
        return VP_STATUS_SUCCESS;
    }

    /* If this is being called before InitDevice, clear the MPI buffer first */
    if (!((pDevObj->busyFlags & VP_DEV_INIT_CMP) ||
          (pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)))
    {
        VpCSLACClearMPIBuffer(deviceId);
        VpCSLACClearMPIBuffer(deviceId);
    }

    VpSlacRegRead(pDevCtx, NULL, VP886_R_RCNPCN_RD, VP886_R_RCNPCN_LEN, pDevObj->staticInfo.rcnPcn);

    devicePcn = pDevObj->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX];
    deviceRcn = pDevObj->staticInfo.rcnPcn[VP886_R_RCNPCN_RCN_IDX];

    /* MPI Failure if the PCN and RCN are both 0x00 or 0xFF */
    if (((devicePcn == 0xFF) && (deviceRcn == 0xFF)) ||
        ((devicePcn == 0x00) && (deviceRcn == 0x00))) {
        pDevObj->busyFlags &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);

        VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Detect Revision/PCN Properly: 0x%02X 0x%02X",
            deviceRcn, devicePcn));

        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDevicePcnRcn-"));

        return VP_STATUS_ERR_SPI;
    }

    /* Make sure the product code is for a known device */
    switch (devicePcn) {
        case VP886_R_RCNPCN_PCN_ZL88601:
        case VP886_R_RCNPCN_PCN_ZL88602:
        case VP886_R_RCNPCN_PCN_ZL88701:
        case VP886_R_RCNPCN_PCN_ZL88702:
        case VP886_R_RCNPCN_PCN_LE9661:
        case VP886_R_RCNPCN_PCN_LE9671:
        case VP886_R_RCNPCN_PCN_LE9642:
        case VP886_R_RCNPCN_PCN_LE9641:
        case VP886_R_RCNPCN_PCN_LE9652:
        case VP886_R_RCNPCN_PCN_LE9651:
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Device PCN not recognized in ZL880 or miSLIC families: 0x%02X",
                devicePcn));
            VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDevicePcnRcn-"));
            return VP_STATUS_ERR_SPI;
    }

    if (VP886_IS_1CH(pDevObj)) {
        VP_INFO(VpDevCtxType, pDevCtx, ("Detected single channel device (0x%02X)", devicePcn));
        pDevObj->staticInfo.maxChannels = 1;
    }

    /* Check for SF1AB */
    if (VP886_IS_SF1(pDevObj)) {
        uint8 ec = VP886_EC_BOTH;
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_EC_WRT, VP886_R_EC_LEN, &ec);
        VpSlacRegRead(pDevCtx, NULL, VP886_R_EC_RD, VP886_R_EC_LEN, &ec);
        if (ec != VP886_EC_1) {
            /* Change the revision to 7 to indicate SF1AB */
            deviceRcn = pDevObj->staticInfo.rcnPcn[VP886_R_RCNPCN_RCN_IDX] = 7;
        }
    }

    /* Save the IO capabilities of the device based on rcn/pcn */
    if (VP886_IS_SF(pDevObj)) {
        if (VP886_IS_SF1(pDevObj)) {
            pDevObj->ioCapability = VP886_IO_CAPABILITY_CH0_IO2;
        } else { /* SF2 or SF1AB */
            pDevObj->ioCapability = VP886_IO_CAPABILITY_NONE;
        }
    } else if (VP886_IS_SF0(pDevObj)) {
        pDevObj->ioCapability = VP886_IO_CAPABILITY_NONE;
    } else {
        pDevObj->ioCapability = VP886_IO_CAPABILITY_TWO_PER_CH;
    }

    /* Some ABS devices (ZL88801, Le9662) may have a fuse register bit set to
       indicate that they should be allowed to be treated as shared tracker
       instead of ABS.
       If an ABS has this bit set AND the application created the device object
       as VP_DEV_887_SERIES, allow the device to be configured as shared
       tracker. */
    if (VP886_IS_ABS(pDevObj)) {
        VpSlacRegRead(pDevCtx, NULL, VP886_R_FUSE_REG_8_RD, VP886_R_FUSE_REG_8_LEN, fuse8);
        if (fuse8[0] & VP886_R_FUSE_REG_8_SHSUP) {
            if (pDevCtx->deviceType == VP_DEV_887_SERIES) {
                /* Setting this bit changes the ABS PCN to tracker */
                pDevObj->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] |= VP886_R_RCNPCN_PCN_ABS_TRACKER;
                /* Set a flag to enabled shared-tracker behaviors. */
                pDevObj->stateInt |= VP886_SHARED_SUPPLY;
                /* Set a flag to call Vp886ChangeDeviceType() later on to
                   actually reprogram the device to run in the other mode. */
                pDevObj->stateInt |= VP886_CONVERT_DEVICE_TYPE;
            }
        }
    }

    /* Check that the product code matches the device type that was passed in
       to VpMakeDeviceObject */
    if (VP886_DEVICE_SERIES(pDevObj) != pDevCtx->deviceType) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Specified device type %02X does not match the device PCN 0x%02X",
            pDevCtx->deviceType, devicePcn));
        return VP_STATUS_ERR_VTD_CODE;
    }

    VP_INFO(VpDevCtxType, pDevCtx, ("Device is %s %s, rev 0x%02X\n",
        VP886_IS_ABS(pDevObj) ? "886 (ABS)" : "887 (TRACKER)",
        VP886_IS_HV(pDevObj) ? "HV" : "LV", deviceRcn));
    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886InitDevicePcnRcn-"));

    return VP_STATUS_SUCCESS;
} /* Vp886InitDevicePcnRcn */


/** Vp886InitDeviceAlarmCheck()
  This function is called by the Vp886InitDeviceSM() state machine to check for
  Switcher OverCurrent / Switcher OverVoltage / ChargePump UnderVoltage interrupts
  during the initialization sequence
*/
bool
Vp886InitDeviceAlarmCheck(
    VpDevCtxType *pDevCtx,
    uint16* eventData)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 ssData1, ssData2;
    bool alarm = FALSE;

    /* Switcher 1 over-current ? */
    if ((pDevObj->registers.sigreg[0] & VP886_R_SIGREG_OCALM) == VP886_R_SIGREG_OCALM) {

        /* Send a Device Init Complete event with the CH1_OC flag.
           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Channel 1 Over Current"));
        *eventData |= VP_DEV_INIT_CMP_CH1_OC;

        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        alarm = TRUE;

    }

    /* Switcher 1 over-voltage ? */
    if ((pDevObj->registers.sigreg[2] & VP886_R_SIGREG_OVALM) == VP886_R_SIGREG_OVALM) {

        /* Send a Device Init Complete event with the CH1_OV flag.
           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Channel 1 Over Voltage"));
        *eventData |= VP_DEV_INIT_CMP_CH1_OV;

        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        alarm = TRUE;

    }

    /* Switcher 2 over-current ? */
    if ((pDevObj->registers.sigreg[1] & VP886_R_SIGREG_OCALM) == VP886_R_SIGREG_OCALM) {

        /* Send a Device Init Complete event with the CH2_OC flag.
           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Channel 2 Over Current"));
        *eventData |= VP_DEV_INIT_CMP_CH2_OC;

        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        alarm = TRUE;

    }

    /* Switcher 2 over-voltage ? */
    if ((pDevObj->registers.sigreg[3] & VP886_R_SIGREG_OVALM) == VP886_R_SIGREG_OVALM) {

        /* Send a Device Init Complete event with the CH2_OV flag.
           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Channel 2 Over Voltage"));
        *eventData |= VP_DEV_INIT_CMP_CH2_OV;

        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        alarm = TRUE;

    }

    /* Check the CP again */
    if ((pDevObj->registers.sigreg[2] & VP886_R_SIGREG_CPUVLO) == VP886_R_SIGREG_CPUVLO) {

        /* Send a Device Init Complete event with the CP_FAIL flag.
           Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Charge Pump Undervoltage"));
        *eventData = VP_DEV_INIT_CMP_CP_FAIL;

        pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        alarm = TRUE;
    }

    /* Verify that the channels are still in disconnect */
    pDevObj->ecVal = VP886_EC_1;
    VpSlacRegRead(pDevCtx, VP_NULL, VP886_R_STATE_RD, VP886_R_STATE_LEN, &ssData1);
    pDevObj->ecVal = VP886_EC_2;
    VpSlacRegRead(pDevCtx, VP_NULL, VP886_R_STATE_RD, VP886_R_STATE_LEN, &ssData2);
    pDevObj->ecVal = VP886_EC_GLOBAL;

    if ((ssData1 == 0x0F) || (ssData2 == 0x0F)) {

        /* If channel 1 went back into shutdown ... */
        if (ssData1 == 0x0F) {
            /* Send a Device Init Complete event with the CH1_SD flag.
               Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Ch1 Shutdown"));
            *eventData |= VP_DEV_INIT_CMP_CH1_SD;
            pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        }

        /* If channel 2 went back into shutdown ... */
        if (ssData2 == 0x0F) {
            /* Send a Device Init Complete event with the CH2_SD flag.
               Go to the VP886_INIT_DEVICE_GEN_EVENT state. */
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitDeviceAlarmCheck() Ch2 Shutdown"));
            *eventData |= VP_DEV_INIT_CMP_CH2_SD;
            pDevObj->initDeviceState = VP886_INIT_DEVICE_GEN_EVENT;

        }

        alarm = TRUE;

    }

    return alarm;

} /* Vp886InitDeviceAlarmCheck() */


/** Vp886InitLine()
  Implements VpInitLine() to initialize a line of a device with the specified
  parameters and API default values. It is a "Line Reset".

  See the VP-API-II Reference Guide for more details on VpInitLine().
*/
VpStatusType
Vp886InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886InitLine");

    if (!Vp886ReadyStatus(pDevCtx, VP_NULL, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886InitLine");
        return status;
    }

    if (pLineObj->channelId >= pDevObj->staticInfo.maxChannels) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("InitLine: Channel ID %d exceeds device maximum", pLineObj->channelId));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886InitLine");
        return VP_STATUS_INVALID_LINE;
    }

    if (VP886_IS_ABS(pDevObj) && pLineObj->lineState.usrCurrent == VP_LINE_DISABLED) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Cannot initialize ABS line in VP_LINE_DISABLED.  Must use VpInitDevice."));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886InitLine");
        return VP_STATUS_DEVICE_BUSY;
    }

    /* Check for valid profile arguments. */
    if (
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_AC, &pAcProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_DC, &pDcOrFxoProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_RING, &pRingProfile) != VP_STATUS_SUCCESS)
    ) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886InitLine");
        return VP_STATUS_ERR_PROFILE;
    }

    status = Vp886InitLineInt(pLineCtx, pAcProfile, pDcOrFxoProfile, pRingProfile, FALSE);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886InitLine");
    return status;
} /* Vp886InitLine */


/** Vp886InitLineInt()
  This function can be called internally to skip device status checks
  and critical sections

  The initDevice argument specifies whether this function is being called from
  within VpInitDevice().  If so (initDevice = TRUE), the line-specific registers
  will not be initialized to default values (because they have already been
  intialized by the 0x04 Hardware Reset command), and the line-specific options
  will not be initialized, because they will be initialized later in
  VpInitDevice().
*/
static VpStatusType
Vp886InitLineInt(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile,
    bool initDevice)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpLineIdType lineId;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886InitLineInt+"));

#if (VP886_USER_TIMERS > 0)
    if (!initDevice) {
        /* Subtract this line's USER timer count from the total count */
        if (pDevObj->userTimers < pLineObj->userTimers) {
            VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886InitLineInt - User timer count mismatch. D:%d L:%d",
                pDevObj->userTimers, pLineObj->userTimers));
            pDevObj->userTimers = 0;
        } else {
            pDevObj->userTimers -= pLineObj->userTimers;
        }
    }
    pLineObj->userTimers = 0;
#endif

    if (!initDevice) {
        VpOptionEventMaskType eventMask;
        /* Initialize the line object variables, without disturbing lineId or
           the line event masks.
           InitDevice does this separately, just before ConfigLine so that
           profile data saved in the line object will not be lost. */
        lineId = pLineObj->lineId;
        eventMask = pLineObj->options.eventMask;
        status = Vp886MakeLineObjectInt(pLineObj->termType, channelId, pLineCtx, pLineObj, pDevCtx);
        pLineObj->lineId = lineId;
        pLineObj->options.eventMask = eventMask;
    }

    /* Cancel all timers associated with this line. */
    VpRemoveChannelTimers(pDevCtx, &pDevObj->timerQueueInfo, pDevObj->timerQueueNodes, channelId);

    /* Prevent other functions from accessing this line while initializing. */
    pLineObj->busyFlags = VP886_LINE_INIT_IN_PROGRESS;

    /* Set up initial channel register defaults */
    Vp886InitLineRegisters(pLineCtx, initDevice);

    /* Initialize the relay state. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp886SetRelayStateInt(pLineCtx, VP_RELAY_NORMAL);
    }

    /* Initialize the line state. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp886SetLineStateFxs(pLineCtx, VP_LINE_DISCONNECT);
    }

    /* Apply profile-independent calibration factors.  During InitDevice, the
       InitDevice state machine handles this.  The call to
       Vp886ProgramCalRegisters() will happen in ConfigLine. */
    if (!initDevice) {
        Vp886ApplyCalGeneral(pLineCtx);
    }

    /* Process the profile arguments. */
    if (!initDevice) {
        /* Substitute in default profiles for those that are not provided
           so that the API is kept in sync with what is programmed. */
        Vp886SubDefaultProfiles(pDevCtx, &pAcProfile, &pDcOrFxoProfile, &pRingProfile);
    }
    if (status == VP_STATUS_SUCCESS) {
        status = Vp886ConfigLineInt(pLineCtx, pAcProfile, pDcOrFxoProfile, pRingProfile, initDevice);
    }

    /* Initialize the line-specific options. */
    if (status == VP_STATUS_SUCCESS) {

        if (!initDevice) {
            pLineObj->busyFlags |= VP886_IGNORE_ALL_BUSY_FLAGS;
            pLineObj->busyFlags |= VP886_LINE_DEFAULT_OPTIONS;
            status = VpImplementDefaultSettings(VP_NULL, pLineCtx);
            pLineObj->busyFlags &= ~VP886_LINE_DEFAULT_OPTIONS;
            pLineObj->busyFlags &= ~VP886_IGNORE_ALL_BUSY_FLAGS;
        }

        /* Set automatic thermal fault shutdown based on the device option */
        pLineObj->registers.ssCfg[0] &= ~VP886_R_SSCFG_AUTO_THERMFAULT;
        if (pDevObj->options.criticalFlt.thermFltDiscEn) {
            pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_AUTO_THERMFAULT;
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);

        /* Timeslots are not set by VpImplementDefaultSettings.  Read from the
           device to synchronize */
        VpSlacRegRead(NULL, pLineCtx, VP886_R_TXSLOT_RD, VP886_R_TXSLOT_LEN, &pLineObj->options.timeslot.tx);
        VpSlacRegRead(NULL, pLineCtx, VP886_R_RXSLOT_RD, VP886_R_RXSLOT_LEN, &pLineObj->options.timeslot.rx);
    }

    /* Initialize the IO2 pin configuration per the Device Profile */
    if (initDevice && status == VP_STATUS_SUCCESS &&
        pDevObj->ioCapability != VP886_IO_CAPABILITY_NONE &&
        !(pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2 && channelId == 1))
    {
        uint8 ioDirReg;
        uint8 io2Use;

        /* Get the current IO direction reg contents */
        VpSlacRegRead(NULL, pLineCtx, VP886_R_IODIR_RD, VP886_R_IODIR_LEN, &ioDirReg);
        io2Use = pDevObj->devProfileData.io2Use >> (channelId*4);
        io2Use &= VP886_DEV_PROFILE_IO2_USE_BITS;
        ioDirReg &= ~VP886_R_IODIR_IOD2;

        /* Adjust the IO2 fields */
        if (io2Use == VP886_DEV_PROFILE_IO2_USE_DIG_SIG) {
            /* Mask the IO2 interrupt, default to input */
            ioDirReg |= VP886_R_IODIR_IOD2_INPUT;
            pDevObj->registers.intMask[channelId] |= VP886_R_INTMASK_IO2;
        } else if (io2Use == VP886_DEV_PROFILE_IO2_USE_DIG_INT) {
            /* Enable IO2 interrupt, default to input */
            ioDirReg |= VP886_R_IODIR_IOD2_INPUT;
            pDevObj->registers.intMask[channelId] &= ~VP886_R_INTMASK_IO2;
        } else if (io2Use == VP886_DEV_PROFILE_IO2_USE_VMM) {
            /* Force IO2 to voltage monitor mode, mask interrupt */
            ioDirReg |= VP886_R_IODIR_IOD2_VMON;
            pDevObj->registers.intMask[channelId] |= VP886_R_INTMASK_IO2;
        }

        /* Write back to the device */
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDirReg);

        /* Update the DEVICE_IO option cache */
        if (pDevObj->ioCapability == VP886_IO_CAPABILITY_TWO_PER_CH) {
        pDevObj->options.deviceIo.directionPins_31_0 &= ~(0x03 << (2 * channelId));
        pDevObj->options.deviceIo.outputTypePins_31_0 &= ~(0x03 << (2 * channelId));
        if ((ioDirReg & VP886_R_IODIR_IOD1) == VP886_R_IODIR_IOD1_DIG_OUTPUT) {
            pDevObj->options.deviceIo.directionPins_31_0 |= (0x01 << (2 * channelId));
        }
        if ((ioDirReg & VP886_R_IODIR_IOD1) == VP886_R_IODIR_IOD1_OD_OUTPUT) {
            pDevObj->options.deviceIo.directionPins_31_0 |= (0x01 << (2 * channelId));
            pDevObj->options.deviceIo.outputTypePins_31_0 |= (0x01 << (2 * channelId));
        }
        if ((ioDirReg & VP886_R_IODIR_IOD2) == VP886_R_IODIR_IOD2_OUTPUT) {
            pDevObj->options.deviceIo.directionPins_31_0 |= (0x02 << (2 * channelId));
            }
        } else if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2) {
            if ((ioDirReg & VP886_R_IODIR_IOD2) == VP886_R_IODIR_IOD2_OUTPUT) {
                pDevObj->options.deviceIo.directionPins_31_0 = 0x01;
            } else {
                pDevObj->options.deviceIo.directionPins_31_0 = 0x00;
            }
        } else {
            VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886InitLine() Unhandled ioCapability value %d", pDevObj->ioCapability));
        }
    }

    /* Set the switching regulator parameters based on line termination types.
       The useLowPowerTiming flag is used to handle the possibility of the
       application mixing termination types without having lines instantiated
       during VpInitDevice().  By default, the flag is true, but any non-lowpower
       termination will set it to false so that Vp886SetSwTimingParams() will
       set the normal params from that point forward. */
    if (pLineObj->termType != VP_TERM_FXS_LOW_PWR && pDevObj->useLowPowerTiming == TRUE) {
        pDevObj->useLowPowerTiming = FALSE;
        Vp886SetSwTimingParams(pDevCtx);
    }

    /* Generate the VP_LINE_EVID_LINE_INIT_CMP event unless called from VpInitDevice(). */
    if (status == VP_STATUS_SUCCESS) {
        if (!(pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE,
                VP_LINE_EVID_LINE_INIT_CMP, 0, Vp886GetTimestamp(pDevCtx), FALSE);
        }
        pLineObj->busyFlags |= VP886_LINE_INIT_CMP;
    }

    /* Initialization complete; other functions may access this line. */
    pLineObj->busyFlags &= ~VP886_LINE_INIT_IN_PROGRESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886InitLineInt-"));

    return status;
} /* Vp886InitLineInt() */


/** Vp886InitLineRegisters()
  This function initializes most of the channel registers.  Some are set to
  device defaults, and some are set to other values needed by the API.

  To avoid changing IO pin outputs, we must manually set each register rather
  than using the software reset command.
*/
void
Vp886InitLineRegisters(
    VpLineCtxType *pLineCtx,
    bool initDevice)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;

    /* The Independent Calibration register is updated upon reset with the Fuse register 6 */
    /* HD2 trim value (BAA), this value can't be guessed and has to be read. */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_INDCAL_RD, VP886_R_INDCAL_LEN, pLineObj->registers.indCal);

    /*** Initialize register caches to device reset values */
    pLineObj->registers.sysState[0] = 0x0F;
    pLineObj->registers.opCond[0] = 0x00;
    pLineObj->registers.opFunc[0] = 0x00;
    pLineObj->registers.calCtrl[0] = 0x00;
    pLineObj->registers.calCtrl[1] = 0x00;
    pLineObj->registers.calCtrl[2] = 0x00;
    pLineObj->registers.icr1[0] = 0x00;
    pLineObj->registers.icr1[1] = 0x00;
    pLineObj->registers.icr1[2] = 0x00;
    pLineObj->registers.icr1[3] = 0x00;
    pLineObj->registers.icr3[0] = 0x00;
    pLineObj->registers.icr3[1] = 0x00;
    pLineObj->registers.icr3[2] = 0x00;
    pLineObj->registers.icr3[3] = 0x00;
    pLineObj->registers.icr4[0] = 0x00;
    pLineObj->registers.icr4[1] = 0x00;
    pLineObj->registers.icr4[2] = 0x00;
    pLineObj->registers.icr4[3] = 0x00;
    pLineObj->registers.ssCfg[0] = 0x22;
    pLineObj->registers.ssCfg[1] = 0x19;
    pLineObj->registers.normCal[0] = 0x00;
    pLineObj->registers.normCal[1] = 0x00;
    pLineObj->registers.normCal[2] = 0x00;
    pLineObj->registers.revCal[0] = 0x00;
    pLineObj->registers.revCal[1] = 0x00;
    pLineObj->registers.revCal[2] = 0x00;
    pLineObj->registers.ringCal[0] = 0x00;
    pLineObj->registers.ringCal[1] = 0x00;
    if (VP886_IS_TRACKER(pDevObj)) {
        pLineObj->registers.batCal[0] = 0x00;
        pLineObj->registers.batCal[1] = 0x00;
    } else { /* ABS */
        pLineObj->registers.batCal[0] = 0x04;
        pLineObj->registers.batCal[1] = 0x00;
    }
    pLineObj->registers.ringDelay[0] = VP886_R_RINGDELAY_DEFAULT;
    if (!initDevice) {
        /* These are read or set up during ConfigLine at the beginning of
           InitDevice, so we don't want to blow them away here */
        pLineObj->registers.loopSup[0] = 0x5B;
        pLineObj->registers.loopSup[1] = 0x84;
        pLineObj->registers.loopSup[2] = 0xB3;
        pLineObj->registers.loopSup[3] = 0x8E;
        pLineObj->registers.loopSup[4] = 0x00;
        if (VP886_IS_TRACKER(pDevObj)) {
            pLineObj->registers.dcFeed[0] = 0xD2;
            pLineObj->registers.dcFeed[1] = 0x08;
        } else { /* ABS */
            pLineObj->registers.dcFeed[0] = 0xD1;
            pLineObj->registers.dcFeed[1] = 0x08;
        }
        pLineObj->registers.icr2[0] = 0x00;
        pLineObj->registers.icr2[1] = 0x00;
        pLineObj->registers.icr2[2] = 0x00;
        pLineObj->registers.icr2[3] = 0x00;
    }

    if (!initDevice) {
        /*** Write the cached registers that we are not immediately changing from
           the reset values.  During InitDevice this is not necessary, due to the
           hardware reset. */
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, pLineObj->registers.opCond);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, pLineObj->registers.calCtrl);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, pLineObj->registers.icr1);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_NORMCAL_WRT, VP886_R_NORMCAL_LEN, pLineObj->registers.normCal);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_REVCAL_WRT, VP886_R_REVCAL_LEN, pLineObj->registers.revCal);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN, pLineObj->registers.ringCal);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, pLineObj->registers.dcFeed);
        if (VP886_IS_SF(pDevObj)) {
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_RINGDELAY_WRT, VP886_R_RINGDELAY_LEN, pLineObj->registers.ringDelay);
        }


        /*** Set non-cached registers to device reset values.  Some will be
           excluded because they are set immediately by ConfigLine or by
           VpImplementDefaultSettings.  Others are excluded because they do not
           matter while we are not using them. */
        {
            uint8 sadc[VP886_R_SADC_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            uint8 vadc[VP886_R_VADC_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            uint8 testModeAccess[VP886_R_TM_ACCESS_LEN] = {0x00};
            uint8 metering[VP886_R_METER_LEN] = {0x21, 0x3D, 0x3C, 0x00};
            uint8 siggenCtrl[VP886_R_SIGCTRL_LEN] = {0x00};
            uint8 chanTimer[VP886_R_CHTIMER_LEN] = {0x00, 0x00};
            uint8 cadence[VP886_R_CADENCE_LEN] = {0x01, 0x90, 0x03, 0x20};
            uint8 cidParam[VP886_R_CIDPARAM_LEN] = {0x03};
            uint8 icr6[VP886_R_ICR6_LEN] = {0x00, 0x00, 0x00, 0x00};

            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, sadc);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, vadc);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_TM_ACCESS_WRT, VP886_R_TM_ACCESS_LEN, testModeAccess);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_METER_WRT, VP886_R_METER_LEN, metering);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SIGCTRL_WRT, VP886_R_SIGCTRL_LEN, siggenCtrl);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_CHTIMER_WRT, VP886_R_CHTIMER_LEN, chanTimer);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_CADENCE_WRT, VP886_R_CADENCE_LEN, cadence);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_CIDPARAM_WRT, VP886_R_CIDPARAM_LEN, cidParam);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN, icr6);
        }
    }


    /*** Modify cached values to desired init values */
    /* Disconnect instead of shutdown */
    pLineObj->registers.sysState[0] = VP886_R_STATE_SS_DISCONNECT;
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);

    if (VP886_IS_TRACKER(pDevObj)) {
        /* Set switcher params based on the device data */
        VpMemCpy(pLineObj->registers.swParam, pDevObj->swParams, VP886_R_SWPARAM_LEN);

        /* Enable 100V power supply limit */
        pLineObj->registers.icr2[2] |= VP886_R_ICR2_SW_LIM;
        pLineObj->registers.icr2[3] &= ~VP886_R_ICR2_SW_LIM;
        pLineObj->registers.icr2[3] |= VP886_R_ICR2_SW_LIM_100;
    }

    /* Enable dither cancel correction */
    pLineObj->registers.indCal[1] |= VP886_R_INDCAL_DITHER_CORR_EN;

    /* VREF should remain enabled */
    pLineObj->registers.icr3[0] |= VP886_R_ICR3_VREF_EN;
    pLineObj->registers.icr3[1] |= VP886_R_ICR3_VREF_EN;

    /* Disable auto system state control */
    pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_AUTO_SYSSTATE;

    /* Disable auto CFAIL shutdown */
    pLineObj->registers.ssCfg[0] &= ~VP886_R_SSCFG_AUTO_CFAIL;

    /* Disable auto overcurrent shutdown.  OC interrupts need to be filtered by
       the API. */
    pLineObj->registers.ssCfg[1] &= ~VP886_R_SSCFG_AUTO_OVERCURRENT;

    /* Enable charge undervoltage shutdown if requested by the device profile */
    if (pDevObj->devProfileData.cpProtection == VP886_CP_PROT_UV_SHUTDOWN) {
        pLineObj->registers.ssCfg[1] |= VP886_R_SSCFG_AUTO_CP_UV;
    } else {
        pLineObj->registers.ssCfg[1] &= ~VP886_R_SSCFG_AUTO_CP_UV;
    }

    if (VP886_IS_ABS(pDevObj)) {
        /* Disable automatic low power state transitions for ABS.  This is to
           prevent the battery from collapsing due to the device automatically
           switching to the active state before the API increases switcher power. */
        pLineObj->registers.ssCfg[1] &= ~VP886_R_SSCFG_AUTO_SYSSTATE_LPM;
        pLineObj->registers.ssCfg[1] |= VP886_R_SSCFG_AUTO_SYSSTATE_LPM_DIS;

        /* Use the "new" battery switch algorithm */
        pLineObj->registers.batCal[0] &= ~VP886_R_BATCAL_ABS_ALG;
        pLineObj->registers.batCal[0] |= VP886_R_BATCAL_ABS_ALG_NEW;

        /* Switcher parameters for each line are written in InitDevice.  Read
           them into the line object cache. */
        VpSlacRegRead(VP_NULL, pLineCtx, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);
    }

    /* Activate programmed AC filter coefficients. */
    pLineObj->registers.opFunc[0] |= VP886_R_OPFUNC_ALL_FILTERS;

    /* Enabled hook and groundkey averaging to prevent glitches from power
       supply interference */
    pLineObj->registers.loopSup[4] |= VP886_R_LOOPSUP_HOOK_AVG_EN;

    /* Use debounced hook signal to control hook hysteresis */
    pLineObj->registers.loopSup[4] |= VP886_R_LOOPSUP_HSH_CTL_DBNC;

    /*** Write the modified cached registers */
    if (VP886_IS_TRACKER(pDevObj)) {
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);
    }
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->registers.icr3);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_INDCAL_WRT, VP886_R_INDCAL_LEN, pLineObj->registers.indCal);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_BATCAL_WRT, VP886_R_BATCAL_LEN, pLineObj->registers.batCal);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_OPFUNC_WRT, VP886_R_OPFUNC_LEN, pLineObj->registers.opFunc);

    /*** Write non-cached values that we want to change from the defaults */
    if (VP886_REVISION(pDevObj) == VP886_R_RCNPCN_RCN_AAA) {
        /* Only revision AAA silicon requires non-default dither cancel
           parameters. */
        uint8 dithCancelAAA[VP886_R_DCANCEL_LEN] = {0xFD, 0xFC, 0xFD, 0xFE,
                                                    0xFF, 0x00, 0x01, 0x02,
                                                    0x03, 0x04, 0x03, 0x02,
                                                    0x01, 0x00, 0xFF, 0xFE};

        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_DCANCEL_WRT, VP886_R_DCANCEL_LEN, dithCancelAAA);
    }
    {
        uint8 icr5[VP886_R_ICR5_LEN] = {0x4A, 0xB0, 0xAA};
        /* Adjust the DC Feed and power supply speedup hold time in
           ICR5 depending on the CHL Capacitor speed up setting CHLSU in the
           Device mode register */
        switch (pDevObj->devProfileData.devMode[1] & VP886_R_DEVMODE_SPEEDUP_CUR_LIM) {
            /* Single ended drive, 200uA current limit */
            case 0x00:
            /* Differential drive, 200uA current limit */
            case 0x04:
                /* feed_hold = 12 ms bat_hold = 12 ms */
                icr5[0] = 0xCC;
                break;
            /* Differential drive, 400uA current limit */
            case 0x08:
                /* feed_hold = 8 ms bat_hold = 10 ms */
                icr5[0] = 0x8A;
                break;
            /* Differential drive, 800uA current limit */
            case 0x0C:
                /* feed_hold = 4 ms bat_hold = 10 ms */
                icr5[0] = 0x4A;
                break;
            default:
                break;
        }
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR5_WRT, VP886_R_ICR5_LEN, icr5);
    }
    /* Set timeslots based on channelId */
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_TXSLOT_WRT, VP886_R_TXSLOT_LEN, &channelId);
    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_RXSLOT_WRT, VP886_R_RXSLOT_LEN, &channelId);

    return;
}


/** Vp886ConfigLine()
  Implements VpConfigLine() to apply AC, DC, and/or Ringing profiles to a line.

  See the VP-API-II Reference Guide for more details on VpConfigLine().
*/
VpStatusType
Vp886ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886ConfigLine");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886ConfigLine");
        return status;
    }

    /* Check for valid profile arguments. */
    if (
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_AC, &pAcProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_DC, &pDcOrFxoProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_RING, &pRingProfile) != VP_STATUS_SUCCESS)
    ) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886ConfigLine");
        return VP_STATUS_ERR_PROFILE;
    }

    status = Vp886ConfigLineInt(pLineCtx, pAcProfile, pDcOrFxoProfile, pRingProfile, FALSE);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886ConfigLine");
    return status;
}   /* Vp886ConfigLine() */


/** Vp886ConfigLineInt()
  This function can be called internally to skip device status checks
  and critical sections
*/
static VpStatusType
Vp886ConfigLineInt(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile,
    bool initDevice)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status;

    if (pLineObj->isFxs) {
        status = Vp886ConfigLineFxs(pLineCtx, pAcProfile, pDcOrFxoProfile, pRingProfile, initDevice);
    } else {
        VP_ERROR(VpLineCtxType, pLineCtx, ("ConfigLine not supported for FXO"));
        return VP_STATUS_INVALID_ARG;
    }

    return status;
}


/** Vp886ConfigLineFxs()
  Apply profiles for an FXS line.
*/
VpStatusType
Vp886ConfigLineFxs(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    bool initDevice)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint8 loopSupBuf[VP886_R_LOOPSUP_LEN];
    uint8 swParamBuf[VP886_R_SWPARAM_LEN];
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886ConfigLineFxs+"));

    /* Get local copies of the Loop Supervision Parameters and Switching Reglator Parameters
       in preparation for modify/write */
    if ((pDcProfile != VP_NULL) || (pRingProfile != VP_NULL)) {

        if (initDevice) {
            /* If called from InitDevice, some registers we use here have not
               been cached yet.*/
            /* Read the Loop Supervision Reg from the part */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_LOOPSUP_RD, VP886_R_LOOPSUP_LEN, loopSupBuf);
            /* Read ICR2 from the part */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_ICR2_RD, VP886_R_ICR2_LEN, pLineObj->registers.icr2);
            if (VP886_IS_TRACKER(pDevObj)) {
                /* Use the Switcher Params specified in the Device Profile */
                VpMemCpy(swParamBuf, pDevObj->swParams, VP886_R_SWPARAM_LEN);
            }
        } else {
            /* Otherwise, just use the cached values */
            VpMemCpy(loopSupBuf, pLineObj->registers.loopSup, VP886_R_LOOPSUP_LEN);
            if (VP886_IS_TRACKER(pDevObj)) {
                VpMemCpy(swParamBuf, pLineObj->registers.swParam, VP886_R_SWPARAM_LEN);
            }
        }
    }

#ifdef VP_CSLAC_SEQ_EN
    /* If a ring profile is provided, stop any on-going msg wait pulse */
    if (pRingProfile != VP_NULL && pLineObj->sendSignal.active &&
        pLineObj->sendSignal.type == VP_SENDSIG_MSG_WAIT_PULSE)
    {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886ConfigLineFxs - New ring profile forces msg wait pulse to end"));
        Vp886SendSignalStop(pLineCtx, TRUE);
    }
#endif

    /* Process AC profile (if any) */
    if (pAcProfile != VP_NULL) {
        const uint8 *pData = pAcProfile + VP_PROFILE_DATA_START;

        /* Add AC parameters to SLAC write buffer */
        {
            uint8 mpiLength = pAcProfile[VP_PROFILE_MPI_LEN] - 1;
            uint8 cmd = *pData++;
            VpSlacRegWrite(VP_NULL, pLineCtx, cmd, mpiLength, pData);
        }
    }

    /* Process DC profile (if any) */
    if (pDcProfile != VP_NULL) {
        const uint8 *pData = pDcProfile + VP_PROFILE_DATA_START;

        /* Add DC parameters to SLAC write buffer */
        {
            uint8 mpiLength = pDcProfile[VP_PROFILE_MPI_LEN] - 1;
            uint8 cmd = *pData++;
            VpSlacRegWrite(VP_NULL, pLineCtx, cmd, mpiLength, pData);

            /* Save the dcFeed parameters */
            if (cmd == VP886_R_DCFEED_WRT) {
                pLineObj->registers.dcFeed[0] = pData[0];
                pLineObj->registers.dcFeed[1] = pData[1];
            } else {
                VP_WARNING(VpLineCtxType, pLineCtx, ("Could not save dcFeed data from DC profile"));
                /* Read back from the register to make sure we have these values */
                VpSlacRegRead(VP_NULL, pLineCtx, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN, pLineObj->registers.dcFeed);
            }

            pData += mpiLength;
        }

        /* Update Loop Supervision Parameters (read-modify-write) */
        loopSupBuf[0] &= ~(VP886_R_LOOPSUP_GKEY_ABS | VP886_R_LOOPSUP_GKEY_THRESH | VP886_R_LOOPSUP_HOOK_THRESH);
        loopSupBuf[0] |= (*pData++ & (VP886_R_LOOPSUP_GKEY_ABS | VP886_R_LOOPSUP_GKEY_THRESH | VP886_R_LOOPSUP_HOOK_THRESH));
        loopSupBuf[1] = *pData++;
        loopSupBuf[4] &= ~(VP886_R_LOOPSUP_LPM_HOOK_HYST | VP886_R_LOOPSUP_GKEY_HYST | VP886_R_LOOPSUP_HOOK_HYST);
        loopSupBuf[4] |= (*pData++ & (VP886_R_LOOPSUP_LPM_HOOK_HYST | VP886_R_LOOPSUP_GKEY_HYST | VP886_R_LOOPSUP_HOOK_HYST));
        loopSupBuf[3] &= ~VP886_R_LOOPSUP_LPM_HOOK_THRESH;
        loopSupBuf[3] |= (*pData++ & VP886_R_LOOPSUP_LPM_HOOK_THRESH);

        /* Use the value of the GKEY_ABS bit to determine later whether to
           report GKEY interrupts as GKEY events or DC_FLT events */
        pLineObj->reportDcFaults = (loopSupBuf[0] & VP886_R_LOOPSUP_GKEY_ABS) ? TRUE : FALSE;

        /* Save the target VOC for calibration */
        pLineObj->targetVoc = ((pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VOC) >> 2) * 3;
        if (pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VOCSHIFT) {
            pLineObj->targetVoc += 12;
        } else {
            pLineObj->targetVoc += 36;
        }

        if (VP886_IS_TRACKER(pDevObj)) {
            uint8 lpBatt;

            /* Update Switching Regulator Parameters (read-modify-write) */
            swParamBuf[0] &= ~VP886_R_SWPARAM_FLOOR_V;
            swParamBuf[0] |= (*pData & VP886_R_SWPARAM_FLOOR_V);
            pLineObj->floorVoltage = (swParamBuf[0] & VP886_R_SWPARAM_FLOOR_V) * 5 + 5;

            /* Set the low power switcher voltage to 4V above VOC, rounded to
               the nearest 5V step.  Calibration will apply the extra precision
               by using a target of VOC + 4. */
            lpBatt = VpRoundedDivide(pLineObj->targetVoc + 4, 5) - 1;

            swParamBuf[2] &= ~VP886_R_SWPARAM_LOWPOWER_V;
            swParamBuf[2] |= (lpBatt & VP886_R_SWPARAM_LOWPOWER_V);

            pData++;
        }

        /* update the topology info */
        if (pDcProfile[VP_PROFILE_VERSION] == 0) {
            pData++;
            pData++;
            /* There is not enough information to work backwards to any certain
               resistance values. */
            pLineObj->lineTopology.rInsideDcSense = 0x0000;
            pLineObj->lineTopology.rOutsideDcSense = 0x0000;
        } else if (pDcProfile[VP_PROFILE_VERSION] == 1) {
            pData++;
            /* Work backwards from the ir_overhead setting to determine whether
               the provided resistance was specified as inside or outside the
               DC sense point. */
            if ((pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_IR_OVERHEAD) <= 0x40) {
                /* resistance is outside the DC sense point */
                pLineObj->lineTopology.rInsideDcSense = 0x0000;
                pLineObj->lineTopology.rOutsideDcSense = *pData++;
            } else {
                /* resistance is inside the DC sense point */
                pLineObj->lineTopology.rInsideDcSense = *pData++;
                pLineObj->lineTopology.rOutsideDcSense = 0x0000;
            }
        } else {
            pLineObj->lineTopology.rOutsideDcSense = *pData++;
            pLineObj->lineTopology.rInsideDcSense = *pData++;
        }
    }

    /* Process Ringing profile (if any) */
    if (pRingProfile != VP_PTABLE_NULL) {
        const uint8 *pData = pRingProfile + VP_PROFILE_DATA_START;

        /* Add Ringing parameters to SLAC write buffer */
        {
            uint8 mpiLength = pRingProfile[VP_PROFILE_MPI_LEN] - 1;
            uint8 cmd = *pData++;
            uint8 ringGen[VP886_R_RINGGEN_LEN];
            const uint8 *pRingGen;
            VpSlacRegWrite(VP_NULL, pLineCtx, cmd, mpiLength, pData);

            /* Save parameters that we may need to use later */
            if (cmd == VP886_R_RINGGEN_WRT) {
                pRingGen = pData;
            } else {
                VP_WARNING(VpLineCtxType, pLineCtx, ("Could not save ringGen data from Ringing profile"));
                /* Read back from register to make sure we have these values */
                VpSlacRegRead(VP_NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN, ringGen);
                pRingGen = ringGen;
            }
            pLineObj->ringBias = ((pRingGen[1] << 8) | pRingGen[2]);
            pLineObj->ringAmplitude = ((pRingGen[5] << 8) | pRingGen[6]);
            if ((pData[0] & VP886_R_RINGGEN_WAVE) == VP886_R_RINGGEN_WAVE_SINE) {
                /* Sine, FRQA */
                pLineObj->ringFrequency = ((pRingGen[3] << 8) | pRingGen[4]);
                pLineObj->ringSine = TRUE;
            } else {
                /* Trapezoidal, FRQB */
                pLineObj->ringFrequency = ((pRingGen[7] << 8) | pRingGen[8]);
                pLineObj->ringSine = FALSE;
            }
            if (pRingGen[0] & VP886_R_RINGGEN_RT_CYCDUR) {
                pLineObj->ringTripCycles = 1;
            } else {
                pLineObj->ringTripCycles = 2;
            }

            pData += mpiLength;
        }

        /* For ABS devices, calculate the battery level required to support
           the ringing signal */
        if (VP886_IS_ABS(pDevObj)) {
            int32 amplitude_mV;
            int32 bias_mV;
            
            /* Amplitude is a 16-bit value with a +/- 154.4V range.  This
               means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
            amplitude_mV = VpRoundedDivide(pLineObj->ringAmplitude * VP886_AMP_STEP_NUM, VP886_AMP_STEP_DEN); 

            /* DC Bias is a 16-bit value with a +/- 154.4V range.  This
               means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
            bias_mV = VpRoundedDivide(pLineObj->ringBias * VP886_BIAS_STEP_NUM, VP886_BIAS_STEP_DEN); 
        
            /* Amplitude + Bias, rounded up */
            pDevObj->absRingingPeak[channelId] = (ABS(amplitude_mV) + ABS(bias_mV) + 999) / 1000;
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Vp886ConfigLineFxs: absRingingPeak[%d] = %d",
                channelId, pDevObj->absRingingPeak[channelId]));
        }

        /* Update Loop Supervision Parameters */
        loopSupBuf[2] = *pData;
        pData++;
        /* Save the rtth value */
        pLineObj->rtth = (loopSupBuf[2] & VP886_R_LOOPSUP_RTRIP_THRESH);

        loopSupBuf[3] &= ~VP886_R_LOOPSUP_RING_CUR_LIM;
        /* Check for whether we're using the normal or low ILR range */
        if (*pData & VP886_RING_PROF_LOW_ILR_FLAG) {
            uint8 rtth;
            /* Low range.  Apply workaround. */
            /* Set a flag to remember that we're using the low ILR range */
            pLineObj->lowIlr = TRUE;
            /* Force ILR to be interpreted at DC feed levels to allow ILR
               below 50mA */
            pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
            pLineObj->registers.icr2[1] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);
            /* Double the ring trip threshold (RTTH) */
            rtth = loopSupBuf[2] & VP886_R_LOOPSUP_RTRIP_THRESH;
            rtth = MIN(rtth * 2, VP886_R_LOOPSUP_RTRIP_THRESH);
            loopSupBuf[2] &= ~VP886_R_LOOPSUP_RTRIP_THRESH;
            loopSupBuf[2] |= rtth;

            /* We will read the ILR programming value from a later byte */

        } else {
            /* Normal range. */
            loopSupBuf[3] |= (*pData & VP886_R_LOOPSUP_RING_CUR_LIM);
            /* If the low range workaround was previously applied, undo it */
            if (pLineObj->lowIlr) {
                pLineObj->lowIlr = FALSE;
                pLineObj->registers.icr2[0] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
                VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);
            }
        }
        pData++;

        /* Save the ring trip algorithm so that it can be restored if changed
           by something such as msg waiting pulse */
        pLineObj->ringTripAlg = (loopSupBuf[2] & VP886_R_LOOPSUP_RTRIP_ALG);

        /* Update Switching Regulator Parameters (read-modify-write) */
        if (VP886_IS_TRACKER(pDevObj)) {
            uint8 swrv = *pData & VP886_R_SWPARAM_RINGING_V;
            /* Add 5V overhead to what is in the profile */
            if (swrv + 1 < VP886_R_SWPARAM_RINGING_V) {
                swrv += 1;
            }
            /* Disallow setting the switcher voltage below 60 so that it can not
               run into VOC when entering ringing. */
            if (swrv < 11) {
                swrv = 11;
            }
            swParamBuf[1] &= ~VP886_R_SWPARAM_RINGING_V;
            swParamBuf[1] |= (swrv & VP886_R_SWPARAM_RINGING_V);
            swParamBuf[0] &= ~VP886_R_SWPARAM_RING_TRACKING;
            swParamBuf[0] |= (*pData & VP886_R_SWPARAM_RING_TRACKING);
            /* If using a shared supply, force fixed ringing mode. */
            if ((pDevObj->stateInt & VP886_SHARED_SUPPLY) &&
                (swParamBuf[0] & VP886_R_SWPARAM_RING_TRACKING) == VP886_R_SWPARAM_RING_TRACKING_EN)
            {
                VP_WARNING(VpLineCtxType, pLineCtx, ("Shared supply configuration does not support tracked ringing"));
                swParamBuf[0] &= ~VP886_R_SWPARAM_RING_TRACKING;
                swParamBuf[0] |= VP886_R_SWPARAM_RING_TRACKING_DIS;
            }
        }
        pData++;

        /* If the flag earlier in the profile was set to indicate low ILR range,
           read the actual value to program from here. */
        if (pLineObj->lowIlr) {
            loopSupBuf[3] |= ((*pData & VP886_RING_PROF_LOW_ILR_MASK) >> 1);
        }
        /* Save the Balanced/Unbalanced Ringing flag */
        pLineObj->unbalancedRinging = (*pData & VP886_RING_PROF_BAL_UNBAL) ? TRUE : FALSE;
        pData++;
    }

    /* Add the  updated Loop Supervision Parameters and Switching Reglator
       Parameters to the SLAC write buffer */
    if ((pDcProfile != VP_NULL) || (pRingProfile != VP_NULL)) {
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, loopSupBuf);
        VpMemCpy(pLineObj->registers.loopSup, loopSupBuf, VP886_R_LOOPSUP_LEN);
        if (VP886_IS_TRACKER(pDevObj)) {
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, swParamBuf);
            if (initDevice) {
                VpMemCpy(pDevObj->swParams, swParamBuf, VP886_R_SWPARAM_LEN);
            } else {
                VpMemCpy(pLineObj->registers.swParam, swParamBuf, VP886_R_SWPARAM_LEN);
            }
        }
    }

    /* Update cached transmit and receive gains. Initialize the ABS_GAIN to unknown */
    if (pAcProfile != VP_NULL) {

        uint8 gainCSD[VP886_R_GX_LEN];

        VpSlacRegRead(NULL, pLineCtx, VP886_R_GX_RD, VP886_R_GX_LEN, gainCSD);
        pLineObj->gxBase = 0x4000 + VpConvertCsd2Fixed(gainCSD);
        pLineObj->gxUserLevel = 0x4000;
        pLineObj->options.absGain.gain_AToD = VP_ABS_GAIN_UNKNOWN;

        VpSlacRegRead(NULL, pLineCtx, VP886_R_GR_RD, VP886_R_GR_LEN, gainCSD);
        pLineObj->grBase = VpConvertCsd2Fixed(gainCSD);
        pLineObj->grUserLevel = 0x4000;
        pLineObj->options.absGain.gain_DToA = VP_ABS_GAIN_UNKNOWN;

    }

    if (!initDevice) {
        if (pDevObj->calData[channelId].valid) {
            /* Apply the DC calibration */
            if (pDcProfile != VP_NULL) {
                Vp886ApplyCalDC(pLineCtx);
            }
            /* Apply the Ringing calibration */
            if (pRingProfile != VP_NULL) {
                Vp886ApplyCalRing(pLineCtx);
            }
            if (pDcProfile != VP_NULL || pRingProfile != VP_NULL) {
                Vp886ProgramCalRegisters(pLineCtx);
            }
        }

        if (pRingProfile != VP_NULL) {
            /* ABS switcher settings may need to be updated to support new
               ringing params */
            if (VP886_IS_ABS(pDevObj)) {
                Vp886ManageABSRingingBatt(pDevCtx, channelId == 0 ? FALSE : TRUE, channelId == 1 ? FALSE : TRUE);
            }

        #ifdef VP886_INCLUDE_ADAPTIVE_RINGING
            /* Initialize the thermal ringing power adaption module */
            status = Vp886AdaptiveRingingInit(pLineCtx);
        #endif
        }
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886ConfigLineFxs-"));

    return status;
}


/** Vp886InitProfile()
  Implements VpInitProfile() to initialize an entry in the profile table.

  See the VP-API-II Reference Guide for more details on VpInitProfile().
*/
VpStatusType
Vp886InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886InitProfile");

    if (type >= VP886_NUM_PROFILE_TYPES) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886InitProfile - Invalid profile type"));
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitProfile");
        return VP_STATUS_INVALID_ARG;
    }

    status = Vp886SetProfTable(pDevCtx, pProfile, pProfileIndex, type);

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886InitProfile");
    return status;
} /* Vp886InitProfile() */


/** Vp886FreeRun()
  Implements VpFreeRun() to handle a system restart or removal of PCLK.

  See the VP-API-II Reference Guide for more details on VpFreeRun().
*/
VpStatusType
Vp886FreeRun(
    VpDevCtxType *pDevCtx,
    VpFreeRunModeType freeRunMode)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886FreeRun");

    if (freeRunMode == VP_FREE_RUN_STOP) {
        Vp886FreeRunStop(pDevCtx);
        /* Clear the device as being forced into free run mode by application.
           This allows PCLK fault detection to automatically enter/exit free
           run mode. */
        pDevObj->stateInt &= ~VP886_FORCE_FREE_RUN;
    } else {
        /* Mark the device as being forced into free run mode by application.
           This prevents auto-recovery when PCLK is restored. */
        pDevObj->stateInt |= VP886_FORCE_FREE_RUN;
        Vp886FreeRunStart(pDevCtx);
    }

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886FreeRun");
    return VP_STATUS_SUCCESS;
} /* Vp886FreeRun() */


/** Vp886FreeRunStart()
  Enters free-run mode.  This may be called automatically by the API when a
  clock fault is detected, or by the application through
  VpFreeRun(VP_FREE_RUN_START).

  Entering free-run mode means that both lines will exit ringing and CID, and the
  switcher timing parameters will be replaced with the free-run timing parameters
  from the device profile.
*/
void
Vp886FreeRunStart(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    Vp886LineObjectType *pLineObj;
    uint8 channelId;
    VpLineStateType lineState;

    /* Take the lines out of Ringing and stop caller ID if necessary */
    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            lineState = pLineObj->lineState.usrCurrent;

            if (lineState == VP_LINE_RINGING) {
                Vp886SetLineStateFxs(pLineCtx, VP_LINE_STANDBY);
            }
            if (lineState == VP_LINE_RINGING_POLREV) {
                Vp886SetLineStateFxs(pLineCtx, VP_LINE_STANDBY_POLREV);
            }
#ifdef VP_CSLAC_SEQ_EN
            if (pLineObj->cid.active) {
                Vp886CidStop(pLineCtx);
            }
#endif
        }
    }

    /* Load the free run timing parameters */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWTIMING_WRT, VP886_R_SWTIMING_LEN, pDevObj->swTimingParamsFR);

    /* If the application manually called VpFreeRun, mask clock fault and
       timestamp rollover interrupts */
    if (pDevObj->stateInt & VP886_FORCE_FREE_RUN) {
        pDevObj->registers.intMask[0] |= VP886_R_INTMASK_CFAIL;
        pDevObj->registers.intMask[3] |= VP886_R_INTMASK_TIMESTAMP;
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);
    }

} /* Vp886FreeRunStart() */


/** Vp886FreeRunStart()
  Exits free-run mode.  This may be called automatically by the API when a
  clock fault is cleared, or by the application through
  VpFreeRun(VP_FREE_RUN_STOP).

  Exiting free-run mode means that the switcher timing parameters will be
  restored to the normal values from the device profile.
*/
void
Vp886FreeRunStop(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    /* Restore the original timing  parameters  */
    Vp886SetSwTimingParams(pDevCtx);

    /* If the application manually called VpFreeRun, unmask interrupts */
    if (pDevObj->stateInt & VP886_FORCE_FREE_RUN) {
        pDevObj->registers.intMask[0] &= ~VP886_R_INTMASK_CFAIL;
        pDevObj->registers.intMask[3] &= ~VP886_R_INTMASK_TIMESTAMP;
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);
    }
} /* Vp886FreeRunStop() */


/** Vp886SubDefaultProfiles()
  Replaces NULL profile pointers with pointers to internal default profiles.
  This is used by InitLine and InitDevice to make sure that everything is
  initialized to a known state, with cached values consistent with register
  values.
*/
void
Vp886SubDefaultProfiles(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType *ppAcProfile,
    VpProfilePtrType *ppDcProfile,
    VpProfilePtrType *ppRingProfile)
{
    VpDeviceType deviceType = pDevCtx->deviceType;

    /* If no AC profile is provided, use power-up default values. */
    if (*ppAcProfile == VP_NULL) {
        if (deviceType == VP_DEV_886_SERIES) {
            *ppAcProfile = VP886_INTERNAL_DEFAULT_AC_PROFILE;
        } else {
            *ppAcProfile = VP887_INTERNAL_DEFAULT_AC_PROFILE;
        }
    }

    /* If no DC profile is provided, use power-up default values. */
    if (*ppDcProfile == VP_NULL) {
        if (deviceType == VP_DEV_886_SERIES) {
            *ppDcProfile = VP886_INTERNAL_DEFAULT_DC_PROFILE;
        } else {
            *ppDcProfile = VP887_INTERNAL_DEFAULT_DC_PROFILE;
        }
    }

    /* If no Ringing profile is provided, use power-up default values. */
    if (*ppRingProfile == VP_NULL) {
        if (deviceType == VP_DEV_886_SERIES) {
            *ppRingProfile = VP886_INTERNAL_DEFAULT_RING_PROFILE;
        } else {
            *ppRingProfile = VP887_INTERNAL_DEFAULT_RING_PROFILE;
        }
    }

    return;
}


/** Vp886SetSwTimingParams()
  This is used to combine the normal switching regulator timing parameters with
  the low power mode timing parameters when appropriate.  It is also used to
  restore normal timing parameters when exiting free-run mode.
*/
void
Vp886SetSwTimingParams(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 intSwParam[VP886_R_SWTIMING_LEN];

    VpMemCpy(intSwParam, pDevObj->swTimingParams, VP886_R_SWTIMING_LEN);

    if (pDevObj->useLowPowerTiming) {
        intSwParam[0] = pDevObj->swTimingParamsLP[0];
        intSwParam[1] = pDevObj->swTimingParamsLP[1];
    }

    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWTIMING_WRT, VP886_R_SWTIMING_LEN, intSwParam);

    return;
}


/** Vp886ChangeDeviceType()
  Reprograms a tracker device to look and act like an ABS device, or vice versa.
  This is done when the device is shared supply capable and the application
  specifies a device type in VpMakeDeviceObject() that does not match the device
  PCN.
*/
void
Vp886ChangeDeviceType(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 test1;
    uint8 fuse7;

    VpSlacRegRead(pDevCtx, NULL, VP886_R_TEST_REG_1_RD, VP886_R_TEST_REG_1_LEN, &test1);
    VpSlacRegRead(pDevCtx, NULL, VP886_R_FUSE_REG_7_RD, VP886_R_FUSE_REG_7_LEN, &fuse7);

    /* Force VpSlacRegWrite() calls to insert an EC write with the TME bit set
       (test mode enable).  Doing it this way should allow this function to work
       whether buffered writes are enabled or not. */
    pDevObj->ecVal = VP886_R_EC_GLOBAL | VP886_R_EC_TME;

    /* Enable fuse test mode */
    test1 &= ~VP886_R_TEST_REG_1_MODE;
    test1 |= VP886_R_TEST_REG_1_MODE_FUSE;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_TEST_REG_1_WRT, VP886_R_TEST_REG_1_LEN, &test1);

    /* Fuse register 7 sets the PCN byte of the RCN/PCN register. */
    if (pDevCtx->deviceType == VP_DEV_886_SERIES) {
        /* Clearing this bit changes a tracker PCN to ABS */
        fuse7 &= ~VP886_R_RCNPCN_PCN_ABS_TRACKER;
        VP_INFO(VpDevCtxType, pDevCtx, ("Converting shared-supply device to ABS."));
    } else {
        /* Setting this bit changes an ABS PCN to tracker */
        fuse7 |= VP886_R_RCNPCN_PCN_ABS_TRACKER;
        VP_INFO(VpDevCtxType, pDevCtx, ("Converting shared-supply device to Tracker."));
    }
    pDevObj->ecVal = VP886_R_EC_GLOBAL;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_FUSE_REG_7_WRT, VP886_R_FUSE_REG_7_LEN, &fuse7);

    /* Clear the test mode */
    pDevObj->ecVal = VP886_EC_GLOBAL | VP886_R_EC_TME;
    test1 &= ~VP886_R_TEST_REG_1_MODE;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_TEST_REG_1_WRT, VP886_R_TEST_REG_1_LEN, &test1);

    return;
}


#ifdef VP886_INCLUDE_MPI_QUICK_TEST
/** Vp886QuickMpiTest()
  Performs some basic tests of the MPI and HAL implementation to make sure that
  we can properly communicate with the device before starting.
*/
VpStatusType
Vp886QuickMpiTest(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 writeBuffer[255];
    uint8 readBuffer[16];
    uint8 i;

    /* If the device does not have the second channel registers, run a different
       version of the test */
    if (VP886_IS_SF1(pDevObj)) {
        return Vp886QuickMpiTest1Ch(pDevCtx);
    }

    /* Flush whatever might be in the write buffer */
    if (pDevObj->slacBufData.buffering == TRUE) {
        Vp886SlacBufFlush(pDevCtx);
    }

    /*
        Test 1:
        Write to single-byte global register
    */
    VpMemSet(writeBuffer, 0xD6, sizeof(writeBuffer));
    VpMemSet(readBuffer, 0x94, sizeof(readBuffer));

    /* Write 0xE5 to the clock slots register.  This should read back as 0x65
       later because the top bit always reads as 0.  We have to make sure to
       use a value with the 0x40 bit set, because that bit will always read
       back as 1 in ZSI mode. */
    writeBuffer[1] = 0xE5;
    VpMpiCmdWrapper(deviceId, VP886_EC_GLOBAL, VP886_R_CLKSLOTS_WRT, VP886_R_CLKSLOTS_LEN, &writeBuffer[1]);
    
    /* Make sure the write buffer was not corrupted */
    if (writeBuffer[0] != 0xD6 || writeBuffer[1] != 0xE5 || writeBuffer[2] != 0xD6) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 1 failed. Write buffer corrupted: 0x%02X 0x%02X 0x%02X",
            writeBuffer[0], writeBuffer[1], writeBuffer[2]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Read back the register */
    VpMpiCmdWrapper(deviceId, VP886_EC_GLOBAL, VP886_R_CLKSLOTS_RD, VP886_R_CLKSLOTS_LEN, &readBuffer[1]);
    
    /* Make sure there was no overflow into the data around byte 1 */
    if (readBuffer[0] != 0x94 || readBuffer[2] != 0x94) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 1 failed. Read buffer overflow: 0x%02X 0x%02X 0x%02X",
            readBuffer[0], readBuffer[1], readBuffer[2]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* The high bit of this register always returns 0, so what we read should
       not be exactly what we wrote.  0xE5 becomes 0x65 */
    if (readBuffer[1] != 0x65) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 1 failed. Read value incorrect: 0x%02X, expected 0x65",
            readBuffer[1]));
        status = VP_STATUS_ERR_SPI;
    }
    
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }
    
    /*
        Test 2:
        Write to multi-byte channel registers
    */
    VpMemSet(writeBuffer, 0xD6, sizeof(writeBuffer));
    VpMemSet(readBuffer, 0x94, sizeof(readBuffer));
    
    /* Write to the cadence register for channel 0 */
    writeBuffer[1] = 0xAB;
    writeBuffer[2] = 0xCD;
    writeBuffer[3] = 0xEF;
    writeBuffer[4] = 0x01;
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_WRT, VP886_R_CADENCE_LEN, &writeBuffer[1]);
    
    /* Write to the cadence register for channel 1 */
    writeBuffer[6] = 0x12;
    writeBuffer[7] = 0x34;
    writeBuffer[8] = 0x56;
    writeBuffer[9] = 0x78;
    VpMpiCmdWrapper(deviceId, VP886_EC_2, VP886_R_CADENCE_WRT, VP886_R_CADENCE_LEN, &writeBuffer[6]);
    
    /* Make sure the write buffer was not corrupted */
    if (writeBuffer[0] != 0xD6 || writeBuffer[1] != 0xAB || writeBuffer[2] != 0xCD ||
        writeBuffer[3] != 0xEF || writeBuffer[4] != 0x01 || writeBuffer[5] != 0xD6 ||
        writeBuffer[6] != 0x12 || writeBuffer[7] != 0x34 || writeBuffer[8] != 0x56 ||
        writeBuffer[9] != 0x78 || writeBuffer[10] != 0xD6)
    {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 2 failed. Write buffer corrupted: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            writeBuffer[0], writeBuffer[1], writeBuffer[2], writeBuffer[3], writeBuffer[4], writeBuffer[5],
            writeBuffer[6], writeBuffer[7], writeBuffer[8], writeBuffer[9], writeBuffer[10]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Read back both registers */
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN, &readBuffer[1]);
    VpMpiCmdWrapper(deviceId, VP886_EC_2, VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN, &readBuffer[6]);

    /* Make sure there was no overflow into the surrounding data */
    if (readBuffer[0] != 0x94 || readBuffer[5] != 0x94 || readBuffer[10] != 0x94) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 2 failed. Read buffer overflow: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4], readBuffer[5],
            readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9], readBuffer[10]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Make sure the ch0 data read back correctly.
       Some bits of the register will always read as 0, so we should read
       0x03CD0701 instead of 0xABCDEF01 */
    if (readBuffer[1] != 0x03 || readBuffer[2] != 0xCD || readBuffer[3] != 0x07 || readBuffer[4] != 0x01) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 2 failed. Ch0 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X",
            readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: expected 0x03 0xCD 0x07 0x01"));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Make sure the ch1 data read back correctly.
       Some bits of the register will always read as 0, so we should read
       0x02340678 instead of 0x12345678 */
    if (readBuffer[6] != 0x02 || readBuffer[7] != 0x34 || readBuffer[8] != 0x06 || readBuffer[9] != 0x78) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 2 failed. Ch1 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X",
            readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: expected 0x02 0x34 0x06 0x78"));
        status = VP_STATUS_ERR_SPI;
    }

    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /*
        Test 3:
        Test large write length
    */
    VpMemSet(writeBuffer, 0xD6, sizeof(writeBuffer));
    VpMemSet(readBuffer, 0x94, sizeof(readBuffer));

    /* Build a large write buffer where the first command is a write to the ch0
       cadence register, followed by repeated writes to the ch1 cadence register. */

    /* The first data is written to ch0.  The EC and command are built in to the
       VpMpiCmdWrapper() function call later */
    writeBuffer[1] = 0xAA;
    writeBuffer[2] = 0x55;
    writeBuffer[3] = 0x43;
    writeBuffer[4] = 0x21;
    /* Switch to the second channel */
    writeBuffer[5] = VP886_R_EC_WRT;
    writeBuffer[6] = VP886_R_EC_EC2;
    /* Fill up the middle of the buffer with writes to the ch1 cadence register.
       These should be overwritten by the final values. */
    i = 7;
    while (i < 240) {
        writeBuffer[i] = VP886_R_CADENCE_WRT;
        writeBuffer[i+1] = i;
        writeBuffer[i+2] = i;
        writeBuffer[i+3] = i;
        writeBuffer[i+4] = i;
        i += 5;
    }
    /* Final write to the ch1 register */
    writeBuffer[i++] = VP886_R_CADENCE_WRT;
    writeBuffer[i++] = 0x99;
    writeBuffer[i++] = 0xBB;
    writeBuffer[i++] = 0x56;
    writeBuffer[i] = 0x78;
    /* Send the whole buffer */
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_WRT, i, &writeBuffer[1]);

    /* Read back both register values */
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN, &readBuffer[1]);
    VpMpiCmdWrapper(deviceId, VP886_EC_2, VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN, &readBuffer[6]);

    /* Make sure that both the first and last writes of the large buffer read
       back correctly.
       Some bits of the register read back as 0, so AA554321 becomes 02550321
       and 99BB5678 becomes 01BB0678 */
    if (readBuffer[1] != 0x02 || readBuffer[2] != 0x55 || readBuffer[3] != 0x03 || readBuffer[4] != 0x21) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 3 failed. Ch0 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: expected 0x02 0x55 0x03 0x21"));
        status = VP_STATUS_ERR_SPI;
    }
    if (readBuffer[6] != 0x01 || readBuffer[7] != 0xBB || readBuffer[8] != 0x06 || readBuffer[9] != 0x78) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: Test 3 failed. Ch1 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest: expected 0x01 0xBB 0x06 0x78"));
        status = VP_STATUS_ERR_SPI;
    }

    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Add new tests here if needed */

    return status;
}


/** Vp886QuickMpiTest1Ch()
  Single-channel version of Vp886QuickMpiTest().
  Performs some basic tests of the MPI and HAL implementation to make sure that
  we can properly communicate with the device before starting.
*/
VpStatusType
Vp886QuickMpiTest1Ch(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 writeBuffer[255];
    uint8 readBuffer[16];
    uint8 i;

    /* Flush whatever might be in the write buffer */
    if (pDevObj->slacBufData.buffering == TRUE) {
        Vp886SlacBufFlush(pDevCtx);
    }

    /*
        Test 1:
        Write to single-byte global register
    */
    VpMemSet(writeBuffer, 0xD6, sizeof(writeBuffer));
    VpMemSet(readBuffer, 0x94, sizeof(readBuffer));

    /* Write 0xE5 to the clock slots register.  This should read back as 0x65
       later because the top bit always reads as 0.  We have to make sure to
       use a value with the 0x40 bit set, because that bit will always read
       back as 1 in ZSI mode. */
    writeBuffer[1] = 0xE5;
    VpMpiCmdWrapper(deviceId, VP886_EC_GLOBAL, VP886_R_CLKSLOTS_WRT, VP886_R_CLKSLOTS_LEN, &writeBuffer[1]);
    
    /* Make sure the write buffer was not corrupted */
    if (writeBuffer[0] != 0xD6 || writeBuffer[1] != 0xE5 || writeBuffer[2] != 0xD6) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 1 failed. Write buffer corrupted: 0x%02X 0x%02X 0x%02X",
            writeBuffer[0], writeBuffer[1], writeBuffer[2]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Read back the register */
    VpMpiCmdWrapper(deviceId, VP886_EC_GLOBAL, VP886_R_CLKSLOTS_RD, VP886_R_CLKSLOTS_LEN, &readBuffer[1]);
    
    /* Make sure there was no overflow into the data around byte 1 */
    if (readBuffer[0] != 0x94 || readBuffer[2] != 0x94) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 1 failed. Read buffer overflow: 0x%02X 0x%02X 0x%02X",
            readBuffer[0], readBuffer[1], readBuffer[2]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* The high bit of this register always returns 0, so what we read should
       not be exactly what we wrote.  0xE5 becomes 0x65 */
    if (readBuffer[1] != 0x65) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 1 failed. Read value incorrect: 0x%02X, expected 0x65",
            readBuffer[1]));
        status = VP_STATUS_ERR_SPI;
    }
    
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }
    
    /*
        Test 2:
        Write to multi-byte channel register
    */
    VpMemSet(writeBuffer, 0xD6, sizeof(writeBuffer));
    VpMemSet(readBuffer, 0x94, sizeof(readBuffer));
    
    /* Write to the cadence register */
    writeBuffer[1] = 0xAB;
    writeBuffer[2] = 0xCD;
    writeBuffer[3] = 0xEF;
    writeBuffer[4] = 0x01;
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_WRT, VP886_R_CADENCE_LEN, &writeBuffer[1]);
    
    /* Make sure the write buffer was not corrupted */
    if (writeBuffer[0] != 0xD6 || writeBuffer[1] != 0xAB || writeBuffer[2] != 0xCD ||
        writeBuffer[3] != 0xEF || writeBuffer[4] != 0x01 || writeBuffer[5] != 0xD6)
    {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 2 failed. Write buffer corrupted: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            writeBuffer[0], writeBuffer[1], writeBuffer[2], writeBuffer[3], writeBuffer[4], writeBuffer[5]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Read back the register */
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN, &readBuffer[1]);

    /* Make sure there was no overflow into the surrounding data */
    if (readBuffer[0] != 0x94 || readBuffer[5] != 0x94 || readBuffer[10] != 0x94) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 2 failed. Read buffer overflow: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4], readBuffer[5]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Make sure the data read back correctly.
       Some bits of the register will always read as 0, so we should read
       0x03CD0701 instead of 0xABCDEF01 */
    if (readBuffer[1] != 0x03 || readBuffer[2] != 0xCD || readBuffer[3] != 0x07 || readBuffer[4] != 0x01) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 2 failed. Read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X",
            readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: expected 0x03 0xCD 0x07 0x01"));
        status = VP_STATUS_ERR_SPI;
    }
    
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /*
        Test 3:
        Test large write length
    */
    VpMemSet(writeBuffer, 0xD6, sizeof(writeBuffer));
    VpMemSet(readBuffer, 0x94, sizeof(readBuffer));

    /* Build a large write buffer where the first command is a write to the clock
       slot register, followed by repeated writes to the cadence register. */
    writeBuffer[1] = 0xDA;
    /* Switch to the channel EC */
    writeBuffer[2] = VP886_R_EC_WRT;
    writeBuffer[3] = VP886_R_EC_EC1;
    /* Fill up the middle of the buffer with writes to the cadence register.
       These should be overwritten by the final values. */
    i = 4;
    while (i < 240) {
        writeBuffer[i] = VP886_R_CADENCE_WRT;
        writeBuffer[i+1] = i;
        writeBuffer[i+2] = i;
        writeBuffer[i+3] = i;
        writeBuffer[i+4] = i;
        i += 5;
    }
    /* Final write to the cadence register */
    writeBuffer[i++] = VP886_R_CADENCE_WRT;
    writeBuffer[i++] = 0x99;
    writeBuffer[i++] = 0xBB;
    writeBuffer[i++] = 0x56;
    writeBuffer[i] = 0x78;
    /* Send the whole buffer */
    VpMpiCmdWrapper(deviceId, VP886_EC_GLOBAL, VP886_R_CLKSLOTS_WRT, i, &writeBuffer[1]);

    /* Read back both register values */
    VpMpiCmdWrapper(deviceId, VP886_EC_GLOBAL, VP886_R_CLKSLOTS_RD, VP886_R_CLKSLOTS_LEN, &readBuffer[1]);
    VpMpiCmdWrapper(deviceId, VP886_EC_1, VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN, &readBuffer[2]);

    /* Make sure that both the first and last writes of the large buffer read
       back correctly.
       Some bits of the registers read back as 0, so DA becomes 5A
       and 99BB5678 becomes 01BB0678 */
    if (readBuffer[1] != 0x5A) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 3 failed. First read value incorrect: 0x%02X, expected 0x5A",
            readBuffer[1]));
        status = VP_STATUS_ERR_SPI;
    }
    if (readBuffer[2] != 0x01 || readBuffer[3] != 0xBB || readBuffer[4] != 0x06 || readBuffer[5] != 0x78) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: Test 3 failed. Second read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[2], readBuffer[3], readBuffer[4], readBuffer[5]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886QuickMpiTest1Ch: expected 0x01 0xBB 0x06 0x78"));
        status = VP_STATUS_ERR_SPI;
    }

    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Add new tests here if needed */

    return status;
}
#endif /* VP886_INCLUDE_MPI_QUICK_TEST */

#endif
