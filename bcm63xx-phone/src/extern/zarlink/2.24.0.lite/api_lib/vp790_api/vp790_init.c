/** \file vp790_init.c
 * vp790_init.c
 *
 * This file contains the line and device initialization functions for
 * the Vp790 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 10850 $
 * $LastChangedDate: 2013-03-05 10:34:01 -0600 (Tue, 05 Mar 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_790_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp790_api.h"
#include "vp790_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"

static void
Vp790InitDeviceObject(
    Vp790DeviceObjectType *pDevObj);

static void
Vp790InitLineObject(
    Vp790LineObjectType *pLineObj);

/* Declare prototypes for the functions that are used only in this file */
VpStatusType
Vp790Init(
    VpDevCtxType *pDevCtx);

static VpStatusType
Vp790InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile);

static VpStatusType
Vp790InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp790ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp790InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile);

/**
 * VpMakeVp790DeviceObject()
 *  This function performs the main tasks of VpMakeDeviceObject() for Vp790
 * type of devices.
 *
 * Preconditions:
 *  Same as VpMakeDeviceObject(), and in addition the deviceType pointed to by
 * pDevCtx should be Vp790 series type. Device is in code critical region prior
 * to entering this function (see vp_api.c").
 *
 * Postconditions:
 *  VpAPI Function pointers for pDevCtx are initialized to Vp790 specific
 * functions.  This completes the function abstraction for "this" device.
 */
VpStatusType
VpMakeVp790DeviceObject(
    VpDevCtxType *pDevCtx,          /**< Device context to be initialized
                                     * with function pointers
                                     */
    Vp790DeviceObjectType *pDevObj) /**< Device object containing information
                                     * for the device pointed to by pDevCtx
                                     */
{
    Vp790InitDeviceObject(pDevObj);

    /* Initialize other elements in the device object */
    return VpMakeVp790DeviceCtx(pDevCtx, pDevObj);
}

/**
 * Vp790InitDeviceObject()
 *  This function initializes the Vp790 Device object data structure. It is
 * called only in this file .
 */
static void
Vp790InitDeviceObject(
    Vp790DeviceObjectType *pDevObj)
{
    VpMemSet(pDevObj, 0, sizeof(Vp790DeviceObjectType));
    pDevObj->staticInfo.maxChannels = 4;
}

/**
 * VpMakeVp790DeviceCtx()
 *  This function initializes the device context to handle Vp790 functionality.
 *
 * Preconditions:
 *  This function should be called after initializing the device object. This
 * function can be called more than once without modifying the contents of the
 * device object.
 *
 * Postconditions:
 *  Initializes device context to be able to handle Vp790 functionality.
 */
VpStatusType
VpMakeVp790DeviceCtx(
    VpDevCtxType *pDevCtx,          /**< Device Context to be initialized */
    Vp790DeviceObjectType *pDevObj) /**< Device Object that has been already
                                     * initialized
                                     */
{
    uint8 channelCount, maxChan;

    if((pDevCtx == VP_NULL) || (pDevObj == VP_NULL)){
        return VP_STATUS_INVALID_ARG;
    }

    /* Initialize Device context */
    pDevCtx->pDevObj = pDevObj;
    pDevCtx->deviceType = VP_DEV_790_SERIES;

    /*
     * Initialize all of the line context pointers to null in the device context
     */
    maxChan = pDevObj->staticInfo.maxChannels;
    for (channelCount = 0; channelCount < maxChan; channelCount++) {
        pDevCtx->pLineCtx[channelCount] = VP_NULL;
    }

    /* Functions in apiInit.c */
    pDevCtx->funPtrsToApiFuncs.MakeLineObject = Vp790MakeLineObject;
    pDevCtx->funPtrsToApiFuncs.InitDevice = Vp790InitDevice;
    pDevCtx->funPtrsToApiFuncs.InitLine = Vp790InitLine;
    pDevCtx->funPtrsToApiFuncs.ConfigLine = Vp790ConfigLine;
#if !defined(VP_REDUCED_API_IF)
    pDevCtx->funPtrsToApiFuncs.CalCodec = Vp790CalCodec;
#endif
    pDevCtx->funPtrsToApiFuncs.InitProfile = Vp790InitProfile;

#ifdef VP_CSLAC_SEQ_EN
    pDevCtx->funPtrsToApiFuncs.InitRing = Vp790InitRing;
    pDevCtx->funPtrsToApiFuncs.InitCid = Vp790InitCid;
    pDevCtx->funPtrsToApiFuncs.InitMeter = VpCSLACInitMeter;
#endif

    pDevCtx->funPtrsToApiFuncs.DtmfDigitDetected = VpCSLACDtmfDigitDetected;
#if !defined(VP_REDUCED_API_IF)
    pDevCtx->funPtrsToApiFuncs.ClearResults = VpCSLACClearResults;
#endif

    /* Functions in apicnt.c */
    pDevCtx->funPtrsToApiFuncs.SetLineState = Vp790SetLineState;
    pDevCtx->funPtrsToApiFuncs.SetLineTone = Vp790SetLineTone;
    pDevCtx->funPtrsToApiFuncs.SetRelayState = Vp790SetRelayState;

#ifdef VP_CSLAC_SEQ_EN
    pDevCtx->funPtrsToApiFuncs.SendCid = Vp790SendCid;
    pDevCtx->funPtrsToApiFuncs.ContinueCid = Vp790ContinueCid;
    pDevCtx->funPtrsToApiFuncs.StartMeter = VpCSLACStartMeter;
#endif

    pDevCtx->funPtrsToApiFuncs.SetOption = Vp790SetOption;
#ifndef VP790_SIMPLE_POLLED_MODE
    pDevCtx->funPtrsToApiFuncs.VirtualISR = Vp790VirtualISR;
#endif
    pDevCtx->funPtrsToApiFuncs.ApiTick = Vp790ApiTick;
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    pDevCtx->funPtrsToApiFuncs.LowLevelCmd = Vp790LowLevelCmd;
#endif
    pDevCtx->funPtrsToApiFuncs.DeviceIoAccess = Vp790DeviceIoAccess;

    /* Functions in apiQuery.c */
    pDevCtx->funPtrsToApiFuncs.GetEvent = Vp790GetEvent;
    pDevCtx->funPtrsToApiFuncs.GetLineStatus = VpCSLACGetLineStatus;
    pDevCtx->funPtrsToApiFuncs.GetDeviceStatus = Vp790GetDeviceStatus;
    pDevCtx->funPtrsToApiFuncs.FlushEvents = Vp790FlushEvents;
    pDevCtx->funPtrsToApiFuncs.GetOption = Vp790GetOption;
    pDevCtx->funPtrsToApiFuncs.GetResults = Vp790GetResults;
    pDevCtx->funPtrsToApiFuncs.GetLoopCond = Vp790GetLoopCond;

    return VP_STATUS_SUCCESS;
}

/**
 * VpMake790LineObject()
 *  This function initializes a line context using the information that is
 * passed. This function is like a C++ constructor. It initializes the passed
 * line context and line object based on the paramters provided. The passed line
 * object type should match with the type of device object type. See VP-API
 * reference guide for more information.
 *
 * Preconditions:
 *  This function assumes device context has already been created and
 * initialized. This function should only be called after downloading the boot
 * image the device when applicable (like for VCP class of devices).
 *
 * Postconditions:
 *  This function initializes the line context/line object. Line related VP-API
 * functions can be called after calling this function.
 */
VpStatusType
Vp790MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pVoidLineObj,
    VpDevCtxType *pDevCtx)
{
    Vp790LineObjectType *pLineObj = pVoidLineObj;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    if (channelId >= pDevObj->staticInfo.maxChannels) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    Vp790InitLineObject(pLineObj);

    switch (termType) {
        case VP_TERM_FXS_GENERIC:
        case VP_TERM_FXS_75282:
        case VP_TERM_FXS_TO_TL:
        case VP_TERM_FXS_RR:
            pLineObj->termType = termType;
            break;

        default:
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_INVALID_ARG;
    }

    pLineCtx->pLineObj = pLineObj;
    pLineCtx->pDevCtx = pDevCtx;

    pDevCtx->pLineCtx[channelId] = pLineCtx;
    pLineObj->channelId = channelId;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Everything else done by device/line specific functions */
    return VP_STATUS_SUCCESS;
}

/**
 * Vp790InitLineObject()
 *  This function initializes the Vp790 Line Object data structure. It is
 * called only in this file .
 */
static void
Vp790InitLineObject(
    Vp790LineObjectType *pLineObj)
{
    VpMemSet(pLineObj, 0, sizeof(Vp790LineObjectType));
}

/**
 * Vp790Init
 *  This function initializes the device, and (contrary to InitDevice) does
 * not initialize any channels. This function should be called internal to the
 * API only.
 *
 * Preconditions:
 *  The device context must be of a Vp790 device type.
 *
 * Postconditions:
 *  This function returns a failure code if the clock configuration is not set
 * correctly based on the device data set in InitDevice.
 */
VpStatusType
Vp790Init(
    VpDevCtxType *pDevCtx)
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 data, gdStat;
    uint8 deviceFailure;
    uint8 testCount;
    uint8 mpiReset[] = {
        VP790_NO_OP, VP790_NO_OP, VP790_NO_OP, VP790_NO_OP, VP790_NO_OP,
        VP790_NO_OP, VP790_NO_OP, VP790_NO_OP, VP790_NO_OP, VP790_NO_OP,
        VP790_NO_OP, VP790_NO_OP, VP790_NO_OP, VP790_NO_OP, VP790_NO_OP,
        VP790_NO_OP
    };

    /* Initialize the API's device dynamic variables */
    pDevObj->dynamicInfo.lastChan = 0;

    /*********** Errata D2.1 and/or C3.9-10 (Part 1)************/

    /*
     * If the MPI Bus gets out of sequence for any reason, a HW reset command
     * will not work and this function may fail. To be sure a reset occurs, the
     * following sequence is required.
     */
    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_NO_OP, 16, mpiReset);
    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_HW_RESET_CMD, 0, VP_NULL);
    VpSysWait(20);

    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_DEVTYPE_CMD, VP790_DEVTYPE_LEN,
        pDevObj->staticInfo.rcnPcn);

    if ((pDevObj->staticInfo.rcnPcn[0] == 0x00)
     || (pDevObj->staticInfo.rcnPcn[0] == 0xFF)) {
        return VP_STATUS_FAILURE;
    }

    /*************** End Errata D2.1 (Part 1) ******************/
    /*
     * Setup mclk. The MCLK mask set the mclk frequency, sets the mclk source
     * (the MCLK pin or the PCLK pin), and sets the interrupt pin output drive
     * mode (TTL or open collector)
     */
    data = pDevObj->devProfileData.devCfg1;
    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_DCR1_WRT, VP790_DCR1_LEN, &data);

   /*
    * The MCLK correction register is used if the desired MCLK frequency is
    * NOT a multipal of 512kHz (for example 1.544MHz != n*512kHz, n an integer)
    */
    data = pDevObj->devProfileData.mClkCorrection;
    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_MCCR_WRT, VP790_MCCR_LEN, &data);

    /*
     * Wait for the CFAIL bit to clear before proceding. If the CFAIL bit
     * does not clear after several trys, give up and return an error
     * condition. Wait between each read of the status register.
     */

    /* Setup parameters to clear interrupts .. or at least try */
    deviceFailure = VP790_GD_STAT_CFAIL;
    testCount = MAX_CFAIL_TEST*10;
    while(deviceFailure && (--testCount) != 0) {
        VpSysWait(CFAIL_TEST_INTERVAL);
        VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_GD_STAT_RD, VP790_GD_STAT_LEN,
            &data);
        deviceFailure = data & VP790_GD_STAT_CFAIL;
        VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_INTRPT_RD, VP790_INTRPT_LEN,
            &data);
    }

    /*
     * The CFAIL bit did not clear so the part will not complete initialization.
     * Return error status to indicate failure.
     */
    if(deviceFailure) {
        return VP_STATUS_FAILURE;
    }

    /*
     * The PCM mask tells the device which clock edge to grab and xmit the
     * PCM data on and also which clock period LSB of the PCM data starts on
     */
    data = pDevObj->devProfileData.clockSlot;
    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_XR_CS_WRT, VP790_XR_CS_LEN, &data);

    /* Clear all Device Interrupt Masks except Pos Bat */
    data = VP790_GD_MASK_PINT;
    VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_GD_MASK_WRT, VP790_GD_MASK_LEN,
        &data);

    /* Clear all interrupts that resulted from power-up and reset */
    data = 0x01;
    while(data != 0x00) {
        VpSysWait(2);
        VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_INTRPT_RD, VP790_INTRPT_LEN,
            &data);
        /* Clear a device interrupt if one is pending */
        if(data & VP790_INTR_GLOB) {
            VpMpiCmdWrapper(deviceId, VP790_EC_CH1, VP790_GD_STAT_RD,
                VP790_GD_STAT_LEN, &gdStat);
        }
    }

    return VP_STATUS_SUCCESS;
} /* Vp790Init */

/**
 * Vp790InitDevice
 *  This function initializes the device and all lines associated with this
 * device (if line profiles are passed to this function).
 *
 * Preconditions:
 *  None (device context is not NULL and is of Vp790 type, which is handled in
 * higher level software)
 *
 * Postconditions:
 *  This device is initialized to the configuration specified in the device
 * profile and the FXS lines associated with this device are initialized by the
 * FXS specific AC, DC, and Ringing profiles passed. This function returns an
 * error code if the device profile trying to be used for initialization is
 * VP_PTABLE_NULL (either passed or by a non-initialized index).
 */
VpStatusType
Vp790InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,   /**< The profile pointer for the device
                                     * configuration parameters
                                     */
    VpProfilePtrType pAcProfile,    /**< The profile pointer (or index) for
                                     * the AC characteristic to apply to the
                                     * FXS lines
                                     */
    VpProfilePtrType pDcProfile,    /**< The profile pointer (or index) for
                                     * the DC characteristic to apply to the
                                     * FXS lines
                                     */
    VpProfilePtrType pRingProfile,  /**< The profile pointer (or index) for
                                     * the Ringing characteristic to apply to
                                     * the FXS lines
                                       */
    VpProfilePtrType pFxoAcProfile, /**< The profile pointer (or index) for
                                     * the AC characteristic to apply to the
                                     * FXO lines
                                       */
    VpProfilePtrType pFxoCfgProfile)/**< The profile pointer for the FXO
                                     * specific supervision paramaters.
                                     */
{
    VpLineCtxType *pLineCtx;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};

    uint8 gpioDir[] = {VP790_GIO_DIR_ALL_OUTPUT};

    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    VpProfilePtrType pDevProf;
    uint8 chan;

    VpStatusType status = VP_STATUS_SUCCESS;

    int profIndex = VpGetProfileIndex(pDevProfile);

    /*
     * Get Profile Index returns -1 if the profile passed is a pointer or
     * of VP_PTABLE_NULL type. Otherwise it returns the index
     */
    if (profIndex < 0) {
        /*
         * A pointer is passed or VP_PTABLE_NULL.  If it's a pointer, make
         * sure the content is valid for the profile type.
         */
        if (pDevProfile != VP_PTABLE_NULL) {
            if(VpVerifyProfileType(VP_PROFILE_DEVICE, pDevProfile) != TRUE) {
                return VP_STATUS_ERR_PROFILE;
            }
        }
        pDevProf = pDevProfile;

    } else if (profIndex < VP_CSLAC_DEV_PROF_TABLE_SIZE) {
        pDevProf = pDevObj->devProfileTable.pDevProfileTable[profIndex];

        if ((pDevProf == VP_PTABLE_NULL)
         || (!(pDevObj->profEntry.devProfEntry & (0x01 << profIndex)))) {
            return VP_STATUS_ERR_PROFILE;
        }
    } else {
        return VP_STATUS_ERR_PROFILE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pDevObj->status.globStatReg = 0;
    pDevObj->status.calibrating = 0;
    pDevObj->timeStamp = 0;
    pDevObj->status.state = VP_DEV_INIT_IN_PROGRESS;

    if (pDevProf != VP_PTABLE_NULL) {
        pDevObj->devProfileData.pcmClkRate =
            (uint16)(((pDevProf[VP790_DEV_PROFILE_PCLK_MSB] << 8) & 0xFF00)
                    | (pDevProf[VP790_DEV_PROFILE_PCLK_LSB] & 0x00FF));

        pDevObj->devProfileData.devCfg1 =
            (uint8)(pDevProf[VP790_DEV_PROFILE_DEVCFG1]);
        pDevObj->devProfileData.mClkCorrection =
            (uint8)(pDevProf[VP790_DEV_PROFILE_MCLK_CORR]);
        pDevObj->devProfileData.clockSlot =
            (uint8)(pDevProf[VP790_DEV_PROFILE_CLOCK_SLOT]);
        pDevObj->devProfileData.maxNumInterrupts =
            (uint8)(pDevProf[VP790_DEV_PROFILE_MAX_EVENTS]);

        pDevObj->devProfileData.tickRate =
            (uint16)(((pDevProf[VP790_DEV_PROFILE_TICKRATE_MSB] << 8) & 0xFF00)
                    | (pDevProf[VP790_DEV_PROFILE_TICKRATE_LSB] & 0x00FF));

        pDevObj->devProfileData.calibrationTime =
            MS_TO_TICKRATE(20, pDevObj->devProfileData.tickRate);
    } else {
        pDevObj->status.state &= ~VP_DEV_INIT_IN_PROGRESS;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_ERR_PROFILE;
    }

    /* Initialize device */
    /*
     * If not successful, the Clock Fail bit did not clear so return error code
     */
    if ((status = Vp790Init(pDevCtx)) != VP_STATUS_SUCCESS) {
        pDevObj->status.state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);
        /*
         * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
         * completed. This prevents VpApiTick() from generating events on a non-initialized device
         * context/object.
         */
        pDevObj->devProfileData.tickRate = 0;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return status;
    }

    VpMpiCmdWrapper(deviceId, ecVal[0], VP790_GIO_DIR_WRT, VP790_GIO_DIR_LEN, gpioDir);

    if (pDevProf[VP_PROFILE_VERSION] >=
        VP_CSLAC_DEV_PROFILE_VERSION_LINECARD_PARAM) {
        /* This profile contains the Linecard parameters value */
        VpMpiCmdWrapper(deviceId, ecVal[0], VP790_LINECARD_PARAM_WRT,
            VP790_LINECARD_PARAM_LEN,
            (VpProfileDataType *)(&pDevProf[VP790_DEV_PROFILE_LINECARD_PARAM_DATA0]));
    }

    /* Initialize each channel */
    for (chan = 0; chan < maxChan; chan++) {
        /*
         * For Init Line to work, the device cannot be non-initialized because
         * the init line function tries to set the line state.  Therefore,
         * temporarily set the device init flag to TRUE then immediately after
         * line init, set back to FALSE until device init is complete
         */
        pLineCtx = pDevCtx->pLineCtx[chan];
        if (pLineCtx != VP_NULL) {
            status = Vp790InitLine(pLineCtx, pAcProfile, pDcProfile,
                pRingProfile);

            if (status != VP_STATUS_SUCCESS) {
                pDevObj->status.state &=
                    ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);
                /*
                 * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
                 * completed. This prevents VpApiTick() from generating events on a non-initialized
                 * device context/object.
                 */
                pDevObj->devProfileData.tickRate = 0;
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return status;
            }
        }
    }

    VpImplementDefaultSettings(pDevCtx, VP_NULL);

    /*
     * This clears the Init Line Events and any other erroneous event that
     * may have been created due to initialization
     */
    Vp790FlushEvents(pDevCtx);

    /* Start the device Calibration */
    Vp790CalCodecInt(pDevCtx);

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
} /* Vp790InitDevice */

/**
 * Vp790InitLine
 *  This function initializes a line of a device with the specified parameters
 * and API default values. It is a "Line Reset".
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  The line pointed to be the line context passed is initialized with the
 * profile data specified.  This function returns the success code if the device
 * associated with this line is initialized.
 */
VpStatusType
Vp790InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,    /**< Pointer to AC coefficient data or
                                     * profile index to be applied to this line.
                                     */

    VpProfilePtrType pDcOrFxoProfile,   /**< Pointer to DC Feed (FXS) or Cfg
                                         * (FX0) profile or profile index to be
                                         * applied to this line.
                                         */

    VpProfilePtrType pRingProfile)  /**< Pointer to Ringing profile or profile
                                     * index to apply to this line
                                     */
{
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint8 channelId = pLineObj->channelId;
    uint8 data;

#ifdef VP_CSLAC_SEQ_EN
    uint8 seqByte;
#endif

    uint8 ioReg[VP790_IO_REG_LEN];
    uint8 disableVals[VP790_LOOP_SUP_LEN] =
        {0x0D, 0x64, 0x0D, 0x01, 0x30, 0x00, 0x00, 0x7F};

    VpDeviceIdType deviceId = pDevObj->deviceId;

    /* Proceed if initialized or in progress, and not calibrating */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
        if (pDevObj->status.state & VP_DEV_IN_CAL) {
            return VP_STATUS_DEVICE_BUSY;
        }
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

#ifdef VP_CSLAC_SEQ_EN
    for (seqByte = 0; seqByte < VP790_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    pLineObj->callerId.status = VP_CID_RESET_VALUE;

    pLineObj->thisFskCid = FALSE;
#endif

    pLineObj->pRingingCadence = VP_PTABLE_NULL;
    pLineObj->pCidProfileType1 = VP_PTABLE_NULL;
    pLineObj->pCidProfileType2 = VP_PTABLE_NULL;

    pLineObj->lineState.currentState = VP_LINE_DISCONNECT;
    pLineObj->lineState.previous = VP_LINE_DISCONNECT;

    pLineObj->codec = VP_NUM_OPTION_CODEC_TYPE_IDS;

    pLineObj->lineTimers.type = VP_CSLAC_FXS_TIMER;
    InitTimerVars(pLineCtx);

#ifdef VP_CSLAC_SEQ_EN
    VpMemSet(&pLineObj->cadence, 0, sizeof(VpSeqDataType));
#endif

    /* Software reset the channel */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_SW_RESET_CMD, NO_DATA, &data);
    VpSysWait(3);

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_LOOP_SUP_WRT,
        VP790_LOOP_SUP_LEN, disableVals);

    VpSysWait(1);

    /* Set the default line state */
    Vp790SetLineState(pLineCtx, VP_LINE_DISCONNECT);

    /* Deactiveate the CODEC for the channel */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_DEACTIVATE_CMD, NO_DATA, &data);

    /***************** C3 Operational Issue 3 ******************/
    /* This errata fix does not harm D3 operation */
    data = 0x00;
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_CCR9_WRT, VP790_CCR9_LEN, &data);

    /* We've already deactivated the device */
    VpSysWait(3);
    /**************** End Operational Issue 3 ******************/

    pLineObj->ringCtrl.zeroCross = VP_OPTION_DEFAULT_ZERO_CROSS;
    Vp790ConfigLine(pLineCtx, pAcProfile, pDcOrFxoProfile, pRingProfile);

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_IO_REG_RD,
        VP790_IO_REG_LEN, ioReg);
    ioReg[0] |= VP790_IO_REG_RD1_MANUAL;
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_IO_REG_WRT,
        VP790_IO_REG_LEN, ioReg);

    VpImplementDefaultSettings(VP_NULL, pLineCtx);

    /* Force an initial test of the hook state */
    pLineObj->lineState.condition = VP_CSLAC_STATUS_INVALID;
    pLineObj->dpStruct.hookSt = FALSE;

    VpInitDP(&pLineObj->dpStruct);

    /* Re-activate the channel due to a silicon bug -- and it doesn't hurt */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_ACTIVATE_CMD, NO_DATA,
        VP_NULL);

    /* Post the line init complete event */
    pLineObj->lineEvents.response |= VP_LINE_EVID_LINE_INIT_CMP;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp790InitLine */

/**
 * Vp790ConfigLine
 *  This function reloads a line of a device with the specified parameters.
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  The line pointed to by the line context passed is initialized with the
 * profile data specified.  This function returns the success code if the device
 * associated with this line is initialized.
 */
VpStatusType
Vp790ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,    /**< Pointer to AC coefficient data or
                                     * profile index to be applied to this line.
                                     */

    VpProfilePtrType pDcOrFxoProfile,   /**< Pointer to DC Feed (FXS) or Cfg
                                         * (FX0) profile or profile index to be
                                         * applied to this line.
                                         */

    VpProfilePtrType pRingProfile)  /**< Pointer to Ringing profile or profile
                                     * index to apply to this line
                                     */
{
    Vp790LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpProfileDataType *pCurrentProf;
    uint8 profIndex = VP_PROFILE_MPI_LEN;
    uint8 ioReg[VP790_IO_REG_LEN];
    uint8 ccr4[VP790_CCR4_LEN], ccr4Temp[VP790_CCR4_LEN];
    uint8 defaultVals[VP790_LOOP_SUP_LEN] =
        {0x0D, 0x64, 0x0D, 0x01, 0x30, 0x00, 0x10, 0x0D};

    uint8 ecVal[] = {VP790_EC_CH1, VP790_EC_CH2, VP790_EC_CH3, VP790_EC_CH4};
    uint8 channelId = pLineObj->channelId;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VpProfilePtrType pAcProf = VP_PTABLE_NULL;
    VpProfilePtrType pDcFxoCfgProf = VP_PTABLE_NULL;
    VpProfilePtrType pRingProf = VP_PTABLE_NULL;

    /* Proceed if initialized or in progress, and not calibrating */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
        if (pDevObj->status.state & VP_DEV_IN_CAL) {
            return VP_STATUS_DEVICE_BUSY;
        }
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check the legality of the AC profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_AC,
            VP_CSLAC_AC_PROF_TABLE_SIZE, pDevObj->profEntry.acProfEntry,
            pDevObj->devProfileTable.pAcProfileTable, pAcProfile, &pAcProf)) {

        return VP_STATUS_ERR_PROFILE;
    }

    /* Check the legality of the DC profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_DC, VP_CSLAC_DC_PROF_TABLE_SIZE,
            pDevObj->profEntry.dcProfEntry,
            pDevObj->devProfileTable.pDcProfileTable,
            pDcOrFxoProfile, &pDcFxoCfgProf)) {

        return VP_STATUS_ERR_PROFILE;
    }

    /* Check the legality of the Ringing profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_RING, VP_CSLAC_RINGING_PROF_TABLE_SIZE,
            pDevObj->profEntry.ringingProfEntry,
            pDevObj->devProfileTable.pRingingProfileTable, pRingProfile,
            &pRingProf)) {

        return VP_STATUS_ERR_PROFILE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* If no AC Profile exists use whatever exists in the device */
    if (pAcProfile != VP_PTABLE_NULL) {
        pCurrentProf = (VpProfileDataType *)pAcProfile;
        VpMpiCmdWrapper(deviceId, ecVal[channelId], NOOP_CMD, pCurrentProf[profIndex],
            &pCurrentProf[profIndex + 1]);
    }

    /*
     * The Ring Profile may contain content (command 0x68) that affects
     * bits controlled only by the API-II. So read the register first, then
     * determine what bits are allowed to change, and set the register to
     * the new value.
     */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_CCR4_RD, VP790_CCR4_LEN,
        ccr4Temp);

    /* If no Ringing Profile exists use whatever exists in the device */
    if (pRingProfile != VP_PTABLE_NULL) {
        pCurrentProf = (VpProfileDataType *)pRingProfile;
        VpMpiCmdWrapper(deviceId, ecVal[channelId], NOOP_CMD, pCurrentProf[profIndex],
            &pCurrentProf[profIndex + 1]);
    }

    /*
     * Read what ccr4 is now set to. We'll need to adjust because some bits
     * may not be allowed to be changed by the Ringing Profile
     */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_CCR4_RD, VP790_CCR4_LEN,
        ccr4);

    /* Zero out those bits not controlled by the profile */
    ccr4[0] &= (VP790_EXT_RINGING | VP790_CCR4_RTM);

    /*
     * Or in the bits controlled by API-II as they existed before this profile
     * was sent.
     */
    ccr4Temp[0] &= (uint8)(~(VP790_EXT_RINGING | VP790_CCR4_RTM));
    ccr4[0] |= ccr4Temp[0];

    /*
     * If we are set to external ringing, RD2 has to be set for auto mode.
     * Otherwise it's set for manual
     */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_IO_REG_RD, VP790_IO_REG_LEN,
        ioReg);

    /* Set remaining values of ccr4 based on API-II settings */
    if (ccr4[0] & VP790_EXT_RINGING) {
        ioReg[0] |= VP790_IO_REG_RD2_AUTO;

        if(pLineObj->ringCtrl.zeroCross == VP_OPTION_ZC_NONE) {
            ccr4[0] |= VP790_CCR4_ZXR_DIS;
        } else {
            ccr4[0] &= ~VP790_CCR4_ZXR_DIS;
        }
    } else {
        ioReg[0] &= ~VP790_IO_REG_RD2_AUTO;

        if(pLineObj->ringCtrl.zeroCross == VP_OPTION_ZC_NONE) {
            ccr4[0] &= ~VP790_CCR4_ZXR_DIS;
        } else {
            ccr4[0] |= VP790_CCR4_ZXR_DIS;
        }
    }
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_CCR4_WRT, VP790_CCR4_LEN,
        ccr4);

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_IO_REG_WRT,
        VP790_IO_REG_LEN, ioReg);

    /* If no DC Feed Profile exists use whatever the device default value is */
    if (pDcFxoCfgProf != VP_PTABLE_NULL) {
        pCurrentProf = (VpProfileDataType *)pDcFxoCfgProf;
        VpMpiCmdWrapper(deviceId, ecVal[channelId], NOOP_CMD, pCurrentProf[profIndex],
            &pCurrentProf[profIndex + 1]);
    } else {
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP790_LOOP_SUP_WRT,
            VP790_LOOP_SUP_LEN, defaultVals);
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp790InitProfile
 *  This function is used to initialize profile tables in Vp790.
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  The line pointed to by the line context passed is initialized with the
 * profile data specified.  This function returns the success code if the device
 * associated with this line is initialized and the index being specified is
 * within the range of the device profile table.
 */
VpStatusType
Vp790InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile)
{
    Vp790DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

    /*
     * If the profile passed is an index, indicated by Get Profile Index return
     * value of > -1, return an error (cannot init an indexed entry with an
     * index).
     */
    int profileIndex = VpGetProfileIndex(pProfile);

    if (profileIndex >= 0) {
        return VP_STATUS_INVALID_ARG;
    }

    /*
     * If pProfileIndex is -1, the profile is of pointer type and invalid,
     * otherwise it is an index.  If it's an index, make sure the range is
     * valid.
     */
    profileIndex = VpGetProfileIndex(pProfileIndex);
    if (profileIndex < 0) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    /*
     * The correct types are passed, but check to make sure the specific profile
     * type being initialized is valid as well as the index value
     */
    switch(type) {
        case VP_PROFILE_DEVICE:
            if (profileIndex > VP_CSLAC_DEV_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pDevProfileTable[profileIndex] =
                        pProfile;
            }
            break;

        case VP_PROFILE_AC:
            if (profileIndex > VP_CSLAC_AC_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pAcProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_DC:
            if (profileIndex > VP_CSLAC_DC_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pDcProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_RING:
            if (profileIndex > VP_CSLAC_RINGING_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pRingingProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_RINGCAD:
            if (profileIndex > VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pRingingCadProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_TONE:
            if (profileIndex > VP_CSLAC_TONE_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pToneProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_TONECAD:
            if (profileIndex > VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pToneCadProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_METER:
            if (profileIndex > VP_CSLAC_METERING_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pMeteringProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_CID:
            if (profileIndex > VP_CSLAC_CALLERID_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pCallerIdProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        case VP_PROFILE_FXO_CONFIG:
            if (profileIndex > VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                pDevObj->devProfileTable.pFxoConfigProfileTable[profileIndex] =
                    pProfile;
            }
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;

} /* Vp790InitProfile() */

#endif





