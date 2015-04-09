/** \file vp880_init.c
 * vp880_init.c
 *
 * This file contains the line and device initialization functions for
 * the Vp880 device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11423 $
 * $LastChangedDate: 2014-05-20 16:46:10 -0500 (Tue, 20 May 2014) $
 */

#include "vp_api_cfg.h"

#ifdef VP_CC_880_SERIES

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"

/**< Vp880 Initalization Function Prototypes */
static VpStatusType Vp880Init(VpDevCtxType *pDevCtx);

static VpStatusType
Vp880InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile);

static VpStatusType Vp880DeviceReset(Vp880DeviceObjectType *pDevObj);

static VpStatusType
Vp880InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp880InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile);

#ifdef VP880_INCLUDE_MPI_QUICK_TEST
static VpStatusType
Vp880QuickMpiTest(
    VpDevCtxType *pDevCtx);
#endif /* VP880_INCLUDE_MPI_QUICK_TEST */

#ifdef VP880_FXS_SUPPORT
static void Vp880CopyDefaultFRProfile(Vp880DeviceObjectType *pDevObj);
#endif  /* VP880_FXS_SUPPORT */

/**
 * VpMakeVp880DeviceObject()
 *  This function performs the main tasks of VpMakeDeviceObject() for Vp880 type
 * of devices.
 *
 * Preconditions:
 *  Same as VpMakeDeviceObject(), and in addition the deviceType pointed to by
 * pDevCtx should be Vp880 series type.
 *
 * Postconditions:
 *  VpAPI Function pointers for pDevCtx are initialized to Vp880 specific
 * functions.  This completes the function abstraction for "this" device.
 */
VpStatusType
VpMakeVp880DeviceObject(
    VpDevCtxType *pDevCtx,  /**< Device context to be initialized with function
                             * pointers
                             */
    Vp880DeviceObjectType *pDevObj) /**< Device object containing information
                                     * for the device pointed to by pDevCtx
                                     */
{
    VpMemSet(pDevObj, 0, sizeof(Vp880DeviceObjectType));
    pDevObj->staticInfo.maxChannels = VP880_MAX_NUM_CHANNELS;

#ifdef VP_DEBUG
    pDevObj->debugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;
#endif

    /* Initialize other elements in the device object */
    return VpMakeVp880DeviceCtx(pDevCtx, pDevObj);
}   /* VpMakeVp880DeviceObject() */

/**
 * VpMakeVp880DeviceCtx()
 *  This function initializes the device context to handle Vp880 functionality.
 *
 * Preconditions:
 *  This function should be called after initializing the device object. This
 * function can be called more than once since it does not modify the contents
 * of the device object.
 *
 * Postconditions:
 *  Initializes device context to be able to handle Vp780 functionality.
 */
VpStatusType
VpMakeVp880DeviceCtx(
    VpDevCtxType *pDevCtx,          /**< Device Context to be initialized */
    Vp880DeviceObjectType *pDevObj) /**< Device Object that has been already
                                     * initialized
                                     */
{
    uint8 channelCount, maxChan;

    /* All error checking of arguments performed in calling functions */

    /* Initialize Device context */
    pDevCtx->pDevObj = pDevObj;
    pDevCtx->deviceType = VP_DEV_880_SERIES;

    /*
     * Initialize all of the line context pointers to null in the device context
     */
    maxChan = pDevObj->staticInfo.maxChannels;
    for (channelCount = 0; channelCount < maxChan; channelCount++) {
        pDevCtx->pLineCtx[channelCount] = VP_NULL;
    }

    /* Functions in apiInit.c */
    pDevCtx->funPtrsToApiFuncs.MakeLineObject = Vp880MakeLineObject;
    pDevCtx->funPtrsToApiFuncs.InitDevice = Vp880InitDevice;
    pDevCtx->funPtrsToApiFuncs.InitLine = Vp880InitLine;
    pDevCtx->funPtrsToApiFuncs.ConfigLine = Vp880ConfigLine;
    pDevCtx->funPtrsToApiFuncs.InitProfile = Vp880InitProfile;

#ifdef VP880_FXS_SUPPORT
    pDevCtx->funPtrsToApiFuncs.FreeRun = Vp880FreeRun;
#ifdef VP_CSLAC_SEQ_EN
    pDevCtx->funPtrsToApiFuncs.InitRing = Vp880InitRing;
    pDevCtx->funPtrsToApiFuncs.InitCid = Vp880InitCid;
    pDevCtx->funPtrsToApiFuncs.InitMeter = VpCSLACInitMeter;
    pDevCtx->funPtrsToApiFuncs.DtmfDigitDetected = VpCSLACDtmfDigitDetected;
#endif  /* VP_CSLAC_SEQ_EN */
#endif  /* VP880_FXS_SUPPORT */

#ifndef VP_REDUCED_API_IF
   pDevCtx->funPtrsToApiFuncs.ClearResults = VpCSLACClearResults;
#endif  /* VP_REDUCED_API_IF */

    /* Functions in apicnt.c */
    pDevCtx->funPtrsToApiFuncs.SetLineState = Vp880SetLineState;
    pDevCtx->funPtrsToApiFuncs.SetLineTone = Vp880SetLineTone;
#ifdef CSLAC_GAIN_RELATIVE
    pDevCtx->funPtrsToApiFuncs.SetRelGain = Vp880SetRelGain;
#endif  /* CSLAC_GAIN_RELATIVE */

#ifdef VP880_FXS_SUPPORT
    pDevCtx->funPtrsToApiFuncs.SetRelayState = Vp880SetRelayState;
#endif  /* VP880_FXS_SUPPORT */

#ifdef VP_CSLAC_SEQ_EN
    pDevCtx->funPtrsToApiFuncs.SendSignal = Vp880SendSignal;
#ifdef VP880_FXS_SUPPORT
    pDevCtx->funPtrsToApiFuncs.SendCid = Vp880SendCid;
    pDevCtx->funPtrsToApiFuncs.ContinueCid = Vp880ContinueCid;
    pDevCtx->funPtrsToApiFuncs.StartMeter = VpCSLACStartMeter;
#endif  /* VP880_FXS_SUPPORT */
#endif  /* VP_CSLAC_SEQ_EN */

    pDevCtx->funPtrsToApiFuncs.SetOption = Vp880SetOption;
#ifndef VP880_SIMPLE_POLLED_MODE
    pDevCtx->funPtrsToApiFuncs.VirtualISR = Vp880VirtualISR;
#endif  /* VP880_SIMPLE_POLLED_MODE */

    pDevCtx->funPtrsToApiFuncs.ApiTick = Vp880ApiTick;

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    pDevCtx->funPtrsToApiFuncs.LowLevelCmd = Vp880LowLevelCmd;
#endif  /* !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL) */

    pDevCtx->funPtrsToApiFuncs.DeviceIoAccess = Vp880DeviceIoAccess;

    /* Functions in apiQuery.c */
    pDevCtx->funPtrsToApiFuncs.GetEvent = Vp880GetEvent;
    pDevCtx->funPtrsToApiFuncs.GetLineStatus = VpCSLACGetLineStatus;
    pDevCtx->funPtrsToApiFuncs.GetDeviceStatus = Vp880GetDeviceStatus;
    pDevCtx->funPtrsToApiFuncs.FlushEvents = Vp880FlushEvents;
    pDevCtx->funPtrsToApiFuncs.GetResults = Vp880GetResults;
    pDevCtx->funPtrsToApiFuncs.GetOption = Vp880GetOption;

#ifdef VP880_INCLUDE_TESTLINE_CODE
    /* Technically a "Query" function, but used only for Line Test purposes */
    pDevCtx->funPtrsToApiFuncs.GetRelayState = Vp880GetRelayState;
#endif  /* VP880_INCLUDE_TESTLINE_CODE */

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
    pDevCtx->funPtrsToApiFuncs.RegisterDump = Vp880RegisterDump;
    pDevCtx->funPtrsToApiFuncs.ObjectDump = Vp880ObjectDump;
#endif  /* (VP_CC_DEBUG_SELECT & VP_DBG_INFO) */

    /* Functions in apiTestLine.c */
#ifdef VP880_INCLUDE_TESTLINE_CODE
    pDevCtx->funPtrsToApiFuncs.TestLineInt = Vp880TestLineInt;
    pDevCtx->funPtrsToApiFuncs.TestLineCallback = Vp880TestLineCallback;
    pDevCtx->funPtrsToApiFuncs.TestLine = Vp880TestLine;
#endif  /* VP880_INCLUDE_TESTLINE_CODE */

    /* Functions in apiCal.c */
#if defined (VP880_FXS_SUPPORT) && defined (VP_CSLAC_RUNTIME_CAL_ENABLED)
#if !defined(VP_REDUCED_API_IF) || defined (VP_ENABLE_PROD_TEST)
    /*
     * The VpCalCodec() function is largely removed from the VP-API-II interface
     * because it is not required. Once a system is calibrated, it does not need
     * to be recalibrated AND actions performed in VpCalCodec() occur during
     * VpInitDevice() (hence, no need for the application to directly call
     * VpCalCodec()). However, for Production test purposes it is
     * required. This is because backdoor methods are used to change the target
     * voltage without intending to fully reinitialize the device. This can only
     * be achieved by forcing battery calibration to repeat, which is done in
     * the VpCalCodec() function (i.e., calling VpInitDevice() is not desired).
     */
    pDevCtx->funPtrsToApiFuncs.CalCodec = Vp880CalCodec;
#endif  /* !defined(VP_REDUCED_API_IF) || defined (VP_ENABLE_PROD_TEST) */
    pDevCtx->funPtrsToApiFuncs.CalLine = Vp880CalLine;
#endif  /* defined (VP880_FXS_SUPPORT) && defined (VP_CSLAC_RUNTIME_CAL_ENABLED) */
    pDevCtx->funPtrsToApiFuncs.Cal = Vp880Cal;

    return VP_STATUS_SUCCESS;
}

/**
 * VpMakeVp880LineObject()
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
Vp880MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pVoidLineObj,
    VpDevCtxType *pDevCtx)
{
    Vp880LineObjectType *pLineObj = pVoidLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    if (channelId >= pDevObj->staticInfo.maxChannels) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VpMemSet(pLineObj, 0, sizeof(Vp880LineObjectType));
#ifdef VP_DEBUG
    pLineObj->debugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;
#endif

    switch (termType) {
#ifdef VP880_FXO_SUPPORT
        case VP_TERM_FXO_GENERIC:
        case VP_TERM_FXO_DISC: {
            Vp880DeviceStateIntType chanMap[] = {VP880_LINE0_IS_FXO, VP880_LINE1_IS_FXO};

            /*
             * At this point, it is only a recommendation. We'll adjust this when we determine the
             * device type found in VpInitDevice()
             */
            if (pDevObj->state & VP_DEV_INIT_CMP) {
                if (chanMap[channelId] & pDevObj->stateInt) {
                    pLineObj->status |= VP880_IS_FXO;
                } else {
                    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                    return VP_STATUS_ERR_VTD_CODE;
                }
            } else {
                pLineObj->status |= VP880_IS_FXO;
            }

            pDevObj->stateInt |= ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);

            /*
             * The "calDone" flag for the line object has to be set for VP_CAL_GET_SYSTEM_COEFF to
             * return valid data. Otherwise, it would think the system is not 100% calibrated and
             * return an error code. The FXO lines do not require calibration so this is the only
             * opportuity to set this flag = TRUE to indicate this line has been calibrated.
             */
            pLineObj->calLineData.calDone = TRUE;
            }
            break;
#endif  /* VP880_FXO_SUPPORT */

#ifdef VP880_FXS_SUPPORT
        case VP_TERM_FXS_GENERIC:
        case VP_TERM_FXS_ISOLATE:
        case VP_TERM_FXS_SPLITTER:
            pLineObj->status = VP880_INIT_STATUS;
            pDevObj->stateInt &= ((channelId == 0) ? ~VP880_LINE0_LP : ~VP880_LINE1_LP);
            break;

#ifdef VP880_LP_SUPPORT
        case VP_TERM_FXS_LOW_PWR:
        case VP_TERM_FXS_ISOLATE_LP:
        case VP_TERM_FXS_SPLITTER_LP:
            pLineObj->status = VP880_INIT_STATUS;
            pDevObj->stateInt |= ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);
            break;
#endif  /* VP880_LP_SUPPORT */
#endif  /* VP880_FXS_SUPPORT */

        default:
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_VTD_CODE;
    }

    pLineCtx->pLineObj = pLineObj;
    pLineCtx->pDevCtx = pDevCtx;

    pDevCtx->pLineCtx[channelId] = pLineCtx;
    pLineObj->channelId = channelId;
    pLineObj->termType = termType;
    pLineObj->ecVal = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Everything else done by device/line specific functions */
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880Init
 *  This function initializes the device, and (contrary to InitDevice) does not initialize any
 *  channels. This function should be called internal to the API only.
 *
 * Preconditions:
 *  The device context must be of a Vp880 device type.
 *
 * Postconditions:
 *  This function returns a failure code if the clock configuration is not set correctly based on
 *  the device data set in InitDevice.
 */
VpStatusType
Vp880Init(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 intMaskData[] = {0x7F, 0xFF};
    uint8 clkNotStable;
    uint8 clkTestCount;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880Init+"));

    /*
     * If the MPI Bus gets out of sequence for any reason, a HW reset command
     * will not work and this function may fail. To be sure a reset occurs, the
     * following sequence is required.
     */

    /* First, make sure the MPI buffer is cleared so we can write to the
     * device correctly prior to HW reset. */
    VpCSLACClearMPIBuffer(deviceId);

    if(Vp880DeviceReset(pDevObj) != VP_STATUS_SUCCESS) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Properly"));

        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880Init-"));
        return VP_STATUS_FAILURE;
    }

#ifdef VP880_INCLUDE_MPI_QUICK_TEST
    /* Run a quick test of the MPI interface and HAL layer code */
    status = Vp880QuickMpiTest(pDevCtx);
    if (status != VP_STATUS_SUCCESS) {
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880Init-"));
        return status;
    }
#endif /* VP880_INCLUDE_MPI_QUICK_TEST */

    /*
     * Setup mclk. The MCLK mask set the mclk frequency, sets the mclk source
     * (the MCLK pin or the PCLK pin), and sets the interrupt pin output drive
     * mode (TTL or open collector)
     */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_MCLK_CNT_WRT, VP880_MCLK_CNT_LEN,
        &pDevObj->devProfileData.devCfg1);

    /* Setup the Clock Fail Interrupt */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_INT_MASK_WRT, VP880_INT_MASK_LEN,
        intMaskData);

    /*
     * Wait for the CFAIL bit to clear before proceding. If the CFAIL bit does
     * not clear after several trys, give up and return an error condition. Wait
     * between each read of the status register.
     */
    clkNotStable = VP880_CFAIL_MASK;
    clkTestCount = MAX_CFAIL_TEST;
    while(clkNotStable && (--clkTestCount) != 0) {
        VpSysWait(CFAIL_TEST_INTERVAL*10);
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_UL_SIGREG_RD,
            VP880_UL_SIGREG_LEN, pDevObj->intReg);
        clkNotStable = pDevObj->intReg[0] & VP880_CFAIL_MASK;
    }

    /*
     * The CFAIL bit did not clear so the part will not complete initialization.
     * Return error status to indicate failure.
     */
    if(clkNotStable) {
        pDevObj->deviceEvents.faults |= VP_DEV_EVID_CLK_FLT;

        VP_ERROR(VpDevCtxType, pDevCtx, ("Device Failed to Reset Clock Fault"));

        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880Init-"));
        return VP_STATUS_FAILURE;
    }

    /* Setup interrupts back to default */
    intMaskData[0] = 0xFF;  /* Clear all Device Interrupt Masks */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_INT_MASK_WRT, VP880_INT_MASK_LEN, intMaskData);

    /*
     * The PCM mask tells the device which clock edge to grab and xmit the
     * PCM data on and also which clock period LSB of the PCM data starts on
     */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_XR_CS_WRT, VP880_XR_CS_LEN,
        &pDevObj->devProfileData.clockSlot);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880Init-"));
    return status;
} /* Vp880Init */

/**
 * Vp880DeviceReset
 *  This function resets the MPI buffer and HW reset of the device. The method
 * for doing this depends on the silicon revision due to I/O1 driver
 * requirements.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  The Device is reset, and I/O1 is in a safe condition.
 */
VpStatusType
Vp880DeviceReset(
    Vp880DeviceObjectType *pDevObj)
{
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 mpiCmdData, deviceRcn;
    VpStatusType status;

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880DeviceReset+"));

    /* Verify a valid RCN/PCN. */
    if ((status = Vp880InitDevicePcnRcn(pDevObj, deviceId)) != VP_STATUS_SUCCESS) {
        return status;
    }

    deviceRcn = pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION];

    if (deviceRcn >= VP880_REV_JE) {
        mpiCmdData = VP880_IODATA_IO1;
        VP_LINE_STATE(None, NULL, ("3. Write IODATA 0x%02X on Both Channels",
            VP880_IODATA_IO1));

        VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_IODATA_REG_WRT,
            VP880_IODATA_REG_LEN, &mpiCmdData);
        VpSysWait(24);  /* EMR's take 2-3 ms to open/close */
    }

    /* Proceed with normal device level reset and required delay */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_HW_RESET_WRT, 0, VP_NULL);
    VpSysWait(20);

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880DeviceReset-"));

    return VP_STATUS_SUCCESS;
}

/*******************************************************************************
 * Vp880InitDevicePcnRcn()
 *  Read in the PCN so we will know what type of device specifically we
 *  are working with.  This affects the max number of lines supported by the
 *  device as well as the type of lines (and may affect the line init)
 *
 *  Note: EC value is not important for this command
 *
 * Arguments:
 *  pDevObj         -
 *  deviceId        -
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
VpStatusType
Vp880InitDevicePcnRcn(
    Vp880DeviceObjectType   *pDevObj,
    VpDeviceIdType          deviceId)
{
    uint8 devicePcn, deviceRcn;

    /*
     * If Device has already been succesfully initialized, don't need to run
     * this process.
     */
    if (pDevObj->state & VP_DEV_INIT_CMP) {
        return VP_STATUS_SUCCESS;
    }

    /* If this is being called before InitDevice, clear the MPI buffer first */
    if (!((pDevObj->state & VP_DEV_INIT_CMP) ||
          (pDevObj->state & VP_DEV_INIT_IN_PROGRESS)))
    {
        VpCSLACClearMPIBuffer(deviceId);
    }

    /*
     * Read revision code
     * If >= JA then force I/O as below
     * Force I/O1 to '1', and wait for (if present) the external relay to
     * open
     */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_RCN_PCN_RD, VP880_RCN_PCN_LEN,
        pDevObj->staticInfo.rcnPcn);

    devicePcn = pDevObj->staticInfo.rcnPcn[VP880_PCN_LOCATION];
    deviceRcn = pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION];

    /* MPI Failure if the PCN and RCN are both 0x00 or 0xFF */
    if (((devicePcn == 0xFF) && (deviceRcn == 0xFF)) ||
        ((devicePcn == 0x00) && (deviceRcn == 0x00))) {
        pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);

        VP_ERROR(None, VP_NULL, ("Device Failed to Detect Revision/PCN Properly: 0x%02X 0x%02X",
            deviceRcn, devicePcn));

        VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevicePcnRcn-"));
        return VP_STATUS_FAILURE;
    }

    /*
     * Force the revision code to at least JE on devices that behave just
     * like JE and later devices, but have earlier revisions.
     *
     * NOTE: The revision code is used by I/O control, Set Relay State, Line
     * test and other. Be VERY CAREFULL changing this "rcn force" to make sure
     * no other major area is broken. A global search and code review everywhere
     * the RCN is evaluated is the minimum required before making such a change.
     */
    if (((devicePcn == VP880_DEV_PCN_88536) || (devicePcn == VP880_DEV_PCN_88264))
        && (deviceRcn < VP880_REV_JE)) {
        deviceRcn = VP880_REV_JE;
        pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] = VP880_REV_JE;
    }

    if (deviceRcn < VP880_REV_VC) {
        pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);

        VP_ERROR(None, VP_NULL, ("Unsupported Silicon Revision %d",
            pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION]));

        VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevicePcnRcn-"));
        return VP_STATUS_FAILURE;
    }

    /*
     * Verify that we recognize the device by limiting to the range of those supported in the
     * Vp880PcnType table. If not recognized (although may be a valid PN) return an error because
     * the API-II does not know how to handle it. More often, the error is occuring because the
     * hardware cannot talk to the device.
     */
    switch(devicePcn) {
#ifdef VP880_FXO_SUPPORT
        case VP880_DEV_PCN_88010:   /**< FXO Only Silicon */
            pDevObj->stateInt |= VP880_LINE0_IS_FXO;
#endif  /* VP880_FXO_SUPPORT */

#if defined (VP880_FXS_SUPPORT) && defined (VP880_TRACKER_SUPPORT)
        case VP880_DEV_PCN_88111:   /* FXS-Tracker */
        case VP880_DEV_PCN_88116:   /* FXS-Tracker - Wideband */
        case VP880_DEV_PCN_88131:   /* FXS-Tracker */
        case VP880_DEV_PCN_88136:   /* FXS-Tracker - Wideband */
#endif  /* (VP880_FXS_SUPPORT) && defined (VP880_TRACKER_SUPPORT) */
            pDevObj->staticInfo.maxChannels = 1;
            pDevObj->stateInt |= VP880_IS_SINGLE_CHANNEL;
            break;

#if defined (VP880_TRACKER_SUPPORT) || defined (VP880_FXO_SUPPORT)
        case VP880_DEV_PCN_88311:   /* FXO/FXS-Tracker */
        case VP880_DEV_PCN_88331:   /* FXO/FXS-Tracker */
            pDevObj->stateInt |= VP880_LINE1_IS_FXO;
#endif  /* (VP880_TRACKER_SUPPORT) || defined (VP880_FXO_SUPPORT) */

#ifdef VP880_FXS_SUPPORT
#ifdef VP880_TRACKER_SUPPORT
        case VP880_DEV_PCN_88211:   /* 2FXS-Tracker */
        case VP880_DEV_PCN_88216:   /* 2FXS-Tracker - Wideband */
        case VP880_DEV_PCN_88231:   /* 2FXS-Tracker */
        case VP880_DEV_PCN_88236:   /* 2FXS-Tracker - Wideband */
        case VP880_DEV_PCN_88506:   /* 2FXS-Tracker - Wideband Split Package*/
        case VP880_DEV_PCN_88536:   /* 2FXS-Tracker - Wideband, IP Block */
#endif  /* VP880_TRACKER_SUPPORT */

#ifdef VP880_ABS_SUPPORT
        case VP880_DEV_PCN_88221:   /* 2FXS-ABS */
        case VP880_DEV_PCN_88226:   /* 2FXS-ABS - Wideband */
        case VP880_DEV_PCN_88241:   /* 2FXS-ABS */
        case VP880_DEV_PCN_88246:   /* 2FXS-ABS - Wideband */
        case VP880_DEV_PCN_88264:   /* 2FXS-ABS - Wideband, ZSI */

        case VP880_DEV_PCN_88286_QFN:   /* 2FXS-ABS, HV, WB, No Test Load */
        case VP880_DEV_PCN_88266_QFN:   /* 2FXS-ABS, LV, WB, No Test Load */
#endif  /* VP880_ABS_SUPPORT */
#endif  /* VP880_FXS_SUPPORT */
            pDevObj->staticInfo.maxChannels = 2;
            pDevObj->stateInt &= ~VP880_IS_SINGLE_CHANNEL;
            break;

        default:
            pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);
            VP_ERROR(None, VP_NULL, ("Revision/PCN Unknown"));
            VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevicePcnRcn-"));
            return VP_STATUS_FAILURE;
    }

    if (devicePcn & VP880_TRACKER_MASK) {
#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
        /* Mark as non-ABS device type */
        pDevObj->stateInt &= ~VP880_IS_ABS;
#endif  /* (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT) */
    } else {
        if (devicePcn == VP880_DEV_PCN_88010) {
#ifdef VP880_FXO_SUPPORT
            /* FXO only devices */
            pDevObj->stateInt |= VP880_IS_FXO_ONLY;
#endif  /* VP880_FXO_SUPPORT */
        } else {
#ifdef VP880_ABS_SUPPORT
            /* Last choice is ABS type */
            pDevObj->stateInt |= VP880_IS_ABS;
#endif  /* VP880_ABS_SUPPORT */
        }
    }

    /*
     * Check for High Voltage Device and line test switch
     * Currently high voltage and test switch go hand in hand but may not in
     * the future that is why there are two bits but only a test for one.
     */
    if ((devicePcn & VP880_HV_MASK) == VP880_HV_MASK) {
        pDevObj->stateInt |= VP880_IS_HIGH_VOLTAGE;

        /* All HV devices support Line Test */
        pDevObj->stateInt |= VP880_IS_TEST_CAPABLE;

        if ((devicePcn != VP880_DEV_PCN_88506) && (devicePcn != VP880_DEV_PCN_88286_QFN)) {
            pDevObj->stateInt |= VP880_HAS_TEST_LOAD_SWITCH;
        }
    } else {
        /* Le88266 also supports Line Test but is LV */
        if ((devicePcn == VP880_DEV_PCN_88264) ||
            (devicePcn == VP880_DEV_PCN_88266_QFN) ||
            ((devicePcn == VP880_DEV_PCN_88226) && (deviceRcn > VP880_REV_VC))) {
            pDevObj->stateInt |= VP880_IS_TEST_CAPABLE;
        }
        pDevObj->stateInt &= ~VP880_IS_HIGH_VOLTAGE;
        pDevObj->stateInt &= ~VP880_HAS_TEST_LOAD_SWITCH;
    }

#ifdef VP880_ALWAYS_USE_INTERNAL_TEST_TERMINATION
    /* If this option is defined, we will treat this device as if it has no
     * physical test load.  The internal test termination only works for
     * revisions newer than VC, so ignore this override if it can't be used */
    if (deviceRcn > VP880_REV_VC) {
        pDevObj->stateInt &= ~VP880_HAS_TEST_LOAD_SWITCH;
    }
#endif /* VP880_ALWAYS_USE_INTERNAL_TEST_TERMINATION */

#ifdef VP_ENABLE_PROD_TEST
    /* Workaround to enable LM Production Tests */
    /*
     * The HV flag is incorrectly set (actually) but done so because earlier
     * Line Test used this flag for determing Line Test Capabilities. Later
     * Low Voltage silicon that supported Line Testing of course voided this
     * rule requiring a new flag that specifically indicates Test Capable or not
     * (hence - VP880_IS_TEST_CAPABLE). To be fully compatible, both flags must
     * be set.
     *
     * Note that these settings SHOULD NOT be enabled for normal production
     * purposes. Although it enables some amount of line testing (sufficient to
     * measure some key parameters in the Appliations lab), test performance and
     * accuracy is not gauranteed. Only a very limited number of tests from the
     * VeriVoice Line Test Suite are used. Other tests may not run at all or
     * run into a system lockup condition.
     */
    pDevObj->stateInt |= (VP880_IS_HIGH_VOLTAGE | VP880_IS_TEST_CAPABLE);
#endif  /* VP_ENABLE_PROD_TEST */

    /* Check for Wideband Mode support */
    if ((devicePcn == VP880_DEV_PCN_88506) ||
        (devicePcn == VP880_DEV_PCN_88286_QFN) ||
        (devicePcn == VP880_DEV_PCN_88266_QFN) ||
        ((devicePcn & VP880_IS_IT_WB) == VP880_IS_IT_WB)) {
        pDevObj->stateInt |= VP880_WIDEBAND;
    }

    /* Check for Cal Circuit and relay protection */
    if (VP880_REV_VC == deviceRcn) {
        if (pDevObj->stateInt & VP880_IS_SINGLE_CHANNEL) {
            /* none of the single channel rev 2 devices have a cal circuit */
            pDevObj->stateInt &= ~VP880_HAS_CALIBRATE_CIRCUIT;
        } else {
            pDevObj->stateInt |= VP880_HAS_CALIBRATE_CIRCUIT;
        }
    } else {
        /* all other revs should have cal circuit and require relay protection */
        pDevObj->stateInt |= VP880_HAS_CALIBRATE_CIRCUIT;
        pDevObj->stateInt |= VP880_RLY_PROT_REQ;
    }

    if ((devicePcn == VP880_DEV_PCN_88506) || (devicePcn == VP880_DEV_PCN_88536)) {
        pDevObj->stateInt |= VP880_RLY_PROT_REQ;
    }

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880InitDevice
 *  This function initializes the device and all lines associated with this
 * device (if line profiles are passed to this function). The device profile
 * passed must be valid otherwise an error code is returned and the device
 * remains in it's previously initialized state.
 *
 * Preconditions:
 *  None (device context is not NULL and is of Vp880 type, which is handled in
 * higher level software)
 *
 * Postconditions:
 *  This device is initialized to the configuration specified in the device
 * profile, and the FXS lines associated with this device are initialized by the
 * FXS specific AC, DC, and Ringing profiles passed, and the FXO lines
 * associated with this device are initialized by the FXO specific AC and Config
 * profiles passed.  If the FXO/FXS profiles are all NULL, then only the device
 * initialization occurs. This function returns an error code if the device
 * profile trying to be used for initialization is VP_PTABLE_NULL (either
 * passed or by a non-initialized index).
 */
VpStatusType
Vp880InitDevice(
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
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp880LineObjectType *pLineObj;
    uint8 ecVal = pDevObj->ecVal;
    uint8 ecAll = (VP880_EC_CH1 | VP880_EC_CH2);

    uint8 maxChan, chan, pcn, rcn, mpiData;
    VpProfilePtrType pDevProf;

    /*
     * ICR3 Values are used to enable Vref on all lines of the device before bringing up the
     * supplies. Otherwise, the supplies or channels could shutdown. Even if the supplies remain
     * running correctly, Vref must be stable for other circuitry to work as expected. The SW
     * should not try to proceed until it is certain Vref is up and stable.
     */
    uint8 icr3Vals[VP880_ICR3_LEN] = {0x00, 0x00, 0x00, 0x00};

#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
    uint8 intSwParam[VP880_INT_SWREG_PARAM_LEN] = {0x5C, 0x4B, 0xC4, 0x4B, 0xC4, 0x4B};
#endif  /* defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT) */

#ifdef VP880_ABS_SUPPORT
    uint8 systemConfig = 0;
#endif  /* VP880_ABS_SUPPORT */

#ifdef VP880_CURRENT_LIMIT
    uint8 intSwParamLimit[VP880_INT_SWREG_PARAM_LEN] = {0xB2, 0x00, 0xB1, 0x00, 0x60, 0x40};
#endif  /* VP880_CURRENT_LIMIT */

    VpStatusType status = VP_STATUS_SUCCESS;

    int profIndex = VpGetProfileIndex(pDevProfile);

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice+"));

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
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice-"));
                return VP_STATUS_ERR_PROFILE;
            }
        } else {
            VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
            pDevObj->state &= ~VP_DEV_INIT_IN_PROGRESS;
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice-"));
            return VP_STATUS_ERR_PROFILE;
        }
        pDevProf = pDevProfile;
    } else if (profIndex < VP_CSLAC_DEV_PROF_TABLE_SIZE) {
        pDevProf = pDevObj->devProfileTable.pDevProfileTable[profIndex];
        if (pDevProf == VP_PTABLE_NULL) {
            return VP_STATUS_ERR_PROFILE;
        }
    } else {
        VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice-"));
        return VP_STATUS_ERR_PROFILE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Initialize the API's device status variables */
    if (pDevObj->state & VP_DEV_INIT_CMP) {
        pDevObj->state = (VP_DEV_WARM_REBOOT | VP_DEV_INIT_IN_PROGRESS);
    } else {
        pDevObj->state = VP_DEV_INIT_IN_PROGRESS;
    }
    pDevObj->timeStamp = 0;

    /* Initialize the API's device dynamic variables */
    pDevObj->dynamicInfo.lastChan = 0;
    pDevObj->dynamicInfo.bat1Fault = FALSE;
    pDevObj->dynamicInfo.bat2Fault = FALSE;
    pDevObj->dynamicInfo.bat3Fault = FALSE;
    pDevObj->dynamicInfo.clkFault = FALSE;

    /*
     * Reset the internal state information except for possibly previously
     * loaded calibration values.
     */
    pDevObj->stateInt &= (VP880_SYS_CAL_COMPLETE | VP880_DEVICE_CAL_COMPLETE);

    pDevObj->devProfileData.profVersion = (uint8)(pDevProf[VP_PROFILE_VERSION]);

    pDevObj->devProfileData.pcmClkRate =
        (uint16)(((pDevProf[VP880_DEV_PROFILE_PCLK_MSB] << 8) & 0xFF00)
                | (pDevProf[VP880_DEV_PROFILE_PCLK_LSB] & 0x00FF));

    pDevObj->devProfileData.devCfg1 = (uint8)(pDevProf[VP880_DEV_PROFILE_DEVCFG1]);
    pDevObj->devProfileData.clockSlot = (uint8)(pDevProf[VP880_DEV_PROFILE_CLOCK_SLOT]);
    pDevObj->devProfileData.systemConfig = (uint8)(pDevProf[VP880_DEV_PROFILE_SYSTEM_CFG]);

    pDevObj->devProfileData.tickRate =
        (uint16)(((pDevProf[VP880_DEV_PROFILE_TICKRATE_MSB] << 8) & 0xFF00)
                | (pDevProf[VP880_DEV_PROFILE_TICKRATE_LSB] & 0x00FF));

#ifdef VP880_FXS_SUPPORT
    if (pDevProf[VP880_DEV_PROFILE_OPERATIONAL_CFG] & VP880_DEV_PROFILE_PK_PWR_MGMT) {
        pDevObj->devProfileData.peakManagement = TRUE;
    } else {
        pDevObj->devProfileData.peakManagement = FALSE;
    }

    if (pDevProf[VP880_DEV_PROFILE_OPERATIONAL_CFG] & VP880_DEV_PROFILE_LOW_VOLT_OVERRIDE) {
        pDevObj->devProfileData.lowVoltOverride = TRUE;
    } else {
        pDevObj->devProfileData.lowVoltOverride = FALSE;
    }
#endif  /* VP880_FXS_SUPPORT */

    /*
     * Support only version of the profile that contain the switching
     * regulator parameters. Otherwise, we really don't know what to do...
     */
    if (pDevProf[VP_PROFILE_VERSION] < VP_CSLAC_DEV_PROFILE_VERSION_SW_CONFIG) {
        pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);
        /*
         * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
         * completed. This prevents VpApiTick() from generating events on a non-initialized device
         * context/object.
         */
        pDevObj->devProfileData.tickRate = 0;

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

        VP_ERROR(None, VP_NULL, ("Unsupported Device Profile Version"));

        VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice Unsupported Profile Version-"));
        return VP_STATUS_ERR_PROFILE;
    } else {
        VpMemCpy(pDevObj->swParams, &pDevProf[VP880_DEV_PROFILE_SWITCHER_CMD+1],
            VP880_REGULATOR_PARAM_LEN);
    }

    /* Initialize device */
    /*
     * If not successful, the Clock Fail bit did not clear so return error code
     */
    if ((status = Vp880Init(pDevCtx)) != VP_STATUS_SUCCESS) {
        pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);
        /*
         * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
         * completed. This prevents VpApiTick() from generating events on a non-initialized device
         * context/object.
         */
        pDevObj->devProfileData.tickRate = 0;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC_INT(None, VP_NULL, ("Vp880Init Init Failure-"));
        return status;
    }
    pcn = pDevObj->staticInfo.rcnPcn[VP880_PCN_LOCATION];
    rcn = pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION];

    maxChan = pDevObj->staticInfo.maxChannels;

#ifdef VP880_LP_SUPPORT
    /*
     * Make initial assumption that the device has lines such that low power
     * mode is in affect. The line controls will change as necessary.
     */
    pDevObj->stateInt |= (VP880_LINE0_LP | VP880_LINE1_LP);
#else   /* VP880_LP_SUPPORT */
    pDevObj->stateInt &= ~(VP880_LINE0_LP | VP880_LINE1_LP);
#endif  /* VP880_LP_SUPPORT */

    /*
     * Before powering up the supplies, we have to disable the silicon Auto-State Control.
     * Otherwise, the line can go back into Shutdown after being set to disconnect due to
     * a brief overcurrent detect. Note that for all VE880 silicon, including FXO only
     * devices, the value of 0x02 (ASSC = 1 = Disable) means the same thing. So we don't
     * need here to be concerned about the device or termination type. This value will be
     * corrected during VpInitLine() when we do know the termination type.
     */
    mpiData = VP880_AUTO_SSC_DIS;
    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
        VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, &mpiData);

    /*
     * Enable Vref in order to make sure all other functions of the silicon work correctly
     * during powerup.
     */
    icr3Vals[VP880_ICR3_LINE_CTRL_INDEX] =  VP880_ICR3_VREF_CTRL;
    icr3Vals[VP880_ICR3_LINE_CTRL_INDEX+1] =  VP880_ICR3_VREF_CTRL;
    VpMpiCmdWrapper(deviceId, ecAll, VP880_ICR3_WRT, VP880_ICR3_LEN, icr3Vals);
    VP_LINE_STATE(None, NULL, ("Init: ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
        icr3Vals[0], icr3Vals[1], icr3Vals[2], icr3Vals[3], ecAll));

    /*
     * Must wait at least 5ms before turning the switchers on for Vref to stabilize.
     * This waits 10ms just to be safe.
     */
    VpSysWait(80);

    /* Check if device is Tracker and if so, initialize for FXS functionality */
    if (pcn & VP880_TRACKER_MASK) {
#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
        /* Initialize Tracker device sensitve items */
        /*
         * Configure the Switcher for Flyback or Buckboost per device
         * profile. If the device is in Buckboost mode, config the internal
         * switcher
         */
        if (pDevProf[VP_PROFILE_VERSION] >= VP_CSLAC_DEV_PROFILE_VERSION_INT_SW_CONFIG) {
            /* Write the internal switcher parameters */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
                VP880_INT_SWREG_PARAM_LEN, (uint8 *)&pDevProf[VP880_DEV_PROFILE_TRACKER_INT_SW_REG]);
        } else {
            /* Write the default internal parameters */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
                VP880_INT_SWREG_PARAM_LEN, intSwParam);
        }

        if (pDevProf[VP_PROFILE_VERSION] >= VP_CSLAC_DEV_PROFILE_VERSION_INT_SW_CONFIG_FR) {
            /* Cache the internal switcher parameters for free run mode */
            VpMemCpy(pDevObj->intSwParamsFR, &pDevProf[VP880_DEV_PROFILE_TRACKER_INT_SW_REG +
                VP880_INT_SWREG_PARAM_LEN], VP880_INT_SWREG_PARAM_LEN);
        } else {
            Vp880CopyDefaultFRProfile(pDevObj);
        }

        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_WRT,
            VP880_REGULATOR_PARAM_LEN, pDevObj->swParams);
        VpMemCpy(pDevObj->swParamsCache, pDevObj->swParams, VP880_REGULATOR_PARAM_LEN);

        /*
         * Enable the switchers in low power. The power mode is changed as
         * needed during normal operation.
         */
        mpiData = VP880_SWY_LP | VP880_SWZ_LP;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
            VP880_REGULATOR_CTRL_LEN, &mpiData);
#endif  /* (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)   */
    } else {
#ifdef VP880_ABS_SUPPORT
        if (pDevProf[VP_PROFILE_VERSION] >=
            VP_CSLAC_DEV_PROFILE_VERSION_INT_SW_CONFIG) {

            /* Write the internal switcher parameters */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
                VP880_INT_SWREG_PARAM_LEN, (uint8 *)&pDevProf[VP880_DEV_PROFILE_ABS_INT_SW_REG]);
            /* Extract Y/Z Voltages. "0" if not set by Profile Wizard */
            pDevObj->yVolt = pDevProf[VP880_ABS_DEV_PROFILE_YVOLT];
            pDevObj->zVolt = pDevProf[VP880_ABS_DEV_PROFILE_ZVOLT];
        }

        if (pDevProf[VP_PROFILE_VERSION] >= VP_CSLAC_DEV_PROFILE_VERSION_INT_SW_CONFIG_FR) {
            /* Cache the internal switcher parameters for free run mode */
            VpMemCpy(pDevObj->intSwParamsFR, &pDevProf[VP880_DEV_PROFILE_ABS_INT_SW_REG +
                VP880_INT_SWREG_PARAM_LEN], VP880_INT_SWREG_PARAM_LEN);
        } else {
            /* No free run timing available -> save default ones */
            Vp880CopyDefaultFRProfile(pDevObj);
        }

        /*
         * Make sure lines are in disconnect state (recover from shutdown)
         * even if there are no line contexts associated with this device.
         */
        mpiData = VP880_SS_DISCONNECT;
        VpMpiCmdWrapper(deviceId, ecAll, VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &mpiData);

        mpiData = VP880_SWY_OFF;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
            VP880_REGULATOR_CTRL_LEN, &mpiData);

#ifdef VP880_CURRENT_LIMIT
        /* Implement a user compile option to limit the switcher power */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
            VP880_INT_SWREG_PARAM_LEN, intSwParamLimit);
#endif  /* VP880_CURRENT_LIMIT */

        if (!(pDevObj->stateInt & VP880_IS_FXO_ONLY)) {
            systemConfig = (pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK);

#ifdef VP880_AUTO_BAT_DETECT
            if ((status = Vp880AutoBatDetect(pDevObj,
                &pDevProf[VP880_DEV_PROFILE_SWITCHER_CMD+1])) != VP_STATUS_SUCCESS) {
                pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);

                /*
                 * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
                 * completed. This prevents VpApiTick() from generating events on a non-initialized
                 * device context/object.
                 */
                pDevObj->devProfileData.tickRate = 0;
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice Exit Error %d-", status));
                return status;
            }
#else   /* VP880_AUTO_BAT_DETECT */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_WRT,
                VP880_REGULATOR_PARAM_LEN, pDevObj->swParams);
            VpMemCpy(pDevObj->swParamsCache, pDevObj->swParams, VP880_REGULATOR_PARAM_LEN);

            /*
             * Only enably the supplies here IF run-time calibration is NOT being run. Otherwise,
             * (if run-time cal IS being run) this will turn the supplies on then Calibration will
             * turn it almost immediately off causing an undesireable glitch. Calibration works
             * either way, but may work faster if the voltage is not pre-charged and may provide a
             * more accurate battery offset measurement - not to forget the undesireble glitch.
             */
#ifndef VP_CSLAC_RUNTIME_CAL_ENABLED
            /*
             * All configurations - Master, Single and Slave require both supplies set to "on". This
             * is because the silicon internally looks for the supply bits to be "on" otherwise
             * internally shutdown some circuitry. In case of Slave, this doesn't matter because
             * there are no supplies connected anyway. In case of Master and Single, this is the
             * required starting point. In any case - if the requirement for Slave Mode changes such
             * that the supply does NOT need to be enabled, this logic can be changed.
             */
            mpiData = VP880_SWY_LP | VP880_SWZ_LP;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
                VP880_REGULATOR_CTRL_LEN, &mpiData);

            /*
             * If not Slave Mode, only options are WE are controlling the supplies. Here's where it
             * get's interesting...
             */
            if (systemConfig != VP880_ABS_CFG_SLAVE) {
                VpSysWait(240); /* 125us * 240 = 30ms */
                VpSysWait(160); /* 125us * 160 = 20ms */
                if (systemConfig == VP880_ABS_CFG_SINGLE) {
                    /*
                     * Single Mode means we only have to "feed" ourselves and no other device. So
                     * we only set the supply to Medium Power Mode unless in the Ringing State. In
                     * Ringing we go to High Power Mode.
                     */
                    mpiData = VP880_SWY_MP | VP880_SWZ_MP;
                } else {    /* systemConfig == VP880_ABS_CFG_MASTER */
                    /*
                     * Master Mode means we're feeding outself and others - sort of like the US Tax
                     * System;-) Anyway, it means other devices using the supply we're controlling
                     * WILL be in the Ringing state without our knowledge at some time. So we have
                     * no choice but to remain in High Power Mode forever...
                     */
                    mpiData = VP880_SWY_HP | VP880_SWZ_HP;
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
                    VP880_REGULATOR_CTRL_LEN, &mpiData);
            }
#endif
#endif  /* VP880_AUTO_BAT_DETECT */
        }
#endif  /* VP880_ABS_SUPPORT */
    }

    /*
     * No matter how they were provided, cache the internal switcher parameters
     * and the switcher parameters.
     */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_RD,
        VP880_INT_SWREG_PARAM_LEN, pDevObj->intSwParams);
    VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_RD,
        VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
    VpMemCpy(pDevObj->swParams, pDevObj->swParamsCache,
        VP880_REGULATOR_PARAM_LEN);

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
            pLineObj = pLineCtx->pLineObj;

            if (pLineObj->status & VP880_IS_FXO) {
                status = Vp880InitLine(pLineCtx, pFxoAcProfile, pFxoCfgProfile,
                    VP_PTABLE_NULL);
            } else {
                status = Vp880InitLine(pLineCtx, pAcProfile, pDcProfile,
                    pRingProfile);
#ifdef VP880_INCLUDE_TESTLINE_CODE
                /* initialize the calibration coeffs */
                pDevObj->calOffsets[chan].nullOffset = 0;
                pDevObj->calOffsets[chan].vabOffset = 0;
                pDevObj->calOffsets[chan].vahOffset = 0;
                pDevObj->calOffsets[chan].vbhOffset = 0;
#endif /* VP880_INCLUDE_TESTLINE_CODE */
            }

            if (status != VP_STATUS_SUCCESS) {
                pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_INIT_CMP);
                /*
                 * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
                 * completed. This prevents VpApiTick() from generating events on a non-initialized
                 * device context/object.
                 */
                pDevObj->devProfileData.tickRate = 0;
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice Exit Error %d-",
                    status));
                return status;
            }
        }
    }

    status = VpImplementDefaultSettings(pDevCtx, VP_NULL);

    /*
     * This clears the Init Line Events and any other erroneous event that
     * may have been created due to initialization
     */
    Vp880FlushEvents(pDevCtx);

#ifdef VP880_INCLUDE_TESTLINE_CODE
    /*
     * This clears the Test structure
     */
    pDevObj->currentTest.prepared = FALSE;
    pDevObj->currentTest.testState = -1;
    pDevObj->currentTest.testId = VP_NUM_TEST_IDS;
#endif /* VP880_INCLUDE_TESTLINE_CODE */

    if (status == VP_STATUS_SUCCESS) {
        if (pDevObj->stateInt & VP880_DEVICE_CAL_COMPLETE) {
            VP_CALIBRATION(None, VP_NULL, ("Calibration Previously Complete: Device 0x%08lX RCN %d",
                pDevObj->stateInt, rcn));
            pDevObj->state &= ~VP_DEV_IN_CAL;
            pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS);
            pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;

            VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_BAT_CALIBRATION_WRT,
                VP880_BAT_CALIBRATION_LEN, pDevObj->calData.abvData.switcherAdjust[0]);

            VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_BAT_CALIBRATION_WRT,
                VP880_BAT_CALIBRATION_LEN, pDevObj->calData.abvData.switcherAdjust[1]);
        } else {
#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
            VP_CALIBRATION(None, VP_NULL, ("Cal Required: Device 0x%08lX RCN %d",
                pDevObj->stateInt, rcn));

            pDevObj->state |= VP_DEV_IN_CAL;
            pDevObj->calData.calDeviceState = VP880_CAL_INIT;

            if (pDevObj->stateInt & VP880_IS_ABS) { /* Start for ABS Device */
#ifdef VP880_ABS_SUPPORT
                if ((systemConfig != VP880_ABS_CFG_SLAVE) && (Vp880SetCalFlags(pDevObj) == TRUE)) {
                    VP_CALIBRATION(None, VP_NULL, ("NOT Running Slave Mode Device 0x%08lX RCN %d",
                        pDevObj->stateInt, rcn));
                    Vp880CalCodecInt(pDevCtx);
                } else {
                    VP_CALIBRATION(None, VP_NULL, ("Slave Mode OR Cal Flags = FALSE Device 0x%08lX RCN %d",
                        pDevObj->stateInt, rcn));

                    pDevObj->state &= ~VP_DEV_IN_CAL;
                    pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS);
                    pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
                }
#endif  /* VP880_ABS_SUPPORT */
            } else {    /* Start for Tracker Device if JE (JA 8827x) silicon */
#ifdef VP880_TRACKER_SUPPORT
                if (rcn >= VP880_REV_JE) {
                    pDevObj->state |= VP_DEV_ABV_CAL;
                    Vp880CalCodecInt(pDevCtx);
                } else {
                    pDevObj->state &= ~VP_DEV_IN_CAL;
                    pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS);
                    pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
                }
#endif  /* VP880_TRACKER_SUPPORT */
            }
#else   /* VP_CSLAC_RUNTIME_CAL_ENABLED */
            VP_CALIBRATION(None, VP_NULL, ("\n\rCal Required, Disabled at Compile: Device 0x%08lX RCN %d",
                pDevObj->stateInt, rcn));
            pDevObj->state &= ~VP_DEV_IN_CAL;
            pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS);
            pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
#endif  /* VP_CSLAC_RUNTIME_CAL_ENABLED */
        }
    }

    /*
     * ZSI devices can't have TX Timeslot = 0. So initialize the channels to
     * [tx, rx] -> channel 0 = [1, 1], channel 1 = [2, 2]
     */
    if ((pcn == VP880_DEV_PCN_88536) || (pcn == VP880_DEV_PCN_88264)) {
        uint8 txTimeSlot = 0;
        uint8 rxTimeSlot = 1;

        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_TX_TS_WRT, VP880_TX_TS_LEN,
            &txTimeSlot);
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_RX_TS_WRT, VP880_RX_TS_LEN,
            &rxTimeSlot);

        txTimeSlot = 1;
        rxTimeSlot = 2;

        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_TX_TS_WRT, VP880_TX_TS_LEN,
            &txTimeSlot);
        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_RX_TS_WRT, VP880_RX_TS_LEN,
            &rxTimeSlot);
    }

    /*
     * Success, Failure, or Calibration started -- we're not in "InitDevice"
     * function anymore. So normal rules apply.
     */
    pDevObj->state |= VP_DEV_INIT_CMP;
    if (status != VP_STATUS_SUCCESS) {
        /*
         * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
         * completed. This prevents VpApiTick() from generating events on a non-initialized device
         * context/object.
         */
        pDevObj->devProfileData.tickRate = 0;
    }
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880InitDevice Exit Normal: State 0x%04X Event 0x%04X-",
        pDevObj->state, pDevObj->deviceEvents.response));
    return status;
} /* Vp880InitDevice */

/**
 * Vp880InitLine
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
Vp880InitLine(
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
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 channelId = pLineObj->channelId;
    uint8 ecValMap[]  = {VP880_EC_CH1, VP880_EC_CH2};
    uint8 ecVal       = ecValMap[channelId];

    uint8 alwaysOn[VP880_CADENCE_TIMER_LEN] = {0x3F, 0xFF, 0x00, 0x00};
    uint8 opFunc[VP880_OP_FUNC_LEN] = {VP880_ENABLE_LOADED_COEFFICIENTS};

#ifdef VP880_FXS_SUPPORT
    uint8 defaultRingParams[VP880_RINGER_PARAMS_LEN] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    VpProfileDataType dcDefaultProf[] = {
        0x00, 0x01, 0x01, 0x0A, 0x00, 0x08, 0xC2, 0x1B, 0x84, 0xB3, 0x05, 0xC6,
        0x13, 0x08
    };
#endif  /* VP880_FXS_SUPPORT */

#ifdef VP880_FXO_SUPPORT
    VpProfileDataType fxoDefaultProf[] =
    {
        /* FXO/Dialing Profile */
        0x00, 0xFE, 0x00, 0x12, 0x00, 0x00, 0x00, 0x27, 0x00, 0x28, 0x00, 0x78,
        0x0C, 0x08, 0x00, 0x28, 0xEB, 0x79, 0x04, 0x03, 0x26, 0x3A
    };

    uint8 fxoCidLine;
#endif  /* VP880_FXO_SUPPORT */

    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 mpiData;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    pLineObj->ecVal = ecVal;
    pLineObj->status &= ~VP880_INIT_COMPLETE;

#ifdef VP_CSLAC_SEQ_EN
    VpMemSet(pLineObj->intSequence, 0, VP880_INT_SEQ_LEN);

#ifdef VP880_FXS_SUPPORT
    pLineObj->callerId.status = VP_CID_RESET_VALUE;
    pLineObj->suspendCid = FALSE;

    pLineObj->pRingingCadence = VP_PTABLE_NULL;
    pLineObj->pCidProfileType1 = VP_PTABLE_NULL;
#endif  /* VP880_FXS_SUPPORT */
#endif  /* VP_CSLAC_SEQ_EN */

    pLineObj->status &= ~(VP880_BAD_LOOP_SUP);

    /* Initialize cached transmit and receive gains for SetRelGain to 1.0. */
    pLineObj->gain.gxInt = 0x4000;
    pLineObj->gain.grInt = 0x4000;

    /* Inititialize API line state variables */
    if (pLineObj->status & VP880_IS_FXO) {
#ifdef VP880_FXO_SUPPORT
        pLineObj->lineState.currentState = VP_LINE_FXO_LOOP_OPEN;
        pLineObj->lineState.previous = VP_LINE_FXO_LOOP_OPEN;
#endif  /* VP880_FXO_SUPPORT */
    } else {
#ifdef VP880_FXS_SUPPORT
        pLineObj->lineState.currentState = VP_LINE_DISCONNECT;
        pLineObj->lineState.previous = VP_LINE_DISCONNECT;
        pLineObj->lineState.usrCurrent = VP_LINE_DISCONNECT;
#endif  /* VP880_FXS_SUPPORT */
    }

    /* Force a line state check and update hook information */
    pLineObj->lineState.condition = VP_CSLAC_STATUS_INVALID;

#ifdef VP880_FXS_SUPPORT
    pLineObj->dpStruct.hookSt = FALSE;
    pLineObj->dpStruct2.hookSt = FALSE;

    VpInitDP(&pLineObj->dpStruct);
    VpInitDP(&pLineObj->dpStruct2);

    pLineObj->internalTestTermApplied = FALSE;

    /* Set the cached ICR values to device reset conditions. */
    VpMemSet(pLineObj->icr1Values, 0, VP880_ICR1_LEN);
    VpMemSet(pLineObj->icr2Values, 0, VP880_ICR2_LEN);
    VpMemSet(pLineObj->icr3Values, 0, VP880_ICR3_LEN);
    VpMemSet(pLineObj->icr4Values, 0, VP880_ICR4_LEN);
#endif  /* VP880_FXS_SUPPORT */

#ifdef VP880_LP_SUPPORT
    pLineObj->leakyLineCnt = 0;    /* Used only for LP Mode */

    if (VpIsLowPowerTermType(pLineObj->termType) == TRUE) {
        /*
         * Manually Force the ICR values for LPM-Disconnect since we have no
         * idea the status of the other line.
         */
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Updating Cached ICR Values for LPM on Channel %d", channelId));

        pLineObj->icr1Values[0] = 0x0F;
        pLineObj->icr1Values[2] = 0xC0;

        pLineObj->icr2Values[0] = 0xCC;
        pLineObj->icr2Values[1] = 0x4C;
        pLineObj->icr2Values[2] = 0x2C;
        pLineObj->icr2Values[3] = 0x58;

        pLineObj->icr3Values[0] = 0x31;
        pLineObj->icr3Values[1] = 0x21;

        pLineObj->icr4Values[2] = 0x0E;
        pLineObj->icr4Values[3] = 0x0E;

        pLineObj->status |= VP880_LOW_POWER_EN;
#ifdef VP880_LPM_OVERRIDE
        pLineObj->status |= VP880_LP_STANDBY_IDLE;
#endif /* VP880_LPM_OVERRIDE */
        pDevObj->stateInt |= ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);
    }
#endif  /* VP880_LP_SUPPORT */

#ifdef VP_CSLAC_SEQ_EN
    VpMemSet(&pLineObj->cadence, 0, sizeof(VpSeqDataType));
#endif  /* VP_CSLAC_SEQ_EN */

    /* Force a codec update */
    pLineObj->codec = VP_NUM_OPTION_CODEC_TYPE_IDS;

#ifdef VP880_FXS_SUPPORT
    /* It is possible that CalCodec set ABS calibration values in ICR6 before
     * this point, so read these from the device */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR6_RD, VP880_ICR6_LEN, pLineObj->icr6Values);

    /*
     * For future reference, make sure the battery switch threshold is what we want just so the
     * behavior is consistent.
     */
    if (pDevObj->stateInt & VP880_IS_ABS) {
        pLineObj->icr6Values[1] &= ~VP880_DCCAL_BAT_SW_HYST_MASK;
        pLineObj->icr6Values[1] |= VP880_DCCAL_BAT_SW_HYST_5V;
    }

    /*
     * Workaround for I/O1 relay driver. Problem is the I/O1 pin does not have
     * proper voltage clamps to protect against normal voltage spikes that
     * occur as a result of transitioning to Input or Open Drain on a driven
     * relay coil. So this workaround will check to see if the pin is currently
     * being used to close an external relay, and force it open. Then it has
     * to wait for 3ms for the relay to fully open (coil to fully discharge).
     */
    if ((pDevObj->stateInt & VP880_RLY_PROT_REQ)
      && ((pLineObj->termType == VP_TERM_FXS_ISOLATE)
      ||  (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)
      ||  (pLineObj->termType == VP_TERM_FXS_SPLITTER))) {

        if((pLineObj->termType == VP_TERM_FXS_ISOLATE) ||
           (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)) {
            Vp880SetRelayState(pLineCtx, VP_RELAY_NORMAL);
        } else {
            Vp880SetRelayState(pLineCtx, VP_RELAY_RESET);
        }

        /* Wait for relay coil to fully discharge */
        VpSysWait(24);
    }
#endif  /* VP880_FXS_SUPPORT */

    /*
     * Operating Conditions - Remove all loopbacks, connect TX/RX PCM Hwy
     * Note that TX/RX PCM Highway is set when Set Linestate function is
     * called.
     */
    pLineObj->opCond[0] = VP880_NORMAL_OP_COND_MODE;
    VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT, VP880_OP_COND_LEN,
        pLineObj->opCond);

    /* Operating Functions - Use loaded coefficients */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_FUNC_WRT, VP880_OP_FUNC_LEN,
        opFunc);

    /* Disable the internal device cadencer .. done in the API */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_CADENCE_TIMER_WRT,
        VP880_CADENCE_TIMER_LEN, alwaysOn);

    /* Start the channel out in the standby state or loop open (if FXO)  */
    if (pLineObj->status & VP880_IS_FXO) {
#ifdef VP880_FXO_SUPPORT
        pLineObj->lineTimers.type = VP_CSLAC_FXO_TIMER;
        /*
         * InitTimerVars has to know the timer type but called before
         * Set Line State. If set line state starts any timers, calling
         * InitTimerVars would disable those causing possible initialization
         * issues.
         */
        InitTimerVars(pLineCtx);

        /* Disable auto system state control
         * NOTE: NEVER enable the Auto-Thermal Fault disconnect in the silicon
         * because the silicon is too fast for the VP-API-II. It would be possible
         * to get a thermal fault, have the silicon disable the line, and have
         * the thermal fault go away all before the VP-API-II sees it. In that
         * condition, the line will be disabled without the application being
         * aware of it.
         */
        mpiData = VP880_AUTO_SSC_DIS;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, &mpiData);

        if (pLineObj->termType == VP_TERM_FXO_DISC) {
            fxoCidLine = VP880_IODATA_IO3;
        } else {
            fxoCidLine = VP880_FXO_CID_LINE;
        }

        mpiData &= ~VP880_IODIR_IO1_MASK;
        mpiData = (VP880_IODIR_IO1_OUTPUT | (fxoCidLine << 1));

#ifdef VP880_CLARE_RINGING_DETECT
        mpiData |= VP880_IODIR_EXPDT_MASK;    /* Ringing detector on IO4 */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_IODIR_REG_WRT, VP880_IODIR_REG_LEN, &mpiData);
#endif  /* VP880_CLARE_RINGING_DETECT */

#ifdef VP_CSLAC_SEQ_EN
        pLineObj->digitGenStruct.dtmfOnTime = VP_FXO_DTMF_ON_DEFAULT;
        pLineObj->digitGenStruct.dtmfOffTime = VP_FXO_DTMF_OFF_DEFAULT;
        pLineObj->digitGenStruct.breakTime = VP_FXO_PULSE_BREAK_DEFAULT;
        pLineObj->digitGenStruct.makeTime = VP_FXO_PULSE_MAKE_DEFAULT;
        pLineObj->digitGenStruct.flashTime = VP_FXO_FLASH_HOOK_DEFAULT;
        pLineObj->digitGenStruct.dpInterDigitTime = VP_FXO_INTERDIG_DEFAULT;
        pLineObj->digitGenStruct.dtmfHighFreqLevel[0] = 0x1C;
        pLineObj->digitGenStruct.dtmfHighFreqLevel[1] = 0x32;
        pLineObj->digitGenStruct.dtmfLowFreqLevel[0] = 0x1C;
        pLineObj->digitGenStruct.dtmfLowFreqLevel[1] = 0x32;
#endif  /* VP_CSLAC_SEQ_EN */

        if (pDcOrFxoProfile == VP_NULL) {
            status = Vp880ConfigLine(pLineCtx, pAcProfile, fxoDefaultProf, VP_PTABLE_NULL);
        } else {
            status = Vp880ConfigLine(pLineCtx, pAcProfile, pDcOrFxoProfile, VP_PTABLE_NULL);
        }

        if (status != VP_STATUS_SUCCESS) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return status;
        }
        /* Activate Codec and enable Supervision */
        /*
         * Set to Loop Open. Set current state to Loop Close in order to force line
         * state function to change to Loop Open. Otherwise, no changes are made
         * since it thinks device is already set correctly.
         */
        pLineObj->lineState.currentState = VP_LINE_FXO_LOOP_CLOSE;
        Vp880SetLineStateInt(pLineCtx, VP_LINE_FXO_LOOP_OPEN);
        pLineObj->lineState.usrCurrent = VP_LINE_FXO_LOOP_OPEN;
#endif  /* VP880_FXO_SUPPORT */
    } else {
#ifdef VP880_FXS_SUPPORT
        /*
         * Take care of the calibration flags first. These will be used in "Update Cal Data" later
         * when setting target ILA and VOC.
         *
         * If the line object "calDone" was already set, just leave it alone. It's also possible
         * that "Apply Calibration" was performed and a VpApiTick() has not yet passed. So only
         * the device level flag is aware of it, not the line level flag. We have to set it now in
         * that case.
         */
        /*
         * This flag is set when either the run-time calibration is complete at the device level
         * and at both lines, or when the system calibration coefficients have been provided. Either
         * way, THIS line does not require calibration.
         */
        if (pDevObj->stateInt & VP880_SYS_CAL_COMPLETE) {
            /*
             * IF this isn't set, then Vp880AdjustIla() won't realize the "line" has been calibrated
             * and will return FALSE during Update Calibrate function. This change corrects the
             * VAS=3V initialization issue (if VpCal() is called prior to VpInitDevice()).
             */
            pLineObj->calLineData.calDone = TRUE;
        }
        pLineObj->status &= ~VP880_LINE_IN_CAL;
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("VpInitLine: Ch (%d) Dev State (0x%08lX)  Line Status (0x%04X) Line CalDone (%s)",
             channelId, pDevObj->stateInt, pLineObj->status,
             ((pLineObj->calLineData.calDone) ? "TRUE" : "FALSE")));

        /*
         * LPM supported on HV-Tracker devices only. Note that the HV check for
         * LPM support ignores if there is an application level LV override.
         * The override occurs because customers want a LV (low cost) design,
         * but are given only HV silicon.
         */
        if ((pDevObj->stateInt & VP880_IS_ABS) || (!(pDevObj->stateInt & VP880_IS_HIGH_VOLTAGE))) {
            if (VpIsLowPowerTermType(pLineObj->termType)) {
                pLineObj->termType = VP_TERM_FXS_GENERIC;
                pDevObj->stateInt &= ~VP880_LINE0_LP;
                pDevObj->stateInt &= ~VP880_LINE1_LP;
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_ERR_VTD_CODE;
            }
        }

        pLineObj->lineTimers.type = VP_CSLAC_FXS_TIMER;
        /*
         * InitTimerVars has to know the timer type but called before
         * Set Line State. If set line state starts any timers, calling
         * InitTimerVars would disable those causing possible initialization
         * issues.
         */
        InitTimerVars(pLineCtx);

        /*
         * Enable Auto Bat Switch (ABS), Disable Auto-Battery Shutdown (Tracker)
         * and disable Auto State Control (both).
         * NOTE: NEVER enable the Auto-Thermal Fault disconnect in the silicon
         * because the silicon is too fast for the VP-API-II. It would be possible
         * to get a thermal fault, have the silicon disable the line, and have
         * the thermal fault go away all before the VP-API-II sees it. In that
         * condition, the line will be disabled without the application being
         * aware of it.
         */
        mpiData = VP880_AUTO_SSC_DIS;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT,
            VP880_SS_CONFIG_LEN, &mpiData);

        /* Complete all other non device senstive items */

        /* Initialize default values for Ringing */
        VpMemCpy(pLineObj->ringingParams, defaultRingParams,
            VP880_RINGER_PARAMS_LEN);

        pLineObj->status &= ~(VP880_UNBAL_RINGING);

        if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
            uint8 converterCfg[VP880_CONV_CFG_LEN];

            /* Set the pcm buffer update rate based on the tickrate */
            if(pDevObj->devProfileData.tickRate <=160) {
                converterCfg[0] = VP880_CC_8KHZ_RATE;
                pDevObj->txBufferDataRate = VP880_CC_8KHZ_RATE;

            } else if(pDevObj->devProfileData.tickRate <=320){
                converterCfg[0] = VP880_CC_4KHZ_RATE;
                pDevObj->txBufferDataRate = VP880_CC_4KHZ_RATE;

            } else if(pDevObj->devProfileData.tickRate <=640){
                converterCfg[0] = VP880_CC_2KHZ_RATE;
                pDevObj->txBufferDataRate = VP880_CC_2KHZ_RATE;

            } else if(pDevObj->devProfileData.tickRate <=1280){
                converterCfg[0] = VP880_CC_1KHZ_RATE;
                pDevObj->txBufferDataRate = VP880_CC_1KHZ_RATE;
            } else {
                converterCfg[0] = VP880_CC_500HZ_RATE;
                pDevObj->txBufferDataRate = VP880_CC_500HZ_RATE;
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_WRT,
                VP880_CONV_CFG_LEN, converterCfg);

            pDevObj->devMode[0] |= VP880_DEV_MODE_TEST_DATA;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DEV_MODE_WRT,
                VP880_DEV_MODE_LEN, pDevObj->devMode);
        }

#ifdef VP880_CURRENT_LIMIT
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX] |= VP880_ICR2_SWY_LIM_CTRL;
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] &= ~VP880_ICR2_SWY_LIM_CTRL;
#endif  /* VP880_CURRENT_LIMIT */

#ifdef VP880_TRACKER_SUPPORT
        if (!(pDevObj->stateInt & VP880_IS_ABS)) {
            if (VpIsLowPowerTermType(pLineObj->termType) == FALSE) {
                pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                    (MS_TO_TICKRATE(VP880_SPEEDUP_HOLD_TIME,
                    pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

                /* Initialize the Disconnect Exit Timer State */
                pLineObj->discTimerExitState = 0;

                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Chan %d Setting VP_LINE_DISCONNECT_EXIT time to %d ms (VP880_SPEEDUP_HOLD_TIME) at time %d",
                    pLineObj->channelId, VP880_SPEEDUP_HOLD_TIME, pDevObj->timeStamp));

                pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] |=
                    VP880_ICR1_LINE_BIAS_OVERRIDE;
                pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
                    (VP880_ICR1_LINE_BIAS_OVERRIDE | VP880_ICR1_LINE_BIAS_OVERRIDE_NORM);

                pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
                    VP880_ICR1_LINE_BIAS_OVERRIDE_NORM;
                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR1_WRT, VP880_ICR1_LEN,
                    pLineObj->icr1Values);

                pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] |=
                    (VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE);
                pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] &= ~VP880_ICR2_DAC_SENSE;
                pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] |= VP880_ICR2_FEED_SENSE;
            }

            /* Tracker Workaround:
             *   Enable VRef while in Disconnect. Otherwise the Switcher will
             *    shutdown because it thinks it's not needed.
             */
            pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX] |= VP880_ICR3_VREF_CTRL;
            pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX+1] |= VP880_ICR3_VREF_CTRL;

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR3_WRT, VP880_ICR3_LEN,
                pLineObj->icr3Values);
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Line Init: ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                pLineObj->icr3Values[2], pLineObj->icr3Values[3], channelId));

#ifndef VP880_CURRENT_LIMIT
            /* Eliminate use of 50V clamp for all conditions */
            pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX] |=
                (VP880_ICR2_SWY_LIM_CTRL1 | VP880_ICR2_SWY_LIM_CTRL);
            pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] |= VP880_ICR2_SWY_LIM_CTRL1;
#endif  /* VP880_CURRENT_LIMIT */
            /* Force ICR4 update - make sure it is what we think it is.. */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR4_WRT, VP880_ICR4_LEN, pLineObj->icr4Values);
        }
#endif  /* VP880_TRACKER_SUPPORT */

        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Line Init: ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
            pLineObj->icr2Values[2], pLineObj->icr2Values[3], channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);

        if (pDcOrFxoProfile == VP_PTABLE_NULL) {
            status = Vp880ConfigLine(pLineCtx, pAcProfile, dcDefaultProf, pRingProfile);
        } else {
            status = Vp880ConfigLine(pLineCtx, pAcProfile, pDcOrFxoProfile, pRingProfile);
        }

        if (status != VP_STATUS_SUCCESS) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return status;
        }

        /* Set to Disconnect */
        /*
         * Set to Disconnect. Set current state to Standby in order to force line
         * state function to change to Disconnect. Otherwise, no changes are made
         * since it thinks device is already set correctly.
         */
        pLineObj->lineState.currentState = VP_LINE_STANDBY;
        Vp880SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
        pLineObj->lineState.usrCurrent = VP_LINE_DISCONNECT;
#endif  /* VP880_FXS_SUPPORT */
    }

    status = VpImplementDefaultSettings(VP_NULL, pLineCtx);

#ifdef VP880_FXS_SUPPORT
    Vp880SetRelayState(pLineCtx, VP_RELAY_NORMAL);
#endif  /* VP880_FXS_SUPPORT */

    /* Post the line init complete event if status is succesfull */
    if (status == VP_STATUS_SUCCESS) {
        pLineObj->lineEvents.response |= VP_LINE_EVID_LINE_INIT_CMP;
        pLineObj->status |= VP880_INIT_COMPLETE;
    }

#ifdef VP880_FXO_SUPPORT
    if (pLineObj->status & VP880_IS_FXO) {
        pLineObj->lineTimers.timers.fxoTimer.disconnectDebounce = VP_FXO_DISCONNECT_DEBOUNCE;
    }
#endif  /* VP880_FXO_SUPPORT */

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return status;
} /* Vp880InitLine */

/**
 * Vp880ConfigLine
 *  This function reloads a line of a device with the specified parameters.
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
Vp880ConfigLine(
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
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 ecVal = pLineObj->ecVal;

#ifdef VP880_FXS_SUPPORT
    uint8 channelId = pLineObj->channelId;
    uint8 sysStateConfig[VP880_SS_CONFIG_LEN];
    uint8 ringTypeByte;
    VpProfilePtrType pRingProf = VP_PTABLE_NULL;
#endif

    uint8 profileIndex;

    VpProfileDataType *pMpiData;

    VpProfilePtrType pAcProf = VP_PTABLE_NULL;
    VpProfilePtrType pDcFxoCfgProf = VP_PTABLE_NULL;

    VpDeviceIdType deviceId = pDevObj->deviceId;
#ifdef CSLAC_GAIN_RELATIVE
    uint8 gainCSD[VP880_GR_GAIN_LEN];
#endif

#ifdef VP880_FXO_SUPPORT
    uint8 loopSuperParams;

    /*
     * Default value used if non provided. Note Ringing Detect 17-33Hz at 1/2
     * period due to 2x pulse from ringing detect input.
     */
#ifndef VP880_CLARE_RINGING_DETECT
    uint8 fxoLoopThreshLow[VP880_LOOP_SUP_LEN] = {0x1C, 0xE1, 0x79, 0xEB};
    uint8 fxoLoopThreshHigh[VP880_LOOP_SUP_LEN] = {0x19, 0xF4, 0x79, 0xEB};
#endif

    uint8 fxoLoopThresh[VP880_LOOP_SUP_LEN] = {0x19, 0xF4, 0x38, 0x78};
#endif

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check the legality of the AC profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_AC,
            VP_CSLAC_AC_PROF_TABLE_SIZE, pDevObj->profEntry.acProfEntry,
            pDevObj->devProfileTable.pAcProfileTable, pAcProfile, &pAcProf)) {

        return VP_STATUS_ERR_PROFILE;
    }

    if (pLineObj->status & VP880_IS_FXO) {
#ifdef VP880_FXO_SUPPORT
        /* Check the legality of the FXO profile */
        if (!VpCSLACIsProfileValid(VP_PROFILE_FXO_CONFIG,
                VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE,
                pDevObj->profEntry.fxoConfigProfEntry,
                pDevObj->devProfileTable.pFxoConfigProfileTable,
                pDcOrFxoProfile, &pDcFxoCfgProf)) {

            return VP_STATUS_ERR_PROFILE;
        }
#endif
    } else {
#ifdef VP880_FXS_SUPPORT
        /* Check the legality of the DC profile */
        if (!VpCSLACIsProfileValid(VP_PROFILE_DC, VP_CSLAC_DC_PROF_TABLE_SIZE,
                pDevObj->profEntry.dcProfEntry,
                pDevObj->devProfileTable.pDcProfileTable,
                pDcOrFxoProfile, &pDcFxoCfgProf)) {

            return VP_STATUS_ERR_PROFILE;
        }
#endif
    }

#ifdef VP880_FXS_SUPPORT
    /* Check the legality of the Ringing profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_RING, VP_CSLAC_RINGING_PROF_TABLE_SIZE,
            pDevObj->profEntry.ringingProfEntry,
            pDevObj->devProfileTable.pRingingProfileTable, pRingProfile,
            &pRingProf)) {

        return VP_STATUS_ERR_PROFILE;
    }
#endif

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * Do not modify the line if line is in calibration. This could cause line
     * calibration failure. Write to the update values that will be applied
     * when calibration is complete (for provided profiles).
     */
    if (pLineObj->status & VP880_LINE_IN_CAL) {
        uint8 profileSize = 0;
        if (pAcProf != VP_PTABLE_NULL) {
            /* Error if the provided profile is larger than what we can copy */
            profileSize = pAcProf[VP_PROFILE_LENGTH] + VP_PROFILE_LENGTH + 1;
            if (profileSize > VP880_AC_PROFILE_SIZE) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_DEVICE_BUSY;
            }
            /* Size is ok. Copy profile and set update required flag */
            pLineObj->calLineData.updateFlags |= AC_PROFILE_UPDATE_REQ;
            VpMemCpy(pLineObj->calLineData.acProfile, pAcProf, profileSize);
        }
        if (pDcFxoCfgProf != VP_PTABLE_NULL) {
            /* Error if the provided profile is larger than what we can copy */
            profileSize = pDcFxoCfgProf[VP_PROFILE_LENGTH] + VP_PROFILE_LENGTH + 1;
            if (profileSize > VP880_DC_PROFILE_SIZE) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_DEVICE_BUSY;
            }
            /* Size is ok. Copy profile and set update required flag */
            pLineObj->calLineData.updateFlags |= DC_PROFILE_UPDATE_REQ;
            VpMemCpy(pLineObj->calLineData.dcProfile, pDcFxoCfgProf, profileSize);
        }
#ifdef VP880_FXS_SUPPORT
        if (pRingProf != VP_PTABLE_NULL) {
            /* Error if the provided profile is larger than what we can copy */
            profileSize = pRingProf[VP_PROFILE_LENGTH] + VP_PROFILE_LENGTH + 1;
            if (profileSize > VP880_RINGING_PROFILE_SIZE) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_DEVICE_BUSY;
            }
            /* Size is ok. Copy profile and set update required flag */
            pLineObj->calLineData.updateFlags |= RINGING_PROFILE_UPDATE_REQ;
            VpMemCpy(pLineObj->calLineData.ringingProfile, pRingProf, profileSize);
        }
#endif
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* Load AC Coefficients */
    if (pAcProf != VP_PTABLE_NULL) {
        profileIndex = VP_PROFILE_MPI_LEN + 1;
        pMpiData = (VpProfileDataType *)(&pAcProfile[profileIndex]);
        VpMpiCmdWrapper(deviceId, ecVal, NOOP_CMD,
            pAcProfile[VP_PROFILE_MPI_LEN], pMpiData);

#ifdef CSLAC_GAIN_RELATIVE
        /* Update cached transmit and receive gains for SetRelGain */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_GX_GAIN_RD, VP880_GX_GAIN_LEN, gainCSD);
        pLineObj->gain.gxInt = 0x4000 + VpConvertCsd2Fixed(gainCSD);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_GR_GAIN_RD, VP880_GR_GAIN_LEN, gainCSD);
        pLineObj->gain.grInt = VpConvertCsd2Fixed(gainCSD);
#endif

#ifdef CSLAC_GAIN_ABS
        pLineObj->gain.absGxGain = VP_ABS_GAIN_UNKNOWN;
        pLineObj->gain.absGrGain = VP_ABS_GAIN_UNKNOWN;
#endif
    }

    if (pLineObj->status & VP880_IS_FXO) {
#ifdef VP880_FXO_SUPPORT
        /* Configure an FXO line type */
        if (pDcFxoCfgProf != VP_PTABLE_NULL) {
#ifdef VP880_CLARE_RINGING_DETECT
            uint8 tempRegValue;
#endif  /* VP880_CLARE_RINGING_DETECT */
            /*
             * LIU Threshold may have been provided in Volts or by Register Value. Convert to
             * register value only.
             */
            uint8 liuVoltage = pDcFxoCfgProf[VP_FXO_DIALING_PROFILE_LIU_THRESHOLD_MIN];
            if (liuVoltage > 15) {  /* Only 3-bits in the device, so if larger than 15 it's volts */
                /* Round out to nearest programmable device setting */
                uint8 tempLiuVolt = (liuVoltage - 16);

                /* Steps are in 11V increments, so find the remainder */
                uint8 liuError = (tempLiuVolt % 11);
                if (liuError > 5) {
                    tempLiuVolt += (11 - liuError);
                } else {
                    tempLiuVolt -= liuError;
                }
                /* Update liuVoltage to match Device values */
                liuVoltage = (tempLiuVolt / 11);
            }

#ifndef VP880_CLARE_RINGING_DETECT
            /*
             * Force device Ringing Detector unless the minimum frequency is
             * lower than device can support.
             */
            if (pDcFxoCfgProf[VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX_ACT]
             >= VP880_MAX_RING_DET_PERIOD) {
                /* Cache the "Low Freq" loop supervision register content */
                VpMemCpy(fxoLoopThresh, fxoLoopThreshLow, VP880_LOOP_SUP_LEN);
            } else {
                /* Cache the "High Freq" loop supervision register content */
                VpMemCpy(fxoLoopThresh, fxoLoopThreshHigh, VP880_LOOP_SUP_LEN);
             }

            profileIndex = VP_FXO_DIALING_PROFILE_DISC_VOLTAGE_MIN;
            loopSuperParams = (pDcFxoCfgProf[profileIndex] << 3);
            loopSuperParams &= 0x38;
            fxoLoopThresh[0] &= ~(0x38);
            fxoLoopThresh[0] |= loopSuperParams;

            /*
             * Use the profile parameters for Ringing Detect minimum ONLY if
             * the minimum ringing detect frequency is within range of device.
             * Otherwise, force LIU/Ring Detect to 60V.
             */
            if (pDcFxoCfgProf[VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX_ACT]
             < VP880_MAX_RING_DET_PERIOD) {
                fxoLoopThresh[0] &= ~(VP880_LOOP_SUP_LIU_THRESH_BITS);
                fxoLoopThresh[0] |= liuVoltage;
            }

            fxoLoopThresh[VP880_RING_PERIOD_MIN_INDEX] =
                pDcFxoCfgProf[VP_FXO_DIALING_PROFILE_RING_PERIOD_MIN];

            /* Cache the Minimum Ringing Detect Period that is implemented in SW. */
            pLineObj->ringDetMin = pDcFxoCfgProf[VP_FXO_DIALING_PROFILE_RING_PERIOD_MIN];
            pLineObj->ringDetMin /= 4;

            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX_ACT;
            pLineObj->ringDetMax = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX;
            fxoLoopThresh[3] = pDcFxoCfgProf[profileIndex];
#else   /* VP880_CLARE_RINGING_DETECT */
            profileIndex = VP_FXO_DIALING_PROFILE_DISC_VOLTAGE_MIN;
            loopSuperParams = (pDcFxoCfgProf[profileIndex] << 3);
            loopSuperParams &= 0x38;
            fxoLoopThresh[0] &= ~(0x38);
            fxoLoopThresh[0] |= loopSuperParams;
            fxoLoopThresh[0] |= VP880_RING_DETECT_PERIOD_ONLY;

            profileIndex = VP_FXO_DIALING_PROFILE_LIU_THRESHOLD_MIN;
            fxoLoopThresh[0] &= ~(VP880_LOOP_SUP_LIU_THRESH_BITS);
            fxoLoopThresh[0] |= liuVoltage;

            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MIN;

#ifdef VP880_FXO_FULL_WAVE_RINGING
            tempRegValue = pDcFxoCfgProf[profileIndex] / 2;
#else   /* VP880_FXO_FULL_WAVE_RINGING */
            tempRegValue = pDcFxoCfgProf[profileIndex];
#endif  /* VP880_FXO_FULL_WAVE_RINGING */

            fxoLoopThresh[VP880_RING_PERIOD_MIN_INDEX] = tempRegValue;

            /*
             * Cache the Minimum Ringing Detect Period that is implemented in
             * SW.
             */
            pLineObj->ringDetMin = pDcFxoCfgProf[profileIndex];
            pLineObj->ringDetMin /= 4;

            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX_ACT;
            pLineObj->ringDetMax = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX;

#ifdef VP880_FXO_FULL_WAVE_RINGING
            tempRegValue = pDcFxoCfgProf[profileIndex] / 2;
#else   /* VP880_FXO_FULL_WAVE_RINGING */
            tempRegValue = pDcFxoCfgProf[profileIndex];
#endif  /* VP880_FXO_FULL_WAVE_RINGING */

            fxoLoopThresh[3] = tempRegValue;
#endif  /* VP880_CLARE_RINGING_DETECT */

            if (pLineObj->ringDetMax == 0) {
                pLineObj->ringDetMax = fxoLoopThresh[3] / 4;
            }

#ifdef VP_CSLAC_SEQ_EN
            profileIndex = VP_FXO_DIALING_PROFILE_DTMF_ON_MSB;
            pLineObj->digitGenStruct.dtmfOnTime = (pDcFxoCfgProf[profileIndex] << 8)&0xFF00;

            profileIndex = VP_FXO_DIALING_PROFILE_DTMF_ON_LSB;
            pLineObj->digitGenStruct.dtmfOnTime |=  pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_DTMF_OFF_MSB;
            pLineObj->digitGenStruct.dtmfOffTime = (pDcFxoCfgProf[profileIndex] << 8)&0xFF00;

            profileIndex = VP_FXO_DIALING_PROFILE_DTMF_OFF_LSB;
            pLineObj->digitGenStruct.dtmfOffTime |= pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_PULSE_BREAK;
            pLineObj->digitGenStruct.breakTime = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_PULSE_MAKE;
            pLineObj->digitGenStruct.makeTime = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_FLASH_HOOK_MSB;
            pLineObj->digitGenStruct.flashTime = (pDcFxoCfgProf[profileIndex] << 8)&0xFF00;

            profileIndex = VP_FXO_DIALING_PROFILE_FLASH_HOOK_LSB;
            pLineObj->digitGenStruct.flashTime |= pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_INTERDIGIT_MSB;
            pLineObj->digitGenStruct.dpInterDigitTime = (pDcFxoCfgProf[profileIndex] << 8)&0xFF00;

            profileIndex = VP_FXO_DIALING_PROFILE_INTERDIGIT_LSB;
            pLineObj->digitGenStruct.dpInterDigitTime = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_PROFILE_VERSION;
            if (pDcFxoCfgProf[profileIndex] >= VP_CSLAC_FXO_VERSION_DTMF_LEVEL) {
                profileIndex = VP_FXO_DIALING_PROFILE_DTMF_HIGH_LVL_MSB;
                pLineObj->digitGenStruct.dtmfHighFreqLevel[0] = pDcFxoCfgProf[profileIndex];

                profileIndex = VP_FXO_DIALING_PROFILE_DTMF_HIGH_LVL_LSB;
                pLineObj->digitGenStruct.dtmfHighFreqLevel[1] = pDcFxoCfgProf[profileIndex];

                profileIndex = VP_FXO_DIALING_PROFILE_DTMF_LOW_LVL_MSB;
                pLineObj->digitGenStruct.dtmfLowFreqLevel[0] = pDcFxoCfgProf[profileIndex];

                profileIndex = VP_FXO_DIALING_PROFILE_DTMF_LOW_LVL_LSB;
                pLineObj->digitGenStruct.dtmfLowFreqLevel[1] = pDcFxoCfgProf[profileIndex];
            } else {
                pLineObj->digitGenStruct.dtmfHighFreqLevel[0] = 0x1C;
                pLineObj->digitGenStruct.dtmfHighFreqLevel[1] = 0x32;
                pLineObj->digitGenStruct.dtmfLowFreqLevel[0] = 0x1C;
                pLineObj->digitGenStruct.dtmfLowFreqLevel[1] = 0x32;
            }
#endif  /* VP_CSLAC_SEQ_EN */

            fxoLoopThresh[VP880_LIU_DBNC_INDEX] &= ~VP880_LIU_DBNC_MASK;
            fxoLoopThresh[VP880_LIU_DBNC_INDEX] |=
                ((fxoLoopThresh[VP880_RING_PERIOD_MIN_INDEX] >> 3) & VP880_LIU_DBNC_MASK);

            if (!(fxoLoopThresh[VP880_RING_PERIOD_MIN_INDEX] & VP880_RING_PERIOD_1MS)) {
                if ((fxoLoopThresh[VP880_LIU_DBNC_INDEX] & VP880_LIU_DBNC_MASK) > 0) {
                    fxoLoopThresh[VP880_LIU_DBNC_INDEX]--;
                }
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT, VP880_LOOP_SUP_LEN, fxoLoopThresh);

            /* Cache the loop supervision register content */
            VpMemCpy(pLineObj->loopSup, fxoLoopThresh, VP880_LOOP_SUP_LEN);
        }

        /* Cache this so we don't have to read it all the time */
        VpMemCpy(fxoLoopThresh, pLineObj->loopSup, VP880_LOOP_SUP_LEN);
        pLineObj->lineTimers.timers.fxoTimer.maxPeriod = fxoLoopThresh[3];
#endif  /* VP880_FXO_SUPPORT */
    } else {
#ifdef VP880_FXS_SUPPORT

        /* Configure an FXS line type */

        /* Ringing changed if profile passed */
        if (pRingProf != VP_PTABLE_NULL) {
            uint8 tempRingPr[255];
            int16 biasErr;

            /*
             * Clear flags to indicate generators are not programmed to user specified Ringing
             * values. These are checked when getting ready to set the line to the Ringing States
             * (normal and polrev).
             */
            pLineObj->status &= ~(VP880_RING_GEN_NORM | VP880_RING_GEN_REV);

            /*
             * Ringing Profile May affect the system state register, so read what it is before the
             * profile, and set it back to all values except what can change in the profile.
             */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                VP880_SS_CONFIG_LEN, sysStateConfig);

            profileIndex = VP_PROFILE_MPI_LEN + 1;
            pMpiData = (VpProfileDataType *)&pRingProf[profileIndex];

            VpMemCpy(tempRingPr, pMpiData, pRingProf[VP_PROFILE_MPI_LEN]);

            biasErr = (int16)((((uint16)(tempRingPr[2]) << 8) & 0xFF00) +
                ((uint16)(tempRingPr[3]) & 0x00FF));

            /* Apply the offset calibration to the BIAS */
            if ((pLineObj->slicValueCache & VP880_SS_POLARITY_MASK) == 0x00) {
                /* Normal polarity */
                biasErr -= ((pDevObj->vp880SysCalData.sigGenAError[channelId][0] -
                    pDevObj->vp880SysCalData.vocOffset[channelId][VP880_NORM_POLARITY]) * 16 / 10);
            } else {
                /* Reverse polarity */
                biasErr += ((pDevObj->vp880SysCalData.sigGenAError[channelId][0] -
                    pDevObj->vp880SysCalData.vocOffset[channelId][VP880_REV_POLARITY]) * 16 / 10);
            }
            tempRingPr[2] = (uint8)((biasErr >> 8) & 0x00FF);
            tempRingPr[3] = (uint8)(biasErr & 0x00FF);

            VpMpiCmdWrapper(deviceId, ecVal, NOOP_CMD, pRingProf[VP_PROFILE_MPI_LEN], tempRingPr);
            /*
             * Copy what is currently in the silicon. This will be adjusted in calibration when DC
             * Bias error is determined.
             */
            VpMemCpy(pLineObj->ringingParams, &pRingProf[profileIndex + 1],
                VP880_RINGER_PARAMS_LEN);
            /*
             * Copy to the user defined reference for VP_OPTION_ID_RINGING_PARAMS when the user sets
             * or gets this value.
             */    
            VpMemCpy(pLineObj->ringingParamsRef, pLineObj->ringingParams, VP880_RINGER_PARAMS_LEN);

            ringTypeByte = pRingProf[VP_PROFILE_MPI_LEN +
                pRingProf[VP_PROFILE_MPI_LEN] + VP_PROFILE_RING_TYPE_OFFSET];

            pLineObj->status &= ~(VP880_UNBAL_RINGING);
            pLineObj->status |= (ringTypeByte ? VP880_UNBAL_RINGING : 0x0000);

            /*
             * Nothing in this register should be allowed to change, but the Ringing profile may
             * have changed this value to be compatible with other device profiles. So correct it.
             */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN,
                sysStateConfig);
        }

        /* Set Loop Supervision and DC Feed */
        if (pDcFxoCfgProf != VP_PTABLE_NULL) {
            profileIndex = VP_PROFILE_MPI_LEN + 1;
            pMpiData = (VpProfileDataType *)&pDcFxoCfgProf[profileIndex];

            if (pDevObj->stateInt & VP880_IS_ABS) { /* ABS Workaround */
#ifdef VP880_ABS_SUPPORT
                /* If the VOC set >= theshold, disable longitudinal clamps */
                if ((pDcFxoCfgProf[VP880_VOC_PROFILE_POSITION] & VP880_VOC_MASK) >= VP880_VOC_51V) {
                    pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX] &= ~VP880_ICR3_SAT_LIM_25_CTRL;
                    pLineObj->icr3Values[VP880_ICR3_LONG_UNCLAMP_INDEX] |= VP880_ICR3_LONG_UNCLAMP;
                    pLineObj->icr3Values[VP880_ICR3_LONG_UNCLAMP_INDEX+1] |= VP880_ICR3_LONG_UNCLAMP;
                } else {
                    pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX]
                        |= VP880_ICR3_SAT_LIM_25_CTRL;
                    pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX+1]
                        &= ~VP880_ICR3_SAT_LIM_25_CTRL;
                    pLineObj->icr3Values[VP880_ICR3_LONG_UNCLAMP_INDEX]
                        &= ~VP880_ICR3_LONG_UNCLAMP;
                }
#endif  /* VP880_ABS_SUPPORT */
            } else { /* Tracker Workaround */
#ifdef VP880_TRACKER_SUPPORT
                /* If the VOC set > 48V, disable longitudinal clamps */
                if ((pDcFxoCfgProf[VP880_VOC_PROFILE_POSITION] & VP880_VOC_MASK) > VP880_VOC_48V) {
                    /* Disable longitudinal clamps */
                    pLineObj->icr3Values[VP880_ICR3_LONG_UNCLAMP_INDEX] |= VP880_ICR3_LONG_UNCLAMP;
                    pLineObj->icr3Values[VP880_ICR3_LONG_UNCLAMP_INDEX+1] |= VP880_ICR3_LONG_UNCLAMP;

                    /* Remove Workaround from other conditions */
                    pLineObj->icr3Values[VP880_ICR3_LONG_FIXED_INDEX] &= ~VP880_ICR3_LONG_FIXED;
                } else {
                    /* If the VOC <= 48V, enable longitudinal clamps */
                    pLineObj->icr3Values[VP880_ICR3_LONG_FIXED_INDEX] |= VP880_ICR3_LONG_FIXED;
                    pLineObj->icr3Values[VP880_ICR3_LONG_FIXED_INDEX+1] &= ~VP880_ICR3_LONG_FIXED;

                    /* Remove Workaround from other conditions */
                    pLineObj->icr3Values[VP880_ICR3_LONG_UNCLAMP_INDEX] &= ~VP880_ICR3_LONG_UNCLAMP;
                }
#endif  /* VP880_TRACKER_SUPPORT */
            }

            if (pDcFxoCfgProf[VP_PROFILE_VERSION] >= 1) {
                /* This profile contains a hook hysteresis value */
                pLineObj->hookHysteresis = pDcFxoCfgProf[VP880_HOOK_HYST_POSITION];
            } else {
                pLineObj->hookHysteresis = 0;
            }
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                          ("Config Line: Channel %d: Writing ICR2 0x%02X 0x%02X 0x%02X 0x%02X",
                           channelId, pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                           pLineObj->icr2Values[2], pLineObj->icr2Values[3]));

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                          ("ICR3 Workaround: 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                           pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                           pLineObj->icr3Values[2], pLineObj->icr3Values[3], channelId));

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR3_WRT, VP880_ICR3_LEN, pLineObj->icr3Values);

            /* Cache the loop supervision register content. This is needed for hysteris updates */
            VpMemCpy(pLineObj->loopSup, &pMpiData[1], VP880_LOOP_SUP_LEN);

            /* This cache is needed for VP_OPTION_ID_DCFEED VpGetOption(). It should NEVER be modified outside
             * of this function except for VpSetOption() DC_FEED_PARAMS since it's the USER REFERENCE. */
            VpMemCpy(pLineObj->calLineData.loopSup, pLineObj->loopSup, VP880_LOOP_SUP_LEN);

            /* Copy Feed for Calibration reference */
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Copying DC Feed Reference in VpConfigLine()"));
            VpMemCpy(pLineObj->calLineData.dcFeedRef, &pMpiData[6], VP880_DC_FEED_LEN);

            /* If off-hook -> apply the hysteresis */
            if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK)
                == VP_CSLAC_HOOK) {
                if ((pLineObj->loopSup[0] & VP880_LOOP_SUP_LIU_THRESH_BITS) >= pLineObj->hookHysteresis) {
                    pLineObj->loopSup[0] -= pLineObj->hookHysteresis;
                } else {
                    pLineObj->loopSup[0] &= ~VP880_LOOP_SUP_LIU_THRESH_BITS;
                }
            }

            /* Write the Profile Data */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT, VP880_LOOP_SUP_LEN,
                pLineObj->loopSup);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_WRT, VP880_DC_FEED_LEN,
                pLineObj->calLineData.dcFeedRef);

            if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] >= VP880_REV_JE) {
                uint8 icr5Values[VP880_ICR5_LEN];

                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR5_RD, VP880_ICR5_LEN, icr5Values);

                icr5Values[VP880_ICR5_FEED_HOLD_INDEX] &= ~VP880_ICR5_FEED_HOLD_MASK;

                /* Device value is x + 18mA, so threshold is > 35mA */
                if ((pDcFxoCfgProf[VP880_ILA_PROFILE_POSITION] & VP880_ILA_MASK) > 17) {
                    icr5Values[VP880_ICR5_FEED_HOLD_INDEX] |= 0xF0;
                } else {
                    icr5Values[VP880_ICR5_FEED_HOLD_INDEX] |= 0xA0;
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR5_WRT, VP880_ICR5_LEN, icr5Values);
            }

            /* Update the line and device objects (dc feed register) for calibrated values. */
            Vp880UpdateCalValue(pLineCtx);
        }   /* if (pDcFxoCfgProf != VP_PTABLE_NULL) */
#endif  /* VP880_FXS_SUPPORT */
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}   /* Vp880ConfigLine() */

/*
 * Vp880UpdateCalValue()
 *  This function loads the device with calibration values provided by VpCal()
 * "Apply System Coefficient" process.
 *
 * Preconditions:
 *  System calibration values provided or calibration previously run.
 *
 * Postconditions:
 *  The device is loaded per the applied calibration values.
 */
bool
Vp880UpdateCalValue(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;

    bool calStatus = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880UpdateCalValue+"));

    if ((pLineObj->calLineData.dcFeedRef[0] == 0x00) || (pLineObj->status & VP880_IS_FXO)) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880UpdateCalValue-"));
        return calStatus;
    }

    /*
     * The DC Feed Reference represents the last provided profile. It is updated in Vp88ConfigLine()
     * when a DC Profile is provided. Make sure we start with a fresh copy of THAT profile to work
     * with. NOTE: VAS must be set to MAX if this line has not been calibrated.
     */
    VpMemCpy(pLineObj->calLineData.dcFeed, pLineObj->calLineData.dcFeedRef, VP880_DC_FEED_LEN);
    VpMemCpy(pLineObj->calLineData.dcFeedPr, pLineObj->calLineData.dcFeedRef, VP880_DC_FEED_LEN);

    /*
     * Adjust for the errors if previously calibrated. The ILA function
     * returns "TRUE" if an adjustment was made meaning calibration
     * done previously, FALSE if not.
     */
    if (Vp880AdjustIla(pLineCtx, (pLineObj->calLineData.dcFeedRef[1] & VP880_ILA_MASK)) == TRUE) {
        calStatus = TRUE;
        Vp880AdjustVoc(pLineCtx, ((pLineObj->calLineData.dcFeedRef[0] >> 2) & 0x7), TRUE);

        if (!(pDevObj->stateInt & VP880_IS_ABS)) { /* Tracker only */
#ifdef VP880_TRACKER_SUPPORT
            uint16 vasVoltScale;
            uint8 channelId = pLineObj->channelId;
            uint16 vasNorm = (uint16)pDevObj->vp880SysCalData.vas[channelId][VP880_NORM_POLARITY];
            uint16 vasRev = (uint16)pDevObj->vp880SysCalData.vas[channelId][VP880_REV_POLARITY];

            /* Set VAS to device calibrated values */
            vasVoltScale = (VP880_VAS_START + vasNorm * VP880_VAS_STEP);
            VpCSLACSetVas(pLineObj->calLineData.dcFeed, vasVoltScale);

            vasVoltScale = (VP880_VAS_START + vasRev * VP880_VAS_STEP);
            VpCSLACSetVas(pLineObj->calLineData.dcFeedPr, vasVoltScale);
            /*
             * Correct the battery for this line.
             * Note that the function Vp880BatteryCalAdjust() takes care of the Wideband bit when
             * sending the EC Value passed to the silicon. So it isn't necessary to provide it here,
             * but it also doesn't hurt.
             */
            Vp880BatteryCalAdjust(pDevObj, ecVal);
#endif  /* VP880_TRACKER_SUPPORT */
        } else {
#ifdef VP880_ABS_SUPPORT
            VpLineStateType lineState;
            int16 targetVoltY, targetVoltZ;

            /*
             * Get the line state as seen from the user's perspective. This avoids the function
             * call and debug output caused by calling the external GetLineState function
             */
            if (pLineObj->status & VP880_LINE_IN_CAL) {
                lineState = pLineObj->calLineData.usrState;
            } else {
                lineState = pLineObj->lineState.usrCurrent;
            }
            Vp880GetLineStateABS(pLineCtx, lineState, FALSE);

            /* Compute Errors and make corrections */
            targetVoltY = (pDevObj->swParams[VP880_SWY_LOCATION] & VP880_VOLTAGE_MASK);
            targetVoltZ = (pDevObj->swParams[VP880_SWZ_LOCATION] & VP880_VOLTAGE_MASK);

            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("ABS: Channel %d in State %d TargetY %d TargetZ %d",
                pLineObj->channelId, lineState, targetVoltY, targetVoltZ));

            Vp880AbvMakeAdjustment(pDevObj, &targetVoltY, &targetVoltZ);
#endif  /* VP880_ABS_SUPPORT */
        }
    } else {    /* Line has NOT been Calibrated */
#ifdef VP880_TRACKER_SUPPORT
        /*
         * If calibration has not been done on this line, then VAS must be set to a "safe" value.
         * But with no knowledge of the component characteristics, the only safe value is max. VAS
         * will be reduced if calibration is done later. It will also be reduced if Calibration
         * Coefficients are provided using VpCal() with "Apply System" function, in which case
         * the VAS will come from the device system calibration data object.
         */
        if (!(pDevObj->stateInt & VP880_IS_ABS)) { /* Tracker only */
            pLineObj->calLineData.dcFeed[VP880_VAS_MSB_LOC] |= VP880_VAS_MSB_MASK;
            pLineObj->calLineData.dcFeed[VP880_VAS_LSB_LOC] |= VP880_VAS_LSB_MASK;
            pLineObj->calLineData.dcFeedPr[VP880_VAS_MSB_LOC] |= VP880_VAS_MSB_MASK;
            pLineObj->calLineData.dcFeedPr[VP880_VAS_LSB_LOC] |= VP880_VAS_LSB_MASK;
        }
#endif  /* VP880_TRACKER_SUPPORT */
    }
    if (pLineObj->slicValueCache & VP880_SS_POLARITY_MASK) {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_WRT, VP880_DC_FEED_LEN,
            pLineObj->calLineData.dcFeedPr);
    } else {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_WRT, VP880_DC_FEED_LEN,
            pLineObj->calLineData.dcFeed);
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
                   ("Vp880UpdateCalValue() Chan %d: DC Feed Values Normal 0x%02X 0x%02X",
                    pLineObj->channelId,
                    pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1]));

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
                   ("Vp880UpdateCalValue() Chan %d: DC Feed Values Reverse 0x%02X 0x%02X",
                    pLineObj->channelId,
                    pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1]));

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880UpdateCalValue-"));
    return calStatus;
}   /* Vp880UpdateCalValue() */

/**
 * Vp880InitProfile()
 *  This function is used to initialize profile tables in Vp880.
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  Stores the given profile at the specified index of the profile table.
 */
VpStatusType
Vp880InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

    uint8 profIndex8;   /* Used for 8-bit profile table masking */
    uint16 profIndex16; /* Used for 16-bit profile table masking */

    /*
     * If the profile data is an index, indicated by Get Profile Index return
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

    profIndex8 = (uint8)profileIndex;
    profIndex16 = (uint16)profileIndex;

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    /*
     * The correct types are passed, but check to make sure the specific profile type being
     * initialized is valid as well as the index value
     */
    switch(type) {
        case VP_PROFILE_DEVICE:
            if (profIndex8 >= VP_CSLAC_DEV_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_DEVICE, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pDevProfileTable[profIndex8] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.devProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.devProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_AC:
            if (profIndex8 >= VP_CSLAC_AC_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_AC, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pAcProfileTable[profIndex8] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.acProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.acProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_DC:
            if (profIndex8 >= VP_CSLAC_DC_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_DC, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pDcProfileTable[profIndex8] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.dcProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.dcProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_RING:
            if (profIndex8 >= VP_CSLAC_RINGING_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_RING, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pRingingProfileTable[profIndex8] =  pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.ringingProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.ringingProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_RINGCAD:
            if (profIndex8 >= VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_RINGCAD, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pRingingCadProfileTable[profIndex8] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.ringCadProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.ringCadProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_TONE:
            if (profIndex16 >= VP_CSLAC_TONE_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_TONE, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pToneProfileTable[profIndex16] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.toneProfEntry &= ~(0x01 << profIndex16);
                    } else {
                        pDevObj->profEntry.toneProfEntry |= (0x01 << profIndex16);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_TONECAD:
            if (profIndex16 >= VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_TONECAD, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pToneCadProfileTable[profIndex16] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.toneCadProfEntry &= ~(0x01 << profIndex16);
                    } else {
                        pDevObj->profEntry.toneCadProfEntry |= (0x01 << profIndex16);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_METER:
            if (profIndex8 >= VP_CSLAC_METERING_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_METER, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pMeteringProfileTable[profIndex8] =  pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.meterProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.meterProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_CID:
            if (profIndex8 >= VP_CSLAC_CALLERID_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_CID, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pCallerIdProfileTable[profIndex8] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.cidCadProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.cidCadProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        case VP_PROFILE_FXO_CONFIG:
            if (profIndex8 >= VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_FXO_CONFIG, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pFxoConfigProfileTable[profIndex8] = pProfile;
                    /*
                     * If the profile is null, then clear the flag in the profile entry table to
                     * indicate that this profile is no longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.fxoConfigProfEntry &= ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.fxoConfigProfEntry |= (0x01 << profIndex8);
                    }
                } else {
                    status = VP_STATUS_ERR_PROFILE;
                }
            }
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return status;
} /* Vp880InitProfile() */

#ifdef VP880_FXS_SUPPORT
/**
 * Vp880FreeRun()
 *  This function is called by the application when it wants to prepare the
 * system for a restart, or by the VP-API-II internally when a clock fault or
 * other "sign" of a restart is detected.
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  Device and line are in states 'ready" for a system reboot to occur. Lines
 * are set to VP_LINE_STANDBY if previously ringing.
 */
VpStatusType
Vp880FreeRun(
    VpDevCtxType *pDevCtx,
    VpFreeRunModeType freeRunMode)
{
    VpLineCtxType *pLineCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 maxChan = pDevObj->staticInfo.maxChannels;

    uint8 ecVal = pDevObj->ecVal;
    VpLineStateType lineState;
    uint8 powerMode[VP880_REGULATOR_CTRL_LEN];

    uint8 channelId;

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp880FreeRun Mode %d", freeRunMode));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * Time value is passed in 500us increment. If timeOut = 0, only PCLK
     * recovery exits restart prepare operations. If less than one tick, force
     * a one tick timeout.
     */
    if (freeRunMode == VP_FREE_RUN_STOP) {
        Vp880RestartComplete(pDevCtx);
        /*
         * Clear the device as being forced into free run mode by application.
         * This allows PCLK fault detection to automatically enter/exit free
         * run mode.
         */
        pDevObj->stateInt &= ~VP880_FORCE_FREE_RUN;

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* Take the lines out of Ringing if necessary */
    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

            if (pLineObj->status & VP880_LINE_IN_CAL) {
                lineState = pLineObj->calLineData.usrState;
            } else {
                lineState = pLineObj->lineState.usrCurrent;
            }
            if (lineState == VP_LINE_RINGING) {
                Vp880SetLineState(pLineCtx, VP_LINE_STANDBY);
            }
            if (lineState == VP_LINE_RINGING_POLREV) {
                Vp880SetLineState(pLineCtx, VP_LINE_STANDBY_POLREV);
            }
        }
    }

    /*
     * Load the free run timing, if available. Otherwise just force the switcher
     * to HP mode and take control.
     */
    if (pDevObj->intSwParamsFR[0] != 0x00) {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
            VP880_INT_SWREG_PARAM_LEN, pDevObj->intSwParamsFR);
    } else {
        /* Force control of the power mode */
        pDevObj->swParamsCache[VP880_SWY_AUTOPOWER_INDEX] |= VP880_SWY_AUTOPOWER_DIS;
        pDevObj->swParamsCache[VP880_SWZ_AUTOPOWER_INDEX] |= VP880_SWZ_AUTOPOWER_DIS;

        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_WRT,
            VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);

        /* Change the Switchers to High Power Mode */
        powerMode[0] = VP880_SWY_HP | VP880_SWZ_HP;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
            VP880_REGULATOR_CTRL_LEN, powerMode);
    }

    /*
     * Mark the device as being forced into free run mode by application. This
     * prevents auto-recovery when PCLK is restored.
     */
    pDevObj->stateInt |= VP880_FORCE_FREE_RUN;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}   /* Vp880FreeRun() */

/**
 * Vp880RestartComplete()
 *  This function is called by the VP-API-II internally when a clock fault is
 * removed.
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  Device and line are in states recovered from a reboot.
 */
void
Vp880RestartComplete(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pDevObj->ecVal;
    uint8 powerMode[VP880_REGULATOR_CTRL_LEN];

    /*
     * Restore original timing if they were changed, otherwise change back the
     * power mode and relinquish control.
     */
    if (pDevObj->intSwParamsFR[0] != 0x00) {
        /* Restore the original timings */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
            VP880_INT_SWREG_PARAM_LEN, pDevObj->intSwParams);
    } else {
        /* Change the Switchers to the original Power Mode */
        if (pDevObj->stateInt & VP880_IS_ABS) {
#ifdef VP880_ABS_SUPPORT
            uint8 systemConfig = (pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK);

            if (systemConfig == VP880_ABS_CFG_SLAVE) {
                powerMode[0] = VP880_SWY_LP | VP880_SWY_LP;
            } else if (systemConfig == VP880_ABS_CFG_SINGLE) {
                powerMode[0] = VP880_SWY_MP | VP880_SWY_MP;
            } else {    /* systemConfig == VP880_ABS_CFG_MASTER */
                powerMode[0] = VP880_SWY_HP | VP880_SWY_HP;
            }
#endif  /* VP880_ABS_SUPPORT */
        } else {
#ifdef VP880_TRACKER_SUPPORT
            powerMode[0] = VP880_SWY_LP | VP880_SWZ_LP;
#endif  /* VP880_TRACKER_SUPPORT */
        }
        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
                        VP880_REGULATOR_CTRL_LEN, powerMode);

        /* Relinquish control of the power mode */
        pDevObj->swParamsCache[VP880_SWY_AUTOPOWER_INDEX] &= ~VP880_SWY_AUTOPOWER_DIS;
        pDevObj->swParamsCache[VP880_SWZ_AUTOPOWER_INDEX] &= ~VP880_SWZ_AUTOPOWER_DIS;

        VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_WRT,
            VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
    }
}   /* Vp880RestartComplete() */

/**
 * Vp880CopyDefaultFRProfile()
 *  This function is used to copy the default Free Run switcher timing parameters
 * to the device object used in Free Run mode. This is needed when values are
 * not provided in the device profile.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  The device object is updated with the default free run switcher timing values.
 */
static void
Vp880CopyDefaultFRProfile(
    Vp880DeviceObjectType *pDevObj)
{
    if (pDevObj->stateInt & VP880_IS_ABS) {
#ifdef VP880_ABS_SUPPORT
        /* ABS free run default profile */
        uint8 intSwParamsFR_ABS[VP880_INT_SWREG_PARAM_LEN] = {
            0x2C, 0x40, 0x2C, 0x40, 0x2C, 0x40
        };

        VpMemCpy(pDevObj->intSwParamsFR, intSwParamsFR_ABS, VP880_INT_SWREG_PARAM_LEN);
#endif  /* VP880_ABS_SUPPORT */
    } else {
#ifdef VP880_TRACKER_SUPPORT
        /*
         * The 1st timing value can't be 0x00, so it's a marker value -> no profile
         * available
         */
        uint8 intSwParamsFR_DFLT[VP880_INT_SWREG_PARAM_LEN] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

        VpMemCpy(pDevObj->intSwParamsFR, intSwParamsFR_DFLT, VP880_INT_SWREG_PARAM_LEN);
#endif  /* VP880_TRACKER_SUPPORT */
    }
}   /* Vp880CopyDefaultFRProfile() */
#endif  /*  VP880_FXS_SUPPORT */


#ifdef VP880_INCLUDE_MPI_QUICK_TEST
/** Vp880QuickMpiTest()
  Performs some basic tests of the MPI and HAL implementation to make sure that
  we can properly communicate with the device before starting.
*/
VpStatusType
Vp880QuickMpiTest(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 writeBuffer[255];
    uint8 readBuffer[16];
    uint8 i;

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
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_XR_CS_WRT, VP880_XR_CS_LEN, &writeBuffer[1]);
    
    /* Make sure the write buffer was not corrupted */
    if (writeBuffer[0] != 0xD6 || writeBuffer[1] != 0xE5 || writeBuffer[2] != 0xD6) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 1 failed. Write buffer corrupted: 0x%02X 0x%02X 0x%02X",
            writeBuffer[0], writeBuffer[1], writeBuffer[2]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Read back the register */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_XR_CS_RD, VP880_XR_CS_LEN, &readBuffer[1]);
    
    /* Make sure there was no overflow into the data around byte 1 */
    if (readBuffer[0] != 0x94 || readBuffer[2] != 0x94) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 1 failed. Read buffer overflow: 0x%02X 0x%02X 0x%02X",
            readBuffer[0], readBuffer[1], readBuffer[2]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* The high bit of this register always returns 0, so what we read should
       not be exactly what we wrote.  0xE5 becomes 0x65 */
    if (readBuffer[1] != 0x65) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 1 failed. Read value incorrect: 0x%02X, expected 0x65",
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
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CADENCE_TIMER_WRT, VP880_CADENCE_TIMER_LEN, &writeBuffer[1]);
    
    /* Write to the cadence register for channel 1 */
    writeBuffer[6] = 0x12;
    writeBuffer[7] = 0x34;
    writeBuffer[8] = 0x56;
    writeBuffer[9] = 0x78;
    VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CADENCE_TIMER_WRT, VP880_CADENCE_TIMER_LEN, &writeBuffer[6]);
    
    /* Make sure the write buffer was not corrupted */
    if (writeBuffer[0] != 0xD6 || writeBuffer[1] != 0xAB || writeBuffer[2] != 0xCD ||
        writeBuffer[3] != 0xEF || writeBuffer[4] != 0x01 || writeBuffer[5] != 0xD6 ||
        writeBuffer[6] != 0x12 || writeBuffer[7] != 0x34 || writeBuffer[8] != 0x56 ||
        writeBuffer[9] != 0x78 || writeBuffer[10] != 0xD6)
    {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 2 failed. Write buffer corrupted: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            writeBuffer[0], writeBuffer[1], writeBuffer[2], writeBuffer[3], writeBuffer[4], writeBuffer[5],
            writeBuffer[6], writeBuffer[7], writeBuffer[8], writeBuffer[9], writeBuffer[10]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Read back both registers */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN, &readBuffer[1]);
    VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN, &readBuffer[6]);

    /* Make sure there was no overflow into the surrounding data */
    if (readBuffer[0] != 0x94 || readBuffer[5] != 0x94 || readBuffer[10] != 0x94) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 2 failed. Read buffer overflow: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4], readBuffer[5],
            readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9], readBuffer[10]));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Make sure the ch0 data read back correctly.
       Some bits of the register will always read as 0, so we should read
       0x03CD0701 instead of 0xABCDEF01 */
    if (readBuffer[1] != 0x03 || readBuffer[2] != 0xCD || readBuffer[3] != 0x07 || readBuffer[4] != 0x01) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 2 failed. Ch0 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: expected 0x03 0xCD 0x07 0x01"));
        status = VP_STATUS_ERR_SPI;
    }
    
    /* Make sure the ch1 data read back correctly.
       Some bits of the register will always read as 0, so we should read
       0x02340678 instead of 0x12345678 */
    if (readBuffer[6] != 0x02 || readBuffer[7] != 0x34 || readBuffer[8] != 0x06 || readBuffer[9] != 0x78) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 2 failed. Ch1 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: expected 0x02 0x34 0x06 0x78"));
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
       cadence register, followed by repeated writes to the ch1 GR register. */

    /* The first data is written to ch0.  The EC and command are built in to the
       VpMpiCmdWrapper() function call later */
    writeBuffer[1] = 0xAA;
    writeBuffer[2] = 0x55;
    writeBuffer[3] = 0x43;
    writeBuffer[4] = 0x21;
    /* Switch to the second channel */
    writeBuffer[5] = VP880_EC_WRT;
    writeBuffer[6] = VP880_EC_CH2;
    /* Fill up the middle of the buffer with writes to the ch1 cadence register.
       These should be overwritten by the final values. */
    i = 7;
    while (i < 240) {
        writeBuffer[i] = VP880_CADENCE_TIMER_WRT;
        writeBuffer[i+1] = i;
        writeBuffer[i+2] = i;
        writeBuffer[i+3] = i;
        writeBuffer[i+4] = i;
        i += 5;
    }
    /* Final write to the ch1 register */
    writeBuffer[i++] = VP880_CADENCE_TIMER_WRT;
    writeBuffer[i++] = 0x99;
    writeBuffer[i++] = 0xBB;
    writeBuffer[i++] = 0x56;
    writeBuffer[i] = 0x78;
    /* Send the whole buffer */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CADENCE_TIMER_WRT, i, &writeBuffer[1]);
    
    /* Read back both register values */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN, &readBuffer[1]);
    VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN, &readBuffer[6]);

    /* Make sure that both the first and last writes of the large buffer read
       back correctly.
       Some bits of the register read back as 0, so AA554321 becomes 02550321
       and 99BB5678 becomes 01BB0678 */
    if (readBuffer[1] != 0x02 || readBuffer[2] != 0x55 || readBuffer[3] != 0x03 || readBuffer[4] != 0x21) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 3 failed. Ch0 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: expected 0x02 0x55 0x03 0x21"));
        status =  VP_STATUS_ERR_SPI;
    }
    if (readBuffer[6] != 0x01 || readBuffer[7] != 0xBB || readBuffer[8] != 0x06 || readBuffer[9] != 0x78) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: Test 3 failed. Ch1 read value incorrect: 0x%02X 0x%02X 0x%02X 0x%02X,",
            readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9]));
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880QuickMpiTest: expected 0x01 0xBB 0x06 0x78"));
        status =  VP_STATUS_ERR_SPI;
    }

    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Add new tests here if needed */

    return status;
}
#endif /* VP880_INCLUDE_MPI_QUICK_TEST */

#endif

