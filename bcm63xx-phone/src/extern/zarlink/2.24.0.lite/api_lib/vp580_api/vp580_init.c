/** \file vp580_init.c
 * vp580_init.c
 *
 * This file contains the line and device initialization functions for
 * the Vp580 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 10849 $
 * $LastChangedDate: 2013-03-05 10:28:43 -0600 (Tue, 05 Mar 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_580_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp580_api.h"
#include "vp580_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"

/**< Vp580 Initalization Function Prototypes */
static VpStatusType
Vp580Init(
    VpDevCtxType *pDevCtx);

static VpStatusType
Vp580InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile);

static VpStatusType
Vp580InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp580ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

static VpStatusType
Vp580InitCustomTerm (
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCustomTermProfile);

static VpStatusType
Vp580InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile);

static void
Vp580InitDeviceObject(
    Vp580DeviceObjectType *pDevObj);

static void
Vp580InitLineObject(
    Vp580LineObjectType *pLineObj);

typedef enum vp580_deviceProfileParams {
    VP580_DEV_PROFILE_CHIP_CFG = 6,
    VP580_DEV_PROFILE_PCLK_MSB = 7,
    VP580_DEV_PROFILE_PCLK_LSB = 8,
    VP580_DEV_PROFILE_CLOCK_SLOT = 9,
    VP580_DEV_PROFILE_E1_MUX_MODE = 10,
    VP580_DEV_PROFILE_TICKRATE_MSB = 11,
    VP580_DEV_PROFILE_TICKRATE_LSB = 12,
    VP580_DEV_PROFILE_MAX_EVENTS = 13,
    VP580_DEV_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} vp580_deviceProfileParams;

/**
 * VpMakeVp580DeviceObject()
 *  This function performs the main tasks of VpMakeDeviceObject() for Vp580 type
 * of devices.
 *
 * Preconditions:
 *  Same as VpMakeDeviceObject(), and in addition the deviceType pointed to by
 * pDevCtx should be Vp580 series type.
 *
 * Postconditions:
 *  VpAPI Function pointers for pDevCtx are initialized to Vp580 specific
 * functions.  This completes the function abstraction for "this" device.
 */
VpStatusType
VpMakeVp580DeviceObject(
    VpDevCtxType *pDevCtx,  /**< Device context to be initialized with function
                             * pointers
                             */
    Vp580DeviceObjectType *pDevObj) /**< Device object containing information
                                     * for the device pointed to by pDevCtx
                                     */
{
    Vp580InitDeviceObject(pDevObj);

    /* Initialize other elements in the device object */
    return VpMakeVp580DeviceCtx(pDevCtx, pDevObj);
}

/**
 * Vp580InitDeviceObject()
 *  This function initializes the Vp580 Device object data structure. It is
 * called only in this file .
 */
static void
Vp580InitDeviceObject(
    Vp580DeviceObjectType *pDevObj)
{
    VpMemSet(pDevObj, 0, sizeof(Vp580DeviceObjectType));
    pDevObj->staticInfo.maxChannels = 4;

#ifdef VP_DEBUG
    pDevObj->debugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;
#endif
}

/**
 * VpMakeVp580DeviceCtx()
 *  This function initializes the device context to handle Vp580 functionality.
 *
 * Preconditions:
 *  This function should be called after initializing the device object. This
 * function can be called more than once since it does not modify the contents
 * of the device object.
 *
 * Postconditions:
 *  Initializes device context to be able to handle Vp580 functionality.
 */
VpStatusType
VpMakeVp580DeviceCtx(
    VpDevCtxType *pDevCtx,          /**< Device Context to be initialized */
    Vp580DeviceObjectType *pDevObj) /**< Device Object that has been already
                                     * initialized
                                     */
{
    uint8 channelCount, maxChan;

    if((pDevCtx == VP_NULL) || (pDevObj == VP_NULL)) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Initialize Device context */
    pDevCtx->pDevObj = pDevObj;
    pDevCtx->deviceType = VP_DEV_580_SERIES;

    /*
     * Initialize all of the line context pointers to null in the device context
     */
    maxChan = pDevObj->staticInfo.maxChannels;
    for (channelCount = 0; channelCount < maxChan; channelCount++) {
        pDevCtx->pLineCtx[channelCount] = VP_NULL;
    }

    /* Functions in apiInit.c */
    pDevCtx->funPtrsToApiFuncs.MakeLineObject = Vp580MakeLineObject;
    pDevCtx->funPtrsToApiFuncs.InitDevice = Vp580InitDevice;
    pDevCtx->funPtrsToApiFuncs.InitLine = Vp580InitLine;
    pDevCtx->funPtrsToApiFuncs.ConfigLine = Vp580ConfigLine;

#ifdef VP_CSLAC_SEQ_EN
    pDevCtx->funPtrsToApiFuncs.InitRing = Vp580InitRing;
#endif

    pDevCtx->funPtrsToApiFuncs.InitCustomTerm = Vp580InitCustomTerm;
    pDevCtx->funPtrsToApiFuncs.InitProfile = Vp580InitProfile;
#if !defined(VP_REDUCED_API_IF)
    pDevCtx->funPtrsToApiFuncs.ClearResults = VpCSLACClearResults;
#endif

    /* Functions in apicnt.c */
    pDevCtx->funPtrsToApiFuncs.SetLineState = Vp580SetLineState;

#ifdef VP_CSLAC_SEQ_EN
    pDevCtx->funPtrsToApiFuncs.SendSignal = Vp580SendSignal;
#endif

    pDevCtx->funPtrsToApiFuncs.SetOption = Vp580SetOption;
#ifndef VP580_SIMPLE_POLLED_MODE
    pDevCtx->funPtrsToApiFuncs.VirtualISR = Vp580VirtualISR;
#endif
    pDevCtx->funPtrsToApiFuncs.ApiTick = Vp580ApiTick;
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    pDevCtx->funPtrsToApiFuncs.LowLevelCmd = Vp580LowLevelCmd;
#endif
    pDevCtx->funPtrsToApiFuncs.DeviceIoAccess = Vp580DeviceIoAccess;
    pDevCtx->funPtrsToApiFuncs.SetRelGain = Vp580SetRelGain;

    /* Functions in apiQuery.c */
    pDevCtx->funPtrsToApiFuncs.GetEvent = Vp580GetEvent;
    pDevCtx->funPtrsToApiFuncs.GetLineStatus = VpCSLACGetLineStatus;
    pDevCtx->funPtrsToApiFuncs.GetDeviceStatus = Vp580GetDeviceStatus;
    pDevCtx->funPtrsToApiFuncs.FlushEvents = Vp580FlushEvents;
    pDevCtx->funPtrsToApiFuncs.GetResults = Vp580GetResults;
    pDevCtx->funPtrsToApiFuncs.GetOption = Vp580GetOption;

    return VP_STATUS_SUCCESS;
}

/**
 * VpMakeVp580LineObject()
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
Vp580MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pVoidLineObj,
    VpDevCtxType *pDevCtx)
{
    Vp580LineObjectType *pLineObj = pVoidLineObj;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 lineStateIndex;

    if (channelId >= pDevObj->staticInfo.maxChannels) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    Vp580InitLineObject(pLineObj);

    switch (termType) {
        case VP_TERM_FXO_GENERIC:
            pLineObj->status |= VP580_IS_FXO;
            break;

        case VP_TERM_FXS_CUSTOM:
            /* Only FXS Customer supported */
            pLineObj->status = VP580_INIT_STATUS;
            pLineObj->lineStateInit = FALSE;
            pLineObj->lineStateExist = 0;
            for (lineStateIndex = 0; lineStateIndex< VP580_MAX_NUM_STATES;
                lineStateIndex++) {
                pLineObj->lineStateBytes[lineStateIndex] = 0x00;
            }
            break;

        default:
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_VTD_CODE;
    }

    pLineCtx->pLineObj = pLineObj;
    pLineCtx->pDevCtx = pDevCtx;

    pDevCtx->pLineCtx[channelId] = pLineCtx;
    pLineObj->channelId = channelId;
    pLineObj->termType = termType;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Everything else done by device/line specific functions */
    return VP_STATUS_SUCCESS;
}

/**
 * Vp580InitLineObject()
 *  This function initializes the Vp580 Line Object data structure. It is
 * called only in this file .
 */
static void
Vp580InitLineObject(
    Vp580LineObjectType *pLineObj)
{
    VpMemSet(pLineObj, 0, sizeof(Vp580LineObjectType));

#ifdef VP_DEBUG
    pLineObj->debugSelectMask = VP_OPTION_DEFAULT_DEBUG_SELECT;
#endif
}

/**
 * Vp580Init
 *  This function initializes the device, and (contrary to InitDevice) does
 * not initialize any channels. This function should be called internal to the
 * API only.
 *
 * Preconditions:
 *  The device context must be of a Vp580 device type.
 *
 * Postconditions:
 *  This function returns a failure code if the clock configuration is not set
 * correctly based on the device data set in InitDevice.
 */
VpStatusType
Vp580Init(
    VpDevCtxType *pDevCtx)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 data;
    uint8 clkNotStable;
    uint8 clkTestCount;
    uint8 clkFailReg[VP580_IODIR_REG_LEN];

    uint8 mpiReset[] = {
        VP580_NO_OP, VP580_NO_OP, VP580_NO_OP, VP580_NO_OP, VP580_NO_OP,
        VP580_NO_OP, VP580_NO_OP, VP580_NO_OP, VP580_NO_OP, VP580_NO_OP,
        VP580_NO_OP, VP580_NO_OP, VP580_NO_OP, VP580_NO_OP, VP580_NO_OP,
        VP580_NO_OP
    };

    /*
     * If the MPI Bus gets out of sequence for any reason, a HW reset command
     * will not work and this function may fail. To be sure a reset occurs, the
     * following sequence is required.
     */
    /* Perform a hardware reset */
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_NO_OP, 16, mpiReset);
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_HW_RESET_CMD, 0, VP_NULL);
    VpSysWait(20);

    /*
     * Setup mclk. The MCLK mask set the mclk frequency, sets the mclk source
     * (the MCLK pin or the PCLK pin), and sets the interrupt pin output drive
     * mode (TTL or open collector)
     */
    data = pDevObj->devProfileData.devCfg1;
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_MCLK_CNT_WRT, VP580_MCLK_CNT_LEN,
        &data);

    /*
     * Wait for the CFAIL bit to clear before proceding. If the CFAIL bit does
     * not clear after several trys, give up and return an error condition. Wait
     * between each read of the status register.
     */

    clkNotStable = VP580_CFAIL_MASK;
    clkTestCount = MAX_CFAIL_TEST;
    while(clkNotStable && (--clkTestCount) != 0) {
        VpSysWait(CFAIL_TEST_INTERVAL*10);
        VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_IODIR_REG_RD,
            VP580_IODIR_REG_LEN, clkFailReg);
        clkNotStable = clkFailReg[0] & VP580_CFAIL_MASK;
    }

    /*
     * The CFAIL bit did not clear so the part will not complete initialization.
     * Return error status to indicate failure.
     */
    if(clkNotStable) {
        pDevObj->deviceEvents.faults |= VP_DEV_EVID_CLK_FLT;
        return VP_STATUS_FAILURE;
    }

    /*
     * Enable all interrupts, we'll determine if the interrupt corresponds to
     * an event in software
     */
    data = VP580_INT_NO_MASK;
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_INT_MASK_WRT,
        VP580_INT_MASK_LEN, &data);

    /*
     * Set the E1 Multiplexing mode and "hardcode" the hook switch debounce
     * to 15mS
     */
    data = pDevObj->devProfileData.debounceReg
        | (0x0F << VP580_DEBOUNCE_TIME_HOOK_SW);
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_DEBOUNCE_TIME_WRT,
        VP580_DEBOUNCE_TIME_LEN, &data);

    /*
     * The PCM mask tells the device which clock edge to grab and xmit the
     * PCM data on and also which clock period LSB of the PCM data starts on
     */
    data = pDevObj->devProfileData.clockSlot;
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_XR_CS_WRT, VP580_XR_CS_LEN, &data);

    return VP_STATUS_SUCCESS;
} /* Vp580Init */

/**
 * Vp580InitDevice
 *  This function initializes the device and all lines associated with this
 * device (if line profiles are passed to this function). The device profile
 * passed must be valid otherwise an error code is returned and the device
 * remains in it's previously initialized state.
 *
 * Preconditions:
 *  None (device context is not NULL and is of Vp580 type, which is handled in
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
Vp580InitDevice(
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
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp580LineObjectType *pLineObj;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    VpProfilePtrType pDevProf;

    uint8 chan;

    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 rcnReg[VP580_DEVTYPE_LEN];

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
    } else {
        return VP_STATUS_ERR_PROFILE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Initialize the API's device status variables */
    pDevObj->status.state = VP_DEV_INIT_IN_PROGRESS;
    pDevObj->timeStamp = 0;

    /* Initialize the API's device dynamic variables */
    pDevObj->dynamicInfo.lastChan = 0;
    pDevObj->dynamicInfo.clkFault = FALSE;

    pDevObj->stateInt = 0;  /* Reset the internal state information */

    if (pDevProf != VP_PTABLE_NULL) {
        pDevObj->devProfileData.pcmClkRate =
            (uint16)(((pDevProf[VP580_DEV_PROFILE_PCLK_MSB] << 8) & 0xFF00)
                    | (pDevProf[VP580_DEV_PROFILE_PCLK_LSB] & 0x00FF));

        pDevObj->devProfileData.devCfg1 =
            (uint8)(pDevProf[VP580_DEV_PROFILE_CHIP_CFG]);
        pDevObj->devProfileData.clockSlot =
            (uint8)(pDevProf[VP580_DEV_PROFILE_CLOCK_SLOT]);

        pDevObj->devProfileData.debounceReg =
            (uint8)(pDevProf[VP580_DEV_PROFILE_E1_MUX_MODE]);


        pDevObj->devProfileData.maxNumInterrupts =
            pDevProf[VP580_DEV_PROFILE_MAX_EVENTS];

        pDevObj->devProfileData.tickRate =
            (uint16)(((pDevProf[VP580_DEV_PROFILE_TICKRATE_MSB] << 8) & 0xFF00)
                    | (pDevProf[VP580_DEV_PROFILE_TICKRATE_LSB] & 0x00FF));
    } else {
        pDevObj->status.state &= ~VP_DEV_INIT_IN_PROGRESS;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_ERR_PROFILE;
    }

    /* Initialize device */

    /*
     * If not successful, the Clock Fail bit did not clear so return error code
     */
    if ((status = Vp580Init(pDevCtx)) != VP_STATUS_SUCCESS) {
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

    /*
     * Verify that the device can be read from by checking a non-0x00/0xFF
     * revision code number
     */
    VpMpiCmdWrapper(deviceId, VP580_EC_CH1, VP580_DEVTYPE_CMD,
        VP580_DEVTYPE_LEN, rcnReg);
    if ((rcnReg[0] == 0x00) || (rcnReg[0] == 0xFF)) {
        /*
         * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
         * completed. This prevents VpApiTick() from generating events on a non-initialized device
         * context/object.
         */
        pDevObj->devProfileData.tickRate = 0;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_FAILURE;
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
            pLineObj = pLineCtx->pLineObj;

            if (pLineObj->status & VP580_IS_FXO) {
                status = Vp580InitLine(pLineCtx, pFxoAcProfile, pFxoCfgProfile,
                    VP_PTABLE_NULL);
            } else {
                status = Vp580InitLine(pLineCtx, pAcProfile, pDcProfile,
                    pRingProfile);
            }
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

    status = VpImplementDefaultSettings(pDevCtx, VP_NULL);

    /*
     * This clears the Init Line Events and any other erroneous event that
     * may have been created due to initialization
     */
    Vp580FlushEvents(pDevCtx);

    /* Set a Device Init Complete Event if status is succesfull */
    if (status == VP_STATUS_SUCCESS) {
        pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
        pDevObj->status.state |= VP_DEV_INIT_CMP;
    }

    pDevObj->status.state &= ~(VP_DEV_INIT_IN_PROGRESS);
    if (status != VP_STATUS_SUCCESS) {
        /*
         * Clear the tickRate, which is used by VpApiTick() to indicate VpInitDevice() was
         * completed. This prevents VpApiTick() from generating events on a non-initialized device
         * context/object.
         */
        pDevObj->devProfileData.tickRate = 0;
    }
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return status;
} /* Vp580InitDevice */

/**
 * Vp580InitLine
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
Vp580InitLine(
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
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;

#ifdef VP_CSLAC_SEQ_EN
    uint8 seqByte;
#endif

    uint8 lineStateIndex;

    /*
     * IO Direction and Control used to restore the device IO to the state
     * set prior to the channel Software Reset
     */
    uint8 ioDirection[VP580_IODIR_REG_LEN];
    uint8 ioData[VP580_IODATA_REG_LEN];

    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 data;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    /*
     * If this line is a custom termination type and has not been configured
     * with a line state map, return error.
     */
    if ((pLineObj->termType == VP_TERM_FXS_CUSTOM)
     && (pLineObj->lineStateInit == FALSE)) {
        return VP_STATUS_CUSTOM_TERM_NOT_CFG;
    }

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pLineObj->status &= ~VP580_INIT_COMPLETE;

#ifdef VP_CSLAC_SEQ_EN
    for (seqByte = 0; seqByte < VP580_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }
#endif

    pLineObj->pRingingCadence = VP_PTABLE_NULL;

    /* Initialize cached transmit and receive gains for SetRelGain to 1.0. */
    pLineObj->gain.gxInt = 0x4000;
    pLineObj->gain.grInt = 0x4000;

    /* Inititialize API line state variables */
    if (pLineObj->status & VP580_IS_FXO) {
        pLineObj->lineState.currentState = VP_LINE_FXO_LOOP_OPEN;
        pLineObj->lineState.previous = VP_LINE_FXO_LOOP_OPEN;
    } else {
        pLineObj->lineState.currentState = VP_LINE_DISCONNECT;
        pLineObj->lineState.previous = VP_LINE_DISCONNECT;
    }

    /* Force a line state check */
    pLineObj->lineState.condition = VP_CSLAC_STATUS_INVALID;

    /* Force a codec update */
    pLineObj->codec = VP_NUM_OPTION_CODEC_TYPE_IDS;

#ifdef VP_CSLAC_SEQ_EN
    VpMemSet(&pLineObj->cadence, 0, sizeof(VpSeqDataType));
#endif

    pLineObj->dpStruct.hookSt =
        (pLineObj->lineState.condition & VP_CSLAC_HOOK) ? TRUE : FALSE;

    VpInitDP(&pLineObj->dpStruct);

    /*
     * Read the IO direction and data for the device IO that will be affected
     * by a software reset
     */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_RD,
        VP580_IODIR_REG_LEN, ioDirection);

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_RD,
        VP580_IODATA_REG_LEN, ioData);

    /* Software reset the channel */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_SW_RESET_CMD, NO_DATA, &data);
    VpSysWait(3);

    ioDirection[0] = 0x00;
    /* Based on termination type, program the I/O accordingly */
    switch(pLineObj->termType) {
        case VP_TERM_FXS_CUSTOM:
            for (lineStateIndex = 0; lineStateIndex< VP580_MAX_NUM_STATES;
                lineStateIndex++) {
                ioDirection[0] |= pLineObj->lineStateBytes[lineStateIndex];
            }
            /*
             * Set the I/O to disconnect value before changing direction so the
             * possibility of ringing is avoided
             */
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_WRT,
                VP580_IODATA_REG_LEN,
                &(pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_DISCONNECT]));

            /* Set timer type in union to FXS type */
            pLineObj->lineTimers.type = VP_CSLAC_FXS_TIMER;

            /*
             * InitTimerVars has to know the timer type but called before
             * Set Line State. If set line state starts any timers, calling
             * InitTimerVars would disable those causing possible initialization
             * issues.
             */
            InitTimerVars(pLineCtx);
            break;

        case VP_TERM_FXO_GENERIC:
            ioDirection[0] |= VP580_IODIR_IO3_MASK;

            /* Set timer type in union to FXO type */
            pLineObj->lineTimers.type = VP_CSLAC_FXO_TIMER;

            /*
             * InitTimerVars has to know the timer type but called before
             * Set Line State. If set line state starts any timers, calling
             * InitTimerVars would disable those causing possible initialization
             * issues.
             */
            InitTimerVars(pLineCtx);
            break;

        default:
            break;
    }

    /*
     * Set the I/O direction as necessary. I/O data is either set according to
     * termination type, or unchanged.
     */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_WRT,
        VP580_IODIR_REG_LEN, ioDirection);

    /*
     * Operating Conditions - Remove all loopbacks, connect TX/RX PCM Hwy
     * Note that TX/RX PCM Highway is set when Set Linestate function is
     * called.
     */
    data = VP580_NORMAL_OP_COND_MODE;
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_COND_WRT, VP580_OP_COND_LEN,
        &data);

    /* Start the channel out in the standby state or loop open (if FXO)  */
    if (pLineObj->status & VP580_IS_FXO) {
        pLineObj->ringDetMin = VP_FXO_RING_DET_MIN_DEFAULT;
        pLineObj->ringDetMax = VP_FXO_RING_DET_MAX_DEFAULT;
        pLineObj->digitGenStruct.breakTime = VP_FXO_PULSE_BREAK_DEFAULT;
        pLineObj->digitGenStruct.makeTime = VP_FXO_PULSE_MAKE_DEFAULT;
        pLineObj->digitGenStruct.flashTime = VP_FXO_FLASH_HOOK_DEFAULT;
        pLineObj->digitGenStruct.dpInterDigitTime = VP_FXO_INTERDIG_DEFAULT;

        status = Vp580ConfigLine(pLineCtx, pAcProfile, pDcOrFxoProfile,
            VP_PTABLE_NULL);
        if (status != VP_STATUS_SUCCESS) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return status;
        }
        /* Activate Codec and enable Supervision */
        Vp580SetLineState(pLineCtx, VP_LINE_FXO_LOOP_OPEN);
    } else {
        /* Set to Disconnect -- Redundant if Custom was selected */
        Vp580SetLineState(pLineCtx, VP_LINE_DISCONNECT);
        status = Vp580ConfigLine(pLineCtx, pAcProfile, pDcOrFxoProfile,
            pRingProfile);
        if (status != VP_STATUS_SUCCESS) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return status;
        }
    }

    status = VpImplementDefaultSettings(VP_NULL, pLineCtx);

    /* Post the line init complete event if status is succesfull */
    if (status == VP_STATUS_SUCCESS) {
        pLineObj->lineEvents.response |= VP_LINE_EVID_LINE_INIT_CMP;
        pLineObj->status |= VP580_INIT_COMPLETE;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
} /* Vp580InitLine */

/**
 * Vp580ConfigLine
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
Vp580ConfigLine(
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
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;
    uint8 profileIndex;

    VpProfileDataType *pMpiData;

    VpProfilePtrType pAcProf = VP_PTABLE_NULL;
    VpProfilePtrType pDcFxoCfgProf = VP_PTABLE_NULL;
    VpProfilePtrType pRingProf = VP_PTABLE_NULL;

    uint8 gainCSD[VP580_GR_GAIN_LEN];

    int profIndex;

    uint8 data;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    /*
     * If this line is a custom termination type and has not been configured
     * with a line state map, return error.
     */
    if ((pLineObj->termType == VP_TERM_FXS_CUSTOM)
     && (pLineObj->lineStateInit == FALSE)) {
        return VP_STATUS_CUSTOM_TERM_NOT_CFG;
    }

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    profIndex = VpGetProfileIndex(pAcProfile);
    if (profIndex < 0) {
        /*
         * A pointer is passed or VP_PTABLE_NULL.  If it's a pointer, make
         * sure the content is valid for the profile type.
         */
        if (pAcProfile != VP_PTABLE_NULL) {
            if(VpVerifyProfileType(VP_PROFILE_AC, pAcProfile) != TRUE) {
                return VP_STATUS_ERR_PROFILE;
            }
        }
        /* If we're here, it's a valid profile pointer -- even if NULL */
        pAcProf = pAcProfile;
    } else if (profIndex < VP_CSLAC_AC_PROF_TABLE_SIZE) {
        pAcProf = pDevObj->devProfileTable.pAcProfileTable[profIndex];
        if (!(pDevObj->profEntry.acProfEntry & (0x01 << profIndex))) {
            return VP_STATUS_ERR_PROFILE;
        }
    } else {
        return VP_STATUS_ERR_PROFILE;
    }

    profIndex = VpGetProfileIndex(pDcOrFxoProfile);
    if (profIndex < 0) {
        /*
         * A pointer is passed or VP_PTABLE_NULL.  If it's a pointer, make
         * sure the content is valid for the profile type.
         */
        if (pDcOrFxoProfile != VP_PTABLE_NULL) {
            if (pLineObj->status & VP580_IS_FXO) {
                if (VpVerifyProfileType(VP_PROFILE_FXO_CONFIG, pDcOrFxoProfile)
                     != TRUE) {
                    return VP_STATUS_ERR_PROFILE;
                }
            } else {
                if (VpVerifyProfileType(VP_PROFILE_DC, pDcOrFxoProfile)
                    != TRUE) {
                    return VP_STATUS_ERR_PROFILE;
                }
            }
        }
        /* If we're here, it's a valid profile pointer -- even if NULL */
        pDcFxoCfgProf = pDcOrFxoProfile;
    } else {
        if (pLineObj->status & VP580_IS_FXO) {
            if (profIndex < VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE) {
                pDcFxoCfgProf =
                    pDevObj->devProfileTable.pFxoConfigProfileTable[profIndex];

                if (!(pDevObj->profEntry.fxoConfigProfEntry
                    & (0x01 << profIndex))) {
                    return VP_STATUS_ERR_PROFILE;
                }
            } else {
                return VP_STATUS_ERR_PROFILE;
            }
        }
    }

    profIndex = VpGetProfileIndex(pRingProfile);
    if (profIndex < 0) {
        /*
         * A pointer is passed or VP_PTABLE_NULL.  If it's a pointer, make
         * sure the content is valid for the profile type.
         */
        if (pRingProfile != VP_PTABLE_NULL) {
            if (VpVerifyProfileType(VP_PROFILE_RING, pRingProfile) != TRUE) {
                return VP_STATUS_ERR_PROFILE;
            }
        }
        /* If we're here, it's a valid profile pointer -- even if NULL */
        pRingProf = pRingProfile;
    } else if (profIndex < VP_CSLAC_RINGING_PROF_TABLE_SIZE) {
        pRingProf = pDevObj->devProfileTable.pRingingProfileTable[profIndex];
        if (!(pDevObj->profEntry.ringingProfEntry & (0x01 << profIndex))) {
            return VP_STATUS_ERR_PROFILE;
        }
    } else {
        return VP_STATUS_ERR_PROFILE;
    }

    /* Shutdown previous external ring generation GPIO control method: */
    if (pLineObj->ringParams.method != VP580_RING_METHOD_NONE) {
        Vp580RingSigGen(pLineCtx, VP580_RING_SIG_GEN_SHUTDOWN, VP_NULL);
    }

    /* Initialize external ring generator GPIO control: */
    if (pRingProfile != VP_PTABLE_NULL) {
        if (pRingProfile[VP_PROFILE_LENGTH] < 3) {
            return VP_STATUS_ERR_PROFILE;
        }
        pLineObj->ringParams.method = pRingProf[VP580_RING_PROFILE_METHOD];
        pLineObj->ringParams.timerMs = pRingProf[VP580_RING_PROFILE_TIMER_PERIOD] * 5;
        Vp580RingSigGen(pLineCtx, VP580_RING_SIG_GEN_INIT, VP_NULL);
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Load AC Coefficients */
    if (pAcProf != VP_PTABLE_NULL) {
        profileIndex = VP_PROFILE_MPI_LEN + 1;
        pMpiData = (VpProfileDataType *)(&pAcProfile[profileIndex]);
        VpMpiCmdWrapper(deviceId, ecVal[channelId], NOOP_CMD,
            pAcProfile[VP_PROFILE_MPI_LEN], pMpiData);

        /* Operating Functions - Use loaded coefficients */
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_FUNC_RD,
            VP580_OP_FUNC_LEN, &data);
        data |= VP580_ENABLE_LOADED_COEFFICIENTS;
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_FUNC_WRT,
            VP580_OP_FUNC_LEN, &data);

        /* Update cached transmit and receive gains for SetRelGain */
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_GX_GAIN_RD,
            VP580_GX_GAIN_LEN, gainCSD);
        pLineObj->gain.gxInt = 0x4000 + VpConvertCsd2Fixed(gainCSD);

        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_GR_GAIN_RD,
            VP580_GR_GAIN_LEN, gainCSD);
        pLineObj->gain.grInt = VpConvertCsd2Fixed(gainCSD);
    }

    if (pLineObj->status & VP580_IS_FXO) {
        /* Configure an FXO line type */
        if (pDcFxoCfgProf != VP_PTABLE_NULL) {
            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MIN;
            /* Convert from 250uS to 500uS resolution */
            pLineObj->ringDetMin = (pDcFxoCfgProf[profileIndex] / 2) ;

            profileIndex = VP_FXO_DIALING_PROFILE_RING_PERIOD_MAX;
            /* Convert from 250uS to 500uS resolution */
            pLineObj->ringDetMax = (pDcFxoCfgProf[profileIndex] / 2) ;

            profileIndex = VP_FXO_DIALING_PROFILE_PULSE_BREAK;
            pLineObj->digitGenStruct.breakTime = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_PULSE_MAKE;
            pLineObj->digitGenStruct.makeTime = pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_FLASH_HOOK_MSB;
            pLineObj->digitGenStruct.flashTime =
                (pDcFxoCfgProf[profileIndex] << 8)&0xFF00;

            profileIndex = VP_FXO_DIALING_PROFILE_FLASH_HOOK_LSB;
            pLineObj->digitGenStruct.flashTime |= pDcFxoCfgProf[profileIndex];

            profileIndex = VP_FXO_DIALING_PROFILE_INTERDIGIT_MSB;
            pLineObj->digitGenStruct.dpInterDigitTime =
                (pDcFxoCfgProf[profileIndex] << 8)&0xFF00;

            profileIndex = VP_FXO_DIALING_PROFILE_INTERDIGIT_LSB;
            pLineObj->digitGenStruct.dpInterDigitTime =
                pDcFxoCfgProf[profileIndex];
        }
        /* FXS lines do not support DC Feed Profile Currently */
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp580InitCustomTerm()
 *  This function is used to initialize the control of a custom termination
 * type.
 *
 * Preconditions:
 *  The device and line context must exist.
 *
 * Postconditions:
 *  The profile data passed is stored in the line object if the term type is
 * custom.
 */
VpStatusType
Vp580InitCustomTerm(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCustomTermProfile)
{
    Vp580DeviceObjectType *pDevObj = VP_NULL;
    Vp580LineObjectType *pLineObj = VP_NULL;
    uint8 channelId;
    uint8 maxChan = 0;
    VpProfilePtrType *pProfileTable;
    VpDeviceIdType deviceId;
    VpDevCtxType *pDevCtxLocal;
    VpProfilePtrType pCustomTerm = VP_PTABLE_NULL;
    uint8 stateCnt, stateCntMax, startIndex, state, stateMap;

    int customIndex = VpGetProfileIndex(pCustomTermProfile);

    if (pDevCtx == VP_NULL) {
        pLineObj = pLineCtx->pLineObj;

        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        deviceId = pDevObj->deviceId;
    } else {
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;
        maxChan = pDevObj->staticInfo.maxChannels;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * If the profile passed is an index, make sure it's in the valid range
     * and if so, set the currently used profile to it.
     */
    if ((customIndex >= 0) && (customIndex < VP_CSLAC_CUSTOM_TERM_PROF_TABLE_SIZE)) {
        /* Valid index.  Set it if it's not an invalid table entry */
        if (!(pDevObj->profEntry.customTermProfEntry & (0x01 << customIndex))) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_PROFILE;
        }

        pProfileTable = pDevObj->devProfileTable.pCustomTermProfileTable;
        pCustomTerm = pProfileTable[customIndex];
    } else if (customIndex >= VP_CSLAC_CUSTOM_TERM_PROF_TABLE_SIZE) {
        /* It's an index, but it's out of range */
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_ERR_PROFILE;
    } else {
        /* This is a pointer. Set it if it's the correct type */
        if(VpVerifyProfileType(VP_PRFWZ_PROFILE_CUSTOM_TERM, pCustomTermProfile) == TRUE) {
            pCustomTerm = pCustomTermProfile;
        } else {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_PROFILE;
        }
    }

    /*
     * The profile is valid. See if we're setting it for all lines of the device
     * (that are custom term) or just the line
     */

    if (pDevCtx == VP_NULL) {
        /* Initialize only this line if it's custom type */
        if (pLineObj->termType == VP_TERM_FXS_CUSTOM) {
            if (pCustomTerm == VP_PTABLE_NULL) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_INVALID_ARG;
            } else {
                stateCntMax = pCustomTerm[VP_CUSTOM_TERM_NUM_STATES];
                startIndex = VP_CUSTOM_TERM_NUM_STATES + 1;
                for (stateCnt = 0; stateCnt < stateCntMax; stateCnt++) {
                    state =  pCustomTerm[(2 * stateCnt) + startIndex];
                    stateMap = pCustomTerm[(2 * stateCnt) + startIndex + 1];
                    if (state == VP_PRFWZ_CUSTOM_ST_DET_MAP) {
                        pLineObj->detMap = stateMap;
                        pLineObj->bitMask |= stateMap;
                    } else {
                        pLineObj->lineStateBytes[state] = stateMap;
                        pLineObj->lineStateExist |= (0x0001 << state);
                        pLineObj->bitMask |= stateMap;
                    }
                    pLineObj->lineStateInit = TRUE;
                }
            }
        }
    } else {
        /*
         * Check all the channels for this device and if they are not VP_NULL,
         * verify if they are custom term type. If they are cusom term type,
         * then verify a profile is passed. If not, return an error. Otherwise
         * perform the initialization.
         */
        for (channelId = 0; channelId < maxChan; channelId++) {
            pLineCtx = pDevCtx->pLineCtx[channelId];

            if (pLineCtx != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
                if (pLineObj->termType == VP_TERM_FXS_CUSTOM) {
                    if (pCustomTerm == VP_PTABLE_NULL) {
                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        return VP_STATUS_INVALID_ARG;
                    } else {
                        stateCntMax = pCustomTerm[VP_CUSTOM_TERM_NUM_STATES];
                        startIndex = VP_CUSTOM_TERM_NUM_STATES + 1;
                        for (stateCnt = 0; stateCnt < stateCntMax; stateCnt++) {
                            state =  pCustomTerm[(2 * stateCnt) + startIndex];
                            stateMap = pCustomTerm[(2 * stateCnt) + startIndex + 1];
                            if (state == VP_PRFWZ_CUSTOM_ST_DET_MAP) {
                                pLineObj->detMap = stateMap;
                                pLineObj->bitMask |= stateMap;
                            } else {
                                pLineObj->lineStateBytes[state] = stateMap;
                                pLineObj->lineStateExist |= (0x0001 << state);
                                pLineObj->bitMask |= stateMap;
                            }
                            pLineObj->lineStateInit = TRUE;
                        }
                    }
                }
            }
        }
    }
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp580InitProfile()
 *  This function is used to initialize profile tables in Vp580.
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  Stores the given profile at the specified index of the profile table.
 */
VpStatusType
Vp580InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

    uint8 profIndex8;   /* Used for 8-bit profile table masking */

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

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    /*
     * The correct types are passed, but check to make sure the specific profile
     * type being initialized is valid as well as the index value
     */
    switch(type) {
        case VP_PROFILE_DEVICE:
            if (profIndex8 >= VP_CSLAC_DEV_PROF_TABLE_SIZE) {
                status = VP_STATUS_INVALID_ARG;
            } else {
                if(VpVerifyProfileType(VP_PROFILE_DEVICE, pProfile) == TRUE) {
                    pDevObj->devProfileTable.pDevProfileTable[profIndex8] =
                        pProfile;
                    /*
                     * If the profile is null, then clear the flag in the
                     * profile entry table to indicate that this profile is no
                     * longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.devProfEntry &=
                            ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.devProfEntry |=
                            (0x01 << profIndex8);
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
                    pDevObj->devProfileTable.pAcProfileTable[profIndex8] =
                        pProfile;
                    /*
                     * If the profile is null, then clear the flag in the
                     * profile entry table to indicate that this profile is no
                     * longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.acProfEntry &=
                            ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.acProfEntry |=
                            (0x01 << profIndex8);
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
                    pDevObj->devProfileTable.pRingingCadProfileTable[profIndex8] =
                        pProfile;
                    /*
                     * If the profile is null, then clear the flag in the
                     * profile entry table to indicate that this profile is no
                     * longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.ringCadProfEntry &=
                            ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.ringCadProfEntry |=
                            (0x01 << profIndex8);
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
                    pDevObj->devProfileTable.pFxoConfigProfileTable[profIndex8] =
                        pProfile;
                    /*
                     * If the profile is null, then clear the flag in the
                     * profile entry table to indicate that this profile is no
                     * longer valid.
                     */
                    if (pProfile == VP_PTABLE_NULL) {
                        pDevObj->profEntry.fxoConfigProfEntry &=
                            ~(0x01 << profIndex8);
                    } else {
                        pDevObj->profEntry.fxoConfigProfEntry |=
                            (0x01 << profIndex8);
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
} /* Vp580InitProfile() */

#endif






