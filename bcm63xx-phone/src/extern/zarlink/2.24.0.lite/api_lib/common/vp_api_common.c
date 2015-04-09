/** \file vp_api_common.c
 * vp_api_common.c
 *
 *  This file contains functions that are common to more than one device type
 * but not accessible to the user
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11607 $
 * $LastChangedDate: 2014-10-20 15:31:02 -0500 (Mon, 20 Oct 2014) $
 */

#include "vp_api_cfg.h"

/* INCLUDES */
#include "vp_api.h"     /* Typedefs and function prototypes for API */
#include "vp_api_int.h" /* Device specific typedefs and function prototypes */
#include "sys_service.h"

#include "vp_hal.h"

#if defined (VP_CC_880_SERIES)
#include "vp880_api_int.h"
#endif

static VpProfileType
ConvertPrfWizPrfType2ApiType(
    const VpProfileWizProfileType type);

#if (defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) \
 || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES))

#if defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) || defined(VP_CC_890_SERIES)
#ifdef CSLAC_GAIN_ABS
static VpStatusType
CommonWrite(
    VpLineCtxType *pLineCtx,
    uint8 grGxGmd,
    uint8 grGxCmdLen,
    uint8* grGxValue);
    
static VpStatusType
CommonRead(
    VpLineCtxType *pLineCtx,
    uint8 grGxGmd,
    uint8 grGxCmdLen,
    uint8* grGxValue);
#endif
#endif

#undef  MPI_CMD_SEARCH
#undef  MPI_SHORT_FORMAT
#define MPI_LONG_FORMAT

#if (VP_CC_DEBUG_SELECT & VP_DBG_HAL)
#if defined(MPI_CMD_SEARCH)

#define MPI_CMD_TO_FIND (0xD2u)

static int16
VpMpiFindCmd(
    uint8 byteMatch,
    uint8 mpiCmd,
    uint8 mpiCmdLen,
    uint8 *dataBuffer);

#endif /* MPI_CMD_SEARCH */
#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_HAL) */
#endif /* (790 | 880 | 886 | 890 | 580) */

/**
 * ConvertPrfWizPrfType2ApiType()
 *  This function converts from Profile Wizard profile type to VP-API specific
 * profile type.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
VpProfileType
ConvertPrfWizPrfType2ApiType(
    const VpProfileWizProfileType type)   /* Profile to be converted */
{
    switch (type) {
        case VP_PRFWZ_PROFILE_AC:              return VP_PROFILE_AC;
        case VP_PRFWZ_PROFILE_DC:              return VP_PROFILE_DC;
        case VP_PRFWZ_PROFILE_TONE:            return VP_PROFILE_TONE;
        case VP_PRFWZ_PROFILE_TONECAD:         return VP_PROFILE_TONECAD;
        case VP_PRFWZ_PROFILE_RING:            return VP_PROFILE_RING;
        case VP_PRFWZ_PROFILE_CID_TYPE1:       return VP_PROFILE_CID;
        case VP_PRFWZ_PROFILE_CID_TYPE2:       return VP_PROFILE_CID;
        case VP_PRFWZ_PROFILE_METER:           return VP_PROFILE_METER;
        case VP_PRFWZ_PROFILE_RINGCAD:         return VP_PROFILE_RINGCAD;
        case VP_PRFWZ_PROFILE_DEVICE:          return VP_PROFILE_DEVICE;
        case VP_PRFWZ_PROFILE_FXO_CONFIG:      return VP_PROFILE_FXO_CONFIG;
        case VP_PRFWZ_PROFILE_FXS_CTRL:        return VP_PROFILE_CUSTOM_TERM;
        case VP_PRFWZ_PROFILE_CAL:             return VP_PROFILE_CAL;
        default:                               return VP_NUM_PROFILE_TYPES;
    }
} /* ConvertPrfWizPrfType2ApiType() */

/**
 * VpGetProfileIndex()
 *  This function returns TRUE if the passed profile pointer is an index or just
 * a normal pointer. If the passed profile pointer is an index then the index is
 * also returned.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
int
VpGetProfileIndex (
    const VpProfilePtrType pProfile) /* Given Profile pointer */
{
    if((pProfile >= VP_PTABLE_INDEX(1)) && (pProfile <= VP_PTABLE_INDEX(32))){
        if(pProfile == VP_PTABLE_INDEX(1)) {return 0;}
        else if(pProfile == VP_PTABLE_INDEX(2)) {return 1;}
        else if(pProfile == VP_PTABLE_INDEX(3)) {return 2;}
        else if(pProfile == VP_PTABLE_INDEX(4)) {return 3;}
        else if(pProfile == VP_PTABLE_INDEX(5)) {return 4;}
        else if(pProfile == VP_PTABLE_INDEX(6)) {return 5;}
        else if(pProfile == VP_PTABLE_INDEX(7)) {return 6;}
        else if(pProfile == VP_PTABLE_INDEX(8)) {return 7;}
        else if(pProfile == VP_PTABLE_INDEX(9)) {return 8;}
        else if(pProfile == VP_PTABLE_INDEX(10)) {return 9;}
        else if(pProfile == VP_PTABLE_INDEX(11)) {return 10;}
        else if(pProfile == VP_PTABLE_INDEX(12)) {return 11;}
        else if(pProfile == VP_PTABLE_INDEX(13)) {return 12;}
        else if(pProfile == VP_PTABLE_INDEX(14)) {return 13;}
        else if(pProfile == VP_PTABLE_INDEX(15)) {return 14;}
        else if(pProfile == VP_PTABLE_INDEX(16)) {return 15;}
        else if(pProfile == VP_PTABLE_INDEX(17)) {return 16;}
        else if(pProfile == VP_PTABLE_INDEX(18)) {return 17;}
        else if(pProfile == VP_PTABLE_INDEX(19)) {return 18;}
        else if(pProfile == VP_PTABLE_INDEX(20)) {return 19;}
        else if(pProfile == VP_PTABLE_INDEX(21)) {return 20;}
        else if(pProfile == VP_PTABLE_INDEX(22)) {return 21;}
        else if(pProfile == VP_PTABLE_INDEX(23)) {return 22;}
        else if(pProfile == VP_PTABLE_INDEX(24)) {return 23;}
        else if(pProfile == VP_PTABLE_INDEX(25)) {return 24;}
        else if(pProfile == VP_PTABLE_INDEX(26)) {return 25;}
        else if(pProfile == VP_PTABLE_INDEX(27)) {return 26;}
        else if(pProfile == VP_PTABLE_INDEX(28)) {return 27;}
        else if(pProfile == VP_PTABLE_INDEX(29)) {return 28;}
        else if(pProfile == VP_PTABLE_INDEX(30)) {return 29;}
        else if(pProfile == VP_PTABLE_INDEX(31)) {return 30;}
        else if(pProfile == VP_PTABLE_INDEX(32)) {return 31;}
    }
    return -1;
} /* VpGetProfileIndex() */

/**
 * VpVerifyProfileType()
 *  This function verifies that the profile pointer passed matches the type of
 * profile being passed.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  Returns TRUE if the profile type and profile match.  Otherwise returns
 * FALSE.  Note that a NULL profile is valid and has specific meanings in the
 * API-II depending on the profile.
 */
bool
VpVerifyProfileType(
    VpProfileType type,
    VpProfilePtrType pProfile)
{
    if (pProfile == VP_PTABLE_NULL) {
        return TRUE;
    } else if ((pProfile >= VP_PTABLE_INDEX1)
            && (pProfile <= VP_PTABLE_INDEX15)){
        /* This function does not expect to see profile indexes */
        return FALSE;
    }

    if (ConvertPrfWizPrfType2ApiType((VpProfileWizProfileType)pProfile[VP_PROFILE_TYPE_LSB]) != type) {
        return FALSE;
    } else {
        return TRUE;
    }
}

/**
 * VpIsDigit()
 *  This function returns TRUE if the digit passed is a valid VpDigitType,
 * otherwise returns FALSE. Utility function for the API-II.
 */
bool
VpIsDigit(
    VpDigitType digit)
{
    if ((digit >= 0) && (digit <= 9)) {
        return TRUE;
    }

    switch(digit) {
        case VP_DIG_ZERO:
        case VP_DIG_ASTER:
        case VP_DIG_POUND:
        case VP_DIG_A:
        case VP_DIG_B:
        case VP_DIG_C:
        case VP_DIG_D:
        case VP_DIG_NONE:
            return TRUE;
        default:
            return FALSE;
    }
}

#if defined(VP_CC_880_SERIES) || defined(VP_CC_890_SERIES)
VpStatusType
VpCSLACSetDTMFGenValues(
    uint8 *sigGenABParams,
    VpDigitType digit)
{
    uint8 columnFreqs[] = {
        0x0C, 0xE5,    /* 1209Hz (1, 4, 7, *) */
        0x0E, 0x40,    /* 1336Hz (2, 5, 8, 0) */
        0x0F, 0xC1,    /* 1477Hz (3, 6, 9, #) */
        0x11, 0x6B     /* 1633Hz (A, B, C, D) */
    };

    uint8 rowFreqs[] = {
        0x07, 0x6F,    /* 697Hz (1, 2, 3, A) */
        0x08, 0x36,    /* 770Hz (4, 5, 6, B) */
        0x09, 0x16,    /* 852Hz (7, 8, 9, C) */
        0x0A, 0x09     /* 941Hz (*, 0, #, D) */
    };

    /* Set the Column Freqs first */
    switch(digit) {
        case 1:
        case 4:
        case 7:
        case VP_DIG_ASTER:
            sigGenABParams[0] = columnFreqs[0];
            sigGenABParams[1] = columnFreqs[1];
            break;

        case 2:
        case 5:
        case 8:
        case VP_DIG_ZERO:
            sigGenABParams[0] = columnFreqs[2];
            sigGenABParams[1] = columnFreqs[3];
            break;

        case 3:
        case 6:
        case 9:
        case VP_DIG_POUND:
            sigGenABParams[0] = columnFreqs[4];
            sigGenABParams[1] = columnFreqs[5];
            break;

        case VP_DIG_A:
        case VP_DIG_B:
        case VP_DIG_C:
        case VP_DIG_D:
            sigGenABParams[0] = columnFreqs[6];
            sigGenABParams[1] = columnFreqs[7];
            break;

        /*
         * Digit None will generally be accompanied by EOT. But we need to cover this digit
         * case anyway (to proceed with this function), and technically speaking the device can
         * be programmed to generate "No" digit while the generators are enabled.
         */
        case VP_DIG_NONE:
            sigGenABParams[0] = 0;
            sigGenABParams[1] = 0;
            break;

        default:
            VP_ERROR(None, NULL,
                     ("Inivalid Digit (0x%02X) passed : VpCSLACSetDTMFGenValues-", digit));
            VP_API_FUNC_INT(None, VP_NULL, ("VpCSLACSetDTMFGenValues-"));
            return VP_STATUS_INVALID_ARG;
    }

    /* Now set the row freqs */
    switch(digit) {
        case 1:
        case 2:
        case 3:
        case VP_DIG_A:
            sigGenABParams[4] = rowFreqs[0];
            sigGenABParams[5] = rowFreqs[1];
            break;

        case 4:
        case 5:
        case 6:
        case VP_DIG_B:
            sigGenABParams[4] = rowFreqs[2];
            sigGenABParams[5] = rowFreqs[3];
            break;

        case 7:
        case 8:
        case 9:
        case VP_DIG_C:
            sigGenABParams[4] = rowFreqs[4];
            sigGenABParams[5] = rowFreqs[5];
            break;

        case VP_DIG_ASTER:
        case VP_DIG_ZERO:
        case VP_DIG_POUND:
        case VP_DIG_D:
            sigGenABParams[4] = rowFreqs[6];
            sigGenABParams[5] = rowFreqs[7];
            break;

        /* Digit None - same comments as above */
        case VP_DIG_NONE:
            sigGenABParams[4] = 0;
            sigGenABParams[5] = 0;
            break;

        default:
            VP_ERROR(None, NULL,
                     ("Inivalid Digit (0x%02X) passed : VpCSLACSetDTMFGenValues-", digit));
            VP_API_FUNC_INT(None, VP_NULL, ("VpCSLACSetDTMFGenValues-"));
            return VP_STATUS_INVALID_ARG;
    }
    VP_API_FUNC_INT(None, VP_NULL, ("VpCSLACSetDTMFGenValues-"));
    return VP_STATUS_SUCCESS;
}

/**<
 * VpCSLACHookMaskEnabled()
 *   This function is used by VE880 and VE890 API to determine if any of the line times that
 * affect hook detect mask are enabled. Return TRUE if hook masking is enabled, FALSE otherwise.
 */
bool
VpCSLACHookMaskEnabled(
    uint16 fxsTimers[VP_LINE_TIMER_LAST])
{
    if ((fxsTimers[VP_LINE_RING_EXIT_PROCESS] & VP_ACTIVATE_TIMER)
     || (fxsTimers[VP_LINE_HOOK_FREEZE] & VP_ACTIVATE_TIMER)
     || (fxsTimers[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER)
#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
     || (fxsTimers[VP_LINE_CAL_LINE_TIMER] & VP_ACTIVATE_TIMER)
#endif
    || (fxsTimers[VP_LINE_TRACKER_DISABLE] & VP_ACTIVATE_TIMER)
    || (fxsTimers[VP_LINE_GND_START_TIMER] & VP_ACTIVATE_TIMER)
#ifdef VP_CSLAC_SEQ_EN
    || (fxsTimers[VP_LINE_CID_DEBOUNCE] & VP_ACTIVATE_TIMER)
#endif  /* VP_CSLAC_SEQ_EN */
    ) {
        return TRUE;
    }
    return FALSE;
}

#endif

/* Code used for CSLAC & 792 */
#if defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) \
 || defined(VP_CC_890_SERIES) || defined(VP_CC_792_SERIES) \
 || defined(VP_CC_580_SERIES)


/** COMMON INITIALIZATION FUNCTIONS */
/**
 * VpImplementNonMaskEvents()
 *  This function modifies the line and device event structures with the API
 * standard non-masking event bits.  A non-masked event bit is 0.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  The event structures passed are modified by the non-masked event bits.
 */
void
VpImplementNonMaskEvents(
    VpOptionEventMaskType *pLineEventsMask, /**< Line Events Mask to modify for
                                             * non-masking
                                             */
    VpOptionEventMaskType *pDevEventsMask)  /**< Device Events Mask to modify
                                             * for non-masking
                                             */
{
    pLineEventsMask->faults     &= ~VP_API_NONMASK_FAULT_EVENTS;
    pLineEventsMask->signaling  &= ~VP_API_NONMASK_SIGNALING_EVENTS;
    pLineEventsMask->response   &= ~VP_API_NONMASK_RESPONSE_EVENTS;
    pLineEventsMask->test       &= ~VP_API_NONMASK_TEST_EVENTS;
    pLineEventsMask->process    &= ~VP_API_NONMASK_PROCESS_EVENTS;
    pLineEventsMask->fxo        &= ~VP_API_NONMASK_FXO_EVENTS;
    pLineEventsMask->packet     &= ~VP_API_NONMASK_PACKET_EVENTS;

    pDevEventsMask->faults      &= ~VP_API_NONMASK_FAULT_EVENTS;
    pDevEventsMask->signaling   &= ~VP_API_NONMASK_SIGNALING_EVENTS;
    pDevEventsMask->response    &= ~VP_API_NONMASK_RESPONSE_EVENTS;
    pDevEventsMask->test        &= ~VP_API_NONMASK_TEST_EVENTS;
    pDevEventsMask->process     &= ~VP_API_NONMASK_PROCESS_EVENTS;
    pDevEventsMask->fxo         &= ~VP_API_NONMASK_FXO_EVENTS;
    pDevEventsMask->packet      &= ~VP_API_NONMASK_PACKET_EVENTS;

    return;
}

/**
 * VpImplementDefaultSettings()
 *  This function executes the options to set the device/lines to API-II
 * standard default settings.  It may be passed a valid device context, or a
 * valid line context.  The device and line context do not need to be associated
 * with each other.  This is a convenient function for the API itself to use
 * when a device or line is initialized.
 *
 * Note: This function doesn't set the VP_OPTION_ID_EVENT_MASK option if passed
 *   a Line Context.  This is because doing so would affect the device-specific
 *   event masks for all lines on the same device.  Therefore, special separate
 *   handling of the VP_OPTION_ID_EVENT_MASK option is required outside this
 *   function when applying line-specific defaults.
 *
 * Preconditions:
 * None
 *
 * Postconditions:
 * The device and line associated with this device is initialized with default
 * values.
 */
VpStatusType
VpImplementDefaultSettings(
    VpDevCtxType *pDevCtx,      /**< Device to implement for default API-II
                                 * options
                                 */
    VpLineCtxType *pLineCtx)    /**< Line to implement for default API-II
                                 * options
                                 */
{
    VpStatusType status = VP_STATUS_SUCCESS;

    VpSetOptionFuncPtrType pSetOption = VP_NULL;
    VpOptionPulseType pulseSpec;
    VpOptionLinePulseType linePulseSpec;
    VpOptionPulseType pulseSpec2;
    VpOptionCriticalFltType criticalFault;
    VpOptionZeroCrossType zeroCross;
    VpOptionPulseModeType pulseMode;
    VpOptionCodecType codec;
    VpOptionPcmHwyType pcmHwy;
    VpOptionLoopbackType loopBack;
    VpOptionLineStateType lineState;
    VpOptionRingControlType ringCtrl;
    VpOptionPcmTxRxCntrlType pcmTxRxCtrl;
    VpOptionDtmfSpecType dtmfSpec;
    VpOptionParkModeType parkMode;
    VpOptionEventMaskType eventMask;
    uint16 slopeRate = VP_OPTION_DEFAULT_DCFEED_SLOPE;
    VpOptionPcmSigCtlType pcmSigCtl;
    VpOptionLinestateCtlModeType linestateCtlMode = VP_OPTION_DEFAULT_LINESTATE_CTL_MODE;
    VpOptionHookDetectModeType hookDetectMode = VP_OPTION_DEFAULT_HOOK_DETECT_MODE;
    VpOptionAutoLoopCondType autoLoopCond;
    bool switcherCtrl;
    VpOptionGndFltProtType gndFltProt;
    VpOptionAdaptiveRingingType adaptiveRinging;
    VpOptionDtmfModeType dtmfMode;
    uint16 ringTripConfirm;
    VpOptionRingPhaseSyncType ringPhaseSync;
    VpOptionHighPassFilterType highPassFilter;
    VpOptionDtmfParamsType dtmfParams;

    if ((pDevCtx == VP_NULL) && (pLineCtx == VP_NULL)) {
        return VP_STATUS_INVALID_ARG;
    }

    if(pDevCtx != VP_NULL) {
        pSetOption = pDevCtx->funPtrsToApiFuncs.SetOption;
    } else {
        pSetOption = pLineCtx->pDevCtx->funPtrsToApiFuncs.SetOption;
    }

    if (pSetOption == VP_NULL) {
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    }

    linePulseSpec.breakMin = pulseSpec.breakMin = VP_OPTION_DEFAULT_DP_BREAK_MIN;
    linePulseSpec.breakMax = pulseSpec.breakMax = VP_OPTION_DEFAULT_DP_BREAK_MAX;
    linePulseSpec.makeMin = pulseSpec.makeMin = VP_OPTION_DEFAULT_DP_MAKE_MIN;
    linePulseSpec.makeMax = pulseSpec.makeMax = VP_OPTION_DEFAULT_DP_MAKE_MAX;
    linePulseSpec.interDigitMin = pulseSpec.interDigitMin = VP_OPTION_DEFAULT_DP_INTER_DIG_MIN;
    linePulseSpec.flashMin = pulseSpec.flashMin = VP_OPTION_DEFAULT_DP_FLASH_MIN;
    linePulseSpec.flashMax = pulseSpec.flashMax = VP_OPTION_DEFAULT_DP_FLASH_MAX;
    linePulseSpec.onHookMin = VP_OPTION_DEFAULT_DP_ON_HOOK_MIN;
    linePulseSpec.offHookMin = VP_OPTION_DEFAULT_DP_OFF_HOOK_MIN;

    pulseSpec2.breakMin = VP_OPTION_DEFAULT_DP_BREAK_MIN2;
    pulseSpec2.breakMax = VP_OPTION_DEFAULT_DP_BREAK_MAX2;
    pulseSpec2.makeMin = VP_OPTION_DEFAULT_DP_MAKE_MIN2;
    pulseSpec2.makeMax = VP_OPTION_DEFAULT_DP_MAKE_MAX2;
    pulseSpec2.interDigitMin = VP_OPTION_DEFAULT_DP_INTER_DIG_MIN2;
    pulseSpec2.flashMin = VP_OPTION_DEFAULT_DP_FLASH_MIN2;
    pulseSpec2.flashMax = VP_OPTION_DEFAULT_DP_FLASH_MAX2;

#ifdef EXTENDED_FLASH_HOOK
    pulseSpec.onHookMin = VP_OPTION_DEFAULT_DP_ON_HOOK_MIN;
    pulseSpec2.onHookMin = VP_OPTION_DEFAULT_DP_ON_HOOK_MIN2;
#endif

#ifdef VP_ENABLE_OFFHOOK_MIN
    pulseSpec.offHookMin = VP_OPTION_DEFAULT_DP_OFF_HOOK_MIN;
    pulseSpec2.offHookMin = VP_OPTION_DEFAULT_DP_OFF_HOOK_MIN2;
#endif

    pulseMode = VP_OPTION_DEFAULT_PULSE_MODE;

    criticalFault.acFltDiscEn = VP_OPTION_DEFAULT_CF_AC_DIS_EN;
    criticalFault.dcFltDiscEn = VP_OPTION_DEFAULT_CF_DC_DIS_EN;
    criticalFault.thermFltDiscEn = VP_OPTION_DEFAULT_CF_THERMAL_DIS_EN;

    zeroCross = VP_OPTION_DEFAULT_ZERO_CROSS;

    codec = VP_OPTION_DEFAULT_CODEC_MODE;
    pcmHwy = VP_OPTION_DEFAULT_PCM_HWY;

    pcmTxRxCtrl = VP_OPTION_DEFAULT_PCM_TXRX_CNTRL;

    loopBack = VP_OPTION_DEFAULT_LOOP_BACK;
    lineState.bat = VP_OPTION_DEFAULT_LS_BAT;
    lineState.battRev = VP_OPTION_DEFAULT_LS_BAT_REV;

    eventMask.faults = (uint16)VP_OPTION_DEFAULT_FAULT_EVENT_MASK;
    eventMask.signaling = (uint16)VP_OPTION_DEFAULT_SIGNALING_EVENT_MASK;
    eventMask.response = (uint16)VP_OPTION_DEFAULT_RESPONSE_EVENT_MASK;
    eventMask.test = (uint16)VP_OPTION_DEFAULT_TEST_EVENT_MASK;
    eventMask.process = (uint16)VP_OPTION_DEFAULT_PROCESS_EVENT_MASK;
    eventMask.fxo = (uint16)VP_OPTION_DEFAULT_FXO_EVENT_MASK;
    eventMask.packet = (uint16)VP_OPTION_DEFAULT_PACKET_EVENT_MASK;

    ringCtrl.ringExitDbncDur = VP_OPTION_DEFAULT_RC_RING_EXIT_DBNC_VAL;
    ringCtrl.ringTripExitSt = VP_OPTION_DEFAULT_RC_RING_EXIT_STATE;
    ringCtrl.zeroCross = VP_OPTION_DEFAULT_RC_ZERO_CROSS;

    pcmTxRxCtrl = VP_OPTION_DEFAULT_PCM_TXRX_CNTRL;

    dtmfSpec = VP_OPTION_DEFAULT_DTMF_SPEC;

    parkMode.discTime = VP_OPTION_DEFAULT_PARK_MODE_DISC;
    parkMode.standbyTime = VP_OPTION_DEFAULT_PARK_MODE_STANDBY;

    pcmSigCtl.enable = VP_OPTION_DEFAULT_PCM_SIG_CTL_ENABLE;
    pcmSigCtl.ctlTimeslot = VP_OPTION_DEFAULT_PCM_SIG_CTL_CTLTS;
    pcmSigCtl.sigTimeslot = VP_OPTION_DEFAULT_PCM_SIG_CTL_SIGTS;

    autoLoopCond.select = VP_OPTION_DEFAULT_AUTO_LOOP_COND_SELECT;
    autoLoopCond.delay = VP_OPTION_DEFAULT_AUTO_LOOP_COND_DELAY;

    switcherCtrl = VP_OPTION_DEFAULT_SWITCHER_CTRL;

    gndFltProt.enable = VP_OPTION_DEFAULT_GND_FLT_PROT_ENABLE;
    gndFltProt.confirmTime = VP_OPTION_DEFAULT_GND_FLT_PROT_CONFIRMTIME;
    gndFltProt.pollTime = VP_OPTION_DEFAULT_GND_FLT_PROT_POLLTIME;
    gndFltProt.pollNum = VP_OPTION_DEFAULT_GND_FLT_PROT_POLLNUM;

#ifdef VP_CC_886_SERIES
    /* Ground fault protection needs different defaults on shared power
       supply configurations. */
    {
        VpDevCtxType *pDevCtxTemp;
        if (pDevCtx == VP_NULL) {
            pDevCtxTemp = pLineCtx->pDevCtx;
        } else {
            pDevCtxTemp = pDevCtx;
        }
        if (pDevCtxTemp->deviceType == VP_DEV_887_SERIES) {
            Vp886DeviceObjectType *pDevObj = pDevCtxTemp->pDevObj;
            if (pDevObj->stateInt & VP886_SHARED_SUPPLY) {
                gndFltProt.enable = VP_OPTION_DEFAULT_SHARED_GND_FLT_PROT_ENABLE;
                gndFltProt.confirmTime = VP_OPTION_DEFAULT_SHARED_GND_FLT_PROT_CONFIRMTIME;
                gndFltProt.pollTime = VP_OPTION_DEFAULT_SHARED_GND_FLT_PROT_POLLTIME;
                gndFltProt.pollNum = VP_OPTION_DEFAULT_SHARED_GND_FLT_PROT_POLLNUM;
            }
        }
    }
#endif

    adaptiveRinging.validMask = VP_OPTION_DEFAULT_ADAPTIVE_RINGING_VALIDMASK;
    adaptiveRinging.power = VP_OPTION_DEFAULT_ADAPTIVE_RINGING_POWER;
    adaptiveRinging.minVoltagePercent = VP_OPTION_DEFAULT_ADAPTIVE_RINGING_MIN_V_PCT;
    adaptiveRinging.mode = VP_OPTION_DEFAULT_ADAPTIVE_RINGING_MODE;

    VpMemSet(&dtmfMode, 0, sizeof(VpOptionDtmfModeType));
    dtmfMode.dtmfControlMode = VP_OPTION_DTMF_DECODE_OFF;
    dtmfMode.direction = VP_DIRECTION_US;

    ringTripConfirm = VP_OPTION_DEFAULT_RINGTRIP_CONFIRM;

    ringPhaseSync = VP_OPTION_DEFAULT_RING_PHASE_SYNC;

    highPassFilter = VP_OPTION_DEFAULT_HIGHPASS_FILTER;

    dtmfParams.validMask = VP_OPTION_DEFAULT_DTMF_PARAMS_VALIDMASK;
    dtmfParams.minDetect = VP_OPTION_DEFAULT_DTMF_PARAMS_MIN_DETECT;
    dtmfParams.rowToColLimit = VP_OPTION_DEFAULT_DTMF_PARAMS_ROW_TO_COL;
    dtmfParams.colToRowLimit = VP_OPTION_DEFAULT_DTMF_PARAMS_COL_TO_ROW;


    if (pDevCtx != VP_NULL) {
        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_PULSE,
            &pulseSpec);
        if (status == VP_STATUS_OPTION_NOT_SUPPORTED) {
            /* This device only supports VP_OPTION_ID_PULSE. */
            status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_PULSE,
                &linePulseSpec);
        }
        if ((status != VP_STATUS_SUCCESS) &&
            (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_PULSE2,
            &pulseSpec2);
        if ((status != VP_STATUS_SUCCESS) &&
            (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        /*
         * Some devices do not support AC/DC Fault detection, so setting the
         * critical fault may not be successful. However, all devices (known)
         * support thermal fault detection, so set that to the default
         */
        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_CRITICAL_FLT,
            &criticalFault);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            criticalFault.acFltDiscEn = FALSE;
            criticalFault.dcFltDiscEn = FALSE;
            status = pSetOption(VP_NULL, pDevCtx,
                VP_DEVICE_OPTION_ID_CRITICAL_FLT, &criticalFault);
            if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
                return status;
            }
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_PULSE_MODE,
            &pulseMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_CODEC, &codec);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_PCM_HWY, &pcmHwy);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_LOOPBACK, &loopBack);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_LINE_STATE,
            &lineState);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_EVENT_MASK,
            &eventMask);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_RING_CNTRL,
            &ringCtrl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_ZERO_CROSS,
            &zeroCross);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_PCM_TXRX_CNTRL,
            &pcmTxRxCtrl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_SWITCHER_CTRL,
            &switcherCtrl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_DTMF_SPEC, &dtmfSpec);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_PARK_MODE, &parkMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_DCFEED_SLOPE, &slopeRate);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_PCM_SIG_CTL, &pcmSigCtl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_LINESTATE_CTL_MODE, &linestateCtlMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_HOOK_DETECT_MODE, &hookDetectMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_AUTO_LOOP_COND, &autoLoopCond);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_GND_FLT_PROTECTION, &gndFltProt);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_DTMF_MODE, &dtmfMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_RINGTRIP_CONFIRM, &ringTripConfirm);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_RING_PHASE_SYNC, &ringPhaseSync);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING, &adaptiveRinging);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_HIGHPASS_FILTER, &highPassFilter);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(VP_NULL, pDevCtx, VP_OPTION_ID_DTMF_PARAMS, &dtmfParams);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

    }

    if (pLineCtx != VP_NULL) {
        /* Init only line level options */
        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_PULSE, &linePulseSpec);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_PULSE_MODE,
            &pulseMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_CODEC, &codec);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_PCM_HWY, &pcmHwy);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_LOOPBACK,
            &loopBack);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_LINE_STATE,
            &lineState);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_RING_CNTRL,
            &ringCtrl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_PCM_TXRX_CNTRL,
            &pcmTxRxCtrl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_SWITCHER_CTRL,
            &switcherCtrl);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_DTMF_SPEC, &dtmfSpec);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_DCFEED_SLOPE, &slopeRate);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_LINESTATE_CTL_MODE, &linestateCtlMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_HOOK_DETECT_MODE, &hookDetectMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_AUTO_LOOP_COND, &autoLoopCond);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_GND_FLT_PROTECTION, &gndFltProt);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }
        
        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_DTMF_MODE, &dtmfMode);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_RINGTRIP_CONFIRM, &ringTripConfirm);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_HIGHPASS_FILTER, &highPassFilter);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }

        status = pSetOption(pLineCtx, VP_NULL, VP_OPTION_ID_DTMF_PARAMS, &dtmfParams);
        if ((status != VP_STATUS_SUCCESS) && (status != VP_STATUS_OPTION_NOT_SUPPORTED)) {
            return status;
        }
    }

    return VP_STATUS_SUCCESS;
}
#endif /* 790 | 880 | 886 | 890 | 792 | 580 */

/* Code used for CSLAC only */
#if defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES)\
 || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES)

/*******************************************************************************
 * VpIsLowPowerTermType()
 *  Return TRUE for any VP-API-II Low Power Termination type passed.
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
bool
VpIsLowPowerTermType(
    VpTermType termType)
{
    if ((termType == VP_TERM_FXS_LOW_PWR) ||
        (termType == VP_TERM_FXS_ISOLATE_LP) ||
        (termType == VP_TERM_FXS_SPLITTER_LP)) {
        return TRUE;
    }
    return FALSE;
}

/*******************************************************************************
 * VpCSLACClearMPIBuffer()
 * This function clears the CSLAC MPI Buffer by writing 16 NO-Ops
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
#define VP_CSLAC_NO_OP_WRT  0x06
#define VP_CSLAC_EC1        0x01
void
VpCSLACClearMPIBuffer(
    VpDeviceIdType deviceId)
{
    uint8 mpiData[] = {
        VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT,
        VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT,
        VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT,
        VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT, VP_CSLAC_NO_OP_WRT
    };

    /* Perform NO_OPS to clear the MPI buffers */
    VpMpiCmdWrapper(deviceId, VP_CSLAC_EC1, VP_CSLAC_NO_OP_WRT, 16, mpiData);
}

/*******************************************************************************
 * VpCSLACIsProfileValid()
 * This function checks the validity of a profile passed into the API via
 * a pointer or an index.
 *
 * Arguments:
 *  tableSize       -   size of the profile table being checked
 *  profEntry       -   value of the profile entry being checked
 *  profType        -   type of profile that is being checked
 *  pProfTable      -   pointer to profile table pointers profType
 *  pProfileInput   -   pointer to the profile being checked
 *  pProfileRslt    -   pointer to the resulting profile
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
bool
VpCSLACIsProfileValid(
    VpProfileType       profType,
    int16               tableSize,
    uint16              profEntry,
    VpProfilePtrType    *pProfTable,
    VpProfilePtrType    pProfileInput,
    VpProfilePtrType    *pProfileRslt)
{
    int                 profIndex   = VpGetProfileIndex(pProfileInput);

    /* Fail if profile index is beyond legal table size */
    if (profIndex >= tableSize) {
        VP_ERROR(None, VP_NULL, ("IsProfileValid() - profIndex exceeds table size"));
        return FALSE;
    }

    /* Input profile is null, -- NULL is legal */
    if (pProfileInput == VP_PTABLE_NULL) {
        *pProfileRslt = VP_PTABLE_NULL;
        return TRUE;
    }

    if (profIndex < 0) {
        /* Is the input profile a vaild profile type? */
        if ( VpVerifyProfileType(profType, pProfileInput)) {
            *pProfileRslt = pProfileInput;
            return TRUE;
        }
    } else if (profIndex < tableSize) {
        /* Does the profile table contain a profile at the requested index? */
        if ((profEntry & (0x01 << profIndex))) {
            *pProfileRslt = pProfTable[profIndex];
            return TRUE;
        }
    }

    VP_ERROR(None, VP_NULL, ("IsProfileValid() - invalid profile"));
    return FALSE;
} /* VpCSLACIsProfileValid */

bool
VpCSLACSetTimer(
    uint16 *pTimer,
    uint16 newValue)
{
    if ((*pTimer & VP_ACTIVATE_TIMER) && ((*pTimer & ~VP_ACTIVATE_TIMER) >= newValue)) {
        /* Timer is already active and longer than new time. Do nothing. */
        return FALSE;
    }

    *pTimer = (newValue | VP_ACTIVATE_TIMER);
    return TRUE;
}
#if defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) || defined(VP_CC_890_SERIES)
int16
VpConvertToInt16(
    uint8 *dataPtr)
{
    return (int16)((((uint16)dataPtr[0] << 8) & 0xFF00) | (dataPtr[1] & 0xFF));
}

int32
VpConvertToInt32(
    uint8 *dataPtr)
{
    return (int32)((((uint32)dataPtr[0] << 24) & 0xFF000000)
                 | (((uint32)dataPtr[1] << 16) & 0x00FF0000)
                 | (((uint32)dataPtr[2] << 8) & 0x0000FF00)
                 | ((uint32)dataPtr[3] & 0xFF));
}

int32
VpRoundedDivide(
    int32 x,
    int32 y)
{
    if (y == 0) {
        VP_ERROR(None, VP_NULL, ("Divide by zero"));
        return 0;
    }

    if (((x > 0) && (y > 0)) || ((x <= 0) && (y <= 0))) {
        return (x + (y / 2)) / y;
    } else {
        return (x - (y / 2)) / y;
    }
}

#if defined (VP880_FXS_SUPPORT) || defined (VP890_FXS_SUPPORT)
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
/**
 * VpCLSACHighGainMode()
 *  This function puts the line into and takes it out of High Gain mode for VE880 and VE890
 *  devices. It is used to generate Special Howler Tones.
 *
 *  High Gain mode is done by forcing the Signal Generator to output at Ringing Levels. Since DC
 *  feed does not operate normally in this condition we also have to force Tip/Ring bias (ICR1).
 *  The maximum output level with the coefficients provided here is ~15dBm into 600ohm load (target
 *  is 14.7dBm just to allow for some tolerance while still ensuring < 15dBm max output per UK
 *  Howler Tone Specifications).
 *
 *  When entering High Gain Mode, this function saves off the cached register values in the line
 *  object for those registers modified in the High Gain Setting procedure. Values that are not
 *  already cached in the line objects are (and must be) read in this function. When exiting High
 *  Gain Mode this function writes back the previously saved values, regardless of whether or not
 *  these same values have been changed in the meantime. THIS MEANS that when exiting High Gain
 *  mode, the very last thing that must have been set on the line is to Enter High Gain Mode. There
 *  can be no other state changes in-between.
 */
/**************************************************************************************************
 * IMPORTANT!!!! The device level access required to Enter/Exit High Gain conditions must be 100%
 * complete when this function returns. This function is called in context of "Set Line State"
 * function which itself may start state one more more state machine/timer procedures. In case of
 * parallel procedurs running that affect the same registers/bits, the timer management code will
 * have to be made much more complex. In the current version of the API, there is no need to delay
 * Howler configuration (enter or exit) write operations.
 **************************************************************************************************/
void
VpCLSACHighGainMode(
    VpLineCtxType *pLineCtx,
    bool highGainMode)
{
    void *pLineObj = pLineCtx->pLineObj;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDeviceType deviceType = pDevCtx->deviceType;
    void *pDevObj = pDevCtx->pDevObj;

    VpDeviceIdType deviceId;
    uint8 ecVal;

    /* In case cadencing is not supported, initialize this to a non-Howler Tone value */
    uint8 toneType = 0;

    VpHowlerModeCachedValues *pHowlerModeCache;

    uint8 mpiIndex = 0;
    uint8 mpiBuffer[11 + VP_CSLAC_DC_FEED_LEN + VP_CSLAC_ICR1_LEN + VP_CSLAC_ICR4_LEN
                       + VP_CSLAC_ICR2_LEN + VP_CSLAC_ICR3_LEN + VP_CSLAC_VP_GAIN_LEN
                       + VP_CSLAC_GR_LEN + VP_CSLAC_DISN_LEN + VP_CSLAC_R_LEN
                       + VP_CSLAC_OP_FUNC_LEN + VP_CSLAC_SW_REG_LEN];

    /*
     * These are write only. Values in register currently will be overwritten when entering
     * High Gain Mode
     */
    uint8 dcFeedData[VP_CSLAC_DC_FEED_LEN] = {0x49, 0x16};
    uint8 vpGainData[VP_CSLAC_VP_GAIN_LEN] = {0x18};
    uint8 disnData[VP_CSLAC_DISN_LEN] = {0x00};

    /*
     * With flat R-Filter coefficients, provides 14.5dBm max signal on the line. Used for UK
     * BTRN Version 15, NTT, and normal High Gain mode. All other modes when detected will modify
     * the content of grData
     */
    uint8 grData[VP_CSLAC_GR_LEN] = {0x9F, 0xA1};

    /*
     * UK Draft 960-G has a non-flat Holwer Tone Frequency response. All other types of Howler Tone,
     * including UK Version 15, is a flat frequency response.
     *
     * In case of non-flat response, the loss at 800Hz is 3dB more than the loss at 2500Hz
     */
    uint8 rValue[VP_CSLAC_R_LEN] =
        {0x7D, 0xD0, 0x01, 0x11, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90};

    /* These are read/modify/write */
    uint8 icr1Data[VP_CSLAC_ICR1_LEN];
    uint8 icr4Data[VP_CSLAC_ICR4_LEN];
    uint8 icr2Data[VP_CSLAC_ICR2_LEN];
    uint8 icr3Data[VP_CSLAC_ICR3_LEN];
    uint8 opFunc[VP_CSLAC_OP_FUNC_LEN]; /* Only to disable Z and B Filters */
    uint8 swReg[VP_CSLAC_SW_REG_LEN];   /* Used to set Tracking floor voltage to -30V */

    bool isAbs = FALSE;
    bool rFilterChangeOnly = FALSE;

/*
 * Hopefully, these names and comments provide enough description as to what
 * this function is trying to accomplish at the register level.
 * See the VoicePort Command Set for more details of these registers and bits.
 */
#define VP_CSLAC_ICR1_WRT                   (0xEC)
#define VP_CSLAC_ICR1_TIP_BIAS_OVERRIDE     (0xF0)
#define VP_CSLAC_ICR1_RING_BIAS_OVERRIDE    (0x0F)
#define VP_CSLAC_ICR1_TIP_BIAS_CTRL_INDEX   (0x00)
#define VP_CSLAC_ICR1_RING_BIAS_CTRL_INDEX  (0x02)
#define VP_CSLAC_ICR1_BATTERY_CTRL_INDEX    (0x02)
#define VP_CSLAC_ICR1_BATTERY_CTRL_MASK     (0x30)

#define VP_CSLAC_ICR2_WRT               (0xEE)
#define VP_CSLAC_ICR2_DAC_CTRL_INDEX    (0x00)
#define VP_CSLAC_ICR2_DAC_RING_LEVELS   (0x10)
#define VP_CSLAC_ICR2_SPEEDUP_INDEX     (0x02)
#define VP_CSLAC_ICR2_METALLIC_SPEEDUP  (0x80)
#define VP_CSLAC_ICR2_RINGING_TC        (0x01)

#define VP_CSLAC_ICR3_WRT               (0xF2)
#define VP_CSLAC_ICR3_LINE_CTRL_INDEX   (0x00)
#define VP_CSLAC_ICR3_25V_LOOP_LIMIT    (0x10)
#define VP_CSLAC_ICR3_VP_CFG_INDEX      (0x02)
#define VP_CSLAC_ICR3_DC_FEED_CONNECT   (0x02)

#define VP_CSLAC_ICR4_WRT               (0xF4)
#define VP_CSLAC_ICR4_SMALL_SIGNAL_CTRL (0x80)
#define VP_CSLAC_ICR4_AISN_CTRL         (0x10)

#define VP_CSLAC_RD_BIT (0x01)

#define VP_CSLAC_DC_FEED_WRT    (0xC6)
#define VP_CSLAC_DC_FEED_RD     (VP_CSLAC_DC_FEED_WRT | VP_CSLAC_RD_BIT)

#define VP_CSLAC_VP_GAIN_WRT    (0x50)
#define VP_CSLAC_VP_GAIN_RD     (VP_CSLAC_VP_GAIN_WRT | VP_CSLAC_RD_BIT)

#define VP_CSLAC_GR_WRT         (0x82)
#define VP_CSLAC_GR_RD          (VP_CSLAC_GR_WRT | VP_CSLAC_RD_BIT)

#define VP_CSLAC_R_WRT         (0x8A)
#define VP_CSLAC_R_RD          (VP_CSLAC_R_WRT | VP_CSLAC_RD_BIT)

#define VP_CSLAC_DISN_WRT       (0xCA)
#define VP_CSLAC_DISN_RD        (VP_CSLAC_DISN_WRT | VP_CSLAC_RD_BIT)

#define VP_CSLAC_OP_FUNCT_WRT           (0x60)
#define VP_CSLAC_OP_FUNCT_RD            (VP_CSLAC_OP_FUNCT_WRT | VP_CSLAC_RD_BIT)

    /**< VP_CSLAC_OP_FUNCT_Z_B_DISABLE
     * Lower 2-bits are used for Z-Filters (0x02) and B-Filters (0x01). When = '0' will disable
     * these filters. This should be used as 'AND' operation to disable Z/B-Filters.
     */
#define VP_CSLAC_OP_FUNCT_Z_ENABLE      (0x02)
#define VP_CSLAC_OP_FUNCT_B_ENABLE      (0x01)
#define VP_CSLAC_OP_FUNCT_Z_B_ENABLE    (VP_CSLAC_OP_FUNCT_Z_ENABLE | VP_CSLAC_OP_FUNCT_B_ENABLE)

#define VP_CSLAC_SW_REG_WRT             (0xE4)
#define VP_CSLAC_SW_REG_RD              (VP_CSLAC_SW_REG_WRT | VP_CSLAC_RD_BIT)
#define VP_CSLAC_FLOOR_VOLTAGE_MASK     0x1F

    switch (deviceType) {
#if defined (VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT)
        case VP_DEV_880_SERIES:
            deviceId = ((Vp880DeviceObjectType *)pDevObj)->deviceId;
            ecVal = ((Vp880LineObjectType *)pLineObj)->ecVal;
            pHowlerModeCache = &((Vp880LineObjectType *)pLineObj)->howlerModeCache;
#ifdef VP_CSLAC_SEQ_EN
            toneType = ((Vp880LineObjectType *)pLineObj)->cadence.toneType;
#endif

            /* Cache the ICR values if entering High Gain mode and not already in High Gain Mode */
            if ((highGainMode) && (!(pHowlerModeCache->isInHowlerMode))) {
                /****************************************************************************
                 * Register Cache Process >> START                                          *
                 *   - Copy directly from the line object what we can. This keeps MPI       *
                 *     access to a minimum. We'll read from the silicon for those registers *
                 *     that are not maintained in the line object.                          *
                 ****************************************************************************/
                VpMemCpy(pHowlerModeCache->icr1Reg,
                         ((Vp880LineObjectType *)pLineObj)->icr1Values, VP_CSLAC_ICR1_LEN);
                VpMemCpy(pHowlerModeCache->icr2Reg,
                         ((Vp880LineObjectType *)pLineObj)->icr2Values, VP_CSLAC_ICR2_LEN);
                VpMemCpy(pHowlerModeCache->icr3Reg,
                         ((Vp880LineObjectType *)pLineObj)->icr3Values, VP_CSLAC_ICR3_LEN);
                VpMemCpy(pHowlerModeCache->icr4Reg,
                         ((Vp880LineObjectType *)pLineObj)->icr4Values, VP_CSLAC_ICR4_LEN);
#ifdef VP880_TRACKER_SUPPORT
                VpMemCpy(pHowlerModeCache->swReg,
                         ((Vp880DeviceObjectType *)pDevObj)->swParamsCache, VP_CSLAC_SW_REG_LEN);
#endif
            }
#ifdef VP880_ABS_SUPPORT
            if (((((Vp880DeviceObjectType *)pDevObj)->stateInt) & VP880_IS_ABS) == VP880_IS_ABS) {
                isAbs = TRUE;
            }
#endif
            break;
#endif

#if defined (VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT)
        case VP_DEV_890_SERIES:
            deviceId = ((Vp890DeviceObjectType *)pDevObj)->deviceId;
            ecVal = ((Vp890LineObjectType *)pLineObj)->ecVal;
            pHowlerModeCache = &((Vp890LineObjectType *)pLineObj)->howlerModeCache;
#ifdef VP_CSLAC_SEQ_EN
            toneType = ((Vp890LineObjectType *)pLineObj)->cadence.toneType;
#endif

            /* Cache the ICR values if entering High Gain mode and not already in High Gain Mode */
            if ((highGainMode) && (!(pHowlerModeCache->isInHowlerMode))) {
                /****************************************************************************
                 * Register Cache Process >> START                                          *
                 *   - Copy directly from the line object what we can. This keeps MPI       *
                 *     access to a minimum. We'll read from the silicon for those registers *
                 *     that are not maintained in the line object.                          *
                 ****************************************************************************/
                VpMemCpy(pHowlerModeCache->icr1Reg,
                         ((Vp890LineObjectType *)pLineObj)->icr1Values, VP_CSLAC_ICR1_LEN);
                VpMemCpy(pHowlerModeCache->icr2Reg,
                         ((Vp890LineObjectType *)pLineObj)->icr2Values, VP_CSLAC_ICR2_LEN);
                VpMemCpy(pHowlerModeCache->icr3Reg,
                         ((Vp890LineObjectType *)pLineObj)->icr3Values, VP_CSLAC_ICR3_LEN);
                VpMemCpy(pHowlerModeCache->icr4Reg,
                         ((Vp890LineObjectType *)pLineObj)->icr4Values, VP_CSLAC_ICR4_LEN);
                VpMemCpy(pHowlerModeCache->swReg,
                         ((Vp890DeviceObjectType *)pDevObj)->swParamsCache, VP_CSLAC_SW_REG_LEN);
            }
            break;
#endif

        default:
            return;
    }

    if (highGainMode) { /* Entering High Gain Mode */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Entering High Gain State"));

        /* If not already in High Gain Mode cache content of all registers being modified */
        if (!(pHowlerModeCache->isInHowlerMode)) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Not Previously in High Gain State"));

            /*
             * Set the flag that tells the API the line is in High Gain state. Doing this
             * prevents incorrectly reading/caching register values used for High Gain mode.
             */
            pHowlerModeCache->isInHowlerMode = TRUE;

            /****************************************************************************
             * Register Cache Process >> CONTINUED                                      *
             *   - Continue from where we left off above. Read/cache the registers that *
             *     are not maintained in the line object.                               *
             ****************************************************************************/
            VpMpiCmdWrapper(deviceId, ecVal, VP_CSLAC_DC_FEED_RD, VP_CSLAC_DC_FEED_LEN,
                pHowlerModeCache->dcFeed);

            VpMpiCmdWrapper(deviceId, ecVal, VP_CSLAC_VP_GAIN_RD, VP_CSLAC_VP_GAIN_LEN,
                pHowlerModeCache->digitalRxLoss);

            VpMpiCmdWrapper(deviceId, ecVal, VP_CSLAC_GR_RD, VP_CSLAC_GR_LEN,
                pHowlerModeCache->grValue);

            VpMpiCmdWrapper(deviceId, ecVal, VP_CSLAC_DISN_RD, VP_CSLAC_DISN_LEN,
                pHowlerModeCache->disn);

            VpMpiCmdWrapper(deviceId, ecVal, VP_CSLAC_R_RD, VP_CSLAC_R_LEN,
                pHowlerModeCache->rValue);

            VpMpiCmdWrapper(deviceId, ecVal, VP_CSLAC_OP_FUNCT_RD, VP_CSLAC_OP_FUNC_LEN,
                pHowlerModeCache->opFunc);
            /****************************************************************************
             * Register Cache Process >> END                                            *
             *   - At this point, all registers that need to be restored when exiting   *
             *     High Gain State have been cached. Start configuring for new values   *
             ****************************************************************************/

            /****************************************************************************
             * High Gain Configuration Process >> START                                 *
             *   - Prepare the stack values with content to write to the silicon to put *
             *     the line in the desired High Gain Mode.                              *
             *   - All registers except R Filter and GR are set here. The R/GR values   *
             *     are treated differently because they are set based on the type of    *
             *     Howler Tone (if any) being generated.                                *
             ****************************************************************************/
            /* Force Tracker Floor Voltage to -30V */
            if (!(isAbs)) {
                VpMemCpy(swReg, pHowlerModeCache->swReg, VP_CSLAC_SW_REG_LEN);
                swReg[1] &= ~VP_CSLAC_FLOOR_VOLTAGE_MASK;
                swReg[1] |= 0x05;
            }

            /* Force Line Bias control and set bias values. */
            VpMemCpy(icr1Data, pHowlerModeCache->icr1Reg, VP_CSLAC_ICR1_LEN);

            icr1Data[VP_CSLAC_ICR1_TIP_BIAS_CTRL_INDEX] |= VP_CSLAC_ICR1_TIP_BIAS_OVERRIDE;
            icr1Data[VP_CSLAC_ICR1_TIP_BIAS_CTRL_INDEX+1] &= ~VP_CSLAC_ICR1_TIP_BIAS_OVERRIDE;
            icr1Data[VP_CSLAC_ICR1_TIP_BIAS_CTRL_INDEX+1] |= 0xC0;

            icr1Data[VP_CSLAC_ICR1_RING_BIAS_CTRL_INDEX] |= VP_CSLAC_ICR1_RING_BIAS_OVERRIDE;
            icr1Data[VP_CSLAC_ICR1_RING_BIAS_CTRL_INDEX+1] &= ~VP_CSLAC_ICR1_RING_BIAS_OVERRIDE;
            icr1Data[VP_CSLAC_ICR1_RING_BIAS_CTRL_INDEX+1] |= 0x0C;

            /* Force VBL for ABS only. */
            if (isAbs) {
                icr1Data[VP_CSLAC_ICR1_BATTERY_CTRL_INDEX] |= VP_CSLAC_ICR1_BATTERY_CTRL_MASK;
                icr1Data[VP_CSLAC_ICR1_BATTERY_CTRL_INDEX+1] &= ~VP_CSLAC_ICR1_BATTERY_CTRL_MASK;
            }

            /* Enable ringing current limit range */
            VpMemCpy(icr2Data, pHowlerModeCache->icr2Reg, VP_CSLAC_ICR2_LEN);
            icr2Data[VP_CSLAC_ICR2_DAC_CTRL_INDEX] |= VP_CSLAC_ICR2_DAC_RING_LEVELS;
            icr2Data[VP_CSLAC_ICR2_DAC_CTRL_INDEX+1] |= VP_CSLAC_ICR2_DAC_RING_LEVELS;

            /* Enable Metallic Speedup and set for Ringing Time Constant */
            icr2Data[VP_CSLAC_ICR2_SPEEDUP_INDEX] |=
                (VP_CSLAC_ICR2_METALLIC_SPEEDUP | VP_CSLAC_ICR2_RINGING_TC);
            icr2Data[VP_CSLAC_ICR2_SPEEDUP_INDEX+1] |=
                (VP_CSLAC_ICR2_METALLIC_SPEEDUP | VP_CSLAC_ICR2_RINGING_TC);

            /* The following steps on ICR3 Enable Signal Generator via DC Feed (Ringing Mode) */
            VpMemCpy(icr3Data, pHowlerModeCache->icr3Reg, VP_CSLAC_ICR3_LEN);
            /* Force Saturation Limit of the Longitudinal loop to -30V */
            icr3Data[VP_CSLAC_ICR3_LINE_CTRL_INDEX] |= VP_CSLAC_ICR3_25V_LOOP_LIMIT;
            icr3Data[VP_CSLAC_ICR3_LINE_CTRL_INDEX+1] &= ~VP_CSLAC_ICR3_25V_LOOP_LIMIT;

            /* Connect the Voice Path output to the DC Feed Path */
            icr3Data[VP_CSLAC_ICR3_VP_CFG_INDEX] |= VP_CSLAC_ICR3_DC_FEED_CONNECT;
            icr3Data[VP_CSLAC_ICR3_VP_CFG_INDEX+1] |= VP_CSLAC_ICR3_DC_FEED_CONNECT;

            /* 
             * Force the Longitudinal Battery Sense to Low Battery Independent of 
             * Battery Switch in ICR1 to prevent substate transitions (#8319) ABS only.
             */
            if (isAbs) {
                icr3Data[VP_CSLAC_ICR3_VP_CFG_INDEX] |= 0x38;
                icr3Data[VP_CSLAC_ICR3_VP_CFG_INDEX+1] &= ~0x30;
                icr3Data[VP_CSLAC_ICR3_VP_CFG_INDEX+1] |= 0x08;
            }

            /* Turn off AISN and Metallic Feed */
            VpMemCpy(icr4Data, pHowlerModeCache->icr4Reg, VP_CSLAC_ICR4_LEN);
            icr4Data[0] |= (VP_CSLAC_ICR4_SMALL_SIGNAL_CTRL | VP_CSLAC_ICR4_AISN_CTRL);
            icr4Data[1] &= (uint8)(~(VP_CSLAC_ICR4_SMALL_SIGNAL_CTRL | VP_CSLAC_ICR4_AISN_CTRL));

            /*
             * Disable the Z and B Filters. If this isn't done, the AC loop could oscillate and
             * cause significant Tip/Ring noise to the point of hook detection instability.
             *
             * Typecasting is being done only to avoid compiler warnings. Some compilers would
             * convert a bit-wise inversion of multiple string constants to int which could then
             * generate a compiler warning.
             */
            opFunc[0] = pHowlerModeCache->opFunc[0];
            opFunc[0] &= (uint8)(~VP_CSLAC_OP_FUNCT_Z_B_ENABLE);

            /* DC Feed is being completely changed. No need to read from the silicon */
            /* GR is being completely changed. No need to read from the silicon */
            /* R FIlter is being completely changed. No need to read from the silicon */
            /* DISN is being completely changed. No need to read from the silicon */
            /* Voice Path Gain is being completely changed. No need to read from the silicon */
        } else {
            /*
             * Already in High Gain Mode. We may only change the Rx Path frequency response
             * and/or Gain.
             */
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Previously in High Gain State"));

            /*
             * Set this flag in order to minimize MPI traffic. This condition occurs when the
             * application calls VpSetLineTone() first (with a special Howler Tone enabled) and
             * then sets the line to High Gain state.
             */
            rFilterChangeOnly = TRUE;
        }

        /****************************************************************************
         * High Gain Configuration Process >> CONTINUED                             *
         *   - Prepare the R/GR values as mentioned above (in >> START comments)    *
         *   - If nothing is changed, the total D-A gain is ~14dB and the frequency *
         *     response is flat. This has to change for UK Draft 960-G (non-flat    *
         *     frequency response) and AUS (lower D-A gain to ~10dB)                *
         ****************************************************************************/
        if (toneType == VP_CSLAC_UK_HOWLER_TONE_DRAFT_G) {
            /*
             * These set of coefficients with GR = [0x26, 0x32] provides ~14.5dBm signal on the line
             * and provide a flat frequency response from 800Hz to 3200Hz (per UK 960 Draft-G)
             */
            uint8 rValueUkDraft960[VP_CSLAC_R_LEN] =
                {0x2D, 0xC0, 0x22, 0xC0, 0x43, 0x61, 0xBD, 0xA8, 0x3A, 0xA1, 0x43, 0x2A, 0x22, 0x24};

            /*
             * With sloped R-Filter coefficients, gr has to be adjusted from normal values in
             * order to compensate for the gain at the high frequencies. The overall level of
             * gr is reduced by approximately 2.5dB
             */
            grData[0] = 0x26;
            grData[1] = 0x32;

            /* UK Frequency Response Specific R-Filter. */
            VpMemCpy(rValue, rValueUkDraft960, VP_CSLAC_R_LEN);
        } else if (toneType == VP_CSLAC_UK_HOWLER_TONE_VER15) {
            /*
             * These set of coefficients with GR = [0x26, 0x32] provides 12dBm signal on the line
             * and provide a flat frequency response from 800Hz to 3200Hz (per UK 960 Version 15)
             */
            uint8 rValueUkVer15[VP_CSLAC_R_LEN] =
                {0x7D, 0xD0, 0x01, 0x11, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90};

            /* UK Flat Frequency Response. */
            VpMemCpy(rValue, rValueUkVer15, VP_CSLAC_R_LEN);
        } else if (toneType == VP_CSLAC_AUS_HOWLER_TONE) {
            /*
             * With flat R-Filter coefficients the standard gr coefficients provides 14.5dBm on the
             * line. Australia Howler is nominal +10dBm maximum. So we have to reduce gr by ~4.5dB.
             */
            grData[0] = 0xAB;
            grData[1] = 0xB2;
        } else {
            /*
             * All other conditions where flat frequency response and 14.5dBm maximum signal on
             * the line is acceptable. Since the stack variables are initialized with these
             * settings, there's nothing else to do in most cases.
             */
        }

        /****************************************************************************
         * High Gain Configuration Process >> END                                   *
         *   - At this point the following is TRUE:                                 *
         *        IF (line was not previously in High Gain Mode)                    *
         *        THEN (all variables on the stack are programmed with desired      *
         *              values for putting the line in High Gain Mode)              *
         *        ELSE (line was previously in High Gain Mode)                      *
         *        THEN (only R and GR variables on the stack are programmed with    *
         *              desired values for adjusting the current High Gain mode to  *
         *              the High Gain mode specified by toneType value in the line  *
         *              object cadence member).                                     *
         *                                                                          *
         *    NOTE: The cadence memeber exists only in the line object if the API   *
         *    is compiled with VP_CSLAC_SEQ_EN set to #define. This function will   *
         *    set the default values (if cadence member doesn't exist) to provde    *
         *    +14dBm level and flat frequency response.                             *
         ****************************************************************************/
    } else {    /* Exiting High Gain Mode */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Exiting High Gain State"));
        if (!(pHowlerModeCache->isInHowlerMode)) {
            /* Already out of High Gain Mode (or never entered). Nothing else to do */
            return;
        }
        /*
         * Clear the flag that tells the API the line is in High Gain state. Doing this
         * allows future entrance into High Gain mode to cache and modify all of the
         * registers needed to implement High Gain mode.
         */
        pHowlerModeCache->isInHowlerMode = FALSE;

        /****************************************************************************
         * Non-High Gain Configuration Process >> START                             *
         *   - Prepare the stack variables with content from the silicon and line   *
         *     object as it was prior to entering High Gain Mode.                   *
         *                                                                          *
         *    NOTE: VpMemCpy() is not used in cases where only one byte is being    *
         *    copied. Done purely from an efficiencly perspective, no other reason. *
         ****************************************************************************/
        VpMemCpy(swReg, pHowlerModeCache->swReg, VP_CSLAC_SW_REG_LEN);
        VpMemCpy(icr1Data, pHowlerModeCache->icr1Reg, VP_CSLAC_ICR1_LEN);
        VpMemCpy(icr2Data, pHowlerModeCache->icr2Reg, VP_CSLAC_ICR2_LEN);
        VpMemCpy(icr3Data, pHowlerModeCache->icr3Reg, VP_CSLAC_ICR3_LEN);
        VpMemCpy(icr4Data, pHowlerModeCache->icr4Reg, VP_CSLAC_ICR4_LEN);
        VpMemCpy(dcFeedData, pHowlerModeCache->dcFeed, VP_CSLAC_DC_FEED_LEN);
        vpGainData[0] = pHowlerModeCache->digitalRxLoss[0];
        VpMemCpy(grData, pHowlerModeCache->grValue, VP_CSLAC_GR_LEN);
        VpMemCpy(rValue, pHowlerModeCache->rValue, VP_CSLAC_R_LEN);
        disnData[0] = pHowlerModeCache->disn[0];
        opFunc[0] = pHowlerModeCache->opFunc[0];
    }

    /****************************************************************************
     * MPI Buffer Build Process >> START                                        *
     *   - Prepare the MPI Buffer to reconfigure the line. The order should be  *
     *     such that the gain at any point in time does not exceed the final    *
     *     high gain conditions. This means:                                    *
     *                                                                          *
     *          - When entering High Gain Mode, set the filters first in order  *
     *            to pre-reduce the D-A gain.                                   *
     *          - When exiting High Gain Mode, set the filters last in order to *
     *            avoid having the line in the high gain conditions with normal *
     *            (higher gain) coefficients. We also don't want to create the  *
     *            oscillation scenario where the line is in a non-normal AC     *
     *            impedance mode with normal impedance setting coefficients     *
     *            enabled.                                                      *
     ****************************************************************************/
    /* If Entering High Gain Mode, program the filter coefficients first */
    if (highGainMode) {
        /* Set the supply only if Tracker AND if we were not previously in High Gain Mode */
        if((!(isAbs)) && (!(rFilterChangeOnly))) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Howler Mode %d: SW Wrt 0x%02X 0x%02X 0x%02X to EC 0x%02X",
                 highGainMode, swReg[0], swReg[1], swReg[2], ecVal));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_SW_REG_WRT,
                VP_CSLAC_SW_REG_LEN, swReg);
        }

        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Howler Mode %d:GR Wrt 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, grData[0], grData[1], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_GR_WRT,
            VP_CSLAC_GR_LEN, grData);
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_R_WRT,
            VP_CSLAC_R_LEN, rValue);
    }

    if (!(rFilterChangeOnly)) {
        /* If Entering High Gain Mode, continue with filter coefficients first */
        if (highGainMode) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Howler Mode %d: VP Gain Wrt 0x%02X to EC 0x%02X",
                 highGainMode, vpGainData[0], ecVal));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_VP_GAIN_WRT,
                VP_CSLAC_VP_GAIN_LEN, vpGainData);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Howler Mode %d:DISN Wrt 0x%02X to EC 0x%02X", highGainMode, disnData[0], ecVal));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_DISN_WRT,
                VP_CSLAC_DISN_LEN, disnData);

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_OP_FUNCT_WRT,
                VP_CSLAC_OP_FUNC_LEN, opFunc);
        }

        /* Fill in the control bits that set the line to High Gain Mode */
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Howler Mode %d: DC Feed Wrt 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, dcFeedData[0], dcFeedData[1], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_DC_FEED_WRT,
            VP_CSLAC_DC_FEED_LEN, dcFeedData);

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Howler Mode %d: ICR1 Wrt 0x%02X 0x%02X 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, icr1Data[0], icr1Data[1], icr1Data[2], icr1Data[3], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_ICR1_WRT,
            VP_CSLAC_ICR1_LEN, icr1Data);

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Howler Mode %d: ICR4 Wrt 0x%02X 0x%02X 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, icr4Data[0], icr4Data[1], icr4Data[2], icr4Data[3], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_ICR4_WRT,
            VP_CSLAC_ICR4_LEN, icr4Data);

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Howler Mode %d: ICR2 Wrt 0x%02X 0x%02X 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, icr2Data[0], icr2Data[1], icr2Data[2], icr2Data[3], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_ICR2_WRT,
            VP_CSLAC_ICR2_LEN, icr2Data);

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Howler Mode %d: ICR3 Wrt 0x%02X 0x%02X 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, icr3Data[0], icr3Data[1], icr3Data[2], icr3Data[3], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_ICR3_WRT,
            VP_CSLAC_ICR3_LEN, icr3Data);

        /* If Exiting High Gain Mode, filter coefficients are programmed last */
        if (!(highGainMode)) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Howler Mode %d: VP Gain Wrt 0x%02X to EC 0x%02X",
                 highGainMode, vpGainData[0], ecVal));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_VP_GAIN_WRT,
                VP_CSLAC_VP_GAIN_LEN, vpGainData);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Howler Mode %d:DISN Wrt 0x%02X to EC 0x%02X", highGainMode, disnData[0], ecVal));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_DISN_WRT,
                VP_CSLAC_DISN_LEN, disnData);

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_OP_FUNCT_WRT,
                VP_CSLAC_OP_FUNC_LEN, opFunc);
        }
    }

    /* If Exiting High Gain Mode, program the filter coefficients last */
    if (!(highGainMode)) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Howler Mode %d:GR Wrt 0x%02X 0x%02X to EC 0x%02X",
             highGainMode, grData[0], grData[1], ecVal));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_GR_WRT,
            VP_CSLAC_GR_LEN, grData);
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_R_WRT,
            VP_CSLAC_R_LEN, rValue);

        if(!(isAbs)) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Howler Mode %d: SW Wrt 0x%02X 0x%02X 0x%02X to EC 0x%02X",
                 highGainMode, swReg[0], swReg[1], swReg[2], ecVal));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP_CSLAC_SW_REG_WRT,
                VP_CSLAC_SW_REG_LEN, swReg);
        }
    }
    /****************************************************************************
     * MPI Buffer Build Process >> END                                          *
     *    - The mpiBuffer[] is not configured with the data required to change  *
     *      to or from the High Gain mode with specific command order as needed *
     *      Only the MPI write operation is needed.                             *
     ****************************************************************************/
    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
    return;
}

/**
 * VpCSLACHowlerStateMgmt()
 *  This function supports transitions into and out of VP_LINE_HOWLER and VP_LINE_HOWLER_POLREV
 * line states for the VE880 and VE890 API. It only performs the necessary error checking for the
 * state the line is currently in prior to entering Howler state, but will modify the device
 * when exiting Howler state.
 *
 *  To minimize the complexity of the API, in particular Low Power Mode, this implementation
 * requires the line to be in a normal off-hook state (VP_LINE_ACTIVE, VP_LINE_TALK, or the PolRev
 * conditions of those states) prior to entering Howler State. It is too complex and should not
 * be necessary to support entering Howler state from an on-hook condition/state such as Standby
 * or Disconnect (since Howler Tones are used to notify a customer of a constant off-hook line).
 *
 *  When entering Howler State (or Howler PolRev), the calling "Set Line State" function checks
 * the return value from this function to be sure the transition is legal. If it is, then all other
 * line register values will be modified first before changing the line to the High Gain Mode.
 *
 *  When exiting Howler State (or Howler PolRev), the calling "Set Line State" function calls this
 * function first before setting the line (immediately) to the target state.
 */
/*************************************************************************************************
 * IMPORTANT!!!! The operations required to exit Howler conditions must be done immediately.
 * Adding delay to these operations (i.e., implementing as a state machine managed by timers) can
 * cause undesireble values being programmed in the device. This is due in large part to the use
 * of state machines in handling of certain API line state transitions. These state machines keep
 * line and battery transients to a minimum, but can be negatively affected if parallel timers are
 * running that affect the same registers.
 *************************************************************************************************/
VpStatusType
VpCSLACHowlerStateMgmt(
    VpLineCtxType *pLineCtx,
    VpLineStateType fromState,
    VpLineStateType toState,
    VpLineStateType statePriorToHowler)
{
    /* If going to the Howler State, make sure it's from a legal current state */
    if ((toState == VP_LINE_HOWLER) || (toState == VP_LINE_HOWLER_POLREV)) {
        /*
         * Don't allow repeated calls to Howler State or Howler State PolRev. It complicates
         * the code because it allows SetLineState to operate with a from/to state that is
         * both howler (not as designed). Upper level code will allow same state changes
         * because it then has nothing to do and allows that as success, so this needs to
         * only catch polarity reversals between howler states and treat it as an illegal
         * state change.
         */
        switch (fromState) {
            case VP_LINE_ACTIVE:
            case VP_LINE_TALK:
            case VP_LINE_ACTIVE_POLREV:
            case VP_LINE_TALK_POLREV:
                break;

            /* Note in the default case included are VP_LINE_HOWLER and VP_LINE_HOWLER_POLREV */
            default:
                return VP_STATUS_INVALID_ARG;
        }
    } else { /* Going to a non-Howler state. This is more restrictive than going to Howler */
        /*
         * If coming from Howler State it must be to the state prior to entering Howler. The "to"
         * condition for trying to enter from/to any combination of Howler States is taken care of
         * above. This only needs to verify that if we're in a Howler State already that we're
         * going back to the state we were in prior to entering Howler state.
         */
        if ((fromState == VP_LINE_HOWLER) || (fromState == VP_LINE_HOWLER_POLREV)) {
            if (toState != statePriorToHowler) {
                return VP_STATUS_INVALID_ARG;
            }
            VpCLSACHighGainMode(pLineCtx, FALSE);
        }
    }
    return VP_STATUS_SUCCESS;
}
#endif

/**
 * VpCSLACIsSupportedFxsState()
 *  This function checks to see if the state passed is a supproted FXS state of
 * the VE880 and VE890 API
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  None.
 */
bool
VpCSLACIsSupportedFxsState(
    VpDeviceType deviceType,
    VpLineStateType state)
{
    switch (state) {
        case VP_LINE_STANDBY:
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
        case VP_LINE_HOWLER_POLREV:
#endif
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
        case VP_LINE_STANDBY_POLREV:
            return TRUE;
        case VP_LINE_TIP_OPEN:
            if (deviceType == VP_DEV_890_SERIES) {
                return FALSE;
            } else {
                return TRUE;
            }
        default:
            return FALSE;
    }
}
#endif  /* defined (VP880_FXS_SUPPORT) || defined (VP890_FXS_SUPPORT) */

/**
 * VpCSLACSetAbsGain()
 *
 *    This function implements VP_OPTION_ID_ABS_GAIN for 880 and 890 devices.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
#ifdef CSLAC_GAIN_ABS
VpStatusType
VpCSLACSetAbsGain(
    VpLineCtxType *pLineCtx,
    VpOptionAbsGainType *gains)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDeviceType deviceType = pDevCtx->deviceType;
    void *pLineObj = pLineCtx->pLineObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    uint8 gxCmdWrt, grCmdWrt, gxCmdRd, grCmdRd, grCmdLen, gxCmdLen;

    int16 grMaxGain;
    int16 grMinGain;

    int16 gxMaxGain;
    int16 gxMinGain;

    uint16* pGxInt;
    uint16* pGrInt;

    int16 *pGxAbsGain, *pGrAbsGain;
    int16 *pGxRestoreOption, *pGrRestoreOption;
    uint8 *pGxRestoreReg, *pGrRestoreReg;

    bool updateGr = FALSE;
    bool updateGx = FALSE;

#define VP_GX_TABLE_SIZE (50)
    /* This is the quiet value. Overwrite if a non-quiet level is specified. */
    uint8 gxValue[] = {0xF8, 0xF8};   /* -84.44dB */

    uint8 gxGainLookup[VP_GX_TABLE_SIZE] = {
       /* GX Setting ABS: FXS    880/890-FXO */
       /* ---------------------------------- */
        0x01, 0x90,  /*  -6dB        0dB     */
        0xAA, 0xC4,  /*  -5.5dB      0.5dB   */
        0xCA, 0xD3,  /*  -5dB       +1.0dB   */
        0x5E, 0xA2,  /*  -4.5dB     +1.5dB   */
        0x33, 0x52,  /*  -4dB       +2.0dB   */
        0x22, 0xA1,  /*  -3.5dB     +2.5dB   */
        0x2A, 0xA1,  /*  -3dB       +3.0dB   */
        0x3D, 0xF1,  /*  -2.5dB     +3.5dB   */
        0x2A, 0x21,  /*  -2dB       +4.0dB   */
        0x32, 0xA0,  /*  -1.5dB     +4.5dB   */
        0xBB, 0xA0,  /*  -1dB       +5.0dB   */
        0x3C, 0xB0,  /*  -0.5dB     +5.5dB   */
        0xA9, 0xF0,  /*   0dB       +6.0dB   */
        0xAB, 0x30,  /*  +0.5dB     +6.5dB   */
        0xAC, 0x20,  /*  +1dB       +7.0dB   */
        0x5A, 0x10,  /*  +1.5dB     +7.5dB   */
        0xA5, 0x10,  /*  +2dB       +8.0dB   */
        0x22, 0x10,  /*  +2.5dB     +8.5dB   */
        0xAA, 0x00,  /*  +3dB       +9.0dB   */
        0xCE, 0x00,  /*  +3.5dB     +9.5dB   */
        0x23, 0x00,  /*  +4dB      +10.0dB   */
        0xA1, 0x00,  /*  +4.5dB    +10.5dB   */
        0x31, 0x00,  /*  +5dB      +11.0dB   */
        0xA0, 0x00,  /*  +5.5dB    +11.5dB   */
        0xE0, 0x00   /*  +6dB      +12.0dB   */
    };

#define VP_GR_TABLE_SIZE (50)
    /* This is the quiet value. Overwrite if a non-quiet level is specified. */
    uint8 grValue[] = {0x8F, 0x87};

    uint8 grGainLookup[VP_GR_TABLE_SIZE] = {
       /* GR Setting ABS: FXS    880/890-FXO */
       /* -----------------------------------*/
        0xA9, 0x72,  /*     -12dB       -9dB    */
        0xB5, 0x42,  /*     -11.5dB     -8.5dB  */
        0x87, 0x32,  /*     -11dB       -8dB    */
        0xBA, 0x22,  /*     -10.5dB     -7.5dB  */
        0xC4, 0x22,  /*     -10dB       -7dB    */
        0x22, 0xA1,  /*     -9.5dB      -6.5dB  */
        0x23, 0xA1,  /*     -9dB        -6dB    */
        0xBF, 0xA1,  /*     -8.5dB      -5.5dB  */
        0xAA, 0xA1,  /*     -8dB        -5dB    */
        0x62, 0xB1,  /*     -7.5dB      -4.5dB  */
        0x2B, 0xB1,  /*     -7dB        -4dB    */
        0x3B, 0xC1,  /*     -6.5dB      -3.5dB  */
        0xA8, 0x71,  /*     -6dB        -3dB    */
        0xAE, 0x41,  /*     -5.5dB      -2.5dB  */
        0x8F, 0x31,  /*     -5dB        -2dB    */
        0xCA, 0x21,  /*     -4.5dB      -1.5dB  */
        0xA4, 0x21,  /*     -4dB        -1dB    */
        0x22, 0xA0,  /*     -3.5dB      -0.5dB  */
        0xA2, 0xA0,  /*     -3dB         0dB    */
        0x87, 0xA0,  /*     -2.5dB      +0.5dB  */
        0xAA, 0xA0,  /*     -2dB        +1dB    */
        0x42, 0xB0,  /*     -1.5dB      +1.5dB  */
        0x5B, 0xB0,  /*     -1dB        +2dB    */
        0xBB, 0xC0,  /*     -0.5dB      +2.5dB  */
        0xA8, 0xF0   /*     0dB         +3dB    */
    };

    switch (deviceType) {
#ifdef VP_CC_880_SERIES
        case VP_DEV_880_SERIES:
            gxCmdWrt = VP880_GX_GAIN_WRT;
            grCmdWrt = VP880_GR_GAIN_WRT;
            gxCmdRd = VP880_GX_GAIN_RD;
            grCmdRd = VP880_GR_GAIN_RD;
            grCmdLen = VP880_GR_GAIN_LEN;
            gxCmdLen = VP880_GX_GAIN_LEN;
            pGxInt = &((Vp880LineObjectType *)pLineObj)->gain.gxInt;
            pGrInt = &((Vp880LineObjectType *)pLineObj)->gain.grInt;
            pGxAbsGain = &((Vp880LineObjectType *)pLineObj)->gain.absGxGain;
            pGrAbsGain = &((Vp880LineObjectType *)pLineObj)->gain.absGrGain;
            pGxRestoreOption = &((Vp880LineObjectType *)pLineObj)->gain.absGxRestoreOption;
            pGrRestoreOption = &((Vp880LineObjectType *)pLineObj)->gain.absGrRestoreOption;
            pGxRestoreReg = ((Vp880LineObjectType *)pLineObj)->gain.absGxRestoreReg;
            pGrRestoreReg = ((Vp880LineObjectType *)pLineObj)->gain.absGrRestoreReg;

            if (((Vp880LineObjectType *)pLineObj)->status & VP880_IS_FXO) {
                gxMaxGain = 120;
                gxMinGain = 0;

                grMaxGain = 30;
                grMinGain = -90;
            } else {
                gxMaxGain = 60;
                gxMinGain = -60;

                grMaxGain = 0;
                grMinGain = -120;
            }
            break;
#endif

#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            gxCmdWrt = VP886_R_GX_WRT;
            grCmdWrt = VP886_R_GR_WRT;
            gxCmdRd = VP886_R_GX_RD;
            grCmdRd = VP886_R_GR_RD;
            gxCmdLen = VP886_R_GX_LEN;
            grCmdLen = VP886_R_GR_LEN;
            pGxInt = &((Vp886LineObjectType *)pLineObj)->gxBase;
            pGrInt = &((Vp886LineObjectType *)pLineObj)->grBase;
            pGxAbsGain = &((Vp886LineObjectType *)pLineObj)->options.absGain.gain_AToD;
            pGrAbsGain = &((Vp886LineObjectType *)pLineObj)->options.absGain.gain_DToA;
            pGxRestoreOption = &((Vp886LineObjectType *)pLineObj)->absGxRestoreOption;
            pGrRestoreOption = &((Vp886LineObjectType *)pLineObj)->absGrRestoreOption;
            pGxRestoreReg = ((Vp886LineObjectType *)pLineObj)->absGxRestoreReg;
            pGrRestoreReg = ((Vp886LineObjectType *)pLineObj)->absGrRestoreReg;

            if (((Vp886LineObjectType *)pLineObj)->isFxs != TRUE) {
                gxMaxGain = 120;
                gxMinGain = 0;

                grMaxGain = 30;
                grMinGain = -90;
            } else {
                gxMaxGain = 60;
                gxMinGain = -60;

                grMaxGain = 0;
                grMinGain = -120;
            }
            break;
#endif

#ifdef VP_CC_890_SERIES
        case VP_DEV_890_SERIES:
            gxCmdWrt = VP890_GX_GAIN_WRT;
            grCmdWrt = VP890_GR_GAIN_WRT;
            gxCmdRd = VP890_GX_GAIN_RD;
            grCmdRd = VP890_GR_GAIN_RD;
            grCmdLen = VP890_GR_GAIN_LEN;
            gxCmdLen = VP890_GX_GAIN_LEN;
            pGxInt = &((Vp890LineObjectType *)pLineObj)->gxBase;
            pGrInt = &((Vp890LineObjectType *)pLineObj)->grBase;
            pGxAbsGain = &((Vp890LineObjectType *)pLineObj)->absGxGain;
            pGrAbsGain = &((Vp890LineObjectType *)pLineObj)->absGrGain;
            pGxRestoreOption = &((Vp890LineObjectType *)pLineObj)->absGxRestoreOption;
            pGrRestoreOption = &((Vp890LineObjectType *)pLineObj)->absGrRestoreOption;
            pGxRestoreReg = ((Vp890LineObjectType *)pLineObj)->absGxRestoreReg;
            pGrRestoreReg = ((Vp890LineObjectType *)pLineObj)->absGrRestoreReg;

            if (((Vp890LineObjectType *)pLineObj)->status & VP890_IS_FXO) {
                gxMaxGain = 120;
                gxMinGain = 0;

                grMaxGain = 30;
                grMinGain = -90;
            } else {
                gxMaxGain = 60;
                gxMinGain = -60;

                grMaxGain = 0;
                grMinGain = -120;
            }
            break;
#endif
        default:
            return VP_STATUS_INVALID_ARG;
    }

    VP_GAIN(VpLineCtxType, pLineCtx, ("AToD: %d - DToA: %d",
        gains->gain_AToD, gains->gain_DToA));

    if (gains->gain_AToD == VP_OPTION_ABS_GAIN_RESTORE) {
        if (*pGxAbsGain == VP_OPTION_ABS_GAIN_QUIET) {
            /* Restore the previous option and register value if the current
               setting is QUIET. */
            *pGxAbsGain = *pGxRestoreOption;
            CommonWrite(pLineCtx, gxCmdWrt, gxCmdLen, pGxRestoreReg);
        }
        /* Else, do nothing for RESTORE if the current setting is not QUIET. */

    } else if (gains->gain_AToD == VP_OPTION_ABS_GAIN_QUIET) {
        /* Save the current option and register value so that they can
           be restored using VP_OPTION_ABS_GAIN_RESTORE.  The register value
           may have been set by either this function or VpSetRelGain() so it's
           easiest to just read the register here. */ 
        *pGxRestoreOption = *pGxAbsGain;
        CommonRead(pLineCtx, gxCmdRd, gxCmdLen, pGxRestoreReg);

        /* Save the new option value */
        *pGxAbsGain = gains->gain_AToD;

        /* Apply the QUIET setting */
        CommonWrite(pLineCtx, gxCmdWrt, gxCmdLen, gxValue);

    } else if (gains->gain_AToD != VP_OPTION_ABS_GAIN_NO_CHANGE) {
        /* Roundoff to nearest 0.5dB step */
        int16 tempGain = ABS(gains->gain_AToD);
        int16 gainRound = tempGain % 5;
        tempGain += ((gainRound < 3) ? (-gainRound) : (5 - gainRound));
        tempGain = ((gains->gain_AToD < 0) ? -tempGain : tempGain);

        /* Limit the gain to device/line specific ranges */
        if (tempGain > gxMaxGain) {
            tempGain = gxMaxGain;
            status = VP_STATUS_INPUT_PARAM_OOR;
        } else if (tempGain < gxMinGain) {
            status = VP_STATUS_INPUT_PARAM_OOR;
            tempGain = gxMinGain;
        }

        /* Save the actual gain being applied before converting to index */
        *pGxAbsGain = tempGain;

        /* Convert to index */
        tempGain = (tempGain - gxMinGain);
        tempGain = tempGain / 5;

        VP_GAIN(VpLineCtxType, pLineCtx, ("Final AToD Value: %d",
            *pGxAbsGain));

        gxValue[0] = gxGainLookup[(uint8)tempGain * 2];
        gxValue[1] = gxGainLookup[(uint8)tempGain * 2 + 1];
        updateGx = TRUE;

        CommonWrite(pLineCtx, gxCmdWrt, gxCmdLen, gxValue);
    }

    if (gains->gain_DToA == VP_OPTION_ABS_GAIN_RESTORE) {
        if (*pGrAbsGain == VP_OPTION_ABS_GAIN_QUIET) {
            /* Restore the previous option and register value if the current
               setting is QUIET. */
            *pGrAbsGain = *pGrRestoreOption;
            CommonWrite(pLineCtx, grCmdWrt, grCmdLen, pGrRestoreReg);
        }
        /* Else, do nothing for RESTORE if the current setting is not QUIET. */

    } else if (gains->gain_DToA == VP_OPTION_ABS_GAIN_QUIET) {
        /* Save the current option and register value so that they can
           be restored using VP_OPTION_ABS_GAIN_RESTORE.  The register value
           may have been set by either this function or VpSetRelGain() so it's
           easiest to just read the register here. */ 
        *pGrRestoreOption = *pGrAbsGain;
        CommonRead(pLineCtx, grCmdRd, grCmdLen, pGrRestoreReg);

        /* Save the new option value */
        *pGrAbsGain = gains->gain_DToA;

        /* Apply the QUIET setting */
        CommonWrite(pLineCtx, grCmdWrt, grCmdLen, grValue);

    } else if (gains->gain_DToA != VP_OPTION_ABS_GAIN_NO_CHANGE) {
        /* Roundoff to nearest 0.5dB step */
        int16 tempGain = ABS(gains->gain_DToA);
        int16 gainRound = tempGain % 5;
        tempGain += ((gainRound < 3) ? (-gainRound) : (5 - gainRound));
        tempGain = ((gains->gain_DToA < 0) ? -tempGain : tempGain);

        /* Limit the gain to device/line specific ranges */
        if (tempGain > grMaxGain) {
            status = VP_STATUS_INPUT_PARAM_OOR;
            tempGain = grMaxGain;
        } else if (tempGain < grMinGain) {
            status = VP_STATUS_INPUT_PARAM_OOR;
            tempGain = grMinGain;
        }

        /* Save the actual gain being applied before converting to index */
        *pGrAbsGain = tempGain;

        /* Save the option and the current register value so that they can
           be restored using VP_OPTION_ABS_GAIN_RESTORE.  The register value
           may be set by either this function or VpSetRelGain(). */ 
        *pGrRestoreOption = tempGain;
        CommonRead(pLineCtx, grCmdRd, grCmdLen, pGrRestoreReg);

        /* Convert to index */
        tempGain = (tempGain - grMinGain);
        tempGain = tempGain / 5;

        VP_GAIN(VpLineCtxType, pLineCtx, ("Final DToA Value: %d",
            *pGrAbsGain));

        grValue[0] = grGainLookup[(uint8)tempGain * 2];
        grValue[1] = grGainLookup[(uint8)tempGain * 2 + 1];
        updateGr = TRUE;

        CommonWrite(pLineCtx, grCmdWrt, grCmdLen, grValue);
    }

#ifdef CSLAC_GAIN_RELATIVE
    /* Update cached transmit and receive gains for SetRelGain */
    if (updateGx == TRUE) {
        *pGxInt = 0x4000 + VpConvertCsd2Fixed(gxValue);
    }
    if (updateGr == TRUE) {
        *pGrInt = VpConvertCsd2Fixed(grValue);
    }
#endif

    return status;
}

/**
 * CommonWrite()
 *
 *    This function abstracts MPI writes to the device.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
static VpStatusType
CommonWrite(
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 cmdLen,
    uint8* data)
{

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDeviceType deviceType = pDevCtx->deviceType;
    VpDeviceIdType deviceId;
    void *pLineObj = pLineCtx->pLineObj;
    void *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal;
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (deviceType) {
#ifdef VP_CC_880_SERIES
        case VP_DEV_880_SERIES:
            ecVal = ((Vp880LineObjectType *)pLineObj)->ecVal;
            deviceId = ((Vp880DeviceObjectType *)pDevObj)->deviceId;
            VpMpiCmdWrapper(deviceId, ecVal, cmd, cmdLen, data);
            break;
#endif

#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            (void)deviceId;
            (void)pLineObj;
            (void)pDevObj;
            (void)ecVal;
            VpSlacRegWrite(NULL, pLineCtx, cmd, cmdLen, data);
            break;
#endif

#ifdef VP_CC_890_SERIES
        case VP_DEV_890_SERIES:
            ecVal = ((Vp890LineObjectType *)pLineObj)->ecVal;
            deviceId = ((Vp890DeviceObjectType *)pDevObj)->deviceId;
            VpMpiCmdWrapper(deviceId, ecVal, cmd, cmdLen, data);
            break;
#endif
        default:
            return VP_STATUS_INVALID_ARG;
    }

    return status;
} /* CommonWrite */

/**
 * CommonRead()
 *
 *    This function abstracts MPI reads from the device.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
static VpStatusType
CommonRead(
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 cmdLen,
    uint8* data)
{

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDeviceType deviceType = pDevCtx->deviceType;
    VpDeviceIdType deviceId;
    void *pLineObj = pLineCtx->pLineObj;
    void *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal;
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (deviceType) {
#ifdef VP_CC_880_SERIES
        case VP_DEV_880_SERIES:
            ecVal = ((Vp880LineObjectType *)pLineObj)->ecVal;
            deviceId = ((Vp880DeviceObjectType *)pDevObj)->deviceId;
            VpMpiCmdWrapper(deviceId, ecVal, cmd, cmdLen, data);
            break;
#endif

#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            (void)deviceId;
            (void)pLineObj;
            (void)pDevObj;
            (void)ecVal;
            VpSlacRegRead(NULL, pLineCtx, cmd, cmdLen, data);
            break;
#endif

#ifdef VP_CC_890_SERIES
        case VP_DEV_890_SERIES:
            ecVal = ((Vp890LineObjectType *)pLineObj)->ecVal;
            deviceId = ((Vp890DeviceObjectType *)pDevObj)->deviceId;
            VpMpiCmdWrapper(deviceId, ecVal, cmd, cmdLen, data);
            break;
#endif
        default:
            return VP_STATUS_INVALID_ARG;
    }

    return status;
} /* CommonRead */

#endif  /* #ifdef CSLAC_GAIN_ABS */
#endif  /* defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES)|| defined(VP_CC_890_SERIES) */

/**
 * ConvertApiState2PrfWizState()
 *
 *    Maps an API-II line state into the equivelent state value used in cadence
 *    profiles. This function is used by the internal API functions when creating an internal
 *    cadence. Rather than use a state machine in some cases, it's easier to build a profile
 *    cadence type sequence and run that.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
VpProfileCadencerStateTypes
ConvertApiState2PrfWizState(
    const VpLineStateType state)
{
    switch(state) {
        case VP_LINE_STANDBY:           return VP_PROFILE_CADENCE_STATE_STANDBY;
        case VP_LINE_TIP_OPEN:          return VP_PROFILE_CADENCE_STATE_TIP_OPEN;
        case VP_LINE_ACTIVE:            return VP_PROFILE_CADENCE_STATE_ACTIVE;
        case VP_LINE_ACTIVE_POLREV:     return VP_PROFILE_CADENCE_STATE_POLREV_ACTIVE;

        /* Don't support cadencing in/out of Howler State in this release */
        case VP_LINE_HOWLER:            return VP_PROFILE_CADENCE_STATE_ACTIVE;
        case VP_LINE_HOWLER_POLREV:     return VP_PROFILE_CADENCE_STATE_POLREV_ACTIVE;

        case VP_LINE_TALK:              return VP_PROFILE_CADENCE_STATE_TALK;
        case VP_LINE_TALK_POLREV:       return VP_PROFILE_CADENCE_STATE_POLREV_TALK;
        case VP_LINE_OHT:               return VP_PROFILE_CADENCE_STATE_OHT;
        case VP_LINE_OHT_POLREV:        return VP_PROFILE_CADENCE_STATE_POLREV_OHT;
        case VP_LINE_DISCONNECT:        return VP_PROFILE_CADENCE_STATE_DISCONNECT;
        case VP_LINE_RINGING:           return VP_PROFILE_CADENCE_STATE_RINGING;
        case VP_LINE_RINGING_POLREV:    return VP_PROFILE_CADENCE_STATE_POLREV_RINGING;
        case VP_LINE_STANDBY_POLREV:    return VP_PROFILE_CADENCE_STATE_POLREV_STANDBY;

        case VP_LINE_FXO_OHT:           return VP_PROFILE_CADENCE_STATE_FXO_OHT;
        case VP_LINE_FXO_LOOP_OPEN:     return VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN;
        case VP_LINE_FXO_LOOP_CLOSE:    return VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE;
        case VP_LINE_FXO_TALK:          return VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK;

        /**< The calling function should check this is NOT returned */
        default:                        return VP_PROFILE_CADENCE_STATE_UNKNOWN;
    };
} /* ConvertApiState2PrfWizState() */

/**
 * ConvertPrfWizState2ApiState()
 *
 *    Maps a Profile Wizard State to the equivelent VP-API-II state
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  None
 */
VpLineStateType
ConvertPrfWizState2ApiState(
    uint8 state)
{
    switch(state) {
        case VP_PROFILE_CADENCE_STATE_STANDBY:          return VP_LINE_STANDBY;
        case VP_PROFILE_CADENCE_STATE_TIP_OPEN:         return VP_LINE_TIP_OPEN;
        case VP_PROFILE_CADENCE_STATE_ACTIVE:           return VP_LINE_ACTIVE;
        case VP_PROFILE_CADENCE_STATE_POLREV_ACTIVE:    return VP_LINE_ACTIVE_POLREV;
        case VP_PROFILE_CADENCE_STATE_TALK:             return VP_LINE_TALK;
        case VP_PROFILE_CADENCE_STATE_POLREV_TALK:      return VP_LINE_TALK_POLREV;
        case VP_PROFILE_CADENCE_STATE_OHT:              return VP_LINE_OHT;
        case VP_PROFILE_CADENCE_STATE_POLREV_OHT:       return VP_LINE_OHT_POLREV;
        case VP_PROFILE_CADENCE_STATE_DISCONNECT:       return VP_LINE_DISCONNECT;
        case VP_PROFILE_CADENCE_STATE_RINGING:          return VP_LINE_RINGING;
        case VP_PROFILE_CADENCE_STATE_POLREV_RINGING:   return VP_LINE_RINGING_POLREV;
        case VP_PROFILE_CADENCE_STATE_POLREV_STANDBY:   return VP_LINE_STANDBY_POLREV;

        case VP_PROFILE_CADENCE_STATE_FXO_OHT:          return VP_LINE_FXO_OHT;
        case VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN:    return VP_LINE_FXO_LOOP_OPEN;
        case VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE:   return VP_LINE_FXO_LOOP_CLOSE;
        case VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK:    return VP_LINE_FXO_TALK;

        default:                                        return VP_LINE_STANDBY;
    };
} /* ConvertPrfWizState2ApiState() */

#if defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) \
    || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES)
/**
 * InitTimerVars()
 *  This function initializes the Cadence (sequencer) Variables in the line
 * object associated with the line context passed.
 *
 * Preconditions:
 *  None
 *
 * Postconditions:
 *  The VpSeqDataType structure variables passed in the line context are
 * initialized to pre-determined values.
 */
void
InitTimerVars(
    VpLineCtxType *pLineCtx)    /**< Line to initialize API Timer (internal)
                                 * Variables for
                                 */
{
    VpDeviceType deviceType = pLineCtx->pDevCtx->deviceType;
    void *pLineObj = pLineCtx->pLineObj;
    VpCslacTimerStruct *pTimer;

    switch(deviceType) {
#if defined (VP_CC_790_SERIES)
        case VP_DEV_790_SERIES:
            pTimer = &((Vp790LineObjectType *)pLineObj)->lineTimers;
            break;
#endif

#if defined (VP_CC_880_SERIES)
        case VP_DEV_880_SERIES:
            pTimer = &((Vp880LineObjectType *)pLineObj)->lineTimers;
            break;
#endif

#if defined (VP_CC_890_SERIES)
        case VP_DEV_890_SERIES:
            pTimer = &((Vp890LineObjectType *)pLineObj)->lineTimers;
            break;
#endif

#if defined (VP_CC_580_SERIES)
        case VP_DEV_580_SERIES:
            pTimer = &((Vp580LineObjectType *)pLineObj)->lineTimers;
            break;
#endif
        default:
            /* Nothing known to initialize */
            return;
    }

    VpMemSet(pTimer, 0x00, sizeof(VpCslacTimerStruct));

    if (pTimer->type == VP_CSLAC_FXO_TIMER) {
#if (defined(VP_CC_880_SERIES) && defined (VP880_FXO_SUPPORT)) || \
    (defined(VP_CC_890_SERIES) && defined (VP890_FXO_SUPPORT)) || defined(VP_CC_580_SERIES)
        /* Iniialize non-zero initial values */
        pTimer->timers.fxoTimer.prevHighToLowTime = 0x7FFF;
        pTimer->timers.fxoTimer.noCount = TRUE;
        pTimer->timers.fxoTimer.timeLastPolRev = 0x7FFF;
        pTimer->timers.fxoTimer.timePrevPolRev = 0x7FFF;
        pTimer->timers.fxoTimer.disconnectDebounce = 0xFFFF;
        pTimer->timers.fxoTimer.liuDebounce = 0xFF;
        pTimer->timers.fxoTimer.bCalTimer = 0xFF;
#endif
    }
}
#endif /* defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) \
    || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES) */

#if (defined(VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT)) || \
    (defined(VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT)) || \
     defined(VP_CC_580_SERIES) || defined(VP_CC_790_SERIES)

/**
 * VpUpdateDP()
 *  This function measures the timing of the on/off-hook conditions for Dial
 * Pulsing and Flash Hook events (as well as the immediate Off-Hook, and long
 * On-Hook events).
 *
 * Preconditions:
 *  The pointers passed (Dial Pulse specifications, Dial Pulse data structure,
 * and Line Events structure) cannot be VP_NULL.
 *
 * Postconditions:
 *  The line events structure is updated with an event if the on/off-hook
 * variables provided in the dial pulse structure meet the specifications
 * provided in the dial pulse specification structure. Note: This function has
 * no knowledge of line context/objects, so it can be used for any type of
 * line.
 */
bool
VpUpdateDP(
    uint16 tickRate,    /**< Device API Tickrate used to measure real
                         * on/off-hook time
                         */

    VpOptionPulseType *pPulseSpecs,     /**< Dial Pulse specifications to apply
                                         * to on/off-hooks
                                         */
    VpDialPulseDetectType *pDpStruct,   /**< Structure used to maintain dial
                                         * pulse status
                                         */
    VpOptionEventMaskType *pLineEvents) /**< Line event structure to be modified
                                         * if an event is detected (and needs to
                                         * be reported).
                                         */
{
    bool eventStatus = FALSE;
    uint16 break_min, break_max;
    uint16 make_min, make_max;
    uint16 flash_min, flash_max;
    uint16 interDigitMin, conversionFactor;
    uint16 onHook_min;

    if ((pPulseSpecs == VP_NULL)
     || (pDpStruct == VP_NULL)
     || (pLineEvents == VP_NULL)) {
        return FALSE;
    }

    /* Get the specs in 125us units */
    break_min = pPulseSpecs->breakMin;
    break_max= pPulseSpecs->breakMax;
    make_min = pPulseSpecs->makeMin;
    make_max = pPulseSpecs->makeMax;
    flash_min = pPulseSpecs->flashMin;
    flash_max = pPulseSpecs->flashMax;

#ifdef EXTENDED_FLASH_HOOK
    onHook_min = pPulseSpecs->onHookMin;
#else
    onHook_min = flash_max;
#endif

    interDigitMin = pPulseSpecs->interDigitMin;
#define VP_API_TICKRATE_CONVERSION   (32)
    conversionFactor = (tickRate / VP_API_TICKRATE_CONVERSION);

    switch(pDpStruct->state) {
        case VP_DP_DETECT_STATE_IDLE:
            if(pDpStruct->hookSt) {
                /*
                 * We are off-hook after being on-hook for a "long time". Start
                 * dial pulsing
                 */
                pDpStruct->state = VP_DP_DETECT_STATE_LOOP_CLOSE;
                pDpStruct->lc_time = 0;
                eventStatus = TRUE;
                pLineEvents->signaling &= ~VP_LINE_EVID_PULSE_DIG;
                pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_ON;
                pLineEvents->signaling &= ~VP_LINE_EVID_FLASH;
                pLineEvents->signaling &= ~VP_LINE_EVID_BREAK_MAX;
                pLineEvents->signaling &= ~VP_LINE_EVID_EXTD_FLASH;
                pLineEvents->signaling |= VP_LINE_EVID_HOOK_OFF;

                VP_HOOK(None, NULL, ("VP_DP_DETECT_STATE_IDLE: DP Off-Hook"));
            }
            break;

        case VP_DP_DETECT_STATE_LOOP_OPEN:
            pDpStruct->lo_time += conversionFactor;

            if(pDpStruct->hookSt) {
                /* Detected off-hook */

                if ((pDpStruct->lo_time >= break_min) && (pDpStruct->lo_time <= break_max)) {
                    /*
                     * We think we just dialed a pulse, but don't count it if
                     * this sequence of on/off hooks has already been marked
                     * as invalid (-1)
                     */
                    if (pDpStruct->digits != -1) {
                        pDpStruct->digits++;
                    }

                    VP_HOOK(None, NULL,
                            ("1. VP_DP_DETECT_STATE_LOOP_OPEN going to VP_DP_DETECT_STATE_LOOP_CLOSE: With Loop Open Time %d",
                             pDpStruct->lo_time));

                    pDpStruct->state = VP_DP_DETECT_STATE_LOOP_CLOSE;
                } else if ((pDpStruct->lo_time >= flash_min) && (pDpStruct->lo_time <= flash_max)) {
                    /* We did a hook flash */
                    pDpStruct->signalingData = pDpStruct->lo_time * (tickRate >> 8);

                    eventStatus = TRUE;
                    pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_ON;
                    pLineEvents->signaling &= ~VP_LINE_EVID_PULSE_DIG;
                    pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_OFF;
                    pLineEvents->signaling &= ~VP_LINE_EVID_BREAK_MAX;
                    pLineEvents->signaling &= ~VP_LINE_EVID_EXTD_FLASH;
                    pLineEvents->signaling |= VP_LINE_EVID_FLASH;

                    VpInitDP(pDpStruct);
                    pDpStruct->state = VP_DP_DETECT_STATE_LOOP_CLOSE;
                    pDpStruct->lo_time = flash_max-1;

                    VP_HOOK(None, NULL,
                            ("2. VP_DP_DETECT_STATE_LOOP_OPEN going to VP_DP_DETECT_STATE_LOOP_CLOSE: With Loop Open Time %d",
                             pDpStruct->lo_time));
                } else if ((pDpStruct->lo_time > flash_max) && (pDpStruct->lo_time <= onHook_min)) {
                    /*
                     * We did something between hook flash and on-hook. This is
                     * defined as a "new call" event.
                     */
                    pDpStruct->signalingData = pDpStruct->lo_time * (tickRate >> 8);

                    eventStatus = TRUE;
                    pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_ON;
                    pLineEvents->signaling &= ~VP_LINE_EVID_PULSE_DIG;
                    pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_OFF;
                    pLineEvents->signaling &= ~VP_LINE_EVID_BREAK_MAX;
                    pLineEvents->signaling |= VP_LINE_EVID_EXTD_FLASH;
                    pLineEvents->signaling &= ~VP_LINE_EVID_FLASH;

                    VpInitDP(pDpStruct);
                    pDpStruct->state = VP_DP_DETECT_STATE_LOOP_CLOSE;
                    pDpStruct->lo_time = onHook_min-1;

                    VP_HOOK(None, NULL,
                            ("3. VP_DP_DETECT_STATE_LOOP_OPEN going to VP_DP_DETECT_STATE_LOOP_CLOSE: With Loop Open Time %d",
                             pDpStruct->lo_time));
                } else {
                    /* This occurs for invalid digits */
                    pDpStruct->state = VP_DP_DETECT_STATE_LOOP_CLOSE;

                    /* Mark this sequence of digits invalid */
                    pDpStruct->digits = -1;

                    VP_HOOK(None, NULL,
                            ("INVALID_DIGIT!! VP_DP_DETECT_STATE_LOOP_OPEN going to VP_DP_DETECT_STATE_LOOP_CLOSE: With Loop Open Time %d",
                             pDpStruct->lo_time));
                }
                pDpStruct->lc_time = 0;
            } else {
                if (pDpStruct->lo_time > onHook_min) {
                    VP_HOOK(None, NULL,
                            ("HOOK_ON: VP_DP_DETECT_STATE_LOOP_OPEN going to VP_DP_DETECT_STATE_IDLE: With Loop Open Time %d",
                             pDpStruct->lo_time));

                    /* We're on-hook, report and start over */
                    VpInitDP(pDpStruct);
                    eventStatus = TRUE;
                    pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_OFF;
                    pLineEvents->signaling &= ~VP_LINE_EVID_PULSE_DIG;
                    pLineEvents->signaling &= ~VP_LINE_EVID_FLASH;
                    pLineEvents->signaling &= ~VP_LINE_EVID_BREAK_MAX;
                    pLineEvents->signaling |= VP_LINE_EVID_HOOK_ON;
                    pLineEvents->signaling &= ~VP_LINE_EVID_EXTD_FLASH;
                }
            }
            break;

        case VP_DP_DETECT_STATE_LOOP_CLOSE:
            /* Limit the value lc_time can reach so we don't overflow */
            if(pDpStruct->lc_time < (0xFFFF - conversionFactor)) {
                pDpStruct->lc_time += conversionFactor;
            }

            /* Check to see if we're still off-hook */
            if(pDpStruct->hookSt) {
                /* We're still off-hook. Did we reach the interdigit time? */
                if (pDpStruct->lc_time >= interDigitMin &&
                    pDpStruct->lc_time < interDigitMin + conversionFactor) {
                    /* Interdigit time reached. Report digits collected */
                    if (pDpStruct->digits > 0) {
                        /* We have dialed digits */
                        eventStatus = TRUE;
                        pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_ON;
                        pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_OFF;
                        pLineEvents->signaling &= ~VP_LINE_EVID_FLASH;
                        pLineEvents->signaling &= ~VP_LINE_EVID_BREAK_MAX;
                        pLineEvents->signaling |= VP_LINE_EVID_PULSE_DIG;
                        pDpStruct->signalingData = pDpStruct->digits;
                        pLineEvents->signaling &= ~VP_LINE_EVID_EXTD_FLASH;
                    } else {
                        /*
                         * If we're still off-hook, we either didn't collect
                         * digits, the digits were invalid, this was an initial
                         * off-hook or a Hook Flash. If the digits were not
                         * invalid, the event has been reported. Only report an
                         * event for invalid digits.
                         */
                        if (pDpStruct->digits < 0) {
                            eventStatus = TRUE;
                            pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_ON;
                            pLineEvents->signaling &= ~VP_LINE_EVID_HOOK_OFF;
                            pLineEvents->signaling &= ~VP_LINE_EVID_FLASH;
                            pLineEvents->signaling &= ~VP_LINE_EVID_BREAK_MAX;
                            pLineEvents->signaling |= VP_LINE_EVID_PULSE_DIG;
                            pLineEvents->signaling &= ~VP_LINE_EVID_EXTD_FLASH;
                            pDpStruct->signalingData = VP_DIG_NONE;
                        }
                    }
                }
            } else {
                /* Detected on-hook. */

                /* If the interdigit time has passed or there are no digits
                 * counted yet, this is a new pulse sequence */
                if (pDpStruct->lc_time >= interDigitMin || pDpStruct->digits == 0) {
                    eventStatus = TRUE;
                    pLineEvents->signaling |= VP_LINE_EVID_STARTPULSE;
                    VpInitDP(pDpStruct);
                } else if (pDpStruct->lc_time > make_max || pDpStruct->lc_time < make_min) {
                    /* If this isn't the beginning of a pulse sequence and we're
                     * outside of the make min/max, mark this sequence invalid */
                    pDpStruct->digits = -1;
                }

                VP_HOOK(None, NULL,
                        ("VP_DP_DETECT_STATE_LOOP_CLOSE going to VP_DP_DETECT_STATE_LOOP_OPEN: Digit Close (%d) Open (%d)",
                         pDpStruct->lc_time, pDpStruct->lo_time));

                pDpStruct->state = VP_DP_DETECT_STATE_LOOP_OPEN;
                pDpStruct->lo_time = 0;
            }
            break;

        default:
            return FALSE;
    }

    return eventStatus;
}

/**
 * VpInitDP()
 *  Initializes the dial pulse structure variable passed to values required by
 * VpUpdateDP.
 *
 * Preconditions:
 *  The pointer passed (Dial data structure) cannot be VP_NULL.
 *
 * Postconditions:
 *  The data passed in the dial pulse data structure is initialized.
 */
void
VpInitDP(
    VpDialPulseDetectType *pDpStruct)   /**< Dial pulse structure to init */
{
    if (pDpStruct != VP_NULL) {
        pDpStruct->digits = 0;

        if (pDpStruct->hookSt == FALSE) {
            pDpStruct->state = VP_DP_DETECT_STATE_IDLE;
            pDpStruct->lo_time = 0xFFFF;
            pDpStruct->lc_time = 0;
        } else {
            pDpStruct->state = VP_DP_DETECT_STATE_LOOP_CLOSE;
            pDpStruct->lo_time = 0;
            pDpStruct->lc_time = 0xFFFF;
        }
    }
    return;
}
#endif

/**
 * VpCSLACGetLineStatus()
 *  This function returns the status of the type being request for the line
 * (context) being passed.
 *
 * Preconditions:
 *  The line context pointer passed must be valid.
 *
 * Postconditions:
 *  The location pointed to by the boolean pointer passed is set to either TRUE
 * or FALSE depending on the status of the line for the type of input requested.
 * This function returns the success status code if the information requested
 * is valid for the line type (FXO/FXS) being passed.
 */
VpStatusType
VpCSLACGetLineStatus(
    VpLineCtxType *pLineCtx,
    VpInputType input,
    bool *pStatus)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDeviceType deviceType = pDevCtx->deviceType;

#if defined (VP_CC_580_SERIES) || defined (VP_CC_886_SERIES) || \
    (defined (VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT)) || \
    defined (VP_CC_790_SERIES) || \
    (defined (VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT))
    VpDialPulseDetectType *pDpStruct = VP_NULL;
    bool resLeak = FALSE;
    VpOptionPulseModeType pulseMode = VP_OPTION_PULSE_DECODE_OFF;
#endif

    VpApiIntLineStateType *pLineState;
    VpDeviceDynamicInfoType *pDynamicInfo;
    bool fxo;

    void *pLineObj = pLineCtx->pLineObj;
    void *pDevObj = pDevCtx->pDevObj;

    switch (deviceType) {
#if defined (VP_CC_790_SERIES)
        case VP_DEV_790_SERIES:
            pDpStruct = &((Vp790LineObjectType *)pLineObj)->dpStruct;
            pLineState = &((Vp790LineObjectType *)pLineObj)->lineState;
            pDynamicInfo = &((Vp790DeviceObjectType *)pDevObj)->dynamicInfo;
            fxo = FALSE;
            pulseMode = ((Vp790LineObjectType *)pLineObj)->pulseMode;
            break;
#endif

#if defined (VP_CC_880_SERIES)
        case VP_DEV_880_SERIES:
#ifdef VP880_FXS_SUPPORT
            pDpStruct = &((Vp880LineObjectType *)pLineObj)->dpStruct;
            pulseMode = ((Vp880LineObjectType *)pLineObj)->pulseMode;
            resLeak = ((((Vp880LineObjectType *)pLineObj)->status) & VP880_LINE_LEAK)
                ? TRUE : FALSE;
#endif

            pLineState = &((Vp880LineObjectType *)pLineObj)->lineState;
            pDynamicInfo = &((Vp880DeviceObjectType *)pDevObj)->dynamicInfo;
            if (((Vp880LineObjectType *)pLineObj)->status & VP880_IS_FXO) {
                fxo = TRUE;
            } else {
                fxo = FALSE;
            }
            break;
#endif

#if defined (VP_CC_886_SERIES)
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            pulseMode = ((Vp886LineObjectType *)pLineObj)->options.pulseMode;
            pLineState = &((Vp886LineObjectType *)pLineObj)->lineState;
            pDynamicInfo = &((Vp886DeviceObjectType *)pDevObj)->dynamicInfo;
            if (((Vp886LineObjectType *)pLineObj)->isFxs) {
                fxo = FALSE;
            } else {
                fxo = TRUE;
            }
            break;
#endif

#if defined (VP_CC_890_SERIES)
        case VP_DEV_890_SERIES:
#ifdef VP890_FXS_SUPPORT
            pDpStruct = &((Vp890LineObjectType *)pLineObj)->dpStruct;
            pulseMode = ((Vp890LineObjectType *)pLineObj)->pulseMode;
            resLeak = ((((Vp890LineObjectType *)pLineObj)->status) & VP890_LINE_LEAK)
                ? TRUE : FALSE;
#endif
            pLineState = &((Vp890LineObjectType *)pLineObj)->lineState;
            pDynamicInfo = &((Vp890DeviceObjectType *)pDevObj)->dynamicInfo;
            if (((Vp890LineObjectType *)pLineObj)->status & VP890_IS_FXO) {
                fxo = TRUE;
            } else {
                fxo = FALSE;
            }
            break;
#endif

#if defined (VP_CC_580_SERIES)
        case VP_DEV_580_SERIES:
            pDpStruct = &((Vp580LineObjectType *)pLineObj)->dpStruct;
            pLineState = &((Vp580LineObjectType *)pLineObj)->lineState;
            pDynamicInfo = &((Vp580DeviceObjectType *)pDevObj)->dynamicInfo;
            if (((Vp580LineObjectType *)pLineObj)->status & VP580_IS_FXO) {
                fxo = TRUE;
            } else {
                fxo = FALSE;
            }
            pulseMode = ((Vp580LineObjectType *)pLineObj)->pulseMode;
            break;
#endif

        default:
            return VP_STATUS_INVALID_ARG;
    }

    if (fxo == FALSE) {
        switch(input) {
#if  defined (VP_CC_580_SERIES) || defined (VP_CC_580_SERIES) || \
    (defined (VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT)) || \
     defined (VP_CC_790_SERIES) || \
    (defined (VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT)) || \
     defined (VP_CC_886_SERIES)

            case VP_INPUT_HOOK:
                if(pulseMode == VP_OPTION_PULSE_DECODE_ON) {
                    if ((pDpStruct != VP_NULL) && (pDpStruct->state == VP_DP_DETECT_STATE_IDLE)) {
                        *pStatus = FALSE;
                    } else {
                        *pStatus = TRUE;
                    }
                } else {
                    if ((pLineState->condition & VP_CSLAC_HOOK)
                     && (!(pLineState->condition & VP_CSLAC_LINE_LEAK_TEST))) {
                        *pStatus = TRUE;
                    } else {
                        *pStatus = FALSE;
                    }
                }
                /* 886 doesn't use the same pulse decoder structure, so it must
                   be handled differently. */
#if defined (VP_CC_886_SERIES)
                if (deviceType == VP_DEV_886_SERIES || deviceType == VP_DEV_887_SERIES) {
                    Vp886LineObjectType *pLineObj886 = pLineObj;
                    if (pLineObj886->pulseDecodeData.state == VP_PULSE_DECODE_STATE_IDLE) {
                        /* IDLE could mean onhook or offhook, whichever the
                           raw state is */
                        if (pLineState->condition & VP_CSLAC_HOOK) {
                            *pStatus = TRUE;
                        } else {
                            *pStatus = FALSE;
                        }
                        if (pLineObj886->ringTripConfirm.confirming) {
                            /* Report onhook while confirming a ring trip */
                            *pStatus = FALSE;
                        }
                    } else {
                        /* All states other than IDLE are offhook */
                        *pStatus = TRUE;
                    }
                }
#endif /* defined (VP_CC_886_SERIES) */
                break;

            case VP_INPUT_RAW_HOOK:
                if ((pLineState->condition & VP_CSLAC_HOOK)
                 && (!(pLineState->condition & VP_CSLAC_LINE_LEAK_TEST))) {
                    *pStatus = TRUE;
                } else {
                    *pStatus = FALSE;
                }
#if defined (VP_CC_886_SERIES)
                if (deviceType == VP_DEV_886_SERIES || deviceType == VP_DEV_887_SERIES) {
                    Vp886LineObjectType *pLineObj886 = pLineObj;
                    if (pLineObj886->ringTripConfirm.confirming) {
                        /* Report onhook while confirming a ring trip */
                        *pStatus = FALSE;
                    }
                }
#endif /* defined (VP_CC_886_SERIES) */
                break;

            case VP_INPUT_GKEY:
                *pStatus = (pLineState->condition & VP_CSLAC_GKEY)
                    ? TRUE : FALSE;
                break;

            case VP_INPUT_CLK_FLT:
                *pStatus = (pDynamicInfo->clkFault) ? TRUE : FALSE;
                break;

            case VP_INPUT_THERM_FLT:
                *pStatus = (pLineState->condition & VP_CSLAC_THERM_FLT)
                    ? TRUE : FALSE;
                break;

            case VP_INPUT_AC_FLT:
                *pStatus = (pLineState->condition & VP_CSLAC_AC_FLT)
                    ? TRUE : FALSE;
                break;

            case VP_INPUT_DC_FLT:
                *pStatus = (pLineState->condition & VP_CSLAC_DC_FLT)
                    ? TRUE : FALSE;
                break;

            case VP_INPUT_BAT1_FLT:
                *pStatus = (pDynamicInfo->bat1Fault) ? TRUE : FALSE;
                break;

            case VP_INPUT_BAT2_FLT:
                *pStatus = (pDynamicInfo->bat2Fault) ? TRUE : FALSE;
                break;

            case VP_INPUT_BAT3_FLT:
                *pStatus = (pDynamicInfo->bat3Fault) ? TRUE : FALSE;
                break;

			case VP_INPUT_RES_LEAK:
                *pStatus = resLeak;
				break;
#endif
            default:
                return VP_STATUS_INVALID_ARG;
        }
    } else {
        switch(input) {
            case VP_INPUT_CLK_FLT:
                *pStatus = (pDynamicInfo->clkFault) ? TRUE : FALSE;
                break;

            /* DC Fault is possible from the FXO line on VE890 only */
            case VP_INPUT_DC_FLT:
                if (deviceType == VP_DEV_890_SERIES) {
                    *pStatus = (pLineState->condition & VP_CSLAC_DC_FLT) ? TRUE : FALSE;
                } else {
                    return VP_STATUS_INVALID_ARG;
                }
                break;


            case VP_INPUT_RINGING:
                *pStatus = (pLineState->condition & VP_CSLAC_RINGING) ? TRUE : FALSE;
                break;

            case VP_INPUT_POLREV:
                *pStatus = (pLineState->condition & VP_CSLAC_POLREV) ? TRUE : FALSE;
                break;

            case VP_INPUT_LIU:
                *pStatus = (pLineState->condition & VP_CSLAC_LIU) ? TRUE : FALSE;
                break;

            case VP_INPUT_FEED_DIS:
            case VP_INPUT_DISCONNECT:
                *pStatus = (pLineState->condition & VP_CSLAC_DISC) ? TRUE : FALSE;
                break;

            case VP_INPUT_FEED_EN:
            case VP_INPUT_CONNECT:
                *pStatus = (pLineState->condition & VP_CSLAC_DISC) ? FALSE : TRUE;
                break;

            default:
                return VP_STATUS_INVALID_ARG;
        }
    }
    return VP_STATUS_SUCCESS;
}

#if !defined(VP_REDUCED_API_IF)
#if defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) \
    || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES)
/**
 * VpCSLACClearResults()
 *  This function clears the device read events so that normal read operations
 * may occur. It is done by the application when the device is busy (in read
 * moode) and the application does not know what to read.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized.
 *
 * Postconditions:
 *  Device is no longer busy with read status.
 */
VpStatusType
VpCSLACClearResults(
    VpDevCtxType *pDevCtx)
{
    VpDeviceType deviceType = pDevCtx->deviceType;
    VpDeviceIdType deviceId;
    VpOptionEventMaskType *pDeviceEvents;

    void *pDevObj = pDevCtx->pDevObj;

    switch (deviceType) {
#if defined (VP_CC_790_SERIES)
        case VP_DEV_790_SERIES:
            pDeviceEvents = &((Vp790DeviceObjectType *)pDevObj)->deviceEvents;
            deviceId = ((Vp790DeviceObjectType *)pDevObj)->deviceId;
            break;
#endif

#if defined (VP_CC_880_SERIES)
        case VP_DEV_880_SERIES:
            pDeviceEvents = &((Vp880DeviceObjectType *)pDevObj)->deviceEvents;
            deviceId = ((Vp880DeviceObjectType *)pDevObj)->deviceId;
            break;
#endif

#if defined (VP_CC_890_SERIES)
        case VP_DEV_890_SERIES:
            pDeviceEvents = &((Vp890DeviceObjectType *)pDevObj)->deviceEvents;
            deviceId = ((Vp890DeviceObjectType *)pDevObj)->deviceId;
            break;
#endif

#if defined (VP_CC_580_SERIES)
        case VP_DEV_580_SERIES:
            pDeviceEvents = &((Vp580DeviceObjectType *)pDevObj)->deviceEvents;
            deviceId = ((Vp580DeviceObjectType *)pDevObj)->deviceId;
            break;
#endif

        default:
            return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    pDeviceEvents->response &= (~VP_CSLAC_READ_RESPONSE_MASK);
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}
#endif /*defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) \
    || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES) */
#endif

/**
 * VpCSLACDtmfDigitDetected()
 *  This function is used to set a value in the API-II CSLAC library (where
 * applicable) indicating that a DTMF digit was detected in an external
 * application/process.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  A value in the API-II is set which indicates the digit detected. The most
 * recent value is stored.
 */
#if defined (VP_CC_880_SERIES) || defined (VP_CC_890_SERIES) || defined (VP_CC_790_SERIES)
VpStatusType
VpCSLACDtmfDigitDetected(
    VpLineCtxType *pLineCtx,
    VpDigitType digit,
    VpDigitSenseType sense)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDeviceIdType deviceId;
    VpDeviceType deviceType = pDevCtx->deviceType;
#ifdef VP_CSLAC_SEQ_EN
#if (defined (VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT)) || \
    (defined (VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT)) || \
    defined (VP_CC_790_SERIES)
    VpCallerIdType *pCidStruct = VP_NULL;
#endif
#endif
    VpOptionEventMaskType *pLineEvents;
    uint16p pDtmfDigitSense;

    void *pLineObj = pLineCtx->pLineObj;
    void *pDevObj = pDevCtx->pDevObj;

    switch (deviceType) {
#if defined (VP_CC_790_SERIES)
        case VP_DEV_790_SERIES:
#ifdef VP_CSLAC_SEQ_EN
            pCidStruct = &((Vp790LineObjectType *)pLineObj)->callerId;
#endif
            deviceId = ((Vp790DeviceObjectType *)pDevObj)->deviceId;
            pLineEvents = &((Vp790LineObjectType *)pLineObj)->lineEvents;
            pDtmfDigitSense = &((Vp790LineObjectType *)pLineObj)->dtmfDigitSense;
            break;
#endif

#if defined (VP_CC_880_SERIES)
        case VP_DEV_880_SERIES:
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
            pCidStruct = &((Vp880LineObjectType *)pLineObj)->callerId;
#endif
            deviceId = ((Vp880DeviceObjectType *)pDevObj)->deviceId;
            pLineEvents = &((Vp880LineObjectType *)pLineObj)->lineEvents;
            pDtmfDigitSense = &((Vp880LineObjectType *)pLineObj)->dtmfDigitSense;
            break;
#endif

#if defined (VP_CC_890_SERIES)
        case VP_DEV_890_SERIES:
#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXS_SUPPORT)
            pCidStruct = &((Vp890LineObjectType *)pLineObj)->callerId;
#endif
            deviceId = ((Vp890DeviceObjectType *)pDevObj)->deviceId;
            pLineEvents = &((Vp890LineObjectType *)pLineObj)->lineEvents;
            pDtmfDigitSense = &((Vp890LineObjectType *)pLineObj)->dtmfDigitSense;
            break;
#endif

        default:
            return VP_STATUS_INVALID_ARG;
    }

    switch (sense) {
        case VP_DIG_SENSE_BREAK:
        case VP_DIG_SENSE_MAKE:
#ifdef VP_CSLAC_SEQ_EN
#if (defined (VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT)) || \
    (defined (VP_CC_790_SERIES)) || \
    (defined (VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT))
            /*
             * If the CID sequencer is waiting for a CPE ACK tone, report
             * the DTMF event to the CID sequencer, but mask it from the
             * application.
             */
            if ((pCidStruct != VP_NULL) && ((pCidStruct->status & VP_CID_AWAIT_TONE) != 0)) {
                VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
                pCidStruct->digitDet = digit;
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_SUCCESS;
            }
#endif
#endif
            break;

        default:
            return VP_STATUS_INVALID_ARG;
    }

    /* Toggle the DTMF_DIG event.  If two DTMF_DIG events are received within
     * the same API tick period, report neither. */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    pLineEvents->signaling ^= VP_LINE_EVID_DTMF_DIG;
    *pDtmfDigitSense = digit | sense;
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}
#endif

/**
 * VpGetReverseState()
 *  This function returns the polarity inverted API-II state from the state
 * passed.
 *
 * Preconditions:
 *  None. Helper function only.
 *
 * Postconditions:
 *  None. Helper function only.
 */
VpLineStateType
VpGetReverseState(
    VpLineStateType currentState)
{
    switch(currentState) {
        case VP_LINE_STANDBY:
            return VP_LINE_STANDBY_POLREV;

        case VP_LINE_ACTIVE:
            return VP_LINE_ACTIVE_POLREV;

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
            return VP_LINE_HOWLER_POLREV;

        case VP_LINE_HOWLER_POLREV:
            return VP_LINE_HOWLER;
#endif

        case VP_LINE_ACTIVE_POLREV:
            return VP_LINE_ACTIVE;


        case VP_LINE_TALK:
            return VP_LINE_TALK_POLREV;

        case VP_LINE_TALK_POLREV:
            return VP_LINE_TALK;

        case VP_LINE_OHT:
            return VP_LINE_OHT_POLREV;

        case VP_LINE_OHT_POLREV:
            return VP_LINE_OHT;

        case VP_LINE_RINGING:
            return VP_LINE_RINGING_POLREV;

        case VP_LINE_RINGING_POLREV:
            return VP_LINE_RINGING;

        case VP_LINE_STANDBY_POLREV:
            return VP_LINE_STANDBY;

        default:
            return currentState;
    }
}

#if defined(VP_CC_880_SERIES) || defined(VP_CC_890_SERIES)
/**
 * VpCSLACSetVas()
 *  This function sets the VAS values in dcFeed as specified by the device
 * dc feed register, with VAS value passed. It does not actually access the
 * device, just simply computes the correct hex values for the dc feed reg.
 *
 * Preconditions:
 *  None. Helper function only.
 *
 * Postconditions:
 *  Line not affected. Values in dcFeed contain the VAS values passed.
 */
void
VpCSLACSetVas(
    uint8 *dcFeed,
    uint16 vasValue)
{
    uint16 regValue = 0;

    if (vasValue >= 3000) {
        regValue = (vasValue - 3000) / 750;
    }

    dcFeed[0] &= 0xFC;
    dcFeed[1] &= 0x3F;

    dcFeed[0] |= ((uint8)(regValue & 0xC) >> 2);
    dcFeed[1] |= ((uint8)(regValue & 0x3) << 6);
}

#define VP_CSLAC_ALAW_CODEC     0x00    /**< a-Law compression is used */
#define VP_CSLAC_ULAW_CODEC     0x40    /**< u-law compression is used */
#define VP_CSLAC_LINEAR_CODEC   0x80    /**< Linear mode is used */
/**
 * VpCSLACGetCodecReg()
 *  This function translates the CODEC option provided and modifies the value of codecReg as if
 * it were the silicon CODEC Register (Operating Functions). Note that this doesn't change the WB/NB
 * bit of the EC Register, just the Linear/Compressed Mode bits. It is used at least in the
 * function that initially sets the CODEC mode and VpApiTick() where the "other line" is updated
 * when changing between WB/NB modes. When WB/NB modes are initially changed, the CODEC mode is
 * exactly copied - requiring the ecVal bit (for WB/NB mode) AND the value of Register 0x60/61.
 *
 * Preconditions:
 *  The value of codecReg is assumed to have just been read from register 0x61 with no other bit
 * manipulation. This function will mask off and provide a "ready to write" byte.
 *
 * Postconditions:
 *  This adjusts the value of codecReg provided IF the return value is TRUE. If return is FALSE,
 * then codecReg should be ignored. If TRUE, then the value of codecReg is immediately ready to
 * write back to register 0x60.
 */
bool
VpCSLACGetCodecReg(
    VpOptionCodecType codec,
    uint8 *codecReg)
{
    /*
     * Enable the desired Compression Mode. If using Wideband, the EC bit that controls Narrowband
     * or Wideband is in the line and device object, configured externally. Once the CODEC register
     * is programmed for Compression Mode (i.e., Linear, u-Law or A-Law), WB/NB is implemented in
     * the VP-API-II by maintaining the WB bit in the EC register with the correct value :: '1' for
     * WB, '0' for NB. See the VP-API-II Reference Guide for Timeslot/CODEC timing diagrams.
     */
    switch(codec) {
        case VP_OPTION_LINEAR:          /* 8KHz Sample 16-bit Linear PCM (2 timeslots per frame) */
        case VP_OPTION_WIDEBAND:        /* 16KHz Sample 16-bit Linear PCM (4 timeslots per frame) */
            *codecReg |= VP_CSLAC_LINEAR_CODEC;
            return TRUE;

        case VP_OPTION_ALAW:            /* 8KHz Compressed A-law PCM (1 timeslot per frame) */
        case VP_OPTION_ALAW_WIDEBAND:   /* 16KHz Compressed A-law PCM (2 timeslots per frame) */
            *codecReg &= (uint8)(~(VP_CSLAC_LINEAR_CODEC | VP_CSLAC_ULAW_CODEC));
            return TRUE;

        case VP_OPTION_MLAW:            /* 8KHz Compressed u-law PCM (1 timeslot per frame) */
        case VP_OPTION_MLAW_WIDEBAND:   /* 16KHz Compressed u-law PCM (2 timeslots per frame) */
            *codecReg |= VP_CSLAC_ULAW_CODEC;
            *codecReg &= ~(VP_CSLAC_LINEAR_CODEC);
            return TRUE;

        default:
            return FALSE;
    } /* Switch */
}
#endif /* defined(VP_CC_880_SERIES) || defined(VP_CC_890_SERIES) */

#ifdef CSLAC_GAIN_RELATIVE
/**
 * VpConvertCsd2Fixed()
 *  This function returns a 2.14 fixed-point number whose value matches (as
 * nearly as possible) the value of a given CSD (canonical signed digit)
 * number.
 *
 * Preconditions:
 *  The CSD number must be split into a two-byte array consisting of the high
 * byte followed by the low byte.  Its value must be between 0 and 4.0.
 *
 * Postconditions:
 *  If the value of the passed CSD number is less than 0, 0 will be returned.
 * If the value is greater than or equal to 4.0, 65535 will be returned,
 * which means 3.99994 in 2.14 representation.
 */
uint16
VpConvertCsd2Fixed(
    uint8 *csdBuf)
{
    uint16 csd = (csdBuf[0] << 8) + csdBuf[1];
    int32 result;
    int8 bitPos, C, m;

    /* 2.14 fixed-point format has values ranging from 0 to 3.999.... The bits
     * have the following values:
     *
     *   bit    15    14    13    12          2     1     0
     *       +-----+-----+-----+-----+     +-----+-----+-----+
     * value | 2^1 | 2^0 | 2^-1| 2^-2| ... |2^-12|2^-13|2^-14|
     *       +-----+-----+-----+-----+     +-----+-----+-----+
     */

    /*
     * CSD format is as follows:
     *
     *   bit   15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
     *       +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
     * field |C40|    m40    |C30|    m30    |C20|    m20    |C10|    m10    |
     *       +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
     *
     * where the represented value is calculated as follows (Cxy = 0 or 1
     * above corresponds to Cxy = 1 or -1, respectively, below):
     *
     *      C10 * 2^-m10 * (
     *          1 + C20 * 2^-m20 * (
     *              1 + C30 * 2^-m30 * (
     *                  1 + C40 * 2^-m40 )))
     */

    /*
     * Alternate formula which can be computed with unsigned ints:
     *
     *          C10 * 2^-m10
     *        + C10 * C20 * 2^(-m10-m20)
     *        + C10 * C20 * C30 * 2^(-m10-m20-m30)
     *        + C10 * C20 * C30 * C40 * 2^(-m10-m20-m30-m40)
     */

    C = m = result = 0;
    for (bitPos = 0; bitPos < 16; bitPos += 4) {
        C ^= (csd >> (bitPos + 3)) & 1;
        m += (csd >> bitPos) & 7;
        if (C == 0) {
            result += 0x4000 >> m;
        } else {
            result -= 0x4000 >> m;
        }
    }

    /* Ensure that the result can be stored in a uint16. */
    if (result > 0xFFFF) {
        result = 0xFFFF;
    } else if (result < 0) {
        result = 0;
    }

    return result;
} /* VpConvertCsd2Fixed() */

/**
 * VpConvertFixed2Csd()
 *  This function returns a four-nibble CSD (canonical signed digit) number
 * whose value matches (as nearly as possible) the supplied 2.14 fixed-point
 * number.
 *
 * Preconditions:
 *
 * Postconditions:
 *  The CSD number will be placed into a two-byte array (high byte first) at
 * the address specified in the csdBuf parameter.
 */
void
VpConvertFixed2Csd(
    uint16 fixed,
    uint8 *csdBuf)
{
#define CSD_NIBBLES 4
    uint16 error, power, greaterPower, smallerPower, distGreater, distSmaller;
    uint16 C, m, result, sum = 0;
    int8 n, gp, sp;

    /* Data structure for holding the four terms composing the CSD number. */
    typedef struct {
        bool sign;
        int power;
    } term;
    term t[CSD_NIBBLES + 1];
    t[0].power = 0;
    t[0].sign = 0;

    /*
     * Split the 2.14 value into a sum of powers of 2,
     *   s1 * 2^p1  +  s2 * 2^p2  +  s3 * 2^p3  +  s4 * 2^p4
     * where for term x,
     *   sx = 1 or -1,
     *   px <= 0.
     */
    for (n = 1; n <= CSD_NIBBLES; n++) {

        if (sum == fixed) break;

        /*
         * If current sum is less than actual value, then the next term
         * should be added; otherwise the next term should be
         * subtracted.
         */
        if (sum < fixed) {
            t[n].sign = 0;
            error = fixed - sum;
        } else {
            t[n].sign = 1;
            error = sum - fixed;
        }

        /* If error > 1, then term = +/-1. */
        if (error > 0x4000) {
            t[n].power = 0;
        } else {

            /*
             * Calculate greaterPower = the smallest power of 2 greater
             * than error.  Calculate smallerPower = the largest power
             * of 2 less than error.
             */
            greaterPower = 0x4000; gp = 0;
            for (power = 0x2000; power > error; power >>= 1) {
                greaterPower >>= 1; gp--;
            }
            smallerPower = greaterPower >> 1; sp = gp - 1;

            /*
             * Is error closer to greaterPower or smallerPower?
             * Whichever is closer, choose that for the value of the
             * next term.
             */
            distGreater = greaterPower - error;
            distSmaller = error - smallerPower;
            if (distGreater < distSmaller) {
                t[n].power = gp;
            } else {
                t[n].power = sp;
            }

            /*
             * The power of this term can differ from the power of the
             * previous term by no more than 7.
             */
            if (t[n - 1].power - t[n].power > 7) {
                t[n].power = t[n - 1].power - 7;
            }
        }

        /* Add or subtract the term to the sum, depending on sign. */
        if (t[n].sign == 0) {
            sum += (uint16)1 << (14 + t[n].power);
        } else {
            sum -= (uint16)1 << (14 + t[n].power);
        }
    }

    /*
     * If we reached the exact value with terms left over, fill these
     * extra terms with dummy values which don't affect the CSD value.
     */
    while (n <= CSD_NIBBLES) {
        if (n == 1) {
            t[1] = t[0];
            t[2].power = 0;
            t[2].sign = 1;
            n += 2;
        } else {
            /*
             * Increase the number of terms by replacing the last term
             * with two new terms whose sum is the old term.
             */
            if (t[n - 1].power == t[n - 2].power) {
                t[n - 1].power--;
                t[n] = t[n - 1];
            } else {
                t[n] = t[n - 1];
                t[n - 1].power++;
                t[n].sign = !(t[n - 1].sign);
            }
            n++;
        }
    }

    /* Compute nibble values from the terms. */
    result = 0;
    for (n = 1; n <= CSD_NIBBLES; n++) {
        int8 bitPos = (n - 1) * 4;
        C = (t[n].sign != t[n - 1].sign);
        m = -(t[n].power - t[n - 1].power);
        result |= (C << (bitPos + 3)) | (m << bitPos);
    }

    /* Split the uint16 result into high and low bytes. */
    csdBuf[0] = (uint8)(result >> 8);
    csdBuf[1] = (uint8)(result & 0xFF);
} /* VpConvertFixed2Csd() */
#endif
#endif

#if (defined(VP_CC_890_SERIES) && defined(VP890_INCLUDE_TESTLINE_CODE)) \
    || (defined(VP_CC_880_SERIES) && defined(VP880_INCLUDE_TESTLINE_CODE)) \
    || defined(VP_CC_792_SERIES) || defined(VP_CC_886_SERIES)
uint16
VpComputeSquareRoot(
    uint32 number)
{
    uint8 iteration;
    int32 sqrtEst = 2;
    const int32 sqrtShift = number / 2;
    const uint8 newtonItt = 3;

    /*
     * Find an estimate of the result in the correct octave
     * (approximately 1.5 bits of accuracy)
     */
    while ((sqrtEst * sqrtEst) < sqrtShift) {
        sqrtEst *= 2;
    }

    /*
     * Use Newton's iteration to improve the estimate of the square root
     * If the accuracy is N bits, on Newton's iteration increase the accuracy
     * to 2N+1 bits.
     */
     for (iteration = 0; iteration < newtonItt; iteration++) {
        if (0 == sqrtEst) {
            break;
        } else {
            sqrtEst = (sqrtEst +  (number / sqrtEst)) / 2 ;
        }
     }

    return (uint16)sqrtEst;
}
#endif

#ifdef VP886_INCLUDE_DTMF_DETECT
/* e = 2.71828183
 * Used in VpCompute_ln at *10000 scale */
#define VP_MATH_E 27183

/* ln(10) = 2.302585092
 * Used in VpCompute_log10 at *10000 scale */
#define VP_MATH_LN_10 23026

#ifndef VP_UINT32_MAX
#define VP_UINT32_MAX ((uint32)(0xFFFFFFFF))
#endif
/*
 * Returns approximately:
 *      ln(number) * 10000
 *
 * To compute ln() of a non-integer, call this function multiple times to use
 * the rule: ln(x/y) = ln(x) - ln(y)
 * For example, ln(12.75) = ln(1275/100) = ln(1275) - ln(100)
 *
 * To compute ln() of a large number, beyond the uint16 range, use multiple
 * calls to employ the rule: ln(x*y) = ln(x) + ln(y)
 * For example, ln(20000000) = ln(2000*10000) = ln(2000) + ln(10000)
 * In general, for x >= 2^16, ln(x) = 2 * ln(sqrt(x))
 *
 */
int32
VpCompute_ln (
    uint16 number)
{   
    uint32 X;
    uint32 A;
    uint32 B;
    int32 C;
    int32 seriesSum;
    int32 numerator;
    int32 denominator;
    int32 term;

    if (number == 1) {
        return 0;
    }
    
    if (number == 0) {
        return VP_INT32_MIN;
    }

    /* Scale up to 10000x original units */
    X = number * 10000;
    
    A = 0;
    B = X;
    
    /* ln(X) = ln(e^A * B) 
     *   Find A and B where A is a whole number, and 0 < B < 2.
     *   Start with A = 0 and B = X, then increment A and divide B by e until
     *   B is less than 2.
     *  Using 1.4621 as the limit instead of 2 increases accuracy and reduces
     *  the number of iterations required in the following step.  If B ends up
     *  near 2, that number of iterations increases drastically.  This limit
     *  will still fulfill the requirement 0 < B < 2.  It is chosen to make
     *  1.0 the midpoint for possible B results:
     *      1.4621 - 1 = 1 - 1.4621/e
     */
    while (B >= 14621) {
        /* Different scaling/rounding during division depending on the starting
         * value to preserve as much precision as possible */
        if (B < VP_UINT32_MAX / 10000) {
            B = (B * 10000) / VP_MATH_E;
        } else if (B < VP_UINT32_MAX / 1000) {
            B = ((B * 1000) / VP_MATH_E) * 10 + 5;
        } else if (B < VP_UINT32_MAX / 100) {
            B = ((B * 100) / VP_MATH_E) * 100 + 50;
        } else if (B < VP_UINT32_MAX / 10) {
            B = ((B * 10) / VP_MATH_E) * 1000 + 500;
        } else {
            B = (B / VP_MATH_E) * 10000 + 5000;
        }
        
        A += 10000;        
    }
    
    /* ln(e^A * B) = ln(e^A) + ln(B) = A + ln(B)
     *   We already have A, so we need to calculate ln(B).
     *   Since 0 < B < 2, we can define C = B - 1 such that -1 < C < -1
     *   and then use a Taylor series expansion to calculate ln(C + 1) = ln(B).
     *   The Taylor series for ln(x+1) for -1 < x <= 1 is:
     *      ln(x+1) = x - (x^2)/2 + (x^3)/3  - (x^4)/4 + (x^5)/5 - ...
     *      term(n) = (numerator(n-1) * x * -1) / (denominator(n-1) + 1)
     *  The iteration limit will be 15 terms or whenever the terms reach 0 due
     *  to integer division.  Based on testing, the 15 term limit is never
     *  reached for any uint16 input.
     */
    C = B - 10000;
    seriesSum = 0;
    numerator = C;
    denominator = 1;
    term = C;
    while (denominator <= 15 && term != 0) {
        seriesSum += term;
        
        numerator = (-1) * (numerator * C) / 10000;
        denominator += 1;
        term = numerator / denominator;
    }

    return A + seriesSum;
}

/*
 * Returns approximately:
 *      log10(number) * 10000
 */
int32
VpCompute_log10 (
    uint16 number)
{
    /* log10(x) == ln(x) / ln(10) */
    uint32 lnResult = VpCompute_ln(number);
    return (lnResult * 10000) / VP_MATH_LN_10;
}

/* Performs a table lookup with linear interpolation between entries.
   Table key values must be in ascending order */
int32
VpTableInterpolate(
    const int32 table[][2],
    uint16 size,
    int32 x,
    bool reverse)
{
    int32 interp;
    int32 key, val;
    int32 x1, y1, x2, y2;
    int32 t1, t2, t3;
    uint16 i;
    
    for (i = 0; i < size; i++) {
        if (!reverse) {
            key = table[i][0];
            val = table[i][1];
        } else {
            key = table[i][1];
            val = table[i][0];
        }
        if (key == x) {
            return val;
        }
        if (key > x) {
            break;
        }
    }

    if (i == 0) {
        /* Extrapolate down */
        x1 = table[i][0];
        y1 = table[i][1];
        x2 = table[i+1][0];
        y2 = table[i+1][1];
        
    } else if (i == size) {
        /* Extrapolate up */
        x1 = table[i-2][0];
        y1 = table[i-2][1];
        x2 = table[i-1][0];
        y2 = table[i-1][1];        
    } else {
        /* Interpolate between two entries */
        x1 = table[i-1][0];
        y1 = table[i-1][1];
        x2 = table[i][0];
        y2 = table[i][1];
    }
    if (reverse) {
        int32 temp;
        temp = x1;
        x1 = y1;
        y1 = temp;
        temp = x2;
        x2 = y2;
        y2 = temp;
    }
    t1 = x - x1;
    t2 = x2 - x1;
    t3 = y2 - y1;

    /* Decide best order of operations to preserve precision while avoidin
       overflow. */
    if ((t3 * t1) / t1 == t3) {
        interp = VpRoundedDivide(t3 * t1, t2) + y1;
    } else if (t3 > t1) {
        interp = VpRoundedDivide(t3, t2) * t1 + y1;        
    } else {
        interp = VpRoundedDivide(t1, t2) * t3 + y1;        
    }
        
    return interp;
}

#endif /* VP886_INCLUDE_DTMF_DETECT */

/**
 * VpMemCpyCheck - Copy one area of memory to another while checking if any of
 * the data changed.
 *
 * @dest: Where to copy to
 * @src: Where to copy from
 * @count: The size of the area.
 *
 * Return: TRUE if any of the data from-to was different.
 */
EXTERN bool
VpMemCpyCheck(
    uint8 *dest,
    uint8 *src,
    uint16 count)
{
    bool dataChange = FALSE;
    uint16 currentIndex = 0;

    while (currentIndex < count) {
        currentIndex++;
        if (dest[currentIndex] != src[currentIndex]) {
            dataChange = TRUE;
            dest[currentIndex] = src[currentIndex];
        }
    }
    return dataChange;
}

/**
 * memcpy - Copy one area of memory to another
 * @dest: Where to copy to
 * @src: Where to copy from
 * @count: The size of the area.
 *
 */
EXTERN void *
VpMemCpy(
    void * dest,
    const void *src,
    uint16 count)
{
    char *tmp = (char *) dest, *s = (char *) src;

    while (count--)
        *tmp++ = *s++;

    return dest;
}

/**
 * memset - Fill a region of memory with the given value
 * @s: Pointer to the start of the area.
 * @c: The byte to fill the area with
 * @count: The size of the area.
 */
EXTERN void *
VpMemSet(
    void * s,
    int c,
    uint16 count)
{
    char *xs = (char *) s;

    while (count--)
        *xs++ = (char)c;

    return s;
}

#if defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) \
 || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES)
/**
 * Wrapper for VpMpiCmd() for purposes of providing VP_DBG_HAL output
 */
void
VpMpiCmdWrapper(
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 mpiCmd,
    uint8 mpiCmdLen,
    uint8 *dataBuffer)
{
    VpMpiCmd(deviceId, ecVal, mpiCmd, mpiCmdLen, dataBuffer);

#if (VP_CC_DEBUG_SELECT & VP_DBG_HAL)

#ifdef MPI_SHORT_FORMAT
    if (mpiCmd == 0xCF) {
        VP_HAL(None, NULL, ("Total Length: 17"));
    } else {
        VP_HAL(None, NULL, ("EC Value: 0x%02X Type: %s Cmd: 0x%02X Total Length: %d",
            ecVal, ((mpiCmd & 0x1) ? "READ" : "WRITE"), mpiCmd, (3 + mpiCmdLen)));
    }
#endif
#ifdef MPI_LONG_FORMAT
    {
        uint8 cmdIndex;

        if (mpiCmd == 0xCF) {
            VP_HAL(None, NULL, ("Cmd 0xCF - Read Length: 17"));
        } else if (mpiCmd == 0xCD) {
        } else {
            VP_HAL(None, NULL, ("EC 0x%02X %s Cmd 0x%02X", ecVal, ((mpiCmd & 0x1) ? "READ" : "WRITE"), mpiCmd));
            for (cmdIndex = 0; cmdIndex < mpiCmdLen; cmdIndex++) {
                VP_HAL(None, NULL, (" 0x%02X", dataBuffer[cmdIndex]));
            }
        }
    }
#endif
#ifdef MPI_CMD_SEARCH
{
    int16 cmdIndex = VpMpiFindCmd(MPI_CMD_TO_FIND, mpiCmd, mpiCmdLen, dataBuffer);

    if (cmdIndex >= 0) {
        VP_HAL(None, NULL, ("%s Cmd 0x%02X", ((mpiCmd & 0x1) ? "READ" : "WRITE"), MPI_CMD_TO_FIND));
        cmdIndex = ((mpiCmd == MPI_CMD_TO_FIND) ? 0 : 1);
        for (; cmdIndex < mpiCmdLen; cmdIndex++) {
            VP_HAL(None, NULL, (" 0x%02X", dataBuffer[cmdIndex]));
        }
    }
}
#endif
#endif
}

#if (VP_CC_DEBUG_SELECT & VP_DBG_HAL) && defined (MPI_CMD_SEARCH)
/**
 * Function primarily used to find a specific write command that is part of the
 * MPI data buffer. A simple implementation looks just through the data buffer
 * for a raw match, which could come from MPI data rather than command. A more
 * complex implementation could use known commands+cmd_len to match for commands
 * only.
 */
int16
VpMpiFindCmd(
    uint8 byteMatch,
    uint8 mpiCmd,
    uint8 mpiCmdLen,
    uint8 *dataBuffer)
{
    int16 indexCnt = 0;
    if (mpiCmd == byteMatch) {
        return 0;
    }

    for (indexCnt = 0; indexCnt < mpiCmdLen; indexCnt++) {
        if (dataBuffer[indexCnt] == byteMatch) {
            return indexCnt;
        }
    }
    return -1;
}
#endif

/* Used for buffering MPI data to reduce MPI traffic */
uint8
VpCSLACBuildMpiBuffer(
    uint8 index,
    uint8 *mpiBuffer,
    uint8 mpiCmd,
    uint8 mpiCmdLen,
    const uint8 *mpiData)
{
    mpiBuffer[index++] = mpiCmd;
    VpMemCpy(&mpiBuffer[index], mpiData, (uint16)mpiCmdLen);

    return (index + mpiCmdLen);
}
#endif /* defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) \
 || defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES) */

#ifdef VP_CC_SLAC_BUF_START
/**
 * Internal API functions that need to perform buffered writes to the
 * device must first call this function. Its purpose is to reset/prepare the
 * VpSlacBufData structure in the device object.
 *
 * Between calls to VpSlacBufStart() and VpSlacBufSend(), all calls to
 * VpSlacRegWrite() will add to the buffer instead of writing immediately.  To
 * maintain coherency, VpSlacRegRead() will write the buffer contents before
 * performing a read.
 */
bool
VpSlacBufStart(
    VpDevCtxType *pDevCtx)
{
    if (pDevCtx == NULL) {
        VP_HAL(None, NULL, ("VpSlacBufStart(): Invalid NULL pDevCtx"));
        return FALSE;
    }

    if (pDevCtx->funPtrsToApiFuncs.SlacBufStart == VP_NULL) {
        VP_HAL(VpDevCtxType, pDevCtx, ("VpSlacBufStart(): Function not supported by device type"));
        return FALSE;
    }

    return pDevCtx->funPtrsToApiFuncs.SlacBufStart(pDevCtx);
}
#endif /* VP_CC_SLAC_BUF_START */

#ifdef VP_CC_SLAC_BUF_SEND
/**
 * Once all necessary write commands have been stored in the VpSlacBufData
 * struct, this function must be called to write the data to the device. Its
 * intended purpose is to pass the VpSlacBufData struct's write buffer to the
 * appropriate HAL write function.
 *
 * This function can accept either a device context or line context, and the
 * device-specific implementation can format the final output based on which
 * is provided.
 * If both pDevCtx and pLineCtx are NULL the function will return FALSE.
 *
 * This function will perform error checks on the in the VpSlacBufData struct
 * before attempting to call the HAL write function. If the data in the struct
 * is successfully transferred to the device the function will return TRUE.
 */
bool
VpSlacBufSend(
    VpDevCtxType *pDevCtx)
{
    if (pDevCtx == NULL) {
        VP_HAL(None, NULL, ("VpSlacBufSend(): Invalid NULL pDevCtx"));
        return FALSE;
    }

    if (pDevCtx->funPtrsToApiFuncs.SlacBufSend == VP_NULL) {
        VP_HAL(VpDevCtxType, pDevCtx, ("VpSlacBufSend(): Function not supported by device type"));
        return FALSE;
    }

    return pDevCtx->funPtrsToApiFuncs.SlacBufSend(pDevCtx);
}
#endif /* VP_CC_SLAC_BUF_SEND */

#ifdef VP_CC_SLAC_REG_WRITE
/**
 * Performs buffered and unbuffered writes to the device.  If buffering has been
 * started by VpSlacBufStart(), this function will add to the write buffer.  If
 * buffering is not currently active, it will perform an immediate write.
 *
 * This function can accept either a device context or line context, and the
 * device-specific implementation can format the final output based on which
 * is provided.
 * If both pDevCtx and pLineCtx are NULL the function will return FALSE.
 *
 * This function will perform error checks on the input data before attempting
 * to access the device. The function returns TRUE if successful.
 */
bool
VpSlacRegWrite(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 writeLen,
    const uint8 *pWriteBuf)
{
    VpDevCtxType *pDevCtxLocal = pDevCtx;

    if (pDevCtx == NULL && pLineCtx == NULL) {
        VP_HAL(None, NULL, ("VpSlacRegWrite(): Invalid NULL pDevCtx and pLineCtx"));
        return FALSE;
    }

    if (pDevCtxLocal == NULL) {
        pDevCtxLocal = pLineCtx->pDevCtx;
    }

    if (pDevCtxLocal->funPtrsToApiFuncs.SlacRegWrite == VP_NULL) {
        VP_HAL(VpDevCtxType, pDevCtxLocal, ("VpSlacRegWrite(): Function not supported by device type"));
        return FALSE;
    }

    return pDevCtxLocal->funPtrsToApiFuncs.SlacRegWrite(pDevCtx, pLineCtx, cmd, writeLen, pWriteBuf);
}
#endif /* VP_CC_SLAC_REG_WRITE */

#ifdef VP_CC_SLAC_REG_READ
/**
 * Internal API functions that need to retrieve data from the device
 * should call this function.
 *
 * If buffering has been started by VpSlacBufStart() and the write buffer is
 * not empty, this function will write the current contents of the buffer to the
 * device before performing the read.  This is done to prevent write->read
 * errors when buffering while allowing internal code to behave the same way
 * regardless of whether buffering is enabled or not.
 *
 * This function can accept either a device context or line context, and the
 * device-specific implementation can take different actions based on which
 * is provided.
 * If both pDevCtx and pLineCtx are NULL the function will return FALSE.
 *
 * This function will perform error checks on the input data before attempting
 * to access the device. The function returns TRUE if successful.
 */
bool
VpSlacRegRead(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 readLen,
    uint8 *pReadBuf)
{
    VpDevCtxType *pDevCtxLocal = pDevCtx;

    if (pDevCtx == NULL && pLineCtx == NULL) {
        VP_HAL(None, NULL, ("VpSlacRegRead(): Invalid NULL pDevCtx and pLineCtx"));
        return FALSE;
    }

    if (pDevCtxLocal == NULL) {
        pDevCtxLocal = pLineCtx->pDevCtx;
    }

    if (pDevCtxLocal->funPtrsToApiFuncs.SlacRegRead == VP_NULL) {
        VP_HAL(VpDevCtxType, pDevCtxLocal, ("VpSlacRegRead(): Function not supported by device type"));
        return FALSE;
    }

    return pDevCtxLocal->funPtrsToApiFuncs.SlacRegRead(pDevCtx, pLineCtx, cmd, readLen, pReadBuf);
}
#endif /* VP_CC_SLAC_REG_READ */



