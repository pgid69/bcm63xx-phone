/** \file vp886_control_common.c
 * vp886_control_common.c
 *
 *  This file contains the control functions for the Vp886 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11638 $
 * $LastChangedDate: 2014-11-13 18:34:43 -0600 (Thu, 13 Nov 2014) $
 */
#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_pulse_decode.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "sys_service.h"

#ifdef VP886_INCLUDE_DTMF_DETECT
#include "vp_dtmf_detect.h"
#endif

static void
Vp886SetLineStateFxsDetectFreeze(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

static void
Vp886SetLineStateFxsWorkarounds(
    VpLineCtxType *pLineCtx,
    VpLineStateType prevState,
    VpLineStateType newState);

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
static VpStatusType
Vp886SetLineStateFxsHowler(
    VpLineCtxType *pLineCtx,
    VpLineStateType toState,
    VpLineStateType fromState);

static void
Vp886HighGainEnter(
    VpLineCtxType *pLineCtx);

static void
Vp886HighGainExit(
    VpLineCtxType *pLineCtx);
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */

static VpStatusType
Vp886MapLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state,
    uint8 *stateReg);

static VpStatusType
Vp886SetLineToneInt(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pCadProfile,
    VpDtmfToneGenType *pDtmfControl);

static bool
Vp886RingSyncBegin(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

static VpStatusType
Vp886SetRelGainInt(
    VpLineCtxType   *pLineCtx);

static VpStatusType
Vp886SetOptionDevice(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue);

static VpStatusType
Vp886SetOptionLine(
    VpLineCtxType *pLineCtx,
    VpOptionIdType option,
    void *pValue);

static VpStatusType
Vp886SetOptionTimeslot(
    VpLineCtxType *pLineCtx,
    VpOptionTimeslotType *pTimeslot);

static VpStatusType
Vp886SetOptionCodec(
    VpLineCtxType *pLineCtx,
    VpOptionCodecType *pCodec);

static VpStatusType
Vp886SetOptionIoCfg(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 direction,
    uint8 outputType,
    uint8 *pIoDirReg);

static VpStatusType
Vp886SetOptionDcFeedParams(
    VpLineCtxType *pLineCtx,
    VpOptionDcFeedParamsType *pDcFeedParams);

static VpStatusType
Vp886SetOptionRingingParams(
    VpLineCtxType *pLineCtx,
    VpOptionRingingParamsType *pRingingParams);

#ifdef VP886_INCLUDE_DTMF_DETECT
VpStatusType
Vp886SetOptionDtmfMode(
    VpLineCtxType *pLineCtx,
    VpOptionDtmfModeType *pDtmfMode);
#endif

static void
Vp886ApplyInternalTestTerm(
    VpLineCtxType *pLineCtx);

static void
Vp886RemoveInternalTestTerm(
    VpLineCtxType *pLineCtx);

static VpStatusType
Vp886DeviceIoAccessInt(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData);


#ifdef VP886_INCLUDE_DTMF_DETECT
#define VP_DECIBEL_TABLE_SIZE (sizeof(VpDecibelTable) / (2 * sizeof(int32)))
static const int32 VpDecibelTable[][2] = {
    /* {dB*10, multiplier*100} */
    {-100, 10},
    {-90, 13},
    {-80, 16},
    {-70, 20},
    {-60, 25},
    {-50, 32},
    {-40, 40},
    {-30, 50},
    {-20, 63},
    {-10, 79},
    {0, 100},
    {10, 126},
    {20, 158},
    {30, 200},
    {40, 251},
    {50, 316},
    {60, 398},
    {70, 501},
    {80, 631},
    {90, 794},
    {100, 1000},
    {110, 1259},
    {120, 1585},
    {130, 1995},
    {140, 2512},
    {150, 3162},
    {160, 3981},
    {170, 5012},
    {180, 6310},
    {190, 7943},
    {200, 10000},
    {210, 12589},
    {220, 15849},
    {230, 19953},
    {240, 25119},
    {250, 31623},
    {260, 39811},
    {270, 50119},
    {280, 63096},
    {290, 79433},
    {300, 100000},
    {310, 125893},
    {320, 158489},
    {330, 199526},
    {340, 251189},
    {350, 316228},
    {360, 398107},
    {370, 501187},
    {380, 630957},
    {390, 794328},
    {400, 1000000},
    {410, 1258925},
    {420, 1584893},
    {430, 1995262},
    {440, 2511886},
    {450, 3162278},
    {460, 3981072},
    {470, 5011872},
    {480, 6309573},
    {490, 7943282},
    {500, 10000000},
    {510, 12589254},
    {520, 15848932},
    {530, 19952623},
    {540, 25118864},
    {550, 31622777},
    {560, 39810717},
    {570, 50118723},
    {580, 63095734},
    {590, 79432823},
    {600, 100000000},
    {610, 125892541},
    {620, 158489319},
    {630, 199526231},
    {640, 251188643},
    {650, 316227766},
    {660, 398107171},
    {670, 501187234},
    {680, 630957344},
    {690, 794328235},
    {700, 1000000000},
    {710, 1258925412},
    {720, 1584893192},
    {730, 1995262315}
};
#endif /* VP886_INCLUDE_DTMF_DETECT */

/** Vp886SetLineState()
  Implements VpSetLineState() to set the line to the requested state.

  This higher level function performs readiness and argument checks, then
  calls into either Vp886SetLineStateFxs() or Vp886SetLineStateFxo() depending
  on line type.

  See the VP-API-II Reference Guide for more details on VpSetLineState().
*/
VpStatusType
Vp886SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886SetLineState");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineState");
        return status;
    }

    if (!Vp886IsValidLineState(pLineCtx, state)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetLineState: Invalid line state %d", state));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineState");
        return VP_STATUS_INVALID_ARG;
    }

    if (pLineObj->isFxs) {
        status = Vp886SetLineStateFxs(pLineCtx, state);
    } else {
        status = VP_STATUS_FUNC_NOT_SUPPORTED;
    }

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineState");
    return status;
}


/** Vp886SetLineStateFxs()
  Implements user-level line state changes for FXS lines.

  This function will abort ongoing sequences (ring cadences, caller ID,
  metering, VpSendSignal, etc) and will change the external line state that
  can be seen with VpGetLineState().

  This can be called internally to have the same effect as the application
  calling VpSetLineState().  This is appropriate for actions such as ring trip
  exit and critical fault handling.  To change line states during a sequence or
  signal without changing the API line state or interrupting other activity,
  use Vp886SetLineStateFxsInt().
*/
VpStatusType
Vp886SetLineStateFxs(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    VpLineStateType currentState = pLineObj->lineState.usrCurrent;

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886SetLineStateFxs - state %d", state));

    /* Turn off tone generators if moving to a non-codec state */
    if (!Vp886LineStateInfo(state).codec) {
        Vp886SetToneCtrl(pLineCtx, FALSE, FALSE, FALSE, FALSE, FALSE);
    }

#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
    if (state == VP_LINE_STANDBY) {
        /* Ringing cadence started */
        pLineObj->wasStandby = TRUE;
    }

    if (((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)) && pLineObj->wasStandby){
        /* Ringing cadence started */
        pLineObj->wasStandby = FALSE;
        pLineObj->startRingingCadence = TRUE;
    }
#endif

#ifdef VP_CSLAC_SEQ_EN
    /* End any active cadence unless we are moving to a codec state and the
       cadence is a tone cadence */
    if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
        if (!Vp886LineStateInfo(state).codec ||
            pLineObj->cadence.pActiveCadence[VP_PROFILE_TYPE_LSB] != VP_PRFWZ_PROFILE_TONECAD)
        {
            Vp886CadenceStop(pLineCtx, TRUE, FALSE, TRUE);
        }
    }
    if (pLineObj->metering.active) {
        Vp886MeterStop(pLineCtx, FALSE);
    }
    if (pLineObj->sendSignal.active) {
        Vp886SendSignalStop(pLineCtx, FALSE);
    }
    if (pLineObj->cid.active) {
        Vp886CidStop(pLineCtx);
    }

    if (pLineObj->gndFltProt.state == VP886_GNDFLTPROT_ST_GKEY_DETECTED &&
        state == VP_LINE_DISCONNECT)
    {
        /* If the ground fault protection state machine is waiting for a ground
           key to be qualified and the application sets the state to disconnect
           (presumably in response to the groundkey) then stop the protection
           sequence.
           One exception: the protection state machine itself sets the line
           state to disconnect.  Only stop it if this is not the case. */
        if (pLineObj->gndFltProt.settingDisconnect) {
            pLineObj->gndFltProt.settingDisconnect = FALSE;
        } else {
            Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_STOP);
        }
    } else if (pLineObj->gndFltProt.state != VP886_GNDFLTPROT_ST_INACTIVE) {
        Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_LINESTATE);
    }

    if (Vp886LineStateInfo(state).normalEquiv == VP_LINE_RINGING) {
        bool offhook;
        VpCSLACGetLineStatus(pLineCtx, VP_INPUT_HOOK, &offhook);
        if (offhook) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Already offhook, going to ring trip state"));
            state = pLineObj->options.ringControl.ringTripExitSt;
        }
    }

    if (Vp886LineStateInfo(state).normalEquiv == VP_LINE_RINGING &&
        pLineObj->pRingingCadence != VP_PTABLE_NULL)
    {
        pLineObj->lineState.usrCurrent = state;
        return Vp886CadenceStart(pLineCtx, pLineObj->pRingingCadence, 0);
    }
#endif /* VP_CSLAC_SEQ_EN */

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
    if (Vp886LineStateInfo(state).normalEquiv == VP_LINE_HOWLER ||
        Vp886LineStateInfo(currentState).normalEquiv == VP_LINE_HOWLER)
    {
        return Vp886SetLineStateFxsHowler(pLineCtx, state, currentState);
    }
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */

    pLineObj->lineState.usrCurrent = state;

    status = Vp886SetLineStateFxsInt(pLineCtx, state);
    if (status != VP_STATUS_SUCCESS) {
        pLineObj->lineState.usrCurrent = currentState;
        return status;
    }

    return status;
}


/** Vp886SetLineStateFxsInt()
  Implements low-level line state changes for FXS lines.

  This includes all of the workarounds and other settings that go along with
  different states and transitions, but does not interrupt ongoing sequences
  or change the application-visible API line state.

  This function should be called internally to change line states during
  sequences or workarounds.
*/
VpStatusType
Vp886SetLineStateFxsInt(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineStateInfoType currentStateInfo;
    Vp886LineStateInfoType newStateInfo;
    uint8 stateReg = 0x00;
    VpStatusType status;
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
    bool lineTestInProgress = FALSE;
#endif

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886SetLineStateFxsInt - state %d", state));

    currentStateInfo = Vp886LineStateInfo(pLineObj->lineState.currentState);
    newStateInfo = Vp886LineStateInfo(state);

    /* Don't enter a ringing phase if already offhook */
    if (newStateInfo.normalEquiv == VP_LINE_RINGING &&
        (pLineObj->lineState.condition & VP_CSLAC_HOOK))
    {
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Not entering ringing due to existing offhook"));
        return VP_STATUS_SUCCESS;
    }
    /* Don't enter a ringing phase if using balanced ringing and a groundkey is
       detected */
    if (newStateInfo.normalEquiv == VP_LINE_RINGING &&
        !pLineObj->unbalancedRinging &&
        (pLineObj->lineState.condition & VP_CSLAC_GKEY))
    {
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Not entering ringing due to existing groundkey"));
        return VP_STATUS_SUCCESS;
    }

    if (newStateInfo.normalEquiv == VP_LINE_RINGING) {
        /* Check if we need to synchronize ringing with the other channel */
        if (pDevObj->options.ringPhaseSync == VP_RING_PHASE_SYNC_90_OFFSET &&
            Vp886RingSyncBegin(pLineCtx, state))
        {
            /* Ring sync has been started, exit and allow timers to do the rest */
            return VP_STATUS_SUCCESS;
        }

        /* Restore the RINGDELAY register to default or base value. */
        if (VP886_IS_SF(pDevObj)) {
            /* If the option is enabled, use the base delay.  This is what the
               non-delayed line will be set to when both lines ring. Using
               the base delay in the normal solo-ringing case will use less MPI
               than using the default because we won't need to change it as
               often. */
            if (pDevObj->options.ringPhaseSync == VP_RING_PHASE_SYNC_90_OFFSET &&
                pLineObj->ringSyncState != VP886_RINGSYNC_STATE_FINISHING_DELAY &&
                pLineObj->registers.ringDelay[0] != VP886_R_RINGDELAY_BASE)
            {
                pLineObj->registers.ringDelay[0] = VP886_R_RINGDELAY_BASE;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGDELAY_WRT, VP886_R_RINGDELAY_LEN, pLineObj->registers.ringDelay);
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886SetLineStateFxsInt - ring delay 0x%02X", pLineObj->registers.ringDelay[0]));
            }
            /* If the option is disabled, use the default value */
            if (pDevObj->options.ringPhaseSync == VP_RING_PHASE_SYNC_DISABLE &&
                pLineObj->ringSyncState != VP886_RINGSYNC_STATE_FINISHING_DELAY &&
                pLineObj->registers.ringDelay[0] != VP886_R_RINGDELAY_DEFAULT)
            {
                pLineObj->registers.ringDelay[0] = VP886_R_RINGDELAY_DEFAULT;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGDELAY_WRT, VP886_R_RINGDELAY_LEN, pLineObj->registers.ringDelay);
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886SetLineStateFxsInt - ring delay 0x%02X", pLineObj->registers.ringDelay[0]));
            }
        }
    }

    /* If exiting ringing to a state of the opposite polarity of the ringing,
       not performing a msg waiting pulse signal, and the new state has an
       opposite polarity equivalent:
       First go to the equivalent opposite polarity state, then start checking
       the device ring exit state.  Set the final line state after the ring
       exit state clears. */
    /* TODO: Maybe put this in a separate function.  SetLineStateFxsOverrides? */
    if (currentStateInfo.normalEquiv == VP_LINE_RINGING &&
        newStateInfo.normalEquiv != VP_LINE_RINGING &&
        newStateInfo.polrev != currentStateInfo.polrev &&
#ifdef VP_CSLAC_SEQ_EN
        !pLineObj->sendSignal.active &&
#endif
        newStateInfo.oppositeEquiv != state)
    {
        pLineObj->ringExitCleanupState = state;
        state = newStateInfo.oppositeEquiv;

        /* Avoid using low power mode as an intermediate state */
        if (state == VP_LINE_STANDBY && pLineObj->termType == VP_TERM_FXS_LOW_PWR) {
            state = VP_LINE_ACTIVE;
        }

        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ring exit into polrev state, switching %d to %d",
            pLineObj->ringExitCleanupState, state));

        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP,
            VP886_RING_EXIT_CLEANUP_CHECK_DELAY, 0, VP886_RING_EXIT_CLEANUP_CHECK);

    } else if (currentStateInfo.normalEquiv == VP_LINE_RINGING &&
        pLineObj->unbalancedRinging &&
        state == VP_LINE_STANDBY &&
        pLineObj->termType == VP_TERM_FXS_LOW_PWR)
    {
        /* If exiting from unbalanced ringing to low power mode, the device can
           damage itself.  Go through ACTIVE first, using the same ring exit
           checking mechanism as the polrev ring exit workaround.
           NOTE: This should be a revision AAA issue only, but I'm leaving it
           enabled for everything to be safe for now. */
        pLineObj->ringExitCleanupState = state;
        state = VP_LINE_ACTIVE;
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Unbalanced ring exit into low power state, going ACTIVE first"));

        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP,
            VP886_RING_EXIT_CLEANUP_CHECK_DELAY, 0, VP886_RING_EXIT_CLEANUP_CHECK);

    } else {
        /* Otherwise, cancel the above timer in case it's running */
        Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP, 0, FALSE);
    }

    /* Map the line state to the system state register contents */
    status = Vp886MapLineState(pLineCtx, state, &stateReg);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Set up hook/gkey freeze if necessary */
    Vp886SetLineStateFxsDetectFreeze(pLineCtx, state);

    /* Save the line state info */
    pLineObj->lineState.previous = pLineObj->lineState.currentState;
    pLineObj->lineState.currentState = state;

    if (VP886_IS_ABS(pDevObj)) {
        pDevObj->absPowerReq[pLineObj->channelId] = Vp886GetABSPowerReq(pLineCtx, state);
        Vp886ManageABSPower(pDevCtx);
    }

    /* Process various workarounds */
    Vp886SetLineStateFxsWorkarounds(pLineCtx, pLineObj->lineState.previous, state);

    if (VP886_IS_ABS(pDevObj) && pLineObj->ringExitInProgress == FALSE) {
        /* If not coming FROM a ringing state, update the ringing battery flag
           and ringing battery levels.  When exiting ringing, the RING_EXIT
           timer will do this instead, to avoid dropping the battery level
           too early. */
        Vp886SetABSRingingBattFlag(pLineCtx, FALSE);
        Vp886ManageABSRingingBatt(pDevCtx, TRUE, TRUE);
    }
    
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Setting system state 0x%02X", stateReg));

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, &stateReg);
    pLineObj->registers.sysState[0] = stateReg;

    Vp886ApplyPcmTxRx(pLineCtx);

#ifdef VP886_INCLUDE_DTMF_DETECT
    Vp886DtmfManage(pLineCtx);
#endif
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING

#ifdef VP886_INCLUDE_TESTLINE_CODE
    lineTestInProgress = pLineObj->testInfo.prepared;
#endif
    /* If going into ringing ... */
    if ((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)) {
        /* If we are not running a line test */
        if (!(lineTestInProgress)) {
            /* if the thermal ringing algorithms are enabled */
            if (pDevObj->options.adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED) {
                /* And the line is not currently running the thermal ringing routines */
                if (!(pLineObj->thermalRinging)) {
                    if (pDevObj->staticInfo.maxChannels == 1) {
                        /* Single channel device: the device power is the same as the line power */
                        Vp886AdaptiveRingingSetTargetSlicPower(pLineCtx,
                            pDevObj->options.adaptiveRinging.power);
                    } else {
                        /* Set the target power for this line. Currently just use
                           one-half the target power for the whole SLIC until a more
                           "dynamic" power assignment scheme can be determined */
                        Vp886AdaptiveRingingSetTargetSlicPower(pLineCtx,
                            pDevObj->options.adaptiveRinging.power >> 1);
                    }
                    /* Start the thermal ringing algorithms */
                    Vp886AdaptiveRingingPrepare(pLineCtx);
                }
            }
        }
    }

    /* If exiting ringing ... */
    if ((state != VP_LINE_RINGING) && (state != VP_LINE_RINGING_POLREV)) {
        /* If we are not running a line test */
        if (!(lineTestInProgress)) {
            /* if the thermal ringing algorithms are enabled */
            if (pDevObj->options.adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED) {
                /* And the line is currently running the thermal ringing routines */
                if (pLineObj->thermalRinging) {
                    /* stop the thermal ringing routines */
                    Vp886AdaptiveRingingStop(pLineCtx);
                }
            }
        }
    }

#endif

    switch(pLineObj->ringSyncState) {
        case VP886_RINGSYNC_STATE_DELAYING:
        case VP886_RINGSYNC_STATE_PAUSING:
            /* Line state was changed in the middle of trying to ring sync.
               Cancel the rest of the sequence and clean up. */
            Vp886RingSyncCancel(pLineCtx);
            break;
        case VP886_RINGSYNC_STATE_FINISHING_PAUSE:
        case VP886_RINGSYNC_STATE_FINISHING_DELAY:
            /* This line state change is the final action in a ring sync
               sequence.  Finish by setting the state flag back to idle. */
            pLineObj->ringSyncState = VP886_RINGSYNC_STATE_IDLE;
            break;
        default:
            break;
    }

    /* Release control of VREF if going to shutdown */
    if (state == VP_LINE_DISABLED) {
        pLineObj->registers.icr3[0] &= ~VP886_R_ICR3_VREF_EN;
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->registers.icr3);
    }

    return VP_STATUS_SUCCESS;
}


/** Vp886SetLineStateFxsDetectFreeze()
  Based on the current and new line states, this function sets up, if necessary,
  the appropriate hook and gkey freeze detect masks and timers.
*/
void
Vp886SetLineStateFxsDetectFreeze(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886LineStateInfoType currentStateInfo;
    Vp886LineStateInfoType newStateInfo;
    uint32 hookFreezeMs = 0;
    uint32 gkeyFreezeMs = 0;

    currentStateInfo = Vp886LineStateInfo(pLineObj->lineState.currentState);
    newStateInfo = Vp886LineStateInfo(state);

    /* Freeze the hook and gkey states while in disconnect.  The disconnect exit
       case below will handle unfreezing them when the state changes. */
    if (state == VP_LINE_DISCONNECT) {
        Vp886SetDetectMask(pLineCtx, VP_CSLAC_HOOK);
        Vp886SetDetectMask(pLineCtx, VP_CSLAC_GKEY);
        return;
    }

    /* Determine if hook freeze is necessary */
    /* Ring exit */
    if (currentStateInfo.normalEquiv == VP_LINE_RINGING &&
        newStateInfo.normalEquiv != VP_LINE_RINGING)
    {
        /* Use a defined debounce duration for a ring trip, or the RING_CNTRL
           option setting for a non-trip ring exit */
        if (pLineObj->lineState.condition & VP_CSLAC_HOOK) {
            hookFreezeMs = VP886_HOOKFREEZE_RING_TRIP;
        } else {
            /* SetOption should have limited this to 255ms already */
            hookFreezeMs = pLineObj->options.ringControl.ringExitDbncDur / 8;
        }
    }
    /* Disconnect exit */
    if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT && state != VP_LINE_DISCONNECT) {
        hookFreezeMs = MAX(hookFreezeMs, VP886_HOOKFREEZE_DISC_EXIT);
        gkeyFreezeMs = MAX(hookFreezeMs, VP886_GKEYFREEZE_DISC_EXIT);
    }
    /* Check for Polrev */
    if (newStateInfo.polrev != currentStateInfo.polrev) {
        hookFreezeMs = MAX(hookFreezeMs, VP886_HOOKFREEZE_POLREV);
    }
    /* Entering low power mode while offhook can cause a short hook glitch */
    if (state == VP_LINE_STANDBY && pLineObj->termType == VP_TERM_FXS_LOW_PWR) {
        hookFreezeMs = MAX(hookFreezeMs, VP886_HOOKFREEZE_LOWPOWER);
    }

    if (hookFreezeMs > 0) {
        VP_HOOK(VpLineCtxType, pLineCtx, ("Hook freeze for %lums", hookFreezeMs));
        Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_HOOK_FREEZE, hookFreezeMs, 0);
        Vp886SetDetectMask(pLineCtx, VP_CSLAC_HOOK);
    }
    if (gkeyFreezeMs > 0) {
        Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_GKEY_FREEZE, gkeyFreezeMs, 0);
        Vp886SetDetectMask(pLineCtx, VP_CSLAC_GKEY);
    }
}


/** Vp886SetLineStateFxsWorkarounds()
  This function performs workarounds that are required for particular line
  state transitions.
*/
void
Vp886SetLineStateFxsWorkarounds(
    VpLineCtxType *pLineCtx,
    VpLineStateType prevState,
    VpLineStateType newState)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886LineStateInfoType prevStateInfo;
    Vp886LineStateInfoType newStateInfo;

    prevStateInfo = Vp886LineStateInfo(prevState);
    newStateInfo = Vp886LineStateInfo(newState);

    /* Workaround for polarity reversals.  Disable speedups to prevent
       longitudinal shift.  Skip this if either state is ringing.
       Also skip if exiting disconnect or exiting low power mode, as these
       settings can cause dangerous oscillations (on revision AAA). */
    if (newStateInfo.polrev != prevStateInfo.polrev &&
        prevStateInfo.codec && newStateInfo.codec)
    {
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Polrev fix, applying"));
        pLineObj->registers.icr2[2] |= VP886_R_ICR2_SPEEDUP_BAT;
        pLineObj->registers.icr2[3] &= ~VP886_R_ICR2_SPEEDUP_BAT;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
            pLineObj->registers.icr2);
        pLineObj->registers.icr3[0] |= VP886_R_ICR3_SPEEDUP_LONG;
        pLineObj->registers.icr3[1] &= ~VP886_R_ICR3_SPEEDUP_LONG;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
            pLineObj->registers.icr3);
        Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_POLREV_FIX, VP886_POLREV_FIX_TIME, 0);
    }


    /* Workarounds for entering and exiting ringing
       1. If the device is HV Tracker, change the switcher limit to 150 upon
       entering ringing, and back to 100 when exiting ringing.
       2. Switching regulator floor voltage needs to be raised to 45V upon
       entering ringing, and restored when exiting. Without this fix there can
       be significant distortion on the ringing waveform, and crosstalk to the
       other channel.
    */
    /* TODO: The switcher settings aren't really a workaround.  Maybe this
       function needs a new name */
    if (prevStateInfo.normalEquiv != VP_LINE_RINGING &&
        newStateInfo.normalEquiv == VP_LINE_RINGING)
    {
        /* Entering ringing */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ring entry, applying switcher settings"));

        /* Make sure the timer to remove these settings isn't running */
        Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_RING_EXIT, 0, FALSE);
        pLineObj->ringExitInProgress = FALSE;

        if (VP886_IS_TRACKER(pDevObj) && VP886_IS_HV(pDevObj)) {
            pLineObj->registers.icr2[2] |= VP886_R_ICR2_SW_LIM;
            pLineObj->registers.icr2[3] &= ~VP886_R_ICR2_SW_LIM;
            pLineObj->registers.icr2[3] |= VP886_R_ICR2_SW_LIM_150;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pLineObj->registers.icr2);
        }
        if (VP886_IS_TRACKER(pDevObj) &&
            (pLineObj->registers.swParam[0] & VP886_R_SWPARAM_RING_TRACKING)
                == VP886_R_SWPARAM_RING_TRACKING_EN)
        {
            pLineObj->registers.swParam[0] &= ~VP886_R_SWPARAM_FLOOR_V;
            pLineObj->registers.swParam[0] |= ((45 - 5) / 5); /* 45V */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                pLineObj->registers.swParam);
        }

    } else if (prevStateInfo.normalEquiv == VP_LINE_RINGING &&
               newStateInfo.normalEquiv != VP_LINE_RINGING) {
        /* Exiting ringing.  Some of the workarounds require us to wait for
           ringing to exit cleanly before removing them. */
        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_RING_EXIT,
            VP886_RING_EXIT_DELAY, 0, 0);
        pLineObj->ringExitInProgress = TRUE;
    }


    /* Workarounds involving tip open and ring open */
    if ((newState == VP_LINE_TIP_OPEN || newState == VP_LINE_RING_OPEN) &&
        (prevState != VP_LINE_TIP_OPEN && prevState != VP_LINE_RING_OPEN)) {
        /* Entering tip open or ring open.  Force ICR3 c_long_on to 0 to prevent
           a continuous hook indication */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Entering Ring or Tip Open, forcing c_long_on 0"));
        pLineObj->registers.icr3[2] |= VP886_R_ICR3_LONG_ON;
        pLineObj->registers.icr3[3] &= ~VP886_R_ICR3_LONG_ON;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
            pLineObj->registers.icr3);
    } else if (prevState == VP_LINE_TIP_OPEN && newState == VP_LINE_ACTIVE &&
               (pLineObj->lineState.condition & VP_CSLAC_GKEY)) {
        /* When exiting from tip open to active with a GKEY present, apply a
           workaround to comply with requirements for Ground Start signaling.
           The workaround is to simply allow c_long_on to remain forced to 0 for
           a short time, which will force tip close to ground. */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Exiting tip open to active state, beginning ground start workaround"));
        /* Set a timer to remove c_long_on later */
        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_GROUNDSTART, VP886_GROUNDSTART_DELAY, 0, 0);

    } else if ((newState != VP_LINE_TIP_OPEN && newState != VP_LINE_RING_OPEN) &&
               (prevState == VP_LINE_TIP_OPEN || prevState == VP_LINE_RING_OPEN)) {
        /* Exiting tip open or ring open.  Give up control of ICR3 c_long_on */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Exiting Ring or Tip Open, releasing c_long_on control"));
        pLineObj->registers.icr3[2] &= ~VP886_R_ICR3_LONG_ON;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
            pLineObj->registers.icr3);

    }

    /* This is separate from the above section because I want it to apply to ALL
       exit cases from lead-open states, including tip open to active. */
    if ((newState != VP_LINE_TIP_OPEN && newState != VP_LINE_RING_OPEN) &&
        (prevState == VP_LINE_TIP_OPEN || prevState == VP_LINE_RING_OPEN)) {
        /* While in a lead-open state, groundkey activity is detected as hook
           signals and recorded by the hook buffer.  Since we interpret the bit
           in the signaling register as a GKEY event, we don't read the hook
           buffer while processing events in these states, so it may be filled
           with garbage.  Clear it when exiting a lead-open state. */
        uint8 hookBuf[VP886_R_HOOKBUF_LEN];
        pDevObj->dontFlushSlacBufOnRead = TRUE;
        VpSlacRegRead(NULL, pLineCtx, VP886_R_HOOKBUF_RD, VP886_R_HOOKBUF_LEN, hookBuf);
    }


    /* Mask groundkey detection in unbalanced ringing.  Unbalanced ringing
       inherently generates false gkey detections.  This mask is removed when
       the ring-exit timer is handled. */
    if (newStateInfo.normalEquiv == VP_LINE_RINGING && pLineObj->unbalancedRinging) {
        Vp886SetDetectMask(pLineCtx, VP_CSLAC_GKEY);
    }

    return;
}


#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
/** Vp886SetLineStateFxsHowler()
  This function controls changing to and from HOWLER line states.

  Only allow transitions to/from howler states from same-polarity ACTIVE
  or TALK states.  This also excludes changing between HOWLER and
  HOWLER_POLREV.  Restricting these transitions simplifies things by
  preventing interference between the high gain mode settings and
  other line state settings and workarounds.
*/
VpStatusType
Vp886SetLineStateFxsHowler(
    VpLineCtxType *pLineCtx,
    VpLineStateType toState,
    VpLineStateType fromState)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886LineStateInfoType toInfo;
    Vp886LineStateInfoType fromInfo;

    toInfo = Vp886LineStateInfo(toState);
    fromInfo = Vp886LineStateInfo(fromState);

    /* Entering high gain mode */
    if (toInfo.normalEquiv == VP_LINE_HOWLER) {
        if (fromInfo.polrev != toInfo.polrev) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Cannot enter howler state %d from state %d (polarity must match)", toState, fromState));
            return VP_STATUS_INVALID_ARG;
        }
        switch (fromState) {
            case VP_LINE_ACTIVE:
            case VP_LINE_TALK:
            case VP_LINE_ACTIVE_POLREV:
            case VP_LINE_TALK_POLREV:
                Vp886HighGainEnter(pLineCtx);
                pLineObj->lineState.currentState = toState;
                pLineObj->lineState.usrCurrent = toState;
                Vp886ApplyPcmTxRx(pLineCtx);
                return VP_STATUS_SUCCESS;
            default:
                VP_ERROR(VpLineCtxType, pLineCtx, ("Cannot enter howler state %d from state %d", toState, fromState));
                return VP_STATUS_INVALID_ARG;
        }
    }

    /* Exiting high gain mode */
    if (fromInfo.normalEquiv == VP_LINE_HOWLER) {
        if (fromInfo.polrev != toInfo.polrev) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Cannot exit howler state %d to state %d (polarity must match)", fromState, toState));
            return VP_STATUS_INVALID_ARG;
        }
        switch (toState) {
            case VP_LINE_ACTIVE:
            case VP_LINE_TALK:
            case VP_LINE_ACTIVE_POLREV:
            case VP_LINE_TALK_POLREV:
                Vp886HighGainExit(pLineCtx);
                pLineObj->lineState.currentState = toState;
                pLineObj->lineState.usrCurrent = toState;
                Vp886ApplyPcmTxRx(pLineCtx);
                return VP_STATUS_SUCCESS;
            default:
                VP_ERROR(VpLineCtxType, pLineCtx, ("Cannot exit howler state %d to state %d", fromState, toState));
                return VP_STATUS_INVALID_ARG;
        }
    }
    return VP_STATUS_SUCCESS;
}


/** Vp886HighGainEnter()
  Applies high gain settings to implement HOWLER line states.
*/
void
Vp886HighGainEnter(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 icr1[VP886_R_ICR1_LEN];
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 icr3[VP886_R_ICR3_LEN];
    uint8 icr4[VP886_R_ICR4_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 vpGain[VP886_R_VPGAIN_LEN];
    uint8 disn[VP886_R_DISN_LEN];
    uint8 opFunc[VP886_R_OPFUNC_LEN];
    uint8 swParam[VP886_R_SWPARAM_LEN];

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Entering high gain mode"));

    /* Read current values into the high gain mode cache so that they can be
       restored in Vp886HighGainExit().  Some values come from the main
       register cache, some from device reads */
    VpMemCpy(pLineObj->highGainCache.icr1, pLineObj->registers.icr1, VP886_R_ICR1_LEN);
    VpMemCpy(pLineObj->highGainCache.icr2, pLineObj->registers.icr2, VP886_R_ICR2_LEN);
    VpMemCpy(pLineObj->highGainCache.icr3, pLineObj->registers.icr3, VP886_R_ICR3_LEN);
    VpMemCpy(pLineObj->highGainCache.icr4, pLineObj->registers.icr4, VP886_R_ICR4_LEN);
    VpMemCpy(pLineObj->highGainCache.opFunc, pLineObj->registers.opFunc, VP886_R_OPFUNC_LEN);
    if (VP886_IS_TRACKER(pDevObj)) {
        VpMemCpy(pLineObj->highGainCache.swParam, pLineObj->registers.swParam, VP886_R_SWPARAM_LEN);
    }
    VpSlacRegRead(NULL, pLineCtx, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN, pLineObj->highGainCache.dcFeed);
    VpSlacRegRead(NULL, pLineCtx, VP886_R_VPGAIN_RD, VP886_R_VPGAIN_LEN, pLineObj->highGainCache.vpGain);
    VpSlacRegRead(NULL, pLineCtx, VP886_R_GR_RD, VP886_R_GR_LEN, pLineObj->highGainCache.grValue);
    VpSlacRegRead(NULL, pLineCtx, VP886_R_R_FILT_RD, VP886_R_R_FILT_LEN, pLineObj->highGainCache.rValue);
    VpSlacRegRead(NULL, pLineCtx, VP886_R_DISN_RD, VP886_R_DISN_LEN, pLineObj->highGainCache.disn);

    /* If Entering High Gain Mode, program the filter coefficients first to
       pre-reduce D-A gain and avoid having the line in high gain conditions
       with normal coefficients */
    Vp886HighGainSetRFilter(pLineCtx);

    /* Force tracker floor voltage to -30V */
    if (VP886_IS_TRACKER(pDevObj)) {
        VpMemCpy(swParam, pLineObj->highGainCache.swParam, VP886_R_SWPARAM_LEN);
        swParam[0] &= ~VP886_R_SWPARAM_FLOOR_V;
        swParam[0] |= 0x05; /* -30V */
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, swParam);
    }

    /* VP Gain */
    vpGain[0] = (VP886_R_VPGAIN_AR_LOSS | VP886_R_VPGAIN_DRL);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_VPGAIN_WRT, VP886_R_VPGAIN_LEN, vpGain);

    /* DISN */
    disn[0] = 0x00;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_DISN_WRT, VP886_R_DISN_LEN, disn);

    /* Operating Functions: Disable Z and B filters. If this isn't done, the AC
                            loop could oscillate and cause significant tip/ring
                            noise to the point of hook detection instability */
    VpMemCpy(opFunc, pLineObj->highGainCache.opFunc, VP886_R_OPFUNC_LEN);
    opFunc[0] &= ~VP886_R_OPFUNC_EZ;
    opFunc[0] &= ~VP886_R_OPFUNC_EB;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPFUNC_WRT, VP886_R_OPFUNC_LEN, opFunc);

    /* DC Feed: 18VOC, 40mA ILA */
    VpMemCpy(dcFeed, pLineObj->highGainCache.dcFeed, VP886_R_DCFEED_LEN);
    dcFeed[0] &= ~VP886_R_DCFEED_VOC;
    dcFeed[0] |= 0x08;
    dcFeed[0] |= VP886_R_DCFEED_VOCSHIFT;
    dcFeed[1] &= ~VP886_R_DCFEED_ILA;
    dcFeed[1] |= 0x16;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

    /* ICR1: Set tip/ring bias overrides. For ABS, force VBL. */
    VpMemCpy(icr1, pLineObj->highGainCache.icr1, VP886_R_ICR1_LEN);
    icr1[0] |= VP886_R_ICR1_TIP_BIAS;
    icr1[1] &= ~VP886_R_ICR1_TIP_BIAS;
    icr1[1] |= 0xC0;
    icr1[2] |= VP886_R_ICR1_RING_BIAS;
    icr1[3] &= ~VP886_R_ICR1_RING_BIAS;
    icr1[3] |= 0x0C;
    /* Force VBL for ABS */
    if (VP886_IS_ABS(pDevObj)) {
        icr1[2] |= VP886_R_ICR1_BAT;
        icr1[3] &= ~VP886_R_ICR1_BAT;
        icr1[3] |= VP886_R_ICR1_BAT_VBL;
    }
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, icr1);

    /* ICR4: Turn off AISN and small metallic signal output */
    VpMemCpy(icr4, pLineObj->highGainCache.icr4, VP886_R_ICR4_LEN);
    icr4[0] |= (VP886_R_ICR4_SMALL_SIGNAL_CTRL | VP886_R_ICR4_AISN_CTRL);
    icr4[1] &= ~VP886_R_ICR4_SMALL_SIGNAL_CTRL;
    icr4[1] &= ~VP886_R_ICR4_AISN_CTRL;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, icr4);

    /* ICR2: Enable ringing current limit range.  Enable metallic speedup and
             set for Ringing Time Constant */
    VpMemCpy(icr2, pLineObj->highGainCache.icr2, VP886_R_ICR2_LEN);
    icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
    icr2[1] |= VP886_R_ICR2_DAC_RING_LEVELS;
    icr2[2] |= (VP886_R_ICR2_SPEEDUP_MET | VP886_R_ICR2_RINGING_TC);
    icr2[3] |= (VP886_R_ICR2_SPEEDUP_MET | VP886_R_ICR2_RINGING_TC);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

    /* ICR3: Enable signal generator via DC Feed */
    VpMemCpy(icr3, pLineObj->highGainCache.icr3, VP886_R_ICR3_LEN);
    icr3[2] |= VP886_R_ICR3_RINGING_ON;
    icr3[3] |= VP886_R_ICR3_RINGING_ON;

    /* ICR3: Force the Longitudinal Battery Sense to Low Battery Independent of
             Battery Switch in ICR1 to prevent substate transitions (#8319) ABS only.*/
    if (VP886_IS_ABS(pDevObj)) {
        icr3[2] |= 0x28;
        icr3[3] &= ~0x20;
        icr3[3] |= 0x08;
    }
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, icr3);
    pLineObj->inHighGainMode = TRUE;
}


/** Vp886HighGainExit()
  Exits HOWLER line states by restoring settings that were saved upon entering
  high gain mode.
*/
void
Vp886HighGainExit(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Exiting high gain mode"));

    /* Restore all the values from the high gain buffer.  Restore filters last
       to avoid having the line in the high gain conditions with normal (higher
       gain) coefficients. We also don't want to create the oscillation
       scenario where the line is in a non-normal AC impedance mode with normal
       impedance setting coefficients enabled */

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, pLineObj->highGainCache.dcFeed);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, pLineObj->highGainCache.icr1);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->highGainCache.icr4);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->highGainCache.icr2);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->highGainCache.icr3);
    if (VP886_IS_TRACKER(pDevObj)) {
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->highGainCache.swParam);
    }
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_VPGAIN_WRT, VP886_R_VPGAIN_LEN, pLineObj->highGainCache.vpGain);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_DISN_WRT, VP886_R_DISN_LEN, pLineObj->highGainCache.disn);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPFUNC_WRT, VP886_R_OPFUNC_LEN, pLineObj->highGainCache.opFunc);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_GR_WRT, VP886_R_GR_LEN, pLineObj->highGainCache.grValue);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_R_FILT_WRT, VP886_R_R_FILT_LEN, pLineObj->highGainCache.rValue);

    pLineObj->inHighGainMode = FALSE;
}


/** Vp886HighGainSetRFilter()
  Used in high gain mode to set the R and GR coefficients based on the current
  tone type.  This should be called when entering high gain mode and upon
  starting a tone while in high gain mode.
*/
void
Vp886HighGainSetRFilter(
    VpLineCtxType *pLineCtx)
{
    uint8 grReg[VP886_R_GR_LEN];
    uint8 rFiltReg[VP886_R_R_FILT_LEN];

    const uint8 rValueFlat[VP886_R_R_FILT_LEN] = {
        0x7D, 0xD0, 0x01, 0x11, 0x01, 0x90, 0x01, 0x90, 0x01, 0x90,
        0x01, 0x90, 0x01, 0x90};

#ifdef VP_CSLAC_SEQ_EN
    /* UK Draft 960-G has a non-flat Holwer Tone Frequency response. All other types of Howler Tone,
       including UK Version 15, is a flat frequency response.
       In case of non-flat response, the loss at 800Hz is 3dB more than the loss at 2500Hz */
    const uint8 rValueUk960G[VP886_R_R_FILT_LEN] = {
        0x2D, 0xC0, 0x22, 0xC0, 0x43, 0x61, 0xBD, 0xA8, 0x3A, 0xA1,
        0x43, 0x2A, 0x22, 0x24};

    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 toneType;

    if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
        toneType = pLineObj->cadence.toneType;
    } else {
        toneType = 0;
    }

    switch (toneType) {
        case VP_CSLAC_UK_HOWLER_TONE_VER15: { /* UK Howler Ver15 */
            VpMemCpy(rFiltReg, rValueFlat, VP886_R_R_FILT_LEN);
            grReg[0] = 0x9F;
            grReg[1] = 0xA1;
            break;
        }
        case VP_CSLAC_UK_HOWLER_TONE_DRAFT_G: { /* UK Howler Draft G */
            VpMemCpy(rFiltReg, rValueUk960G, VP886_R_R_FILT_LEN);
            grReg[0] = 0x26;
            grReg[1] = 0x32;
            break;
        }
        case VP_CSLAC_AUS_HOWLER_TONE: {
            VpMemCpy(rFiltReg, rValueFlat, VP886_R_R_FILT_LEN);
            grReg[0] = 0xAB;
            grReg[1] = 0xB2;
            break;
        }
        default: {
            VpMemCpy(rFiltReg, rValueFlat, VP886_R_R_FILT_LEN);
            grReg[0] = 0x9F;
            grReg[1] = 0xA1;
            break;
        }
    }
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Set high gain mode GR and R for toneType 0x%X", toneType));
#else
    VpMemCpy(rFiltReg, rValueFlat, VP886_R_R_FILT_LEN);
#endif /* VP_CSLAC_SEQ_EN */

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_GR_WRT, VP886_R_GR_LEN, grReg);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_R_FILT_WRT, VP886_R_R_FILT_LEN, rFiltReg);


    return;
}
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */


/** Vp886MapLineState()
  Maps a VP-API-II line state to a system state register value.
*/
VpStatusType
Vp886MapLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state,
    uint8 *stateReg)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886LineStateInfoType newStateInfo;

    newStateInfo = Vp886LineStateInfo(state);

    /* Determine the system state "activate codec" bit based on line state */
    *stateReg &= ~VP886_R_STATE_CODEC;
    if (newStateInfo.codec) {
        *stateReg |= VP886_R_STATE_CODEC;
    }

    /* Determine the polrev bit based on line state */
    if (newStateInfo.polrev) {
        *stateReg |= VP886_R_STATE_POL;
    }

    /* Map the API line state to the system state field */
    *stateReg &= ~VP886_R_STATE_SS;
    switch (state) {
        case VP_LINE_STANDBY:
            if (pLineObj->termType == VP_TERM_FXS_LOW_PWR) {
                *stateReg |= VP886_R_STATE_SS_LOWPOWER;
            } else {
                *stateReg |= VP886_R_STATE_SS_IDLE;
            }
            break;
        case VP_LINE_STANDBY_POLREV:
            /* The device only supports the low power state in normal polarity,
               so always use the IDLE state for STANDBY_POLREV. */
            *stateReg |= VP886_R_STATE_SS_IDLE;
            break;
        case VP_LINE_TIP_OPEN:
            *stateReg |= VP886_R_STATE_SS_TIPOPEN;
            break;
        case VP_LINE_RING_OPEN:
            *stateReg |= VP886_R_STATE_SS_RINGOPEN;
            break;
        case VP_LINE_DISCONNECT:
            *stateReg |= VP886_R_STATE_SS_DISCONNECT;
            break;
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
            if (pLineObj->unbalancedRinging) {
                *stateReg |= VP886_R_STATE_SS_UNBAL_RING;
            } else {
                *stateReg |= VP886_R_STATE_SS_BAL_RING;
            }
            break;
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
            *stateReg |= VP886_R_STATE_SS_ACTIVE;
            break;
        case VP_LINE_DISABLED:
            *stateReg |= VP886_R_STATE_SS_SHUTDOWN;
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    return VP_STATUS_SUCCESS;
}


/** Vp886ApplyPcmTxRx()
  Enables or disables the PCM Transmit and Receive paths based on line state,
  the PCM_TXRX_CNTRL option, and special cases.
*/
void
Vp886ApplyPcmTxRx(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 opCondReg = pLineObj->registers.opCond[0];

    /* Determine whether to cut or enable TX/RX based on line state */
    opCondReg &= ~(VP886_R_OPCOND_CUT_TX | VP886_R_OPCOND_CUT_RX);
    if (!Vp886LineStateInfo(pLineObj->lineState.currentState).voice) {
        /* Cut both in non-voice states */
        opCondReg |= (VP886_R_OPCOND_CUT_TX | VP886_R_OPCOND_CUT_RX);
    }

    /* Adjust for the PCM_TXRX_CNTRL option */
    switch (pLineObj->options.pcmTxRxCntrl) {
        case VP_OPTION_PCM_BOTH:
            /* no change */
            break;
        case VP_OPTION_PCM_RX_ONLY:
            /* cut TX */
            opCondReg |= VP886_R_OPCOND_CUT_TX;
            break;
        case VP_OPTION_PCM_TX_ONLY:
            /* cut RX */
            opCondReg |= VP886_R_OPCOND_CUT_RX;
            break;
        case VP_OPTION_PCM_OFF:
            /* cut both */
            opCondReg |= VP886_R_OPCOND_CUT_TX;
            opCondReg |= VP886_R_OPCOND_CUT_RX;
            break;
        case VP_OPTION_PCM_ALWAYS_ON:
            /* enable both */
            opCondReg &= ~(VP886_R_OPCOND_CUT_TX | VP886_R_OPCOND_CUT_RX);
            break;
        default:
            break;
    }

#ifdef VP_CSLAC_SEQ_EN
    /* Cut both during caller ID if Mute On was requested or during FSK */
    if (pLineObj->cid.active && (pLineObj->cid.mute || pLineObj->cid.fskEnabled)) {
        opCondReg |= VP886_R_OPCOND_CUT_RX | VP886_R_OPCOND_CUT_TX;
    }

    /* Transmit must be enabled during caller ID DTMF detection intervals */
    if (pLineObj->cid.active && pLineObj->cid.state == VP886_CID_ST_DETECT_INTERVAL) {
        opCondReg &= ~VP886_R_OPCOND_CUT_TX;
    }
#endif

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, &opCondReg);
    pLineObj->registers.opCond[0] = opCondReg;
}


/** Vp886GetABSPowerReq()
  Returns an ABS power requirement value based on lineState.
*/
Vp886AbsPowerReqType
Vp886GetABSPowerReq(
    VpLineCtxType *pLineCtx,
    VpLineStateType lineState)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886AbsPowerReqType powerReq;

    switch (lineState) {
        /* Off */
        case VP_LINE_DISABLED:
            powerReq = VP886_ABS_POWER_REQ_OFF;
            break;
        /* Low power */
        case VP_LINE_DISCONNECT:
            powerReq = VP886_ABS_POWER_REQ_LOW;
            break;
        case VP_LINE_STANDBY:
            if (pLineObj->termType == VP_TERM_FXS_LOW_PWR) {
                powerReq = VP886_ABS_POWER_REQ_LOW;
                break;
            } /* else fall through to medium */
        /* Medium Power */
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_RING_OPEN:
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
        case VP_LINE_HOWLER:
        case VP_LINE_HOWLER_POLREV:
            powerReq = VP886_ABS_POWER_REQ_MED;
            break;
        /* High Power */
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
            powerReq = VP886_ABS_POWER_REQ_HIGH;
            break;
        default:
            VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886GetABSPowerReq() Unhandled line state %d", lineState));
            powerReq = VP886_ABS_POWER_REQ_HIGH;
            break;
    }

#ifdef VP886_INCLUDE_TESTLINE_CODE
    /* Enforce a minimum of medium power during line test */
    if (pLineObj->inLineTest && powerReq != VP886_ABS_POWER_REQ_HIGH) {
        powerReq = VP886_ABS_POWER_REQ_MED;
    }
#endif

    return powerReq;
}


/** Vp886ManageABSPower()
  Sets ABS switcher power levels based on line states and device configuration
  settings.
*/
void
Vp886ManageABSPower(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId;
    bool fromLowPower;
    bool fromHighPower;
    bool toLowPower;
    bool toHighPower;
    uint8 swCtrl[VP886_R_SWCTRL_LEN];

    /* If either line is in shutdown (VP_LINE_DISABLED) turn off the switchers
       and return. */
    for (channelId = 0; channelId < 2; channelId++) {
        if (pDevObj->absPowerReq[channelId] == VP886_ABS_POWER_REQ_OFF) {
            pDevObj->registers.swCtrl[0] &= ~VP886_R_SWCTRL_MODE_Y;
            pDevObj->registers.swCtrl[0] |= VP886_R_SWCTRL_MODE_Y_OFF;
            pDevObj->registers.swCtrl[0] &= ~VP886_R_SWCTRL_MODE_Z;
            pDevObj->registers.swCtrl[0] |= VP886_R_SWCTRL_MODE_Z_OFF;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWCTRL_WRT, VP886_R_SWCTRL_LEN, pDevObj->registers.swCtrl);
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Setting switcher ctrl to 0x%02X", pDevObj->registers.swCtrl[0]));
            return;
        }
    }

    /* Only change the power mode for SINGLE configurations.  MASTER should
       always remain at high power, and SLAVE can be ignored */
    switch (pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_MODE) {
        case VP886_DEV_PROFILE_ABS_SUPP_CFG_SLAVE:
        case VP886_DEV_PROFILE_ABS_SUPP_CFG_MASTER:
            return;
        default:
            break;
    }

    Vp886CancelTimer(pDevCtx, NULL, VP886_TIMERID_ABS_POWER, 0, FALSE);

    /* Initialize lowPower = TRUE so that it can be changed to FALSE if either
       line is NOT low power, but will remain TRUE if both lines are low power */
    toLowPower = TRUE;

    /* Initialize highPower = FALSE so that it can be changed to TRUE if either
       line requires high power, but will remain FALSE if both lines do not
       require high power */
    toHighPower = FALSE;

    for (channelId = 0; channelId < 2; channelId++) {
        switch (pDevObj->absPowerReq[channelId]) {
            case VP886_ABS_POWER_REQ_LOW:
                break;
            case VP886_ABS_POWER_REQ_MED:
                toLowPower = FALSE;
                break;
            case VP886_ABS_POWER_REQ_HIGH:
                toLowPower = FALSE;
                toHighPower = TRUE;
                break;
            default:
                break;
        }
    }

    VpMemCpy(swCtrl, pDevObj->registers.swCtrl, VP886_R_SWCTRL_LEN);

    /* Determine the previous power mode. */
    if ((swCtrl[0] & VP886_R_SWCTRL_MODE_Y) == VP886_R_SWCTRL_MODE_Y_HP ||
        (swCtrl[0] & VP886_R_SWCTRL_MODE_Z) == VP886_R_SWCTRL_MODE_Z_HP)
    {
        /* High */
        fromLowPower = FALSE;
        fromHighPower = TRUE;
    } else if ((swCtrl[0] & VP886_R_SWCTRL_MODE_Y) == VP886_R_SWCTRL_MODE_Y_MP ||
               (swCtrl[0] & VP886_R_SWCTRL_MODE_Z) == VP886_R_SWCTRL_MODE_Z_MP)
    {
        /* Medium */
        fromLowPower = FALSE;
        fromHighPower = FALSE;
    } else {
        /* Low */
        fromLowPower = TRUE;
        fromHighPower = FALSE;
    }

    /* Only modify the mode for switchers that are being controlled, according
       to the device profile. */
    if (!(pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Y_SLAVE)) {
        swCtrl[0] &= ~VP886_R_SWCTRL_MODE_Y;
        if (toHighPower) {
            swCtrl[0] |= VP886_R_SWCTRL_MODE_Y_HP;
        } else if (toLowPower) {
            swCtrl[0] |= VP886_R_SWCTRL_MODE_Y_LP;
        } else {
            swCtrl[0] |= VP886_R_SWCTRL_MODE_Y_MP;
        }
    }
    if (!(pDevObj->devProfileData.absSuppCfg & VP886_DEV_PROFILE_ABS_SUPP_CFG_Z_SLAVE)) {
        swCtrl[0] &= ~VP886_R_SWCTRL_MODE_Z;
        if (toHighPower) {
            swCtrl[0] |= VP886_R_SWCTRL_MODE_Z_HP;
        } else if (toLowPower) {
            swCtrl[0] |= VP886_R_SWCTRL_MODE_Z_LP;
        } else {
            swCtrl[0] |= VP886_R_SWCTRL_MODE_Z_MP;
        }
    }

    /* Only do anything if the mode is changing */
    if (swCtrl[0] != pDevObj->registers.swCtrl[0]) {
        if (fromLowPower || toHighPower) {
            /* If changing to a higher power level (from low or to high), make
               the change immediately */
            pDevObj->registers.swCtrl[0] = swCtrl[0];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWCTRL_WRT, VP886_R_SWCTRL_LEN, swCtrl);
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Setting switcher ctrl to 0x%02X", swCtrl[0]));
        } else if (fromHighPower || toLowPower) {
            /* If changing to a lower power level (from high or to low) use a
               delay so that we don't cut power too early from a state that
               needs it.  For example, the ringing state won't actually end
               until a zero-cross. */
            /* Include the register value in the timer handle so that
               Vp886TimerHandler() doesn't need to redo any of this processing */
            Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_ABS_POWER,
                VP886_ABS_POWER_DECREASE_DELAY, 0, swCtrl[0]);
        } else {
            VP_WARNING(VpDevCtxType, pDevCtx, ("Unhandled ABS power management case.  %02X -> %02X",
                pDevObj->registers.swCtrl[0], swCtrl[0]));
        }
    }
    return;
}


/** Vp886SetABSRingingBattFlag()
  Sets the absRingingBattRequired flag for a channel.  These flags are used in
  Vp886ManageABSRingingBatt() to determine whether ringing battery levels should
  be set.
*/
void
Vp886SetABSRingingBattFlag(
    VpLineCtxType *pLineCtx,
    bool ringExit)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpLineStateType state = pLineObj->lineState.usrCurrent;

#ifdef VP886_INCLUDE_TESTLINE_CODE
    /* Always force ringing battery levels during line test */
    if (pLineObj->inLineTest == TRUE && pLineObj->testInfo.nonIntrusiveTest == FALSE) {
        if (pDevObj->absRingingBattRequired[channelId] == FALSE) {
            pDevObj->absRingingBattRequired[channelId] = TRUE;
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Setting absRingingBattRequired for ch%d due to linetest", channelId));
        }
        return;
    }
#endif

    switch (state) {
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
            /* Set the flag when entering ringing */
            if (pDevObj->absRingingBattRequired[channelId] == FALSE) {
                pDevObj->absRingingBattRequired[channelId] = TRUE;
                VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Setting absRingingBattRequired for ch%d, state %d", channelId, state));
            }
            break;
        case VP_LINE_DISABLED:
        case VP_LINE_DISCONNECT:
        case VP_LINE_STANDBY:
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_RING_OPEN:
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_HOWLER:
        case VP_LINE_HOWLER_POLREV:
            /* Clear the flag when entering one of these offhook or idle states,
               under the assumption that the application is not manually ring
               cadencing anymore. */
            if (pDevObj->absRingingBattRequired[channelId] == TRUE) {
                pDevObj->absRingingBattRequired[channelId] = FALSE;
                VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Clearing absRingingBattRequired for ch%d, state %d", channelId, state));
            }
            break;
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
            /* Leave the flag alone when changing to a state that is normally
               used between ringing bursts, so that the battery isn't being
               changed repeatedly during manual ring cadencing. */
            if (ringExit) {
                VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Leaving absRingingBattRequired as-is for ch%d, state %d", channelId, state));
            } else {
                if (pDevObj->absRingingBattRequired[channelId] == TRUE) {
                    pDevObj->absRingingBattRequired[channelId] = FALSE;
                    VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Clearing absRingingBattRequired for ch%d, state %d", channelId, state));
                }
            }
        default:
            break;
    }
    
    return;
}

/** Vp886ManageABSRingingBatt()
  Sets ABS switcher voltage based on required ringing voltages.  Required
  ringing voltages are set by Vp886ConfigLineFxs() or Vp886SetOptionRingingParams().
*/
void
Vp886ManageABSRingingBatt(
    VpDevCtxType *pDevCtx,
    bool programCalRegistersCh0,
    bool programCalRegistersCh1)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    Vp886LineObjectType *pLineObj;
    uint8 maxRingPeak;
    uint8 newY;
    uint8 newZ;
    uint8 swParamBatVal;
    uint8 channelId;
    uint8 vbhFactor;
    
    if (pDevObj->devProfileData.swCfg == VP886_DEV_PROFILE_SW_CONF_BB) {
        vbhFactor = 2;
    } else {
        /* vbhFactor = 3; */
        /* For now, skip all of this on non-buckboost designs */
        return;
    }

    maxRingPeak = 0;
    
    /* Find the highest required ringing battery voltage, among the lines that
       are currently in ringing state (high power required) */
    for (channelId = 0; channelId < 2; channelId++) {
        if (pDevObj->absRingingBattRequired[channelId] == TRUE) {
            if (pDevObj->absRingingPeak[channelId] > maxRingPeak) {
                maxRingPeak = pDevObj->absRingingPeak[channelId];
            }
        }
    }
    
    if (maxRingPeak == 0) {
        /* No lines in ringing.  Set VBL and VBH to device profile settings */
        newY = pDevObj->devProfileData.swyv;
        newZ = pDevObj->devProfileData.swzv;
    } else {
        /* Determine the required VBL to generate the required VBH, rounding up. */
        newY = (maxRingPeak + pDevObj->devProfileData.vbhOverhead + (vbhFactor - 1)) / vbhFactor;
        /* Determine the Z value to program as the VBH longitudinal point */
        newZ = maxRingPeak + pDevObj->devProfileData.vbhOffset;
        /* Use the device profile values as minimums */
        if (newY <= pDevObj->devProfileData.swyv) {
            newY = pDevObj->devProfileData.swyv;
        }
        if (newZ <= pDevObj->devProfileData.swzv) {
            newZ = pDevObj->devProfileData.swzv;
        }
    }
    
    if (newY == pDevObj->swyv && newZ == pDevObj->swzv) {
        /* No change from the current setting, so do nothing. */
        return;
    }

    /* Update the device object values.  These values will be used when
       calculting calibration settings, and compared against when this function
       is run later. */
    pDevObj->swyv = newY;
    pDevObj->swzv = newZ;

    VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Vp886ManageABSRingingBatt: New swyv %d, swzv %d",
        pDevObj->swyv, pDevObj->swzv));


    pLineCtx = pDevCtx->pLineCtx[0];
    if (pLineCtx != VP_NULL) {
        /* Program switcher params for ch0 with swyv (vbl) */
        pLineObj = pLineCtx->pLineObj;
        pLineObj->registers.swParam[0] &= ~VP886_R_SWPARAM_ABS_V;
        swParamBatVal = VpRoundedDivide(pDevObj->swyv - 5, 5);
        pLineObj->registers.swParam[0] |= swParamBatVal;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);

        /* Apply calibration factors for the new battery settings (battery cal and
           longitudinal cal */
        Vp886ApplyCalGeneral(pLineCtx);

        /* Program the cal registers if required.  This function may be called from
           Vp886ConfigLineFxs() or Vp886SetOptionRingingParams() which already
           program the cal registers at the end. */
        if (programCalRegistersCh0) {
            Vp886ProgramCalRegisters(pLineCtx);
        }
    }

    pLineCtx = pDevCtx->pLineCtx[1];
    if (pLineCtx != VP_NULL) {
        /* Program switcher params for ch1 with swzv (vbh) */
        pLineObj = pLineCtx->pLineObj;
        pLineObj->registers.swParam[0] &= ~VP886_R_SWPARAM_ABS_V;
        swParamBatVal = VpRoundedDivide(pDevObj->swzv - 5, 5);
        pLineObj->registers.swParam[0] |= swParamBatVal;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);

        /* Apply calibration factors for the new battery settings (battery cal and
           longitudinal cal */
        Vp886ApplyCalGeneral(pLineCtx);

        /* Program the cal registers if required.  This function may be called from
           Vp886ConfigLineFxs() or Vp886SetOptionRingingParams() which already
           program the cal registers at the end. */
        if (programCalRegistersCh1) {
            Vp886ProgramCalRegisters(pLineCtx);
        }
    }

    return;
}


/** Vp886RingSyncBegin()
  Begins ringing synchronization if necessary.
  
  This function is called for the line that is entering ringing.
  
  Returns TRUE if beginning ring sync, FALSE otherwise.
*/
bool
Vp886RingSyncBegin(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    uint8 otherChId = (pLineObj->channelId == 0 ? 1 : 0);
    VpLineCtxType *pOtherLineCtx = pDevCtx->pLineCtx[otherChId];
    Vp886LineObjectType *pOtherLineObj;
    Vp886LineStateInfoType otherLineStateInfo;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886RingSyncBegin()"));

    if (pLineObj->ringSyncState != VP886_RINGSYNC_STATE_IDLE) {
        /* Don't start ring sync if we're already in the middle of it */
        return FALSE;
    }
    
    if (pOtherLineCtx == VP_NULL) {
        /* Can't do anything if we can't see the other line */
        return FALSE;
    }
    
    pOtherLineObj = pOtherLineCtx->pLineObj;
    otherLineStateInfo = Vp886LineStateInfo(pOtherLineObj->lineState.currentState);

    if (otherLineStateInfo.normalEquiv != VP_LINE_RINGING) {
        /* No need to do anything if the other line isn't ringing */
        return FALSE;
    }
    
    if (pLineObj->inLineTest || pOtherLineObj->inLineTest) {
        /* Don't try to sync during line test */
        return FALSE;
    }

    if (pLineObj->ringFrequency != pOtherLineObj->ringFrequency) {
        /* Can't sync if ringing frequencies are different.  Ignoring the
           possibility of the register value for trapezoidal ringing matching
           the register value for sine ringing because they don't overlap in
           any realistic way. */
        return FALSE;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886RingSyncBegin() Delaying for ring sync"));
    VP_LINE_STATE(VpLineCtxType, pOtherLineCtx, ("Vp886RingSyncBegin() Pausing for ring sync"));

    /* Set the other line to active */
    if (otherLineStateInfo.polrev) {
        Vp886SetLineStateFxsInt(pOtherLineCtx, VP_LINE_ACTIVE_POLREV);
    } else {
        Vp886SetLineStateFxsInt(pOtherLineCtx, VP_LINE_ACTIVE);
    }
    
    /* Start the ring exit cleanup timer for the other line, which will watch
       the device substate to see when it has finished exiting ringing.  Set
       the ringSyncState flag so that the ring exit cleanup timer handler will
       know to call Vp886RingSyncFinish() */
    Vp886AddTimerMs(NULL, pOtherLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP,
        VP886_RING_EXIT_CLEANUP_CHECK_DELAY, 0, VP886_RING_EXIT_CLEANUP_CHECK);
    pOtherLineObj->ringSyncState = VP886_RINGSYNC_STATE_PAUSING;

    pLineObj->ringSyncState = VP886_RINGSYNC_STATE_DELAYING;

    pLineObj->ringSyncLineState = state;

    return TRUE;
}

/** Vp886RingSyncFinish()
  This function is called via timers for the line that was already in ringing.

  It will resume ringing on the line which is PAUSING (pLineCtx), then enter
  ringing after a delay on the line which is DELAYING (pOtherLineCtx).
*/
void
Vp886RingSyncFinish(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 otherChId = (pLineObj->channelId == 0 ? 1 : 0);
    VpLineCtxType *pOtherLineCtx = pDevCtx->pLineCtx[otherChId];
    Vp886LineObjectType *pOtherLineObj;
    int32 frequency_mHz;
    uint8 delayReg;
    uint32 delay_us;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886RingSyncFinish()"));

    /* Return the paused line to the ringing state it was in before */
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886RingSyncFinish() resuming ringing"));
    pLineObj->ringSyncState = VP886_RINGSYNC_STATE_FINISHING_PAUSE;
    Vp886SetLineStateFxsInt(pLineCtx, pLineObj->lineState.previous);

    if (pOtherLineCtx == VP_NULL) {
        /* Can't do anything if we can't see the other line */
        return;
    }
    
    pOtherLineObj = pOtherLineCtx->pLineObj;

    if (pOtherLineObj->ringSyncState != VP886_RINGSYNC_STATE_DELAYING) {
        /* The other line cancelled, so don't touch it. */
        return;
    }

    /* Determine the ringing frequency. */
    if (pLineObj->ringSine) {
        /* Sinusoidal ringing frequency is a 15-bit value with a 0-12000 Hz
           range.  12000000 mHz per 0x8000 reduces to 46875 mHz per 0x80. */
        frequency_mHz = VpRoundedDivide(pLineObj->ringFrequency * VP886_FREQ_STEP_NUM, VP886_FREQ_STEP_DEN);
    } else {
        /* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
           This means the maximum is 8000 Hz (8000000 mHz), and the minimum
           is 8000Hz / 0x7FFF, or 244.1 mHz.  Frequency can be calculated by 
           8000000mHz / FRQB. */
        frequency_mHz = VpRoundedDivide(VP886_TRAPFREQ_MAX, pLineObj->ringFrequency);
    }

    /* Calculate 1/4 of the ring period in microseconds
         1000000 * (1 / (frequency_mHz / 1000)) / 4
       = 250000000 / frequency_mHz */
    if (frequency_mHz == 0) {
        delay_us = 0;
    } else {
        delay_us = 250000000 / frequency_mHz;
    }

    VP_LINE_STATE(VpLineCtxType, pOtherLineCtx, ("Vp886RingSyncFinish() delaying ring entry by %luus", delay_us));
    
    /* Wait for ringing to be synced 90 degrees out of phase */
    if (VP886_IS_SF(pDevObj)) {
        /* SF revision devices have added a ring entry delay register so
           that we don't need the syswait. */
        /* Delay register is in units of 0.25ms, or 250 microseconds. */
        delayReg = VpRoundedDivide(delay_us, 250);

        /* Add the base delay, which the other line will be set to.
           We can't use the default value (0x04) as the base because it is a
           special case that does not delay the phase counter. */
        delayReg += VP886_R_RINGDELAY_BASE;

        /* Only update the delay register if the value changes. */
        if (delayReg != pOtherLineObj->registers.ringDelay[0]) {
            pOtherLineObj->registers.ringDelay[0] = delayReg;
            VpSlacRegWrite(NULL, pOtherLineCtx, VP886_R_RINGDELAY_WRT, VP886_R_RINGDELAY_LEN, pOtherLineObj->registers.ringDelay);
            VP_LINE_STATE(VpLineCtxType, pOtherLineCtx, ("Vp886RingSyncFinish() ring delay reg 0x%02X", pOtherLineObj->registers.ringDelay[0]));
        }
    } else {
        /* Send the SLAC Write Buffer, then SysWait for the delay. */
        Vp886SlacBufFlush(pDevCtx);
        VpSysWait(delay_us / 125);
        pDevObj->timestampValid = FALSE;
    }

    /* Enter ringing on the delayed line */
    pOtherLineObj->ringSyncState = VP886_RINGSYNC_STATE_FINISHING_DELAY;
    Vp886SetLineStateFxsInt(pOtherLineCtx, pOtherLineObj->ringSyncLineState);

    return;
}

/** Vp886RingSyncCancel()
  Call when a line needs to cancel its ring sync activity due to a new line
  state or line test request.

  If the other line is currently PAUSING for ring sync, its timer will continue
  to run, and it will resume ringing on its own.
  
  If the other line is currently DELAYING its ring entry, this function will
  set it to the requested ringing state immediately.
*/
void
Vp886RingSyncCancel(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 otherChId = (pLineObj->channelId == 0 ? 1 : 0);
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpLineCtxType *pOtherLineCtx = pDevCtx->pLineCtx[otherChId];
    Vp886LineObjectType *pOtherLineObj;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886RingSyncCancel()"));

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886RingSyncCancel()"));

    switch (pLineObj->ringSyncState) {
        case VP886_RINGSYNC_STATE_PAUSING:
            /* Clean up below for this state */
            break;
        case VP886_RINGSYNC_STATE_DELAYING:
            /* Only clear the ring sync state flag for this line.  We have to
               allow the ring exit cleanup timer to finish for the other line.
               If we try to set it back to ringing before its ring exit has
               finished, the line will not actually start ringing again. */
            pLineObj->ringSyncState = VP886_RINGSYNC_STATE_IDLE;
            return;
        default:
            /* Nothing to do here for idle or finishing states. */
            return;
    }

    pLineObj->ringSyncState = VP886_RINGSYNC_STATE_IDLE;

    Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP, 0, FALSE);

    if (pOtherLineCtx == VP_NULL) {
        /* Can't do anything if we can't see the other line */
        return;
    }
    
    pOtherLineObj = pOtherLineCtx->pLineObj;

    if (pOtherLineObj->ringSyncState == VP886_RINGSYNC_STATE_DELAYING) {
        /* Set the other line to its requested ringing state */
        VP_LINE_STATE(VpLineCtxType, pOtherLineCtx, ("Vp886RingSyncCancel() called on other line.  Entering requested ringing state"));
        pOtherLineObj->ringSyncState = VP886_RINGSYNC_STATE_IDLE;
        Vp886SetLineStateFxsInt(pOtherLineCtx, pOtherLineObj->ringSyncLineState);
    }
    
    return;
}

#ifdef CSLAC_GAIN_RELATIVE
/** Vp886SetRelGain()
  Implements VpSetRelGain() to adjust the transmit and receive gain for a line.

  Because this function generates an event with results attached, it cannot
  be called more than once, or in combination with other result-generating
  functions, without calling VpGetResults() in between to clear the results
  buffer.

  txLevel and rxLevel are 2.14 fixed-point unsigned numbers with a range of 0
  to 4.0, representing voltage gains.

  See the VP-API-II Reference Guide for more details on VpSetRelGain().
*/
VpStatusType
Vp886SetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886SetRelGain");

    /* Get out if device/line is not ready */
    if (!Vp886ReadyStatus(pDevCtx, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetRelGain");
        return status;
    }

    pLineObj->gxUserLevel = txLevel;
    pLineObj->grUserLevel = rxLevel;

    status = Vp886SetRelGainInt(pLineCtx);

    if (status == VP_STATUS_SUCCESS) {
        /* Generate the gain-complete event */
        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_LINE_EVID_GAIN_CMP, 0, handle, TRUE);
    }

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetRelGain");

    return status;

} /* Vp886SetRelGain() */


/** Vp886SetRelGainInt()
  Sets the GR and GX values for a channel according to the AC profile base
  values (pLineObj->gxBase,grBase) and the user-specified gains
  (pLineObj->gxUserLevel,grUserLevel).
*/
VpStatusType
Vp886SetRelGainInt(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpRelGainResultsType *relGainResults = &pLineObj->setRelGainResults;
    uint32 gxInt, grInt;
    uint8 gainCSD[VP886_R_GX_LEN];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetRelGainInt()+"));

    relGainResults->gResult = VP_GAIN_SUCCESS;

    /* Multiply the profile gain values by the user's adjustments. */
    gxInt = (uint32)pLineObj->gxBase * pLineObj->gxUserLevel / 16384;
    grInt = (uint32)pLineObj->grBase * pLineObj->grUserLevel / 16384;
    VP_GAIN(VpLineCtxType, pLineCtx, ("Post Multiplied gxInt (%ld) grInt (%ld)",
        gxInt, grInt));

    /* If overflow or underflow occurred, generate out-of-range result. */
    /* Requirement: 1.0 <= gxInt <= 4.0 */
    if ((gxInt < (uint32)VP886_REL_GAIN_GX_LOW_LIMIT) || (gxInt > (uint32)0x10000)) {
        relGainResults->gResult |= VP_GAIN_GX_OOR;
        gxInt = pLineObj->gxBase;
    }
    /* Requirement: 0.25 <= grInt <= 1.0 */
    if ((grInt < (uint32)VP886_REL_GAIN_GR_LOW_LIMIT) || (grInt > (uint32)0x4000)) {
        relGainResults->gResult |= VP_GAIN_GR_OOR;
        grInt = pLineObj->grBase;
    }

    VP_GAIN(VpLineCtxType, pLineCtx, ("Post Range Check gxInt (%ld) grInt (%ld)",
        gxInt, grInt));
    /* Write adjusted gain values to the device, and remember them for
       VpGetResults(). */
    Vp886ConvertSignedFixed2Csd((int32)gxInt - 0x4000, gainCSD);
    VP_GAIN(VpLineCtxType, pLineCtx, ("Post Converted gxInt: gainCSD = 0x%02X 0x%02X",
        gainCSD[0], gainCSD[1]));

    relGainResults->gxValue = ((uint16)gainCSD[0] << 8) + gainCSD[1];
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_GX_WRT, VP886_R_GX_LEN, gainCSD);

    Vp886ConvertSignedFixed2Csd((int32)grInt, gainCSD);
    VP_GAIN(VpLineCtxType, pLineCtx, ("Post Converted grInt: gainCSD = 0x%02X 0x%02X",
        gainCSD[0], gainCSD[1]));

    relGainResults->grValue = ((uint16)gainCSD[0] << 8) + gainCSD[1];
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_GR_WRT, VP886_R_GR_LEN, gainCSD);

    /* Absolute gain values are now unknown */
    pLineObj->options.absGain.gain_AToD = VP_ABS_GAIN_UNKNOWN;
    pLineObj->options.absGain.gain_DToA = VP_ABS_GAIN_UNKNOWN;

    return VP_STATUS_SUCCESS;
} /* Vp886SetRelGainInt() */
#endif /* CSLAC_GAIN_RELATIVE */


/** Vp886SetLineTone()
  Implements VpSetLineTone() to start or stop a tone or tone cadence.

  This layer performs error checking.  Vp886SetLineToneInt() performs the work.

  See the VP-API-II Reference Guide for more details on VpSetLineTone().
*/
VpStatusType
Vp886SetLineTone(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pCadProfile,
    VpDtmfToneGenType *pDtmfControl)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");
        return status;
    }

    /* Check for valid profile arguments. */
    if (
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_TONE, &pToneProfile) != VP_STATUS_SUCCESS) ||
        (Vp886GetProfileArg(pDevCtx, VP_PROFILE_TONECAD, &pCadProfile) != VP_STATUS_SUCCESS)
    ) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");
        return VP_STATUS_ERR_PROFILE;
    }

    if (pDtmfControl != VP_NULL) {
        if (pDtmfControl->dir != VP_DIRECTION_DS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetLineTone - DTMF generation only supported in the downstream direction"));
            Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");
            return VP_STATUS_INVALID_ARG;
        }
        if (pDtmfControl->toneId > 0x0F && pDtmfControl->toneId != VP_DIG_NONE) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetLineTone - Unknown DTMF digit %d", pDtmfControl->toneId));
            Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");
            return VP_STATUS_INVALID_ARG;
        }
    }

    /* Disallow tones in lines states where they cannot be generated */
    if (!Vp886LineStateInfo(pLineObj->lineState.usrCurrent).codec &&
        (pToneProfile != VP_PTABLE_NULL || pDtmfControl != VP_NULL))
    {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetLineToneInt - Tones cannot be generated in the current line state (%d)",
            pLineObj->lineState.usrCurrent));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");
        return VP_STATUS_DEVICE_BUSY;
    }

    status = Vp886SetLineToneInt(pLineCtx, pToneProfile, pCadProfile, pDtmfControl);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetLineTone");
    return status;
}


/** Vp886SetLineToneInt()
  Performs the work of VpSetLineTone().  This begins by ending any existing
  tone or sequence.  Then we determine further action based on which of the
  function arguments are non-NULL.
*/
VpStatusType
Vp886SetLineToneInt(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pCadProfile,
    VpDtmfToneGenType *pDtmfControl)
{
    VpStatusType status = VP_STATUS_SUCCESS;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    /* End any current tone or cadence. */
    Vp886SetToneCtrl(pLineCtx, FALSE, FALSE, FALSE, FALSE, FALSE);
#ifdef VP_CSLAC_SEQ_EN
    if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
        Vp886CadenceStop(pLineCtx, TRUE, FALSE, FALSE);
    }
    if (pLineObj->cid.active) {
        Vp886CidStop(pLineCtx);
    }
#endif /* VP_CSLAC_SEQ_EN */

    if (pToneProfile != VP_PTABLE_NULL) {
#ifdef VP_CSLAC_SEQ_EN
        if (pCadProfile != VP_PTABLE_NULL) {
            /* Cadence profile provided.  Check whether it is normal or a
               howler tone profile. */
            if (Vp886HowlerToneInit(pLineCtx, pCadProfile, &status)) {
                /* Cadence profile specifies a howler tone */
                return status;
            } else {
                /* Normal cadence profile */
                status = Vp886ApplyToneProfile(pLineCtx, pToneProfile);
                if (status != VP_STATUS_SUCCESS) {
                    return status;
                }
                status = Vp886CadenceStart(pLineCtx, pCadProfile, 0);
            }
        } else {
            /* Always-on tone, no cadence */
            status = Vp886ApplyToneProfile(pLineCtx, pToneProfile);
            if (status != VP_STATUS_SUCCESS) {
                return status;
            }
            Vp886SetToneCtrl(pLineCtx,
                pLineObj->toneGens & VP886_TONEGEN_BIAS,
                pLineObj->toneGens & VP886_TONEGEN_A,
                pLineObj->toneGens & VP886_TONEGEN_B,
                pLineObj->toneGens & VP886_TONEGEN_C,
                pLineObj->toneGens & VP886_TONEGEN_D);
        }
#else
        /* When the sequencer code is not compiled in, assume always-on tone */
        status = Vp886ApplyToneProfile(pLineCtx, pToneProfile);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }
        Vp886SetToneCtrl(pLineCtx,
            pLineObj->toneGens & VP886_TONEGEN_BIAS,
            pLineObj->toneGens & VP886_TONEGEN_A,
            pLineObj->toneGens & VP886_TONEGEN_B,
            pLineObj->toneGens & VP886_TONEGEN_C,
            pLineObj->toneGens & VP886_TONEGEN_D);
#endif /* VP_CSLAC_SEQ_EN */

    } else if (pDtmfControl != VP_NULL) {
        if (pDtmfControl->toneId == VP_DIG_NONE) {
            return VP_STATUS_SUCCESS;
        }
        status = Vp886ProgramDTMFDigit(pLineCtx, pDtmfControl->toneId, 0);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }
        Vp886SetToneCtrl(pLineCtx, FALSE, FALSE, FALSE, TRUE, TRUE);
    }

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
    /* If high gain mode (howler line state) is active, adjust R and GR based
       on the tone type */
    if (status == VP_STATUS_SUCCESS && pLineObj->inHighGainMode) {
        Vp886HighGainSetRFilter(pLineCtx);
    }
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */

    return status;
}


/** Vp886ApplyToneProfile()
  Programs the signal generator parameters from a tone profile.
*/
VpStatusType
Vp886ApplyToneProfile(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    if (pToneProfile == VP_PTABLE_NULL) {
        return VP_STATUS_ERR_PROFILE;
    }

    if (pToneProfile[VP_PROFILE_VERSION] < 2) {
        /* Apply an old version tone profile */

        uint8 sigABReg[VP886_R_SIGAB_LEN];
        /* The AB register is 11 bytes, but the profile only contains the last 8 -
           the frequency and amplitude.  Set the first three bytes (bias and other
           options) to 0 for normal tones. */
        VpMemSet(sigABReg, 0, VP886_R_SIGAB_LEN);
        VpMemCpy(&sigABReg[3], &pToneProfile[VP_TONE_PROFILE_AB_DATA], 8);

        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SIGAB_WRT, VP886_R_SIGAB_LEN, sigABReg);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SIGCD_WRT, VP886_R_SIGCD_LEN, &pToneProfile[VP_TONE_PROFILE_CD_DATA]);

        pLineObj->toneGens = VP886_TONEGEN_A | VP886_TONEGEN_B | VP886_TONEGEN_C | VP886_TONEGEN_D;

    } else {
        /* Apply a new version tone profile */

        uint8 mpiLen = pToneProfile[VP_PROFILE_MPI_LEN];
        uint8 mpiCmd = pToneProfile[VP_PROFILE_DATA_START];
        VpProfilePtrType pTemp = &pToneProfile[VP_PROFILE_DATA_START + mpiLen];

        /* Check wideband setting */
        if ((*pTemp) & VP_TONE_PROFILE_V2_WIDEBAND) {
            switch (pLineObj->options.codec) {
                case VP_OPTION_LINEAR_WIDEBAND:
                case VP_OPTION_ALAW_WIDEBAND:
                case VP_OPTION_MLAW_WIDEBAND:
                    break;
                default:
                    VP_ERROR(VpLineCtxType, pLineCtx, ("Line must be in wideband mode to use a wideband tone profile"));
                    return VP_STATUS_ERR_PROFILE;
            }
        }

        /* Get the enable generator flags */
        pLineObj->toneGens = ((*pTemp) & VP_TONE_PROFILE_V2_GENS);

        /* Send the MPI section data */
        pTemp = &pToneProfile[VP_PROFILE_DATA_START + 1];
        VpSlacRegWrite(NULL, pLineCtx, mpiCmd, mpiLen - 1, pTemp);
    }

    return VP_STATUS_SUCCESS;
}


/** Vp886ProgramDTMFDigit()
  Programs signal generators A and B with the frequencies of the specified
  DTMF digit.

  When the 'amplitude' argument is 0, the default levels are -10dBm0.
*/
VpStatusType
Vp886ProgramDTMFDigit(
    VpLineCtxType *pLineCtx,
    VpDigitType digit,
    uint16 amplitude)
{
    uint8 sigGenCDParams[VP886_R_SIGCD_LEN] = {
        0x00, 0x00, /* Replace with required column Frequency */
        0x1C, 0x32, /* Level = -10dBm0 */
        0x00, 0x00, /* Replace with required row Frequency */
        0x1C, 0x32  /* Level = -10dBm0 */
    };

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

    switch(digit) {
        case 1:
        case 4:
        case 7:
        case VP_DIG_ASTER:
            sigGenCDParams[0] = columnFreqs[0];
            sigGenCDParams[1] = columnFreqs[1];
            break;

        case 2:
        case 5:
        case 8:
        case VP_DIG_ZERO:
            sigGenCDParams[0] = columnFreqs[2];
            sigGenCDParams[1] = columnFreqs[3];
            break;

        case 3:
        case 6:
        case 9:
        case VP_DIG_POUND:
            sigGenCDParams[0] = columnFreqs[4];
            sigGenCDParams[1] = columnFreqs[5];
            break;

        case VP_DIG_A:
        case VP_DIG_B:
        case VP_DIG_C:
        case VP_DIG_D:
            sigGenCDParams[0] = columnFreqs[6];
            sigGenCDParams[1] = columnFreqs[7];
            break;

        default:
            return VP_STATUS_INVALID_ARG;
    }

    switch(digit) {
        case 1:
        case 2:
        case 3:
        case VP_DIG_A:
            sigGenCDParams[4] = rowFreqs[0];
            sigGenCDParams[5] = rowFreqs[1];
            break;

        case 4:
        case 5:
        case 6:
        case VP_DIG_B:
            sigGenCDParams[4] = rowFreqs[2];
            sigGenCDParams[5] = rowFreqs[3];
            break;

        case 7:
        case 8:
        case 9:
        case VP_DIG_C:
            sigGenCDParams[4] = rowFreqs[4];
            sigGenCDParams[5] = rowFreqs[5];
            break;

        case VP_DIG_ASTER:
        case VP_DIG_ZERO:
        case VP_DIG_POUND:
        case VP_DIG_D:
            sigGenCDParams[4] = rowFreqs[6];
            sigGenCDParams[5] = rowFreqs[7];
            break;

        default:
            return VP_STATUS_INVALID_ARG;
    }

    if (amplitude != 0) {
        sigGenCDParams[2] = (amplitude & 0xFF00) >> 8;
        sigGenCDParams[3] = (amplitude & 0x00FF);
        sigGenCDParams[6] = (amplitude & 0xFF00) >> 8;
        sigGenCDParams[7] = (amplitude & 0x00FF);
    }

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SIGCD_WRT, VP886_R_SIGCD_LEN, sigGenCDParams);

    return VP_STATUS_SUCCESS;
}


/** Vp886SetToneCtrl
  Enables or disables each of the signal generators - A, B, C, D, and bias.
*/
void
Vp886SetToneCtrl(
    VpLineCtxType *pLineCtx,
    bool bias,
    bool sigA,
    bool sigB,
    bool sigC,
    bool sigD)
{
    uint8 sigctrlReg = 0x00;

    if (bias) {
        sigctrlReg |= VP886_R_SIGCTRL_EN_BIAS;
    }
    if (sigA) {
        sigctrlReg |= VP886_R_SIGCTRL_EN_SIGA;
    }
    if (sigB) {
        sigctrlReg |= VP886_R_SIGCTRL_EN_SIGB;
    }
    if (sigC) {
        sigctrlReg |= VP886_R_SIGCTRL_EN_SIGC;
    }
    if (sigD) {
        sigctrlReg |= VP886_R_SIGCTRL_EN_SIGD;
    }

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SIGCTRL_WRT, VP886_R_SIGCTRL_LEN, &sigctrlReg);

    return;
}


/** Vp886SetOption()
  Implements VpSetOption() to apply line or device settings.

  This layer makes decisions on how to proceed based on pDevCtx, pLineCtx, and
  the option ID.
   - If a line context is given, call Vp886SetOptionLine().
   - If a device context and device-specific option are given, call
     Vp886SetOptionDevice().
   - If a device context and line-specific option are given, call
     Vp886SetOptionLine() for each line.

  See the VP-API-II Reference Guide for more details on VpSetOption().
*/
VpStatusType
Vp886SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    if (pDevCtx != VP_NULL) {
        Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
        VpLineCtxType *pLineCtxLocal;
        Vp886LineObjectType *pLineObj;
        uint8 channelId;

        if (!(pDevObj->busyFlags & VP_DEV_IMPL_DEFAULT_OPTIONS)) {
            /* Skip critical section during VpImplementDefaultSettings() */
            Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886SetOption");
        }

        switch (option) {
            /* Line Options */
            case VP_OPTION_ID_EVENT_MASK:
            case VP_OPTION_ID_ZERO_CROSS:
            case VP_OPTION_ID_PULSE_MODE:
            case VP_OPTION_ID_LINE_STATE:
            case VP_OPTION_ID_RING_CNTRL:
            case VP_OPTION_ID_SWITCHER_CTRL:
            case VP_OPTION_ID_TIMESLOT:
            case VP_OPTION_ID_CODEC:
            case VP_OPTION_ID_PCM_HWY:
            case VP_OPTION_ID_LOOPBACK:
            case VP_OPTION_ID_PCM_TXRX_CNTRL:
            case VP_OPTION_ID_ABS_GAIN:
            case VP_OPTION_ID_DCFEED_PARAMS:
            case VP_OPTION_ID_RINGING_PARAMS:
            case VP_OPTION_ID_GND_FLT_PROTECTION:
            case VP_OPTION_ID_DTMF_MODE:
            case VP_OPTION_ID_RINGTRIP_CONFIRM:
            case VP_OPTION_ID_HIGHPASS_FILTER:
            case VP_OPTION_ID_DTMF_PARAMS:
                /* Loop through all of the valid channels associated with this
                   device. */
                for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
                    pLineCtxLocal = pDevCtx->pLineCtx[channelId];
                    if (pLineCtxLocal == VP_NULL) {
                        continue;
                    }
                    pLineObj = pLineCtxLocal->pLineObj;

                    /* FXS-only events */
                    if ((option == VP_OPTION_ID_ZERO_CROSS) ||
                        (option == VP_OPTION_ID_PULSE_MODE) ||
                        (option == VP_OPTION_ID_LINE_STATE) ||
                        (option == VP_OPTION_ID_RING_CNTRL) ||
                        (option == VP_OPTION_ID_SWITCHER_CTRL))
                    {
                        /* TODO: If device is FXO-only, return VP_STATUS_OPTION_NOT_SUPPORTED
                        if (pDevObj->stateInt & VP886_IS_FXO_ONLY) {
                            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886SetOption");
                            return VP_STATUS_OPTION_NOT_SUPPORTED;
                        } */
                        if (!pLineObj->isFxs) {
                            continue;
                        }
                    }
                    status = Vp886SetOptionLine(pLineCtxLocal, option, pValue);
                    if (status != VP_STATUS_SUCCESS) {
                        break;
                    }
                }
                break;
            default:
                status = Vp886SetOptionDevice(pDevCtx, option, pValue);
                break;
        }

        if (!(pDevObj->busyFlags & VP_DEV_IMPL_DEFAULT_OPTIONS)) {
            /* Skip critical section during VpImplementDefaultSettings() */
            Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886SetOption");
        }

    } else {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        if (!(pLineObj->busyFlags & VP886_LINE_DEFAULT_OPTIONS)) {
            /* Skip critical section during VpImplementDefaultSettings() */
            Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886SetOption");
        }

        status = Vp886SetOptionLine(pLineCtx, option, pValue);

        if (!(pLineObj->busyFlags & VP886_LINE_DEFAULT_OPTIONS)) {
            /* Skip critical section during VpImplementDefaultSettings() */
            Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetOption");
        }
    }

    return status;
}


/** Vp886SetOptionDevice()
  Implements VpSetOption() for device specific options.
*/
VpStatusType
Vp886SetOptionDevice(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886DevOptionsCacheType *pCache = &pDevObj->options;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    VpLineCtxType *pLineCtx;
    Vp886LineObjectType *pLineObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice+"));

    if (option != VP_OPTION_ID_DEBUG_SELECT && !Vp886ReadyStatus(pDevCtx, VP_NULL, &status)) {
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice-"));
        return status;
    }

    switch (option) {
        case VP_DEVICE_OPTION_ID_PULSE: {
            VpOptionPulseType *pUserBuf = pValue;
            pCache->pulse = *pUserBuf;
            break;
        }
        case VP_DEVICE_OPTION_ID_PULSE2: {
            VpOptionPulseType *pUserBuf = pValue;
            pCache->pulse2 = *pUserBuf;
            break;
        }
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT: {
            VpOptionCriticalFltType *pUserBuf = pValue;

            /* Set or clear the device automatic thermal fault shutdown bit
               according to the option.  The API will ALSO set the line state
               to VP_LINE_DISCONNECT if a fault occurs to keep the state in
               sync. */
            for (channelId = 0; channelId < maxChan; channelId++) {
                pLineCtx = pDevCtx->pLineCtx[channelId];
                if (pLineCtx == VP_NULL) {
                    continue;
                }
                pLineObj = pLineCtx->pLineObj;
                pLineObj->registers.ssCfg[0] &= ~VP886_R_SSCFG_AUTO_THERMFAULT;
                if (pUserBuf->thermFltDiscEn) {
                    pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_AUTO_THERMFAULT;
                }
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
            }

            pCache->criticalFlt = *pUserBuf;
            break;
        }
        case VP_DEVICE_OPTION_ID_DEVICE_IO: {
            VpOptionDeviceIoType *pUserBuf = pValue;
            VpOptionDeviceIoType tempCache = pCache->deviceIo;
            VpStatusType tempStatus;
            uint8 ioDirReg[VP886_MAX_NUM_CHANNELS];
            
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
                /* These devices do not have usable IO pins */
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice() - Device does not support I/O lines"));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;  
            }

            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_TWO_PER_CH) {
            /* For each channel .... */
            for (channelId = 0; channelId < maxChan; channelId++) {
                uint8 direction;
                uint8 outputType;

                if (channelId == 0) {
                    direction = (pUserBuf->directionPins_31_0 & 0x3);
                    outputType = (pUserBuf->outputTypePins_31_0 & 0x3);
                } else {
                    direction = (pUserBuf->directionPins_31_0 & 0xC) >> 2;
                    outputType = (pUserBuf->outputTypePins_31_0 & 0xC) >> 2;
                }
                tempStatus = Vp886SetOptionIoCfg(pDevCtx, channelId, direction, outputType, &ioDirReg[channelId]);
                if (tempStatus != VP_STATUS_SUCCESS) {
                    status = tempStatus;
                }
            }
            /* Write the IO direction registers if there was no critical error */
            if (status == VP_STATUS_SUCCESS || status == VP_STATUS_DEDICATED_PINS) {
                for (channelId = 0; channelId < maxChan; channelId++) {
                    VP_LINE_STATE(VpDevCtxType, pDevCtx, ("ch%d ioDirReg %02X", channelId, ioDirReg[channelId]));
                    pDevObj->ecVal = (channelId == 0 ? VP886_EC_1 : VP886_EC_2);
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDirReg[channelId]);
                    pDevObj->ecVal = VP886_EC_GLOBAL;
                }
            } else {
                /* If we don't write the registers, restore the old cache */
                pCache->deviceIo = tempCache;
            }
            } else if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2) {
                /* Ch0 IO2 only */
                uint8 direction;
                uint8 outputType;
                
                direction = (pUserBuf->directionPins_31_0 & 0x1);
                outputType = (pUserBuf->outputTypePins_31_0 & 0x1);
                status = Vp886SetOptionIoCfg(pDevCtx, 0, direction, outputType, &ioDirReg[0]);
                /* Write the IO direction register if there was no critical error */
                if (status == VP_STATUS_SUCCESS || status == VP_STATUS_DEDICATED_PINS) {
                    VP_LINE_STATE(VpDevCtxType, pDevCtx, ("ch0 ioDirReg %02X", ioDirReg[0]));
                    pDevObj->ecVal = VP886_EC_1;
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDirReg[0]);
                    pDevObj->ecVal = VP886_EC_GLOBAL;
                } else {
                    /* If we don't write the registers, restore the old cache */
                    pCache->deviceIo = tempCache;
                }
            } else {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice() - Unknown I/O capability type %d", pDevObj->ioCapability));
            }
            break;
        }
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING: {
            VpOptionAdaptiveRingingType *pUserBuf = pValue;
            for (channelId = 0; channelId < maxChan; channelId++) {
                pLineCtx = pDevCtx->pLineCtx[channelId];
                if (pLineCtx == VP_NULL) {
                    continue;
                }
                pLineObj = pLineCtx->pLineObj;
                if (pLineObj->lineState.usrCurrent == VP_LINE_RINGING ||
                    pLineObj->lineState.usrCurrent == VP_LINE_RINGING_POLREV)
                {
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice() Cannot change VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING while channel %d is ringing", channelId));
                    status = VP_STATUS_DEVICE_BUSY;
                    break;
                }
            }
            if (status != VP_STATUS_SUCCESS) {
                break;
            }
            /* Range checking */
            if (pUserBuf->validMask & VP_ADAPTIVE_RINGING_CFG_MIN_V_PCT) {
                if (pUserBuf->minVoltagePercent < 10 || pUserBuf->minVoltagePercent > 100) {
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice() minVoltagePercent %d invalid. Must be between 10 and 100", pUserBuf->minVoltagePercent));
                    status = VP_STATUS_INVALID_ARG;
                    break;
                }
            }

            /* Change option values for each bit that is set in validMask */
            if (pUserBuf->validMask & VP_ADAPTIVE_RINGING_CFG_POWER) {
                /* If adaptive ringing is being disabled after previously being
                   enabled, reset ringing parameters back to their normal values. */
                if (pUserBuf->power == VP_ADAPTIVE_RINGING_DISABLED &&
                    pCache->adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED &&
                    (pDevObj->busyFlags & VP_DEV_INIT_CMP))
                {
                    VpOptionRingingParamsType ringingParams;
                    for (channelId = 0; channelId < maxChan; channelId++) {
                        pLineCtx = pDevCtx->pLineCtx[channelId];
                        if (pLineCtx == VP_NULL) {
                            continue;
                        }
                        pLineObj = pLineCtx->pLineObj;
                        /* Restore the ringing amplitude and RTTH.
                           Have to do it through the SetOption to apply cal.
                           ILR will automatically be restored too (loop supervision register write) */
                        ringingParams.validMask = VP_OPTION_CFG_AMPLITUDE | VP_OPTION_CFG_RINGTRIP_THRESHOLD;
                        ringingParams.amplitude = VpRoundedDivide(pLineObj->ringAmplitude * VP886_AMP_STEP_NUM, VP886_AMP_STEP_DEN);
                        ringingParams.ringTripThreshold = pLineObj->rtth * VP886_RTTH_STEP + VP886_RTTH_OFFSET;
                        Vp886SetOptionRingingParams(pLineCtx, &ringingParams);

                        /* If we're doing fixed battery, restore the switcher ringing voltage */
                        if (VP886_IS_TRACKER(pDevObj) &&
                            (pLineObj->registers.swParam[0] & VP886_R_SWPARAM_RING_TRACKING) == VP886_R_SWPARAM_RING_TRACKING_DIS)
                        {
                            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);
                        }
                    }
                }
                pCache->adaptiveRinging.power = pUserBuf->power;
            }
            if (pUserBuf->validMask & VP_ADAPTIVE_RINGING_CFG_MIN_V_PCT) {
                pCache->adaptiveRinging.minVoltagePercent = pUserBuf->minVoltagePercent;
            }
            if (pUserBuf->validMask & VP_ADAPTIVE_RINGING_CFG_MODE) {
                /* If the adaptive ringing mode is being changed while enabled,
                   reset ringing parameters back to their normal values. */
                if (pUserBuf->mode != pCache->adaptiveRinging.mode &&
                    pCache->adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED &&
                    (pDevObj->busyFlags & VP_DEV_INIT_CMP))
                {
                    for (channelId = 0; channelId < maxChan; channelId++) {
                        pLineCtx = pDevCtx->pLineCtx[channelId];
                        if (pLineCtx == VP_NULL) {
                            continue;
                        }
                        pLineObj = pLineCtx->pLineObj;

                        /* Reset the adaptive ringing parameters */
                        Vp886AdaptiveRingingInit(pLineCtx);
                    }
                }

                if (pCache->adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED) {
                    if ((pUserBuf->mode == VP_ADAPT_RING_SHARED_TRACKER) && (pDevObj->stateInt & VP886_SHARED_SUPPLY)) {
                        pCache->adaptiveRinging.mode = VP_ADAPT_RING_SHARED_TRACKER;
                    } else if ((pUserBuf->mode == VP_ADAPT_RING_SHARED_BB_ABS) && VP886_IS_ABS(pDevObj) &&
                               (pDevObj->devProfileData.swCfg == VP886_DEV_PROFILE_SW_CONF_BB)) {
                        pCache->adaptiveRinging.mode = VP_ADAPT_RING_SHARED_BB_ABS;
                    } else if ((pUserBuf->mode == VP_ADAPT_RING_SINGLE_BB_TRACKER) && VP886_IS_TRACKER(pDevObj) &&
                        (pDevObj->devProfileData.swCfg == VP886_DEV_PROFILE_SW_CONF_BB) && (pDevObj->staticInfo.maxChannels == 1)) {
                        pCache->adaptiveRinging.mode = VP_ADAPT_RING_SINGLE_BB_TRACKER;
                    } else {
                        pCache->adaptiveRinging.power = VP_ADAPTIVE_RINGING_DISABLED;
                        status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    }
                }
            }
            break;
        }
#endif
        case VP_DEVICE_OPTION_ID_RING_PHASE_SYNC: {
            VpOptionRingPhaseSyncType *pUserBuf = pValue;
            pCache->ringPhaseSync = *pUserBuf;
            break;
        }
        case VP_DEVICE_OPTION_ID_FSYNC_RATE: {
            VpOptionFsyncRateType *pUserBuf = pValue;
            if (!VP886_IS_SF1(pDevObj)) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            if (*pUserBuf == VP_FSYNC_RATE_8KHZ) {
                pDevObj->registers.devCfg[0] &= ~VP886_R_DEVCFG_FS_16K;
                VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_DEVCFG_WRT, VP886_R_DEVCFG_LEN, pDevObj->registers.devCfg);
            } else if (*pUserBuf == VP_FSYNC_RATE_16KHZ) {
                pDevObj->registers.devCfg[0] |= VP886_R_DEVCFG_FS_16K;
                VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_DEVCFG_WRT, VP886_R_DEVCFG_LEN, pDevObj->registers.devCfg);
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid VpOptionFsyncRateType %d", *pUserBuf));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            pCache->fsyncRate = *pUserBuf;
            break;
        }
#if (VP_CC_DEBUG_SELECT != 0)
        case VP_OPTION_ID_DEBUG_SELECT: {
            uint32 *pUserBuf = pValue;
            pDevObj->debugSelectMask = *pUserBuf;
            break;
        }
#endif /* (VP_CC_DEBUG_SELECT != 0) */
        default:
            if (pDevObj->busyFlags & VP_DEV_INIT_CMP) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice() Device option not supported 0x%02X", option));
            }
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            break;
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886SetOptionDevice-"));
    return status;
}


/** Vp886SetOptionLine()
  Implements VpSetOption() for line-specific options.
*/
VpStatusType
Vp886SetOptionLine(
    VpLineCtxType *pLineCtx,
    VpOptionIdType option,
    void *pValue)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineOptionsCacheType *pCache = &pLineObj->options;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine+"));

    if (option != VP_OPTION_ID_DEBUG_SELECT && !Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine-"));
        return status;
    }

    switch (option) {
        /* Line Options */
        case VP_OPTION_ID_EVENT_MASK: {
            VpOptionEventMaskType *pUserBuf = pValue;
            status = Vp886SetOptionEventMask(pLineCtx, pUserBuf);
            /* Cached values for event masks are handled uniquely in
               Vp886SetOptionEventMask() */
            break;
        }
        case VP_OPTION_ID_PULSE_MODE: {
            VpOptionPulseModeType *pUserBuf = pValue;
            pCache->pulseMode = *pUserBuf;
            Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_PULSE_DECODE, 0, FALSE);
            VpPulseDecodeInit(&pLineObj->pulseDecodeData);
            break;
        }
        case VP_OPTION_ID_LINE_STATE: {
            VpOptionLineStateType *pUserBuf = pValue;

            /* Option does not apply to FXO */
            if (pLineObj->isFxs == FALSE) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }

            /* Only supports one type of battery control, so make sure it
               is set correctly. If not, return error otherwise continue */
            if (pUserBuf->bat != VP_OPTION_BAT_AUTO) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() - Invalid battery option %d", pUserBuf->bat));
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine-"));
                return VP_STATUS_INVALID_ARG;
            }

            /* Arguments are confirmed valid, save in line object */
            pCache->lineState = *pUserBuf;

            /* Set the clear the SPR bit as necessary */
            if (pUserBuf->battRev == TRUE) {
                pLineObj->registers.ssCfg[0] &= ~(VP886_R_SSCFG_SMOOTHPOLREV);
            } else {
                pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_SMOOTHPOLREV;
            }
            /* Write the value back to the device */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
            break;
        }
        case VP_OPTION_ID_RING_CNTRL: {
            VpOptionRingControlType *pUserBuf = pValue;
            if (!Vp886IsValidLineState(pLineCtx, pUserBuf->ringTripExitSt)) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine: Invalid ring trip exit state %d",
                    pUserBuf->ringTripExitSt));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            pCache->ringControl = *pUserBuf;
            if (pCache->ringControl.ringExitDbncDur > 255 * 8) {
                VP_WARNING(VpLineCtxType, pLineCtx,
                    ("Vp886SetOptionLine: Capping ringExitDbncDur to 255ms from %d",
                    pCache->ringControl.ringExitDbncDur / 8));
                pCache->ringControl.ringExitDbncDur = 255 * 8;
            }

            /* Set or clear the ASSC_RNG bit.  When the bit is set, automatic
               ring trip is disabled */
            if (pUserBuf->ringTripExitSt == VP_LINE_RINGING ||
                pUserBuf->ringTripExitSt == VP_LINE_RINGING_POLREV)
            {
                pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_AUTO_RINGTRIP;
            } else {
                pLineObj->registers.ssCfg[0] &= ~VP886_R_SSCFG_AUTO_RINGTRIP;
            }

            /* Set or clear the ZXR bit.  When the bit is set, zero cross is disabled */
            if (pUserBuf->zeroCross == VP_OPTION_ZC_NONE) {
                pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_ZEROCROSS;
            } else {
                pLineObj->registers.ssCfg[0] &= ~VP886_R_SSCFG_ZEROCROSS;
            }
            /* Write the value back to the device */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
            break;
        }
        case VP_OPTION_ID_SWITCHER_CTRL: {
            bool *pUserBuf = pValue;
            /*  TODO: Since this option defaults to false (per the API user's guide)
                should autoshutdown be disabled in VpInitDevice() after it is confirmed
                that the device came up successfully? */
            bool shutDownEn = *pUserBuf;
            /* Option does not apply to FXO. This check should have
               been performed earlier, but check it again just in case */
            if (!pLineObj->isFxs) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }

            /* Argument is confirmed valid, save in line object */
            pCache->switcherCtrl = *pUserBuf;

            /* Set or clear the overvoltage shutdown bits as neceesary.
               We can't use the overcurrent autoshutdown because it requires
               extra filtering from the API to determine real overcurrent
               situations. */
            if (shutDownEn == TRUE) {
                pLineObj->registers.ssCfg[1] |= VP886_R_SSCFG_AUTO_OVERVOLTAGE;
            } else {
                pLineObj->registers.ssCfg[1] &= ~VP886_R_SSCFG_AUTO_OVERVOLTAGE;
            }

            /* Write the value back to the device */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
            break;
        }
        case VP_OPTION_ID_TIMESLOT: {
            VpOptionTimeslotType *pUserBuf = pValue;
            status = Vp886SetOptionTimeslot(pLineCtx, pUserBuf);
            if (status == VP_STATUS_SUCCESS) {
                pCache->timeslot = *pUserBuf;
            }
            break;
        }
        case VP_OPTION_ID_CODEC: {
            VpOptionCodecType *pUserBuf = pValue;
            status = Vp886SetOptionCodec(pLineCtx, pUserBuf);
            if (status == VP_STATUS_SUCCESS) {
                pCache->codec = *pUserBuf;
            }
            break;
        }
        case VP_OPTION_ID_LOOPBACK: {
            VpOptionLoopbackType *pUserBuf = pValue;

            /* Timeslot loopback via loopback register */
            switch(*pUserBuf) {
                case VP_OPTION_LB_TIMESLOT:
                    pLineObj->registers.opCond[0] |= VP886_R_OPCOND_TSA_LOOPBACK;
                    /* Disable the voice DAC to prevent signals in the receive
                       path from showing up on tip and ring. */
                    pLineObj->registers.icr4[0] |= VP886_R_ICR4_VDAC_EN;
                    pLineObj->registers.icr4[1] &= ~VP886_R_ICR4_VDAC_EN;
                    break;

                case VP_OPTION_LB_OFF:
                    pLineObj->registers.opCond[0] &= ~(VP886_R_OPCOND_TSA_LOOPBACK);
                    pLineObj->registers.icr4[0] &= ~VP886_R_ICR4_VDAC_EN;
                    break;

                case VP_OPTION_LB_DIGITAL:
                case VP_OPTION_LB_CHANNELS:
                default:
                    VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() - Invalid Loopback Type %d", *pUserBuf));
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine-"));
                    return VP_STATUS_INVALID_ARG;
            }

            VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, &pLineObj->registers.opCond[0]);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, &pLineObj->registers.icr4[0]);
            pCache->loopback = *pUserBuf;
            break;
        }
        case VP_OPTION_ID_PCM_TXRX_CNTRL: {
            VpOptionPcmTxRxCntrlType *pUserBuf = pValue;
            pCache->pcmTxRxCntrl = *pUserBuf;
            Vp886ApplyPcmTxRx(pLineCtx);
            break;
        }
        case VP_OPTION_ID_ABS_GAIN: {
            VpOptionAbsGainType *pUserBuf = pValue;
            /* The cached value will be updated by VpCSLACSetAbsGain(). */
            status = VpCSLACSetAbsGain(pLineCtx, pUserBuf);
            break;
        }
        case VP_OPTION_ID_LINE_IO_CFG: {
            VpOptionLineIoConfigType *pUserBuf = pValue;
            VpOptionDeviceIoType tempCache = pDevObj->options.deviceIo;
            uint8 ioDirReg;
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
                /* These devices do not have usable IO pins */
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() - Device does not support I/O lines"));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;  
            }
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2 && pLineObj->channelId == 1) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() - This channel does not have I/O lines"));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            status = Vp886SetOptionIoCfg(pDevCtx, pLineObj->channelId,
                pUserBuf->direction, pUserBuf->outputType, &ioDirReg);
            /* Write the IO direction register if there was no critical error */
            if (status == VP_STATUS_SUCCESS || status == VP_STATUS_DEDICATED_PINS) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("ioDirReg %02X", ioDirReg));
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, &ioDirReg);
            } else {
                /* If we don't write the registers, restore the old cache */
                pDevObj->options.deviceIo = tempCache;
            }
            break;
        }
        case VP_OPTION_ID_DCFEED_PARAMS: {
            VpOptionDcFeedParamsType *pUserBuf = pValue;
            status = Vp886SetOptionDcFeedParams(pLineCtx, pUserBuf);
            break;
        }
        case VP_OPTION_ID_RINGING_PARAMS: {
            VpOptionRingingParamsType *pUserBuf = pValue;
            status = Vp886SetOptionRingingParams(pLineCtx, pUserBuf);
            break;
        }
        case VP_OPTION_ID_GND_FLT_PROTECTION: {
            VpOptionGndFltProtType *pUserBuf = pValue;
            /* If the option is disabled while the feature is running, stop it. */
            if (pUserBuf->enable == FALSE &&
                pLineObj->gndFltProt.state != VP886_GNDFLTPROT_ST_INACTIVE)
            {
                Vp886GndFltProtStop(pLineCtx, TRUE);
            }
            pCache->gndFltProt = *pUserBuf;
            break;
        }
#ifdef VP886_INCLUDE_DTMF_DETECT
        case VP_OPTION_ID_DTMF_MODE: {
            VpOptionDtmfModeType *pUserBuf = pValue;
            status = Vp886SetOptionDtmfMode(pLineCtx, pUserBuf);
            break;
        }
        case VP_OPTION_ID_DTMF_PARAMS: {
            VpOptionDtmfParamsType *pUserBuf = pValue;
            VpDtmfDetectParamsType *pParams = &pLineObj->dtmf.detectData.params;
            int32 dB;
            int32 factor;
            if (pUserBuf->validMask & VP_OPTION_CFG_MIN_DETECT) {
                if (pUserBuf->minDetect > 0 || pUserBuf->minDetect < -535) {
                    VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine - minDetect out of range (%ld)",
                        pUserBuf->minDetect));
                    return VP_STATUS_INVALID_ARG;
                }
                pCache->dtmfParams.minDetect = pUserBuf->minDetect;
                /* We know empirically that -5dBm0 corresponds to about 1000000
                   energy.  Calculate the dB difference from -5, and multiply
                   that with the known 1000000 energy to set the requested
                   threshold. */
                /* The lookup table covers mostly the positive dB range, and
                   provides factor*100 for dB*10. For example, the 3dB entry is
                   {30, 200} */
                dB = (pUserBuf->minDetect * -1) - 50;
                factor = VpTableInterpolate(VpDecibelTable, VP_DECIBEL_TABLE_SIZE, dB, FALSE);
                pParams->basicThreshold = VpRoundedDivide(1000000 * 100, factor);
                if (pParams->basicThreshold == 0) {
                    pParams->basicThreshold = 1;
                }
                VP_DTMF(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine - minDetect %ld -> basicThreshold %ld",
                    pUserBuf->minDetect, pParams->basicThreshold));
            }
            if (pUserBuf->validMask & VP_OPTION_CFG_ROW_TO_COL_LIMIT) {
                if (pUserBuf->rowToColLimit < 0 || pUserBuf->rowToColLimit > 700) {
                    VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine - rowToColLimit out of range (%ld)",
                        pUserBuf->rowToColLimit));
                    return VP_STATUS_INVALID_ARG;
                }
                pCache->dtmfParams.rowToColLimit = pUserBuf->rowToColLimit;
                factor = VpTableInterpolate(VpDecibelTable, VP_DECIBEL_TABLE_SIZE, pUserBuf->rowToColLimit, FALSE);
                pParams->normalTwistFactor =  VpRoundedDivide(factor, 10);
            }
            if (pUserBuf->validMask & VP_OPTION_CFG_COL_TO_ROW_LIMIT) {
                if (pUserBuf->colToRowLimit < 0 || pUserBuf->colToRowLimit > 700) {
                    VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine - colToRowLimit out of range (%ld)",
                        pUserBuf->colToRowLimit));
                    return VP_STATUS_INVALID_ARG;
                }
                pCache->dtmfParams.colToRowLimit = pUserBuf->colToRowLimit;
                factor = VpTableInterpolate(VpDecibelTable, VP_DECIBEL_TABLE_SIZE, pUserBuf->colToRowLimit, FALSE);
                pParams->reverseTwistFactor =  VpRoundedDivide(factor, 10);
            }
            break;
        }
#endif /* VP886_INCLUDE_DTMF_DETECT */
        case VP_OPTION_ID_RINGTRIP_CONFIRM: {
            uint16 *pUserBuf = pValue;
            pCache->ringTripConfirm = *pUserBuf;
            break;
        }
        case VP_OPTION_ID_HIGHPASS_FILTER: {
            VpOptionHighPassFilterType *pUserBuf = pValue;
            pCache->highPassFilter = *pUserBuf;
            if (pCache->highPassFilter == VP_HIGHPASS_FILTER_DISABLE) {
                pLineObj->registers.opCond[0] |= VP886_R_OPCOND_HIGHPASS_DIS;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, pLineObj->registers.opCond);
            } else {
                pLineObj->registers.opCond[0] &= ~VP886_R_OPCOND_HIGHPASS_DIS;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, pLineObj->registers.opCond);
            }
            break;
        }
#if (VP_CC_DEBUG_SELECT != 0)
        case VP_OPTION_ID_DEBUG_SELECT: {
            uint32 *pUserBuf = pValue;
            pLineObj->debugSelectMask = *pUserBuf;
            break;
        }
#endif /* (VP_CC_DEBUG_SELECT != 0) */
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_PULSE2:
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
        case VP_DEVICE_OPTION_ID_DEVICE_IO:
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING:
#endif
        case VP_DEVICE_OPTION_ID_RING_PHASE_SYNC:
        case VP_DEVICE_OPTION_ID_FSYNC_RATE:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() Device option (0x%02X) not valid at line level", option));
            status = VP_STATUS_INVALID_ARG;
            break;
        default:
            if ((pLineObj->busyFlags & VP886_LINE_INIT_CMP) &&
                (pDevObj->busyFlags & VP_DEV_INIT_CMP))
            {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() Line option 0x%02X not supported", option));
            }
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine-"));
    return status;
}


/** Vp886SetOptionEventMask()
  Applies the VP_OPTION_ID_EVENT_MASK option for a line.
*/
VpStatusType
Vp886SetOptionEventMask(
    VpLineCtxType *pLineCtx,
    VpOptionEventMaskType *pEventMask)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpOptionEventMaskType *pDevMask = &pDevObj->options.eventMask;
    VpOptionEventMaskType *pLineMask = &pLineObj->options.eventMask;

    /* Include only device-specific bits for the device object cache */
    pDevMask->faults    = pEventMask->faults    & VP_EVCAT_FAULT_DEV_EVENTS;
    pDevMask->signaling = pEventMask->signaling & VP_EVCAT_SIGNALING_DEV_EVENTS;
    pDevMask->response  = pEventMask->response  & VP_EVCAT_RESPONSE_DEV_EVENTS;
    pDevMask->test      = pEventMask->test      & VP_EVCAT_TEST_DEV_EVENTS;
    pDevMask->process   = pEventMask->process   & VP_EVCAT_PROCESS_DEV_EVENTS;
    pDevMask->fxo       = pEventMask->fxo       & VP_EVCAT_FXO_DEV_EVENTS;

    /* Zero out the device-specific bits for the line object cache */
    pLineMask->faults    = pEventMask->faults    & ~VP_EVCAT_FAULT_DEV_EVENTS;
    pLineMask->signaling = pEventMask->signaling & ~VP_EVCAT_SIGNALING_DEV_EVENTS;
    pLineMask->response  = pEventMask->response  & ~VP_EVCAT_RESPONSE_DEV_EVENTS;
    pLineMask->test      = pEventMask->test      & ~VP_EVCAT_TEST_DEV_EVENTS;
    pLineMask->process   = pEventMask->process   & ~VP_EVCAT_PROCESS_DEV_EVENTS;
    pLineMask->fxo       = pEventMask->fxo       & ~VP_EVCAT_FXO_DEV_EVENTS;

    /* Unmask the unmaskable defined in vp_api_common.c */
    VpImplementNonMaskEvents(pLineMask, pDevMask);

    /* Mask events not supported by vp886 */
    pDevMask->faults    |= VP886_NONSUPPORT_FAULT_EVENTS;
    pDevMask->signaling |= VP886_NONSUPPORT_SIGNALING_EVENTS;
    pDevMask->response  |= VP886_NONSUPPORT_RESPONSE_EVENTS;
    pDevMask->test      |= VP886_NONSUPPORT_TEST_EVENTS;
    pDevMask->process   |= VP886_NONSUPPORT_PROCESS_EVENTS;
    pDevMask->fxo       |= VP886_NONSUPPORT_FXO_EVENTS;

    pLineMask->faults    |= VP886_NONSUPPORT_FAULT_EVENTS;
    pLineMask->signaling |= VP886_NONSUPPORT_SIGNALING_EVENTS;
    pLineMask->response  |= VP886_NONSUPPORT_RESPONSE_EVENTS;
    pLineMask->test      |= VP886_NONSUPPORT_TEST_EVENTS;
    pLineMask->process   |= VP886_NONSUPPORT_PROCESS_EVENTS;
    pLineMask->fxo       |= VP886_NONSUPPORT_FXO_EVENTS;

    return VP_STATUS_SUCCESS;
}


/** Vp886SetOptionTimeslot()
  Applies the VP_OPTION_ID_TIMESLOT option for a line.
*/
VpStatusType
Vp886SetOptionTimeslot(
    VpLineCtxType *pLineCtx,
    VpOptionTimeslotType *pTimeslot)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 txSlot = pTimeslot->tx;
    uint8 rxSlot = pTimeslot->rx;
    uint8 maxTimeslot;
    bool wideband = FALSE;;

    /* Determine the maximum timeslot based on PCM clock rate and codec mode.
       Linear codec reduces the max by 1 because it uses an additional slot
       after the one specified.  Wideband reduces the max by half because it
       takes up the specified slot and a second slot in the second half of
       the frame */
    maxTimeslot = MIN(127, pDevObj->devProfileData.pcmClkRate / 64 - 1);
    switch(pLineObj->options.codec) {
        case VP_OPTION_ALAW:
        case VP_OPTION_MLAW:
            wideband = FALSE;
            break;
        case VP_OPTION_LINEAR:
            maxTimeslot -= 1;
            wideband = FALSE;
            break;
        case VP_OPTION_LINEAR_WIDEBAND:
            maxTimeslot /= 2;
            maxTimeslot -= 1;
            wideband = TRUE;
            break;
        case VP_OPTION_ALAW_WIDEBAND:
        case VP_OPTION_MLAW_WIDEBAND:
            maxTimeslot /= 2;
            wideband = TRUE;
            break;
        default:
            break;
    }

    /* Validate the tx time slot value */
    if(txSlot > maxTimeslot) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionTimeslot() - Bad Tx time slot value %d, max %d",
            txSlot, maxTimeslot));
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    /* Validate the rx time slot value */
    if(rxSlot > maxTimeslot) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionTimeslot() - Bad Rx time slot value %d, max %d",
            rxSlot, maxTimeslot));
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    /* ZSI adds 2 clocks of delay on the transmit side.  This means that in
       order to transmit in the requested slot, we need to shift backwards 2
       clocks.  The clock slot register only allows forward shifting, so we
       must shift back one whole slot (8 clocks), and the clock slot register
       will be set for +6 clocks on TX. */
    if (pDevObj->stateInt & VP886_ZSI_DETECTED) {
        if (txSlot == 0) {
            txSlot = MIN(127, pDevObj->devProfileData.pcmClkRate / 64 - 1);
            /* In wideband mode, we can't program the timeslot in the upper half
               of the frame.  So for example at 8MHz ZCLK, we will use 63
               instead of 127, and the device will put the other sample at 127
               for us.  Then those will be shifted to 64 and 0 by the ZSI and
               clock slot setting. */
            if (wideband) {
                txSlot /= 2;
            }
        } else {
            txSlot--;
        }
    }

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_TXSLOT_WRT, VP886_R_TXSLOT_LEN, &txSlot);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RXSLOT_WRT, VP886_R_RXSLOT_LEN, &rxSlot);

    return VP_STATUS_SUCCESS;
}


/** Vp886SetOptionCodec()
  Applies the VP_OPTION_ID_CODEC option for a line.
*/
VpStatusType
Vp886SetOptionCodec(
    VpLineCtxType *pLineCtx,
    VpOptionCodecType *pCodec)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    bool toWideband = FALSE;
    bool fromWideband = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp886SetOptionCodec()"));

    /* Basic error checking and set toWideband flag */
    switch (*pCodec) {
        case VP_OPTION_LINEAR:
        case VP_OPTION_ALAW:
        case VP_OPTION_MLAW:
            toWideband = FALSE;
            break;
        case VP_OPTION_LINEAR_WIDEBAND:
        case VP_OPTION_ALAW_WIDEBAND:
        case VP_OPTION_MLAW_WIDEBAND:
            toWideband = TRUE;
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionCodec() Invalid codec %d", *pCodec));
            return VP_STATUS_INVALID_ARG;
    }

    /* Determine if the current setting is wideband */
    switch (pLineObj->options.codec) {
        case VP_OPTION_LINEAR:
        case VP_OPTION_ALAW:
        case VP_OPTION_MLAW:
            fromWideband = FALSE;
            break;
        case VP_OPTION_LINEAR_WIDEBAND:
        case VP_OPTION_ALAW_WIDEBAND:
        case VP_OPTION_MLAW_WIDEBAND:
            fromWideband = TRUE;
            break;
        default:
            break;
    }

    if (toWideband && !fromWideband) {
        /* Switching to wideband */
        pLineObj->registers.ssCfg[0] |= VP886_R_SSCFG_WBAND;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
    } else if (fromWideband && !toWideband) {
        /* Switching out of wideband */
        pLineObj->registers.ssCfg[0] &= ~VP886_R_SSCFG_WBAND;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
    }

    /* If changing to or from wideband while in a codec activated state, we need
       to briefly disable the codec by going to the idle state, then restore
       it.  Otherwise the codec gets into a bad state with a large loss in the
       receive path, and possibly other issues. */
    if (toWideband != fromWideband &&
        Vp886LineStateInfo(pLineObj->lineState.currentState).codec)
    {
        uint8 sysState;
        sysState = pLineObj->registers.sysState[0];
        sysState &= ~VP886_R_STATE_SS;
        sysState &= ~VP886_R_STATE_CODEC;
        sysState |= VP886_R_STATE_SS_IDLE;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, &sysState);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);
    }

    /* Enable the desired CODEC mode */
    pLineObj->registers.opFunc[0] &= ~VP886_R_OPFUNC_CODEC;
    switch(*pCodec) {
        case VP_OPTION_LINEAR:
        case VP_OPTION_LINEAR_WIDEBAND:
            pLineObj->registers.opFunc[0] |= VP886_R_OPFUNC_CODEC_LINEAR;
            break;
        case VP_OPTION_ALAW_WIDEBAND:
        case VP_OPTION_ALAW:
            pLineObj->registers.opFunc[0] |= VP886_R_OPFUNC_CODEC_ALAW;
            break;
        case VP_OPTION_MLAW_WIDEBAND:
        case VP_OPTION_MLAW:
            pLineObj->registers.opFunc[0] |= VP886_R_OPFUNC_CODEC_ULAW;
            break;
        default:
            /* Cannot reach here.  Error checking at top */
            break;
    }
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPFUNC_WRT, VP886_R_OPFUNC_LEN, pLineObj->registers.opFunc);

    /* If we are using ZSI, the TX timeslot is set to 0, and we are entering or
       exiting wideband mode, we will need to adjust the programmed TX slot.
       See Vp886SetOptionTimeslot() for more details. */
    if ((pDevObj->stateInt & VP886_ZSI_DETECTED) &&
        pLineObj->options.timeslot.tx == 0 &&
        toWideband != fromWideband)
    {
        uint8 txSlot = MIN(127, pDevObj->devProfileData.pcmClkRate / 64 - 1);
        if (toWideband) {
            txSlot /= 2;
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_TXSLOT_WRT, VP886_R_TXSLOT_LEN, &txSlot);
    }

    return VP_STATUS_SUCCESS;
}


/** Vp886SetOptionIoCfg()
  Applies the VP_OPTION_ID_LINE_IO_CFG option for a line.  Also used to apply
  the VP_DEVICE_OPTION_ID_DEVICE_IO option, which is why we use just the
  channel ID and not a line context or object.  We use the same option cache
  value in the device object for both options to keep them consistent.
*/
VpStatusType
Vp886SetOptionIoCfg(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 direction,
    uint8 outputType,
    uint8 *pIoDirReg)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886DevOptionsCacheType *pCache = &pDevObj->options;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 pinMask;
    uint8 dirBit;
    uint8 typeBit;
    uint8 io2Use;

    if (pDevObj->ioCapability == VP886_IO_CAPABILITY_TWO_PER_CH) {
        pCache->deviceIo.directionPins_31_0 &= ~(0x3 << (channelId * 2));
        pCache->deviceIo.outputTypePins_31_0 &= ~(0x3 << (channelId * 2));

        /* For pin I/O 1 */

        /* Extract the direction and type bits */
        pinMask = 0x01;
        dirBit = direction & pinMask;
        typeBit = outputType & pinMask;

        /* Assume input to start off */
        *pIoDirReg = VP886_R_IODIR_IOD1_INPUT;

        /* If the direction bit is set ... */
        if (dirBit) {
            /* I/O 1 is an output pin */
            /* If the type bit is not set ... */
            if (!typeBit) {
                /* I/O 1 is a digital output */
                *pIoDirReg |= VP886_R_IODIR_IOD1_DIG_OUTPUT;
            } else {
                /* Otherwise, it is an open-drain output */
                *pIoDirReg |= VP886_R_IODIR_IOD1_OD_OUTPUT;
            }
        }
        pCache->deviceIo.directionPins_31_0 |= (dirBit << (channelId * 2));
        pCache->deviceIo.outputTypePins_31_0 |= (typeBit << (channelId * 2));

        /* For pin I/O 2 */

        /* Extract the direction and type bits */
        pinMask = 0x01 << 1;
        dirBit = direction & pinMask;
        typeBit = outputType & pinMask;

        /* Device profile I/O 2 usage */
        io2Use = pDevObj->devProfileData.io2Use >> (4 * channelId);
        io2Use &= VP886_DEV_PROFILE_IO2_USE_BITS;

        /* Assume input to start off. No need to modify ioDirReg yet */

        /* If the direction bit is set ... */
        if (dirBit) {
            /* I/O 2 is an output pin */
            /* If this is a digital interrupt pin or a voltage monitor mode pin,
               it cannot be configured as an output pin */
            if (io2Use != VP886_DEV_PROFILE_IO2_USE_DIG_SIG) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionIoCfg() ch%d I/O 2 reserved as an input", channelId));
                status = VP_STATUS_DEDICATED_PINS;
                dirBit = 0;
                typeBit = 0;
            } else {
                /* I/O 2 is a digital output */
                /* If the type bit is not set .. */
                if (!typeBit) {
                    /* OR in the bits */
                    *pIoDirReg |= VP886_R_IODIR_IOD2_OUTPUT;
                } else {
                    /* Otherwise .... invalid I/O 2 configuration */
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionIoCfg() ch%d I/O 2 cannot be configured as an open output", channelId));
                    return VP_STATUS_INVALID_ARG;
                }
            }
        }
        pCache->deviceIo.directionPins_31_0 |= (dirBit << (channelId * 2));
        pCache->deviceIo.outputTypePins_31_0 |= (typeBit << (channelId * 2));

        /* If I/O 2 is a voltage monitor mode pin ... */
        if (io2Use == VP886_DEV_PROFILE_IO2_USE_VMM) {
            *pIoDirReg |= VP886_R_IODIR_IOD2_VMON;
        }
    } else if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2) {
        if (channelId == 1) {
            return status;
        }
        io2Use = pDevObj->devProfileData.io2Use & VP886_DEV_PROFILE_IO2_USE_BITS;

        /* IO2 Input by default, IO1 always output */
        *pIoDirReg = VP886_R_IODIR_IOD1_DIG_OUTPUT;
        pCache->deviceIo.directionPins_31_0 = 0x0;

        /* If the direction bit is set ... */
        if (direction & 0x1) {
            /* I/O 2 is an output pin */
            if (outputType & 0x1) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionIoCfg() I/O cannot be configured as an open output"));
                return VP_STATUS_INVALID_ARG;
            }
            /* If this is a digital interrupt pin or a voltage monitor mode pin,
               it cannot be configured as an output pin */
            if (io2Use != VP886_DEV_PROFILE_IO2_USE_DIG_SIG) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886SetOptionIoCfg() I/O reserved as an input"));
                status = VP_STATUS_DEDICATED_PINS;
            } else {
                *pIoDirReg |= VP886_R_IODIR_IOD2_OUTPUT;
                pCache->deviceIo.directionPins_31_0 = 0x1;
            }
        }

        /* If I/O 2 is a voltage monitor mode pin ... */
        if (io2Use == VP886_DEV_PROFILE_IO2_USE_VMM) {
            *pIoDirReg |= VP886_R_IODIR_IOD2_VMON;
        }
    } else {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886SetOptionIoCfg() Unknown I/O capability type %d", pDevObj->ioCapability));
    }

    return status;
}


/** Vp886SetOptionDcFeedParams()
  Applies the VP_OPTION_ID_DCFEED_PARAMS option for a line.
*/
VpStatusType
Vp886SetOptionDcFeedParams(
    VpLineCtxType *pLineCtx,
    VpOptionDcFeedParamsType *pDcFeedParams)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    uint8 voc = 0;
    uint8 vocsft = 0;
    uint8 vocTarget = 0;
    uint8 lpBatt = 0;
    uint8 ila = 0;
    uint8 tsh = 0;
    uint8 lptsh = 0;
    uint8 tgk = 0;
    uint8 battFloor = 0;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp886SetOptionDcFeedParams()"));

    /* Perform all error checks and calculations BEFORE changing cached values
       or accessing the device to make this atomic. */

    if (pDcFeedParams->validMask == 0) {
        return VP_STATUS_SUCCESS;
    }

    if (pDcFeedParams->validMask == 0xFFFFFFFF) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - validMask should not be blindly set to 0xFFFFFFFF"));
        return VP_STATUS_INVALID_ARG;
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_VOC) {
        /* VOC is a 3-bit field with step size of 3 V and a range of either
           12-33 or 36-57 V depending on the VOCSFT bit.  Allow step size of 1V
           to be achieved via calibration. */
        if (pDcFeedParams->voc < VP886_VOC_MIN || pDcFeedParams->voc > VP886_VOC_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - voc out of range (%ld)",
                pDcFeedParams->voc));
            return VP_STATUS_INVALID_ARG;
        }
        voc = VpRoundedDivide(pDcFeedParams->voc - VP886_VOC_OFFSET, VP886_VOC_STEP);
        if (voc > 0x7) {
            voc -= 0x8;
            vocsft = 0;
        } else {
            vocsft = 1;
        }

        vocTarget = VpRoundedDivide(pDcFeedParams->voc, VP886_VOC_TARGET_STEP);

        if (VP886_IS_TRACKER(pDevObj)) {
            /* Set the low power switcher voltage to 4V above VOC, rounded to
               the nearest 5V step.  Calibration will apply the extra precision
               by using a target of VOC + 4. */
            lpBatt = VpRoundedDivide(vocTarget + 4, 5) - 1;
        }
    }


    if (pDcFeedParams->validMask & VP_OPTION_CFG_ILA) {
        /* ILA is a 5-bit value with a 18-49 mA scale (1 mA per step + 18 mA
           offset) */
        if (pDcFeedParams->ila < VP886_ILA_MIN || pDcFeedParams->ila > VP886_ILA_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - ila out of range (%ld)",
                pDcFeedParams->ila));
            return VP_STATUS_INVALID_ARG;
        }
        ila = VpRoundedDivide(pDcFeedParams->ila - VP886_ILA_OFFSET, VP886_ILA_STEP);
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_HOOK_THRESHOLD) {
        /* TSH is a 3-bit value with a 7-14 mA scale (1 mA per step + 7 mA
           offset) */
        if (pDcFeedParams->hookThreshold < VP886_TSH_MIN || pDcFeedParams->hookThreshold > VP886_TSH_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - hookThreshold out of range (%ld)",
                pDcFeedParams->hookThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        tsh = VpRoundedDivide(pDcFeedParams->hookThreshold - VP886_TSH_OFFSET, VP886_TSH_STEP);
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_LP_HOOK_THRESHOLD) {
        /* LPTSH is a 3-bit value with a 14-28 V scale (2 V per step + 14 V
           offset) */
        if (pDcFeedParams->lpHookThreshold < VP886_LPTSH_MIN || pDcFeedParams->lpHookThreshold > VP886_LPTSH_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - lpHookThreshold out of range (%ld)",
                pDcFeedParams->lpHookThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        lptsh = VpRoundedDivide(pDcFeedParams->lpHookThreshold - VP886_LPTSH_OFFSET, VP886_LPTSH_STEP);
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_GKEY_THRESHOLD) {
        /* TGK is a 3-bit value with a 0-42 mA scale (6 mA per step) */
        if (pDcFeedParams->gkeyThreshold < VP886_TGK_MIN || pDcFeedParams->gkeyThreshold > VP886_TGK_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - gkeyThreshold out of range (%ld)",
                pDcFeedParams->gkeyThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        tgk = VpRoundedDivide(pDcFeedParams->gkeyThreshold - VP886_TGK_OFFSET, VP886_TGK_STEP);
    }

    if (VP886_IS_TRACKER(pDevObj) &&
        (pDcFeedParams->validMask & VP_OPTION_CFG_BATT_FLOOR))
    {
        /* Allow battery floor voltage from -5 to -50 V in 1V steps */
        if (pDcFeedParams->battFloor < VP886_BATTFLR_MIN || pDcFeedParams->battFloor > VP886_BATTFLR_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionDcFeedParams - battFloor out of range (%ld)",
                pDcFeedParams->battFloor));
            return VP_STATUS_INVALID_ARG;
        }
        battFloor = VpRoundedDivide(pDcFeedParams->battFloor - VP886_BATTFLR_OFFSET, VP886_BATTFLR_STEP);
    }

    /* Program the DC feed parameters (VOC, ILA) */
    if ((pDcFeedParams->validMask & VP_OPTION_CFG_VOC) ||
        (pDcFeedParams->validMask & VP_OPTION_CFG_ILA))
    {
        if (pDcFeedParams->validMask & VP_OPTION_CFG_VOC) {
            pLineObj->registers.dcFeed[0] &= ~VP886_R_DCFEED_VOC;
            pLineObj->registers.dcFeed[0] |= ((voc << 2) & VP886_R_DCFEED_VOC);
            pLineObj->registers.dcFeed[0] &= ~VP886_R_DCFEED_VOCSHIFT;
            pLineObj->registers.dcFeed[0] |= ((vocsft << 5) & VP886_R_DCFEED_VOCSHIFT);
            pLineObj->targetVoc = vocTarget;
        }
        if (pDcFeedParams->validMask & VP_OPTION_CFG_ILA) {
            pLineObj->registers.dcFeed[1] &= ~VP886_R_DCFEED_ILA;
            pLineObj->registers.dcFeed[1] |= (ila & VP886_R_DCFEED_ILA);
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, pLineObj->registers.dcFeed);
    }

    /* Program the switching regulator params (floor and low power battery) */
    if (VP886_IS_TRACKER(pDevObj) &&
        ((pDcFeedParams->validMask & VP_OPTION_CFG_VOC) ||
         (pDcFeedParams->validMask & VP_OPTION_CFG_BATT_FLOOR)))
    {
        if (pDcFeedParams->validMask & VP_OPTION_CFG_BATT_FLOOR) {
            pLineObj->registers.swParam[0] &= ~VP886_R_SWPARAM_FLOOR_V;
            pLineObj->registers.swParam[0] |= VpRoundedDivide(battFloor - 5, 5);
            pLineObj->floorVoltage = battFloor;
        }
        if (pDcFeedParams->validMask & VP_OPTION_CFG_VOC) {
            pLineObj->registers.swParam[2] &= ~VP886_R_SWPARAM_LOWPOWER_V;
            pLineObj->registers.swParam[2] |= (lpBatt & VP886_R_SWPARAM_LOWPOWER_V);
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);
    }

    /* Program the loop supervision parameters (TSH, LPTSH, TGK) */
    if ((pDcFeedParams->validMask & VP_OPTION_CFG_HOOK_THRESHOLD) ||
        (pDcFeedParams->validMask & VP_OPTION_CFG_LP_HOOK_THRESHOLD) ||
        (pDcFeedParams->validMask & VP_OPTION_CFG_GKEY_THRESHOLD))
    {
        if (pDcFeedParams->validMask & VP_OPTION_CFG_HOOK_THRESHOLD) {
            pLineObj->registers.loopSup[0] &= ~VP886_R_LOOPSUP_HOOK_THRESH;
            pLineObj->registers.loopSup[0] |= (tsh & VP886_R_LOOPSUP_HOOK_THRESH);
        }
        if (pDcFeedParams->validMask & VP_OPTION_CFG_LP_HOOK_THRESHOLD) {
            pLineObj->registers.loopSup[3] &= ~VP886_R_LOOPSUP_LPM_HOOK_THRESH;
            pLineObj->registers.loopSup[3] |= ((lptsh << 5) & VP886_R_LOOPSUP_LPM_HOOK_THRESH);
        }
        if (pDcFeedParams->validMask & VP_OPTION_CFG_GKEY_THRESHOLD) {
            pLineObj->registers.loopSup[0] &= ~VP886_R_LOOPSUP_GKEY_THRESH;
            pLineObj->registers.loopSup[0] |= ((tgk << 3) & VP886_R_LOOPSUP_GKEY_THRESH);
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
    }

    if (pDevObj->calData[channelId].valid) {
        /* Since the target parameters have changed, we must recalculate and
           program calibration adjustments */
        Vp886ApplyCalDC(pLineCtx);
        Vp886ProgramCalRegisters(pLineCtx);
    }

    return VP_STATUS_SUCCESS;
}


/** Vp886SetOptionRingingParams()
  Applies the VP_OPTION_ID_RINGING_PARAMS option for a line.
*/
VpStatusType
Vp886SetOptionRingingParams(
    VpLineCtxType *pLineCtx,
    VpOptionRingingParamsType *pRingingParams)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    uint8 ringGen[VP886_R_RINGGEN_LEN];
    bool trapezoidal = FALSE;
    int16 amplitude = 0;
    int16 frequency = 0;
    int16 bias = 0;
    uint8 rtth = 0;
    uint8 ilr = 0;
    int16 riseTime = 0;
    bool setLowIlr = FALSE;
    bool setNormalIlr = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp886SetOptionRingingParams()"));

    /* Perform all error checks and calculations BEFORE changing cached values
       or accessing the device to make this atomic. */

    if (pRingingParams->validMask == 0) {
        return VP_STATUS_SUCCESS;
    }

#ifdef VP_CSLAC_SEQ_EN
    /* Stop any on-going msg wait pulse because it messes with the ringing
       register */
    if (pLineObj->sendSignal.active && pLineObj->sendSignal.type == VP_SENDSIG_MSG_WAIT_PULSE) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - New ring params force msg wait pulse to end"));
        Vp886SendSignalStop(pLineCtx, TRUE);
    }
#endif

    if ((pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY) ||
        (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) ||
        (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) ||
        (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME))
    {
        VpSlacRegRead(NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN, ringGen);
        if ((ringGen[0] & VP886_R_RINGGEN_WAVE) == VP886_R_RINGGEN_WAVE_SINE) {
            trapezoidal = FALSE;
        } else {
            trapezoidal = TRUE;
        }
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY) {
        if (!trapezoidal) {
            /* Frequency is a 15-bit value with a 0-12000 Hz range.  12000000 mHz
               per 0x8000 reduces to 46875 mHz per 0x80. */
            if (pRingingParams->frequency < VP886_FREQ_MIN || pRingingParams->frequency >= VP886_FREQ_MAX) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - frequency out of range (%ld)",
                    pRingingParams->frequency));
                return VP_STATUS_INVALID_ARG;
            }
            frequency = (int16)VpRoundedDivide(pRingingParams->frequency * VP886_FREQ_STEP_DEN, VP886_FREQ_STEP_NUM);
        } else {
            /* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
               This means the maximum is 8000 Hz (8000000 mHz), and the minimum
               is 8000Hz / 0x7FFF, or 244.1 mHz.  FRQB can be calculated by
               8000000mHz / frequency. */
            if (pRingingParams->frequency <= VP886_TRAPFREQ_MIN || pRingingParams->frequency > VP886_TRAPFREQ_MAX) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - frequency out of range (%ld)",
                    pRingingParams->frequency));
                return VP_STATUS_INVALID_ARG;
            }
            frequency = (int16)VpRoundedDivide(VP886_TRAPFREQ_MAX, pRingingParams->frequency);
        }
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) {
        /* Amplitude is a 16-bit value with a +/- 154.4V range.  This
           means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
        if (pRingingParams->amplitude < VP886_AMP_MIN || pRingingParams->amplitude >= VP886_AMP_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - amplitude out of range (%ld)",
                pRingingParams->amplitude));
            return VP_STATUS_INVALID_ARG;
        }
        amplitude = (int16)VpRoundedDivide(pRingingParams->amplitude * VP886_AMP_STEP_DEN, VP886_AMP_STEP_NUM);
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) {
        /* DC Bias is a 16-bit value with a +/- 154.4V range.  This
           means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
        if (pRingingParams->dcBias < VP886_BIAS_MIN || pRingingParams->dcBias > VP886_BIAS_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - dcBias out of range (%ld)",
                pRingingParams->dcBias));
            return VP_STATUS_INVALID_ARG;
        }
        bias = (int16)VpRoundedDivide(pRingingParams->dcBias * VP886_BIAS_STEP_DEN, VP886_BIAS_STEP_NUM);
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD) {
        /* Ring trip threshold is a 7-bit value with a 0-63.5 mA scale (0.5 mA
           per step). */
        if (pRingingParams->ringTripThreshold < VP886_RTTH_MIN || pRingingParams->ringTripThreshold > VP886_RTTH_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - ringTripThreshold out of range (%ld)",
                pRingingParams->ringTripThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        rtth = VpRoundedDivide(pRingingParams->ringTripThreshold - VP886_RTTH_OFFSET, VP886_RTTH_STEP);
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT) {
        /* Ring current limit can be specified in either of two overlapping ranges
           for a total range of 22-112 mA */
        if (pRingingParams->ringCurrentLimit < VP886_ILR_MIN || pRingingParams->ringCurrentLimit > VP886_ILR_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - ringCurrentLimit out of range (%ld)",
                pRingingParams->ringCurrentLimit));
            return VP_STATUS_INVALID_ARG;
        }
        if (pRingingParams->ringCurrentLimit > VP886_ILR_LOW_MAX) {
            /* Ring current limit is a 5-bit value with a 50-112 mA scale (2 mA per
               step + 50 mA offset). */
            ilr = VpRoundedDivide(pRingingParams->ringCurrentLimit - VP886_ILR_OFFSET, VP886_ILR_STEP);
            /* Set a flag to perform additional operations for possibly exiting
               the low ILR range after error checking has finished */
            setNormalIlr = TRUE;
        } else {
            /* Low range ring current limit is a 5-bit value with a 18-49 mA scale.
               But in order to work around an issue with how the device targets 7/8
               of the programmed ILR value, we're going to force a +4mA adjustment
               in the calibration register.  This effectively bumps up the offset
               in programming the ILR field (1 mA per step + 22 mA offset). */
            ilr = VpRoundedDivide(pRingingParams->ringCurrentLimit - VP886_ILR_LOW_OFFSET, VP886_ILR_LOW_STEP);
            /* Set a flag to perform additional operations supporting the low
               ILR range after error checking has finished */
            setLowIlr = TRUE;
        }
    }

    if (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME)) {
        /* Trapezoidal rise time is defined as 2.7307sec / FRQA. This means the
           maximum is 2.7307 sec (2730700 usec), and the minimum
           is 2.7307sec / 0x7FFF, or 83.3 usec.  FRQA can be calculated by
           2730700usec / trapRiseTime. */
        if (pRingingParams->trapRiseTime <= VP886_TRAPRISE_MIN || pRingingParams->trapRiseTime > VP886_TRAPRISE_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionRingingParams - trapRiseTime out of range (%ld)",
                pRingingParams->trapRiseTime));
            return VP_STATUS_INVALID_ARG;
        }
        riseTime = (int16)VpRoundedDivide(VP886_TRAPRISE_MAX, pRingingParams->trapRiseTime);
    }


    /* Program the ringing generator parameters (frequency, amplitude, bias) */
    if ((pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY) ||
        (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) ||
        (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) ||
        (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME)))
    {
        if (!trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY)) {
            /* FRQA for sine ringing */
            ringGen[3] = (frequency & 0x7F00) >> 8;
            ringGen[4] = (frequency & 0x00FF);
            pLineObj->ringFrequency = frequency;
        }
        if (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY)) {
            /* FRQB for trapezoidal ringing */
            ringGen[7] = (frequency & 0x7F00) >> 8;
            ringGen[8] = (frequency & 0x00FF);
            pLineObj->ringFrequency = frequency;
        }
        if (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) {
            ringGen[5] = (amplitude & 0xFF00) >> 8;
            ringGen[6] = (amplitude & 0x00FF);
            pLineObj->ringAmplitude = amplitude;
        }
        if (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) {
            ringGen[1] = (bias & 0xFF00) >> 8;
            ringGen[2] = (bias & 0x00FF);
            pLineObj->ringBias = bias;
        }
        if (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME)) {
            ringGen[3] = (riseTime & 0x7F00) >> 8;
            ringGen[4] = (riseTime & 0x00FF);
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, ringGen);
    }

    /* If the amplitude or bias is changing and we're doing fixed battery
       ringing on a tracker device, update the switcher ringing voltage */
    if (VP886_IS_TRACKER(pDevObj) &&
        ((pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) ||
         (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS)) &&
        (pLineObj->registers.swParam[0] & VP886_R_SWPARAM_RING_TRACKING) == VP886_R_SWPARAM_RING_TRACKING_DIS)
    {
        int32 amplitude_mV;
        int32 bias_mV;
        int32 battery_mV;
        uint8 swrv;

        /* Calculate the programmed amplitude and bias in mV.
           int16 range is +/- 154.4V (154400 mV per 0x8000, reduces to 4825 mV
           per 0x400). */
        amplitude_mV = VpRoundedDivide(pLineObj->ringAmplitude * VP886_AMP_STEP_NUM, VP886_AMP_STEP_DEN); 
        bias_mV = VpRoundedDivide(pLineObj->ringBias * VP886_BIAS_STEP_NUM, VP886_BIAS_STEP_DEN); 

        /* The battery should be AT LEAST 5V higher than the sum of the
           absolute values of bias and amplitude. */
        battery_mV = ABS(amplitude_mV) + ABS(bias_mV) + 5000;

        /* SWRV is a 5-bit value with a range of 5-160 V (step size of 5 V +
           5 V offset).  Round up. */
        /* Round to the nearest Volt */
        swrv = VpRoundedDivide(battery_mV, 1000);
        if (swrv > 160) {
            swrv = 160;
        }
        /* Disallow setting the switcher voltage below 60 so that it can not
           run into VOC when entering ringing. */
        if (swrv < 60) {
            swrv = 60;
        }
        swrv = (swrv - 5 + 4) / 5;

        pLineObj->registers.swParam[1] &= ~VP886_R_SWPARAM_RINGING_V;
        pLineObj->registers.swParam[1] |= (swrv & VP886_R_SWPARAM_RINGING_V);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, pLineObj->registers.swParam);
    }
    
    /* For ABS devices, calculate the battery level required to support
       the ringing signal */
    if (VP886_IS_ABS(pDevObj) &&
        ((pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) ||
         (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS)))
    {
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
        VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Vp886SetOptionRingingParams: absRingingPeak[%d] = %d",
            channelId, pDevObj->absRingingPeak[channelId]));

        /* Switcher settings may need to be updated to support new ringing
           params */
        Vp886ManageABSRingingBatt(pDevCtx, channelId == 0 ? FALSE : TRUE, channelId == 1 ? FALSE : TRUE);
    }

    /* Program the loop supervision parameters (RTTH, ILR) */
    if ((pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD) ||
        (pRingingParams->validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT))
    {
        if (pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD) {
            /* Save the requested RTTH value */
            pLineObj->rtth = rtth;
            /* If we're are currently using the low ILR range or we are swiching
               to the low ILR range, double RTTH as part of the workaround. */
            if (setLowIlr || (pLineObj->lowIlr && !setNormalIlr)) {
                rtth = MIN(rtth * 2, VP886_R_LOOPSUP_RTRIP_THRESH);
            }
            pLineObj->registers.loopSup[2] &= ~VP886_R_LOOPSUP_RTRIP_THRESH;
            pLineObj->registers.loopSup[2] |= (rtth & VP886_R_LOOPSUP_RTRIP_THRESH);
        }
        if (pRingingParams->validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT) {
            pLineObj->registers.loopSup[3] &= ~VP886_R_LOOPSUP_RING_CUR_LIM;
            pLineObj->registers.loopSup[3] |= (ilr & VP886_R_LOOPSUP_RING_CUR_LIM);
        }
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
    }

    /* If ILR was specified in the low range, apply the workaround */
    if (setLowIlr && !pLineObj->lowIlr) {
        /* Set a flag to remember that we're using the low ILR range */
        pLineObj->lowIlr = TRUE;
        /* Force ILR to be interpreted at DC feed levels to allow ILR
           below 50mA */
        pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
        pLineObj->registers.icr2[1] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);

        /* If RTTH wasn't handled above, double it here */
        if (!(pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD)) {
            rtth = MIN(pLineObj->rtth * 2, VP886_R_LOOPSUP_RTRIP_THRESH);
            pLineObj->registers.loopSup[2] &= ~VP886_R_LOOPSUP_RTRIP_THRESH;
            pLineObj->registers.loopSup[2] |= (rtth & VP886_R_LOOPSUP_RTRIP_THRESH);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
        }
    }

    /* If ILR was previously set to the low range, and it has now been changed
       to the normal range, undo the workaround */
    if (setNormalIlr && pLineObj->lowIlr) {
        pLineObj->lowIlr = FALSE;
        pLineObj->registers.icr2[0] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);

        /* If RTTH wasn't changed above, set it back to normal here */
        if (!(pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD)) {
            pLineObj->registers.loopSup[2] &= ~VP886_R_LOOPSUP_RTRIP_THRESH;
            pLineObj->registers.loopSup[2] |= (pLineObj->rtth & VP886_R_LOOPSUP_RTRIP_THRESH);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
        }
    }

    if (pDevObj->calData[channelId].valid) {
        /* Since the target parameters have changed, we must recalculate and
           program calibration adjustments */
        Vp886ApplyCalRing(pLineCtx);
        Vp886ProgramCalRegisters(pLineCtx);
    }

    return VP_STATUS_SUCCESS;
}


#ifdef VP886_INCLUDE_DTMF_DETECT
/** Vp886SetOptionDtmfMode()
  Applies the VP_OPTION_ID_DTMF_MODE option for a line.
*/
VpStatusType
Vp886SetOptionDtmfMode(
    VpLineCtxType *pLineCtx,
    VpOptionDtmfModeType *pDtmfMode)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDtmfDetectParamsType *pParams = &pLineObj->dtmf.detectData.params;

    switch (pDtmfMode->dtmfControlMode) {
        case VP_OPTION_DTMF_DECODE_OFF:
            pLineObj->options.dtmfMode = *pDtmfMode;
            Vp886DtmfManage(pLineCtx);
            break;
        case VP_OPTION_DTMF_DECODE_ON: {
            pLineObj->dtmf.sampling = FALSE;
            pLineObj->dtmf.overflow = FALSE;
            pLineObj->dtmf.currentDigit = VP_DIG_NONE;

            if (VP886_IS_SF(pDevObj) && pDtmfMode->direction == VP_DIRECTION_US) {
                pLineObj->dtmf.useVadc = TRUE;
            } else {
                pLineObj->dtmf.useVadc = FALSE;
            }

            #if defined(VP886_DTMF_DETECT_4KHZ_SAMPLING)
                pParams->sampleRate = VP_DTMF_SAMPLE_RATE_4KHZ;
            #elif defined(VP886_DTMF_DETECT_8KHZ_SAMPLING) /* 8kHz */
                pParams->sampleRate = VP_DTMF_SAMPLE_RATE_8KHZ;
            #endif

            /* Minimum detect threshold and twist factors are set by the
               DTMF_PARAMS option.  Set other thresholds here */
            pParams->relPeakRowFactor = 63;     /* 6.3, 8dB, divide result by 10 */
            pParams->relPeakColFactor = 63;     /* 6.3, 8dB, divide result by 10 */
            pParams->secondHarmRowFactor = 25;  /* 2.5, 4dB, divide result by 10 */
            pParams->secondHarmColFactor = 631; /* 63.1, 18dB, divide result by 10 */
            if (pLineObj->dtmf.useVadc) {
                pParams->totalEnergyFactor = 370;   /* 37.0, divide result by 10 */
            } else {
                pParams->totalEnergyFactor = 280;   /* 28.0, divide result by 10 */
            }

            /* Adjust the samples so that we get about the same final results
               from both VADC and SADC.  1000 = 1.0 multiplier */
            if (pLineObj->dtmf.useVadc) {
                pParams->normalizeGain = 461;
            } else {
                pParams->normalizeGain = 1000;
            }

            /* These gain tables are used in DTMF detection to compensate for the frequency
               response of the SADC decimator.
               The SADC freq response is sinc(pi*f/Fs)^3
               To compensate, we apply the inverse at each frequency.  The factors saved
               here are also squared, since they will be applied to a squared magnitude
               result.  Each factor is normalized around 1000, so a value of 1354 means
               a multiplier of 1.354.
               Formula used: 1000/sinc(pi*f/Fs)^6
               */
            if (pLineObj->dtmf.useVadc) {
                if (pParams->sampleRate == VP_DTMF_SAMPLE_RATE_4KHZ) {
                    /* 4kHz VADC */
                    pParams->gainRow[0] = 1000;     /* 697 Hz */
                    pParams->gainRow[1] = 1000;     /* 770 Hz */
                    pParams->gainRow[2] = 1000;     /* 852 Hz */
                    pParams->gainRow[3] = 1000;     /* 941 Hz */
                    pParams->gainCol[0] = 1000;     /* 1209 Hz */
                    pParams->gainCol[1] = 1000;     /* 1336 Hz */
                    pParams->gainCol[2] = 1000;     /* 1477 Hz */
                    pParams->gainCol[3] = 1000;     /* 1633 Hz */
                    pParams->gain2ndRow[0] = 1000;  /* 1394 Hz */
                    pParams->gain2ndRow[1] = 1000;  /* 1540 Hz */
                    pParams->gain2ndRow[2] = 1000;  /* 1704 Hz */
                    pParams->gain2ndRow[3] = 1000;  /* 1882 Hz */
                    pParams->gain2ndCol[0] = 0;     /* 2418 Hz */
                    pParams->gain2ndCol[1] = 0;     /* 2672 Hz */
                    pParams->gain2ndCol[2] = 0;     /* 2954 Hz */
                    pParams->gain2ndCol[3] = 0;     /* 3266 Hz */
                    pParams->gainLowGap = 1000;     /* 615 Hz */
                    pParams->gainMidGap = 1000;     /* 2076 Hz */
                    pParams->gainHighGap = 1000;    /* 1820 Hz */
                } else {
                    /* 8kHz VADC */
                    pParams->gainRow[0] = 1000;     /* 697 Hz */
                    pParams->gainRow[1] = 1000;     /* 770 Hz */
                    pParams->gainRow[2] = 1000;     /* 852 Hz */
                    pParams->gainRow[3] = 1000;     /* 941 Hz */
                    pParams->gainCol[0] = 1000;     /* 1209 Hz */
                    pParams->gainCol[1] = 1000;     /* 1336 Hz */
                    pParams->gainCol[2] = 1000;     /* 1477 Hz */
                    pParams->gainCol[3] = 1000;     /* 1633 Hz */
                    pParams->gain2ndRow[0] = 1000;  /* 1394 Hz */
                    pParams->gain2ndRow[1] = 1000;  /* 1540 Hz */
                    pParams->gain2ndRow[2] = 1000;  /* 1704 Hz */
                    pParams->gain2ndRow[3] = 1000;  /* 1882 Hz */
                    pParams->gain2ndCol[0] = 1000;  /* 2418 Hz */
                    pParams->gain2ndCol[1] = 1000;  /* 2672 Hz */
                    pParams->gain2ndCol[2] = 1000;  /* 2954 Hz */
                    pParams->gain2ndCol[3] = 1000;  /* 3266 Hz */
                    pParams->gainLowGap = 1000;     /* 615 Hz */
                    pParams->gainMidGap = 1000;     /* 2076 Hz */
                    pParams->gainHighGap = 1000;    /* 1820 Hz */
                }
            } else {
                if (pParams->sampleRate == VP_DTMF_SAMPLE_RATE_4KHZ) {
                    /* 4kHz SADC */
                    pParams->gainRow[0] = 1354;     /* 697 Hz */
                    pParams->gainRow[1] = 1448;     /* 770 Hz */
                    pParams->gainRow[2] = 1576;     /* 852 Hz */
                    pParams->gainRow[3] = 1745;     /* 941 Hz */
                    pParams->gainCol[0] = 2536;     /* 1209 Hz */
                    pParams->gainCol[1] = 3141;     /* 1336 Hz */
                    pParams->gainCol[2] = 4103;     /* 1477 Hz */
                    pParams->gainCol[3] = 5731;     /* 1633 Hz */
                    pParams->gain2ndRow[0] = 3492;  /* 1394 Hz */
                    pParams->gain2ndRow[1] = 4673;  /* 1540 Hz */
                    pParams->gain2ndRow[2] = 6767;  /* 1704 Hz */
                    pParams->gain2ndRow[3] = 10702; /* 1882 Hz */
                    pParams->gain2ndCol[0] = 0;     /* 2418 Hz */
                    pParams->gain2ndCol[1] = 0;     /* 2672 Hz */
                    pParams->gain2ndCol[2] = 0;     /* 2954 Hz */
                    pParams->gain2ndCol[3] = 0;     /* 3266 Hz */
                    pParams->gainLowGap = 1265;     /* 615 Hz */
                    pParams->gainMidGap = 2076;     /* 2076 Hz */
                    pParams->gainHighGap = 9059;    /* 1820 Hz */
                } else {
                    /* 8kHz SADC */
                    pParams->gainRow[0] = 1078;     /* 697 Hz */
                    pParams->gainRow[1] = 1096;     /* 770 Hz */
                    pParams->gainRow[2] = 1119;     /* 852 Hz */
                    pParams->gainRow[3] = 1147;     /* 941 Hz */
                    pParams->gainCol[0] = 1255;     /* 1209 Hz */
                    pParams->gainCol[1] = 1320;     /* 1336 Hz */
                    pParams->gainCol[2] = 1405;     /* 1477 Hz */
                    pParams->gainCol[3] = 1517;     /* 1633 Hz */
                    pParams->gain2ndRow[0] = 1354;  /* 1394 Hz */
                    pParams->gain2ndRow[1] = 1448;  /* 1540 Hz */
                    pParams->gain2ndRow[2] = 1576;  /* 1704 Hz */
                    pParams->gain2ndRow[3] = 1745;  /* 1882 Hz */
                    pParams->gain2ndCol[0] = 2536;  /* 2418 Hz */
                    pParams->gain2ndCol[1] = 3141;  /* 2672 Hz */
                    pParams->gain2ndCol[2] = 4103;  /* 2954 Hz */
                    pParams->gain2ndCol[3] = 5731;  /* 3266 Hz */
                    pParams->gainLowGap = 1060;     /* 615 Hz */
                    pParams->gainMidGap = 1196;     /* 2076 Hz */
                    pParams->gainHighGap = 1682;    /* 1820 Hz */
                }
            }

            VpDtmfDetectInit(pLineCtx, &pLineObj->dtmf.detectData);

            pLineObj->options.dtmfMode = *pDtmfMode;

            /* Begin sampling, if in an applicable state. */
            Vp886DtmfManage(pLineCtx);

            break;
        }
        case VP_OPTION_DTMF_GET_STATUS:
            /* This feature isn't needed for this device, since we can perform
               detection on every line. */
            /* Fallthrough */
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine() - Invalid DTMF Control Mode Type %d", pDtmfMode->dtmfControlMode));
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886SetOptionLine-"));
            return VP_STATUS_INVALID_ARG;
    }
    return VP_STATUS_SUCCESS;
}
#endif /* VP886_INCLUDE_DTMF_DETECT */


/** Vp886SetRelayState()
  Implements VpSetRelayState() to provide access to relays controlled by the
  device.

  See the VP-API-II Reference Guide for more details on VpSetRelayState().
*/
VpStatusType
Vp886SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState)
{
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886SetRelayState");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetRelayState");
        return status;
    }

    status = Vp886SetRelayStateInt(pLineCtx, rState);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886SetRelayState");
    return status;
}


/** Vp886SetRelayStateInt()
  Internally accessible implementation of Vp886SetRelayState().
*/
VpStatusType
Vp886SetRelayStateInt(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (rState) {
        case VP_RELAY_NORMAL: {
            Vp886RemoveInternalTestTerm(pLineCtx);
            break;
        }
        case VP_RELAY_BRIDGED_TEST: {
            Vp886ApplyInternalTestTerm(pLineCtx);
            break;
        }
        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

#ifdef VP886_INCLUDE_TESTLINE_CODE
    if (status == VP_STATUS_SUCCESS) {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        pLineObj->relayState = rState;
    }
#endif

    return status;
}


/** Vp886ApplyInternalTestTerm()
  Applies the internal test termination initial register settings. This provides
  an approximation of the VP_RELAY_BRIDGED_TEST relay state by creating an
  internal tip-ring short that will trigger an offhook detection.

  The ICR1 settings applied here are temporary, will be changed after a delay.
*/
void
Vp886ApplyInternalTestTerm(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    if (pLineObj->intTestTermApplied) {
        return;
    }

    /* Disconnect VAB sensing */
    pLineObj->registers.calCtrl[2] &= ~VP886_R_CALCTRL_RING_INP_SEL;
    pLineObj->registers.calCtrl[2] |= VP886_R_CALCTRL_RING_INP_SEL_DISC;
    pLineObj->registers.calCtrl[2] &= ~VP886_R_CALCTRL_TIP_INP_SEL;
    pLineObj->registers.calCtrl[2] |= VP886_R_CALCTRL_TIP_INP_SEL_DISC;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, pLineObj->registers.calCtrl);

    /* Reverse the polarity of the ground key detector to disable ground
       key event */
    pLineObj->registers.icr4[2] |= VP886_R_ICR4_GKEY_POLR;
    pLineObj->registers.icr4[3] |= VP886_R_ICR4_GKEY_POLR_NEG;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);

    /* The above change to the ground key detector polarity won't do anything
       if it's set to absolute mode (DC fault reporting).  Temporarily force it
       to normal mode to avoid getting DC_FLT events. */
    if (pLineObj->reportDcFaults) {
        pLineObj->registers.loopSup[0] &= ~VP886_R_LOOPSUP_GKEY_ABS;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
    }

    /* Forcing the SLIC DC bias for 200ms helps collapse the battery
       voltage, especially for fixed tracking designs. */
    pLineObj->registers.icr1[0] = 0xFF;
    pLineObj->registers.icr1[1] = 0xFF;
    pLineObj->registers.icr1[2] = 0x3F;
    pLineObj->registers.icr1[3] = 0x0F;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, pLineObj->registers.icr1);

    /* Start a timer to change the ICR1 settings later to make tip and ring
       outputs high impedance so that they tend to pull to battery. */
    Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_INT_TEST_TERM, VP886_INT_TEST_TERM_DELAY, 0, 0);

    pLineObj->intTestTermApplied = TRUE;
} /* Vp886ApplyInternalTestTerm() */


/** Vp886RemoveInternalTestTerm()
  Reverses the internal test termination initial register settings to restore
  normal operation.
*/
void
Vp886RemoveInternalTestTerm(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    if (!pLineObj->intTestTermApplied) {
        return;
    }

    /* Restore VAB sensing */
    pLineObj->registers.calCtrl[2] &= ~VP886_R_CALCTRL_RING_INP_SEL;
    pLineObj->registers.calCtrl[2] |= VP886_R_CALCTRL_RING_INP_SEL_CONN;
    pLineObj->registers.calCtrl[2] &= ~VP886_R_CALCTRL_TIP_INP_SEL;
    pLineObj->registers.calCtrl[2] |= VP886_R_CALCTRL_TIP_INP_SEL_CONN;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, pLineObj->registers.calCtrl);

    /* Restore ground key polarity setting */
    pLineObj->registers.icr4[2] &= ~VP886_R_ICR4_GKEY_POLR;
    pLineObj->registers.icr4[3] &= ~VP886_R_ICR4_GKEY_POLR;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);

    /* Restore the original ground key detector mode if necessary. */
    if (pLineObj->reportDcFaults) {
        pLineObj->registers.loopSup[0] |= VP886_R_LOOPSUP_GKEY_ABS;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pLineObj->registers.loopSup);
    }

    /* Restore ICR1.  TODO: this may need to be something other than all 0 */
    pLineObj->registers.icr1[0] = 0x00;
    pLineObj->registers.icr1[1] = 0x00;
    pLineObj->registers.icr1[2] = 0x00;
    pLineObj->registers.icr1[3] = 0x00;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, pLineObj->registers.icr1);

    Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_INT_TEST_TERM, 0, FALSE);

    pLineObj->intTestTermApplied = FALSE;
} /* Vp886RemoveInternalTestTerm() */


/** Vp886DeviceIoAccess()
  Implements VpDeviceIoAccess() to provide access to the input/output pins of
  the device.

  See the VP-API-II Reference Guide for more details on VpDeviceIoAccess().
*/
VpStatusType
Vp886DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886DeviceIoAccess");

    /* Get out if device is not ready */
    if (!Vp886ReadyStatus(pDevCtx, VP_NULL, &status)) {
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886DeviceIoAccess");
        return status;
    }

    /* Some devices do not support I/O lines */
    if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886DeviceIoAccess() - Device does not support I/O lines"));
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886DeviceIoAccess");
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    }

    /* Get out if there are still results that need to be read from a previous event .. */
    if (pDevObj->getResultsRequired && pDeviceIoData->accessType == VP_DEVICE_IO_READ) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886DeviceIoAccess() - Waiting to clear previous read"));
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886DeviceIoAccess");
        return VP_STATUS_DEVICE_BUSY;
    }

    status = Vp886DeviceIoAccessInt(pDevCtx, pDeviceIoData);

    if (status == VP_STATUS_SUCCESS || status == VP_STATUS_DEDICATED_PINS) {
        /* Generate the i/o access complete event */
        pDevObj->getResultsRequired = (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE) ? FALSE : TRUE;
        Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_RESPONSE,
            VP_DEV_EVID_IO_ACCESS_CMP, pDeviceIoData->accessType, Vp886GetTimestamp(pDevCtx), pDevObj->getResultsRequired);
    }

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886DeviceIoAccess");
    return status;
}


/** Vp886DeviceIoAccessInt()
  Internally accessible implementation of Vp886DeviceIoAccess().  This is used
  by both Vp886DeviceIoAccess() and Vp886LineIoAccess().
*/
VpStatusType
Vp886DeviceIoAccessInt(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint32 accessMask = pDeviceIoData->accessMask_31_0;

    bool isDedicatedPins = FALSE;

    /* Get a pointer to the return data structure used to store the results for
       VpGetResults() */
    VpDeviceIoAccessDataType *pAccessData =
        &(pDevObj->basicResults.deviceIoAccess);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("+Vp886DeviceIoAccessInt()"));

    /* If the operation is a WRITE, remove from the access mask any IO bits
       that are configured as inputs */
    if (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE) {
        accessMask &= pDevObj->options.deviceIo.directionPins_31_0;
    }

    if (pDevObj->ioCapability == VP886_IO_CAPABILITY_TWO_PER_CH) {
        uint8 chanNum;
        uint8 ioDataReg[2] = {0x00, 0x00};  /* IO Status from each channel */

        /* tempIoData and tempIoMask are representations of the device content to be
           written. */
        uint8 tempIoData[2] = {0x00, 0x00};
        uint8 tempIoMask[2] = {0x00, 0x00};

        /* Extract the mask and data bits for each channel */
        for (chanNum = 0; chanNum < pDevObj->staticInfo.maxChannels; chanNum++) {
            uint16 dataMask;
            uint8 loopCnt;
            uint16 tempData;
            for (loopCnt = 0; loopCnt < 2; loopCnt++) {
                dataMask = 0x01;
                dataMask = (dataMask << (2 * chanNum + loopCnt));

                tempData = 0;
                tempData = (uint16)(accessMask & dataMask);
                tempIoMask[chanNum] |= (uint8)(tempData >> (2 * chanNum));

                tempData = 0;
                tempData = (uint16)(pDeviceIoData->deviceIOData_31_0 & dataMask);

                tempIoData[chanNum] |= (uint8)(tempData >> (2 * chanNum));
            }
        }

        /* Read the current state of the IO lines */
        for (chanNum = 0; chanNum < pDevObj->staticInfo.maxChannels; chanNum++) {
            /* Read the IO Data, whether a line exists or not */
            pDevObj->ecVal = (chanNum == 0 ? VP886_EC_1 : VP886_EC_2);
            pDevObj->dontFlushSlacBufOnRead = TRUE;
            VpSlacRegRead(pDevCtx, NULL, VP886_R_IODATA_RD, VP886_R_IODATA_LEN, &ioDataReg[chanNum]);
            pDevObj->ecVal = VP886_EC_GLOBAL;
        }

        /* Copy the input device I/O access data structure over to the output */
        *pAccessData = *pDeviceIoData;

        /* If this is an I/O pin write ... */
        if (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE) {
            /* For each channel, determine what needs to be written to the I/O data reg */
            for (chanNum = 0; chanNum < pDevObj->staticInfo.maxChannels; chanNum++) {
                uint8 tempData = ioDataReg[chanNum];

                tempData &= ~tempIoMask[chanNum];
                tempData |= (tempIoMask[chanNum] & tempIoData[chanNum]);

                VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Vp886DeviceIoAccessInt: Write IODATA 0x%02X on Ch %d",
                    tempData, chanNum));

                ioDataReg[chanNum] = tempData;
            }

            /* Then write to the device */
            for (chanNum = 0; chanNum < pDevObj->staticInfo.maxChannels; chanNum++) {
                pDevObj->ecVal = (chanNum == 0 ? VP886_EC_1 : VP886_EC_2);
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_IODATA_WRT, VP886_R_IODATA_LEN, &ioDataReg[chanNum]);
                pDevObj->ecVal = VP886_EC_GLOBAL;
            }

        } else {    /* VP_DEVICE_IO_READ */

            pAccessData->deviceIOData_31_0 = 0;
            pAccessData->deviceIOData_63_32 = 0;

            /* Extract the read data from the device and populate the return struct */
            for (chanNum = 0; chanNum < pDevObj->staticInfo.maxChannels; chanNum++) {
                uint8 loopCnt;
                uint32 tempIoRdData;
                uint16 dataMask;

                for (loopCnt = 0; loopCnt < 2; loopCnt++) {
                    dataMask = 0x01;
                    dataMask = (dataMask << loopCnt);

                    /* Extract the bit we're after in this loop */
                    tempIoRdData = ioDataReg[chanNum];

                    /* This is the location per the device. Move to API location */
                    tempIoRdData &= dataMask;
                    tempIoRdData = (tempIoRdData << (2 * chanNum));

                    /* OR into the result data field */
                    pAccessData->deviceIOData_31_0 |= tempIoRdData;
                }
            }

            /* Apply the mask to keep only the bits asked for */
            pAccessData->deviceIOData_31_0 &= (uint16)accessMask;
        }
    } else if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2) {
        uint8 ioDataReg;
        if (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE) {
            if (!(accessMask & 0x1)) {
                /* Nothing to write */
                VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp886DeviceIoAccessInt()"));
                return VP_STATUS_SUCCESS;
            }
            if (pDeviceIoData->deviceIOData_31_0 & 0x1) {
                ioDataReg = VP886_R_IODATA_IO2;
            } else {
                ioDataReg = 0x00;
            }
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Vp886DeviceIoAccessInt: Write IODATA 0x%02X on Ch0",
                ioDataReg));
            pDevObj->ecVal = VP886_EC_1;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_IODATA_WRT, VP886_R_IODATA_LEN, &ioDataReg);
            pDevObj->ecVal = VP886_EC_GLOBAL;

        } else {    /* VP_DEVICE_IO_READ */
            pAccessData->deviceIOData_63_32 = 0;
            if (!(accessMask & 0x1)) {
                /* Nothing to read */
                pAccessData->deviceIOData_31_0 = 0;
                VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp886DeviceIoAccessInt()"));
                return VP_STATUS_SUCCESS;
            }
            pDevObj->ecVal = VP886_EC_1;
            pDevObj->dontFlushSlacBufOnRead = TRUE;
            VpSlacRegRead(pDevCtx, NULL, VP886_R_IODATA_RD, VP886_R_IODATA_LEN, &ioDataReg);
            pDevObj->ecVal = VP886_EC_GLOBAL;
            
            if (ioDataReg & VP886_R_IODATA_IO2) {
                pAccessData->deviceIOData_31_0 = 0x1;
            }
        }
    } else {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886DeviceIoAccessInt() Unknown I/O capability type %d", pDevObj->ioCapability));
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp886DeviceIoAccessInt()"));

    /* TODO: Where/how does/should isDedicatedPins get set ? */
    return ((isDedicatedPins == TRUE) ? VP_STATUS_DEDICATED_PINS : VP_STATUS_SUCCESS);
} /* Vp886DeviceIoAccessInt */


/**  Vp886LineIoAccess()
  Implements VpLineIoAccess() to access device IO pins on a per-line basis.
  This provides an alternate interface to the sometimes awkward function
  VpDeviceIoAccess().

  This function is implemented by mapping the line IO access input arguments
  to the inputs used by VpDeviceIoAccess(), then calling the internal
  Vp886DeviceIoAccessInt() to do the work.

  See the VP-API-II Reference Guide for more details on VpLineIoAccess().
*/
VpStatusType
Vp886LineIoAccess(
    VpLineCtxType *pLineCtx,
    VpLineIoAccessType *pLineIoAccess,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpDeviceIoAccessDataType deviceIoData;
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886LineIoAccess");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886LineIoAccess");
        return status;
    }

    /* Some devices do not support I/O lines */
    if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886LineIoAccess() - Device does not support I/O lines"));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886LineIoAccess");
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    }
    if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2 && channelId == 1) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886LineIoAccess() - This channel does not have I/O lines"));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886LineIoAccess");
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    }

    /* Get out if there are still results that need to be read from a previous event .. */
    if (pDevObj->getResultsRequired && pLineIoAccess->direction == VP_IO_READ) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886LineIoAccess() - Waiting to clear previous read"));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886LineIoAccess");
        return VP_STATUS_DEVICE_BUSY;
    }

    /* Map the line IO access to an internal VpDeviceIoAccess() call */
    if (pLineIoAccess->direction == VP_IO_WRITE) {
        deviceIoData.accessType = VP_DEVICE_IO_WRITE;
    } else {
        deviceIoData.accessType = VP_DEVICE_IO_READ;
    }
    /* Each channel has two bits in the device IO access structure. This may
       be different for future revisions of the device (if there are more or
       less IOs per channel). */
    if (pDevObj->ioCapability == VP886_IO_CAPABILITY_TWO_PER_CH) {
    if (channelId == 0) {
        deviceIoData.accessMask_31_0 = (pLineIoAccess->ioBits.mask & 0x3);
        deviceIoData.deviceIOData_31_0 = (pLineIoAccess->ioBits.data & 0x3);
    } else {
        deviceIoData.accessMask_31_0 = (pLineIoAccess->ioBits.mask & 0x3) << 2;
        deviceIoData.deviceIOData_31_0 = (pLineIoAccess->ioBits.data & 0x3) << 2;
        }
    } else if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2) {
        deviceIoData.accessMask_31_0 = (pLineIoAccess->ioBits.mask & 0x1);
        deviceIoData.deviceIOData_31_0 = (pLineIoAccess->ioBits.data & 0x1);
    }
    deviceIoData.accessMask_63_32 = 0;
    deviceIoData.deviceIOData_63_32 = 0;
    status = Vp886DeviceIoAccessInt(pDevCtx, &deviceIoData);

    if (status == VP_STATUS_SUCCESS || status == VP_STATUS_DEDICATED_PINS) {
        /* Generate the i/o access complete event */
        if (pLineIoAccess->direction == VP_IO_WRITE) {
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE,
                VP_LINE_EVID_LINE_IO_WR_CMP, 0, handle, FALSE);
        } else {
            pDevObj->getResultsRequired = TRUE;
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE,
                VP_LINE_EVID_LINE_IO_RD_CMP, 0, handle, TRUE);
        }
    }

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886LineIoAccess");
    return status;
}


/**  Vp886DtmfDigitDetected()
  Implements VpDtmfDigitDetected(). This is used primarily for the application
  to inform the caller ID state machine that an ACK signal was detected.

  The API will also generate a VP_LINE_EVID_DTMF_DIG event to provide
  compatibility with other device families supported by VP-API-II that can
  perform DTMF digit detection.

  See the VP-API-II Reference Guide for more details on VpDtmfDigitDetected().
*/
VpStatusType
Vp886DtmfDigitDetected(
    VpLineCtxType *pLineCtx,
    VpDigitType digit,
    VpDigitSenseType sense)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 eventData;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886DtmfDigitDetected");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886DtmfDigitDetected");
        return status;
    }

    if (digit > 0x0F) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886DtmfDigitDetected - Invalid digit 0x%02X", digit));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886DtmfDigitDetected");
        return VP_STATUS_INVALID_ARG;
    }

    switch (sense) {
        case VP_DIG_SENSE_BREAK:
        case VP_DIG_SENSE_MAKE:
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886DtmfDigitDetected - Invalid sense value %d", sense));
            Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886DtmfDigitDetected");
            return VP_STATUS_INVALID_ARG;
    }

#ifdef VP_CSLAC_SEQ_EN
    if (pLineObj->cid.active) {
        /* During CID, use this information to validate ACK detect intervals. */
        Vp886CidAckDetect(pLineCtx, digit, sense);
    }
#endif /* VP_CSLAC_SEQ_EN */

    /* Generate a DTMF_DIG event */
    eventData = (digit & 0x000F) | (sense & 0x0010);
    Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_DTMF_DIG,
        eventData, Vp886GetTimestamp(pDevCtx), FALSE);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886DtmfDigitDetected");
    return status;
}


#if (VP886_USER_TIMERS > 0)
/**  Vp886GenTimerCtrl()
  Implements VpGenTimerCtrl() to allow an application to utilize the device and
  API timer system to generate a VP_LINE_EVID_GEN_TIMER event after a delay.

  We keep track of the number of timers per device to avoid exceeding the
  VP886_USER_TIMERS limit.  We keep track of the number of timers per line so
  that we know how much to subtract from the device count if a line is
  reinitialized.

  See the VP-API-II Reference Guide for more details on VpGenTimerCtrl().
*/
VpStatusType
Vp886GenTimerCtrl(
    VpLineCtxType *pLineCtx,
    VpGenTimerCtrlType timerCtrl,
    uint32 duration,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    bool success;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886GenTimerCtrl");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GenTimerCtrl");
        return status;
    }

    switch (timerCtrl) {
        case VP_GEN_TIMER_START: {
            if (pDevObj->userTimers >= VP886_USER_TIMERS) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_GEN_TIMER,
                    VP_GEN_TIMER_STATUS_RESRC_NA, handle, FALSE);
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886GenTimerCtrl - User timer limit reached"));
                status = VP_STATUS_DEVICE_BUSY;
                break;
            }
            success = Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_USER, duration, 0, handle);
            if (!success) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_GEN_TIMER,
                    VP_GEN_TIMER_STATUS_RESRC_NA, handle, FALSE);
                status = VP_STATUS_DEVICE_BUSY;
                break;
            }
            pDevObj->userTimers++;
            pLineObj->userTimers++;
            break;
        }
        case VP_GEN_TIMER_CANCEL: {
            success = Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_USER, handle, TRUE);
            if (success) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_GEN_TIMER,
                    VP_GEN_TIMER_STATUS_CANCELED, handle, FALSE);
                pDevObj->userTimers--;
                pLineObj->userTimers--;
            } else {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_GEN_TIMER,
                    VP_GEN_TIMER_STATUS_UNKNOWN, handle, FALSE);
            }
            break;
        }
        default:
            status = VP_STATUS_INVALID_ARG;
    }
    if (status == VP_STATUS_SUCCESS) {
        VP_TIMER(VpLineCtxType, pLineCtx, ("User timer counts: Dev:%d Line:%d",
            pDevObj->userTimers, pLineObj->userTimers));
    }

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GenTimerCtrl");
    return status;
}
#endif /* (VP886_USER_TIMERS > 0) */

/**  Vp886ShutdownDevice()
  Implements VpShutdownDevice() to shut down the power supply and all other
  resources in use by the device.

  See the VP-API-II Reference Guide for more details on VpShutdownDevice().
*/
VpStatusType
Vp886ShutdownDevice(
    VpDevCtxType *pDevCtx)
{
    VpStatusType status;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886ShutdownDevice");

    status = Vp886ShutdownDeviceInt(pDevCtx);

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886ShutdownDevice");
    return status;
}

/** Vp886ShutdownDeviceInt()
  Internally accessible implementation of Vp886ShutdownDevice().
*/
VpStatusType
Vp886ShutdownDeviceInt(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 channelId;
    VpLineCtxType *pLineCtx;
    Vp886LineObjectType *pLineObj;
    uint8 sysState;
    uint8 gTimer[VP886_R_GTIMER_LEN];
    uint8 chTimer[VP886_R_CHTIMER_LEN];
    uint8 icr3[VP886_R_ICR3_LEN];

    /* Set both lines to shutdown */
    sysState = VP886_R_STATE_SS_SHUTDOWN;
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, &sysState);
    
    /* Shut down switcher supplies.  This should be automatic on tracker, but it
       shouldn't hurt to do it anyway in case the auto control got disabled. */
    pDevObj->registers.swCtrl[0] &= ~VP886_R_SWCTRL_MODE_Y;
    pDevObj->registers.swCtrl[0] |= VP886_R_SWCTRL_MODE_Y_OFF;
    pDevObj->registers.swCtrl[0] &= ~VP886_R_SWCTRL_MODE_Z;
    pDevObj->registers.swCtrl[0] |= VP886_R_SWCTRL_MODE_Z_OFF;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWCTRL_WRT, VP886_R_SWCTRL_LEN, pDevObj->registers.swCtrl);

    /* Cancel device timers */
    gTimer[0] = gTimer[1] = chTimer[0] = chTimer[1] = 0x00;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_GTIMER_WRT, VP886_R_GTIMER_LEN, gTimer);
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_CHTIMER_WRT, VP886_R_CHTIMER_LEN, chTimer);

    /* Mask device interrupts */
    pDevObj->registers.intMask[0] = pDevObj->registers.intMask[1] = 
        pDevObj->registers.intMask[2] = pDevObj->registers.intMask[3] = 0xFF;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);

    /* Release control of VREF */
    icr3[0] = icr3[1] = icr3[2] = icr3[3] = 0x00;
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, icr3);

    /* Reset timer data */
    Vp886InitTimerQueue(pDevCtx);

    /* Flag the device as no longer initialized */
    pDevObj->busyFlags &= ~VP_DEV_INIT_IN_PROGRESS;
    pDevObj->busyFlags &= ~VP_DEV_INIT_CMP;

    /* Operations which require line object access.  Not critical, so it's OK if
       they don't exist in the current memory space. */
    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx == VP_NULL) {
            continue;
        }
        pLineObj = pLineCtx->pLineObj;
        
        pLineObj->lineState.usrCurrent = VP_LINE_DISABLED;
        pLineObj->lineState.currentState = VP_LINE_DISABLED;
    }

    return status;
}

#endif /* defined (VP_CC_886_SERIES) */
