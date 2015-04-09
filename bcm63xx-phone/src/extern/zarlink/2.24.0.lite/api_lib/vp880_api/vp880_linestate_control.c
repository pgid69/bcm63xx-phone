/** \file vp880_linestate_control.c
 * vp880_linestate_control.c
 *
 *  This file contains the control functions for the Vp880 device API.
 *
 * Copyright (c) 2012, Microsemi
 *
 * $Revision: 10953 $
 * $LastChangedDate: 2013-04-26 14:35:48 -0500 (Fri, 26 Apr 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "sys_service.h"

/**< Functions called by Set Line State to abstract device states used for ABS
 * and non ABS devices (if they are different). Set line state then operates
 * on device state only (abstracted from API-II line state).
 */
#ifdef VP880_FXS_SUPPORT
#ifdef VP880_TRACKER_SUPPORT
static uint8
Vp880GetLineStateNonABS(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);
#endif

static bool
Vp880SetStateRinging(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);
#endif

/**
 * Vp880SetLineState()
 *  This function is the API-II wrapper function for Set Line State - Internal
 * for the Vp880 API.
 *
 * Preconditions:
 *  Same as Vp880SetLineStateInt()
 *
 * Postconditions:
 *  Same as Vp880SetLineStateInt()
 */
VpStatusType
Vp880SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    bool overrideSameState;
    bool isFxs = FALSE;

    VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880SetLineState+"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * Do not proceed unless device init has already completed or in progress. Also, don't proceed
     * if device is currently running calibration
     */
    if ((!(pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS))) ||
        (pDevObj->state & VP_DEV_IN_CAL)) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880SetLineState-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }
    overrideSameState = FALSE;

    isFxs = (((pLineObj->status & VP880_IS_FXO) == VP880_IS_FXO) ? FALSE : TRUE);
#ifdef VP880_FXS_SUPPORT
    if (isFxs) {
        if (!(VpCSLACIsSupportedFxsState(pDevCtx->deviceType, state))) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC(VpLineCtxType, pLineCtx, ("-Vp890SetLineState() - Unsupported FXS State"));
            return VP_STATUS_INVALID_ARG;
        }
        if (Vp880IsChnlUndrTst(pDevObj, pLineObj->channelId)) {
            overrideSameState = TRUE;
        }
        if ((pLineObj->slicValueCache & VP880_SS_METERING_MASK)
#ifdef VP_CSLAC_SEQ_EN
            || (pLineObj->callerId.status & VP_CID_IN_PROGRESS)
#endif
            ) {
            overrideSameState = TRUE;
        }
    }
#endif

#ifdef VP_CSLAC_SEQ_EN
    if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
        overrideSameState = TRUE;
    }
#endif

    if (!overrideSameState && (state == pLineObj->lineState.usrCurrent)) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Skipping VpSetLineState() for ch %d, same state %d, time %d",
            pLineObj->channelId, state, pDevObj->timeStamp));
        VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880SetLineState-"));
        return VP_STATUS_SUCCESS;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Setting Channel %d to State %d at time %d",
        pLineObj->channelId, state, pDevObj->timeStamp));

    /* Clear the "called from API" flag. This affects the cadencer */
    pLineObj->status &= ~(VP880_SLS_CALL_FROM_API);

#ifdef VP880_FXS_SUPPORT
    /*
     * Special FXS handling to prevent setting the line to ringing if
     * off-hook
     */
    if (isFxs) {
        if ((pLineObj->lineState.condition & (VP_CSLAC_HOOK | VP_CSLAC_GKEY))
         && ((state == VP_LINE_RINGING_POLREV) || (state == VP_LINE_RINGING))) {
            /*
             * Go to Ring Trip Exit state instead, which could be ringing -- but
             * that's up to the application.
             */
            state = pLineObj->ringCtrl.ringTripExitSt;
        }

#ifdef VP880_TRACKER_SUPPORT
        if ((pDevObj->stateInt & VP880_IS_ABS) != VP880_IS_ABS) {
            /* Stop the entering ringing timer and restore swRegParam */
            if (pDevObj->ringParams.channelArray[pLineObj->channelId] == TRUE) {
                pDevObj->ringParams.channelArray[pLineObj->channelId] = FALSE;

                if (pDevObj->ringParams.channelArray[0] == FALSE) {
                    if ((pDevObj->ringParams.swRegParam[1] & VP880_SWY_AUTOPOWER_MASK) ==
                        VP880_SWY_AUTOPOWER_DIS) {
                        pDevObj->ringParams.swRegParam[1] |= VP880_SWY_AUTOPOWER_DIS;
                    } else {
                        pDevObj->ringParams.swRegParam[1] &= ~VP880_SWY_AUTOPOWER_DIS;
                    }
                }
                if (pDevObj->ringParams.channelArray[1] == FALSE) {
                    if ((pDevObj->ringParams.swRegParam[1] & VP880_SWZ_AUTOPOWER_MASK) ==
                        VP880_SWZ_AUTOPOWER_DIS) {
                        pDevObj->ringParams.swRegParam[1] |= VP880_SWZ_AUTOPOWER_DIS;
                    } else {
                        pDevObj->ringParams.swRegParam[1] &= ~VP880_SWZ_AUTOPOWER_DIS;
                    }
                }
                if ((pDevObj->ringParams.channelArray[0] == FALSE) &&
                    (pDevObj->ringParams.channelArray[1] == FALSE)) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,("Vp880LimitInRushCurrent(): kill the timer"));
                    pDevObj->devTimer[VP_DEV_TIMER_ENTER_RINGING] = 0;
                }

                if ((pDevObj->swParamsCache[0] != pDevObj->ringParams.swRegParam[0])
                 || (pDevObj->swParamsCache[1] != pDevObj->ringParams.swRegParam[1])
                 || (pDevObj->swParamsCache[2] != pDevObj->ringParams.swRegParam[2])) {
                    VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_PARAM_WRT,
                        VP880_REGULATOR_PARAM_LEN, pDevObj->ringParams.swRegParam);
                    VpMemCpy(pDevObj->swParamsCache, pDevObj->ringParams.swRegParam,
                        VP880_REGULATOR_PARAM_LEN);
                    VP_LINE_STATE(VpDevCtxType, pDevCtx,
                        ("Ringing Exit Timer Management Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                        pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                        pDevObj->timeStamp));
                }
            }
        }
#endif  /* VP880_TRACKER_SUPPORT */
    }
#endif  /* VP880_FXS_SUPPORT */

    status = Vp880SetLineStateInt(pLineCtx, state);
    if (status == VP_STATUS_SUCCESS) {
        /*
         * Reset the "Count" for leaky line conditions because there are some
         * normal state change conditions that will increment the count, therefore
         * causing exit of LP for non-leaky line
         */
#ifdef VP880_LP_SUPPORT /* Forced to undef if FXS Support is undef (see vp_api_cfg_int.h) */
        pLineObj->leakyLineCnt = 0;
#endif
        /*
         * IMPORTANT!!! Do not update usrCurrent until AFTER Vp880SetLineStateInt() returns with
         * VP_STATUS_SUCCESS. First, it's logical because the "state" value is not really the
         * user current state until "success" is actually returned, and Second, it will mess up
         * the logic of Vp880SetLineStateInt() if this order is changed.
         */
        pLineObj->lineState.usrCurrent = state;
    }

    /*
     * Set the "called from API" flag. Convenience for API functions so setting
     * this flag does not need to occur in multiple locations
     */
    pLineObj->status |= VP880_SLS_CALL_FROM_API;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    VP_API_FUNC(VpLineCtxType, pLineCtx, ("Vp880SetLineState-"));

    return status;
}

/**
 * Vp880SetLineStateInt()
 *  This function sets the line state for a given channel of a given device. The valid line states
 * are defined in the VpLineState type.
 *
 * Preconditions:
 *  The line must first be initialized prior to setting the line state.  The state must be a valid
 * line state as defined in the VpLineState type.
 *
 * Postconditions:
 *  Returns success code if the channel can be set and the line state is valid for the type of line
 * specified by the line context. If successfull, the line specified by the line context is set to
 * the line state specified.
 */
VpStatusType
Vp880SetLineStateInt(
    VpLineCtxType *pLineCtx,    /**< Line context to change line state on */
    VpLineStateType state)      /**< The desired line state to set */
{
#ifdef VP880_FXS_SUPPORT
    uint8 userByte = 0xFF;
    uint8 currentStateByte;
    bool disconnectTimerSet = FALSE;
    bool feedToDisable = FALSE;
    bool polarityInversion = FALSE;
#endif /* VP880_FXS_SUPPORT */

    uint8 mpiByte = 0;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal = pLineObj->ecVal;

    uint8 mpiIndex = 0;
    uint8 mpiBuffer[3 + VP880_OP_COND_LEN + VP880_LOOP_SUP_LEN + VP880_GEN_CTRL_LEN];

    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpLineStateType currentState = pLineObj->lineState.currentState;

#ifdef VP_CSLAC_SEQ_EN
    bool disableTones = TRUE;
    VpSeqDataType *pCadence = &pLineObj->cadence;
#endif /* VP_CSLAC_SEQ_EN */

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineStateInt+"));

#if defined (VP880_FXS_SUPPORT) && defined (VP_HIGH_GAIN_MODE_SUPPORTED)
    /*
     * Take care of Howler To/From State conditions. Note: Calling this function does NOT put
     * the line INTO the Howler State. It only verifies that the conditions for entering Howler
     * state are correct. It does however handle exiting the Howler Tone State.
     */
    if (((state == VP_LINE_HOWLER) || (state == VP_LINE_HOWLER_POLREV)) &&
        (currentState == state)) {
        /*
         * Same state changes are done here because the VpCSLACHowlerStateMgmt function handles
         * illegal transitions and the upper level SetLineState function will allow setting the
         * line to a Howler State during Tone Cadencing. So it can be called in a bypass mode for
         * the same state.
         */
        return VP_STATUS_SUCCESS;
    }

    status = VpCSLACHowlerStateMgmt(pLineCtx, pLineObj->lineState.usrCurrent,
                                    state, pLineObj->lineState.previous);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }
    if (((currentState == VP_LINE_HOWLER) || (currentState == VP_LINE_HOWLER_POLREV)) &&
       ((state != VP_LINE_HOWLER) && (state != VP_LINE_HOWLER_POLREV))) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                      ("Howler Exit at time (%d) Starting Hook Freeze Timer", pDevObj->timeStamp));
        VpCSLACSetTimer(&pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE],
                        MS_TO_TICKRATE(30, pDevObj->devProfileData.tickRate));
    }
#endif  /* defined (VP880_FXS_SUPPORT) && defined (VP_HIGH_GAIN_MODE_SUPPORTED) */

    /* Modify Operating conditions ONLY if not running calibration */
    if (pLineObj->lineState.calType == VP_CSLAC_CAL_NONE) {
        uint8 opCondTarget = pLineObj->opCond[0];
        /*
         * Read the status of the Operating Conditions register so we can change
         * only the TX and RX if the line state is a non-communication mode.
         * This also performs the line type/state verification.
         */
        opCondTarget &= (uint8)(~(VP880_CUT_TXPATH | VP880_CUT_RXPATH));
        opCondTarget &= (uint8)(~(VP880_HIGH_PASS_DIS | VP880_OPCOND_RSVD_MASK));

        status = Vp880GetTxRxPcmMode(pLineObj, state, &mpiByte);
        if (status == VP_STATUS_SUCCESS) {
            opCondTarget |= mpiByte;
        } else {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineStateInt-"));
            return status;
        }
        if (opCondTarget != pLineObj->opCond[0]) {
            pLineObj->opCond[0] = opCondTarget;
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("2. Writing 0x%02X to Operating Conditions",
                opCondTarget));
            VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT, VP880_OP_COND_LEN,
                            pLineObj->opCond);
        }
    }

#ifdef VP_CSLAC_SEQ_EN
    /* We're no longer in the middle of a time function */
    pCadence->status &= ~VP_CADENCE_STATUS_MID_TIMER;
    pCadence->timeRemain = 0;
#endif /* VP_CSLAC_SEQ_EN */
    /*
     * If this function is called by the application, stop the cadencer and
     * reset the Loop Supervision if it is incorrect
     */
    if (!(pLineObj->status & VP880_SLS_CALL_FROM_API)) {
        uint8 sigGenCtrl[VP880_GEN_CTRL_LEN];
#ifdef VP_CSLAC_SEQ_EN

        /* Abort ongoing metering cadences */
        if ((pCadence->status & VP_CADENCE_STATUS_METERING) &&
            (pCadence->status & VP_CADENCE_STATUS_ACTIVE)) {
            if (pLineObj->slicValueCache & VP880_SS_METERING_MASK) {
                /* Currently in the middle of a metering pulse.  Schedule a
                 * pending abort, but don't change anything yet */
                pCadence->meterPendingAbort = TRUE;
                pCadence->meterAbortLineState = state;
                return VP_STATUS_SUCCESS;
            } else {
                /* If not in the middle of a pulse, terminate the cadence. */
                pLineObj->lineEvents.process |= VP_LINE_EVID_MTR_ABORT;
                pLineObj->processData = pLineObj->cadence.meteringBurst;
                pCadence->status = VP_CADENCE_RESET_VALUE;
                pCadence->pActiveCadence = VP_PTABLE_NULL;
            }

        } else if (pLineObj->slicValueCache & VP880_SS_METERING_MASK) {
            /* If metering is on without an active cadence, terminate the
             * forever-on metering signal */
            pLineObj->lineEvents.process |= VP_LINE_EVID_MTR_ABORT;
            pLineObj->processData = pLineObj->cadence.meteringBurst;
        }

#endif /* VP_CSLAC_SEQ_EN */
        if (pLineObj->status & VP880_BAD_LOOP_SUP) {
            pLineObj->status &= ~(VP880_BAD_LOOP_SUP);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_LOOP_SUP_WRT,
                VP880_LOOP_SUP_LEN, pLineObj->loopSup);
        }

        /*
         * Disable tones and cadencing if going to a state that prevents it and
         * in all cases for the FXO line
         */
        switch(state) {
            case VP_LINE_STANDBY:
            case VP_LINE_STANDBY_POLREV:
            case VP_LINE_TIP_OPEN:
            case VP_LINE_DISCONNECT:
            case VP_LINE_RINGING:
            case VP_LINE_RINGING_POLREV:
            case VP_LINE_FXO_LOOP_OPEN:
            case VP_LINE_FXO_LOOP_CLOSE:
            case VP_LINE_FXO_RING_GND:
            case VP_LINE_FXO_OHT:
            case VP_LINE_FXO_TALK:
              /*
                * Disable signal generator A/B/C/D before making any changes and
                * stop previous cadences.
                */
                sigGenCtrl[0] = 0;
                if (sigGenCtrl[0] != pLineObj->sigGenCtrl[0]) {
                    pLineObj->sigGenCtrl[0] = sigGenCtrl[0];
                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_GEN_CTRL_WRT,
                        VP880_GEN_CTRL_LEN, pLineObj->sigGenCtrl);
                }
                break;

            default:
#ifdef VP_CSLAC_SEQ_EN
                /*
                 * Stop also if coming from Ringing. Note wwhen this function is called, the value
                 * usrCurrent is the previous User state. The next User state is set in the "state"
                 * value passed, presumably if this function returns VP_STATUS_SUCCESS. We know
                 * this because 1. Vp880SetLineState() sets the value usrCurrent = state after
                 * calling this function with return VP_STATUS_SUCCESS, and 2. This switch/case is
                 * inside "if (!(pLineObj->status & VP880_SLS_CALL_FROM_API))" which is TRUE when
                 * called by the Application (i.e., flag is set when called inside the API).
                 */

                if ((pLineObj->lineState.usrCurrent != VP_LINE_RINGING) &&
                    (pLineObj->lineState.usrCurrent != VP_LINE_RINGING_POLREV)) {
                    /* Previous state is NOT Ringing, so keep tones/cadencing running */
                    disableTones = FALSE;
                } else {
                    /*
                     * Previous state is Ringing or Ringing PolRev, so stop tones/cadencing.
                     * The value of disableTones is TRUE by default. Just need to disable the
                     * Generator itself here.
                     */
                    sigGenCtrl[0] = 0;
                    if (sigGenCtrl[0] != pLineObj->sigGenCtrl[0]) {
                        pLineObj->sigGenCtrl[0] = sigGenCtrl[0];
                        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_GEN_CTRL_WRT,
                            VP880_GEN_CTRL_LEN, pLineObj->sigGenCtrl);
                    }
                }
#endif /* VP_CSLAC_SEQ_EN */
                break;
        }

        /* send down the previous mpi commands if there are any */
        if (mpiIndex > 0) {
            VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
            mpiIndex = 0;
        }

#ifdef VP_CSLAC_SEQ_EN
        if (disableTones == TRUE) {
            pCadence->status = VP_CADENCE_RESET_VALUE;
            pCadence->pActiveCadence = VP_PTABLE_NULL;
        }

#ifdef VP880_FXS_SUPPORT
        /*  If the user is changing the line state stop callerId */
        if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
            VpCliStopCli(pLineCtx);
        }
#endif /* VP880_FXS_SUPPORT */
#endif /* VP_CSLAC_SEQ_EN */
    }   /* if NOT called from Inside the API */

    /* FXO TYPE LINE HANDLING */
    if (pLineObj->status & VP880_IS_FXO) {
#ifdef VP880_FXO_SUPPORT
        status = Vp880SetFxoState(pLineCtx, state);
        if (status == VP_STATUS_SUCCESS) {
            pLineObj->lineState.previous = currentState;
            pLineObj->lineState.currentState = state;
        }
#endif /* VP880_FXO_SUPPORT */
    } else {  /* FXS Handling */
#ifdef VP880_FXS_SUPPORT
        if (pDevObj->stateInt & VP880_IS_ABS) {
#ifdef VP880_ABS_SUPPORT
            userByte = Vp880GetLineStateABS(pLineCtx, state, FALSE);
#endif /* VP880_ABS_SUPPORT */
        } else {
#ifdef VP880_TRACKER_SUPPORT
            userByte = Vp880GetLineStateNonABS(pLineCtx, state);
#endif /* VP880_TRACKER_SUPPORT */
        }

        if (userByte == 0xFF) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineStateInt-"));
            return VP_STATUS_INVALID_ARG;
        }

        /* Modify userByte depending on the current polarity */
        currentStateByte = pLineObj->slicValueCache;

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Fetched userByte 0x%02X Compare with currentByte 0x%02X for Chan %d State %d Line Status 0x%04X",
            userByte, currentStateByte, pLineObj->channelId, state, pLineObj->status));

        /*
         * If changing state while the Disconnect Exit Timer is Active, update the target SLIC
         * state. Otherwise, the timer handler when complete will write the previous target SLIC
         * state - not the new one.
         */
        if ((!VpIsLowPowerTermType(pLineObj->termType)) && /* Don't set for LPM Term Types yet... */
            (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER)) {
            pLineObj->nextSlicValue = userByte;
        }

#ifdef VP_CSLAC_SEQ_EN
        if (pCadence->pActiveCadence != VP_NULL) {
            if ((pCadence->status &
                (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_IGNORE_POLARITY)) ==
                (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_IGNORE_POLARITY)) {

                userByte &= ~VP880_SS_POLARITY_MASK;
                userByte |= (currentStateByte & VP880_SS_POLARITY_MASK);
            }
        }
#endif /* VP_CSLAC_SEQ_EN */

        if ((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)
         || (currentState == VP_LINE_RINGING) || (currentState == VP_LINE_RINGING_POLREV)) {
            /*
             * Set State Ringing Returns TRUE if the line has NOT actually been changed. So return
             * at this point results in preventing the internal line state values to be updated
             * until the line is changed by the cadencer.
             */
            if (Vp880SetStateRinging(pLineCtx, state) == TRUE) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineStateInt-"));
                return VP_STATUS_SUCCESS;
            }

            if ((state != VP_LINE_RINGING) && (state != VP_LINE_RINGING_POLREV)) {
                /*
                 * Ringing exit scenario. We have to prevent Polarity Reversals from occuring while
                 * ringing is on the line otherwise the polarity reversal will occur during ringing
                 * which creates an undesirable transient on the line.
                 */
                /* Save the desired slic byte to be used in the ring exit timer */
                pLineObj->nextSlicValue = userByte;

                /* Check if polarity reversal is being performed. */
                if ((currentStateByte & VP880_SS_POLARITY_MASK) ^ (userByte & VP880_SS_POLARITY_MASK)) {
                    /*
                     * Polarity reversal is in this ring exit transition, so force the new slic
                     * state polarity to the same polarity as current ringing.
                     */
                    userByte &= ~VP880_SS_POLARITY_MASK;
                    userByte |= (currentStateByte & VP880_SS_POLARITY_MASK);
                }
            }
        }

#ifdef VP880_LP_SUPPORT
        if ((VpIsLowPowerTermType(pLineObj->termType))
            && (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER)) {
            pLineObj->nextSlicValue = userByte;

            if ((pLineObj->lineState.currentState == VP_LINE_STANDBY) &&
               ((state != VP_LINE_STANDBY) && (state != VP_LINE_DISCONNECT))) {
                /*
                 * Disconnect Exit to LPM-Standby was started but we're now
                 * changing to non-LPM. Need to correct ICR values.
                 */
                Vp880SetLPRegisters(pLineObj, FALSE);
                Vp880WriteLPExitRegisters(pLineCtx, deviceId, pLineObj->ecVal, &userByte);
                Vp880LowPowerMode(pDevCtx);
            }
        }
#endif /* VP880_LP_SUPPORT */

        /*
         * Enable Disconnect Recovery time for hook status if going FROM
         * Disconnect to a state that can detect off-hook
         */
             /* Coming from Disconnect...*/
        if ((pLineObj->lineState.currentState == VP_LINE_DISCONNECT) &&
            /* ..going to a state that can detect some feed condition */
            ((state != VP_LINE_DISCONNECT) && (state != VP_LINE_RING_OPEN))) {
            if (!(pLineObj->lineState.condition & VP_CSLAC_LINE_LEAK_TEST)) {
                VpCSLACSetTimer(&pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT],
                    MS_TO_TICKRATE(VP_DISCONNECT_RECOVERY_TIME, pDevObj->devProfileData.tickRate));

                /* Initialize the Disconnect Exit Timer State */
                pLineObj->discTimerExitState = 0;

                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Chan %d Setting VP_LINE_DISCONNECT_EXIT time to %d ms (VP_DISCONNECT_RECOVERY_TIME) at time %d line status 0x%04X",
                    pLineObj->channelId, VP_DISCONNECT_RECOVERY_TIME, pDevObj->timeStamp, pLineObj->lineState.condition));
                disconnectTimerSet = TRUE;
            }
        }

        /*
         * Set the disable flag if disabling feed. This prevents the
         * polrev timer from starting.
         */
        if ((state == VP_LINE_DISCONNECT) || (state == VP_LINE_TIP_OPEN)
            || (state == VP_LINE_RING_OPEN)) {
            feedToDisable = TRUE;
        }

        /*
         * Set Polarity Reverse timer if the SLIC is changing polarity. Exclude
         * Disconnect type recovery conditions since a timer is set above for
         * those conditions.
         */
        if ((currentStateByte & VP880_SS_POLARITY_MASK) != (userByte & VP880_SS_POLARITY_MASK)) {
            /*
             * Set this to mark the condition that requires loading of new
             * calibration coefficients.
             */
            polarityInversion = TRUE;

            /*
             * Timer is set if not exiting disconnect and staying in feed
             * conditions.
             */
            if ((disconnectTimerSet == FALSE) && (feedToDisable == FALSE)) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Setting Polarity Reversal Timer on channel %d at time %d status 0x%04X",
                    pLineObj->channelId, pDevObj->timeStamp, pLineObj->lineState.condition));
                pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE] =
                     (MS_TO_TICKRATE(VP_POLREV_DEBOUNCE_TIME,
                         pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER);
            }
        }

        if (pLineObj->calLineData.calDone == TRUE) {
            if ((polarityInversion == TRUE) ||
                (pLineObj->calLineData.forceCalDataWrite == TRUE)) {
                pLineObj->calLineData.forceCalDataWrite = FALSE;
                if (userByte & VP880_SS_POLARITY_MASK) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Updating DC Feed PolRev 0x%02X 0x%02X for Byte 0x%02X on channel %d at time %d",
                        pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1],
                        userByte, pLineObj->channelId, pDevObj->timeStamp));

                    VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_WRT,
                        VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);
                } else {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Updating DC Feed Normal 0x%02X 0x%02X for Byte 0x%02X on channel %d at time %d",
                        pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1],
                        userByte, pLineObj->channelId, pDevObj->timeStamp));

                    VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_WRT,
                        VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeed);
                }
            }
        }

        /*
         * Update the line object previous and current line state variables.
         * Note that this is not updated with ringing cadence until the line is
         * actually set to ringing (i.e., not when the application sets the line
         * to ringing with (possibly) a non-ringing state via the ringing
         * cadence.
         */
        pLineObj->lineState.previous = currentState;
        pLineObj->lineState.currentState = state;

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("In Vp880SetLineStateInt() Current state %d, next State %d Byte 0x%02X Chan %d at time %d",
            currentState, state, userByte, pLineObj->channelId, pDevObj->timeStamp));
#ifdef VP880_LP_SUPPORT
        if ((VpIsLowPowerTermType(pLineObj->termType))
             /*
              * If going from/to Disconnect/Standby in LPM, states are the same
              * so don't continue.
              */
         && ((state == VP_LINE_DISCONNECT) || (currentState == VP_LINE_DISCONNECT))) {
            if ((currentState == state) && (Vp880IsChnlUndrTst(pDevObj, pLineObj->channelId) == FALSE)) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineStateInt-"));
                return VP_STATUS_SUCCESS;
            } else {
                if (currentState == VP_LINE_TIP_OPEN) {
                    Vp880GroundStartProc(FALSE, pLineCtx, currentStateByte, userByte);
                }

                if (state == VP_LINE_DISCONNECT) {
                    Vp880RunLPDisc(pLineCtx, TRUE, userByte);
                } else {
                    Vp880RunLPDisc(pLineCtx, FALSE, userByte);
                }
            }
        } else {
#endif /* VP880_LP_SUPPORT */
            if ((userByte & VP880_SS_LINE_FEED_MASK) == VP880_SS_TIP_OPEN) {
                Vp880GroundStartProc(TRUE, pLineCtx, currentStateByte, userByte);
            } else {
                Vp880GroundStartProc(FALSE, pLineCtx, currentStateByte, userByte);
            }
#ifdef VP880_LP_SUPPORT
        }
#endif /* VP880_LP_SUPPORT */
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        /* If going to the Howler State, set registers after all other changes have been made */
        if ((state == VP_LINE_HOWLER) || (state == VP_LINE_HOWLER_POLREV)) {
            /*
             * This error check should be unnecessary since the code at the top of this function
             * returns immediately if going to/from any combination of howler states. But since
             * doing this will result in permanent incorrect setting of the DC Feed and ICR
             * registers, do everything possible to prevent it.
             */
            if ((pLineObj->lineState.previous != VP_LINE_HOWLER) &&
                (pLineObj->lineState.previous != VP_LINE_HOWLER_POLREV)) {
                VpCLSACHighGainMode(pLineCtx, TRUE);
            } else {
                VP_ERROR(VpLineCtxType, pLineCtx,
                         ("Howler Change from (%d) To (%d)", pLineObj->lineState.previous, state));
            }
        }
#endif  /* VP_HIGH_GAIN_MODE_SUPPORTED */
#endif  /* VP880_FXS_SUPPORT */
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineStateInt-"));
    return status;
}   /* Vp880SetLineStateInt */

#ifdef VP880_FXS_SUPPORT
/**
 * Vp880SetStateRinging()
 *  This function starts cadence and non-cadence ringing, as well as ringing exit.
 *
 * Preconditions:
 *  None. Calling function must know that this code should execute.
 *
 * Postconditions:
 *  Line object is modified if ringing cadence or exiting (timers). If not cadencing and ringing
 * is started, then return TRUE. Otherwise return FALSE.
 */
bool
Vp880SetStateRinging(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    /* Extract Line information used often in this function */
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;
    uint8 channelId = pLineObj->channelId;
    VpLineStateType currentState = pLineObj->lineState.currentState;

    /* Extract Device information used often in this function */
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

#ifdef VP_CSLAC_SEQ_EN
    VpProfilePtrType pProfile;
    VpSeqDataType *pCadence = &pLineObj->cadence;
#endif  /* VP_CSLAC_SEQ_EN */

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetStateRinging+"));

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("Ringing Control for channel %d: CurrentState %d, UserState %d NextState %d",
        channelId, currentState, pLineObj->lineState.usrCurrent, state));

    if ((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Ringing Enter on Ch (%d) time (%d)", channelId, pDevObj->timeStamp));

#ifdef VP_CSLAC_SEQ_EN
        pCadence->pActiveCadence = pLineObj->pRingingCadence;
        /*
         * We're entering a ringing state, so determine if we need to cadence. If we're not
         * cadencing, this is "always on", so we can disable the currently active cadence sequence
         * and immediately implement the ringing state change.
         */
        pProfile = pLineObj->cadence.pActiveCadence;
        if (pProfile == VP_PTABLE_NULL) {
            pCadence->status = VP_CADENCE_RESET_VALUE;
        } else {
            /*
             * We have a non-null cadence. If the cadence was not previously started, we'll start
             * it here and let the sequencer take over. Otherwise, it was previously started and
             * this state change is at the request of the sequencer.
             */
            if (!(pCadence->status & VP_CADENCE_STATUS_ACTIVE)) {
                /* We have a cadence and are just starting it */
                pCadence->status |= VP_CADENCE_STATUS_ACTIVE;
                pCadence->index = VP_PROFILE_TYPE_SEQUENCER_START;
                pCadence->pCurrentPos = &pProfile[VP_PROFILE_TYPE_SEQUENCER_START];
                pCadence->length = pProfile[VP_PROFILE_LENGTH];
                pCadence->status &= ~VP_CADENCE_STATUS_IGNORE_POLARITY;
                pCadence->status |=
                    (pProfile[VP_PROFILE_MPI_LEN] & 0x01) ? VP_CADENCE_STATUS_IGNORE_POLARITY : 0;

                /*
                 * Nullify any internal sequence so that the API doesn't think that an internal
                 * sequence of some sort is running
                 */
                pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_NONE;

                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetStateRinging-"));
                return TRUE;
            }
        }
#endif  /* VP_CSLAC_SEQ_EN */
        /*
         * Cadencing already called or null cadence.  We're ready to set the line to the Ringing
         * State but we have to first make sure that the signal generator parameters in the device
         * are setup for the ringing profile
         */
        /*
         * This step is only necessary if called internal to the VP-API-II. If externally called,
         * generators are disabled by higher level state control function.
         */
        if (pLineObj->status & VP880_SLS_CALL_FROM_API) {
            uint8 tempRingCtrlData = VP880_GEN_ALLOFF;
            if (tempRingCtrlData != pLineObj->sigGenCtrl[0]) {
                pLineObj->sigGenCtrl[0] = tempRingCtrlData;
                VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT,
                    VP880_GEN_CTRL_LEN, pLineObj->sigGenCtrl);
            }
        }

        if (pLineObj->ringingParams != VP_PTABLE_NULL) {
            if (((pLineObj->status & VP880_RING_GEN_NORM) && (state == VP_LINE_RINGING))
             || ((pLineObj->status & VP880_RING_GEN_REV) && (state == VP_LINE_RINGING_POLREV))) {
                /* Last generator programming was with the ringnig parameters. Nothing to do */
            } else {
                int16 biasErr;
                uint8 bias[VP880_RINGER_PARAMS_LEN];

                VpMemCpy(bias, pLineObj->ringingParams, VP880_RINGER_PARAMS_LEN);
                biasErr = (int16)((((uint16)(pLineObj->ringingParams[1]) << 8) & 0xFF00) +
                    ((uint16)(pLineObj->ringingParams[2]) & 0x00FF));
                if (state == VP_LINE_RINGING) {
                    /* Normal polarity */
                    biasErr -= ((pDevObj->vp880SysCalData.sigGenAError[channelId][0] -
                        pDevObj->vp880SysCalData.vocOffset[channelId][VP880_NORM_POLARITY]) * 16 / 10);
                    pLineObj->status |= VP880_RING_GEN_NORM;
                    pLineObj->status &= ~VP880_RING_GEN_REV;
                } else {
                    /* Reverse polarity */
                    biasErr += ((pDevObj->vp880SysCalData.sigGenAError[channelId][0] -
                        pDevObj->vp880SysCalData.vocOffset[channelId][VP880_REV_POLARITY]) * 16 / 10);
                    pLineObj->status |= VP880_RING_GEN_REV;
                    pLineObj->status &= ~VP880_RING_GEN_NORM;
                }
                bias[1] = (uint8)((biasErr >> 8) & 0x00FF);
                bias[2] = (uint8)(biasErr & 0x00FF);

                VpMpiCmdWrapper(deviceId, ecVal, VP880_RINGER_PARAMS_WRT,
                    VP880_RINGER_PARAMS_LEN, bias);
            }
        }

        /*
         * If the application chose auto-ring trip as determined by a non-ringing ring-trip-exit
         * state, enable the silicon Auto-State Changes (Auto-Ring Trip). This provides the fastest
         * ringing removal. When changing the line to a non-ringing state the silicon waits for a
         * zero crossing to actually remove ringing. If using only the API then it's possible the
         * API would miss a recent zero crossing in which case the silicon wouldn't remove ringing
         * until the next zero crossing - much later. When exiting ringing, we'll restore this
         * bit back to non-Auto-State control.
         */
        {
            uint8 ssCfg[VP880_SS_CONFIG_LEN] = {0x00};
            if ((pLineObj->ringCtrl.ringTripExitSt != VP_LINE_RINGING)
             && (pLineObj->ringCtrl.ringTripExitSt != VP_LINE_RINGING_POLREV)) {
                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD, VP880_SS_CONFIG_LEN, ssCfg);
                ssCfg[0] &= ~VP880_AUTO_SSC_DIS;
                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, ssCfg);
            }
        }

#ifdef VP880_ABS_SUPPORT
        /*
         * Disable the ABS Ringing Exit Timer if previously running. Enable device HPM -- BUT only
         * do this for single control systems. No need to do this for slave which are not in control
         * of their own supplies, and can't do this for Master supply systems because another device
         * being fed by this one may require HP.
         */
        if ((pDevObj->stateInt & VP880_IS_ABS) &&
            ((pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK) == VP880_ABS_CFG_SINGLE)) {
            uint8 regCtrl = VP880_SWY_HP | VP880_SWZ_HP;

            /*
             * Disable the currently running time that will try to change the supply mode back to
             * Medium Power. This is normal during ring cadence condition. The timer only has a
             * chance to expire when ringing cadence is complete via ring trip or when the calling
             * party "gives up" (i.e., VpSetLineState() is called with non-Ringing state)
             */
            pDevObj->devTimer[VP_DEV_TIMER_EXIT_RINGING] = 0;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_CTRL_WRT,
                            VP880_REGULATOR_CTRL_LEN, &regCtrl);
        }
#endif  /* VP880_ABS_SUPPORT */

#ifdef VP_CSLAC_SEQ_EN
        /*
         * If we're in an active Ringing Cadence, and ready to go into the Ringing state, generate
         * the Ringing Event and indicate that this is the Ringing On event
         */
        if ((pCadence->status & VP_CADENCE_STATUS_ACTIVE)
         && (pProfile[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_RINGCAD)) {
            pLineObj->lineEvents.process |= VP_LINE_EVID_RING_CAD;
            pLineObj->processData = VP_RING_CAD_MAKE;
        }
#endif  /* VP_CSLAC_SEQ_EN */

#ifdef VP880_TRACKER_SUPPORT
        if ((!(pDevObj->stateInt & VP880_IS_ABS)) /* Tracker Device */
            /* Disable HV Clamps ONLY when Ringing AND operating in High Voltage mode. */
            && (pDevObj->stateInt & VP880_IS_HIGH_VOLTAGE)
            && (pDevObj->devProfileData.lowVoltOverride == FALSE)) {
            pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] |= VP880_ICR2_SWY_LIM_CTRL;
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Disable HV Clamps: ICR2_WRT 0x%02X 0x%02X 0x%02X 0x%02X Ch %d Time %d",
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                pLineObj->channelId, pDevObj->timeStamp));

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);
        }
#endif  /* VP880_TRACKER_SUPPORT */
    } else if ((currentState == VP_LINE_RINGING_POLREV) || (currentState == VP_LINE_RINGING)) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Ringing Exit on Ch (%d) time (%d)", channelId, pDevObj->timeStamp));

        /*
         * Regardless of why we're exiting ringing, need to start the timer that monitors the
         * SLIC state for ringing exit and restores ASSC bit (from enable while in ringing to
         * disable for all other conditions). This timer needs to start ASAP.
         */
        pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] = (1 | VP_ACTIVATE_TIMER);
        pLineObj->lineTimers.timers.trackingTime = 0;
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Enabling VP_LINE_RING_EXIT_PROCESS on Ch %d timeStamp (%d)",
            pLineObj->channelId, pDevObj->timeStamp));

#ifdef VP880_TRACKER_SUPPORT
        /* Re-Enable the HV Clamp since Ringing is being removed */
        if (!(pDevObj->stateInt & VP880_IS_ABS)) {  /* Tracker Device */
            /*
             * Enable HV Clamps ONLY if they were previously disabled -- occurs only
             * operating in High Voltage mode.
             */
            if ((pDevObj->stateInt & VP880_IS_HIGH_VOLTAGE) &&
                (pDevObj->devProfileData.lowVoltOverride == FALSE)) {
                pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] &= ~(VP880_ICR2_SWY_LIM_CTRL);

                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Enable HV Clamps: ICR2_WRT 0x%02X 0x%02X 0x%02X 0x%02X Ch %d Time %d",
                    pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                    pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                    pLineObj->channelId, pDevObj->timeStamp));

                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT,
                                VP880_ICR2_LEN, pLineObj->icr2Values);
            } else {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("No change to HV Clamps needed Ch %d Time %d",
                     pLineObj->channelId, pDevObj->timeStamp));
            }
        }
#endif  /* VP880_TRACKER_SUPPORT */
#ifdef VP880_ABS_SUPPORT
        /*
         * For ABS devices, start the ringing exit timer used to trigger Switcher Medium Power
         * Enterance. OK if this was already set, simply causes it to continue.
         *
         * Don't set it unless the device is supplying it's own supply and not to any other device.
         */
        if ((pDevObj->stateInt & VP880_IS_ABS) &&
            (pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK) == VP880_ABS_CFG_SINGLE) {
            VpCSLACSetTimer(&pDevObj->devTimer[VP_DEV_TIMER_EXIT_RINGING],
                MS_TO_TICKRATE(VP_DEV_TIMER_EXIT_RINGING_SAMPLE, pDevObj->devProfileData.tickRate));
        }
#endif  /* VP880_ABS_SUPPORT */
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetStateRinging-"));
    return FALSE;
}   /* Vp880SetStateRinging() */

/**
 * Vp880GroundStartProc()
 *  This function implements the Ground Start procedures when entering or exiting a ground start
 * state. It should be called only by SetLineState.
 *
 * This function handles all transitions into TIP-OPEN for all termination types. It also handles
 * LPM Exit from TIP-OPEN and all other non-LPM state transitions.
 *
 * gsMode: TRUE  => we're going to TIP-OPEN state for all termination types
 *         FALSE => we're exiting TIP-OPEN state for LPM or other non-LPM state transition except
 *                  going to TIP-OPEN
 *
 * Preconditions:
 *  None. Calling function must know that this code should execute.
 *
 * Postconditions:
 *  Procedures according to operational notes are implemented for enter/exit a ground start state.
 * A timer is set when exiting ground start to delay disable of DC bias.
 */
void
Vp880GroundStartProc(
    bool gsMode,
    VpLineCtxType *pLineCtx,
    uint8 currentLineState,
    uint8 userByte)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 beforeState = (currentLineState & VP880_SS_LINE_FEED_MASK);
    uint8 afterState =  (userByte & VP880_SS_LINE_FEED_MASK);
    uint8 originalUserByte = userByte;

    uint8 mpiBuffer[5 + VP880_ICR1_LEN + VP880_ICR2_LEN + VP880_ICR3_LEN +
                        VP880_ICR4_LEN + VP880_SS_CONFIG_LEN];
    uint8 mpiIndex = 0;
    uint8 icr1Before[VP880_ICR1_LEN];
    uint8 icr2Before[VP880_ICR2_LEN];
    uint8 icr3Before[VP880_ICR3_LEN];
    uint8 icr4Before[VP880_ICR4_LEN];

#ifdef VP880_ABS_SUPPORT
    uint8 sysStateCfg[VP880_SS_CONFIG_LEN];
#endif

    bool transferToRinging = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GroundStartProc+"));

    if (((currentLineState & VP880_SS_LINE_FEED_MASK) == VP880_SS_TIP_OPEN)
     &&  (((afterState & VP880_SS_LINE_FEED_MASK) == VP880_SS_FEED_BALANCED_RINGING)
       || ((afterState & VP880_SS_LINE_FEED_MASK) == VP880_SS_FEED_UNBALANCED_RINGING))) {
        transferToRinging = TRUE;
    }

    VpMemCpy(icr1Before, pLineObj->icr1Values, VP880_ICR1_LEN);
    VpMemCpy(icr2Before, pLineObj->icr2Values, VP880_ICR2_LEN);
    VpMemCpy(icr3Before, pLineObj->icr3Values, VP880_ICR3_LEN);
    VpMemCpy(icr4Before, pLineObj->icr4Values, VP880_ICR4_LEN);

    if (gsMode == TRUE) {
        /* gsMode: TRUE  => we're going to TIP-OPEN state */
        /*
         * Implement "Return to Ground Start Idle" part of Ground Start
         * workaround for VoicePort Devices
         */
        pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION] |= VP880_ICR1_RING_AND_DAC_B2_3;
        pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION+1] |= VP880_ICR1_RING_AND_DAC_B2_3;

        if (beforeState == VP880_SS_DISCONNECT) {
            pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] &=
                ~VP880_ICR1_LINE_BIAS_OVERRIDE;
            pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &=
                (uint8)(~(VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE));

            if (icr2Before[VP880_ICR2_SENSE_INDEX] != pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX]) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Ground Start Enter: Write ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                    pLineObj->icr2Values[2], pLineObj->icr2Values[3], pLineObj->channelId,
                    pDevObj->timeStamp));
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                    VP880_ICR2_LEN, pLineObj->icr2Values);
            }
        }

        if ((pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION] != icr1Before[VP880_ICR1_RING_AND_DAC_LOCATION])
         || (pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION+1] != icr1Before[VP880_ICR1_RING_AND_DAC_LOCATION+1])
         || (pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] != icr1Before[VP880_ICR1_BIAS_OVERRIDE_LOCATION])) {
            mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);
        }

#ifdef VP880_ABS_SUPPORT
        if (pDevObj->stateInt & VP880_IS_ABS) {
            /*
             * Disable Auto Battery Switch so Ring stays near -48V for a
             * Ring to Ground, but first send down previously buffered MPI Cmd/Data if any.
             */
            if (mpiIndex > 0) {
                VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
                mpiIndex = 0;
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD, VP880_SS_CONFIG_LEN, sysStateCfg);
            sysStateCfg[0] |= VP880_AUTO_BAT_SWITCH_DIS;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, sysStateCfg);
        }
#endif  /* VP880_ABS_SUPPORT */
        /* Set Polarity of input to Gkey Detector */
        pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION] |= VP880_ICR4_GKEY_DET;
        pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION+1] &= ~VP880_ICR4_GKEY_DET;

        if ((pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION] != icr4Before[VP880_ICR4_GKEY_DET_LOCATION])
         || (pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION+1] != icr4Before[VP880_ICR4_GKEY_DET_LOCATION+1])) {
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
                VP880_ICR4_LEN, pLineObj->icr4Values);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Ground Start Enter: Write ICR4 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
            pLineObj->icr4Values[0], pLineObj->icr4Values[1],
                pLineObj->icr4Values[2], pLineObj->icr4Values[3], pLineObj->channelId,
                pDevObj->timeStamp));
        }

        if (mpiIndex > 0) {
            /* send down the mpi commands if any exist */
            VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
            mpiIndex = 0;
        }

        /*
         * Perform the SLIC state change, controlled by logic that determines
         * if low power mode can be used.
         */
        Vp880LLSetSysState(deviceId, pLineCtx, userByte, TRUE);

        /*
         * Connect Longitudinal Loop Control to ILout Pin Override
         * After this point, the Hook Bit indicates Ground Key
         */
        pLineObj->icr3Values[VP880_ICR3_LONG_LOOP_CTRL_LOCATION] |= VP880_ICR3_LONG_LOOP_CONTROL;
        pLineObj->icr3Values[VP880_ICR3_LONG_LOOP_CTRL_LOCATION+1] &= ~VP880_ICR3_LONG_LOOP_CONTROL;

        if ((pLineObj->icr3Values[VP880_ICR3_LONG_LOOP_CTRL_LOCATION] !=
             icr3Before[VP880_ICR3_LONG_LOOP_CTRL_LOCATION])
         || (pLineObj->icr3Values[VP880_ICR3_LONG_LOOP_CTRL_LOCATION+1] !=
             icr3Before[VP880_ICR3_LONG_LOOP_CTRL_LOCATION+1])) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Ground Start Enter: Write ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
            pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                pLineObj->icr3Values[2], pLineObj->icr3Values[3], pLineObj->channelId,
                pDevObj->timeStamp));

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR3_WRT, VP880_ICR3_LEN, pLineObj->icr3Values);
         }

        /*
         * Enable a (30ms) State Change Mask to allow for the line to stabilize
         * before considering valid Ring Ground detection. Initliaze the Ground Start Service timer
         */
        pLineObj->lineTimers.timers.timer[VP_LINE_GND_START_TIMER] =
            (MS_TO_TICKRATE(VP880_GND_START_DEBOUNCE,
                pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
        pLineObj->gsTimerExitState = 0;
    } else {
        /*
         * gsMode: FALSE => we're exiting TIP-OPEN state for LPM or other
         *         non-LPM state transition except going to TIP-OPEN
         */
#ifdef VP880_LP_SUPPORT
        if (VpIsLowPowerTermType(pLineObj->termType)) {
            /*
             * This is a LPM Termination Type. Check the line object to see if
             * it's currently capable of LPM.
             */
            if (pLineObj->status & VP880_LOW_POWER_EN) {
                /*
                 * The line is capable of LPM. That means this transition will be
                 * handled as part of the Low Power Mode Entry algorithm.
                 */
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Line in LPM - Adjusting LP Mode on channel %d, NOT writing to device time %d",
                    pLineObj->channelId, pDevObj->timeStamp));
                Vp880LLSetSysState(deviceId, pLineCtx, userByte, FALSE);
            } else {
                /*
                 * If we're coming from non-LPM into a LPM state, the LPM
                 * exit function handles the state transition cleanly except
                 * when exiting ringing. So under all other conditions we don't
                 * want to write the state here.
                 */

                /*
                 * We're going to a LPM state IF the other line is in a state
                 * that supports LPM AND this line is going to a LPM state.
                 */
                /* devState "set" means the other line is in a LPM state. */
                Vp880DeviceStateIntType devState = (pLineObj->channelId == 0) ?
                    (pDevObj->stateInt & VP880_LINE1_LP) :
                    (pDevObj->stateInt & VP880_LINE0_LP);
                uint8 beforeRing = (beforeState & VP880_SS_LINE_FEED_MASK);

                /*
                 * If we're transitioning from Ringing to a Low Power Mode state (i.e, a VP-API-II
                 * state that supports LPM 'AND' the line is not otherwise being prevented from
                 * going into Full LPM), then proceed into the 'if' conditions...
                 */
                if (((userByte == VP880_SS_DISCONNECT) || (userByte == VP880_SS_IDLE))
                  && (devState) && (beforeRing != VP880_SS_FEED_BALANCED_RINGING)
                  && (beforeRing != VP880_SS_FEED_UNBALANCED_RINGING)
                  /* Prevent LPM for Leaky Lines and User Override */
                  && (!(pLineObj->status & (VP880_LINE_LEAK | VP880_LP_STANDBY_IDLE)))) {

                    if (userByte == VP880_SS_IDLE) {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Line Going to LPM - Adjusting LP Mode on channel %d, writing to device time %d",
                            pLineObj->channelId, pDevObj->timeStamp));
                        Vp880LLSetSysState(deviceId, pLineCtx, VP880_SS_ACTIVE, TRUE);
                    } else {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Line Going to LPM - Adjusting LP Mode on channel %d, NOT writing to device time %d",
                            pLineObj->channelId, pDevObj->timeStamp));
                        Vp880LLSetSysState(deviceId, pLineCtx, userByte, FALSE);
                    }
                } else {
                    /*
                     * Almost there. Last "block" is if we're going to Ringing
                     * from Tip Open. In that case, we want the workarounds
                     * to be removed first, which is done in the "Ground Start
                     * Timer Service" routine. But set to Active to start
                     * providing normal feed.
                     */
                    if (transferToRinging) {
                        userByte = VP880_SS_ACTIVE;
                    }
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Adjusting LP Mode on channel %d, writing to device time %d",
                        pLineObj->channelId, pDevObj->timeStamp));
                    Vp880LLSetSysState(deviceId, pLineCtx, userByte, TRUE);
                }
            }
        } else {
#endif
            if (beforeState != afterState) {
                bool writeIcrReg = FALSE;

                /*
                 * When exiting from Disconnect to a feed state in silicon <= VC, the API goes
                 * through Tip-Open to avoid a short polarity reversal transition ("Ping Timer").
                 * While in Disconnect, some ICR settings are made to minimize power but prevent
                 * normal operation while in other feed states. The ICR settings are managed here,
                 * but this logic needs to treat > VC silicon Disconnect state the same as <= VC
                 * silicon Disconnect State AND Active "Ping" timer.
                 */
                if ((beforeState == VP880_SS_DISCONNECT) ||
                    (pLineObj->lineTimers.timers.timer[VP_LINE_PING_TIMER] & VP_ACTIVATE_TIMER)) {
                    /*
                     * Due to Speedup management, all Disoonnect exit state
                     * changes need to start with Acitve. Most transitions can
                     * tolerate the final state setting after 100ms (standard
                     * disconnect exit time) duration, except Ringing. Ringing
                     * should start as soon as possible, which is generally some
                     * time after the ~30ms device speedup.
                     */
                    Vp880LLSetSysState(deviceId, pLineCtx,
                        (VP880_SS_ACTIVE | (userByte & VP880_SS_POLARITY_MASK)), TRUE);
                    pLineObj->nextSlicValue = userByte;

                    /* Release Line Bias control */
                    pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] &=
                        ~VP880_ICR1_LINE_BIAS_OVERRIDE;

                    pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &=
                        (uint8)(~(VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE));
                    writeIcrReg = TRUE;

                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Disconnect Exit: Write ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                        pLineObj->icr1Values[0], pLineObj->icr1Values[1],
                        pLineObj->icr1Values[2], pLineObj->icr1Values[3],
                        pLineObj->channelId, pDevObj->timeStamp));

                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Disconnect Exit: Write ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                        pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                        pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                        pLineObj->channelId, pDevObj->timeStamp));

                    if (((userByte & VP880_SS_LINE_FEED_MASK) == VP880_SS_FEED_BALANCED_RINGING) ||
                        ((userByte & VP880_SS_LINE_FEED_MASK) ==  VP880_SS_FEED_UNBALANCED_RINGING)) {
                        /*
                         * Going to Ringing. Overwrite the standard disconnect
                         * exit time to cause ringing entry ASAP.
                         */
                        pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                            (MS_TO_TICKRATE(VP880_SPEEDUP_HOLD_TIME * 2,
                            pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

                        /* Initialize the Disconnect Exit Timer State */
                        pLineObj->discTimerExitState = 0;

                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Chan %d Setting VP_LINE_DISCONNECT_EXIT time to %d ms (VP880_SPEEDUP_HOLD_TIME * 2) at time %d line status 0x%04X",
                            pLineObj->channelId, (VP880_SPEEDUP_HOLD_TIME * 2), pDevObj->timeStamp, pLineObj->lineState.condition));
                    }
                } else {
                    Vp880LLSetSysState(deviceId, pLineCtx, userByte, TRUE);

                    if (afterState == VP880_SS_DISCONNECT) {
                        pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                            (MS_TO_TICKRATE(VP880_SPEEDUP_HOLD_TIME,
                            pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Chan %d Setting VP_LINE_DISCONNECT_EXIT time to %d ms (VP880_SPEEDUP_HOLD_TIME) at time %d",
                            pLineObj->channelId, VP880_SPEEDUP_HOLD_TIME, pDevObj->timeStamp));

                        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] |=
                            VP880_ICR1_LINE_BIAS_OVERRIDE;
                        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
                            ~(VP880_ICR1_LINE_BIAS_OVERRIDE);
                        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
                            VP880_ICR1_LINE_BIAS_OVERRIDE_NORM;

                        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] |=
                            (VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE);
                        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] &= ~VP880_ICR2_DAC_SENSE;
                        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] |= VP880_ICR2_FEED_SENSE;
                        writeIcrReg = TRUE;

                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Disconnect Enter: Write ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                            pLineObj->icr1Values[0], pLineObj->icr1Values[1],
                            pLineObj->icr1Values[2], pLineObj->icr1Values[3],
                            pLineObj->channelId, pDevObj->timeStamp));

                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Disconnect Enter: Write ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                            pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                            pLineObj->channelId, pDevObj->timeStamp));
                    }
                }

                if (writeIcrReg == TRUE) {
                    mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                        VP880_ICR2_LEN, pLineObj->icr2Values);

                    /* send down the mpi commands */
                    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
                    mpiIndex = 0;
                }
            } else {
                if (beforeState != VP_LINE_DISCONNECT) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Setting channel %d to device Linestate 0x%02X time %d",
                        pLineObj->channelId, userByte, pDevObj->timeStamp));
                    Vp880LLSetSysState(deviceId, pLineCtx, userByte, TRUE);
                }
            }
#ifdef VP880_LP_SUPPORT
        }
#endif

        /*
         * We are transferring from Tip Open to some other state. Need to remove
         * the Ground Start workarounds
         */
        if ((currentLineState & VP880_SS_LINE_FEED_MASK) == VP880_SS_TIP_OPEN) {
#ifdef VP880_ABS_SUPPORT
            if (pDevObj->stateInt & VP880_IS_ABS) {
                /* Re-enable Auto Battery switch only on VC and later silicon */
                if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] >= VP880_REV_VC) {
                    VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                        VP880_SS_CONFIG_LEN, sysStateCfg);
                    sysStateCfg[0] &= ~VP880_AUTO_BAT_SWITCH_DIS;
                    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                        VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, sysStateCfg);
                }
            }
#endif
            pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION] &=
                ~VP880_ICR1_RING_AND_DAC_B2_3;
            pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION+1] |=
                VP880_ICR1_RING_AND_DAC_B2_3;
            mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Tip Open Exit: Write ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d time %d",
                pLineObj->icr1Values[0], pLineObj->icr1Values[1],
                pLineObj->icr1Values[2], pLineObj->icr1Values[3],
                pLineObj->channelId, pDevObj->timeStamp));

            if (mpiIndex > 0) {
                /* send down the mpi commands */
                VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
            }

            /*
             * Unless the state we're going to is ringing, set a timer for 210ms
             * before disabling the DC bias. If we are going to Ringing, disable
             * the DC Bias immediately.
             * Holding the DC bias results in retaining the Tip voltage near
             * ground. For normal ground start signaling, 210ms is long enough
             * for the CPE to detect the Tip-Gnd voltage. If we're going directly
             * to ringing, this isn't a normal ground start sequence.
             */
            if (((afterState & VP880_SS_LINE_FEED_MASK) == VP880_SS_FEED_BALANCED_RINGING)
             || ((afterState & VP880_SS_LINE_FEED_MASK) == VP880_SS_FEED_UNBALANCED_RINGING)) {
                pLineObj->lineTimers.timers.timer[VP_LINE_GND_START_TIMER] =
                    (1 | VP_ACTIVATE_TIMER);
            } else {
                pLineObj->lineTimers.timers.timer[VP_LINE_GND_START_TIMER] =
                    (MS_TO_TICKRATE(VP880_GND_START_TIME,
                    pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
            }
            /* Initialize the ground start exit timer state */
            pLineObj->gsTimerExitState = 0;

            /*
             * Delay writing to the Ringing State until the workarounds are
             * removed.
             */
            pLineObj->nextSlicValue = originalUserByte;
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Chan %d Setting VP_LINE_GND_START_TIMER time to %d ms (VP880_GND_START_TIME) at time %d",
                pLineObj->channelId, VP880_GND_START_TIME, pDevObj->timeStamp));
        }
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GroundStartProc-"));
}   /* Vp880GroundStartProc() */

#ifdef VP880_ABS_SUPPORT
/**
 * Vp880GetLineStateABS
 *  Locally used function by Vp880SetLineState to get the line state byte used
 * for ABS devices.
 *
 * Preconditions:
 *  None. State to byte mapping only.
 *
 * Postconditions:
 *  Returns the byte that should be used in the device System State register
 * for the API State passed.
 */
uint8
Vp880GetLineStateABS(
    VpLineCtxType *pLineCtx,
    VpLineStateType state,
    bool forced)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;
    uint8 channelId = pLineObj->channelId;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 returnVal = 0xFF;
    uint8 icr6PreValues[VP880_ICR6_LEN];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateABS+"));

    VpMemCpy(icr6PreValues, pLineObj->icr6Values, VP880_ICR6_LEN);
    pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] &= ~VP880_DC_CAL_ABS_MASK;

    switch(state) {
        case VP_LINE_STANDBY:
            returnVal = VP880_SS_IDLE;
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absNormCal[channelId];
            break;

        case VP_LINE_TIP_OPEN:
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                (icr6PreValues[0] & VP880_DC_CAL_ABS_MASK);
            returnVal = VP880_SS_TIP_OPEN;
            break;

        case VP_LINE_ACTIVE:
        case VP_LINE_TALK:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
#endif
            returnVal = VP880_SS_ACTIVE;
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absNormCal[channelId];
            break;

        case VP_LINE_STANDBY_POLREV:
            returnVal = VP880_SS_IDLE_POLREV;
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absPolRevCal[channelId];
            break;

        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK_POLREV:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER_POLREV:
#endif
            returnVal = VP880_SS_ACTIVE_POLREV;
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absPolRevCal[channelId];
            break;

        case VP_LINE_OHT:
            returnVal = VP880_SS_ACTIVE_MID_BAT;
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absNormCal[channelId];
            break;

        case VP_LINE_OHT_POLREV:
            returnVal = VP880_SS_ACTIVE_MID_BAT_PR;
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absPolRevCal[channelId];
            break;

        case VP_LINE_DISCONNECT:
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                (icr6PreValues[0] & VP880_DC_CAL_ABS_MASK);
            returnVal = VP880_SS_DISCONNECT;
            break;

        case VP_LINE_RINGING:
            if (pLineObj->status & VP880_UNBAL_RINGING) {
                returnVal = VP880_SS_UNBALANCED_RINGING;
            } else {
                returnVal = VP880_SS_BALANCED_RINGING;
            }
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absNormCal[channelId];
            break;

        case VP_LINE_RINGING_POLREV:
            if (pLineObj->status & VP880_UNBAL_RINGING) {
                returnVal = VP880_SS_UNBALANCED_RINGING_PR;
            } else {
                returnVal = VP880_SS_BALANCED_RINGING_PR;
            }
            pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] |=
                pDevObj->vp880SysCalData.absPolRevCal[channelId];
            break;

        default:
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Error in Vp880GetLineStateABS-"));
            return returnVal;
    }

    /*
     * Only modifying the first byte (possibly), so that's the only one we
     * should compare with to determine if MPI write is needed.
     */
    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("ABS: Channel %d ICR6-Pre-ABS 0x%02X Target 0x%02X",
        pLineObj->channelId, icr6PreValues[0], pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX]));

    if ((pLineObj->icr6Values[VP880_DC_CAL_ABS_INDEX] != icr6PreValues[VP880_DC_CAL_ABS_INDEX])
     || (forced == TRUE)) {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR6_WRT, VP880_ICR6_LEN, pLineObj->icr6Values);
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateABS-"));

    return returnVal;
}
#endif

#ifdef VP880_TRACKER_SUPPORT
/**
 * Vp880GetLineStateNonABS
 *  Locally used function by Vp880SetLineState to get the line state byte used
 * for non-ABS devices.
 *
 * Preconditions:
 *  None. State to byte mapping only.
 *
 * Postconditions:
 *  Returns the byte that should be used in the device System State register
 * for the API State passed.
 */
uint8
Vp880GetLineStateNonABS(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS+"));

    switch(state) {
        case VP_LINE_STANDBY:
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
            return VP880_SS_IDLE;

        case VP_LINE_TIP_OPEN:
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
            return VP880_SS_TIP_OPEN;

        case VP_LINE_ACTIVE:
        case VP_LINE_TALK:
        case VP_LINE_OHT:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
#endif
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
            return VP880_SS_ACTIVE;

        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT_POLREV:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER_POLREV:
#endif
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
            return VP880_SS_ACTIVE_POLREV;

        case VP_LINE_STANDBY_POLREV:
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
            return VP880_SS_IDLE_POLREV;

        case VP_LINE_DISCONNECT:
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
            return VP880_SS_DISCONNECT;

        case VP_LINE_RINGING:
            if (pLineObj->status & VP880_UNBAL_RINGING) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
                return VP880_SS_UNBALANCED_RINGING;
            } else {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
                return VP880_SS_BALANCED_RINGING;
            }

        case VP_LINE_RINGING_POLREV:
            if (pLineObj->status & VP880_UNBAL_RINGING) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
                return VP880_SS_UNBALANCED_RINGING_PR;
            } else {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));
                return VP880_SS_BALANCED_RINGING_PR;
            }

        default:
            break;
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880GetLineStateNonABS-"));

    return 0xFF;
}
#endif
#endif

/**
 * Vp880LLSetSysState()
 *  This function writes to the System State register and based on the state
 * being set, determines if low power mode can or cannot be used, or if the line
 * is recovering from Disconnect to a Feed state. In the latter case, a timer
 * is set to transition through Tip Open first to prevent "ping" of the phone.
 * In this case, the final state is set when the timer expires.
 *
 * Preconditions:
 *  This function is called internally by the API-II only.
 *
 * Postconditions:
 *  The System State Register is updated with either the value passed, or
 * Tip Open. If low power mode termination type exists, a flag in the device
 * object indicating that low power can or cannot be used is modified. If coming
 * out of Disconnect, Tip Open is written and a timer is set.
 */
void
Vp880LLSetSysState(
    VpDeviceIdType deviceId,
    VpLineCtxType *pLineCtx,
    uint8 lineState,
    bool writeToDevice)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;
    uint8 channelId = pLineObj->channelId;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
#ifdef VP880_FXS_SUPPORT
    bool lineIsFxs = FALSE;
#endif
    uint8 lineStatePre[VP880_SYS_STATE_LEN];
    lineStatePre[0] = pLineObj->slicValueCache;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LLSetSysState+"));

#ifdef VP880_FXS_SUPPORT
    if (!(pLineObj->status & VP880_IS_FXO)) {
        lineIsFxs = TRUE;

        if ((Vp880IsChnlUndrTst(pDevObj, channelId) == TRUE) ||
            (pLineObj->status & VP880_LINE_IN_CAL)) {
            pDevObj->stateInt &=
                ((channelId == 0) ? ~VP880_LINE0_LP : ~VP880_LINE1_LP);
            VP_LINE_STATE(VpLineCtxType, pLineCtx,("3. Clearing LP flag for channel %d",channelId));
        } else {
            bool lpTermType = FALSE;
            bool hookStatus = FALSE;
#ifdef VP880_LP_SUPPORT
            if (VpIsLowPowerTermType(pLineObj->termType)) {
                lpTermType = TRUE;
                VpCSLACGetLineStatus(pLineCtx, VP_INPUT_RAW_HOOK, &hookStatus);
            }
#endif
            hookStatus = (pLineObj->lineState.currentState == VP_LINE_STANDBY) ? hookStatus : FALSE;

            if ((lpTermType == TRUE) && (hookStatus == FALSE) && 
                (!(pLineObj->status & (VP880_LINE_LEAK | VP880_LP_STANDBY_IDLE)))) {
                if ((pLineObj->lineState.currentState == VP_LINE_DISCONNECT) ||
                    (pLineObj->lineState.currentState == VP_LINE_STANDBY)) {

                    if (((lineStatePre[0] & VP880_SS_LINE_FEED_MASK) != VP880_SS_FEED_BALANCED_RINGING)
                     && ((lineStatePre[0] & VP880_SS_LINE_FEED_MASK) != VP880_SS_FEED_UNBALANCED_RINGING)) {

                        pDevObj->stateInt |=
                            ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);

                        VP_LINE_STATE(VpLineCtxType, pLineCtx,("1. Setting LP flag for channel %d Hook Status %d",
                            channelId, hookStatus));
                    } else {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,("1. Delay Setting LP flag for channel %d",channelId));
                    }
                } else {
                    pDevObj->stateInt &=
                        ((channelId == 0) ? ~VP880_LINE0_LP : ~VP880_LINE1_LP);
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,("1. Clearing LP flag for channel %d Device Status 0x%08lX",
                        channelId, pDevObj->stateInt));
                }
            } else {
                pDevObj->stateInt &=
                    ((channelId == 0) ? ~VP880_LINE0_LP : ~VP880_LINE1_LP);
                VP_LINE_STATE(VpLineCtxType, pLineCtx,("2. Clearing LP flag for channel %d Device Status 0x%08lX",
                    channelId, pDevObj->stateInt));
            }
        }
    }
#endif

    /* Device Write Override: setting flags without a device write */
    if (writeToDevice == FALSE) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LLSetSysState-"));
        return;
    }

#ifdef VP880_FXS_SUPPORT
    if (((lineStatePre[0] & VP880_SS_LINE_FEED_MASK) == VP880_SS_DISCONNECT)
      && (lineIsFxs == TRUE)) {
        bool nextIsFeed = TRUE;
        switch (lineState & VP880_SS_LINE_FEED_MASK) {
            case VP880_SS_DISCONNECT:
            case VP880_SS_TIP_OPEN:
            case VP880_SS_RING_OPEN:
            case VP880_SS_SHUTDOWN:
                nextIsFeed = FALSE;
                break;
            default:
                break;
        }

        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Preparing 0x%02X for Next Slic State on Channel %d",
            lineState, channelId));

        pLineObj->nextSlicValue = lineState;

        if ((pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] <= VP880_REV_VC) && (nextIsFeed)) {
            lineState = VP880_SS_TIP_OPEN;

            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("2. Setting Chan %d to TIP_OPEN at Time %d",
                channelId, pDevObj->timeStamp));

            pLineObj->lineTimers.timers.timer[VP_LINE_PING_TIMER] =
                (MS_TO_TICKRATE(VP880_PING_TIME,
                    pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
        }

        Vp880UpdateBufferChanSel(pDevObj, channelId, lineState, TRUE);
#ifdef VP880_TRACKER_SUPPORT
        if ((lineState == VP880_SS_BALANCED_RINGING) && (IN_RUSH_CYCLE_TOTAL > 0) &&
            (pDevObj->devProfileData.peakManagement == TRUE)) {
            Vp880LimitInRushCurrent(pDevObj, ecVal, FALSE);
            pLineObj->slicValueCache = VP880_SS_BALANCED_RINGING;
        } else {
#endif
            if (pLineObj->slicValueCache != lineState) {
                pLineObj->slicValueCache = lineState;

                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("2. Setting Chan %d to System State 0x%02X at Time %d",
                    channelId, lineState, pDevObj->timeStamp));

                VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }
#ifdef VP880_TRACKER_SUPPORT
        }
#endif
    } else {
#endif
        if (pLineObj->lineTimers.timers.timer[VP_LINE_PING_TIMER] & VP_ACTIVATE_TIMER) {
            pLineObj->nextSlicValue = lineState;
        } else {
            Vp880UpdateBufferChanSel(pDevObj, channelId, lineState, TRUE);
#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
            if (((lineState == VP880_SS_BALANCED_RINGING) || (lineState == VP880_SS_UNBALANCED_RINGING))
                && (IN_RUSH_CYCLE_TOTAL > 0) &&
                (pDevObj->devProfileData.peakManagement == TRUE)) {
                Vp880LimitInRushCurrent(pDevObj, ecVal, FALSE);
                pLineObj->slicValueCache = lineState;
            } else {
#endif
                if (pLineObj->slicValueCache != lineState) {
                    pLineObj->slicValueCache = lineState;

                    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("1. Setting Chan %d to System State 0x%02X at Time %d",
                        channelId, lineState, pDevObj->timeStamp));

                    VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                        VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

                    if ((pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER) &&
                        (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] <= VP880_REV_VC)){
                        pLineObj->nextSlicValue = pLineObj->slicValueCache;
                    }
                }
#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
            }
#endif
        }
#ifdef VP880_FXS_SUPPORT
    }
#endif
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LLSetSysState-"));
}

#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
/**
 * Vp880LimitInRushCurrent()
 *  This function reduces the in-rush current during the battery transition
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  Slac state is in ringing
 */
void
Vp880LimitInRushCurrent(
    Vp880DeviceObjectType *pDevObj,
    uint8 ecVal,    /* 0x00, 0x80, 0x01, 0x02, 0x81, 0x82 */
    bool callback)  /* TRUE if called from device timer, FALSE if called by Set Line State */
{
#if (IN_RUSH_CYCLE_TOTAL > 0)
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 regCtrl, ssConf, sStateRing, sState, channelId;

    if (callback == FALSE) {    /* Must be valid channel */
        channelId = ((ecVal & VP880_EC_BITS_MASK) >> 1);

        VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): Initialization"));

        /* Limit only in fixed ringing */
        if (pDevObj->ringParams.channelArray[channelId] == FALSE) {
            VpMemCpy(pDevObj->ringParams.swRegParam, pDevObj->swParamsCache, VP880_REGULATOR_PARAM_LEN);
        } else {
            VpMemCpy(pDevObj->swParamsCache, pDevObj->ringParams.swRegParam, VP880_REGULATOR_PARAM_LEN);
        }

        pDevObj->ringParams.channelArray[channelId] = TRUE;

        /* The in-rush current limitation applies only in fixed voltage */
        /* Assuming that SWY is for VP880_EC_CH1 and SWZ is for VP880_EC_CH2 */
        if ((((ecVal & VP880_EC_BITS_MASK) == VP880_EC_CH1) &&
            (((pDevObj->swParamsCache[0] & VP880_YRING_TRACK_MASK) == VP880_YRING_TRACK_EN) ||
             ((pDevObj->stateInt & VP880_LINE0_IS_FXO) == VP880_LINE0_IS_FXO))) ||
            (((ecVal & VP880_EC_BITS_MASK) == VP880_EC_CH2) &&
            (((pDevObj->swParamsCache[0] & VP880_ZRING_TRACK_MASK) == VP880_ZRING_TRACK_EN) ||
             (pDevObj->stateInt & VP880_LINE1_IS_FXO)))) {

            sState = VP880_SS_BALANCED_RINGING;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, &sState);
            VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): Tracking enable, set and exit"));
            return;
        } else {
            if (pDevObj->ringParams.channelArray[0] == TRUE) {
                pDevObj->swParamsCache[1] |= VP880_SWY_AUTOPOWER_DIS;
            }

            if (pDevObj->ringParams.channelArray[1] == TRUE) {
                pDevObj->swParamsCache[2] |= VP880_SWZ_AUTOPOWER_DIS;
            }

            VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_PARAM_WRT,
                VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
            VP_LINE_STATE(None, NULL,
                ("Inrush Current Limit Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                pDevObj->timeStamp));
        }

        /* Set the battery to medium power */
        VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_CTRL_RD,
            VP880_REGULATOR_CTRL_LEN, &regCtrl);
        if (channelId == 0) {
            regCtrl &= ~VP880_SWY_MODE_MASK;
            regCtrl |= VP880_SWY_MP;
        } else {
            regCtrl &= ~VP880_SWZ_MODE_MASK;
            regCtrl |= VP880_SWZ_MP;
        }
        VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_CTRL_WRT,
            VP880_REGULATOR_CTRL_LEN, &regCtrl);

        /* Enable ZXR and set the slac state in balanced ringing */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD, VP880_SS_CONFIG_LEN,
            &ssConf);
        if ((ssConf & VP880_ZXR_MASK) == VP880_ZXR_DIS) {
            ssConf &= ~VP880_ZXR_DIS;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT,
                VP880_SS_CONFIG_LEN, &ssConf);
        }

        sStateRing = VP880_SS_BALANCED_RINGING;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
            VP880_SYS_STATE_LEN, &sStateRing);

        pDevObj->ringParams.stage[channelId] = 0;
        pDevObj->devTimer[VP_DEV_TIMER_ENTER_RINGING] = (MS_TO_TICKRATE(10,
            pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
    } else {    /* callback - no valid channel */
        VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): callback"));

        for (channelId = 0; channelId < VP880_MAX_NUM_CHANNELS; channelId++) {
            uint8 ecValCh = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);

            if (pDevObj->ringParams.channelArray[channelId] == TRUE) {
                pDevObj->ringParams.stage[channelId]++;

                if ((pDevObj->ringParams.stage[channelId] <= IN_RUSH_CYCLE_TOTAL) &&
                    (pDevObj->ringParams.stage[channelId] >
                    (IN_RUSH_CYCLE_TOTAL - IN_RUSH_CYCLE_END))) {
                    /* Charge the battery capacitor in medium power and allow ringing */
                    VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): Finish Settling %d",
                        pDevObj->ringParams.stage[channelId]));
                    pDevObj->devTimer[VP_DEV_TIMER_ENTER_RINGING] = (MS_TO_TICKRATE(10,
                        pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
                } else if (pDevObj->ringParams.stage[channelId] < IN_RUSH_CYCLE_TOTAL) {
                    /* Charge the battery capacitor in medium power and avoid ringing */
                    VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): Settling %d",
                        pDevObj->ringParams.stage[channelId]));
                    sState = VP880_SS_ACTIVE;
                    VpMpiCmdWrapper(deviceId, (ecValCh | pDevObj->ecVal),
                        VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &sState);

                    sState = VP880_SS_BALANCED_RINGING;
                    VpMpiCmdWrapper(deviceId, (ecValCh | pDevObj->ecVal),
                        VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &sState);

                    pDevObj->devTimer[VP_DEV_TIMER_ENTER_RINGING] = (MS_TO_TICKRATE(10,
                        pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
                } else {
                    VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): Conclusion"));

                    /* Reset the regulator in original mode */
                    pDevObj->ringParams.channelArray[channelId] = FALSE;

                    if (pDevObj->ringParams.channelArray[0] == FALSE) {
                        if ((pDevObj->swParamsCache[1] & VP880_SWY_AUTOPOWER_MASK) == VP880_SWY_AUTOPOWER_DIS) {
                            pDevObj->swParamsCache[1] |= VP880_SWY_AUTOPOWER_DIS;
                        } else {
                            pDevObj->swParamsCache[1] &= ~VP880_SWY_AUTOPOWER_DIS;
                        }
                    }

                    if (pDevObj->ringParams.channelArray[1] == FALSE) {
                        if ((pDevObj->swParamsCache[1] & VP880_SWZ_AUTOPOWER_MASK) == VP880_SWZ_AUTOPOWER_DIS) {
                            pDevObj->swParamsCache[1] |= VP880_SWZ_AUTOPOWER_DIS;
                        } else {
                            pDevObj->swParamsCache[1] &= ~VP880_SWZ_AUTOPOWER_DIS;
                        }
                    }

                    if ((pDevObj->ringParams.channelArray[0] == FALSE) &&
                        (pDevObj->ringParams.channelArray[1] == FALSE)) {
                        VP_LINE_STATE(None, NULL,("Vp880LimitInRushCurrent(): kill the timer"));
                        pDevObj->devTimer[VP_DEV_TIMER_ENTER_RINGING] = 0;
                    }
                    VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_PARAM_WRT,
                        VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
                    VP_LINE_STATE(None, NULL,
                        ("Inrush Current Limit Conclusion Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                        pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                        pDevObj->timeStamp));

                    /* Set the battery to high power */
                    VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_CTRL_RD,
                        VP880_REGULATOR_CTRL_LEN, &regCtrl);
                    regCtrl |= ((channelId == 0) ? VP880_SWY_HP : VP880_SWZ_HP);
                    VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_CTRL_WRT,
                        VP880_REGULATOR_CTRL_LEN, &regCtrl);
                }
            }
        }
    }
#endif
}
#endif  /* VP880_TRACKER_SUPPORT */

#endif
