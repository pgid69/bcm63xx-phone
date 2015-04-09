/** \file vp880_fxo_control.c
 * vp880_fxo_control.c
 *
 *  This file contains the control functions for the Vp880 device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 10504 $
 * $LastChangedDate: 2012-09-27 16:23:24 -0500 (Thu, 27 Sep 2012) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES) && defined (VP880_FXO_SUPPORT)

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
#ifndef VP880_CLARE_RINGING_DETECT
static void
Vp880LowRingFreqDetect(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType *tempRingingSt,
    VpCslacLineCondType *tempPolRevSt,
    bool *retFlag);
#endif

/**
 * Vp880SetFxoState()
 *  This function sets the line state for an FXO line.
 *
 * Preconditions:
 *  See VpSetLineState()
 *
 * Postconditions:
 *  Returns success code if the channel can be set and the line state.
 */
VpStatusType
Vp880SetFxoState(
    VpLineCtxType *pLineCtx,    /**< Line context to change line state on */
    VpLineStateType state)      /**< The desired line state to set */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VpLineStateType currentState = pLineObj->lineState.currentState;
    uint8 fxoCidLine;
    uint8 userByte, userByteBefore;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetFxoState+"));

    /*
     * FXO is straightforward, just set as defined by termination type since we already know it's
     * not an unsupported state (except error maybe)
     */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_IODATA_REG_RD, VP880_IODATA_REG_LEN, &userByteBefore);
    userByte = userByteBefore;

    if ((pLineObj->termType == VP_TERM_FXO_GENERIC)
     || (pLineObj->termType == VP_TERM_FXO_DISC)) {
        /* Pre-clear both bits for convenience */
        if (pLineObj->termType == VP_TERM_FXO_DISC) {
            fxoCidLine = VP880_IODATA_IO3;
        } else {
            fxoCidLine = VP880_FXO_CID_LINE;
        }

        userByte &= (uint8)(~(VP880_IODATA_IO1 | fxoCidLine));

        switch(state) {
            case VP_LINE_FXO_TALK:
                /* IO3/IO1 = 00, so ok */
                break;

            case VP_LINE_FXO_OHT:
                /* IO3/IO1 = 01, so set IO1 to 1 */
                userByte |= VP880_IODATA_IO1;
                break;

            case VP_LINE_FXO_LOOP_CLOSE:
                /* IO3/IO1 = 10, so set IO3 to 1 */
                userByte |= fxoCidLine;
                break;

            case VP_LINE_FXO_LOOP_OPEN:
                /* IO3/IO1 = 11, so set IO3 and IO1 to 1 */
                userByte |= (fxoCidLine | VP880_IODATA_IO1);
                break;

            default:
                /* This should be redundant from TX/RX PCM code section above */
                return VP_STATUS_INVALID_ARG;
        }

        VP_FXO_FUNC(VpLineCtxType, pLineCtx, ("Set FXO State %d Register 0x%02X at time %d",
            state, userByte, pDevObj->timeStamp));

       /* Set the loop open/close bit */
       VpMpiCmdWrapper(deviceId, ecVal, VP880_IODATA_REG_WRT, VP880_IODATA_REG_LEN, &userByte);

    } else {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetFxoState-"));
        return VP_STATUS_INVALID_ARG;
    }

    if ( ((state == VP_LINE_FXO_TALK) || (state == VP_LINE_FXO_LOOP_CLOSE))
      && ((currentState == VP_LINE_FXO_OHT) || (currentState == VP_LINE_FXO_LOOP_OPEN)) ) {
        VP_FXO_FUNC(VpLineCtxType, pLineCtx, ("Last State Change Timer Set at %d",
            pDevObj->timeStamp));

        pLineObj->lineTimers.timers.fxoTimer.lastStateChange = 0;
    }

    if ( ((state == VP_LINE_FXO_OHT) || (state == VP_LINE_FXO_LOOP_OPEN))
      && ((currentState == VP_LINE_FXO_TALK) || (currentState == VP_LINE_FXO_LOOP_CLOSE)) ) {
        VP_FXO_FUNC(VpLineCtxType, pLineCtx, ("Last State Change Timer Set at %d",
            pDevObj->timeStamp));

        pLineObj->lineTimers.timers.fxoTimer.lastStateChange = 0;
    }

    /* Set the FXO CODEC Mode */
    userByte = (VP880_FXO_ACTIVATE_CODEC | VP880_FXO_SUPERVISION_EN);

    /* Perform the FXO Line State change */
    if (pLineObj->slicValueCache != userByte) {
        pLineObj->slicValueCache = userByte;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
            VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetFxoState-"));

    return VP_STATUS_SUCCESS;
}

#ifndef VP880_CLARE_RINGING_DETECT
/**
 * Vp880LowRingFreqDetect()
 *  This function should only be called by Vp880ProcessFxoLine. It performs
 * Ringing and PolRev detection on lines configured to detect ringing
 * frequencies below what the device can support.
 *
 * Preconditions:
 *  Conditions defined by purpose of Vp880ProcessFxoLine.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated. Return Flag passed is set to TRUE if an event is pending.
 */
void
Vp880LowRingFreqDetect(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType *tempRingingSt,
    VpCslacLineCondType *tempPolRevSt,
    bool *retFlag)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    uint16 polRevPeriod;

    if (*tempPolRevSt !=
        (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_POLREV)) {
        pLineObj->lineState.condition &= ~VP_CSLAC_POLREV;
        pLineObj->lineState.condition |= *tempPolRevSt;
        pLineObj->lineState.condition &= ~VP_CSLAC_POLREV_REPORTED;

        /*
         * Capture the period of the last two pol revs. Used for
         * Ringing Detection
         */
        pLineObj->lineTimers.timers.fxoTimer.prevHighToLowTime =
            (uint16)((uint32)(pLineObj->lineTimers.timers.fxoTimer.timePrevPolRev)
            + (uint32)(pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev) / 4L);

        pLineObj->lineTimers.timers.fxoTimer.timePrevPolRev =
            pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev;
        pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev = 0;
    }

    if (pDevObj->intReg[channelId] & VP880_LIU1_MASK) {
        pLineObj->lineState.condition |= VP_CSLAC_LIU;
    } else {
        pLineObj->lineState.condition &= ~VP_CSLAC_LIU;
        pLineObj->lineTimers.timers.fxoTimer.lastNotLiu =
            pLineObj->ringDetMax * 2;
    }

    polRevPeriod =
        pLineObj->lineTimers.timers.fxoTimer.prevHighToLowTime;

    if ((pLineObj->lineTimers.timers.fxoTimer.timePrevPolRev / 4)
        < pLineObj->ringDetMax) {
        if (pLineObj->lineState.condition & VP_CSLAC_POLREV) {
            pLineObj->fxoData = VP_POLREV_REVERSE;
        } else {
            pLineObj->fxoData = VP_POLREV_NORMAL;
        }
    }

    /* Evaluate the detectors */
    /*
     * If the LIU Threshold has been exceeded, it's definitely not
     * PolRev, but may be ringing. If it has been completely
     * debounced, then Ringing is removed if we previously had
     * Ringing.
     */
    if (pLineObj->lineTimers.timers.fxoTimer.lastNotLiu) {
        /*
         * The threshold has been exceeded for a period within the
         * debounce interval. Check on Ringing condition.
         */

        if ((pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev / 4)
            > pLineObj->ringDetMax) {
            /*
             * This occurs because we had a recent LIU threshold,
             * but the frequency is not correct. No action other
             * than clearing Ringing state is necessary.
             */
            *tempRingingSt = VP_CSLAC_STATUS_INVALID;
            if (pLineObj->lineState.condition & VP_CSLAC_POLREV) {
                pLineObj->fxoData = VP_POLREV_REVERSE;
            } else {
                pLineObj->fxoData = VP_POLREV_NORMAL;
            }
        } else if ((polRevPeriod <= pLineObj->ringDetMax)
                && (polRevPeriod >= pLineObj->ringDetMin)) {
            *tempRingingSt = VP_CSLAC_RINGING;
        } else {
            /*
             * This prevents compiler warning because it forces
             * the value to an initialized state
             */
            *tempRingingSt =
                (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_RINGING);
        }
    } else {
        *tempRingingSt = 0;

        /* We were not ringing, so process for polrev event */
        if (!(pLineObj->lineState.condition & VP_CSLAC_RINGING)) {
            /*
             * Require a 5ms delay (plus LIU 2ms the debounce time)
             * from previous polrev to occur before allowing it to
             * be detected as "Not Ringing". This gives time for
             * most ringing signals to exceed the LIU threshold.
             */
            if ((pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev / 4) >= 5) {
                if (!(pLineObj->lineState.condition & VP_CSLAC_POLREV_REPORTED)) {
                    pLineObj->lineState.condition |= VP_CSLAC_POLREV_REPORTED;

                    /*
                     * Based on how Ringing behaves, we could get out
                     * of sync w.r.t., PolRev. So don't report an event
                     * unless the PolRev changed.
                     */
                    if (pLineObj->lineState.condition & VP_CSLAC_POLREV) {
                        if (pLineObj->fxoData != VP_POLREV_REVERSE) {
                            pLineObj->fxoData = VP_POLREV_REVERSE;
                            pLineObj->lineEventHandle = pDevObj->timeStamp;
                            pLineObj->lineEvents.fxo |= VP_LINE_EVID_POLREV;
                            *retFlag = TRUE;
                            pLineObj->preRingPolRev = VP_POLREV_REVERSE;
                        }
                    } else {
                        if (pLineObj->fxoData != VP_POLREV_NORMAL) {
                            pLineObj->fxoData = VP_POLREV_NORMAL;
                            pLineObj->lineEventHandle = pDevObj->timeStamp;
                            pLineObj->lineEvents.fxo |= VP_LINE_EVID_POLREV;
                            pLineObj->preRingPolRev = VP_POLREV_NORMAL;
                            *retFlag = TRUE;
                        }
                    }
                }
            }
        }
    }
}

/**
 * Vp880ProcessFxoLine()
 *  This function should only be called by Vp880ServiceInterrupts on FXO lines.
 * It performs all line processing for operations that are Tick based
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated.
 */
bool
Vp880ProcessFxoLine(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    bool retFlag = FALSE;
    VpCslacLineCondType tempRingingSt, tempDiscSt, tempPolRevSt, tempLiu;
    uint8 channelId = pLineObj->channelId, discMask = 0;
    VpLineStateType currentState = pLineObj->lineState.currentState;

    /*
     * Ignore the detector for a period after the last hook state change, or
     * a longer period after the last hook state change AND if the previous
     * line condition was Ringing
     */
    if ((pLineObj->lineTimers.timers.fxoTimer.lastStateChange < VP_FXO_STATE_CHANGE_DEBOUNCE) ||
        ((pLineObj->lineTimers.timers.fxoTimer.lastStateChange < VP_FXO_RING_TRIP_DEBOUNCE)
      && (pLineObj->lineState.condition & VP_CSLAC_RINGING))
#ifdef VP_CSLAC_SEQ_EN
      || ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)
       && (pLineObj->intSequence[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_MOMENTARY_LOOP_OPEN_INT))
#endif
            ) {
        /* Middle of detector Mask. Skip this process */
    } else {
        if ((pLineObj->termType == VP_TERM_FXO_DISC)
        && ((currentState == VP_LINE_FXO_TALK) || (currentState == VP_LINE_FXO_LOOP_CLOSE))) {
            discMask = pDevObj->intReg[channelId] & VP880_IO2_1_MASK;
        } else {
            discMask = pDevObj->intReg[channelId] & VP880_DISC1_MASK;
        }

        if (discMask) {
            tempDiscSt = VP_CSLAC_DISC;
        } else {
            tempDiscSt = VP_CSLAC_STATUS_INVALID;
        }

        if (pDevObj->intReg[channelId] & VP880_POL1_MASK) {
            tempPolRevSt = VP_CSLAC_POLREV;
        } else {
            tempPolRevSt = VP_CSLAC_STATUS_INVALID;
        }

        if (pLineObj->ringDetMax >= VP880_MAX_RING_DET_PERIOD) {
            Vp880LowRingFreqDetect(pLineCtx, &tempRingingSt, &tempPolRevSt,
                &retFlag);
        } else {
            if (pDevObj->intReg[channelId] & VP880_LIU1_MASK) {
                tempLiu = VP_CSLAC_LIU;
            } else {
                tempLiu = VP_CSLAC_STATUS_INVALID;
            }

            if (tempLiu != (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_LIU)) {
                pLineObj->lineState.condition &= ~VP_CSLAC_LIU;
                pLineObj->lineState.condition |= tempLiu;

                pLineObj->lineEventHandle = pDevObj->timeStamp;
                pLineObj->lineEvents.fxo |=
                    ((tempLiu) ? VP_LINE_EVID_LIU : VP_LINE_EVID_LNIU);
                retFlag = TRUE;
            }

            if (pDevObj->intReg[channelId] & VP880_RING1_DET_MASK) {
                tempRingingSt = VP_CSLAC_RINGING;
            } else {
                tempRingingSt = VP_CSLAC_STATUS_INVALID;
            }

            if (tempPolRevSt !=
                (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_POLREV)) {
                pLineObj->lineState.condition &= ~VP_CSLAC_POLREV;
                pLineObj->lineState.condition |= tempPolRevSt;
                pLineObj->lineState.condition &= ~VP_CSLAC_POLREV_REPORTED;

                if ((tempRingingSt != VP_CSLAC_RINGING)
                 && ((pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev / 4)
                    > pLineObj->ringDetMax)) {
                    pLineObj->lineEventHandle = pDevObj->timeStamp;
                    pLineObj->lineEvents.fxo |= VP_LINE_EVID_POLREV;
                    retFlag = TRUE;
                    pLineObj->preRingPolRev =
                        (pLineObj->lineState.condition & VP_CSLAC_POLREV) ?
                        VP_POLREV_REVERSE : VP_POLREV_NORMAL;
                }

                pLineObj->lineTimers.timers.fxoTimer.timeLastPolRev = 0;

                if (pLineObj->lineState.condition & VP_CSLAC_POLREV) {
                    pLineObj->fxoData = VP_POLREV_REVERSE;
                } else {
                    pLineObj->fxoData = VP_POLREV_NORMAL;
                }
            }
        }

        /*
         * Our cached state is inconsistent with recently detected conditions.
         * Generate the event.
         */
        if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_RINGING) != tempRingingSt) {
            pLineObj->lineEventHandle = pDevObj->timeStamp;

            if (tempRingingSt == VP_CSLAC_RINGING) {
                pLineObj->lineEvents.fxo |= VP_LINE_EVID_RING_ON;
                pLineObj->lineState.condition |= VP_CSLAC_RINGING;
            } else {
                pLineObj->lineEvents.fxo |= VP_LINE_EVID_RING_OFF;
                pLineObj->lineState.condition &= ~(VP_CSLAC_RINGING);

                if (pLineObj->preRingPolRev != pLineObj->fxoData) {
                    pLineObj->preRingPolRev = pLineObj->fxoData;
                    pLineObj->lineEvents.fxo |= VP_LINE_EVID_POLREV;
                }
            }
            retFlag = TRUE;
        }

         /* If the feed conditions changed, continue line processing */
        if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_DISC) != tempDiscSt) {
            /*
             * Update actual line condition, even if not reporting an
             * event
             */
            if (tempDiscSt == VP_CSLAC_DISC) {
                pLineObj->lineState.condition |= VP_CSLAC_DISC;
            } else {
                pLineObj->lineState.condition &= ~(VP_CSLAC_DISC);
            }

            /*
             * If line has been stable, update the pre-Disconnect value that
             * is used at end of timer to determine if event should be
             * generated.
             */
            if (pLineObj->lineTimers.timers.fxoTimer.disconnectDebounce == 0) {
                pLineObj->preDisconnect = tempDiscSt;
            }

            /* Line status changed. Therefore, reset timer */
            pLineObj->lineTimers.timers.fxoTimer.disconnectDebounce =
                VP_FXO_DISCONNECT_DEBOUNCE;

            /*
             * Immediate disconnect changes detected do not result in
             * API-II event. So retFlag remains as set previously.
             */
        } else {
            /*
             * If the disconnect signal came back to the current state, stop
             * the debounce count
             */
            if (pLineObj->termType == VP_TERM_FXO_DISC) {
                pLineObj->lineTimers.timers.fxoTimer.noCount = TRUE;
                pLineObj->lineTimers.timers.fxoTimer.fxoDiscIO2Change = 0;
            }
        }
    } /* end interrupt detect */
    return retFlag;
} /* end Vp880ProcessFxoLine */
/* end of VP880_CLARE_RINGING_DETECT */
#else

/**
 * Vp880ProcessFxoLine()
 *  This function should only be called by Vp880ServiceInterrupts on FXO lines.
 * It performs all line processing for operations that are Tick based
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated.
 */
bool
Vp880ProcessFxoLine(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    bool retFlag = FALSE;
    VpCslacLineCondType tempRingingSt, tempDiscSt, tempPolRevSt, tempLiu;
    uint8 channelId = pLineObj->channelId, discMask = 0;
    VpLineStateType currentState = pLineObj->lineState.currentState;
    VpFXOTimerType *pFxoTimers = &pLineObj->lineTimers.timers.fxoTimer;

    /*
     * Ignore the detector for a period after the last hook state change, or
     * a longer period after the last hook state change AND if the previous
     * line condition was Ringing
     */
    if ((pFxoTimers->lastStateChange < VP_FXO_STATE_CHANGE_DEBOUNCE) ||
        ((pFxoTimers->lastStateChange < VP_FXO_RING_TRIP_DEBOUNCE)
      && (pLineObj->lineState.condition & VP_CSLAC_RINGING))
#ifdef VP_CSLAC_SEQ_EN
      || ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)
       && (pLineObj->intSequence[VP_PROFILE_TYPE_LSB]
            == VP_PRFWZ_PROFILE_MOMENTARY_LOOP_OPEN_INT))
#endif
            ) {
        /* Middle of detector Mask. Skip this process */
    } else {
        if ((pLineObj->termType == VP_TERM_FXO_DISC)
        && ((currentState == VP_LINE_FXO_TALK)
         || (currentState == VP_LINE_FXO_LOOP_CLOSE))) {
            discMask = pDevObj->intReg[channelId] & VP880_IO2_1_MASK;
        } else {
            discMask = pDevObj->intReg[channelId] & VP880_DISC1_MASK;
        }

        tempDiscSt = ((discMask != 0) ? VP_CSLAC_RAW_DISC : VP_CSLAC_STATUS_INVALID);

        tempPolRevSt = ((pDevObj->intReg[channelId] & VP880_POL1_MASK)
                        ? VP_CSLAC_POLREV : VP_CSLAC_STATUS_INVALID);
        tempRingingSt = ((pDevObj->intReg[channelId] & VP880_RING1_DET_MASK)
                        ? VP_CSLAC_RINGING : VP_CSLAC_STATUS_INVALID);

        /*
         * Ringing Detector is unstable in presence of Disconnect, and the
         * Disconnect is filtered for generally 28-40ms. So if currently
         * detecting Disconnect or just "recently" stopped detecting Disconnect,
         * prevent Ringing events and Ring_On status.
         */
        if ((pLineObj->lineState.condition & (VP_CSLAC_LIU | VP_CSLAC_RAW_DISC | VP_CSLAC_DISC)) ||
            (pFxoTimers->disconnectDebounce > 0)) {
            tempRingingSt = VP_CSLAC_STATUS_INVALID;
        }

        if ((currentState == VP_LINE_FXO_TALK)
         || (currentState == VP_LINE_FXO_LOOP_CLOSE)) {
            tempLiu = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_LIU);
        } else {
            tempLiu = (pDevObj->intReg[channelId] & VP880_LIU1_MASK)
                ? VP_CSLAC_LIU : VP_CSLAC_STATUS_INVALID;
        }

        /*
         * Multiple PolRevs can/will be detected prior to ringing but not after
         * (ringing "off" occurs when the line is stable). So we have to know
         * what the polarity was if a PolRev event was generated prior to
         * ringing, so we generate a corresponding correction event post ringing.
         */
        if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_RINGING) != tempRingingSt) {
            pLineObj->lineEventHandle = pDevObj->timeStamp;
            if (tempRingingSt == VP_CSLAC_RINGING) {
                pLineObj->lineEvents.fxo |= VP_LINE_EVID_RING_ON;
                pLineObj->lineState.condition |= VP_CSLAC_RINGING;
            } else {
                pLineObj->lineEvents.fxo |= VP_LINE_EVID_RING_OFF;
                pLineObj->lineState.condition &= ~(VP_CSLAC_RINGING);

                VP_FXO_FUNC(VpLineCtxType, pLineCtx, ("Ringing Stop. Pre-Ring PolRev %d Current %d",
                    pLineObj->preRingPolRev, tempPolRevSt));

                if (((pLineObj->preRingPolRev == VP_POLREV_NORMAL) && (tempPolRevSt))
                || ((pLineObj->preRingPolRev == VP_POLREV_REVERSE) && (!(tempPolRevSt)))) {
                    /*
                     * PolRev is out of sync due to transitions prior to ringing
                     * detect. Correct it.
                     */
                    tempPolRevSt = ((tempPolRevSt == VP_CSLAC_POLREV)
                                    ? VP_CSLAC_STATUS_INVALID : VP_CSLAC_POLREV);
                }
            }
            retFlag = TRUE;
        }

        if (pLineObj->lineState.condition & VP_CSLAC_RINGING) {
            /* Not PolRev events if ringing is detected. */
            tempPolRevSt = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_POLREV);
        }

        if ((tempPolRevSt != (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_POLREV))) {
            pLineObj->lineState.condition &= ~VP_CSLAC_POLREV;
            pLineObj->lineState.condition |= tempPolRevSt;
            pLineObj->lineState.condition &= ~VP_CSLAC_POLREV_REPORTED;

            if ((pFxoTimers->timeLastPolRev / 4) > pLineObj->ringDetMax) {
                pLineObj->lineEventHandle = pDevObj->timeStamp;
                pLineObj->lineEvents.fxo |= VP_LINE_EVID_POLREV;
                retFlag = TRUE;
                pLineObj->preRingPolRev =
                    (pLineObj->lineState.condition & VP_CSLAC_POLREV) ?
                    VP_POLREV_REVERSE : VP_POLREV_NORMAL;
            }

            pFxoTimers->timeLastPolRev = 0;

            if (pLineObj->lineState.condition & VP_CSLAC_POLREV) {
                pLineObj->fxoData = VP_POLREV_REVERSE;
            } else {
                pLineObj->fxoData = VP_POLREV_NORMAL;
            }
        }

        if (tempLiu != (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_LIU)) {
            VP_FXO_FUNC(VpLineCtxType, pLineCtx, ("LIU Change to %d at time %d",
                tempLiu, pDevObj->timeStamp));

            pLineObj->lineState.condition &= ~VP_CSLAC_LIU;
            pLineObj->lineState.condition |= tempLiu;

            pLineObj->lineEventHandle = pDevObj->timeStamp;
            pLineObj->lineEvents.fxo |=
                ((tempLiu == VP_CSLAC_LIU) ? VP_LINE_EVID_LIU : VP_LINE_EVID_LNIU);

            if (tempLiu == VP_CSLAC_STATUS_INVALID) {
                /*
                 * Line status changed. Therefore, reset timer to prevent
                 * Ringing events.
                 */
                pFxoTimers->disconnectDebounce = VP_FXO_DISCONNECT_DEBOUNCE;
            }

            retFlag = TRUE;
        }

        /* If the feed conditions changed, continue line processing */
        if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_RAW_DISC) != tempDiscSt) {
            VP_FXO_FUNC(VpLineCtxType, pLineCtx, ("RAW_DISC Change to %d at time %d",
                tempDiscSt, pDevObj->timeStamp));
            /*
             * Update actual line condition, even if not reporting an
             * event
             */
            if (tempDiscSt == VP_CSLAC_RAW_DISC) {
                pLineObj->lineState.condition |= VP_CSLAC_RAW_DISC;
            } else {
                pLineObj->lineState.condition &= ~VP_CSLAC_RAW_DISC;
            }

            /*
             * Line status changed. Therefore, reset timer and state we think
             * the line is in. This is resolved when timer expires.
             */
            pFxoTimers->disconnectDebounce = VP_FXO_DISCONNECT_DEBOUNCE;
            pLineObj->preDisconnect = tempDiscSt;

            /*
             * Immediate disconnect changes detected do not result in
             * API-II event. So retFlag remains as set previously.
             */
        } else {
            /*
             * If the disconnect signal came back to the current state, stop
             * the debounce count
             */
            if (pLineObj->termType == VP_TERM_FXO_DISC) {
                pFxoTimers->noCount = TRUE;
                pFxoTimers->fxoDiscIO2Change = 0;
            }
        }
    } /* end interrupt detect */
    return retFlag;
} /* end Vp880ProcessFxoLine */
#endif

#endif
