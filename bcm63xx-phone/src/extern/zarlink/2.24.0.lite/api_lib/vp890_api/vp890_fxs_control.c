/** \file vp890_fxs_control.c
 * vp890_fxs_control.c
 *
 *  This file contains the implementation of the VP-API 890 Series Control Functions for FXS lines.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11489 $
 * $LastChangedDate: 2014-07-14 17:28:31 -0500 (Mon, 14 Jul 2014) $
 */

/* INCLUDES */
#include    "vp_api.h"

#if defined (VP_CC_890_SERIES) && defined (VP890_FXS_SUPPORT)

#include    "vp_api_int.h"
#include    "vp890_api_int.h"

/* =================================
    Prototypes for Static Functions
   ================================= */
static bool
Vp890SetStateRinging(
    VpLineCtxType           *pLineCtx,
    VpLineStateType         state);

static void
Vp890GroundStartProc(
    VpLineCtxType *pLineCtx,
    uint8 currentLineState,
    uint8 userByte);

static void
Vp890ServiceDiscExitTimer(
    VpLineCtxType *pLineCtx);

static void
Vp890ApplyInternalTestTerm(
    VpLineCtxType *pLineCtx);

static void
Vp890RemoveInternalTestTerm(
    VpLineCtxType *pLineCtx);

/*******************************************************************************
 * Vp890ProcessFxsLine()
 *  This function should only be called by Vp890ApiTick on FXS lines. It
 * performs all line processing for operations that are Tick based
 *
 * Arguments:
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated.
 ******************************************************************************/
void
Vp890ProcessFxsLine(
    Vp890DeviceObjectType   *pDevObj,
    VpLineCtxType           *pLineCtx)
{
    Vp890LineObjectType         *pLineObj   = pLineCtx->pLineObj;
    bool                        dpStatus[2] = {FALSE, FALSE};
    VpOptionEventMaskType       lineEvents1;
    VpOptionEventMaskType       lineEvents2;
    VpDialPulseDetectStatesType beforeState, afterState;

    uint8 hookStatus = 0, i, validSamples;
    uint8 hookIncrement;
    bool dp2Valid;

    /* Skip processing during line test */
    if (Vp890IsChnlUndrTst(pDevObj, pLineObj->channelId) == TRUE) {
        return;
    }

    lineEvents1.signaling = 0;
    lineEvents2.signaling = 0;

    /* If the secondary pulse params are all 0 (default), mark them as invalid
     * so that they will not be used. */
    if (pDevObj->pulseSpecs2.breakMin == 0 &&
        pDevObj->pulseSpecs2.breakMax == 0 &&
        pDevObj->pulseSpecs2.makeMin == 0 &&
        pDevObj->pulseSpecs2.makeMax == 0 &&
#ifdef EXTENDED_FLASH_HOOK
        pDevObj->pulseSpecs2.onHookMin == 0 &&
#endif
        pDevObj->pulseSpecs2.interDigitMin == 0 &&
        pDevObj->pulseSpecs2.flashMin == 0 &&
        pDevObj->pulseSpecs2.flashMax == 0)
    {
        dp2Valid = FALSE;
    } else {
        dp2Valid = TRUE;
    }

    /*
     * If the line is configured for Dial Pulse Detection, run the Dial Pulse
     * detection code. Dial Pulse detection code will generate the appropriate
     * events
     */
    if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) {
        /*
         * Based on the following (vp890_api_int.h):
         *
         *  #define VP890_CC_500HZ_RATE     0x40
         *  #define VP890_CC_1KHZ_RATE      0x30
         *  #define VP890_CC_2KHZ_RATE      0x20
         *  #define VP890_CC_4KHZ_RATE      0x10
         *  #define VP890_CC_8KHZ_RATE      0x00
         */
        uint8 shiftAmt = ((pDevObj->txBufferDataRate >> 4) & 0x7);
        hookIncrement = (1 << shiftAmt);

        validSamples = ((pDevObj->testDataBuffer[VP890_TEST_DATA_LEN_INDEX]
            & VP890_TEST_DATA_LEN_MASK) >> 4);

        if (validSamples == 7) {
            validSamples = 6;
        }

        hookStatus = pDevObj->testDataBuffer[VP890_TEST_DATA_HK_INDEX]
            & VP890_TEST_DATA_HK_MASK;

        if (pLineObj->status & VP890_LOW_POWER_EN) {
            hookStatus = ~hookStatus;
        }

        beforeState = pLineObj->dpStruct.state;

        /* Adjust the end of the loop open or closed period if the test buffer
         * has been updated */
        if (pDevObj->state & VP_DEV_TEST_BUFFER_READ) {
            /*
            VP_HOOK(VpLineCtxType, pLineCtx, ("CH%d Validsamples %d, buffer %02X %02X %d%d%d%d%d%d\n",
                channelId,
                ((pDevObj->testDataBuffer[VP890_TEST_DATA_LEN_INDEX] & VP890_TEST_DATA_LEN_MASK) >> 4),
                pDevObj->txBuffer[0],
                pDevObj->txBuffer[1],
                (hookStatus & 0x20) == 0x20,
                (hookStatus & 0x10) == 0x10,
                (hookStatus & 0x08) == 0x08,
                (hookStatus & 0x04) == 0x04,
                (hookStatus & 0x02) == 0x02,
                (hookStatus & 0x01) == 0x01));
            */
            for (i = 1; i < (1 << validSamples); i <<= 1) {
                if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE) {
                    if (!(hookStatus & i)) {
                        if (pLineObj->dpStruct.lc_time > hookIncrement) {
                            pLineObj->dpStruct.lc_time -= hookIncrement;
                        }
                        if (pLineObj->dpStruct2.lc_time > hookIncrement) {
                            pLineObj->dpStruct2.lc_time -= hookIncrement;
                        }
                    } else {
                        break;
                    }
                } else if (beforeState == VP_DP_DETECT_STATE_LOOP_OPEN) {
                    if (hookStatus & i) {
                        if (pLineObj->dpStruct.lo_time > hookIncrement) {
                            pLineObj->dpStruct.lo_time -= hookIncrement;
                        }
                        if (pLineObj->dpStruct2.lo_time > hookIncrement) {
                            pLineObj->dpStruct2.lo_time -= hookIncrement;
                        }
                    } else {
                        break;
                    }
                }
            }
        }

        /*
         * Compensate for slow onhook detection when using slow external supply. Not 100% sure
         * technically, but we have to assume it when using Fixed-Ringing. This does mean that
         * fast supplies (tracking) specifying a Fixed Ringing mode will have Dial Pulse Detect
         * issues.
         */
        if (pDevObj->swParamsCache[VP890_REGULATOR_TRACK_INDEX] & VP890_REGULATOR_FIXED_RING) {
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct.hookSt)) {
                if (pLineObj->dpStruct.lc_time > VP890_PULSE_DETECT_ADJUSTMENT_LONG) {
                    pLineObj->dpStruct.lc_time -= VP890_PULSE_DETECT_ADJUSTMENT_LONG;
                }
            }
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct2.hookSt)) {
                if (pLineObj->dpStruct2.lc_time > VP890_PULSE_DETECT_ADJUSTMENT_LONG) {
                    pLineObj->dpStruct2.lc_time -= VP890_PULSE_DETECT_ADJUSTMENT_LONG;
                }
            }
        } else {
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct.hookSt)) {
                if (pLineObj->dpStruct.lc_time > VP890_PULSE_DETECT_ADJUSTMENT_SHORT) {
                    pLineObj->dpStruct.lc_time -= VP890_PULSE_DETECT_ADJUSTMENT_SHORT;
                }
            }
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct2.hookSt)) {
                if (pLineObj->dpStruct2.lc_time > VP890_PULSE_DETECT_ADJUSTMENT_SHORT) {
                    pLineObj->dpStruct2.lc_time -= VP890_PULSE_DETECT_ADJUSTMENT_SHORT;
                }
            }
        }

        dpStatus[0] = VpUpdateDP(pDevObj->devProfileData.tickRate,
            &pDevObj->pulseSpecs, &pLineObj->dpStruct, &lineEvents1);
        if (dp2Valid == TRUE) {
            dpStatus[1] = VpUpdateDP(pDevObj->devProfileData.tickRate,
                &pDevObj->pulseSpecs2, &pLineObj->dpStruct2, &lineEvents2);
        }
        afterState = pLineObj->dpStruct.state;
        /* Update the loop open and close times according to the hook change
         * within a tick */

        /* If the state changed, adjust the hook timings */
        if (beforeState != afterState) {
            if (pDevObj->state & VP_DEV_TEST_BUFFER_READ) {
                for (i = 1; i < (1 << validSamples); i <<= 1) {
                    if (afterState == VP_DP_DETECT_STATE_LOOP_CLOSE) {
                        if (hookStatus & i) {
                            pLineObj->dpStruct.lc_time += hookIncrement;
                            pLineObj->dpStruct2.lc_time += hookIncrement;
                        } else {
                            break;
                        }
                    } else if (afterState == VP_DP_DETECT_STATE_LOOP_OPEN) {
                        if (!(hookStatus & i)) {
                            pLineObj->dpStruct.lo_time += hookIncrement;
                            pLineObj->dpStruct2.lo_time += hookIncrement;
                        } else {
                            break;
                        }
                    }
                }
            }
            /*
             * Compensate for slow onhook detection when using slow external supply. Not 100% sure
             * technically, but we have to assume it when using Fixed-Ringing. This does mean that
             * fast supplies (tracking) specifying a Fixed Ringing mode will have Dial Pulse Detect
             * issues.
             */
            if (afterState == VP_DP_DETECT_STATE_LOOP_OPEN) {
                if (pDevObj->swParamsCache[VP890_REGULATOR_TRACK_INDEX] & VP890_REGULATOR_FIXED_RING) {
                    pLineObj->dpStruct.lo_time += VP890_PULSE_DETECT_ADJUSTMENT_LONG;
                    pLineObj->dpStruct2.lo_time += VP890_PULSE_DETECT_ADJUSTMENT_LONG;
                } else {
                    pLineObj->dpStruct.lo_time += VP890_PULSE_DETECT_ADJUSTMENT_SHORT;
                    pLineObj->dpStruct2.lo_time += VP890_PULSE_DETECT_ADJUSTMENT_SHORT;
                }
            }
        }

        /*
         * The state machines will not necessarily complete at the same time, so
         * keep track of each and when both are done, report a passing digit if
         * one exists, or invalid if no criteria was met.
         */
        if (dpStatus[0] == TRUE) {
            pLineObj->signaling1 = lineEvents1.signaling;
            pLineObj->lineEventHandle = pDevObj->timeStamp;

            if (!(pLineObj->lineEvents.signaling & VP_LINE_EVID_BREAK_MAX)) {
                pLineObj->status |= VP890_DP_SET1_DONE;
            }
        }

        if (dpStatus[1] == TRUE && dp2Valid == TRUE) {
            pLineObj->signaling2 = lineEvents2.signaling;
            pLineObj->lineEventHandle = pDevObj->timeStamp;

            if (!(pLineObj->lineEvents.signaling & VP_LINE_EVID_BREAK_MAX)) {
                pLineObj->status |= VP890_DP_SET2_DONE;
            }
        }

        /* Report events if:
         *  Both DP sets are done, OR
         *  Set 1 is done and set 2 is invalid */
        if ((pLineObj->status & VP890_DP_SET1_DONE) &&
            ((pLineObj->status & VP890_DP_SET2_DONE) ||
             dp2Valid == FALSE))
        {
            /* Use the results of DP set 1 if it detected a valid digit, or
             * if DP set 2 detected an invalid digit, or if set 2 is disabled */
            if (pLineObj->dpStruct.digits != -1 ||
                pLineObj->dpStruct2.digits == -1 ||
                dp2Valid == FALSE)
            {
                pLineObj->signalingData = pLineObj->dpStruct.digits;
                pLineObj->lineEvents.signaling |= pLineObj->signaling1;
                pLineObj->lineEventHandle = VP_DP_PARAM1;
            } else {
                pLineObj->signalingData = pLineObj->dpStruct2.digits;
                pLineObj->lineEvents.signaling |= pLineObj->signaling2;
                pLineObj->lineEventHandle = VP_DP_PARAM2;
            }

            if (pLineObj->signalingData == 0) {
                pLineObj->signalingData = pLineObj->lineEventHandle;
                pLineObj->lineEventHandle = pDevObj->timeStamp;
            }

            pLineObj->status &= ~(VP890_DP_SET1_DONE | VP890_DP_SET2_DONE);
            pLineObj->signaling1 = 0;
            pLineObj->signaling2 = 0;
        }
    }

#ifdef VP_CSLAC_SEQ_EN
    /*
     * If Caller ID sequencer is in progress, update unless it's in a state of
     * suspension. If suspended, re-enable if device is in underrun (no more
     * data to transmit).
     */
    if ((pLineObj->callerId.status & VP_CID_IN_PROGRESS)
     || (pLineObj->suspendCid == TRUE)) {
        VpDeviceIdType  deviceId    = pDevObj->deviceId;
        uint8           ecVal       = pLineObj->ecVal;

        /* First read of the CID State Machine this tick. */
        VpMpiCmdWrapper(deviceId, ecVal, VP890_CID_PARAM_RD,
            VP890_CID_PARAM_LEN, pLineObj->tickBeginState);
        pLineObj->delayConsumed = FALSE;

        if (pLineObj->suspendCid == TRUE) {
            /*
             * Check to see if the Device Buffer is empty. If it is, continue
             * with CID.
             */
            uint8 cidState = (pLineObj->tickBeginState[0] & VP890_CID_STATE_MASK);

            if ((cidState == VP890_CID_STATE_URUN)
             || (cidState == VP890_CID_STATE_IDLE)) {
                uint8 cidParam = pLineObj->tickBeginState[0];

                pLineObj->suspendCid = FALSE;
                cidParam &= ~(VP890_CID_FRAME_BITS);
                cidParam |= VP890_CID_DIS;

                Vp890MuteChannel(pLineCtx, FALSE);
                if (cidParam != pLineObj->tickBeginState[0]) {
                    pLineObj->tickBeginState[0] = cidParam;
                    VP_CID(VpLineCtxType, pLineCtx,
                        ("Stopping FSK Generator with 0x%02X to CID Params at time %d",
                        pLineObj->tickBeginState[0], pDevObj->timeStamp));
                    VpMpiCmdWrapper(deviceId, ecVal, VP890_CID_PARAM_WRT,
                        VP890_CID_PARAM_LEN, pLineObj->tickBeginState);
                    pLineObj->callerId.status &= ~VP_CID_FSK_ACTIVE;
                }
            }
        }

        if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
            VpCidSeq(pLineCtx);
        }
    }

#endif

    return;
} /* Vp890ProcessFxsLine() */

/*******************************************************************************
 * Vp890ServiceFxsInterrupts()
 *  This function should only be called by Vp890ApiTick when an interrupt
 * occurs.
 *
 * Preconditions:
 *  The device must first be initialized.
 *
 * Postconditions:
 *  The Global Signaling Register is read and the data is stored in the device
 * object.  Depending on the dial pulse mode option set, the hook event (on/off)
 * is generated if a hook status changed.  This function will return
 * TRUE if an event has been generated.
 ******************************************************************************/
void
Vp890ServiceFxsInterrupts(
    VpLineCtxType         *pLineCtx)
{
    VpDevCtxType          *pDevCtx  = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    Vp890LineObjectType   *pLineObj = pLineCtx->pLineObj;

#ifdef VP_CSLAC_SEQ_EN
    VpProfilePtrType      pCadence;
#endif
    uint16                *pTimerAry = pLineObj->lineTimers.timers.timer;
    uint8                 channelId = pLineObj->channelId;
    VpLineStateType       state     = pLineObj->lineState.currentState;
    VpLineStateType       usrState  = pLineObj->lineState.usrCurrent;

    VpCslacLineCondType   tempHookSt, tempGnkSt, tempThermFault;
    uint8                 loopSupervision[VP890_LOOP_SUP_LEN];
    VpDeviceIdType        deviceId  = pDevObj->deviceId;
    uint8                 ecVal     = pLineObj->ecVal;

    /*
     * Ring Trip Flag set to TRUE if either Hook or Ground Key transitioned to
     * active condition this tick. Used to process ringTripExitState
     */
    bool                  ringTrip  = FALSE;

    /* If debouncing for Ring Exit or Caller ID, ignore hook.
     * Otherwise process. */
    if (VpCSLACHookMaskEnabled(pTimerAry)
     || (pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE] & VP_ACTIVATE_TIMER)
     || (pDevObj->state & VP_DEV_IN_CAL)
#ifdef VP_CSLAC_SEQ_EN
     || ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)
     && (pLineObj->intSequence[VP_PROFILE_TYPE_LSB] ==  VP_PRFWZ_PROFILE_FWD_DISC_INT))

     || ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)
     && (pLineObj->intSequence[VP_PROFILE_TYPE_LSB] ==  VP_PRFWZ_PROFILE_TIP_OPEN_INT))
#endif
     || (pLineObj->lineState.calType != VP_CSLAC_CAL_NONE)
     || (state == VP_LINE_DISCONNECT)) {
        tempHookSt = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK);
    } else {
        if (pLineObj->status & VP890_LOW_POWER_EN) {
            if (pDevObj->intReg[channelId] & VP890_HOOK_MASK) {
                tempHookSt = VP_CSLAC_STATUS_INVALID;
            } else {
                tempHookSt = VP_CSLAC_HOOK;
            }
        } else {
            if (pDevObj->intReg[channelId] & VP890_HOOK_MASK) {
                tempHookSt = VP_CSLAC_HOOK;
            } else {
                tempHookSt = VP_CSLAC_STATUS_INVALID;
            }
        }
    }

    if (pDevObj->intReg[channelId] & VP890_TEMPA_MASK) {
        tempThermFault = VP_CSLAC_THERM_FLT;
    } else {
        tempThermFault = VP_CSLAC_STATUS_INVALID;
    }

    if ((pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE] & VP_ACTIVATE_TIMER)
     || (pDevObj->state & VP_DEV_IN_CAL)
     || (state == VP_LINE_DISCONNECT)
     || (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT]
         & VP_ACTIVATE_TIMER)) {
        tempGnkSt = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_GKEY);
    } else {
        if (pDevObj->intReg[channelId] & VP890_GNK_MASK) {
            tempGnkSt = VP_CSLAC_GKEY;
        } else {
            tempGnkSt = VP_CSLAC_STATUS_INVALID;
        }
    }

    /* If the hook conditions changed, continue line processing */
    if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK) != tempHookSt) {
        pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
        pLineObj->lineState.condition |= tempHookSt;
        VpMemCpy(loopSupervision, pLineObj->loopSup, VP890_LOOP_SUP_LEN);
        if ((pLineObj->status & VP890_LOW_POWER_EN) && (tempHookSt == VP_CSLAC_HOOK)) {
            /*
             * Line is currently in a Low Power Mode condition with off-hook
             * detect. This can only occur for valid off-hook OR for a leaky
             * line (< 400kohm). Don't update the externally visible off-hook
             * status or report off-hook event. Simply NOT generating the event
             * is easy. VpGetLineStatus() uses a combination of VP_CSLAC_HOOK
             * and VP_CSLAC_LINE_LEAK_TEST to determine the off-hook condition
             * to be reported. While the leaky line test is in progress only
             * on-hook is reported. It's important that the "LINE_LEAK.." flag
             * gets cleared when the test is complete. If not, VpGetLineStatus()
             * will always report on-hook.
             */
            VP_HOOK(VpLineCtxType, pLineCtx,
                ("Off-Hook Detected in Low Power Mode on line %d time %d in User State %d",
                channelId, pDevObj->timeStamp, usrState));

            if ((loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] & VP890_SWHOOK_THRESH_MASK)
                >= pLineObj->hookHysteresis) {
                loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] -= pLineObj->hookHysteresis;
            } else {
                loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] &= ~(VP890_SWHOOK_THRESH_MASK);
            }

            /* Force line to feed state and start leaky line detection */
            pLineObj->lineState.currentState = VP_LINE_OHT;
            pDevObj->stateInt &= ~(VP890_LINE0_LP | VP890_LINE1_LP);

            pLineObj->lineState.condition |= VP_CSLAC_LINE_LEAK_TEST;
        } else {

#ifdef VP_CSLAC_SEQ_EN
            /*
             * There was a sufficient hook activity to stop the active
             * CID -- unless the CID sequence knew this would happen and
             * set the debounce flag. In which case, let CID continue.
             */
            if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
                if (pLineObj->callerId.status & VP_CID_IS_DEBOUNCE) {
                    /* Hook event is fully debounced and ready to go */
                    pLineObj->callerId.status &= ~VP_CID_IS_DEBOUNCE;
                } else {
                    VpCliStopCli(pLineCtx);
                    Vp890SetLineTone(pLineCtx, VP_PTABLE_NULL, VP_PTABLE_NULL, VP_NULL);
                }
            }
#endif
            /*
             * Read the test buffer IF it was not yet read this tick AND we're
             * running Dial Pulse Detection AND we're not in LPM (i.e., if
             * Dial Pulse Detection is not occurring).
             */
            if ((pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) &&
                (!(pDevObj->state & VP_DEV_TEST_BUFFER_READ)) &&
                (!(pLineObj->status & VP890_LOW_POWER_EN))) {
                pDevObj->state |= VP_DEV_TEST_BUFFER_READ;

                /* Collect data from the device test data buffer */
                VpMpiCmdWrapper(deviceId, ecVal, VP890_TEST_DATA_RD,
                    VP890_TEST_DATA_LEN, pDevObj->testDataBuffer);
            }

            if (tempHookSt == VP_CSLAC_HOOK) {
                VP_HOOK(VpLineCtxType, pLineCtx, ("Off-Hook on Line %d at Time %d Low Power 0x%02X",
                    channelId, pDevObj->timeStamp, (pLineObj->status & VP890_LOW_POWER_EN)));
                if ((loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] & VP890_SWHOOK_THRESH_MASK)
                    >= pLineObj->hookHysteresis) {
                    loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] -= pLineObj->hookHysteresis;
                } else {
                    loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] &= ~(VP890_SWHOOK_THRESH_MASK);
                }

                pLineObj->dpStruct.hookSt = TRUE;
                pLineObj->dpStruct2.hookSt = TRUE;

#ifdef VP890_LP_SUPPORT
                pLineObj->leakyLineCnt = 0;
                pLineObj->status &= ~VP890_LINE_LEAK;
#endif

                if (pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
                    pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_OFF;
                    pLineObj->lineEventHandle = pDevObj->timeStamp;
                } else {
                    VP_HOOK(VpLineCtxType, pLineCtx,
                        ("Off-Hook on Line %d at Time %d Signaling 0x%02X 0x%02X User State %d",
                        channelId, pDevObj->timeStamp, pDevObj->intReg[0], pDevObj->intReg[1], usrState));
                }

#ifdef VP_CSLAC_SEQ_EN
                /*
                 * If an off-hook is detected when the active cadence
                 * is a Message Waiting Pulse on the line, restore the
                 * line state.
                 */
                pCadence = pLineObj->cadence.pActiveCadence;
                if (pCadence != VP_PTABLE_NULL) {
                    if (pCadence[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_MSG_WAIT_PULSE_INT) {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Channel %d Changing Line API State to %d at time %d for Message Waiting",
                            channelId, state, pDevObj->timeStamp));
                        Vp890SetFxsLineState(pLineCtx, state);
                    }
                }
#endif
                /*
                 * This is a valid off-hook, different from LPM possible leaky
                 * line issue. We may ring-trip, depending on other conditions.
                 */
                ringTrip = TRUE;
            } else {
                VP_HOOK(VpLineCtxType, pLineCtx, ("On-Hook on Line %d at Time %d",
                    channelId, pDevObj->timeStamp));
                /* Restore the on-hook current threshold */
                loopSupervision[VP890_LOOP_SUP_THRESH_BYTE] =
                    pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE];

                pLineObj->dpStruct.hookSt = FALSE;
                pLineObj->dpStruct2.hookSt = FALSE;

                if ((pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) &&
                    ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_LINE_LEAK_TEST)
                     == VP_CSLAC_STATUS_INVALID) &&
                    (!(pLineObj->status & VP890_LINE_LEAK))) {
                    /*
                     * If this is the first time after initialization
                     * that we are checking for on-hook and it is
                     * on-hook, don't generate an interrupt
                     */
                    if (!(pLineObj->lineState.condition & VP_CSLAC_STATUS_VALID)) {
                        pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_ON;
                        pLineObj->lineEventHandle = pDevObj->timeStamp;
                    }
                }

                if (VpIsLowPowerTermType(pLineObj->termType)) {
                    VP_HOOK(VpLineCtxType, pLineCtx, ("User State %d Current State %d",
                        pLineObj->lineState.usrCurrent, pLineObj->lineState.currentState));

                    if (pLineObj->lineState.usrCurrent == VP_LINE_STANDBY) {
                        pLineObj->lineState.currentState = VP_LINE_STANDBY;
                        Vp890LLSetSysState(deviceId, pLineCtx, 0x00, FALSE);
                    }
                }
            }
        }
        /*
         * If hysterisis is used, adjust hook threshold due to on or off-hook.
         * Note that this does not modify the cached value since the cached
         * value holds the Init/Config values provided by the application. This
         * method restores from those values when on-hook, and adjusts the
         * device only when off-hook.
         */
        if (pLineObj->hookHysteresis != 0) {
            VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOP_SUP_WRT,
                VP890_LOOP_SUP_LEN, loopSupervision);
        }
    }

    /* If the gkey conditions changed, continue line processing */
    if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_GKEY) != tempGnkSt) {

        if (tempGnkSt == VP_CSLAC_GKEY) {
            ringTrip = TRUE;
            pLineObj->lineEvents.signaling |= VP_LINE_EVID_GKEY_DET;
            pLineObj->lineState.condition |= VP_CSLAC_GKEY;
        } else {
            pLineObj->lineEvents.signaling |= VP_LINE_EVID_GKEY_REL;
            pLineObj->lineState.condition &= ~(VP_CSLAC_GKEY);
        }
        pLineObj->lineEventHandle = pDevObj->timeStamp;
    }

    /*
     * If an off-hook or ground key is detected during the user set state of
     * Ringing (incl. ringing and silent interval) while a test is running,
     *  don't allow the api to go to the ringtrip state. If not during a line
     * test, go to the ring trip exit state.
     */
    if ((ringTrip == TRUE)
     && (Vp890IsChnlUndrTst(pDevObj, channelId) == FALSE)
     && ((usrState == VP_LINE_RINGING) || (usrState == VP_LINE_RINGING_POLREV))) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Off-Hook During Ringing -- Going to State %d on Line %d at Time %d",
            pLineObj->ringCtrl.ringTripExitSt, channelId, pDevObj->timeStamp));
        Vp890SetLineState(pLineCtx, pLineObj->ringCtrl.ringTripExitSt);

        /*
         * Stop all currently running tones just in case tones were part of the ringing cadence.
         * Not typical, but possible. If we don't do this and tones are part of the ringing cadence,
         * then tones will remain enabled when a ring trip occurs.
         */
        Vp890SetLineTone(pLineCtx, VP_NULL, VP_NULL, VP_NULL);
    }
    
    /* 
     * only attempt to generate a thermal fault event if the thermal fault persists for more than 1 tick
     * However, always try to generate a thermal fault clear because it could clear at anytime.
     */
    if (  ((pLineObj->lineState.thermFltDebounce) && (pLineObj->lineState.thermFltDebounceTimestamp != pDevObj->timeStamp)) ||
           (tempThermFault != VP_CSLAC_THERM_FLT) ) {
        
        if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_THERM_FLT) != tempThermFault) {
            pLineObj->lineEventHandle = pDevObj->timeStamp;
            pLineObj->lineState.condition &= ~(VP_CSLAC_THERM_FLT);
            pLineObj->lineState.condition |= tempThermFault;

            pLineObj->lineEvents.faults |= VP_LINE_EVID_THERM_FLT;

            if (tempThermFault == VP_CSLAC_THERM_FLT) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Thermal Fault Event on Line %d at Time %d", channelId, pDevObj->timeStamp));
#ifdef VP890_INCLUDE_TESTLINE_CODE
                if((Vp890IsChnlUndrTst(pDevObj, channelId) == TRUE) ||
                   (pDevObj->currentTest.nonIntrusiveTest == TRUE)) {
                    pLineObj->lineEvents.test |= VP_LINE_EVID_ABORT;
                } else if (pDevObj->criticalFault.thermFltDiscEn == TRUE) {
#endif
                    Vp890SetLineState(pLineCtx, VP_LINE_DISCONNECT);
#ifdef VP890_INCLUDE_TESTLINE_CODE
                }
#endif
            }
        }
    }
    
    /* Need to debounce thermal faults by one tick */
    if (tempThermFault == VP_CSLAC_THERM_FLT) {
        pLineObj->lineState.thermFltDebounce = TRUE;
    } else {
        pLineObj->lineState.thermFltDebounce = FALSE;
    }
    
    pLineObj->lineState.thermFltDebounceTimestamp = pDevObj->timeStamp;

} /* Vp890ServiceFxsInterrupts() */

/*******************************************************************************
 * Vp890ServiceFxsTimers()
 * This function services FXS-specific line timers.
 *
 * Arguments:   *pDevCtx    -   Device context ptr
 *              *pLineCtx   -   Line context ptr
 *              *pLineObj   -   Line object ptr
 *              deviceId    -   User-defined deviceId
 *              ecVal       -   Enable Channel value including wideband info
 *
 * Preconditions:  Sould be called once per tick by Vp890ApiTick.
 *
 * Postconditions: FXS line timers will be serviced.
 ******************************************************************************/
void
Vp890ServiceFxsTimers(
    VpDevCtxType            *pDevCtx,
    VpLineCtxType           *pLineCtx,
    Vp890LineObjectType     *pLineObj,
    VpDeviceIdType          deviceId,
    uint8                   ecVal)
{
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    uint8         timerType;
    uint16                  *pLineTimer;
    uint8                   mpiBuffer[6 + VP890_ICR1_LEN + VP890_ICR2_LEN
                                        + VP890_REGULATOR_PARAM_LEN + VP890_DC_CAL_REG_LEN
                                        + VP890_SYS_STATE_LEN + VP890_REGULATOR_CTRL_LEN];
    uint8                   mpiIndex    = 0;

    for (timerType = 0; timerType < VP_LINE_TIMER_LAST;  timerType++) {
        pLineTimer = &pLineObj->lineTimers.timers.timer[timerType];

        /* if the timer is not active then skip to next timer */
        if (!((*pLineTimer) & VP_ACTIVATE_TIMER)) {
            continue;
        }

        /* get the bits associated only with the time of the timer */
        (*pLineTimer) &= VP_TIMER_TIME_MASK;

        /* decrement the timer */
        if ((*pLineTimer) > 0) {
            (*pLineTimer)--;
        }

        /* if time left on the timer, active it and move on to the next one */
        if (*pLineTimer != 0) {
            *pLineTimer |= VP_ACTIVATE_TIMER;
            continue;
        }

        /*  if any of the timers have expired then serivce them */
        switch(timerType) {
            case VP_LINE_RING_EXIT_PROCESS: {
                /*
                 * There are two reasons to delay ringing exit - 1. in LPM to correct for
                 * the initial delay when entering ringing, and 2. to delay a polarity
                 * reversal change when exiting ringing to prevent the polarity reversal
                 * from occurring during the ringing cycle.
                 */
                uint8 ringExitSt[VP890_SYS_STATE_LEN];
                uint8 slicState = (pLineObj->slicValueCache & VP890_SS_LINE_FEED_MASK);
                bool exitRinging = FALSE;

                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_RING_EXIT_PROCESS Expired on Ch %d at time %d",
                    pLineObj->channelId, pDevObj->timeStamp));


                VpMpiCmdWrapper(deviceId, ecVal, VP890_SYS_STATE_RD,
                    VP890_SYS_STATE_LEN, ringExitSt);

                /*
                 * Check to see if we're completely out of ringing. Although the SLIC state
                 * should have already been modified prior to this point and therefore NOT
                 * indicate Ringing State, it's possible in theory that the time between
                 * changing the SLIC to a non-Ringing state to the point the register is
                 * read could be too fast for the silicon state machine to update. It's
                 * just theory...but we want to be 100% sure not to change register settings
                 * until we're really out of Ringing.
                 */
                if ((ringExitSt[0] & VP890_SS_RING_EXIT_MASK) ||
                    (slicState ==  VP890_SS_FEED_BALANCED_RINGING) ||
                    (slicState ==  VP890_SS_FEED_UNBALANCED_RINGING)) {
                    /*
                     * If any of the conditions are TRUE, it means Ringing is still on
                     * Tip/Ring. Ringing should be removed in <= 100ms from the time the
                     * new SLIC state is programmed, so keep track how long we're in this
                     * condition to eventually force stop it
                     */
                    if (pLineObj->lineTimers.timers.trackingTime < VP_CSLAC_RINGING_EXIT_MAX) {
                        /*
                         * We haven't exceeded our maximum allowable ringing exit time,
                         * so just restart this timer to run as soon as possible and update
                         * the trackingTime (note: tracking time is in ms).
                         */
                        pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] =
                            (1 | VP_ACTIVATE_TIMER);
                        pLineObj->lineTimers.timers.trackingTime = 0;
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Ringing still present on the line. RING_EXIT_TIMER restart Ch %d at time %d",
                            pLineObj->channelId, pDevObj->timeStamp));

                        /*
                         * Keep track of how long we've been in this condition so we don't
                         * repeat this forever.
                         */
                        pLineObj->lineTimers.timers.trackingTime +=
                            TICKS_TO_MS(1, pDevObj->devProfileData.tickRate);
                    } else {
                        /*
                         * Ringing is still on the line, but we've run out of patience.
                         * Time to just force ringing removal
                         */
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                      ("Forced Ring Exit after (%d) on Ch %d at time %d",
                                       pLineObj->lineTimers.timers.trackingTime,
                                       pLineObj->channelId, pDevObj->timeStamp));
                        exitRinging = TRUE;
                    }
                } else {
                    /*
                     * Ringing is totally removed from the line. It's now ok to change to
                     * the final line state and re-disable the Auto-State Transitions of
                     * the silicon.
                     */
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                  ("Normal Ring Exit after (%d) on Ch %d at time %d",
                                   pLineObj->lineTimers.timers.trackingTime, pLineObj->channelId,
                                   pDevObj->timeStamp));
                    exitRinging = TRUE;
                }

                if (exitRinging) {
#ifdef VP_CSLAC_SEQ_EN
                    bool resetTimer = FALSE;
                    uint16 timeRemain = 0;
#endif
                    /*
                     * Even though the MPI read will initialize this value, some compilers
                     * may not be that smart and can generate a warning if this isn't done.
                     */
                    uint8 ssCfg[VP890_SS_CONFIG_LEN] = {0x00};

                    /*
                     * Before doing anything, disable the silicon auto-system state control
                     * mechanism. This can interfere with Low Power Mode and/or other
                     * API line state managed changes
                     */
                    VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD,
                        VP890_SS_CONFIG_LEN, ssCfg);
                    ssCfg[0] |= VP890_AUTO_SSC_DIS;
                    VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_WRT,
                        VP890_SS_CONFIG_LEN, ssCfg);

#ifdef VP_CSLAC_SEQ_EN
                    /*
                     * Calling Vp890SetFxsLineState() will reset the timer if in the proces
                     * of Ringing Cadence. Save the timeRemain value and write it back after
                     * calling Vp890SetFxsLineState() if necessary.
                     */
                    if (pLineObj->cadence.status & VP_CADENCE_STATUS_MID_TIMER) {
                        resetTimer = TRUE;
                        timeRemain = pLineObj->cadence.timeRemain;
                    }
#endif /* VP_CSLAC_SEQ_EN */

                    /*
                     * If performing a polarity reversal this will start the Hook Freeze
                     * timer based on polarity reversal hook activity. So we won't need to
                     * account for this again when modifying the same timer for ring trip
                     * or user specified ringing exit
                     */
                    Vp890SetFxsLineState(pLineCtx, pLineObj->lineState.currentState);

#ifdef VP_CSLAC_SEQ_EN
                    /*
                     * Restore the cadence timer status and timer if set prior to calling
                     * Vp890SetFxsLineState(). This occurs during Ringing Cadence.
                     */
                    if (resetTimer) {
                        pLineObj->cadence.status |= VP_CADENCE_STATUS_MID_TIMER;
                        pLineObj->cadence.timeRemain = timeRemain;
                    }
#endif /* VP_CSLAC_SEQ_EN */

                    /*
                     * Start the hook switch debounce timer based on conditions of ringing
                     * exit - either debounce for internally known continued hook activity
                     * up a ring trip or debounce for normal ringing exit if on-hook. For
                     * polarity reversal, Vp890SetLineState() will set the Hook Freeze
                     * timer to the correct value. The VpCSLACSetTimer() just updates the
                     * timer if the new value is longer than the previously set value.
                     */
                    if (pLineObj->status & VP_CSLAC_HOOK) { /* Ring Trip */
                        VpCSLACSetTimer(&pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE],
                            MS_TO_TICKRATE(VP890_RING_TRIP_DEBOUNCE,
                                           pDevObj->devProfileData.tickRate));
                    } else {    /* On-Hook Ringing Exit */
#ifdef VP_CSLAC_SEQ_EN
                        /*
                         * Generate the Ringing Cadence Off Event now that ringing is
                         * removed from the line or now that we've given up on trying. Only
                         * do this if Cadence is supported in the API and if we're exiting
                         * ringing while running a ringing cadence.
                         */
                        if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                            VpProfilePtrType pProfile = pLineObj->cadence.pActiveCadence;
                            if ((pProfile != VP_NULL) && (pProfile[VP_PROFILE_TYPE_LSB]
                                                           == VP_PRFWZ_PROFILE_RINGCAD)) {
                                pLineObj->lineEvents.process |= VP_LINE_EVID_RING_CAD;
                                pLineObj->processData = VP_RING_CAD_BREAK;
                            }
                        }
#endif  /* VP_CSLAC_SEQ_EN */
                        /* Note the "/8" conversion from 125us to ms */
                        VpCSLACSetTimer(&pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE],
                            MS_TO_TICKRATE((pLineObj->ringCtrl.ringExitDbncDur / 8),
                                           pDevObj->devProfileData.tickRate));
                    }
                }
                }
                /* May not be ready for this, but it doesn't hurt */
                pDevObj->state |= VP_DEV_PENDING_INT;
                break;

            case VP_LINE_HOOK_FREEZE:
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_HOOK_FREEZE Expired on Ch %d at time %d",
                        pLineObj->channelId, pDevObj->timeStamp));
                pDevObj->state |= VP_DEV_PENDING_INT;
                break;

            case VP_LINE_DISCONNECT_EXIT:
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_DISCONNECT_EXIT Expired on Ch %d at time %d",
                        pLineObj->channelId, pDevObj->timeStamp));
                Vp890ServiceDiscExitTimer(pLineCtx);
                break;

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
            case VP_LINE_CAL_LINE_TIMER:
                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("VP_LINE_CAL_LINE_TIMER Expired on Ch %d at time %d",
                        pLineObj->channelId, pDevObj->timeStamp));
                Vp890CalLineInt(pLineCtx);
                break;
#endif
#ifdef VP890_LP_SUPPORT
            case VP_LINE_TRACKER_DISABLE:
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_TRACKER_DISABLE Expired on Ch %d at time %d",
                        pLineObj->channelId, pDevObj->timeStamp));
                Vp890TrackerDisableTimerMgmt(pLineCtx);
                break;
#endif

            case VP_LINE_SPEEDUP_RECOVERY_TIMER:
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_SPEEDUP_RECOVERY_TIMER Expired on Ch %d at time %d",
                        pLineObj->channelId, pDevObj->timeStamp));

                if (VpIsLowPowerTermType(pLineObj->termType)) {
                    pLineObj->icr2Values[VP890_ICR2_SPEEDUP_INDEX] &=
                        (uint8)(~(VP890_ICR2_MET_SPEED_CTRL | VP890_ICR2_BAT_SPEED_CTRL));

                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("VP_LINE_SPEEDUP_RECOVERY_TIMER: LP Term Type - Restoring Normal Speedup in ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                        pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                        pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                        pLineObj->channelId));

                    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP890_ICR2_WRT,
                        VP890_ICR2_LEN, pLineObj->icr2Values);
                } else {
                    if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Non-LP Entering Disconnect"));

                        /* Entering Disconnect */
                        Vp890LLSetSysState(deviceId, pLineCtx, VP890_SS_DISCONNECT, TRUE);

                        pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                            (MS_TO_TICKRATE(VP890_SPEEDUP_HOLD_TIME,
                            pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

                        pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION] |=
                            VP890_ICR1_LINE_BIAS_OVERRIDE;
                        pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
                            (VP890_ICR1_LINE_BIAS_OVERRIDE | VP890_ICR1_LINE_BIAS_OVERRIDE_NORM);

                        pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
                            VP890_ICR1_LINE_BIAS_OVERRIDE_NORM;
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Non-LP Term Type - ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                            pLineObj->icr1Values[0], pLineObj->icr1Values[1],
                            pLineObj->icr1Values[2], pLineObj->icr1Values[3],
                            pLineObj->channelId));
                        mpiIndex = Vp890ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

                        pLineObj->icr2Values[VP890_ICR2_SENSE_INDEX] |=
                            (VP890_ICR2_DAC_SENSE | VP890_ICR2_FEED_SENSE);
                        pLineObj->icr2Values[VP890_ICR2_SENSE_INDEX+1] &= ~VP890_ICR2_DAC_SENSE;
                        pLineObj->icr2Values[VP890_ICR2_SENSE_INDEX+1] |= VP890_ICR2_FEED_SENSE;
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Non-LP Term Type - ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                            pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                            pLineObj->channelId));

                        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR2_WRT,
                            VP890_ICR2_LEN, pLineObj->icr2Values);
                    } else {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Non-LP Exiting Disconnect"));

                        /*
                         * SLIC is always set to Active when exiting Disconnect. All
                         * states other than IDLE can be set here with fast response
                         * time. If IDLE is the ending state, wait until full
                         * Disconnect Exit time.
                         */
                        if ((pLineObj->slicValueCache != pLineObj->nextSlicValue) &&
                            ((pLineObj->nextSlicValue & VP890_SS_LINE_FEED_MASK) != VP890_SS_IDLE)) {
                            pLineObj->slicValueCache = pLineObj->nextSlicValue;

                            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Setting Ch %d to State 0x%02X at time %d",
                                pLineObj->channelId, pLineObj->slicValueCache, pDevObj->timeStamp));

                            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                                VP890_SYS_STATE_WRT, VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);
                        }

                        /* Restore normal Speedup control */
                        pLineObj->icr2Values[VP890_ICR2_SPEEDUP_INDEX] &=
                            (uint8)(~(VP890_ICR2_MET_SPEED_CTRL | VP890_ICR2_BAT_SPEED_CTRL));
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Non-LP Term Type - ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                            pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                            pLineObj->channelId));
                        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR2_WRT,
                            VP890_ICR2_LEN, pLineObj->icr2Values);
                    }
                    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
                        mpiIndex-1, &mpiBuffer[1]);
                }
                break;

            case VP_LINE_INTERNAL_TESTTERM_TIMER: {
                /* Apply new bias settings to keep tip/ring near battery. */

                /* While the internal test termination is applied, the
                 * line object ICR1 cache is used to keep track of what
                 * ICR1 needs to be once the internal test termination
                 * is removed.  It will not match the actual register
                 * value.  This is part of the internal test termination
                 * ICR1 override, so don't copy it to the line object */
                uint8 icr1Reg[VP890_ICR1_LEN];
                icr1Reg[0] = 0xFF;
                icr1Reg[1] = 0x18;
                icr1Reg[2] = 0xFF;
                icr1Reg[3] = 0x04;
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_INTERNAL_TESTTERM_TIMER Expired on Ch %d at time %d",
                        pLineObj->channelId, pDevObj->timeStamp));

                VpMpiCmdWrapper(deviceId, ecVal, VP890_ICR1_WRT, VP890_ICR1_LEN,
                    icr1Reg);
                break;
            }

            default:
                /*
                 * If we don't know what the timer is most likely it was masking hook or
                 * some other line detect activity. It can't hurt to force a signaling
                 * register read. If there are pending masks, the device specific API will
                 * freeze whatever status is necessary regardless of whether the signaling
                 * register is read or not.
                 */
                pDevObj->state |= VP_DEV_PENDING_INT;
                break;
        }
    } /* Loop through all timerTypes for chanID */

    return;
} /* Vp890ServiceFxsTimers() */

/**
 * Vp890ServiceDiscExitTimer()
 *  This function services active Disconnect Exit API timers.
 *
 * Preconditions:
 *  This Function must be called from the ApiTick function once per device.
 *
 * Postconditions:
 *  Active Disconnect Exit Timers for the current line have been serviced.
 */
void
Vp890ServiceDiscExitTimer(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8                   mpiBuffer[6 + VP890_ICR1_LEN + VP890_ICR2_LEN
                                        + VP890_REGULATOR_PARAM_LEN + VP890_DC_CAL_REG_LEN
                                        + VP890_SYS_STATE_LEN + VP890_REGULATOR_CTRL_LEN];
    uint8                   mpiIndex    = 0;

#ifdef VP890_LP_SUPPORT
    if (VpIsLowPowerTermType(pLineObj->termType)) {

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Disconnect Exit Timer: Setting Ch %d to State 0x%02X at time %d",
            pLineObj->channelId, pLineObj->nextSlicValue, pDevObj->timeStamp));

        /*
         * If we're in Disconnect when this timer expires (entering Disconnect)
         * need to correct ICR bits set to force feed toward ground and minimize
         * power.
         */
        if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
            pLineObj->icr2Values[VP890_ICR2_VOC_DAC_INDEX] |= VP890_ICR2_ILA_DAC;
            pLineObj->icr2Values[VP890_ICR2_VOC_DAC_INDEX] &= ~VP890_ICR2_VOC_DAC_SENSE;
            pLineObj->icr2Values[VP890_ICR2_VOC_DAC_INDEX+1] &= ~VP890_ICR2_ILA_DAC;

            pLineObj->icr3Values[VP890_ICR3_LINE_CTRL_INDEX+1] &= ~VP890_ICR3_LINE_CTRL;
        } else if (pLineObj->nextSlicValue == VP890_SS_DISCONNECT) {
            pLineObj->icr2Values[VP890_ICR2_SENSE_INDEX] =
                (VP890_ICR2_TIP_SENSE | VP890_ICR2_RING_SENSE
                | VP890_ICR2_ILA_DAC | VP890_ICR2_FEED_SENSE);
            pLineObj->icr2Values[VP890_ICR2_SENSE_INDEX+1] =
                (VP890_ICR2_TIP_SENSE | VP890_ICR2_RING_SENSE
               | VP890_ICR2_FEED_SENSE);
        }

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Disconnect Exit Timer: ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
            pLineObj->icr2Values[2], pLineObj->icr2Values[3],
            pLineObj->channelId));

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);

        /*
         * Release Tip and Ring Bias Override. Set Line Bias to
         * normal values.
         */
        pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION] &=
            ~VP890_ICR1_TIP_BIAS_OVERRIDE;
        pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
            (uint8)(~(VP890_ICR1_TIP_BIAS_OVERRIDE | VP890_ICR1_LINE_BIAS_OVERRIDE));

        /* Restore Normal SLIC Bias if NOT in Disconnect */
        if (pLineObj->lineState.currentState != VP_LINE_DISCONNECT) {
            pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
                VP890_ICR1_LINE_BIAS_OVERRIDE_NORM;
        }

        pLineObj->icr1Values[VP890_ICR1_RING_BIAS_OVERRIDE_LOCATION] &=
            ~VP890_ICR1_RING_BIAS_OVERRIDE;

        mpiIndex = Vp890ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

        pLineObj->slicValueCache = pLineObj->nextSlicValue;
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Disconnect Exit Timer: Channel %d Writing System State 0x%02X Time %d",
            pLineObj->channelId, pLineObj->slicValueCache, pDevObj->timeStamp));
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP890_SYS_STATE_WRT, VP890_SYS_STATE_LEN,
            &pLineObj->nextSlicValue);

        /* Disable Line Control if in Disconnect */
        if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR3_WRT, VP890_ICR3_LEN, pLineObj->icr3Values);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Disconnect Exit Timer: ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                pLineObj->icr3Values[2], pLineObj->icr3Values[3],
                pLineObj->channelId));
        }

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
            mpiIndex-1, &mpiBuffer[1]);
    } else {
#endif
        /* Non-LPM Disconnect Enter/Exit Additional Handling */
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Disconnect Exit Timer for Non-LPM Termination Type Ch %d at time %d",
            pLineObj->channelId, pDevObj->timeStamp));

        if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
            /* Line is in Disconnect */
            /*
             * Line State is already set, need to disable SLIC Bias in ICR1 and
             * disable DC Feed/Battery Hold Speed in preperation for Disconnect
             * exit.
             */
            pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION+1] &=
                ~(VP890_ICR1_LINE_BIAS_OVERRIDE);
            mpiIndex = Vp890ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

            pLineObj->icr2Values[VP890_ICR2_SPEEDUP_INDEX] |=
                (VP890_ICR2_MET_SPEED_CTRL | VP890_ICR2_BAT_SPEED_CTRL);
            pLineObj->icr2Values[VP890_ICR2_SPEEDUP_INDEX+1] |=
                (VP890_ICR2_MET_SPEED_CTRL | VP890_ICR2_BAT_SPEED_CTRL);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Disconnect Exit Timer: ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                pLineObj->channelId));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);

            VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
                mpiIndex-1, &mpiBuffer[1]);
        } else {
            /* Line is exiting Disconnect */
            if (pLineObj->slicValueCache != pLineObj->nextSlicValue) {
                pLineObj->slicValueCache = pLineObj->nextSlicValue;
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Disconnect Exit Timer: Setting Ch %d to State 0x%02X at time %d",
                    pLineObj->channelId, pLineObj->slicValueCache, pDevObj->timeStamp));

                VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP890_SYS_STATE_WRT,
                    VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }
        }
#ifdef VP890_LP_SUPPORT
    }
#endif
    pDevObj->state |= VP_DEV_PENDING_INT;
}

/*******************************************************************************
 * Vp890SetFxsLineState()
 *  This function sets the line state for a given channel of a given device. The
 * valid line states are defined in the VpLineState type.
 *
 * Preconditions:
 *  The line must first be initialized prior to setting the line state.  The
 * state must be a valid line state as defined in the VpLineState type.
 *
 * Postconditions:
 *  Returns success code if the channel can be set and the line state is valid
 * for the type of line specified by the line context.  If successfull, the line
 * specified by the line context is set to the line state specified.
 ******************************************************************************/
VpStatusType
Vp890SetFxsLineState(
    VpLineCtxType           *pLineCtx,
    VpLineStateType         state)
{
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    uint8                   ecVal       = pLineObj->ecVal;

    uint8                   userByte, mpiByte;
    bool                    nextSlicUpdated = FALSE;
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
    VpStatusType            status;
#endif
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    VpLineStateType         currentState = pLineObj->lineState.currentState;

    bool                    disconnectTimerSet = FALSE;
    bool                    feedToDisable = FALSE;
    bool                    polarityInversion = FALSE;
    uint8                   opCondTarget = pLineObj->opCond[0];

#ifdef VP_CSLAC_SEQ_EN
    bool                    disableTones = TRUE;
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890SetFxsLineState()"));

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
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
#endif
    /*
     * Read the status of the Operating Conditions register so we can change
     * only the TX and RX if the line state is a non-communication mode.
     * This also performs the line type/state verification.
     */
    opCondTarget = pLineObj->opCond[0];
    opCondTarget &=
        (uint8)(~(VP890_CUT_TXPATH | VP890_CUT_RXPATH | VP890_HIGH_PASS_DIS | VP890_OPCOND_RSVD_MASK));

    Vp890GetFxsTxRxPcmMode(pLineObj, state, &mpiByte);
    opCondTarget |= mpiByte;
    if (opCondTarget != pLineObj->opCond[0]) {
        pLineObj->opCond[0] = opCondTarget;
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("13.b Writing 0x%02X to Operating Conditions",
            pLineObj->opCond[0]));
        VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_COND_WRT, VP890_OP_COND_LEN, pLineObj->opCond);
    }

#ifdef VP_CSLAC_SEQ_EN
    /* We're no longer in the middle of a time function */
    pLineObj->cadence.status &= ~VP_CADENCE_STATUS_MID_TIMER;
    pLineObj->cadence.timeRemain = 0;
#endif

    /*
     * If this function is called by the application, stop the cadencer and
     * reset the Loop Supervision if it is incorrect. Disable all tones if going
     * to a state that does not support tone generation.
     */
    if (!(pLineObj->status & VP890_SLS_CALL_FROM_API)) {
        /* Correct the loop supervision if currently incorrect. */
        if (pLineObj->status & VP890_BAD_LOOP_SUP) {
            pLineObj->status &= ~(VP890_BAD_LOOP_SUP);
            VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOP_SUP_WRT,
                            VP890_LOOP_SUP_LEN, pLineObj->loopSup);
        }

        /* Disable tones and cadencing if going to a state that prevents it */
        switch(state) {
            case VP_LINE_STANDBY:
            case VP_LINE_DISCONNECT:
            case VP_LINE_RINGING:
            case VP_LINE_RINGING_POLREV:
            case VP_LINE_STANDBY_POLREV:
                /*
                 * Disable signal generator A/B/C/D before making any changes and stop
                 * previous cadences
                 */
                if (pLineObj->sigGenCtrl[0] != 0) {
                    pLineObj->sigGenCtrl[0] = 0;
                    VpMpiCmdWrapper(deviceId, ecVal, VP890_GEN_CTRL_WRT, VP890_GEN_CTRL_LEN,
                        pLineObj->sigGenCtrl);
                }
                break;

            default:
                /* Stop also if coming from Ringing */
                if ((pLineObj->lineState.usrCurrent != VP_LINE_RINGING) &&
                    (pLineObj->lineState.usrCurrent != VP_LINE_RINGING_POLREV)) {
#ifdef VP_CSLAC_SEQ_EN
                    /* Keep tones/cadencing running */
                    disableTones = FALSE;
#endif
                }
                break;
        }

#ifdef VP_CSLAC_SEQ_EN
        if (disableTones == TRUE) {
            pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
            pLineObj->cadence.pActiveCadence = VP_PTABLE_NULL;
        }

        /*  If the user is changing the line state, we should stop callerId */
        if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
            VpCliStopCli(pLineCtx);
        }
#endif
    }

    userByte = LineStateMap(state);

    /* Modify userByte depending on the current polarity */
#ifdef VP_CSLAC_SEQ_EN
    if (pLineObj->cadence.pActiveCadence != VP_NULL) {
        if ((pLineObj->cadence.status &
            (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_IGNORE_POLARITY)) ==
            (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_IGNORE_POLARITY)) {

            userByte &= ~VP890_SS_POLARITY_MASK;
            userByte |= (pLineObj->slicValueCache & VP890_SS_POLARITY_MASK);
        }
    }
#endif
    if ((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)
     || (currentState == VP_LINE_RINGING) || (currentState == VP_LINE_RINGING_POLREV)) {
        /*
         * Set State Ringing Returns TRUE if the line has NOT actually been changed. So return
         * at this point results in preventing the internal line state values to be updated
         * until the line is changed by the cadencer.
         */
        if (Vp890SetStateRinging(pLineCtx, state) == TRUE) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890SetLineStateInt-"));
            return VP_STATUS_SUCCESS;
        }
        if ((state != VP_LINE_RINGING) && (state != VP_LINE_RINGING_POLREV)) {
            /*
             * Ringing exit scenario. We have to prevent Polarity Reversals from occuring while
             * ringing is on the line otherwise the polarity reversal will occur during ringing
             * which creates an undesirable transient on the line.
             */

            /* Save the desired slic byte to be used in the ring exit timer. */
            pLineObj->nextSlicValue = userByte;
            nextSlicUpdated = TRUE; /* Stop Disconnect Timer from changing this */

            /* Check if polarity reversal is being performed. */
            if ((pLineObj->slicValueCache & VP890_SS_POLARITY_MASK)
              ^ (userByte & VP890_SS_POLARITY_MASK)) {
                /*
                 * Polarity reversal is in this ring exit transition, so force the new slic
                 * state polarity to the same polarity as current ringing.
                 */
                userByte &= ~VP890_SS_POLARITY_MASK;
                userByte |= (pLineObj->slicValueCache & VP890_SS_POLARITY_MASK);
            }
        }
    }

    if (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER) {
        /*
         * If we didn't save this before (from Ringing) AND we're in the middle of the Disconnect
         * exit timer, we have to save it now. When the Disconnect Timer expires, it's going to
         * use this value to update the SLIC Feed Register (0x56/57). If we don't update it, the
         * register will be updated with the old value, not the current value.
         */
        pLineObj->nextSlicValue = ((nextSlicUpdated)  ? pLineObj->nextSlicValue : userByte);
#ifdef VP890_LP_SUPPORT
        if ((VpIsLowPowerTermType(pLineObj->termType))
         && ((pLineObj->lineState.currentState == VP_LINE_STANDBY)
          && ((state != VP_LINE_STANDBY) && (state != VP_LINE_DISCONNECT)))) {
            /*
             * Disconnect Exit to LPM-Standby was started but we're now changing to non-LPM. Need
             * to correct ICR values.
             */
            Vp890SetLPRegisters(pDevObj, pLineObj, FALSE);
            Vp890WriteLPExitRegisters(pLineCtx, deviceId, pLineObj->ecVal, &userByte);
            Vp890LowPowerMode(pDevCtx);
        }
#endif
    }

    /*
     * Enable Disconnect Recovery time for hook status if going FROM Disconnect to a state that
     * can detect off-hook
     */
    if ((pLineObj->lineState.currentState == VP_LINE_DISCONNECT) && (state != VP_LINE_DISCONNECT)) {
        /* ..going to a state that can detect feed */
        if (!(pLineObj->lineState.condition & VP_CSLAC_LINE_LEAK_TEST)) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
              ("Setting Disconnect Recovery Timer on channel %d at time %d status 0x%04X",
                pLineObj->channelId, pDevObj->timeStamp, pLineObj->lineState.condition));

            pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                MS_TO_TICKRATE(VP_DISCONNECT_RECOVERY_TIME,
                    pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

            if (VpIsLowPowerTermType(pLineObj->termType)) {
                if (((userByte & VP890_SS_LINE_FEED_MASK) == VP890_SS_FEED_BALANCED_RINGING) ||
                    ((userByte & VP890_SS_LINE_FEED_MASK) ==  VP890_SS_FEED_UNBALANCED_RINGING)) {
                    /*
                     * Going to Ringing. Overwrite the standard disconnect exit time to cause
                     * ringing entry ASAP.
                     */
                    pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                        (MS_TO_TICKRATE(VP890_SPEEDUP_HOLD_TIME * 2,
                        pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;
                }
            }
            disconnectTimerSet = TRUE;
        }
    }

    /*
     * Set the disable flag if disabling feed. This prevents the
     * polrev timer from starting.
     */
    if (state == VP_LINE_DISCONNECT) {
        feedToDisable = TRUE;
    }

    /*
     * Set Polarity Reverse timer if the SLIC is changing polarity. Exclude
     * Disconnect type recovery conditions since a timer is set above for
     * those conditions.
     */
    if ((pLineObj->slicValueCache & VP890_SS_POLARITY_MASK)
     != (userByte & VP890_SS_POLARITY_MASK)) {
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
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Setting Polarity Reversal Timer on channel %d at time %d status 0x%04X",
                pLineObj->channelId, pDevObj->timeStamp, pLineObj->lineState.condition));

            pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE] =
                 MS_TO_TICKRATE(VP_POLREV_DEBOUNCE_TIME,
                     pDevObj->devProfileData.tickRate);

            pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE]
                |= VP_ACTIVATE_TIMER;
        }
    }

    if (pLineObj->calLineData.calDone == TRUE) {
        if ((polarityInversion == TRUE) ||
            (pLineObj->calLineData.forceCalDataWrite == TRUE)) {
            pLineObj->calLineData.forceCalDataWrite = FALSE;

            if (userByte & VP890_SS_POLARITY_MASK) {
                VpMpiCmdWrapper(deviceId, ecVal, VP890_DC_FEED_WRT,
                    VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);
            } else {
                VpMpiCmdWrapper(deviceId, ecVal, VP890_DC_FEED_WRT,
                    VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeed);
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

#ifdef VP890_LP_SUPPORT
    if ((VpIsLowPowerTermType(pLineObj->termType)
         /*
          * If going from/to Disconnect/Standby in LPM, states are the same
          * so don't continue.
          */
     && ((state == VP_LINE_DISCONNECT) || (currentState == VP_LINE_DISCONNECT)))) {
        if ((currentState == state) && (Vp890IsChnlUndrTst(pDevObj, pLineObj->channelId) == FALSE)) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890SetFxsLineState() SAME STATE"));
            return VP_STATUS_SUCCESS;
        } else {
            if (state == VP_LINE_DISCONNECT) {
                Vp890RunLPDisc(pLineCtx, TRUE, userByte);
            } else {
                Vp890RunLPDisc(pLineCtx, FALSE, userByte);
            }
        }
    } else {
#endif
        Vp890GroundStartProc(pLineCtx, pLineObj->slicValueCache, userByte);
#ifdef VP890_LP_SUPPORT
    }
#endif
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
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890SetFxsLineState() NORMAL"));
    return VP_STATUS_SUCCESS;
} /* SetFxsLineState() */

/**
 * Vp890GroundStartProc()
 *  This function implements special state transition handling
 *
 * Preconditions:
 *  None. Calling function must know that this code should execute.
 *
 * Postconditions:
 *  Depends on the stete transition. In some cases, partial sequences are done
 * and timers set to complete the rest. In other cases, the line is in it's final
 * state setting.
 */
void
Vp890GroundStartProc(
    VpLineCtxType *pLineCtx,
    uint8 currentLineState,
    uint8 userByte)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    uint8                   beforeState = (currentLineState & VP890_SS_LINE_FEED_MASK);
    uint8                   afterState  =  (userByte & VP890_SS_LINE_FEED_MASK);

    uint8                   mpiBuffer[6 + VP890_ICR1_LEN + VP890_ICR2_LEN
                                        + VP890_REGULATOR_PARAM_LEN + VP890_DC_CAL_REG_LEN
                                        + VP890_SYS_STATE_LEN + VP890_REGULATOR_CTRL_LEN];
    uint8                   mpiIndex    = 0;

#if (VP_CC_DEBUG_SELECT & VP_DBG_LINE_STATE)
    uint8                   channelId   = pLineObj->channelId;
#endif

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("+Vp890GroundStartProc() userByte 0x%02X", userByte));

#ifdef VP890_LP_SUPPORT
    if (VpIsLowPowerTermType(pLineObj->termType)) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP-Term Type Disconnect State Change Ch %d", channelId));

        if (pLineObj->status & VP890_LOW_POWER_EN) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Line in LPM - Adjusting LP Mode on Ch %d, NOT writing to device",
                channelId));
            Vp890LLSetSysState(deviceId, pLineCtx, userByte, FALSE);
        } else {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Line NOT in LPM - Adjusting LP Mode on Ch %d, writing to device",
                pLineObj->channelId));
            Vp890LLSetSysState(deviceId, pLineCtx, userByte, TRUE);
        }
    } else {
#endif
        if (beforeState != afterState) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Non-LP Disconnect State change Ch %d", channelId));

            if (beforeState == VP890_SS_DISCONNECT) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Non-LP Exiting Disconnect Ch %d", channelId));

                /*
                 * All Disoonnect exit state changes need to start with Acitve
                 * in order to quickly charge the line (in case of high REN).
                 * The silicon speedup time is very low, so the final state
                 * values an be written in 20ms.
                 */
                pLineObj->slicValueCache =
                    (VP890_SS_ACTIVE | (userByte & VP890_SS_POLARITY_MASK));
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_SYS_STATE_WRT,
                    VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);
                pLineObj->nextSlicValue = userByte;
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Exiting Disconnect with byte 0x%02X NextByte 0x%02X Ch %d at Time %d",
                    pLineObj->slicValueCache, userByte, channelId, pDevObj->timeStamp));

                /* Release Line Bias control */
                pLineObj->icr1Values[VP890_ICR1_BIAS_OVERRIDE_LOCATION] &= ~VP890_ICR1_LINE_BIAS_OVERRIDE;
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("VP_LINE_SPEEDUP_RECOVERY_TIMER: Non-LP Term Type - ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                    pLineObj->icr1Values[0], pLineObj->icr1Values[1],
                    pLineObj->icr1Values[2], pLineObj->icr1Values[3],
                    pLineObj->channelId));
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR1_WRT,
                    VP890_ICR1_LEN, pLineObj->icr1Values);

                pLineObj->icr2Values[VP890_ICR2_SENSE_INDEX] &=
                    (uint8)(~(VP890_ICR2_DAC_SENSE | VP890_ICR2_FEED_SENSE));

                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Exiting Disconnect - ICR2 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
                    pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                    pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                    channelId));
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR2_WRT,
                    VP890_ICR2_LEN, pLineObj->icr2Values);

                VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
                    mpiIndex-1, &mpiBuffer[1]);

                pLineObj->lineTimers.timers.timer[VP_LINE_SPEEDUP_RECOVERY_TIMER] =
                    MS_TO_TICKRATE(VP890_GEN_FXS_SPEEDUP_TIME,
                    pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
            } else {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Non-LP Entering Disconnect Ch %d", channelId));

                if (afterState == VP890_SS_DISCONNECT) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Starting VP_LINE_SPEEDUP_RECOVERY_TIMER (%d) Ch %d at time %d",
                        VP890_GEN_FXS_SPEEDUP_TIME, channelId, pDevObj->timeStamp));

                    /*
                     * The delay entering Disconnect is to compensate for the
                     * delay exiting Disconnect.
                     */
                    pLineObj->lineTimers.timers.timer[VP_LINE_SPEEDUP_RECOVERY_TIMER] =
                        MS_TO_TICKRATE(VP890_GEN_FXS_SPEEDUP_TIME,
                        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                } else {
                    Vp890LLSetSysState(deviceId, pLineCtx, userByte, TRUE);
                }
            }
        } else {
            if (beforeState != VP_LINE_DISCONNECT) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Setting Ch %d to device Linestate 0x%02X",
                    channelId, userByte));
                Vp890LLSetSysState(deviceId, pLineCtx, userByte, TRUE);
            }
        }

#ifdef VP890_LP_SUPPORT
    }
#endif
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("-Vp890GroundStartProc()"));

}

/*******************************************************************************
 * Vp890SetStateRinging()
 *  This function starts cadence and non-cadence ringing, as well as ringing
 * exit.
 *
 * Preconditions:
 *  None. Calling function must know that this code should execute.
 *
 * Postconditions:
 *  Line object is modified if ringing cadence or exiting (timers). If not
 * cadencing and ringing is started, then return TRUE. Otherwise return FALSE.
 ******************************************************************************/
bool
Vp890SetStateRinging(
    VpLineCtxType           *pLineCtx,
    VpLineStateType         state)
{
   /* Extract Line information used often in this function */
    Vp890LineObjectType     *pLineObj       = pLineCtx->pLineObj;
    uint8                   ecVal           = pLineObj->ecVal;
    VpLineStateType         currentState    = pLineObj->lineState.currentState;

    /* Extract Device information used often in this function */
    VpDevCtxType            *pDevCtx        = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj        = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId        = pDevObj->deviceId;
    uint8                   tempRingCtrlData = VP890_GEN_CTRL_EN_BIAS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890SetStateRinging()"));

    /*
     * No matter what the current line state is,
     * go to ringing if the input state is ringing
     */
    if ((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)) {
#ifdef VP_CSLAC_SEQ_EN
        VpProfilePtrType pProfile;
        pLineObj->cadence.pActiveCadence = pLineObj->pRingingCadence;
        pProfile = pLineObj->cadence.pActiveCadence;
        /*
         * We're entering a ringing state, so determine if we need to
         * cadence. If we're not cadencing, this is "always on", so we can
         * disable the currently active cadence sequence and immediately
         * implement the ringing state change.
         */
        if (pProfile == VP_PTABLE_NULL) {
            pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
        } else if (!(pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)) {
            /*
             * We have a non-null cadence and the candence is still active.
             * If the cadence was not previously started, then start it here
             * and let the sequencer take over. Otherwise, the sequencer was
             * previously started and this state change is at the request of
             * the sequencer.
             */

            /* We have a cadence and are just starting it */
            pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
            pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
            pLineObj->cadence.pCurrentPos = &pProfile[VP_PROFILE_TYPE_SEQUENCER_START];
            pLineObj->cadence.length = pProfile[VP_PROFILE_LENGTH];
            pLineObj->cadence.status &= ~VP_CADENCE_STATUS_IGNORE_POLARITY;
            pLineObj->cadence.status |= (pProfile[VP_PROFILE_MPI_LEN] & 0x01) ?
                VP_CADENCE_STATUS_IGNORE_POLARITY : 0;

            /* Nullify any internal sequence so that the API doesn't think that
             * an internal sequence of some sort is running */
            pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_NONE;

            return TRUE;

        }
        /*
         * If we're in an active Ringing Cadence, and ready to go into the
         * Ringing state, generate the Ringing Event and indicate that this
         * is the Ringing On event
         */
        if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
            if (pProfile[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_RINGCAD) {
                pLineObj->lineEvents.process |= VP_LINE_EVID_RING_CAD;
                pLineObj->processData = VP_RING_CAD_MAKE;
            }
        }
#endif
        /*
         * Cadencing already called or null cadence.  We're ready to set
         * the line to the Ringing State but we have to first make sure
         * that the signal generator parameters in the device are setup
         * for the ringing profile
         */
        tempRingCtrlData &= ~VP890_GEN_BIAS;

        if (pLineObj->sigGenCtrl[0] != tempRingCtrlData) {
            pLineObj->sigGenCtrl[0] = tempRingCtrlData;
            VpMpiCmdWrapper(deviceId, ecVal, VP890_GEN_CTRL_WRT, VP890_GEN_CTRL_LEN,
                pLineObj->sigGenCtrl);
        }

        if (pLineObj->ringingParams != VP_PTABLE_NULL) {
            if (((pLineObj->status & VP890_RING_GEN_NORM) && (state == VP_LINE_RINGING))
             || ((pLineObj->status & VP890_RING_GEN_REV) && (state == VP_LINE_RINGING_POLREV))) {
                /*
                 * We're already programmed to the correct set of generator
                 * parameters. So skip this step.
                 */
            } else {
                int16 biasErr;

                VpMemCpy(pLineObj->ringingParams, pLineObj->ringingParamsRef, VP890_RINGER_PARAMS_LEN);
                biasErr = (int16)((((uint16)(pLineObj->ringingParams[1]) << 8) & 0xFF00) +
                    ((uint16)(pLineObj->ringingParams[2]) & 0x00FF));
                if (state == VP_LINE_RINGING) {
                    /* Normal polarity */
                    biasErr -= ((pDevObj->vp890SysCalData.sigGenAError[pLineObj->channelId][0] -
                        pDevObj->vp890SysCalData.vocOffset[pLineObj->channelId][VP890_NORM_POLARITY]) *
                        16L / 10L);
                    pLineObj->status |= VP890_RING_GEN_NORM;
                    pLineObj->status &= ~VP890_RING_GEN_REV;
                } else {
                    /* Reverse polarity */
                    biasErr += ((pDevObj->vp890SysCalData.sigGenAError[pLineObj->channelId][0] -
                        pDevObj->vp890SysCalData.vocOffset[pLineObj->channelId][VP890_REV_POLARITY]) *
                        16L / 10L);
                    pLineObj->status |= VP890_RING_GEN_REV;
                    pLineObj->status &= ~VP890_RING_GEN_NORM;
                }
                pLineObj->ringingParams[1] = (uint8)((biasErr >> 8) & 0x00FF);
                pLineObj->ringingParams[2] = (uint8)(biasErr & 0x00FF);

                VpMpiCmdWrapper(deviceId, ecVal, VP890_RINGER_PARAMS_WRT,
                    VP890_RINGER_PARAMS_LEN, pLineObj->ringingParams);
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
            uint8 ssCfg[VP890_SS_CONFIG_LEN] = {0x00};
            if ((pLineObj->ringCtrl.ringTripExitSt != VP_LINE_RINGING)
             && (pLineObj->ringCtrl.ringTripExitSt != VP_LINE_RINGING_POLREV)) {
                VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD, VP890_SS_CONFIG_LEN, ssCfg);
                ssCfg[0] &= ~VP890_AUTO_SSC_DIS;
                VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_WRT, VP890_SS_CONFIG_LEN, ssCfg);
            }
        }

        /* If High Voltage Device AND not using LV override, disable 100V clamp */
        if ((pDevObj->stateInt & VP890_IS_HIGH_VOLTAGE) &&
            (pDevObj->devProfileData.lowVoltOverride == FALSE)) {
            pLineObj->icr2Values[VP890_ICR2_SWY_CTRL_INDEX+1] |= VP890_ICR2_SWY_LIM_CTRL;
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Ringing Entry - Disabling Clamps ICR2 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                pLineObj->channelId));
            VpMpiCmdWrapper(deviceId, ecVal, VP890_ICR2_WRT,
                VP890_ICR2_LEN, pLineObj->icr2Values);
        }
    } else if ((VP_LINE_RINGING_POLREV == currentState) ||
               (VP_LINE_RINGING        == currentState)) {
        /*
         * Since this is part of an "if" statement that checked the state we're going to and
         * falls into if Ringing or Ringing PolRev, we know this condition checking the from
         * state can only be conditions where Ringing exit into a non-Ringing state.
         */
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Ringing Exit on Ch (%d) time (%d)", pLineObj->channelId, pDevObj->timeStamp));
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

        /* Re-enable clamps if previously disabled (HV and NOT using LV override) */
        if ((pDevObj->stateInt & VP890_IS_HIGH_VOLTAGE) &&
            (pDevObj->devProfileData.lowVoltOverride == FALSE)) {
            pLineObj->icr2Values[VP890_ICR2_SWY_CTRL_INDEX+1] &= ~(VP890_ICR2_SWY_LIM_CTRL);
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Ringing Exit - Re-Enabling Clamps ICR2 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                pLineObj->icr2Values[2], pLineObj->icr2Values[3],
                pLineObj->channelId));
            VpMpiCmdWrapper(deviceId, ecVal, VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);
        }
    }

    return FALSE;
} /* Vp890SetStateRinging() */

/*******************************************************************************
 * Vp890SetRelayState()
 *  This function controls the state of controlled relays for the VP890 device.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The indicated relay state is set for the given line.
 ******************************************************************************/
VpStatusType
Vp890SetRelayState(
    VpLineCtxType       *pLineCtx,
    VpRelayControlType  rState)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    uint8                   ecVal       = pLineObj->ecVal;

    uint8                   mpiBuffer[2 + VP890_IODIR_REG_LEN + VP890_IODATA_REG_LEN];
    uint8                   mpiIndex    = 0;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890SetRelayState()"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->state, TRUE)) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Handle the VP_RELAY_BRIDGED_TEST case, which is available to all term
     * types.  Due to the lack of an actual test load for 890, this relay
     * state is implemented by the internal test termination algorithm */
    if (rState == VP_RELAY_BRIDGED_TEST) {
        Vp890ApplyInternalTestTerm(pLineCtx);

        pLineObj->relayState = VP_RELAY_BRIDGED_TEST;

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* If the internal test termination is currently applied and we're going
     * to a state other than BRIDGED_TEST, restore the internal settings */
    if (pLineObj->internalTestTermApplied == TRUE &&
        rState != VP_RELAY_BRIDGED_TEST) {
        Vp890RemoveInternalTestTerm(pLineCtx);
    }

    /* Check term type */
    if ((pLineObj->termType == VP_TERM_FXS_GENERIC) ||
        (pLineObj->termType == VP_TERM_FXO_GENERIC) ||
        (pLineObj->termType == VP_TERM_FXS_LOW_PWR)) {
        /* Check requested relay state */
        if (rState != VP_RELAY_NORMAL) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SetRelayState() - Invalid relay state 0x%02X",
                rState));
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_INVALID_ARG;
        }
    } else if ((pLineObj->termType == VP_TERM_FXS_SPLITTER_LP) ||
               (pLineObj->termType == VP_TERM_FXS_SPLITTER) ||
               (pLineObj->termType == VP_TERM_FXS_ISOLATE) ||
               (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)) {

        uint8 ioData[VP890_IODATA_REG_LEN];
        uint8 ioDir[VP890_IODIR_REG_LEN];

        VpMpiCmdWrapper(deviceId, ecVal, VP890_IODATA_REG_RD, VP890_IODATA_REG_LEN,
            ioData);
        VpMpiCmdWrapper(deviceId, ecVal, VP890_IODIR_REG_RD, VP890_IODIR_REG_LEN,
            ioDir);

        ioData[0] &= ~VP890_IODATA_IO1;

        ioDir[0] &= ~VP890_IODIR_IO1_MASK;
        ioDir[0] |= VP890_IODIR_IO1_OUTPUT;

        switch(rState) {
            case VP_RELAY_NORMAL:
                /* Normal: Create High on Isolate, Low on Splitter */
                if ((pLineObj->termType == VP_TERM_FXS_ISOLATE) ||
                    (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)) {
                    ioData[0] |= VP890_IODATA_IO1;
                } else {
                    ioData[0] &= ~VP890_IODATA_IO1;
                }
                break;

            case VP_RELAY_RESET:
                /* Normal: Create Low on Isolate, High on Splitter */
                if ((pLineObj->termType == VP_TERM_FXS_ISOLATE) ||
                    (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)) {
                    ioData[0] &= ~VP890_IODATA_IO1;
                } else {
                    ioData[0] |= VP890_IODATA_IO1;
                }
                break;

            default:
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SetRelayState() - Invalid relay state 0x%02X",
                    rState));
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_INVALID_ARG;
        }

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_IODIR_REG_WRT,
            VP890_IODIR_REG_LEN, ioDir);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_IODATA_REG_WRT,
            VP890_IODATA_REG_LEN, ioData);

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
            &mpiBuffer[1]);
    } else {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SetRelayState() - Invalid termination type 0x%02X",
            pLineObj->termType));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_INVALID_ARG;
    }

    pLineObj->relayState = rState;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp890SetRelayState() */

/*******************************************************************************
 * Vp890ApplyInternalTestTerm()
 *  Configures ICR settings for the internal test termination algorithm, which
 * is used instead of a physical test load for devices which do not have one.
 * The internal test termination works by internally shorting tip and ring.
 ******************************************************************************/
static void
Vp890ApplyInternalTestTerm(
    VpLineCtxType *pLineCtx)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    uint8 icr1Reg[VP890_ICR1_LEN];

    uint8 mpiBuffer[3 + VP890_DC_CAL_REG_LEN + VP890_ICR4_LEN + VP890_ICR1_LEN];
    uint8 mpiIndex = 0;

    if (pLineObj->internalTestTermApplied == TRUE) {
        return;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pLineObj->internalTestTermApplied = TRUE;

    /* Disconnect VAB sensing */
    pLineObj->dcCalValues[VP890_DC_CAL_CUT_INDEX] |= VP890_C_TIP_SNS_CUT;
    pLineObj->dcCalValues[VP890_DC_CAL_CUT_INDEX] |= VP890_C_RING_SNS_CUT;

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_DC_CAL_REG_WRT,
        VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

    /* Reverse the polarity of the ground key detector to disable ground
     * key event */
    pLineObj->icr4Values[VP890_ICR4_GKEY_DET_LOCATION] |= VP890_ICR4_GKEY_POL;
    pLineObj->icr4Values[VP890_ICR4_GKEY_DET_LOCATION+1] |= VP890_ICR4_GKEY_POL;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR4_WRT,
        VP890_ICR4_LEN, pLineObj->icr4Values);

    /* Forcing the SLIC DC bias for 200ms helps collapse the battery
     * voltage, especially for fixed tracking designs.  We're taking over
     * ICR1 completely here.  Other parts of the code will set
     * pLineObj->icr1Values but will not actually write to the register
     * while this relay state is active.  See Vp890ProtectedWriteICR1().  When
     * we leave this relay state, we will restore pLineObj->icr1Values */
    icr1Reg[0] = 0xFF;
    icr1Reg[1] = 0xFF;
    icr1Reg[2] = 0xFF;
    icr1Reg[3] = 0x0F;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR1_WRT,
        VP890_ICR1_LEN, icr1Reg);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
        &mpiBuffer[1]);

    /* Start a timer to change the ICR1 settings later to make tip and ring
     * outputs high impedance so that they tend to pull to battery. */
    pLineObj->lineTimers.timers.timer[VP_LINE_INTERNAL_TESTTERM_TIMER] =
        MS_TO_TICKRATE(VP890_INTERNAL_TESTTERM_SETTLING_TIME,
            pDevObj->devProfileData.tickRate);
    pLineObj->lineTimers.timers.timer[VP_LINE_INTERNAL_TESTTERM_TIMER]
        |= VP_ACTIVATE_TIMER;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
} /* Vp890ApplyInternalTestTerm() */

/*******************************************************************************
 * Vp890RemoveInternalTestTerm()
 *  This function reverts the settings that control the internal test
 * termination.
 ******************************************************************************/
static void
Vp890RemoveInternalTestTerm(
    VpLineCtxType *pLineCtx)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;

    uint8 mpiBuffer[3 + VP890_DC_CAL_REG_LEN + VP890_ICR4_LEN + VP890_ICR1_LEN];
    uint8 mpiIndex = 0;

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Restore VAB sensing */
    pLineObj->dcCalValues[VP890_DC_CAL_CUT_INDEX] &= ~VP890_C_TIP_SNS_CUT;
    pLineObj->dcCalValues[VP890_DC_CAL_CUT_INDEX] &= ~VP890_C_RING_SNS_CUT;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_DC_CAL_REG_WRT,
        VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

    /* Restore ground key polarity setting */
    pLineObj->icr4Values[VP890_ICR4_GKEY_DET_LOCATION] &= ~VP890_ICR4_GKEY_POL;
    pLineObj->icr4Values[VP890_ICR4_GKEY_DET_LOCATION+1] &= ~VP890_ICR4_GKEY_POL;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR4_WRT,
        VP890_ICR4_LEN, pLineObj->icr4Values);

    /*
     * Restore ICR1 to the cached value in pLineObj->icr1Values. This would
     * have been changed in normal program execution but not written to the
     * device while in protected mode.
     */
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp890RemoveInternalTestTerm: Writing ICR1 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
        pLineObj->icr1Values[0], pLineObj->icr1Values[1],
        pLineObj->icr1Values[2], pLineObj->icr1Values[3], pLineObj->channelId));

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_ICR1_WRT,
        VP890_ICR1_LEN, pLineObj->icr1Values);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
        &mpiBuffer[1]);

    /* Deactivate the timer in case it is still running */
    pLineObj->lineTimers.timers.timer[VP_LINE_INTERNAL_TESTTERM_TIMER]
        &= ~VP_ACTIVATE_TIMER;

    pLineObj->internalTestTermApplied = FALSE;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
} /* Vp890RemoveInternalTestTerm() */

#endif /* VP_CC_890_SERIES */

