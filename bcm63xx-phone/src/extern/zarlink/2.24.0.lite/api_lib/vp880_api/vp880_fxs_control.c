/** \file vp880_fxs_control.c
 * vp880_fxs_control.c
 *
 *  This file contains the control functions for the FXS lines of the VP880
 * device.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 10001 $
 * $LastChangedDate: 2012-05-14 11:52:10 -0500 (Mon, 14 May 2012) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES) && defined (VP880_FXS_SUPPORT)

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
static void
Vp880ApplyInternalTestTerm(
    VpLineCtxType *pLineCtx);

static void
Vp880RemoveInternalTestTerm(
    VpLineCtxType *pLineCtx);

/**
 * Vp880ProcessFxsLine()
 *  This function should only be called by Vp880ApiTick on FXS lines. It
 * performs all line processing for operations that are Tick based
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated.
 */
VpStatusType
Vp880ProcessFxsLine(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    bool dpStatus[2] = {FALSE, FALSE};
    VpOptionEventMaskType lineEvents1;
    VpOptionEventMaskType lineEvents2;
    VpDialPulseDetectStatesType beforeState, afterState;
    uint8 hookStatus = 0, i, validSamples;
    uint8 hookIncrement;
    bool dp2Valid;

#ifdef VP880_INCLUDE_TESTLINE_CODE
    /* Skip processing during line test */
    if (Vp880IsChnlUndrTst(pDevObj, channelId) == TRUE) {
        return VP_STATUS_SUCCESS;
    }
#endif

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
        pDevObj->pulseSpecs2.flashMax == 0) {

        dp2Valid = FALSE;

    } else {
        dp2Valid = TRUE;
    }

    /*
     * If the line is configured for Dial Pulse Detection, run the Dial Pulse
     * detection code. Dial Pulse detection code will generate the appropriate
     * events
     */
    if((pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) &&
       (!(pLineObj->lineState.condition & VP_CSLAC_LINE_LEAK_TEST))) {
        /*
         * Based on the following (vp880_api_int.h):
         *
         *  #define VP880_CC_500HZ_RATE     0x40
         *  #define VP880_CC_1KHZ_RATE      0x30
         *  #define VP880_CC_2KHZ_RATE      0x20
         *  #define VP880_CC_4KHZ_RATE      0x10
         *  #define VP880_CC_8KHZ_RATE      0x00
         */

        /*
         * Get the incremental steps sizes in 125us increments used to apply to the on/off-hook
         * times below, which is also the same scale as the Dial Pulse Detection Parameters.
         */
        uint8 shiftAmt = ((pDevObj->txBufferDataRate >> 4) & 0x7);
        hookIncrement = (1 << shiftAmt);

        validSamples = ((pDevObj->txBuffer[VP880_TX_BUF_HOOK_MSB_INDEX]
            & VP880_TX_BUF_LEN_MASK) >> 4);

        if (validSamples == 7) {
            validSamples = 6;
        }

        if (channelId == 0) {
            hookStatus = pDevObj->txBuffer[VP880_TX_BUF_HOOK_LSB_INDEX]
                & VP880_TX_BUF_HOOK_CHAN1_MASK;
        } else {
            hookStatus = (pDevObj->txBuffer[VP880_TX_BUF_HOOK_MSB_INDEX]
                & VP880_TX_BUF_HOOK_MSB_MASK) << 2;
            hookStatus |= ((pDevObj->txBuffer[VP880_TX_BUF_HOOK_LSB_INDEX]
                & VP880_TX_BUF_HOOK_CHAN2_MASK) >> 6);
        }
        /*
         * If the line is currently in Low Power Mode, the meaning applied to the hook detector is
         * inverted from it's normal meaning. A "high" signal level into the hook detector, which
         * normally measures current, will cause the hook bit to be set = '1'. The SW has to
         * understand that the hook detector is being fed line voltage when in LPM and interpret
         * that to mean the customer phone is on-hook. In normal detection mode where the
         * hook detector is being fed current, a "high" signal level means high current = phone is
         * off-hook.
         */
        if (pLineObj->status & VP880_LOW_POWER_EN) {
            hookStatus = ~hookStatus;
        }

        beforeState = pLineObj->dpStruct.state;

        /*
         * This flag is set when the Test Buffer (more specifically, the Hook-Bit buffer) has been
         * read this tick. It could happen when a hook interrupt occurs or not (i.e., there are a
         * few reasons for reading the test buffer). It should ONLY be set on silicon > VC, but it
         * isn't too much effort to check for both Hook-Bit Buffer read AND > VC silicon before
         * processing the data.
         */
        if ((pDevObj->state & VP_DEV_TEST_BUFFER_READ) &&
            (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC)) {
            /*
             * Loop Open and Loop Close times are only at the accuracy of the tickrate now. We
             * want to improve that by looking for the point the hook status changed in the hook-bit
             * buffer and adjust based on the hook bit buffer sample rate (i.e., hook increment set
             * above).
             */
            VP_HOOK(VpLineCtxType, pLineCtx,
                    ("CH%d Validsamples %d, buffer %02X %02X %d%d%d%d%d%d  at time %d",
                     channelId,
                     ((pDevObj->txBuffer[VP880_TX_BUF_HOOK_MSB_INDEX] & VP880_TX_BUF_LEN_MASK) >> 4),
                     pDevObj->txBuffer[0], pDevObj->txBuffer[1],
                     (hookStatus & 0x20) == 0x20,
                     (hookStatus & 0x10) == 0x10,
                     (hookStatus & 0x08) == 0x08,
                     (hookStatus & 0x04) == 0x04,
                     (hookStatus & 0x02) == 0x02,
                     (hookStatus & 0x01) == 0x01,
                     pDevObj->timeStamp));

            for (i = 1; i < (1 << validSamples); i <<= 1) {
                if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE) {
                    if (!(hookStatus & i)) {
                        VP_HOOK(VpLineCtxType, pLineCtx,
                                ("CH%d - Loop Close Time (%d) at time (%d) Hook Increment (%d)",
                                 channelId, pLineObj->dpStruct.lc_time, pDevObj->timeStamp,
                                 hookIncrement));

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
                        VP_HOOK(VpLineCtxType, pLineCtx,
                                ("CH%d - Loop Open Time (%d) at time (%d) Hook Increment (%d)",
                                 channelId, pLineObj->dpStruct.lo_time, pDevObj->timeStamp,
                                 hookIncrement));

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

            VP_HOOK(VpLineCtxType, pLineCtx,
                    ("CH%d Fine Measurement Loop Open Time (%d) Loop Close Time (%d) at time %d",
                     channelId, pLineObj->dpStruct.lo_time, pLineObj->dpStruct.lc_time,
                     pDevObj->timeStamp));
        }

        if ((!(pDevObj->stateInt & VP880_IS_ABS)) &&
            (pDevObj->swParamsCache[VP880_REGULATOR_TRACK_INDEX] & VP880_REGULATOR_FIXED_RING)) {
            /* Compensate for slow onhook detection with Fixed Mode Tracking Designs */
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct.hookSt)) {
                if (pLineObj->dpStruct.lc_time > VP880_PULSE_DETECT_ADJUSTMENT_LONG) {
                    pLineObj->dpStruct.lc_time -= VP880_PULSE_DETECT_ADJUSTMENT_LONG;
                    VP_HOOK(VpLineCtxType, pLineCtx,
                            ("CH%d Long Compensation to Loop Close Time (%d) at time (%d)",
                             channelId, pLineObj->dpStruct.lc_time, pDevObj->timeStamp));
                }
            }
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct2.hookSt)) {
                if (pLineObj->dpStruct2.lc_time > VP880_PULSE_DETECT_ADJUSTMENT_LONG) {
                    pLineObj->dpStruct2.lc_time -= VP880_PULSE_DETECT_ADJUSTMENT_LONG;
                }
            }
        } else {
            /* Compensate just for the silicon */
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct.hookSt)) {
                if (pLineObj->dpStruct.lc_time > VP880_PULSE_DETECT_ADJUSTMENT_SHORT) {
                    pLineObj->dpStruct.lc_time -= VP880_PULSE_DETECT_ADJUSTMENT_SHORT;
                    VP_HOOK(VpLineCtxType, pLineCtx,
                            ("CH%d Short Compensation to Loop Close Time (%d) at time (%d)",
                             channelId, pLineObj->dpStruct.lc_time, pDevObj->timeStamp));
                }
            }
            if (beforeState == VP_DP_DETECT_STATE_LOOP_CLOSE && !(pLineObj->dpStruct2.hookSt)) {
                if (pLineObj->dpStruct2.lc_time > VP880_PULSE_DETECT_ADJUSTMENT_SHORT) {
                    pLineObj->dpStruct2.lc_time -= VP880_PULSE_DETECT_ADJUSTMENT_SHORT;
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
            if (afterState == VP_DP_DETECT_STATE_LOOP_OPEN) {
                if ((!(pDevObj->stateInt & VP880_IS_ABS)) &&
                    (pDevObj->swParamsCache[VP880_REGULATOR_TRACK_INDEX] & VP880_REGULATOR_FIXED_RING)) {
                    /* Compensate for slow onhook detection with Fixed Mode Tracking Designs */
                    pLineObj->dpStruct.lo_time += VP880_PULSE_DETECT_ADJUSTMENT_LONG;
                    pLineObj->dpStruct2.lo_time += VP880_PULSE_DETECT_ADJUSTMENT_LONG;
                } else {
                    /* Compensate for silicon only */
                    pLineObj->dpStruct.lo_time += VP880_PULSE_DETECT_ADJUSTMENT_SHORT;
                    pLineObj->dpStruct2.lo_time += VP880_PULSE_DETECT_ADJUSTMENT_SHORT;
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
                pLineObj->status |= VP880_DP_SET1_DONE;
            }
        }

        if (dpStatus[1] == TRUE && dp2Valid == TRUE) {
            pLineObj->signaling2 = lineEvents2.signaling;
            pLineObj->lineEventHandle = pDevObj->timeStamp;

            if (!(pLineObj->lineEvents.signaling & VP_LINE_EVID_BREAK_MAX)) {
                pLineObj->status |= VP880_DP_SET2_DONE;
            }
        }

        /* Report events if:
         *  Both DP sets are done, OR
         *  Set 1 is done and set 2 is invalid */
        if ((pLineObj->status & VP880_DP_SET1_DONE) &&
            ((pLineObj->status & VP880_DP_SET2_DONE) ||
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

            pLineObj->status &= ~(VP880_DP_SET1_DONE | VP880_DP_SET2_DONE);
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
        VpDeviceIdType deviceId = pDevObj->deviceId;
        uint8 ecVal = pLineObj->ecVal;

        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880ProcessFxsLine()+"));

        /* First read of the CID State Machine this tick. */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_CID_PARAM_RD,
            VP880_CID_PARAM_LEN, pLineObj->tickBeginState);
        pLineObj->delayConsumed = FALSE;

        if (pLineObj->suspendCid == TRUE) {
            /*
             * Check to see if the Device Buffer is empty. If it is, continue
             * with CID.
             */
            uint8 cidState = (pLineObj->tickBeginState[0] & VP880_CID_STATE_MASK);

            if ((cidState == VP880_CID_STATE_URUN)
             || (cidState == VP880_CID_STATE_IDLE)) {
                uint8 cidParam = pLineObj->tickBeginState[0];

                pLineObj->suspendCid = FALSE;
                cidParam &= ~(VP880_CID_FRAME_BITS);
                cidParam |= VP880_CID_DIS;

                Vp880MuteChannel(pLineCtx, FALSE);
                if (cidParam != pLineObj->tickBeginState[0]) {
                    pLineObj->tickBeginState[0] = cidParam;
                    VP_CID(VpLineCtxType, pLineCtx,
                        ("Stopping FSK Generator with 0x%02X to CID Params at time %d",
                        pLineObj->tickBeginState[0], pDevObj->timeStamp));
                    VpMpiCmdWrapper(deviceId, ecVal, VP880_CID_PARAM_WRT,
                        VP880_CID_PARAM_LEN, pLineObj->tickBeginState);
                    pLineObj->callerId.status &= ~VP_CID_FSK_ACTIVE;
                }
            }
        }

        if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
            VpCidSeq(pLineCtx);
        }
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880ProcessFxsLine()-"));
    }
#endif
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SetRelayState()
 *  This function controls the state of controlled relays for the VP880 device.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  The indicated relay state is set for the given line.
 */
VpStatusType
Vp880SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;
    VpTermType termType = pLineObj->termType;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 pcn = pDevObj->staticInfo.rcnPcn[VP880_PCN_LOCATION];
    uint8 revCode = pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION];

    uint8 ioDirection[VP880_IODIR_REG_LEN];
    uint8 ioData[VP880_IODATA_REG_LEN];

    uint8 mpiBuffer[3 + VP880_IODIR_REG_LEN + VP880_IODATA_REG_LEN + VP880_ICR1_LEN];
    uint8 mpiIndex = 0;

    /*
     * In case this function fails, make sure the line object is unchanged by
     * pre-saving it's current value. First step prior to 100% verifiying if
     * this function will be success is to pre-clear the relay control bits.
     */
    uint8 preIcr1Values = pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION];

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState+"));

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Handle the VP_RELAY_BRIDGED_TEST case for devices that do not have a
     * physical test load.  Use the internal test termination algorithm instead.
     * If the VP880_ALWAYS_USE_INTERNAL_TEST_TERMINATION option is defined in
     * vp_api_cfg.h the HAS_TEST_LOAD_SWITCH flag will always be cleared at
     * initialization so that this method will always be used.  The internal
     * test termination is only supported for revisions newer than VC.
     */
    if (rState == VP_RELAY_BRIDGED_TEST &&
        !(pDevObj->stateInt & VP880_HAS_TEST_LOAD_SWITCH) &&
        revCode > VP880_REV_VC) {
        Vp880ApplyInternalTestTerm(pLineCtx);

        pLineObj->relayState = VP_RELAY_BRIDGED_TEST;

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* If the internal test termination is currently applied and we're going
     * to a state other than BRIDGED_TEST, restore the internal settings */
    if (pLineObj->internalTestTermApplied == TRUE &&
        rState != VP_RELAY_BRIDGED_TEST) {
        Vp880RemoveInternalTestTerm(pLineCtx);
    }

    /* Read registers that may have partial modifications */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_IODIR_REG_RD, VP880_IODIR_REG_LEN,
        ioDirection);

    VpMpiCmdWrapper(deviceId, ecVal, VP880_IODATA_REG_RD, VP880_IODATA_REG_LEN,
        ioData);

    /* Always set the value for Test Load Control. */
    pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION-1] |=
        VP880_ICR1_TEST_LOAD_MASK;

    /* Preclear test load bits */
    pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION]
        &= ~(VP880_ICR1_TEST_LOAD_MASK);

    switch (pLineObj->termType) {
        case VP_TERM_FXS_GENERIC:
        case VP_TERM_FXS_LOW_PWR:
            switch (rState) {
                case VP_RELAY_BRIDGED_TEST:
                    if (!(pDevObj->stateInt & VP880_HAS_TEST_LOAD_SWITCH)) {

                        /* Restore values prior to caling this function. */
                        pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION] = preIcr1Values;

                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
                        return VP_STATUS_INVALID_ARG;
                    }

                    pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION]
                        |= VP880_ICR1_TEST_LOAD_METALLIC;

                    /* Test Load bits pre-cleared. Can pick up at Normal */

                case VP_RELAY_NORMAL:
                    break;

                default:
                    /* Restore values prior to caling this function. */
                    pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION] = preIcr1Values;

                    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
                    return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_TERM_FXS_ISOLATE:
        case VP_TERM_FXS_ISOLATE_LP:
        case VP_TERM_FXS_SPLITTER:
        case VP_TERM_FXS_SPLITTER_LP:
            ioDirection[0] &= ~VP880_IODIR_IO1_MASK;
            ioData[0] &= ~(VP880_IODATA_IO1);

            switch (rState) {
                case VP_RELAY_BRIDGED_TEST:
                    if (!(pDevObj->stateInt & VP880_HAS_TEST_LOAD_SWITCH)) {
                        /* Restore values prior to caling this function. */
                        pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION] = preIcr1Values;

                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
                        return VP_STATUS_INVALID_ARG;
                    }

                    pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION]
                        |= VP880_ICR1_TEST_LOAD_METALLIC;

                    /* Test Load bits pre-cleared. Can pick up at Normal */

                case VP_RELAY_NORMAL:
                    if (revCode <= VP880_REV_VC) {
                        ioDirection[0] |= VP880_IODIR_IO1_OPEN_DRAIN;

                        if ((termType == VP_TERM_FXS_ISOLATE) ||
                            (termType == VP_TERM_FXS_ISOLATE_LP)) {
                            /*
                             * Dir = open drain, Set Data = 0
                             * This causes the output to go high.
                             */
                            ioData[0] &= ~VP880_IODATA_IO1;
                        } else {
                            /*
                             * Dir = open drain, Set Data = 1
                             * This causes the output to go low.
                             */
                            ioData[0] |= VP880_IODATA_IO1;
                        }
                    } else {
                        /*
                         * Devices VP880_DEV_PCN_88536 and VP880_DEV_PCN_88264
                         * are affected here because the VP-API-II forces the
                         * revision code to >= JA
                         */
                        if ((termType == VP_TERM_FXS_ISOLATE) ||
                            (termType == VP_TERM_FXS_ISOLATE_LP)) {
                            /*
                             * Dir = output, Set Data = 1
                             * This causes the output to go high.
                             */
                            ioDirection[0] |= VP880_IODIR_IO1_OUTPUT;
                            ioData[0] |= VP880_IODATA_IO1;
                        } else {
                            if ((pcn == VP880_DEV_PCN_88536) ||
                                (pcn == VP880_DEV_PCN_88264)) {
                                /*
                                 * Dir = output, Set Data = 0
                                 * This causes the output to go low.
                                 */
                                ioDirection[0] |= VP880_IODIR_IO1_OUTPUT;
                                ioData[0] &= ~VP880_IODATA_IO1;
                            } else {
                                /*
                                 * Dir = open drain, Set Data = 1
                                 * This causes the output to go low.
                                 */
                                ioDirection[0] |= VP880_IODIR_IO1_OPEN_DRAIN;
                                ioData[0] |= VP880_IODATA_IO1;
                            }
                        }
                    }
                    break;

                case VP_RELAY_RESET:
                    if (revCode <= VP880_REV_VC) {
                        ioDirection[0] |= VP880_IODIR_IO1_OPEN_DRAIN;
                        if ((termType == VP_TERM_FXS_ISOLATE) ||
                            (termType == VP_TERM_FXS_ISOLATE_LP)) {
                            /*
                             * Dir = open drain, Set Data = 1.
                             * This causes the output to go low
                             */
                            ioData[0] |= VP880_IODATA_IO1;
                        } else {
                            /*
                             * Dir = open drain, Set Data = 0
                             * This causes the output to go high.
                             */
                            ioData[0] &= ~VP880_IODATA_IO1;
                        }
                    } else {
                        /*
                         * Devices VP880_DEV_PCN_88536 and VP880_DEV_PCN_88264
                         * are affected here because the VP-API-II forces the
                         * revision code to >= JA
                         */
                        if ((termType == VP_TERM_FXS_ISOLATE) ||
                            (termType == VP_TERM_FXS_ISOLATE_LP)) {
                            if ((pcn == VP880_DEV_PCN_88536) ||
                                (pcn == VP880_DEV_PCN_88264)) {
                                /*
                                 * Dir = output, Set Data = 0.
                                 * This causes the output to go low
                                 */
                                ioDirection[0] |= VP880_IODIR_IO1_OUTPUT;
                                ioData[0] &= ~VP880_IODATA_IO1;
                            } else {
                                /*
                                 * Dir = open drain, Set Data = 1.
                                 * This causes the output to go low
                                 */
                                ioDirection[0] |= VP880_IODIR_IO1_OPEN_DRAIN;
                                ioData[0] |= VP880_IODATA_IO1;
                            }
                        } else {
                            /*
                             * Dir = output, Set Data = 1
                             * This causes the output to go high.
                             */
                            ioDirection[0] |= VP880_IODIR_IO1_OUTPUT;
                            ioData[0] |= VP880_IODATA_IO1;
                        }
                    }
                    break;

                default:
                    /* Restore values prior to caling this function. */
                    pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION] = preIcr1Values;

                    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
                    return VP_STATUS_INVALID_ARG;
            }
            break;

        default:
            /* Restore values prior to caling this function. */
            pLineObj->icr1Values[VP880_ICR1_TEST_LOAD_LOCATION] = preIcr1Values;

            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
            return VP_STATUS_INVALID_ARG;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("1. Write IODATA 0x%02X on Ch %d",
        ioData[0], pLineObj->channelId));

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("2. Write IODIR 0x%02X on Channel %d",
        ioDirection[0], pLineObj->channelId));

    /*
     * Note in this ONE VP-API-II case for ICR1 Write, we don't go through
     * protected write because that's handled above in this function.
     */
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT,
        VP880_ICR1_LEN, pLineObj->icr1Values);

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp880SetRelayState(): Write ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
        pLineObj->icr1Values[0], pLineObj->icr1Values[1],
        pLineObj->icr1Values[2], pLineObj->icr1Values[3], pLineObj->channelId));

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_IODATA_REG_WRT,
        VP880_IODATA_REG_LEN, ioData);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_IODIR_REG_WRT,
        VP880_IODIR_REG_LEN, ioDirection);

    /* send down the mpi commands */
    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

    pLineObj->relayState = rState;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelayState-"));
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880ApplyInternalTestTerm()
 *  Configures ICR settings for the internal test termination algorithm, which
 * is used instead of a physical test load for devices which do not have one.
 * The internal test termination works by internally shorting tip and ring.
 */
static void
Vp880ApplyInternalTestTerm(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pLineObj->ecVal;
    uint16 timerDelay;
    uint8 icr1Reg[VP880_ICR1_LEN];

    uint8 mpiBuffer[3 + VP880_ICR6_LEN + VP880_ICR4_LEN + VP880_ICR1_LEN];
    uint8 mpiIndex = 0;

    if (pLineObj->internalTestTermApplied == TRUE) {
        return;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Disconnect VAB sensing */
    pLineObj->icr6Values[VP880_DC_CAL_CUT_INDEX] |= VP880_C_TIP_SNS_CUT;
    pLineObj->icr6Values[VP880_DC_CAL_CUT_INDEX] |= VP880_C_RING_SNS_CUT;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR6_WRT,
        VP880_ICR6_LEN, pLineObj->icr6Values);

    /* Reverse the polarity of the ground key detector to disable ground
     * key event */
    pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION] |= VP880_ICR4_GKEY_POL;
    pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION+1] |= VP880_ICR4_GKEY_POL;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
        VP880_ICR4_LEN, pLineObj->icr4Values);

    /*
     * Forcing the SLIC DC bias for 200ms helps collapse the battery voltage, especially for fixed
     * tracking designs.  We're taking over ICR1 completely here.  Other parts of the code will set
     * pLineObj->icr1Values but will not actually write to the register while this relay state is
     * active. See Vp880ProtectedWriteICR1(). When we leave this relay state, we will restore
     * pLineObj->icr1Values
     */
    icr1Reg[0] = 0xFF;
    icr1Reg[2] = 0xFF;
    if (pDevObj->stateInt & VP880_IS_ABS) {
#ifdef VP880_ABS_SUPPORT
        icr1Reg[1] = 0x68;
        icr1Reg[3] = 0x06;
#endif
    } else {
#ifdef VP880_TRACKER_SUPPORT
        icr1Reg[1] = 0xFF;
        icr1Reg[3] = 0x0F;
#endif
    }
    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                  ("Vp880ApplyInternalTestTerm(): Write ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
                   icr1Reg[0], icr1Reg[1], icr1Reg[2], icr1Reg[3], pLineObj->channelId));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT, VP880_ICR1_LEN, icr1Reg);

    /* send down the mpi commands */
    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

    /* Start a timer to change the ICR1 settings later to make tip and ring
     * outputs high impedance so that they tend to pull to battery. */
    if ((pDevObj->stateInt & VP880_IS_ABS) ||
        !(pDevObj->swParams[VP880_REGULATOR_TRACK_INDEX]
             & VP880_REGULATOR_FIXED_RING_SWY)) {
        /* Use a short delay for ABS and non-fixed tracking devices */
        timerDelay = VP880_INTERNAL_TESTTERM_SETTLING_TIME_SHORT;
    } else {
        /* Use a longer delay for fixed tracking devices */
        timerDelay = VP880_INTERNAL_TESTTERM_SETTLING_TIME_LONG;
    }
    pLineObj->lineTimers.timers.timer[VP_LINE_INTERNAL_TESTTERM_TIMER] =
        (MS_TO_TICKRATE(timerDelay, pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

    pLineObj->internalTestTermApplied = TRUE;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
} /* Vp880ApplyInternalTestTerm() */

/**
 * Vp880RemoveInternalTestTerm()
 *  This function reverts the settings that control the internal test
 * termination.
 */
static void
Vp880RemoveInternalTestTerm(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pLineObj->ecVal;

    uint8 mpiBuffer[3 + VP880_ICR6_LEN + VP880_ICR4_LEN + VP880_ICR1_LEN];
    uint8 mpiIndex = 0;

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Restore VAB sensing */
    pLineObj->icr6Values[VP880_DC_CAL_CUT_INDEX] &= ~VP880_C_TIP_SNS_CUT;
    pLineObj->icr6Values[VP880_DC_CAL_CUT_INDEX] &= ~VP880_C_RING_SNS_CUT;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR6_WRT,
        VP880_ICR6_LEN, pLineObj->icr6Values);

    /* Restore ground key polarity setting */
    pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION] &= ~VP880_ICR4_GKEY_POL;
    pLineObj->icr4Values[VP880_ICR4_GKEY_DET_LOCATION+1] &= ~VP880_ICR4_GKEY_POL;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
        VP880_ICR4_LEN, pLineObj->icr4Values);

    /* Restore ICR1 to the cached value in pLineObj->icr1Values */
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp880RemoveInternalTestTerm(): Write ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
        pLineObj->icr1Values[0], pLineObj->icr1Values[1],
        pLineObj->icr1Values[2], pLineObj->icr1Values[3], pLineObj->channelId));

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT,
        VP880_ICR1_LEN, pLineObj->icr1Values);

    /* send down the mpi commands */
    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

    /* Deactivate the timer in case it is still running */
    pLineObj->lineTimers.timers.timer[VP_LINE_INTERNAL_TESTTERM_TIMER]
        &= ~VP_ACTIVATE_TIMER;

    pLineObj->internalTestTermApplied = FALSE;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
} /* Vp880RemoveInternalTestTerm() */

/**
 * Vp880OffHookMgmt()
 *  This function manages the device and API behavior when an off-hook is
 * detected from the device AFTER all normal debounce timers have expired.
 *
 * Preconditions:
 *  This function is called internally by the API-II only.
 *
 * Postconditions:
 *  Device and API is updated accordingly. Returns TRUE if an event is posted.
 */
bool
Vp880OffHookMgmt(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx,
    uint8 ecVal)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    bool retFlag = FALSE;

#ifdef VP_CSLAC_SEQ_EN
    VpProfilePtrType pCadence;
#endif

    VP_HOOK(VpLineCtxType, pLineCtx, ("Off-Hook on Line %d at Time %d Low Power 0x%02X",
        pLineObj->channelId, pDevObj->timeStamp, (pLineObj->status & VP880_LOW_POWER_EN)));

    pLineObj->dpStruct.hookSt = TRUE;
    pLineObj->dpStruct2.hookSt = TRUE;

    pLineObj->leakyLineCnt = 0;
    pLineObj->status &= ~VP880_LINE_LEAK;

    if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
        pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_OFF;
        pLineObj->lineEventHandle = pDevObj->timeStamp;
        retFlag = TRUE;
    }

#ifdef VP_CSLAC_SEQ_EN
    /*
     * If an off-hook is detected when the active cadence is a Message Waiting
     * Pulse on the line, restore the line state.
     */
    pCadence = pLineObj->cadence.pActiveCadence;
    if (pCadence != VP_PTABLE_NULL) {
        if (pCadence[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_MSG_WAIT_PULSE_INT) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Stopping Active Message Waiting Pulse"));
            Vp880SetLineState(pLineCtx, pLineObj->lineState.currentState);
        }
    }
#endif
    return retFlag;
}

/**
 * Vp880OnHookMgmt()
 *  This function manages the device and API behavior when an on-hook is
 * detected from the device AFTER all normal debounce timers have expired.
 *
 * Preconditions:
 *  This function is called internally by the API-II only.
 *
 * Postconditions:
 *  Device and API is updated accordingly.
 */
bool
Vp880OnHookMgmt(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx,
    uint8 ecVal)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    bool retFlag = FALSE;

#ifdef VP880_ABS_SUPPORT
    uint8 slacState;
#endif

    VP_HOOK(VpLineCtxType, pLineCtx, ("On-Hook on Line %d at Time %d",
        pLineObj->channelId, pDevObj->timeStamp));

    /* Restore the initial threshold */
    if (pLineObj->hookHysteresis != 0) {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT, VP880_LOOP_SUP_LEN,
            pLineObj->loopSup);
    }

    pLineObj->dpStruct.hookSt = FALSE;
    pLineObj->dpStruct2.hookSt = FALSE;

    if ((pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) &&
        (!(pLineObj->status & VP880_LINE_LEAK))) {
        /*
         * If this is the first time after initialization that we are checking
         * for on-hook and it is on-hook, don't generate an interrupt
         */
        VP_HOOK(VpLineCtxType, pLineCtx,
            ("Dial Pulse Detect Disabled - checking initial on-hook.."));

        if (!(pLineObj->lineState.condition & VP_CSLAC_STATUS_VALID)) {
            VP_HOOK(VpLineCtxType, pLineCtx,
                ("Line Previously Initialized - Generating On-Hook Event"));

            pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_ON;
            pLineObj->lineEventHandle = pDevObj->timeStamp;
            retFlag = TRUE;
        }
    } else {
        VP_HOOK(VpLineCtxType, pLineCtx,
            ("Dial Pulse Detect Enabled. On-Hook from Dial Pulse State Machine."));
    }
    if (VpIsLowPowerTermType(pLineObj->termType)) {
        VP_HOOK(VpLineCtxType, pLineCtx, ("User State %d Current State %d",
            pLineObj->lineState.usrCurrent, pLineObj->lineState.currentState));

        if (pLineObj->lineState.usrCurrent == VP_LINE_STANDBY) {
            pLineObj->lineState.currentState = VP_LINE_STANDBY;
            Vp880LLSetSysState(deviceId, pLineCtx, 0x00, FALSE);
        }
    }

#ifdef VP880_ABS_SUPPORT
    slacState = pLineObj->slicValueCache;

    if (pDevObj->stateInt & VP880_IS_ABS) {
        switch(slacState & VP880_SS_LINE_FEED_MASK) {
            /*
             * Feed states where the SLIC needs to be put into high battery mode
             * to optimize feed conditions and transient response.
             */
            case (VP880_SS_ACTIVE & VP880_SS_LINE_FEED_MASK):
            case (VP880_SS_IDLE & VP880_SS_LINE_FEED_MASK):
            case (VP880_SS_ACTIVE_MID_BAT & VP880_SS_LINE_FEED_MASK):
                slacState &= ~VP880_SS_LINE_FEED_MASK;
                if(pLineObj->lineState.usrCurrent == VP_LINE_STANDBY) {
                    slacState |= VP880_SS_IDLE;
                } else {
                    slacState |= VP880_SS_ACTIVE_MID_BAT;
                }
                Vp880LLSetSysState(deviceId, pLineCtx, slacState, TRUE);
                break;

            default:
                /*
                 * Another state that either should not cause off-hook detection,
                 * or state that is handled by API-II functionality (e.g.,
                 * Ring Trip).
                 */
                break;
        }
    }
#endif

    return retFlag;
}

/*
 * Vp880ProtectedWriteICR1()
 *  This function is a wrapper for writing to ICR1.  If the internal test
 * termination is applied, ICR1 must not be changed, so this function copies
 * the data into the line object cache and returns without writing anything to
 * the device.  If the internal test termination is not applied, the write
 * is performed.
 *
 * Note: This function must be called from within a critical section.
 */
#ifdef VP880_FXS_SUPPORT
uint8
Vp880ProtectedWriteICR1(
    Vp880LineObjectType *pLineObj,
    uint8 mpiIndex,
    uint8 *mpiBuffer)
{
    if (pLineObj->internalTestTermApplied == FALSE) {
        VP_LINE_STATE(None, VP_NULL, ("Vp880ProtectedWriteICR1(): Channel %d Writing ICR1 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->channelId,
            pLineObj->icr1Values[0], pLineObj->icr1Values[1],
            pLineObj->icr1Values[2], pLineObj->icr1Values[3]));

        return VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT,
            VP880_ICR1_LEN, pLineObj->icr1Values);
    }
    return mpiIndex;
}
#endif  /* VP880_FXS_SUPPORT */
#endif
