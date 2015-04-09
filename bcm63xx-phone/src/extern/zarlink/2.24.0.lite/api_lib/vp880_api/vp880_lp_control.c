/** \file vp880_lp_control.c
 * vp880_lp_control.c
 *
 *  This file contains the control functions for the Vp880 device API for LPM
 * termination types.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11093 $
 * $LastChangedDate: 2013-07-26 11:57:57 -0500 (Fri, 26 Jul 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES) && defined (VP880_LP_SUPPORT)

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
Vp880WriteLPEnterRegisters(
    VpLineCtxType *pLineOtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 *lineState);

/**
 * Vp880SetDiscTimers()
 *  This function provides the value for Disconnect Timing using LPM.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  None. Returns a value in ms.
 */
uint16
Vp880SetDiscTimers(
    Vp880DeviceObjectType *pDevObj)
{
    /*
     * Only two options -- Fixed Ringing (long discharge), or Tracked Ringing
     * (short discharge).
     */
    /*
     * This is actually checking CH0 (SWY) Switching Regulator mode which
     * technically could be different from CH1 (SWZ). But since Profile Wizard
     * doesn't support different CH0/CH1 Ringing Mode configurations AND it's
     * not practical in a real application, we can get away with checking just
     * one of the settings. Best to use CH0 in case a single channel device
     * defines the second setting as RSVD and set to a single value in all
     * cases.
     */
    if (pDevObj->swParams[VP880_REGULATOR_TRACK_INDEX]
        & VP880_REGULATOR_FIXED_RING_SWY) {
        /*
         * Longest is using "fixed" ringing mode because the external
         * capacitors are generally very large.
         */
        return VP880_FIXED_TRACK_DISABLE_TIME;
    } else {
        return VP880_INVERT_BOOST_DISABLE_TIME;
    }
}

/**
 * Vp880RunLPDisc()
 *  This function implements the Disconnect Enter/Exit for Low Power Mode.
 *
 * Preconditions:
 *  None. Calling function must know that this code should execute.
 *
 * Postconditions:
 *  Initial procedures are performed and timers set to enter or exit Disconnect
 * state for Low Power termination type.
 */
void
Vp880RunLPDisc(
    VpLineCtxType *pLineCtx,
    bool discMode,
    uint8 nextSlicByte)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint8 ecVal = pLineObj->ecVal;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 mpiBuffer[7 + VP880_ICR2_LEN + VP880_ICR1_LEN + VP880_DEV_MODE_LEN
                      + VP880_SYS_STATE_LEN + VP880_ICR5_LEN + VP880_ICR3_LEN
                      + VP880_ICR4_LEN];
    uint8 mpiIndex = 0;

    /* devState "set" means the other line is in a LPM state. */
    Vp880DeviceStateIntType devState = (channelId == 0) ?
        (pDevObj->stateInt & VP880_LINE1_LP) :
        (pDevObj->stateInt & VP880_LINE0_LP);

    /*
     * myDevState "set" means *this* line is in a LPM state prior to calling
     * this function.
     */
    Vp880DeviceStateIntType myDevState = (channelId == 0) ?
        (pDevObj->stateInt & VP880_LINE0_LP) :
        (pDevObj->stateInt & VP880_LINE1_LP);

    /*
     * Enter/Exit Disconnect uses Active (w/Polarity) to cause fast charge or
     * discharge of the line.
     */
    uint8 lineState[] = {VP880_SS_ACTIVE};

    bool lineInTest = Vp880IsChnlUndrTst(pDevObj, channelId);
    /*
     * Force logic to operate same as "leaky line" if Standby State is forced to IDLE mode 'OR' if
     * there is a leaky line. Either way, don't change the value of leakyLine from what it's been in
     * past releases just to minimize the changes. Otherwise, additional logic below will have to
     * changed. This method ensures the leakyLine remains to be either 0 or VP880_LINE_LEAK.
     */
    uint16 leakyLine =
        ((pLineObj->status & (VP880_LINE_LEAK | VP880_LP_STANDBY_IDLE)) ? VP880_LINE_LEAK : 0);


    bool hookStatus;

    /*
     * If coming from a Feed state to Disconnect, OK to check on hook state to
     * determine if other line was in LPM. But don't run this test if coming
     * from Disconnect because the hook state is frozen in Disconnect but
     * other line could have been set to LPM. In that case, try to enter LPM
     * and if exists a real off-hook, the VP-API-II will resolve it in hook
     * management.
     */
    if (discMode == TRUE) {
        VpCSLACGetLineStatus(pLineCtx, VP_INPUT_RAW_HOOK, &hookStatus);
        leakyLine = (hookStatus == FALSE) ? leakyLine : VP880_LINE_LEAK;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880RunLPDisc+"));

    /*
     * The other line is not really in LPM if this line is in test or has a
     * leaky line condition.
     */
    devState = (lineInTest == TRUE) ?  0 : devState;
    devState = ((leakyLine == VP880_LINE_LEAK) ? 0 : devState);

    if (discMode == TRUE) {        /* Entering Disconnect */
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Entering Disconnect on Chan %d time %d",
            channelId, pDevObj->timeStamp));
        pDevObj->stateInt |= ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);

        /*
         * There are three cases to consider when entering Disconnect:
         *     1. This line is coming from LPM-Standby and other line is in LPM.
         *     2. This line is coming from a non-LPM state and other line is in LPM.
         *     3. This line is coming from any state and other line is not in LPM.
         *        Condition #3 occurs also if this line has a resistive leak.
         *
         *  All cases require the ICR values modified to disable the switcher.
         */

        /*
         * Step 1: Program all ICR registers including Disable Switcher. Note
         * these are writing to cached values only, not to the device -- yet.
         */

        Vp880SetLPRegisters(pLineObj, TRUE);
        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &= ~VP880_ICR2_ILA_DAC;
        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] |= VP880_ICR2_VOC_DAC_SENSE;
        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] &= ~VP880_ICR2_VOC_DAC_SENSE;

        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX] |= VP880_ICR2_SWY_CTRL_EN;
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] &= ~VP880_ICR2_SWY_CTRL_EN;

        /*
         * Always start by disabling the switcher. Remaining steps will be done
         * either in this function (immediate) or delayed. In all cases, the
         * final state of Disconnect is delayed in order to allow the supply
         * to discharge once disabled.
         */
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP: Entering Disconnect: Channel %d: Writing ICR2 0x%02X 0x%02X 0x%02X 0x%02X time %d",
            pLineObj->channelId,
            pLineObj->icr2Values[0], pLineObj->icr2Values[1],
            pLineObj->icr2Values[2], pLineObj->icr2Values[3], pDevObj->timeStamp));

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
            VP880_ICR2_LEN, pLineObj->icr2Values);

        /*
         * Force Tip, Ring and Line Bias Override and set values to max. This
         * forces values toward ground.
         */
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] |=
            (VP880_ICR1_TIP_BIAS_OVERRIDE | VP880_ICR1_LINE_BIAS_OVERRIDE);
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
            (VP880_ICR1_TIP_BIAS_OVERRIDE | VP880_ICR1_LINE_BIAS_OVERRIDE);

        pLineObj->icr1Values[VP880_ICR1_RING_BIAS_OVERRIDE_LOCATION] |=
            VP880_ICR1_RING_BIAS_OVERRIDE;
        pLineObj->icr1Values[VP880_ICR1_RING_BIAS_OVERRIDE_LOCATION+1] |=
            VP880_ICR1_RING_BIAS_OVERRIDE;

        mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

        if ((devState) && (myDevState)) {
            /*
             * 1. This line is coming from LPM-Standby and other line is in LPM.
             *      Step 1: Program all ICR registers including Disable Switcher. - done
             *      Step 2: Set line to Active (for fast discharge)
             *      Step 3: Delay, then set line to VP_LINE_DISCONNECT. - outside
             *              this if statement.
             */
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Entering Disconnect: Channel %d: Case 1 State 0x%02X time %d",
                pLineObj->channelId, lineState[0], pDevObj->timeStamp));

            /*
             * Step 2: Line is in LPM-Standby (i.e., Disconnect) so need to
             * set to Active to cause fast discharge
             */
            if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
                Vp880UpdateBufferChanSel(pDevObj, channelId, lineState[0], FALSE);
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DEV_MODE_WRT,
                    VP880_DEV_MODE_LEN, pDevObj->devMode);
            }

            if (pLineObj->slicValueCache != lineState[0]) {
                pLineObj->slicValueCache = lineState[0];
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }
            VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
                &mpiBuffer[1]);
            mpiIndex = 0;
        } else if ((devState) && (!myDevState)) {
            /*
             * 2. This line is coming from a non-LPM state and other line is in LPM.
             *      Step 1: Program all ICR registers including Disable Switcher. - done
             *      Step 2: Set line to Active w/Polarity (for fast discharge)
             *      Step 3: Delay, then set device to LPM - done at next tick.
             */

            /*
             * Step 2: Set line to Active w/Polarity (for fast discharge). Note
             * that the state may already be in a condition that will allow fast
             * discharge, but it could also be in Tip Open or Ringing or "other"
             * so we just want to be sure it's in a state that we know.
             */

            lineState[0] = pLineObj->slicValueCache;
            lineState[0] &= ~VP880_SS_LINE_FEED_MASK;
            lineState[0] |= VP880_SS_ACTIVE;

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Entering Disconnect: Channel %d: Case 2 State 0x%02X time %d",
                pLineObj->channelId, lineState[0], pDevObj->timeStamp));

            if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
                Vp880UpdateBufferChanSel(pDevObj, channelId, lineState[0], FALSE);
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DEV_MODE_WRT,
                    VP880_DEV_MODE_LEN, pDevObj->devMode);
            }

            if (pLineObj->slicValueCache != lineState[0]) {
                pLineObj->slicValueCache = lineState[0];
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }

            VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
                &mpiBuffer[1]);
            mpiIndex = 0;

            pLineObj->nextSlicValue = VP880_SS_DISCONNECT;
            pLineObj->nextSlicValue |= ((lineInTest == TRUE) ? VP880_SS_ACTIVATE_MASK : 0);

            /* Step 3: Delay, then set device to LPM. */
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Delaying Disconnect Enter Channel %d at time %d from Line State 0x%08lX Other Line 0x%08lX",
                channelId, pDevObj->timeStamp, myDevState, devState));
        } else {
            /*
             * 3. This line is coming from any state and other line is not in LPM.
             *      Step 1: Program all ICR registers including Disable Switcher. - done
             *      Step 2: Set line to Active w/polarity (for fast discharge)
             *      Step 3: Delay, then set line to VP_LINE_DISCONNECT. - outside
             *              this if statement.
             */

            /* Step 2: Set line to Active w/polarity (for fast discharge) */
            lineState[0] = pLineObj->slicValueCache;
            lineState[0] &= ~VP880_SS_LINE_FEED_MASK;
            lineState[0] |= VP880_SS_ACTIVE;

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Entering Disconnect: Channel %d: Case 3 State 0x%02X time %d",
                pLineObj->channelId, lineState[0], pDevObj->timeStamp));

            if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
                Vp880UpdateBufferChanSel(pDevObj, channelId, lineState[0], FALSE);
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DEV_MODE_WRT,
                    VP880_DEV_MODE_LEN, pDevObj->devMode);
            }

            if (pLineObj->slicValueCache != lineState[0]) {
                pLineObj->slicValueCache = lineState[0];
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
                VP880_ICR3_LEN, pLineObj->icr3Values);
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Entering Disconnect: Channel %d: Writing ICR3 0x%02X 0x%02X 0x%02X 0x%02X time %d",
                pLineObj->channelId,
                pLineObj->icr3Values[0], pLineObj->icr3Values[1],
                pLineObj->icr3Values[2], pLineObj->icr3Values[3], pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
                VP880_ICR4_LEN, pLineObj->icr4Values);
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Entering Disconnect: Channel %d: Writing ICR4 0x%02X 0x%02X 0x%02X 0x%02X time %d",
                pLineObj->channelId,
                pLineObj->icr4Values[0], pLineObj->icr4Values[1],
                pLineObj->icr4Values[2], pLineObj->icr4Values[3], pDevObj->timeStamp));

            VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
                &mpiBuffer[1]);
            mpiIndex = 0;
        }

        if ((devState) && (!myDevState)) {
            /* Only condition where LPM tick is handling final sequence. */
        } else {
            /* Step 3: Delay, then set line to VP_LINE_DISCONNECT. */
            pLineObj->nextSlicValue = VP880_SS_DISCONNECT;
            pLineObj->nextSlicValue |= ((lineInTest == TRUE) ? VP880_SS_ACTIVATE_MASK : 0);

            /* Set Discharge Time based on Supply Configuration. */
            pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                (MS_TO_TICKRATE(Vp880SetDiscTimers(pDevObj),
                pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

            /* Initialize the Disconnect Exit Timer State */
            pLineObj->discTimerExitState = 0;

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Chan %d Setting VP_LINE_DISCONNECT_EXIT time to %d ms (Vp880SetDiscTimers()) at time %d",
                pLineObj->channelId, Vp880SetDiscTimers(pDevObj), pDevObj->timeStamp));
        }

        Vp880LLSetSysState(deviceId, pLineCtx, VP880_SS_DISCONNECT, FALSE);

    } else {    /* Exiting Disconnect */
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Recovering Chan %d from DISCONNECT at time %d with target value 0x%02X",
            channelId, pDevObj->timeStamp, nextSlicByte));

        /*
         * Disabling Feed and Battery Hold allows quicker/smoother transitions
         * out of Disconnect state.
         *
         * Note that the non-LPM Termination Types have these speedup bits set
         * while in Disconnect rather than setting them in the transition.
         * Technically better to have these set while in disconnect because the
         * setting itself requires 1 8kHz frame (125us). This minor delay isn't
         * noticeable of course because we're transitioning from a hold
         * condition of ~30ms to almost immediate.
         */
        pLineObj->icr2Values[VP880_ICR2_SPEEDUP_INDEX] |=
            (VP880_ICR2_MET_SPEED_CTRL | VP880_ICR2_BAT_SPEED_CTRL);
        pLineObj->icr2Values[VP880_ICR2_SPEEDUP_INDEX+1] |=
            (VP880_ICR2_MET_SPEED_CTRL | VP880_ICR2_BAT_SPEED_CTRL);

        pLineObj->lineTimers.timers.timer[VP_LINE_SPEEDUP_RECOVERY_TIMER] =
            MS_TO_TICKRATE(VP880_SPEEDUP_HOLD_TIME,
                pDevObj->devProfileData.tickRate ) | VP_ACTIVATE_TIMER;

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("Chan %d Setting VP_LINE_SPEEDUP_RECOVERY_TIMER time to %d ms (VP880_SPEEDUP_HOLD_TIME) at time %d",
            pLineObj->channelId, VP880_SPEEDUP_HOLD_TIME, pDevObj->timeStamp));

        /* Restore Normal Line Bias Control unless changed by LPM Enter */
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] |=
            VP880_ICR1_LINE_BIAS_OVERRIDE_NORM;

        /*
         * There are four cases to consider when exiting Disconnect:
         *     1. This line is going to VP_LINE_STANDBY and other line is in LPM.
         *     2. This line is going to VP_LINE_STANDBY and other line is not in LPM.
         *     3. This line is going to non-LPM state and other line is in LPM.
         *     4. This line is going to non-LPM state and other line is not in LPM.
         */
        pLineObj->icr2Values[VP880_ICR2_VOC_DAC_INDEX] &= ~VP880_ICR2_VOC_DAC_SENSE;
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX] &= ~VP880_ICR2_SWY_CTRL_EN;
        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &= ~VP880_ICR2_ILA_DAC;

        pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX+1] |= VP880_ICR3_LINE_CTRL;

        if ((nextSlicByte == VP880_SS_IDLE) && (devState)) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Exiting Disconnect Case 1 Channel %d time %d", channelId, pDevObj->timeStamp));

            /*
             * 1. This line is going to VP_LINE_STANDBY and other line is in LPM.
             *      Step 1: Enable Switcher.
             *      Step 2: Set line to Active (for fast charge)
             *      Step 3: Set line to SLIC-Disconnect (end of timer)
             */
            pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] =
                (VP880_ICR2_TIP_SENSE | VP880_ICR2_RING_SENSE);

            /*
             * This should not be required. The only way the floor voltage is
             * not set to -70V is when one of the lines is not in LPM. But if
             * we're exiting to LPM, the other line must either be in LPM-Standby
             * or in Disconnect (state that allows LPM). Meaning, the switcher
             * would not be at -70V only if this line was in a non-LPM state
             * entering Disconnect AND the other line was in Disconnect.
             * However, Case 2 for Entering Disconnect would occur when this line
             * changed from OHT to Disconnect, which would cause the system to
             * enter LPM and set the Switcher to -70V.
             *
             * That said, this small code makes 100% sure that if optimizations
             * are made in the future, that the supply is at the correct voltage
             * when the line is in LPM-Standby.
             */
            if ((pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] & 0x0D) != 0x0D) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                    ("Exiting Disconnect Case 1 Switcher Adjustment Required time %d",
                    pDevObj->timeStamp));

                pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] &= ~VP880_FLOOR_VOLTAGE_MASK;
                pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] |= 0x0D;   /* 70V */
                VP_LINE_STATE(VpDevCtxType, pDevCtx,
                    ("Exiting Disconnect Case 1 Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                    pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                    pDevObj->timeStamp));
                VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_WRT,
                    VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
                /*
                 * If we have to turn on the supply here, wait longer than the normal disconnect
                 * exit time. Normal time is based only on line stability into a 5REN load. Added
                 * time for switcher to fully charge the supply caps/line.
                 */
                VpCSLACSetTimer(&pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE],
                    (MS_TO_TICKRATE(Vp880SetDiscTimers(pDevObj), pDevObj->devProfileData.tickRate)));
                VP_LINE_STATE(VpDevCtxType, pDevCtx,
                    ("Exiting Disconnect Channel %d: Starting VP_DEV_TIMER_LP_CHANGE at time %d",
                    pLineObj->channelId, pDevObj->timeStamp));
            }

            Vp880WriteLPEnterRegisters(pLineCtx, deviceId, ecVal, lineState);

            if (leakyLine) {    /* This condition includes Resistive Leak 'OR' LPM Override */
                pLineObj->nextSlicValue = VP880_SS_IDLE;
            } else {
                pLineObj->nextSlicValue = VP880_SS_DISCONNECT;
                /*
                 * Done entering LPM (exept for final state setting, so set
                 * line flag so the hook bit is not mis-interpreted.
                 */
                pLineObj->status |= VP880_LOW_POWER_EN;
            }

            /*
             * Set flag indicating this line could be in LPM (e.g., line state
             * being set to VP_LINE_STANDBY)
             */
            pDevObj->stateInt |= ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);

        } else if ((nextSlicByte == VP880_SS_IDLE) && (!(devState))) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Exiting Disconnect Case 2 Channel %d time %d",
                channelId, pDevObj->timeStamp));

            /*
             * 2. This line is going to VP_LINE_STANDBY and other line is not in LPM.
             *      Step 1: Enable Switcher on this line
             *      Step 2: Set line to Active (for fast charge)
             *      Step 3: Set remaining ICR registers for LPM exit settings.
             *      Step 4: Set line to SLIC-IDLE (end of timer)
             */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                VP880_ICR2_LEN, pLineObj->icr2Values);

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("LP: Case 2: Channel %d: Writing ICR2 0x%02X 0x%02X 0x%02X 0x%02X time %d",
                pLineObj->channelId,
                pLineObj->icr2Values[0], pLineObj->icr2Values[1],
                pLineObj->icr2Values[2], pLineObj->icr2Values[3], pDevObj->timeStamp));

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("LP: Case 2: Setting Ch %d to State 0x%02X at time %d",
                channelId, lineState[0], pDevObj->timeStamp));

            if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
                Vp880UpdateBufferChanSel(pDevObj, channelId, lineState[0], FALSE);
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DEV_MODE_WRT,
                    VP880_DEV_MODE_LEN, pDevObj->devMode);
            }

            if (pLineObj->slicValueCache != lineState[0]) {
                pLineObj->slicValueCache = lineState[0];
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
            }

            pLineObj->nextSlicValue = nextSlicByte;
            pLineObj->lineState.usrCurrent = VP_LINE_STANDBY;

            VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
                &mpiBuffer[1]);
            mpiIndex = 0;

            Vp880SetLP(FALSE, pLineCtx);

            /*
             * Set flag indicating this line could be in LPM (e.g., line state
             * being set to VP_LINE_STANDBY)
             */
            pDevObj->stateInt |= ((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);
        } else if ((nextSlicByte != VP880_SS_IDLE) && (devState)) {
            /*
             * 3. This line is going to non-LPM state and other line is in LPM.
             *      Step 1: Do nothing. Set the device flag to start LPM exit.
             */
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Exiting Disconnect Case 3: Delaying Disconnect Exit Channel %d at time %d Slic State 0x%02X",
                channelId, pDevObj->timeStamp, nextSlicByte));

            /* Make sure the ICR Values are correct */
            Vp880SetLPRegisters(pLineObj, FALSE);
            Vp880LLSetSysState(deviceId, pLineCtx, nextSlicByte, FALSE);
            pLineObj->nextSlicValue = nextSlicByte;
        } else {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Exiting Disconnect Case 4 Channel %d: All Non-LP time %d",
                channelId, pDevObj->timeStamp));

            /*
             * 4. This line is going to non-LPM state and other line is not in LPM.
             *      Step 1: Enable Switcher on this line
             *      Step 2: Set line to Active w/Polarity (for fast charge)
             *      Step 3: Set remaining ICR registers for LPM exit settings.
             *      Step 4: Set line to new SLIC state (end of timer)
             */
            Vp880SetLPRegisters(pLineObj, FALSE);   /* ICR1, 3, 4 correct */

            /* Prepare for the SLIC Line State change */
            lineState[0] |= (VP880_SS_POLARITY_MASK & nextSlicByte);
            pLineObj->nextSlicValue = nextSlicByte;

            if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
                Vp880UpdateBufferChanSel(pDevObj, channelId, lineState[0], FALSE);
                VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP880_DEV_MODE_WRT,
                    VP880_DEV_MODE_LEN, pDevObj->devMode);
            }

            /* Complete LPM Exit Sequence */
            Vp880WriteLPExitRegisters(pLineCtx, deviceId, ecVal, lineState);

            /*
             * Correct the device flag indicating this line could go to LPM in
             * case being set to VP_LINE_STANDBY.
             */
            if (nextSlicByte != VP880_SS_IDLE) {
                pDevObj->stateInt &= ((channelId == 0) ? ~VP880_LINE0_LP : ~VP880_LINE1_LP);
            }
        }

        /*
         * Disable LPM Exit sequence if for some reason it was previously started
         * (e.g., application quickly went from Standby-OHT-Disconnect). When
         * the Tracker Disable Time expires, it will set the line to Disconnect
         * and Disable the Tracker (ICR2). While some of the steps may be what
         * we're also trying to do, Disconnect handling is done with the
         * Disconnect Exit (which also is Disoconnect Enter) timer.
         */
        pLineObj->lineTimers.timers.timer[VP_LINE_TRACKER_DISABLE]
            &= ~VP_ACTIVATE_TIMER;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880RunLPDisc-"));
}

/**
 * Vp880LowPowerMode()
 *  This function is called when the device should be updated for Low Power
 * mode. It determines if the device can be put into low power mode and does
 * (if it can), sets a flag in the device object, and sets the device timer
 * for hook debounce.
 *
 * Preconditions:
 *
 * Postconditions:
 */
void
Vp880LowPowerMode(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VpLineCtxType *pLineCtx;
    bool isValidCtx[VP880_MAX_NUM_CHANNELS] = {FALSE, FALSE};
    Vp880LineObjectType *pLineObj;
    bool lowPower, switcherSet = FALSE;

    uint8 maxChannels = pDevObj->staticInfo.maxChannels;
    uint8 channelId;

#ifdef VP880_INCLUDE_TESTLINE_CODE
    /* We don't want to interact with the line in this state */
    if (pDevObj->currentTest.nonIntrusiveTest == TRUE) {
        return;
    }
#endif

    /* Don't do anything if device is in calibration */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        return;
    }

    /*
     * Low Power is only possible if both lines can use low power mode.
     * Otherwise, exit.
     */
    if ((pDevObj->stateInt & (VP880_LINE0_LP | VP880_LINE1_LP)) !=
        (VP880_LINE0_LP | VP880_LINE1_LP)) {
        lowPower = FALSE;
    } else {
        lowPower = TRUE;
    }

    /*
     * Determine which lines are valid in case we have to adjust their line
     * states. Consider "valid" only those lines that are FXS Low Power type.
     */
    for (channelId = 0; channelId < maxChannels; channelId++) {
        if (pDevCtx->pLineCtx[channelId] != VP_NULL) {
            pLineCtx = pDevCtx->pLineCtx[channelId];
            pLineObj = pLineCtx->pLineObj;

            if ((!(pLineObj->status & VP880_IS_FXO)) &&
                (VpIsLowPowerTermType(pLineObj->termType))) {
                bool hookStatus = FALSE;
                VpLineStateType currentState;

                isValidCtx[channelId] = TRUE;
                VpCSLACGetLineStatus(pLineCtx, VP_INPUT_RAW_HOOK, &hookStatus);
                /*
                 * Get the line state as seen from the user's perspective. This avoids the function
                 * call and debug output caused by calling the external GetLineState function
                 */
                if (pLineObj->status & VP880_LINE_IN_CAL) {
                    currentState = pLineObj->calLineData.usrState;
                } else {
                    currentState = pLineObj->lineState.usrCurrent;
                }
                /*
                 * Disconnect needs to allow LPM enter for the other line. In
                 * case it was detecting off-hook prior to entering disconnect,
                 * the hook state is frozen. So need to "ignore" it.
                 */
                if ((currentState == VP_LINE_DISCONNECT)
                 || (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER)) {
                    hookStatus = FALSE;
                }

                if ((Vp880IsChnlUndrTst(pDevObj, channelId) == TRUE) ||
                    (pLineObj->status & (VP880_LINE_LEAK | VP880_LP_STANDBY_IDLE)) ||
                    (hookStatus == TRUE)) {
                    lowPower = FALSE;
                }

                /*
                 * Can't go directly from PolRev into LPM because the LPM feed
                 * is too weak to quickly charge up (potential) line capacitance.
                 * The Vp880SetLineStateInt() function needs to detect this
                 * transition and set the SLIC to Active in order to get the
                 * correct polarity started immediately. The PolRev Debounce
                 * timer is defined as the point when the line is stable. So
                 * it can be used to determine when LPM is ok to enter.
                 */
                if ((pLineObj->lineTimers.timers.timer[VP_LINE_HOOK_FREEZE]
                    & VP_ACTIVATE_TIMER) && (lowPower == TRUE)){
                    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Delay LP Enter for PolRev at time %d",
                        pDevObj->timeStamp));
                    return;
                }
            }
        }
    }

    if (lowPower == FALSE) {
        /*
         * Take the device out of low power mode and set channels to correct
         * states. Do not affect device or channels if change has already
         * been made.
         */

        for (channelId = 0; channelId < maxChannels; channelId++) {
            if (isValidCtx[channelId] == TRUE) {
                pLineCtx = pDevCtx->pLineCtx[channelId];
                pLineObj = pLineCtx->pLineObj;

                if (pLineObj->status & VP880_LOW_POWER_EN) {
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LowPowerMode+"));

                    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Exit LPM for Line %d Device Status 0x%08lX Line Status 0x%04X time %d",
                        channelId, pDevObj->stateInt, pLineObj->status, pDevObj->timeStamp));

                    if (switcherSet == FALSE) {
                        VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_PARAM_WRT,
                            VP880_REGULATOR_PARAM_LEN, pDevObj->swParams);
                        VpMemCpy(pDevObj->swParamsCache, pDevObj->swParams,
                            VP880_REGULATOR_PARAM_LEN);
                        VP_LINE_STATE(VpDevCtxType, pDevCtx,
                            ("Exiting LPM Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                            pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                            pDevObj->timeStamp));

                        switcherSet = TRUE;
                    }

                    Vp880SetLP(FALSE, pLineCtx);

                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LowPowerMode-"));
                }
            }
        }
    } else {
        /*
         * We should be in low power mode because both lines can be put into
         * low power mode. Don't need to call Set Line State in this case for
         * each line because there are a limited number of API-II states that
         * can allow Low Power, and all required the SLIC state to be set to
         * Disconnect.
         */
        for (channelId = 0; channelId < maxChannels; channelId++) {
            if (isValidCtx[channelId] == TRUE) {
                pLineCtx = pDevCtx->pLineCtx[channelId];
                pLineObj = pLineCtx->pLineObj;

                if (!(pLineObj->status & VP880_LOW_POWER_EN)) {
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LowPowerMode+"));

                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Enter LPM for Line %d Device Status 0x%08lX Line Status 0x%04X time %d",
                        channelId, pDevObj->stateInt, pLineObj->status, pDevObj->timeStamp));

                    if (switcherSet == FALSE) {
                        pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] &= ~VP880_FLOOR_VOLTAGE_MASK;
                        pDevObj->swParamsCache[VP880_FLOOR_VOLTAGE_BYTE] |= 0x0D;   /* 70V */
                        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP880_REGULATOR_PARAM_WRT,
                            VP880_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
                        VP_LINE_STATE(VpDevCtxType, pDevCtx,
                            ("Enter LPM Switcher Programming: 0x%02X 0x%02X 0x%02X time %d",
                            pDevObj->swParamsCache[0], pDevObj->swParamsCache[1], pDevObj->swParamsCache[2],
                            pDevObj->timeStamp));
                        switcherSet = TRUE;
                    }

                    Vp880SetLP(TRUE, pLineCtx);
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880LowPowerMode-"));
                }
            }
        }
    }
}

/**
 * Vp880SetLP()
 *  This function modifies the line/device to enter/exit LPM.
 *
 * Preconditions:
 *
 * Postconditions:
 */
void
Vp880SetLP(
    bool lpMode,
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint16 debounceTime, deviceTimer;
    uint8 lineState[VP880_SYS_STATE_LEN];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLP+"));

    /*
     * Need timer here to allow switcher to stabilize whether entering or
     * exiting LPM.
     */
    if (pDevObj->swParams[VP880_REGULATOR_TRACK_INDEX] & VP880_REGULATOR_FIXED_RING_SWY) {
        /*
         * Longest is using "fixed" ringing mode because the external
         * capacitors are generally very large.
         */
        debounceTime = VP880_PWR_SWITCH_DEBOUNCE_FXT;
    } else {
        debounceTime = VP880_PWR_SWITCH_DEBOUNCE_FUT;
    }
    /* Increase the debounce time by the amount of the Switch Hook debounce time. Note that Switch
     * Hook is in 2ms steps, so multiply by 2 to get the debounce value in ms. */
    debounceTime +=
        ((pLineObj->loopSup[VP880_LOOP_SUP_DEBOUNCE_BYTE] & VP880_SWHOOK_DEBOUNCE_MASK) << 1);

    deviceTimer = MS_TO_TICKRATE(debounceTime, pDevObj->devProfileData.tickRate);

    /*
     * If the Disconnect Exit Timer is active, make sure the power timer waits
     * until after the Disconnect Exit Timer expires.
     */
    if (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & VP_ACTIVATE_TIMER) {
        uint16 currentDiscTime =
            pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] & ~VP_ACTIVATE_TIMER;
        deviceTimer = ((currentDiscTime < deviceTimer) ? deviceTimer : (currentDiscTime + 1));
    }

    /*
     * In case this timer is being modified by both lines (corner case of state change timing and
     * seqence), make sure a previous longer value is not reduced.
     */
    VpCSLACSetTimer(&pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE], deviceTimer);
    VP_LINE_STATE(VpDevCtxType, pDevCtx,
        ("Vp880SetLP() Channel %d: Starting VP_DEV_TIMER_LP_CHANGE with %d ms at time %d",
        pLineObj->channelId,
        TICKS_TO_MS((pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE] & ~VP_ACTIVATE_TIMER),
                    pDevObj->devProfileData.tickRate), pDevObj->timeStamp));

    if (lpMode == FALSE) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                      ("Taking Channel %d out of Low Power Mode at time %d User State %d",
                       pLineObj->channelId, pDevObj->timeStamp, pLineObj->lineState.usrCurrent));

        pLineObj->status &= ~VP880_LOW_POWER_EN;

        if (pLineObj->lineState.usrCurrent != VP_LINE_DISCONNECT) {
            bool internalApiOperation = FALSE;

            Vp880SetLPRegisters(pLineObj, FALSE);

#ifdef VP_CSLAC_SEQ_EN
            if ((pLineObj->cadence.status & (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_SENDSIG)) ==
                (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_SENDSIG)) {
                internalApiOperation = TRUE;
            }
#endif

            if ((pLineObj->lineState.usrCurrent == VP_LINE_STANDBY)
             && (internalApiOperation == FALSE)) {
                lineState[0] = VP880_SS_ACTIVE;
                Vp880WriteLPExitRegisters(pLineCtx, deviceId, ecVal, lineState);
                pLineObj->nextSlicValue = VP880_SS_IDLE;
            } else {
                Vp880WriteLPExitRegisters(pLineCtx, deviceId, ecVal, VP_NULL);
            }
       }

        /*
         * Disable LPM Exit sequence. When the Tracker Disable Time expires, it
         * will set the line to Disconnect and Disable the Tracker (ICR2).
         * Obviously, we no longer want to do this.
         */
        pLineObj->lineTimers.timers.timer[VP_LINE_TRACKER_DISABLE]
            &= ~VP_ACTIVATE_TIMER;
    } else {
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Putting Channel %d in Low Power Mode at time %d",
            pLineObj->channelId, pDevObj->timeStamp));

        pLineObj->status |= VP880_LOW_POWER_EN;

        if (pLineObj->lineState.usrCurrent != VP_LINE_DISCONNECT) {
            /* Set timer to wait before making final changes. When this timer
             * expires, the following sequence is run:
             *
             *  - Set SLIC state to Disconnect
             *  - Write the following:
             *      icr2[VP880_ICR2_VOC_DAC_INDEX] |= VP880_ICR2_ILA_DAC;
             *      icr2[VP880_ICR2_VOC_DAC_INDEX] &= ~VP880_ICR2_VOC_DAC_SENSE;
             *      icr2[VP880_ICR2_VOC_DAC_INDEX+1] &= ~VP880_ICR2_ILA_DAC;
             */
            pLineObj->lineTimers.timers.timer[VP_LINE_TRACKER_DISABLE] =
                (MS_TO_TICKRATE(VP880_TRACKER_DISABLE_TIME,
                    pDevObj->devProfileData.tickRate)) | VP_ACTIVATE_TIMER;

            /*
             * We are entering LPM into VP_LINE_STANDBY. So the required ICR
             * values are not yet set in the line object (as they would be if
             * entering from VP_LINE_DISCONNECT).
             */
            lineState[0] = VP880_SS_DISCONNECT;

            Vp880SetLPRegisters(pLineObj, TRUE);
        } else {
            uint16 dischargeTime = Vp880SetDiscTimers(pDevObj);

            /*
             * We are entering LPM into VP_LINE_DISCONNECT. So the required ICR
             * values have been set in the line object, just need to force the
             * required sequence (Active w/polarity, then Disconnect).
             */
            lineState[0] = VP880_SS_ACTIVE;
            pLineObj->nextSlicValue = VP880_SS_DISCONNECT;

            /* Set Discharge Time based on Supply Configuration. */
            if (dischargeTime < VP880_TRACKER_DISABLE_TIME) {
                dischargeTime = VP880_TRACKER_DISABLE_TIME;
            }

            /* Set timer to wait before making final changes. When this timer
             * expires, the following sequence is run:
             *
             *  - Set SLIC state to Disconnect
             *  - Write the following:
             *      icr2[VP880_ICR2_VOC_DAC_INDEX] |= VP880_ICR2_ILA_DAC;
             *      icr2[VP880_ICR2_VOC_DAC_INDEX] &= ~VP880_ICR2_VOC_DAC_SENSE;
             *      icr2[VP880_ICR2_VOC_DAC_INDEX+1] &= ~VP880_ICR2_ILA_DAC;
             */
            pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT] =
                (MS_TO_TICKRATE(dischargeTime, pDevObj->devProfileData.tickRate))
                | VP_ACTIVATE_TIMER;

            /* Initialize the Disconnect Exit Timer State */
            pLineObj->discTimerExitState = 0;

            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("Chan %d Setting VP_LINE_DISCONNECT_EXIT time to %d ms (dischargeTime) at time %d",
                pLineObj->channelId, dischargeTime, pDevObj->timeStamp));
        }

        Vp880WriteLPEnterRegisters(pLineCtx, deviceId, ecVal,
            lineState);
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLP-"));
}

/**
 * Vp880WriteLPExitRegisters()
 *  This function writes the ICR and State values to the device for LPM exit.
 *
 * Preconditions:
 *  None. Modification of line object data only.
 *
 * Postconditions:
 *  The device registers have been modified.
 */
void
Vp880WriteLPExitRegisters(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 *lineState)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 mpiBuffer[6 + VP880_DEV_MODE_LEN + VP880_SYS_STATE_LEN +
                        VP880_ICR1_LEN + VP880_ICR2_LEN + VP880_ICR3_LEN +
                        VP880_ICR4_LEN];
    uint8 mpiIndex = 0;

    if (pLineObj->lineState.currentState == VP_LINE_TIP_OPEN) {
        Vp880SetLineStateInt(pLineCtx, pLineObj->lineState.currentState);
        return;
    }

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
        VP880_ICR3_LEN, pLineObj->icr3Values);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
        VP880_ICR4_LEN, pLineObj->icr4Values);

    if (lineState == VP_NULL) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP Exit: Writing ICR3 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
            pLineObj->icr3Values[0], pLineObj->icr3Values[1],
            pLineObj->icr3Values[2], pLineObj->icr3Values[3],
            pLineObj->channelId, pDevObj->timeStamp));

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP Exit: Writing ICR4 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
            pLineObj->icr4Values[0], pLineObj->icr4Values[1],
            pLineObj->icr4Values[2], pLineObj->icr4Values[3],
            pLineObj->channelId, pDevObj->timeStamp));

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP Exit: Writing LineState %d on Ch %d time %d",
            pLineObj->lineState.currentState, pLineObj->channelId, pDevObj->timeStamp));

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
            &mpiBuffer[1]);
        mpiIndex = 0;

        Vp880SetLineStateInt(pLineCtx, pLineObj->lineState.currentState);
    } else {
        if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
            Vp880UpdateBufferChanSel(pLineCtx->pDevCtx->pDevObj, pLineObj->channelId,
                lineState[0], FALSE);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DEV_MODE_WRT,
                VP880_DEV_MODE_LEN, pDevObj->devMode);
        }

        if (pLineObj->slicValueCache != lineState[0]) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx,
                ("LP Exit: Writing SLIC State 0x%02X on Ch %d time %d",
                lineState[0], pLineObj->channelId, pDevObj->timeStamp));

            pLineObj->slicValueCache = lineState[0];
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
        }
    }

    mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
        VP880_ICR2_LEN, pLineObj->icr2Values);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
        &mpiBuffer[1]);

    if (lineState != VP_NULL) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP Exit: Writing ICR3 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
             pLineObj->icr3Values[0], pLineObj->icr3Values[1],
             pLineObj->icr3Values[2], pLineObj->icr3Values[3],
             pLineObj->channelId, pDevObj->timeStamp));

        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("LP Exit: Writing ICR4 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
             pLineObj->icr4Values[0], pLineObj->icr4Values[1],
             pLineObj->icr4Values[2], pLineObj->icr4Values[3],
             pLineObj->channelId, pDevObj->timeStamp));
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("LP Exit: Writing ICR2 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
        pLineObj->icr2Values[0], pLineObj->icr2Values[1],
        pLineObj->icr2Values[2], pLineObj->icr2Values[3],
        pLineObj->channelId, pDevObj->timeStamp));
}

/**
 * Vp880WriteLPEnterRegisters()
 *  This function writes the ICR and State values to the device for LPM enter.
 *
 * Preconditions:
 *  None. Modification of line object data only.
 *
 * Postconditions:
 *  The device registers have been modified.
 */
void
Vp880WriteLPEnterRegisters(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 *lineState)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 mpiBuffer[6 + VP880_DEV_MODE_LEN + VP880_SYS_STATE_LEN +
                        VP880_ICR1_LEN + VP880_ICR2_LEN + VP880_ICR3_LEN +
                        VP880_ICR4_LEN];
    uint8 mpiIndex = 0;

    mpiIndex = Vp880ProtectedWriteICR1(pLineObj, mpiIndex, mpiBuffer);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
        VP880_ICR2_LEN, pLineObj->icr2Values);

    if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
        Vp880UpdateBufferChanSel(pLineCtx->pDevCtx->pDevObj, pLineObj->channelId,
            lineState[0], FALSE);
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DEV_MODE_WRT,
            VP880_DEV_MODE_LEN, pDevObj->devMode);
    }

    pLineObj->slicValueCache = lineState[0];
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
        VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
        VP880_ICR3_LEN, pLineObj->icr3Values);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
        VP880_ICR4_LEN, pLineObj->icr4Values);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
        &mpiBuffer[1]);

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("LP Enter: Writing ICR2 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
        pLineObj->icr2Values[0], pLineObj->icr2Values[1],
        pLineObj->icr2Values[2], pLineObj->icr2Values[3],
        pLineObj->channelId, pDevObj->timeStamp));

    /* Set line to desired state */
    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("LP Enter: Setting State 0x%02X on Ch %d time %d",
        lineState[0], pLineObj->channelId, pDevObj->timeStamp));

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("LP Enter: Writing ICR3 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
        pLineObj->icr3Values[0], pLineObj->icr3Values[1],
        pLineObj->icr3Values[2], pLineObj->icr3Values[3],
        pLineObj->channelId, pDevObj->timeStamp));

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("LP Enter: Writing ICR4 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d time %d",
        pLineObj->icr4Values[0], pLineObj->icr4Values[1],
        pLineObj->icr4Values[2], pLineObj->icr4Values[3],
        pLineObj->channelId, pDevObj->timeStamp));
}

/**
 * Vp880SetLPRegisters()
 *  This function modifies the line object ICR register values. It does not
 * write to the device.
 *
 * Preconditions:
 *  None. Modification of line object data only.
 *
 * Postconditions:
 *  The line object data (ICR values) have been modified.
 */
void
Vp880SetLPRegisters(
    Vp880LineObjectType *pLineObj,
    bool lpModeTo)
{
    VP_API_FUNC_INT(None, VP_NULL, ("Vp880SetLPRegisters+"));

    if (lpModeTo == TRUE) {
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] =
            VP880_ICR1_LINE_BIAS_OVERRIDE;
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION+1] =
            VP880_ICR1_LINE_BIAS_OVERRIDE_NORM;

        pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION] &=
            ~VP880_ICR1_RING_BIAS_DAC_MASK;
        pLineObj->icr1Values[VP880_ICR1_RING_AND_DAC_LOCATION+1] &=
            ~VP880_ICR1_RING_BIAS_DAC_MASK;

        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] =
            (VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE |
             VP880_ICR2_TIP_SENSE | VP880_ICR2_RING_SENSE);

        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX+1] =
            (VP880_ICR2_FEED_SENSE | VP880_ICR2_TIP_SENSE | VP880_ICR2_RING_SENSE);

        /*
         * Preclear all controls except SWY, Battery Speedup and DC Feed
         * Speedup (set if exiting Disconnect)
         */
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX] &=
            (VP880_ICR2_SWY_LIM_CTRL1 | VP880_ICR2_SWY_LIM_CTRL
            | VP880_ICR2_MET_SPEED_CTRL | VP880_ICR2_BAT_SPEED_CTRL);

        /* Make sure SWY controls are set */
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX] |=
            (VP880_ICR2_SWY_LIM_CTRL1 | VP880_ICR2_SWY_LIM_CTRL);
        pLineObj->icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] |=
            (VP880_ICR2_SWY_LIM_CTRL1);

        pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX] |= VP880_ICR3_LINE_CTRL;
        pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX+1] |= VP880_ICR3_LINE_CTRL;

        pLineObj->icr4Values[VP880_ICR4_SUP_INDEX] |=
            (VP880_ICR4_SUP_DAC_CTRL | VP880_ICR4_SUP_DET_CTRL | VP880_ICR4_SUP_POL_CTRL);
        pLineObj->icr4Values[VP880_ICR4_SUP_INDEX+1] |=
            (VP880_ICR4_SUP_DAC_CTRL | VP880_ICR4_SUP_DET_CTRL | VP880_ICR4_SUP_POL_CTRL);
    } else {
        pLineObj->icr3Values[VP880_ICR3_LINE_CTRL_INDEX] &= ~VP880_ICR3_LINE_CTRL;

        pLineObj->icr4Values[VP880_ICR4_SUP_INDEX] &=
            (uint8)(~(VP880_ICR4_SUP_DAC_CTRL | VP880_ICR4_SUP_DET_CTRL | VP880_ICR4_SUP_POL_CTRL));

        /* Remove previously set SW control of ICR1 */
        pLineObj->icr1Values[VP880_ICR1_BIAS_OVERRIDE_LOCATION] &= ~VP880_ICR1_LINE_BIAS_OVERRIDE;

        pLineObj->icr2Values[VP880_ICR2_SENSE_INDEX] &=
            (uint8)(~(VP880_ICR2_RING_SENSE | VP880_ICR2_TIP_SENSE |
                      VP880_ICR2_DAC_SENSE | VP880_ICR2_FEED_SENSE));
    }

    VP_LINE_STATE(None, VP_NULL, ("LP %s Cache: ICR1 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
        ((lpModeTo == TRUE) ? "Enter" : "Exit"),
        pLineObj->icr1Values[0], pLineObj->icr1Values[1],
        pLineObj->icr1Values[2], pLineObj->icr1Values[3], pLineObj->channelId));

    VP_LINE_STATE(None, VP_NULL, ("LP %s Cache: ICR2 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
        ((lpModeTo == TRUE) ? "Enter" : "Exit"),
        pLineObj->icr2Values[0], pLineObj->icr2Values[1],
        pLineObj->icr2Values[2], pLineObj->icr2Values[3], pLineObj->channelId));

    VP_LINE_STATE(None, VP_NULL, ("LP %s Cache: ICR3 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
        ((lpModeTo == TRUE) ? "Enter" : "Exit"),
        pLineObj->icr3Values[0], pLineObj->icr3Values[1],
        pLineObj->icr3Values[2], pLineObj->icr3Values[3], pLineObj->channelId));

    VP_LINE_STATE(None, VP_NULL, ("LP %s Cache: ICR4 0x%02X 0x%02X 0x%02X 0x%02X on Ch %d",
        ((lpModeTo == TRUE) ? "Enter" : "Exit"),
        pLineObj->icr4Values[0], pLineObj->icr4Values[1],
        pLineObj->icr4Values[2], pLineObj->icr4Values[3], pLineObj->channelId));

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880SetLPRegisters-"));
}

#endif
