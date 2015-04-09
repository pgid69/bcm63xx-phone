/** \file vp880_tracker_calibration.c
 * vp880_tracker_calibration.c
 *
 * This file contains the tracker device calibration functions for the Vp880 device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 10725 $
 * $LastChangedDate: 2013-01-24 18:41:02 -0600 (Thu, 24 Jan 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES) && defined (VP880_TRACKER_SUPPORT)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED

/* Functions that are called only inside this file. */
static void Vp880AbvInit(VpDevCtxType *pDevCtx);
static void Vp880AbvSetAdc(VpDevCtxType *pDevCtx);
static void Vp880AbvStateChange(VpDevCtxType *pDevCtx);
static void Vp880AbvMeasure(VpDevCtxType *pDevCtx);

#endif /* VP_CSLAC_RUNTIME_CAL_ENABLED */

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED

/**
 * Vp880CalAbv() -- Tracker Only Function
 *  This function initiates a calibration operation for Absolute Switcher circuits associated with
 * all the lines of a device. See VP-API reference guide for more information. SWYV SWZV are global
 * for every Channels Line must be in Disconnect state before to start the Calibration
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp880CalAbv(
     VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType  status = VP_STATUS_SUCCESS;
    uint8 ecVal;
    bool calCleanup = FALSE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalAbv+"));

    if ((pDevObj->stateInt & VP880_IS_SINGLE_CHANNEL) || (pDevObj->stateInt & VP880_LINE1_IS_FXO)) {
        ecVal = VP880_EC_CH1;
    } else {
        ecVal = VP880_EC_CH1 | VP880_EC_CH2;
    }

    if (pDevObj->calData.calDeviceState == VP880_CAL_INIT
     || pDevObj->calData.calDeviceState == VP880_CAL_EXIT) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Vp880CalAbv:  - Setting to Vp880CalInit at time %d", pDevObj->timeStamp));

        pDevObj->calData.calDeviceState = VP880_CAL_INIT;
        Vp880CalInit(pDevCtx);
    }

    switch(pDevObj->calData.calDeviceState) {
        case VP880_CAL_INIT:
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp880CalAbv: - Running Vp880AbvInit at time %d",
                pDevObj->timeStamp));
            Vp880AbvInit(pDevCtx);
            break;

        case VP880_CAL_ADC:
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp880CalAbv: - Running Vp880AbvSetAdc at time %d",
                pDevObj->timeStamp));
            Vp880AbvSetAdc(pDevCtx);
            break;

        case VP880_CAL_STATE_CHANGE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp880CalAbv: - Running Vp880AbvStateChange at time %d",
                pDevObj->timeStamp));
            Vp880AbvStateChange(pDevCtx);
            break;

        case VP880_CAL_MEASURE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp880CalAbv - Running Vp880AbvMeasure at time %d",
                pDevObj->timeStamp));
            Vp880AbvMeasure(pDevCtx);
            break;

        case VP880_CAL_DONE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("ABV Cal Done at time %d", pDevObj->timeStamp));
            calCleanup = TRUE;
            break;

        case VP880_CAL_ERROR:
            /* Fall through intentional */
        default:
            VP_ERROR(VpDevCtxType, pDevCtx,
                ("Vp880CalAbv: ERROR - Cal Done at time %d", pDevObj->timeStamp));
            calCleanup = TRUE;
            status = VP_STATUS_FAILURE;
            pDevObj->responseData = VP_CAL_FAILURE;
            break;
    } /* end of switch(pDevObj->calData.calDeviceState) */

    if (calCleanup == TRUE) {
        uint8 channelId;
        uint8 convCfg[VP880_CONV_CFG_LEN];
        uint8 mpiBuffer[9 + VP880_SYS_STATE_LEN + VP880_ICR1_LEN
                          + VP880_ICR2_LEN + VP880_ICR3_LEN + VP880_ICR4_LEN
                          + VP880_DISN_LEN + VP880_VP_GAIN_LEN
                          + VP880_OP_FUNC_LEN + VP880_CONV_CFG_LEN];
        uint8 mpiIndex = 0;

        convCfg[0] = (VP880_METALLIC_AC_V | pDevObj->txBufferDataRate);

        pDevObj->calData.calDeviceState = VP880_CAL_EXIT;
        if (pDevObj->state & VP_DEV_INIT_IN_PROGRESS) {
            pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
            pDevObj->state |= VP_DEV_INIT_CMP;
        } else {
            pDevObj->deviceEvents.response |= VP_EVID_CAL_CMP;
        }
        pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_IN_CAL);
        pDevObj->state &= ~VP_DEV_ABV_CAL;

        if (pDevObj->responseData == VP_CAL_FAILURE) {
            pDevObj->stateInt &= ~VP880_DEVICE_CAL_COMPLETE;
        } else {
            pDevObj->stateInt |= VP880_DEVICE_CAL_COMPLETE;
        }

        /* Restore device mode to test buffer if exists */
        if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
            pDevObj->devMode[0] |= VP880_DEV_MODE_TEST_DATA;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
                            pDevObj->devMode);
        }

        for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
            VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];

            mpiIndex = 0;
            ecVal = (channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2;

            /* Restore slic state -- could be FXO also */
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Setting to State 0x%02X at time %d",
                channelId, pDevObj->calData.abvData.sysState[channelId][0], pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, &pDevObj->calData.abvData.sysState[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing ICR1 to 0x%02X 0x%02X 0x%02X 0x%02X",
                channelId,
                pDevObj->calData.abvData.icr1[channelId][0],
                pDevObj->calData.abvData.icr1[channelId][1],
                pDevObj->calData.abvData.icr1[channelId][2],
                pDevObj->calData.abvData.icr1[channelId][3]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT,
                VP880_ICR1_LEN, &pDevObj->calData.abvData.icr1[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing ICR2 to 0x%02X 0x%02X 0x%02X 0x%02X",
                channelId,
                pDevObj->calData.abvData.icr2[channelId][0],
                pDevObj->calData.abvData.icr2[channelId][1],
                pDevObj->calData.abvData.icr2[channelId][2],
                pDevObj->calData.abvData.icr2[channelId][3]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                VP880_ICR2_LEN, &pDevObj->calData.abvData.icr2[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing ICR3 to 0x%02X 0x%02X 0x%02X 0x%02X",
                channelId,
                pDevObj->calData.abvData.icr3[channelId][0],
                pDevObj->calData.abvData.icr3[channelId][1],
                pDevObj->calData.abvData.icr3[channelId][2],
                pDevObj->calData.abvData.icr3[channelId][3]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT,
                VP880_ICR3_LEN, &pDevObj->calData.abvData.icr3[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing ICR4 to 0x%02X 0x%02X 0x%02X 0x%02X",
                channelId,
                pDevObj->calData.abvData.icr4[channelId][0],
                pDevObj->calData.abvData.icr4[channelId][1],
                pDevObj->calData.abvData.icr4[channelId][2],
                pDevObj->calData.abvData.icr4[channelId][3]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT,
                VP880_ICR4_LEN, &pDevObj->calData.abvData.icr4[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing DISN to 0x%02X",
                channelId, pDevObj->calData.abvData.disnVal[channelId][0]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DISN_WRT,
                VP880_DISN_LEN, &pDevObj->calData.abvData.disnVal[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing VP Gain to 0x%02X",
                channelId, pDevObj->calData.abvData.vpGain[channelId][0]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_VP_GAIN_WRT,
                VP880_VP_GAIN_LEN, &pDevObj->calData.abvData.vpGain[channelId][0]);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing OP FUNC to 0x%02X",
                channelId, pDevObj->calData.abvData.opFunc[channelId][0]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_OP_FUNC_WRT,
                VP880_OP_FUNC_LEN, &pDevObj->calData.abvData.opFunc[channelId][0]);

            /* Restore Converter Configuration */
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("CH %d: Calibration Cleanup -- Writing CONVERTER to 0x%02X",
                channelId, convCfg[0]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_CONV_CFG_WRT,
                VP880_CONV_CFG_LEN, convCfg);

            /* send down the mpi commands */
            VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

            /*
             * The Disconnect Pending flag will be set if the line started a disconnect sequence
             * and before the timer completed then started this calibration. In fact, this is
             * normal when coming out of VpInitDevice(). So once all other registers are restored
             * we have to complete the disconnect sequence.
             */
            if (pDevObj->state & VP_DEV_DISC_PENDING) {
                if (pLineCtx != VP_NULL) {
                    Vp880ServiceDiscExitTimer(pLineCtx);
                }
            }
        }   /* end of loop "for (channelId = 0; ...)" */
        pDevObj->state &= ~VP_DEV_DISC_PENDING;

    } /* eid of "if (calCleanup == TRUE)" */

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalAbv-"));
    return status;
}

/**
 * Vp880AbvInit() -- Tracker (ABV Cal) only Function
 *  This function initiates a calibration operation for ABV associated with all the lines of a
 * device. See VP-API reference guide for more information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
void
Vp880AbvInit(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 disnVal[VP880_DISN_LEN] = {0x00};
    uint8 vpGain[VP880_VP_GAIN_LEN] = {0x00};
    uint8 channelId;

    uint8 isrpMods[VP880_INT_SWREG_PARAM_LEN] = {
        0x00, 0x40, 0x00, 0x40, 0x00, 0x40
    };

    uint8 icr1[VP880_ICR1_LEN] = {0x00, 0x00, 0x00, 0x00};
    uint8 icr2[VP880_ICR2_LEN] = {0x00, 0xEC, 0x2C, 0x2C};
    uint8 icr3[VP880_ICR3_LEN] = {0x30, 0x20, 0x00, 0x00};
    uint8 icr4[VP880_ICR4_LEN] = {0x01, 0x01, 0x00, 0x00};

    uint8 data, ecVal;
    uint8 swCal[VP880_BAT_CALIBRATION_LEN];

    uint8 mpiBuffer[9 + VP880_ICR2_LEN + VP880_SYS_STATE_LEN + VP880_ICR3_LEN +
                        VP880_ICR4_LEN + VP880_OP_FUNC_LEN + VP880_OP_COND_LEN +
                        VP880_DISN_LEN + VP880_VP_GAIN_LEN + VP880_ICR1_LEN];
    uint8 mpiIndex = 0;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvInit+"));

    /* Initialize and use to measure each channels offset and voltage using same functions. */
    pDevObj->calData.abvData.passCnt = 0;

    /* Channel specific registers to restore at end of calibration */
    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
        ecVal = (channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2;

        /* Save off current slic state */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_RD, VP880_SYS_STATE_LEN,
            &pDevObj->calData.abvData.sysState[channelId][0]);

        /* Disable switcher by setting duty cycle = 0. Global, so only need to do once. */
        if (channelId == 0) {
            VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_SWREG_PARAM_WRT,
                VP880_INT_SWREG_PARAM_LEN, isrpMods);
        }

        /* Clear existing correction factors */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_BAT_CALIBRATION_RD,
            VP880_BAT_CALIBRATION_LEN, swCal);
        swCal[0] &= ~(VP880_BAT_CAL_SWCAL_MASK);
        VpMpiCmdWrapper(deviceId, ecVal, VP880_BAT_CALIBRATION_WRT,
            VP880_BAT_CALIBRATION_LEN, swCal);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR1_RD, VP880_ICR1_LEN,
            &pDevObj->calData.abvData.icr1[channelId][0]);
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Saving ICR1 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
            pDevObj->calData.abvData.icr1[channelId][0],
            pDevObj->calData.abvData.icr1[channelId][1],
            pDevObj->calData.abvData.icr1[channelId][2],
            pDevObj->calData.abvData.icr1[channelId][3],
            channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_RD, VP880_ICR2_LEN,
            &pDevObj->calData.abvData.icr2[channelId][0]);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Saving ICR2 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
            pDevObj->calData.abvData.icr2[channelId][0],
            pDevObj->calData.abvData.icr2[channelId][1],
            pDevObj->calData.abvData.icr2[channelId][2],
            pDevObj->calData.abvData.icr2[channelId][3],
            channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR3_RD, VP880_ICR3_LEN,
            &pDevObj->calData.abvData.icr3[channelId][0]);
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Saving ICR3 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
            pDevObj->calData.abvData.icr3[channelId][0],
            pDevObj->calData.abvData.icr3[channelId][1],
            pDevObj->calData.abvData.icr3[channelId][2],
            pDevObj->calData.abvData.icr3[channelId][3],
            channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR4_RD, VP880_ICR4_LEN,
            &pDevObj->calData.abvData.icr4[channelId][0]);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Saving ICR4 0x%02X 0x%02X 0x%02X 0x%02X Ch %d",
            pDevObj->calData.abvData.icr4[channelId][0],
            pDevObj->calData.abvData.icr4[channelId][1],
            pDevObj->calData.abvData.icr4[channelId][2],
            pDevObj->calData.abvData.icr4[channelId][3],
            channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_DISN_RD, VP880_DISN_LEN,
            &pDevObj->calData.abvData.disnVal[channelId][0]);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,("Saving DISN 0x%02X Ch %d",
            pDevObj->calData.abvData.disnVal[channelId][0],
            channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_VP_GAIN_RD, VP880_VP_GAIN_LEN,
            &pDevObj->calData.abvData.vpGain[channelId][0]);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,("Saving VP Gain 0x%02X Ch %d",
            pDevObj->calData.abvData.vpGain[channelId][0],
            channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN,
            &pDevObj->calData.abvData.opFunc[channelId][0]);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,("Saving OP FUNC 0x%02X Ch %d",
            pDevObj->calData.abvData.opFunc[channelId][0],
            channelId));
    }

    if ((pDevObj->stateInt & VP880_IS_SINGLE_CHANNEL)
     || (pDevObj->stateInt & VP880_LINE1_IS_FXO)) {
        ecVal = VP880_EC_CH1;
    } else {
        ecVal = VP880_EC_CH1 | VP880_EC_CH2;
    }

    /* Disable the switchers */
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Writing ICR2 0x%02X 0x%02X 0x%02X 0x%02X BOTH channels",
         icr2[0], icr2[1], icr2[2], icr2[3]));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT, VP880_ICR2_LEN, icr2);

    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Writing ICR1 0x%02X 0x%02X 0x%02X 0x%02X BOTH channels",
        icr1[0], icr1[1], icr1[2], icr1[3]));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR1_WRT, VP880_ICR1_LEN, icr1);

    /* Force sink supply current to reduce voltage */
    data = VP880_SS_ACTIVE;
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: Setting Ch %d to State 0x%02X at time %d BOTH channels",
        channelId, data, pDevObj->timeStamp));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
        VP880_SYS_STATE_LEN, &data);

    /* Enable line control to access VBAT sense */
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("1. Calibration: Write ICR3 0x%02X 0x%02X 0x%02X 0x%02X BOTH channels",
        icr3[0], icr3[1], icr3[2], icr3[3]));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR3_WRT, VP880_ICR3_LEN, icr3);

    /* Enable ADC */
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: Write ICR4 0x%02X 0x%02X 0x%02X 0x%02X BOTH channels",
        icr4[0], icr4[1], icr4[2], icr4[3]));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR4_WRT, VP880_ICR4_LEN, icr4);

    /* Set compression to Linear Mode and default AC Coefficients */
    data = VP880_LINEAR_CODEC;
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_OP_FUNC_WRT,
        VP880_OP_FUNC_LEN, &data);
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: Force OP Func to 0x%02X BOTH channels", data));

    /* Cut TX/RX PCM and disable HPF */
    data = (VP880_CUT_TXPATH | VP880_CUT_RXPATH | VP880_HIGH_PASS_DIS);
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
        VP880_OP_COND_WRT, VP880_OP_COND_LEN, &data);
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: Force OP Cond to 0x%02X BOTH channels", data));

    /* Set DISN = 0 and Voice Path Gain to 0dB. */
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_DISN_WRT, VP880_DISN_LEN, disnVal);
     VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: DISN to 0x%02X BOTH channels", disnVal[0]));

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_VP_GAIN_WRT,
        VP880_VP_GAIN_LEN, vpGain);

    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: VP Gain to 0x%02X BOTH channels", vpGain[0]));

    /* send down the mpi commands */
    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

    /* Wait at least 100ms before collecting data */
    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
        MS_TO_TICKRATE(VP880_CAL_ABV_LONG, pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    /* Advance state to measure ADC offset */
    pDevObj->calData.calDeviceState = VP880_CAL_STATE_CHANGE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvInit-"));
} /* end Vp880AbvInit */

/**
 * Vp880AbvStateChange () -- Tracker (ABV Cal) only function
 *  This function changes the line state and sets the converter configuration in order to give time
 * for the converter to stabilize before taking the first set of data.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling this function.
 *
 * Postconditions:
 */
void
Vp880AbvStateChange(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 data, ecVal;
    uint8 converterCfg[VP880_CONV_CFG_LEN] = {VP880_SWITCHER_Y};

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvStateChange+"));

    if ((pDevObj->stateInt & VP880_IS_SINGLE_CHANNEL)
     || (pDevObj->stateInt & VP880_LINE1_IS_FXO)) {
        ecVal = VP880_EC_CH1;
    } else {
        ecVal = VP880_EC_CH1 | VP880_EC_CH2;
    }

    /* Help discharge the battery */
    data = (VP880_SS_ACTIVE | VP880_SS_ACTIVATE_MASK);
    VP_CALIBRATION(VpDevCtxType, pDevCtx,
        ("Calibration: Setting ALL_LINES to State 0x%02X at time %d", data, pDevObj->timeStamp));
    VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &data);

    /* Don't care about the data, just force the converter configuration */
    VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN, converterCfg);
    /*
     * Force bad initial sample to force decay check to fail. This should be a reasonably high value
     * (far above any reasonable offset) to force the first test to fail.
     */
    pDevObj->vp880SysCalData.swyOffset[0] = 13000L;

    /* Reset the iteration value used later to avoid infinite retries */
    pDevObj->calData.iteration = 0;

    /*
     * Allow the converter to stabilize and most line conditions to fully
     * discharge the battery. Note that the very first check of the data will
     * fail because the preset value will be much different than the new value.
     * The decay algorithm takes that into account and sets the first delay
     * based on iteration.
     */
    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
        MS_TO_TICKRATE(VP880_CAL_ABV_LONG,
        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    pDevObj->calData.calDeviceState = VP880_CAL_ADC;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvStateChange-"));
} /* end Vp880AbvStateChange */

/**
 * Vp880AbvSetAdc() -- Tracker (ABV Cal) only function
 *  This function set the converter to read the right pcm set the right state machine
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling this function.
 *
 * Postconditions:
 */
void
Vp880AbvSetAdc(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal, channelId;
    uint8 swYZ[VP880_REGULATOR_PARAM_LEN];
    bool abvSetAdcDone = FALSE;
    uint8 converterCfg[VP880_CONV_CFG_LEN] = {VP880_SWITCHER_Z};
    bool validData;
    int16 newValue;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvSetAdc+"));

    if ((pDevObj->stateInt & VP880_IS_SINGLE_CHANNEL)
     || (pDevObj->stateInt & VP880_LINE1_IS_FXO)) {
        ecVal = VP880_EC_CH1;
    } else {
        ecVal = VP880_EC_CH1 | VP880_EC_CH2;
    }

    pDevObj->calData.iteration++;

    /* Now we'll switch to channel specific measurements */
    /* Read SWY from first channel, SWZ from second channel */
    if (pDevObj->calData.abvData.passCnt == 0) {
        newValue = Vp880AdcSettling(pDevObj, VP880_EC_CH1, VP880_SWITCHER_Y, &validData)
                 + VP880_TRACKER_BAT_OFFSET;

        if (validData == FALSE) {
            /*
             * Data is bad. Need to reset converter (done automatically in
             * ADC Settling function) and repeat measurement.
             */
            if (pDevObj->calData.iteration < 10) {
                pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                    MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                return;
            } else {
                /* Repeated as much as we can. Set 0V offset and move on. */
                newValue = 0;
            }
        } else if ((ABS(pDevObj->vp880SysCalData.swyOffset[0] - newValue) > VP880_V_1V_PCM)
                && (ABS(newValue) > VP880_V_1V_PCM)) {
            /* We're in a voltage decay. Need to wait longer to settle. */
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Voltage Decay Detected: Old Value %d New Value %d",
                pDevObj->vp880SysCalData.swyOffset[0], newValue));

            pDevObj->vp880SysCalData.swyOffset[0] = newValue;

            if (pDevObj->calData.iteration < 10) {
                pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                    MS_TO_TICKRATE(VP880_CAL_ABV_DECAY_STEP,
                    pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                return;
            } else {
                newValue = 0;
            }
        }

        /*
         * Data is valid or we're going with what we've got. Reset iteration
         * count for next data set.
         */
        pDevObj->calData.iteration = 0;
        pDevObj->vp880SysCalData.swyOffset[0] = newValue;

        /* This is done to be compatible with VVA P1.3.0 */
        pDevObj->calData.abvData.swyOffset[0] =
            pDevObj->vp880SysCalData.swyOffset[0];

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 0 Offset (10mV): SWY %d",
            (int16)((pDevObj->vp880SysCalData.swyOffset[0] * VP880_V_PCM_LSB / VP880_V_SCALE))));

        if (ecVal == VP880_EC_CH1) {
            abvSetAdcDone = TRUE;
        } else {
            VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CONV_CFG_WRT,
                VP880_CONV_CFG_LEN, converterCfg);

            /* Wait for converter data to settle */
            pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                    pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

            pDevObj->calData.abvData.passCnt++;
        }
    } else if (pDevObj->calData.abvData.passCnt == 1) {
        pDevObj->vp880SysCalData.swzOffset[1] =
            Vp880AdcSettling(pDevObj, VP880_EC_CH2, VP880_SWITCHER_Z, &validData)
            + VP880_TRACKER_BAT_OFFSET;

        if (validData == FALSE) {
            /*
             * Data is bad. Need to reset converter (done automatically in
             * ADC Settling function) and repeat measurement
             */
            if (pDevObj->calData.iteration < 10) {
                pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                    MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                return;
            } else {
                /* Repeated as much as we can. Set 0V offset and move on. */
                pDevObj->vp880SysCalData.swzOffset[1] = 0;
            }
        }
        /*
         * Data is valid or we're going with what we've got. Reset iteration
         * count for next data set.
         */
        pDevObj->calData.iteration = 0;

        /* This is done to be compatible with VVA P1.3.0 */
        pDevObj->calData.abvData.swzOffset[1] =
            pDevObj->vp880SysCalData.swzOffset[1];

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 1 Offset (10mV): SWZ %d",
            (int16)((pDevObj->vp880SysCalData.swzOffset[1] * VP880_V_PCM_LSB / VP880_V_SCALE))));

        converterCfg[0] = VP880_SWITCHER_Y;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CONV_CFG_WRT,
            VP880_CONV_CFG_LEN, converterCfg);

        converterCfg[0] = VP880_SWITCHER_Z;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CONV_CFG_WRT,
            VP880_CONV_CFG_LEN, converterCfg);

        /* Wait for converter data to settle */
        pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
            pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

        pDevObj->calData.abvData.passCnt++;
    } else {
        pDevObj->vp880SysCalData.swzOffset[0] =
            Vp880AdcSettling(pDevObj, VP880_EC_CH1, VP880_SWITCHER_Z, &validData)
            + VP880_TRACKER_BAT_OFFSET;

        if (validData == FALSE) {
            /*
             * Data is bad. Need to reset converter (done automatically in
             * ADC Settling function and repeat measurement
             */
            if (pDevObj->calData.iteration < 10) {
                pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                    MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                return;
            } else {
                /* Repeated as much as we can. Set 0V offset and move on. */
                pDevObj->vp880SysCalData.swzOffset[0] = 0;

                /*
                 * Data is valid or we're going with what we've got. Reset
                 * iteration count for next data set.
                 */
                pDevObj->calData.iteration = 0;
            }
        }

        /* This is done to be compatible with VVA P1.3.0 */
        pDevObj->calData.abvData.swzOffset[0] =
            pDevObj->vp880SysCalData.swzOffset[0];

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 0 Offset: SWZ %d",
            (int16)((pDevObj->vp880SysCalData.swzOffset[0] * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        pDevObj->vp880SysCalData.swyOffset[1] =
            Vp880AdcSettling(pDevObj, VP880_EC_CH2, VP880_SWITCHER_Y, &validData)
            + VP880_TRACKER_BAT_OFFSET;

        if (validData == FALSE) {
            /*
             * Data is bad. Need to reset converter (done automatically in
             * ADC Settling function and repeat measurement
             */
            if (pDevObj->calData.iteration < 10) {
                pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                    MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                                   pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                return;
            } else {
                pDevObj->vp880SysCalData.swyOffset[1] = 0;
            }
        }
        /*
         * Data is valid or we're going with what we've got. Reset iteration
         * count for next data set.
         */
        pDevObj->calData.iteration = 0;

        /* This is done to be compatible with VVA P1.3.0 */
        pDevObj->calData.abvData.swyOffset[1] =
            pDevObj->vp880SysCalData.swyOffset[1];

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 1 Offset: SWY %d",
            (int16)((pDevObj->vp880SysCalData.swyOffset[1] * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        abvSetAdcDone = TRUE;
    }

    if (abvSetAdcDone == TRUE) {
        uint8 lineState = (VP880_SS_DISCONNECT | VP880_SS_ACTIVATE_MASK);
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Calibration: Returning ALL_LINES to State 0x%02X at time %d",
            lineState, pDevObj->timeStamp));
        VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
            VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &lineState);

        /* Copy the Ringing Voltage to the Floor Voltage, everything else from the device profile */
        swYZ[0] = pDevObj->swParams[0];
        swYZ[VP880_SWY_LOCATION] = (pDevObj->swParams[VP880_SWZ_LOCATION] & VP880_VOLTAGE_MASK);
        swYZ[VP880_SWY_LOCATION] |= (pDevObj->swParams[VP880_SWY_LOCATION] & ~VP880_VOLTAGE_MASK);
        swYZ[2] = pDevObj->swParams[2];

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880AbvInit: swYZ: 0x%02X 0x%02X 0x%02X",
            swYZ[0], swYZ[1], swYZ[2]));

        /* Program switcher floor voltage to target ringing voltage */
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_REGULATOR_PARAM_WRT,
            VP880_REGULATOR_PARAM_LEN, swYZ);
        VpMemCpy(pDevObj->swParamsCache, swYZ, VP880_REGULATOR_PARAM_LEN);

        /*
         * Restore internal switcher parameters to take voltage measurement. This
         * is a global register so it doesn't matter which EC value is used.
         */
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_INT_SWREG_PARAM_WRT,
            VP880_INT_SWREG_PARAM_LEN, pDevObj->intSwParams);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Writing to Internal Switching Regulator: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            pDevObj->intSwParams[0], pDevObj->intSwParams[1], pDevObj->intSwParams[2],
            pDevObj->intSwParams[3], pDevObj->intSwParams[4], pDevObj->intSwParams[5]));

        pDevObj->calData.calSet = pDevObj->swParams[VP880_SWREG_RING_V_BYTE];

        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Vp880AbvInit: ABV Set Value Target %d",
            ((pDevObj->calData.calSet * 5) + 5)));

        /*
         * If the line is or was a Low Power Termination type the ICR values
         * could be in a condition where the switcher is not enabled. So force
         * to known/working values - Note: Actual device values are restored at
         * the end of calibration
         */
        for (channelId = 0;
             channelId < pDevObj->staticInfo.maxChannels; channelId++) {
            uint8 icr1[VP880_ICR1_LEN] = {0x0F, 0x08, 0xC0, 0x00};
            uint8 icr2[VP880_ICR2_LEN] = {0x00, 0x00, 0x2C, 0x7C};
            uint8 icr3[VP880_ICR3_LEN] = {0x30, 0x21, 0x00, 0x00};
            uint8 icr4[VP880_ICR4_LEN] = {0x01, 0x01, 0x00, 0x00};

            ecVal = (channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2;

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR1_WRT, VP880_ICR1_LEN, icr1);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT, VP880_ICR2_LEN, icr2);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR3_WRT, VP880_ICR3_LEN, icr3);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR4_WRT, VP880_ICR4_LEN, icr4);
        }

        /* Things will take time to settle after programming switcher. */
        pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = MS_TO_TICKRATE(VP880_CAL_ABV_LONG,
            pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

        pDevObj->calData.calDeviceState = VP880_CAL_MEASURE;

        /* Reset iteration and pass count for next data set. */
        pDevObj->calData.iteration = 0;
        pDevObj->calData.abvData.passCnt = 0;
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvSetAdc-"));
} /* end Vp880AbvSetAdc  */

/**
 * Vp880AbvMeasure() -- Tracker (ABV Cal) only function
 *  This function read switcher value and compare with the value read from the
 * pcm data if the value is bigger than 1.25v this function will make a
 * correction  voltage.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  Battery calibration registers are adjusted
. Channel specific registers are
 * restored.
 */
void
Vp880AbvMeasure(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal;
    int32 abvError, abvTarget;
    bool validData;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvMeasure+"));

    pDevObj->calData.iteration++;

    if ((pDevObj->stateInt & VP880_IS_SINGLE_CHANNEL)
     || (pDevObj->stateInt & VP880_LINE1_IS_FXO)) {
        ecVal = VP880_EC_CH1;
    } else {
        ecVal = VP880_EC_CH1 | VP880_EC_CH2;
    }

    if (pDevObj->calData.abvData.passCnt == 0) {
        /*
         * Exiting this state always requires resetting the timer to keep the
         * calibration moving along.
         */
        pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
            MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

        /* Now we'll switch to channel specific measurements */
        /* Read SWY from first channel */
        pDevObj->calData.abvData.swyVolt[0] =
            Vp880AdcSettling(pDevObj, VP880_EC_CH1, VP880_SWITCHER_Y, &validData);

        if (validData == FALSE) {
            /*
             * Data is bad. Need to reset converter (done automatically in
             * ADC Settling function and repeat measurement
             */
            if (pDevObj->calData.iteration < 10) {
                return;
            }
        }
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Chan 0 Voltage: SWY %d at time %d",
            (int16)((pDevObj->calData.abvData.swyVolt[0] * VP880_V_PCM_LSB) / VP880_V_SCALE),
            pDevObj->timeStamp));

        /* Now have all data necessary to compute error and adjust channel 0 */
        abvTarget = (pDevObj->calData.calSet * 5) + 5;   /* Gets it to V scale */
        abvTarget *= 1000;
        abvTarget *= 1000;
        abvTarget /= VP880_V_PCM_LSB;   /* Now we're scaled to the PCM data */

        if (pDevObj->calData.iteration >= 10) {
            pDevObj->calData.abvData.swyVolt[0] = abvTarget;
        }
        abvError = abvTarget -
            (pDevObj->calData.abvData.swyVolt[0]
           - pDevObj->vp880SysCalData.swyOffset[0]);

        pDevObj->vp880SysCalData.abvError[0] = (int16)((abvError * VP880_V_PCM_LSB) / VP880_V_SCALE);
        /*
         * Check if the error is "excessive" - meaning after a max 3.75V
         * adjustment, it is still more than 30% out of spec.
         */
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Chan 0 Pct Error %li", (ABS((abvError * 100) / abvTarget))));
        if ((ABS((abvError * 100) / abvTarget)) > 30) {
            pDevObj->responseData = VP_CAL_FAILURE;
        }

        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("1. Chan 0 Voltage Error: SWY %d Target Converted %ld",
            pDevObj->vp880SysCalData.abvError[0], ((abvTarget * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        /* Write the correction value to CH1 register. Steps in 1.25V increment */
        Vp880BatteryCalAdjust(pDevObj, VP880_EC_CH1);

        pDevObj->calData.abvData.passCnt = 1;
        pDevObj->calData.iteration = 0;
        return;
    } else if (pDevObj->calData.abvData.passCnt == 1) {
        if (ecVal == (VP880_EC_CH1 | VP880_EC_CH2)) {
            /* Read SWZ voltages from second channel */
            pDevObj->calData.abvData.swzVolt[1] =
                Vp880AdcSettling(pDevObj, VP880_EC_CH2, VP880_SWITCHER_Z, &validData);

            if (validData == FALSE) {
                /*
                 * Data is bad. Need to reset converter (done automatically in
                 * ADC Settling function) and repeat measurement.
                 */
                if (pDevObj->calData.iteration < 10) {
                    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                        MS_TO_TICKRATE(VP880_CAL_ABV_DELAY,
                            pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                    return;
                }
            }

            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 1 Voltage: SWZ %d",
                (int16)((pDevObj->calData.abvData.swzVolt[1] * VP880_V_PCM_LSB) / VP880_V_SCALE)));

            /* Now have all data necessary to compute error and adjust channel 0 */
            abvTarget = (pDevObj->calData.calSet * 5L) + 5;   /* Gets it to V scale */
            abvTarget *= 1000;
            abvTarget *= 1000;
            abvTarget /= VP880_V_PCM_LSB;   /* Now we're scaled to the PCM data */

            if (pDevObj->calData.iteration >= 10) {
                pDevObj->calData.abvData.swzVolt[1] = abvTarget;
            }

            abvError = abvTarget -
                (pDevObj->calData.abvData.swzVolt[1]
               - pDevObj->vp880SysCalData.swzOffset[1]);

            pDevObj->vp880SysCalData.abvError[1] =
                (int16)((abvError * VP880_V_PCM_LSB) / VP880_V_SCALE);

            /*
             * Check if the error is "excessive" - meaning after a max 3.75V
             * adjustment, it is still more than 30% out of spec.
             */
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Chan 1 Pct Error %li", (ABS((abvError * 100) / abvTarget))));
            if ((ABS((abvError * 100) / abvTarget)) > 30) {
                pDevObj->responseData = VP_CAL_FAILURE;
            }

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Chan 1 (SWZ): Offset %d Voltage %d Error Raw %ld (PCM) Error Converted %d (10mV)",
                pDevObj->vp880SysCalData.swzOffset[1],
                pDevObj->calData.abvData.swzVolt[1],
                abvError,
                pDevObj->vp880SysCalData.abvError[1]));

            /* Write the correction value to CH2 register. Steps in 1.25V increment */
            Vp880BatteryCalAdjust(pDevObj, VP880_EC_CH2);
        }
    }

    /* Restore Switching Regulator Parameters */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_REGULATOR_PARAM_WRT,
        VP880_REGULATOR_PARAM_LEN, pDevObj->swParams);
    VpMemCpy(pDevObj->swParamsCache, pDevObj->swParams, VP880_REGULATOR_PARAM_LEN);

    pDevObj->calData.calDeviceState = VP880_CAL_DONE;

    /* Things will take time to settle. */
    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = MS_TO_TICKRATE(VP880_CAL_ABV_LONG,
        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvMeasure-"));
    return;
} /*end Vp880AbvMeasure */

/**
 * Vp880CalInit() -- Tracker Only function, called by Vp880CalAbv()
 *  This function initiates a calibration operation for VOC associated with all
 * the lines of a device. See VP-API reference guide for more information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
void
Vp880CalInit(
     VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal = (VP880_EC_CH1 | VP880_EC_CH2);
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalInit+"));

    /*
     * Make sure device mode is where we need it. All channel specific registers
     * read (for restore purposes) in functions called by the tick.
     */
    if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
        pDevObj->devMode[0] &= ~(VP880_DEV_MODE_TEST_DATA);
        VpMpiCmdWrapper(deviceId, ecVal, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
            pDevObj->devMode);
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalInit-"));
}

#endif /* VP_CSLAC_RUNTIME_CAL_ENABLED */
#endif /* VP_CC_880_SERIES && VP880_TRACKER_SUPPORT */
