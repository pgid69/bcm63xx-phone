/** \file vp880_abs_calibration.c
 * vp880_abs_calibration.c
 *
 * This file contains the line and device calibration functions for
 * the VP880 ABS device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 10725 $
 * $LastChangedDate: 2013-01-24 18:41:02 -0600 (Thu, 24 Jan 2013) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES) && defined (VP880_ABS_SUPPORT)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED

/*****************************************************************************/
/*  START: Values associated ONLY with function Vp880AbsCalibration()        */
/*
 * The calibration state machine state, channel, and polarity being tested
 * are all stored in a single byte masked as follows:
 */
#define ABS_DC_CAL_STATE_BITS       0x3F    /* State Mask */
#define ABS_DC_CAL_CHAN_BITS        0x80    /* '0x00' = Ch 0, '0x80' = Ch 1 */
#define ABS_DC_CAL_POLREV_BITS      0x40    /* '0x00' = Normal, 0x40' = Reverse Polarity */

/*
 * State values corresponding to what the state is "doing". Masked with
 * ABS_DC_CAL_STATE_BITS
 */
#define ABS_DC_CAL_INIT_STATE       0
#define ABS_DC_CAL_CONNECT_STATE    1
#define ABS_DC_CAL_MEAS_ADJ_STATE   2
#define ABS_DC_CAL_FINAL_STATE      3   /* Used only in VC silicon when Battery Calibration is not being done */
#define ABS_DC_CAL_ACTIVE_HOLD      4   /* State to verify device is in full running condition */

/* Time in ms between key steps of the calibration process */
#define ABS_CAL_FAULT_DELAY     (100)   /* Delay if fault condition is detected */
#define ABS_CAL_INITIAL_DELAY   (30)    /* Normal delay after Init Step */
#define ABS_CAL_SAMPLE_DELAY    (5)     /* Time between stable conditions */

/*
 * Maximum number of tries on ABS Battery Switch Calibration before defaulting
 * the current setting to 0.
 */
#define ABS_CAL_FAULT_MAX_FIRST (100)   /* Max Count for first line/polarity */
#define ABS_CAL_FAULT_MAX_ALL   (3)     /* Max Count for remaining line/polarity */

/*  END: Values associated ONLY with function Vp880AbsCalibration()          */
/*****************************************************************************/

/* Helper function for Battery Switch Calibration */
static bool Vp880AdvanceAbsCal(uint8 *calState);

static void Vp880AbvInitAbs(VpDevCtxType *pDevCtx);

static void Vp880AbvSetAdcAbs(VpDevCtxType *pDevCtx);

static void Vp880AbvStateChangeAbs(VpDevCtxType *pDevCtx);

static bool Vp880AbvMeasureAbs(VpDevCtxType *pDevCtx);

static void Vp880AbsCalPrepare(VpDevCtxType *pDevCtx);

static void Vp880AbsCalSysStateConclude(VpDevCtxType *pDevCtx);

static void Vp880CalAbvAbsDevEnd(Vp880DeviceObjectType *pDevObj);
#endif

/**
 * Vp880AbvMakeAdjustment() -- ABS Only Function
 *  This function computes the measured error of ABV voltage and uses some
 * logic based on decay time to determine the device correction.
 *
 * NOTE: This function is used in run-time AND pre-run calibration for adjusting
 * the device/line parameters. DO NOT compile it out when run-time calibration
 * is disabled.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  Battery calibration registers are adjusted.
 */
void
Vp880AbvMakeAdjustment(
    Vp880DeviceObjectType *pDevObj,
    int16 *targetVoltY,
    int16 *targetVoltZ)
{
    VpDeviceIdType deviceId = pDevObj->deviceId;
    int32 abvError, abvTarget, errorScaled;
    uint8 swParams[VP880_REGULATOR_PARAM_LEN];
    uint8 data = (VP880_SWY_MP | VP880_SWZ_MP);
    uint8 channelId;
    uint8 swCal[VP880_BAT_CALIBRATION_LEN];
    uint16 swCalError;
    uint8 systemConfig = (pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK);

    /* Offset correction is 600mV if decay took a long time, -300mV if it occurred rapidly. */
    int8 offsetCorrection = ((pDevObj->calData.abvData.passCnt & 0x3F) > 3) ? 82 : -41;

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880AbvMakeAdjustment+"));

    /* Load the original (from the profile) SWY value to the temporary swParams */
    VpMemCpy(swParams, pDevObj->swParams, VP880_REGULATOR_PARAM_LEN);

    for (channelId = 0;
         channelId < pDevObj->staticInfo.maxChannels;
         channelId++) {
        uint8 ecVal, swIndex;
        int16 fineVoltTarget, coarseVoltTarget;
        int16 measVolt, measOffset;

        if (channelId == 0) {
            ecVal = VP880_EC_CH1;
            fineVoltTarget = pDevObj->yVolt;
            measVolt = pDevObj->calData.abvData.swyVolt[0];
            measOffset = pDevObj->vp880SysCalData.swyOffset[0];
            coarseVoltTarget = *targetVoltY;
            swIndex = VP880_SWY_LOCATION;
        } else {
            ecVal = VP880_EC_CH2;
            fineVoltTarget = pDevObj->zVolt;
            measVolt = pDevObj->calData.abvData.swzVolt[0];
            measOffset = pDevObj->vp880SysCalData.swzOffset[0];
            coarseVoltTarget = *targetVoltZ;
            swIndex = VP880_SWZ_LOCATION;
        }

        if (fineVoltTarget) {
            abvTarget = fineVoltTarget;
        } else {
            abvTarget = ((int32)coarseVoltTarget * 5) + 5;   /* Gets it to V scale */
        }
        abvTarget *= 1000L;
        abvTarget *= 1000L;
        abvTarget /= VP880_V_PCM_LSB;   /* Now we're scaled to the PCM data */

        /* If "reload" is in progress, we're being told what the error is. */
        if (pDevObj->stateInt & VP880_CAL_RELOAD_REQ) {
            abvError = (pDevObj->vp880SysCalData.abvError[channelId] * VP880_V_SCALE) / VP880_V_PCM_LSB;
        } else {
            abvError = abvTarget - (measVolt - measOffset - offsetCorrection);
            /* Save the computed total error used if/when parameters are changed. */
            pDevObj->vp880SysCalData.abvError[channelId] =
                (int16)((abvError * VP880_V_PCM_LSB) / VP880_V_SCALE);
        }

        /*
         * Start adjustment assuming it's in +/-5V type range. This is readjusted if
         * a +/-5V coarse adjustment is made, but does not affect the previously
         * saved "total" error value.
         */
        errorScaled = pDevObj->vp880SysCalData.abvError[channelId];

        /* errorScaled = ((abvError * VP880_V_PCM_LSB) / 10000); */
        VP_CALIBRATION(None, NULL,
            ("2. Chan %d Voltage Error: SW%s %d (10mV), Target Converted %ld (10mV) Offset Correction %d",
            channelId, ((channelId == 0) ? "Y" : "Z"), (int16)errorScaled,
            ((abvTarget * VP880_V_PCM_LSB) / VP880_V_SCALE), offsetCorrection));

        /* If the error is more than 10V, we can't adjust this supply */
        if ((ABS(errorScaled) < 1000)) {
            /*
             * If the coarse error requires a decrease/increase, make sure the current
             * setting can go down/up by 1 step and adjust the error.
             */
            /* Positive Error means the Battery voltage is not negative enough */
            /* Negative Error means the Battery voltage is too negative */

            if (((errorScaled * 10) < -4375) &&
                (swParams[swIndex] & VP880_VOLTAGE_MASK)) {
                swParams[swIndex]--;
                abvError += 683;   /* Fixed PCM Value for 5V */
                errorScaled = ((abvError * VP880_V_PCM_LSB) / VP880_V_SCALE);
                VP_CALIBRATION(None, NULL, ("Adjusted Switcher %s Down to 0x%02X. New Error %ld",
                    ((channelId == 0) ? "Y" : "Z"), swParams[swIndex], errorScaled));
            } else if (((errorScaled * 10) > 4375) &&
                       ((swParams[swIndex] & VP880_VOLTAGE_MASK) < VP880_VOLTAGE_MASK)) {
                swParams[swIndex]++;
                abvError -= 683;   /* Fixed PCM Value for 5V */
                errorScaled = ((abvError * VP880_V_PCM_LSB) / VP880_V_SCALE);
                VP_CALIBRATION(None, NULL, ("Adjusted Switcher Up to 0x%02X. New Error %ld",
                    swParams[swIndex], errorScaled));
            } else {
                VP_CALIBRATION(None, NULL,
                    ("Error within fine steps. No Adjustment to Switching Reegulator Params Required."));
            }

            if ((ABS(errorScaled) * 10) < 4375) {
                /* Write the correction value to CH1 register. Steps in 1.25V increment */
                VpMpiCmdWrapper(deviceId, (pDevObj->ecVal | ecVal),
                    VP880_BAT_CALIBRATION_RD, VP880_BAT_CALIBRATION_LEN, swCal);
                swCal[0] &= ~(VP880_BAT_CAL_SWCAL_MASK);

                /* Conversion from 7.324mV to 1.25V */
                swCalError = (ABS(abvError) / 171);
                VP_CALIBRATION(None, NULL, ("Ch %d: Initial Cal Adjustment %d (1.25V steps)",
                    channelId, swCalError));

                if (((ABS(abvError) + 85) /  171) > swCalError) {
                    swCalError+=1;
                }
                VP_CALIBRATION(None, NULL, ("Ch %d: Final Cal Adjustment %d (1.25V steps)",
                    channelId, swCalError));

                swCalError = (swCalError > 3) ? 3 : swCalError;
                swCal[0] |= (swCalError << 3);

                /*
                 * Positive error means voltage is too low (not negative enough). Positive
                 * adjustment makes the battery voltage more negative.
                 */
                swCal[0] |= (abvError > 0) ? 0 : VP880_BAT_CAL_SWCAL_SIGN;

                VP_CALIBRATION(None, NULL, ("Ch %d: Battery Calibration Correction 0x%02X 0x%02X",
                    channelId, swCal[0], swCal[1]));

                VpMpiCmdWrapper(deviceId, (pDevObj->ecVal | ecVal),
                    VP880_BAT_CALIBRATION_WRT, VP880_BAT_CALIBRATION_LEN, swCal);
                pDevObj->calData.abvData.switcherAdjust[channelId][0] = swCal[0];
                pDevObj->calData.abvData.switcherAdjust[channelId][1] = swCal[1];
            }
        } else {
            VP_CALIBRATION(None, NULL, ("Channel %d Cannot Control Switcher - Setting for Nominal Voltage",
                channelId));

            /*
             * Device is at nominal voltage IF the 1V parameter is not set. If
             * it is set, fineVoltage is always equal to or higher than the
             * coarse setting. So in 1V steps, it is always adjustable in the
             * channel correction register (never requires coarse adjustment).
             */
            if (fineVoltTarget) {
                /* channelAdjust in 10mV steps, same as 1.25V adjustments */
                int16 channelAdjust = 100 * (fineVoltTarget - ((coarseVoltTarget * 5) + 5));

                /* Remainder to determine if round-up/down */
                int16 remainder = channelAdjust % 125;

                /* Get the rounded down step... */
                uint8 stepSize = (channelAdjust - remainder)/ 125;

                VP_CALIBRATION(None, NULL, ("Channel %d Adjust %d Remainder %d Step %d - Fine Voltage %d  Coarse Setting %d" ,
                    channelId, channelAdjust, remainder, stepSize, fineVoltTarget, coarseVoltTarget));

                /* Round as needed */
                stepSize += (remainder <= 62) ? 0 : 1;

                VP_CALIBRATION(None, NULL, ("Adjusted Step %d", stepSize));

                /* Write the correction value to CH1 register. Steps in 1.25V increment */
                VpMpiCmdWrapper(deviceId, (pDevObj->ecVal | ecVal),
                    VP880_BAT_CALIBRATION_RD, VP880_BAT_CALIBRATION_LEN, swCal);
                swCal[0] &= ~(VP880_BAT_CAL_SWCAL_MASK);
                swCal[0] |= ((stepSize << 3) & VP880_BAT_CAL_SWCAL_MASK);

                VP_CALIBRATION(None, NULL, ("Adjusting Regsiter to 0x%02X", swCal[0]));

                VpMpiCmdWrapper(deviceId, (pDevObj->ecVal | ecVal),
                    VP880_BAT_CALIBRATION_WRT, VP880_BAT_CALIBRATION_LEN, swCal);
            }

            pDevObj->calData.abvData.switcherAdjust[channelId][0] = 0;
            pDevObj->calData.abvData.switcherAdjust[channelId][1] = 0;
        }
    }

    VP_CALIBRATION(None, NULL, ("1. Adjusting Switching Regulator 0x%02X 0x%02X 0x%02X",
        swParams[0], swParams[1], swParams[2]));

    VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_PARAM_WRT,
        VP880_REGULATOR_PARAM_LEN, swParams);
    VpMemCpy(pDevObj->swParamsCache, swParams, VP880_REGULATOR_PARAM_LEN);

    /* Re-adjust switchers for target power control */
    if (systemConfig != VP880_ABS_CFG_SLAVE) {
        if (systemConfig == VP880_ABS_CFG_SINGLE) {
            data = VP880_SWY_MP | VP880_SWZ_MP;
        } else {    /* systemConfig == VP880_ABS_CFG_MASTER */
            data = VP880_SWY_HP | VP880_SWZ_HP;
        }

        VpMpiCmdWrapper(deviceId, pDevObj->ecVal, VP880_REGULATOR_CTRL_WRT,
            VP880_REGULATOR_CTRL_LEN, &data);
    }
    VP_API_FUNC_INT(None, VP_NULL, ("Vp880AbvMakeAdjustment-"));
} /* end Vp880AbvMakeAdjustment */

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED

/**
 * Vp880SetCalFlags() -- ABS Only Function
 *  This function sets the calibration flags to define start of calibration and
 * calibration function to start with for the ABS device.
 *
 * Preconditions:
 *  The device must be created and initialized before calling this function.
 * This function should be called only by API-II internal functions, generally
 * CalCodec and InitDevice.
 *
 * Postconditions:
 *  Flags in the device object are set appropriately for the silicon revision.
 */
bool
Vp880SetCalFlags(
    Vp880DeviceObjectType *pDevObj)
{
    /*
     * Start with Battery Switch Calibration first. At the end of Batery Switch
     * calibration IT will start Battery Voltage calibration.
     */
    VP_API_FUNC_INT(None, VP_NULL, ("Vp880SetCalFlags+"));
    pDevObj->state |= VP_DEV_ABS_BAT_CAL;
    pDevObj->state |= VP_DEV_IN_CAL;
    pDevObj->calData.calDeviceState = VP880_CAL_INIT;
    VP_API_FUNC_INT(None, VP_NULL, ("Vp880SetCalFlags-"));

    return TRUE;
}

/**
 * Vp880AbsCalibration() -- ABS Only Function
 *  This function is called only through Vp880ApiTick() to run an ABS Battery
 * Switch Calibrary algorithm. The result is a computed DC offset for each
 * polartiy.
 *
 * Preconditions:
 *  The device must first be initialized.
 *
 * Postconditions:
 *  Upon completion, proceed to Battery Calibration. The offset values for each
 * line in both normal and reverse polarity are stored in the device object.
 */
void
Vp880AbsCalibration(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 switcherData[VP880_REGULATOR_CTRL_LEN];

    /*
     * During battery switch calibration we want to disable battery switch hysterisis. But once
     * complete, we want to restore it back to a reasonable value. The specific value chosed here
     * (5V from least 2-bits of byte[1]) is only because it's the silicon default value.
     */
    /* No Battery Switch Hysterisis Voltage - applied at start of calibration */
    uint8 firstValue[VP880_ICR6_LEN] = {0xF4, 0xE4};
    /* 5V Battery Switch Hysterisis Voltage - applied at end of calibration */
    uint8 lastValue[VP880_ICR6_LEN] = {0x00, VP880_DCCAL_BAT_SW_HYST_5V};

    uint8 icr2Values[VP880_ICR2_LEN] = {0xC0, 0x00, 0x00, 0x00};
    uint8 icr6Values[VP880_ICR6_LEN];
    uint8 sysState[VP880_SYS_STATE_LEN];
    uint16 tickRate = pDevObj->devProfileData.tickRate;

    uint8 mpiBuffer[5 + VP880_REGULATOR_PARAM_LEN + VP880_REGULATOR_CTRL_LEN
                      + VP880_SYS_STATE_LEN + VP880_ICR2_LEN + VP880_ICR6_LEN];
    uint8 mpiIndex = 0;

    uint8 channelId = ((pDevObj->calState & ABS_DC_CAL_CHAN_BITS) >> 7);

    /*
     * The ecVal is overwritten ONLY in case of initialization - when looping
     * through the channels to read registers that will be restored at the end
     * of calibration.
     */
    uint8 ecVal = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);

    /* If set, then running line in reverse polarity */
    uint8 revPolTest = (pDevObj->calState & ABS_DC_CAL_POLREV_BITS);

    bool complete = FALSE;
    bool failCondition = FALSE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbsCalibration+"));

    switch(pDevObj->calState & ABS_DC_CAL_STATE_BITS) {

        case ABS_DC_CAL_INIT_STATE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABS_DC_CAL_INIT_STATE at Time %d",
                pDevObj->timeStamp));

            /* Save off channel specific content. Pre-clear calibration values */
            Vp880AbsCalPrepare(pDevCtx);

            /*
             * This is the first step in device calibration, so it is possible
             * the device is currently programmed with post-calibrated values.
             * Reset those to the user input values and will be re-adjusted if
             * necessary (i.e., during calibration).
             */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_REGULATOR_PARAM_WRT,
                VP880_REGULATOR_PARAM_LEN, pDevObj->swParams);
            VpMemCpy(pDevObj->swParamsCache, pDevObj->swParams, VP880_REGULATOR_PARAM_LEN);
            pDevObj->stateInt &= (uint32)(~(VP880_SWZ_DECAY_CMP | VP880_SWY_DECAY_CMP));

            /* Steps 1 and first part of 2 */
            switcherData[0] = VP880_SWY_OFF;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_REGULATOR_CTRL_WRT,
                VP880_REGULATOR_CTRL_LEN, switcherData);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("ABS_DC_CAL_INIT_STATE: Setting Channels to DISCONNECT at time %d",
                pDevObj->timeStamp));

            sysState[0] = VP880_SS_DISCONNECT;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, sysState);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("ABS_DC_CAL_INIT_STATE: Writing ICR2 Channels to 0x%02X 0x%02X 0x%02X 0x%02X",
                icr2Values[0], icr2Values[1], icr2Values[2], icr2Values[3]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR2_WRT,
                VP880_ICR2_LEN, icr2Values);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("ABS_DC_CAL_INIT_STATE: Writing DC CAL Channels to 0x%02X 0x%02X",
                firstValue[0], firstValue[1]));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR6_WRT,
                VP880_ICR6_LEN, firstValue);

            /* send down the mpi commands */
            VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), mpiBuffer[0],
                mpiIndex-1, &mpiBuffer[1]);

            /* Start first state on Channel 2 in Reverse Polarity */
            pDevObj->calState =
                (ABS_DC_CAL_CONNECT_STATE | 0x80 | ABS_DC_CAL_POLREV_BITS);

            pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] =
                MS_TO_TICKRATE(ABS_CAL_INITIAL_DELAY, tickRate) | VP_ACTIVATE_TIMER;
            break;

        case ABS_DC_CAL_CONNECT_STATE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("ABS_DC_CAL_CONNECT_STATE for Chan %d Polarity %d",
                channelId, revPolTest));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_ICR6_WRT,
                VP880_ICR6_LEN, firstValue);

            /* Last part of 2 and Step 3 */
            sysState[0] = (revPolTest) ?
                VP880_SS_ACTIVE_MID_BAT_PR : VP880_SS_ACTIVE_MID_BAT;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Setting Channel %d to State 0x%02X at time %d",
                channelId, sysState[0], pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, sysState);

            /* send down the mpi commands */
            VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

            pDevObj->calState &= ~ABS_DC_CAL_STATE_BITS;
            pDevObj->calState |= ABS_DC_CAL_MEAS_ADJ_STATE;

            /* Preclear value used for fail-safe */
            pDevObj->calData.iteration = 0;

            pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] =
                MS_TO_TICKRATE(ABS_CAL_INITIAL_DELAY, tickRate) | VP_ACTIVATE_TIMER;
            break;

        case ABS_DC_CAL_ACTIVE_HOLD: {
                uint8 nextStateDelay = ABS_CAL_FAULT_DELAY;
                uint8 maxCount;

                /* First case is on line 2 (channelId = 1), Reverse Polarity.
                 * Only in this first case do we wait a bit longer because it can be
                 * the initial current startup. After the current is stable, all
                 * other lines/conditions should calibrate much faster.
                 */
                if ((channelId == 1) && (revPolTest == ABS_DC_CAL_POLREV_BITS)) {
                    maxCount = ABS_CAL_FAULT_MAX_FIRST;
                } else {
                    maxCount = ABS_CAL_FAULT_MAX_ALL;
                }

                /*
                 * This state is used to hold the line in the first fault condition
                 * until the system currents recover or until we give up. At very
                 * cold tempertures, the system can generally be kicked into working
                 * at normal speeds.
                 */
                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR6_RD, VP880_ICR6_LEN,
                    icr6Values);
                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("ABS_DC_CAL_ACTIVE_HOLD for Chan %d Polarity %d Value 0x%02X 0x%02X",
                    channelId, revPolTest, icr6Values[0], icr6Values[1]));

                pDevObj->calData.iteration++;

                if (icr6Values[VP880_DC_CAL_BLIM_INDEX] & VP880_DC_CAL_BLIM) {
                    if (pDevObj->calData.iteration > maxCount) {
                        /*
                         * Give up - this is taking too long. Force calibration
                         * values to 0, which is reasonable at least.
                         */
                        pDevObj->vp880SysCalData.absPolRevCal[0] = 0x00;
                        pDevObj->vp880SysCalData.absPolRevCal[1] = 0x00;
                        pDevObj->vp880SysCalData.absNormCal[0] = 0x00;
                        pDevObj->vp880SysCalData.absNormCal[1] = 0x00;
                        complete = TRUE;
                    }
                } else {
                    /* Good. Now we can move on. */
                    pDevObj->calState &= ~ABS_DC_CAL_STATE_BITS;
                    pDevObj->calState |= ABS_DC_CAL_MEAS_ADJ_STATE;
                    nextStateDelay = ABS_CAL_SAMPLE_DELAY;
                }

                /*
                 * Start the timer except in case of error condition exiting this
                 * state. That only occurs when we're giving up on ABS Battery
                 * Switch calibration and need to move on.
                 */
                if (complete != TRUE) {
                    pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] =
                        MS_TO_TICKRATE(nextStateDelay,
                        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
                }
            }
            break;

        case ABS_DC_CAL_MEAS_ADJ_STATE:
            /* Most cases, we're returning. */
            pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] =
                MS_TO_TICKRATE(ABS_CAL_SAMPLE_DELAY,
                pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR6_RD, VP880_ICR6_LEN,
                icr6Values);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("ABS_DC_CAL_MEAS_ADJ_STATE for Chan %d Polarity %d Value 0x%02X 0x%02X",
                channelId, revPolTest, icr6Values[0], icr6Values[1]));

            if (icr6Values[VP880_DC_CAL_BLIM_INDEX] & VP880_DC_CAL_BLIM) {
                if (pDevObj->calData.iteration == 0) {
                    /*
                     * The first value is only "converged" at very cold
                     * temperatures and is not really correct. The internal
                     * reference currents take a bit longer to come up and needs
                     * to be kicked by re-writing Active state. Get out of this
                     * process and enter the error handling states.
                     */
                    sysState[0] = (revPolTest) ?
                        VP880_SS_ACTIVE_MID_BAT_PR : VP880_SS_ACTIVE_MID_BAT;
                    VP_CALIBRATION(VpDevCtxType, pDevCtx,
                        ("ABS_DC_CAL_MEAS_ADJ_STATE: Forcing Channel %d to State 0x%02X at time %d",
                        channelId, sysState[0], pDevObj->timeStamp));
                    VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                        VP880_SYS_STATE_LEN, sysState);

                    pDevObj->calState &= ~ABS_DC_CAL_STATE_BITS;
                    pDevObj->calState |= ABS_DC_CAL_ACTIVE_HOLD;
                    return;
                }

                /* Good. Save this value */
                if (revPolTest) {
                    /* Saving the polarity reversal information */
                    pDevObj->vp880SysCalData.absPolRevCal[channelId] =
                        icr6Values[VP880_DC_CAL_ABS_INDEX] & 0xF0;

                    VP_CALIBRATION(VpDevCtxType, pDevCtx,
                        ("Saving PolRev 0x%02X for Ch %d",
                        pDevObj->vp880SysCalData.absPolRevCal[channelId], channelId));
                } else {
                    /* Saving the normal polarity information */
                    pDevObj->vp880SysCalData.absNormCal[channelId] =
                        icr6Values[VP880_DC_CAL_ABS_INDEX] & 0xF0;

                    VP_CALIBRATION(VpDevCtxType, pDevCtx,
                        ("Saving Normal 0x%02X for Ch %d",
                        pDevObj->vp880SysCalData.absNormCal[channelId], channelId));
                }

                /* Determine if there's anything else to do */
                if (Vp880AdvanceAbsCal(&pDevObj->calState) == FALSE) {
                    /* Done. Start the termination sequence. */
                    VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Cal Complete"));
                    complete = TRUE;
                }
            } else {
                /*
                 * Flag the system to indicate this is not the first time
                 * through. This value is used above when a good indication is
                 * detected.
                 */
                pDevObj->calData.iteration = 1;

                /* Change the current offset and try again */
                if (icr6Values[VP880_DC_CAL_ABS_INDEX] & 0x80) {
                    if ((icr6Values[VP880_DC_CAL_ABS_INDEX] & 0xF0) == 0x80) {
                        icr6Values[VP880_DC_CAL_ABS_INDEX] = 0;
                    } else {
                        icr6Values[VP880_DC_CAL_ABS_INDEX] -= 16;
                    }
                } else {
                    if ((icr6Values[VP880_DC_CAL_ABS_INDEX] & 0xF0) == 0x70) {
                        /*
                         * Something wrong happened. Restore back to 0 and end
                         * algorithm.
                         */
                        complete = TRUE;
                        VP_ERROR(VpDevCtxType, pDevCtx,
                            ("Calibration Algorithm Error 0x%02X on Channel %d, Polarity %d",
                            pDevObj->state, channelId, revPolTest));
                        failCondition = TRUE;
                    } else {
                        icr6Values[VP880_DC_CAL_ABS_INDEX] += 16;
                    }
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR6_WRT, VP880_ICR6_LEN,
                    icr6Values);

                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Adjusting Offset 0x%02X",
                    icr6Values[VP880_DC_CAL_ABS_INDEX]));
            }
            break;

        case ABS_DC_CAL_FINAL_STATE:
            /*
             * We're here because it's VC silicon and we're not moving on to
             * Battery Calibration. So the switchers have to be properly turned
             * on here. They should already be in LP and sufficient time passed.
             * Just complete remaining exit sequence.
             */
            complete = TRUE;
            break;

        default: /* oops. shouldn't be here. Restore and exit. */
            VP_ERROR(VpDevCtxType, pDevCtx, ("Calibration Case Error %d",
                (pDevObj->calState & ABS_DC_CAL_STATE_BITS)));

            complete = TRUE;
            failCondition = TRUE;
            break;
    }

    if (complete == TRUE) {
#ifdef VP880_CURRENT_LIMIT
        icr2Values[VP880_ICR2_SWY_CTRL_INDEX] |= VP880_ICR2_SWY_LIM_CTRL;
        icr2Values[VP880_ICR2_SWY_CTRL_INDEX+1] &= ~VP880_ICR2_SWY_LIM_CTRL;
#endif
        icr2Values[0] = 0x00;

        sysState[0] = VP880_SS_DISCONNECT;
        VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_SYS_STATE_WRT,
            VP880_SYS_STATE_LEN, sysState);

        /* Cache the Calibration values for existing line objects */
        for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
            VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
            if (pLineCtx != VP_NULL) {
                Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
                ecVal = pLineObj->ecVal;

                VpMemCpy(pLineObj->icr6Values, lastValue, VP880_ICR6_LEN);
                Vp880GetLineStateABS(pLineCtx, pLineObj->lineState.currentState, TRUE);
                VpMemCpy(icr2Values, pLineObj->icr2Values, VP880_ICR2_LEN);
            } else {
                ecVal = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);
                VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR6_WRT, VP880_ICR6_LEN, lastValue);
            }
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp880AbsCalibration() done - Writing ICR2: 0x%02X 0x%02X 0x%02X 0x%02X",
                icr2Values[0], icr2Values[1], icr2Values[2], icr2Values[3]));
            VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR2_WRT, VP880_ICR2_LEN, icr2Values);
        }

        if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] >= VP880_REV_JE) {
            pDevObj->state &= ~VP_DEV_ABS_BAT_CAL;
            pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] = 0;

            /* Move on to ABS Absolute Battery Voltage Calibration */
            pDevObj->state |= VP_DEV_ABV_CAL_ABS;
            pDevObj->calData.calDeviceState = VP880_CAL_INIT;
            pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = (1 | VP_ACTIVATE_TIMER);
        } else {
            uint8 regControl[VP880_REGULATOR_CTRL_LEN] = {VP880_SWY_MP | VP880_SWZ_MP};
            uint8 swControl[VP880_REGULATOR_CTRL_LEN] = {VP880_SWY_LP | VP880_SWZ_LP};
            uint8 systemConfig = (pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK);

            /*
             * Done if using VC silicon. Need to re-enable the switchers because
             * they were disabled by this calibration.
             */

            /*
             * Low power mode then "come back" in 50ms before setting to Medium
             * power, unless an error occurred. In that case, do everything now.
             */
            if (failCondition == TRUE) {
                /* Fail and bail... */
                VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
                    VP880_REGULATOR_CTRL_WRT, VP880_REGULATOR_CTRL_LEN, swControl);
                if (systemConfig != VP880_ABS_CFG_SLAVE) {
                    VpSysWait(240); /* 125us * 240 = 30ms */
                    VpSysWait(160); /* 125us * 160 = 20ms */

                    if (systemConfig == VP880_ABS_CFG_MASTER) {
                        regControl[0] = VP880_SWY_HP | VP880_SWZ_HP;
                    }
                    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
                        VP880_REGULATOR_CTRL_WRT, VP880_REGULATOR_CTRL_LEN, regControl);
                }
            } else if (pDevObj->calState != ABS_DC_CAL_FINAL_STATE) {
                /* OK. Step 1, start delay and make sure we come back here. */
                VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
                    VP880_REGULATOR_CTRL_WRT, VP880_REGULATOR_CTRL_LEN, swControl);

                pDevObj->calState = ABS_DC_CAL_FINAL_STATE;
                pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] =
                    MS_TO_TICKRATE(50, tickRate) | VP_ACTIVATE_TIMER;
                return;
            } else {
                /*
                 * This is the normal exit. If configured as a Master device,
                 * can only use High Power Mode because feeding other lines that
                 * will need Ringing power. All other modes can use Medium Power.
                 * A "Single" device will later enter High Power for Ringing.
                 * A "Slave" device will no nothing but needs the power modes on
                 * (even though they don't do anything) to enable key internal
                 * silicon circuitry.
                 */
                if (systemConfig == VP880_ABS_CFG_MASTER) {
                    regControl[0] = VP880_SWY_HP | VP880_SWZ_HP;
                }
                VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
                    VP880_REGULATOR_CTRL_WRT, VP880_REGULATOR_CTRL_LEN, regControl);
            }

            Vp880AbsCalSysStateConclude(pDevCtx);

            pDevObj->state &= ~VP_DEV_ABS_BAT_CAL;
            pDevObj->devTimer[VP_DEV_TIMER_ABSCAL] = 0;

            if (pDevObj->state & VP_DEV_INIT_IN_PROGRESS) {
                pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
            } else {
                pDevObj->deviceEvents.response |= VP_EVID_CAL_CMP;
            }

            pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_IN_CAL);
            pDevObj->calState = VP880_CAL_INIT;
            pDevObj->state &= ~VP_DEV_ABS_BAT_CAL;
        }
    }
    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbsCalibration-"));
}

/*
 * Vp880AdvanceAbsCal()
 *    Helper function for ABS Battery Switch Calibration. Adjusts the device
 * object (should be) calibration byte and modifies it for the next state. If
 * complete, returns FALSE.
 *
 * States are as follows (with post shifting by 6):
 *
 * From channel 2 polarity reverse => next state is channel 2 normal polarity
 *    ABS_DC_CAL_CHAN_BITS | ABS_DC_CAL_POLREV_BITS => ABS_DC_CAL_CHAN_BITS
 *    (0x03 => 0x02)
 *
 * From channel 2 normal polarity => next state is channel 1 reverse polarity
 *    ABS_DC_CAL_CHAN_BITS => ABS_DC_CAL_POLREV_BITS
 *    (0x02 => 0x01)
 *
 * From channel 1 reverse polarity => next state is channel 1 normal polarity
 *    ABS_DC_CAL_CHAN_BITS | ABS_DC_CAL_POLREV_BITS => ABS_DC_CAL_CHAN_BITS
 *    (0x01 => 0x00)
 *
 * From channel 1 normal polarity => next state is done
 *    ABS_DC_CAL_CHAN_BITS | ABS_DC_CAL_POLREV_BITS => ABS_DC_CAL_CHAN_BITS
 *    (0x00 => done)
 */
bool
Vp880AdvanceAbsCal(
    uint8 *calState)
{
    /*
     * This is a lookup table from post-shifted bits of the channel and polarity
     * conditions listed above.
     */
    uint8 nextState[] = {
        0x00,   /* From 0x00, we're done */
        0x00,   /* From 0x01, go to channel 0 in normal polarity */
        ABS_DC_CAL_POLREV_BITS, /* From 0x02, go to channel 0 in reverse polarity */
        ABS_DC_CAL_CHAN_BITS    /* From 0x03, go to channel 1 in normal polarity */
    };

    uint8 nextStateIndex =
        (*calState & (ABS_DC_CAL_CHAN_BITS | ABS_DC_CAL_POLREV_BITS));
    nextStateIndex = ((nextStateIndex >> 6) & 0x03);

    if ((*calState & (ABS_DC_CAL_CHAN_BITS | ABS_DC_CAL_POLREV_BITS)) == 0x00) {
        *calState = ABS_DC_CAL_FINAL_STATE;
        return FALSE;
    }

    *calState = nextState[nextStateIndex];

    /* Repeat with new connections */
    *calState |= ABS_DC_CAL_CONNECT_STATE;

    VP_CALIBRATION(None, VP_NULL,
        ("Changing to %s Polarity for Channel %d - calState 0x%02X",
        ((*calState & ABS_DC_CAL_POLREV_BITS) ? "Reverse" : "Normal"),
        ((*calState & ABS_DC_CAL_CHAN_BITS) >> 7),
        *calState));

    return TRUE;
}

/**
 * Vp880AbsCalibrationPrepare() -- ABS Only Function
 *
 * Preconditions:
 *
 * Postconditions:
 */
void
Vp880AbsCalPrepare(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId, ecVal;
    uint8 swCal[VP880_BAT_CALIBRATION_LEN] = {0x00, 0x10};

    /*
     * Channel specific registers to restore at end of calibration. Note
     * that this set includes registers that are modified during Battery
     * calibration (i.e., not Battery Switch Calibration).
     */
    for (channelId = 0;
         channelId < pDevObj->staticInfo.maxChannels;
         channelId++) {
        ecVal = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);

        /* Clear existing correction factors */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_BAT_CALIBRATION_WRT,
            VP880_BAT_CALIBRATION_LEN, swCal);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_RD, VP880_SYS_STATE_LEN,
            &pDevObj->calData.abvData.sysState[channelId][0]);

        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Vp880AbvInitAbs: Saving SLIC State 0x%02X from channel %d",
            pDevObj->calData.abvData.sysState[channelId][0], channelId));

        VpMpiCmdWrapper(deviceId, ecVal, VP880_DISN_RD, VP880_DISN_LEN,
            &pDevObj->calData.abvData.disnVal[channelId][0]);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_VP_GAIN_RD, VP880_VP_GAIN_LEN,
            &pDevObj->calData.abvData.vpGain[channelId][0]);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN,
            &pDevObj->calData.abvData.opFunc[channelId][0]);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_RD, VP880_OP_COND_LEN,
            &pDevObj->calData.abvData.opCond[channelId][0]);

        VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_RD, VP880_CONV_CFG_LEN,
            &pDevObj->calData.abvData.converterCfg[channelId][0]);
    }
}

/**
 * Vp880AbsCalSysStateConclude() -- ABS Only Function
 *
 * Preconditions:
 *
 * Postconditions:
 */
void
Vp880AbsCalSysStateConclude(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId, ecVal;
    uint8 mpiBuffer[6 + VP880_SYS_STATE_LEN + VP880_DISN_LEN + VP880_OP_FUNC_LEN +
                         VP880_OP_COND_LEN + VP880_CONV_CFG_LEN + VP880_VP_GAIN_LEN];
    uint8 mpiIndex = 0;

    /*
     * Channel specific registers to restore at end of calibration. Note
     * that this set includes registers that are modified during Battery
     * calibration (i.e., not Battery Switch Calibration).
     */
    for (channelId = 0;
         channelId < pDevObj->staticInfo.maxChannels;
         channelId++) {
        ecVal = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);
        mpiIndex = 0;

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN,
            &pDevObj->calData.abvData.sysState[channelId][0]);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP880_DISN_WRT, VP880_DISN_LEN,
            &pDevObj->calData.abvData.disnVal[channelId][0]);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP880_VP_GAIN_WRT, VP880_VP_GAIN_LEN,
            &pDevObj->calData.abvData.vpGain[channelId][0]);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP880_OP_FUNC_WRT, VP880_OP_FUNC_LEN,
            &pDevObj->calData.abvData.opFunc[channelId][0]);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP880_OP_COND_WRT, VP880_OP_COND_LEN,
            &pDevObj->calData.abvData.opCond[channelId][0]);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
            VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            &pDevObj->calData.abvData.converterCfg[channelId][0]);

        /* send down the mpi commands */
        VpMpiCmdWrapper(deviceId, ecVal,  mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);
    }
}

/**
 * Vp880AbvInitAbs() -- ABS Only Function
 *  This function initiates a calibration operation for ABV associated with all
 * the lines of an ABS device. See VP-API reference guide for more information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
void
Vp880AbvInitAbs(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 disnVal[VP880_DISN_LEN] = {0x00};
    uint8 vpGain[VP880_VP_GAIN_LEN] = {0x00};
    uint8 data;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvInitAbs+"));

    /*
     * Initialize and use to measure each channels offset and voltage using
     * same functions.
     */
    pDevObj->calData.abvData.passCnt = 0;

    /* Device Mode */
    if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
        pDevObj->devMode[0] &= ~(VP880_DEV_MODE_TEST_DATA);
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
            pDevObj->devMode);
    }

    /*
     * Channel specific registers to restore at end of calibration are saved
     * during the Battery Switch Calibration process. Also, the battery switch
     * calibration process does not restore all registers because it "knows"
     * that Battery Calibration occurs next. So don't try to read/save off the
     * channel specific registers here.
     */

    /* Set for Linear Mode and disable AC Coefficients */
    data = VP880_LINEAR_CODEC;
    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_OP_FUNC_WRT,
        VP880_OP_FUNC_LEN, &data);
    VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880AbvInitAbs: Setting OP Functions to 0x%02X",
        data));

    /* Cut TX/RX PCM and disable HPF */
    data = (VP880_CUT_TXPATH | VP880_CUT_RXPATH | VP880_HIGH_PASS_DIS);
    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_OP_COND_WRT,
        VP880_OP_COND_LEN,  &data);

    /* Set DISN = 0 and Voice Path Gain to 0dB. */
    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_DISN_WRT,
        VP880_DISN_LEN, disnVal);

    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_VP_GAIN_WRT,
        VP880_VP_GAIN_LEN, vpGain);

    /*
     * Disable Switchers and wait for discharge. Typically, 2.5 seconds for a
     * warm re-cal
     */
    data = (VP880_SWY_OFF | VP880_SWZ_OFF);
    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_REGULATOR_CTRL_WRT,
        VP880_REGULATOR_CTRL_LEN, &data);

    /* Force sink supply current to reduce voltage. */
    data = VP880_SS_ACTIVE;
    VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Calibration: Setting BOTH LINES to State 0x%02X at time %d",
        data, pDevObj->timeStamp));
    VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2), VP880_SYS_STATE_WRT,
        VP880_SYS_STATE_LEN, &data);

    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
    MS_TO_TICKRATE(VP880_CAL_ABV_ABS_INIT,
        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    /* Advance state to measure ADC offset */
    pDevObj->calData.calDeviceState = VP880_CAL_STATE_CHANGE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvInitAbs-"));
} /* end Vp880AbvInitAbs */

/**
 * Vp880AbvStateChangeAbs() -- ABS Only Function
 *  This function TBD
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 */
void
Vp880AbvStateChangeAbs(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 converterCfg[VP880_CONV_CFG_LEN];

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvStateChangeAbs+"));

    /* First iteration VP880_SWITCHER_Y -> Ch1 AND VP880_SWITCHER_Z -> Ch2 */
    /* Second iteration VP880_SWITCHER_Y -> Ch2 AND VP880_SWITCHER_Z -> Ch1 */
    /* Third iteration VP880_XBR -> Ch1 AND VP880_XBR -> Ch2 */
    if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x00) {
        /* Don't care about the data, just force the converter configuration */
        converterCfg[0] = VP880_SWITCHER_Y;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);

        converterCfg[0] = VP880_SWITCHER_Z;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);
    } else if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x40) {
        /* Don't care about the data, just force the converter configuration */
        converterCfg[0] = VP880_SWITCHER_Y;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);

        converterCfg[0] = VP880_SWITCHER_Z;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);
    } else {    /* ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x80) */
        /* Don't care about the data, just force the converter configuration */
        converterCfg[0] = VP880_XBR;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);

        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);
    }

    /* Allow the converter to stabilize */
    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = MS_TO_TICKRATE(VP880_CAL_ABV_LONG,
        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    pDevObj->calData.calDeviceState =  VP880_CAL_ADC;
    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvStateChangeAbs-"));
} /* end Vp880AbvStateChangeAbs */

/**
 * Vp880AbvSetAdcAbs() -- ABS Only Function
 *  This function set the converter to read the right pcm set the right state
 * machine
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 */
void
Vp880AbvSetAdcAbs(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    int16 sw1OffsetNew, sw2OffsetNew;
    uint8 sw1, sw2;
    int16 *sw1Offset, *sw2Offset;
    bool validData;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvSetAdcAbs+"));

    if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x00) {
        sw1Offset = (int16*)&pDevObj->vp880SysCalData.swyOffset[0];
        sw2Offset = (int16*)&pDevObj->vp880SysCalData.swzOffset[1];
        sw1 = VP880_SWITCHER_Y;
        sw2 = VP880_SWITCHER_Z;
    } else if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x40) {
        sw1Offset = (int16*)&pDevObj->vp880SysCalData.swzOffset[0];
        sw2Offset = (int16*)&pDevObj->vp880SysCalData.swyOffset[1];
        sw1 = VP880_SWITCHER_Z;
        sw2 = VP880_SWITCHER_Y;
    } else { /* ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x80) */
        sw1Offset = (int16*)&pDevObj->vp880SysCalData.swxbOffset[0];
        sw2Offset = (int16*)&pDevObj->vp880SysCalData.swxbOffset[1];
        sw1 = VP880_XBR;
        sw2 = VP880_XBR;
    }

    /* Read SWY from first channel, SWZ from second channel */
    if ((pDevObj->calData.abvData.passCnt & 0x3F) == 0) {
        /*
         * Take first measurement, then increment the pass counter to track
         * how long it takes to settle. The time it takes will determine an
         * offset correction to the final value.
         */
        *sw1Offset = Vp880AdcSettling(pDevObj, VP880_EC_CH1, sw1, &validData);

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("1. Chan 0 Offset, Sequence %d: %d",
            pDevObj->calData.abvData.passCnt >> 6,
            (int16)((*sw1Offset * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        *sw2Offset = Vp880AdcSettling(pDevObj, VP880_EC_CH2, sw2, &validData);

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("1. Chan 1 Offset, Sequence %d: %d",
            pDevObj->calData.abvData.passCnt >> 6,
            (int16)((*sw2Offset * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        pDevObj->calData.abvData.passCnt++;
    } else {
        sw1OffsetNew = Vp880AdcSettling(pDevObj, VP880_EC_CH1, sw1, &validData);

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("%d. Chan 0 Offset, Sequence %d: %d",
            (pDevObj->calData.abvData.passCnt & 0x3F) + 1,
            pDevObj->calData.abvData.passCnt >> 6,
            (int16)((sw1OffsetNew * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        sw2OffsetNew = Vp880AdcSettling(pDevObj, VP880_EC_CH2, sw2, &validData);

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("%d. Chan 1 Offset, Sequence %d: %d",
            (pDevObj->calData.abvData.passCnt & 0x3F) + 1,
            pDevObj->calData.abvData.passCnt >> 6,
            (int16)((sw2OffsetNew * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ch0 OffsetNew %d Old %d  :: Ch1 OffsetNew %d Old %d",
            sw1OffsetNew, *sw1Offset, sw2OffsetNew, *sw2Offset));

        /* Repeat until delta between two samples is less than max error */
        if (ABS(sw1OffsetNew - *sw1Offset) <= VP880_CAL_ABV_SAMPLE_ERR) {
            pDevObj->stateInt |= VP880_SWY_DECAY_CMP;
        }

        if (ABS(sw2OffsetNew - *sw2Offset) <= VP880_CAL_ABV_SAMPLE_ERR) {
            pDevObj->stateInt |= VP880_SWZ_DECAY_CMP;
        }

        if ((pDevObj->stateInt & (VP880_SWZ_DECAY_CMP | VP880_SWY_DECAY_CMP)) ==
            (VP880_SWZ_DECAY_CMP | VP880_SWY_DECAY_CMP)) {
            if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x00) {
                pDevObj->calData.abvData.passCnt &= 0x3F;
                pDevObj->calData.abvData.passCnt |= 0x40;

                pDevObj->calData.calDeviceState = VP880_CAL_STATE_CHANGE;
            } else if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x40) {
                pDevObj->calData.abvData.passCnt &= 0x3F;
                pDevObj->calData.abvData.passCnt |= 0x80;

                pDevObj->calData.calDeviceState = VP880_CAL_STATE_CHANGE;
            } else {
                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("-- Offset VP880_SWITCHER_Y Ch0 = %d",
                    pDevObj->vp880SysCalData.swyOffset[0]));
                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("-- Offset VP880_SWITCHER_Y Ch1 = %d",
                    pDevObj->vp880SysCalData.swyOffset[1]));
                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("-- Offset VP880_SWITCHER_Z Ch0 = %d",
                    pDevObj->vp880SysCalData.swzOffset[0]));
                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("-- Offset VP880_SWITCHER_Z Ch1 = %d",
                    pDevObj->vp880SysCalData.swzOffset[1]));
                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("-- Offset VP880_XBR Ch0        = %d",
                    pDevObj->vp880SysCalData.swxbOffset[0]));
                VP_CALIBRATION(VpDevCtxType, pDevCtx, ("-- Offset VP880_XBR Ch1        = %d",
                    pDevObj->vp880SysCalData.swxbOffset[1]));
                pDevObj->calData.calDeviceState = VP880_CAL_MEASURE;
            }
        } else {
            if ((pDevObj->calData.abvData.passCnt & 0x3F) <= 0x3F) {
                pDevObj->calData.abvData.passCnt++;
            }

            /*
             * Error is exceeded between consecutive values. Terminate the loop
             * if we reached max number of iterations.
             */
            if ((pDevObj->calData.abvData.passCnt & 0x3F) >= VP880_CAL_ABV_SAMPLE_MAX) {
                if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x00) {
                    pDevObj->calData.abvData.passCnt &= 0x3F;
                    pDevObj->calData.abvData.passCnt |= 0x40;

                    pDevObj->calData.calDeviceState = VP880_CAL_STATE_CHANGE;
                } else if ((pDevObj->calData.abvData.passCnt & 0xC0) == 0x40) {
                    pDevObj->calData.abvData.passCnt &= 0x3F;
                    pDevObj->calData.abvData.passCnt |= 0x80;

                    pDevObj->calData.calDeviceState = VP880_CAL_STATE_CHANGE;
                } else {
                    pDevObj->calData.calDeviceState = VP880_CAL_MEASURE;
                }
            }
        }
        *sw1Offset = sw1OffsetNew;
        *sw2Offset = sw2OffsetNew;
    }

    if (pDevObj->calData.calDeviceState == VP880_CAL_MEASURE) {
        uint8 data = (VP880_SWY_LP | VP880_SWZ_LP);

        /* Check if we were able to collapse the battery (>7.3V) */
        /* if not set the calibration factors to 0 */
        if (ABS(pDevObj->vp880SysCalData.swyOffset[0]) > 1370) {
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880AbvSetAdcAbs(): Impossible to collapse the battery, %d",
                ABS(pDevObj->vp880SysCalData.swyOffset[0])));
            pDevObj->vp880SysCalData.swyOffset[0] = 0;
            pDevObj->vp880SysCalData.swyOffset[1] = 0;
            pDevObj->vp880SysCalData.swzOffset[0] = 0;
            pDevObj->vp880SysCalData.swzOffset[1] = 0;
            pDevObj->vp880SysCalData.swxbOffset[0] = 0;
            pDevObj->vp880SysCalData.swxbOffset[1] = 0;
        }

        pDevObj->calData.abvData.initChange = TRUE;

        /* Re-enable the switchers for target measurement */
        VpMpiCmdWrapper(deviceId, (VP880_EC_CH1 | VP880_EC_CH2),
            VP880_REGULATOR_CTRL_WRT, VP880_REGULATOR_CTRL_LEN, &data);
    }

    /* Things will take time to settle after programming switcher. */
    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = MS_TO_TICKRATE(VP880_CAL_ABV_SAMPLE,
        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvSetAdcAbs-"));

} /* end Vp880AbvSetAdcAbs */

/**
 * Vp880AbvMeasureAbs() -- ABS Only Function
 *  This is the last functional step for ABV calibration on ABS devices. It
 * takes the SWY and SWZ measurements, computes the error, makes the adjustment,
 * and restores line registers to values prior to running calibration.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  Battery calibration registers are adjusted. Channel specific registers are
 * restored.
 */
bool
Vp880AbvMeasureAbs(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    bool validData;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvMeasureAbs+"));

    if (pDevObj->calData.abvData.initChange == TRUE) {
        /* Make sure converters are configured correctly */
        uint8 converterCfg[VP880_CONV_CFG_LEN];

        /* Don't care about the data, just force the converter configuration */
        converterCfg[0] = VP880_SWITCHER_Y;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);

        converterCfg[0] = VP880_SWITCHER_Z;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN,
            converterCfg);

        pDevObj->calData.abvData.initChange = FALSE;
    } else {
        int16 targetVoltY, targetVoltZ;

        pDevObj->calData.abvData.swyVolt[0] =
            Vp880AdcSettling(pDevObj, VP880_EC_CH1, VP880_SWITCHER_Y, &validData);

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 0 Voltage: SWY %d (10mV)",
            (int16)((pDevObj->calData.abvData.swyVolt[0] * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        pDevObj->calData.abvData.swzVolt[0] =
            Vp880AdcSettling(pDevObj, VP880_EC_CH2, VP880_SWITCHER_Z, &validData);

        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Chan 1 Voltage: SWZ %d (10mV)",
            (int16)((pDevObj->calData.abvData.swzVolt[0] * VP880_V_PCM_LSB) / VP880_V_SCALE)));

        /* Compute Errors and make corrections */
        targetVoltY = (pDevObj->swParams[VP880_SWY_LOCATION] & VP880_VOLTAGE_MASK);
        targetVoltZ = (pDevObj->swParams[VP880_SWZ_LOCATION] & VP880_VOLTAGE_MASK);

        Vp880AbvMakeAdjustment(pDevObj, &targetVoltY, &targetVoltZ);

        Vp880AbsCalSysStateConclude(pDevCtx);

        /* Device Mode */
        pDevObj->devMode[0] |= VP880_DEV_MODE_TEST_DATA;
        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
            pDevObj->devMode);

        pDevObj->calData.calDeviceState = VP880_CAL_DONE;
    }

    /* Things will take time to settle. */
    pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] = MS_TO_TICKRATE(VP880_CAL_ABV_SAMPLE,
        pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880AbvMeasureAbs-"));

    return TRUE;
} /* end Vp880AbvMeasureAbs */

/**
 * Vp880CalAbvAbsDev() -- ABS Only Function
 *  This function initiates a calibration operation for Absolute Switcher
 * circuits associated with all the lines of a device. See VP-API reference
*  guide for more information. SWYV SWZV are global for every Channels
 * Line must be in Disconnect state before to start the Calibration
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp880CalAbvAbsDev(
     VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType  status = VP_STATUS_SUCCESS;
    uint8 swyPower = (VP880_SWY_MP | VP880_SWZ_MP);
    uint8 systemConfig = (pDevObj->devProfileData.systemConfig & VP880_ABS_CFG_MASK);

    uint8 ecVal[] = {VP880_EC_CH1, VP880_EC_CH2};
    bool calCleanup = FALSE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalAbvAbsDev+"));

    if (pDevObj->calData.calDeviceState == VP880_CAL_INIT
        || pDevObj->calData.calDeviceState == VP880_CAL_EXIT) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbvAbsDev:  - Setting to Vp880CalInit"));

        pDevObj->calData.calDeviceState = VP880_CAL_INIT;
    }

    switch(pDevObj->calData.calDeviceState) {
        case VP880_CAL_INIT:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbv: - Running Vp880AbvInitAbs"));
            Vp880AbvInitAbs(pDevCtx);
            break;

        case VP880_CAL_ADC:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbv: - Running Vp880AbvSetAdcAbs"));
            Vp880AbvSetAdcAbs(pDevCtx);
            break;

        case VP880_CAL_STATE_CHANGE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbv: - Running Vp880AbvStateChangeAbs"));
            Vp880AbvStateChangeAbs(pDevCtx);
            break;

        case VP880_CAL_MEASURE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbv - Running Vp880AbvMeasureAbs"));
            Vp880AbvMeasureAbs(pDevCtx);
            break;

        case VP880_CAL_DONE:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Cal Done for ABS"));

            /* Re-adjust switchers for target power control */
            if (systemConfig != VP880_ABS_CFG_SLAVE) {
                if (systemConfig == VP880_ABS_CFG_SINGLE) {
                    swyPower = VP880_SWY_MP | VP880_SWZ_MP;
                } else {    /* systemConfig == VP880_ABS_CFG_MASTER */
                    swyPower = VP880_SWY_HP | VP880_SWZ_HP;
                }

                VpMpiCmdWrapper(deviceId, (ecVal[0] | ecVal[1]), VP880_REGULATOR_CTRL_WRT,
                    VP880_REGULATOR_CTRL_LEN, &swyPower);
            }

            calCleanup = TRUE;
            pDevObj->calData.calDeviceState = VP880_CAL_CLEANUP;
            break;

        case VP880_CAL_CLEANUP:
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Cal Cleanup for ABS"));
            Vp880CalAbvAbsDevEnd(pDevObj);
            return VP_STATUS_SUCCESS;

        case VP880_CAL_ERROR:
            /* Fall through intentional */
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880CalAbvAbsDev: ERROR - Cal Done"));
            calCleanup = TRUE;
            status = VP_STATUS_FAILURE;
            pDevObj->responseData = VP_CAL_FAILURE;
            break;
    }

    if (calCleanup == TRUE) {
        if (pDevObj->responseData == VP_CAL_FAILURE) {
            pDevObj->stateInt &= ~VP880_DEVICE_CAL_COMPLETE;
        } else {
            pDevObj->stateInt |= VP880_DEVICE_CAL_COMPLETE;
        }

        /* Reset Line states */
        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbvAbsDev: Setting Ch 0 to State 0x%02X at time %d",
            pDevObj->calData.abvData.sysState[0][0], pDevObj->timeStamp));
        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Vp880CalAbvAbsDev: Setting Ch 1 to State 0x%02X at time %d",
            pDevObj->calData.abvData.sysState[1][0], pDevObj->timeStamp));

        VpMpiCmdWrapper(deviceId, VP880_EC_CH1, VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN,
            &pDevObj->calData.abvData.sysState[0][0]);
        VpMpiCmdWrapper(deviceId, VP880_EC_CH2, VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN,
            &pDevObj->calData.abvData.sysState[1][0]);

        if ((pDevObj->calData.abvData.sysState[0][0] == VP880_SS_DISCONNECT) &&
            (pDevObj->calData.abvData.sysState[1][0] == VP880_SS_DISCONNECT)) {
            /*
             * No debounce time is needed because the line is not entering a
             * feed state that will generate gkey or hook events.
             */
            Vp880CalAbvAbsDevEnd(pDevObj);
        } else {
            /*
             * Need time for the supplies to come up if the lines are also in
             * feed state to prevent generating false hook/gkey events. Worst
             * case is Disconnect Exit timer. Note that this algorithm only
             * runs as part of VpInitDevice() so it ends with the lines in
             * VP_LINE_DISCONNECT state (so no events generated until line is
             * set to a feed state). This added delay is precaution in case
             * using VpCalCodec() directly with lines in a previous feed state.
             */
            pDevObj->devTimer[VP_DEV_TIMER_ABV_CAL] =
                MS_TO_TICKRATE(VP_DISCONNECT_RECOVERY_TIME,
                pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;
        }

        /* Restore device mode to test buffer if exists */
        if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
            pDevObj->devMode[0] |= VP880_DEV_MODE_TEST_DATA;
            VpMpiCmdWrapper(deviceId, (ecVal[0] | ecVal[1]), VP880_DEV_MODE_WRT,
                VP880_DEV_MODE_LEN, pDevObj->devMode);
        }
    }
    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalAbvAbsDev-"));
    return status;
}

/**
 * Vp880CalAbvAbsDevEnd() -- ABS Only Function
 *  This function completes the ABS Battery Voltage calibration. Basically,
 * cleans up the device object values and generates the required event.
 *
 * Preconditions:
 *  Battery (and Battery Switch) calibration are comlete.
 *
 * Postconditions:
 *  The device object values have been updated to normal operating mode.
 */
void
Vp880CalAbvAbsDevEnd(
    Vp880DeviceObjectType *pDevObj)
{
    /* Complete calibration. Set flags back to initial "type" of values. */
    pDevObj->state |= VP_DEV_ABS_BAT_CAL;
    pDevObj->calState = ABS_DC_CAL_INIT_STATE;
    pDevObj->calData.calDeviceState = VP880_CAL_EXIT;
    pDevObj->state &= ~VP_DEV_ABV_CAL_ABS;

    if (pDevObj->state & VP_DEV_INIT_IN_PROGRESS) {
        pDevObj->deviceEvents.response |= VP_DEV_EVID_DEV_INIT_CMP;
    } else {
        pDevObj->deviceEvents.response |= VP_EVID_CAL_CMP;
    }
    pDevObj->state &= ~(VP_DEV_INIT_IN_PROGRESS | VP_DEV_IN_CAL);
}
#endif /* VP_CSLAC_RUNTIME_CAL_ENABLED */
#endif /* VP_CC_880_SERIES && VP880_ABS_SUPPORT */

