/** \file vp886_calibration_common.c
 * vp886_calibration_common.c
 *
 * This file contains the line and device calibration functions that are common
 * within the 886 device family (for ABS and Tracker).
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11630 $
 * $LastChangedDate: 2014-11-11 10:26:38 -0600 (Tue, 11 Nov 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

#ifndef ABS
    #define ABS(a) ( ((a) < 0) ? -(a) : (a))
#endif

#ifndef MIN
    #define MIN(a,b) ( ((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
    #define MAX(a,b) ( ((a) > (b)) ? (a) : (b))
#endif

/* Calibration states define */
#define VP886_SENSE_SWY     0
#define VP886_SENSE_SWZ     1
#define VP886_SENSE_TIP     2
#define VP886_SENSE_RING    3
#define VP886_SENSE_IO2     4

#define VP886_ERR_ILA       0
#define VP886_ERR_VOC       1
#define VP886_ERR_RINGBIAS  2
#define VP886_ERR_FLOOR     3
#define VP886_ERR_LOWPOWER  4
#define VP886_ERR_FIXEDRING 5
#define VP886_ERR_60V_LONG  6
#define VP886_ERR_BAT_SAT   7
#define VP886_ERR_ABS_LONG  8
#define VP886_ERR_BAT_LIMIT 9
#define VP886_ERR_HOOK      10
#define VP886_ERR_GND_KEY   11

/* Calibration increments */
#define VP886_ILA_CORR_STEP         571L
#define VP886_VOC_CORR_STEP         714L
#define VP886_HOOK_CORR_STEP        500L
#define VP886_HOOK_CORR_STEP_LP     1000L
#define VP886_GND_KEY_CORR_STEP     2000L
#define VP886_FIXEDBAT_CORR_STEP    630L
#define VP886_VAS_CORR_STEP         804L
#define VP886_LONG_CORR_STEP_REV1   857L
#define VP886_LONG_CORR_STEP_REV2   1143L
#define VP886_BAT_SAT_CORR_STEP     1608L
#define VP886_BAT_LIM_CORR_STEP     2857L

/* Measurement stability threshold */
#define VP886_ADC_ERR_TRESH   30
#define VP886_ADC_ERR_TRESH_GP   200

/* Reasonable gain range */
#define VP886_MAX_GAIN   1300
#define VP886_MIN_GAIN   700

/* The minimum required overhead near ground is different from the minimum
   overhead near battery.  Shift the longitudinal center point by half of this
   value to equalize the overhead between both sides.  A positive shift moves it
   closer to ground. */
#define VP886_LONG_OVERHEAD_SHIFT 2000

/* Battery saturation steady state target voltage (mV) */
#define VP886_BAT_SAT_TARGET 4500L

/* Standard calibration settling and integration times */
#define VP886_CAL_SETTLE_TIME 4
#define VP886_CAL_SETTLE_EXT_TIME 15
#define VP886_CAL_INTEGRATE_TIME 4
#define VP886_CAL_INTEGRATE_EXT_TIME 15

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"


static VpStatusType
Vp886ApplyCal(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCalProf);

static VpStatusType
Vp886ApplyCalProfile(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCalProf);

static void
Vp886PrepareCal(
    VpDevCtxType *pDevCtx);

static void
Vp886ConcludeCal(
    VpDevCtxType *pDevCtx);

static bool
Vp886SadcCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886SadcVModeCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886VadcActiveCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886SenseCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 path);

static bool
Vp886VabSenseCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886ImtCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886VocCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886RingingCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886VocSenseCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886FloorCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886TrackerCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886ABSCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886BatterySenseCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886SwitchHookCalibration(
    VpLineCtxType *pLineCtx);

static bool
Vp886BatterySatCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886SwitcherLimitCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static bool
Vp886GndKeyCalibration(
    VpLineCtxType *pLineCtx);

static void
Vp886ApplyCalTrackingBat(
    VpLineCtxType *pLineCtx,
    uint8 *polCal,
    uint8 polarity);

static void
Vp886ApplyCalAbsBat(
    VpLineCtxType *pLineCtx,
    uint8 *polCal,
    uint8 polarity);

static void
Vp886ApplyCalTrackingBatRinging(
    VpLineCtxType *pLineCtx,
    uint8 *ringCal);

static int32
Vp886ComputeTrackingBat(
    Vp886TrkCalDeviceDataType *pCalData,
    int32 vab,          /* mV */
    int32 vas,          /* mV */
    int32 imt,          /* uA */
    int32 rOverhead,    /* Ohms */
    uint8 polarity);

static int16
Vp886ComputeRingingParam(
    Vp886CmnCalDeviceDataType *pCalData,
    int16 ringParam);

static int32
Vp886ComputeError(
    VpLineCtxType *pLineCtx,
    uint8 errorType,
    int16 target,
    uint8 polarity);

static VpStatusType
Vp886AdjustCalReg(
    VpLineCtxType *pLineCtx,
    uint8 *calReg,
    uint8 errorType,
    int32 diff,
    uint8 polarity);

static bool
Vp886IsGainError(
    Vp886CalGainOffsetType* gainOffset,
    int16 calStep,
    uint8 channelId,
    bool calCodec);

/**
 * Vp886StoreIcal() -- Tracker and ABS Function
 *  This function reads the ICAL trim register and saves the updated calibration current in the
 * device object. This function should be called by Vp886InitDevice.
 *
 * Preconditions:
 *  - The line context must be created and initialized before calling this function.
 *  - The device object must be created and initialized before calling this function.
 *
 * Postconditions:
 *  Both Ical values are stored in the device object.
 */
void
Vp886StoreIcal(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 fuse8[VP886_R_FUSE_REG_8_LEN];
    int32 iCalLowPc, iCalHighPc;
    uint8 icl, ich;

    VpSlacRegRead(pDevCtx, NULL, VP886_R_FUSE_REG_8_RD, VP886_R_FUSE_REG_8_LEN, fuse8);

    /* Convert the 6 bits signed value (2's complement) to int32 */
    icl = fuse8[1] & 0x3F;
    if (icl & 0x20) {
        icl = (~icl & 0x3F) + 1;
        iCalLowPc = (int32)icl * -1;
    } else {
        iCalLowPc = (int32)icl;
    }

    ich = fuse8[0] & 0x3F;
    if (ich & 0x20) {
        ich = (~ich & 0x3F) + 1;
        iCalHighPc = (int32)ich * -1;
    } else {
        iCalHighPc = (int32)ich;
    }

    /* Save ICAL_LOW in 10nA increment */
    pDevObj->icalL = (int16)((25000L + (250L * iCalLowPc) / 10L) / -100L);

    /* Save ICAL_HIGH in 10nA increment */
    pDevObj->icalH = (int16)((500000L + (5000L * iCalHighPc) / 10L) / -100L);

    VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ICAL_LOW = %d (x10nA), ICAL_HIGH = %d (x10nA)",
        pDevObj->icalL, pDevObj->icalH));
}

/**
 * Vp886SetSingleAdcMath() -- Tracker and ABS Function
 *  This function initiates an ADC measurement. The measurement will be done in single mode
 * using the math block.
 *
 * Preconditions:
 *  The line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  The measurement will be available in the Supervision Data Buffer 1 when the SADC generates an interrupt.
 */
void
Vp886SetSingleAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 sadcRoute,
    uint16 settlingTimeMs,
    uint16 integrationTimeMs)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 settlingSamples;
    uint16 integrationSamples;
    uint8 convConf[VP886_R_SADC_LEN];
    uint8 prevEcVal = pDevObj->ecVal;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;

    /* Enable the SADC in math mode with a sampling rate of 2kHz */
    convConf[0] = VP886_R_SADC_ENABLE | VP886_R_SADC_MATH | VP886_R_SADC_DRATE_SINGLE_2KHZ |
        VP886_R_SADC_TX_INTERRUPT;

    /* Program the SADC route */
    convConf[1] = sadcRoute;

    /* Save the SADC route */
    pDevObj->sadcSetting[channelId] = sadcRoute;

    /* Set the remaining ADCs to no-connect (minimize noise) */
    convConf[2] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[3] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[4] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[5] = VP886_R_SADC_SEL_ADC_OFFSET;

    /* Compute the number of samples assuming a 2kHz sampling rate */
    integrationSamples = integrationTimeMs * 2;
    convConf[6] = (uint8)((integrationSamples >> 8) & 0x00FF);
    convConf[7] = (uint8)(integrationSamples & 0x00FF);
    settlingSamples = settlingTimeMs * 2;
    convConf[8] = (uint8)((settlingSamples >> 8) & 0x00FF);
    convConf[9] = (uint8)(settlingSamples & 0x00FF);

    /* Setup the EC channel */
    pDevObj->ecVal = ecVal;

    /* Send down the Supervision Converter Configuration */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);

    /* Restore the previous EC value */
    pDevObj->ecVal = prevEcVal;
}

/**
 * Vp886SetGroupAdcMath() -- Tracker and ABS Function
 *  This function initiates an ADC measurement. The measurement will be done in group mode
 * using the math block.
 *
 * Preconditions:
 *  The line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  The measurement will be available in the Supervision Data Buffers 1 when the SADC generates an interrupt.
 */
void
Vp886SetGroupAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 sadcRoute[5],
    uint16 settlingTimeMs,
    uint16 integrationTimeMs)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 settlingSamples;
    uint16 integrationSamples;
    uint8 convConf[VP886_R_SADC_LEN];
    uint8 prevEcVal = pDevObj->ecVal;
    uint8 route;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;

    /* Enable the SADC in math mode with a sampling rate of 2kHz */
    convConf[0] = VP886_R_SADC_ENABLE | VP886_R_SADC_MATH | VP886_R_SADC_GROUP_MODE |
        VP886_R_SADC_DRATE_GROUP_2KHZ | VP886_R_SADC_TX_INTERRUPT;

    for (route = 0 ; route < 5 ; route++) {
        /* Program the SADC routes */
        convConf[1 + route] = sadcRoute[route];

        /* Save the SADC routes */
        pDevObj->sadcSettingGp[channelId][route] = sadcRoute[route];
    }

    /* Compute the number of samples assuming a 2kHz sampling rate */
    integrationSamples = integrationTimeMs * 2;
    convConf[6] = (uint8)((integrationSamples >> 8) & 0x00FF);
    convConf[7] = (uint8)(integrationSamples & 0x00FF);
    settlingSamples = settlingTimeMs * 2;
    convConf[8] = (uint8)((settlingSamples >> 8) & 0x00FF);
    convConf[9] = (uint8)(settlingSamples & 0x00FF);

    /* Setup the EC channel */
    pDevObj->ecVal = ecVal;

    /* Send down the Supervision Converter Configuration */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);

    /* Restore the previous EC value */
    pDevObj->ecVal = prevEcVal;
}

/**
 * Vp886SetSingleVadcMath() -- Tracker and ABS Function
 *  This function initiates a VADC measurement. The measurement will be done in single mode
 * using the math block.
 *
 * Preconditions:
 *  The line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  The measurement will be available in the Supervision Data Buffer 1 when the timer expires.
 */
void
Vp886SetSingleVadcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 vadcRoute,
    uint16 settlingTimeMs,
    uint16 integrationTimeMs)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 settlingSamples;
    uint16 integrationSamples;
    uint8 convConf[VP886_R_VADC_LEN];
    uint8 prevEcVal = pDevObj->ecVal;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;

    /* Enable the VADC in math mode with a sampling rate of 2kHz */
    /* No interrup. Use a timer */
    convConf[0] = VP886_R_VADC_SM_OVERRIDE | VP886_R_VADC_MATH | VP886_R_VADC_DRATE_2KHZ;

    /* Program the VADC route */
    convConf[1] = vadcRoute;

    /* Compute the number of samples assuming a 2kHz sampling rate */
    integrationSamples = integrationTimeMs * 2;
    convConf[2] = (uint8)((integrationSamples >> 8) & 0x00FF);
    convConf[3] = (uint8)(integrationSamples & 0x00FF);
    settlingSamples = settlingTimeMs * 2;
    convConf[4] = (uint8)((settlingSamples >> 8) & 0x00FF);
    convConf[5] = (uint8)(settlingSamples & 0x00FF);

    /* Setup the EC channel */
    pDevObj->ecVal = ecVal;

    /* Send down the Voice Converter Configuration */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_VADC_WRT, VP886_R_VADC_LEN, convConf);

    /* Restore the previous EC value */
    pDevObj->ecVal = prevEcVal;
}

/**
 * Vp886GetSingleAdcMath() -- Tracker and ABS Function
 *  This function reads the Supervision Data Buffer 1, it should be called when
 * Vp886SetSingleAdcMath() generates an interrupt.
 *
 * Preconditions:
 *  - The line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  - The average value is returned.
 *  - SADC A2D signals are set to "no connect" (0x0B)
 */
int16
Vp886GetSingleAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    bool adcScale)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
    Vp886LineObjectType *pLineObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    uint8 sBuffer1[VP886_R_B1_LEN];
    uint16 numSamples;
    int32 sumSamples;
    int16 min, max, result, tmpResult, gain, offset;
    uint8 convConf[VP886_R_SADC_LEN] = {0x00, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00};
    uint8 prevEcVal = pDevObj->ecVal;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;

    /* Setup the EC channel */
    pDevObj->ecVal = ecVal;

    /* Read the Supervision Data Buffer 1 */
    VpSlacRegRead(pDevCtx, NULL, VP886_R_B1_RD, VP886_R_B1_LEN, sBuffer1);

    /* Turn the SADC off (Math mode workaround, otherwise a bad sample may show-up in the sum)
       Don't do this during line test, because if PCLK is gone this will prevent
       the next measurement from starting, and the quick cal at the start of the
       test will hang. */
    if (pLineCtx == VP_NULL) {
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);
    } else {
        pLineObj = pLineCtx->pLineObj;
        if (!pLineObj->inLineTest) {
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);
        }
    }

    /* Restore the previous EC value */
    pDevObj->ecVal = prevEcVal;

    numSamples = ((uint16)(sBuffer1[2]) << 8) & 0xFF00;
    numSamples |= (uint16)(sBuffer1[3]) & 0x00FF;

    /* Check the number of samples */
    if (numSamples == 0) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetSingleAdcMath(ch:%d|0x%02X): Empty buffer",
            channelId, pDevObj->sadcSetting[channelId]));
        return 0;
    } else if (numSamples == 0xFFFF) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetSingleAdcMath(ch:%d|0x%02X): Buffer overrun",
            channelId, pDevObj->sadcSetting[channelId]));
        return 0;
    }

    sumSamples = ((int32)(sBuffer1[8]) << 24) & 0xFF000000;
    sumSamples |= ((int32)(sBuffer1[9]) << 16) & 0x00FF0000;
    sumSamples |= ((int32)(sBuffer1[10]) << 8) & 0x0000FF00;
    sumSamples |= ((int32)sBuffer1[11]) & 0x000000FF;

    /* Get min and max to check is the measured data is stable */
    min = ((int16)(sBuffer1[4]) << 8) & 0xFF00;
    min |= (int16)(sBuffer1[5]) & 0x00FF;
    max = ((int16)(sBuffer1[6]) << 8) & 0xFF00;
    max |= (int16)(sBuffer1[7]) & 0x00FF;

    if (ABS(min - max) > VP886_ADC_ERR_TRESH) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Vp886GetSingleAdcMath(ch:%d|0x%02X): Unsettled measurement (min:%d, max:%d)",
            channelId, pDevObj->sadcSetting[channelId], min, max));
    }
    /* Compute the average data */
    result = (int16)(sumSamples / (int32)numSamples);

    /* Use the proper sense calibration */
    switch (pDevObj->sadcSetting[channelId]) {
        case VP886_R_SADC_SEL_SWY:
            gain = pCalData->swySense.gain;
            offset = pCalData->swySense.offset;
            break;

        case VP886_R_SADC_SEL_SWZ:
            gain = pCalData->swzSense.gain;
            offset = pCalData->swzSense.offset;
            break;

        case VP886_R_SADC_SEL_TIP_GROUND_V:
            gain = pCalData->tipSense.gain;
            offset = pCalData->tipSense.offset;
            break;

        case VP886_R_SADC_SEL_RING_GROUND_V:
            gain = pCalData->ringSense.gain;
            offset = pCalData->ringSense.offset;
            break;

        case VP886_R_SADC_SEL_TIP_RING_DC_V:
            /* Normal and reverse polarity are close */
            gain = pCalData->vabSenseNormal.gain;
            offset = pCalData->vabSenseNormal.offset;
            break;

        default:
            /* SADC calibration */
            gain = pCalData->sadc.gain;
            offset = pCalData->sadc.offset;
            break;
    }

    /* Apply the SADC gain and offset if requested */
    if (adcScale) {
        if (pCalData->sadc.gain == 0) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetSingleAdcMath(ch:%d|0x%02X): calibration issue",
                channelId, pDevObj->sadcSetting[channelId]));
            return result;
        }

        tmpResult = VpRoundedDivide(((int32)result + (int32)offset) * (int32)gain, 1000L);

        /* Check for saturation */
        if ((result < -20000) && (tmpResult > 0)) {
            VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886GetSingleAdcMath(ch:%d|0x%02X): Saturation VP_INT16_MIN",
                channelId, pDevObj->sadcSetting[channelId]));
            result = VP_INT16_MIN;
        } else if ((result > 20000) && (tmpResult < 0)) {
            VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886GetSingleAdcMath(ch:%d|0x%02X): Saturation VP_INT16_MAX",
                channelId, pDevObj->sadcSetting[channelId]));
            result = VP_INT16_MAX;
        } else {
            result = tmpResult;
        }
    }

    return result;
}

/**
 * Vp886GetGroupAdcMath() -- Tracker and ABS Function
 *  This function reads the Supervision Data Buffers, it should be called when
 * Vp886SetGroupAdcMath() generates an interrupt.
 *
 * Preconditions:
 *  - The line context must be created and initialized before calling this function.
 *  - numRoutes specifies the number of result buffers to read, starting from
 *    buffer 1.
 *
 * Postconditions:
 *  - The average value is returned.
 *  - SADC A2D signals are set to "no connect" (0x0B)
 */
bool
Vp886GetGroupAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    int16* result,
    bool adcScale,
    uint8 numRoutes)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
    Vp886LineObjectType *pLineObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    uint8 sBuffer[5][VP886_R_B1_LEN];
    uint16 numSamples;
    int32 sumSamples;
    int16 min, max, tmpResult, gain, offset;
    uint8 convConf[VP886_R_SADC_LEN] = {0x00, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00};
    uint8 prevEcVal = pDevObj->ecVal;
    uint8 route;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    bool issue = FALSE;

    /* Setup the EC channel */
    pDevObj->ecVal = ecVal;

    /* Read the Supervision Data Buffers, up to the number requested */
    switch (numRoutes) {
        case 5:
            VpSlacRegRead(pDevCtx, NULL, VP886_R_B5_RD, VP886_R_B5_LEN, sBuffer[4]);
        case 4:
            VpSlacRegRead(pDevCtx, NULL, VP886_R_B4_RD, VP886_R_B4_LEN, sBuffer[3]);
        case 3:
            VpSlacRegRead(pDevCtx, NULL, VP886_R_B3_RD, VP886_R_B3_LEN, sBuffer[2]);
        case 2:
            VpSlacRegRead(pDevCtx, NULL, VP886_R_B2_RD, VP886_R_B2_LEN, sBuffer[1]);
        case 1:
            VpSlacRegRead(pDevCtx, NULL, VP886_R_B1_RD, VP886_R_B1_LEN, sBuffer[0]);
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetGroupAdcMath(ch:%d): Invalid numRoutes %d",
                channelId, numRoutes));
            return TRUE;
    }

    /* Turn the SADC off (Math mode workaround, otherwise a bad sample may show-up in the sum)
       Don't do this during line test, because if PCLK is gone this will prevent
       the next measurement from starting, and the quick cal at the start of the
       test will hang. */
    if (pLineCtx == VP_NULL) {
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);
    } else {
        pLineObj = pLineCtx->pLineObj;
        if (!pLineObj->inLineTest) {
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);
        }
    }

    /* Restore the previous EC value */
    pDevObj->ecVal = prevEcVal;

    for (route = 0 ; route < numRoutes ; route++) {
        numSamples = ((uint16)(sBuffer[route][2]) << 8) & 0xFF00;
        numSamples |= (uint16)(sBuffer[route][3]) & 0x00FF;

        /* Check the number of samples */
        if (numSamples == 0) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetGroupAdcMath(ch:%d|0x%02X): Empty buffer",
                channelId, pDevObj->sadcSettingGp[channelId][route]));
            issue = TRUE;
        } else if (numSamples == 0xFFFF) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetGroupAdcMath(ch:%d|0x%02X): Buffer overrun",
                channelId, pDevObj->sadcSettingGp[channelId][route]));
            issue = TRUE;
            return 0;
        }

        sumSamples = ((int32)(sBuffer[route][8]) << 24) & 0xFF000000;
        sumSamples |= ((int32)(sBuffer[route][9]) << 16) & 0x00FF0000;
        sumSamples |= ((int32)(sBuffer[route][10]) << 8) & 0x0000FF00;
        sumSamples |= ((int32)sBuffer[route][11]) & 0x000000FF;

        /* Get min and max to check is the measured data is stable */
        min = ((int16)(sBuffer[route][4]) << 8) & 0xFF00;
        min |= (int16)(sBuffer[route][5]) & 0x00FF;
        max = ((int16)(sBuffer[route][6]) << 8) & 0xFF00;
        max |= (int16)(sBuffer[route][7]) & 0x00FF;

        if (ABS(min - max) > VP886_ADC_ERR_TRESH_GP) {
            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886GetGroupAdcMath(ch:%d|0x%02X): Unsettled measurement (min:%d, max:%d)",
                channelId, pDevObj->sadcSettingGp[channelId][route], min, max));
        }
        /* Compute the average data */
        result[route] = (int16)(sumSamples / (int32)numSamples);

        /* Use the proper sense calibration */
        switch (pDevObj->sadcSettingGp[channelId][route]) {
            case VP886_R_SADC_SEL_SWY:
                gain = pCalData->swySense.gain;
                offset = pCalData->swySense.offset;
                break;

            case VP886_R_SADC_SEL_SWZ:
                gain = pCalData->swzSense.gain;
                offset = pCalData->swzSense.offset;
                break;

            case VP886_R_SADC_SEL_TIP_GROUND_V:
                gain = pCalData->tipSense.gain;
                offset = pCalData->tipSense.offset;
                break;

            case VP886_R_SADC_SEL_RING_GROUND_V:
                gain = pCalData->ringSense.gain;
                offset = pCalData->ringSense.offset;
                break;

            case VP886_R_SADC_SEL_TIP_RING_DC_V:
                /* Normal and reverse polarity are close */
                gain = pCalData->vabSenseNormal.gain;
                offset = pCalData->vabSenseNormal.offset;
                break;

            default:
                /* SADC calibration */
                gain = pCalData->sadc.gain;
                offset = pCalData->sadc.offset;
                break;
        }

        /* Apply the SADC gain and offset if requested */
        if (adcScale) {
            if (pCalData->sadc.gain == 0) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetGroupAdcMath(ch:%d|0x%02X): calibration issue",
                    channelId, pDevObj->sadcSettingGp[channelId][route]));
                issue = TRUE;
                continue;
            }

            tmpResult = VpRoundedDivide(((int32)result[route] + (int32)offset) * (int32)gain, 1000L);

            /* Check for saturation */
            if ((result[route] < -20000) && (tmpResult > 0)) {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886GetGroupAdcMath(ch:%d|0x%02X): Saturation VP_INT16_MIN",
                    channelId, pDevObj->sadcSettingGp[channelId][route]));
                result[route] = VP_INT16_MIN;
            } else if ((result[route] > 20000) && (tmpResult < 0)) {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886GetGroupAdcMath(ch:%d|0x%02X): Saturation VP_INT16_MAX",
                    channelId, pDevObj->sadcSettingGp[channelId][route]));
                result[route] = VP_INT16_MAX;
            } else {
                result[route] = tmpResult;
            }
        }
    }

    return issue;
}

/**
 * Vp886GetSingleVadcMath() -- Tracker and ABS Function
 *  This function reads the Voice Data Buffer, it should be called when
 * Vp886SetSingleVadcMath() timer expires.
 *
 * Preconditions:
 *  - The line context must be created and initialized before calling this function.
 *  - Vp886SetSingleVadcMath() timer must have expired.
 *
 * Postconditions:
 *  - The average value is returned.
 *  - VADC A2D signal is set to "no connect" (0x0B)
 */
int16
Vp886GetSingleVadcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    bool adcScale)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    uint8 vBuffer[VP886_R_VBUFFER_LEN];
    uint16 numSamples;
    int32 sumSamples;
    int16 min, max, result;
    uint8 prevEcVal = pDevObj->ecVal;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;

    /* Setup the EC channel */
    pDevObj->ecVal = ecVal;

    /* Read the Voice Data Buffer 1 */
    VpSlacRegRead(pDevCtx, NULL, VP886_R_VBUFFER_RD, VP886_R_VBUFFER_LEN, vBuffer);

    numSamples = ((uint16)(vBuffer[2]) << 8) & 0xFF00;
    numSamples |= (uint16)(vBuffer[3]) & 0x00FF;

    /* Restore the previous EC value */
    pDevObj->ecVal = prevEcVal;

    /* Check the number of samples */
    if (numSamples == 0) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetSingleVadcMath(ch:%d): Empty buffer", channelId));
        return 0;
    } else if (numSamples == 0xFFFF) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetSingleVadcMath(ch:%d): Buffer overrun", channelId));
        return 0;
    }

    sumSamples = ((int32)(vBuffer[8]) << 24) & 0xFF000000;
    sumSamples |= ((int32)(vBuffer[9]) << 16) & 0x00FF0000;
    sumSamples |= ((int32)(vBuffer[10]) << 8) & 0x0000FF00;
    sumSamples |= ((int32)vBuffer[11]) & 0x000000FF;


    /* Get min and max to check is the measured data is stable */
    min = ((int16)(vBuffer[4]) << 8) & 0xFF00;
    min |= (int16)(vBuffer[5]) & 0x00FF;
    max = ((int16)(vBuffer[6]) << 8) & 0xFF00;
    max |= (int16)(vBuffer[7]) & 0x00FF;

    if (ABS(min - max) > VP886_ADC_ERR_TRESH) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx,
            ("Vp886GetSingleVadcMath(ch:%d): Unsettled measurement (min:%d, max:%d)",
            channelId, min, max));
    }
    /* Compute the average data */
    result = (int16)(sumSamples / (int32)numSamples);

    /* Apply the VADC gain and offset if requested */
    if (adcScale) {
        if (pCalData->vadcActive.gain == 0) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetSingleVadcMath(ch:%d): VADC calibration issue",
            channelId));
            return result;
        }

        result = (int16)(((int32)result + (int32)(pCalData->vadcActive.offset))
            * (int32)(pCalData->vadcActive.gain) / 1000L);
    }

    return result;
}

#if defined (VP_CSLAC_RUNTIME_CAL_ENABLED)
/**
 * Vp886CalCodec() -- Tracker and ABS Function
 *  This function initiates a calibration operation for analog circuits
 * associated with a given device. See VP-API reference guide for more
 * information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp886CalCodec(
    VpLineCtxType *pLineCtx,
    VpDeviceCalType mode)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channel;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886CalCodec");

    /* Set the lines in disconnect and default the calibration registers */
    for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
        VpLineCtxType *pLineCtxLocal = pDevCtx->pLineCtx[channel];
        Vp886LineObjectType *pLineObjLocal = pLineCtxLocal->pLineObj;

        /* Default the calibration registers */
        VpSlacRegRead(NULL, pLineCtxLocal, VP886_R_INDCAL_RD, VP886_R_INDCAL_LEN,
            pLineObjLocal->registers.indCal);

        pLineObjLocal->registers.indCal[0] = 0x00;
        pLineObjLocal->registers.indCal[1] = 0x80;
        pLineObjLocal->registers.indCal[2] &= 0xF0;

        pLineObjLocal->registers.normCal[0] = 0x00;
        pLineObjLocal->registers.normCal[1] = 0x00;
        pLineObjLocal->registers.normCal[2] = 0x00;

        pLineObjLocal->registers.revCal[0] = 0x00;
        pLineObjLocal->registers.revCal[1] = 0x00;
        pLineObjLocal->registers.revCal[2] = 0x00;

        pLineObjLocal->registers.ringCal[0] = 0x00;
        pLineObjLocal->registers.ringCal[1] = 0x00;

        if (VP886_IS_TRACKER(pDevObj)) {
            pLineObjLocal->registers.batCal[0] = 0x00;
            pLineObjLocal->registers.batCal[1] = 0x00;
        } else { /* ABS */
            pLineObjLocal->registers.batCal[0] = 0x04;
            pLineObjLocal->registers.batCal[1] = 0x00;
        }

        Vp886ProgramCalRegisters(pLineCtxLocal);

        /* VpCalCodec() runs in disconnect */
        Vp886SetLineStateFxs(pLineCtxLocal, VP_LINE_DISCONNECT);

        /* Re-calibrate */
        pDevObj->calData[channel].valid = FALSE;
    }

    /* Set the first step of calCodec */
    pDevObj->calCodecState[0] = VP886_CAL_CODEC_START;

    Vp886CalCodecHandler(pDevCtx);

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886CalCodec");

    return VP_STATUS_SUCCESS;
}

/**
 * Vp886CalLine() -- Tracker and ABS Function
 *  This function initiates a calibration operation for analog circuits
 * associated with a given line. See VP-API reference guide for more
 * information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp886CalLine(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886CalLine");

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886CalLine");
        return status;
    }
    
    pLineObj->calLineState = VP886_CAL_LINE_START;
    Vp886CalLineSM(pLineCtx);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886CalLine");
    return status;
}
#endif

/**
 * Vp886Cal() -- Tracker and ABS Function
 *  This function calibrates a selected block of the device/line.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp886Cal(
    VpLineCtxType *pLineCtx,
    VpCalType calType,
    void *inputArgs)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886Cal");

    switch (calType) {
        case VP_CAL_APPLY_LINE_COEFF: {
            status = Vp886ApplyCal(pLineCtx, (VpProfilePtrType)inputArgs);
            break;
        }
        case VP_CAL_GET_LINE_COEFF: {
            VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
            Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
            Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
            uint8 channelId = pLineObj->channelId;
            uint8 profLen;
            uint8 mpiLen;

            if (!pDevObj->calData[channelId].valid) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Line has not been calibrated"));
                status = VP_STATUS_LINE_NOT_CONFIG;
                break;
            }
            if (VP886_IS_TRACKER(pDevObj)) {
                mpiLen = 0;
                profLen = 4 + VP886_CAL_PROF_TRACKER_LENGTH + 2 + mpiLen;
            } else {
                mpiLen = 0;
                profLen = 4 + VP886_CAL_PROF_ABS_LENGTH + 2 + mpiLen;
            }
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP, profLen, Vp886GetTimestamp(pDevCtx), TRUE);
            break;
        }
        case VP_CAL_QUICKCAL: {
            Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
            if (!(pLineObj->lineState.usrCurrent == VP_LINE_DISCONNECT ||
                  (pLineObj->lineState.usrCurrent == VP_LINE_STANDBY &&
                   pLineObj->termType == VP_TERM_FXS_LOW_PWR)))
            {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Line must be in DISCONNECT or low-power STANDBY"));
                status = VP_STATUS_DEVICE_BUSY;
                break;
            }
            if (pLineObj->quickCal.state != VP886_QUICKCAL_INACTIVE) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Line is already performing QUICKCAL"));
                status = VP_STATUS_DEVICE_BUSY;
                break;
            }
            Vp886QuickCalStart(pLineCtx, VP886_TIMERID_QUICKCAL, 0, VP886_LINE_QUICKCAL, TRUE);
            break;
        }
        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886Cal");
    return status;
}   /* Vp886Cal() */

VpStatusType
Vp886ApplyCal(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCalProf)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    /* Parts of this function will depend on knowing the exact device type and
       revision code.  If the device has not yet been initialized, we need to
       read the PCN and RCN. */
    if (!(pDevObj->busyFlags & VP_DEV_INIT_CMP)) {
        status = Vp886InitDevicePcnRcn(pDevCtx);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }
    }

    /* Check for valid profile arguments. */
    if (Vp886GetProfileArg(pDevCtx, VP_PROFILE_CAL, &pCalProf) != VP_STATUS_SUCCESS) {
        return VP_STATUS_ERR_PROFILE;
    }

    /* If the profile argument is NULL, just clear the cal "valid" flag for the
       channel so that calibration will be repeated upon the next InitDevice.
       We do not actually clear the cal factors here because the device is not
       supposed to be run uncalibrated. */
    if (pCalProf == VP_PTABLE_NULL) {
        pDevObj->calData[channelId].valid = FALSE;
        if (pDevObj->busyFlags & VP_DEV_INIT_CMP) {
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP, 0, Vp886GetTimestamp(pDevCtx), FALSE);
        }
        return VP_STATUS_SUCCESS;
    }

    /* Copy profile data to device object. */
    status = Vp886ApplyCalProfile(pLineCtx, pCalProf);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Program the cal registers and generate the CAL_CMP event only if the
       device has been initialized */
    if (pDevObj->busyFlags & VP_DEV_INIT_CMP) {
        /* Apply calibration factors that do not depend on DC or Ring profile
           parameters */
        status = Vp886ApplyCalGeneral(pLineCtx);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }

        /* Apply calibration factors that modify DC profile parameters */
        status = Vp886ApplyCalDC(pLineCtx);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }

        /* Apply calibration factors that modify Ring profile parameters */
        status = Vp886ApplyCalRing(pLineCtx);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }

        /* Write the new cached values to the registers.  This is done instead
           of writing to the registers in each of the above functions to
           reduce traffic to the device. */
        Vp886ProgramCalRegisters(pLineCtx);

        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP, 0, Vp886GetTimestamp(pDevCtx), FALSE);
    }

    return VP_STATUS_SUCCESS;
}

VpStatusType
Vp886ApplyCalProfile(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCalProf)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CalDeviceDataType *pCalData = &pDevObj->calData[channelId];
    uint8 version = pCalProf[VP_PROFILE_VERSION];
    uint8 expectedLength;
    uint8 mpiLen = pCalProf[VP_PROFILE_MPI_LEN];
    uint8 length = pCalProf[VP_PROFILE_LENGTH] - 2 - mpiLen;
    VpProfilePtrType pProf = pCalProf + VP_PROFILE_MPI_LEN + 1;

    /* Send MPI data if there is any */
    if (mpiLen > 0) {
        VpSlacRegWrite(VP_NULL, pLineCtx, *pProf, mpiLen - 1, pProf + 1);
        pProf += mpiLen;
    }

    if (VP886_IS_TRACKER(pDevObj)) {
        switch (version) {
            case 0:
                expectedLength = VP886_CAL_PROF_TRACKER_LENGTH_V0;
                break;
            case 1:
                expectedLength = VP886_CAL_PROF_TRACKER_LENGTH_V1;
                break;
            case 2:
                expectedLength = VP886_CAL_PROF_TRACKER_LENGTH_V2;
                break;
            case 3:
                expectedLength = VP886_CAL_PROF_TRACKER_LENGTH_V3;
                break;
            case 4:
                expectedLength = VP886_CAL_PROF_TRACKER_LENGTH_V4;
                break;
            case 5:
                expectedLength = VP886_CAL_PROF_TRACKER_LENGTH_V5;
                break;
            default:
                VP_ERROR(VpLineCtxType, pLineCtx, ("Invalid Tracker cal profile version %d, maximum %d",
                    version, VP886_CAL_PROF_TRACKER_MAX_VERSION));
                return VP_STATUS_ERR_PROFILE;
        }

        if (length != expectedLength) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Invalid Tracker cal profile length (minus header&MPI) %d, expecting %d",
                length, expectedLength));
            return VP_STATUS_ERR_PROFILE;
        }

        pCalData->cmn.sadc.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.sadc.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        if (version >= 1) {
            pCalData->cmn.sadcVMode.gain = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.sadcVMode.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
        }
        pCalData->cmn.swySense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swySense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swzSense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swzSense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        if (version >= 3) {
            pCalData->cmn.io2Sense.gain = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.io2Sense.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
        }
        pCalData->cmn.tipSense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.tipSense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringSense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringSense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocBuffer.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocBuffer.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingGenerator.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingGenerator.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingBuffer.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingBuffer.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocSenseNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swLimit100V.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.fixedBat.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.fixedBat.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.reserved_5.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.reserved_5.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.reserved_6.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.reserved_6.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVabNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVabNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVabReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVabReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVasNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVasNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVasReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.trk.trackerVasReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.batterySense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longActive.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longActive.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longRinging.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longRinging.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.reserved_1.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.reserved_2.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        if (version < 5) {
            pCalData->cmn.vabSenseRinging.gain = pCalData->cmn.vabSenseNormal.gain;
        } else {
            pCalData->cmn.vabSenseRinging.gain = (pProf[0] << 8) + pProf[1];
        }
        pProf += 2;
        if (version < 5) {
            pCalData->cmn.vabSenseRinging.offset = pCalData->cmn.vabSenseNormal.gain;
        } else {
            pCalData->cmn.vabSenseRinging.offset = (pProf[0] << 8) + pProf[1];
        }
        pProf += 2;
        if (version >= 2) {
            pCalData->cmn.vadcActive.gain = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.vadcActive.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
        }
        if (version >= 4) {
            pCalData->cmn.tipCapCal = (pProf[0] << 24) + (pProf[1] << 16) + (pProf[2] << 8) + pProf[3];
            pProf += 4;
            pCalData->cmn.hookLPM.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.hookReverse.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.hookNormal.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.batSat.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.swLimit50V.gain = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.swLimit50V.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.ringCapCal = (pProf[0] << 24) + (pProf[1] << 16) + (pProf[2] << 8) + pProf[3];
            pProf += 4;
            pCalData->cmn.gndKeyLong.offset = (pProf[0] << 8) + pProf[1];
            /* gkey calibration is no longer required */
            pCalData->cmn.gndKeyLong.offset = 0;
            pProf += 2;
        }

        pCalData->valid = TRUE;
    }

    if (VP886_IS_ABS(pDevObj)) {
        switch (version) {
            case 0:
                expectedLength = VP886_CAL_PROF_ABS_LENGTH_V0;
                break;
            case 1:
                expectedLength = VP886_CAL_PROF_ABS_LENGTH_V1;
                break;
            case 2:
                expectedLength = VP886_CAL_PROF_ABS_LENGTH_V2;
                break;
            case 3:
                expectedLength = VP886_CAL_PROF_ABS_LENGTH_V3;
                break;
            default:
                VP_ERROR(VpLineCtxType, pLineCtx, ("Invalid ABS cal profile version %d, maximum %d",
                    version, VP886_CAL_PROF_ABS_MAX_VERSION));
                return VP_STATUS_ERR_PROFILE;
        }

        if (length != expectedLength) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Invalid ABS cal profile length (minus header&MPI) %d, expecting %d",
                length, expectedLength));
            return VP_STATUS_ERR_PROFILE;
        }

        pCalData->cmn.sadc.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.sadc.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.sadcVMode.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.sadcVMode.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swySense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swySense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swzSense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swzSense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        if (version >= 1) {
            pCalData->cmn.io2Sense.gain = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.io2Sense.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
        }
        pCalData->cmn.tipSense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.tipSense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringSense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringSense.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vabSenseReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ilaReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocBuffer.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocBuffer.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingGenerator.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingGenerator.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingBuffer.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.ringingBuffer.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vocSenseNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.swLimit100V.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.fixedBat.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.fixedBat.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_5.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_5.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_6.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_6.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.absVabNormal.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.absVabNormal.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.absVabReverse.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.absVabReverse.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_7.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_7.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_8.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->spe.abs.reserved_8.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.batterySense.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longActive.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longActive.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longRinging.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.longRinging.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.reserved_1.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.reserved_2.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        if (version < 3) {
            pCalData->cmn.vabSenseRinging.gain = pCalData->cmn.vabSenseNormal.gain;
        } else {
            pCalData->cmn.vabSenseRinging.gain = (pProf[0] << 8) + pProf[1];
        }
        pProf += 2;
        if (version < 3) {
            pCalData->cmn.vabSenseRinging.offset = pCalData->cmn.vabSenseNormal.gain;
        } else {
            pCalData->cmn.vabSenseRinging.offset = (pProf[0] << 8) + pProf[1];
        }
        pProf += 2;
        pCalData->cmn.vadcActive.gain = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        pCalData->cmn.vadcActive.offset = (pProf[0] << 8) + pProf[1];
        pProf += 2;
        if (version >= 2) {
            pCalData->cmn.tipCapCal = (pProf[0] << 24) + (pProf[1] << 16) + (pProf[2] << 8) + pProf[3];
            pProf += 4;
            pCalData->cmn.hookLPM.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.hookReverse.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.hookNormal.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.batSat.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.swLimit50V.gain = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.swLimit50V.offset = (pProf[0] << 8) + pProf[1];
            pProf += 2;
            pCalData->cmn.ringCapCal = (pProf[0] << 24) + (pProf[1] << 16) + (pProf[2] << 8) + pProf[3];
            pProf += 4;
            pCalData->cmn.gndKeyLong.offset = (pProf[0] << 8) + pProf[1];
            /* gkey calibration is no longer required */
            pCalData->cmn.gndKeyLong.offset = 0;
            pProf += 2;
        }

        pCalData->valid = TRUE;
    }

    return VP_STATUS_SUCCESS;
}

VpStatusType
Vp886ApplyCalGeneral(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    int32 diff;
    int16 floor;

    diff = Vp886ComputeError(pLineCtx, VP886_ERR_60V_LONG, -60, VP_POL_NONE);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.indCal, VP886_ERR_60V_LONG, diff, VP_POL_NONE);

    if (VP886_IS_ABS(pDevObj)) {
        /* Correct switcher floor voltage */
        if (pLineObj->channelId == 0) {
            floor = pDevObj->swyv;
        } else {
            floor = pDevObj->swzv;
        }
        diff = Vp886ComputeError(pLineCtx, VP886_ERR_FLOOR, floor, VP_POL_NONE);
        Vp886AdjustCalReg(pLineCtx, pLineObj->registers.batCal,  VP886_ERR_FLOOR, diff, VP_POL_NONE);

        /* Apply the battery switch calibration to both polarity */
        Vp886ApplyCalAbsBat(pLineCtx, pLineObj->registers.normCal, VP_NORMAL);
        Vp886ApplyCalAbsBat(pLineCtx, pLineObj->registers.revCal, VP_POLREV);

        if (pLineObj->channelId == 0) {
            /* Compute feed longitudinal correction based on SWY */
            diff = Vp886ComputeError(pLineCtx, VP886_ERR_ABS_LONG, pDevObj->swyv, VP_POL_NONE);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.normCal,  VP886_ERR_ABS_LONG, diff, VP_POL_NORM);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.revCal,  VP886_ERR_ABS_LONG, diff, VP_POL_REV);

            /* Compute ringing longitudinal correction based on SWZ */
            diff = Vp886ComputeError(pLineCtx, VP886_ERR_ABS_LONG, pDevObj->swzv, VP_POL_NONE);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.ringCal,  VP886_ERR_ABS_LONG, diff, VP_POL_NONE);
        } else {
            /* Compute feed longitudinal correction based on SWY */
            diff = Vp886ComputeError(pLineCtx, VP886_ERR_ABS_LONG, pDevObj->swyv, VP_POL_NONE);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.normCal,  VP886_ERR_ABS_LONG, diff, VP_POL_NORM);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.revCal,  VP886_ERR_ABS_LONG, diff, VP_POL_REV);

            /* Compute ringing longitudinal correction based on SWZ */
            diff = Vp886ComputeError(pLineCtx, VP886_ERR_ABS_LONG, pDevObj->swzv, VP_POL_NONE);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.ringCal,  VP886_ERR_ABS_LONG, diff, VP_POL_NONE);
        }
    }

    return VP_STATUS_SUCCESS;
}

VpStatusType
Vp886ApplyCalDC(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    int16 ila, voc, vas, floor, iHookTh, gndKeyTh;
    int32 diff;

    voc = pLineObj->targetVoc;
    ila = (pLineObj->registers.dcFeed[1] & VP886_R_DCFEED_ILA) + 18;

    /*  Read the loopSup parameters */
    iHookTh = (pLineObj->registers.loopSup[0] & VP886_R_LOOPSUP_HOOK_THRESH) + 7;
    gndKeyTh = ((pLineObj->registers.loopSup[0] & VP886_R_LOOPSUP_GKEY_THRESH) >> 3) * 6;

    /* Correct ILA in normal polarity */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_ILA, ila, VP_POL_NORM);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.normCal, VP886_ERR_ILA, diff, VP_POL_NORM);

    /* Correct ILA in reverse polarity */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_ILA, ila, VP_POL_REV);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.revCal,  VP886_ERR_ILA, diff, VP_POL_REV);

    /* Correct VOC in normal polarity */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_VOC, voc, VP_POL_NORM);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.normCal, VP886_ERR_VOC, diff, VP_POL_NORM);

    /* Correct VOC in reverse polarity */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_VOC, pLineObj->targetVoc, VP_POL_REV);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.revCal,  VP886_ERR_VOC, diff, VP_POL_REV);

    /* Correct Hook threshold in normal polarity */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_HOOK, iHookTh, VP_POL_NORM);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.normCal, VP886_ERR_HOOK, diff, VP_POL_NORM);

    /* Correct Hook threshold in reverse polarity */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_HOOK, iHookTh, VP_POL_REV);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.revCal, VP886_ERR_HOOK, diff, VP_POL_REV);

    /* Correct Hook threshold in low power mode */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_HOOK, iHookTh, VP_POL_NONE);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.indCal, VP886_ERR_HOOK, diff, VP_POL_NONE);

    /* Correct Ground Key threshold */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_GND_KEY, gndKeyTh, VP_POL_NONE);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.indCal, VP886_ERR_GND_KEY, diff, VP_POL_NONE);

    /* Ringing battery cal relies on both DC (VAS) and ringing profiles, so
       apply it when either one changes */
    if (VP886_IS_TRACKER(pDevObj)) {
        Vp886ApplyCalTrackingBat(pLineCtx, pLineObj->registers.normCal, VP_POL_NORM);
        Vp886ApplyCalTrackingBat(pLineCtx, pLineObj->registers.revCal, VP_POL_REV);

        Vp886ApplyCalTrackingBatRinging(pLineCtx, pLineObj->registers.ringCal);
    }

    /* Only for tracker is battery cal based on the DC profile */
    if (VP886_IS_TRACKER(pDevObj)) {
        /* Correct switcher floor voltage */
        floor = pLineObj->floorVoltage;
        diff = Vp886ComputeError(pLineCtx, VP886_ERR_FLOOR, floor, VP_POL_NONE);
        Vp886AdjustCalReg(pLineCtx, pLineObj->registers.batCal,  VP886_ERR_FLOOR, diff, VP_POL_NONE);

        /* Low power switcher voltage should target 4V above VOC */
        /* NOTE: This assumes that ConfigLine is also programming the SWP
           register to the nearest 5V step to 4V above VOC */
        diff = Vp886ComputeError(pLineCtx, VP886_ERR_LOWPOWER, voc + 4, VP_POL_NONE);
        Vp886AdjustCalReg(pLineCtx, pLineObj->registers.batCal,  VP886_ERR_LOWPOWER, diff, VP_POL_NONE);

        /* Battery limit calibration */
        vas = ((pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VAS_MSB) << 2) +
          ((pLineObj->registers.dcFeed[1] & VP886_R_DCFEED_VAS_LSB) >> 6);
        diff = Vp886ComputeError(pLineCtx, VP886_ERR_BAT_SAT, vas, VP_POL_NONE);
        Vp886AdjustCalReg(pLineCtx, pLineObj->registers.indCal, VP886_ERR_BAT_SAT, diff, VP_POL_NONE);
    }

    /* Switcher limit calibration */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_BAT_LIMIT, 0, VP_POL_NONE);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.batCal,  VP886_ERR_BAT_LIMIT, diff, VP_POL_NONE);

    return VP_STATUS_SUCCESS;
}

VpStatusType
Vp886ApplyCalRing(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    uint8 ringGen[VP886_R_RINGGEN_LEN];
    int16 amplitude;
    int32 diff;

    if (VP886_IS_TRACKER(pDevObj)) {
        /* Ringing battery cal relies on both DC (VAS) and ringing profiles, so
           apply it when either one changes */
        Vp886ApplyCalTrackingBatRinging(pLineCtx, pLineObj->registers.ringCal);

        /* If using fixed ringing, apply fixed battery cal */
        if ((pLineObj->registers.swParam[0] & VP886_R_SWPARAM_RING_TRACKING) == VP886_R_SWPARAM_RING_TRACKING_DIS) {
            int16 fixedRingBat;

            fixedRingBat = (pLineObj->registers.swParam[1] & VP886_R_SWPARAM_RINGING_V);
            fixedRingBat = fixedRingBat * 5 + 5;
            pLineObj->fixedRingBat = fixedRingBat;
            diff = Vp886ComputeError(pLineCtx, VP886_ERR_FIXEDRING, fixedRingBat, VP_POL_NONE);
            Vp886AdjustCalReg(pLineCtx, pLineObj->registers.batCal,  VP886_ERR_FIXEDRING, diff, VP_POL_NONE);
        }
    }

    /* Adjust the ringing DC bias using the VOC correction in the RINGCAL register */
    diff = Vp886ComputeError(pLineCtx, VP886_ERR_RINGBIAS, pLineObj->ringBias, VP_POL_NONE);
    Vp886AdjustCalReg(pLineCtx, pLineObj->registers.ringCal,  VP886_ERR_RINGBIAS, diff, VP_POL_NONE);

    /* Adjust the ringing amplitude by reprogramming the ringing generator parameters. */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN, ringGen);
    amplitude = pLineObj->ringAmplitude;
    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Original Ring Amplitude register value %d", amplitude));

    amplitude = Vp886ComputeRingingParam(pCalData, amplitude);
    pLineObj->ringAmplitudeCal = amplitude;
    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Adjusted Ring Amplitude register value %d", amplitude));

    ringGen[5] = (amplitude >> 8) & 0x00FF;
    ringGen[6] = amplitude & 0x00FF;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, ringGen);

    /* In the low ILR range workaround, set ILR correction to +4mA to correct
       for the way the device targets 7/8 of the programmed ILR */
    pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
    if (pLineObj->lowIlr) {
        pLineObj->registers.ringCal[1] |= 0x07;
    }

    return VP_STATUS_SUCCESS;
}

void
Vp886ProgramCalRegisters(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Programming cal registers for ch%d", pLineObj->channelId));

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_INDCAL_WRT, VP886_R_INDCAL_LEN, pLineObj->registers.indCal);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_NORMCAL_WRT, VP886_R_NORMCAL_LEN, pLineObj->registers.normCal);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_REVCAL_WRT, VP886_R_REVCAL_LEN, pLineObj->registers.revCal);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN, pLineObj->registers.ringCal);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_BATCAL_WRT, VP886_R_BATCAL_LEN, pLineObj->registers.batCal);

    pLineObj->calRegsProgrammed = TRUE;

    return;
}

VpStatusType
Vp886GetCalProfile(
    VpLineCtxType *pLineCtx,
    uint8 *calProf)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CalDeviceDataType *pCalData = &pDevObj->calData[channelId];
    uint8 mpiLen;
    uint8* pProf;

    /* Set header info */
    calProf[VP_PROFILE_TYPE_MSB] = VP886_DEVICE_SERIES(pDevObj);
    calProf[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_CAL;
    calProf[VP_PROFILE_INDEX] = 0x00;

    if (VP886_IS_TRACKER(pDevObj)) {
        mpiLen = 0;
        calProf[VP_PROFILE_VERSION] = VP886_CAL_PROF_TRACKER_MAX_VERSION;
        calProf[VP_PROFILE_LENGTH] = VP886_CAL_PROF_TRACKER_LENGTH + 2 + mpiLen;
        calProf[VP_PROFILE_MPI_LEN] = mpiLen;
        pProf = calProf + VP_PROFILE_MPI_LEN + 1 + mpiLen;

        *pProf++ = pCalData->cmn.sadc.gain >> 8;
        *pProf++ = pCalData->cmn.sadc.gain & 0x00FF;
        *pProf++ = pCalData->cmn.sadc.offset >> 8;
        *pProf++ = pCalData->cmn.sadc.offset & 0x00FF;
        *pProf++ = pCalData->cmn.sadcVMode.gain >> 8;
        *pProf++ = pCalData->cmn.sadcVMode.gain & 0x00FF;
        *pProf++ = pCalData->cmn.sadcVMode.offset >> 8;
        *pProf++ = pCalData->cmn.sadcVMode.offset & 0x00FF;
        *pProf++ = pCalData->cmn.swySense.gain >> 8;
        *pProf++ = pCalData->cmn.swySense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swySense.offset >> 8;
        *pProf++ = pCalData->cmn.swySense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.swzSense.gain >> 8;
        *pProf++ = pCalData->cmn.swzSense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swzSense.offset >> 8;
        *pProf++ = pCalData->cmn.swzSense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.io2Sense.gain >> 8;
        *pProf++ = pCalData->cmn.io2Sense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.io2Sense.offset >> 8;
        *pProf++ = pCalData->cmn.io2Sense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.tipSense.gain >> 8;
        *pProf++ = pCalData->cmn.tipSense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.tipSense.offset >> 8;
        *pProf++ = pCalData->cmn.tipSense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ringSense.gain >> 8;
        *pProf++ = pCalData->cmn.ringSense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ringSense.offset >> 8;
        *pProf++ = pCalData->cmn.ringSense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseNormal.gain >> 8;
        *pProf++ = pCalData->cmn.vabSenseNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseNormal.offset >> 8;
        *pProf++ = pCalData->cmn.vabSenseNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseReverse.gain >> 8;
        *pProf++ = pCalData->cmn.vabSenseReverse.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseReverse.offset >> 8;
        *pProf++ = pCalData->cmn.vabSenseReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ilaNormal.gain >> 8;
        *pProf++ = pCalData->cmn.ilaNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ilaNormal.offset >> 8;
        *pProf++ = pCalData->cmn.ilaNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ilaReverse.gain >> 8;
        *pProf++ = pCalData->cmn.ilaReverse.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ilaReverse.offset >> 8;
        *pProf++ = pCalData->cmn.ilaReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocNormal.gain >> 8;
        *pProf++ = pCalData->cmn.vocNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vocNormal.offset >> 8;
        *pProf++ = pCalData->cmn.vocNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocReverse.gain >> 8;
        *pProf++ = pCalData->cmn.vocReverse.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vocReverse.offset >> 8;
        *pProf++ = pCalData->cmn.vocReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocBuffer.gain >> 8;
        *pProf++ = pCalData->cmn.vocBuffer.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vocBuffer.offset >> 8;
        *pProf++ = pCalData->cmn.vocBuffer.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ringingGenerator.gain >> 8;
        *pProf++ = pCalData->cmn.ringingGenerator.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ringingGenerator.offset >> 8;
        *pProf++ = pCalData->cmn.ringingGenerator.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ringingBuffer.gain >> 8;
        *pProf++ = pCalData->cmn.ringingBuffer.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ringingBuffer.offset >> 8;
        *pProf++ = pCalData->cmn.ringingBuffer.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocSenseNormal.gain >> 8;
        *pProf++ = pCalData->cmn.vocSenseNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swLimit100V.offset >> 8;
        *pProf++ = pCalData->cmn.swLimit100V.offset & 0x00FF;
        *pProf++ = pCalData->cmn.fixedBat.gain >> 8;
        *pProf++ = pCalData->cmn.fixedBat.gain & 0x00FF;
        *pProf++ = pCalData->cmn.fixedBat.offset >> 8;
        *pProf++ = pCalData->cmn.fixedBat.offset & 0x00FF;
        *pProf++ = pCalData->spe.trk.reserved_5.gain >> 8;
        *pProf++ = pCalData->spe.trk.reserved_5.gain & 0x00FF;
        *pProf++ = pCalData->spe.trk.reserved_5.offset >> 8;
        *pProf++ = pCalData->spe.trk.reserved_5.offset & 0x00FF;
        *pProf++ = pCalData->spe.trk.reserved_6.gain >> 8;
        *pProf++ = pCalData->spe.trk.reserved_6.gain & 0x00FF;
        *pProf++ = pCalData->spe.trk.reserved_6.offset >> 8;
        *pProf++ = pCalData->spe.trk.reserved_6.offset & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVabNormal.gain >> 8;
        *pProf++ = pCalData->spe.trk.trackerVabNormal.gain & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVabNormal.offset >> 8;
        *pProf++ = pCalData->spe.trk.trackerVabNormal.offset & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVabReverse.gain >> 8;
        *pProf++ = pCalData->spe.trk.trackerVabReverse.gain & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVabReverse.offset >> 8;
        *pProf++ = pCalData->spe.trk.trackerVabReverse.offset & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVasNormal.gain >> 8;
        *pProf++ = pCalData->spe.trk.trackerVasNormal.gain & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVasNormal.offset >> 8;
        *pProf++ = pCalData->spe.trk.trackerVasNormal.offset & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVasReverse.gain >> 8;
        *pProf++ = pCalData->spe.trk.trackerVasReverse.gain & 0x00FF;
        *pProf++ = pCalData->spe.trk.trackerVasReverse.offset >> 8;
        *pProf++ = pCalData->spe.trk.trackerVasReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.batterySense.gain >> 8;
        *pProf++ = pCalData->cmn.batterySense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.longActive.gain >> 8;
        *pProf++ = pCalData->cmn.longActive.gain & 0x00FF;
        *pProf++ = pCalData->cmn.longActive.offset >> 8;
        *pProf++ = pCalData->cmn.longActive.offset & 0x00FF;
        *pProf++ = pCalData->cmn.longRinging.gain >> 8;
        *pProf++ = pCalData->cmn.longRinging.gain & 0x00FF;
        *pProf++ = pCalData->cmn.longRinging.offset >> 8;
        *pProf++ = pCalData->cmn.longRinging.offset & 0x00FF;
        *pProf++ = pCalData->cmn.reserved_1.gain >> 8;
        *pProf++ = pCalData->cmn.reserved_1.gain & 0x00FF;
        *pProf++ = pCalData->cmn.reserved_2.gain >> 8;
        *pProf++ = pCalData->cmn.reserved_2.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseRinging.gain >> 8;
        *pProf++ = pCalData->cmn.vabSenseRinging.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseRinging.offset >> 8;
        *pProf++ = pCalData->cmn.vabSenseRinging.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vadcActive.gain >> 8;
        *pProf++ = pCalData->cmn.vadcActive.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vadcActive.offset >> 8;
        *pProf++ = pCalData->cmn.vadcActive.offset & 0x00FF;
        *pProf++ = (pCalData->cmn.tipCapCal >> 24) & 0x000000FF;
        *pProf++ = (pCalData->cmn.tipCapCal >> 16) & 0x000000FF;
        *pProf++ = (pCalData->cmn.tipCapCal >> 8) & 0x000000FF;
        *pProf++ = pCalData->cmn.tipCapCal & 0x000000FF;
        *pProf++ = pCalData->cmn.hookLPM.offset >> 8;
        *pProf++ = pCalData->cmn.hookLPM.offset & 0x00FF;
        *pProf++ = pCalData->cmn.hookReverse.offset >> 8;
        *pProf++ = pCalData->cmn.hookReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.hookNormal.offset >> 8;
        *pProf++ = pCalData->cmn.hookNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.batSat.offset >> 8;
        *pProf++ = pCalData->cmn.batSat.offset & 0x00FF;
        *pProf++ = pCalData->cmn.swLimit50V.gain >> 8;
        *pProf++ = pCalData->cmn.swLimit50V.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swLimit50V.offset >> 8;
        *pProf++ = pCalData->cmn.swLimit50V.offset & 0x00FF;
        *pProf++ = (pCalData->cmn.ringCapCal >> 24) & 0x000000FF;
        *pProf++ = (pCalData->cmn.ringCapCal >> 16) & 0x000000FF;
        *pProf++ = (pCalData->cmn.ringCapCal >> 8) & 0x000000FF;
        *pProf++ = pCalData->cmn.ringCapCal & 0x000000FF;
        *pProf++ = pCalData->cmn.gndKeyLong.offset >> 8;
        *pProf++ = pCalData->cmn.gndKeyLong.offset & 0x00FF;
    }

    if (VP886_IS_ABS(pDevObj)) {
        mpiLen = 0;
        calProf[VP_PROFILE_VERSION] = VP886_CAL_PROF_ABS_MAX_VERSION;
        calProf[VP_PROFILE_LENGTH] = VP886_CAL_PROF_ABS_LENGTH + 2 + mpiLen;
        calProf[VP_PROFILE_MPI_LEN] = mpiLen;
        pProf = calProf + VP_PROFILE_MPI_LEN + 1 + mpiLen;

        *pProf++ = pCalData->cmn.sadc.gain >> 8;
        *pProf++ = pCalData->cmn.sadc.gain & 0x00FF;
        *pProf++ = pCalData->cmn.sadc.offset >> 8;
        *pProf++ = pCalData->cmn.sadc.offset & 0x00FF;
        *pProf++ = pCalData->cmn.sadcVMode.gain >> 8;
        *pProf++ = pCalData->cmn.sadcVMode.gain & 0x00FF;
        *pProf++ = pCalData->cmn.sadcVMode.offset >> 8;
        *pProf++ = pCalData->cmn.sadcVMode.offset & 0x00FF;
        *pProf++ = pCalData->cmn.swySense.gain >> 8;
        *pProf++ = pCalData->cmn.swySense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swySense.offset >> 8;
        *pProf++ = pCalData->cmn.swySense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.swzSense.gain >> 8;
        *pProf++ = pCalData->cmn.swzSense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swzSense.offset >> 8;
        *pProf++ = pCalData->cmn.swzSense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.io2Sense.gain >> 8;
        *pProf++ = pCalData->cmn.io2Sense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.io2Sense.offset >> 8;
        *pProf++ = pCalData->cmn.io2Sense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.tipSense.gain >> 8;
        *pProf++ = pCalData->cmn.tipSense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.tipSense.offset >> 8;
        *pProf++ = pCalData->cmn.tipSense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ringSense.gain >> 8;
        *pProf++ = pCalData->cmn.ringSense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ringSense.offset >> 8;
        *pProf++ = pCalData->cmn.ringSense.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseNormal.gain >> 8;
        *pProf++ = pCalData->cmn.vabSenseNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseNormal.offset >> 8;
        *pProf++ = pCalData->cmn.vabSenseNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseReverse.gain >> 8;
        *pProf++ = pCalData->cmn.vabSenseReverse.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseReverse.offset >> 8;
        *pProf++ = pCalData->cmn.vabSenseReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ilaNormal.gain >> 8;
        *pProf++ = pCalData->cmn.ilaNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ilaNormal.offset >> 8;
        *pProf++ = pCalData->cmn.ilaNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ilaReverse.gain >> 8;
        *pProf++ = pCalData->cmn.ilaReverse.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ilaReverse.offset >> 8;
        *pProf++ = pCalData->cmn.ilaReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocNormal.gain >> 8;
        *pProf++ = pCalData->cmn.vocNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vocNormal.offset >> 8;
        *pProf++ = pCalData->cmn.vocNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocReverse.gain >> 8;
        *pProf++ = pCalData->cmn.vocReverse.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vocReverse.offset >> 8;
        *pProf++ = pCalData->cmn.vocReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocBuffer.gain >> 8;
        *pProf++ = pCalData->cmn.vocBuffer.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vocBuffer.offset >> 8;
        *pProf++ = pCalData->cmn.vocBuffer.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ringingGenerator.gain >> 8;
        *pProf++ = pCalData->cmn.ringingGenerator.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ringingGenerator.offset >> 8;
        *pProf++ = pCalData->cmn.ringingGenerator.offset & 0x00FF;
        *pProf++ = pCalData->cmn.ringingBuffer.gain >> 8;
        *pProf++ = pCalData->cmn.ringingBuffer.gain & 0x00FF;
        *pProf++ = pCalData->cmn.ringingBuffer.offset >> 8;
        *pProf++ = pCalData->cmn.ringingBuffer.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vocSenseNormal.gain >> 8;
        *pProf++ = pCalData->cmn.vocSenseNormal.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swLimit100V.offset >> 8;
        *pProf++ = pCalData->cmn.swLimit100V.offset & 0x00FF;
        *pProf++ = pCalData->cmn.fixedBat.gain >> 8;
        *pProf++ = pCalData->cmn.fixedBat.gain & 0x00FF;
        *pProf++ = pCalData->cmn.fixedBat.offset >> 8;
        *pProf++ = pCalData->cmn.fixedBat.offset & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_5.gain >> 8;
        *pProf++ = pCalData->spe.abs.reserved_5.gain & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_5.offset >> 8;
        *pProf++ = pCalData->spe.abs.reserved_5.offset & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_6.gain >> 8;
        *pProf++ = pCalData->spe.abs.reserved_6.gain & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_6.offset >> 8;
        *pProf++ = pCalData->spe.abs.reserved_6.offset & 0x00FF;
        *pProf++ = pCalData->spe.abs.absVabNormal.gain >> 8;
        *pProf++ = pCalData->spe.abs.absVabNormal.gain & 0x00FF;
        *pProf++ = pCalData->spe.abs.absVabNormal.offset >> 8;
        *pProf++ = pCalData->spe.abs.absVabNormal.offset & 0x00FF;
        *pProf++ = pCalData->spe.abs.absVabReverse.gain >> 8;
        *pProf++ = pCalData->spe.abs.absVabReverse.gain & 0x00FF;
        *pProf++ = pCalData->spe.abs.absVabReverse.offset >> 8;
        *pProf++ = pCalData->spe.abs.absVabReverse.offset & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_7.gain >> 8;
        *pProf++ = pCalData->spe.abs.reserved_7.gain & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_7.offset >> 8;
        *pProf++ = pCalData->spe.abs.reserved_7.offset & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_8.gain >> 8;
        *pProf++ = pCalData->spe.abs.reserved_8.gain & 0x00FF;
        *pProf++ = pCalData->spe.abs.reserved_8.offset >> 8;
        *pProf++ = pCalData->spe.abs.reserved_8.offset & 0x00FF;
        *pProf++ = pCalData->cmn.batterySense.gain >> 8;
        *pProf++ = pCalData->cmn.batterySense.gain & 0x00FF;
        *pProf++ = pCalData->cmn.longActive.gain >> 8;
        *pProf++ = pCalData->cmn.longActive.gain & 0x00FF;
        *pProf++ = pCalData->cmn.longActive.offset >> 8;
        *pProf++ = pCalData->cmn.longActive.offset & 0x00FF;
        *pProf++ = pCalData->cmn.longRinging.gain >> 8;
        *pProf++ = pCalData->cmn.longRinging.gain & 0x00FF;
        *pProf++ = pCalData->cmn.longRinging.offset >> 8;
        *pProf++ = pCalData->cmn.longRinging.offset & 0x00FF;
        *pProf++ = pCalData->cmn.reserved_1.gain >> 8;
        *pProf++ = pCalData->cmn.reserved_1.gain & 0x00FF;
        *pProf++ = pCalData->cmn.reserved_2.gain >> 8;
        *pProf++ = pCalData->cmn.reserved_2.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseRinging.gain >> 8;
        *pProf++ = pCalData->cmn.vabSenseRinging.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vabSenseRinging.offset >> 8;
        *pProf++ = pCalData->cmn.vabSenseRinging.offset & 0x00FF;
        *pProf++ = pCalData->cmn.vadcActive.gain >> 8;
        *pProf++ = pCalData->cmn.vadcActive.gain & 0x00FF;
        *pProf++ = pCalData->cmn.vadcActive.offset >> 8;
        *pProf++ = pCalData->cmn.vadcActive.offset & 0x00FF;
        *pProf++ = (pCalData->cmn.tipCapCal >> 24) & 0x000000FF;
        *pProf++ = (pCalData->cmn.tipCapCal >> 16) & 0x000000FF;
        *pProf++ = (pCalData->cmn.tipCapCal >> 8) & 0x000000FF;
        *pProf++ = pCalData->cmn.tipCapCal & 0x000000FF;
        *pProf++ = pCalData->cmn.hookLPM.offset >> 8;
        *pProf++ = pCalData->cmn.hookLPM.offset & 0x00FF;
        *pProf++ = pCalData->cmn.hookReverse.offset >> 8;
        *pProf++ = pCalData->cmn.hookReverse.offset & 0x00FF;
        *pProf++ = pCalData->cmn.hookNormal.offset >> 8;
        *pProf++ = pCalData->cmn.hookNormal.offset & 0x00FF;
        *pProf++ = pCalData->cmn.batSat.offset >> 8;
        *pProf++ = pCalData->cmn.batSat.offset & 0x00FF;
        *pProf++ = pCalData->cmn.swLimit50V.gain >> 8;
        *pProf++ = pCalData->cmn.swLimit50V.gain & 0x00FF;
        *pProf++ = pCalData->cmn.swLimit50V.offset >> 8;
        *pProf++ = pCalData->cmn.swLimit50V.offset & 0x00FF;
        *pProf++ = (pCalData->cmn.ringCapCal >> 24) & 0x000000FF;
        *pProf++ = (pCalData->cmn.ringCapCal >> 16) & 0x000000FF;
        *pProf++ = (pCalData->cmn.ringCapCal >> 8) & 0x000000FF;
        *pProf++ = pCalData->cmn.ringCapCal & 0x000000FF;
        *pProf++ = pCalData->cmn.gndKeyLong.offset >> 8;
        *pProf++ = pCalData->cmn.gndKeyLong.offset & 0x00FF;
    }

    return VP_STATUS_SUCCESS;
}

bool
Vp886IsGainError(
    Vp886CalGainOffsetType* gainOffset,
    int16 calStep,
    uint8 channelId,
    bool calCodec)
{
    int16* gain = &gainOffset->gain;
    int16* offset = &gainOffset->offset;
#if (VP_CC_DEBUG_SELECT & VP_DBG_WARNING)
    char *CalCodecSteps[] = {
        "VP886_CAL_CODEC_START",
        "VP886_CAL_CODEC_PREPARE",
        "VP886_CAL_CODEC_SADC",
        "VP886_CAL_CODEC_SWY_SENSE",
        "VP886_CAL_CODEC_SWZ_SENSE",
        "VP886_CAL_CODEC_TIP_SENSE",
        "VP886_CAL_CODEC_RING_SENSE",
        "VP886_CAL_CODEC_IO2_SENSE",
        "VP886_CAL_CODEC_VOC_SENSE",
        "VP886_CAL_CODEC_FLOOR",
        "VP886_CAL_CODEC_BATTERY",
        "VP886_CAL_CODEC_BATTERY_SENSE",
        "VP886_CAL_CODEC_BATTERY_SAT",
        "VP886_CAL_CODEC_BATTERY_LIMIT",
        "VP886_CAL_CODEC_DISCONNECT",
        "VP886_CAL_CODEC_LONGITUDINAL"};
    char *CalLineSteps[] = {
        "VP886_CAL_LINE_START",
        "VP886_CAL_LINE_VADC_ACTIVE",
        "VP886_CAL_LINE_SADC_VMODE",
        "VP886_CAL_LINE_VAB_SENSE",
        "VP886_CAL_LINE_IMT",
        "VP886_CAL_LINE_VOC",
        "VP886_CAL_LINE_RINGING",
        "VP886_CAL_LINE_HOOK_DETECTOR",
        "VP886_CAL_LINE_GROUND_KEY",
        "VP886_CAL_LINE_ABS"};
#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_WARNING) */

    if ((*gain < VP886_MIN_GAIN) || (*gain > VP886_MAX_GAIN)) {
        if (calCodec) {
            VP_WARNING(None, VP_NULL, ("Gain issue (step:%s|ch:%d): gain = %d (/1000), offset = %d",
                CalCodecSteps[calStep], channelId, *gain, *offset));
        } else {
            VP_WARNING(None, VP_NULL, ("Gain issue (step:%s|ch:%d): gain = %d (/1000), offset = %d",
                CalLineSteps[calStep], channelId, *gain, *offset));
        }

        /* Default the gain/offset to a safe value */
        *gain = 1000;
        *offset = 0;
        return TRUE;
    } else {
        return FALSE;
    }
}


VpStatusType
Vp886SetDefaultCal(
    Vp886DeviceObjectType *pDevObj,
    uint8 channelId)
{
    Vp886CalDeviceDataType *pCalData = &pDevObj->calData[channelId];

    /* Senses */
    pCalData->cmn.sadc.gain = 1000;
    pCalData->cmn.sadc.offset = 0;
    pCalData->cmn.sadcVMode.gain = 1000;
    pCalData->cmn.sadcVMode.offset = 0;
    pCalData->cmn.vadcActive.gain = 1000;
    pCalData->cmn.vadcActive.offset = 0;
    pCalData->cmn.swySense.gain = 1000;
    pCalData->cmn.swySense.offset = 0;
    pCalData->cmn.swzSense.gain = 1000;
    pCalData->cmn.swzSense.offset = 0;
    pCalData->cmn.io2Sense.gain = 1000;
    pCalData->cmn.io2Sense.offset = 0;
    pCalData->cmn.tipSense.gain = 1000;
    pCalData->cmn.tipSense.offset = 0;
    pCalData->cmn.ringSense.gain = 1000;
    pCalData->cmn.ringSense.offset = 0;
    pCalData->cmn.vabSenseNormal.gain = 1000;
    pCalData->cmn.vabSenseNormal.offset = 0;
    pCalData->cmn.vabSenseReverse.gain = 1000;
    pCalData->cmn.vabSenseReverse.offset = 0;
    pCalData->cmn.vabSenseRinging.gain = 1000;
    pCalData->cmn.vabSenseRinging.offset = 0;

    /* ILA */
    pCalData->cmn.ilaNormal.gain = 1000;
    pCalData->cmn.ilaNormal.offset = 0;
    pCalData->cmn.ilaReverse.gain = 1000;
    pCalData->cmn.ilaReverse.offset = 0;

    /* VOC */
    pCalData->cmn.vocNormal.gain = 1000;
    pCalData->cmn.vocNormal.offset = 0;
    pCalData->cmn.vocReverse.gain = 1000;
    pCalData->cmn.vocReverse.offset = 0;
    pCalData->cmn.vocBuffer.gain = 1000;
    pCalData->cmn.vocBuffer.offset = 0;

    /* Ringing */
    pCalData->cmn.ringingGenerator.gain = 1000;
    pCalData->cmn.ringingGenerator.offset = 0;
    pCalData->cmn.ringingBuffer.gain = 1000;
    pCalData->cmn.ringingBuffer.offset = 0;

    /* VOC, Ringing shared */
    pCalData->cmn.vocSenseNormal.gain = 1000;
    pCalData->cmn.vocSenseNormal.offset = 0;

    /* Fixed Battery voltage */
    pCalData->cmn.fixedBat.gain = 1000;
    pCalData->cmn.fixedBat.offset = 0;
    pCalData->cmn.batterySense.gain = 1000;
    pCalData->cmn.batterySense.offset = 0;

    if (VP886_IS_TRACKER(pDevObj)) {
        /* Tracking battery */
        pCalData->spe.trk.reserved_5.gain = 1000;
        pCalData->spe.trk.reserved_5.offset = 0;
        pCalData->spe.trk.reserved_6.gain = 1000;
        pCalData->spe.trk.reserved_6.offset = 0;
        pCalData->spe.trk.trackerVabNormal.gain = 1000;
        pCalData->spe.trk.trackerVabNormal.offset = 0;
        pCalData->spe.trk.trackerVabReverse.gain = 1000;
        pCalData->spe.trk.trackerVabReverse.offset = 0;
        pCalData->spe.trk.trackerVasNormal.gain = 1000;
        pCalData->spe.trk.trackerVasNormal.offset = 0;
        pCalData->spe.trk.trackerVasReverse.gain = 1000;
        pCalData->spe.trk.trackerVasReverse.offset = 0;
    } else {    /* ABS */
        /* Battery switch */
        pCalData->spe.abs.reserved_5.gain = 1000;
        pCalData->spe.abs.reserved_5.offset = 0;
        pCalData->spe.abs.reserved_6.gain = 1000;
        pCalData->spe.abs.reserved_6.offset = 0;
        pCalData->spe.abs.absVabNormal.gain = 1000;
        pCalData->spe.abs.absVabNormal.offset = 0;
        pCalData->spe.abs.absVabReverse.gain = 1000;
        pCalData->spe.abs.absVabReverse.offset = 0;
        pCalData->spe.abs.reserved_7.gain = 1000;
        pCalData->spe.abs.reserved_7.offset = 0;
        pCalData->spe.abs.reserved_8.gain = 1000;
        pCalData->spe.abs.reserved_8.offset = 0;
    }

    /* Longitudinal point */
    pCalData->cmn.longActive.gain = 1000;
    pCalData->cmn.longActive.offset = 0;
    pCalData->cmn.longRinging.gain = 1000;
    pCalData->cmn.longRinging.offset = 0;
    pCalData->cmn.reserved_1.gain = 1000;
    pCalData->cmn.reserved_1.offset = 0;
    pCalData->cmn.reserved_2.gain = 1000;
    pCalData->cmn.reserved_2.offset = 0;

    /* Hook Detector */
    pCalData->cmn.hookLPM.offset = 0;
    pCalData->cmn.hookReverse.offset = 0;
    pCalData->cmn.hookNormal.offset = 0;

    /* Battery saturation */
    pCalData->cmn.batSat.offset = 0;

    /* Switcher limit */
    pCalData->cmn.swLimit50V.gain = 1000;
    pCalData->cmn.swLimit50V.offset = 0;
    pCalData->cmn.swLimit100V.gain = 1000;
    pCalData->cmn.swLimit100V.offset = 0;

    /* Ground Key detector */
    pCalData->cmn.gndKeyLong.offset = 0;

    /* Capacitance test tip/ring offsets (pF) */
    pCalData->cmn.tipCapCal = 0;
    pCalData->cmn.ringCapCal = 0;

    return VP_STATUS_SUCCESS;
}

void
Vp886CalCodecHandler(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channel;

    /* PREPARE */
    if (pDevObj->calCodecState[0] == VP886_CAL_CODEC_START) {

        if (pDevObj->calData[0].valid) {
            /* Calibration factors already set */
            if (!(pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
                Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP,
                    0, Vp886GetTimestamp(pDevCtx), FALSE);
            }

            pDevObj->calCodecState[0] = VP886_CAL_CODEC_COMPLETE;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Skipping calibration, calibration factors already set"));
            return;
        }

        pDevObj->busyFlags |= VP_DEV_IN_CAL;

        /* Store the actual ICAL currents in the device object */
        Vp886StoreIcal(pDevCtx);

        /* Initialize the calibration sub-state */
        for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
            pDevObj->calCodecState[channel] = VP886_CAL_CODEC_SADC;
            pDevObj->calCodecSubState[channel] = VP886_SADC_INIT;
            pDevObj->channelCalBack[channel] = TRUE;
            pDevObj->channelLocked[channel] = FALSE;
            pDevObj->calData[channel].valid = FALSE;
        }

        /* Prepare the line(s) for calibration (sets a timer) */
        Vp886PrepareCal(pDevCtx);

        pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;
        pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

    /* CONCLUDE */
    } else if (pDevObj->calCodecState[0] == VP886_CAL_CODEC_CONCLUDE) {
        /* Restore the line(s) after calibration */
        Vp886ConcludeCal(pDevCtx);

        /* If calibration is being run as part of VpInitDevice(), we don't
           generate the CAL_CMP event, because a DEV_INIT_CMP event will
           be generated instead. */
        if (!(pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
            Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP,
                0, Vp886GetTimestamp(pDevCtx), FALSE);
        }

        pDevObj->busyFlags &= ~VP_DEV_IN_CAL;
        pDevObj->calCodecState[0] = VP886_CAL_CODEC_COMPLETE;

    /* IN PROGRESS */
    } else {
        uint8 runChan = 0, queueSize = 0;

        for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
            /* Check which channels need to be precessed */
            if ((pDevObj->channelCalBack[channel]) && !(pDevObj->channelLocked[channel])) {
                if (queueSize++ == 0) {
                    runChan = channel;
                }
            }
        }

        /* Process the unlocked channels, if any (channel are locked if cal can only run on one
           channel at a time) */
        if (queueSize > 0) {
            pDevObj->channelCalBack[runChan] = FALSE;
            Vp886CalCodecSM(pDevCtx, runChan);
            /* Check if another channel needs to be processes (max two channels per device) */
            if ((queueSize > 1) && !(pDevObj->channelLocked[1])) {
                /* The first channel is always processed first if both channels are waiting,
                    if (queueSize > 1), it's always the second channel */
                pDevObj->channelCalBack[1] = FALSE;
                Vp886CalCodecSM(pDevCtx, 1);
            }
        } else {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Unexpected codec calibration error"));
#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
            if (pDevObj->debugSelectMask & VP_DBG_INFO) {
                Vp886ObjectDumpInt(VP_NULL, pDevCtx);
            }
#endif
        }
    }
}

void
Vp886CalCodecSM(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    bool runAnotherState = TRUE;
    bool done = TRUE;
    uint8 channel;

    /* Keep looping throught the main switch until
     * it's time to set a timer and leave */
    while (runAnotherState) {
        runAnotherState = FALSE;

        switch (pDevObj->calCodecState[channelId]) {

            case VP886_CAL_CODEC_SADC:
                runAnotherState = Vp886SadcCalibration(pDevCtx, channelId);
                break;

            case VP886_CAL_CODEC_SWY_SENSE:
                runAnotherState = Vp886SenseCalibration(pDevCtx, channelId, VP886_SENSE_SWY);
                break;

            case VP886_CAL_CODEC_SWZ_SENSE:
                runAnotherState = Vp886SenseCalibration(pDevCtx, channelId, VP886_SENSE_SWZ);
                break;

            case VP886_CAL_CODEC_TIP_SENSE:
                runAnotherState = Vp886SenseCalibration(pDevCtx, channelId, VP886_SENSE_TIP);
                break;

            case VP886_CAL_CODEC_RING_SENSE:
                runAnotherState = Vp886SenseCalibration(pDevCtx, channelId, VP886_SENSE_RING);
                break;

            case VP886_CAL_CODEC_IO2_SENSE:

                /* Only run the IO2 calibration if the pin is defined as a voltage monitor.
                   Also skip for reduced devices with no I/O pins and for the second channel
                   I/Os of single-channel devices. */
                if (((pDevObj->devProfileData.io2Use >> (channelId * 4)) & 0x03) == VP886_DEV_PROFILE_IO2_USE_VMM &&
                    pDevObj->ioCapability != VP886_IO_CAPABILITY_NONE &&
                    channelId < pDevObj->staticInfo.maxChannels)
                {
                    runAnotherState = Vp886SenseCalibration(pDevCtx, channelId, VP886_SENSE_IO2);
                } else {
                    /* Prepare for the next calibration step */
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_VOC_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_VOC_S_INIT;

                    runAnotherState = TRUE;
                }
                break;

            case VP886_CAL_CODEC_VOC_SENSE:
                runAnotherState = Vp886VocSenseCalibration(pDevCtx, channelId);
                break;

            case VP886_CAL_CODEC_FLOOR:
                runAnotherState = Vp886FloorCalibration(pDevCtx, channelId);
                break;

            case VP886_CAL_CODEC_BATTERY:
                if (VP886_IS_TRACKER(pDevObj)) {
                    runAnotherState = Vp886TrackerCalibration(pDevCtx, channelId);
                } else {
                    /* Prepare for the next calibration step */
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_BATTERY_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_BAT_SENSE_INIT;
                    /* Start next calibration in polrev */
                    pDevObj->polarity[channelId] = VP_POLREV;
                    runAnotherState = TRUE;

                }
                break;

            case VP886_CAL_CODEC_BATTERY_SENSE:
                runAnotherState = Vp886BatterySenseCalibration(pDevCtx, channelId);
                break;

            case VP886_CAL_CODEC_BATTERY_SAT:
                if (VP886_IS_TRACKER(pDevObj)) {
                    runAnotherState = Vp886BatterySatCalibration(pDevCtx, channelId);
                } else {
                    /* Prepare for the next calibration step */
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_BATTERY_LIMIT;
                    pDevObj->calCodecSubState[channelId] = VP886_SW_LIM_INIT;
                    runAnotherState = TRUE;
                }
                break;

            case VP886_CAL_CODEC_BATTERY_LIMIT:
                runAnotherState = Vp886SwitcherLimitCalibration(pDevCtx, channelId);
                break;

            case VP886_CAL_CODEC_DISCONNECT:
                if (VP886_IS_ABS(pDevObj) || (pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
                    /* Calibration complete for this channel */
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_SYNC;
                    runAnotherState = TRUE;
                } else {
                    VpLineCtxType *pLineCtxLocal = pDevCtx->pLineCtx[channelId];
                    uint8 calZero[VP886_R_CALCTRL_LEN] = {0, 0, 0};

                    Vp886SetLineStateFxs(pLineCtxLocal, VP_LINE_DISCONNECT);
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calZero);

                    Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_CAL_CODEC, 18, 0, channelId);
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_LONGITUDINAL;
                    pDevObj->calCodecSubState[channelId] = VP886_LONG_INIT;
                }
                break;

            case VP886_CAL_CODEC_LONGITUDINAL:
                runAnotherState = Vp886LongitudinalCalibration(pDevCtx, channelId);
                break;

            case VP886_CAL_CODEC_SYNC:
                for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
                    if (pDevObj->calCodecState[channel] != VP886_CAL_CODEC_SYNC) {
                        done = FALSE;
                    }
                }
                if (done) {
                    pDevObj->calCodecState[0] = VP886_CAL_CODEC_CONCLUDE;
                    Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_CAL_CODEC, 8, 0, channelId);
                }
                break;

            default:
                /* Should never happen. */
                break;
        }
    }
}

void
Vp886CalLineSM(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    bool runAnotherState = TRUE;
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 icr1[VP886_R_ICR1_LEN];
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 slacState[VP886_R_STATE_LEN];

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886CalLineSM+"));

    /* Keep looping throught the main switch until it's time to set a timer and leave */
    while (runAnotherState) {
        runAnotherState = FALSE;

        switch (pLineObj->calLineState) {

            case VP886_CAL_LINE_START:

                /* Do not run the line calibration if the cal data are valid */
                if (pDevObj->calData[channelId].valid) {
                    Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP,
                        0, Vp886GetTimestamp(pDevCtx), FALSE);
                    break;
                }

                if (pLineObj->gndFltProt.state != VP886_GNDFLTPROT_ST_INACTIVE) {
                    Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_STOP);
                }

                pLineObj->busyFlags |= VP886_LINE_IN_CAL;

                if (pLineObj->calRegsProgrammed) {
                    /* Clear cal registers if they were previously programmed */
                    pLineObj->registers.indCal[0] = 0x00;
                    pLineObj->registers.indCal[1] = 0x80;
                    pLineObj->registers.indCal[2] = 0xF0;
                    VpMemSet(pLineObj->registers.normCal, 0, VP886_R_NORMCAL_LEN);
                    VpMemSet(pLineObj->registers.revCal, 0, VP886_R_REVCAL_LEN);
                    VpMemSet(pLineObj->registers.ringCal, 0, VP886_R_RINGCAL_LEN);
                    VpMemSet(pLineObj->registers.batCal, 0, VP886_R_BATCAL_LEN);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_INDCAL_WRT, VP886_R_INDCAL_LEN, pLineObj->registers.indCal);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_NORMCAL_WRT, VP886_R_NORMCAL_LEN, pLineObj->registers.normCal);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_REVCAL_WRT, VP886_R_REVCAL_LEN, pLineObj->registers.revCal);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN, pLineObj->registers.ringCal);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_BATCAL_WRT, VP886_R_BATCAL_LEN, pLineObj->registers.batCal);
                    pLineObj->calRegsProgrammed = FALSE;
                }

                /* Save the original DcFeed register */
                VpSlacRegRead(NULL, pLineCtx, VP886_R_STATE_RD, VP886_R_STATE_LEN,
                    pDevObj->regPad[channelId].slacState);
                VpSlacRegRead(NULL, pLineCtx, VP886_R_ICR1_RD, VP886_R_ICR1_LEN,
                    pDevObj->regPad[channelId].icr1);
                VpSlacRegRead(NULL, pLineCtx, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                    pDevObj->regPad[channelId].icr2);

                /* Mask hook and gkey interrupts */
                pDevObj->registers.intMask[channelId] |= (VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK);
                VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);

                if (VP886_IS_TRACKER(pDevObj)) {
                    VpSlacRegRead(NULL, pLineCtx, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN,
                        pDevObj->regPad[channelId].srp);

                    /* Duplicate the low power floor voltage to the active floor voltage (tracker only) */
                    srp[0] = (pDevObj->regPad[channelId].srp[0] & ~VP886_R_SWPARAM_FLOOR_V) |
                        (pDevObj->regPad[channelId].srp[2] & VP886_R_SWPARAM_LOWPOWER_V);
                    srp[1] = (pDevObj->regPad[channelId].srp[1] & ~VP886_R_SWPARAM_RINGING_V) |
                        (pDevObj->regPad[channelId].srp[2] & VP886_R_SWPARAM_LOWPOWER_V);
                    srp[2] = pDevObj->regPad[channelId].srp[2];
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);
                }

                /* Force fixed battery */
                icr2[0] = pDevObj->regPad[channelId].icr2[0];
                icr2[1] = pDevObj->regPad[channelId].icr2[1];
                icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SW_DAC_CTRL | VP886_R_ICR2_SPEEDUP_MET;
                icr2[3] = pDevObj->regPad[channelId].icr2[3] | VP886_R_ICR2_SW_DAC_CTRL | VP886_R_ICR2_SPEEDUP_MET;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

                /* Force low power mode with 0 bias to eliminate the transients */
                icr1[0] = pDevObj->regPad[channelId].icr1[0] | VP886_R_ICR1_TIP_BIAS | VP886_R_ICR1_SLIC_BIAS;
                icr1[1] = pDevObj->regPad[channelId].icr1[1] & (uint8)(~(VP886_R_ICR1_TIP_BIAS | VP886_R_ICR1_SLIC_BIAS));
                icr1[2] = pDevObj->regPad[channelId].icr1[2] | VP886_R_ICR1_RING_BIAS | VP886_R_ICR1_LPM;
                icr1[3] = (pDevObj->regPad[channelId].icr1[3] & ~(VP886_R_ICR1_RING_BIAS | VP886_R_ICR1_LPM)) |
                    VP886_R_ICR1_LP_STATE;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, icr1);

                /* Go in active, with the codec enabled */
                slacState[0] = VP886_R_STATE_CODEC | VP886_R_STATE_SS_ACTIVE;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

                pLineObj->calLineState = VP886_CAL_LINE_SADC_VMODE_CAL;
                pLineObj->calLineSubState = VP886_SADC_VMODE_INIT;
                Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 48, 0, 0);
                break;

            case VP886_CAL_LINE_SADC_VMODE_CAL:
                runAnotherState = Vp886SadcVModeCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_VADC_ACTIVE_CAL:
                runAnotherState = Vp886VadcActiveCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_VAB_SENSE_CAL:
                runAnotherState = Vp886VabSenseCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_IMT_CAL:
                runAnotherState = Vp886ImtCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_VOC_CAL:
                runAnotherState = Vp886VocCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_RINGING_CAL:
                runAnotherState = Vp886RingingCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_HOOK_DETECTOR_CAL:
                runAnotherState = Vp886SwitchHookCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_GROUND_KEY_CAL:
                runAnotherState = Vp886GndKeyCalibration(pLineCtx);
                break;

            case VP886_CAL_LINE_ABS_CAL:
                if (VP886_IS_ABS(pDevObj)) {
                    runAnotherState = Vp886ABSCalibration(pLineCtx);
                } else {
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN,
                        pDevObj->regPad[channelId].icr1);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
                        pDevObj->regPad[channelId].slacState);
                    /* The device needs some time with metallic speed-up enabled to recover
                       from low power */
                    Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 48, 0, 0);
                    pLineObj->calLineState = VP886_CAL_LINE_RESTORE;
                }
                break;

            case VP886_CAL_LINE_RESTORE:
                if (VP886_IS_TRACKER(pDevObj)) {
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                        pDevObj->regPad[channelId].srp);
                }
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pDevObj->regPad[channelId].icr2);

                /* Unmask hook and gkey interrupts */
                pDevObj->registers.intMask[channelId] &= ~(VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK);
                VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_INTMASK_WRT, VP886_R_INTMASK_LEN, pDevObj->registers.intMask);

                pLineObj->calLineState = VP886_CAL_LINE_GEN_EVENT;
                runAnotherState = TRUE;
                break;

            case VP886_CAL_LINE_GEN_EVENT: {
                uint8 hookBuf[VP886_R_HOOKBUF_LEN];

                pLineObj->busyFlags &= ~VP886_LINE_IN_CAL;


                /* Apply cal */
                Vp886ApplyCalGeneral(pLineCtx);

                /* Apply calibration factors that modify DC profile parameters */
                Vp886ApplyCalDC(pLineCtx);

                /* Apply calibration factors that modify Ring profile parameters */
                Vp886ApplyCalRing(pLineCtx);

                /* Write the new cached values to the registers.  This is done instead
                   of writing to the registers in each of the above functions to
                   reduce traffic to the device. */
                Vp886ProgramCalRegisters(pLineCtx);

                /* Read the hook buffer to clear out any hooks that occurred
                   during calibration */
                VpSlacRegRead(NULL, pLineCtx, VP886_R_HOOKBUF_RD, VP886_R_HOOKBUF_LEN, hookBuf);

                /* Calibration complete on this channel */
                pDevObj->calData[channelId].valid = TRUE;

                /* Prepare the calibration for re-run */
                pLineObj->calLineState = VP886_CAL_LINE_START;

                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP,
                    0, Vp886GetTimestamp(pDevCtx), FALSE);
                break;
            }
            case VP886_CAL_LINE_COMPLETE:
            case VP886_CAL_LINE_FAILED:
                /* Do nothing here.  These are only included in case the state
                   machine is run again after completion / failure. */
                break;

            default:
                /* Should never happen. */
                break;

        }
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886CalLineSM-"));
}

void
Vp886PrepareCal(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 ioDir[VP886_R_IODIR_LEN] = {VP886_R_IODIR_IOD2_VMON};
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    uint8 channel;
    uint8 icr1[VP886_R_ICR1_LEN];

    #if defined(__KERNEL__) && defined(ZARLINK_CFG_INTERNAL)
    VpSysServiceToggleLed(10);
    #endif

    /* Force SLIC in disconnect when an ABS device is running VpCalCodec() */
    if (VP886_IS_ABS(pDevObj)) {
        VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR1_RD, VP886_R_ICR1_LEN, pDevObj->regPad[0].icr1);

        /* Force SLIC in disconnect */
        icr1[0] = VP886_R_ICR1_TIP_BIAS | VP886_R_ICR1_SLIC_BIAS;
        icr1[1] = 0x00;
        icr1[2] = VP886_R_ICR1_RING_BIAS | VP886_R_ICR1_LPM;
        icr1[3] = VP886_R_ICR1_NON_LP_STATE;
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, icr1);
    }

    /* As "all" switchers are disconnected in VP886_INIT_DEVICE_SWITCHER_PREP 
       channel 2 needs to be set to zero for a one channel device */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
    calReset[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;

    for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {

        /* Save ioDir if called from the standalone VpCalCodec */
        if (!(pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
            VpLineCtxType *pLineCtxLocal = pDevCtx->pLineCtx[channel];

            VpSlacRegRead(NULL, pLineCtxLocal, VP886_R_IODIR_RD, VP886_R_IODIR_LEN,
                pDevObj->regPad[channel].ioDir);
        }

        Vp886SetDefaultCal(pDevObj, channel);

        /* Setup the EC channel */
        pDevObj->ecVal = (channel == 0) ? VP886_EC_1 : VP886_EC_2;

        /* Disable the switchers */
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

        /* Force channels to active */
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

        /* Place IO2 pins in V monitor mode, calibration currents get affected by the IO pins... */
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN, ioDir);

        /* Restore the global EC channel */
        pDevObj->ecVal = VP886_EC_GLOBAL;

        /* Next step: SADC calibration */
        pDevObj->calCodecState[channel] = VP886_CAL_CODEC_SADC;
        pDevObj->polarity[channel] = VP_NORMAL;
    }

    /* 10ms for the device to settle */
    Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_CAL_CODEC, 8, 0, 0);
}

void
Vp886ConcludeCal(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_DISCONNECT};
    uint8 convConf[VP886_R_SADC_LEN] = {0x00, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00};
    uint8 calZero[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    uint8 channel;

    #if defined(__KERNEL__) && defined(ZARLINK_CFG_INTERNAL)
    VpSysServiceToggleLed(10);
    #endif

    if (!(pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
        for (channel = 0; channel < pDevObj->staticInfo.maxChannels; channel++) {
            VpLineCtxType *pLineCtxLocal = pDevCtx->pLineCtx[channel];

            /* Restore ioDir if called from the standalone VpCalCodec */
            VpSlacRegWrite(NULL, pLineCtxLocal, VP886_R_IODIR_WRT, VP886_R_IODIR_LEN,
                pDevObj->regPad[channel].ioDir);
        }
    }

    /* Force both channels to disconnect */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

    /* Clear the Supervision Converter Configuration */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);

    /* Restore the SLIC configuration */
    if (VP886_IS_ABS(pDevObj)) {
        VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, pDevObj->regPad[0].icr1);
    }

    /* Re-enable the switchers */
    VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calZero);

    /* Next step: Event generation or other line calibration  */
    pDevObj->calCodecState[0] = VP886_CAL_CODEC_GEN_EVENT;
}

bool
Vp886SadcCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* ICAL(uA) * 10 * (32768 / 74.412u) * -1 */
    int16 expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * -4404L, 1000L);
    int16 expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * -4404L, 1000L);

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_SADC_INIT:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Start ICAL_LOW measurement */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_LOW_CAL_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;

            pDevObj->calCodecSubState[channelId] = VP886_SADC_ICAL_L;
            break;

        case VP886_SADC_ICAL_L:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Measure ICAL_LOW */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

            /* Start ICAL_HIGH measurement */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_HIGH_CAL_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;

            pDevObj->calCodecSubState[channelId] = VP886_SADC_ICAL_H;
            break;

        case VP886_SADC_ICAL_H:
            /* Measure ICAL_HIGH */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

            /* Compute the SADC gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)expHigh - (int32)expLow) * 1000L,
                ((int32)result - (int32)(pDevObj->tmpResultA[channelId])));
            pCalData->sadc.gain = gain;

            /* Compute the SADC offset (offset computed on "int16" scale) */
            offset = (result - (int16)VpRoundedDivide((int32)expHigh * 1000L, (int32)gain)) * -1;
            pCalData->sadc.offset = offset;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886SadcCalibration(ch:%d): gain = %d (/1000), offset = %d (int16 based)",
                channelId, gain, offset));

            Vp886IsGainError(&pCalData->sadc, VP886_CAL_CODEC_SADC, channelId, TRUE);

            /* Prepare for the next calibration step */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_SWY_SENSE;
            pDevObj->calCodecSubState[channelId] = VP886_SENSE_INIT;

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886SenseCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 path)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    uint8 otherChan = pDevObj->staticInfo.maxChannels - (channelId + 1);
    uint8 ecValOther = (otherChan == 0) ? VP886_EC_1 : VP886_EC_2;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 1MOhm sense (2.5x) and a 8x path gain */
    /* ICAL(uA) * 10 * (32768 * ( 2.5 / (8 * 74.412u)) */
    int16 expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * 1376L, 1000L);
    int16 expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * 1376L, 1000L);
    bool runAnotherState = FALSE;
    uint8 icalL[VP886_R_CALCTRL_LEN];
    uint8 icalH[VP886_R_CALCTRL_LEN];
    uint8 calZero[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    uint8 calReset[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC |
        VP886_R_CALCTRL_SWZ_INP_SEL_DISC, 0, 0};
    uint8 adcRoute = VP886_R_SADC_SEL_ADC_OFFSET;

    switch (path) {
        case VP886_SENSE_SWY:
            icalH[0] = VP886_R_CALCTRL_SWZ_INP_SEL_DISC | VP886_R_CALCTRL_SWY_INP_SEL_CAL_H;
            icalH[1] = VP886_R_CALCTRL_SWY_CORR_DIS;
            icalH[2] = 0x00;
            icalL[0] = VP886_R_CALCTRL_SWZ_INP_SEL_DISC | VP886_R_CALCTRL_SWY_INP_SEL_CAL_L;
            icalL[1] = VP886_R_CALCTRL_SWY_CORR_DIS;
            icalL[2] = 0x00;
            adcRoute = VP886_R_SADC_SEL_SWY;
            break;

        case VP886_SENSE_SWZ:
            icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_CAL_H;
            icalH[1] = VP886_R_CALCTRL_SWZ_CORR_DIS;
            icalH[2] = 0x00;
            icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_CAL_L;
            icalL[1] = VP886_R_CALCTRL_SWZ_CORR_DIS;
            icalL[2] = 0x00;
            adcRoute = VP886_R_SADC_SEL_SWZ;
            break;

        case VP886_SENSE_TIP:
            icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
            icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS;
            icalH[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_H;
            icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
            icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS;
            icalL[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_L;
            adcRoute = VP886_R_SADC_SEL_TIP_GROUND_V;
            break;

        case VP886_SENSE_RING:
            icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
            icalH[1] = VP886_R_CALCTRL_RING_CORR_DIS;
            icalH[2] = VP886_R_CALCTRL_RING_INP_SEL_CAL_H;
            icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
            icalL[1] = VP886_R_CALCTRL_RING_CORR_DIS;
            icalL[2] = VP886_R_CALCTRL_RING_INP_SEL_CAL_L;
            adcRoute = VP886_R_SADC_SEL_RING_GROUND_V;
            break;

        case VP886_SENSE_IO2:
            icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
            icalH[1] = VP886_R_CALCTRL_IO2_CORR_DIS;
            icalH[2] = VP886_R_CALCTRL_IO2_INP_SEL_CAL_H;
            icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
            icalL[1] = VP886_R_CALCTRL_IO2_CORR_DIS;
            icalL[2] = VP886_R_CALCTRL_IO2_INP_SEL_CAL_L;
            adcRoute = VP886_R_SADC_SEL_IO2X_V;
            break;

        default:
            break;
    }

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_SENSE_INIT:

            /* The other channel (if any) must be at the same stage to prevent glitchs on the bat */
            if (pDevObj->calCodecState[otherChan] < VP886_CAL_CODEC_SWY_SENSE) {
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            if ((path == VP886_SENSE_SWY) && (pDevObj->staticInfo.maxChannels == 2) &&
                (pDevObj->channelLocked[otherChan] == FALSE)) {

                pDevObj->ecVal = ecValOther;

                /* Free up the cal register to allow the current channel to control the SW senses */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calZero);
                pDevObj->channelLocked[otherChan] = TRUE;
                pDevObj->channelLocked[channelId] = FALSE;
            }

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Start ICAL_LOW measurement */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalL);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_SENSE_ICAL_L;
            break;

        case VP886_SENSE_ICAL_L:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure ICAL_LOW */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;

            /* Start ICAL_HIGH measurement */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalH);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_SENSE_ICAL_H;
            break;

        case VP886_SENSE_ICAL_H:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure ICAL_HIGH */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

            /* Compute the sense path gain including the SADC (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)expHigh - (int32)expLow) * 1000L,
                ((int32)result - (int32)(pDevObj->tmpResultA[channelId])));

            /* Compute the sense path offset including the SADC(offset computed on "int16" scale) */
            offset = (result - (int16)VpRoundedDivide((int32)expHigh * 1000L, (int32)gain)) * -1;

            switch (path) {
                case VP886_SENSE_SWY:
                    pCalData->swySense.gain = gain;
                    pCalData->swySense.offset = offset;
                    Vp886IsGainError(&pCalData->swySense, VP886_CAL_CODEC_SWY_SENSE, channelId, TRUE);
                    /* Bypass SWZ sense calibration for a one channel device */
                    if (pDevObj->staticInfo.maxChannels == 1) {
                        pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_TIP_SENSE;
                    } else {
                        pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_SWZ_SENSE;
                    }
                    pDevObj->calCodecSubState[channelId] = VP886_SENSE_INIT;

                    runAnotherState = TRUE;
                    break;

                case VP886_SENSE_SWZ:
                    pCalData->swzSense.gain = gain;
                    pCalData->swzSense.offset = offset;
                    Vp886IsGainError(&pCalData->swzSense, VP886_CAL_CODEC_SWZ_SENSE, channelId, TRUE);
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_TIP_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_SENSE_INIT;

                    if (pDevObj->calCodecState[otherChan] == VP886_CAL_CODEC_SWY_SENSE) {
                        /* Free up the cal register to allow the current channel to control the SW senses
                           and lock other channel */
                        VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calZero);
                        pDevObj->channelLocked[channelId] = TRUE;
                        pDevObj->channelCalBack[otherChan] = FALSE;
                        Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_CAL_CODEC, 8, 0, otherChan);

                        pDevObj->ecVal = ecValOther;
                        VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
                    } else {
                        /* Switcher senses calibration is done, proceed to the rest */
                        pDevObj->channelCalBack[channelId] = TRUE;
                        pDevObj->channelCalBack[otherChan] = TRUE;

                        pDevObj->ecVal = ecValOther;
                        VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

                        runAnotherState = TRUE;
                    }
                    pDevObj->channelLocked[otherChan] = FALSE;
                    break;

                case VP886_SENSE_TIP:
                    pCalData->tipSense.gain = gain;
                    pCalData->tipSense.offset = offset;
                    Vp886IsGainError(&pCalData->tipSense, VP886_CAL_CODEC_TIP_SENSE, channelId, TRUE);
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_RING_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_SENSE_INIT;

                    runAnotherState = TRUE;
                    break;

                case VP886_SENSE_RING:
                    pCalData->ringSense.gain = gain;
                    pCalData->ringSense.offset = offset;
                    Vp886IsGainError(&pCalData->ringSense, VP886_CAL_CODEC_RING_SENSE, channelId, TRUE);
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_IO2_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_SENSE_INIT;

                    runAnotherState = TRUE;
                    break;

                case VP886_SENSE_IO2:
                    pCalData->io2Sense.gain = gain;
                    pCalData->io2Sense.offset = offset;
                    Vp886IsGainError(&pCalData->io2Sense, VP886_CAL_CODEC_IO2_SENSE, channelId, TRUE);

                    /* Prepare for the next calibration step */
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_VOC_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_VOC_S_INIT;

                    runAnotherState = TRUE;
                    break;

                default:
                    break;
            }

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886SenseCalibration(ch:%d, path:%d): gain = %d (/1000), offset = %d (int16 based)",
                channelId, path, gain, offset));

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886VocSenseCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain;
    bool runAnotherState = FALSE;
    uint8 icalH[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC,
        VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_CAL_H | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 icalL[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC,
        VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_CAL_L | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC |
        VP886_R_CALCTRL_SWZ_INP_SEL_DISC, 0, 0};
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 1MOhm sense (2.5x) and a 4x path gain */
    /* ICAL(uA) * 10 * 32768 * ( 2.5 / (4 * 74.412u)) */
    int16 expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * 2752L, 1000L);
    int16 expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * 2752L, 1000L);

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_VOC_S_INIT:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Start ICAL_LOW measurement */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalL);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_IN,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_VOC_S_ICAL_L;
            break;

        case VP886_VOC_S_ICAL_L:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure ICAL_LOW */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;

            /* Start ICAL_HIGH measurement */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalH);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_IN,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_VOC_S_ICAL_H;
            break;

        case VP886_VOC_S_ICAL_H:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure ICAL_HIGH */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

            /* Compute the VOC sense path gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)pDevObj->tmpResultA[channelId] - (int32)result) * 1000L,
                ((int32)expHigh - (int32)expLow));

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886VocSenseCalibration(ch:%d): gain = %d (/1000)", channelId, gain));

            pCalData->vocSenseNormal.gain = gain;
            pCalData->vocSenseNormal.offset = 0;
            Vp886IsGainError(&pCalData->vocSenseNormal, VP886_CAL_CODEC_VOC_SENSE, channelId, TRUE);

            /* Prepare for the next calibration step */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_FLOOR;
            pDevObj->calCodecSubState[channelId] = VP886_FLOOR_INIT;

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886FloorCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    uint8 adcRoute;
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 icr6[VP886_R_ICR6_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN] = {
        VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC |
        VP886_R_CALCTRL_SWZ_INP_SEL_DISC, 0, 0};
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 402k x 4 path gain  *-1 for the SADC inversion */
    /* FLOOR * 32768 / (402k * 8 * 74.412u) */
    int16 expLow = -1369;
    int16 expHigh = -13693;

    /* Select the proper SADC setting */
    if (ecVal == VP886_EC_1) {
        adcRoute = VP886_R_SADC_SEL_SWY_ERR;
    } else {
        adcRoute = VP886_R_SADC_SEL_SWZ_ERR;
    }

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_FLOOR_INIT:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Disconnect the following senses: BATY, BATZ, TIP, RING */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Save the original registers */
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR6_RD, VP886_R_ICR6_LEN,
                pDevObj->regPad[channelId].icr6);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Force "non-track", disconnect the battery speed-up, unclamp */
            icr2[0] = pDevObj->regPad[channelId].icr2[0];
            icr2[1] = pDevObj->regPad[channelId].icr2[1];
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SW_LIM_150 | VP886_R_ICR2_SPEEDUP_BAT |
                VP886_R_ICR2_SW_DAC_CTRL;
            icr2[3] = pDevObj->regPad[channelId].icr2[3] | VP886_R_ICR2_SW_LIM_150 | VP886_R_ICR2_SPEEDUP_BAT |
                VP886_R_ICR2_SW_DAC_CTRL;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            /* Disable VAS DAC */
            icr6[0] = pDevObj->regPad[channelId].icr6[0] | VP886_R_ICR6_VAS_DAC_EN;
            icr6[1] = pDevObj->regPad[channelId].icr6[1] & ~VP886_R_ICR6_VAS_DAC_EN;
            icr6[2] = pDevObj->regPad[channelId].icr6[2];
            icr6[3] = pDevObj->regPad[channelId].icr6[3];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN, icr6);

            /* No IR overhead */
            dcFeed[0] = pDevObj->regPad[channelId].dcFeed[0] & ~VP886_R_DCFEED_IR_OVERHEAD;
            dcFeed[1] = pDevObj->regPad[channelId].dcFeed[1];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_FLOOR_START;
            runAnotherState = TRUE;
            break;

        case VP886_FLOOR_START:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Program -10V in the Switching Regulator Parameters register */
            srp[0] = (pDevObj->regPad[channelId].srp[0] & ~VP886_R_SWPARAM_FLOOR_V) | 0x01;
            srp[1] = pDevObj->regPad[channelId].srp[1];
            srp[2] = pDevObj->regPad[channelId].srp[2];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);
            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_FLOOR_LOW;
            break;

        case VP886_FLOOR_LOW:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure the floor voltage, expect -10V */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program -100V in the Switching Regulator Parameters register */
            srp[0] = (pDevObj->regPad[channelId].srp[0] & ~VP886_R_SWPARAM_FLOOR_V) | 0x13;
            srp[1] = pDevObj->regPad[channelId].srp[1];
            srp[2] = pDevObj->regPad[channelId].srp[2];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);
            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_FLOOR_HIGH;
            break;

        case VP886_FLOOR_HIGH:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure the floor voltage, expect -100V */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Restore the registers */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN,
                pDevObj->regPad[channelId].icr6);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

            /* Compute the floor voltage generator gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)result - (int32)(pDevObj->tmpResultA[channelId])) * 1000L,
                ((int32)expHigh - (int32)expLow));

            /* Compute the floor voltage generator offset (offset computed on "int16" scale) */
            offset = result - (int16)VpRoundedDivide((int32)gain * (int32)expHigh, 1000L);

            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 8 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)offset * -7303L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886FloorCalibration(ch:%d): gain = %d (/1000), offset = %d mV",
                channelId, gain, offset));

            pCalData->fixedBat.gain = gain;
            pCalData->fixedBat.offset = offset;
            Vp886IsGainError(&pCalData->fixedBat, VP886_CAL_CODEC_FLOOR, channelId, TRUE);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            /* Prepare for the next calibration step, skip the tracker calibration for ABS */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_BATTERY;
            if (VP886_IS_TRACKER(pDevObj)) {
                pDevObj->calCodecSubState[channelId] = VP886_TRACKER_INIT;
            } else {    /*  ABS */
                pDevObj->calCodecSubState[channelId] = VP886_ABS_INIT;
            }
            /* Start next calibration in polrev */
            pDevObj->polarity[channelId] = VP_POLREV;

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886TrackerCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886TrkCalDeviceDataType *pCalData = &pDevObj->calData[channelId].spe.trk;
    int16 result, gain, offset;
    int16 lowMeasurement, highMeasurement;
    bool runAnotherState = FALSE;
    uint8 adcRoute;
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 icr6[VP886_R_ICR6_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 icalL[VP886_R_CALCTRL_LEN];
    uint8 icalH[VP886_R_CALCTRL_LEN];
    uint8 icalDisc[VP886_R_CALCTRL_LEN] = {
        VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 calReset[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC |
        VP886_R_CALCTRL_SWZ_INP_SEL_DISC, 0, 0};
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    int16 expLow;
    int16 expHigh;

    /* Select the proper SADC setting */
    if (ecVal == VP886_EC_1) {
        adcRoute = VP886_R_SADC_SEL_SWY_ERR;
    } else {
        adcRoute = VP886_R_SADC_SEL_SWZ_ERR;
    }

    if (pDevObj->polarity[channelId] == VP_NORMAL) {
        icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
        icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalH[2] = VP886_R_CALCTRL_RING_INP_SEL_CAL_H | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
        icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
        icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalL[2] = VP886_R_CALCTRL_RING_INP_SEL_CAL_L | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
    } else {
        /* pDevObj->polarity[channelId] == VP_POLREV */
        icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
        icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalH[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_H | VP886_R_CALCTRL_RING_INP_SEL_DISC;
        icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
        icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalL[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_L | VP886_R_CALCTRL_RING_INP_SEL_DISC;
        slacState[0] = VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL;
    }

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_TRACKER_INIT:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Save the original registers */
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR6_RD, VP886_R_ICR6_LEN,
                pDevObj->regPad[channelId].icr6);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Enable tracking mode and battery speed-up, unclamp */
            icr2[0] = pDevObj->regPad[channelId].icr2[0];
            icr2[1] = pDevObj->regPad[channelId].icr2[1];
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SW_DAC_CTRL |
                VP886_R_ICR2_SPEEDUP_BAT | VP886_R_ICR2_SW_LIM_150;
            icr2[3] = (pDevObj->regPad[channelId].icr2[3] & ~VP886_R_ICR2_SW_DAC_CTRL) |
                VP886_R_ICR2_SPEEDUP_BAT | VP886_R_ICR2_SW_LIM_150;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            /* Set the floor voltage to the minimum (5.03V) and enable tracking */
            srp[0] = pDevObj->regPad[channelId].srp[0] &
                ~(VP886_R_SWPARAM_FLOOR_V | VP886_R_SWPARAM_RING_TRACKING_DIS);
            srp[1] = pDevObj->regPad[channelId].srp[1];
            srp[2] = pDevObj->regPad[channelId].srp[2];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_TRACKER_VAB_START;
            runAnotherState = TRUE;
            break;

        case VP886_TRACKER_VAB_START:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Program the active state polarity */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Remove the IR overhead voltage drop, force VOC 48V and VAS 9V */
            dcFeed[0] = 0x12;
            dcFeed[1] = (pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_VAS_LSB);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            /* Program the low calibration current (on Tip or Ring according to the polarity) */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalL);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;
            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_TRACKER_VAB_LOW;
            break;

        case VP886_TRACKER_VAB_LOW:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE) {
                /* Ical currents in use, come back later, expect a callback from the other line */
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure SWY or SWZ according to the channel, expect ICAL_LOW x 402kR + 9V */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program the high calibration current (on Tip or Ring according to the polarity) */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalH);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_TRACKER_VAB_HIGH;
            break;

        case VP886_TRACKER_VAB_HIGH:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure SWY or SWZ according to the channel, expect ICAL_HIGH x 402kR + 9V */
            pDevObj->tmpResultB[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Disconnect the VAS generator */
            icr6[0] = pDevObj->regPad[channelId].icr6[0] | 0x20;
            icr6[1] = pDevObj->regPad[channelId].icr6[1] & 0xDF;
            icr6[2] = pDevObj->regPad[channelId].icr6[2];
            icr6[3] = pDevObj->regPad[channelId].icr6[3];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN, icr6);

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_TRACKER_VAS_LOW;
            break;

        case VP886_TRACKER_VAS_LOW:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Remove the base current provided by the VAS, expect VAS only (9V) */
            pDevObj->tmpResultC[channelId] = pDevObj->tmpResultB[channelId] - Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Reconnect the VAS generator */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN, pDevObj->regPad[channelId].icr6);

            /* Disconnect Tip and Ring to measure the only the VAS contribution */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalDisc);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;
            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_TRACKER_VAS_HIGH;
            break;

        case VP886_TRACKER_VAS_HIGH:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure SWY or SWZ according to the channel, expect VAS (9V) */
            pDevObj->tmpResultD[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Remove the IR overhead voltage drop, force VOC 48V and VAS 14V */
            dcFeed[0] = 0x13;
            dcFeed[1] = pDevObj->regPad[channelId].dcFeed[1] | VP886_R_DCFEED_VAS_LSB;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_TRACKER_COMPUTE;
            break;

        case VP886_TRACKER_COMPUTE:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure SWY or SWZ according to the channel, expect VAS (14V) */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Tracker VAB calibration */
            /* Remove the offset introduced by VAS to keep ICALx contribution only */
            lowMeasurement = pDevObj->tmpResultA[channelId] - pDevObj->tmpResultC[channelId];
            highMeasurement = pDevObj->tmpResultB[channelId] - pDevObj->tmpResultC[channelId];

            /* Scale the value from 74.412uA to INT_MAX full scale */
            /* Assuming a 1MOhm sense (2.5x) and a 8x path gain */
            /* ICAL(uA) * 10 * (32768 * ( 2.5 / (8 * 74.412u)) */
            expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * 1376L, 1000L);
            expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * 1376L, 1000L);

            /* Compute the switchers gain related to VAB(actual gain * 1000) */
            gain = (int16)VpRoundedDivide(
                ((int32)highMeasurement -(int32)lowMeasurement) * 1000L,
                ((int32)expHigh - (int32)expLow));

            /* Compute the switchers offset related to VAB (offset computed on "int16" scale) */
            offset = highMeasurement - (int16)VpRoundedDivide((int32)gain * (int32)expHigh, 1000L);

            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 8 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)offset * -7303L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886TrackingCalibration(ch:%d, pol:%d) / VAB: gain = %d (/1000), offset = %d mV",
                channelId, pDevObj->polarity[channelId], gain, offset));

            if (pDevObj->polarity[channelId] == VP_NORMAL) {
                pCalData->trackerVabNormal.gain = gain;
                pCalData->trackerVabNormal.offset = offset;
                Vp886IsGainError(&pCalData->trackerVabNormal, VP886_CAL_CODEC_BATTERY, channelId, TRUE);
            } else {
                /* pDevObj->polarity[channelId] == VP_POLREV */
                pCalData->trackerVabReverse.gain = gain;
                pCalData->trackerVabReverse.offset = offset;
                Vp886IsGainError(&pCalData->trackerVabReverse, VP886_CAL_CODEC_BATTERY, channelId, TRUE);
            }

            /* Tracker VAS calibration */
            lowMeasurement = pDevObj->tmpResultD[channelId];
            highMeasurement = result;

            /* Scale the value from 74.412uA to INT_MAX full scale */
            /* expLow = 32768 * 0.5 * (1.818u + 0.455u * 8) * -1 / 74.412u */
            /* expHigh = 32768 * 0.5 * (1.818u + 0.455u * 15) * -1 / 74.412u */
            expLow = -1202L;
            expHigh = -1903L;

            /* Compute the switchers gain related to VAS (actual gain * 1000), no offset required */
            gain = (int16)VpRoundedDivide(
                ((int32)highMeasurement -(int32)lowMeasurement) * 1000L,
                ((int32)expHigh - (int32)expLow));

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886TrackingCalibration(ch:%d, pol:%d) / VAS: gain = %d (/1000)",
                channelId, pDevObj->polarity[channelId], gain));

            if (pDevObj->polarity[channelId] == VP_NORMAL) {
                pCalData->trackerVasNormal.gain = gain;
                pCalData->trackerVasNormal.offset = 0;
            } else {
                /* pDevObj->polarity[channelId] == VP_POLREV */
                pCalData->trackerVasReverse.gain = gain;
                pCalData->trackerVasReverse.offset = 0;
            }

            switch (pDevObj->polarity[channelId]) {
                case VP_POLREV:
                    pDevObj->polarity[channelId] = VP_NORMAL;
                    pDevObj->calCodecSubState[channelId] = VP886_TRACKER_VAB_START;
                    break;

                case VP_NORMAL:
                    /* Set the calibration control register back to the default */
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

                    /* Restore the original registers */
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                        pDevObj->regPad[channelId].icr2);
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                        pDevObj->regPad[channelId].srp);
                    VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                        pDevObj->regPad[channelId].dcFeed);

                    /* Prepare for the next calibration step */
                    pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_BATTERY_SENSE;
                    pDevObj->calCodecSubState[channelId] = VP886_BAT_SENSE_INIT;
                    break;

                default:
                    break;
            }

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886BatterySenseCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain;
    bool runAnotherState = FALSE;
    uint8 adcRoute;
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 icalL[VP886_R_CALCTRL_LEN];
    uint8 icalH[VP886_R_CALCTRL_LEN];
    uint8 calZero[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    uint8 calReset[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC |
        VP886_R_CALCTRL_SWZ_INP_SEL_DISC, 0, 0};
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    uint8 otherChan = pDevObj->staticInfo.maxChannels - (channelId + 1);
    uint8 ecValOther = (otherChan == 0) ? VP886_EC_1 : VP886_EC_2;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 1MOhm sense (2.5x) and a 8x path gain */
    /* ICAL(uA) * 10 * (32768 * ( 2.5 / (8 * 74.412u)) */
    int16 expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * -1376L, 1000L);
    int16 expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * -1376L, 1000L);

    /* Select the proper settings according to the channel */
    if (ecVal == VP886_EC_1) {
        icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_CAL_H | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
        icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalH[2] = VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
        icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_CAL_L | VP886_R_CALCTRL_SWZ_INP_SEL_DISC;
        icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalL[2] = VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
        adcRoute = VP886_R_SADC_SEL_SWY_ERR;
    } else {
        icalH[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_CAL_H;
        icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalH[2] = VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
        icalL[0] = VP886_R_CALCTRL_SWY_INP_SEL_DISC | VP886_R_CALCTRL_SWZ_INP_SEL_CAL_L;
        icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS |
            VP886_R_CALCTRL_SWY_CORR_DIS | VP886_R_CALCTRL_SWZ_CORR_DIS;
        icalL[2] = VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
        adcRoute = VP886_R_SADC_SEL_SWZ_ERR;
    }

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_BAT_SENSE_INIT:

            /* The other channel (if any) must be at the same stage to prevent glitchs on the bat */
            if (pDevObj->calCodecState[otherChan] < VP886_CAL_CODEC_BATTERY_SENSE) {
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            if ((pDevObj->staticInfo.maxChannels == 2) && (pDevObj->channelLocked[otherChan] == FALSE)) {

                pDevObj->ecVal = ecValOther;

                /* Free up the cal register to allow the current channel to control the SW senses */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calZero);
                pDevObj->channelLocked[otherChan] = TRUE;
                pDevObj->channelLocked[channelId] = FALSE;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Save the original registers */
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Enable tracking mode and battery speed-up, unclamp */
            icr2[0] = pDevObj->regPad[channelId].icr2[0];
            icr2[1] = pDevObj->regPad[channelId].icr2[1];
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SW_DAC_CTRL |
                VP886_R_ICR2_SPEEDUP_BAT | VP886_R_ICR2_SW_LIM_150;
            icr2[3] = pDevObj->regPad[channelId].icr2[3] | VP886_R_ICR2_SW_DAC_CTRL |
                VP886_R_ICR2_SPEEDUP_BAT | VP886_R_ICR2_SW_LIM_150;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            /* Set the floor voltage to the minimum (5.03V) and enable tracking for tracker */
            srp[0] = (pDevObj->regPad[channelId].srp[0] & ~VP886_R_SWPARAM_FLOOR_V) | 0x05;
            srp[1] = pDevObj->regPad[channelId].srp[1];
            srp[2] = pDevObj->regPad[channelId].srp[2];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);

            /* Remove the IR overhead voltage drop */
            dcFeed[0] = pDevObj->regPad[channelId].dcFeed[0] & ~VP886_R_DCFEED_IR_OVERHEAD;
            dcFeed[1] = pDevObj->regPad[channelId].dcFeed[1];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            /* Program the low calibration current (on Tip or Ring according to the polarity) */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalL);

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_BAT_SENSE_LOW;
            break;

        case VP886_BAT_SENSE_LOW:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure SWY or SWZ according to the channel, expect ICAL_LOW x 402kR */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program the high calibration current (on Tip or Ring according to the polarity) */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalH);

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_BAT_SENSE_HIGH;
            break;

        case VP886_BAT_SENSE_HIGH:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure SWY or SWZ according to the channel, expect ICAL_HIGH x 402kR */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Restore the original registers */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

            /* Compute the switcher sense gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)result - (int32)(pDevObj->tmpResultA[channelId])) * 1000L,
                ((int32)expHigh - (int32)expLow));

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886BatterySenseCalibration(ch:%d): gain = %d (/1000)",
                channelId, gain));

            pCalData->batterySense.gain = gain;
            pCalData->batterySense.offset = 0;
            Vp886IsGainError(&pCalData->batterySense, VP886_CAL_CODEC_BATTERY_SENSE, channelId, TRUE);

            /* Prepare for the next calibration step */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_BATTERY_SAT;
            pDevObj->calCodecSubState[channelId] = VP886_BAT_SAT_INIT;

            if (pDevObj->calCodecState[otherChan] == VP886_CAL_CODEC_BATTERY_SENSE) {
                /* Free up the cal register to allow the current channel to control the SW senses
                   and lock other channel */
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calZero);
                pDevObj->channelLocked[channelId] = TRUE;
                pDevObj->channelCalBack[otherChan] = FALSE;
                Vp886AddTimerMs(pDevCtx, NULL, VP886_TIMERID_CAL_CODEC, 8, 0, otherChan);

                pDevObj->ecVal = ecValOther;
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
            } else {
                /* Vp886BatterySenseCalibration() is done, proceed to the rest */
                pDevObj->channelCalBack[channelId] = TRUE;
                pDevObj->channelCalBack[otherChan] = TRUE;
                runAnotherState = TRUE;
            }
            pDevObj->channelLocked[otherChan] = FALSE;

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886BatterySatCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, offset;
    bool runAnotherState = FALSE;
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 calReset[VP886_R_CALCTRL_LEN] = {VP886_R_CALCTRL_SWY_INP_SEL_DISC |
        VP886_R_CALCTRL_SWZ_INP_SEL_DISC,
        VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 402k x 8 path gain  *-1 for the SADC inversion */
    /* VAS(9V) * 0.75 / (74.412u * 402k * 8) * 32768 * (-1 SADC) */
    int16 exp = -924;

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_BAT_SAT_INIT:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Save the original registers */
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

            /* Enable the battery speed-up */
            icr2[0] = pDevObj->regPad[channelId].icr2[0];
            icr2[1] = pDevObj->regPad[channelId].icr2[1];
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SPEEDUP_BAT;
            icr2[3] = pDevObj->regPad[channelId].icr2[3] | VP886_R_ICR2_SPEEDUP_BAT;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            /* Program VAS to 9V */
            dcFeed[0] = (pDevObj->regPad[channelId].dcFeed[0] & ~VP886_R_DCFEED_VAS_MSB) | 0x02;
            dcFeed[1] = (pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_VAS_LSB);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            /* Measure the Battery Detector input voltage */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_BAT_SAT_DET_CMP,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_BAT_SAT_MEAS;
            break;

        case VP886_BAT_SAT_MEAS:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure Battery Detector input voltage, expect 0.75 * VAS(9V) = 6.75V */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            offset = result - exp;

            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 8 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)offset * -7303L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886BatterySatCalibration(ch:%d): offset = %d mV",
                channelId, offset));

            pCalData->batSat.offset = offset;

            /* Restore the original registers */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            /* Prepare for the next calibration step */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_BATTERY_LIMIT;
            pDevObj->calCodecSubState[channelId] = VP886_SW_LIM_INIT;

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886SwitcherLimitCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result;
    bool runAnotherState = FALSE;
    uint8 adcRoute;
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 402k x 4 path gain  *-1 for the SADC inversion */
    /* CLAMP_VLT(51.46V, 102.92V) * 32768 / (402k * 8 * 74.412u) */
    int16 expLow = -7046;
    int16 expHigh = -14093;

    /* Select the proper SADC setting */
    if (ecVal == VP886_EC_1) {
        adcRoute = VP886_R_SADC_SEL_SWY_ERR;
    } else {
        adcRoute = VP886_R_SADC_SEL_SWZ_ERR;
    }

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_SW_LIM_INIT:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Save the original registers */
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);

            /* Force "non-track", disconnect the battery speed-up, clamp at 50V */
            icr2[0] = pDevObj->regPad[channelId].icr2[0];
            icr2[1] = pDevObj->regPad[channelId].icr2[1];
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SW_LIM_150 | VP886_R_ICR2_SPEEDUP_BAT |
                VP886_R_ICR2_SW_DAC_CTRL;
            icr2[3] = (pDevObj->regPad[channelId].icr2[3] & ~VP886_R_ICR2_SW_LIM_150) | VP886_R_ICR2_SW_LIM_50 |
                VP886_R_ICR2_SPEEDUP_BAT | VP886_R_ICR2_SW_DAC_CTRL;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            /* Program -150V in the Switching Regulator Parameters register */
            srp[0] = (pDevObj->regPad[channelId].srp[0] & ~VP886_R_SWPARAM_FLOOR_V) | 0x1D;
            srp[1] = pDevObj->regPad[channelId].srp[1];
            srp[2] = pDevObj->regPad[channelId].srp[2];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_SW_LIM_LOW;
            break;

        case VP886_SW_LIM_LOW:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure the floor voltage, expect -51.46V */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Force "non-track", disconnect the battery speed-up, clamp at 100V */
            icr2[0] = pDevObj->regPad[channelId].icr2[0];
            icr2[1] = pDevObj->regPad[channelId].icr2[1];
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SW_LIM_150 | VP886_R_ICR2_SPEEDUP_BAT |
                VP886_R_ICR2_SW_DAC_CTRL;
            icr2[3] = (pDevObj->regPad[channelId].icr2[3] & ~VP886_R_ICR2_SW_LIM_150) | VP886_R_ICR2_SW_LIM_100 |
                VP886_R_ICR2_SPEEDUP_BAT | VP886_R_ICR2_SW_DAC_CTRL;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            Vp886SetSingleAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_SW_LIM_HIGH;
            break;

        case VP886_SW_LIM_HIGH:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Measure the floor voltage, expect -102.92V */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Restore the registers */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);

            pCalData->swLimit50V.offset =
                (int16)VpRoundedDivide((int32)(pDevObj->tmpResultA[channelId] - expLow) * -7303L, 1000L);

            pCalData->swLimit100V.offset =
                (int16)VpRoundedDivide((int32)(result - expHigh) * -7303L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886SwitcherLimitCalibration(ch:%d): offset(50V) = %d mV, offset(100V) = %d mV",
                channelId, pCalData->swLimit50V.offset, pCalData->swLimit100V.offset));

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            /* Prepare for the next calibration step */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_DISCONNECT;
            /* pDevObj->calCodecSubState[channelId] = ...; */

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

/* The longitudinal point is calibrated at the end of VpInitDevice (switchers on, feed disabled) */
bool
Vp886LongitudinalCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 icr3[VP886_R_ICR3_LEN];
    uint8 icr5[VP886_R_ICR5_LEN];
    uint8 icr6[VP886_R_ICR6_LEN];
    uint8 ringGen[VP886_R_RINGGEN_LEN] = {0x00, 0x00, 0x00, 0x0A, 0xAB, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
    uint8 sysConfig[VP886_R_SSCFG_LEN];
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 adcRoute[5];
    uint8 swRoute;
    int16 result[5] = {0, 0, 0, 0, 0};
    int16 tipRingSum;
    bool runAnotherState = FALSE;
    uint8 ecVal = (channelId == 0) ? VP886_EC_1 : VP886_EC_2;
    uint8 otherChan = pDevObj->staticInfo.maxChannels - (channelId + 1);

    if ((channelId == 0) || VP886_IS_ABS(pDevObj)) {
        swRoute = VP886_R_SADC_SEL_SWY;
    } else {
        swRoute = VP886_R_SADC_SEL_SWZ;
    }

    switch (pDevObj->calCodecSubState[channelId]) {
        case VP886_LONG_INIT:

            /* The other channel (if any) must be at the same stage to prevent glitchs on the bat */
            if (pDevObj->calCodecState[otherChan] < VP886_CAL_CODEC_LONGITUDINAL) {
                pDevObj->channelCalBack[channelId] = TRUE;
                break;
            }

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            /* Save the original registers */
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR3_RD, VP886_R_ICR3_LEN,
                pDevObj->regPad[channelId].icr3);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR5_RD, VP886_R_ICR5_LEN,
                pDevObj->regPad[channelId].icr5);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_ICR6_RD, VP886_R_ICR6_LEN,
                pDevObj->regPad[channelId].icr6);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN,
                pDevObj->regPad[channelId].ringGen);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_SSCFG_RD, VP886_R_SSCFG_LEN,
                pDevObj->regPad[channelId].sysConfig);
            VpSlacRegRead(pDevCtx, NULL, VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);

            /* If ABS, for VBL */
            if (VP886_IS_ABS(pDevObj)) {
                icr3[0] = pDevObj->regPad[channelId].icr3[0];
                icr3[1] = pDevObj->regPad[channelId].icr3[1];
                icr3[2] = pDevObj->regPad[channelId].icr3[2] |
                    VP886_R_ICR3_SWY_TO_LONG | VP886_R_ICR3_SWZ_TO_LONG;
                icr3[3] = (pDevObj->regPad[channelId].icr3[3] &
                    ~VP886_R_ICR3_SWZ_TO_LONG) | VP886_R_ICR3_SWY_TO_LONG;
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, icr3);
            }

            /* Disconnect the VOC DAC */
            icr2[0] = pDevObj->regPad[channelId].icr2[0] | VP886_R_ICR2_VOC_DAC_EN;
            icr2[1] = pDevObj->regPad[channelId].icr2[1] & ~VP886_R_ICR2_VOC_DAC_EN;
            icr2[2] = pDevObj->regPad[channelId].icr2[2] | VP886_R_ICR2_SPEEDUP_MET | VP886_R_ICR2_SW_DAC_CTRL;
            icr2[3] = pDevObj->regPad[channelId].icr2[3] | VP886_R_ICR2_SPEEDUP_MET | VP886_R_ICR2_SW_DAC_CTRL;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

            /* Set speed-up duration 2+2 ms */
            icr5[0] = (pDevObj->regPad[channelId].icr5[0] & 0x00) | 0x22;
            icr5[1] = pDevObj->regPad[channelId].icr5[1];
            icr5[2] = pDevObj->regPad[channelId].icr5[2];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR5_WRT, VP886_R_ICR5_LEN, icr5);

            /* Disconnect the VOC Correction DAC */
            icr6[0] = pDevObj->regPad[channelId].icr6[0] | VP886_R_ICR6_VOC_CORR_DAC_EN;
            icr6[1] = pDevObj->regPad[channelId].icr6[1] & ~VP886_R_ICR6_VOC_CORR_DAC_EN;
            icr6[2] = pDevObj->regPad[channelId].icr6[2];
            icr6[3] = pDevObj->regPad[channelId].icr6[3];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN, icr6);

            /* Disable zero cross */
            sysConfig[0] = pDevObj->regPad[channelId].sysConfig[0] | VP886_R_SSCFG_ZEROCROSS;
            sysConfig[1] = pDevObj->regPad[channelId].sysConfig[1];
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, sysConfig);

            /* If using fixed ringing, change the ringing floor voltage to match the switcher one */
            if (VP886_IS_TRACKER(pDevObj) &&
                ((pDevObj->regPad[channelId].srp[0] & VP886_R_SWPARAM_RING_TRACKING) == VP886_R_SWPARAM_RING_TRACKING_DIS)) {
                srp[0] = pDevObj->regPad[channelId].srp[0];
                srp[1] = (pDevObj->regPad[channelId].srp[1] & ~VP886_R_SWPARAM_RINGING_V) |
                        (pDevObj->regPad[channelId].srp[0] & VP886_R_SWPARAM_FLOOR_V);
                srp[2] = pDevObj->regPad[channelId].srp[2];
                VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, srp);
            }

            /* Program the active state polarity */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            adcRoute[0] = VP886_R_SADC_SEL_TIP_GROUND_V;
            adcRoute[1] = VP886_R_SADC_SEL_RING_GROUND_V;
            adcRoute[2] = swRoute;
            adcRoute[3] = VP886_R_SADC_SEL_LONG_CUR;
            adcRoute[4] = VP886_R_SADC_SEL_ADC_OFFSET;
            Vp886SetGroupAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_EXT_TIME,
                VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_LONG_GET_MEAS;
            break;

        case VP886_LONG_GET_MEAS:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            Vp886GetGroupAdcMath(pDevCtx, channelId, result, TRUE, 4);

            /* If ILG exceeds the limit (2mA / 1069 adu), save a default offset of one step */
            /* Force a fixed offset for RevA silicon (group mode doesn't work correctly) */
            if ((ABS(result[3]) > 1069) || (VP886_REVISION(pDevObj) == VP886_R_RCNPCN_RCN_AAA)) {
                /* 1000 is half way between RevA step size and RevB step size, the apply function
                   will pick the right one */
                pCalData->longActive.offset = 1000;

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886LongitudinalCalibration(ch:%d): feed ILG exceeds 2mA -> fixed 1 step offset",
                    channelId));
            } else {
                tipRingSum = result[0] + result[1];

                /* Compute the longitudinal error (x2) */
                /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 8 * 1000 * (-1 SADC) */
                pCalData->longActive.offset =
                    (int16)VpRoundedDivide((int32)(result[2] - tipRingSum) * -7303L, 1000L);

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886LongitudinalCalibration(ch:%d): feed offset = %d mV",
                    channelId, pCalData->longActive.offset));
            }

            /* Program 0V in the ringing generator */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, ringGen);

            /* Set the SLAC in ringing */
            slacState[0] = VP886_R_STATE_SS_BAL_RING;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            adcRoute[0] = VP886_R_SADC_SEL_TIP_GROUND_V;
            adcRoute[1] = VP886_R_SADC_SEL_RING_GROUND_V;
            adcRoute[2] = swRoute;
            adcRoute[3] = VP886_R_SADC_SEL_LONG_CUR;
            adcRoute[4] = VP886_R_SADC_SEL_ADC_OFFSET;
            Vp886SetGroupAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_EXT_TIME,
                VP886_CAL_INTEGRATE_TIME);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            pDevObj->calCodecSubState[channelId] = VP886_LONG_GET_MEAS_RINGING;
            break;

        case VP886_LONG_GET_MEAS_RINGING:

            /* Setup the EC channel */
            pDevObj->ecVal = ecVal;

            Vp886GetGroupAdcMath(pDevCtx, channelId, result, TRUE, 4);

            /* If ILG exceeds the limit (2mA / 1069 adu), save a default offset of one step */
            /* Force a fixed offset for RevA silicon (group mode doesn't work correctly) */
            if ((ABS(result[3]) > 1069) || (VP886_REVISION(pDevObj) == VP886_R_RCNPCN_RCN_AAA)) {
                /* 1000 is half way between RevA step size and RevB step size, the apply function
                   will pick the right one */
                pCalData->longRinging.offset = 1000;

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886LongitudinalCalibration(ch:%d): ringing ILG exceeds 2mA -> fixed 1 step offset",
                    channelId));
            } else {
                tipRingSum = result[0] + result[1];

                /* Compute the longitudinal error (x2) */
                /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 8 * 1000 * (-1 SADC) */
                pCalData->longRinging.offset =
                    (int16)VpRoundedDivide((int32)(result[2] - tipRingSum) * -7303L, 1000L);

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886LongitudinalCalibration(ch:%d): ringing offset = %d mV",
                    channelId, pCalData->longRinging.offset));
            }

            /* Set the SLAC in disconnect */
            slacState[0] = VP886_R_STATE_SS_DISCONNECT;

            /* Restore the original registers */
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pDevObj->regPad[channelId].icr2);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
                pDevObj->regPad[channelId].icr3);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR5_WRT, VP886_R_ICR5_LEN,
                pDevObj->regPad[channelId].icr5);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN,
                pDevObj->regPad[channelId].icr6);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN,
                pDevObj->regPad[channelId].ringGen);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN,
                pDevObj->regPad[channelId].sysConfig);
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                pDevObj->regPad[channelId].srp);

            /* Restore the global EC channel */
            pDevObj->ecVal = VP886_EC_GLOBAL;

            /* Prepare for the next calibration step */
            pDevObj->calCodecState[channelId] = VP886_CAL_CODEC_SYNC;
            /* pDevObj->calCodecSubState[channelId] = ...; */

            if (!(pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS)) {
                runAnotherState = TRUE;
            }

            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886SadcVModeCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    int16 expLow = 0;
    int16 expHigh = -0x4000; /* -0.5V on the +/-1V scale */

    switch (pLineObj->calLineSubState) {
        case VP886_SADC_VMODE_INIT:
            /* Start VOFF measurement */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_VMODE_OFFSET, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            pLineObj->calLineSubState = VP886_SADC_VMODE_VOFF;
            break;

        case VP886_SADC_VMODE_VOFF:
            /* Measure VOFF (0V) */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

            /* Start VCAL measurement */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_VMODE_REF, VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            pLineObj->calLineSubState = VP886_SADC_VMODE_VCAL;
            break;

        case VP886_SADC_VMODE_VCAL:
            /* Measure VCAL (-0.5V) */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

            /* Compute the SADC gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)expHigh - (int32)expLow) * 1000L,
                ((int32)result - (int32)(pDevObj->tmpResultA[channelId])));
            pCalData->sadcVMode.gain = gain;

            /* Compute the SADC offset (offset computed on "int16" scale) */
            offset = (result - (int16)VpRoundedDivide((int32)expHigh * 1000L, (int32)gain)) * -1;
            pCalData->sadcVMode.offset = offset;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886SadcVModeCalibration(ch:%d): gain = %d (/1000), offset = %d (int16 based)",
                channelId, gain, offset));

            Vp886IsGainError(&pCalData->sadcVMode, VP886_CAL_LINE_SADC_VMODE_CAL, channelId, FALSE);

            /* Prepare for the next calibration step */
            pLineObj->calLineState = VP886_CAL_LINE_VADC_ACTIVE_CAL;
            pLineObj->calLineSubState = VP886_VADC_INIT;
            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886VadcActiveCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* ICAL(uA) * 10 * (32768 / 74.412u) * -1 */
    int16 expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * -4404L, 1000L);
    int16 expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * -4404L, 1000L);
    uint8 opCond, opFunc;

    switch (pLineObj->calLineSubState) {
        case VP886_VADC_INIT:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) {
                /* Ical currents in use, come back later */
                Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 8, 0, 0);
                break;
            }

            /* Save the original Operating Conditions register */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_OPCOND_RD, VP886_R_OPCOND_LEN,
                pDevObj->regPad[channelId].opCond);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_OPFUNC_RD, VP886_R_OPFUNC_LEN,
                pDevObj->regPad[channelId].opFunc);

            /* Make sure that the HPF is disabled */
            opCond = pDevObj->regPad[channelId].opCond[0] | VP886_R_OPCOND_HIGHPASS_DIS;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, &opCond);

            /* Make sure that all of the filters are set to defaults (pass through) */
            opFunc = pDevObj->regPad[channelId].opFunc[0] & ~(VP886_R_OPFUNC_ALL_FILTERS);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPFUNC_WRT, VP886_R_OPFUNC_LEN, &opFunc);

            /* Start ICAL_LOW measurement */
            Vp886SetSingleVadcMath(pDevCtx, channelId, VP886_R_VADC_SEL_LOW_CAL_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;

            /* Set a timer to come back when VADC has collected its data and performed its math */
            Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE,
                VP886_CAL_SETTLE_TIME + VP886_CAL_INTEGRATE_TIME, 0, 0);

            pLineObj->calLineSubState = VP886_VADC_ICAL_L;
            break;
        case VP886_VADC_ICAL_L:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE) {
                /* Ical currents in use, come back later */
                Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 8, 0, 0);
                break;
            }

            /* Measure ICAL_LOW */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleVadcMath(pDevCtx, channelId, FALSE);

            /* Start ICAL_HIGH measurement */
            Vp886SetSingleVadcMath(pDevCtx, channelId, VP886_R_VADC_SEL_HIGH_CAL_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;

            /* Set a timer to come back when VADC has collected its data and performed its math */
            Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE,
                VP886_CAL_SETTLE_TIME + VP886_CAL_INTEGRATE_TIME, 0, 0);

            pLineObj->calLineSubState = VP886_VADC_ICAL_H;
            break;

        case VP886_VADC_ICAL_H: {

            uint8 convConf[VP886_R_VADC_LEN];
            VpMemSet(convConf, 0, sizeof(convConf));

            /* Measure ICAL_HIGH */
            result = Vp886GetSingleVadcMath(pDevCtx, channelId, FALSE);

            /* Compute the VADC gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)expHigh - (int32)expLow) * 1000L,
                ((int32)result - (int32)(pDevObj->tmpResultA[channelId])));
            pCalData->vadcActive.gain = gain * -1;

            /* Compute the VADC offset (offset computed on "int16" scale) */
            offset = result - (int16)VpRoundedDivide((int32)expHigh * 1000L, (int32)gain);
            pCalData->vadcActive.offset = offset;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886VadcCalibration(ch:%d): gain = %d (/1000), offset = %d (int16 based)",
                channelId, gain, offset));

            Vp886IsGainError(&pCalData->vadcActive, VP886_CAL_LINE_VADC_ACTIVE_CAL, channelId, FALSE);

            /*
             * The calibration currents can only be used by the SADC or the VADC
             * but not both. So we have to set the VADC to something other cal currents.
             * I'm just going to zero the whole thing out
             */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, convConf);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

            /* Restore the operatioing conditions and functions values */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPCOND_WRT, VP886_R_OPCOND_LEN, pDevObj->regPad[channelId].opCond);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_OPFUNC_WRT, VP886_R_OPFUNC_LEN, pDevObj->regPad[channelId].opFunc);

            /* Prepare for the next calibration step */
            pLineObj->calLineState = VP886_CAL_LINE_VAB_SENSE_CAL;
            pLineObj->calLineSubState = VP886_VAB_INIT;

            /* Start next calibration in polrev */
            pDevObj->polarity[channelId] = VP_POLREV;

            runAnotherState = TRUE;

            break;
        }

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886VabSenseCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 icalL[VP886_R_CALCTRL_LEN];
    uint8 icalH[VP886_R_CALCTRL_LEN];
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 1MOhm sense (2.5x) and a 8x path gain */
    /* ICAL(uA) * 10 * (32768 * ( 2.5 / (8 * 74.412u)) * -1 */
    int16 expLow = (int16)VpRoundedDivide((int32)(pDevObj->icalL) * -1376L, 1000L);
    int16 expHigh = (int16)VpRoundedDivide((int32)(pDevObj->icalH) * -1376L, 1000L);

    if ((pDevObj->polarity[channelId] == VP_NORMAL) || (pDevObj->polarity[channelId] == VP_RINGING)) {
        icalH[0] = 0;
        icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
        icalH[2] = VP886_R_CALCTRL_RING_INP_SEL_CAL_H | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
        icalL[0] = 0;
        icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
        icalL[2] = VP886_R_CALCTRL_RING_INP_SEL_CAL_L | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
     } else {
        /* pDevObj->polarity[channelId] == VP_POLREV */
        icalH[0] = 0;
        icalH[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
        icalH[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_H | VP886_R_CALCTRL_RING_INP_SEL_DISC;
        icalL[0] = 0;
        icalL[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
        icalL[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_L | VP886_R_CALCTRL_RING_INP_SEL_DISC;
        expLow *= -1;
        expHigh *= -1;
        slacState[0] = VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL;
    }

    switch (pLineObj->calLineSubState) {
        case VP886_VAB_INIT:

            /* Program the active state polarity */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            pLineObj->calLineSubState = VP886_VAB_START;
            runAnotherState = TRUE;
            break;

        case VP886_VAB_START:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) {
                /* Ical currents in use, come back later */
                Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 8, 0, 0);
                break;
            }

            /* Start ICAL_LOW measurement */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalL);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_TIP_RING_DC_V,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_VAB_ICAL_L;
            break;

        case VP886_VAB_ICAL_L:

            if (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE) {
                /* Ical currents in use, come back later */
                Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 8, 0, 0);
                break;
            }

            /* Measure ICAL_LOW */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

            /* Start ICAL_HIGH measurement */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, icalH);
            pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_TIP_RING_DC_V,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_VAB_ICAL_H;
            break;

        case VP886_VAB_ICAL_H:

            /* Measure ICAL_HIGH */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
            pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

            /* Compute the VAB sense path gain including the SADC (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)expHigh - (int32)expLow) * 1000L,
                ((int32)result - (int32)(pDevObj->tmpResultA[channelId])));

            /* Compute the VAB sense path offset + SADC (offset computed on "int16" scale) */
            offset = (result - (int16)VpRoundedDivide((int32)expHigh * 1000L, (int32)gain)) * -1;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886VabSenseCalibration(ch:%d, pol:%d): gain = %d (/1000), offset = %d (int16 based)",
                channelId, pDevObj->polarity[channelId], gain, offset));

            if (pDevObj->polarity[channelId] == VP_POLREV) {
                pCalData->vabSenseReverse.gain = gain;
                pCalData->vabSenseReverse.offset = offset;
                Vp886IsGainError(&pCalData->vabSenseReverse, VP886_CAL_LINE_VAB_SENSE_CAL, channelId, FALSE);
                pDevObj->polarity[channelId] = VP_NORMAL;
                pLineObj->calLineSubState = VP886_VAB_INIT;
             } else if (pDevObj->polarity[channelId] == VP_NORMAL) {
                pCalData->vabSenseNormal.gain = gain;
                pCalData->vabSenseNormal.offset = offset;
                Vp886IsGainError(&pCalData->vabSenseNormal, VP886_CAL_LINE_VAB_SENSE_CAL, channelId, FALSE);

                /* Save the original registers (use ICR3 regPad... ICR2 already used) */
                VpSlacRegRead(NULL, pLineCtx, VP886_R_ICR2_RD, VP886_R_ICR2_LEN,
                    pDevObj->regPad[channelId].icr3);

                /* Backdoor ringing mode*/
                icr2[0] = pDevObj->regPad[channelId].icr3[0] | VP886_R_ICR2_DAC_RING_LEVELS;
                icr2[1] = pDevObj->regPad[channelId].icr3[1] | VP886_R_ICR2_DAC_RING_LEVELS;
                icr2[2] = pDevObj->regPad[channelId].icr3[2];
                icr2[3] = pDevObj->regPad[channelId].icr3[3];
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, icr2);

                pDevObj->polarity[channelId] = VP_RINGING;
                pLineObj->calLineSubState = VP886_VAB_INIT;
             } else {
                /* pDevObj->polarity[channelId] == VP_RINGING */
                pCalData->vabSenseRinging.gain = gain;
                pCalData->vabSenseRinging.offset = offset;

                /* Restore the registers */
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pDevObj->regPad[channelId].icr3);

                /* Prepare for the next calibration step */
                pLineObj->calLineState = VP886_CAL_LINE_IMT_CAL;
                pLineObj->calLineSubState = VP886_IMT_INIT;

                /* Start next calibration in polrev */
                pDevObj->polarity[channelId] = VP_POLREV;
            }

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886ImtCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN] = {
        0,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 200 x 4 path gain and *-1 for the SADC inversion */
    /* IMT * 32768 / (200 * 4 * 74.412u) */
    int16 expLow = -10459;
    int16 expHigh = -26422;

    if (pDevObj->polarity[channelId] == VP_POLREV) {
        expLow *= -1;
        expHigh *= -1;
        slacState[0] = VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL;
    }

    switch (pLineObj->calLineSubState) {
        case VP886_IMT_INIT:

            /* Disconnect the following senses: BATY, BATZ, TIP, RING */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Save the original DcFeed register */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            pLineObj->calLineSubState = VP886_IMT_START;
            runAnotherState = TRUE;
            break;

        case VP886_IMT_START:

            /* Program the active state polarity */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Program 19mA (18 + 1) in the DcFeed register */
            dcFeed[0] = pDevObj->regPad[channelId].dcFeed[0];
            dcFeed[1] = (pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_ILA) | 0x01;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METALLIC_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_IMT_LOW;
            break;

        case VP886_IMT_LOW:

            /* Measure IMT, expect 19mA */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program 48mA (18 + 30) in the DcFeed register */
            dcFeed[0] = pDevObj->regPad[channelId].dcFeed[0];
            dcFeed[1] = (pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_ILA) | 0x1E;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METALLIC_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_IMT_HIGH;
            break;

        case VP886_IMT_HIGH:
            /* Measure IMT, expect 48mA */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Compute the IMT generator gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)result - (int32)(pDevObj->tmpResultA[channelId])) * 1000L,
                ((int32)expHigh - (int32)expLow));

            /* Compute the IMT generator offset (offset computed on "int16" scale) */
            offset = result - (int16)VpRoundedDivide((int32)gain * (int32)expHigh, 1000L);

            /* Convert the offset in uA: offset / 32768 * 74.412u * 200 * 4 * 1e6 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)offset * -1817L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886ImtCalibration(ch:%d, pol:%d): gain = %d (/1000), offset = %d uA",
                channelId, pDevObj->polarity[channelId], gain, offset));

            if (pDevObj->polarity[channelId] == VP_POLREV) {
                pCalData->ilaReverse.gain = gain;
                pCalData->ilaReverse.offset = offset;
                Vp886IsGainError(&pCalData->ilaReverse, VP886_CAL_LINE_IMT_CAL, channelId, FALSE);
                pDevObj->polarity[channelId] = VP_NORMAL;
                pLineObj->calLineSubState = VP886_IMT_START;
             } else {
                /* Set the calibration control register back to the default */
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
                /* Restore the DcFeed register */
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                    pDevObj->regPad[channelId].dcFeed);

                /* pDevObj->polarity[channelId] == VP_NORMAL */
                pCalData->ilaNormal.gain = gain;
                pCalData->ilaNormal.offset = offset;
                Vp886IsGainError(&pCalData->ilaNormal, VP886_CAL_LINE_IMT_CAL, channelId, FALSE);

                /* Prepare for the next calibration step */
                pLineObj->calLineState = VP886_CAL_LINE_VOC_CAL;
                pLineObj->calLineSubState = VP886_VOC_INIT;
                /* Start next calibration in polrev */
                pDevObj->polarity[channelId] = VP_POLREV;
            }

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886VocCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN] = {
        0,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 402k x 4 path gain  *-1 for the SADC inversion */
    /* VOC * 32768 / (402k * 4 * 74.412u) */
    int16 expLow = -3286;
    int16 expHigh = -15610;

    if (pDevObj->polarity[channelId] == VP_POLREV) {
        expLow *= -1;
        expHigh *= -1;
        slacState[0] = VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL;
    }

    switch (pLineObj->calLineSubState) {
        case VP886_VOC_INIT:

            /* Disconnect the following senses: BATY, BATZ, TIP, RING */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Save the original DcFeed register */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            pLineObj->calLineSubState = VP886_VOC_START;
            runAnotherState = TRUE;
            break;

        case VP886_VOC_START:

            /* Program the active state polarity */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Program 12V in the DcFeed register and max out VAS (force LI=0) */
            dcFeed[0] = (pDevObj->regPad[channelId].dcFeed[0] &
                ~(VP886_R_DCFEED_VOCSHIFT | VP886_R_DCFEED_VOC)) | 0x20 | VP886_R_DCFEED_VAS_MSB;
            dcFeed[1] = (pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_LONG_IMPED) |
                VP886_R_DCFEED_VAS_LSB;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_IN,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_VOC_LOW;
            break;

        case VP886_VOC_LOW:

            /* Measure VOC, expect 12V */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program 57V in the DcFeed register and max out VAS (force LI=0) */
            dcFeed[0] = (pDevObj->regPad[channelId].dcFeed[0] &
                ~(VP886_R_DCFEED_VOCSHIFT | VP886_R_DCFEED_VOC)) | 0x1C | VP886_R_DCFEED_VAS_MSB;
            dcFeed[1] = (pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_LONG_IMPED) |
                VP886_R_DCFEED_VAS_LSB;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_IN,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_VOC_HIGH;
            break;

        case VP886_VOC_HIGH:

            /* Measure VOC, expect 57V */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Compute the VOC generator gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)result - (int32)(pDevObj->tmpResultA[channelId])) * 1000L,
                ((int32)expHigh - (int32)expLow));

            /* Compute the VOC generator offset (offset computed on "int16" scale) */
            offset = result - (int16)VpRoundedDivide((int32)gain * (int32)expHigh, 1000L);

            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 4 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)offset * -3652L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886VocCalibration(ch:%d, pol:%d): gain = %d (/1000), offset = %d mV",
                channelId, pDevObj->polarity[channelId], gain, offset));

            if (pDevObj->polarity[channelId] == VP_POLREV) {
                pCalData->vocReverse.gain = gain;
                pCalData->vocReverse.offset = offset;
                Vp886IsGainError(&pCalData->vocReverse, VP886_CAL_LINE_VOC_CAL, channelId, FALSE);
                pDevObj->polarity[channelId] = VP_NORMAL;
                pLineObj->calLineSubState = VP886_VOC_START;
                runAnotherState = TRUE;
             } else {
                /* pDevObj->polarity[channelId] == VP_NORMAL */
                pCalData->vocNormal.gain = gain;
                pCalData->vocNormal.offset = offset;
                Vp886IsGainError(&pCalData->vocNormal, VP886_CAL_LINE_VOC_CAL, channelId, FALSE);

                /* Preapare the 40x ILA driver amplifier offset measurement, disconnect input */
                ical[0] |= VP886_R_CALCTRL_ILA_INP_SEL_DISC;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

                Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_OUT,
                    VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
                pLineObj->calLineSubState = VP886_VOC_BUFFER;
            }
            break;

        case VP886_VOC_BUFFER:

            /* Measure ILA driver offset (expect 0) */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Set the calibration control register back to the default */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
            /* Restore the DcFeed register */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Offset measured at the output, may divide by 40 to have the input offset */
            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 4 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)result * -3652L, 1000L);
            pCalData->vocBuffer.gain = 1000;
            pCalData->vocBuffer.offset = offset;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886VocCalibration(ch:%d): buffer offset output = %d mV, input = %d mV",
                channelId, offset, (int16)VpRoundedDivide(offset, 40)));

            /* Prepare for the next calibration step */
            pLineObj->calLineState = VP886_CAL_LINE_RINGING_CAL;
            pLineObj->calLineSubState = VP886_RINGING_INIT;
            /* Start next calibration in polrev */
            /* pDevObj->polarity[channelId] = VP_POLREV; */

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886RingingCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, gain, offset;
    bool runAnotherState = FALSE;
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_BAL_RING};
    uint8 sysConfig[VP886_R_SSCFG_LEN];
    uint8 ringGen[VP886_R_RINGGEN_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN] = {
        0,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    /* Scale the value from 74.412uA to INT_MAX full scale */
    /* Assuming a 402k x 4 path gain  *-1 for the SADC inversion */
    /* BIAS * 32768 / (402k * 4 * 74.412u) */
    int16 expLow = 0;
    int16 expHigh = -24647;

    switch (pLineObj->calLineSubState) {
        case VP886_RINGING_INIT:

            /* Disconnect the following senses: BATY, BATZ, TIP, RING */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Save the original registers */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_SSCFG_RD, VP886_R_SSCFG_LEN,
                pDevObj->regPad[channelId].sysConfig);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN,
                pDevObj->regPad[channelId].ringGen);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);

            /* Disable the auto state transition (prevent ring trip) */
            sysConfig[0] = pDevObj->regPad[channelId].sysConfig[0] | VP886_R_SSCFG_AUTO_RINGTRIP;
            sysConfig[1] = pDevObj->regPad[channelId].sysConfig[1];
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, sysConfig);

            /* Program a high ringing frequency (1kHz) to enter ringing rapidly */
            ringGen[0] = pDevObj->regPad[channelId].ringGen[0];
            ringGen[1] = pDevObj->regPad[channelId].ringGen[1];
            ringGen[2] = pDevObj->regPad[channelId].ringGen[2];
            ringGen[3] = 0x0A;
            ringGen[4] = 0xAB;
            ringGen[5] = pDevObj->regPad[channelId].ringGen[5];
            ringGen[6] = pDevObj->regPad[channelId].ringGen[6];
            ringGen[7] = pDevObj->regPad[channelId].ringGen[7];
            ringGen[8] = pDevObj->regPad[channelId].ringGen[8];
            ringGen[9] = pDevObj->regPad[channelId].ringGen[9];
            ringGen[10] = pDevObj->regPad[channelId].ringGen[10];
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, ringGen);

            /* Force LI=0 */
            dcFeed[0] = pDevObj->regPad[channelId].dcFeed[0];
            dcFeed[1] = pDevObj->regPad[channelId].dcFeed[1] & ~VP886_R_DCFEED_LONG_IMPED;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            /* Set the SLAC in ringing (requires at least 63ms to enter ringing) */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Need time for the device to enter ringing */
            pLineObj->calLineSubState = VP886_RINGING_START;
            Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 8, 0, 0);
            break;

        case VP886_RINGING_START:

            /* Program 0V in the ringing generator */
            ringGen[0] = 0x00;
            ringGen[1] = 0x00;
            ringGen[2] = 0x00;
            ringGen[3] = 0x0A;
            ringGen[4] = 0xAB;
            ringGen[5] = 0x00;
            ringGen[6] = 0x02;
            ringGen[7] = 0x00;
            ringGen[8] = 0x00;
            ringGen[9] = 0x00;
            ringGen[10] = 0x00;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, ringGen);
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_IN,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_RINGING_LOW;
            break;

        case VP886_RINGING_LOW:

            /* Measure the ringing voltage, expect 0V */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program 90V bias in the ringing generator */
            ringGen[0] = 0x00;
            ringGen[1] = 0x4A;
            ringGen[2] = 0x9D;
            ringGen[3] = 0x0A;
            ringGen[4] = 0xAB;
            ringGen[5] = 0x00;
            ringGen[6] = 0x02;
            ringGen[7] = 0x00;
            ringGen[8] = 0x00;
            ringGen[9] = 0x00;
            ringGen[10] = 0x00;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, ringGen);
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_IN,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_RINGING_HIGH;
            break;

        case VP886_RINGING_HIGH:

            /* Measure the ringing voltage, expect 90V */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Compute the Ringing Generator gain (actual gain * 1000) */
            gain = (int16)VpRoundedDivide(((int32)result - (int32)(pDevObj->tmpResultA[channelId])) * 1000L,
                ((int32)expHigh - (int32)expLow));

            /* Compute the Ringing Generator offset (offset computed on "int16" scale) */
            offset = result - (int16)VpRoundedDivide((int32)gain * (int32)expHigh, 1000L);

            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 4 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)offset * -3652L, 1000L);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886RingingCalibration(ch:%d): gain = %d (/1000), offset = %d mV",
                channelId, gain, offset));

            pCalData->ringingGenerator.gain = gain;
            pCalData->ringingGenerator.offset = offset;
            Vp886IsGainError(&pCalData->ringingGenerator, VP886_CAL_LINE_RINGING_CAL, channelId, FALSE);

            /* Preapare the 40x ILA driver amplifier offset measurement, disconnect input */
            /* NOTE: 40x is the nominal value for LI = 0 (DCFeed Parameter), if LI = 1 -> 80x */
            ical[0] |= VP886_R_CALCTRL_ILA_INP_SEL_DISC;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METTALIC_CAL_OUT,
                VP886_CAL_SETTLE_EXT_TIME, VP886_CAL_INTEGRATE_TIME);
            pLineObj->calLineSubState = VP886_RINGING_BUFFER;
            break;

        case VP886_RINGING_BUFFER:

            /* Measure ILA driver offset in ringing (expect 0) */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Restore the registers */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN,
                pDevObj->regPad[channelId].sysConfig);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN,
                pDevObj->regPad[channelId].ringGen);

            /* Offset measured at the output, may divide by 40 to have the input offset */
            /* Convert the offset in mV: offset / 32768 * 74.412u * 402k * 4 * 1000 * (-1 SADC) */
            offset = (int16)VpRoundedDivide((int32)result * -3652L, 1000L);
            pCalData->ringingBuffer.gain = 1000;
            pCalData->ringingBuffer.offset = offset;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886RingingCalibration(ch:%d): buffer offset output = %d mV, input = %d mV",
                channelId, offset, (int16)VpRoundedDivide(offset, 40)));

            /* Prepare for the next calibration step */
            pLineObj->calLineState = VP886_CAL_LINE_HOOK_DETECTOR_CAL;
            pLineObj->calLineSubState = VP886_HOOK_DET_INIT;

            /* Need time for the device to exit ringing */
            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886SwitchHookCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, offset;
    bool runAnotherState = FALSE;
    uint8 loopSup[VP886_R_LOOPSUP_LEN];
    uint8 sysConfig[VP886_R_SSCFG_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN] = {
        VP886_R_CALCTRL_ILA_INP_SEL_DISC,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    uint8 *slacState = (uint8*)&pDevObj->tmpResultB[channelId];

    switch (pLineObj->calLineSubState) {
        case VP886_HOOK_DET_INIT:

            /* Save the original registers */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_LOOPSUP_RD, VP886_R_LOOPSUP_LEN,
                pDevObj->regPad[channelId].loopSup);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_SSCFG_RD, VP886_R_SSCFG_LEN,
                pDevObj->regPad[channelId].sysConfig);

            /* Setup the first state */
            slacState[0] = VP886_R_STATE_SS_LOWPOWER;

            /* Disconnect the following senses: BATY, BATZ, TIP, RING, ILA */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Disable the auto state transition in LPM */
            sysConfig[0] = pDevObj->regPad[channelId].sysConfig[0];
            sysConfig[1] = pDevObj->regPad[channelId].sysConfig[1] | VP886_R_SSCFG_AUTO_SYSSTATE_LPM_DIS;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, sysConfig);

            /* Disable the hook hysteresis for both LP and normal states */
            /* Program an average threshold (Active: 10mA, LPM: 20V) */
            loopSup[0] = (pDevObj->regPad[channelId].loopSup[0] & ~VP886_R_LOOPSUP_HOOK_THRESH) | 0x03;
            loopSup[1] = pDevObj->regPad[channelId].loopSup[1];
            loopSup[2] = pDevObj->regPad[channelId].loopSup[2];
            loopSup[3] = (pDevObj->regPad[channelId].loopSup[3] & ~VP886_R_LOOPSUP_LPM_HOOK_THRESH) | 0x60;
            loopSup[4] = pDevObj->regPad[channelId].loopSup[4] &
                ~(VP886_R_LOOPSUP_LPM_HOOK_HYST | VP886_R_LOOPSUP_HOOK_HYST);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, loopSup);

            pLineObj->calLineSubState = VP886_HOOK_DET_METALIC;
            runAnotherState = TRUE;
            break;

        case VP886_HOOK_DET_METALIC:

            /* Set the proper line state */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Measure signal before the detector */
            if (slacState[0] == VP886_R_STATE_SS_LOWPOWER) {
                Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_TIP_RING_DC_V,
                    VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            } else {
                Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_METALLIC_CUR,
                    VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);
            }

            pLineObj->calLineSubState = VP886_HOOK_DET_COMPARATOR;
            break;

        case VP886_HOOK_DET_COMPARATOR:

            /* Measure the switch hook current/voltage, expect 10mA/20V */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_HOOK_DET_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_HOOK_DET_COMPUTE;
            break;

        case VP886_HOOK_DET_COMPUTE:

            /* Measure the switch hook current/voltage, expect 0mA/0V */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            if (slacState[0] == VP886_R_STATE_SS_LOWPOWER) {
                /* Compute the actual hook threshold */
                result = pDevObj->tmpResultA[channelId] / 2 - result;

                /* Convert the offset in mV: offset / 32768 * 74.412u * 200 * 4 * 1e6 * 2.02 */
                offset = (int16)VpRoundedDivide((int32)result * -3670L, 1000L);

                /* Compute the difference to the target (20V) */
                offset = offset - 20000;
            } else {
                /* Compute the actual hook threshold */
                result = pDevObj->tmpResultA[channelId] - result;

                /* Convert the offset in uA: offset / 32768 * 74.412u * 200 * 4 * 1e6 */
                offset = (int16)VpRoundedDivide((int32)result * -1817L, 1000L);

                /* Compute the difference to the target (10mA) */
                offset = offset - 10000;
            }

            if (slacState[0] == VP886_R_STATE_SS_LOWPOWER) {

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886SwitchHookCalibration(ch:%d): Low power, offset = %d mV",
                    channelId, offset));

                pCalData->hookLPM.offset = offset;

                slacState[0] = VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL;
                pLineObj->calLineSubState = VP886_HOOK_DET_METALIC;
            } else if (slacState[0] == (VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL)) {

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886SwitchHookCalibration(ch:%d): Active polrev, offset = %d uA",
                    channelId, offset));

                pCalData->hookReverse.offset = offset;

                slacState[0] = VP886_R_STATE_SS_ACTIVE;
                pLineObj->calLineSubState = VP886_HOOK_DET_METALIC;
            } else if (slacState[0] == VP886_R_STATE_SS_ACTIVE) {

                VP_CALIBRATION(VpDevCtxType, pDevCtx,
                    ("Vp886SwitchHookCalibration(ch:%d): Active normal, offset = %d uA",
                    channelId, offset));

                pCalData->hookNormal.offset = offset;

                /* Restore the original registers */
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN,
                    pDevObj->regPad[channelId].loopSup);
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN,
                    pDevObj->regPad[channelId].sysConfig);
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

                /* Prepare for the next calibration step */
                pLineObj->calLineState = VP886_CAL_LINE_GROUND_KEY_CAL;
                pLineObj->calLineSubState = VP886_GND_KEY_INIT;
            }

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

bool
Vp886GndKeyCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int16 result, offset;
    bool runAnotherState = FALSE;
    uint8 loopSup[VP886_R_LOOPSUP_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN] = {
        VP886_R_CALCTRL_ILG_INP_SEL_DISC,
        VP886_R_CALCTRL_RING_CORR_DIS | VP886_R_CALCTRL_TIP_CORR_DIS,
        VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC
    };
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};

    switch (pLineObj->calLineSubState) {
        case VP886_GND_KEY_INIT:

            /* Save the original registers */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_SSCFG_RD, VP886_R_SSCFG_LEN,
                pDevObj->regPad[channelId].sysConfig);

            /* Disconnect the following senses: BATY, BATZ, TIP, RING, ILG */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Disable the ground key hysteresis, force ABSGK */
            /* Program an average threshold (24mA) */
            loopSup[0] = (pDevObj->regPad[channelId].loopSup[0] &
                ~VP886_R_LOOPSUP_GKEY_ABS & ~VP886_R_LOOPSUP_GKEY_THRESH) | 0xA0;
            loopSup[1] = pDevObj->regPad[channelId].loopSup[1];
            loopSup[2] = pDevObj->regPad[channelId].loopSup[2];
            loopSup[3] = pDevObj->regPad[channelId].loopSup[3];
            loopSup[4] = pDevObj->regPad[channelId].loopSup[4] & ~VP886_R_LOOPSUP_GKEY_HYST;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, loopSup);

            pLineObj->calLineSubState = VP886_GND_KEY_LONG;
            Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 18, 0, 0);
            break;

        case VP886_GND_KEY_LONG:

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_LONG_CUR,
                VP886_CAL_SETTLE_EXT_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_GND_KEY_COMPARATOR;
            break;

        case VP886_GND_KEY_COMPARATOR:

            /* Measure longitudinal current */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_GNDKEY_CUR,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_GND_KEY_COMPUTE;
            break;

        case VP886_GND_KEY_COMPUTE:

            /* Measure the ground key current, expect 0mA */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Compute the difference to the target (24mA) */
            /* Scale the hook current (uA), artificially doubled to match the specs*/
            /* Convert the offset in uA: offset / 32768 * 74.412u * 200 * 4 * 1e6 * 2 */
            offset = VpRoundedDivide((ABS(pDevObj->tmpResultA[channelId]) + ABS(result)) * 3634L, 1000L) - 24000L;

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886GndKeyCalibration(ch:%d): offset = %d uA", channelId, offset));

            pCalData->gndKeyLong.offset = offset;
            /* effectively removing gkey calibration, no longer needed */
            pCalData->gndKeyLong.offset = 0;

            /* Restore the original registers */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN,
                pDevObj->regPad[channelId].loopSup);
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

            /* Prepare for the next calibration step */
            pLineObj->calLineState = VP886_CAL_LINE_ABS_CAL;
            pLineObj->calLineSubState = VP886_ABS_INIT;

            /* Start next calibration in polrev */
            pDevObj->polarity[channelId] = VP_POLREV;

            runAnotherState = TRUE;
            break;

        default:
            break;
    }

    return runAnotherState;
}

/* The battery switch point is calibrated during VpCalLine() */
bool
Vp886ABSCalibration(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886AbsCalDeviceDataType *pCalData = &pDevObj->calData[channelId].spe.abs;
    int16 result;
    bool runAnotherState = FALSE;
    uint8 icr6[VP886_R_ICR6_LEN];
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 ical[VP886_R_CALCTRL_LEN];
    uint8 normCal[VP886_R_NORMCAL_LEN];
    uint8 revCal[VP886_R_REVCAL_LEN];
    uint8 slacState[VP886_R_STATE_LEN] = {VP886_R_STATE_SS_ACTIVE};
    uint8 calReset[VP886_R_CALCTRL_LEN] = {0, 0, 0};
    int32 target, lowValue, highValue, zeroCross;
    int32 expLow = -5628;
    int32 expHigh = 5628;

    if (pDevObj->polarity[channelId] == VP_NORMAL) {
        ical[0] = 0x00;
        ical[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
        ical[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_L | VP886_R_CALCTRL_RING_INP_SEL_CAL_H;
    } else {
        /* pDevObj->polarity[channelId] == VP_POLREV */
        ical[0] = 0x00;
        ical[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
        ical[2] = VP886_R_CALCTRL_TIP_INP_SEL_CAL_H | VP886_R_CALCTRL_RING_INP_SEL_CAL_L;
        slacState[0] = VP886_R_STATE_SS_ACTIVE | VP886_R_STATE_POL;
    }

    switch (pLineObj->calLineSubState) {
        case VP886_ABS_INIT:

            /* Save the original registers */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_ICR6_RD, VP886_R_ICR6_LEN,
                pDevObj->regPad[channelId].icr6);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN,
                pDevObj->regPad[channelId].dcFeed);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_NORMCAL_RD, VP886_R_NORMCAL_LEN,
                pDevObj->regPad[channelId].normCal);
            VpSlacRegRead(NULL, pLineCtx, VP886_R_REVCAL_RD, VP886_R_REVCAL_LEN,
                pDevObj->regPad[channelId].revCal);

            /* Program the active state polarity */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Disconnect the VAS generator */
            icr6[0] = pDevObj->regPad[channelId].icr6[0] | VP886_R_ICR6_VAS_DAC_EN;
            icr6[1] = pDevObj->regPad[channelId].icr6[1] & ~VP886_R_ICR6_VAS_DAC_EN;
            icr6[2] = pDevObj->regPad[channelId].icr6[2];
            icr6[3] = pDevObj->regPad[channelId].icr6[3];
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN, icr6);

            /* Remove the IR overhead voltage drop */
            dcFeed[0] = pDevObj->regPad[channelId].dcFeed[0] & ~VP886_R_DCFEED_IR_OVERHEAD;
            dcFeed[1] = pDevObj->regPad[channelId].dcFeed[1];
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN, dcFeed);

            pLineObj->calLineSubState = VP886_ABS_SWY_START;
            runAnotherState = TRUE;
            break;

        case VP886_ABS_SWY_START:

            /* Measure SWY voltage */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_SWY,
                VP886_CAL_SETTLE_EXT_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_ABS_SWY;
            break;

        case VP886_ABS_SWY:

            /* Measure SWY voltage */
            pDevObj->tmpResultA[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            pLineObj->calLineSubState = VP886_ABS_CHECK_ICAL;
            runAnotherState = TRUE;
            break;

        case VP886_ABS_CHECK_ICAL:

            if ((pDevObj->stateInt & VP886_DEVICE_ICAL_L_IN_USE) || (pDevObj->stateInt & VP886_DEVICE_ICAL_H_IN_USE)) {
                /* Ical currents in use, come back later */
                Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 8, 0, 0);
            } else {
                /* Set the Ical in use now and proceed */
                pDevObj->stateInt |= VP886_DEVICE_ICAL_L_IN_USE;
                pDevObj->stateInt |= VP886_DEVICE_ICAL_H_IN_USE;
                pLineObj->calLineSubState = VP886_ABS_BSW_START;
                runAnotherState = TRUE;
            }
            break;

        case VP886_ABS_BSW_START:

            /* Program the required state polarity */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, slacState);

            /* Simulate a feed voltage with the ICAL curents */
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, ical);

            /* Program the low point */
            if (pDevObj->polarity[channelId] == VP_NORMAL) {
                normCal[0] = pDevObj->regPad[channelId].normCal[0];
                normCal[1] = pDevObj->regPad[channelId].normCal[1];
                normCal[2] = pDevObj->regPad[channelId].normCal[2] | 0xF0;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_NORMCAL_WRT, VP886_R_NORMCAL_LEN, normCal);
            } else {
                /* pDevObj->polarity[channelId] == VP_POLREV */
                revCal[0] = pDevObj->regPad[channelId].revCal[0];
                revCal[1] = pDevObj->regPad[channelId].revCal[1];
                revCal[2] = pDevObj->regPad[channelId].revCal[2] | 0xF0;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_REVCAL_WRT, VP886_R_REVCAL_LEN, revCal);
            }

            /* Measure Battery switch comparator input voltage */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_SW_CAL_CMP_HIGH,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_ABS_BSW_LOW;
            break;

        case VP886_ABS_BSW_LOW:

            /* Measure Battery switch comparator input voltage, expect low */
            pDevObj->tmpResultB[channelId] = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Program the low point */
            if (pDevObj->polarity[channelId] == VP_NORMAL) {
                normCal[0] = pDevObj->regPad[channelId].normCal[0];
                normCal[1] = pDevObj->regPad[channelId].normCal[1];
                normCal[2] = (pDevObj->regPad[channelId].normCal[2] & 0x0F) | 0x70;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_NORMCAL_WRT, VP886_R_NORMCAL_LEN, normCal);
            } else {
                /* pDevObj->polarity[channelId] == VP_POLREV */
                revCal[0] = pDevObj->regPad[channelId].revCal[0];
                revCal[1] = pDevObj->regPad[channelId].revCal[1];
                revCal[2] = (pDevObj->regPad[channelId].revCal[2] & 0x0F) | 0x70;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_REVCAL_WRT, VP886_R_REVCAL_LEN, revCal);
            }

            /* Measure Battery switch comparator input voltage */
            Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_SW_CAL_CMP_HIGH,
                VP886_CAL_SETTLE_TIME, VP886_CAL_INTEGRATE_TIME);

            pLineObj->calLineSubState = VP886_ABS_BSW_HIGH;
            break;

        case VP886_ABS_BSW_HIGH:

            /* Measure Battery switch comparator input voltage, expect high */
            result = Vp886GetSingleAdcMath(pDevCtx, channelId, TRUE);

            /* Scale the value in mV */
            /* (IcalH(uA) - IcalL(uA)) * 402k * 2.5 */
            target = VpRoundedDivide((int32)(pDevObj->icalH - pDevObj->icalL) * -1005, 100);

            /* target = vab(mV) - swy(mV) */
            /* swy(mV) = swy  / 32768 * 74.412u * 402k * 8 * 1000 * (-1 SADC)*/
            target -= VpRoundedDivide((int32)pDevObj->tmpResultA[channelId] * -7303L, 1000L);

            /* Compute zero crossing in mV*/
            lowValue = VpRoundedDivide((int32)pDevObj->tmpResultB[channelId] * 7303L, 1000L);
            highValue = VpRoundedDivide((int32)result * 7303L, 1000L);
            zeroCross = (expLow * highValue - expHigh * lowValue) / (expLow - expHigh);

            VP_CALIBRATION(VpDevCtxType, pDevCtx,
                ("Vp886ABSCalibration(ch:%d|pol:%d): offset = %li mV",
                channelId, pDevObj->polarity[channelId], target - zeroCross));

            switch (pDevObj->polarity[channelId]) {
                case VP_POLREV:
                    pCalData->absVabReverse.offset = (int16)(target - zeroCross);

                    pDevObj->polarity[channelId] = VP_NORMAL;
                    pLineObj->calLineSubState = VP886_ABS_BSW_START;

                    runAnotherState = TRUE;
                    break;

                case VP_NORMAL:
                    pCalData->absVabNormal.offset = (int16)(target - zeroCross);

                    /* Set the calibration control register back to the default */
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calReset);

                    /* Release Ical */
                    pDevObj->stateInt &= ~VP886_DEVICE_ICAL_L_IN_USE;
                    pDevObj->stateInt &= ~VP886_DEVICE_ICAL_H_IN_USE;

                    /* Restore the original registers */
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN,
                        pDevObj->regPad[channelId].icr1);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR6_WRT, VP886_R_ICR6_LEN,
                        pDevObj->regPad[channelId].icr6);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_DCFEED_WRT, VP886_R_DCFEED_LEN,
                        pDevObj->regPad[channelId].dcFeed);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
                        pDevObj->regPad[channelId].slacState);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_NORMCAL_WRT, VP886_R_NORMCAL_LEN,
                        pDevObj->regPad[channelId].normCal);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_REVCAL_WRT, VP886_R_REVCAL_LEN,
                        pDevObj->regPad[channelId].revCal);


                    /* The device needs some time with metallic speed-up enabled to recover
                       form low power */
                    Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_CAL_LINE, 38, 0, 0);
                    pLineObj->calLineState = VP886_CAL_LINE_RESTORE;
                    /* pLineObj->calLineSubState = ; */
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return runAnotherState;
}

void
Vp886ApplyCalTrackingBat(
    VpLineCtxType *pLineCtx,
    uint8 *polCal,
    uint8 polarity)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CalDeviceDataType *pCalData = &pDevObj->calData[channelId];
    int32 voc;
    int32 vas;
    int32 ila;
    int16 irOverheadArr[4] = {0, 50, 100, 200};
    int16 irOverhead;
    int32 vab;
    int32 target;
    int32 calculatedBat;
    int32 adjustedBat;
    int32 diff;
    int16 steps;
    uint8 calSteps;

    voc = ((pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VOC) >> 2) * 3;
    if (pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VOCSHIFT) {
        voc += 12;
    } else {
        voc += 36;
    }
    voc *= 1000; /* Scale to mV */

    vas = ((pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VAS_MSB) << 2) +
          ((pLineObj->registers.dcFeed[1] & VP886_R_DCFEED_VAS_LSB) >> 6);
    vas = vas * 732 + 2923;

    ila = (pLineObj->registers.dcFeed[1] & VP886_R_DCFEED_ILA) + 18;
    ila *= 1000; /* Scale to uA */

    irOverhead = irOverheadArr[(pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_IR_OVERHEAD) >> 6];

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBat: voc %li, vas %li, ila %li, irOverhead %i", voc, vas, ila, irOverhead));

    /* On-hook VAS */
    vab = voc;
    target = vab + vas;
    target *= -1;
    calculatedBat = Vp886ComputeTrackingBat(&pCalData->spe.trk, vab, vas, 0, 0, polarity);
    adjustedBat = VpRoundedDivide(calculatedBat * 1000, pCalData->cmn.batterySense.gain);

    diff = adjustedBat - target;
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBat(pol:%d): Onhook VAS error = %li mV (target = %li, adjustedBat = %li uA)",
        polarity, diff, target, adjustedBat));
    steps = VpRoundedDivide(diff, VP886_VAS_CORR_STEP);
    if (steps > 7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBat(pol:%d): Onhook VAS correction saturates = %d ", polarity, steps));
        steps = 7;
    }
    if (steps < -7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBat(pol:%d): Onhook VAS correction saturates = %d ", polarity, steps));
        steps = -7;
    }
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBat(pol:%d): Adjust Onhook VAS by %d step(s)", polarity, steps));
    if (steps < 0) {
        calSteps = ABS(steps) | 0x08;
    } else {
        calSteps = steps;
    }
    if (polarity == VP_POL_NORM) {
        polCal[2] &= ~VP886_R_NORMCAL_VASONHKCOR;
        polCal[2] |= (calSteps & VP886_R_NORMCAL_VASONHKCOR);
    } else {
        polCal[2] &= ~VP886_R_REVCAL_VASONHKCOR;
        polCal[2] |= (calSteps & VP886_R_REVCAL_VASONHKCOR);
    }


    /* Off-hook VAS */
    /* Assume 600 Ohm load to determine VAB */
    vab = VpRoundedDivide(ila * 600, 1000);
    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBat: Offhook VAS - vab %li", vab));
    target = vab + vas + VpRoundedDivide(ila * irOverhead, 1000);
    target *= -1;
    calculatedBat = Vp886ComputeTrackingBat(&pCalData->spe.trk, vab, vas, ila, irOverhead, polarity);
    adjustedBat = VpRoundedDivide(calculatedBat * 1000, pCalData->cmn.batterySense.gain);

    diff = adjustedBat - target;
    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBat(pol:%d): Offhook VAS error = %li mV (target = %li, adjustedBat = %li mV)",
        polarity, diff, target, adjustedBat));
    steps = VpRoundedDivide(diff, VP886_VAS_CORR_STEP);
    if (steps > 7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBat(pol:%d): Offhook VAS correction saturates = %d ", polarity, steps));
        steps = 7;
    }
    if (steps < -7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBat(pol:%d): Offhook VAS correction saturates = %d ", polarity, steps));
        steps = -7;
    }
    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBat(pol:%d): Adjust Offhook VAS by %d step(s)", polarity, steps));
    if (steps < 0) {
        calSteps = ABS(steps) | 0x08;
    } else {
        calSteps = steps;
    }
    if (polarity == VP_POL_NORM) {
        polCal[2] &= ~VP886_R_NORMCAL_VASOFFHKCOR;
        polCal[2] |= ((calSteps << 4) & VP886_R_NORMCAL_VASOFFHKCOR);
    } else {
        polCal[2] &= ~VP886_R_REVCAL_VASOFFHKCOR;
        polCal[2] |= ((calSteps << 4) & VP886_R_REVCAL_VASOFFHKCOR);
    }

    /* Logic is reverse on the longitudinal side */
    diff = VP886_LONG_OVERHEAD_SHIFT - (int32)pCalData->cmn.longActive.offset;

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBat(pol:%d): Long Pt error = %li mV",
        polarity, diff));

    if (VP886_REVISION(pDevObj) == VP886_R_RCNPCN_RCN_AAA) {
        steps = VpRoundedDivide(diff, VP886_LONG_CORR_STEP_REV1);
    } else {
        steps = VpRoundedDivide(diff, VP886_LONG_CORR_STEP_REV2);
    }

    if (steps > 7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBat(pol:%d): Long Pt correction saturates = %d ", polarity, steps));
        steps = 7;
    }
    if (steps < -7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBat(pol:%d): Long Pt correction saturates = %d ", polarity, steps));
        steps = -7;
    }
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBat(pol:%d): Adjust Long Pt by %d step(s)", polarity, steps));
    if (steps < 0) {
        calSteps = ABS(steps) | 0x08;
    } else {
        calSteps = steps;
    }
    if (polarity == VP_POL_NORM) {
        polCal[0] &= ~VP886_R_NORMCAL_EXTBATCOR;
        polCal[0] |= (calSteps & VP886_R_NORMCAL_EXTBATCOR);
    } else {
        polCal[0] &= ~VP886_R_REVCAL_EXTBATCOR;
        polCal[0] |= (calSteps & VP886_R_REVCAL_EXTBATCOR);
    }

    return;
}

void
Vp886ApplyCalAbsBat(
    VpLineCtxType *pLineCtx,
    uint8 *polCal,
    uint8 polarity)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886AbsCalDeviceDataType *pCalData = &pDevObj->calData[channelId].spe.abs;
    int32 diff;
    int16 steps;
    uint8 calSteps;

    /* Battery switch calibration */
    if (polarity == VP_POL_NORM) {
        diff = (int32)pCalData->absVabNormal.offset;
    } else {
        diff = (int32)pCalData->absVabReverse.offset;
    }

    steps = VpRoundedDivide(diff, VP886_VAS_CORR_STEP);
    if (steps > 7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalAbsBat(pol:%d): Battery switch correction saturates = %d ", polarity, steps));
        steps = 7;
    }
    if (steps < -7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalAbsBat(pol:%d): Battery switch correction saturates = %d ", polarity, steps));
        steps = -7;
    }
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalAbsBat(pol:%d): Adjust Battery switch by %d step(s)", polarity, steps));
    if (steps < 0) {
        calSteps = ABS(steps) | 0x08;
    } else {
        calSteps = steps;
    }
    if (polarity == VP_POL_NORM) {
        polCal[2] &= ~VP886_R_NORMCAL_BATSWITCH;
        polCal[2] |= ((calSteps << 4) & VP886_R_NORMCAL_BATSWITCH);
    } else {
        polCal[2] &= ~VP886_R_REVCAL_BATSWITCH;
        polCal[2] |= ((calSteps << 4) & VP886_R_REVCAL_BATSWITCH);
    }

    return;
}

void
Vp886ApplyCalTrackingBatRinging(
    VpLineCtxType *pLineCtx,
    uint8 *ringCal)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CalDeviceDataType *pCalData = &pDevObj->calData[channelId];
    int32 amplitude;
    int32 bias;
    int32 vas;
    int32 vab;
    int32 target;
    int32 calculatedBatNorm;
    int32 calculatedBatRev;
    int32 diff;
    int16 steps;
    uint8 calSteps;

    amplitude = pLineObj->ringAmplitude;
    bias = pLineObj->ringBias;

    /* int16 range is +/- 154.4V (154400 mV per 0x8000, reduces to 4825 mV
       per 0x400) */
    amplitude = VpRoundedDivide(amplitude * 4825, 0x400);
    bias = VpRoundedDivide(bias * 4825, 0x400);

    vas = ((pLineObj->registers.dcFeed[0] & VP886_R_DCFEED_VAS_MSB) << 2) +
          ((pLineObj->registers.dcFeed[1] & VP886_R_DCFEED_VAS_LSB) >> 6);
    vas = vas * 732 + 2923;

    vab = ABS(amplitude) + ABS(bias);

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBatRinging: Bias %li, Amp %li, vas %li, vab %li", bias, amplitude, vas, vab));

    /* Ringing VAS */
    target = (vab + vas) * -1;

    /* As ringing swipes across both polarity, pick the worst one */
    calculatedBatNorm = Vp886ComputeTrackingBat(&pCalData->spe.trk, vab, vas, 0, 0, VP_POL_NORM);
    calculatedBatRev = Vp886ComputeTrackingBat(&pCalData->spe.trk, vab, vas, 0, 0, VP_POL_REV);

    calculatedBatNorm = VpRoundedDivide(calculatedBatNorm * 1000L, pCalData->cmn.batterySense.gain);
    calculatedBatRev = VpRoundedDivide(calculatedBatRev * 1000L, pCalData->cmn.batterySense.gain);

    if ((calculatedBatNorm - target) > (calculatedBatRev - target)) {
        diff = calculatedBatNorm - target;
    } else {
        diff = calculatedBatRev - target;
    }

    steps = VpRoundedDivide(diff, VP886_VAS_CORR_STEP);
    if (steps > 7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBatRinging: Ringing VAS correction saturates = %d ", steps));
        steps = 7;
    }
    if (steps < -7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBatRinging: Ringing VAS correction saturates = %d ", steps));
        steps = -7;
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBatRinging: Adjust Ringing VAS by %d step(s)", steps));

    if (steps < 0) {
        calSteps = ABS(steps) | 0x08;
    } else {
        calSteps = steps;
    }

    ringCal[1] &= ~VP886_R_RINGCAL_VASCOR;
    ringCal[1] |= ((calSteps << 4) & VP886_R_RINGCAL_VASCOR);

    /* Apply longitudinal correction */
    /* Logic is reverse on the longitudinal side */
    diff = VP886_LONG_OVERHEAD_SHIFT - (int32)pCalData->cmn.longRinging.offset;

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886ApplyCalTrackingBatRinging: Long Pt error = %li mV",
        diff));

    if (VP886_REVISION(pDevObj) == VP886_R_RCNPCN_RCN_AAA) {
        steps = VpRoundedDivide(diff, VP886_LONG_CORR_STEP_REV1);
    } else {
        steps = VpRoundedDivide(diff, VP886_LONG_CORR_STEP_REV2);
    }

    if (VP886_REVISION(pDevObj) == VP886_R_RCNPCN_RCN_AAA) {
        steps = VpRoundedDivide(diff, VP886_LONG_CORR_STEP_REV1);
    } else {
        steps = VpRoundedDivide(diff, VP886_LONG_CORR_STEP_REV2);
    }

    if (steps > 7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBatRinging: Ringing Long Pt correction saturates = %d ", steps));
        steps = 7;
    }
    if (steps < -7) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Vp886ApplyCalTrackingBatRinging: Ringing Long Pt correction saturates = %d ", steps));
        steps = -7;
    }
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp886ApplyCalTrackingBatRinging: Adjust Ringing Long Pt by %d step(s)", steps));
    if (steps < 0) {
        calSteps = ABS(steps) | 0x08;
    } else {
        calSteps = steps;
    }
    ringCal[0] &= ~VP886_R_RINGCAL_EXTBATCOR;
    ringCal[0] |= ((calSteps << 4) & VP886_R_RINGCAL_EXTBATCOR);

    return;
}

int32
Vp886ComputeTrackingBat(
    Vp886TrkCalDeviceDataType *pCalData,
    int32 vab,          /* mV */
    int32 vas,          /* mV */
    int32 imt,          /* uA */
    int32 rOverhead,    /* Ohms */
    uint8 polarity)
{
    int32 trackingBat;

    if (polarity == VP_POL_NORM) {
        vab = VpRoundedDivide(vab * (int32)pCalData->trackerVabNormal.gain, 1000);
        vab += pCalData->trackerVabNormal.offset;
        vas = VpRoundedDivide(vas * (int32)pCalData->trackerVasNormal.gain, 1000);
    } else {
        vab = VpRoundedDivide(vab * (int32)pCalData->trackerVabReverse.gain, 1000);
        vab += pCalData->trackerVabReverse.offset;
        vas = VpRoundedDivide(vas * (int32)pCalData->trackerVasReverse.gain, 1000);
    }

    trackingBat = vab + vas + VpRoundedDivide(imt * rOverhead, 1000);
    trackingBat *= -1;

    return trackingBat;
}

int16
Vp886ComputeRingingParam(
    Vp886CmnCalDeviceDataType *pCalData,
    int16 ringParam)
{
    int32 ringMv;

    /* int16 range is +/- 154.4V (154400 mV per 0x8000, reduces to 4825 mV
       per 0x400) */
    ringMv = (int32)ringParam * 4825;
    ringMv = VpRoundedDivide(ringMv, 0x400);

    /* Start by removing the sense gain (measurement feedback error) */
    ringMv *= pCalData->vocSenseNormal.gain;
    ringMv = VpRoundedDivide(ringMv, 1000);

    /* Remove the ring generator gain */
    ringMv *= 1000;
    ringMv = VpRoundedDivide(ringMv, pCalData->ringingGenerator.gain);

    /* Return to original int16 range */
    ringParam = (int16)VpRoundedDivide(ringMv * 0x400, 4825);

    return ringParam;
}

int32
Vp886ComputeError(
    VpLineCtxType *pLineCtx,
    uint8 errorType,
    int16 target,
    uint8 polarity)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    int32 actual, diff = 0;

    switch (errorType) {
        case VP886_ERR_ILA: {
            int32 targetUa;

            if (polarity == VP_POL_NORM) {
                targetUa = (int32)target * 1000;
                actual = targetUa * pCalData->ilaNormal.gain;
                actual = VpRoundedDivide(actual, 1000);
                actual += pCalData->ilaNormal.offset;
            } else {
                targetUa = (int32)target * (-1) * 1000;
                actual = targetUa * pCalData->ilaReverse.gain;
                actual = VpRoundedDivide(actual, 1000);
                actual += pCalData->ilaReverse.offset;
            }

            /* Compute the difference to the target */
            diff = actual - targetUa;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError(pol:%d): ILA error = %li uA (actual = %li uA)",
                polarity, diff, actual));
            break;
        }
        case VP886_ERR_VOC: {
            int32 targetMv;
            int32 buffErr;

            /* We can only program 3V steps, but the target is in 1V steps.
               The DCFEED register will be programmed to the nearest 3V step to
               the target, so start the calculation of "actual" from there. */
            actual = 3 * VpRoundedDivide(target, 3);

            /* Remove the VOC generator gain and offset */
            if (polarity == VP_POL_NORM) {
                targetMv = (int32)target * 1000;
                actual *= pCalData->vocNormal.gain;
                actual += pCalData->vocNormal.offset;
            } else {
                targetMv = (int32)target * (-1) * 1000;
                actual *= (-1) * pCalData->vocReverse.gain;
                actual += pCalData->vocReverse.offset;
            }

            /* Remove the offset from the output buffer */
            buffErr = (int32)pCalData->ringingBuffer.offset * 1000;
            buffErr = VpRoundedDivide(buffErr, (int32)pCalData->ringingBuffer.gain * 40);
            actual += buffErr;

            /* Remove the gain from the feedback path */
            actual = VpRoundedDivide(actual * 1000, pCalData->vocSenseNormal.gain);

            diff = actual - targetMv;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError(pol:%d): VOC error = %li mV (actual = %li mV).  buffErr = %ld",
                polarity, diff, actual, buffErr));
            break;
        }
        case VP886_ERR_HOOK: {
            if (polarity == VP_POL_NORM) {
                /* compute the error in uA */
                diff = pCalData->hookNormal.offset;
            } else if (polarity == VP_POL_REV) {
                /* compute the error in uA */
                diff = pCalData->hookReverse.offset * -1;
            } else {    /* polarity == VP_POL_NONE */
                /* compute the error in mV */
                diff = pCalData->hookLPM.offset * -1;
            }

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError(pol:%d): Hook threshold error = %li uA", polarity, diff));
            break;
        }
        case VP886_ERR_GND_KEY: {
            /* compute the error in uA */
            diff = pCalData->gndKeyLong.offset * -1;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError(pol:%d): Ground Key threshold error = %li uA", polarity, diff));
            break;
        }
        case VP886_ERR_RINGBIAS: {
            int32 targetMv;
            int32 buffErr;

            /* int16 range is +/- 154.4V (154400 mV per 0x8000, reduces to 4825 mV
               per 0x400) */
            targetMv = (int32)target * 4825;
            targetMv = VpRoundedDivide(targetMv, 0x400);

            /* Remove the ringing generator gain and offset */
            actual = targetMv * pCalData->ringingGenerator.gain;
            actual = VpRoundedDivide(actual, 1000);
            actual += pCalData->ringingGenerator.offset;

            /* Remove the offset from the output buffer */
            buffErr = (int32)pCalData->ringingBuffer.offset * 1000;
            buffErr = VpRoundedDivide(buffErr, (int32)pCalData->ringingBuffer.gain * 40);
            actual += buffErr;

            /* Remove the gain from the feedback path */
            actual *= 1000;
            actual = VpRoundedDivide(actual, pCalData->vocSenseNormal.gain);

            diff = actual - targetMv;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError: Ringing DC Bias error = %li mV (actual = %li mV).  buffErr = %ld",
                diff, actual, buffErr));
            break;
        }
        case VP886_ERR_FLOOR:
            /* We can only program 5V steps.  The SWPARAMS register will be
               set to nearest 5V step to the target, so start the "actual"
               calculation from there. */
            actual = 5 * VpRoundedDivide(target, 5);

            /* Do not calibrate SWZ on Buck-boost ABS as it shifts the ringing longitudinal point */
            if (VP886_IS_ABS(pDevObj) && (channelId == 1) &&
                (pDevObj->devProfileData.swCfg == VP886_DEV_PROFILE_SW_CONF_BB)) {
                /* Convert in mV */
                actual = actual * 1000L;
                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Vp886ComputeError: FloorV not calibrated on BB ABS"));
            } else {
                actual = actual * pCalData->fixedBat.gain + pCalData->fixedBat.offset;
                actual = VpRoundedDivide(actual * 1000, pCalData->batterySense.gain);
            }

            diff = actual - (int32)target * 1000;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError: FloorV error = %li mV (actual = %li mV)",
                diff, actual));
            break;

        case VP886_ERR_ABS_LONG:
            break;

        case VP886_ERR_FIXEDRING:
            actual = (int32)target * pCalData->fixedBat.gain + pCalData->fixedBat.offset;
            actual = VpRoundedDivide(actual * 1000, pCalData->batterySense.gain);

            diff = actual - (int32)target * 1000;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError: Fixed Ring Bat error = %li mV (actual = %li mV)",
                diff, actual));
            break;

        case VP886_ERR_LOWPOWER:
            /* We can only actually program in 5V units.  Round the target to
               the nearest 5V step, then allow the computed error to naturally
               give us the extra precision by comparing to the original target */
            /* NOTE: This assumes that ConfigLine is also programming the SWP
               register to the nearest 5V step to 4V above VOC */
            actual = 5 * VpRoundedDivide(target, 5);
            actual = (int32)actual * pCalData->fixedBat.gain + pCalData->fixedBat.offset;
            actual = VpRoundedDivide(actual * 1000, pCalData->batterySense.gain);

            diff = actual - (int32)target * 1000;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Vp886ComputeError: Low Power Batt error = %li mV (actual = %li mV, target = %d V)",
                diff, actual, target));
            break;

        case VP886_ERR_60V_LONG: {
            break;
        }

        case VP886_ERR_BAT_SAT: {
            /* Compute the programmed VAS in mV */
            actual = target * 732L + 2923L;

            /* Compute the diff to VP886_BAT_SAT_TARGET including the offset error */
            diff = (actual * 3 / 4 - VP886_BAT_SAT_TARGET + pCalData->batSat.offset) * -1;
            break;
        }

        case VP886_ERR_BAT_LIMIT: {
            if (channelId == 0) {
                actual = (int32)pDevObj->devProfileData.swyLimit * 1000L;
            } else {
                actual = (int32)pDevObj->devProfileData.swzLimit * 1000L;
            }

            /* If the switcher limit is not specified in the profile, just correct the offset */
            if (actual == 0) {
                if (VP886_IS_HV(pDevObj)) {
                    /* Assume the battery clamp set to 154.38V in ringing */
                    diff = pCalData->swLimit50V.offset + pCalData->swLimit100V.offset;
                } else {
                    /* Assume the battery clamp set to 102.92V */
                    diff = pCalData->swLimit100V.offset;
                }
            } else {
                if (VP886_IS_HV(pDevObj)) {
                    /* Assume the battery clamp set to 154.38V in ringing */
                    diff = 154380L - actual + pCalData->swLimit50V.offset + pCalData->swLimit100V.offset;
                } else {
                    /* Assume the battery clamp set to 102.92V */
                    diff = 102920L - actual + pCalData->swLimit100V.offset;
                }
            }
            break;
        }
        default:
            break;
    }

    return diff;
}

VpStatusType
Vp886AdjustCalReg(
    VpLineCtxType *pLineCtx,
    uint8 *calReg,
    uint8 errorType,
    int32 diff,
    uint8 polarity)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    int16 steps;
    uint8 calSteps;

    switch (errorType) {
        case VP886_ERR_ILA:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_ILA_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): ILA correction saturates = %d ",
                    polarity, steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): Adjust ILA by %d step(s)",
                polarity, steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the calibration register according to polarity */
            if (polarity == VP_POL_NORM) {
                calReg[1] &= ~VP886_R_NORMCAL_ILACOR;
                calReg[1] |= calSteps;
            } else {
                calReg[1] &= ~VP886_R_REVCAL_ILACOR;
                calReg[1] |= calSteps;
            }
            break;

        case VP886_ERR_VOC:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_VOC_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): VOC correction saturates = %d ",
                    polarity, steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): Adjust VOC by %d step(s)",
                polarity, steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the calibration register according to polarity */
            if (polarity == VP_POL_NORM) {
                calReg[1] &= ~VP886_R_NORMCAL_VOCCOR;
                calReg[1] |= (calSteps << 4);
            } else {
                calReg[1] &= ~VP886_R_REVCAL_VOCCOR;
                calReg[1] |= (calSteps << 4);
            }
            break;

        case VP886_ERR_HOOK:
            /* Compute the number of calibration increments (rounded) */
            if (polarity == VP_POL_NONE) {
                steps = VpRoundedDivide(diff, VP886_HOOK_CORR_STEP_LP);
            } else {
                steps = VpRoundedDivide(diff, VP886_HOOK_CORR_STEP);
            }

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): HOOK correction saturates = %d ",
                    polarity, steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): Adjust HOOK by %d step(s)",
                polarity, steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the calibration register according to polarity */
            if (polarity == VP_POL_NORM) {
                calReg[0] &= ~VP886_R_NORMCAL_SHCOR;
                calReg[0] |= (calSteps << 4);
            } else if (polarity == VP_POL_REV) {
                calReg[0] &= ~VP886_R_REVCAL_SHCOR;
                calReg[0] |= (calSteps << 4);
            } else {    /* polarity == VP_POL_NONE */
                calReg[0] &= ~VP886_R_INDCAL_LP_SHCOR;
                calReg[0] |= (calSteps << 4);
            }
            break;

        case VP886_ERR_GND_KEY:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_GND_KEY_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): GndKey correction saturates = %d ",
                    polarity, steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg(pol:%d): Adjust GndKey by %d step(s)",
                polarity, steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the calibration register according to polarity */
            calReg[0] &= ~VP886_R_INDCAL_GND_KEY;
            calReg[0] |= calSteps;
            break;

        case VP886_ERR_RINGBIAS:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_VOC_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Ringing DC Bias correction saturates = %d ",
                    steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Adjust Ringing DC Bias by %d step(s)",
                steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the correction */
            calReg[0] &= ~VP886_R_RINGCAL_VOCCOR;
            calReg[0] |= calSteps;
            break;

        case VP886_ERR_FLOOR:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_FIXEDBAT_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: FloorV correction saturates = %d ",
                    steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Adjust FloorV by %d step(s)",
                steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the correction */
            if (VP886_IS_TRACKER(pDevObj)) {
                calReg[0] &= ~VP886_R_BATCAL_FLOOR;
                calReg[0] |= (calSteps << 4);
            } else {    /*  ABS */
                calReg[0] &= ~VP886_R_BATCAL_FLOOR_ABS;
                calReg[0] |= (calSteps << 3);
            }
            break;

        case VP886_ERR_FIXEDRING:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_FIXEDBAT_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Fixed Ring Bat correction saturates = %d ",
                    steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Adjust Fixed Ring Bat by %d step(s)",
                steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the correction */
            calReg[0] &= ~VP886_R_BATCAL_RING;
            calReg[0] |= calSteps;
            break;

        case VP886_ERR_ABS_LONG:
            /* Apply a fixed 1 step toward ground longitudinal correction */
            if (polarity == VP_POL_NORM) {
                calReg[0] &= ~VP886_R_NORMCAL_EXTBATCOR;
                calReg[0] |= 0x01;
            } else if (polarity == VP_POL_REV) {
                calReg[0] &= ~VP886_R_REVCAL_EXTBATCOR;
                calReg[0] |= 0x01;
            } else {
                calReg[0] &= ~VP886_R_RINGCAL_EXTBATCOR;
                calReg[0] |= 0x10;
            }
            break;

        case VP886_ERR_LOWPOWER:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_FIXEDBAT_CORR_STEP);
            steps *= -1;

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Low Power Batt correction saturates = %d ",
                    steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Adjust Low Power Batt by %d step(s)",
                steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the correction */
            calReg[1] &= ~VP886_R_BATCAL_LOWPOWER;
            calReg[1] |= (calSteps << 4);
            break;

        case VP886_ERR_60V_LONG:
            /* Apply a fixed 1 step toward ground longitudinal correction */
            calReg[1] &= ~VP886_R_INDCAL_60VCLAMP;
            calReg[1] |= 0x01;
            break;

        case VP886_ERR_BAT_SAT:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff, VP886_BAT_SAT_CORR_STEP);

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Battery Sat correction saturates = %d ",
                    steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Adjust Battery Sat by %d step(s)",
                steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the correction */
            calReg[2] &= ~VP886_R_INDCAL_BATTERY_SAT;
            calReg[2] |= calSteps;
            break;

        case VP886_ERR_BAT_LIMIT:
            /* Compute the number of calibration increments (rounded) */
            steps = VpRoundedDivide(diff * -1, VP886_BAT_LIM_CORR_STEP);

            if (ABS(steps) > 7) {
                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Battery Limit correction saturates = %d ",
                    steps));
                steps = MAX(-7, MIN(steps, 7));
            }
            VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886AdjustCalReg: Adjust Battery Limit by %d step(s)",
                steps));

            calSteps = ABS(steps);
            if (steps < 0) {
                calSteps |= 0x08;
            }
            /* Program the correction */
            calReg[1] &= ~VP886_R_BATCAL_SW_LIMIT;
            calReg[1] |= calSteps;
            break;

        default:
            break;
    }

    return VP_STATUS_SUCCESS;
}

/** Vp886QuickCalStart()
  Begins the process of quickly recalibrating the VA, VB, and VAB sense offsets.

  The timerId, timerHandle, and sadcFlag arguments are intended to allow the
  state machine for this operation to be driven by another state machine such
  as line test.  When this procedure needs to set a timer or use the SADC, it
  will use these inputs.

  The externalCall flag, when TRUE, indicates that we need to mask hook events
  at completion and generate a CAL_CMP event.
*/
VpStatusType
Vp886QuickCalStart(
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 timerHandle,
    uint16 sadcFlag,
    bool externalCall)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886QuickCalStart(): Beginning sense recal, ts %u.",
        Vp886GetTimestamp(pDevCtx)));

    /* Save the original registers */
    VpMemCpy(pDevObj->regPad[channelId].slacState, pLineObj->registers.sysState, VP886_R_STATE_LEN);
    VpMemCpy(pDevObj->regPad[channelId].icr2, pLineObj->registers.icr2, VP886_R_ICR2_LEN);
    VpMemCpy(pDevObj->regPad[channelId].icr3, pLineObj->registers.icr3, VP886_R_ICR3_LEN);
    VpMemCpy(pDevObj->regPad[channelId].icr4, pLineObj->registers.icr4, VP886_R_ICR4_LEN);

    pLineObj->quickCal.timerId = timerId;
    pLineObj->quickCal.timerHandle = timerHandle;
    pLineObj->quickCal.sadcFlag = sadcFlag;
    pLineObj->quickCal.externalCall = externalCall;

    pLineObj->quickCal.state = VP886_QUICKCAL_INIT;

    return Vp886QuickCalHandler(pLineCtx);
}

/** Vp886QuickCalStop()
  Ends the quick calibration routine, either normally or for an abort case.

  If the externalCall flag was set to TRUE, we mask hook events and generate
  a CAL_CMP event.
*/
VpStatusType
Vp886QuickCalStop(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    uint8 calCtrl[VP886_R_CALCTRL_LEN];

    if (pLineObj->quickCal.state == VP886_QUICKCAL_INACTIVE) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886QuickCalStop(): Sense recal was not active."));
        return VP_STATUS_SUCCESS;
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886QuickCalStop(): Ending sense recal, ts %u.",
        Vp886GetTimestamp(pDevCtx)));

    /* Reset sense inputs and correction currents */
    calCtrl[0] = 0;
    calCtrl[1] = 0;
    calCtrl[2] = 0;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calCtrl);

    /* Restore original registers */
    VpMemCpy(pLineObj->registers.sysState, pDevObj->regPad[channelId].slacState, VP886_R_STATE_LEN);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);
    VpMemCpy(pLineObj->registers.icr2, pDevObj->regPad[channelId].icr2, VP886_R_ICR2_LEN);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);
    VpMemCpy(pLineObj->registers.icr3, pDevObj->regPad[channelId].icr3, VP886_R_ICR3_LEN);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->registers.icr3);
    VpMemCpy(pLineObj->registers.icr4, pDevObj->regPad[channelId].icr4, VP886_R_ICR4_LEN);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);

    /* Ignore hook and gkey indications briefly after cal is complete.
       Having the sense inputs disconnected will create false signals. */
    Vp886SetDetectMask(pLineCtx, VP_CSLAC_HOOK);
    Vp886SetDetectMask(pLineCtx, VP_CSLAC_GKEY);
    Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_HOOK_FREEZE, VP886_HOOKFREEZE_CAL_CMP, 0);
    Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_GKEY_FREEZE, VP886_GKEYFREEZE_CAL_CMP, 0);

    if (pLineObj->quickCal.externalCall) {
        /* Generate a CAL_CMP event */
        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_EVID_CAL_CMP, 0, Vp886GetTimestamp(pDevCtx), FALSE);
    }

    pLineObj->quickCal.state = VP886_QUICKCAL_INACTIVE;

    return VP_STATUS_SUCCESS;
}

/** Vp886QuickCalHandler()
  Implements the state machine for quickly recalibrating sense offsets.
*/
VpStatusType
Vp886QuickCalHandler(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[channelId].cmn;
    bool runAnotherState = TRUE;

    while (runAnotherState == TRUE) {
        runAnotherState = FALSE;

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886QuickCalHandler(): state %d, ts %u",
            pLineObj->quickCal.state, Vp886GetTimestamp(pDevCtx)));

        switch (pLineObj->quickCal.state) {
            case VP886_QUICKCAL_INACTIVE: {
                VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886QuickCalHandler(): Handler should not be running while inactive"));
                break;
            }
            case VP886_QUICKCAL_INIT: {
                uint8 calCtrl[VP886_R_CALCTRL_LEN];
                /* small setup when if line state is disconnect so that voltages can be measured */
                if ((pLineObj->registers.sysState[0] & VP886_R_STATE_SS) == VP886_R_STATE_SS_DISCONNECT) {
                    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp886QuickCalHandler(): measuring in disconnect"));

                    /* Ensure the tip and ring sense signals are on */
                    pLineObj->registers.icr2[0] = 0x0C;
                    pLineObj->registers.icr2[1] = 0x0C;
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);

                    /* Ensure the line control circuits are on */
                    pLineObj->registers.icr3[0] |= 0x21;
                    pLineObj->registers.icr3[1] |= 0x21;
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->registers.icr3);

                    /* Connect External Ctrl Network to AC Feedback Loop */
                    pLineObj->registers.icr4[0] |= 0x01;
                    pLineObj->registers.icr4[1] |= 0x01;
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);
                }

                /* Ensure normal polarity */
                pLineObj->registers.sysState[0] &= ~VP886_R_STATE_POL;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);

                /* Disconnect tip and ring sense inputs and correction currents. */
                calCtrl[0] = 0;
                calCtrl[1] = VP886_R_CALCTRL_TIP_CORR_DIS | VP886_R_CALCTRL_RING_CORR_DIS;
                calCtrl[2] = VP886_R_CALCTRL_RING_INP_SEL_DISC | VP886_R_CALCTRL_TIP_INP_SEL_DISC;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_CALCTRL_WRT, VP886_R_CALCTRL_LEN, calCtrl);

                runAnotherState = TRUE;
                pLineObj->quickCal.state = VP886_QUICKCAL_SETUP_VA_VB_VAB;
                break;
            }
            case VP886_QUICKCAL_SETUP_VA_VB_VAB: {
                uint8 adcRoute[5];

                adcRoute[0] = VP886_R_SADC_SEL_TIP_GROUND_V;
                adcRoute[1] = VP886_R_SADC_SEL_RING_GROUND_V;
                adcRoute[2] = VP886_R_SADC_SEL_TIP_RING_DC_V;
                adcRoute[3] = VP886_R_SADC_SEL_TIP_GROUND_V; /* Not used */
                adcRoute[4] = VP886_R_SADC_SEL_TIP_GROUND_V; /* Not used */

                Vp886SetGroupAdcMath(pDevCtx, channelId, adcRoute, VP886_CAL_SETTLE_TIME,
                    VP886_CAL_INTEGRATE_EXT_TIME);
                pLineObj->busyFlags |= pLineObj->quickCal.sadcFlag;
                pLineObj->quickCal.state = VP886_QUICKCAL_READ_VA_VB_VAB;
                break;
            }
            case VP886_QUICKCAL_READ_VA_VB_VAB: {
                int16 results[5];

                Vp886GetGroupAdcMath(pDevCtx, channelId, results, FALSE, 3);

                pCalData->tipSense.offset = results[0] * (-1);
                pCalData->ringSense.offset = results[1] * (-1);
                pCalData->vabSenseNormal.offset = results[2] * (-1);

                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Vp886QuickCalHandler(): tipSense.offset %d", pCalData->tipSense.offset));
                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Vp886QuickCalHandler(): ringSense.offset %d", pCalData->ringSense.offset));
                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Vp886QuickCalHandler(): vabSenseNormal.offset %d", pCalData->vabSenseNormal.offset));

                runAnotherState = TRUE;
                pLineObj->quickCal.state = VP886_QUICKCAL_SETUP_VABREV;
                break;
            }
            case VP886_QUICKCAL_SETUP_VABREV: {
                /* Go to polrev state. */
                pLineObj->registers.sysState[0] |= VP886_R_STATE_POL;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);

                Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_TIP_RING_DC_V, VP886_CAL_SETTLE_TIME,
                    VP886_CAL_INTEGRATE_EXT_TIME);
                pLineObj->busyFlags |= pLineObj->quickCal.sadcFlag;
                pLineObj->quickCal.state = VP886_QUICKCAL_READ_VABREV;
                break;
            }
            case VP886_QUICKCAL_READ_VABREV: {
                int16 result;

                result = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

                pCalData->vabSenseReverse.offset = result * (-1);

                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Vp886QuickCalHandler(): vabSenseReverse.offset %d", pCalData->vabSenseReverse.offset));

                runAnotherState = TRUE;
                pLineObj->quickCal.state = VP886_QUICKCAL_SETUP_VABRING;
                break;
            }
            case VP886_QUICKCAL_SETUP_VABRING: {
                /* Backdoor ringing mode*/
                pLineObj->registers.sysState[0] &= ~VP886_R_STATE_POL;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);
                pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
                pLineObj->registers.icr2[1] |= VP886_R_ICR2_DAC_RING_LEVELS;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);

                Vp886SetSingleAdcMath(pDevCtx, channelId, VP886_R_SADC_SEL_TIP_RING_DC_V, VP886_CAL_SETTLE_TIME,
                    VP886_CAL_INTEGRATE_EXT_TIME);
                pLineObj->busyFlags |= pLineObj->quickCal.sadcFlag;
                pLineObj->quickCal.state = VP886_QUICKCAL_READ_VABRING;
                break;
            }
            case VP886_QUICKCAL_READ_VABRING: {
                int16 result;

                result = Vp886GetSingleAdcMath(pDevCtx, channelId, FALSE);

                pCalData->vabSenseRinging.offset = result * (-1);

                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Vp886QuickCalHandler(): vabSenseRinging.offset %d", pCalData->vabSenseRinging.offset));

                runAnotherState = TRUE;
                pLineObj->quickCal.state = VP886_QUICKCAL_FINISHED;
                break;
            }
            case VP886_QUICKCAL_FINISHED: {
                Vp886QuickCalStop(pLineCtx);
                break;
            }
            default: {
                VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886QuickCalHandler(): Unhandled state %d", pLineObj->quickCal.state));
                break;
            }
        }
    }
    return VP_STATUS_SUCCESS;
}

#endif /* VP_CC_886_SERIES */
