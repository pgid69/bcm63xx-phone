/** \file vp886_adaptive_ringing.c
 * vp886_adaptive_ringing.c
 *
 *  This file contains the Adaptive Ringing functions for
 *  the Vp886 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11079 $
 * $LastChangedDate: 2013-07-19 10:43:36 -0500 (Fri, 19 Jul 2013) $
 */

#include "vp_api_cfg.h"

#ifdef VP886_INCLUDE_ADAPTIVE_RINGING

#if defined (VP_CC_886_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_pulse_decode.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "sys_service.h"

/* As a note for the reader:
 *
 * In the comments, the terms:
 *
 * Thermal Ringing
 * Power Adaptation
 * Ringing Power
 * The algorithm
 * .
 * .
 *
 * or any combination or subset thereof are all referring to the same thing
 *
 * Also, Vab, Vtr, Vtip-ring are all referring to the same thing
 */

/* Uncomment to display warning messages on the console if SADC/VADC
 * data buffers overflow.
 */
#define INCLUDE_BUFFER_OVERFLOW_WARNINGS

/* Prototypes for internal helper functions */
static VpStatusType
Vp886AdaptiveRingingADCCfg(
    VpLineCtxType *pLineCtx);

static VpStatusType
Vp886AdaptiveRingingGetSADCData(
    VpLineCtxType *pLineCtx);

static void
Vp886AdaptiveRingingProcessData(
    VpLineCtxType *pLineCtx);

static void
Vp886AdaptiveRingingComputeRingGainFromLoadPower(
    VpLineCtxType *pLineCtx,
    RingPowerAdaptChannelData* pRPACD,
    int32 loadPwrInt,
    int16 samples);

static void
Vp886AdaptiveRingingAdaptVoltages (
    VpLineCtxType *pLineCtx,
    RingPowerAdaptChannelData* pRPACD);

static void
Vp886AdaptiveRingingMeasureRingPower(
    VpLineCtxType *pLineCtx);

static void
Vp886AdaptiveRingingAdaptionLoopIteration(
    VpLineCtxType *pLineCtx,
    void* arg);

static void
Vp886AdaptiveRingingChanReset(
    VpLineCtxType *pLineCtx);

static int16
Vp886AdaptiveRingingScaleData(
    int16 inputVal,
    int16 origFullScale,
    int16 newFullscale);

static int32
Vp886AdaptiveRingingSumSat(
    int32 input1,
    int32 input2);

static void
Vp886AdaptiveRingingSetTickCount(
    VpLineCtxType *pLineCtx);

static void
Vp886AdaptiveRingingSetFuseResistance(
    VpLineCtxType *pLineCtx,
    int16 rFuse);

#undef DBG_BUFF_DATA
#ifdef DBG_BUFF_DATA
int16 vtrDbgBuf[1024], vbatDbgBuf[1024], imtDbgBuf[1024];
int16 vtrDbgIdx, vbatDbgIdx, imtDbgIdx;
#endif

/**
 * Vp886AdaptiveRingingInit()
 *
 * This function initializes thermal ringing power adaptation for the
 * specified line, most notably the lines RingPowerAdaptChannelData data
 * structure. This includes data related to the filters and state machine.
 *
 * This function is called from with VpInitLineInit()
 *
 * \param[in]   VpLineCtxType *pLineCtx        the line's line context
 *
 * \retval      VP_STATUS_SUCCESS
 */
EXTERN VpStatusType
Vp886AdaptiveRingingInit(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType*            pLineObj = pLineCtx->pLineObj;
    uint8                               chan = pLineObj->channelId;
    RingPowerAdaptChannelData*        pRPACD = &pLineObj->ringPowerAdapt.rpaChanData;
    VpStatusType                      status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingInit+"));

    /* Init the otherwise constant channel ID for each array element. */
    *(unsigned short*)&pRPACD->chan = chan;

    Vp886AdaptiveRingingChanReset(pLineCtx);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingInit-"));

    return status;

} /* Vp886AdaptiveRingingInit() */

/**
 * Vp886AdaptiveRingingChanReset()
 *
 * This function does some of the actual work of the Vp886AdaptiveRingingInit()
 * above, namely initializing the line's RingPowerAdaptChannelData struct
 *
 * This function is called from with Vp886AdaptiveRingingInit()
 *
 * \param[in]   VpLineCtxType *pLineCtx        the line's line context
 *
 * \retval      void
 */
static void
Vp886AdaptiveRingingChanReset(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType*          pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*         pRPA = &pLineObj->ringPowerAdapt;
    RingPowerAdaptChannelData*      pRPACD = &pRPA->rpaChanData;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingChanReset+"));

    /* Init state machine related vars */
    pRPACD->halfCycCount = 0;
    pRPACD->halfCycIdx = -1; /* IDLE */

    /* Set the fuse resistance */
    Vp886AdaptiveRingingSetFuseResistance(pLineCtx, pLineObj->lineTopology.rInsideDcSense);

    /* Be sure to update ring param only on zero cross */
    if (!(pLineObj->registers.ssCfg[1] & VP886_R_SSCFG_RING_UPDATE)) {
        pLineObj->registers.ssCfg[1] |= VP886_R_SSCFG_RING_UPDATE;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SSCFG_WRT, VP886_R_SSCFG_LEN, pLineObj->registers.ssCfg);
    }

    /* Save off the original RTTH value */
    pRPA->rtth = pLineObj->registers.loopSup[2] & VP886_R_LOOPSUP_RTRIP_THRESH;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingChanReset-"));

} /* Vp886AdaptiveRingingChanReset() */

/**
 * Vp886AdaptiveRingingMeasureRingPower()
 *
 * \todo Document me!
 *
 * This gets called after the SADC and VADC buffers have been read,
 * and data collected and calibration applied
 *
 *  \param[in]   VpLineCtxType *pLineCtx        the line's line context
 *
 */

/* This the equivalent to RingPowerMeasurementJob(void* arg) in the NGSLAC. There, it
 * is set up as a 2kHz Job. Here, we will just have it run at 500Hz as the SADC/VADC samples
 * come in. For now, it will called in Vp886AdaptiveRingingProcessData() via Vp886AdaptiveRingingHandler()
 */
static void
Vp886AdaptiveRingingMeasureRingPower(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType*          pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*         pRPA = &pLineObj->ringPowerAdapt;
    RingPowerAdaptChannelData*      pRPACD = &pRPA->rpaChanData;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingMeasureRingPower+"));

    /* For all available samples .. */
    while(pRPA->samplesAvail > 0) {
        int16 vab, imt;
        SlicPowerIntegrator acc;

        /* The half-cycle index variable serves two purposes:
         *
         *  1.  It is a single-bit counter, incremented at each half-cycle, that
         *      determines which one of two half-cycle buffers the integrator
         *      data is copied into.
         *
         *  2.  It is the state variable for a very simple state machine, used
         *      to detect the beginning and end of ringing bursts, where special
         *      actions are taken to condition the power adaptation loop filter.
         *      The state diagram follows.
         *
         *      +------+  state is ringing   +------+                   +------+
         *      |      |                     |      |------------------>|      |
         *      | IDLE |-------------------->| EVEN |  half-cycle end   |  ODD |
         *      | (-1) |                     |  (0) |<------------------| (+1) |
         *      +------+                     +------+                   +------+
         *          ^                            |                          |
         *          |    state is not ringing    |                          |
         *          \----------------------------+--------------------------/
         */

        /* It should be impossible for the code to even take this route if
         * the half cycle index is -1. But if it does, bail out now */
        if (pRPACD->halfCycIdx < 0) break;

        pRPA->samplesAvail--;

        /* Grab the VAB and the IMT. Scale them to match that of the original
         * NGSLAC code */
        vab = Vp886AdaptiveRingingScaleData(pRPA->vtrBuf[pRPA->vtrBufSWRdIdx++],
            VAB_INPUT_FULL_SCALE, VAB_DESIRED_FULL_SCALE);
        pRPA->vtrBufSWRdIdx %= VP_886_THERMAL_RINGING_BUF_SIZE;

        imt = Vp886AdaptiveRingingScaleData(pRPA->imtBuf[pRPA->imtBufSWRdIdx++],
            IMT_INPUT_FULL_SCALE, IMT_DESIRED_FULL_SCALE);
        pRPA->imtBufSWRdIdx %= VP_886_THERMAL_RINGING_BUF_SIZE;

        #ifdef DBG_BUFF_DATA
        vtrDbgBuf[vtrDbgIdx++] = vab;
        if (vtrDbgIdx >= 1024) vtrDbgIdx = 1023;
        imtDbgBuf[imtDbgIdx++] = imt;
        if (imtDbgIdx >= 1024) imtDbgIdx = 1023;
        #endif

        /* Sample the active battery voltage, compute the instantaneous battery
         * power output, and accumulate. */
        {
            int16 vBat;
            int32 batPwr;

            /* Grab the VBAT and scale to match NGSLAC code */
            vBat = Vp886AdaptiveRingingScaleData(pRPA->vbatBuf[pRPA->vbatBufSWRdIdx++],
                VBAT_INPUT_FULL_SCALE, VBAT_DESIRED_FULL_SCALE);
            pRPA->vbatBufSWRdIdx %= VP_886_THERMAL_RINGING_BUF_SIZE;

            #ifdef DBG_BUFF_DATA
            vbatDbgBuf[vbatDbgIdx++] = vBat;
            if (vbatDbgIdx >= 1024) vbatDbgIdx = 1023;
            #endif

            /* Note that battery voltages are converted from +/- 200V full-scale
             * to +/- 400V full-scale for two reasons:
             *  + so that the voltage between rails can be calculated for any
             *    battery voltages without saturating (e.g. +200 - -200 = 400)
             *  + so that battery power samples are in same scale as load power,
             *    which makes net power calculations easy. */
            batPwr = ABS(vBat) * ABS(imt) * 2;
            batPwr /= 128;
            acc.batPwr = Vp886AdaptiveRingingSumSat(pRPACD->integrator.batPwr, batPwr);

        }

        /* Compute and accumulate the instantaneous load power. */
        {
            int32 loadPwr;

            /*loadPwr = L_mult(abs_s(vab), abs_s(imt));*/
            loadPwr = ABS(vab) * ABS(imt) * 2;
            loadPwr /= 128;
            acc.loadPwr = Vp886AdaptiveRingingSumSat(pRPACD->integrator.loadPwr, loadPwr);
        }

        /* Compute and accumulate the instantaneous squared load current. */
        {
            int32 imtSqr;

            imtSqr = imt * imt * 2;
            imtSqr /= 128;
            acc.imtSqr = Vp886AdaptiveRingingSumSat(pRPACD->integrator.imtSqr, imtSqr);
        }

        /* Count the samples accumulated thus far */
        acc.samples = pRPACD->integrator.samples + 1;

        /* Check for a ringing half cycle */
        pRPA->halfCycleTickCount--;

        if (pRPA->halfCycleTickCount == 0) {
            int16 halfCycIdx;

            halfCycIdx = pRPACD->halfCycIdx;

            /* Copy local accumulator state to one of two half-cycle buffers and
             * clear the local accumulator. */
            VpMemCpy(&pRPACD->halfCyc[halfCycIdx], &acc,
                sizeof(pRPACD->halfCyc[halfCycIdx]));
            VpMemSet(&acc, 0, sizeof(acc));

            /* Setup the half-cycle index to write the other buffer the next
             * time around. */
            pRPACD->halfCycIdx = (int16)(halfCycIdx ^ 1);

            pRPA->halfCycleTickCount = pRPA->halfCycleTicks[halfCycIdx];

            /* Execute an interation of the control loop filter */
            Vp886AdaptiveRingingAdaptionLoopIteration(pLineCtx, (void*)pRPACD);
        }

        /* Copy local accumulator state to static integrator. */
        VpMemCpy(&pRPACD->integrator, &acc, sizeof(pRPACD->integrator));
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingMeasureRingPower-"));

} /* Vp886AdaptiveRingingMeasureRingPower() */

/**
 * Vp886AdaptiveRingingAdaptionLoopIteration()
 *
 * \todo Document me!
 *
 * This function is called every half-ringing cycle
 *
 *  \param[in]   VpLineCtxType *pLineCtx        the line's line context
 *  \param[in]   void* arg                      void pointer that gets
 *                                              casted as a pointer to this
 *                                              line's RingPowerAdaptChannelData
 *
 */
static void
Vp886AdaptiveRingingAdaptionLoopIteration(
    VpLineCtxType *pLineCtx,
    void* arg)
{
    RingPowerAdaptChannelData*        pRPACD = (RingPowerAdaptChannelData*)arg;

    int32 loadPwrInt;
    int16 samples;
#if defined(VP_DEBUG) && (VP_CC_DEBUG_SELECT & VP_DBG_ADP_RING)
    Vp886LineObjectType*            pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*           pRPA = &pLineObj->ringPowerAdapt;
    int32 batPwrInt, fusePwrInt, slicPwrInt;
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdaptionLoopIteration+"));

    /* Calculate the integration period, which is the number of samples
     * taken through a complete ringing cycle.
     *
     * NOTE: In the case of the very first half-cycle of a ringing pulse,
     * only one of the two half-cycle integrators contains meaningful data.
     * The net sample count will be 1/2 of the ringing period, so the first
     * half-cycle is effectively doubled. */
    samples = pRPACD->halfCyc[0].samples + pRPACD->halfCyc[1].samples;

    /* Samples have been pre-scaled by a factor of 128, which should be
     * sufficient to collect one full-cycle of data for ringing signals
     * down to 16 Hz (2000 Hz / 16 Hz = 125 samples). If we exceed 128
     * samples then the accumulators could theoretically overflow. */
    /* The above can probably be modified since we are samplting at 500Hz */

    /* Compute the load power for the full cycle as the sum of the load
     * power for the last two half-cycles. */
    loadPwrInt = Vp886AdaptiveRingingSumSat(pRPACD->halfCyc[0].loadPwr, pRPACD->halfCyc[1].loadPwr);

    /* Power debug output */
#if defined(VP_DEBUG) && (VP_CC_DEBUG_SELECT & VP_DBG_ADP_RING)
    /* Compute the battery power for the full cycle as the sum of the battery
     * power for the last two half-cycles. */
    batPwrInt = Vp886AdaptiveRingingSumSat(pRPACD->halfCyc[0].batPwr, pRPACD->halfCyc[1].batPwr);

    /* Compute power dissipated through the fuse resistors as IMT^2 * 2 * RF.
     * RF is assumed constant during ringing. RF is 32-bit signed integer
     * in the range of 0 to 100 Ohms. The conversion function is derived below.
     *
     * Let 'imt' be the real metallic current in Amps.
     * Let 'IMT' be the signed fractional 1.15 current from the AFE driver.
     *
     *  imt = IMT * 0.1 * 2^-15                                         (1)
     *
     * Instantaneous power through the fuse resistor is calculated as
     *
     *  pf = imt^2 * rf                                             (2)
     *
     * Let real 'rf' equal firmware 'RF', and substitute eq 1 for imt.
     *
     *  pf = (IMT * 0.1 * 2^-15)^2 * RF
     *  pf = IMT^2 * 0.1^2 * 2^(-15*2) * RF
     *  pf = IMT^2 * 0.01 * 2^-30 * RF                              (3)
     *
     * Recall that the common conversion from "real" power (pptc) in Watts
     * to "firmware" power (PPTC) in signed fractional 1.31 is
     *
     *  pf = PF * 40 * 2^-31                                        (4)
     *
     * Solving equations 3 and 4 for PPTC gives
     *
     *  PF = IMT^2 * 0.01 * 2^-30 * RF * 1/40 * 2^31
     *  PF = IMT^2 * RF * 1/2000                                    (5)
     *
     * Equation 5 is what we want to calculate for every IMT sample, taking
     * the average of this instantaneous power over a complete ring period.
     * However, it is more efficient to accumulate the sum of IMT^2 and
     * multiply this sum by RF * 1/2000 once for each full ring period.
     * Remember that the power numbers we've computed thus far are the sum
     * (integral) of power over a ring cycle. PTC power is effectively
     * handled the same way.
     *
     * NOTE: The fractional mult() intrinsics have an implicit multiply by
     * two (left shift by one), so we need NOT explicitly multiply RF by
     * two here to account for the power dissipated in both PTC devices. */
    {
        int32 imtSqrInt;
        int16 rFuseLocal;

        /* Compute RF/2000 as signed fractional 1.15. The result must be
         * between 0 and 32767 (+0.99997). */
        imtSqrInt = VpRoundedDivide(pRPA->rFuse * 32768L, 2000);
        if (imtSqrInt > VP_INT16_MAX) {
            rFuseLocal = VP_INT16_MAX;
        } else {
            rFuseLocal = imtSqrInt;
        }

        imtSqrInt = Vp886AdaptiveRingingSumSat(pRPACD->halfCyc[0].imtSqr, pRPACD->halfCyc[1].imtSqr);
        fusePwrInt = imtSqrInt * rFuseLocal;
    }

   /* Compute the integral of the power dissipated by the SLIC. */
    slicPwrInt = batPwrInt - Vp886AdaptiveRingingSumSat(loadPwrInt, fusePwrInt);

    {
        int32 batPwrDbg, loadPwrDbg, fusePwrDbg, slicPwrDbg;
        int16 fScale = VpRoundedDivide(620, samples);
        uint16 tStamp = Vp886GetTimestamp(pLineCtx->pDevCtx);
        bool verboseDebug = FALSE;

        batPwrDbg = (batPwrInt * fScale) >> 18;
        loadPwrDbg = (loadPwrInt * fScale) >> 18;
        fusePwrDbg = (fusePwrInt * fScale) >> 18;
        slicPwrDbg = (slicPwrInt * fScale) >> 18;
        slicPwrDbg = slicPwrDbg;
        fusePwrDbg = fusePwrDbg;
        batPwrDbg = batPwrDbg;

        /* The verbose debug can desynchronize the algorithm , even using dmesg */
        if (verboseDebug) {
            VP_ADP_RING(VpLineCtxType, pLineCtx, ("L = %4ld mW, ringV = %3ld V, threshold = %4d mW, dCntLow = %d, dCntHigh = %d, ts = %5d ms",
                loadPwrDbg, ((pLineObj->ringAmplitudeCal * pRPACD->ringGain) >> 15) * 473L / 100000, pRPA->rspt, pRPA->debCounterLow, pRPA->debCounterHigh, tStamp / 2));
        } else {
            VP_ADP_RING(VpLineCtxType, pLineCtx, ("L = %4ld mW", loadPwrDbg));
        }
    }
#endif

    Vp886AdaptiveRingingComputeRingGainFromLoadPower(pLineCtx, pRPACD, loadPwrInt, samples);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdaptionLoopIteration-"));

} /* Vp886AdaptiveRingingAdaptionLoopIteration */


/**
 * Vp886AdaptiveRingingComputeRingGainFromLoadPower()
 *
 * \todo Document me!
 *
 * This function is called from above.
 *
 *  \param[in]   VpLineCtxType *pLineCtx            the line's line context
 *  \param[in]   RingPowerAdaptChannelData* pRPACD  thisline's RingPowerAdaptChannelData
 *  \param[in]   int32 loadPwrInt                   integral of the power dissipated by the load
 *  \param[in]   int16 samples                      number of cycles integrated
 *
 */
static void
Vp886AdaptiveRingingComputeRingGainFromLoadPower(
    VpLineCtxType *pLineCtx,
    RingPowerAdaptChannelData* pRPACD,
    int32 loadPwrInt,
    int16 samples)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;
    int32 loadPwr;
    int16 fScale = VpRoundedDivide(620, samples);
    bool needsUpdate = FALSE;
    uint8 channelId = pLineObj->channelId;
    uint8 otherChannelId = pDevObj->staticInfo.maxChannels - (channelId + 1);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingComputeRingGainFromLoadPower+"));

    if (!((pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SHARED_TRACKER) ||
        (pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SHARED_BB_ABS) ||
        (pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SINGLE_BB_TRACKER))) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingComputeRingGainFromLoadPower(): Unsupported mode %d",
            pDevObj->options.adaptiveRinging.mode));
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingComputeRingGainFromLoadPower-"));
        return;
    }

    /* Do not change the gain while debouncing */
    if (pRPA->debouncing) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingComputeRingGainFromLoadPower-"));
        return;
    }

    /* Convert the power in mW */
    loadPwr = (loadPwrInt * fScale) >> 18;

    /* Low ringing voltage */
    if (pRPACD->ringGain < FULL_RINGING_GAIN) {
        if (loadPwr < pRPA->rspt) {
            pRPA->debCounterLow = 0;

            if (pRPA->debCounterHigh > (DEB_COUNTER_MAX - 1)) {
                pRPA->debCounterHigh = 0;

                /* This channel could use high ringing gain */
                pDevObj->isPowerLimited[channelId] = FALSE;

                /* Try to go to high ringing voltage */
                if ((otherChannelId == channelId) || (pDevObj->isPowerLimited[otherChannelId] == FALSE)) {
                    /* 1 channel device OR other channel not power limited */
                    pRPACD->ringGain = FULL_RINGING_GAIN;
                    pRPA->rspt = pRPA->rsptHigh;
                    needsUpdate = TRUE;
                }
            } else {
                /* Next power measurement below the threshold will change the level */
                pRPA->debCounterHigh++;
            }
        } else {
            pRPA->debCounterHigh = 0;

            if (pRPA->debCounterLow > (DEB_COUNTER_MAX - 1)) {
                pRPA->debCounterLow = 0;

                /* Stay in low ringing voltage */
                pRPA->rspt = pRPA->rsptLow;
            } else {
                /* Next power measurement above the threshold will change the level */
                pRPA->debCounterLow++;
            }
        }
    /* High ringing voltage */
    } else {
        if ((loadPwr > pRPA->rspt) || (pDevObj->isPowerLimited[otherChannelId])) {
            pRPA->debCounterHigh = 0;

            if (pRPA->debCounterLow > (DEB_COUNTER_MAX - 1)) {
                pRPA->debCounterLow = 0;

                /* This channel is now power limited */
                pDevObj->isPowerLimited[channelId] = TRUE;

                /* Go to low ringing voltage */
                pRPACD->ringGain = FULL_RINGING_GAIN * pDevObj->options.adaptiveRinging.minVoltagePercent / 100;
                pRPA->rspt = pRPA->rsptLow;
                needsUpdate = TRUE;
            } else {
                /* Next power measurement above the threshold will change the level */
                pRPA->debCounterLow++;
            }
        } else {
            pRPA->debCounterLow = 0;

            if (pRPA->debCounterHigh > (DEB_COUNTER_MAX - 1)) {
                pRPA->debCounterHigh = 0;

                /* Stay in high ringing voltage */
                pRPA->rspt = pRPA->rsptHigh;
            } else {
                /* Next power measurement above the threshold will change the level */
                pRPA->debCounterHigh++;
            }
        }
    }

    if (needsUpdate) {
        /* Update ringing voltage and battery */
        Vp886AdaptiveRingingAdaptVoltages(pLineCtx, pRPACD);

        pRPA->debouncing = TRUE;
        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_THERMAL_RINGING,
            VP_886_THERMAL_RINGING_DEBOUNCE_DURATION, pLineObj->channelId, VP886_THERMAL_RINGING_DEBOUNCE);
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingComputeRingGainFromLoadPower-"));
}  /* Vp886AdaptiveRingingComputeRingGainFromLoadPower */

/**
 * Vp886AdaptiveRingingAdaptVoltages()
 *
 * \todo Document me!
 *
 * This function is called from above.
 *
 *  \param[in]   VpLineCtxType *pLineCtx            the line's line context
 *  \param[in]   RingPowerAdaptChannelData* pRPACD  thisline's RingPowerAdaptChannelData
 *
 */
static void
Vp886AdaptiveRingingAdaptVoltages (
    VpLineCtxType *pLineCtx,
    RingPowerAdaptChannelData* pRPACD)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;
    uint8 nomIlr, highIlr;
    int32 battery_V;
    uint8 swParam[VP886_R_SWPARAM_LEN];
    int32 newRingAmp;
    int32 amplitude;
    int32 dcBias;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdaptVoltages+"));

    if (pLineObj->lowIlr) {
        nomIlr = (pLineObj->registers.loopSup[3] & VP886_R_LOOPSUP_RING_CUR_LIM) + 22;
    } else {
        nomIlr = (pLineObj->registers.loopSup[3] & VP886_R_LOOPSUP_RING_CUR_LIM) * 2 + 50;
    }

    /* High ILR is 1.67 time larger than nominal ILR */
    highIlr = nomIlr * 167L / 100;
    if (highIlr > 112) {
        highIlr = 112;
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdaptVoltages(): ILR saturation"));
    }

    /* Program ILR to 1.67 x mominal_ILR in low ringing and mominal_ILR in high ringing voltage */
    pRPA->loopSup[2] |= VP886_R_LOOPSUP_RTRIP_THRESH;
    pRPA->loopSup[3] &= ~VP886_R_LOOPSUP_RING_CUR_LIM;

    if (pRPACD->ringGain < FULL_RINGING_GAIN) {
        /* High ILR range */
        if (highIlr >= 50) {
            pRPA->loopSup[3] |= (highIlr - 50) / 2;

            if (pRPA->lowIlr) {
                pRPA->lowIlr = FALSE;

                pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN,
                    pLineObj->registers.ringCal);

                pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
                pLineObj->registers.icr2[1] |= VP886_R_ICR2_DAC_RING_LEVELS;
                VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pLineObj->registers.icr2);
            }

        /* Low ILR range */
        } else {    /* highIlr < 50 */
            pRPA->loopSup[3] |= highIlr - 22;

            if (!pRPA->lowIlr) {
                pRPA->lowIlr = TRUE;

                pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
                pLineObj->registers.ringCal[1] |= 0x07;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN,
                    pLineObj->registers.ringCal);

                pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
                pLineObj->registers.icr2[1] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
                VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pLineObj->registers.icr2);
            }
        }
    } else { /* pRPACD->ringGain == FULL_RINGING_GAIN */
        /* High ILR range */
        if (nomIlr >= 50) {
            pRPA->loopSup[3] |= (nomIlr - 50) / 2;

            if (pRPA->lowIlr) {
                pRPA->lowIlr = FALSE;

                pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN,
                    pLineObj->registers.ringCal);

                pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
                pLineObj->registers.icr2[1] |= VP886_R_ICR2_DAC_RING_LEVELS;
                VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pLineObj->registers.icr2);
            }

        /* Low ILR range */
        } else {    /* nomIlr < 50 */
            pRPA->loopSup[3] |= nomIlr - 22;

            if (!pRPA->lowIlr) {
                pRPA->lowIlr = TRUE;

                pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
                pLineObj->registers.ringCal[1] |= 0x07;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN,
                    pLineObj->registers.ringCal);

                pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
                pLineObj->registers.icr2[1] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
                VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pLineObj->registers.icr2);
            }
        }
    }

    VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN, pRPA->loopSup);

    /* Calculate the new gain to be written into the sig gen register */
    newRingAmp = (pLineObj->ringAmplitudeCal * pRPACD->ringGain + FULL_RINGING_GAIN / 2) / FULL_RINGING_GAIN;

    switch (pDevObj->options.adaptiveRinging.mode) {
        case VP_ADAPT_RING_SHARED_TRACKER:
        case VP_ADAPT_RING_SINGLE_BB_TRACKER:
            /* Update battery voltage */
            battery_V = ((pLineObj->registers.swParam[1] & VP886_R_SWPARAM_RINGING_V) * 5 + 5);
            battery_V = (battery_V * pRPACD->ringGain + FULL_RINGING_GAIN / 2) / FULL_RINGING_GAIN;

            /* Prevent the battery to go below 60V */
            if (battery_V < 60) {
                battery_V = 60;
                VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdaptVoltages(): Battery saturation"));
            }

            /* Update the switching parameters register */
            swParam[0] = pLineObj->registers.swParam[0];
            swParam[1] = pLineObj->registers.swParam[1] & ~VP886_R_SWPARAM_RINGING_V;
            swParam[1] |= (battery_V - 5) / 5;
            swParam[2] = pLineObj->registers.swParam[2];
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, swParam);
            break;

        case VP_ADAPT_RING_SHARED_BB_ABS:
            /* Get the DC bias */
            dcBias = (pLineObj->ringBias * 471L + 50) / 100;

            /* Convert newRingAmp in mV */
            amplitude = (newRingAmp * 473L + 50) / 100;

            /* Amplitude + Bias, rounded up */
            pDevObj->absRingingPeak[pLineObj->channelId] = (ABS(amplitude) + ABS(dcBias) + 999) / 1000;

            /* Update the battery voltage (apply cal) */
            Vp886ManageABSRingingBatt(pDevCtx, TRUE, TRUE);
            break;

        default:
            break;
    }

    /* Update the amplitude field of the ring gen params register */
    pRPA->ringGenParams[5] = (uint8)((newRingAmp & 0x0000FF00) >> 8);
    pRPA->ringGenParams[6] = (uint8)(newRingAmp & 0x000000FF);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN, pRPA->ringGenParams);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdaptVoltages-"));
}

/**
 * Vp886AdaptiveRingingPrepare()
 *
 * Thermal Ringing Prepare. Should reset/intialize the thermal ringing variables,
 * data structures, etc. Should initialize the SADC block. Should start the
 * initial timer.
 *
 * This is called from Vp886SetLineStateFxsInt(). This may also be called when the
 * "make" part of a ring cadence is started (if it is not done with the above mentioned
 * function). This may also be called if a line state is returning to ringing upon
 * completion of a line test.
 *
 * There needs to be a flag to make sure this isn't called during a line test.
 *
 * There will probably be other scenarios and cases where calling this function
 * may or may not be allowed. TBD.
 *
 * Inputs:
 *  VpLineCtxType *pLineCtx         pointer to this line's line context
 *
 * Outputs:
 *  status
 */
/* Note: This function is always in flux and will settle down soon */

#define VTR_DELAY   2
#define VBAT_DELAY  2
#define IMT_DELAY   0

VpStatusType
Vp886AdaptiveRingingPrepare(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;
    RingPowerAdaptChannelData*    pRPACD = &pRPA->rpaChanData;
    int32 battery_V;
    uint8 swParam[VP886_R_SWPARAM_LEN];

    VP_ADP_RING(VpLineCtxType, pLineCtx, ("Adaptive ringing start"));

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingPrepare+"));

    if (pLineCtx == NULL) {
        /* If this channel doesn't exist, leave */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingPrepare-"));
        return VP_STATUS_INVALID_ARG;
    }

    Vp886AdaptiveRingingSetTickCount(pLineCtx);

    /* Get the current values in the ring gen register so we don't
     * have to do a read-mod-write every time we update the amplitude
     * for a gain change. We will just do the write.
     */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN, pRPA->ringGenParams);

    /* Configure the Supervision and Voice ADCs */
    Vp886AdaptiveRingingADCCfg(pLineCtx);

    /* Initialize the SADC buffer read and write indicies.
     * Are we going to carry over power info from one
     * ring burst to the next? If so, we probably don't
     * want to reinitialize the indicies here.
     */
    pRPA->vtrBufSWWrtIdx      = pRPA->vtrBufSWRdIdx     = 0;
    pRPA->vbatBufSWWrtIdx     = pRPA->vbatBufSWRdIdx    = 0;
    pRPA->imtBufSWWrtIdx      = pRPA->imtBufSWRdIdx     = 0;

    /* By delaying the Vtr, Vbat, Imt samples by various amount,
     * we can achieve better phase alignment. These numbers
     * were determined experimentally
     */
    pRPA->vtrDelay = VTR_DELAY;
    pRPA->vbatDelay = VBAT_DELAY;
    pRPA->imtDelay = IMT_DELAY;

    #ifdef DBG_BUFF_DATA
    vtrDbgIdx = vbatDbgIdx = imtDbgIdx = 0;
    #endif

    VpMemSet(&pRPA->vtrBuf[0], 0, sizeof(pRPA->vtrBuf));
    VpMemSet(&pRPA->vbatBuf[0], 0, sizeof(pRPA->vbatBuf));
    VpMemSet(&pRPA->imtBuf[0], 0, sizeof(pRPA->imtBuf));

    /* Set up a timer to come back and read the SADC data */
    Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_THERMAL_RINGING,
        VP_886_THERMAL_RINGING_TIMER_DURATION, pLineObj->channelId, VP886_THERMAL_RINGING_CADENCE);

    /* We want to throw out the first buffers worth of data from the SADC/VADC */
    pRPA->firstBufferThrownOut = FALSE;
    pRPA->crunchTheNumbers = FALSE;

    /* Set the flag in the line object to indicate that the thermal ringing algorithms
     * are running
     */
    pLineObj->thermalRinging = TRUE;

    /* Change ring power measurement state to EVEN. */
    pRPACD->halfCycIdx = 0;

    /* Backup the current loop supervision register */
    VpMemCpy(pRPA->loopSup, pLineObj->registers.loopSup, VP886_R_LOOPSUP_LEN);

    switch (pDevObj->options.adaptiveRinging.mode) {
        case VP_ADAPT_RING_SHARED_TRACKER:
        case VP_ADAPT_RING_SINGLE_BB_TRACKER:
        case VP_ADAPT_RING_SHARED_BB_ABS:
            if (pLineObj->startRingingCadence) {

                /* Backup the current lowIlr flag setting */
                pRPA->lowIlr = pLineObj->lowIlr;

                /* Only probe once per ringing cycle */
                pLineObj->startRingingCadence = FALSE;

                /* Start with the minimum gain (lower ringing volage / battery) */
                pRPACD->ringGain = FULL_RINGING_GAIN * pDevObj->options.adaptiveRinging.minVoltagePercent / 100;

                /* This channel is power limited for now */
                pDevObj->isPowerLimited[pLineObj->channelId] = TRUE;

                /* Start with the medium hysteresis threshold (agressive) */
                pRPA->rspt = pRPA->rsptMid;

                /* Update ringing voltage and battery */
                Vp886AdaptiveRingingAdaptVoltages(pLineCtx, pRPACD);
            } else {
                /* Restore the proper pRPA->lowIlr if needed */
                if (!pRPA->lowIlr && pLineObj->lowIlr) {
                    /* Get out of the lowIlr mode and restore ILR */
                    pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN,
                        pLineObj->registers.ringCal);

                    pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
                    pLineObj->registers.icr2[1] |= VP886_R_ICR2_DAC_RING_LEVELS;
                    VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                        pLineObj->registers.icr2);
                }
                /* Need to restore every time because of RTTH (maxed out during adaptive ringing) */
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN,
                    pRPA->loopSup);

                /* Restore the ringing amplitude if needed */
                if (pRPACD->ringGain != FULL_RINGING_GAIN) {
                    int32 newRingAmp;

                    /* This channel was power limited */
                    pDevObj->isPowerLimited[pLineObj->channelId] = TRUE;

                    newRingAmp = (pLineObj->ringAmplitudeCal * pRPACD->ringGain + FULL_RINGING_GAIN / 2) /
                        FULL_RINGING_GAIN;

                    /* Update the amplitude field of the ring gen params register */
                    pRPA->ringGenParams[5] = (uint8)((newRingAmp & 0x0000FF00) >> 8);
                    pRPA->ringGenParams[6] = (uint8)(newRingAmp & 0x000000FF);
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN,
                        pRPA->ringGenParams);

                    /* Restore the battery voltage if needed */
                    if ((pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SHARED_TRACKER) ||
                        (pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SINGLE_BB_TRACKER)) {
                        /* Update battery voltage */
                        battery_V = ((pLineObj->registers.swParam[1] & VP886_R_SWPARAM_RINGING_V) * 5 + 5);
                        battery_V = (battery_V * pRPACD->ringGain + FULL_RINGING_GAIN / 2) / FULL_RINGING_GAIN;

                        /* Prevent the battery to go below 60V */
                        if (battery_V < 60) {
                            battery_V = 60;
                            VP_WARNING(VpLineCtxType, pLineCtx,
                                ("Vp886AdaptiveRingingAdaptVoltages(): Battery saturation"));
                        }

                        /* Update the switching parameters register */
                        swParam[0] = pLineObj->registers.swParam[0];
                        swParam[1] = pLineObj->registers.swParam[1] & ~VP886_R_SWPARAM_RINGING_V;
                        swParam[1] |= (battery_V - 5) / 5;
                        swParam[2] = pLineObj->registers.swParam[2];
                        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN, swParam);
                    }
                }
            }
            break;

        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingPrepare(): Invalid mode %d",
                pDevObj->options.adaptiveRinging.mode));
            break;
    }

    /* Only update the power after 'DEB_COUNTER_MAX' samples above/below the threshold */
    pRPA->debCounterLow = 0;
    pRPA->debCounterHigh = 0;

    /* Set a timer to keep ILR "high long enough" when entering ringing */
    pRPA->debouncing = TRUE;
    Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_THERMAL_RINGING,
        VP_886_THERMAL_RINGING_DEBOUNCE_DURATION, pLineObj->channelId, VP886_THERMAL_RINGING_DEBOUNCE);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingPrepare-"));
    return VP_STATUS_SUCCESS;

} /* Vp886AdaptiveRingingPrepare */

/**
 * Vp886AdaptiveRingingADCCfg()
 *
 * Configures the Supervision ADC to monitor the Tip-Ring (Ringing) voltage,
 * and the battery/switcher voltage. It configures the VADC to measure the
 * metallic current.
 *
 * It is called from Vp886AdaptiveRingingPrepare
 *
 * Inputs:
 *  VpLineCtxType *pLineCtx         pointer to this line's line context
 *
 */
/* Note: This function is always in flux and will settle down soon */

#define VTR_SLOT    1
#define VBAT_SLOT   2

VpStatusType
Vp886AdaptiveRingingADCCfg(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    uint8                           chan = pLineObj->channelId;

    uint8 sadcCtrl[10];
    uint8 vadcCtrl[6];
    uint16 batterySelection;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingSADCCfg+"));

    if (VP886_IS_TRACKER(pDevObj)) {
        batterySelection = (chan == 0) ? VP886_R_SADC_SEL_SWY : VP886_R_SADC_SEL_SWZ;
    } else {    /* ABS */
        batterySelection = VP886_R_SADC_SEL_SWZ;
    }

    if (pLineCtx == NULL) {
        /* If this channel doesn't exist, leave */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingSADCCfg-"));
        return VP_STATUS_INVALID_ARG;
    }

    /* Supervision ADC Setup */
    /* No interupt. Raw data. Group mode. 500Hz. Don't enable yet */
    sadcCtrl[0] = VP886_R_SADC_GROUP_MODE | VP_886_THERMAL_RINGING_SADC_DRATE | VP886_R_SADC_ENABLE;
    /* Default all SADC slots to none */
    sadcCtrl[1] = sadcCtrl[2] = sadcCtrl[3] =
    sadcCtrl[4] = sadcCtrl[5] = VP886_R_SADC_SEL_ADC_OFFSET;
    /* Configure the specified slot for Tip-Ring Voltage */
    sadcCtrl[VTR_SLOT] = VP886_R_SADC_SEL_TIP_RING_DC_V;
    /* Configure the specified slot for Switcher/Battery SWY or SWZ */
    sadcCtrl[VBAT_SLOT] = batterySelection;
    /* Continuous Mode */
    sadcCtrl[6] = sadcCtrl[7] = 0;
    /* Must skip at least 2 samples for a HW bug */
    sadcCtrl[8] = 0; sadcCtrl[9] = 2;


    /* Voice ADC Setup */
    /* 500 Hz */
    vadcCtrl[0] = VP_886_THERMAL_RINGING_VADC_DRATE;
    /* ADC Source --> Tip - Ring metallic current */
    vadcCtrl[1] = VP886_R_VADC_SEL_METALLIC_CUR;
    /* Continuous Mode */
    vadcCtrl[2] = vadcCtrl[3] = 0;
    /* Don't skip any samples */
    vadcCtrl[4] = 0; vadcCtrl[5] = 2;


    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, sadcCtrl);
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, vadcCtrl);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingSADCCfg-"));

    return VP_STATUS_SUCCESS;


} /* Vp886AdaptiveRingingADCCfg */


/**
 * Vp886AdaptiveRingingGetSADCData()
 *
 * Reads the data from the SADC and VADC buffers. Applies calibration and stores
 * the data software buffers for later processing.
 *
 * It is called from the thermal ringing Vp886AdaptiveRingingHandler().
 *
 * Inputs:
 *  VpLineCtxType *pLineCtx         pointer to this line's line context
 *
 * Outputs:
 *  none
 */
/* Note: This function is always in flux and will settle down soon */
#define VP886_APPLY_SADC_CAL(X) (int16)VpRoundedDivide(((int32)X+(int32)(sadcOffset))*(int32)(sadcGain),1000L)
#define VP886_APPLY_VADC_CAL(X) (int16)VpRoundedDivide(((int32)X+(int32)(vadcOffset))*(int32)(vadcGain),1000L)

#define VTR_B_RD    (VP886_R_B1_RD+2*(VTR_SLOT-1))
#define VBAT_B_RD   (VP886_R_B1_RD+2*(VBAT_SLOT-1))

#define VTR_B_LEN   VP886_R_B1_LEN
#define VBAT_B_LEN  VP886_R_B1_LEN

VpStatusType
Vp886AdaptiveRingingGetSADCData(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;
    uint8                           chan = pLineObj->channelId;
    int16                       sadcGain = pDevObj->calData[chan].cmn.sadc.gain;
    int16                     sadcOffset = pDevObj->calData[chan].cmn.sadc.offset;
    int16                       vadcGain = pDevObj->calData[chan].cmn.vadcActive.gain;
    int16                     vadcOffset = pDevObj->calData[chan].cmn.vadcActive.offset;

    uint8 vtrSADCBuf[VP886_R_B1_LEN], vbatSADCBuf[VP886_R_B2_LEN], imtVADCBuf[VP886_R_VBUFFER_LEN];
    int16 vtrPtr, vbatPtr, imtPtr;
    uint8 sampleIdx, samplesAvail;
    uint8* vtrHWBufPtr;
    uint8* vbatHWBufPtr;
    uint8* imtHWBufPtr;
    int16 vtrLessSampAvail, vbatLessSampAvail, imtLessSampAvail;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData+"));

    if (pLineCtx == NULL) {
        /* If this channel doesn't exist, leave */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData-"));
        return VP_STATUS_INVALID_ARG;
    }

    /* Read the appropriate SADC data buffers and the VADC data buffer */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_VBUFFER_RD, VP886_R_VBUFFER_LEN, imtVADCBuf);
    VpSlacRegRead(NULL, pLineCtx, VTR_B_RD, VTR_B_LEN, vtrSADCBuf);
    VpSlacRegRead(NULL, pLineCtx, VBAT_B_RD, VBAT_B_LEN, vbatSADCBuf);

    /* If this is the first buffer of data, toss it */
    if (pRPA->firstBufferThrownOut == FALSE) {
        pRPA->crunchTheNumbers = FALSE;
        pRPA->firstBufferThrownOut = TRUE;
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData-"));
        return VP_STATUS_SUCCESS;

    } else {
        pRPA->crunchTheNumbers = TRUE;
    }

    /* Extract the last write pointer for each buffer. This is effectively a count
     * of the number of 16-bit samples that are ready */
    vtrPtr = vtrSADCBuf[0] & VP886_R_BX_PTR;
    vtrHWBufPtr = &vtrSADCBuf[1];
    vbatPtr = vbatSADCBuf[0] & VP886_R_BX_PTR;
    vbatHWBufPtr = &vbatSADCBuf[1];
    imtPtr = imtVADCBuf[0] & VP886_R_VBUFFER_POINTER;
    imtHWBufPtr = &imtVADCBuf[1];

#ifdef INCLUDE_BUFFER_OVERFLOW_WARNINGS
    if (vtrSADCBuf[0] & VP886_R_BX_OVFL) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData(): Vtr Buffer Overflow"));
    }
    if (vbatSADCBuf[0] & VP886_R_BX_OVFL) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData(): Vbat Buffer Overflow"));
    }
    if (imtVADCBuf[0] & VP886_R_VBUFFER_OVERFLOW) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData(): Imt Buffer Overflow"));
    }
#endif

    vtrLessSampAvail = pRPA->vtrDelay;
    vbatLessSampAvail = pRPA->vbatDelay;
    imtLessSampAvail = pRPA->imtDelay;

    /**************************************************************
     *
     * How the buffer indicies are used, incremented, wrapped, etc.
     *
     * SADC/VADC SW Read Index:
     *  Used elsewhere when the data is extracted to be used
     *  for the power calculations. Gets saved each iteration and is
     *  wrapped if necessary when the end of the buffer is reached.
     *
     * SADC/VADC SW Write Index:
     *  Used here to write the data extracted from the HW buffer into the
     *  SW buffer. It is saved and wrapped as necessary.
     *
     ************************************************************/

    /* The data *bytes* from the HW are used to construct 16-bit words */
    /* For the Vtr data */
    for (sampleIdx = 0; sampleIdx  < vtrPtr; sampleIdx++) {
        pRPA->vtrBuf[pRPA->vtrBufSWWrtIdx++] = VP886_APPLY_SADC_CAL((int16)((vtrHWBufPtr[sampleIdx*2]<<8)|(vtrHWBufPtr[sampleIdx*2+1]&0xFF)));
        if (pRPA->vtrDelay != 0) {
            pRPA->vtrDelay--;
            pRPA->vtrBufSWWrtIdx--;
        }
        pRPA->vtrBufSWWrtIdx %= VP_886_THERMAL_RINGING_BUF_SIZE;
    }

    /* For the Vbat data */
    for (sampleIdx = 0; sampleIdx  < vbatPtr; sampleIdx++) {
        pRPA->vbatBuf[pRPA->vbatBufSWWrtIdx++] = VP886_APPLY_SADC_CAL((int16)((vbatHWBufPtr[sampleIdx*2]<<8)|(vbatHWBufPtr[sampleIdx*2+1]&0xFF)));
        if (pRPA->vbatDelay != 0) {
            pRPA->vbatDelay--;
            pRPA->vbatBufSWWrtIdx--;
        }
        pRPA->vbatBufSWWrtIdx %= VP_886_THERMAL_RINGING_BUF_SIZE;
    }

    /* For the Imt data */
    for (sampleIdx = 0; sampleIdx  < imtPtr; sampleIdx++) {
        /* In low ILR mode, imt is halved */
        if (pRPA->lowIlr) {
            pRPA->imtBuf[pRPA->imtBufSWWrtIdx++] = -1*VP886_APPLY_VADC_CAL((int16)((imtHWBufPtr[sampleIdx*2]<<8)|(imtHWBufPtr[sampleIdx*2+1]&0xFF)))/2;
        } else {
            pRPA->imtBuf[pRPA->imtBufSWWrtIdx++] = -1*VP886_APPLY_VADC_CAL((int16)((imtHWBufPtr[sampleIdx*2]<<8)|(imtHWBufPtr[sampleIdx*2+1]&0xFF)));
        }
        if (pRPA->imtDelay != 0) {
            pRPA->imtDelay--;
            pRPA->imtBufSWWrtIdx--;
        }
        pRPA->imtBufSWWrtIdx %= VP_886_THERMAL_RINGING_BUF_SIZE;
    }

    /* Update the number of samples available */
    samplesAvail = MIN(MIN(vtrPtr-vtrLessSampAvail,
        vbatPtr-vbatLessSampAvail), imtPtr-imtLessSampAvail);
    pRPA->samplesAvail += samplesAvail;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingGetSADCData-"));

    return VP_STATUS_SUCCESS;

} /* Vp886AdaptiveRingingGetSADCData */


/**
 * Vp886AdaptiveRingingHandler()
 *
 *  Calls the routine which reads the data from the SADC data buffer.
 *  After applying calibration, it passes the data over to the routines
 *  that perform the power calculations, run the control loop,
 *  calculate the new ring gain, etc.
 *
 *  This function is called either from a timer handler or from the
 *  API tick.
 *
 *  It then sets a timer to start all over again.
 *  How do we corrdinate this with ring enter/exit and ringtrip>
 *  How/when to the aplitude corrections get applied?
 *
 *
 * Inputs:
 *  pLineCtx: pointer to this line's line context
 *  overrun:  not sure how relevant this is here
 *
 * Outputs:
 *  none
 */
void
Vp886AdaptiveRingingHandler(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingHandler+"));

    /* Maybe add a check to verify that thermal ringing is actually running ? */
    /* Just in case there was some sort of race condition with timers, flags,
     * etc. and this got through */
    if (pLineObj->thermalRinging == FALSE) {
        /* What should be done here ? */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingHandler-"));
    }

    /* Set a timer to do it again */
    Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_THERMAL_RINGING,
        VP_886_THERMAL_RINGING_TIMER_DURATION, pLineObj->channelId, VP886_THERMAL_RINGING_CADENCE);

    /* Get the data from the SADC buffer */
    Vp886AdaptiveRingingGetSADCData(pLineCtx);

    /* Crunch the numbers */
    if (pRPA->crunchTheNumbers) {
        Vp886AdaptiveRingingProcessData(pLineCtx);
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingHandler-"));
    return;

} /* Vp886AdaptiveRingingHandler */


/**
 * Vp886AdaptiveRingingStop()
 * Thermal Ringing Stop.
 *
 * This function stops the thermal ringing algorithm from running. It kills
 * the timer if one is running. It turns off the SADC. It clears the flag
 * indicating that thermal ringing is running.
 *
 * This is called from Vp886SetLineStateFxsInt(). This may also be called when the
 * "break" part of a ring cadence is started (if it is not done with the above mentioned
 * function). This may also be called if a line state starting a line test.
 *
 *
 * There will probably be other scenarios and cases where calling this function
 * may or may not be allowed. TBD.
 *
 * Inputs:
 * VpLineCtxType *pLineCtx pointer to this line's line context
 *
 * Outputs:
 *  status
 */
/* Should we just zero out the whole data struct? Are there power and filter
 * data that need to be carried over from the previous run? If so, we probably
 * shouldn't zero out the whole thing
 */
VpStatusType
Vp886AdaptiveRingingStop(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*       pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptChannelData*   pRPACD = &pLineObj->ringPowerAdapt.rpaChanData;
    RingPowerAdaptModuleType*      pRPA = &pLineObj->ringPowerAdapt;

    uint8 sadcCtrl[10];

    VP_ADP_RING(VpLineCtxType, pLineCtx, ("Adaptive ringing stop"));

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingStop+"));

    if (pLineObj->thermalRinging == FALSE) {
        /* If this function gets called when thermal ringing is not running, leave */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingStop-"));
        return VP_STATUS_INVALID_ARG;
    }

    if (pLineCtx == NULL) {
        /* If this channel doesn't exist, leave */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingStop-"));
        return VP_STATUS_INVALID_ARG;
    }

    /* Cancel any thermal ringing timers */
    Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_THERMAL_RINGING, 0, FALSE);

    /* No interupt. Raw data. Group mode. 500kHz. Make sure SADC in not enabled */
    sadcCtrl[0] = VP886_R_SADC_GROUP_MODE | VP_886_THERMAL_RINGING_SADC_DRATE;
    /* Turn off all SADC measurements */
    sadcCtrl[1] = VP886_R_SADC_SEL_ADC_OFFSET;
    sadcCtrl[2] = VP886_R_SADC_SEL_ADC_OFFSET;
    sadcCtrl[3] = VP886_R_SADC_SEL_ADC_OFFSET;
    sadcCtrl[4] = VP886_R_SADC_SEL_ADC_OFFSET;
    sadcCtrl[5] = VP886_R_SADC_SEL_ADC_OFFSET;
    /* Continuous Mode */
    sadcCtrl[6] = sadcCtrl[7] = 0;
    /* Don't skip any samples */
    sadcCtrl[8] = sadcCtrl[9] = 0;

    /* Write to part */
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, sadcCtrl);

    /* Indicate that we are NOT in thermal ringing anymore */
    pLineObj->thermalRinging = FALSE;

    /* This channel is not power limited anymore */
    pDevObj->isPowerLimited[pLineObj->channelId] = FALSE;

    /* When the drive state changes to non-ringing, reset the ring
     * power measurement block, including the zero-cross detector,
     * integrator, and integrated half-cycle data storage. Note that
     * we DO NOT reset the filter state here. */
    /* Again, do we have to perform zero-cross checking here ? */
    VpMemSet(&pRPACD->integrator, 0, sizeof(pRPACD->integrator));
    VpMemSet(&pRPACD->halfCyc[0], 0, sizeof(pRPACD->halfCyc));

    /* Change ring power measurement state to IDLE. */
    pRPACD->halfCycIdx = -1;

    /* Restore the proper lowIlr ILR if needed */
    if (!pRPA->lowIlr && pLineObj->lowIlr) {
        /* Go in lowIlr mode and restore ILR */
        pLineObj->registers.ringCal[1] &= ~VP886_R_RINGCAL_ILRCOR;
        pLineObj->registers.ringCal[1] |= 0x07;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGCAL_WRT, VP886_R_RINGCAL_LEN,
            pLineObj->registers.ringCal);

        pLineObj->registers.icr2[0] |= VP886_R_ICR2_DAC_RING_LEVELS;
        pLineObj->registers.icr2[1] &= ~VP886_R_ICR2_DAC_RING_LEVELS;
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
            pLineObj->registers.icr2);

    }
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_LOOPSUP_WRT, VP886_R_LOOPSUP_LEN,
        pLineObj->registers.loopSup);

    /* Restore the ringing amplitude if needed */
    if ((pRPACD->ringGain != FULL_RINGING_GAIN) &&
        ((pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SHARED_TRACKER) ||
         (pDevObj->options.adaptiveRinging.mode == VP_ADAPT_RING_SINGLE_BB_TRACKER))) {

        /* Restore the ringing floor voltage */
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
            pLineObj->registers.swParam);

        /* Update the amplitude field of the ring gen params register */
        pRPA->ringGenParams[5] = (uint8)((pLineObj->ringAmplitudeCal & 0x0000FF00) >> 8);
        pRPA->ringGenParams[6] = (uint8)(pLineObj->ringAmplitudeCal & 0x000000FF);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_RINGGEN_WRT, VP886_R_RINGGEN_LEN,
            pRPA->ringGenParams);
    }

    /* Only enable if printk is disable (use dmesg) otherwise is slows down the
       measurements and skip samples */
    #ifdef DBG_BUFF_DATA
    {
        uint16 i;

        VpSysDebugPrintf("vab =");
        for (i = 0 ; i < vtrDbgIdx ; i++) {
            VpSysDebugPrintf(" %d", vtrDbgBuf[i]);
        }
        VpSysDebugPrintf("\n");

        VpSysDebugPrintf("imt =");
        for (i = 0 ; i < imtDbgIdx ; i++) {
            VpSysDebugPrintf(" %d", imtDbgBuf[i]);
        }
        VpSysDebugPrintf("\n");

        VpSysDebugPrintf("vbat =");
        for (i = 0 ; i < vbatDbgIdx ; i++) {
            VpSysDebugPrintf(" %d", vbatDbgBuf[i]);
        }
        VpSysDebugPrintf("\n");
    }
    #endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingStop-"));

    return VP_STATUS_SUCCESS;

} /* Vp886AdaptiveRingingStop */

/**
 * Vp886AdaptiveRingingProcessData()
 *
 * Processes the voltage and current data retrieved from the device.
 * It does so by just passing the data over to the power calculation
 * routines.
 *
 * Inputs:
 *  pLineCtx: pointer to this channels Line Context
 *
 * Outputs:
 *  none
 */
void
Vp886AdaptiveRingingProcessData(
    VpLineCtxType *pLineCtx)

{
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingProcessData+"));

    /* Send over to the power calculation routines */
    Vp886AdaptiveRingingMeasureRingPower(pLineCtx);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingProcessData-"));

}   /* Vp886AdaptiveRingingProcessData */

/**
 * Vp886AdaptiveRingingScaleData()
 *
 *  Adjusts the fullscale of a value from one to another.
 *
 * TODO: Make this more robust for overflow handling.
 *
 *
 *
 * Inputs:
 *  inputVal:           value to be rescaled
 *  origFullScale:      value's current full-scale
 *  newFullscale:       value's desired full-scale
 *
 * Outputs:
 *  rescaled value
 */
int16
Vp886AdaptiveRingingScaleData(
    int16 inputVal,
    int16 origFullScale,
    int16 newFullscale)
{
    int32 tempLocal;
    int16 result;

    /* Multiply by orig full-scale first to keep as many bits of precision as possible */
    tempLocal = inputVal * origFullScale;
    /* divide by desired full-scale */
    tempLocal /= newFullscale;

    /* Make sure the result stays within 16-bits */
    if (tempLocal > VP_INT16_MAX) {
        result = VP_INT16_MAX;
    } else {
        result = tempLocal;
    }

    return result;

} /* Vp886AdaptiveRingingScaleData */

/**
 * Vp886AdaptiveRingingSumSat()
 *
 *  Sum the two longs and saturate if needed.
 *
 * Outputs:
 *  Saturated sum of input1 and input2
 */
int32
Vp886AdaptiveRingingSumSat(
    int32 input1,
    int32 input2)
{
    int32 tempLocal = input1 + input2;

    if ((input1 > 0) && (input2 > 0) && (tempLocal < MIN(input1, input2))) {
        return VP_INT32_MAX;
    }

    if ((input1 < 0) && (input2 < 0) && (tempLocal > MAX(input1, input2))) {
        return VP_INT32_MIN;
    }

    return tempLocal;

} /* Vp886AdaptiveRingingSumSat */

/**
 * Vp886AdaptiveRingingSetTickCount()
 *
 * Determines how many ticks are in a half ringing cycle
 * and sets the appropriate varibles
 *
 * The Sinusoidal case: SINTRAP == 0
 *
 * From the Command Set Doc --> FRQA is an unsigned number with a 0.3662 Hz step size
 *
 * Fring                    = FRQA * 0.3662 Hz
 * Period in seconds        = 1 / Fring = 1 / (FRQA * 0.3662)
 *
 * Since we are performing the power integration calculations
 * at a rate of 500Hz, we want the period in 2ms ticks.
 *
 * Period in 2ms ticks      = 500 / Fring = 500 / (FRQA * 0.3662)
 *                          = 1365.3741125068268705625341343528 / FRQA
 *                          = (21845.985800109229929000546149645 / (FRQA)) / 16
 *
 * For fixed point purposes, round 21845.985800109229929000546149645 -> 21846
 *
 * So,
 *
 * Period in 2ms ticks      = (21846 / FRQA) / 16
 *                          = (21846 / FRQA) >> 4
 *
 * But, we should round, so
 *
 * Period in 2ms ticks      = ((21846 / FRQA) + 0x0008) >> 4
 *
 * But, since we are executing the control loop every half-cycle
 * we whould downshift one more time.
 *
 * So,
 *
 * Half-cycle in 2ms ticks  = ((21846 / FRQA) + 0x0010) >> 5
 *
 * If the half-cycle isn't an integer number of 2ms ticks, we would effectively
 * be running the control loop at a slightly higher frequency than the correct
 * frequency. To attempt to combat this problem, we will have two different
 * half cycle tick counts. The half cycle tick count will alternate between
 * these two values so that on average, the half cycle loop is executed at the
 * right frequency.
 *
 * To do this, we first perform the above operation, but only downshift by 4.
 * If bit 0 is set, the half cycle tick count will alternate between
 * (((21846 / FRQA) + 0x0008) >> 5) and (((21846 / FRQA) + 0x0008) >> 5) + 1
 *
 * If bit 0 is not set, the half cycle count will be ((21846 / FRQA) + 0x0008) >> 5
 * everytime.
 *
 * This isn't perfect, but it is better.
 *
 * Example, for a 20Hz ringing signal, there are 25 2ms ticks, leaving
 * 12.5 2 ms ticks per half cycle. We could just let it run every 12 cycles
 * which would give an execution frequency of 20.83 Hz. Or we could
 * employ some mechanism by which the control loop runs at an alternating
 * 12 then 13 samples.
 *
 * The Trapezoidal case: SINTRAP == 1
 *
 * From the Command Set Doc --> FRQA is a signed number which sets the rise
 * time according to the following formula:
 *
 * FRQA = 8000/Fring
 *
 * Fring                    = 8000/FRQA
 *
 * Period in seconds        = 1/Fring
 *                          = FRQA/8000
 *
 * Since we are performing the power integration calculations
 * at a rate of 500Hz, we want the period in 2ms ticks.
 *
 * Period in 2ms ticks      = 500 / Fring = 500 / (8000/FRQA)
 *                          = FRQA * 500/8000
 *                          = FRQA / 16
 *                          = FRQA >> 4
 *
 * But, we should round, so
 *
 * Period in 2ms ticks      = (FRQA + 0x0008) >> 4
 *
 * But, since we are executing the control loop every half-cycle
 * we whould downshift one more time.
 *
 * So,
 *
 * Half-cycle in 2ms ticks  = (FRQA + 0x0010) >> 5
 *
 * If the half-cycle isn't an integer number of 2ms ticks, we would effectively
 * be running the control loop at a slightly higher frequency than the correct
 * frequency. To attempt to combat this problem, we will have two different
 * half cycle tick counts. The half cycle tick count will alternate between
 * these two values so that on average, the half cycle loop is executed at the
 * right frequency.
 *
 * To do this, we first perform the above operation, but only downshift by 4.
 * If bit 0 is set, the half cycle tick count will alternate between
 * (FRQA + 0x0008) >> 5 and (FRQA + 0x0008) >> 5 + 1
 *
 * If bit 0 is not set, the half cycle count will be (FRQA + 0x0008) >> 5
 * everytime.
 *
 * This isn't perfect, but it is better.
 *
 *
 *
 */
void
Vp886AdaptiveRingingSetTickCount(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;

    uint8 ringGenParams[VP886_R_RINGGEN_LEN];
    int16 frqa;
    int16 tickTmp;

    VpSlacRegRead(NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN, ringGenParams);
    frqa = (int16)(((ringGenParams[3] << 8) & 0x7F00) | (ringGenParams[4] & 0x00FF));

    /* If this is sinusoidal ringing */
    if ((ringGenParams[0] & VP886_R_RINGGEN_WAVE) == VP886_R_RINGGEN_WAVE_SINE) {

        /* Perform the division and the round, but only downshift by 4 */
        tickTmp = ((21846 / frqa) + 0x0008) >> 4;

    } else {

        /* For trapezoidal round but only downshift by 4 */
        tickTmp = (frqa + 0x0008) >> 4;

    }

    pRPA->halfCycleTicks[0] = tickTmp >> 1;
    pRPA->halfCycleTicks[1] = tickTmp >> 1;

    /* If bit 0 is set, add an extra tick to the second tick count */
    if ((tickTmp & 0x0001) == 0x0001) {
        pRPA->halfCycleTicks[1] += 1;
    }

    pRPA->halfCycleTickCount = pRPA->halfCycleTicks[0];

} /* Vp886AdaptiveRingingSetTickCount */


/**
 * Vp886AdaptiveRingingSetTargetSlicPower()
 *
 *  Allows the Target SLIC power to be adjusted
 *  by some outside entity. The can be used
 *  if a different target power is desired while
 *  the algorithm is runnung. If one line is in
 *  ringing and then another line goes into ringing, etc.
 *
 *
 * Inputs:
 *  VpLineCtxType *pLineCtx
 *  int16 targetSlicPower
 *
 *
 * Outputs:
 *  none
 */
EXTERN void
Vp886AdaptiveRingingSetTargetSlicPower(
    VpLineCtxType *pLineCtx,
    int16 targetSlicPower)
{
    VpDevCtxType*                pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType*       pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;


    int16 pcMin = pDevObj->options.adaptiveRinging.minVoltagePercent;
    int16 rspt;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdjustTargetSlicPower-"));

    /* Convert the power from 40W to 10W full scale (39mW per steps) */
    rspt = (int32)targetSlicPower * 3900 / 100;

    switch (pDevObj->options.adaptiveRinging.mode) {
        case VP_ADAPT_RING_SHARED_TRACKER:
        case VP_ADAPT_RING_SINGLE_BB_TRACKER:
        case VP_ADAPT_RING_SHARED_BB_ABS:
            /* High hysteresis threshold */
            pRPA->rsptHigh = rspt;

            /* Compute the scalling factor */
            pcMin *= pcMin;
            rspt = (int32)pcMin * rspt / 10000L;

            /* Medium hysteresis threshold (65% nominal of the specified threshold) */
            pRPA->rsptMid = rspt * 95L / 100;

            /* Low hysteresis threshold (55% nominal of the specified threshold) */
            pRPA->rsptLow = rspt * 80L / 100;
            break;

        default:
            pRPA->rspt = pRPA->rsptLow = pRPA->rsptMid = pRPA->rsptHigh = rspt;
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingSetTargetSlicPower(): Invalid mode %d",
                pDevObj->options.adaptiveRinging.mode));
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdjustTargetSlicPower-"));

} /* Vp886AdaptiveRingingSetTargetSlicPower */

/**
 * Vp886AdaptiveRingingSetFuseResistance()
 *
 *  Sets the fuse resistance. The fuse resistance will be provided
 *  by some mechanism yet to be determined.
 *
 *
 * Inputs:
 *  VpLineCtxType *pLineCtx
 *  int16 targetSlicPower
 *
 *
 * Outputs:
 *  none
 */
static void
Vp886AdaptiveRingingSetFuseResistance(
    VpLineCtxType *pLineCtx,
    int16 rFuse)
{
    Vp886LineObjectType*        pLineObj = pLineCtx->pLineObj;
    RingPowerAdaptModuleType*       pRPA = &pLineObj->ringPowerAdapt;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdjustTargetSlicPower-"));

    pRPA->rFuse = rFuse;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886AdaptiveRingingAdjustTargetSlicPower-"));

} /* Vp886AdaptiveRingingSetFuseResistance */

#endif /* defined (VP_CC_886_SERIES) */

#endif /* VP886_INCLUDE_ADAPTIVE_RINGING */
