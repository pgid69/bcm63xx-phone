/** \file vp_adaptive_ringing.h
 * vp_adaptive_ringing.h
 *
 *  Header file that defines the data structureds related to the 
 *  adaptive ringing power management routines
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9497 $
 * $LastChangedDate: 2012-02-06 17:03:04 -0600 (Mon, 06 Feb 2012) $
 */
 
#ifndef VP_THERMAL_RINGING_H
#define VP_THERMAL_RINGING_H

#include "vp886_registers.h"

/* Thermal Ringing Timer duration in ms. This needs to be short enough
 * so that the SADC and VADC buffers are emptied often enough not
 * to overflow. At the same time, the more often the timer fires,
 * the greater amount of SPI traffic to read the buffers. It's
 * a tradeoff that is system dependant. Choose wisely.
 */
#define VP_886_THERMAL_RINGING_TIMER_DURATION 10

/* Timer to debounce any power adaptation steps. */
#define VP_886_THERMAL_RINGING_DEBOUNCE_DURATION 160

/* Specifies the sampling rate for the SADC and VADC */
#define VP_886_THERMAL_RINGING_SADC_DRATE VP886_R_SADC_DRATE_GROUP_500HZ
#define VP_886_THERMAL_RINGING_VADC_DRATE VP886_R_VADC_DRATE_500HZ

/* In order to easily re-use the NGSLAC code, we scale the inputs to use
 * that of the NGSLAC */
#define VAB_INPUT_FULL_SCALE 240
#define VAB_DESIRED_FULL_SCALE 400

#define VBAT_INPUT_FULL_SCALE 240
#define VBAT_DESIRED_FULL_SCALE 400

#define IMT_INPUT_FULL_SCALE 1191
#define IMT_DESIRED_FULL_SCALE 1000

/* Full scale ringing gain */
#define FULL_RINGING_GAIN 32767L

/* Number of consecutive samples before taking a decision */
#define DEB_COUNTER_MAX 4

/* Size of data buffers used to hold data samples
 * obtained from the ADCs. This is arbitrary at the
 * moment and will likely be tweaked as necessary
 * to avoid line object bloat while still holding
 * enough samples for the algorithm to process.
 * This could simply be just the size of the HW
 * ADC buffers (12) or maybe some more just in-case
 * a delay in processing is required.
 */
#define VP_886_THERMAL_RINGING_BUF_SIZE 32

/* SLIC power calculation struct */
typedef struct SlicPowerIntegratorStruct {
    int16 samples;
    int32 batPwr;
    int32 loadPwr;
    int32 imtSqr;
} SlicPowerIntegrator;

/* There will an instance on this struct in each line object */
typedef struct RingPowerAdaptChannelDataStruct {
    const unsigned short chan;
    int16 halfCycCount;
    int16 halfCycIdx;
    SlicPowerIntegrator integrator;
    SlicPowerIntegrator halfCyc[2];
    int16 ringGain;
} RingPowerAdaptChannelData;

/* Need to define a struct that contains an instance of the above
 * along with the necessary buffers and indicies for the SADC
 * VADC data collection for Vring, Vbat, and IMT
 */
typedef struct RingPowerAdaptModuleStruct {
    RingPowerAdaptChannelData rpaChanData;
    int16 vtrBuf[VP_886_THERMAL_RINGING_BUF_SIZE];
    int16 vbatBuf[VP_886_THERMAL_RINGING_BUF_SIZE];
    int16 imtBuf[VP_886_THERMAL_RINGING_BUF_SIZE];
    int16 vtrBufSWWrtIdx;
    int16 vtrBufSWRdIdx;
    int16 vbatBufSWWrtIdx;
    int16 vbatBufSWRdIdx;
    int16 imtBufSWWrtIdx;
    int16 imtBufSWRdIdx;
    int16 halfCycleTickCount;
    int16 halfCycleTicks[2];
    int16 rspt;
    int16 rsptLow;
    int16 rsptMid;
    int16 rsptHigh;
    int16 rtth;
    int16 vtrDelay;
    int16 vbatDelay;
    int16 imtDelay;
    uint16 rFuse;
    int16 samplesAvail;
    bool firstBufferThrownOut;
    bool crunchTheNumbers;
    uint8 ringGenParams[VP886_R_RINGGEN_LEN];
    uint8 loopSup[VP886_R_LOOPSUP_LEN];
    bool lowIlr;
    bool debouncing;
    uint8 debCounterHigh;
    uint8 debCounterLow;
} RingPowerAdaptModuleType;

#endif /* VP_THERMAL_RINGING_H */
