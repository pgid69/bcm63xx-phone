/** \file vp790_api.h
 * vp790_api.h
 *
 *  Header file that defines the Vp790 Device and Line Objects
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9115 $
 * $LastChangedDate: 2011-11-15 15:25:17 -0600 (Tue, 15 Nov 2011) $
 */

#ifndef VP790_API_H
#define VP790_API_H

#include "vp_hal.h"
#include "vp_CSLAC_types.h"

#define VP790_MAX_MPI_DATA  0x0E    /**< Max data from any MPI read command */
#define VP790_INT_SEQ_LEN   22
/*
 * Required Vp790_ Device and Line Objects for user instantiation if a Vp790
 * device is used
 */
typedef struct Vp790DeviceProfileType {
    uint8  maxNumInterrupts;
    uint16 pcmClkRate;
    uint8  mClkMask;
    uint8  mClkCorrection;
    uint16 tickRate;
    uint8  devCfg1;
    uint8  clockSlot;
    uint8  calibrationTime;
} Vp790DeviceProfileType;

typedef struct Vp790LineObjectType {
    uint8 channelId;    /**< Channel # associated with "this" line on the
                         * device
                         */

    VpTermType termType;    /**< Termination type of this line */

    VpOptionCodecType codec;    /**< Maintains the currently used code mode to
                                 * reduce MPI activity
                                 */

    VpOptionLineStateType lineStateBatOption;

#ifdef VP_CSLAC_SEQ_EN
    uint8 cidEncodeBuff[6]; /**< Encoded CID data from user data */
    uint8 cidEncodeIndex;   /**< current index into encoded buffer */
    uint8 cidEncodeSize;    /**< Size of valid CID encoded data */

    VpCallerIdType callerId;    /**< Caller ID related information */
    VpSeqDataType cadence;      /**< Sequencer related information */
    VpCidSeqDataType cidSeq;    /**< CID Sequencer related information */

    /**< Array to control internally run sequences */
    VpProfileDataType intSequence[VP790_INT_SEQ_LEN];

    bool thisFskCid;   /**< TRUE if device tick has not changed since last call
                        * to FSK CID Generator Function call. Used to prevent
                        * CID lockup with while() loop to load CID data in the
                        * device.
                        */
#endif

    bool slsCalledFromApi;  /**< TRUE if SetLineState is called by the API */

    VpDialPulseDetectType dpStruct; /**< Dial Pulse Parameters for detecting
                                     * digits on FXS lines
                                     */

    VpOptionEventMaskType lineEventsMask;
    VpOptionEventMaskType lineEvents;

    VpOptionPulseModeType pulseMode;

    uint8 signalingData;    /**< Holds data for Signaling events on this line */
    uint16 processData;     /**< Holds data for Process events on this line */

    VpCslacTimerStruct lineTimers; /**< Timers for "this" line */

    VpApiIntLineStateType lineState;    /**< Line state info, used for state
                                         * transition handling.
                                         */

    VpOptionPcmTxRxCntrlType pcmTxRxCtrl;   /* Defines how the PCM highway is
                                             * set for "talk" linestates
                                             */
    VpOptionRingControlType ringCtrl;

    VpOptionZeroCrossType zeroCross;    /**< Defines how zero crossing is
                                         * implemented. The device doesn't really
                                         * support zero crossing "methods", but
                                         * storing what the user selected is
                                         * necessary for Get Option (so the
                                         * operations of Set/Get can be
                                         * orthogonal)
                                         */

    VpVirtualDeviceReg virtualDeviceReg;    /**< Virtual device register to
                                             * minimize device access
                                             */

    VpProfilePtrType pRingingCadence;   /**< Currently used ringing cadence on
                                         * this line
                                         */

    VpProfilePtrType pCidProfileType1;  /**< Currently used caller ID profile
                                         * on this line for sequenced cid
                                         */

    VpProfilePtrType pCidProfileType2;  /**< Currently used caller ID profile
                                         * on this line for non-sequenced cid
                                         */

    uint16 lineEventHandle; /**< Line specific event handle information */

    uint16 dtmfDigitSense;          /**< Used to hold the DTMF digit reported
                                     * with VpDtmfDigitDetected() until the
                                     * VP_LINE_EVID_DTMF_DIG is generated.
                                     */

    VpLineIdType lineId;	/**< Application provided value for mapping a line to
                             * a line context
                             */
} Vp790LineObjectType;

typedef struct Vp790DeviceObjectType {
    VpDeviceIdType deviceId;    /**< Device chip select ID or type defined by
                                 * user
                                 */

    VpDeviceStaticInfoType staticInfo;      /**< Info that will not change
                                             * during runtime
                                             */
    VpDeviceDynamicInfoType dynamicInfo;    /**< Info that will change during
                                             * runtime
                                             */
    VpCSLACDeviceStatusType status;

    VpOptionEventMaskType deviceEventsMask;
    VpOptionEventMaskType deviceEvents;

    VpOptionPulseType pulseSpecs;

    VpOptionCriticalFltType criticalFault;

    uint16 devTimer[VP_DEV_TIMER_LAST];
    Vp790DeviceProfileType devProfileData;
    VpCSLACDeviceProfileTableType devProfileTable;
    VpCSLACProfileTableEntryType profEntry;

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    uint8 mpiData[VP790_MAX_MPI_DATA];  /**< Buffer for MPI Low level reads to
                                         * hold maximum amount of MPI data that
                                         * is possible
                                         */

    uint8 mpiLen;       /**< Length of data to be copied into mpiData buffer */
#endif

    uint16 eventHandle; /**< Device level event handle information */
    uint16 timeStamp;
    int16 timeRemainder;

    VpGetResultsOptionsType getResultsOption;
} Vp790DeviceObjectType;

#endif  /* vp790_api.h */




