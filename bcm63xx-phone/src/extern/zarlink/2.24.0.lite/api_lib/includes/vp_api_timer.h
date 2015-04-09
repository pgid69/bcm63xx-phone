/** \file vp_api_timer.h
 * vp_api_timer.h
 *
 * Header file for all timer typedefs used in API-II (internal).
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9115 $
 * $LastChangedDate: 2011-11-15 15:25:17 -0600 (Tue, 15 Nov 2011) $
 */

#ifndef VP_API_TIMER_H
#define VP_API_TIMER_H

/* IMT average for Voltage to Frequency (FXO) smoothing */
#define VP_IMT_AVG_LENGTH 2  /* FXO variable, but need this to compile */

#define VP_ACTIVATE_TIMER       0x8000
#define VP_TIMER_TIME_MASK      0x7FFF

/**************************************************************************************************
 * Common Timer Values - all times in ms                                                          *
 **************************************************************************************************/
/**< VP_DEV_TIMER_EXIT_RINGING_SAMPLE
 * Used on VE880 ABS only at the moment. This is the initial time to delay switching from High Power
 * to medium Power when exiting Ringing. It should be longer than the longest silent interval in any
 * Ringing Cadence ideally. If not, it just means the supply would be switched to Medium Power and
 * then quickly set back to High Power. No big deal, but not preferred.
 */
#define VP_DEV_TIMER_EXIT_RINGING_SAMPLE    (5000)

/**< VP_DEV_TIMER_EXIT_RINGING_UPDATE
 * Used on VE880 ABS only at the moment. This is the interval used to recheck for Ringing Exit after
 * waiting the time specified by VP_DEV_TIMER_EXIT_RINGING_SAMPLE.
 */
#define VP_DEV_TIMER_EXIT_RINGING_UPDATE    (1000)

/**< VP_WB_CHANGE_MASK_TIME
 * The time used to mask Clock Fault after changing to or from Wideband mode on VE880 or VE890
 */
#define VP_WB_CHANGE_MASK_TIME              (20)

/**< VP_DISCONNECT_RECOVERY_TIME
 * Hook debounce time when exiting disconnect. This is to allow hook stability in case of high
 * REN load conditions. Time of 100ms is sufficient for all normal field conditions, but if testing
 * with 5REN per GR57 (40uF + 1386ohm) this time needs to be 120-130ms.
 */
#define VP_DISCONNECT_RECOVERY_TIME         (100)

/**< VP_POLREV_DEBOUNCE_TIME
 * Debounce time to mask hook activity when reversing polarity on the FXS line. The value (120ms)
 * is designed to prevent false off-hook from being reported when changing line polarity under
 * load conditions of 5REN FCC Part 68 load with 16ms Switch Hook Debounce in the DC Feed Profile.
 * Applications that require detection of hook changes after polarity reversal in less than 120ms
 * will have to change this value. Applications that need to prevent spurious hook events for load
 * conditions of GR57 5REN (see "Ringing Capability) using 40uf + 1386ohm may need to increase this
 * value. Note however that 40uf + 1386ohm condition is for test purposes only, not representative
 * of real-world conditions.
 */
#define VP_POLREV_DEBOUNCE_TIME     (120)

/**< Line Timer Definitions/Indexes
 * Note that since the timer indexes themselves are generally used in incrememtal loops through
 * the line timer array, these values must be sequential, unsigned type
 */
/**< VP_LINE_CID_DEBOUNCE: Hook debounce timer for conditions determined by the CID State Machine */
#define VP_LINE_CID_DEBOUNCE            0

/**< VP_LINE_TIMER_CID_DTMF: Timer for DTMF generation in CID */
#define VP_LINE_TIMER_CID_DTMF          (VP_LINE_CID_DEBOUNCE + 1)

/**< VP_LINE_TIMER_FAULT: VE790 only fault management timer */
#define VP_LINE_TIMER_FAULT             (VP_LINE_TIMER_CID_DTMF + 1)

/**< VP_LINE_RING_EXIT_PROCESS: Timer used for Ringing Exit handing */
#define VP_LINE_RING_EXIT_PROCESS      (VP_LINE_TIMER_FAULT + 1)

    /**< VP_CSLAC_RINGING_EXIT_MAX
     * This value sets the maximum time the API will wait for the sillicon to exit it's own internal
     * Ringing Exit state (used for certain conditions) before forcing the line to a know value and
     * giving up on the timer. This value is monitored in the VE880/890 VP_LINE_RING_EXIT_PROCESS
     * timer management functions. The time specified is in mSec.
     */
    #define VP_CSLAC_RINGING_EXIT_MAX       (100)

/**< VP_LINE_HOOK_FREEZE:
 * Hook Mask when making any line change that could cause spurious hook activity. This time should
 * be used ONLY to mask hook activity, not as part of a control sequence. When it expires, the only
 * action remaining should be to read the signaling register and process hook activity.
 */
#define VP_LINE_HOOK_FREEZE         (VP_LINE_RING_EXIT_PROCESS + 1)

/**< VP_LINE_DISCONNECT_EXIT: Line Debounce for Disconnect Recovery */
#define VP_LINE_DISCONNECT_EXIT         (VP_LINE_HOOK_FREEZE + 1)

/**< VP_LINE_GND_START_TIMER: Timer for Recovery from Tip-Open */
#define VP_LINE_GND_START_TIMER         (VP_LINE_DISCONNECT_EXIT + 1)

/**< VP_LINE_CAL_LINE_TIMER: Timer used to sequence the CID state machine */
#define VP_LINE_CAL_LINE_TIMER          (VP_LINE_GND_START_TIMER + 1)

/**< VP_LINE_PING_TIMER: VE880 Rev VC only used to prevent phone "ping" when exiting disconnect */
#define VP_LINE_PING_TIMER              (VP_LINE_CAL_LINE_TIMER + 1)

/**< VP_LINE_TRACKER_DISABLE: Used to disable switcher with low power termination type. */
#define VP_LINE_TRACKER_DISABLE         (VP_LINE_PING_TIMER + 1)

/**< VP_LINE_GPIO_CLKOUT_TIMER: VE580 only time used to slow ringing signal clock output */
#define VP_LINE_GPIO_CLKOUT_TIMER       (VP_LINE_TRACKER_DISABLE + 1)

/**< VP_LINE_INTERNAL_TESTTERM_TIMER: Timer for the internal test termination */
#define VP_LINE_INTERNAL_TESTTERM_TIMER (VP_LINE_GPIO_CLKOUT_TIMER + 1)

/**< VP_LINE_SPEEDUP_RECOVERY_TIMER: Time used to manage the mettalic and battery speedup when
 * making certain state transitions for VE880 and VE890
 */
#define VP_LINE_SPEEDUP_RECOVERY_TIMER  (VP_LINE_INTERNAL_TESTTERM_TIMER + 1)

/**< VP_LINE_TIMER_LAST: Place holder to indicate number of line timers */
#define VP_LINE_TIMER_LAST              (VP_LINE_SPEEDUP_RECOVERY_TIMER + 1)

/**< Device Timer Definitions */
/* Test line timer to ensure a more consistant MPI PCM collect routine, leave
 * this timer as the first timer. */
#define VP_DEV_TIMER_TESTLINE   0

/* Clock fail interrupt timer - used only in 790 API */
#define VP_DEV_TIMER_CLKFAIL            (VP_DEV_TIMER_TESTLINE + 1)

/* ABS Calibration timer */
#define VP_DEV_TIMER_ABSCAL             (VP_DEV_TIMER_CLKFAIL + 1)

/* Lower Power Mode Switcher Changes */
#define VP_DEV_TIMER_LP_CHANGE          (VP_DEV_TIMER_ABSCAL + 1)

/* ABV Caibration  device timers */
#define VP_DEV_TIMER_ABV_CAL            (VP_DEV_TIMER_LP_CHANGE + 1)

/* In-rush limiting enter ringing device timers */
#define VP_DEV_TIMER_ENTER_RINGING      (VP_DEV_TIMER_ABV_CAL + 1)

/* Used on VE880-ABS only to time switching of the Supply Mode from HP (in Ringing) to MP */
#define VP_DEV_TIMER_EXIT_RINGING       (VP_DEV_TIMER_ENTER_RINGING + 1)

/* Used to mask clock faults */
#define VP_DEV_TIMER_WB_MODE_CHANGE     (VP_DEV_TIMER_EXIT_RINGING + 1)

/* Place holder to indicate number of device timers */
#define VP_DEV_TIMER_LAST               (VP_DEV_TIMER_WB_MODE_CHANGE + 1)

/** FXO specific timer variables to be used for FXO type lines only */
typedef enum VpCslacTimerType {
    VP_CSLAC_FXS_TIMER,
    VP_CSLAC_FXO_TIMER,
    VP_CSLAC_TIMER_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpCslacTimerType;


#if (defined (VP_CC_880_SERIES) && defined (VP880_FXO_SUPPORT)) || \
    (defined (VP_CC_890_SERIES) && defined (VP890_FXO_SUPPORT)) || \
     defined (VP_CC_580_SERIES) || defined (VP_CC_KWRAP)
typedef struct VpFXOTimerType {
    uint16  highToLowTime;      /**< Device timestamp of last high to low change */
    uint16  prevHighToLowTime;
    bool    noCount;            /**< TRUE when not counting - lack of activity */
    bool    lastState;          /**< TRUE if last known high, FALSE if low */
    uint16  timeLastPolRev;     /**< Time in 0.25ms since polrev detected */
    uint16  timePrevPolRev;     /**< Time in 0.25ms since prev polrev detected */
    uint8   maxPeriod;          /**< Time in 0.25ms that ringing is detected */
    uint16  lastStateChange;    /**< Time in 1mS since last state change */
    uint16  lastNotLiu;         /**< Time in 1mS since last Not LIU detected */
    uint16  disconnectDebounce; /**< Time in 1mS to debounce disconnect events */
    uint16  disconnectDuration; /**< Time in ticks that disconnect is detected */
    uint8   liuDebounce;        /**< Time in 1mS to debounce LIU detection that
                                     accompanies some kinds of ringing */
    uint8   ringOffDebounce;    /**< Time in 1mS to ignore disconnect after ring_off */
    uint8   ringTimer;          /**< When this timer expires, ringing is off */
    uint8   cidCorrectionTimer; /**< 1mS increments for CID correction timing */
    uint8   bCalTimer;          /**< 1mS increments for BFilter sampling time */
    uint16  fxoDiscIO2Change;   /**< Countdown in 1ms since IO2 changed for FXO_DISC termType */
    uint16  pllRecovery;        /**< Timer in ticks to recover PLL when FXO Disconnect to OHT */
    uint16  currentMonitorTimer;/**< Used to provide FXO line current buffer */
    uint16  measureBFilterTimer;/**< 1mS increments for BFilter sampling time */
    uint8   lowVoltageTimer;    /**< Timer in ticks for the low voltage
                                     disconnect/LIU distinction workaround */
} VpFXOTimerType;
#endif

/** Union of FXO and FXS timers since only one type can exist for each line */
typedef union VpCslacTimers {
#if (defined (VP_CC_880_SERIES) && defined (VP880_FXO_SUPPORT)) || \
    (defined (VP_CC_890_SERIES) && defined (VP890_FXO_SUPPORT)) || \
     defined (VP_CC_580_SERIES) || defined (VP_CC_KWRAP)
    VpFXOTimerType fxoTimer;
#endif
    uint16 timer[VP_LINE_TIMER_LAST]; /**< FXS Array of timers */
    uint16 trackingTime;    /**< Used in VE880/VE890 to track the total time the Ring Exit Timer
                             * has been running. The Ring Exit Timer is restarted while waiting for
                             * the silicon to make it's internal SLIC state change (i.e., get out of
                             * the Ring Exit State). But in case the SLIC never makes the expected
                             * state change, we want to force the final change and terminate the
                             * timer.
                             */
} VpCslacTimers;

typedef struct VpCslacTimerStruct {
    VpCslacTimerType type;
    VpCslacTimers timers;
} VpCslacTimerStruct;
#endif
