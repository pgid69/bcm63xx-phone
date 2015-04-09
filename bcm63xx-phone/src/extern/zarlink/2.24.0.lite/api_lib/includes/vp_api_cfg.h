/** \file vp_api_cfg.h
 * vp_api_cfg.h
 *
 * This file contains the configuration and compile time settings for
 * building appropriate VP-API library modules needed for any application.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11545 $
 * $LastChangedDate: 2014-09-10 17:52:11 -0500 (Wed, 10 Sep 2014) $
 */

#ifndef VP_API_CFG_H
#define VP_API_CFG_H

#include "vp_debug_masks.h"

/**< NOTE Regarding compile-time settings to reduce code size. The values
 * provided in these comments is for approximation and comparison purposes
 * only. Specific savings will vary depending on the compiler and target
 * processor.
 */

/**< VP_REDUCED_API_IF
 * Define this to remove the I/F for unsupported functions. This reduces
 * code size with the only limitation that the application cannot call
 * functions that would return "Unsupported" for all devices used in the
 * application. The specific functions removed will depend on the device and
 * test packages compiled in.
 */
#define VP_REDUCED_API_IF

/******************************************************************************
 * COMPILE-TIME OPTIONS:: API DEBUG OUTPUT SELECTIONS                         *
 *****************************************************************************/

/**< VP_DEBUG
 * If VP_DEBUG is undefined, then ALL debug messages will be compiled out. In
 * this case, the code size will be smaller, but debug output CANNOT be enabled
 * at runtime.
 */
#undef VP_DEBUG

/**< VP_CC_DEBUG_SELECT
 * Choose which types of debug messages to be compiled in.  Only these message
 * types can be enabled at runtime. See vp_debug_masks.h for a list of debug
 * message types.
 */
#define VP_CC_DEBUG_SELECT (VP_DBG_ALL)

/**< VP_OPTION_DEFAULT_DEBUG_SELECT
 * The VP_OPTION_ID_DEBUG_SELECT option enables/disables debug output at
 * runtime.  Define the default value of this option here.  The option will be
 * automatically set to the default value at device initialization.  See
 * vp_debug_masks.h for a list of debug message types.
 *
 * Note: The VP_OPTION_ID_DEBUG_SELECT option can be changed with VpSetOption()
 *       only AFTER device initialization (VpInitDevice()).  If you wish to
 *       see debug output messages during device initialization, you must set
 *       the default VP_OPTION_ID_DEBUG_SELECT value accordingly.
 */
#define VP_OPTION_DEFAULT_DEBUG_SELECT (VP_DBG_ERROR | VP_DBG_WARNING)

/******************************************************************************
 * COMPILE-TIME OPTIONS:: Conditionally-Compiled API Libraries                *
 *****************************************************************************/

/*
 * Define (or undefine) the appropriate compile time switches based on your
 * application needs.
 *
 * NOTE: Such library selection compile time option(s) MUST be defined before
 * including any other file from VoicePath library.
 *
 * NOTE: More than one Library modules can be built simultaneosuly (if needed).
 */
#ifndef ZARLINK_CFG_INTERNAL


  /*
   * Define the API libraries that will be included in this build
   * only if the kernel wrapper library is not supported.
   */

  #undef VP_CC_790_SERIES   /**< define to build 790 specific API library;
                              *   undef to exclude this library. */

  #undef VP_CC_792_SERIES   /**< define to build 792 specific API library;
                              *   undef to exclude this library. */

  #define VP_CC_880_SERIES   /**< define to build 880 specific API library;
                              *   undef to exclude this library. */

  #undef VP_CC_886_SERIES   /**< define to build 886/887 specific API library;
                             *   undef to exclude this library. */

  #undef VP_CC_890_SERIES   /**< define to build 890 specific API library;
                              *   undef to exclude this library. */

  #undef VP_CC_580_SERIES   /**< define to build 580 specific API library;
                              *   undef to exclude this library. */

  /* VCP (VCP) library */
  #undef VP_CC_VCP_SERIES   /**< define to build VCP specific API library;
                              *   undef to exclude this library. */
  /* VCP2 library */
  #undef VP_CC_VCP2_SERIES  /**< define to build VCP2 specific API library;
                              *   undef to exclude this library. */

  #undef VP_CC_MELT_SERIES  /**< define to build Melt specific API library;
                              *   undef to exclude this library. */

  #undef VP_CC_KWRAP

  #if defined(VP_CC_KWRAP) && defined(__KWRAP__)
    #error "VP_CC_KWRAP is only applicable to user space builds"
  #endif

#endif


/******************************************************************************
 * Include Files for the API                                                  *
 *****************************************************************************/
/* Include the API types for this architecture */
#include "vp_api_types.h"

/*
 * Undef the following to remove the code necessary to run the API-II sequencer
 * for CSLAC devices and to enable compile of API-II Lite. This is useful to
 * reduce code space if the host is not using API-II "advanced" features.
 */
#if defined(VP_CC_790_SERIES) || defined(VP_CC_880_SERIES) || \
    defined(VP_CC_890_SERIES) || defined(VP_CC_580_SERIES) || \
    defined(VP_CC_886_SERIES)
    /**< VP_CSLAC_SEQ_EN
     * This is used to support all cadencing type operations on both FXO and
     * FXS line types. This includes Ringing and Tone Cadencing, Metering
     * Pulse, and VpSendSignal(). When set to #undef the VE890 code reduces by
     * ~17K, the VE880 code reduces by ~20K.  The ZL880 code reduces by ~13K.
     */
#undef VP_CSLAC_SEQ_EN
#endif

#if defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) || defined(VP_CC_890_SERIES)
    /**< VP_CSLAC_RUNTIME_CAL_ENABLED
     * This is used to support run-time calibration via VpInitDevice() and
     * VpCalLine(). With this #undef, System Coefficients MUST be applied
     * using VpCal(). Set to #undef saves VE880 code size by ~19K bytes (if
     * both Tracker and ABS are supported) and VE890 by ~10K bytes.
     */
    #define VP_CSLAC_RUNTIME_CAL_ENABLED

    /**< VP_HIGH_GAIN_MODE_SUPPORTED
     * This is used to support High Gain States VP_LINE_HOWLER and
     * VP_LINE_HOWLER_POLREV. Set to #undef saves code size for VE880, ZL880 and
     * VE890 by ~2K bytes.
     */
    #undef VP_HIGH_GAIN_MODE_SUPPORTED

    /**< UK Howler Tone Specifications
     * VE880, ZL880, VE890 Applications only. The values below affect how UK
     * Howler Tone will be provided on the line. To send UK Howler Tone the
     * application must do the following:
     *
     *      - Have the full version of VP_API-II (i.e., Le71SK0002)
     *      - Set VP_CLSAC_SEQ_EN to #define. Note: if setting VP_CLSAC_SEQ_EN
     *        to #define causes compiler errors, the Make process may either
     *        not be including the required *.c and *.h files or may be using
     *        the "Lite" version of the API (i.e., Le71SDKAPIL).
     *      - Create a Tone Cadence Profile with UK Howler Tone.
     *      - Provide this tone cadence to VpSetLineTone()
     *
     * There are two types of UK Howler Tones supported in the API:
     *
     *      - VP_UK_HOWLER_BTNR_VER15
     *          The BTNR Version 15 Howler Tone will perform a frequency sweep
     *          between [800Hz to 3200Hz] at a 500ms period. Total sweep from
     *          [800-3200-800] will be nominal 1 second +/-166ms. At each 1
     *          second interval over a total duration of 12 seconds the tone
     *          level will increase by 3dB for total level increase of 36dB.
     *          The final level will be a nominal +14.5dBm @ 600ohm on T/R
     *          when also used with VP_LINE_HOWLER or VP_LINE_HOWLER_POLREV
     *          line states (see VpSetLineState() in API Reference Guide).
     *
     *      - VP_UK_HOWLER_BTNR_DRAFT_G
     *          Draft 960-G Howler Tone is very similar to Version 15 with
     *          only a few differences. The frequency sweep is between [800Hz
     *          to 2500Hz] and the level varies by 3dB during the sweep
     *          interval such that the level at 2500Hz is ~3dB higher than the
     *          level at 800Hz. The total sweep time of 12 seconds is the same
     *          as well as the final output level of +14.5dBm @ 600ohm on T/R
     *          when also used with VP_LINE_HOWLER or VP_LINE_HOWLER_POLREV
     *          line states (see VpSetLineState() in API Reference Guide).
     *
     * The Application may choose which BTNR Specification to use when
     * generating the UK Howler Tone by setting VP_UK_HOWLER_IN_USE value as
     * follows:
     *
     *      To use BTNR Version 15:
     *          #define VP_UK_HOWLER_IN_USE VP_UK_HOWLER_BTNR_VER15
     *
     *      To use BTNR Draft-960 G:
     *          #define VP_UK_HOWLER_IN_USE VP_UK_HOWLER_BTNR_DRAFT_G
     *
     *      Previous releases (P2.18.0 and earlier) provided UK Howler Tone
     *      per VP_UK_HOWLER_BTNR_VER15.
     */
    #define VP_UK_HOWLER_BTNR_VER15     0
    #define VP_UK_HOWLER_BTNR_DRAFT_G   1
    #define VP_UK_HOWLER_IN_USE         VP_UK_HOWLER_BTNR_VER15

#endif  /* defined(VP_CC_880_SERIES) || defined(VP_CC_886_SERIES) || defined(VP_CC_890_SERIES) */

/**< EXTENDED_FLASH_HOOK
 * Define the following to include the onHookMin member in the
 * VpOptionPulseType struct.  Please note, this member (even if enabled here)
 * is ignored in VCP1-790 applications.
 */
#undef EXTENDED_FLASH_HOOK

/**< VP_ENABLE_OFFHOOK_MIN
 * Define the following to include the offHookMin member in the
 * VpOptionPulseType struct for the VP_DEVICE_OPTION_ID_PULSE option to enable
 * VP_LINE_EVID_HOOK_PREQUAL events.
 */
#undef VP_ENABLE_OFFHOOK_MIN


/******************************************************************************
 * Library Specific COMPILE-TIME OPTIONS and defines                          *
 *****************************************************************************/
#ifdef VP_CC_790_SERIES
/* CSLAC library can be configured to handle the interrupt in four different
 * configurations. Choose (only) one of the mechanisms by defining one of the
 * following. */
#define VP790_SIMPLE_POLLED_MODE        /* Define to use simple polled interrupt
                                         * handling mode */
#undef VP790_EFFICIENT_POLLED_MODE      /* Define to use efficient polled
                                         * interrupt handling mode */
#undef VP790_INTERRUPT_LEVTRIG_MODE     /* Define to use level triggered
                                         * interrupt handling mode */
#undef VP790_INTERRUPT_EDGETRIG_MODE    /* Define to use edge triggered
                                         * interrupt handling mode */
#endif /* VP_CC_790_SERIES */

#ifdef VP_CC_792_SERIES

#undef VP_COMMON_ADDRESS_SPACE          /**< Define if all VP-API-II data
                                         * structures (Line Objects, Line
                                         * Contexts, Device Objects, Device
                                         * Contexts, Profiles) are accessible
                                         * at the same address in all processes
                                         * (tasks) which call VP-API-II
                                         * functions.
                                         */

#define VP_CC_792_GROUP                 /**< Define to include support for
                                         * sharing an interrupt pin between
                                         * multiple VP792 devices.
                                         */

#define VP792_MAILBOX_SPINWAIT 50000    /**< Number of times to poll the
                                         * device's command mailbox before
                                         * returning VP_STATUS_MAILBOX_BUSY.
                                         */

#define VP792_INCLUDE_TESTLINE_CODE     /**< Defines whether or not to enable
                                         * 792 line test capabilities
                                         */

#define VP792_SUPPORT_792388_REV_E_SILICON  /**< Define this if you want to
                                             * support Revision E of the
                                             * Le792388 silicon. An appropriate
                                             * SLAC firmware patch will be
                                             * compiled in.
                                             */
#define VP792_SUPPORT_792388_REV_F_SILICON  /**< Define this if you want to
                                             * support Revision F of the
                                             * Le792388 silicon. An appropriate
                                             * SLAC firmware patch will be
                                             * compiled in. */
#define VP792_SUPPORT_792588_REV_A_SILICON  /**< Define this if you want to
                                             * support Revision A of the
                                             * Le792588 silicon. An appropriate
                                             * SLAC firmware patch will be
                                             * compiled in.
                                             */
#define VP792_SUPPORT_792588_REV_B_SILICON  /**< Define this if you want to
                                             * support Revision B of the
                                             * Le792588 silicon. An appropriate
                                             * SLAC firmware patch will be
                                             * compiled in.
                                             */
#endif /* VP_CC_792_SERIES */

#ifdef VP_CC_880_SERIES

/*******************************************************************************
 * Applications can reduce VP-API-II driver size by changing the following for
 * only the required silicon and termination types needed
 *
 * See VP-API-II User's Guide for Details.
 ******************************************************************************/
/**< Device Type Support.
 * One of these MUST be defined if FXS Support is enabled. If you're not sure
 * whether the silicon used in your application is ABS or Tracker, contact
 * Microsemi customer support.
 *
 * Note that disabling either ABS or Tracker removes the associated calibration
 * and line specific support (e.g., if disabling Tracker, then LPM also must be
 * disabled).
 */
#define VP880_ABS_SUPPORT   /**< VP880_ABS_SUPPORT
                             *  #define this to support VE880 ABS type silicon.
                             * This includes but not limited to Le88266/86 and
                             * Le8822x/24x. When set to #undef reduces code
                             * size by ~9K bytes.
                             */

#undef VP880_TRACKER_SUPPORT   /**< VP880_TRACKER_SUPPORT
                                 * #define this to support VE880 Tracker type
                                 * silicon. This includes but not limited to
                                 * VE8820 Chipset and Le88276.
                                 */

/**< VE880 Termination Type Support.
 *
 * Note: Both ABS and Tracker can be #undef while FXO defined. This results in
 * support for FXO only silicon, and FXO/FXS silicon support for the FXO
 * termination type only. FXS support requires either ABS or Tracker support.
 */
#undef VP880_FXO_SUPPORT
#define VP880_FXS_SUPPORT

/**< VP880_LP_SUPPORT
 * When set to #undef, all of the code in the VE880 API that supports
 * termination types VP_TERM_FXS_LOW_PWR, VP_TERM_FXS_SPLITTER_LP, and
 * VP_TERM_FXS_ISOLATE_LP will be removed. When disabled, the VE880 code
 * size is reduced by ~7K bytes.
 */
#undef VP880_LP_SUPPORT

/**< Specific FXO Algorithm support.
 *
 * The VP880_CLARE Ringing Detect algorithm uses features of the Clare and VE880
 * silicon to provide maximum performance of signal detection on FXO lines.
 * Alternative methods (with VP880_CLARE_RINGING_DETECT set to #undef) do not
 * require connection of I/O4 to Clare device, but have the following drawbacks:
 *
 *    1. Ringing Signals MUST contain polarity reversal to be detected AND
 *       must exceed ~70V peak.
 *    2. Line-In-Use is disabled.
 *    3. Frequency Discriminaation is poor (limited to tickrate).
 *
 * It is recommended that all designs connect the CLARE Ringing output to IO4
 * of the silicon. In that case, this value should be #define.
 *
 * Note: If FXO Support is disabled, this setting has no affect.
 */
#define VP880_CLARE_RINGING_DETECT

/**< VP880_FXO_FULL_WAVE_RINGING
 * Selects whether the Ringing input is 1/2-wave of full-wave. This must be
 * set to #undef when using CPC5621 since this device provides only 1/2-wave
 * ringing. For all other Clare devices, set to #define.
 */
#define VP880_FXO_FULL_WAVE_RINGING

/**< VP880_ALWAYS_USE_INTERNAL_TEST_TERMINATION
 * Define this option to always use the internal test termination for relay
 * state VP_RELAY_BRIDGED_TEST, even for devices that have the test load
 * switch. This is Microsemi's recommendation since the Internal Test
 * Termination provides better isolation from the customer load and loop
 * conditions when compared to the external test load.
 */
#undef VP880_ALWAYS_USE_INTERNAL_TEST_TERMINATION

/* CSLAC library can be configured to handle the interrupt in four different
 * configurations. Choose (only) one of the mechanisms by defining one of the
 * following. */
#define VP880_SIMPLE_POLLED_MODE        /* Define to use simple polled interrupt
                                         * handling mode */
#undef  VP880_EFFICIENT_POLLED_MODE     /* Define to use efficient polled
                                         * interrupt handling mode */
#undef  VP880_INTERRUPT_LEVTRIG_MODE    /* Define to use level triggered
                                         * interrupt handling mode */
#undef  VP880_INTERRUPT_EDGETRIG_MODE   /* Define to use edge triggered
                                         * interrupt handling mode */
#undef VP880_INCLUDE_TESTLINE_CODE     /* Defines whether or not to enable
                                         * 880 line test capabilities */

/**< VE880 Line Test Package Defines
 * Defines all possible VE880 test line packages. There should be no reason to
 * change these values. But if they are changed, they must be different from
 * each other.
 */
#define VP880_LINE_TEST_AUDITOR 1
#define VP880_LINE_TEST_PROFESSIONAL 2

#ifdef VP880_INCLUDE_TESTLINE_CODE

/**< VP880_INCLUDE_LINE_TEST_PACKAGE
 * Selects the VE880 Line Test Package to include. Note that both Auditor and
 * Professional cannot be included in the same build.
 */
#define VP880_INCLUDE_LINE_TEST_PACKAGE VP880_LINE_TEST_PROFESSIONAL

/**< VP880_EZ_MPI_PCM_COLLECT
 * #define this to have the api collect PCM samples via the MPI bus.
 * If #undef, the Line Test code will expect the PCM samples from an
 * external process (DSP or uProcessor) connected to the PCM BUS
 */
#define  VP880_EZ_MPI_PCM_COLLECT

/**< VP880_PCM_CALCULATION_TIME
 * Define the maximum expected computation time to arrive at results for
 * VpTestLineCallback() after having collected the necessary PCM samples.
 */
#define VP880_PCM_CALCULATION_TIME 1000     /* time is in ms */

/**< VP880_INCLUDE_MPI_QUICK_TEST
 * #define to enable a quick test of the MPI interface and HAL implementation
 * during VpInitDevice().  This can be #undef to save on code size and MPI
 * traffic at initialization after a design is proven to work.
 */
#define VP880_INCLUDE_MPI_QUICK_TEST

#endif /* VP880_INCLUDE_TESTLINE_CODE */

#endif /* VP_CC_880_SERIES */

#ifdef VP_CC_580_SERIES
/* CSLAC library can be configured to handle the interrupt in four different
 * configurations. Choose (only) one of the mechanisms by defining one of the
 * following. */
#define VP580_SIMPLE_POLLED_MODE        /* Define to use simple polled interrupt
                                         * handling mode */
#undef VP580_EFFICIENT_POLLED_MODE      /* Define to use efficient polled
                                         * interrupt handling mode */
#undef VP580_INTERRUPT_LEVTRIG_MODE     /* Define to use level triggered
                                         * interrupt handling mode */
#undef VP580_INTERRUPT_EDGETRIG_MODE    /* Define to use edge triggered
                                         * interrupt handling mode */
#endif

#ifdef VP_CC_890_SERIES

/**< VE890 Generic Termination Type Support */
#define VP890_FXO_SUPPORT
#define VP890_FXS_SUPPORT

/**< VP890_LP_SUPPORT
 * When set to #undef, all of the code in the VE890 API that supports
 * termination types VP_TERM_FXS_LOW_PWR, VP_TERM_FXS_SPLITTER_LP, and
 * VP_TERM_FXS_ISOLATE_LP will be removed. When disabled, the VE890 code
 * size is reduced by ~5K bytes.
 */
#define VP890_LP_SUPPORT

#undef VP890_INCLUDE_TESTLINE_CODE     /* Defines whether or not to enable
                                         * 890 line test capabilities */
#ifdef VP890_INCLUDE_TESTLINE_CODE

/**< VE890 Line Test Package Defines
 * Defines all possible VE890 test line packages. There should be no reason to
 * change these values. But if they are changed, they must be different from
 * each other.
 */
#define VP890_LINE_TEST_AUDITOR 1
#define VP890_LINE_TEST_PROFESSIONAL 2

/**< VP890_INCLUDE_LINE_TEST_PACKAGE
 * Selects the VE890 Line Test Package to include. Note that both Auditor and
 * Professional cannot be included in the same build.
 */
#define VP890_INCLUDE_LINE_TEST_PACKAGE VP890_LINE_TEST_PROFESSIONAL

/**< VP890_EZ_MPI_PCM_COLLECT
 * #define this to have the api collect PCM samples via the MPI bus.
 * If #undef, the Line Test code will expect the PCM samples from an
 * external process (DSP or uProcessor) connected to the PCM BUS
 */
#define VP890_EZ_MPI_PCM_COLLECT

/**< VP890_PCM_CALCULATION_TIME
 * Define the maximum expected computation time to arrive at results for
 * Vp890TestLineCallback() after having collected the necessary PCM samples.
 */
#define VP890_PCM_CALCULATION_TIME 1000     /* time is in ms */

#endif /* VP890_INCLUDE_TESTLINE_CODE */

#define VP890_SIMPLE_POLLED_MODE        /* Define to use simple polled interrupt
                                         * handling mode */
#undef  VP890_EFFICIENT_POLLED_MODE     /* Define to use efficient polled
                                         * interrupt handling mode */
#undef  VP890_INTERRUPT_LEVTRIG_MODE    /* Define to use Level Triggered
                                         * interrupt handling mode */
#undef  VP890_INTERRUPT_EDGETRIG_MODE   /* Define to use edge triggered
                                         * interrupt handling mode */

/**< VP890_INCLUDE_MPI_QUICK_TEST
 * #define to enable a quick test of the MPI interface and HAL implementation
 * during VpInitDevice().  This can be #undef to save on code size and MPI
 * traffic at initialization after a design is proven to work.
 */
#define VP890_INCLUDE_MPI_QUICK_TEST

#endif /* VP_CC_890_SERIES*/


#ifdef VP_CC_886_SERIES

#undef VP886_INCLUDE_TESTLINE_CODE      /* Defines whether or not to enable
                                         * 886 line test capabilities */

/* Select one of the following four options which corresponds to the
   application's interrupt handling mode.  The selection made here will
   determine which system service layer functions VpApiTick() calls.  The
   VP-API-II may also use this information to improve performance regardless of
   whether the application uses VpApiTick().
   NOTE: Applications which are triggered on interrupt edges MUST define
         VP886_INTERRUPT_EDGETRIG_MODE, even if not using VpApiTick(). */
#undef  VP886_SIMPLE_POLLED_MODE        /* Define to use simple polled interrupt
                                         * handling mode */
#define VP886_EFFICIENT_POLLED_MODE     /* Define to use efficient polled
                                         * interrupt handling mode */
#undef  VP886_INTERRUPT_LEVTRIG_MODE    /* Define to use Level Triggered
                                         * interrupt handling mode */
#undef  VP886_INTERRUPT_EDGETRIG_MODE   /* Define to use edge triggered
                                         * interrupt handling mode */
/* Ensure that only one polling mode is selected */
#if (defined(VP886_SIMPLE_POLLED_MODE) && defined(VP886_EFFICIENT_POLLED_MODE)) || \
    (defined(VP886_SIMPLE_POLLED_MODE) && defined(VP886_INTERRUPT_LEVTRIG_MODE)) || \
    (defined(VP886_SIMPLE_POLLED_MODE) && defined(VP886_INTERRUPT_EDGETRIG_MODE)) || \
    (defined(VP886_EFFICIENT_POLLED_MODE) && defined(VP886_INTERRUPT_LEVTRIG_MODE)) || \
    (defined(VP886_EFFICIENT_POLLED_MODE) && defined(VP886_INTERRUPT_EDGETRIG_MODE)) || \
    (defined(VP886_INTERRUPT_LEVTRIG_MODE) && defined(VP886_INTERRUPT_EDGETRIG_MODE))
#error "Only one VP886 polling mode may be defined."
#endif

#undef  VP886_INCLUDE_ADAPTIVE_RINGING  /* Define to enable Thermal
                                         * Ringing Algorithm */

#define VP886_EVENT_QUEUE_SIZE 6

/* Set this to a value greater than 0 to allow use of the VpGenTimerCtrl()
   function for VP886.  The value will be the limit of USER timers allowed
   per device.  Larger values will result in larger device objects. */
#define VP886_USER_TIMERS 2

/* The VP-API-II sequencer code for older devices took one tick to process each
   instruction in a cadence profile.  This meant that delays were extended
   by the number of instructions processed in between them.  The ZL880/VP886
   sequencer code, by default, can execute cadence profile instructions without
   delay until it reaches a delay instruction.  This causes a minor difference
   in the how cadences play out.  The VP886_LEGACY_SEQUENCER will force the
   VP886 sequencer to insert a delay in between each cadence instruction to
   emulate the old behavior.
   This option should only be used if both of these conditions are true:
    - The application is using cadence profiles that are specifically designed
      to account for the tick delay.
    - The application is polling at the same consistent tick rate as for the
      older device.
   This option may produce undesirable behavior if used in a non-ticked
   interrupt-driven mode. */
#undef VP886_LEGACY_SEQUENCER

/* VP886_INCLUDE_MPI_QUICK_TEST
   define to enable a quick test of the MPI interface and HAL implementation
   during VpInitDevice().  This can be #undef to save on code size and MPI
   traffic at initialization after a design is proven to work. */
#define VP886_INCLUDE_MPI_QUICK_TEST

/* VP886_INCLUDE_DTMF_DETECT
   Define to include support for VP_OPTION_ID_DTMF_MODE for ZL880/miSLIC
   devices.  #undef to save on code size. */
#define VP886_INCLUDE_DTMF_DETECT

/* Set the DTMF detection sampling rate.  4kHz is recommended, as it requires
   much less SPI traffic and processor time.  8kHz has not been tested as much,
   and provides only minimally better detection.
   One mode must be selected. */
#define VP886_DTMF_DETECT_4KHZ_SAMPLING
#undef VP886_DTMF_DETECT_8KHZ_SAMPLING

/* Define to use non-continuous sampling mode.  This relaxes all of the timing
   requirements for functional DTMF detection.  It also means that the time
   taken to detect a change in digit status will increase, and will vary
   depending on the tickrate or interrupt handling delay.  In an interrupt
   driven system this mode will use less MPI traffic.  This mode allows for
   tickrates slower than 10ms.  Each detection block is 13.25ms, so an ideal
   tick rate would be only slightly longer than that (e.g. 15ms) to minimize
   the gaps between blocks.
   Only works with 4kHz sampling. */
#undef VP886_DTMF_DETECT_NONCONTINUOUS

/* Error-checking of DTMF_DETECT definitions */
#ifdef VP886_INCLUDE_DTMF_DETECT
    #if defined(VP886_DTMF_DETECT_4KHZ_SAMPLING) && defined(VP886_DTMF_DETECT_8KHZ_SAMPLING)
        #error Must not define both VP886_DTMF_DETECT_4KHZ_SAMPLING or VP886_DTMF_DETECT_8KHZ_SAMPLING
    #elif !defined(VP886_DTMF_DETECT_4KHZ_SAMPLING) && !defined(VP886_DTMF_DETECT_8KHZ_SAMPLING)
        #error Must define either VP886_DTMF_DETECT_4KHZ_SAMPLING or VP886_DTMF_DETECT_8KHZ_SAMPLING
    #elif defined(VP886_DTMF_DETECT_NONCONTINUOUS) && !defined(VP886_DTMF_DETECT_4KHZ_SAMPLING)
        #error Must define VP886_DTMF_DETECT_4KHZ_SAMPLING when using VP886_DTMF_DETECT_NONCONTINUOUS
    #endif
#endif

#endif /* VP_CC_886_SERIES */


/*
 * The following lines include necessary compile time options/definitions that
 * are required for boot loading VCP/VCP2 devices.  By default, compression
 * is not supported for boot images.  If you would to use compressed boot
 * images, the compile time options in the following lines need to be modified.
 */

#if !defined(VP_CC_VCP_SERIES) && !defined(VP_CC_VCP2_SERIES) && \
    !defined(VP_CC_MELT_SERIES) && !defined(VP_CC_KWRAP)
typedef uint8 VpScratchMemType;
#else
/* All the following definitions are necessary only for VCP/VCP2 devices */
/******************************************************************************
 * COMPILE-TIME OPTIONS:: Boot load compression method and size               *
 *****************************************************************************/
#define VP_CLEAR_CODE_MEM
    /*
     * After VpBootLoad(), a VP_LINE_EVID_BOOT_CMP event is generated, and
     * VpGetResults() is then used to retrieve the loadChkSum value, which is
     * a checksum calculated over the VCP's code memory.  VpCodeCheckSum()
     * can then be called at any later time to verify that VCP code memory
     * has not been corrupted.
     *
     * With VP_CLEAR_CODE_MEM defined, VpBootLoad() clears all VCP code memory
     * to zeroes before loading the boot image.
     * With VP_CLEAR_CODE_MEM disabled, unused VCP code memory is not
     * initialized after a hardware reset.  However, the loadChkSum is
     * calculated over the entire VCP code memory space, including uninitialized
     * parts.  If it is important to you that the loadChkSum value stay the
     * same after a hardware reset, then enable VP_CLEAR_CODE_MEM.
     *
     * This option adds a small amount of execution time to VpBootLoad(), when
     * state = VP_BOOT_STATE_FIRST or VP_BOOT_STATE_FIRSTLAST.
     */

/*
 * Options related to BootLoad() decompression
 */
#undef VP_NO_COMPRESS
    /* Define this option to exclude all decompression code from VpBootLoad().*/
#define VP_COMPRESS_FAST
    /*
     *  Define this to include additional speed-optimized code to increase the
     * speed of VpBootLoad().
     */

    /*
     *  On our test platform, code size is affected by the above options as
     * shown:
     *  VP_NO_COMPRESS      VP_COMPRESS_FAST        added code size
     *  --------------      ----------------        ---------------
     *  #define             (don't care)            0
     *  #undef              #undef                  9.6 kB
     *  #undef              #define                 12.2 kB
     */

    /*
     * The following options determine the size of VpScratchMemType, and the
     * speed of VpBootLoad() when using a compressed VCP image.
     */
#define VP_COMPRESS_WINDOW_BITS 8
    /*
     * There is no advantage in increasing this to a value greater than the
     * value used when the VCP image was compressed.  Setting it to less than
     * the value used in compression will cause VpBootLoad() to fail.  [The
     * number of window bits used in compression is indicated in the VCP
     * image's filename; for example, dvp.bin10z was compressed with 10 window
     * bits, etc.  If the number of bits is not indicated (*.binz), then
     * 8 bits were used.]  Higher values improve compression.  Don't
     * change this value unless you change the corresponding value used
     * in compressing the VCP image.  Adds (1 << VP_COMPRESS_WINDOW_BITS)
     * bytes to sizeof(VpScratchMemType).  This option can range from
     * 8 to 15.  */
#define VP_COMPRESS_OUTBUF_PAGES 50
    /*
     * Size of the buffer for holding data after it is decompressed, and before
     * it is sent to the HBI bus.  Must be greater than 0 and less than 512.
     * Larger values increase the speed of VpBootLoad().  If VP_COMPRESS_FAST
     * is defined, VP_COMPRESS_OUTBUF_PAGES should be at least 3.
     * Adds (128 * VP_COMPRESS_OUTBUF_PAGES) bytes to sizeof(VpScratchMemType).
     */

#ifndef VP_NO_COMPRESS
#include "zstructs.h"    /**< Components of VpScratchMemType */
typedef struct {
    struct VpZlibInflateStateType state;
    VpZlibStreamType stream;
    uint8 window[1 << VP_COMPRESS_WINDOW_BITS];
    uint8 inflated[VP_COMPRESS_OUTBUF_PAGES * 128];
} VpScratchMemType;
#else /* VP_NO_COMPRESS */
typedef uint8 VpScratchMemType;
#endif /* VP_NO_COMPRESS */
#endif  /* !VP_CC_VCP_SERIES */

/******************************************************************************
 *    VCP2 library specific other defines and conditional compile flags       *
 ******************************************************************************/
/*
 * Normally, when a VP-API function needs to write to the VCP2's command
 * mailbox, it checks to see if the mailbox is busy, and if so, returns
 * VP_STATUS_MAILBOX_BUSY.  The application can then wait for a
 * VP_DEV_EVID_DNSTR_MBOX ("mailbox free") event before retrying the VP-API
 * function call.
 *
 * Use the following option to include code which spin-waits for the VCP2 to
 * relinquish control of the command mailbox.  In this case, the VP-API
 * function will continuously poll the VCP2 device until the mailbox becomes
 * available.  The number of polling iterations is specified in the following
 * option.  If this option is 0, no spin-waiting occurs.  If this option is
 * nonzero, the VP-API functions will return VP_STATUS_MAILBOX_BUSY only if
 * unable to acquire control of the mailbox after the specified number of
 * polling iterations.
 *
 * Application programming may be simplified by defining this option to a
 * conservative value and treating VP_STATUS_MAILBOX_BUSY as an error.
 * However, this is at the expense of increased CPU utilization.
 */
#define VP_VCP2_MAILBOX_SPINWAIT 2000000UL

/* When bootloading a VCP2-792, the API uses a special bootloading process to
 * load the firmware for both the VCP2 and the 792 SLACs.  This process
 * spinwaits for the command mailbox between sending blocks of the SLAC image.
 * The spinwait limit here should be a higher number than the general limit, and
 * should not be zero because we actually expect to wait in this case, and a
 * timeout aborts the SLAC bootloading. */
#define VP_VCP2_BOOTLOAD_MAILBOX_SPINWAIT 20000000UL

/* When spinwaiting for the command mailbox using either of the above
 * parameters, the VP-API can exit and re-enter the critical section between
 * each specified number of reads of the mailbox flags.  Doing this should
 * allow other threads to interrupt while spinwaiting.  Set this value to 0
 * to disable the exit/enter critical behavior. */
#define VP_VCP2_SPINS_PER_CRITICAL_SEC 1

/* When spinwaiting for the command mailbox, it may be desirable to detect a
 * loss of communication with the VCP and exit the spinwait.  If this option
 * is enabled, the VP-API will perform a write-read HBI check along with
 * reading the mailbox status flags.  The function will return status code
 * VP_STATUS_ERR_HBI if the check fails. */
#undef VP_VCP2_MAILBOX_ACQUIRE_HBI_CHECK

/*
 * Define the following option to include some extra error checking in the
 * VCP2 boot code.  This is most useful to diagnose board problems during
 * initial board bring-up.
 */
#define VP_VCP2_BOARD_BOOT_DEBUG

/*
 * Set this option to reflect the CLKSRC pin value.  Possible values:
 *   1 => crystal oscillator
 *   0 => PCLK (worst case value)
 */
#define VP_VCP2_BOARD_CLKSRC 0
#define VP_MELT_BOARD_CLKSRC 0
/*
 * Set these options to initialize the PCLKA_SEL and PCLKB_SEL registers.  See
 * the Le79114 data sheet for details.  A safe value is 0xC000.  These options
 * are only relevant if VCP2_BOARD_CLKSRC = 0.
 */
#define VP_VCP2_BOARD_PCLKA_SEL 0xC000
#define VP_VCP2_BOARD_PCLKB_SEL 0xC000

/******************************************************************************
 *     VCP library specific other defines and conditional compile flags       *
 ******************************************************************************/

/* Define the following to wait for certain amount of time for VCP mailbox
 * to become available. By enabling this conditional compile time option,
 * application would rarely have to handle the VP_STATUS_MAILBOX_BUSY error
 * (assuimg configured properly; more later). Basically when this conditional
 * compile option is defined, API repeats checking for mailbox ownership (if
 * it already does not own). This checking continues until either mailbox
 * ownership is acquired or maximum specified number of repetitions happen (in
 * which case API throws an error).
 * If this compile flag is undefined, API checks for mailbox only once.
 */
#define WAIT_TO_ACQUIRE_VCP_MB

/* Define the number iterations host should wait for mailbox owenership */
#define ITERATIONS_TO_WAIT_FOR_VCP_MB      (100000)

#ifdef  VP_CC_MELT_SERIES
/* These are Melt equivalents for some similarly named VCP2 defines so that we can
 * keep the two api's separate.
 */
#define VP_MELT_MAILBOX_SPINWAIT 2000000UL

#define VP_MELT_BOOTLOAD_MAILBOX_SPINWAIT 20000000UL

#define VP_MELT_BOARD_PCLKA_SEL 0xC000
#define VP_MELT_BOARD_PCLKB_SEL 0xC000

#endif  /* VP_CC_MELT_SERIES */

/* Include internal options required to build the VP-API-II library */
#include "vp_api_cfg_int.h"

#endif /* VP_API_CFG_H */


