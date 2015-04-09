/*
 * ZLR88621L_ABS_profiles.c --
 *
 * This file contains profile data in byte format
 *
 * Project Info --
 *   File:   X:\shafer\lightning\ztap2012\ztap\applications\quickstarts\sdk_examples\ZL880_initialization\ZLR88621_ABS_profiles.vpw
 *   Type:   Design for ZLR88621L SM2 Line Module Featuring the ZL88601, Lite Coefficients
 *   Date:   Monday, August 06, 2012 17:55:09
 *   Device: ZL880 ZL88601
 *
 *   This file was generated with Profile Wizard Version: P2.3.0
 */

#include "ZLR88621L_ABS_profiles.h"


/************** Device_Parameters **************/

/* Device Configuration Data - ZL88601 100V ABS (-81V/-27V) */
const VpProfileDataType ABS_DEV_PROFILE[] =
{
/* Profile Header ---------------------------------------------------------- */
   0x0C, 0xFF, 0x00,       /* Device Profile for ZL88601 device */
   0x29,                   /* Profile length = 41 + 4 = 45 bytes */
   0x03,                   /* Profile format version = 3 */
   0x14,                   /* MPI section length = 20 bytes */
/* Raw MPI Section --------------------------------------------------------- */
   0x46, 0x8A,             /* PCLK = 8.192 MHz; INTM = Open Drain */
   0x44, 0x00,             /* PCM Clock Slot = 0 TX / 0 RX; XE = Neg. */
   0x5E, 0x24, 0x04,       /* Device Mode Register */
   0xF6, 0x66, 0x00, 0x4C, /* Switching Regulator Timing Params */
         0x30, 0x64, 0x30,
   0xE4, 0x06, 0xA0, 0x00, /* Switching Regulator Params */
   0xE6, 0x00,             /* Switching Regulator Control */
/* Formatted Parameters Section -------------------------------------------- */
   0x20,                   /* IO21 Mode: Digital */
                           /* IO22 Mode: Analog Voltage Sense */
   0x22,                   /* Dial pulse correction: 2 ms */
                           /* Switcher Configuration = ABS (12 V in, 100 V out) */
   0x00, 0xFF,             /* Over-current shutdown count = 255 */
   0x01,                   /* Leading edge blanking window = 81 ns */
                           /* ABS supply config: 1 = Single; 2 = Single */
   0x45, 0x00, 0x44, 0x30, /* Free-Run Timing Parameters */
         0x64, 0x30,
   0x1B,                   /* Switcher 1 Voltage = 27 V */
   0x51,                   /* Switcher 2 Voltage = 81 V */
   0xFF,                   /* Adaptive Ringing Disabled */
   0x66, 0x00,             /* Low-Power Timing Parameters */
   0x20, 0x6E,             /* Switcher Limit Voltages 32 V, 110 V */
   0x03                    /* Charge pump enabled */
                           /* Charge Pump Overload Protection: Switcher Cycle Skip */
};

/************** DC_Parameters **************/

/* USA DC FXS Parameters - 23mA Current Feed */
const VpProfileDataType ABS_DC_23MA_CC[] =
{
/* Profile Header ---------------------------------------------------------- */
   0x0C, 0x01, 0x00,       /* DC Profile for ZL88601 device */
   0x0B,                   /* Profile length = 11 + 4 = 15 bytes */
   0x01,                   /* Profile format version = 1 */
   0x03,                   /* MPI section length = 3 bytes */
/* Raw MPI Section --------------------------------------------------------- */
   0xC6, 0x91, 0xE5,       /* DC Feed Parameters: ir_overhead = 100 ohm; */
                           /* VOC = 48 V; LI = 50 ohm; ILA = 23 mA */
                           /* Battery Switch Offset Voltage = 4.02 V */
/* Formatted Parameters Section -------------------------------------------- */
   0x1C,                   /* Switch Hook Threshold = 11 mA */
                           /* Ground-Key Threshold = 18 mA */
   0x86,                   /* Switch Hook Debounce = 12 ms */
                           /* Ground-Key Debounce = 16 ms */
   0x58,                   /* Low-Power Switch Hook Hysteresis = 2 V */
                           /* Ground-Key Hysteresis = 6 mA */
                           /* Switch Hook Hysteresis = 2 mA */
   0x80,                   /* Low-Power Switch Hook Threshold = 22 V */
   0x98,                   /* Reserved */
   0x07                    /* RPTC = 7 ohms */
};

/************** AC_Coefficients **************/

/* AC FXS RF14 600R Normal Coefficients (Default)  */
const VpProfileDataType ABS_AC_600_FXS[] =
{
  /* AC Profile */
 0xA4, 0x00, 0xF4, 0x4C, 0x01, 0x49, 0xCA, 0xF5, 0x98, 0xAA, 0x7B, 0xAB,
 0x2C, 0xA3, 0x25, 0xA5, 0x24, 0xB2, 0x3D, 0x9A, 0x2A, 0xAA, 0xA6, 0x9F,
 0x01, 0x8A, 0x1D, 0x01, 0xA3, 0xA0, 0x2E, 0xB2, 0xB2, 0xBA, 0xAC, 0xA2,
 0xA6, 0xCB, 0x3B, 0x45, 0x88, 0x2A, 0x20, 0x3C, 0xBC, 0x4E, 0xA6, 0x2B,
 0xA5, 0x2B, 0x3E, 0xBA, 0x8F, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0x2A, 0x42, 0xA1, 0xCB, 0x1B, 0xA3, 0xA8, 0xFB, 0x87, 0xAA,
 0xFB, 0x9F, 0xA9, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/************** Ring_Parameters **************/

/* Ringing 25Hz 50Vrms Fixed, AC Trip */
const VpProfileDataType ABS_RING_25HZ_SINE[] =
{
/* Profile Header ---------------------------------------------------------- */
   0x0C, 0x04, 0x00,       /* Ringing Profile for ZL88601 device */
   0x12,                   /* Profile length = 18 + 4 = 22 bytes */
   0x01,                   /* Profile format version = 1 */
   0x0C,                   /* MPI section length = 12 bytes */
/* Raw MPI Section --------------------------------------------------------- */
   0xC0, 0x00, 0x00, 0x00, /* Ringing Generator Parameters: */
         0x00, 0x44, 0x3A, /* 24.9 Hz Sine; 1.41 CF; 70.70 Vpk; 0.00 V bias */
         0x9D, 0x00, 0x00, /* Ring trip cycles = 2; RTHALFCYC = 0 */
         0x00, 0x00,
/* Formatted Parameters Section -------------------------------------------- */
   0xAA,                   /* Ring Trip Method = AC; Threshold = 21.0 mA */
   0x02,                   /* Ringing Current Limit = 54 mA */
   0x4E,                   /* Fixed; Max Ringing Supply = 75 V */
   0x00                    /* Balanced; Ring Cadence Control Disabled */
};

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/************** Metering_Profile **************/

/* end of file ZLR88621L_ABS_profiles.c */
