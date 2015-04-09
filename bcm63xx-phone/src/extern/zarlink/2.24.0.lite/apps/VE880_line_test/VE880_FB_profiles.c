/*
 * VE880_FB_profiles.c --
 *
 * This file contains profile data in byte format
 *
 * Project Info --
 *   File:   X:\shafer\lightning\ztap2012\ztap\applications\quickstarts\sdk_examples\VE880_initialization\VE880_FB_profiles.vpw
 *   Type:   VE880 Tracker Configuration for a 100V Flyback Power Supply, Lite Narrowband Coefficients and PCM/MPI Interface at 8.192MHz PCLK
 *   Date:   Tuesday, August 07, 2012 11:49:08
 *   Device: VE880 Le88506
 *
 *   This file was generated with Profile Wizard Version: P2.3.0
 */

#include "VE880_FB_profiles.h"


/************** Device Parameters **************/

/* Device Configuration Data - VE880 Tracker 100V Flyback, PCM/MPI Interface */
const VpProfileDataType FB_DEV_PROFILE[] =
{
  /* Device Profile for VE880 Device Family */
 0x00, 0xFF, 0x02, 0x1A, /* Profile Type and Length */
 0x05, 0x00, /* Version and MPI Command */
 0x20, 0x00, /* PCLK Frequency */
 0x8A, 0x00, /* Device Config 1 and MCLK Correction */
 0x40, 0x02, /* Clock Slot and Max Events */
 0x05, 0x00, /* Device Tickrate */
 0xE4, 0x00, 0x04, 0x11, /* Flyback (100V) Switching Regulator Parameters */
 0x97, 0xFF, 0x95, 0x00, 0x94, 0x40, /* Regulator Timing Parameters */
 0x14, 0xF6, 0x14, 0x40, 0x14, 0x40  /* FreeRun Timing Parameters */
};

/************** AC Coefficients **************/

/* AC FXS RF14 600R Normal Coefficients (Default)  */
const VpProfileDataType FB_AC_600_FXS[] =
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

/************** DC Feed Parameters **************/

/* DC FXS Default -- Use for for all countries unless country file exists - 25 mA current feed */
const VpProfileDataType FB_DC_25MA_CC[] =
{
 /* DC Profile */
 0x00, 0x01, 0x00, 0x0B, 0x01, 0x08, 0xC2, 0x19, 0x84, 0xB2, 0x07, 0xC6,
 0x33, 0x07, 0x01
};

/************** Ring Parameters **************/

/* Default Ringing 25Hz 60Vrms Tracking - Use for all countries unless country file exists */
const VpProfileDataType FB_RING_25HZ_SINE[] =
{
  /* Sine, 25 Hz, 1.41 CF, 85.00 Vpk, 0.00 Bias */
  /* Ringing Profile */
 0x00, 0x04, 0x00, 0x12, 0x00, 0x0E, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x44,
 0x46, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x68, 0x01, 0x00, 0x00
};

/************** Call Progress Tones **************/

/************** Cadence Definitions **************/

/************** Caller ID **************/

/************** Metering Profile **************/

/* end of file VE880_FB_profiles.c */
