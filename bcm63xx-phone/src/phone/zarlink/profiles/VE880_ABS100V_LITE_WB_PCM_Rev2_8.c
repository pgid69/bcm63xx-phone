/*
 * VE880_ABS100V_LITE_WB_PCM_Rev2_8.c --
 *
 * This file contains profile data in byte format
 *
 * Project Info --
 *   File:   C:\Microsemi\Profile_Wizard_P2.7.0\Data\bcm63xx-phone\VE880_ABS100V_LITE_WB_PCM_Rev2_8.vpw
 *   Type:   VE880 ABS Configuration for -27/-81V Supply with VBL Regulation, Lite Wideband Coefficients and PCM/MPI Interface at 2.048MHz PCLK and tick rate at 10ms
 *   Date:   Sunday, April 19, 2015 16:50:42
 *   Device: VE880 Le88266
 *
 *   This file was generated with Profile Wizard Version: P2.7.0
 *
 * Project Comments --
 *  ----------------------------------------------------------------------------------------------------------------------------------
 *  Profile Wizard Coefficient Revision 2.8 Release Notes:
 *  ----------------------------------------------------------------------------------------------------------------------------------
 *  I. General:
 *  1. This is the first release of the VE880 worldwide coefficients.
 *  2. This release includes support for the following 44 countries:
 *  Argentina (AR), Austria (AT), Australia (AU), Belgium (BE), Brazil (BR), Bulgaria (BG), Canada (CA), Switzerland (CH),
 *  Chile (CL), China (CN), Czech Republic (CZ), Germany (DE), Denmark (DK), Ecuador (EC), Spain (ES), Finland (FI),
 *  France (FR), UK (GB), Greece (GR), Hong Kong SAR China (HK), Hungary (HU), Indonesia (ID), Ireland (IE), Israel (IL),
 *  India (IN), Iceland (IS), Italy (IT), Japan (JP), S. Korea (KR), Mexico (MX), Malaysia (MY), Netherlands (NL),
 *  Norway (NO), New Zealand (NZ), Poland (PL), Portugal (PT), Russian Federation (RU), Sweden (SE), Singapore (SG),
 *  Thailand (TH), Turkey (TK), Taiwan (TW), USA (US), and South Africa (ZA).
 *  2. The coefficients in this and all releases are provided for use only with the Microsemi VoicePath API-II (VP-API-II). Please refer
 *  to the terms and conditions for licensing the software regarding terms and conditions of usage. These profiles are provided for
 *  reference only with no guarantee whatsoever by Microsemi Corporation.
 *  3. This release is for the VE880 ABS devices and chipsets with PCM/MPI host interfaces including Le88266 and Le88286.
 *
 *  II. Device Profile:
 *  1. The default settings for the Device Profile are:
 *         PCLK = 8192 kHz
 *         PCM Transmit Edge = Positive
 *         Transmit Time Slot = 0
 *         Receive Time Slot = 0
 *         Interrupt Mode = Open Drain
 *         Y Voltage: -27V
 *         Z Voltage: -5V
 *         Battery Sense Input: VBL=SWVSY, VBM=XB, VBH=SWVSZ
 *         Switching Frequency: LP=48kHz, MP=96kHz, HP=384kHz
 *         Driver Tick Rate = 5 ms
 *
 *  2. These settings correspond to a VBL = -27V and VBH = -81V. It is possible to change these settings for a maximum VBL = -30V and
 *  VBH = -90V.
 *  3. These settings are for VBL regulation. VBH is only regulated via transformer coupling.
 *  4. The settings may be changed by the user as necessary.  Please refer to the VE880 and VP-API-II documentation for information
 *  about the supported settings.
 *
 *  II. AC Profiles:
 *  1. FXS Coefficients assume -6dBr RX (Output from chipset) and 0dB TX relative gain levels.
 *  2. Supported countries not individually listed should use the default 600R profile AC_FXS_RF14_600R_DEF.
 *  4. AC FXS Coefficients assume the use of two 7 ohm series resistors or PTCs. Customers using other PTC resistance values (such as
 *  25ohms or 50 ohms) should not use these AC coefficients and can request alternate ones from Microsemi.
 *  5. This release includes Wideband coefficients for the FXS port. Narrowband coefficients are available upon request.
 *
 *  III. DC Profile:
 *  1. The DC_FXS_VE880_ABS100V_DEF Profile is the default used for all countries.  Additional profiles may be created by the user if necessary.
 *
 *  IV. Ring Profiles:
 *  1. RING_25HZ_VE880_ABS100V_DEF is the default ringing profile and should be used for all countries which do not have a listed ringing profile.
 *  The default ringing profile is set for a sine wave ringing with an amplitude of 50 Vrms and a frequency of 25 Hz.
 *  2. All ringing profiles on the list are sinusoidal with an amplitude of 70 Vpk (50Vrms) with no DC bias.
 *  3. The ringing definitions may be changed based on the requirements of the target market as long as the total amplitude (AC +DC
 *  components) does not exceed the limits set forth in the VE880 data sheets.
 *
 *  V. Tone Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  VI. Cadence Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  VII. Caller ID Profiles:
 *  1. These profiles are available only in the full version of the VP-API-II.
 *
 *  VIII. Metering Profiles:
 *  1. Metering profiles are provided for 12kHz and 16kHz signaling. They are not currently listed by the countries that use them.
 *
 *  (c) Copyright 2011 Microsemi Corporation. All rights reserved.
 *
 *  -----------------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "VE880_ABS100V_LITE_WB_PCM_Rev2_8.h"


/************** Device Parameters **************/

/************** AC Filter Coefficients **************/

/* AC FXS RF14 600R Wideband Coefficients (Default)  */
const VpProfileDataType AC_FXS_RF14_WB_600R_DEF[] =
{
  /* AC Profile */
 0xED, 0x00, 0xA9, 0x4C, 0x01, 0x49, 0xCA, 0xFC, 0x98, 0xCC, 0x2A, 0xA2,
 0x3B, 0x98, 0xB3, 0x43, 0xA2, 0x3A, 0xAB, 0x9A, 0x2D, 0xAA, 0xB6, 0x9F,
 0x01, 0x8A, 0x2E, 0x01, 0xC4, 0xA0, 0x2E, 0x23, 0x3A, 0x2E, 0x9F, 0x87,
 0x4C, 0xCE, 0xB2, 0x27, 0x88, 0x4A, 0x20, 0xBA, 0xAC, 0x22, 0xB5, 0x5A,
 0xB5, 0x3B, 0x4D, 0xA2, 0x27, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0xCB, 0x5E, 0x22, 0xBC, 0x12, 0xA3, 0xA9, 0xFA, 0xC6, 0x34,
 0xE2, 0x9F, 0xBA, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/* AC FXS RF14 ETSI ES201 970 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_ETSI[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Telcordia GR-57 900R+2.16uF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_GR57[] =
{
  /* AC Profile */
 0x1D, 0x00, 0x10, 0x4C, 0x01, 0x49, 0xCA, 0x0B, 0x98, 0xC2, 0xBB, 0x2A,
 0x2B, 0x2B, 0x8F, 0x6A, 0xB3, 0x2A, 0xA7, 0x9A, 0x7B, 0xA2, 0x36, 0x6E,
 0x01, 0x8A, 0x2D, 0xD0, 0x2C, 0xA0, 0x2B, 0xAA, 0x2C, 0x62, 0x22, 0xAA,
 0xBC, 0xA4, 0x23, 0xAE, 0x88, 0xA2, 0x30, 0xAD, 0xC9, 0x2C, 0x41, 0xB3,
 0xA9, 0xB2, 0x53, 0xAB, 0x3D, 0x82, 0xB8, 0x71, 0x80, 0x7A, 0xF0, 0x50,
 0x00, 0x86, 0x2B, 0x55, 0xB1, 0x3A, 0x0A, 0xA2, 0x98, 0xCA, 0xA4, 0xAA,
 0x5A, 0x87, 0x6A, 0x60, 0x96, 0x1E, 0x01, 0x00
};

/* AC FXS RF14 Austria 220R+820R//115nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_AT[] =
{
  /* AC Profile */
 0x30, 0x00, 0xC0, 0x4C, 0x01, 0x49, 0xCA, 0xDB, 0x98, 0xBA, 0xA1, 0xBA,
 0x22, 0x2A, 0xF4, 0xAA, 0xAB, 0x22, 0x65, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x01, 0x11, 0xA3, 0x10, 0x2B, 0x29, 0x53, 0xB9, 0xBD, 0xB3,
 0x7B, 0xCC, 0xDB, 0xA6, 0x88, 0x2A, 0xB0, 0x2B, 0x9F, 0x2E, 0x3E, 0x22,
 0xBE, 0xC3, 0x26, 0x35, 0x2F, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0x2B, 0x53, 0x52, 0x4B, 0x12, 0x52, 0xDF, 0x32, 0xB4, 0x3A,
 0x5A, 0xA7, 0xB9, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/* AC FXS RF14 Australia 220R+820R//120nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_AU[] =
{
  /* AC Profile */
 0xC2, 0x00, 0x2E, 0x4C, 0x01, 0x49, 0xCA, 0xF0, 0x98, 0xBA, 0xA1, 0xAD,
 0xA3, 0xA2, 0x2C, 0x2D, 0xAB, 0x22, 0xD4, 0x9A, 0xCC, 0xA2, 0x97, 0x9F,
 0x01, 0x8A, 0x2E, 0x01, 0x7A, 0x10, 0xBA, 0x29, 0x2A, 0x3A, 0xA2, 0x32,
 0xB2, 0xAA, 0xF2, 0xA4, 0x88, 0xBA, 0xA0, 0x5C, 0x3F, 0x2C, 0xCE, 0xAA,
 0x87, 0xC2, 0x66, 0xDB, 0x7E, 0x82, 0xA8, 0x71, 0x80, 0x39, 0x70, 0x50,
 0x00, 0x86, 0xAA, 0x59, 0x81, 0x25, 0x1A, 0xA1, 0xBB, 0x2B, 0xB3, 0x2C,
 0x46, 0xD5, 0xBC, 0x60, 0x96, 0xAA, 0xB1, 0x00
};

/* AC FXS RF14 Belgium 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_BE[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Bulgaria 220R+820R//115nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_BG[] =
{
  /* AC Profile */
 0x30, 0x00, 0xC0, 0x4C, 0x01, 0x49, 0xCA, 0xDB, 0x98, 0xBA, 0xA1, 0xBA,
 0x22, 0x2A, 0xF4, 0xAA, 0xAB, 0x22, 0x65, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x01, 0x11, 0xA3, 0x10, 0x2B, 0x29, 0x53, 0xB9, 0xBD, 0xB3,
 0x7B, 0xCC, 0xDB, 0xA6, 0x88, 0x2A, 0xB0, 0x2B, 0x9F, 0x2E, 0x3E, 0x22,
 0xBE, 0xC3, 0x26, 0x35, 0x2F, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0x2B, 0x53, 0x52, 0x4B, 0x12, 0x52, 0xDF, 0x32, 0xB4, 0x3A,
 0x5A, 0xA7, 0xB9, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/* AC FXS RF14 Brazil 900R Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_BR[] =
{
  /* AC Profile */
 0xF3, 0x00, 0x33, 0x4C, 0x01, 0x49, 0xCA, 0x0B, 0x98, 0xAA, 0xAB, 0xAC,
 0xAA, 0x42, 0xAD, 0xB8, 0xF3, 0x34, 0xA5, 0x9A, 0xAC, 0xAA, 0xC7, 0x9F,
 0x01, 0x8A, 0x2E, 0x01, 0xAA, 0xA0, 0xCB, 0xDA, 0xAB, 0x22, 0xBC, 0xAA,
 0x3B, 0xA4, 0x42, 0x2F, 0x88, 0xAA, 0x40, 0x4A, 0x4A, 0xB3, 0x22, 0x3D,
 0x2B, 0xB2, 0xAF, 0x4A, 0x37, 0x82, 0xA9, 0xF1, 0x80, 0xB9, 0xF0, 0x50,
 0x00, 0x86, 0x2F, 0x5A, 0xA1, 0x3A, 0x03, 0xA2, 0xA3, 0xCC, 0x25, 0x3A,
 0x54, 0x3E, 0xCB, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/* AC FXS RF14 Switzerland 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_CH[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 China 200R+680R//100nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_CN[] =
{
  /* AC Profile */
 0xEF, 0x00, 0x46, 0x4C, 0x01, 0x49, 0xCA, 0xDA, 0x98, 0xB3, 0xB1, 0x23,
 0x32, 0x64, 0x97, 0xAA, 0x2B, 0x4A, 0xA4, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xCD, 0x10, 0x2B, 0x29, 0x22, 0x2A, 0xA2, 0x33,
 0x4B, 0x4C, 0xBD, 0xF6, 0x88, 0x2A, 0xC0, 0x52, 0x97, 0xF2, 0x2E, 0xAA,
 0x4E, 0xAC, 0xB5, 0x5B, 0xBE, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0x2A, 0x5A, 0xA2, 0xBA, 0x1B, 0x22, 0xCA, 0x24, 0xC3, 0x43,
 0x42, 0xA4, 0x4B, 0x50, 0x96, 0x3C, 0x41, 0x00
};

/* AC FXS RF14 Germany 220R+820R//115nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_DE[] =
{
  /* AC Profile */
 0x27, 0x00, 0xD7, 0x4C, 0x01, 0x49, 0xCA, 0xDB, 0x98, 0xBA, 0xA1, 0xBA,
 0x22, 0x2A, 0xF4, 0xAA, 0xAB, 0x22, 0x65, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xA3, 0x10, 0x2B, 0x29, 0x53, 0xB9, 0xBD, 0xB3,
 0x7B, 0xCC, 0xDB, 0xA6, 0x88, 0x2A, 0xB0, 0x2B, 0x9F, 0x2E, 0x3E, 0x22,
 0xBE, 0xC3, 0x26, 0x35, 0x2F, 0x82, 0xB8, 0x71, 0x80, 0xAB, 0xF0, 0x50,
 0x00, 0x86, 0xCA, 0x5A, 0xA2, 0xBA, 0x1F, 0x22, 0x3B, 0x2A, 0x23, 0x2B,
 0x3A, 0x24, 0x4B, 0x40, 0x96, 0x22, 0x21, 0x00
};

/* AC FXS RF14 Denmark 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_DK[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Spain 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_ES[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Finland 270R+910R//120nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_FI[] =
{
  /* AC Profile */
 0xF1, 0x00, 0xB1, 0x4C, 0x01, 0x49, 0xCA, 0xE5, 0x98, 0x2E, 0x52, 0x2A,
 0xB2, 0xBA, 0xB3, 0xA2, 0xAE, 0x22, 0x37, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x01, 0x11, 0x2B, 0x10, 0xA3, 0xB9, 0xA2, 0xC9, 0xAA, 0xB4,
 0x3A, 0xBC, 0x22, 0x37, 0x88, 0xEC, 0xB0, 0xB3, 0x2E, 0xE2, 0x3F, 0xAA,
 0x8F, 0x25, 0x97, 0xDB, 0xAF, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0x7A, 0x5A, 0xB2, 0x3B, 0x12, 0xA1, 0xAC, 0x24, 0xA2, 0x73,
 0x32, 0xB3, 0x22, 0x40, 0x96, 0x24, 0xA0, 0x00
};

/* AC FXS RF14 France 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_FR[] =
{
  /* AC Profile */
 0x98, 0x00, 0x26, 0x4C, 0x01, 0x49, 0xCA, 0xD7, 0x98, 0x4F, 0x22, 0xB5,
 0xB1, 0xC6, 0x23, 0x3A, 0xAB, 0x24, 0xB5, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x01, 0x11, 0x22, 0x10, 0x3A, 0x29, 0x3C, 0x29, 0x32, 0xB3,
 0x4A, 0x2C, 0x2E, 0xB6, 0x88, 0xAC, 0xB0, 0x43, 0x37, 0xAB, 0x3E, 0xB4,
 0x3E, 0x22, 0x37, 0x22, 0xAF, 0x82, 0xA8, 0x71, 0x80, 0xA9, 0xF0, 0x50,
 0x00, 0x86, 0xAA, 0x52, 0xE2, 0x2B, 0x12, 0x62, 0x63, 0x3A, 0x24, 0xA2,
 0x5C, 0x36, 0x39, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/* AC FXS RF14 UK 370R+620R//310nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_GB[] =
{
  /* AC Profile */
 0xDB, 0x00, 0x85, 0x4C, 0x01, 0x49, 0xCA, 0xE6, 0x98, 0x2B, 0x2A, 0x3B,
 0xD1, 0xB4, 0x22, 0x23, 0x6C, 0xA6, 0x43, 0x9A, 0xBA, 0x23, 0x25, 0x2D,
 0x01, 0x8A, 0x32, 0xC0, 0x2A, 0x10, 0xA5, 0x4A, 0x62, 0xBA, 0xA6, 0xA9,
 0xAA, 0xC3, 0xA2, 0xAB, 0x88, 0x32, 0xC0, 0x3D, 0xB3, 0xA3, 0xBA, 0x53,
 0xC2, 0xAA, 0xAA, 0xC4, 0xA3, 0x82, 0xE8, 0x71, 0x80, 0xAA, 0xF0, 0x50,
 0x00, 0x86, 0x3A, 0x5B, 0xC2, 0xFA, 0x12, 0xB2, 0xE2, 0x33, 0xE3, 0xBA,
 0x33, 0x24, 0xA4, 0x40, 0x96, 0x23, 0xB0, 0x00
};

/* AC FXS RF14 Greece 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_GR[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Hungary 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_HU[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Ireland 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_IE[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Israel 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_IL[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Iceland 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_IS[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Italy 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_IT[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Japan 600R+1uF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_JP[] =
{
  /* AC Profile */
 0x83, 0x00, 0xF8, 0x4C, 0x01, 0x49, 0xCA, 0xFF, 0x98, 0xAB, 0x3A, 0x35,
 0x2C, 0xA2, 0x43, 0xAA, 0x23, 0x26, 0xAB, 0x9A, 0xAA, 0xA2, 0x24, 0x1D,
 0x01, 0x8A, 0xAA, 0xB0, 0xA3, 0xA0, 0x22, 0xA3, 0x2A, 0xCE, 0x2A, 0xBD,
 0x62, 0xA5, 0x54, 0xBE, 0x88, 0x23, 0x30, 0x2E, 0xAC, 0xB2, 0xA6, 0xBA,
 0x26, 0xA2, 0xED, 0x22, 0x27, 0x82, 0xAA, 0x61, 0x80, 0x2A, 0x60, 0x50,
 0x00, 0x86, 0x23, 0x62, 0x22, 0x98, 0x0C, 0x43, 0x2A, 0x44, 0xA4, 0xBD,
 0x62, 0xB5, 0x4F, 0x50, 0x96, 0xBC, 0x01, 0x00
};

/* AC FXS RF14 Netherlands 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_NL[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Norway 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_NO[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 New Zealand 370R+620R//310nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_NZ[] =
{
  /* AC Profile */
 0x11, 0x00, 0xE2, 0x4C, 0x01, 0x49, 0xCA, 0xF2, 0x98, 0x5A, 0x29, 0xCB,
 0xA1, 0xA3, 0xA1, 0xB4, 0x3D, 0x22, 0x24, 0x9A, 0x22, 0xA2, 0xE5, 0x2E,
 0x01, 0x8A, 0x3A, 0xB0, 0xDA, 0x30, 0xBD, 0xAB, 0x42, 0xAF, 0xCA, 0xA9,
 0x32, 0x23, 0xBA, 0xAB, 0x88, 0x2A, 0xB0, 0xAD, 0x42, 0x4C, 0xA9, 0xA6,
 0xA1, 0x2A, 0x2A, 0xA2, 0x43, 0x82, 0x23, 0x71, 0x80, 0x4A, 0x60, 0x50,
 0x00, 0x86, 0xCA, 0x59, 0x81, 0xBA, 0x12, 0xA2, 0x44, 0x3B, 0xA3, 0x23,
 0x4A, 0xC4, 0x3A, 0x40, 0x96, 0x22, 0xB0, 0x00
};

/* AC FXS RF14 Portugal 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_PT[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Russia 150R+510R//47nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_RU[] =
{
  /* AC Profile */
 0x2E, 0x00, 0x60, 0x4C, 0x01, 0x49, 0xCA, 0xE6, 0x98, 0x2D, 0xC2, 0x62,
 0x43, 0xA2, 0xBC, 0x24, 0xEB, 0xBA, 0xB4, 0x9A, 0x24, 0xB5, 0x97, 0x9F,
 0x01, 0x8A, 0x2E, 0x01, 0x2C, 0x30, 0x62, 0x3A, 0x22, 0x2B, 0x3B, 0xA2,
 0xA2, 0xCB, 0x2A, 0xE5, 0x88, 0xDA, 0x40, 0xBA, 0x35, 0x24, 0xDD, 0x4A,
 0xAD, 0xD4, 0xA4, 0xAF, 0xAD, 0x82, 0xA8, 0x71, 0x80, 0xB9, 0xF0, 0x50,
 0x00, 0x86, 0x2A, 0x5A, 0xB2, 0x3B, 0x1A, 0x32, 0x2A, 0x32, 0xA5, 0xA8,
 0x72, 0x9F, 0x29, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/* AC FXS RF14 Sweden 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_SE[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 Turkey 270R+750R//150nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_TK[] =
{
  /* AC Profile */
 0xAA, 0x00, 0x9C, 0x4C, 0x01, 0x49, 0xCA, 0xE0, 0x98, 0xAA, 0xA3, 0x6B,
 0x22, 0x43, 0xA2, 0xAC, 0xBF, 0x22, 0xA6, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xBA, 0x10, 0xA2, 0x3A, 0xBA, 0xE9, 0xAA, 0xB5,
 0x2A, 0xBC, 0xAF, 0x97, 0x88, 0xAB, 0xC0, 0x4A, 0xBE, 0xBA, 0x9F, 0xC3,
 0xAF, 0x3A, 0x3F, 0xCD, 0xA7, 0x82, 0xC8, 0x71, 0x80, 0x69, 0xF0, 0x50,
 0x00, 0x86, 0xBA, 0x52, 0xB2, 0xBA, 0x12, 0x42, 0x4A, 0x2A, 0x33, 0xBA,
 0x33, 0x34, 0x4B, 0x40, 0x96, 0xB2, 0xA0, 0x00
};

/* AC FXS RF14 South Africa 220R+820R//115nF Wideband Coefficients */
const VpProfileDataType AC_FXS_RF14_WB_ZA[] =
{
  /* AC Profile */
 0x27, 0x00, 0xD7, 0x4C, 0x01, 0x49, 0xCA, 0xDB, 0x98, 0xBA, 0xA1, 0xBA,
 0x22, 0x2A, 0xF4, 0xAA, 0xAB, 0x22, 0x65, 0x9A, 0x01, 0x90, 0x01, 0x01,
 0x90, 0x8A, 0x2E, 0x01, 0xA3, 0x10, 0x2B, 0x29, 0x53, 0xB9, 0xBD, 0xB3,
 0x7B, 0xCC, 0xDB, 0xA6, 0x88, 0x2A, 0xB0, 0x2B, 0x9F, 0x2E, 0x3E, 0x22,
 0xBE, 0xC3, 0x26, 0x35, 0x2F, 0x82, 0xB8, 0x71, 0x80, 0xAB, 0xF0, 0x50,
 0x00, 0x86, 0xCA, 0x5A, 0xA2, 0xBA, 0x1F, 0x22, 0x3B, 0x2A, 0x23, 0x2B,
 0x3A, 0x24, 0x4B, 0x40, 0x96, 0x22, 0x21, 0x00
};

/************** DC Feed Parameters **************/

/************** Ring Signal Parameters **************/

/************** Call Progress Tones **************/

/************** Cadence Definitions **************/

/************** Caller ID **************/

/************** Metering Profile **************/

/* end of file VE880_ABS100V_LITE_WB_PCM_Rev2_8.c */
