/*
 * Le71HR8927G_100V_IB_Profile.c --
 *
 * This file contains profile data in byte format
 *
 * Project Info --
 *   File:   P:\SysApps\HardwareApps\LineModules_VOB\VE890 line modules\Le71HR8927G_Le89156_MCM\Software\Le71HR8927G_100V_IB_Profile.vpw
 *   Type:   VE890 Configuration for 100V Tracking Inverting-Boost Power Supply, Lite Narrowband Coefficients, and 8.192MHz PCLK
 *   Date:   Wednesday, January 18, 2012 18:10:16
 *   Device: VE890 Le89116
 *
 *   This file was generated with Profile Wizard Version: P2.1.2
 *
 * Project Comments --
 *  --------------------------------------------
 *  Profile Wizard Coefficient Release 2.7 Notes:
 *  --------------------------------------------
 *  Replaced FXS AC Coefficients for GR57, China, Finland, and Japan.
 *  
 *  --------------------------------------------
 *  Profile Wizard Coefficient Release 2.6 Notes:
 *  --------------------------------------------
 *  I. General:
 *  1. This release adds support for Mexico, Turkey, Thailand, Malaysia, Indonesia, and Ecuador, bringing
 *  the total number of supported countries to 44.  They are:
 *  Argentina (AR), Austria (AT), Australia (AU), Belgium (BE), Bulgaria (BG), Brazil (BR), Canada (CA), Switzerland (CH),
 *  Chile (CL), China (CN), Czech Republic (CZ), Germany (DE), Denmark (DK), Ecuador (EC), Spain (ES), Finland (FI),
 *  France (FR), UK (GB), Greece (GR), Hong Kong SAR China (HK), Hungary (HU), Indonesia (ID), Ireland (IE), Israel (IL),
 *  India (IN), Iceland (IS), Italy (IT), Japan (JP), S. Korea (KR), Mexico (MX), Malaysia (MY), Netherlands (NL),
 *  Norway (NO), New Zealand (NZ), Poland (PL), Portugal (PT), Russian Federation (RU), Sweden (SE), Singapore (SG),
 *  Thailand (TH), Turkey (TK), Taiwan (TW), USA (US), and South Africa (ZA).
 *  2. This release also corrects some Caller ID implementations and signal levels that were incorrect in release 2.3.
 *  3. The coefficients in this and all releases are provided for use only with the Zarlink VoicePath API. Please refer to the terms 
 *  and conditions for licensing the software regarding terms and conditions of usage.
 *  4. This release is for the VE8910 chipset and includes coefficients required for FXS operation.
 *  
 *  II. Device Profile:
 *  1. The default settings for the Device Profile are:
 *         PCLK = 8192 kHz
 *         PCM Transmit Edge = Positive
 *         Transmit Time Slot = 0
 *         Receive Time Slot = 0
 *         Interrupt Mode = Open Drain
 *         Switcher = Inverting Boost (100V)
 *         Driver Tick Rate = 5 ms
 *         Maximum Events / Tick = 2
 *  2. The settings may be changed by the user as necessary.  Please refer to the VE890 and API documentation for information about
 *  the supported settings.
 *  
 *  II. AC Profiles:
 *  1. FXS Coefficients assume -6dBr RX (Output from chipset) and 0dB TX relative gain levels.
 *  2. Supported countries not individually listed should use the default 600R profile AC_FXS_RF50_600R_DEF.
 *  4. AC FXS Coefficients assume the use of two 25 ohm series resistors or PTCs.
 *  5. This release includes Normal (or narrowband) coefficients for the FXS port.  Wideband coefficients are available upon request.
 *  
 *  III. DC Profile:
 *  1. The DC_FXS_DEF Profile is the default used for all countries.  Additional profiles may be created by the user if necessary.
 *  
 *  IV. Ring Profiles:
 *  1. RING_25HZ_DEF is the default ringing profile and should be used for all countries which do not have a listed ringing profile.  The default
 *  ringing profile is set for a sine wave ringing with an amplitude of 60 Vrms and a frequency of 25 Hz.
 *  2. All ringing profiles on the list have a 60 Vrms ringing level with tracking switcher output.
 *  3. DC biasing is set to 0 in the sample ringing profiles.
 *  
 *  V. Tone Profiles:
 *  1. These profiles are available only in the full version of the API.
 *  2. The shown levels assume a 6dB attenuation in the chipset before being outputed to line.
 *  3. Call progress tone levels may be arbitrary as they are not always specified in national standards, or the standards may not be available to Zarlink.
 *  4. ITU-T Recommendation E.180 (03/1998) revised on 02/2003 and ETSI TR 101 041-2 V.1.1.1 (05/1997) were used if national standards were not
 *  available.
 *  5. Recommended ETSI ES 201 970 call progress tones are provided for reference.
 *  6. Modulated tones f1 x f2 are approximated as the sum of f1 + (f1+f2)/2 + (f1-f2)/2.
 *  7. The data in these profiles may be changed by the user as necessary.
 *   8. T_CAS_DEF is not a country-specific tone and is used by several national Caller ID profiles.
 *  
 *  V. Cadence Profiles:
 *  1. These profiles are available only in the full version of the API.
 *  2.  ITU-T Recommendation E.180 (03/1998) revised on 02/2003 and ETSI TR 101 041-2 V.1.1.1 (05/1997) were used if national standards were not
 *  available.
 *  3. Recommended ETSI ES 201 970 call progress cadences are provided for reference.
 *  4. Some countries support multiple call progress tone cadences.  The ones used are believed to be representative and most common.  The user may
 *  wish to edit some of the cadence definitions or add additional cadences.
 *  5. Ringing signal cadences include the alerting signal(s) and necessary delays for Type 1 Caller ID, if it is supported below in the Caller ID Profiles.
 *  
 *  VI. Caller ID Profiles:
 *  1. These profiles are available only in the full version of the API.
 *  2. The option to calculate the checksum in the API is selected for all countries except Japan, which requires that the CRC checksum be calculated by
 *  host application.
 *  
 *  VII. FXO/Dialing Profiles:
 *  1. Not provided.
 *  
 *  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "Le71HR8927G_100V_IB_Profile.h"


/************** Device_Parameters **************/

/* Device Configuration Data */
const VpProfileDataType DEV_PROFILE_Inverting_Boost_100V[] =
{
  /* Device Profile for VE890 Device Family */
 0x00, 0xFF, 0x00, 0x14, /* Profile Type and Length */
 0x05, 0x00, /* Version and MPI Command */
 0x20, 0x00, /* PCLK Frequency */
 0x8A, 0x00, /* Device Config 1 and MCLK Correction */
 0x40, 0x02, /* Clock Slot and Max Events */
 0x0A, 0x00, /* Device Tickrate */
 0xE4, 0x02, 0x04, 0x0A, /* Inverting Boost (100V) Switching Regulator Parameters */
 0xB3, 0xFF, 0x94, 0x52, 0xD0, 0x52 /* Regulator Timing Parameters */
};

/************** AC_Coefficients **************/

/* AC FXS RF50 600R Normal Coefficients (Default)  */
const VpProfileDataType AC_FXS_RF50_600R_DEF[] =
{
  /* AC Profile */
 0xC0, 0x00, 0xBC, 0x4C, 0x01, 0x49, 0xCA, 0xF1, 0x98, 0xAA, 0x5B, 0xEB,
 0x2C, 0x72, 0xA4, 0x26, 0x24, 0xEB, 0x2D, 0x9A, 0x2B, 0xBA, 0x27, 0x9F,
 0x01, 0x8A, 0x2D, 0x01, 0x2B, 0xA0, 0xD2, 0xA2, 0xAC, 0xCB, 0xE5, 0xA3,
 0x2B, 0xBC, 0xAD, 0x46, 0x88, 0x3C, 0x20, 0x33, 0x4D, 0xAA, 0xB7, 0x45,
 0xE6, 0x24, 0x5F, 0x22, 0x9F, 0x82, 0xAA, 0x71, 0x80, 0xC8, 0xF0, 0x50,
 0x00, 0x86, 0x2A, 0x42, 0xA1, 0xDB, 0x1C, 0xA3, 0xA8, 0xFE, 0x87, 0xAA,
 0xFA, 0x9F, 0xB9, 0xF0, 0x96, 0x2E, 0x01, 0x00
};

/************** DC_Parameters **************/

/* DC FXS Default -- Use for for all countries unless country file exists */
const VpProfileDataType DC_FXS_DEF[] =
{
 /* DC Profile */
 0x00, 0x01, 0x00, 0x09, 0x02, 0x03, /* Header Information */
 0xC6, 0x13, 0xC7, /* Feed Parameters */
 0x19, 0x84, /* Loop Supervision Parameters */
 0x03, /* Min Floor Voltage 20V */
 0x01  /* Hook Threshold Hysteresis */
};

/************** Ring_Parameters **************/

/* Default Ringing 25 Hz 50 Vrms Tracking - Use for all countries unless country file exists */
const VpProfileDataType RING_25HZ_DEF[] =
{
  /* Sine, 25 Hz, 1.41 CF, 70.00 Vpk, 0.00 Bias */
  /* Ringing Profile */
 0x00, 0x04, 0x00, 0x12, 0x01, 0x0C, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x44,
 0x39, 0xCE, 0x00, 0x00, 0x00, 0x00,
 0xAA, 0x02,  /* Ring Trip Threshold (21mA) and Current Limit (54mA)*/
 0x0D, /* Max Supply Ringing Voltage 70V */
 0x00  /* Ringing Tracking Mode */
};

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/* end of file Le71HR8927G_100V_IB_Profile.c */
