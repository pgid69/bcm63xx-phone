/*
 * Le71HR8927G_100V_IB_Profile.h --
 *
 * This header file exports the Profile data types
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

#ifndef LE71HR8927G_100V_IB_PROFILE_H
#define LE71HR8927G_100V_IB_PROFILE_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device_Parameters **************/
extern const VpProfileDataType DEV_PROFILE_Inverting_Boost_100V[];/* Device Configuration Data */

/************** AC_Coefficients **************/
extern const VpProfileDataType AC_FXS_RF50_600R_DEF[];/* AC FXS RF50 600R Normal Coefficients (Default)  */

/************** DC_Parameters **************/
extern const VpProfileDataType DC_FXS_DEF[];         /* DC FXS Default -- Use for for all countries unless country file exists */

/************** Ring_Parameters **************/
extern const VpProfileDataType RING_25HZ_DEF[];      /* Default Ringing 25 Hz 50 Vrms Tracking - Use for all countries unless country file exists */

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

#endif /* LE71HR8927G_100V_IB_PROFILE_H */

