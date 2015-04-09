/*
 * profiles_common.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   File:   C:\Microsemi\Profile_Wizard_P2.7.0\Data\bcm63xx-phone\profiles_common.vpw
 *   Type:   VP880 VoicePort MiniPBX Project (Line Module LE71HE0824)
 *   Date:   Monday, March 23, 2015 09:14:40
 *   Device: VE880 Le88241
 *
 *   This file was generated with Profile Wizard Version: P2.7.0
 */

#ifndef PROFILES_COMMON_H
#define PROFILES_COMMON_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device Parameters **************/

/************** AC_Filter_Coefficients **************/

/************** DC_Feed_Parameters **************/

/************** Ring_Signal_Parameters **************/

/************** Call_Progress_Tones **************/
extern const VpProfileDataType TONE_DIAL[];          /* US Dial Tone */
extern const VpProfileDataType TONE_SIT[];           /* Special Information Tone (Called Number Not Connected) */
extern const VpProfileDataType TONE_BUSY[];          /* Busy Signal */
extern const VpProfileDataType TONE_RINGBACK[];      /* Standard Ringback */
extern const VpProfileDataType TONE_CALLWAIT[];      /* Call Waiting Beep */
extern const VpProfileDataType TONE_ONEKHZ_L[];      /* A 1kHz tone at -10dBm0 */
extern const VpProfileDataType TONE_ONEKHZ_H[];      /* A 1kHz tone at 0dBm0 */
extern const VpProfileDataType TONE_CLI[];           /* Caller ID Alert Tone for UK Cadence */
extern const VpProfileDataType TONE_ROH[];           /* Receiver Off-Hook */

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/************** Metering_Profile **************/

#endif /* PROFILES_COMMON_H */

