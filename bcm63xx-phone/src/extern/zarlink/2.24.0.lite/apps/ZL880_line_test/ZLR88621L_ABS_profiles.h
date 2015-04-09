/*
 * ZLR88621L_ABS_profiles.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   File:   X:\shafer\lightning\ztap2012\ztap\applications\quickstarts\sdk_examples\ZL880_initialization\ZLR88621_ABS_profiles.vpw
 *   Type:   Design for ZLR88621L SM2 Line Module Featuring the ZL88601, Lite Coefficients
 *   Date:   Monday, August 06, 2012 17:55:09
 *   Device: ZL880 ZL88601
 *
 *   This file was generated with Profile Wizard Version: P2.3.0
 */

#ifndef ZLR88621L_ABS_PROFILES_H
#define ZLR88621L_ABS_PROFILES_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device_Parameters **************/
extern const VpProfileDataType ABS_DEV_PROFILE[];    /* Device Configuration Data - ZL88601 100V ABS (-81V/-27V) */

/************** DC_Parameters **************/
extern const VpProfileDataType ABS_DC_23MA_CC[];     /* USA DC FXS Parameters - 23mA Current Feed */

/************** AC_Coefficients **************/
extern const VpProfileDataType ABS_AC_600_FXS[];     /* AC FXS RF14 600R Normal Coefficients (Default)  */

/************** Ring_Parameters **************/
extern const VpProfileDataType ABS_RING_25HZ_SINE[]; /* Ringing 25Hz 50Vrms Fixed, AC Trip */

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/************** Metering_Profile **************/

#endif /* ZLR88621L_ABS_PROFILES_H */

