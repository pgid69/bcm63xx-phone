/*
 * ZLR88721L_FB_profiles.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   File:   X:\shafer\lightning\ztap2012\ztap\applications\quickstarts\sdk_examples\ZL880_initialization\ZLR88721L_FB_profiles.vpw
 *   Type:   Design for ZLR88721L SM2 Line Module Featuring the  ZL88701, Lite Coefficients
 *   Date:   Monday, August 06, 2012 17:55:23
 *   Device: ZL880 ZL88701
 *
 *   This file was generated with Profile Wizard Version: P2.3.0
 */

#ifndef ZLR88721L_FB_PROFILES_H
#define ZLR88721L_FB_PROFILES_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device Profile **************/
extern const VpProfileDataType FB_DEV_PROFILE[];     /* Device Configuration Data - ZL88701 Tracker 100V Flyback */

/************** DC_Coefficients **************/
extern const VpProfileDataType FB_DC_25MA_CC[];      /* USA DC FXS Parameters - 25mA Current Feed */

/************** AC_Coefficients **************/
extern const VpProfileDataType FB_AC_600_FXS[];      /* AC FXS RF14 600R Normal Coefficients (Default)  */

/************** Ring_Parameters **************/
extern const VpProfileDataType FB_RING_25HZ_SINE[];  /* Ringing 25Hz 50Vrms Tracking, AC Trip */

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/************** Metering_Profile **************/

#endif /* ZLR88721L_FB_PROFILES_H */

