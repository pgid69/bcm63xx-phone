/*
 * ZLR88722L_IB_profiles.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   File:   X:\shafer\lightning\ztap2012\ztap\applications\quickstarts\sdk_examples\ZL880_initialization\ZLR88722L_IB_profiles.vpw
 *   Type:   Design for ZLR88722L SM2 Line Module Featuring the  ZL88701, Lite Coefficients
 *   Date:   Monday, August 06, 2012 17:54:41
 *   Device: ZL880 ZL88701
 *
 *   This file was generated with Profile Wizard Version: P2.3.0
 */

#ifndef ZLR88722L_IB_PROFILES_H
#define ZLR88722L_IB_PROFILES_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device Profile **************/
extern const VpProfileDataType IB_DEV_PROFILE[];     /* Device Configuration Data */

/************** DC Profile **************/
extern const VpProfileDataType IB_DC_25MA_CC[];      /* USA DC FXS Parameters - 25mA Current Feed */

/************** AC Profile **************/
extern const VpProfileDataType IB_AC_600_FXS[];      /* AC FXS RF14 600R Normal Coefficients (Default)  */

/************** Ringing Profile **************/
extern const VpProfileDataType IB_RING_25HZ_SINE[];  /* USA Ringing 20Hz 50Vrms Tracking, AC Trip */

/************** Tone Profile **************/

/************** Cadence Profile **************/

/************** Caller ID Profile **************/

/************** Metering_Profile **************/

#endif /* ZLR88722L_IB_PROFILES_H */

