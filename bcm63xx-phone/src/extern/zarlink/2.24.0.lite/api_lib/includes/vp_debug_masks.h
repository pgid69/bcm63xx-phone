/** \file vp_debug_masks.h
 * vp_debug_masks.h
 *
 * This file contains the debug masks for the VP-APi_II
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11245 $
 * $LastChangedDate: 2013-12-17 11:24:47 -0600 (Tue, 17 Dec 2013) $
 */

#ifndef VP_DEBUG_MASKS_H
#define VP_DEBUG_MASKS_H

/* VP-API debug message types: */
#define VP_DBG_ERROR        0x00000001L /* Any error condition */
#define VP_DBG_WARNING      0x00000002L /* Any warning condition */
#define VP_DBG_INFO         0x00000004L /* Un-categorized information */
#define VP_DBG_API_FUNC     0x00000008L /* API function entry/exit (except tick) */

#define VP_DBG_API_FUNC_INT 0x00000010L /* Internal API function entry/exit */
#define VP_DBG_HAL          0x00000020L /* HAL traffic */
#define VP_DBG_SSL          0x00000040L /* SSL function entry/exit */
#define VP_DBG_EVENT        0x00000080L /* Events/results from VpGetEvent()/VpGetResults() */

#define VP_DBG_HOOK         0x00000100L /* Hook Based Code */
#define VP_DBG_LINE_STATE   0x00000200L /* Set Line State Based Code */
#define VP_DBG_CALIBRATION  0x00000400L /* VpCalCodec(), VpCalLine(), VpCal() */
#define VP_DBG_TEST         0x00000800L /* Verbose test debug */

#define VP_DBG_TEST_FUNC    0x00001000L /* Test I/F function enter and exit */
#define VP_DBG_FXO_FUNC     0x00002000L /* FXO Detection Code */
#define VP_DBG_SEQUENCER    0x00004000L /* Sequencer Based Code */
#define VP_DBG_CID          0x00008000L /* Caller ID funcitons */

#define VP_DBG_TEST_PCM     0x00010000L /* Verbose test pcm collection */
#define VP_DBG_GAIN         0x00020000L /* VpSetRelGain() and VP_OPTION_ID_ABS_GAIN */
#define VP_DBG_TIMER        0x00040000L /* API Timers */
#define VP_DBG_TEST_CALC    0x00080000L /* Verbose test calculations */

#define VP_DBG_ADP_RING     0x00100000L /* Adaptive Ringing */
#define VP_DBG_INTERRUPT    0x00200000L /* Interrupts */
#define VP_DBG_DTMF         0x00400000L /* DTMF Detection */
#define VP_DBG_DTMF_DETAIL  0x00800000L /* Detailed DTMF Detection */

#define VP_DBG_ALL          0xFFFFFFFFL

#endif /* VP_DEBUG_MASKS_H */
