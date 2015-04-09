/** \file vp886_registers.h
 * vp886_registers.h
 *
 * Header file for the VP886 register and command set.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11605 $
 * $LastChangedDate: 2014-10-20 10:44:13 -0500 (Mon, 20 Oct 2014) $
 */

#ifndef VP886_REGISTERS_H
#define VP886_REGISTERS_H


/* =============================================================================
                        Global Register Definitions
   ========================================================================== */


/* 02 : Software Reset Command */
#define VP886_R_SWRESET_WRT                     0x02
#define VP886_R_SWRESET_LEN                     0


/* 04 : Hardware Reset Command */
#define VP886_R_HWRESET_WRT                     0x04
#define VP886_R_HWRESET_LEN                     0


/* 06 : No Operation */
#define VP886_R_NOOP_WRT                        0x06
#define VP886_R_NOOP_LEN                        0
    /* No-op abbreviation */
#define VP886_NOOP                              VP886_R_NOOP_WRT


/* 30/31 : Test Register 1 */
#define VP886_R_TEST_REG_1_WRT                  0x30
#define VP886_R_TEST_REG_1_RD                   0x31
#define VP886_R_TEST_REG_1_LEN                  1
#define VP886_R_TEST_REG_1_MODE                 0x0F
#define VP886_R_TEST_REG_1_MODE_DISABLED        0x00
#define VP886_R_TEST_REG_1_MODE_FUSE            0x03


/* 32/33 : Test Register 2 */
#define VP886_R_TEST_REG_2_WRT                  0x32
#define VP886_R_TEST_REG_2_RD                   0x33
#define VP886_R_TEST_REG_2_LEN                  1


/* 34/35 : Fuse Control */
#define VP886_R_FUSE_CTRL_WRT                   0x34
#define VP886_R_FUSE_CTRL_RD                    0x35
#define VP886_R_FUSE_CTRL_LEN                   1


/* 36/37 : Fuse Register 1 */
#define VP886_R_FUSE_REG_1_WRT                  0x36
#define VP886_R_FUSE_REG_1_RD                   0x37
#define VP886_R_FUSE_REG_1_LEN                  1


/* 38/39 : Fuse Register 2  */
#define VP886_R_FUSE_REG_2_WRT                  0x38
#define VP886_R_FUSE_REG_2_RD                   0x39
#define VP886_R_FUSE_REG_2_LEN                  1


/* 3A/3B : Fuse Register 8 */
#define VP886_R_FUSE_REG_8_WRT                  0x3A
#define VP886_R_FUSE_REG_8_RD                   0x3B
#define VP886_R_FUSE_REG_8_LEN                  4
    /* Byte 0 */
#define VP886_R_FUSE_REG_8_SHSUP                0x80


/* 3E/3F : Fuse Register 5 */
#define VP886_R_FUSE_REG_5_WRT                  0x3E
#define VP886_R_FUSE_REG_5_RD                   0x3F
#define VP886_R_FUSE_REG_5_LEN                  1


/* 44/45 : Transmit and Receive Clock Slot and Transmit Clock Edge Register */
#define VP886_R_CLKSLOTS_WRT                    0x44
#define VP886_R_CLKSLOTS_RD                     0x45
#define VP886_R_CLKSLOTS_LEN                    1
#define VP886_R_CLKSLOTS_EDGE                   0x40
#define VP886_R_CLKSLOTS_EDGE_NEG               0x00
#define VP886_R_CLKSLOTS_EDGE_POS               0x40
#define VP886_R_CLKSLOTS_RCS                    0x38
#define VP886_R_CLKSLOTS_TCS                    0x07


/* 46/47 : Device Configuration Register */
#define VP886_R_DEVCFG_WRT                      0x46
#define VP886_R_DEVCFG_RD                       0x47
#define VP886_R_DEVCFG_LEN                      1
#define VP886_R_DEVCFG_INT_MODE                 0x80
#define VP886_R_DEVCFG_DOUBLE_PCLK              0x40
#define VP886_R_DEVCFG_PCM_SIG                  0x20
#define VP886_R_DEVCFG_FS_16K                   0x10
#define VP886_R_DEVCFG_CLKSEL                   0x0F
#define VP886_R_DEVCFG_CLKSEL_1536              0x00
#define VP886_R_DEVCFG_CLKSEL_1544              0x01
#define VP886_R_DEVCFG_CLKSEL_2048              0x02
#define VP886_R_DEVCFG_CLKSEL_1024              0x03
#define VP886_R_DEVCFG_CLKSEL_3072              0x04
#define VP886_R_DEVCFG_CLKSEL_3088              0x05
#define VP886_R_DEVCFG_CLKSEL_4096              0x06
#define VP886_R_DEVCFG_CLKSEL_6144              0x08
#define VP886_R_DEVCFG_CLKSEL_6176              0x09
#define VP886_R_DEVCFG_CLKSEL_8192              0x0A


/* 4A/4B : Channel Enable & Operating Mode Register */
#define VP886_R_EC_WRT                          0x4A
#define VP886_R_EC_RD                           0x4B
#define VP886_R_EC_LEN                          1
#define VP886_R_EC_TME                          0x80
#define VP886_R_EC_GLOBAL                       0x03
#define VP886_R_EC_EC2                          0x02
#define VP886_R_EC_EC1                          0x01
    /* EC abbreviations */
#define VP886_EC_1                              0x01
#define VP886_EC_2                              0x02
#define VP886_EC_BOTH                           0x03
#define VP886_EC_GLOBAL                         0x03


/* 4D/4F : Signaling Register */
#define VP886_R_SIGREG_NO_UL_RD                 0x4D
#define VP886_R_SIGREG_UL_RD                    0x4F
#define VP886_R_SIGREG_LEN                      4
    /* Byte 0 only */
#define VP886_R_SIGREG_CFAIL                    0x80
    /* Bytes 0 and 1, per-channel */
#define VP886_R_SIGREG_OCALM                    0x40
#define VP886_R_SIGREG_TEMPA                    0x20
#define VP886_R_SIGREG_IO2                      0x10
#define VP886_R_SIGREG_CAD                      0x08
#define VP886_R_SIGREG_CID                      0x04
#define VP886_R_SIGREG_GNK                      0x02
#define VP886_R_SIGREG_HOOK                     0x01
    /* Byte 2 only */
#define VP886_R_SIGREG_CPUVLO                   0x20
#define VP886_R_SIGREG_GTIMER                   0x08
    /* Byte 3 only */
#define VP886_R_SIGREG_TIMESTAMP                0x08
    /* Bytes 2 and 3, per-channel */
#define VP886_R_SIGREG_OVALM                    0x40
#define VP886_R_SIGREG_CHTIMER                  0x04
#define VP886_R_SIGREG_SDAT                     0x02
#define VP886_R_SIGREG_VDAT                     0x01


/* 5E/5F : Device Mode Register */
#define VP886_R_DEVMODE_WRT                     0x5E
#define VP886_R_DEVMODE_RD                      0x5F
#define VP886_R_DEVMODE_LEN                     2
    /* Byte 0 */
#define VP886_R_DEVMODE_OVP_THRESH              0x30
#define VP886_R_DEVMODE_CHARGEPUMP_EN           0x08
#define VP886_R_DEVMODE_CHARGEPUMP_V            0x07
    /* Byte 1 */
#define VP886_R_DEVMODE_SPEEDUP_CUR_LIM         0x0C
#define VP886_R_DEVMODE_FORCE_LSP               0x02
#define VP886_R_DEVMODE_LINE_SENSE              0x01

/* 6C/6D : Interrupt Mask Register */
#define VP886_R_INTMASK_WRT                     0x6C
#define VP886_R_INTMASK_RD                      0x6D
#define VP886_R_INTMASK_LEN                     4
    /* Byte 0 only */
#define VP886_R_INTMASK_CFAIL                   0x80
    /* Bytes 0 and 1 per-channel */
#define VP886_R_INTMASK_OCALM                   0x40
#define VP886_R_INTMASK_TEMPA                   0x20
#define VP886_R_INTMASK_IO2                     0x10
#define VP886_R_INTMASK_CAD                     0x08
#define VP886_R_INTMASK_CID                     0x04
#define VP886_R_INTMASK_GNK                     0x02
#define VP886_R_INTMASK_HOOK                    0x01
    /* Byte 2 only */
#define VP886_R_INTMASK_CPUVLO                  0x20
#define VP886_R_INTMASK_GTIMER                  0x08
    /* Byte 3 only */
#define VP886_R_INTMASK_TIMESTAMP               0x08
    /* Bytes 2 and 3 per-channel */
#define VP886_R_INTMASK_OVALM                   0x40
#define VP886_R_INTMASK_CHTIMER                 0x04


/* 73 : Revision and Product Code Number */
#define VP886_R_RCNPCN_RD                       0x73
#define VP886_R_RCNPCN_LEN                      2
#define VP886_R_RCNPCN_RCN_IDX                  0
#define VP886_R_RCNPCN_PCN_IDX                  1
    /* Byte 0 */
#define VP886_R_RCNPCN_RCN_AAA                  0x01
#define VP886_R_RCNPCN_RCN_BAA                  0x02
    /* Byte 1 */
#define VP886_R_RCNPCN_PCN_ZL88601              0x73    /* Low voltage ABS */
#define VP886_R_RCNPCN_PCN_ZL88602              0x77    /* High voltage ABS (Also Le9662) */
#define VP886_R_RCNPCN_PCN_ZL88701              0x7B    /* Low voltage 2ch Tracker */
#define VP886_R_RCNPCN_PCN_ZL88702              0x7F    /* High voltage 2ch Tracker (Also Le9672) */
#define VP886_R_RCNPCN_PCN_LE9661               0x4B    /* Low voltage 1ch Tracker */
#define VP886_R_RCNPCN_PCN_LE9671               0x4F    /* High voltage 1ch Tracker */
#define VP886_R_RCNPCN_PCN_LE9642               0x75    /* SF2 High Voltage ABS */
#define VP886_R_RCNPCN_PCN_LE9641               0x49    /* SF1 Low voltage 1ch Tracker */
#define VP886_R_RCNPCN_PCN_LE9652               0x7D    /* SF2 High voltage 2ch Tracker */
#define VP886_R_RCNPCN_PCN_LE9651               0x4D    /* SF1 High voltage 1ch Tracker */
#define VP886_R_RCNPCN_PCN_NOT_SF               0x02
#define VP886_R_RCNPCN_PCN_LV_HV                0x04
#define VP886_R_RCNPCN_PCN_ABS_TRACKER          0x08
#define VP886_R_RCNPCN_PCN_2CH                  0x30


/* B9 : Internal Revision Information */
#define VP886_R_INTREV_RD                       0xB9
#define VP886_R_INTREV_LEN                      1


/* BB : Charge Pump Cycle Request Counter */
#define VP886_R_CPCYCLES_RD                     0xBB
#define VP886_R_CPCYCLES_LEN                    2


/* D6/D7 : Global Timer */
#define VP886_R_GTIMER_WRT                      0xD6
#define VP886_R_GTIMER_RD                       0xD7
#define VP886_R_GTIMER_LEN                      2
    /* Byte 0 */
#define VP886_R_GTIMER_ENABLE                   0x80
#define VP886_R_GTIMER_MODE                     0x40
#define VP886_R_GTIMER_MODE_CONT                0x00
#define VP886_R_GTIMER_MODE_1SHOT               0x40
#define VP886_R_GTIMER_TIME_MSB                 0x3F
#define VP886_R_GTIMER_TIME_MAXIMUM             0x3FFF


/* DA/DB : Timestamp */
#define VP886_R_TIMESTAMP_WRT                   0xDA
#define VP886_R_TIMESTAMP_RD                    0xDB
#define VP886_R_TIMESTAMP_LEN                   2


/* E6/E7 : Switching Regulator Control */
#define VP886_R_SWCTRL_WRT                      0xE6
#define VP886_R_SWCTRL_RD                       0xE7
#define VP886_R_SWCTRL_LEN                      1
#define VP886_R_SWCTRL_SWPOL                    0x40
#define VP886_R_SWCTRL_FREQ                     0x30
#define VP886_R_SWCTRL_MODE_Z                   0x0C
#define VP886_R_SWCTRL_MODE_Z_OFF               0x00
#define VP886_R_SWCTRL_MODE_Z_LP                0x04
#define VP886_R_SWCTRL_MODE_Z_MP                0x08
#define VP886_R_SWCTRL_MODE_Z_HP                0x0C
#define VP886_R_SWCTRL_MODE_Y                   0x03
#define VP886_R_SWCTRL_MODE_Y_OFF               0x00
#define VP886_R_SWCTRL_MODE_Y_LP                0x01
#define VP886_R_SWCTRL_MODE_Y_MP                0x02
#define VP886_R_SWCTRL_MODE_Y_HP                0x03


/* F0/F1 : Fuse Register 6  */
#define VP886_R_FUSE_REG_6_WRT                  0xF0
#define VP886_R_FUSE_REG_6_RD                   0xF1
#define VP886_R_FUSE_REG_6_LEN                  1


/* F6/F7 : Internal Switching Regulator Parameters */
#define VP886_R_SWTIMING_WRT                    0xF6
#define VP886_R_SWTIMING_RD                     0xF7
#define VP886_R_SWTIMING_LEN                    6


/* F8/F9 : Fuse Register 7  */
#define VP886_R_FUSE_REG_7_WRT                  0xF8
#define VP886_R_FUSE_REG_7_RD                   0xF9
#define VP886_R_FUSE_REG_7_LEN                  1



/* =============================================================================
                        Channel Register Definitions
   ========================================================================== */


/* 02 : Software Reset */
#define VP886_R_SWRESET_WRT                     0x02
#define VP886_R_SWRESET_LEN                     0


/* 40/41 : Transmit Time Slot */
#define VP886_R_TXSLOT_WRT                      0x40
#define VP886_R_TXSLOT_RD                       0x41
#define VP886_R_TXSLOT_LEN                      1


/* 42/43 : Receive Time Slot */
#define VP886_R_RXSLOT_WRT                      0x42
#define VP886_R_RXSLOT_RD                       0x43
#define VP886_R_RXSLOT_LEN                      1


/* 50/51 : Voice Path Gains */
#define VP886_R_VPGAIN_WRT                      0x50
#define VP886_R_VPGAIN_RD                       0x51
#define VP886_R_VPGAIN_LEN                      1
#define VP886_R_VPGAIN_AX                       0x40
#define VP886_R_VPGAIN_AR                       0x30
#define VP886_R_VPGAIN_AR_0                     0x00
#define VP886_R_VPGAIN_AR_LOSS                  0x10
#define VP886_R_VPGAIN_AR_GAIN                  0x20
#define VP886_R_VPGAIN_DRL                      0x08


/* 52/53 : Input/Output Data Register */
#define VP886_R_IODATA_WRT                      0x52
#define VP886_R_IODATA_RD                       0x53
#define VP886_R_IODATA_LEN                      1
#define VP886_R_IODATA_IO6                      0x20
#define VP886_R_IODATA_IO5                      0x10
#define VP886_R_IODATA_IO4                      0x08
#define VP886_R_IODATA_IO3                      0x04
#define VP886_R_IODATA_IO2                      0x02
#define VP886_R_IODATA_IO1                      0x01


/* 54/55 : Input/Output Direction Register */
#define VP886_R_IODIR_WRT                       0x54
#define VP886_R_IODIR_RD                        0x55
#define VP886_R_IODIR_LEN                       1
#define VP886_R_IODIR_IOD2                      0x0C
#define VP886_R_IODIR_IOD2_INPUT                0x00
#define VP886_R_IODIR_IOD2_OUTPUT               0x04
#define VP886_R_IODIR_IOD2_VMON                 0x08
#define VP886_R_IODIR_IOD2_CAL                  0x0C
#define VP886_R_IODIR_IOD1                      0x03
#define VP886_R_IODIR_IOD1_INPUT                0x00
#define VP886_R_IODIR_IOD1_DIG_OUTPUT           0x01
#define VP886_R_IODIR_IOD1_OD_OUTPUT            0x02


/* 56/57 : System State Register */
#define VP886_R_STATE_WRT                       0x56
#define VP886_R_STATE_RD                        0x57
#define VP886_R_STATE_LEN                       1
#define VP886_R_STATE_REX                       0x80
#define VP886_R_STATE_METER                     0x40
#define VP886_R_STATE_CODEC                     0x20
#define VP886_R_STATE_POL                       0x10
#define VP886_R_STATE_SS                        0x0F
#define VP886_R_STATE_SS_DISCONNECT             0x00
#define VP886_R_STATE_SS_TIPOPEN                0x01
#define VP886_R_STATE_SS_RINGOPEN               0x02
#define VP886_R_STATE_SS_ACTIVE                 0x03    /* Tracker State */
#define VP886_R_STATE_SS_ACTIVE_LOW             0x03    /* ABS State */
#define VP886_R_STATE_SS_IDLE                   0x04
#define VP886_R_STATE_SS_LONGTEST               0x05
#define VP886_R_STATE_SS_METTEST                0x06
#define VP886_R_STATE_SS_BAL_RING               0x07
#define VP886_R_STATE_SS_LOWGAIN                0x08
#define VP886_R_STATE_SS_UNBAL_RING             0x0A
#define VP886_R_STATE_SS_ACTIVE_HIGH            0x0B    /* ABS State */
#define VP886_R_STATE_SS_LOWPOWER               0x0C
#define VP886_R_STATE_SS_SHUTDOWN               0x0F
#define VP886_R_STATE_SS_FEED_MASK              0x0F

/* 5B : Switcher ON Time Parameters */
#define VP886_R_SW_ON_TIME_RD                   0x5B
#define VP886_R_SW_ON_TIME_LEN                  4

/* 58/59 : Switcher Over-current Detection */
#define VP886_R_SW_OC_WRT                       0x58
#define VP886_R_SW_OC_RD                        0x59
#define VP886_R_SW_OC_LEN                       5


/* 5C/5D : Hook Detect Freeze Time */
#define VP886_R_HOOKFREEZE_WRT                  0x5C
#define VP886_R_HOOKFREEZE_RD                   0x5D
#define VP886_R_HOOKFREEZE_LEN                  1


/* 60/61 : Operating Functions */
#define VP886_R_OPFUNC_WRT                      0x60
#define VP886_R_OPFUNC_RD                       0x61
#define VP886_R_OPFUNC_LEN                      1
#define VP886_R_OPFUNC_CODEC                    0xC0
#define VP886_R_OPFUNC_CODEC_ALAW               0x00
#define VP886_R_OPFUNC_CODEC_ULAW               0x40
#define VP886_R_OPFUNC_CODEC_LINEAR             0x80
#define VP886_R_OPFUNC_EGR                      0x20
#define VP886_R_OPFUNC_EGX                      0x10
#define VP886_R_OPFUNC_EX                       0x08
#define VP886_R_OPFUNC_ER                       0x04
#define VP886_R_OPFUNC_EZ                       0x02
#define VP886_R_OPFUNC_EB                       0x01
#define VP886_R_OPFUNC_ALL_FILTERS              0x3F


/* 68/69 : Channel System State Configuration */
#define VP886_R_SSCFG_WRT                       0x68
#define VP886_R_SSCFG_RD                        0x69
#define VP886_R_SSCFG_LEN                       2
    /* Byte 0 */
#define VP886_R_SSCFG_WBAND                     0x80
#define VP886_R_SSCFG_AUTO_RINGTRIP             0x40
#define VP886_R_SSCFG_AUTO_CFAIL                0x20
#define VP886_R_SSCFG_AUTO_THERMFAULT           0x10
#define VP886_R_SSCFG_ZEROCROSS                 0x08
#define VP886_R_SSCFG_SMOOTHPOLREV              0x04
#define VP886_R_SSCFG_AUTO_SYSSTATE             0x02
#define VP886_R_SSCFG_AUTO_BAT                  0x01    /* ABS Only */
    /* Byte 1 */
#define VP886_R_SSCFG_AUTO_OVERCURRENT          0x10
#define VP886_R_SSCFG_AUTO_OVERVOLTAGE          0x08
#define VP886_R_SSCFG_RING_UPDATE               0x04
#define VP886_R_SSCFG_AUTO_SYSSTATE_LPM         0x02
#define VP886_R_SSCFG_AUTO_SYSSTATE_LPM_EN      0x00
#define VP886_R_SSCFG_AUTO_SYSSTATE_LPM_DIS     0x02
#define VP886_R_SSCFG_AUTO_CP_UV                0x01


/* 70/71 : Operating Conditions */
#define VP886_R_OPCOND_WRT                      0x70
#define VP886_R_OPCOND_RD                       0x71
#define VP886_R_OPCOND_LEN                      1
#define VP886_R_OPCOND_CUT_TX                   0x80
#define VP886_R_OPCOND_CUT_RX                   0x40
#define VP886_R_OPCOND_HIGHPASS                 0x20
#define VP886_R_OPCOND_HIGHPASS_DIS             0x20
#define VP886_R_OPCOND_LRG                      0x10
#define VP886_R_OPCOND_TSA_LOOPBACK             0x04
#define VP886_R_OPCOND_1K_TONE                  0x01

/* 75 : Buffer B1 Read */
#define VP886_R_B1_RD                           0x75
#define VP886_R_B1_LEN                          25
#define VP886_R_B1_OVFL                         0xC0
#define VP886_R_B1_PTR                          0x3F

/* 77 : Buffer B2 Read */
#define VP886_R_B2_RD                           0x77
#define VP886_R_B2_LEN                          25

/* 79 : Buffer B3 Read */
#define VP886_R_B3_RD                           0x79
#define VP886_R_B3_LEN                          25

/* 7B : Buffer B4 Read */
#define VP886_R_B4_RD                           0x7B
#define VP886_R_B4_LEN                          25

/* 7D : Buffer B5 Read */
#define VP886_R_B5_RD                           0x7D
#define VP886_R_B5_LEN                          25

/* Byte 0 */
#define VP886_R_BX_OVFL                         0xC0
#define VP886_R_BX_PTR                          0x3F

/* 7F : Buffer B6 Read */
#define VP886_R_B6_RD                           0x7F
#define VP886_R_B6_LEN                          10

/* 80/81 : GX Filter Coefficients */
#define VP886_R_GX_WRT                          0x80
#define VP886_R_GX_RD                           0x81
#define VP886_R_GX_LEN                          2

/* 82/83 : GR Filter Coefficients */
#define VP886_R_GR_WRT                          0x82
#define VP886_R_GR_RD                           0x83
#define VP886_R_GR_LEN                          2

/* 86/87 : B Filter FIR Coefficients */
#define VP886_R_B_FIR_FILT_WRT                  0x86
#define VP886_R_B_FIR_FILT_RD                   0x87
#define VP886_R_B_FIR_FILT_LEN                  14

/* 88/89 : X Filter Coefficients */
#define VP886_R_X_FILT_WRT                      0x88
#define VP886_R_X_FILT_RD                       0x89
#define VP886_R_X_FILT_LEN                      12

/* 8A/8B : R Filter Coefficients */
#define VP886_R_R_FILT_WRT                      0x8A
#define VP886_R_R_FILT_RD                       0x8B
#define VP886_R_R_FILT_LEN                      14

/* 96/97 : B Filter IIR Coefficients */
#define VP886_R_B_IIR_FILT_WRT                  0x96
#define VP886_R_B_IIR_FILT_RD                   0x97
#define VP886_R_B_IIR_FILT_LEN                  2

/* 98/99 : Z Filter FIR Coefficients */
#define VP886_R_Z_FIR_FILT_WRT                  0x98
#define VP886_R_Z_FIR_FILT_RD                   0x99
#define VP886_R_Z_FIR_FILT_LEN                  10

/* 9A/9B : Z Filter IIR Coefficients */
#define VP886_R_Z_IIR_FILT_WRT                  0x9A
#define VP886_R_Z_IIR_FILT_RD                   0x9B
#define VP886_R_Z_IIR_FILT_LEN                  5

/* A0/A1 : Supervision ADC Control */
#define VP886_R_SADC_WRT                        0xA0
#define VP886_R_SADC_RD                         0xA1
#define VP886_R_SADC_LEN                        10
    /* Byte 0 */
#define VP886_R_SADC_TX_INTERRUPT               0x80
#define VP886_R_SADC_MATH                       0x40
#define VP886_R_SADC_GROUP_MODE                 0x20
#define VP886_R_SADC_ENABLE                     0x10
#define VP886_R_SADC_DRATE                      0x03
#define VP886_R_SADC_DRATE_SINGLE_2KHZ          0x00
#define VP886_R_SADC_DRATE_SINGLE_4KHZ          0x01
#define VP886_R_SADC_DRATE_SINGLE_8KHZ          0x02
#define VP886_R_SADC_DRATE_GROUP_500HZ          0x00
#define VP886_R_SADC_DRATE_GROUP_1KHZ           0x01
#define VP886_R_SADC_DRATE_GROUP_2KHZ           0x02
    /* Byte 1, 2, 3, 4, 5 */
#define VP886_R_SADC_SEL                        0x1F
#define VP886_R_SADC_SEL_TIP_RING_AC_V          0x00
#define VP886_R_SADC_SEL_SWY                    0x01
#define VP886_R_SADC_SEL_SWZ                    0x02
#define VP886_R_SADC_SEL_IO2X_V                 0x03
#define VP886_R_SADC_SEL_TIP_GROUND_V           0x04
#define VP886_R_SADC_SEL_RING_GROUND_V          0x05
#define VP886_R_SADC_SEL_TIP_RING_DC_V          0x06
#define VP886_R_SADC_SEL_METALLIC_CUR           0x07
#define VP886_R_SADC_SEL_LONG_CUR               0x08
#define VP886_R_SADC_SEL_HIGH_CAL_CUR           0x09
#define VP886_R_SADC_SEL_TIP_PLUS_RING          0x0A
#define VP886_R_SADC_SEL_ADC_OFFSET             0x0B
#define VP886_R_SADC_SEL_LOW_CAL_CUR            0x0C
#define VP886_R_SADC_SEL_TEMP                   0x0D
#define VP886_R_SADC_SEL_IM_PLUS_IL             0x0E
#define VP886_R_SADC_SEL_IM_MINUS_IL            0x0F
#define VP886_R_SADC_SEL_VDDSW                  0x10
#define VP886_R_SADC_SEL_VMODE_OFFSET           0x11
#define VP886_R_SADC_SEL_VMODE_REF              0x12
#define VP886_R_SADC_SEL_SWY_ERR                0x13
#define VP886_R_SADC_SEL_SWZ_ERR                0x14
#define VP886_R_SADC_SEL_HOOK_DET_CUR           0x15
#define VP886_R_SADC_SEL_GNDKEY_CUR             0x16
#define VP886_R_SADC_SEL_LONG_CAL_OUT           0x17
#define VP886_R_SADC_SEL_LONG_CAL_IN            0x18
#define VP886_R_SADC_SEL_METTALIC_CAL_OUT       0x19
#define VP886_R_SADC_SEL_METTALIC_CAL_IN        0x1A
#define VP886_R_SADC_SEL_SW_CAL_CMP_HIGH        0x1B
#define VP886_R_SADC_SEL_SW_CAL_CMP_LOW         0x1C
#define VP886_R_SADC_SEL_BAT_SAT_DET_CMP        0x1D

/* A6/A7 : Voice A/D Converter Configuration */
#define VP886_R_VADC_WRT                        0xA6
#define VP886_R_VADC_RD                         0xA7
#define VP886_R_VADC_LEN                        6
    /* Byte 0 */
#define VP886_R_VADC_TX_INTERRUPT               0x80
#define VP886_R_VADC_MATH                       0x40
#define VP886_R_VADC_SM_OVERRIDE                0x10
#define VP886_R_VADC_DRATE                      0x07
#define VP886_R_VADC_DRATE_8KHZ                 0x00
#define VP886_R_VADC_DRATE_4KHZ                 0x01
#define VP886_R_VADC_DRATE_2KHZ                 0x02
#define VP886_R_VADC_DRATE_1KHZ                 0x03
#define VP886_R_VADC_DRATE_500HZ                0x04
#define VP886_R_VADC_DRATE_250HZ                0x05
#define VP886_R_VADC_DRATE_125HZ                0x06
    /* Byte 1 */
#define VP886_R_VADC_SEL                        0x0F
#define VP886_R_VADC_SEL_TIP_RING_AC_V          0x00
#define VP886_R_VADC_SEL_SWY                    0x01
#define VP886_R_VADC_SEL_SWZ                    0x02
#define VP886_R_VADC_SEL_TIP_GROUND_V           0x04
#define VP886_R_VADC_SEL_RING_GROUND_V          0x05
#define VP886_R_VADC_SEL_TIP_RING_DC_V          0x06
#define VP886_R_VADC_SEL_METALLIC_CUR           0x07
#define VP886_R_VADC_SEL_LOWGAIN_LONG_CUR       0x07
#define VP886_R_VADC_SEL_LONG_CUR               0x08
#define VP886_R_VADC_SEL_LOWGAIN_METALLIC_CUR   0x08
#define VP886_R_VADC_SEL_HIGH_CAL_CUR           0x09
#define VP886_R_VADC_SEL_VDAC_LOOPBACK          0x0A
#define VP886_R_VADC_SEL_ADC_OFFSET             0x0B
#define VP886_R_VADC_SEL_LOW_CAL_CUR            0x0C


/* A8/A9 : Dither Cancel Parameters */
#define VP886_R_DCANCEL_WRT                     0xA8
#define VP886_R_DCANCEL_RD                      0xA9
#define VP886_R_DCANCEL_LEN                     16


/* AA/AB : Test Mode Access */
#define VP886_R_TM_ACCESS_WRT                   0xAA
#define VP886_R_TM_ACCESS_RD                    0xAB
#define VP886_R_TM_ACCESS_LEN                   1


/* AE/AF : Calibration Control */
#define VP886_R_CALCTRL_WRT                     0xAE
#define VP886_R_CALCTRL_RD                      0xAF
#define VP886_R_CALCTRL_LEN                     3
    /* Byte 0 */
#define VP886_R_CALCTRL_SWZ_INP_SEL             0xC0
#define VP886_R_CALCTRL_SWZ_INP_SEL_CONN        0x00
#define VP886_R_CALCTRL_SWZ_INP_SEL_DISC        0x40
#define VP886_R_CALCTRL_SWZ_INP_SEL_CAL_H       0x80
#define VP886_R_CALCTRL_SWZ_INP_SEL_CAL_L       0xC0
#define VP886_R_CALCTRL_SWY_INP_SEL             0x30
#define VP886_R_CALCTRL_SWY_INP_SEL_CONN        0x00
#define VP886_R_CALCTRL_SWY_INP_SEL_DISC        0x10
#define VP886_R_CALCTRL_SWY_INP_SEL_CAL_H       0x20
#define VP886_R_CALCTRL_SWY_INP_SEL_CAL_L       0x30
#define VP886_R_CALCTRL_ILA_INP_SEL             0x0C
#define VP886_R_CALCTRL_ILA_INP_SEL_CONN        0x00
#define VP886_R_CALCTRL_ILA_INP_SEL_DISC        0x04
#define VP886_R_CALCTRL_ILA_INP_SEL_CAL_H       0x08
#define VP886_R_CALCTRL_ILA_INP_SEL_CAL_L       0x0C
#define VP886_R_CALCTRL_ILG_INP_SEL             0x03
#define VP886_R_CALCTRL_ILG_INP_SEL_CONN        0x00
#define VP886_R_CALCTRL_ILG_INP_SEL_DISC        0x01
#define VP886_R_CALCTRL_ILG_INP_SEL_CAL_H       0x02
#define VP886_R_CALCTRL_ILG_INP_SEL_CAL_L       0x03
    /* Byte 1 */
#define VP886_R_CALCTRL_SWZ_CORR_DIS            0x80
#define VP886_R_CALCTRL_DITHER_EN               0x40
#define VP886_R_CALCTRL_SWY_CORR_DIS            0x20
#define VP886_R_CALCTRL_RING_CORR_DIS           0x10
#define VP886_R_CALCTRL_TIP_CORR_DIS            0x08
#define VP886_R_CALCTRL_IO2_CORR_DIS            0x04
#define VP886_R_CALCTRL_60V_CLAMP_SEL           0x03
#define VP886_R_CALCTRL_60V_CLAMP_SEL_NORM      0x00
#define VP886_R_CALCTRL_60V_CLAMP_SEL_CLAMP     0x01
#define VP886_R_CALCTRL_60V_CLAMP_SEL_EXT       0x02
    /* Byte 2 */
#define VP886_R_CALCTRL_RING_INP_SEL            0x30
#define VP886_R_CALCTRL_RING_INP_SEL_CONN       0x00
#define VP886_R_CALCTRL_RING_INP_SEL_DISC       0x10
#define VP886_R_CALCTRL_RING_INP_SEL_CAL_H      0x20
#define VP886_R_CALCTRL_RING_INP_SEL_CAL_L      0x30
#define VP886_R_CALCTRL_TIP_INP_SEL             0x0C
#define VP886_R_CALCTRL_TIP_INP_SEL_CONN        0x00
#define VP886_R_CALCTRL_TIP_INP_SEL_DISC        0x04
#define VP886_R_CALCTRL_TIP_INP_SEL_CAL_H       0x08
#define VP886_R_CALCTRL_TIP_INP_SEL_CAL_L       0x0C
#define VP886_R_CALCTRL_IO2_INP_SEL             0x03
#define VP886_R_CALCTRL_IO2_INP_SEL_CONN        0x00
#define VP886_R_CALCTRL_IO2_INP_SEL_DISC        0x01
#define VP886_R_CALCTRL_IO2_INP_SEL_CAL_H       0x02
#define VP886_R_CALCTRL_IO2_INP_SEL_CAL_L       0x03


/* B0/B1 : State Independent Calibration Data */
#define VP886_R_INDCAL_WRT                      0xB0
#define VP886_R_INDCAL_RD                       0xB1
#define VP886_R_INDCAL_LEN                      3
    /* Byte 0 */
#define VP886_R_INDCAL_LP_SHCOR                 0xF0
#define VP886_R_INDCAL_GND_KEY                  0x0F
    /* Byte 1 */
#define VP886_R_INDCAL_DITHER_CORR_EN           0x80
#define VP886_R_INDCAL_60VCLAMP                 0x0F
    /* Byte 2 */
#define VP886_R_INDCAL_BATTERY_SAT              0x0F


/* B2/B3 : State Normal Polarity Calibration Data */
#define VP886_R_NORMCAL_WRT                     0xB2
#define VP886_R_NORMCAL_RD                      0xB3
#define VP886_R_NORMCAL_LEN                     3
    /* Byte 0 */
#define VP886_R_NORMCAL_SHCOR                   0xF0
#define VP886_R_NORMCAL_EXTBATCOR               0x0F
    /* Byte 1 */
#define VP886_R_NORMCAL_VOCCOR                  0xF0
#define VP886_R_NORMCAL_ILACOR                  0x0F
    /* Byte 2 */
#define VP886_R_NORMCAL_VASOFFHKCOR             0xF0    /* Tracker */
#define VP886_R_NORMCAL_VASONHKCOR              0x0F    /* Tracker */
#define VP886_R_NORMCAL_BATSWITCH               0xF0    /* ABS */

/* B4/B5 : State Reverse Polarity Calibration Data */
#define VP886_R_REVCAL_WRT                      0xB4
#define VP886_R_REVCAL_RD                       0xB5
#define VP886_R_REVCAL_LEN                      3
    /* Byte 0 */
#define VP886_R_REVCAL_SHCOR                    0xF0
#define VP886_R_REVCAL_EXTBATCOR                0x0F
    /* Byte 1 */
#define VP886_R_REVCAL_VOCCOR                   0xF0
#define VP886_R_REVCAL_ILACOR                   0x0F
    /* Byte 2 */
#define VP886_R_REVCAL_VASOFFHKCOR              0xF0    /* Tracker */
#define VP886_R_REVCAL_VASONHKCOR               0x0F    /* Tracker */
#define VP886_R_REVCAL_BATSWITCH                0xF0    /* ABS */


/* B6/B7 : State Ringing Calibration Data */
#define VP886_R_RINGCAL_WRT                     0xB6
#define VP886_R_RINGCAL_RD                      0xB7
#define VP886_R_RINGCAL_LEN                     2
    /* Byte 0 */
#define VP886_R_RINGCAL_EXTBATCOR               0xF0
#define VP886_R_RINGCAL_VOCCOR                  0x0F
    /* Byte 1 */
#define VP886_R_RINGCAL_VASCOR                  0xF0
#define VP886_R_RINGCAL_ILRCOR                  0x0F


/* C0/C1 : Ringing Generator Parameters */
#define VP886_R_RINGGEN_WRT                     0xC0
#define VP886_R_RINGGEN_RD                      0xC1
#define VP886_R_RINGGEN_LEN                     11
    /* Byte 0 */
#define VP886_R_RINGGEN_RT_HALFCYC              0x10
#define VP886_R_RINGGEN_RT_CYCDUR               0x08
#define VP886_R_RINGGEN_RAMP_SLOPE              0x04
#define VP886_R_RINGGEN_OPERATION               0x02
#define VP886_R_RINGGEN_OPERATION_CONT          0x00
#define VP886_R_RINGGEN_OPERATION_RAMP          0x02
#define VP886_R_RINGGEN_WAVE                    0x01
#define VP886_R_RINGGEN_WAVE_SINE               0x00
#define VP886_R_RINGGEN_WAVE_TRAP               0x01


/* C2/C3 : Loop Supervision Parameters */
#define VP886_R_LOOPSUP_WRT                     0xC2
#define VP886_R_LOOPSUP_RD                      0xC3
#define VP886_R_LOOPSUP_LEN                     5
    /* Byte 0 */
#define VP886_R_LOOPSUP_GKEY_ABS                0x80
#define VP886_R_LOOPSUP_LONG_BAL_ENH            0x40
#define VP886_R_LOOPSUP_GKEY_THRESH             0x38
#define VP886_R_LOOPSUP_HOOK_THRESH             0x07
    /* Byte 1 */
#define VP886_R_LOOPSUP_GKEY_DBNC               0xE0
#define VP886_R_LOOPSUP_HOOK_DBNC               0x1F
    /* Byte 2 */
#define VP886_R_LOOPSUP_RTRIP_ALG               0x80
#define VP886_R_LOOPSUP_RTRIP_ALG_DC            0x00
#define VP886_R_LOOPSUP_RTRIP_ALG_AC            0x80
#define VP886_R_LOOPSUP_RTRIP_THRESH            0x7F
    /* Byte 3 */
#define VP886_R_LOOPSUP_LPM_HOOK_THRESH         0xE0
#define VP886_R_LOOPSUP_RING_CUR_LIM            0x1F
    /* Byte 4 */
#define VP886_R_LOOPSUP_LPM_HOOK_HYST           0xC0
#define VP886_R_LOOPSUP_GKEY_HYST               0x30
#define VP886_R_LOOPSUP_HOOK_HYST               0x0C
#define VP886_R_LOOPSUP_HSH_CTL_DBNC            0x02
#define VP886_R_LOOPSUP_HOOK_AVG_EN             0x01


/* C4/C5 : Ringing Phase Offset Register (Ring Entry Delay) */
#define VP886_R_RINGDELAY_WRT                   0xC4
#define VP886_R_RINGDELAY_RD                    0xC5
#define VP886_R_RINGDELAY_LEN                   1
#define VP886_R_RINGDELAY_DEFAULT               0x04    /* Special case */
#define VP886_R_RINGDELAY_BASE                  0x05


/* C6/C7 : DC Feed Parameters */
#define VP886_R_DCFEED_WRT                      0xC6
#define VP886_R_DCFEED_RD                       0xC7
#define VP886_R_DCFEED_LEN                      2
    /* Byte 0 */
#define VP886_R_DCFEED_IR_OVERHEAD              0xC0
#define VP886_R_DCFEED_VOCSHIFT                 0x20
#define VP886_R_DCFEED_VOC                      0x1C
#define VP886_R_DCFEED_VAS_MSB                  0x03
    /* Byte 1 */
#define VP886_R_DCFEED_VAS_LSB                  0xC0
#define VP886_R_DCFEED_LONG_IMPED               0x20
#define VP886_R_DCFEED_ILA                      0x1F


/* C9 : Hook Buffer */
#define VP886_R_HOOKBUF_RD                      0xC9
#define VP886_R_HOOKBUF_LEN                     4
    /* Byte 0 */
#define VP886_R_HOOKBUF_COUNT                   0xF8
#define VP886_R_HOOKBUF_HOOKS                   0x07
#define VP886_R_HOOKBUF_HK2                     0x04
#define VP886_R_HOOKBUF_HK1                     0x02
#define VP886_R_HOOKBUF_HK0                     0x01


/* CA/CB : Digital Impedance Scaling Network (DISN) */
#define VP886_R_DISN_WRT                        0xCA
#define VP886_R_DISN_RD                         0xCB
#define VP886_R_DISN_LEN                        1


/* CF : Voice/Test Buffer */
#define VP886_R_VBUFFER_RD                      0xCF
#define VP886_R_VBUFFER_LEN                     25
    /* Byte 0 */
#define VP886_R_VBUFFER_OVERFLOW                0xC0
#define VP886_R_VBUFFER_POINTER                 0x2F


/* D0/D1 : Metering Parameters */
#define VP886_R_METER_WRT                       0xD0
#define VP886_R_METER_RD                        0xD1
#define VP886_R_METER_LEN                       4
    /* Byte 0 */
#define VP886_R_METER_FREQ                      0x80
#define VP886_R_METER_PEAK_VOLTAGE              0x7F
    /* Byte 1 */
#define VP886_R_METER_SOREV                     0x80
#define VP886_R_METER_SLOPE                     0x7F


/* D2/D3 : Signal Generator A, B, and Bias Parameters */
#define VP886_R_SIGAB_WRT                       0xD2
#define VP886_R_SIGAB_RD                        0xD3
#define VP886_R_SIGAB_LEN                       11
    /* Byte 0 */
#define VP886_R_SIGAB_SLOPE                     0x04
#define VP886_R_SIGAB_SLOPE_POS                 0x00
#define VP886_R_SIGAB_SLOPE_NEG                 0x04
#define VP886_R_SIGAB_OPERATION                 0x02
#define VP886_R_SIGAB_OPERATION_CONT            0x00
#define VP886_R_SIGAB_OPERATION_RAMP            0x02
#define VP886_R_SIGAB_WAVE                      0x01
#define VP886_R_SIGAB_WAVE_SINE                 0x00
#define VP886_R_SIGAB_WAVE_TRAP                 0x01


/* D4/D5 : Signal Generator C and D Parameters */
#define VP886_R_SIGCD_WRT                       0xD4
#define VP886_R_SIGCD_RD                        0xD5
#define VP886_R_SIGCD_LEN                       8
    /* Byte 4 */
#define VP886_R_SIGCD_FM_ENABLE                 0x80
#define VP886_R_SIGCD_FREQD_MSB                 0x7F


/* D8/D9 : Channel Timer */
#define VP886_R_CHTIMER_WRT                     0xD8
#define VP886_R_CHTIMER_RD                      0xD9
#define VP886_R_CHTIMER_LEN                     2
    /* Byte 0 */
#define VP886_R_CHTIMER_ENABLE                  0x80
#define VP886_R_CHTIMER_MODE                    0x40
#define VP886_R_CHTIMER_MODE_CONT               0x00
#define VP886_R_CHTIMER_MODE_1SHOT              0x40
#define VP886_R_CHTIMER_TIME_MSB                0x3F
    /* Byte 1 */
#define VP886_R_CHTIMER_INSTANT_INTERRUPT       0x00


/* DE/DF : Signal Generator Control */
#define VP886_R_SIGCTRL_WRT                     0xDE
#define VP886_R_SIGCTRL_RD                      0xDF
#define VP886_R_SIGCTRL_LEN                     1
#define VP886_R_SIGCTRL_CADENCE                 0x80
#define VP886_R_SIGCTRL_MODE                    0x40
#define VP886_R_SIGCTRL_MODE_CONT               0x00
#define VP886_R_SIGCTRL_MODE_1SHOT              0x40
#define VP886_R_SIGCTRL_EN_BIAS                 0x10
#define VP886_R_SIGCTRL_EN_SIGABCD              0x0F
#define VP886_R_SIGCTRL_EN_SIGD                 0x08
#define VP886_R_SIGCTRL_EN_SIGC                 0x04
#define VP886_R_SIGCTRL_EN_SIGB                 0x02
#define VP886_R_SIGCTRL_EN_SIGA                 0x01


/* E0/E1 : Cadence Timer */
#define VP886_R_CADENCE_WRT                     0xE0
#define VP886_R_CADENCE_RD                      0xE1
#define VP886_R_CADENCE_LEN                     4
#define VP886_R_CADENCE_MSB_MASK                0x07


/* E2/E3 : Caller Identification Number Data */
#define VP886_R_CIDDATA_WRT                     0xE2
#define VP886_R_CIDDATA_RD                      0xE3
#define VP886_R_CIDDATA_LEN                     1


/* E4/E5 : Switching Regulator Parameters */
#define VP886_R_SWPARAM_WRT                     0xE4
#define VP886_R_SWPARAM_RD                      0xE5
#define VP886_R_SWPARAM_LEN                     3
    /* Byte 0 */
#define VP886_R_SWPARAM_POWER_MODE              0x80    /* Tracker */
#define VP886_R_SWPARAM_POWER_MODE_AUTO         0x00    /* Tracker */
#define VP886_R_SWPARAM_POWER_MODE_MANUAL       0x80    /* Tracker */
#define VP886_R_SWPARAM_RING_TRACKING           0x40    /* Tracker */
#define VP886_R_SWPARAM_RING_TRACKING_EN        0x00    /* Tracker */
#define VP886_R_SWPARAM_RING_TRACKING_DIS       0x40    /* Tracker */
#define VP886_R_SWPARAM_COUNTER                 0x20
#define VP886_R_SWPARAM_ABS_V                   0x1F    /* ABS */
#define VP886_R_SWPARAM_FLOOR_V                 0x1F    /* Tracker */
    /* Byte 1 */
#define VP886_R_SWPARAM_SWOVP                   0x80
#define VP886_R_SWPARAM_IO2OVP                  0x40
#define VP886_R_SWPARAM_RINGING_V               0x1F    /* Tracker */
    /* Byte 2 */
#define VP886_R_SWPARAM_LOWPOWER_V              0x1F    /* Tracker */


/* E8/E9 : Battery Calibration Data Register */
#define VP886_R_BATCAL_WRT                      0xE8
#define VP886_R_BATCAL_RD                       0xE9
#define VP886_R_BATCAL_LEN                      2
    /* Byte 0 */
#define VP886_R_BATCAL_FLOOR                    0xF0    /* Tracker */
#define VP886_R_BATCAL_RING                     0x0F    /* Tracker */
#define VP886_R_BATCAL_FLOOR_ABS                0x78    /* ABS */
#define VP886_R_BATCAL_HYSTERESIS               0x06    /* ABS */
#define VP886_R_BATCAL_ABS_ALG                  0x01    /* ABS */
#define VP886_R_BATCAL_ABS_ALG_ORIG             0x00    /* ABS */
#define VP886_R_BATCAL_ABS_ALG_NEW              0x01    /* ABS */
    /* Byte 1 */
#define VP886_R_BATCAL_LOWPOWER                 0xF0    /* Tracker */
#define VP886_R_BATCAL_SW_LIMIT                 0x0F    /* ABS */


/* EA/EB : Caller Identification Number Parameters */
#define VP886_R_CIDPARAM_WRT                    0xEA
#define VP886_R_CIDPARAM_RD                     0xEB
#define VP886_R_CIDPARAM_LEN                    1
#define VP886_R_CIDPARAM_ST                     0xE0
#define VP886_R_CIDPARAM_ST_IDLE                0x00
#define VP886_R_CIDPARAM_ST_EMPTY               0x20
#define VP886_R_CIDPARAM_ST_ONETHIRD            0x40
#define VP886_R_CIDPARAM_ST_TWOTHIRDS           0x60
#define VP886_R_CIDPARAM_ST_UNDERRUN            0xA0
#define VP886_R_CIDPARAM_ST_FULL                0xC0
#define VP886_R_CIDPARAM_ST_FINISHING           0xE0
#define VP886_R_CIDPARAM_EOM                    0x10
#define VP886_R_CIDPARAM_FRAME_DIS              0x08
#define VP886_R_CIDPARAM_START_BIT              0x04
#define VP886_R_CIDPARAM_START_BIT_0            0x00
#define VP886_R_CIDPARAM_START_BIT_1            0x04
#define VP886_R_CIDPARAM_STOP_BIT               0x02
#define VP886_R_CIDPARAM_STOP_BIT_0             0x00
#define VP886_R_CIDPARAM_STOP_BIT_1             0x02
#define VP886_R_CIDPARAM_CID_DIS                0x01
#define VP886_R_CIDPARAM_CID_EN                 0x00


/* EC/ED Internal Configuration Register 1 */
#define VP886_R_ICR1_WRT                        0xEC
#define VP886_R_ICR1_RD                         0xED
#define VP886_R_ICR1_LEN                        4
    /* Byte 0 (mask) and 1 */
#define VP886_R_ICR1_TIP_BIAS                   0xF0
#define VP886_R_ICR1_SLIC_BIAS                  0x0F
    /* Byte 2 (mask) and 3 */
#define VP886_R_ICR1_LPM                        0xC0
#define VP886_R_ICR1_LP_STATE                   0x80
#define VP886_R_ICR1_NON_LP_STATE               0x40
#define VP886_R_ICR1_BAT                        0x30
#define VP886_R_ICR1_BAT_VBL                    0x00
#define VP886_R_ICR1_BAT_VBH                    0x20
#define VP886_R_ICR1_RING_BIAS                  0x0F


/* EE/EF Internal Configuration Register 2 */
#define VP886_R_ICR2_WRT                        0xEE
#define VP886_R_ICR2_RD                         0xEF
#define VP886_R_ICR2_LEN                        4
    /* Byte 0 (mask) and 1 */
#define VP886_R_ICR2_VOC_DAC_EN                 0x20
#define VP886_R_ICR2_DAC_RING_LEVELS            0x10
#define VP886_R_ICR2_C_TIP_ON                   0x08
#define VP886_R_ICR2_C_RING_ON                  0x04
    /* Byte 2 (mask) and 3 */
#define VP886_R_ICR2_SPEEDUP_MET                0x80
#define VP886_R_ICR2_SPEEDUP_BAT                0x40
#define VP886_R_ICR2_SWITCHER_EN                0x20
#define VP886_R_ICR2_SW_DAC_CTRL                0x10
#define VP886_R_ICR2_SW_LIM                     0x0C
#define VP886_R_ICR2_SW_LIM_50                  0x04
#define VP886_R_ICR2_SW_LIM_100                 0x08
#define VP886_R_ICR2_SW_LIM_150                 0x0C
#define VP886_R_ICR2_RINGING_TC                 0x01


/* F2/F3 Internal Configuration Register 3 */
#define VP886_R_ICR3_WRT                        0xF2
#define VP886_R_ICR3_RD                         0xF3
#define VP886_R_ICR3_LEN                        4
    /* Byte 0 (mask) and 1 */
#define VP886_R_ICR3_SPEEDUP_LONG               0x10
#define VP886_R_ICR3_VREF_EN                    0x01
    /* Byte 2 (mask) and 3 */
#define VP886_R_ICR3_LONG_ON                    0x80
#define VP886_R_ICR3_LONG_UNCLAMP               0x40
#define VP886_R_ICR3_SWZ_TO_LONG                0x20
#define VP886_R_ICR3_LONG_60V_CLAMP_ON          0x10
#define VP886_R_ICR3_SWY_TO_LONG                0x08
#define VP886_R_ICR3_RINGING_ON                 0x02


/* F4/F5 Internal Configuration Register 4 */
#define VP886_R_ICR4_WRT                        0xF4
#define VP886_R_ICR4_RD                         0xF5
#define VP886_R_ICR4_LEN                        4
    /* Byte 0 (mask) and 1 */
#define VP886_R_ICR4_SMALL_SIGNAL_CTRL          0x80
#define VP886_R_ICR4_AISN_CTRL                  0x10
#define VP886_R_ICR4_VDAC_EN                    0x02
#define VP886_R_ICR4_VADC_EN                    0x01
    /* Byte 2 (mask) and 3 */
#define VP886_R_ICR4_GKEY_DAC_EN                0x20
#define VP886_R_ICR4_GKEY_POLR                  0x10
#define VP886_R_ICR4_GKEY_POLR_POS              0x00
#define VP886_R_ICR4_GKEY_POLR_NEG              0x10
#define VP886_R_ICR4_SUPD_DAC_EN                0x08
#define VP886_R_ICR4_SUPD_VORI                  0x04
#define VP886_R_ICR4_SUPD_ABSV                  0x01


/* FA/FB Internal Configuration Register 5 */
#define VP886_R_ICR5_WRT                        0xFA
#define VP886_R_ICR5_RD                         0xFB
#define VP886_R_ICR5_LEN                        3
    /* byte 0 */
#define VP886_R_ICR5_FEED_HOLD                  0xF0
#define VP886_R_ICR5_BAT_HOLD                   0x0F
    /* byte 1 */
#define VP886_R_ICR5_H2MS                       0xF0
#define VP886_R_ICR5_SUBST                      0x0F
    /* byte 2 */
#define VP886_R_ICR5_DISC_ENTRY_HOLD            0xF0
#define VP886_R_ICR5_LPM_HOLD                   0x0F


/* FC/FD Internal Configuration Register 6 */
#define VP886_R_ICR6_WRT                        0xFC
#define VP886_R_ICR6_RD                         0xFD
#define VP886_R_ICR6_LEN                        4
    /* Byte 0 (mask) and 1 */
#define VP886_R_ICR6_VOC_CORR_DAC_EN            0x80
#define VP886_R_ICR6_VAS_DAC_EN                 0x20
    /* Byte 2 (mask) and 3 */



#endif /* VP886_REGISTERS_H */
