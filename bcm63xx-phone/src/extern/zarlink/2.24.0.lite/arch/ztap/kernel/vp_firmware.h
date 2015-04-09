
#ifndef VP_FIRMWARE_H
#define VP_FIRMWARE_H

#ifdef ZARLINK_CFG_INTERNAL

typedef struct {
    uint16 *pImage;
    uint32  length;
} VpApiMod792FirmwareType;

typedef struct VpApiMod792Image {
    uint16    *pImage;
    uint32    size;
    
} VpApiMod792FirmWareType;

extern int VpApiIoctl792Firmware (
    unsigned long arg);


extern VpApiMod792FirmwareType g792Firmware;
#endif

#endif /* ZARLINK_CFG_INTERNAL */
