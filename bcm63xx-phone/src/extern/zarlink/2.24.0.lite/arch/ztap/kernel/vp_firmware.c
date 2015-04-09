
#ifdef MODULE

#include <linux/module.h>
#include <asm/uaccess.h>        /* loacation copy_xyz_user*/
#include <linux/slab.h>         /* kmalloc and kfree */

#include "vp_api_types.h"


#ifdef ZARLINK_CFG_INTERNAL
#include "vp_firmware.h"
VpApiMod792FirmwareType g792Firmware = {NULL, 0};

extern int VpApiIoctl792Firmware (
    unsigned long arg)
{
    VpApiMod792FirmWareType firmwareInfo;

    /* copy info from user space */
    unsigned long cu = copy_from_user( &firmwareInfo, (void*)arg, sizeof(VpApiMod792FirmWareType) );

    /* decide if we are storing or freeing a 792 firmware image */
    if (firmwareInfo.size && firmwareInfo.pImage && !cu) {
        uint16 *pImage = kmalloc( (sizeof(uint16) * firmwareInfo.size) , GFP_KERNEL);

        if (pImage == NULL) {
            printk("no mem\n");
            return -1;
        }

        if (firmwareInfo.pImage == NULL) {
            printk("no image\n");
            return -1;
        }
        
        memcpy(pImage, firmwareInfo.pImage, (sizeof(uint16) * firmwareInfo.size));
        g792Firmware.pImage = pImage;
        g792Firmware.length = firmwareInfo.size;
    } else {
        if (g792Firmware.pImage == NULL) {
            return 0;
        }

        kfree(g792Firmware.pImage);
        g792Firmware.pImage = NULL;
    }

    return 0;
}


#endif  /* ZARLINK_CFG_INTERNAL */
#endif  /* MODULE */

