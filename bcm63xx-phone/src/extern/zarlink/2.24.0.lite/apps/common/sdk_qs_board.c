
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h> /* sleep() */

#include "sdk_qs_board.h"
#include "vp_api.h"
#include "sys_service.h"
#include "telecom_lib.h"

int fpgaDriver = -1;


/* This function includes any board-specific initialization that needs to be
   done at startup. */
void
BoardInit(
    void)
{
    /* Pull down the SLAC's reset signal for 100 ms. */
    if (TelecomLibRegWrite(RESET_CFG_REG, 0x606, 0x600) < 0) {
        printf("TelecomLibRegWrite() failed.\n");
    }
    usleep(100000);

    /* Raise the SLAC's reset signal and wait 100 ms. */
    if (TelecomLibRegWrite(RESET_CFG_REG, 0x606, 0x606) < 0) {
        printf("TelecomLibRegWrite() failed.\n");
    }
    usleep(500000);

    /* Initialize semaphore for critical sections: */
    if (VpSysInit() == FALSE) {
        printf("VpSysInit() failed.\n");
    }

    /* Open the ZTAP FPGA driver */
    fpgaDriver = open(TELECOM_MOD_FILE_DES, O_RDWR);
    if (fpgaDriver == -1) {
        printf("Can't open ZTAP FPGA driver, fd \"%s\"", TELECOM_MOD_FILE_DES);
    } else {
        ioctl(fpgaDriver, TELECOM_MOD_IOCTL_TSA_RESET);
    }

    return;
}
