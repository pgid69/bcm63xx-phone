

#include <fcntl.h>

#include "sdk_qs_pin_ctrl.h"

/**
 * sdk_qs_pin_ctrl.c
 *
 * Most of the ZTAP SM and DIN connectors pins are directly controlled by the ZTAP FPGA.
 * All SM an DIN pins have a default behavior as an input (interrupts) or an 
 * output (chip selects, clocks etc). The default behavior can be overridden by using 
 * the functions in this library. 
 *
 * Many (NOT ALL) FPGA pins are can be configured as inputs or outputs. See the ZTAP 
 * User's Guide Section 2.3.4 "Connector Definitions" for direction info.
 *
 * Pins that can be configured as outputs can be driven high or low.
 * Pins that can be configured as inputs are tristated.
 *
 * Example usage: Override ZTAP DIN connector PCLK signal to a low value isntead of a
 *                square wave clock:
 *               
 *                PinControlSet(DIN_PCLK, PIN_CTRL_OUTPUT_LOW);
 *                PinControlEnable();
 * 
 * Appliactions that enable the pin control feature MUST also disable the feature if
 * the user expets the ZTAP to function properly after the application completes.
 *
 * The feature can be disabled from the ZTAP command line with the following:
 * 
 *    "pin override disable"
 */
 
/**
 * Function:  PinControlEnable()
 *
 * Description: This function enables the ZTAP pin control override feature.
 * 
 * Result: Any pins not configured as default (PIN_CTRL_DEFAULT) will be overridden as:
 *
 *         a tristated input (PIN_CTRL_INPUT_TRI)
 *         a forced output low (PIN_CTRL_OUTPUT_LOW)
 *         a forced output high (PIN_CTRL_OUTPUT_HIGH)
 * 
 *         as specified by the PinControlSet() function.
 */
int PinControlEnable(void)
{
    unsigned long mask = TELECOM_FPGA_PIN_ORIDE_MASK | TELECOM_FPGA_PIN_ORIDE_EN;
    unsigned long data = TELECOM_FPGA_PIN_ORIDE_MASK | TELECOM_FPGA_PIN_ORIDE_EN;
    
    if (TelecomLibRegWrite(FEATURE_CTRL_REG, mask, data) < 0) {
        fprintf(stderr, "  unable to enable pin control\n");
        return -1;
    }

    return 0;
}

/**
 * Function:  PinControlDisable()
 *
 * Description: This function disables the ZTAP pin control override feature.
 * 
 * Result: All pins will fall back to their default behavior:
 *
 */
int PinControlDisable(void)
{
    unsigned long mask = TELECOM_FPGA_PIN_ORIDE_MASK | TELECOM_FPGA_PIN_ORIDE_EN;
    unsigned long data = TELECOM_FPGA_PIN_ORIDE_MASK;
    
    if (TelecomLibRegWrite(FEATURE_CTRL_REG, mask, data) < 0) {
        fprintf(stderr, "  unable to disable pin control\n");
        return -1;
    }

    return 0;
}

/**
 * Function:  PinControlSet()
 *
 * Arguments:
 *     pinAddress - Address of the pin to configure ( see sdk_qs_pin_ctrl.h )
 *     ioLogic -    Logic behavior
 *
 * Description: This function overrides a pins behavior. The requested behavior only takes
 *              place if the PinControlEnable() function is called.
 * 
 *         Legal pinAddress are defined in sdk_qs_pin_ctrl.h
 * 
 *         Legal ioLogic values are:
 *              
 *             PIN_CTRL_INPUT_TRI   - force pin to be an input.
 *             PIN_CTRL_OUTPUT_LOW  - driver the pin low
 *             PIN_CTRL_OUTPUT_HIGH - drive the pin high
 *             PIN_CTRL_DEFAULT     - force pin back to default direction/functionality.
 * 
 * Result: Pins set to any value other than PIN_CTRL_DEFAULT will be override to
 *         reflect the 
 *
 * Example usage: Override ZTAP DIN connector PCLK signal to a low value isntead of a
 *                square wave clock:
 *               
 *                PinControlSet(DIN_PCLK, PIN_CTRL_OUTPUT_LOW);
 *                PinControlEnable();
 *
 */
int PinControlSet(
    unsigned short pinAddress, 
    PinControlLogicTypes ioLogic) 
{

    /* ensure that the requested pin is in the proper address range */
    if ((pinAddress < PIN_CTRL_MIN_ADDR) || 
        (pinAddress > PIN_CTRL_MAX_ADDR) || 
        (pinAddress % 4)) {
        fprintf(stderr, "  unable to set pin control: invalid pinAddress %i\n", pinAddress);
        return -1;
    }

    if (TelecomLibRegWrite(pinAddress, 0x7, ioLogic) < 0) {
        fprintf(stderr, "  unable to set pin control\n");
        return -1;
    }

    return 0;
}
