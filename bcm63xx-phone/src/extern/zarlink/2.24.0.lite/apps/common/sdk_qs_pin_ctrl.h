#ifndef SDK_QS_PIN_CTRL
#define SDK_QS_PIN_CTRL

#include "telecom_lib.h"

#define PIN_DEFAULT

typedef enum {
    PIN_CTRL_DEFAULT        = 0,
    PIN_CTRL_INPUT_TRI      = 1,
    PIN_CTRL_OUTPUT_LOW     = 4,
    PIN_CTRL_OUTPUT_HIGH    = 5,
    PIN_CTRL_ENUM_SIZE      = FORCE_STANDARD_C_ENUM_SIZE
} PinControlLogicTypes;

int PinControlEnable(void);
int PinControlDisable(void);
int PinControlSet(
    unsigned short pinAddress, 
    PinControlLogicTypes ioLogic);

#define PIN_CTRL_MIN_ADDR      0x0E00

#define DIN_GPI_0              0x0E00
#define DIN_GPI_1              0x0E04
#define DIN_GPI_2              0x0E08
#define DIN_GPI_3              0x0E0C
#define DIN_GPI_4              0x0E10
#define DIN_GPI_5              0x0E14
#define DIN_GPI_6              0x0E18
#define DIN_GPI_7_SPI_MISO     0x0E1C
#define DIN_GPI_8              0x0E20
#define DIN_GPI_9              0x0E24
#define DIN_GPI_10             0x0E28
#define DIN_GPI_11             0x0E2C
#define DIN_GPI_12             0x0E30
#define DIN_GPI_13             0x0E34
#define DIN_GPI_14             0x0E38
#define DIN_GPI_15             0x0E3C

#define SM_GPI_0               0x0E40
#define SM_GPI_1               0x0E44
#define SM_GPI_2               0x0E48
#define SM_GPI_3               0x0E4C
#define SM_GPI_4               0x0E50
#define SM_GPI_5               0x0E54
#define SM_GPI_6               0x0E58
#define SM_GPI_7_SPI_MISO      0x0E5C
#define SM_GPI_8               0x0E60
#define SM_GPI_9               0x0E64
#define SM_GPI_10              0x0E68
#define SM_GPI_11              0x0E6C
#define SM_GPI_12              0x0E70
#define SM_GPI_13              0x0E74
#define SM_GPI_14              0x0E78
#define SM_GPI_15              0x0E7C

#define DIN_CS_0               0x0E80
#define DIN_CS_1               0x0E84
#define DIN_CS_2               0x0E88
#define DIN_CS_3               0x0E8C
#define DIN_CS_4               0x0E90
#define DIN_CS_5               0x0E94
#define DIN_CS_6               0x0E98
#define DIN_CS_7               0x0E9C

#define SM_CS_0                0x0EA0
#define SM_CS_1                0x0EA4
#define SM_CS_2                0x0EA8
#define SM_CS_3                0x0EAC
#define SM_CS_4                0x0EB0
#define SM_CS_5                0x0EB4
#define SM_CS_6                0x0EB8
#define SM_CS_7                0x0EBC

#define DIN_INT_0              0x0EC0
#define DIN_INT_1              0x0EC4
#define DIN_INT_2              0x0EC8
#define DIN_INT_3              0x0ECC

#define SM_INT_0               0x0ED0
#define SM_INT_1               0x0ED4
#define SM_INT_2               0x0ED8
#define SM_INT_3               0x0EDC

#define RST_0                  0x0EE0
#define RST_1                  0x0EE4
#define RST_2                  0x0EE8
#define RST_3                  0x0EEC

#define DIN_DRA                0x0EF0
#define DIN_DRB                0x0EF4
#define DIN_DXA                0x0EF8
#define DIN_DXB                0x0EFC
#define DIN_FS                 0x0F00
#define DIN_GPI_ADDR           0x0F04
#define DIN_GPI_RD_SPI_MOSI    0x0F08
#define DIN_GPI_WAIT           0x0F0C
#define DIN_GPI_WR_SPI_CLK     0x0F10
#define DIN_MCLK               0x0F14
#define DIN_PCLK               0x0F18
#define DIN_RING_LL            0x0F1C
#define DIN_RING_SYNC          0x0F20
#define DIN_SPI_TXD_PULLUP     0x0F24

#define SM_DRA                 0x0F28
#define SM_DRB                 0x0F2C
#define SM_DXA                 0x0F30
#define SM_DXB                 0x0F34
#define SM_FS                  0x0F38
#define SM_GPI_ADDR            0x0F3C
#define SM_GPI_RD_SPI_MOSI     0x0F40
#define SM_GPI_WAIT            0x0F44
#define SM_GPI_WR_SPI_CLK      0x0F48
#define SM_MCLK                0x0F4C
#define SM_PCLK                0x0F50
#define SM_RING_LL             0x0F54
#define SM_RING_SYNC           0x0F58
#define SM_SPI_TXD_PULLUP      0x0F5C

#define FRAMER_DRA             0x0F60
#define FRAMER_DXA             0x0F64
#define FRAMER_FS              0x0F68
#define FRAMER_PCLK            0x0F6C

#define HOST_DRA               0x0F70
#define HOST_DXA               0x0F74
#define HOST_FS                0x0F78
#define HOST_PCLK              0x0F7C

#define PIN_CTRL_MAX_ADDR      0x0F7C

#endif /*SDK_QS_PIN_CTRL*/

