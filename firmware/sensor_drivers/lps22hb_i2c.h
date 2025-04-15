#ifndef LPS22HB_I2C_H
#define LPS22HB_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h" // Adjust to your MCU family, e.g. stm32xxxx_hal.h

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------*/
/* REGISTER DEFINITIONS (PARTIAL)                                             */
/*----------------------------------------------------------------------------*/
#define LPS22HB_REG_WHO_AM_I 0x0F
#define LPS22HB_WHO_AM_I_VAL 0xB1 // expected WHO_AM_I value for LPS22HB

#define LPS22HB_REG_PRESS_OUT_XL 0x28
#define LPS22HB_REG_PRESS_OUT_L 0x29
#define LPS22HB_REG_PRESS_OUT_H 0x2A

#define LPS22HB_REG_TEMP_OUT_L 0x2B
#define LPS22HB_REG_TEMP_OUT_H 0x2C

#define LPS22HB_REG_STATUS 0x27
#define LPS22HB_STATUS_PRESS_READY 0x01
#define LPS22HB_STATUS_TEMP_READY 0x02

#define LPS22HB_REG_CTRL_1 0x10
#define LPS22HB_REG_CTRL_2 0x11
#define LPS22HB_REG_CTRL_3 0x12

/* Example default I2C addresses:
 * If SA0 pin is low => 7-bit device addr= 0x5C,
 * If SA0 pin is high => 7-bit device addr= 0x5D.
 * 8-bit addresses => 0xB8 or 0xBA (for writes), 0xB9 or 0xBB (for reads).
 */
#define LPS22HB_I2C_ADDR_0 (0x5C << 1)
#define LPS22HB_I2C_ADDR_1 (0x5D << 1)

    /*----------------------------------------------------------------------------*/
    /* DEVICE HANDLE                                                              */
    /*----------------------------------------------------------------------------*/
    typedef struct
    {
        I2C_HandleTypeDef *hi2c; ///< Pointer to I2C HAL handle
        uint8_t i2cAddress;      ///< 8-bit I2C device address (e.g. 0xB8 if SA0=0)
    } LPS22HB_I2C_Handle_t;

    /*----------------------------------------------------------------------------*/
    /* PUBLIC DRIVER API                                                           */
    /*----------------------------------------------------------------------------*/
    int LPS22HB_I2C_Init(LPS22HB_I2C_Handle_t *dev);
    int LPS22HB_I2C_ReadPressure(LPS22HB_I2C_Handle_t *dev, int32_t *pressure);
    int LPS22HB_I2C_ReadTemp(LPS22HB_I2C_Handle_t *dev, int16_t *temp);
    int LPS22HB_I2C_ReadReg(LPS22HB_I2C_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
    int LPS22HB_I2C_WriteReg(LPS22HB_I2C_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
    int LPS22HB_I2C_WhoAmI(LPS22HB_I2C_Handle_t *dev);
    int LPS22HB_I2C_Status(LPS22HB_I2C_Handle_t *dev, uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif // LPS22HB_I2C_H
