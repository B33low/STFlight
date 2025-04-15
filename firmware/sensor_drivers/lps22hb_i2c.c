#include "lps22hb_i2c.h"
#include <string.h> // for memset, if needed

/*----------------------------------------------------------------------------*/
/* INTERNAL UTILITY FUNCTIONS                                                 */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* PUBLIC API IMPLEMENTATION                                                  */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Read (len) bytes starting at register 'reg'
 * @param  dev    Pointer to driver handle
 * @param  reg    Starting register address
 * @param  data   Pointer to data buffer to store the result
 * @param  len    Number of bytes to read
 * @retval 0 if OK, negative otherwise
 */
int LPS22HB_I2C_ReadReg(LPS22HB_I2C_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hi2c || !data)
    {
        return -1; // invalid params
    }

    // The LPS22HB expects reg address -> read
    // Use 8-bit dev address, e.g. 0xB8/0xB9 if SA0=0
    // or 0xBA/0xBB if SA0=1, but typically you store that as dev->i2cAddress
    if (HAL_I2C_Mem_Read(dev->hi2c,
                         dev->i2cAddress,
                         (uint16_t)reg,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         len,
                         100) != HAL_OK)
    {
        return -2;
    }
    return 0;
}

/**
 * @brief  Write (len) bytes starting at register 'reg'
 * @param  dev    Pointer to driver handle
 * @param  reg    Starting register address
 * @param  data   Pointer to data buffer containing bytes to write
 * @param  len    Number of bytes to write
 * @retval 0 if OK, negative otherwise
 */
int LPS22HB_I2C_WriteReg(LPS22HB_I2C_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hi2c || !data)
    {
        return -1; // invalid params
    }

    if (HAL_I2C_Mem_Write(dev->hi2c,
                          dev->i2cAddress,
                          (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)data,
                          len,
                          100) != HAL_OK)
    {
        return -2;
    }
    return 0;
}

/**
 * @brief  Verify WHO_AM_I register (should be 0xB1)
 * @param  dev  Pointer to driver handle
 * @retval 0 if success, negative otherwise
 */
int LPS22HB_I2C_WhoAmI(LPS22HB_I2C_Handle_t *dev)
{
    uint8_t who_am_i = 0;
    if (!dev)
    {
        return -1;
    }
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_WHO_AM_I, &who_am_i, 1) != 0)
    {
        return -2;
    }
    if (who_am_i != LPS22HB_WHO_AM_I_VAL)
    {
        return -3;
    }
    return 0; // OK
}

/**
 * @brief  Initialize LPS22HB with basic configuration
 * @param  dev Pointer to driver handle (dev->hi2c and dev->i2cAddress must be set!)
 * @retval 0 if success, negative on error
 */
int LPS22HB_I2C_Init(LPS22HB_I2C_Handle_t *dev)
{
    if (!dev || !dev->hi2c)
    {
        return -1;
    }

    // 1) Check WHO_AM_I
    if (LPS22HB_I2C_WhoAmI(dev) != 0)
    {
        return -2; // Not recognized
    }

    // 2) Example config: ODR=50 Hz, BDU=1, Enable LPF
    //    See datasheet for your preferred config
    //    0x4E => 01001110b
    //    ODR=50Hz (100 in bits [6:4]), BDU=1 => bit2, LPF=1 => bit3, LPF_CFG=1 => bit1
    uint8_t ctrl1 = 0x4E;
    if (LPS22HB_I2C_WriteReg(dev, LPS22HB_REG_CTRL_1, &ctrl1, 1) != 0)
    {
        return -3;
    }

    // 3) Example config for CTRL_2 => 0x10 to enable IF_ADD_INC on I2C
    //    But typically you can set any bits (FIFO_EN, etc.) you need
    //    0x10 => 00010000b => IF_ADD_INC=1
    uint8_t ctrl2 = 0x10;
    if (LPS22HB_I2C_WriteReg(dev, LPS22HB_REG_CTRL_2, &ctrl2, 1) != 0)
    {
        return -4;
    }

    // Ready to measure!
    return 0;
}

/**
 * @brief Read the 24-bit pressure, sign-extended into int32
 */
int LPS22HB_I2C_ReadPressure(LPS22HB_I2C_Handle_t *dev, int32_t *pressure)
{
    if (!dev || !pressure)
    {
        return -1;
    }
    uint8_t raw[3] = {0};
    // The device auto-increments if IF_ADD_INC=1 in CTRL_REG2
    // We can read them one by one or in a single Mem_Read of length=3
    // For clarity, let's do separate reads here:
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_PRESS_OUT_XL, &raw[0], 1) != 0)
    {
        return -2;
    }
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_PRESS_OUT_L, &raw[1], 1) != 0)
    {
        return -3;
    }
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_PRESS_OUT_H, &raw[2], 1) != 0)
    {
        return -4;
    }

    // Combine 24-bit
    int32_t val = (int32_t)((raw[2] << 16) | (raw[1] << 8) | raw[0]);
    // Sign-extend if 24th bit is set
    if (val & 0x00800000)
    {
        val |= 0xFF000000;
    }
    *pressure = val;
    return 0;
}

/**
 * @brief Read the 16-bit temperature, sign-extended into int16
 */
int LPS22HB_I2C_ReadTemp(LPS22HB_I2C_Handle_t *dev, int16_t *temp)
{
    if (!dev || !temp)
    {
        return -1;
    }
    uint8_t raw[2];
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_TEMP_OUT_L, &raw[0], 1) != 0)
    {
        return -2;
    }
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_TEMP_OUT_H, &raw[1], 1) != 0)
    {
        return -3;
    }
    *temp = (int16_t)((raw[1] << 8) | raw[0]);
    return 0;
}

/**
 * @brief Retrieve STATUS register (bits P_DA/T_DA)
 */
int LPS22HB_I2C_Status(LPS22HB_I2C_Handle_t *dev, uint8_t *status)
{
    if (!dev || !status)
    {
        return -1;
    }
    if (LPS22HB_I2C_ReadReg(dev, LPS22HB_REG_STATUS, status, 1) != 0)
    {
        return -2;
    }
    return 0; // success
}
