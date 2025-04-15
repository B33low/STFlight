#include "lps22hb.h"

/*----------------------------------------------------------------------------*/
/* INTERNAL UTILITY FUNCTIONS                                                 */
/*----------------------------------------------------------------------------*/

/**
 * @brief Helper to set or clear the chip-select line
 */
static inline void LPS22HB_Select(LPS22HB_Handle_t *dev, bool select)
{
    if (select)
    {
        // Active low typically
        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
    }
}

/*----------------------------------------------------------------------------*/
/* PUBLIC API IMPLEMENTATION                                                  */
/*----------------------------------------------------------------------------*/

int LPS22HB_ReadReg(LPS22HB_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hspi || !data)
    {
        return -1; // invalid parameters
    }

    // LPS22HB SPI read protocol: set cMSB = 1 for read
    uint8_t addr = reg | 0x80;

    LPS22HB_Select(dev, true);

    // Transmit register address
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, 100) != HAL_OK)
    {
        LPS22HB_Select(dev, false);
        return -2;
    }

    // Receive data
    if (HAL_SPI_Receive(dev->hspi, data, len, 100) != HAL_OK)
    {
        LPS22HB_Select(dev, false);
        return -3;
    }

    LPS22HB_Select(dev, false);
    return 0;
}

int LPS22HB_WriteReg(LPS22HB_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hspi || !data)
    {
        return -1;
    }

    // LPS22HB SPI write protocol: MSB = 0
    uint8_t addr = reg & 0x7F;

    LPS22HB_Select(dev, true);

    // Transmit register address
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, 100) != HAL_OK)
    {
        LPS22HB_Select(dev, false);
        return -2;
    }

    // Transmit data
    if (HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, 100) != HAL_OK)
    {
        LPS22HB_Select(dev, false);
        return -3;
    }

    LPS22HB_Select(dev, false);
    return 0;
}

int LPS22HB_Init(LPS22HB_Handle_t *dev)
{
    if (!dev || !dev->hspi)
    {
        return -1;
    }

    // 1) Check WHO_AM_I
    uint8_t whoAmI = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_WHO_AM_I, &whoAmI, 1) != 0)
    {
        return -2;
    }
    if (whoAmI != LPS22HB_WHO_AM_I_VAL)
    {
        return -3; // not the correct device
    }

    uint8_t ctrl1_xl = 0;
    ctrl1_xl = 0x4e; // 5a//(1 << 6) | (1 << 4) | (0 << 3) | (0 << 1); // 50 Hz & EN_LPFP & BW=ODR/9 & BDU & 4WSPI

    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_1, &ctrl1_xl, 1) != 0)
    {
        return -4;
    }

    uint8_t ctrl2_g = 0x0C;
    // ctrl2_g = 1 << 3; // remove I2C and auto addr inc
    if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_2, &ctrl2_g, 1) != 0)
    {
        return -5;
    }

    // Interrupt linked reguister
    // uint8_t ctrl3 = 0;
    // ctrl3 = 1 << 3; // remove I2C and auto addr inc
    // if (LPS22HB_WriteReg(dev, LPS22HB_REG_CTRL_3, &ctrl3, 1) != 0)
    // {
    //     return -5;
    // }

    // Additional registers (CTRL3_C, CTRL9_XL, etc.) for enabling features

    return 0; // success
}

int LPS22HB_ReadPressure(LPS22HB_Handle_t *dev, int32_t *pressure)
{
    if (!dev || !pressure)
    {
        return -1;
    }

    uint8_t rawData[3] = {0};
    uint8_t addrToRead[3] = {
        LPS22HB_REG_PRESS_OUT_XL,
        LPS22HB_REG_PRESS_OUT_L,
        LPS22HB_REG_PRESS_OUT_H,
    };
    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t received;
        if (LPS22HB_ReadReg(dev, addrToRead[i], &received, 1) != 0)
        {
            return -2;
        }
        rawData[i] = received;
    }

    // combine LSB/MSB for each axis
    *pressure = (int32_t)((rawData[2] << 16) | (rawData[1] << 8) | rawData[0]);
    if (*pressure & 0x00800000) // Sign extending
    {
        *pressure |= 0xFF000000;
    }
    return 0;
}

int LPS22HB_ReadPressure_hPa(LPS22HB_Handle_t *dev, float *pressure)
{
    int32_t raw_pressure;
    LPS22HB_ReadPressure(dev, &raw_pressure);
    *pressure = raw_pressure / 4096.0f;
    return 0;
}

int LPS22HB_ReadTemp_C(LPS22HB_Handle_t *dev, float *temp)
{
    int16_t raw_temp;
    LPS22HB_ReadTemp(dev, &raw_temp);
    *temp = raw_temp / 100.0f;
    return 0;
}

int LPS22HB_ReadTemp(LPS22HB_Handle_t *dev, int16_t *temp)
{
    if (!dev || !temp)
    {
        return -1;
    }

    uint8_t rawData[2] = {0};
    uint8_t addrToRead[2] = {
        LPS22HB_REG_TEMP_OUT_L,
        LPS22HB_REG_TEMP_OUT_H,
    };
    for (uint8_t i = 0; i < 2; i++)
    {
        uint8_t received;
        if (LPS22HB_ReadReg(dev, addrToRead[i], &received, 1) != 0)
        {
            return -2;
        }
        rawData[i] = received;
    }

    // combine LSB/MSB for each axis
    *temp = (int16_t)((rawData[1] << 8) | rawData[0]);

    return 0;
}

int LPS22HB_WhoIAm(LPS22HB_Handle_t *dev)
{
    if (!dev)
    {
        return -1;
    }

    uint8_t whoAmI = 0;
    if (LPS22HB_ReadReg(dev, LPS22HB_REG_WHO_AM_I, &whoAmI, 1))
    {
        return -2;
    }
    if (whoAmI != LPS22HB_WHO_AM_I_VAL)
    {
        return -3;
    }
    return 0;
}

int LPS22HB_Status(LPS22HB_Handle_t *dev, uint8_t *status)
{
    if (!dev)
    {
        return -1;
    }

    if (LPS22HB_ReadReg(dev, LPS22HB_REG_STATUS, status, 1) != 0)
    {
        return -2;
    }
    *status &= 0x03;
    return 0;
}