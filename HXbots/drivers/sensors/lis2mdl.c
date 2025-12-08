#include "lis2mdl.h"

/*----------------------------------------------------------------------------*/
/* INTERNAL UTILITY FUNCTIONS                                                 */
/*----------------------------------------------------------------------------*/

/**
 * @brief Helper to set or clear the chip-select line
 */
static inline void LIS2MDL_Select(LIS2MDL_Handle_t *dev, bool select)
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

int LIS2MDL_ReadReg(LIS2MDL_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hspi || !data)
    {
        return -1; // invalid parameters
    }

    // LIS2MDL SPI read protocol: set MSB = 1 for read
    uint8_t addr = reg | 1 << 7;

    LIS2MDL_Select(dev, true);

    // Transmit register address
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, 100) != HAL_OK)
    {
        LIS2MDL_Select(dev, false);
        return -2;
    }

    // Receive data
    if (HAL_SPI_Receive(dev->hspi, data, len, 100) != HAL_OK)
    {
        LIS2MDL_Select(dev, false);
        return -3;
    }

    LIS2MDL_Select(dev, false);
    return 0;
}

int LIS2MDL_WriteReg(LIS2MDL_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hspi || !data)
    {
        return -1;
    }

    // LSM6DSO32 SPI write protocol: MSB = 0
    uint8_t addr = reg;

    LIS2MDL_Select(dev, true);

    // Transmit register address
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, 100) != HAL_OK)
    {
        LIS2MDL_Select(dev, false);
        return -2;
    }

    // Transmit data
    if (HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, 100) != HAL_OK)
    {
        LIS2MDL_Select(dev, false);
        return -3;
    }

    LIS2MDL_Select(dev, false);
    return 0;
}

int LIS2MDL_Init(LIS2MDL_Handle_t *dev)
{
    if (!dev || !dev->hspi)
    {
        return -1;
    }

    // LIS2MDL_Enable4WireMode(dev);

    uint8_t reg_val = 1 << 7 | 1 << 3 | 1 << 2; // Set Temp Com, 50Hz Measure amd continuous mode
    if (LIS2MDL_WriteReg(dev, LIS2MDL_CFG_REG_A, &reg_val, 1) != 0)
    {
        return -2;
    }

    reg_val = 1 << 0; // LPF (BW = 25Hz)
    if (LIS2MDL_WriteReg(dev, LIS2MDL_CFG_REG_B, &reg_val, 1) != 0)
    {
        return -3;
    }

    reg_val = 1 << 4 | 1 << 2; // Set BDU, 4WSPI
    if (LIS2MDL_WriteReg(dev, LIS2MDL_CFG_REG_C, &reg_val, 1) != 0)
    {
        return -3;
    }

    // 1) Check WHO_AM_I
    uint8_t whoAmI = 0;
    if (LIS2MDL_ReadReg(dev, LIS2MDL_REG_WHO_AM_I, &whoAmI, 1) != 0)
    {
        return -2;
    }
    if (whoAmI != LIS2MDL_WHO_AM_I_VAL)
    {
        return -3; // not the correct device
    }

    // 2) Configure accelerometer in CTRL1_XL
    //    For example, 0x40 -> ODR=1.66kHz, ±4g
    //    But let's parse from dev->config
    //    We'll do a simplistic approach here:

    // // 3) Configure gyroscope in CTRL2_G if needed
    // uint8_t ctrl2_g = 0;
    // ctrl2_g = 0x4c; // e.g., ODR=1.66kHz, ±2000 dps
    // if (LIS2MDL_WriteReg(dev, LSM6DSO32_REG_CTRL2_G, &ctrl2_g, 1) != 0)
    // {
    //     return -5;
    // }

    // Additional registers (CTRL3_C, CTRL9_XL, etc.) for enabling features

    return 0; // success
}

int LIS2MDL_Enable4WireMode(LIS2MDL_Handle_t *dev)
{
    uint8_t reg_val = 0;
    if (LIS2MDL_ReadReg(dev, LIS2MDL_CFG_REG_C, &reg_val, 1) != 0)
    {
        return -1;
    }
    reg_val |= 0x04; // set bit2 to 1 to enable 4-wire mode
    if (LIS2MDL_WriteReg(dev, LIS2MDL_CFG_REG_C, &reg_val, 1) != 0)
    {
        return -2;
    }
    return 0;
}

int LIS2MDL_ReadMagneticRaw(LIS2MDL_Handle_t *dev, LIS2MDL_Mag_Raw *mag)
{
    if (!dev || !mag)
    {
        return -1;
    }

    uint8_t rawData[6] = {0};
    uint8_t addrToRead[6] = {LIS2MDL_REG_OUTX_L,
                             LIS2MDL_REG_OUTX_H,
                             LIS2MDL_REG_OUTY_L,
                             LIS2MDL_REG_OUTY_H,
                             LIS2MDL_REG_OUTZ_L,
                             LIS2MDL_REG_OUTZ_H};
    for (uint8_t i = 0; i < 6; i++)
    {
        uint8_t received;
        if (LIS2MDL_ReadReg(dev, addrToRead[i], &received, 1) != 0)
        {
            return -2;
        }
        rawData[i] = received;
    }

    // combine LSB/MSB for each axis
    mag->x = (int16_t)((rawData[1] << 8) | rawData[0]);
    mag->y = (int16_t)((rawData[3] << 8) | rawData[2]);
    mag->z = (int16_t)((rawData[5] << 8) | rawData[4]);

    return 0;
}