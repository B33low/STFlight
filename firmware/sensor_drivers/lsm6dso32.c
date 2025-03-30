#include "lsm6dso32.h"

/*----------------------------------------------------------------------------*/
/* INTERNAL UTILITY FUNCTIONS                                                 */
/*----------------------------------------------------------------------------*/

/**
 * @brief Helper to set or clear the chip-select line
 */
static inline void LSM6DSO32_Select(LSM6DSO32_Handle_t *dev, bool select)
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

int LSM6DSO32_ReadReg(LSM6DSO32_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hspi || !data)
    {
        return -1; // invalid parameters
    }

    // LSM6DSO32 SPI read protocol: set MSB = 1 for read
    uint8_t addr = reg | 0x80;

    LSM6DSO32_Select(dev, true);

    // Transmit register address
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, 100) != HAL_OK)
    {
        LSM6DSO32_Select(dev, false);
        return -2;
    }

    // Receive data
    if (HAL_SPI_Receive(dev->hspi, data, len, 100) != HAL_OK)
    {
        LSM6DSO32_Select(dev, false);
        return -3;
    }

    LSM6DSO32_Select(dev, false);
    return 0;
}

int LSM6DSO32_WriteReg(LSM6DSO32_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (!dev || !dev->hspi || !data)
    {
        return -1;
    }

    // LSM6DSO32 SPI write protocol: MSB = 0
    uint8_t addr = reg & 0x7F;

    LSM6DSO32_Select(dev, true);

    // Transmit register address
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, 100) != HAL_OK)
    {
        LSM6DSO32_Select(dev, false);
        return -2;
    }

    // Transmit data
    if (HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, 100) != HAL_OK)
    {
        LSM6DSO32_Select(dev, false);
        return -3;
    }

    LSM6DSO32_Select(dev, false);
    return 0;
}

int LSM6DSO32_Init(LSM6DSO32_Handle_t *dev)
{
    if (!dev || !dev->hspi)
    {
        return -1;
    }

    // 1) Check WHO_AM_I
    uint8_t whoAmI = 0;
    if (LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_WHO_AM_I, &whoAmI, 1) != 0)
    {
        return -2;
    }
    if (whoAmI != LSM6DSO32_WHO_AM_I_VAL)
    {
        return -3; // not the correct device
    }

    // 2) Configure accelerometer in CTRL1_XL
    //    For example, 0x40 -> ODR=1.66kHz, ±4g
    //    But let's parse from dev->config
    //    We'll do a simplistic approach here:
    uint8_t ctrl1_xl = 0;
    // bits[7:4] = ODR, bits[3:2] = FS (range), bits[1:0]=BW
    // This is just an example, you can map dev->config.accelOdr, etc.
    ctrl1_xl = 0x48; // 1.66 Khz && 8g accekleration

    if (LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL1_XL, &ctrl1_xl, 1) != 0)
    {
        return -4;
    }

    // 3) Configure gyroscope in CTRL2_G if needed
    uint8_t ctrl2_g = 0;
    ctrl2_g = 0x4c; // e.g., ODR=1.66kHz, ±2000 dps
    if (LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL2_G, &ctrl2_g, 1) != 0)
    {
        return -5;
    }

    // Additional registers (CTRL3_C, CTRL9_XL, etc.) for enabling features

    return 0; // success
}

int LSM6DSO32_ReadAccelRaw(LSM6DSO32_Handle_t *dev, LSM6DSO32_AccelRaw_t *accel)
{
    if (!dev || !accel)
    {
        return -1;
    }

    uint8_t rawData[6] = {0};

    // read 6 bytes from OUTX_L_A
    if (LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_OUTX_L_A, rawData, 6) != 0)
    {
        return -2;
    }

    // combine LSB/MSB for each axis
    accel->x = (int16_t)((rawData[1] << 8) | rawData[0]);
    accel->y = (int16_t)((rawData[3] << 8) | rawData[2]);
    accel->z = (int16_t)((rawData[5] << 8) | rawData[4]);

    return 0;
}

int LSM6DS032_WhoIAm(LSM6DSO32_Handle_t *dev)
{
    if (!dev)
    {
        return -1;
    }

    uint8_t whoAmI = 0;
    if (LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_WHO_AM_I, &whoAmI, 1))
    {
        return -2;
    }
    if (whoAmI != LSM6DSO32_WHO_AM_I_VAL)
    {
        return -3;
    }
    return 0;
}