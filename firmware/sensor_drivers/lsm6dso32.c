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
    if (!dev || !dev->hspi) {
        return -1;
    }

    HAL_Delay(10); // power-up time

    // 1) WHO_AM_I
    uint8_t who = 0;
    if (LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_WHO_AM_I, &who, 1) != 0) {
        return -2;
    }
    if (who != LSM6DSO32_WHO_AM_I_VAL) {
        return -3;
    }

    // 2) Reset + reboot (optional but nice to be sure)
    uint8_t ctrl3 = 0x01;             // SW_RESET
    LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL3_C, &ctrl3, 1);
    HAL_Delay(20);
    ctrl3 = 0x00;
    LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL3_C, &ctrl3, 1);

    // 3) Enable auto-increment + BDU
    // CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2) -> 0b01000100 = 0x44
    ctrl3 = 0x44;
    if (LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL3_C, &ctrl3, 1) != 0) {
        return -4;
    }

    // 4) Accelerometer @ 104 Hz, ±8 g
    // CTRL1_XL: ODR_XL[3:0]=0100 (104 Hz), FS_XL[1:0]=10 (±8g), BW_XL[1:0]=00
    // -> 0b0100 1000 = 0x48
    uint8_t ctrl1_xl = 0x48;
    if (LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL1_XL, &ctrl1_xl, 1) != 0) {
        return -5;
    }

    // 5) Gyro @ 104 Hz, ±2000 dps
    // CTRL2_G: ODR_G[3:0]=0100 (104 Hz), FS_G[1:0]=11 (±2000 dps)
    // -> 0b0100 1100 = 0x4C
    uint8_t ctrl2_g = 0x4C;
    if (LSM6DSO32_WriteReg(dev, LSM6DSO32_REG_CTRL2_G, &ctrl2_g, 1) != 0) {
        return -6;
    }

    // 6) Read back to be *sure* the writes really landed
    uint8_t r1=0, r2=0, r3=0;
    LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_CTRL1_XL, &r1, 1);
    LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_CTRL2_G, &r2, 1);
    LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_CTRL3_C, &r3, 1);

    // If these are still 0x00, something is wrong at hardware/SPI level
    if (r1 != ctrl1_xl || r2 != ctrl2_g || r3 != ctrl3) {
        return -7;
    }

    return 0;
}


int LSM6DSO32_ReadAccelRaw(LSM6DSO32_Handle_t *dev,
                           LSM6DSO32_AccelRaw_t *accel)
{
    if (!dev || !accel) {
        return -1;
    }

    uint8_t rawData[6] = {0};

    // thanks to IF_INC=1, this reads X_L, X_H, Y_L, Y_H, Z_L, Z_H in one shot
    if (LSM6DSO32_ReadReg(dev, LSM6DSO32_REG_OUTX_L_A, rawData, 6) != 0) {
        return -2;
    }

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