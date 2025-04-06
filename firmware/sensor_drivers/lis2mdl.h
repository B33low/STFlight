#ifndef LIS2MDL_H
#define LIS2MDL_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------*/
/* REGISTER DEFINITIONS (PARTIAL)                                             */
/*----------------------------------------------------------------------------*/
#define LIS2MDL_REG_WHO_AM_I 0x4F
#define LIS2MDL_WHO_AM_I_VAL 0x40

#define LIS2MDL_REG_OUTX_L 0x68
#define LIS2MDL_REG_OUTX_H 0x69
#define LIS2MDL_REG_OUTY_L 0x6A
#define LIS2MDL_REG_OUTY_H 0x6B
#define LIS2MDL_REG_OUTZ_L 0x6C
#define LIS2MDL_REG_OUTZ_H 0x6D

#define LIS2MDL_CFG_REG_A 0x60
#define LIS2MDL_CFG_REG_B 0x61
#define LIS2MDL_CFG_REG_C 0x62

    typedef struct
    {
        int dummy;
    } LIS2MDL_Config_t;

    /**
     * @brief  Driver handle for LSM6DSO32 sensor.
     *         Contains the SPI handle, chip select info, config, etc.
     */
    typedef struct
    {
        SPI_HandleTypeDef *hspi; ///< Pointer to SPI HAL handle
        GPIO_TypeDef *csPort;    ///< GPIO port for chip-select
        uint16_t csPin;          ///< GPIO pin for chip-select

        LIS2MDL_Config_t config; ///< Desired sensor configuration
    } LIS2MDL_Handle_t;

    typedef struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } LIS2MDL_Mag_Raw;

    /*----------------------------------------------------------------------------*/
    /* PUBLIC DRIVER API                                                           */
    /*----------------------------------------------------------------------------*/

    /**
     * @brief Initialize the LIS2MDL sensor using the specified handle & config
     * @param[in,out] dev  Pointer to driver handle (SPI handle & config must be set)
     * @retval  0 on success, negative on error
     */
    int LIS2MDL_Init(LIS2MDL_Handle_t *dev);

    /**
     * @brief Read a device register
     * @param[in]  dev  Pointer to driver handle
     * @param[in]  reg  Register address
     * @param[out] data Buffer to store read data
     * @param[in]  len  Number of bytes to read
     * @retval  0 on success, negative on error
     */
    int LIS2MDL_ReadReg(LIS2MDL_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

    /**
     * @brief Write a device register
     * @param[in] dev   Pointer to driver handle
     * @param[in] reg   Register address
     * @param[in] data  Buffer containing data to write
     * @param[in] len   Number of bytes to write
     * @retval  0 on success, negative on error
     */
    int LIS2MDL_WriteReg(LIS2MDL_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

    /**
     * @brief Enable the SPI 4 wire mode
     *
     * @param[in] dev Pointer to driver handler
     * @retval 0 on success, negative on error
     */
    int LIS2MDL_Enable4WireMode(LIS2MDL_Handle_t *dev);

    /**
     * @brief Read raw magnetic values (X,Y,Z)
     * @param[in]  dev   Pointer to driver handle
     * @param[out] mag Pointer to structure that will store raw mag data
     * @retval  0 on success, negative on error
     */
    int LIS2MDL_ReadMagneticRaw(LIS2MDL_Handle_t *dev, LIS2MDL_Mag_Raw *mag);

#ifdef __cplusplus
}
#endif
#endif