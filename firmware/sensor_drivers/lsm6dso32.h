#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h" // or stm32xxxx_hal.h matching your MCU

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------*/
/* REGISTER DEFINITIONS (PARTIAL)                                             */
/*----------------------------------------------------------------------------*/
#define LSM6DSO32_REG_WHO_AM_I 0x0F
#define LSM6DSO32_REG_CTRL1_XL 0x10
#define LSM6DSO32_REG_CTRL2_G 0x11
#define LSM6DSO32_REG_OUTX_L_A 0x28 // first accel data register

#define LSM6DSO32_WHO_AM_I_VAL 0x6C // expected WHO_AM_I value for LSM6DSO32

    /*----------------------------------------------------------------------------*/
    /* DATA STRUCTURES                                                            */
    /*----------------------------------------------------------------------------*/

    /**
     * @brief A structure to hold raw accelerometer data from LSM6DSO32
     */
    typedef struct
    {
        int16_t x; // raw X-axis
        int16_t y; // raw Y-axis
        int16_t z; // raw Z-axis
    } LSM6DSO32_AccelRaw_t;

    /**
     * @brief LSM6DSO32 configuration parameters.
     *        Expand with ODR, range, etc. as needed.
     */
    typedef struct
    {
        uint8_t accelOdr;   ///< Accelerometer Output Data Rate setting
        uint8_t accelRange; ///< Accelerometer full-scale range setting
        uint8_t gyroOdr;    ///< Gyroscope ODR setting
        uint8_t gyroRange;  ///< Gyroscope full-scale range setting
    } LSM6DSO32_Config_t;

    /**
     * @brief  Driver handle for LSM6DSO32 sensor.
     *         Contains the SPI handle, chip select info, config, etc.
     */
    typedef struct
    {
        SPI_HandleTypeDef *hspi; ///< Pointer to SPI HAL handle
        GPIO_TypeDef *csPort;    ///< GPIO port for chip-select
        uint16_t csPin;          ///< GPIO pin for chip-select

        LSM6DSO32_Config_t config; ///< Desired sensor configuration
    } LSM6DSO32_Handle_t;

    /*----------------------------------------------------------------------------*/
    /* PUBLIC DRIVER API                                                           */
    /*----------------------------------------------------------------------------*/

    /**
     * @brief Initialize the LSM6DSO32 sensor using the specified handle & config
     * @param[in,out] dev  Pointer to driver handle (SPI handle & config must be set)
     * @retval  0 on success, negative on error
     */
    int LSM6DSO32_Init(LSM6DSO32_Handle_t *dev);

    /**
     * @brief Read raw accelerometer values (X,Y,Z)
     * @param[in]  dev   Pointer to driver handle
     * @param[out] accel Pointer to structure that will store raw accel data
     * @retval  0 on success, negative on error
     */
    int LSM6DSO32_ReadAccelRaw(LSM6DSO32_Handle_t *dev, LSM6DSO32_AccelRaw_t *accel);

    /**
     * @brief Read a device register
     * @param[in]  dev  Pointer to driver handle
     * @param[in]  reg  Register address
     * @param[out] data Buffer to store read data
     * @param[in]  len  Number of bytes to read
     * @retval  0 on success, negative on error
     */
    int LSM6DSO32_ReadReg(LSM6DSO32_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

    /**
     * @brief Write a device register
     * @param[in] dev   Pointer to driver handle
     * @param[in] reg   Register address
     * @param[in] data  Buffer containing data to write
     * @param[in] len   Number of bytes to write
     * @retval  0 on success, negative on error
     */
    int LSM6DSO32_WriteReg(LSM6DSO32_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSO32_H */
