#ifndef LPS22HB_H
#define LPS22HB_H

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
#define LPS22HB_REG_WHO_AM_I 0x0F
#define LPS22HB_WHO_AM_I_VAL 0xB1 // expected WHO_AM_I value for LPS22HB

#define LPS22HB_REG_PRESS_OUT_XL 0x28
#define LPS22HB_REG_PRESS_OUT_L 0x29
#define LPS22HB_REG_PRESS_OUT_H 0x2A

#define LPS22HB_REG_TEMP_OUT_L 0x2B
#define LPS22HB_REG_TEMP_OUT_H 0x2C

#define LPS22HB_REG_CTRL_1 0x10
#define LPS22HB_REG_CTRL_2 0x11
#define LPS22HB_REG_CTRL_3 0x12
#define LPS22HB_READ_INSTRUCTION 0x40

#define LPS22HB_REG_STATUS 0x27
#define LPS22HB_STATUS_PRESS_READY 0x01
#define LPS22HB_STATUS_TEMP_READY 0x02

    /*------------------------#ifdev __cplusplusrange, etc. as needed.
     */
    typedef struct
    {
        uint8_t c;
    } LPS22HB_Config_t;

    /**
     * @brief  Driver handle for LPS22HB sensor.
     *         Contains the SPI handle, chip select info, config, etc.
     */
    typedef struct
    {
        SPI_HandleTypeDef *hspi; ///< Pointer to SPI HAL handle
        GPIO_TypeDef *csPort;    ///< GPIO port for chip-select
        uint16_t csPin;          ///< GPIO pin for chip-select

        LPS22HB_Config_t config; ///< Desired sensor configuration
    } LPS22HB_Handle_t;

    /*----------------------------------------------------------------------------*/
    /* PUBLIC DRIVER API                                                           */
    /*----------------------------------------------------------------------------*/

    /**
     * @brief Initialize the LSM6DSO32 sensor using the specified handle & config
     * @param[in,out] dev  Pointer to driver handle (SPI handle & config must be set)
     * @retval  0 on success, negative on error
     */
    int LPS22HB_Init(LPS22HB_Handle_t *dev);

    /**
     * @brief Read pressure value
     * @param[in]  dev   Pointer to driver handle
     * @param[out] pressure Pointer to int32 data (real value is 24 bit)
     * @retval  0 on success, negative on error
     */
    int LPS22HB_ReadPressure(LPS22HB_Handle_t *dev, int32_t *pressure);

    /**
     * @brief Read temperature value
     * @param[in]  dev   Pointer to driver handle
     * @param[out] temp Pointer to int16 data
     * @retval  0 on success, negative on error
     */
    int LPS22HB_ReadTemp(LPS22HB_Handle_t *dev, int16_t *temp);

    /**
     * @brief Read a device register
     * @param[in]  dev  Pointer to driver handle
     * @param[in]  reg  Register address
     * @param[out] data Buffer to store read data
     * @param[in]  len  Number of bytes to read
     * @retval  0 on success, negative on error
     */
    int LPS22HB_ReadReg(LPS22HB_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

    /**
     * @brief Write a device register
     * @param[in] dev   Pointer to driver handle
     * @param[in] reg   Register address
     * @param[in] data  Buffer containing data to write
     * @param[in] len   Number of bytes to write
     * @retval  0 on success, negative on error
     */
    int LPS22HB_WriteReg(LPS22HB_Handle_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

    /**
     * @brief Verify the WHO_I_AM Register
     *
     * @param[in] dev Pointer to driver handle
     * @retval 0 on success, negative on error
     */
    int LPS22HB_WhoIAm(LPS22HB_Handle_t *dev);

    /**
     * @brief Retrieve Status Register
     *
     * @param[in] dev Pointer to driver handle
     * @retval 0 on success, negative on error
     */
    int LPS22HB_Status(LPS22HB_Handle_t *dev, uint8_t *status);

    /**
     * @brief Read pressure value in hPa
     * @param[in]  dev   Pointer to driver handle
     * @param[out] pressure Pointer to int32 data (real value is 24 bit)
     * @retval  0 on success, negative on error
     */
    int LPS22HB_ReadPressure_hPa(LPS22HB_Handle_t *dev, float *pressure);

    /**
     * @brief Read temperature value in Celcius
     * @param[in]  dev   Pointer to driver handle
     * @param[out] temp Pointer to int16 data
     * @retval  0 on success, negative on error
     */
    int LPS22HB_ReadTemp_C(LPS22HB_Handle_t *dev, float *temp);

#ifdef __cplusplus
}
#endif

#endif
