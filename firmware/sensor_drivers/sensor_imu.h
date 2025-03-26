#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Simple struct that represents data from a mock sensor.
     *        In a real IMU, you'd have accel, gyro, etc.
     */
    typedef struct
    {
        float dummy1;
        float dummy2;
        // you might have float pitch, float roll, etc. in a real IMU
    } SensorData_t;

    /**
     * @brief Initializes the (dummy) sensor.
     * @return 0 if success, negative if error.
     */
    int SensorIMU_Init(void);

    /**
     * @brief Reads data from the (dummy) sensor.
     * @param outData Pointer to a SensorData_t struct to fill with sensor values.
     * @return 0 if success, negative if error.
     */
    int SensorIMU_ReadData(SensorData_t *outData);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_IMU_H
