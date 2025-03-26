#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "sensor_imu.h" // We'll use the SensorData_t struct from the sensor code

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the flight control module.
     */
    void FlightControl_Init(void);

    /**
     * @brief Updates the flight control logic with new sensor data.
     * @param sensorData Pointer to the sensor data structure.
     */
    void FlightControl_Update(const SensorData_t *sensorData);

#ifdef __cplusplus
}
#endif

#endif // FLIGHT_CONTROL_H
