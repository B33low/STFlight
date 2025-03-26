#include "flight_control.h"
#include <stdio.h> // just for printf examples (if you want logging)

/**
 * @brief Simple example: track a dummy "altitude" and "heading".
 */
static float s_altitude = 0.0f;
static float s_heading = 0.0f;

void FlightControl_Init(void)
{
    // Example: Initialize variables, etc.
    s_altitude = 0.0f;
    s_heading = 0.0f;

    // In a real system, you might set up PID controllers, read config, etc.
}

void FlightControl_Update(const SensorData_t *sensorData)
{
    if (!sensorData)
        return;

    // Dummy logic: just update altitude/heading from sensor data
    // This does nothing meaningful – purely illustrative
    s_altitude = sensorData->dummy1 * 1.5f;
    s_heading = sensorData->dummy2 * 2.0f;

    // Optional: do something with s_altitude and s_heading
    // e.g., print them (if you have a UART printf or semihosting)
    // printf("Altitude: %.2f, Heading: %.2f\r\n", s_altitude, s_heading);
}
