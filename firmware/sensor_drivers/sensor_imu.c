#include "sensor_imu.h"
#include <stdlib.h> // for rand() if you want random data
#include <time.h>   // if you want to seed random

static int s_isInitialized = 0;

int SensorIMU_Init(void)
{
    // Dummy initialization, e.g. seed a random generator
    s_isInitialized = 1;
    srand(12345);

    return 0; // success
}

int SensorIMU_ReadData(SensorData_t *outData)
{
    if (!s_isInitialized || !outData)
        return -1; // error if not initialized or invalid pointer

    // Provide some dummy values
    outData->dummy1 = (float)(rand() % 100) / 10.0f; // e.g. random 0..9.9
    outData->dummy2 = (float)(rand() % 100) / 10.0f;

    // In a real system, you'd read actual SPI/I2C registers
    // from the IMU, parse them, etc.

    return 0;
}
