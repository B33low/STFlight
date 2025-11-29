#include "fc_hw_stm32.h"
#include "sensor_imu.h"   // dans firmware/sensor_drivers, par ex.
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htimX; // pour PWM, etc.

static float time_s_from_timer(void) {
    uint32_t us = 0;  /* lire un timer hardware */
    return us * 1e-6f;
}

void fc_hw_init(void) {
    // init IMU, timers, PWM, etc.
    //sensor_imu_init();
    // ...
}

void fc_hw_fill_input(FcInput* in) {
    //ImuRaw imu;
    //sensor_imu_read(&imu); // ton driver maison

    // in->ax = imu.ax;
    // in->ay = imu.ay;
    // in->az = imu.az;
    // in->gx = imu.gx;
    // in->gy = imu.gy;
    // in->gz = imu.gz;
    // in->time_s = time_s_from_timer();
}

void fc_hw_apply_output(const FcOutput* out) {
    // map 0..1 -> PWM et envoyer sur tes ESC
    // ou pour l'instant, juste l’envoyer en UART pour debug
}
