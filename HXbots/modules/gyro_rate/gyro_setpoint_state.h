#pragma once
#include <stdint.h>
#include "state_base.h"

typedef struct
{
    int16_t gx, gy, gz;
} GyroSetpointState;

typedef struct
{
    StateAny base;
    GyroSetpointState storage;
} StateGyroSetpoint;

extern StateGyroSetpoint g_gyro_setpoint;

void state_gyro_setpoint_init(StateGyroSetpoint *s);

void state_gyro_setpoint_set(StateGyroSetpoint *s, GyroSetpointState v, uint32_t now_us);

uint32_t state_gyro_setpoint_get(const StateGyroSetpoint *s, GyroSetpointState *out, uint32_t *t_us_out);