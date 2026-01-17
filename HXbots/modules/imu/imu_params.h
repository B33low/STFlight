#pragma once

#include "param_base.h"
#include "state_base.h"
typedef struct {
    float accel_lsb_to_ms2;
    float gyro_lsb_to_rads;
    float accel_bias[3];
    float gyro_bias[3];
} ImuConvMeta;

typedef struct {
    StateAny base;
    ImuConvMeta storage;
} ParamImuConv;

extern ParamImuConv g_imu_conv;

static inline void imu_raw_param_conv_init(ParamImuConv *p) {
    state_any_init(&p->base, &p->storage, sizeof(ImuConvMeta));
}
