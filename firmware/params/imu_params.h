#pragma once

#include "param_base.h"
typedef struct {
    float accel_lsb_to_ms2;
    float gyro_lsb_to_rads;
    float accel_bias[3];
    float gyro_bias[3];
} ImuConvMeta;

typedef struct {
    ParamAny base;
    ImuConvMeta storage;
} ParamImuConv;

extern ParamImuConv g_imu_conv;

static inline void param_imu_conv_init(ParamImuConv *p,
                                       ParamValidateFn v, ParamApplyFn a) {
    param_any_init(&p->base, &p->storage, sizeof(ImuConvMeta), v, a);
}
