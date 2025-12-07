#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "stream_base.h"
#include "param_base.h"
#include "state_base.h"


typedef struct {
    float roll, pitch, yaw;
    uint32_t last_t_us;
    bool has_prev;
} AttEstCtx;

void att_est_init(AttEstCtx *ctx);

bool att_est_step(AttEstCtx *ctx,
                  StreamAny *imu_raw_stream,
                  ParamAny  *imu_conv_meta,
                  ParamAny  *att_filter_params,
                  StateAny  *attitude_state);
