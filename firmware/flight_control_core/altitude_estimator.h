#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "stream_base.h"
#include "param_base.h"
#include "state_base.h"


#include "stream_base.h"
#include "param_base.h"
#include "state_base.h"


typedef struct {
    float vz;
    float pz;
    uint32_t last_t_us;
    bool has_prev;
} AltEstCtx;

void alt_est_init(AltEstCtx *ctx);

/**
 * Reads latest IMU raw + meta, integrates, writes altitude state.
 * Returns true if it updated the state.
 */
bool alt_est_step(AltEstCtx *ctx,
                  StreamAny *imu_stream,
                  ParamAny  *imu_meta,
                  StateAny  *alt_state);
