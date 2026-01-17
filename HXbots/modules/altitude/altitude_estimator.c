#include <string.h>
#include "altitude_estimator.h"
#include "imu_stream.h"
#include "imu_params.h"
#include "altitude_state.h"
#include "stream_base.h"


extern bool state_any_get_latest(StateAny *s, void *out_payload);

static bool param_any_get_latest_copy(ParamImuConv *p, void *out_payload) {
    if (!p) return false;
    uint32_t t_us_out;
    return state_any_get(&p->base, out_payload,&t_us_out);
}

/* ----------------------------------------------------------- */

void alt_est_init(AltEstCtx *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

bool alt_est_step(AltEstCtx *ctx,
                  ImuRawStream *imu_stream,
                  ParamImuConv  *imu_meta,
                  StateAny  *alt_state)
{
    if (!ctx || !imu_stream || !imu_meta || !alt_state) return false;

    ImuRawSample raw;
    ImuConvMeta  meta;

    if (!stream_any_latest(&imu_stream->base, &raw)) {
        return false; // no IMU yet
    }

    if (!param_any_get_latest_copy(imu_meta, &meta)) {
        memset(&meta, 0, sizeof(meta));
        meta.accel_lsb_to_ms2 = 0.01f; // safe fallback
    }

    uint32_t t_us = raw.t_us;

    if (!ctx->has_prev) {
        ctx->last_t_us = t_us;
        ctx->has_prev = true;

        AltitudeState a0 = {0};
        a0.az_ms2 = raw.az * meta.accel_lsb_to_ms2;
        a0.vz_ms = 0.0f;
        a0.pz_m   = 0.0f;
        a0.t_us   = t_us;

        state_any_set(alt_state, &a0, t_us);
        return true;
    }

    int32_t dt_us_i = (int32_t)(t_us - ctx->last_t_us);
    if (dt_us_i <= 0) return false;

    float dt = (float)dt_us_i * 1e-6f;

    // Basic vertical accel conversion
    float az_ms2 = (float)raw.az * meta.accel_lsb_to_ms2;

    // Optional naive gravity removal (only if your az axis is aligned with gravity)
    // az_ms2 -= 9.80665f;

    // Integrate
    ctx->vz += az_ms2 * dt;
    ctx->pz += ctx->vz * dt;

    ctx->last_t_us = t_us;

    AltitudeState est;
    est.az_ms2 = az_ms2;
    est.vz_ms = ctx->vz;
    est.pz_m   = ctx->pz;
    est.t_us   = t_us;

    state_any_set(alt_state, &est, t_us);

    return true;
}
