#include "attitude_estimator.h"
#include <math.h>
#include <string.h>

#include "imu_stream.h"
#include "imu_params.h"
#include "attitude_state.h"
#include "att_filter_params.h"

// Assumed helper (like your PC side):

static bool param_any_get_latest_copy(ParamAny *p, void *out_payload) {
    if (!p) return false;
    uint32_t t_us_out;
    return state_any_get(&p->base, out_payload,&t_us_out);
}

void att_est_init(AttEstCtx *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

bool att_est_step(AttEstCtx *ctx,
                  StreamAny *imu_raw_stream,
                  ParamAny  *imu_conv_meta,
                  ParamAny  *att_filter_params,
                  StateAny  *attitude_state)
{
    if (!ctx || !imu_raw_stream || !imu_conv_meta || !attitude_state) return false;

    ImuRawSample raw;
    ImuConvMeta meta;
    AttFilterParams fp;

    if (!stream_any_latest(imu_raw_stream, &raw)) return false;

    if (!param_any_get_latest_copy(imu_conv_meta, &meta)) {
        memset(&meta, 0, sizeof(meta));
        meta.accel_lsb_to_ms2 = 0.01f;
        // If you have gyro scale in meta, add it; else fallback below.
    }

    if (!param_any_get_latest_copy(att_filter_params, &fp)) {
        fp.alpha = 0.98f;
    }

    const float alpha = fp.alpha;

    // Convert IMU
    float ax = (float)raw.ax * meta.accel_lsb_to_ms2;
    float ay = (float)raw.ay * meta.accel_lsb_to_ms2;
    float az = (float)raw.az * meta.accel_lsb_to_ms2;

    // If you have gyro conversion in meta:
    // float gx = raw.gx * meta.gyro_lsb_to_rads;
    // else naive fallback (adjust later):
    float gx = (float)raw.gx * 0.001f;
    float gy = (float)raw.gy * 0.001f;
    float gz = (float)raw.gz * 0.001f;

    uint32_t t_us = raw.t_us;

    if (!ctx->has_prev) {
        ctx->last_t_us = t_us;
        ctx->has_prev = true;

        // Initialize from accel only
        ctx->roll  = atan2f(ay, az);
        ctx->pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
        ctx->yaw   = 0.0f;

        Attitude att = {0};
        att.roll = ctx->roll;
        att.pitch = ctx->pitch;
        att.yaw = ctx->yaw;

        state_any_set(attitude_state, &att, t_us);
        return true;
    }

    int32_t dt_us_i = (int32_t)(t_us - ctx->last_t_us);
    if (dt_us_i <= 0) return false;
    float dt = (float)dt_us_i * 1e-6f;

    ctx->last_t_us = t_us;

    // Accel-derived angles
    float roll_acc  = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

    // Gyro integration
    float roll_gyro  = ctx->roll  + gx * dt;
    float pitch_gyro = ctx->pitch + gy * dt;
    float yaw_gyro   = ctx->yaw   + gz * dt;

    // Complementary fusion
    ctx->roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    ctx->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    ctx->yaw   = yaw_gyro; // No mag correction here (yet)

    Attitude att = {0};
    att.roll = ctx->roll;
    att.pitch = ctx->pitch;
    att.yaw = ctx->yaw;

    state_any_set(attitude_state, &att, t_us);
    return true;
}
