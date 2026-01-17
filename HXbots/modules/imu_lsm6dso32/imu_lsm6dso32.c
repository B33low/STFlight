#include "imu_lsm6dso32.h"
#include <stdbool.h>

void imu_lsm6dso32_init(ImuLsm6Ctx *ctx, LSM6DSO32_Handle_t *lsm6dso32,
                        ImuRawStream *out_raw, ParamImuConv *imu_params,
                        uint32_t period_us) {
  // Populate the context
  ctx->lsm6dso32 = lsm6dso32;
  ctx->out_raw_stream = out_raw;
  ctx->imu_params = imu_params;
  ctx->period_us = period_us;
  ctx->next_deadline_us = 0;
}

void imu_lsm6dso32_update(ImuLsm6Ctx *ctx){
  if(!ctx) return;
  uint32_t now_us = HAL_GetTick() * 1000u;

  LSM6DSO32_AccelRaw_t acceleration;
  LSM6DSO32_GyroRaw_t gyro_raw;

  int lastResult = LSM6DSO32_ReadAccelGyroRaw(ctx->lsm6dso32, &acceleration, &gyro_raw);
  if (lastResult == 0) {
    int16_t ax_raw = acceleration.x;
    int16_t ay_raw = acceleration.y;
    int16_t az_raw = acceleration.z;

    int16_t gx_raw = gyro_raw.x;
    int16_t gy_raw = gyro_raw.y;
    int16_t gz_raw = gyro_raw.z;

    // Push raw IMU sample into the stream
    ImuRawSample s;
    s.ax = ax_raw;
    s.ay = ay_raw;
    s.az = az_raw;
    s.gx = gx_raw;
    s.gy = gy_raw;
    s.gz = gz_raw;
    s.t_us = now_us;
    stream_any_push(&ctx->out_raw_stream->base, &s);
  }
  
}