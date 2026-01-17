#include "imu_params.h"
#include "imu_stream.h"
#include "lsm6dso32.h"

typedef struct {
  LSM6DSO32_Handle_t *lsm6dso32;

  ImuRawStream *out_raw_stream;     // Stream<ImuRawSample>

  // optionnel: params, scale, calib, offsets, stats, etc.
  ParamImuConv *imu_params;
  
  uint32_t period_us;
  uint32_t next_deadline_us;
} ImuLsm6Ctx;


void imu_lsm6dso32_init(ImuLsm6Ctx *ctx,
                        LSM6DSO32_Handle_t *lsm6dso32,
                        ImuRawStream *out_raw,
                        ParamImuConv *imu_params,
                        uint32_t period_us);

void imu_lsm6dso32_update(ImuLsm6Ctx *ctx);