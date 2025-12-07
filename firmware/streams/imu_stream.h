#pragma once
#include "stream_base.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct __attribute__((packed)) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
    uint32_t t_us;
} ImuRawSample;

typedef struct {
    StreamAny base;
    ImuRawSample storage[64];
} ImuRawStream;

extern ImuRawStream   g_imu_raw;

static inline void imu_raw_stream_init(ImuRawStream *s) {
    stream_any_init(&s->base, s->storage, 64, sizeof(ImuRawSample));
}

static inline void imu_raw_stream_push(ImuRawStream *s, ImuRawSample v) {
    stream_any_push(&s->base, &v);
}

static inline bool imu_raw_stream_latest(const ImuRawStream *s, ImuRawSample *out) {
    return stream_any_latest(&s->base, out);
}
