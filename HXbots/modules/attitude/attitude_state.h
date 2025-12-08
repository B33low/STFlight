#pragma once
#include "state_base.h"

typedef struct {
    float roll, pitch, yaw;
} Attitude;

typedef struct {
    StateAny base;
    Attitude storage;
} StateAttitude;

extern StateAttitude  g_attitude;

static inline void state_attitude_init(StateAttitude *s) {
    state_any_init(&s->base, &s->storage, sizeof(Attitude));
}

static inline void state_attitude_set(StateAttitude *s, Attitude v, uint32_t now_us) {
    state_any_set(&s->base, &v, now_us);
}

static inline uint32_t state_attitude_get(const StateAttitude *s, Attitude *out, uint32_t *t_us_out) {
    return state_any_get(&s->base, out, t_us_out);
}
