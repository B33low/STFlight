#pragma once
#include <stdint.h>
#include "state_base.h"

typedef struct {
    float az_ms2;   // derived vertical accel (demo)
    float vz_ms;    // integrated vertical velocity
    float pz_m;     // integrated altitude
    uint32_t t_us;  // time of last update
} AltitudeState;

typedef struct {
    StateAny base;
    AltitudeState storage;
} StateAltitude;

extern StateAltitude g_altitude;

static inline void altitude_state_init(StateAltitude *s) {
    state_any_init(&s->base, &s->storage, sizeof(AltitudeState));
}
