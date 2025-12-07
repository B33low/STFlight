#pragma once
#include "state_base.h"
#include <stdbool.h>

typedef bool (*ParamValidateFn)(const void *value, uint16_t size);
typedef void (*ParamApplyFn)(const void *value, uint16_t size);

typedef struct {
    StateAny base;
    ParamValidateFn validate;
    ParamApplyFn apply; // e.g. reconfigure sensor
} ParamAny;

void param_any_init(ParamAny *p, void *storage, uint16_t size,
                    ParamValidateFn validate, ParamApplyFn apply);

// returns true if accepted
bool param_any_set(ParamAny *p, const void *value, uint32_t now_us);
