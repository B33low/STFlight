#pragma once
#include <stdint.h>

typedef struct {
    void    *storage;
    uint16_t size;
    uint32_t seq;
    uint32_t t_us;   // last update time
} StateAny;

void state_any_init(StateAny *s, void *storage, uint16_t size);

// set/get copy bytes
void state_any_set(StateAny *s, const void *value, uint32_t now_us);
uint32_t state_any_get(const StateAny *s, void *out, uint32_t *t_us_out);
