#include "state_base.h"

static void mem_copy(uint8_t *dst, const uint8_t *src, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) dst[i] = src[i];
}

void state_any_init(StateAny *s, void *storage, uint16_t size) {
    s->storage = storage;
    s->size = size;
    s->seq = 0;
    s->t_us = 0;
}

void state_any_set(StateAny *s, const void *value, uint32_t now_us) {
    mem_copy((uint8_t*)s->storage, (const uint8_t*)value, s->size);
    s->t_us = now_us;
    s->seq++;
}

uint32_t state_any_get(const StateAny *s, void *out, uint32_t *t_us_out) {
    mem_copy((uint8_t*)out, (const uint8_t*)s->storage, s->size);
    if (t_us_out) *t_us_out = s->t_us;
    return s->seq;
}
