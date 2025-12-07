#include "stream_base.h"

static void mem_copy(uint8_t *dst, const uint8_t *src, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) dst[i] = src[i];
}

void stream_any_init(StreamAny *s, void *storage,
                     uint16_t cap, uint16_t elem_size) {
    s->buf = (uint8_t*)storage;
    s->cap = cap;
    s->elem_size = elem_size;
    s->head = 0;
    s->count = 0;
}

void stream_any_push(StreamAny *s, const void *elem) {
    uint16_t i = s->head;
    uint8_t *dst = s->buf + i * s->elem_size;
    mem_copy(dst, (const uint8_t*)elem, s->elem_size);

    s->head = (uint16_t)((i + 1) % s->cap);
    if (s->count < s->cap) s->count++;
}

bool stream_any_latest(const StreamAny *s, void *out) {
    if (s->count == 0) return false;
    uint16_t i = (uint16_t)((s->head + s->cap - 1) % s->cap);
    const uint8_t *src = s->buf + i * s->elem_size;
    mem_copy((uint8_t*)out, src, s->elem_size);
    return true;
}

bool stream_any_pop(StreamAny *s, void *out) {
    if (s->count == 0) return false;

    // oldest index
    uint16_t tail = (uint16_t)((s->head + s->cap - s->count) % s->cap);
    uint8_t *src = s->buf + tail * s->elem_size;
    mem_copy((uint8_t*)out, src, s->elem_size);

    s->count--;
    return true;
}
