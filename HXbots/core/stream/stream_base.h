#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t  *buf;
    uint16_t cap;
    uint16_t elem_size;
    uint16_t head;
    uint16_t count;
} StreamAny;

void stream_any_init(StreamAny *s, void *storage,
                     uint16_t cap, uint16_t elem_size);

void stream_any_push(StreamAny *s, const void *elem);

// Get latest element (copy out). Returns false if empty.
bool stream_any_latest(const StreamAny *s, void *out);

// Optional: pop oldest (if you want queue-like behavior).
bool stream_any_pop(StreamAny *s, void *out);
