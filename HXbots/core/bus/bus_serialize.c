#include "bus_serialize.h"
#include "stream_base.h"
#include "state_base.h"
#include "param_base.h"
#include <string.h>

// You already have these in your registry:
const BusItem* bus_find(uint8_t id, BusKind kind);
uint16_t bus_item_size(const BusItem *it); // if you implemented it like we discussed

static void write_u8(bus_write_fn_t w, void *ctx, uint8_t v) {
    w(&v, 1, ctx);
}

static void write_frame(bus_write_fn_t w, void *ctx,
                        uint8_t kind, uint8_t id,
                        const uint8_t *payload, uint8_t len) {
    write_u8(w, ctx, 0xAA);
    write_u8(w, ctx, 0x55);
    write_u8(w, ctx, BUS_MSG_PUBLISH);
    write_u8(w, ctx, kind);
    write_u8(w, ctx, id);
    write_u8(w, ctx, len);
    if (len && payload) w(payload, len, ctx);
}

bool bus_serialize_latest(uint8_t id, BusKind kind, bus_write_fn_t w, void *ctx) {
    const BusItem *it = bus_find(id, kind);
    if (!it) return false;

    uint16_t size = bus_item_size(it);
    if (size == 0 || size > 255) return false;

    uint8_t tmp[256]; // safe for small telemetry payloads

    switch (kind) {
        case BUS_KIND_STATE: {
            uint32_t t;
            state_any_get((const StateAny*)it->ptr, tmp, &t);
            write_frame(w, ctx, (uint8_t)kind, id, tmp, (uint8_t)size);
            return true;
        }
        case BUS_KIND_PARAM: {
            uint32_t t;
            const ParamAny *p = (const ParamAny*)it->ptr;
            state_any_get(&p->base, tmp, &t);
            write_frame(w, ctx, (uint8_t)kind, id, tmp, (uint8_t)size);
            return true;
        }
        case BUS_KIND_STREAM: {
            // For streams, serialize "latest element"
            if (!stream_any_latest((const StreamAny*)it->ptr, tmp)) return false;
            write_frame(w, ctx, (uint8_t)kind, id, tmp, (uint8_t)size);
            return true;
        }
        default:
            return false;
    }
}
