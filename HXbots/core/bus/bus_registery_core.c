#include "bus_registery_core.h"
#include "param_base.h"
#include "state_base.h"
#include "stream_base.h"

static const BusItem *s_items = 0;
static uint32_t s_count = 0;

void bus_registry_set(const BusItem *items, uint32_t count) {
    s_items = items;
    s_count = count;
}

uint32_t bus_registry_count(void) {
    return s_count;
}

const BusItem *bus_find(uint8_t id, BusKind kind) {
    if (!s_items || s_count == 0) return 0;

    for (uint32_t i = 0; i < s_count; i++) {
        if (s_items[i].id == id && s_items[i].kind == kind)
            return &s_items[i];
    }
    return 0;
}

uint16_t bus_item_size(const BusItem *it) {
    if (!it) return 0;

    switch (it->kind) {
    case BUS_KIND_STREAM:
        return ((StreamAny *)it->ptr)->elem_size;

    case BUS_KIND_STATE:
        return ((StateAny *)it->ptr)->size;

    case BUS_KIND_PARAM:
        // ParamAny contient un StateAny base
        return ((ParamAny *)it->ptr)->base.size;

    default:
        return 0;
    }
}
