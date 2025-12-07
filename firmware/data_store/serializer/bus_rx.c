#include "bus_rx.h"
#include "bus_desc.h"
#include "stream_base.h"
#include "state_base.h"
#include "param_base.h"

const BusItem* bus_find(uint8_t id, BusKind kind);
uint16_t bus_item_size(const BusItem *it);



bool bus_handle_frame(uint8_t msg, uint8_t kind_u8, uint8_t id,
                      const uint8_t *payload, uint8_t len,
                      uint32_t now_us)
{
    BusKind kind = (BusKind)kind_u8;
    const BusItem *it = bus_find(id, kind);
    if (!it) return false;

    uint16_t expected = bus_item_size(it);
    if (expected == 0) return false;

    // READREQ could be handled elsewhere (trigger serialize+send)
    if (msg == BUS_MSG_READREQ) {
        return true;
    }

    // PUBLISH is for telemetry-in; you may ignore on MCU RX
    if (msg == BUS_MSG_PUBLISH) {
        return true;
    }

    if (msg == BUS_MSG_INJECT && kind == BUS_KIND_STREAM) {
        if (len != expected) return false;
        stream_any_push((StreamAny*)it->ptr, payload);
        return true;
    }

    if (msg == BUS_MSG_WRITE && kind == BUS_KIND_STATE) {
        if (len != expected) return false;
        state_any_set((StateAny*)it->ptr, payload, now_us);
        return true;
    }

    if (msg == BUS_MSG_WRITE && kind == BUS_KIND_PARAM) {
        if (len != expected) return false;
        param_any_set((ParamAny*)it->ptr, payload, now_us);
        return true;
    }

    return false;
}
