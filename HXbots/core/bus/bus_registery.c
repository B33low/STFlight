#include "bus_desc.h"
#include "param_base.h"
#include "state_base.h"
#include "stream_base.h"


// These are your real objects elsewhere:
extern StateAny g_state_attitude;  // base object
extern StateAny g_state_altitude;  // base object
extern ParamAny g_param_imu_conv;  // base object
extern StreamAny g_stream_imu_raw; // base object

static const BusItem g_items[] = {
    {.id = 1,
     .kind = BUS_KIND_STREAM,
     .ptr = (void *)&g_stream_imu_raw,
     .name = "IMU_RAW"},
    {.id = 2,
     .kind = BUS_KIND_PARAM,
     .ptr = (void *)&g_param_imu_conv,
     .name = "IMU_CONV_META"},
    {.id = 3,
     .kind = BUS_KIND_STATE,
     .ptr = (void *)&g_state_attitude,
     .name = "ATTITUDE"},
    {.id = 4,
     .kind = BUS_KIND_STATE,
     .ptr = (void *)&g_state_altitude,
     .name = "ALTITUDE"},
};

const BusItem *bus_find(uint8_t id, BusKind kind) {
  for (uint32_t i = 0; i < sizeof(g_items) / sizeof(g_items[0]); i++) {
    if (g_items[i].id == id && g_items[i].kind == kind)
      return &g_items[i];
  }
  return 0;
}

uint16_t bus_item_size(const BusItem *it) {
  if (!it)
    return 0;

  switch (it->kind) {
  case BUS_KIND_STREAM:
    return ((StreamAny *)it->ptr)->elem_size;
  case BUS_KIND_STATE:
    return ((StateAny *)it->ptr)->size;
  case BUS_KIND_PARAM:
    return ((ParamAny *)it->ptr)->base.size;
  default:
    return 0;
  }
}
