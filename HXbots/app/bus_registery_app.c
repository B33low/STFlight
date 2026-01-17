#include "bus_registery_app.h"

#include "bus_registery_core.h"
#include "bus_desc.h"

#include "imu_stream.h"
#include "imu_params.h"
#include "attitude_state.h"
#include "altitude_state.h"

enum {
    ID_IMU_RAW   = 1,
    ID_IMU_CONV  = 2,
    ID_ATT_STATE = 3,
    ID_ALT_STATE = 4
};

extern StateAny  g_state_attitude;
extern StateAny  g_state_altitude;
extern ParamAny  g_param_imu_conv;
extern ImuRawStream g_stream_imu_raw;

static const BusItem g_items[] = {
    {.id = ID_IMU_RAW,   .kind = BUS_KIND_STREAM, .ptr = &g_stream_imu_raw.base, .name = "IMU_RAW"},
    {.id = ID_IMU_CONV,  .kind = BUS_KIND_PARAM,  .ptr = &g_param_imu_conv, .name = "IMU_CONV_META"},
    {.id = ID_ATT_STATE, .kind = BUS_KIND_STATE,  .ptr = &g_state_attitude, .name = "ATTITUDE"},
    {.id = ID_ALT_STATE, .kind = BUS_KIND_STATE,  .ptr = &g_state_altitude, .name = "ALTITUDE"},
};

void bus_registry_app_init(void) {
    bus_registry_set(g_items, (uint32_t)(sizeof(g_items) / sizeof(g_items[0])));
}
