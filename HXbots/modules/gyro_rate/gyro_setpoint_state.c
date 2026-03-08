#include "gyro_setpoint_state.h"
StateGyroSetpoint g_gyro_setpoint;

void state_gyro_setpoint_init(StateGyroSetpoint *s)
{
    state_any_init(&s->base, &s->storage, sizeof(GyroSetpointState));
    // set inital value to 0
    GyroSetpointState initial = {.gx = 0, .gy = 1, .gz = 2};
    state_gyro_setpoint_set(s, initial, 0);
}

void state_gyro_setpoint_set(StateGyroSetpoint *s, GyroSetpointState v, uint32_t now_us)
{
    state_any_set(&s->base, &v, now_us);
}

uint32_t state_gyro_setpoint_get(const StateGyroSetpoint *s, GyroSetpointState *out, uint32_t *t_us_out)
{
    return state_any_get(&s->base, out, t_us_out);
}
