#include "simulator.h"

void sim_init(SimulatorState* sim_state){
    sim_state->ax = sim_state->ay = sim_state->az = 0.0f;
    sim_state->vx = sim_state->vy = sim_state->vz = 0.0f;
    sim_state->px = sim_state->py = sim_state->pz = 0.0f;
    sim_state->time_s = 0.0f;
}

void sim_step(SimulatorState* sim_state, SimulatorInput* sim_in, SimulatorOutput* sim_out){
    float t_now = sim_in->time_s;
    float dt = t_now - sim_state->time_s;
    if (dt <= 0.0f) dt = 0.001f;

    float u = sim_in->u;
    if (u > 1.0f) u = 1.0f;
    if (u < 0.0f) u = 0.0f;

    float u_hover = 0.5f;
    float a_max = 10.0f;
    float k_d = 2.0f;

    float a = a_max * (u - u_hover) * 2.0f
              - k_d * sim_state->vz;

    sim_state->az = a;
    sim_state->vz += a * dt;
    sim_state->pz += sim_state->vz * dt;
    sim_state->time_s = t_now;

    sim_out->ax = 0.0f;
    sim_out->ay = 0.0f;
    sim_out->az = sim_state->az;
    sim_out->time_s = t_now;
}
