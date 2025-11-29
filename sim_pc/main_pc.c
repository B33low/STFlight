#include <stdio.h>
#include "fc.h"
#include "simulator.h"

int main(void) {
    FcState s;
    fc_init(&s);


    SimulatorState s_state = {0};

    sim_init(&s_state);



    float vz_cons = 1; // m/s
    



    SimulatorInput s_in = {0};
    SimulatorOutput s_out = {0};

    FcInput in = {0};
    FcOutput out = {{0,0,0,0}};
    FcDebug dbg = {0};

    float t = 0.0f;
    printf("time,motor1,motor2,motor3,motor4,sim_az,sim_vz,sim_pz,setpoint,"
       "dt,e_vz,u_p,u_i,u_d,u_raw,u_sat,vz_est,pz_est\n");

    for (int i = 0; i < 10000; ++i) { // 10 secondes à 1 kHz
        in.time_s = t;

        in.ax = s_out.ax; in.ay = s_out.ay; in.az = s_out.az; // False sensor simulation
        in.setpoint_vz = vz_cons;
        in.gx = in.gy = in.gz = 0.0f;

        fc_step(&in, &out, &s, &dbg);

        s_in.time_s = t;
        s_in.u = out.motor[0];

        // Drone state update (Simulation)
        sim_step(  &s_state, &s_in, &s_out);


        printf("%.3f,%.3f,%.3f,%.3f,%.3f,"  // time, motors
           "%.3f,%.3f,%.3f,%.3f,"        // sim_az,sim_vz,sim_pz,setpoint
           "%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           t,
           out.motor[0], out.motor[1], out.motor[2], out.motor[3],
           s_state.az, s_state.vz, s_state.pz, in.setpoint_vz,
           dbg.dt, dbg.e_vz, dbg.u_p, dbg.u_i, dbg.u_d,
           dbg.u_raw, dbg.u_sat, dbg.vz_est, dbg.pz_est);

        t += 0.001f;
    }

    return 0;
}


void sim_init(SimulatorState* sim_state){
    sim_state->ax = sim_state->ay = sim_state->az = 0.0f;
    sim_state->vx = sim_state->vy = sim_state->vz = 0.0f;
    sim_state->px = sim_state->py = sim_state->pz = 0.0f;
    sim_state->time_s = 0.0f;

}

void sim_step( SimulatorState* sim_state, SimulatorInput* sim_in, SimulatorOutput* sim_out){
    float t_now = sim_in -> time_s;
    float dt = t_now - (sim_state -> time_s);
    if (dt <= 0.0f) dt =0.001f;

    float u = (sim_in -> u);
    if (u > 1.0f) u = 1.0f;
    if (u < 0.0f) u= 0.0f;

    float u_hover = 0.5f; // 50% thrust = hover to model weight effect
    float a_max = 10.0f; // m/s2
    float k_d     = 2.0f;          // dumping from speed

    float a = a_max * (u - u_hover) * 2.0f   // thrust
              - k_d * sim_state->vz;         // dumping

    sim_state->az = a;
    sim_state->vz += a * dt;
    sim_state->pz += sim_state->vz * dt;
    sim_state->time_s = t_now;

    // IMU simulation
    sim_out->ax = 0.0f;
    sim_out->ay = 0.0f;
    sim_out->az = sim_state->az;
    sim_out->time_s = t_now;
}