#pragma once

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float setpoint_vz;
    float time_s;   // temps absolu fourni par la plateforme
} FcInput;

typedef struct {
    float motor[4]; // 0..1
} FcOutput;

typedef struct {
    float last_time_s;
    int   first_call;
    float vx_estimate,vy_estimate,vz_estimate;
    float px_estimate,py_estimate,pz_estimate;
    float vz_i;
    float vz_prev_err;
    // filtres, PID, états attitude, etc.
} FcState;

typedef struct {
    float dt;

    float e_vz;
    float u_p;
    float u_i;
    float u_d;
    float u_raw;
    float u_sat;

    float vz_est;
    float pz_est;    
} FcDebug;

void fc_init(FcState* s);
void fc_step(const FcInput* in, FcOutput* out, FcState* s,FcDebug *dbg);
