#include "fc.h"

void fc_init(FcState *s) {
  s->first_call = 1;
  s->last_time_s = 0;

  s->vx_estimate = 0;
  s->vy_estimate = 0;
  s->vz_estimate = 0;

  s->px_estimate = 0;
  s->py_estimate = 0;
  s->pz_estimate = 0;

  s->vz_i = 0.0f;
  s->vz_prev_err = 0.0f;
}
void fc_step(const FcInput *in, FcOutput *out, FcState *s, FcDebug *dbg) {
  float dt;
  if (s->first_call) {
    dt = 0.0f; // ou un dt nominal
    s->first_call = 0;
  } else {
    dt = in->time_s - s->last_time_s;
  }
  s->last_time_s = in->time_s;

  float estimate_vx = (s->vx_estimate) + (in->ax) * dt;
  float estimate_vy = (s->vy_estimate) + (in->ay) * dt;
  float estimate_vz = (s->vz_estimate) + (in->az) * dt;

  float estimate_px = (s->px_estimate) + estimate_vx * dt;
  float estimate_py = (s->py_estimate) + estimate_vy * dt;
  float estimate_pz = (s->pz_estimate) + estimate_vz * dt;

  // --- PID on vz (for now: only P) ---
  float e_vz = in->setpoint_vz - estimate_vz;

  float Kp = 10.0f;
  float Ki = 0.015f;
  float Kd = 0.0f;

  float u_p = Kp * e_vz;
  float u_i = s->vz_i + Ki * e_vz;
  float u_d = 0.0f;

  float u_raw = u_p + u_i + u_d;

  // saturate
  float u_sat = u_raw;
  if (u_sat > 1.0f)
    u_sat = 1.0f;
  if (u_sat < 0.0f)
    u_sat = 0.0f;

  out->motor[0] = u_sat;
  out->motor[1] = u_sat;
  out->motor[2] = u_sat;
  out->motor[3] = u_sat;

  // --- save state ---

  s->vx_estimate = estimate_vx;
  s->vy_estimate = estimate_vy;
  s->vz_estimate = estimate_vz;

  s->px_estimate = estimate_px;
  s->py_estimate = estimate_py;
  s->pz_estimate = estimate_pz;

  s -> vz_i = u_i;
  // --- fill debug if provided ---
  if (dbg) {
    dbg->dt = dt;

    dbg->e_vz = e_vz;
    dbg->u_p = u_p;
    dbg->u_i = u_i;
    dbg->u_d = u_d;
    dbg->u_raw = u_raw;
    dbg->u_sat = u_sat;

    dbg->vz_est = estimate_vz;
    dbg->pz_est = estimate_pz;
  }
}