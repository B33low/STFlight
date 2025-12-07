#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "fc.h"
#include "simulator.h"

#include "stream_base.h"
#include "state_base.h"
#include "param_base.h"

#include "imu_stream.h"
#include "imu_params.h"
#include "attitude_state.h"

#include "bus_desc.h"
#include "bus_serialize.h"

/* =========================================================
   Globals expected by bus_registery.c
   ========================================================= */
StreamAny g_stream_imu_raw;
ParamAny  g_param_imu_conv;
StateAny  g_state_attitude;

/* Backing storage for PC simulation */
static ImuRawSample g_imu_raw_storage[64];
static ImuConvMeta  g_imu_conv_storage;
static Attitude     g_attitude_storage;

/* =========================================================
   PC binary sink for embedded-style serialization
   ========================================================= */


typedef enum {
    UART_WAIT_AA = 0,
    UART_WAIT_55,
    UART_READ_KIND,
    UART_READ_ID,
    UART_READ_LEN,
    UART_READ_PAYLOAD
} UartParseState;

typedef struct {
    FILE *bin;
    FILE *txt;

    // --- frame parser state ---
    UartParseState st;
    uint8_t kind;
    uint8_t id;
    uint8_t len;
    uint8_t idx;
    uint8_t payload[255];

    // Optional: set this from your main loop before serializing
    float time_s;
} PcUartCtx;


static void pc_uart_write(const uint8_t *data, uint16_t len, void *ctx) {
    PcUartCtx *c = (PcUartCtx*)ctx;
    if (!c || !data || len == 0) return;

    // Still keep the raw binary capture
    if (c->bin) {
        fwrite(data, 1, len, c->bin);
    }

    // No text target? Then we are done.
    if (!c->txt) return;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        switch (c->st) {
            case UART_WAIT_AA:
                c->st = (b == 0xAA) ? UART_WAIT_55 : UART_WAIT_AA;
                break;

            case UART_WAIT_55:
                if (b == 0x55) c->st = UART_READ_KIND;
                else c->st = (b == 0xAA) ? UART_WAIT_55 : UART_WAIT_AA;
                break;

            case UART_READ_KIND:
                c->kind = b;
                c->st = UART_READ_ID;
                break;

            case UART_READ_ID:
                c->id = b;
                c->st = UART_READ_LEN;
                break;

            case UART_READ_LEN:
                c->len = b;
                c->idx = 0;

                if (c->len == 0) {
                    // Emit empty-payload frame
                    fprintf(c->txt, "time=%.6f kind=%u id=%u len=0 payload=\n",
                            c->time_s, c->kind, c->id);
                    c->st = UART_WAIT_AA;
                } else if (c->len > sizeof(c->payload)) {
                    // Invalid length -> resync
                    c->st = UART_WAIT_AA;
                } else {
                    c->st = UART_READ_PAYLOAD;
                }
                break;

            case UART_READ_PAYLOAD:
                c->payload[c->idx++] = b;

                if (c->idx >= c->len) {
                    // Complete frame -> print one nice line
                    fprintf(c->txt, "time=%.6f kind=%u id=%u len=%u payload=",
                            c->time_s, c->kind, c->id, c->len);

                    for (uint8_t k = 0; k < c->len; k++) {
                        fprintf(c->txt, "%02X", c->payload[k]);
                        if (k + 1 < c->len) fputc(' ', c->txt);
                    }
                    fputc('\n', c->txt);

                    c->st = UART_WAIT_AA;
                }
                break;

            default:
                c->st = UART_WAIT_AA;
                break;
        }
    }
}


/* =========================================================
   Time helper for PC sim
   ========================================================= */
static uint32_t micros_sim(float t_s) {
    double us = (double)t_s * 1000000.0;
    if (us < 0) us = 0;
    return (uint32_t)us;
}

/* =========================================================
   Minimal fake IMU producer
   ========================================================= */
static void imu_fake_push(float t_s, int i) {
    ImuRawSample raw;
    memset(&raw, 0, sizeof(raw));

    raw.ax = 0;
    raw.ay = 0;
    raw.az = (int16_t)i;          // your old fake pattern
    raw.gx = raw.gy = raw.gz = 0;
    raw.temp = 0;
    raw.t_us = micros_sim(t_s);

    stream_any_push(&g_stream_imu_raw, &raw);
}

/* =========================================================
   Optional: if you want to update attitude state for test
   (kept minimal: we just zero it each step)
   ========================================================= */
static void attitude_fake_update(float t_s) {
    (void)t_s;
    Attitude a;
    memset(&a, 0, sizeof(a));
    state_any_set(&g_state_attitude, &a, micros_sim(t_s));
}

/* =========================================================
   MAIN
   ========================================================= */
int main(void) {
    // ---- Existing FC setup ----
    FcState s;
    fc_init(&s);

    // ---- Simple simulator ----
    SimulatorState s_state = {0};
    sim_init(&s_state);

    SimulatorInput s_in = {0};
    SimulatorOutput s_out = {0};

    FcInput in = {0};
    FcOutput out = {{0,0,0,0}};
    FcDebug dbg = {0};

    float vz_cons = 1.0f;

    // ---- Init globals expected by bus registry ----
    stream_any_init(&g_stream_imu_raw, g_imu_raw_storage, 64, sizeof(ImuRawSample));
    state_any_init(&g_param_imu_conv.base, &g_imu_conv_storage, sizeof(ImuConvMeta));
    state_any_init(&g_state_attitude, &g_attitude_storage, sizeof(Attitude));

    // Default meta
    ImuConvMeta m0 = {0};
    m0.accel_lsb_to_ms2 = 0.01f;  // adjust to your intended scale
    state_any_set(&g_param_imu_conv.base, &m0, 0);

    // ---- Embedded-style binary telemetry file ----
    PcUartCtx uart = {0};
    uart.bin = fopen("telemetry.bin", "wb");
    uart.txt = fopen("telemetry_uart.txt", "w");
    uart.st = UART_WAIT_AA;
    // ---- CSV log ----
    FILE *csv = fopen("telemetry.csv", "w");
    if (csv) {
        fprintf(csv,
            "time,"
            "motor1,motor2,motor3,motor4,"
            "sim_az,sim_vz,sim_pz,"
            "setpoint,"
            "dt,e_vz,u_p,u_i,u_d,u_raw,u_sat,vz_est,pz_est\n"
        );
    }

    // Console header (optional)
    printf("time,motor1,motor2,motor3,motor4,sim_az,sim_vz,sim_pz,setpoint,"
           "dt,e_vz,u_p,u_i,u_d,u_raw,u_sat,vz_est,pz_est\n");

    float t = 0.0f;

    for (int i = 0; i < 100; ++i) { // 0.1s at 1 kHz
        in.time_s = t;

        // ---- New arch: push fake raw into stream ----
        imu_fake_push(t, i);

        // Keep attitude state alive for registry tests
        attitude_fake_update(t);

        // ---- Legacy FC input fields for now ----
        // You can later replace these with "state-derived" values
        in.ax = 0.0f;
        in.ay = 0.0f;
        in.az = (float)i;  // keep your previous simple test path
        in.gx = in.gy = in.gz = 0.0f;
        in.setpoint_vz = vz_cons;

        fc_step(&in, &out, &s, &dbg);

        s_in.time_s = t;
        s_in.u = out.motor[0];
        // sim_step(&s_state, &s_in, &s_out);

        // ---- Embedded-style serialization reuse ----
        if (uart.bin && uart.txt) {
            uart.time_s = t;
            bus_serialize_latest(1, BUS_KIND_STREAM, pc_uart_write, &uart);
            bus_serialize_latest(2, BUS_KIND_PARAM,  pc_uart_write, &uart);
            bus_serialize_latest(3, BUS_KIND_STATE,  pc_uart_write, &uart);
        }

        // ---- CSV ----
        if (csv) {
            fprintf(csv,
                "%.6f,"
                "%.3f,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,"
                "%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                t,
                out.motor[0], out.motor[1], out.motor[2], out.motor[3],
                s_state.az, s_state.vz, s_state.pz,
                in.setpoint_vz,
                dbg.dt, dbg.e_vz, dbg.u_p, dbg.u_i, dbg.u_d,
                dbg.u_raw, dbg.u_sat, dbg.vz_est, dbg.pz_est
            );
        }

        // ---- Console ----
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,"
               "%.3f,%.3f,%.3f,%.3f,"
               "%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               t,
               out.motor[0], out.motor[1], out.motor[2], out.motor[3],
               s_state.az, s_state.vz, s_state.pz, in.setpoint_vz,
               dbg.dt, dbg.e_vz, dbg.u_p, dbg.u_i, dbg.u_d,
               dbg.u_raw, dbg.u_sat, dbg.vz_est, dbg.pz_est);

        t += 0.001f;
    }

    if (uart.bin) fclose(uart.bin);
    if (uart.txt) fclose(uart.txt);
    if (csv) fclose(csv);

    return 0;
}
