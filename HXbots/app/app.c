// app.c
#include "app.h"
#include "fc.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bus_desc.h"
#include "bus_serialize.h"
#include "param_base.h"
#include "state_base.h"
#include "stream_base.h"

#include "attitude_state.h"
#include "imu_params.h"
#include "imu_stream.h"

#include "altitude_estimator.h"
#include "altitude_state.h"

#include "att_filter_params.h"
#include "attitude_estimator.h"


#define MODE_FLIGHT 0x01
#define MODE_OPEN_LOOP 0x02
#define MODE_SENSOR_LPS22_DEMO 0x04
#define MODE_SENSOR_LSM6DSO32_DEMO 0x08
#define MODE_HIL 0x10

#define MODE_SENSOR_BOARD (MODE_SENSOR_LPS22_DEMO | MODE_SENSOR_LSM6DSO32_DEMO)

#define APP_MODE (MODE_OPEN_LOOP | MODE_HIL)

#if (APP_MODE & MODE_SENSOR_LPS22_DEMO) == MODE_SENSOR_LPS22_DEMO
extern SPI_HandleTypeDef hspi2;
static LPS22HB_Handle_t lps22hb;

static float pressure;
static float temp;
static uint8_t status;
static int lastResult;
static char buffer[128];
#endif

#if (APP_MODE & MODE_SENSOR_LSM6DSO32_DEMO) == MODE_SENSOR_LSM6DSO32_DEMO
extern SPI_HandleTypeDef hspi2;
static LSM6DSO32_Handle_t lsm6dso32;

static LSM6DSO32_AccelRaw_t acceleration;
static uint8_t status;
static int lastResult;
static char buffer[128];
#endif

extern UART_HandleTypeDef huart2;

static FcState fc_state;
static FcInput fc_in;
static FcOutput fc_out;
static FcDebug fc_dbg;

StreamAny g_stream_imu_raw;

ParamAny g_param_imu_conv;
ParamAny g_param_att_filter;

StateAny g_state_attitude;
StateAny g_state_altitude;

static AttEstCtx g_att_ctx;

static AltEstCtx g_alt_ctx;
static ImuRawSample g_imu_raw_storage[64];

static uint32_t last_tick_ms = 0;

static void uart_log(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
}

static void mcu_uart_write(const uint8_t *data, uint16_t len, void *ctx) {
  UART_HandleTypeDef *hu = (UART_HandleTypeDef *)ctx;
  if (!hu || !data || !len)
    return;
  HAL_UART_Transmit(hu, (uint8_t *)data, len, 10);
}

// ========================= UART RX PARSER =========================
#ifndef BUS_MSG_PUBLISH
#define BUS_MSG_PUBLISH 1
#define BUS_MSG_WRITE 2
#define BUS_MSG_INJECT 3
#define BUS_MSG_READREQ 4
#endif
static uint8_t hil_rx_byte;

static void hil_uart_start_rx(void) {
  HAL_UART_Receive_IT(&huart2, &hil_rx_byte, 1);
}

typedef enum {
  RX_WAIT_AA = 0,
  RX_WAIT_55,
  RX_READ_MSG,
  RX_READ_KIND,
  RX_READ_ID,
  RX_READ_LEN,
  RX_READ_PAYLOAD
} HilRxState;

static HilRxState hil_rx_st = RX_WAIT_AA;

static uint8_t hil_msg = 0;
static uint8_t hil_kind = 0;
static uint8_t hil_id = 0;
static uint8_t hil_len = 0;

static uint8_t hil_idx = 0;
static uint8_t hil_payload[255];

// Forward decl from registry
const BusItem *bus_find(uint8_t id, BusKind kind);
uint16_t bus_item_size(const BusItem *it);

static bool hil_apply_frame(uint8_t msg_u8, uint8_t kind_u8, uint8_t id,
                            const uint8_t *payload, uint8_t len,
                            uint32_t now_us) {
  BusKind kind = (BusKind)kind_u8;

  // We only accept host->MCU control messages
  // (be permissive: allow WRITE and INJECT)
  if (msg_u8 != BUS_MSG_INJECT && msg_u8 != BUS_MSG_WRITE) {
    return false;
  }

  const BusItem *it = bus_find(id, kind);
  if (!it)
    return false;

  uint16_t expected = bus_item_size(it);
  if (expected == 0 || expected != len)
    return false;

  if (kind == BUS_KIND_STREAM) {
    // INJECT makes sense for streams
    stream_any_push((StreamAny *)it->ptr, payload);
    return true;
  }

  if (kind == BUS_KIND_STATE) {
    // WRITE/INJECT both could be allowed if you want flexibility
    state_any_set((StateAny *)it->ptr, payload, now_us);
    return true;
  }

  if (kind == BUS_KIND_PARAM) {
    param_any_set((ParamAny *)it->ptr, payload, now_us);
    return true;
  }

  return false;
}

static void hil_on_uart_byte(uint8_t b, uint32_t now_us) {
  switch (hil_rx_st) {
  case RX_WAIT_AA:
    hil_rx_st = (b == 0xAA) ? RX_WAIT_55 : RX_WAIT_AA;
    break;

  case RX_WAIT_55:
    if (b == 0x55)
      hil_rx_st = RX_READ_MSG;
    else
      hil_rx_st = (b == 0xAA) ? RX_WAIT_55 : RX_WAIT_AA;
    break;

  case RX_READ_MSG:
    hil_msg = b;
    hil_rx_st = RX_READ_KIND;
    break;

  case RX_READ_KIND:
    hil_kind = b;
    hil_rx_st = RX_READ_ID;
    break;

  case RX_READ_ID:
    hil_id = b;
    hil_rx_st = RX_READ_LEN;
    break;

  case RX_READ_LEN:
    hil_len = b;
    hil_idx = 0;

    if (hil_len == 0) {
      (void)hil_apply_frame(hil_msg, hil_kind, hil_id, hil_payload, 0, now_us);
      hil_rx_st = RX_WAIT_AA;
    } else {
      hil_rx_st = RX_READ_PAYLOAD;
    }
    break;

  case RX_READ_PAYLOAD:
    hil_payload[hil_idx++] = b;

    if (hil_idx >= hil_len) {
      (void)hil_apply_frame(hil_msg, hil_kind, hil_id, hil_payload, hil_len,
                            now_us);
      hil_rx_st = RX_WAIT_AA;
    }
    break;

  default:
    hil_rx_st = RX_WAIT_AA;
    break;
  }
}

void app_hil_uart_rx_callback(UART_HandleTypeDef *huart) {
#if (APP_MODE & MODE_HIL) == MODE_HIL
  if (huart != &huart2)
    return;

  uint32_t now_us = HAL_GetTick() * 1000u;
  hil_on_uart_byte(hil_rx_byte, now_us);

  // Re-arm 1-byte RX interrupt
  HAL_UART_Receive_IT(&huart2, &hil_rx_byte, 1);
#else
  (void)huart;
#endif
}

// ========================= END UART RX PARSER =========================

void app_init(void) {
  // Init du flight controller
  fc_init(&fc_state);
  memset(&fc_in, 0, sizeof(fc_in));
  memset(&fc_out, 0, sizeof(fc_out));
  memset(&fc_dbg, 0, sizeof(fc_dbg));

  last_tick_ms = 0; // HAL_GetTick();

#if (APP_MODE & MODE_HIL) == MODE_HIL

  HAL_GPIO_WritePin(CS_LSM6DSO32_GPIO_Port, CS_LSM6DSO32_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port, CS_LIS2MDL_Pin, GPIO_PIN_SET);

  stream_any_init(&g_stream_imu_raw, g_imu_raw_storage, 64,
                  sizeof(ImuRawSample));

  state_any_init(&g_param_imu_conv.base, &g_imu_conv, sizeof(ImuConvMeta));
  state_any_init(&g_state_attitude, &g_attitude, sizeof(Attitude));

  state_any_init(&g_state_altitude, &g_altitude, sizeof(AltitudeState));
  alt_est_init(&g_alt_ctx);

  state_any_init(&g_param_att_filter.base, &g_att_filter_storage,
                 sizeof(AttFilterParams));
  AttFilterParams fp0 = {.alpha = 0.98f};
  state_any_set(&g_param_att_filter.base, &fp0, 0);
  att_est_init(&g_att_ctx);

  ImuConvMeta m0 = {0};
  m0.accel_lsb_to_ms2 = 0.01f; // same default as PC sim
  state_any_set(&g_param_imu_conv.base, &m0, 0);

  uart_log("=== HIL ENABLED (UART inject) ===\r\n");
  hil_uart_start_rx();
#endif
#if (APP_MODE & MODE_OPEN_LOOP) == MODE_OPEN_LOOP
  uart_log("=== FC OPEN LOOP TEST ===\r\n");
#elif APP_MODE == MODE_FLIGHT
  uart_log("=== FC FLIGHT MODE (stub) ===\r\n");
#endif
#if (APP_MODE & MODE_SENSOR_LPS22_DEMO) == MODE_SENSOR_LPS22_DEMO

  // --- CONFIG CAPTEUR COMME DANS TON main.c ---
  lps22hb.hspi = &hspi2;
  lps22hb.csPort = CS_LPS22HB_GPIO_Port;
  lps22hb.csPin = CS_LPS22HB_Pin;
  lps22hb.config.interupt_mode = LPS22HB_CONFIG_INTERRUPT_MODE_DATA_READY;
  lps22hb.config.odr = LPS22HB_CONFIG_ODR_75HZ;
  lps22hb.config.lp_bw = LPS22HB_CONFIG_LP_BW_ODR_20;

  while (LPS22HB_Init(&lps22hb)) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
  }

  pressure = 0.0f;
  temp = 0.0f;
  status = 0;
  lastResult = 0;

  uart_log("LPS22HB sensor demo started\r\n");
#endif
#if (APP_MODE & MODE_SENSOR_LSM6DSO32_DEMO) == MODE_SENSOR_LSM6DSO32_DEMO

  // --- CONFIG CAPTEUR COMME DANS TON main.c ---
  lsm6dso32.hspi = &hspi2;
  lsm6dso32.csPort = CS_LSM6DSO32_GPIO_Port;
  lsm6dso32.csPin = CS_LSM6DSO32_Pin;

  while (LSM6DSO32_Init(&lsm6dso32)) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
  }

  acceleration.x = acceleration.y = acceleration.z = 0;
  status = 0;
  lastResult = 0;

  uart_log("LSM6DSO32 sensor demo started\r\n");
  uint8_t c1, c2, c3;
  LSM6DSO32_ReadReg(&lsm6dso32, LSM6DSO32_REG_CTRL1_XL, &c1, 1);
  LSM6DSO32_ReadReg(&lsm6dso32, LSM6DSO32_REG_CTRL2_G, &c2, 1);
  LSM6DSO32_ReadReg(&lsm6dso32, LSM6DSO32_REG_CTRL3_C, &c3, 1);

  int len =
      snprintf(buffer, sizeof(buffer),
               "Init regs: CTRL1_XL=0x%02X, CTRL2_G=0x%02X, CTRL3_C=0x%02X\r\n",
               c1, c2, c3);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);
#endif
}

void app_loop(void) {
  uint32_t now_ms = HAL_GetTick();
  float t = now_ms / 1000.0f;

  (void)t;
  static uint32_t last_bus_tx_ms = 0;
  if ((now_ms - last_bus_tx_ms) >= 50) { // 20 Hz, change as you like
    last_bus_tx_ms = now_ms;
    bus_serialize_latest(1, BUS_KIND_STREAM, mcu_uart_write, &huart2);
    bus_serialize_latest(2, BUS_KIND_PARAM, mcu_uart_write, &huart2);
    bus_serialize_latest(3, BUS_KIND_STATE, mcu_uart_write, &huart2);
    bus_serialize_latest(4, BUS_KIND_STATE, mcu_uart_write, &huart2);
  }
#if (APP_MODE & MODE_HIL) == MODE_HIL
  // Run estimator as fast as loop executes
  alt_est_step(&g_alt_ctx, &g_stream_imu_raw, &g_param_imu_conv,
               &g_state_altitude);
  att_est_step(&g_att_ctx, &g_stream_imu_raw, &g_param_imu_conv,
               &g_param_att_filter, &g_state_attitude);
#endif

#if (APP_MODE & MODE_OPEN_LOOP) == MODE_OPEN_LOOP

  static int step = 0;

  fc_in.time_s = ((float)step) / 1000;
  fc_in.ax = 0.0f;
  fc_in.ay = 0.0f;
  fc_in.az = (float)step;
  fc_in.gx = fc_in.gy = fc_in.gz = 0.0f;
  fc_in.setpoint_vz = 1.0f;

  fc_step(&fc_in, &fc_out, &fc_state, &fc_dbg);

#if (APP_MODE & MODE_HIL) != MODE_HIL
  // Only print CSV when NOT in HIL
  char buf[200];
  int len = snprintf(buf, sizeof(buf),
                     "%.3f,%.3f,%.3f,%.3f,%.3f,"
                     "%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                     fc_in.time_s, fc_out.motor[0], fc_out.motor[1],
                     fc_out.motor[2], fc_out.motor[3], fc_dbg.dt, fc_dbg.e_vz,
                     fc_dbg.u_p, fc_dbg.u_i, fc_dbg.u_d, fc_dbg.u_raw,
                     fc_dbg.u_sat, fc_dbg.vz_est, fc_dbg.pz_est);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 10);
#endif
  step++;

#elif APP_MODE == MODE_FLIGHT
  // Here will be real IMU and flight stuff
#elif APP_MODE == MODE_SENSOR_LSM6DSO32_DEMO || APP_MODE == MODE_SENSOR_BOARD
  lsm6dso32_data_ready = true; // for now always poll

  if (lsm6dso32_data_ready) {
    lastResult = LSM6DSO32_ReadAccelRaw(&lsm6dso32, &acceleration);
    lsm6dso32_data_ready = false;

    int16_t x_raw = acceleration.x;
    int16_t y_raw = acceleration.y;
    int16_t z_raw = acceleration.z;

    // Sensitivity for ±8 g: 0.244 mg/LSB  -> 0.244 / 1000 g/LSB
    const float acc_sens_g = 0.244f / 1000.0f; // g / LSB

    float ax = x_raw * acc_sens_g;
    float ay = y_raw * acc_sens_g;
    float az = z_raw * acc_sens_g;

    // ---- Format ax ----
    char *signAx = (ax < 0.0f) ? "-" : "";
    float valAx = (ax < 0.0f) ? -ax : ax;
    int intAx = (int)valAx;
    float fracAx = valAx - intAx;
    int fracAx3 = (int)truncf(fracAx * 1000.0f); // 3 decimals

    // ---- Format ay ----
    char *signAy = (ay < 0.0f) ? "-" : "";
    float valAy = (ay < 0.0f) ? -ay : ay;
    int intAy = (int)valAy;
    float fracAy = valAy - intAy;
    int fracAy3 = (int)truncf(fracAy * 1000.0f);

    // ---- Format az ----
    char *signAz = (az < 0.0f) ? "-" : "";
    float valAz = (az < 0.0f) ? -az : az;
    int intAz = (int)valAz;
    float fracAz = valAz - intAz;
    int fracAz3 = (int)truncf(fracAz * 1000.0f);

    uint8_t who = 0;
    LSM6DSO32_ReadReg(&lsm6dso32, LSM6DSO32_REG_WHO_AM_I, &who, 1);

    int len = snprintf(
        buffer, sizeof(buffer),
        "who_am_i: %d, ax: %s%d.%03d g, ay: %s%d.%03d g, az: %s%d.%03d g\r\n",
        (int)who, signAx, intAx, fracAx3, signAy, intAy, fracAy3, signAz, intAz,
        fracAz3);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);
    uint8_t r_ctrl1 = 0, r_ctrl2 = 0, r_ctrl3 = 0, r_status = 0;
    LSM6DSO32_ReadReg(&lsm6dso32, LSM6DSO32_REG_CTRL1_XL, &r_ctrl1, 1);
    LSM6DSO32_ReadReg(&lsm6dso32, LSM6DSO32_REG_CTRL2_G, &r_ctrl2, 1);
    LSM6DSO32_ReadReg(&lsm6dso32, 0x12, &r_ctrl3, 1);  // CTRL3_C
    LSM6DSO32_ReadReg(&lsm6dso32, 0x1E, &r_status, 1); // STATUS_REG

    int len2 =
        snprintf(buffer, sizeof(buffer),
                 "raw: %d %d %d, CTRL1_XL=0x%02X, CTRL2_G=0x%02X, "
                 "CTRL3_C=0x%02X, STATUS=0x%02X\r\n",
                 x_raw, y_raw, z_raw, r_ctrl1, r_ctrl2, r_ctrl3, r_status);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len2, 1000);
    HAL_Delay(100);
  }

  // The rest is unchanged (status + LED handling)
  //   if (LPS22HB_Status(&lps22hb, &status) != 0) {
  //     // Stall mode en cas d'erreur
  //     while (1) {
  //       HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //       HAL_Delay(500);
  //       HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //       HAL_Delay(500);
  //     }
  //   }

  //   if ((status & 0x03) == 0x03) {
  //     HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  //   } else {
  //     HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  //   }

  if (lastResult != 0) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
#elif APP_MODE == MODE_SENSOR_LPS22_DEMO

  if (lps22hb_data_ready) {
    lastResult = LPS22HB_ReadPT_Burst_hPa_C(&lps22hb, &pressure, &temp);
    lps22hb_data_ready = false;

    char *tmpSignPressure = (pressure < 0) ? "-" : "";
    float tmpValPressure = (pressure < 0) ? -pressure : pressure;
    int tmpInt1Pressure = (int)tmpValPressure;
    float tmpFracPressure = tmpValPressure - tmpInt1Pressure;
    int tmpInt2Pressure = (int)truncf(tmpFracPressure * 10000.0f);

    char *tmpSignTemp = (temp < 0) ? "-" : "";
    float tmpValTemp = (temp < 0) ? -temp : temp;
    int tmpInt1Temp = (int)tmpValTemp;
    float tmpFracTemp = tmpValTemp - tmpInt1Temp;
    int tmpInt2Temp = (int)truncf(tmpFracTemp * 100.0f);

    int len = snprintf(buffer, sizeof(buffer), "p: %s%d.%04d, t: %s%d.%02d\r\n",
                       tmpSignPressure, tmpInt1Pressure, tmpInt2Pressure,
                       tmpSignTemp, tmpInt1Temp, tmpInt2Temp);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);
  }

  if (LPS22HB_Status(&lps22hb, &status) != 0) {
    // Stall mode en cas d'erreur
    while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(500);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(500);
    }
  }

  if ((status & 0x03) == 0x03) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }

  if (lastResult != 0) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  HAL_Delay(100);

#endif
}
