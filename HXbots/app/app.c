// app.c
#include "app.h"
#include "fc.h"
#include "lps22hb.h"
#include "lsm6dso32.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bus_desc.h"
#include "bus_serialize.h"
#include "param_base.h"
#include "state_base.h"
#include "stream_base.h"

#include "bus_registery_app.h"

#include "attitude_state.h"
#include "imu_params.h"
#include "imu_stream.h"
#include "imu_lsm6dso32.h"

#include "altitude_estimator.h"
#include "altitude_state.h"

#include "att_filter_params.h"
#include "attitude_estimator.h"

#define MODE_FLIGHT 0x01
#define MODE_OPEN_LOOP 0x02
#define MODE_SENSOR_LPS22_DEMO 0x04
#define MODE_SENSOR_LSM6DSO32_DEMO 0x08
#define MODE_HIL 0x10

#define US_1KHZ  1000

#define MODE_SENSOR_BOARD (MODE_SENSOR_LPS22_DEMO | MODE_SENSOR_LSM6DSO32_DEMO)
#define MODE_HXBOT_TEST (MODE_OPEN_LOOP | MODE_HIL)

/* CURRENT MODE */
#define APP_MODE (MODE_SENSOR_LSM6DSO32_DEMO | MODE_HIL )

static LPS22HB_Handle_t lps22hb;
static LSM6DSO32_Handle_t lsm6dso32;

/* ------------ LPS22HB DEMO VARS ------------ */
#if (APP_MODE & MODE_SENSOR_LPS22_DEMO) == MODE_SENSOR_LPS22_DEMO
extern SPI_HandleTypeDef hspi2;

static float pressure;
static float temp;
static uint8_t status;
static int lastResult;
static char buffer[128];
static uint32_t last_tick_ms_lps22 = 0;
#endif

/* ------------ LSM6DSO32 DEMO VARS ------------ */
#if (APP_MODE & MODE_SENSOR_LSM6DSO32_DEMO) == MODE_SENSOR_LSM6DSO32_DEMO
extern SPI_HandleTypeDef hspi2;

static LSM6DSO32_AccelRaw_t acceleration;
static LSM6DSO32_GyroRaw_t gyro_raw;
static uint8_t status;
static int lastResult;

#endif

extern UART_HandleTypeDef huart2;

static FcState fc_state;
static FcInput fc_in;
static FcOutput fc_out;
static FcDebug fc_dbg;

ImuRawStream g_stream_imu_raw;

ParamImuConv g_param_imu_conv;
ParamAny g_param_att_filter;

StateAny g_state_attitude;
StateAny g_state_altitude;

static AttEstCtx g_att_ctx;
static AltEstCtx g_alt_ctx;

static ImuLsm6Ctx g_imu_lsm6_ctx;
static uint32_t last_tick_ms = 0;

/* ===================== LOG HELPERS ===================== */

static void uart_log(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
}

static void mcu_uart_write(const uint8_t *data, uint16_t len, void *ctx) {
  UART_HandleTypeDef *hu = (UART_HandleTypeDef *)ctx;
  if (!hu || !data || !len)
    return;
  HAL_UART_Transmit(hu, (uint8_t *)data, len, 10);
}

/* ===================== UART RX PARSER (HIL) ===================== */
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
    stream_any_push((StreamAny *)it->ptr, payload);
    return true;
  }

  if (kind == BUS_KIND_STATE) {
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

  HAL_UART_Receive_IT(&huart2, &hil_rx_byte, 1);
#else
  (void)huart;
#endif
}

/* ===================== INIT ===================== */

void app_init(void) {
  fc_init(&fc_state);
  memset(&fc_in, 0, sizeof(fc_in));
  memset(&fc_out, 0, sizeof(fc_out));
  memset(&fc_dbg, 0, sizeof(fc_dbg));

  last_tick_ms = 0;

#if (APP_MODE & MODE_HIL) == MODE_HIL
  HAL_GPIO_WritePin(CS_LSM6DSO32_GPIO_Port, CS_LSM6DSO32_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port, CS_LIS2MDL_Pin, GPIO_PIN_SET);

  // imu data services
  imu_raw_stream_init(&g_stream_imu_raw);
  imu_raw_param_conv_init(&g_param_imu_conv);

  ImuConvMeta m0 = {0};
  m0.accel_lsb_to_ms2 = 0.01f; // adjust later for real sensor if you want
  state_any_set(&g_param_imu_conv.base, &m0, 0);
  imu_lsm6dso32_init(&g_imu_lsm6_ctx,&lsm6dso32,&g_stream_imu_raw,&g_param_imu_conv,US_1KHZ);

  // attitude data services
  state_any_init(&g_state_attitude, &g_attitude, sizeof(Attitude));
  att_est_init(&g_att_ctx);
  state_any_init(&g_param_att_filter.base, &g_att_filter_storage,
                 sizeof(AttFilterParams));

  AttFilterParams fp0 = {.alpha = 0.98f};
  state_any_set(&g_param_att_filter.base, &fp0, 0);


  // altitude data services
  state_any_init(&g_state_altitude, &g_altitude, sizeof(AltitudeState));
  alt_est_init(&g_alt_ctx);



  uart_log("=== HIL ENABLED (UART inject) ===\r\n");
  hil_uart_start_rx();
#endif
bus_registry_app_init();

#if (APP_MODE & MODE_OPEN_LOOP) == MODE_OPEN_LOOP
  uart_log("=== FC OPEN LOOP TEST ===\r\n");
#elif APP_MODE == MODE_FLIGHT
  uart_log("=== FC FLIGHT MODE (stub) ===\r\n");
#endif

  /* ------- LPS22 CONFIG (if enabled) ------- */
  lps22hb.hspi = &hspi2;
  lps22hb.csPort = CS_LPS22HB_GPIO_Port;
  lps22hb.csPin = CS_LPS22HB_Pin;
  lps22hb.config.interupt_mode = LPS22HB_CONFIG_INTERRUPT_MODE_DATA_READY;
  lps22hb.config.odr = LPS22HB_CONFIG_ODR_75HZ;
  lps22hb.config.lp_bw = LPS22HB_CONFIG_LP_BW_ODR_20;

#if (APP_MODE & MODE_SENSOR_LPS22_DEMO) == MODE_SENSOR_LPS22_DEMO
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
  lastResult = LPS22HB_ReadPT_Burst_hPa_C(&lps22hb, &pressure, &temp);
  last_tick_ms_lps22 = HAL_GetTick();
  uart_log("LPS22HB sensor demo started\r\n");
#endif

  /* ------- LSM6DSO32 CONFIG (if enabled) ------- */
#if (APP_MODE & MODE_SENSOR_LSM6DSO32_DEMO) == MODE_SENSOR_LSM6DSO32_DEMO
  lsm6dso32.hspi = &hspi2;
  lsm6dso32.csPort = CS_LSM6DSO32_GPIO_Port;
  lsm6dso32.csPin = CS_LSM6DSO32_Pin;

  LSM6DSO32_Select(&lsm6dso32, false);
  LPS22HB_Select(&lps22hb, false);

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

  /* Dummy read to clear any pending DRDY at startup */
  LSM6DSO32_ReadAccelGyroRaw(&lsm6dso32, &acceleration, &gyro_raw);
#endif
}

/* ===================== MAIN LOOP ===================== */

void app_loop(void) {
  uint32_t now_ms = HAL_GetTick();
  float t = now_ms / 1000.0f;
  (void)t;

  static uint32_t last_bus_tx_ms = 0;
  if ((now_ms - last_bus_tx_ms) >= 50) { // 20 Hz
    last_bus_tx_ms = now_ms;
    bus_serialize_latest(1, BUS_KIND_STREAM, mcu_uart_write, &huart2);
    bus_serialize_latest(2, BUS_KIND_PARAM, mcu_uart_write, &huart2);
    bus_serialize_latest(3, BUS_KIND_STATE, mcu_uart_write, &huart2);
    bus_serialize_latest(4, BUS_KIND_STATE, mcu_uart_write, &huart2);
  }

#if (APP_MODE & MODE_HIL) == MODE_HIL
  alt_est_step(&g_alt_ctx, &g_stream_imu_raw, &g_param_imu_conv,
               &g_state_altitude);
  att_est_step(&g_att_ctx, &g_stream_imu_raw, &g_param_imu_conv,
               &g_param_att_filter, &g_state_attitude);
#endif

#if (APP_MODE & MODE_OPEN_LOOP) == MODE_OPEN_LOOP
  /* ... open loop test unchanged ... */
#elif APP_MODE == MODE_FLIGHT
  /* future FC flight mode */
#endif

/* ----------- LSM6DSO32 DEMO LOOP + STREAM FEED ----------- */
#if (APP_MODE & MODE_SENSOR_LSM6DSO32_DEMO) == MODE_SENSOR_LSM6DSO32_DEMO
  if (lsm6dso32_data_ready) {
    imu_lsm6dso32_update(&g_imu_lsm6_ctx);
    lsm6dso32_data_ready = false;
  }

  if (lastResult != 0) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
#endif /* LSM6 DEMO */

/* ----------- LPS22HB DEMO LOOP ----------- */
#if (APP_MODE & MODE_SENSOR_LPS22_DEMO) == MODE_SENSOR_LPS22_DEMO
  status = 0;
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
    last_tick_ms_lps22 = now_ms;
  } else if ((now_ms - last_tick_ms_lps22) > 500 &&
             LPS22HB_Status(&lps22hb, &status) != 0) {
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
#endif /* LPS22 DEMO */
}
