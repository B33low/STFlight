// app.c
#include "app.h"
#include "fc.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define MODE_FLIGHT 1
#define MODE_OPEN_LOOP 2
#define MODE_SENSOR_LPS22_DEMO 3
#define MODE_SENSOR_LSM6DSO32_DEMO 4

#define APP_MODE MODE_SENSOR_LSM6DSO32_DEMO

#if APP_MODE == MODE_SENSOR_LPS22_DEMO
extern SPI_HandleTypeDef hspi2;
static LPS22HB_Handle_t lps22hb;

static float pressure;
static float temp;
static uint8_t status;
static int lastResult;
static char buffer[64];
#endif

#if APP_MODE == MODE_SENSOR_LSM6DSO32_DEMO
extern SPI_HandleTypeDef hspi2;
static LSM6DSO32_Handle_t lsm6dso32;

static LSM6DSO32_AccelRaw_t acceleration;
static uint8_t status;
static int lastResult;
static char buffer[64];
#endif

extern UART_HandleTypeDef huart2;

static FcState fc_state;
static FcInput fc_in;
static FcOutput fc_out;
static FcDebug fc_dbg;

static uint32_t last_tick_ms = 0;

static void uart_log(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
}

void app_init(void) {
  // Init du flight controller
  fc_init(&fc_state);
  memset(&fc_in, 0, sizeof(fc_in));
  memset(&fc_out, 0, sizeof(fc_out));
  memset(&fc_dbg, 0, sizeof(fc_dbg));

  last_tick_ms = 0; // HAL_GetTick();

#if APP_MODE == MODE_OPEN_LOOP
  uart_log("=== FC OPEN LOOP TEST ===\r\n");
#elif APP_MODE == MODE_FLIGHT
  uart_log("=== FC FLIGHT MODE (stub) ===\r\n");
#elif APP_MODE == MODE_SENSOR_LPS22_DEMO

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
#elif APP_MODE == MODE_SENSOR_LSM6DSO32_DEMO

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
#endif
}

void app_loop(void) {
  uint32_t now_ms = HAL_GetTick();
  float t = now_ms / 1000.0f;

  (void)t;

#if APP_MODE == MODE_OPEN_LOOP

  static int step = 0;

  fc_in.time_s = ((float)step) / 1000;
  fc_in.ax = 0.0f;
  fc_in.ay = 0.0f;
  fc_in.az = (float)step;
  fc_in.gx = fc_in.gy = fc_in.gz = 0.0f;
  fc_in.setpoint_vz = 1.0f;

  fc_step(&fc_in, &fc_out, &fc_state, &fc_dbg);

  char buf[200];
  int len = snprintf(buf, sizeof(buf),
                     "%.3f,%.3f,%.3f,%.3f,%.3f,"
                     "%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                     fc_in.time_s, fc_out.motor[0], fc_out.motor[1],
                     fc_out.motor[2], fc_out.motor[3], fc_dbg.dt, fc_dbg.e_vz,
                     fc_dbg.u_p, fc_dbg.u_i, fc_dbg.u_d, fc_dbg.u_raw,
                     fc_dbg.u_sat, fc_dbg.vz_est, fc_dbg.pz_est);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 10);

  step++;

#elif APP_MODE == MODE_FLIGHT
  // Here will be real IMU and flight stuff
#elif APP_MODE == MODE_SENSOR_LSM6DSO32_DEMO
  lsm6dso32_data_ready = true; // for now always poll

  if (lsm6dso32_data_ready) {
    lastResult = LSM6DSO32_ReadAccelRaw(&lsm6dso32, &acceleration);
    lsm6dso32_data_ready = false;

    int16_t x_raw = acceleration.x;
    int16_t y_raw = acceleration.y;
    int16_t z_raw = acceleration.z;

    // ---- Pre-calculation: convert raw LSB -> g (adjust sensitivity to your FS
    // config) ---- Example: for ~16384 LSB/g (typical ±2g config on many IMUs)
    const float acc_sens =
        1.0f / 16384.0f; // g / LSB  (change to match your LSM6DSO32 config)

    float ax = x_raw * acc_sens;
    float ay = y_raw * acc_sens;
    float az = z_raw * acc_sens;

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

    // ---- Final string (similar style to the LPS22 code) ----
    int len = snprintf(buffer, sizeof(buffer),
                       "ax: %s%d.%03d g, ay: %s%d.%03d g, az: %s%d.%03d g\r\n",
                       signAx, intAx, fracAx3, signAy, intAy, fracAy3, signAz,
                       intAz, fracAz3);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);
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
#endif
}
