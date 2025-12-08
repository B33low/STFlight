#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"   // Change to your MCU family HAL header if needed

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*uart_rx_byte_cb_t)(uint8_t byte, void *user);

/**
 * Minimal UART port wrapper.
 * - TX uses HAL_UART_Transmit (blocking).
 * - RX uses HAL_UART_Receive_IT one byte at a time.
 */
typedef struct {
    UART_HandleTypeDef *huart;

    uart_rx_byte_cb_t   rx_cb;
    void               *rx_user;

    uint8_t             rx_byte;  // internal 1-byte buffer for IT RX
} UartPort;

/** Initialize port wrapper with a HAL UART handle. */
void uart_port_init(UartPort *p, UART_HandleTypeDef *huart);

/** Blocking write. Good enough for debug/telemetry. */
HAL_StatusTypeDef uart_port_write(UartPort *p,
                                  const uint8_t *data,
                                  size_t len,
                                  uint32_t timeout);

/** Convenience single byte write. */
static inline HAL_StatusTypeDef uart_port_write_u8(UartPort *p,
                                                   uint8_t b,
                                                   uint32_t timeout) {
    return uart_port_write(p, &b, 1, timeout);
}

/** Set a callback for each received byte. */
void uart_port_set_rx_callback(UartPort *p,
                               uart_rx_byte_cb_t cb,
                               void *user);

/**
 * Start 1-byte interrupt RX.
 * Call once after init to begin receiving.
 */
HAL_StatusTypeDef uart_port_start_rx_it(UartPort *p);

/**
 * You must call this from your HAL_UART_RxCpltCallback.
 * This allows the wrapper to remain platform-friendly.
 */
void uart_port_on_rx_complete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
