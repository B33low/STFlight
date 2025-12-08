#include "uart_port.h"

// Minimal support for a small number of ports.
// Increase if needed.
#ifndef UART_PORT_MAX
#define UART_PORT_MAX 4
#endif

static UartPort *s_ports[UART_PORT_MAX];
static uint8_t s_port_count = 0;

static UartPort* find_port(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < s_port_count; i++) {
        if (s_ports[i] && s_ports[i]->huart == huart) {
            return s_ports[i];
        }
    }
    return NULL;
}

void uart_port_init(UartPort *p, UART_HandleTypeDef *huart) {
    p->huart = huart;
    p->rx_cb = NULL;
    p->rx_user = NULL;
    p->rx_byte = 0;

    // Register port
    if (s_port_count < UART_PORT_MAX) {
        s_ports[s_port_count++] = p;
    }
}

HAL_StatusTypeDef uart_port_write(UartPort *p,
                                  const uint8_t *data,
                                  size_t len,
                                  uint32_t timeout) {
    if (!p || !p->huart || !data || len == 0) return HAL_ERROR;
    return HAL_UART_Transmit(p->huart, (uint8_t*)data, (uint16_t)len, timeout);
}

void uart_port_set_rx_callback(UartPort *p,
                               uart_rx_byte_cb_t cb,
                               void *user) {
    if (!p) return;
    p->rx_cb = cb;
    p->rx_user = user;
}

HAL_StatusTypeDef uart_port_start_rx_it(UartPort *p) {
    if (!p || !p->huart) return HAL_ERROR;
    return HAL_UART_Receive_IT(p->huart, &p->rx_byte, 1);
}

void uart_port_on_rx_complete(UART_HandleTypeDef *huart) {
    UartPort *p = find_port(huart);
    if (!p) return;

    if (p->rx_cb) {
        p->rx_cb(p->rx_byte, p->rx_user);
    }

    // re-arm reception
    (void)HAL_UART_Receive_IT(p->huart, &p->rx_byte, 1);
}
