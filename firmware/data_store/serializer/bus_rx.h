#pragma once
#include <stdint.h>
#include <stdbool.h>

bool bus_handle_frame(uint8_t msg, uint8_t kind, uint8_t id,
                      const uint8_t *payload, uint8_t len,
                      uint32_t now_us);
