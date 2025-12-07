#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "bus_desc.h"

// Generic write callback (transport-agnostic)
typedef void (*bus_write_fn_t)(const uint8_t *data, uint16_t len, void *ctx);

// Serialize one item as a simple frame:
// [AA 55][kind][id][len][payload...]
// Returns false if item not found.
bool bus_serialize_latest(uint8_t id, BusKind kind, bus_write_fn_t write, void *ctx);
