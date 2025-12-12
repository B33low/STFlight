#pragma once
#include <stdint.h>
#include "bus_desc.h"

void bus_registry_set(const BusItem *items, uint32_t count);

const BusItem* bus_find(uint8_t id, BusKind kind);
uint16_t bus_item_size(const BusItem *it);
uint32_t bus_registry_count(void);
