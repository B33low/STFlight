#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "state_base.h"
#include "stream_base.h"
#include "param_base.h"

typedef enum {
    BUS_KIND_STATE = 1,
    BUS_KIND_PARAM = 2,
    BUS_KIND_STREAM = 3
} BusKind;

typedef struct {
    uint8_t  id;
    BusKind  kind;
    void    *ptr;         // StateAny*, ParamAny*, StreamAny*
    const char *name;
} BusItem;

typedef enum {
    BUS_MSG_PUBLISH = 1,
    BUS_MSG_WRITE   = 2,
    BUS_MSG_INJECT  = 3,
    BUS_MSG_READREQ = 4,
} BusMsgType;

const BusItem* bus_find(uint8_t id, BusKind kind);
uint16_t bus_item_size(const BusItem *it);