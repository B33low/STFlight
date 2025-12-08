#pragma once
#include <stdint.h>

typedef struct {
    float alpha;          // 0.95..0.99 typical
} AttFilterParams;

extern AttFilterParams g_att_filter_storage;
