#pragma once
#include "fc.h"  // depuis firmware/flight_control_core

void fc_hw_init(void);
void fc_hw_fill_input(FcInput* in);
void fc_hw_apply_output(const FcOutput* out);
