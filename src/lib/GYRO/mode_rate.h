#pragma once
#include "gyro_types.h"

void rate_controller_initialize();
void rate_controller_calculate_pid();
float rate_controller_out(gyro_output_channel_function_t channel_function, float command);
