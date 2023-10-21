#pragma once
#include "mixer.h"

void safe_controller_initialize();
void safe_controller_calculate_pid();
float safe_controller_out(gyro_output_channel_function_t channel_function, float command);
