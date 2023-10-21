#pragma once
#include "mixer.h"

void hover_controller_initialize();
void hover_controller_calculate_pid();
float hover_controller_out(gyro_output_channel_function_t channel_function, float command);
