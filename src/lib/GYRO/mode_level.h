#pragma once
#include "mixer.h"

void level_controller_initialize();
void level_controller_calculate_pid();
float level_controller_out(gyro_output_channel_function_t channel_function, uint16_t us);
