#pragma once
#include "mixer.h"

void normal_controller_initialize();
void normal_controller_calculate_pid();
float normal_controller_out(gyro_output_channel_function_t channel_function, uint16_t us);
