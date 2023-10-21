#pragma once
#include "gyro_types.h"

bool mixer_initialize();

float us_command_to_float(uint16_t us);
float crsf_command_to_float(uint32_t us);
float us_command_to_float(uint8_t ch, uint16_t us);
uint16_t float_to_us(uint8_t ch, float value);
void mixer_channel_update(uint8_t ch, uint16_t us);

// Channel configuration for minimum, subtrim and maximum us values
extern uint16_t ch_us[GYRO_MAX_CHANNELS][3];
