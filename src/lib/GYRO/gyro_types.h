#if defined(HAS_GYRO)
#pragma once
#include <stdint.h>

typedef enum
{
    GYRO_MODE_OFF,
    GYRO_MODE_NORMAL,
    GYRO_MODE_SAFE,
    GYRO_MODE_HOVER,
    GYRO_MODE_RATE,
    GYRO_MODE_LEVEL,
} gyro_mode_t;

typedef enum {
    FN_IN_NONE,
    FN_IN_ROLL,
    FN_IN_PITCH,
    FN_IN_YAW,
    FN_IN_GYRO_MODE,
    FN_IN_GYRO_GAIN
} gyro_input_channel_function_t;

typedef enum {
    FN_NONE,
    FN_AILERON,
    FN_ELEVATOR,
    FN_RUDDER,
    FN_ELEVON,
    FN_VTAIL
} gyro_output_channel_function_t;

typedef struct {
    float p;
    float i;
    float d;
    float gain;
} rx_config_gyro_gains_t;

typedef struct {
    uint16_t min;
    uint16_t mid;
    uint16_t max;
} rx_config_gyro_timings_t;

typedef union {
    struct {
        uint32_t input_mode:5,
                 output_mode:5,
                 inverted:1,     // invert gyro output
                 auto_subtrim:1, // Set subtrim at first connection
                 unused:20;
    } val;
    uint32_t raw;
} rx_config_gyro_channel_t;

typedef union {
    struct {
        uint32_t pos1: 4,
                 pos2: 4,
                 pos3: 4,
                 pos4: 4,
                 pos5: 4,
                 unused: 12;
    } val;
    uint32_t raw;
} rx_config_gyro_mode_pos_t;

constexpr uint8_t GYRO_MAX_CHANNELS = 16;
#endif
