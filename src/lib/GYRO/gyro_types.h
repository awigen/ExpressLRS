#pragma once
#include <stdint.h>

typedef enum
{
    GYRO_EVENT_NONE,
    GYRO_EVENT_CALIBRATE,
    GYRO_EVENT_SUBTRIMS
} gyro_event_t;

typedef enum
{
    GYRO_MODE_OFF,
    GYRO_MODE_RATE,
    GYRO_MODE_SAFE,
    GYRO_MODE_LEVEL,
    GYRO_MODE_LAUNCH,
    GYRO_MODE_HOVER,
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
    // GYRO_ALIGN_ALIGN_DEFAULT = 0, // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

                            // R, P, Y
    GYRO_ALIGN_CW0_DEG = 0,            // 00,00,00
    GYRO_ALIGN_CW90_DEG = 1,           // 00,00,01
    GYRO_ALIGN_CW180_DEG = 2,          // 00,00,10
    GYRO_ALIGN_CW270_DEG = 3,          // 00,00,11
    GYRO_ALIGN_CW0_DEG_FLIP = 4,       // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    GYRO_ALIGN_CW90_DEG_FLIP = 5,      // 00,10,01
    GYRO_ALIGN_CW180_DEG_FLIP = 6,     // 00,10,10
    GYRO_ALIGN_CW270_DEG_FLIP = 7,     // 00,10,11

    GYRO_ALIGN_ALIGN_CUSTOM = 8,    // arbitrary sensor angles, e.g. for external sensors
} gyro_sensor_align_t;

#define GYRO_N_AXES 3

typedef enum {
    GYRO_AXIS_ROLL,
    GYRO_AXIS_PITCH,
    GYRO_AXIS_YAW
} gyro_axis_t;

typedef enum {
    GYRO_RATE_VARIABLE_P,
    GYRO_RATE_VARIABLE_I,
    GYRO_RATE_VARIABLE_D
} gyro_rate_variable_t;

typedef enum {
    FN_NONE,
    FN_AILERON,
    FN_ELEVATOR,
    FN_RUDDER,
    FN_ELEVON,
    FN_VTAIL
} gyro_output_channel_function_t;

typedef struct {
    uint8_t p;
    uint8_t i;
    uint8_t d;
    uint8_t gain;
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

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} rx_config_gyro_calibration_t;

constexpr uint8_t GYRO_MAX_CHANNELS = 16;
