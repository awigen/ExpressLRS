#if defined(HAS_GYRO)
#include "targets.h"
#include "elrs_eeprom.h"
#include "config.h"
#include "crsf_protocol.h"

#include "mode_level.h"
#include "gyro.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"

/**
 * Airplane Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 *
 */

const float max_angle_roll = 0.5;
const float max_angle_pitch = 0.5;
float pitch_offset = 0.0;

void level_controller_initialize(float pitch)
{
    pitch_offset = pitch;
    configure_pids(1.0, 1.0, 1.0);
}

float channel_command(uint8_t ch)
{
    return us_command_to_float(ch, CRSF_to_US(ChannelData[ch]));
}

void level_controller_calculate_pid()
{
    int8_t channel = config.GetGyroInputChannelNumber(FN_IN_ROLL);
    if (channel != -1) {
        pid_roll.calculate(
            channel_command(channel) * max_angle_roll,
            gyro.gravity.y
        );
    }

    channel = config.GetGyroInputChannelNumber(FN_IN_PITCH);
    if (channel != -1) {
        pid_pitch.calculate(
            constrain(channel_command(channel) - pitch_offset, -1.0, 1.0) * max_angle_pitch,
            -gyro.gravity.x
        );
            // channel_command(channel) * max_angle_pitch,
            // pitch_offset - gyro.gravity.x
    }

    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float level_controller_out(
    gyro_output_channel_function_t channel_function,
    float command
) {
    if (channel_function == FN_AILERON)
        return pid_roll.output;
    else if (channel_function == FN_ELEVATOR)
        return pid_pitch.output;
    else if (channel_function == FN_RUDDER)
        return pid_yaw.output;

    return 0.0;
}
#endif
