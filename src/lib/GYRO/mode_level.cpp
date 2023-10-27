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
const float max_angle_pitch= 0.5;

void level_controller_initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void level_controller_calculate_pid()
{
    for (uint8_t i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        const rx_config_gyro_channel_t *channel = config.GetGyroChannel(i);
        gyro_input_channel_function_t input_mode = (gyro_input_channel_function_t) channel->val.input_mode;
        if (input_mode == FN_IN_ROLL) {
            pid_roll.calculate(
                // FIXME: ChannelData[config.GetPwmChannel(ch)->val.inputChannel
                // newPwmCh.val.failsafe = CRSF_to_UINT10(
                // constrain(ChannelData[config.GetPwmChannel(ch)->val.inputChannel],
                //           CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX));
                // new function:
                // CRSF_to_FLOAT(ChannelData[i])
                us_command_to_float(CRSF_to_UINT10(ChannelData[i])) * max_angle_roll,
                // us_command_to_float(ch_values[i]) * max_angle_roll,
                gyro.gravity.y / 90
            );
            break;
        }
    }

    for (uint8_t i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        const rx_config_gyro_channel_t *channel = config.GetGyroChannel(i);
        gyro_input_channel_function_t input_mode = (gyro_input_channel_function_t) channel->val.input_mode;
        if (input_mode == FN_IN_PITCH) {
            pid_pitch.calculate(
                us_command_to_float(ch_values[i]) * max_angle_pitch,
                gyro.gravity.x / 90
            );
            break;
        }
    }
}

float level_controller_out(
    gyro_output_channel_function_t channel_function,
    uint16_t us
) {
    float command = us_command_to_float(us);
    float correction = 0.0;

    if (channel_function == FN_AILERON)
        correction = pid_roll.output;
    else if (channel_function == FN_ELEVATOR)
        correction = pid_pitch.output;

    correction *= gyro.gain;

    return command + correction;
}
#endif
