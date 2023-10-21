#include "gyro.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "crsf_protocol.h"

#include "mode_level.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 *
 */

float pitch_offset = 0.0;

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)

void level_controller_initialize(float offset)
{
    pitch_offset = offset;
    configure_pids(1.0, 1.0, 1.0);
}

/*
float channel_command(uint8_t ch)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
    uint16_t us = CRSF_to_US(crsfVal);
    return us_command_to_float(ch, us);
}
*/
extern uint16_t midpoint[GYRO_MAX_CHANNELS];
extern bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS];

/**
 * Get a -1 to +1 float for the gyro input command
*/
float get_command(mix_destination_t type)
{
    for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
    {
        const rx_config_mix_t *mix = config.GetMix(mix_number);
        if (!mix->val.active || mix->val.destination != type)
            continue;

        uint8_t ch = mix->val.source;
        const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;
        uint16_t val_us = CRSF_to_US(ChannelData[mix->val.source]) - (mid - GYRO_US_MID);
        return us_command_to_float(val_us);
        //return crsf_command_to_float(ChannelMixedData[mix->val.source]);
    }
    return 0;
}

void level_controller_calculate_pid()
{
    pid_roll.calculate(
        get_command(MIX_DESTINATION_GYRO_ROLL) * degToRad(config.GetGyroLevelRoll()),
        gyro.ypr[2]
    );

    pid_pitch.calculate(
        get_command(MIX_DESTINATION_GYRO_PITCH), // * degToRad(config.GetGyroLevelPitch()),
        // For the pitch access in launch mode (pitch_offset != 0)
        // we change what the PID controllers sees as level
        degToRad(pitch_offset) - gyro.ypr[1]
    );
    
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
    /*
    int8_t channel = config.GetGyroInputChannelNumber(FN_IN_ROLL);
    if (channel != -1) {
        pid_roll.calculate(
            channel_command(channel) * degToRad(config.GetGyroLevelRoll()),
            gyro.ypr[2]
        );
    }

    channel = config.GetGyroInputChannelNumber(FN_IN_PITCH);
    if (channel != -1) {
        pid_pitch.calculate(
                channel_command(channel) * degToRad(config.GetGyroLevelPitch()),
                // For the pitch access in launch mode (pitch_offset != 0)
                // we change what the PID controllers sees as level
                degToRad(pitch_offset) - gyro.ypr[1]
        );
    }

    pid_yaw.calculate(0, -gyro.f_gyro[2]);
    */
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
        // Because the calling gyro fuction will not add command to the output
        // of the level controller, we have to add it ourselves here.
        return pid_yaw.output + command;

    return 0.0;
}
#endif
