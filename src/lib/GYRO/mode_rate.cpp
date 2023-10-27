#if defined(HAS_GYRO)
#include "targets.h"
#include "elrs_eeprom.h"
#include "config.h"
#include "mode_rate.h"
#include "gyro.h"
#include "pid.h"
#include "logging.h"
#include "gyro_types.h"

// PID controller values
const float maxRate_ail = 90.0;  // Max roll rate in deg/s
const float maxRate_ele = 90.0;  // Max pitch rate in deg/s
const float maxRate_yaw = 90.0;  // Max pitch rate in deg/s

void rate_controller_initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void rate_controller_calculate_pid()
{
    int8_t channel = config.GetGyroInputChannelNumber(FN_IN_ROLL);
    if (channel > -1)
        pid_roll.calculate(
            us_command_to_float(ch_values[channel]) * maxRate_ail,
            gyro.f_gyro[0]
        );

    channel = config.GetGyroInputChannelNumber(FN_IN_PITCH);
    if (channel > -1)
        pid_pitch.calculate(
            us_command_to_float(ch_values[channel]) * maxRate_ele,
            gyro.f_gyro[1]
        );

    channel = config.GetGyroInputChannelNumber(FN_IN_YAW);
    if (channel > -1)
        pid_yaw.calculate(
            us_command_to_float(ch_values[channel]) * maxRate_yaw,
            gyro.f_gyro[2]
        );
}

float rate_controller_out(
    gyro_output_channel_function_t channel_function,
    uint16_t us
) {

    if (channel_function == FN_AILERON) {
        return pid_roll.output;
    }
    else if (channel_function == FN_ELEVATOR) {
        return pid_pitch.output;
    }
    else if (channel_function == FN_RUDDER) {
        return pid_yaw.output;
    }
    return us_command_to_float(us);
}
#endif
