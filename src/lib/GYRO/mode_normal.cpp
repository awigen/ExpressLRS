#if defined(HAS_GYRO)
#include "targets.h"
#include "elrs_eeprom.h"
#include "config.h"
#include "mode_rate.h"
#include "gyro.h"
#include "pid.h"
#include "logging.h"
#include "gyro_types.h"

/**
 * Airplane Normal Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases the correction decreases allowing unlimited
 * angular rates.
*/

void normal_controller_initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void normal_controller_calculate_pid()
{
    pid_roll.calculate(0, gyro.f_gyro[0]);
    pid_pitch.calculate(0, gyro.f_gyro[1]);
    pid_yaw.calculate(0, gyro.f_gyro[2]);
}

float normal_controller_out(
    gyro_output_channel_function_t channel_function,
    uint16_t us
) {
    float command = us_command_to_float(us);
    float correction = 0.0;

    switch (channel_function)
    {
    case FN_AILERON:
        correction = pid_roll.output;
        break;

    case FN_ELEVATOR:
        correction = pid_pitch.output;
        break;

    case FN_RUDDER:
        correction = pid_yaw.output;
        break;

    default:
        break;
    }

    correction *= gyro.gain;
    correction *= 1 - fabs(command);

    return command + correction;
}
#endif
