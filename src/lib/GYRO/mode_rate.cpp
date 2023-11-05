#include "gyro.h"

#if defined(HAS_GYRO)
#include "mode_rate.h"

/**
 * Airplane Normal Mode / Rate Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases the correction decreases allowing unlimited
 * angular rates.
 */

void rate_controller_initialize()
{
    configure_pids(1.0, 1.0, 1.0);

    // For rate mode we have a basic derivative from the gyro which is the
    // angular velocity. Therefor we turn of any derivitive term.
    // pid_pitch._Kd = 0;
    // pid_roll._Kd = 0;
    // pid_yaw._Kd = 0;
}

void rate_controller_calculate_pid()
{
    // Desired angular rate is zero
    pid_roll.calculate(0, gyro.f_gyro[0]);
    pid_pitch.calculate(0, gyro.f_gyro[1]);
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float rate_controller_out(
    gyro_output_channel_function_t channel_function,
    float command
) {
    switch (channel_function)
    {
    case FN_AILERON:
        return pid_roll.output;

    case FN_ELEVATOR:
        return pid_pitch.output;

    case FN_RUDDER:
        return pid_yaw.output;

    default: ;
    }
    return 0.0;
}
#endif
