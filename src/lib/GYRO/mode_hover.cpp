#include "mode_hover.h"
#include "gyro.h"
#include "config.h"

#if defined(HAS_GYRO)
/**
 * Airplane Hover Mode
 *
 * For hover mode we care about two angles
 *
 * 1. The "pitch" angle of the horizon.
 *    This is the amount of error.
 *
 * 2. Rotation around the "roll" axis.
 *    This is the modulation between elevator and rudder to correct the error.
 *
 * With these angles we can command elevator and rudder to correct towards a nose
 * directly up attitude.
 */

void hover_controller_initialize() {
    configure_pids(1.0, 1.0, 1.0);
}

void hover_controller_calculate_pid()
{
    pid_roll.calculate(0, gyro.f_gyro[0]);
    pid_pitch.calculate(0, gyro.f_gyro[1]);
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float hover_controller_out(
    gyro_output_channel_function_t channel_function,
    float command
)
{
    float correction = 0.0;
    float error = gyro.ypr[1] - M_PI_2; // Pi/2 = 90degrees
    error *= (float) config.GetGyroHoverStrength() / 16;

    switch (channel_function)
    {
    case FN_AILERON:
        return pid_roll.output;

    case FN_ELEVATOR:
        return pid_pitch.output + (error * cos(gyro.ypr[2]));

    case FN_RUDDER:
        return pid_yaw.output + (error * sin(gyro.ypr[2]));

    default: ;
    }

    return correction;
}
#endif
