#include "mode_hover.h"
#include "gyro.h"

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

void hover_controller_initialize() {}

float hover_controller_out(
    gyro_output_channel_function_t channel_function,
    float command
)
{
    if (channel_function != FN_ELEVATOR && channel_function != FN_RUDDER)
        return 0.0;

    float correction = 0.0;
    float error = gyro.ypr[1] - M_PI_2; // Pi/2 = 90degrees

    switch (channel_function)
    {
    case FN_ELEVATOR:
        return error * cos(gyro.ypr[2]);

    case FN_RUDDER:
        return error * sin(gyro.ypr[2]);

    default: ;
    }

    return correction;
}

void hover_controller_calculate_pid()
{
    // TODO: Change this to use PID controllers
}
#endif