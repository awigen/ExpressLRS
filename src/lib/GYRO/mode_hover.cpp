#if defined(HAS_GYRO)
#include "targets.h"
#include "mode_hover.h"
#include "device.h"
#include "gyro.h"
#include "mixer.h"

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
    uint16_t us
)
{
    float command = us_command_to_float(us);
    float correction = 0.0;
    float error = 0.0;

    if (channel_function == FN_ELEVATOR || channel_function == FN_RUDDER)
    {
        error = gyro.ypr[1] - 1.5708;
    } else {
        return command;
    }

    switch (channel_function)
    {
    case FN_ELEVATOR:
        correction = error * cos(gyro.ypr[2]);
        break;

    case FN_RUDDER:
        correction = error * sin(gyro.ypr[2]);
        break;

    default:
        break;
    }

    correction *= gyro.gain;

    return command + correction;
}

void hover_controller_calculate_pid()
{
    // TODO: Change this to use PID controllers
}
#endif
