#include "gyro.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "mode_safe.h"
#include "mixer.h"
#include "pid.h"
#include "gyro_types.h"

/**
 * Airplane Safe Mode
 *
 * This allows normal flying, but tries to stop the plane going past set angles.
*/

#define PI_180 0.0174532925199

const float max_angle_roll = 30 * PI_180; // Convert degrees to radians
const float max_angle_pitch = 30 * PI_180; // Convert degrees to radians

void safe_controller_initialize()
{
    // Set limits to two to be able to fully override a full stick input command
    // on roll and pitch axes
    configure_pids(2.0, 2.0, 1.0);
}

void _calculate_pid(PID *pid, float angle, float max_angle)
{
    if (abs(angle) < max_angle) {
        pid->reset();
    } else {
        float setpoint = angle > 0 ? max_angle : - max_angle;
        pid->calculate(setpoint, angle);
    }
}

void safe_controller_calculate_pid()
{
    _calculate_pid(&pid_pitch, -gyro.ypr[1], config.GetGyroSAFEPitch() * PI_180);
    _calculate_pid(&pid_roll, gyro.ypr[2], config.GetGyroSAFERoll() * PI_180);

    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

float safe_controller_out(
    gyro_output_channel_function_t channel_function,
    float command
) {
    // TODO: Invert elevator if craft is inverted
    // TODO: Modulate elevator and rudder when needed (knife edge nose down)
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
