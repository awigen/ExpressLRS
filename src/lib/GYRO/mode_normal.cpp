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

// PID controller values
const float kP_roll = 0.01;  // Proportional gain
const float kI_roll = 0.00;  // Integral gain
const float kD_roll = 0.00;  // Derivative gain

const float kP_pitch = 0.01; // Proportional gain
const float kI_pitch = 0.00; // Integral gain
const float kD_pitch = 0.00; // Derivative gain

const float kP_yaw = 0.01;   // Proportional gain
const float kI_yaw = 0.00;   // Integral gain
const float kD_yaw = 0.00;   // Derivative gain

void normal_controller_initialize()
{
    pid_roll.configure(kP_roll, kI_roll, kD_roll, 1.0, -1.0);
    pid_roll.reset();
    pid_pitch.configure(kP_pitch, kI_pitch, kD_pitch, 1.0, -1.0);
    pid_pitch.reset();
    pid_yaw.configure(kP_yaw, kI_yaw, kD_yaw, 1.0, -1.0);
    pid_yaw.reset();
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
