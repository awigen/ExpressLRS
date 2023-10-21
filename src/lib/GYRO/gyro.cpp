#if defined(HAS_GYRO)
#include "targets.h"
#include "elrs_eeprom.h"
#include "config.h"
#include "gyro.h"
#include "gyro_types.h"
#include "mixer.h"
#include "device.h"
#include "gyro_mpu6050.h"
#include "mode_normal.h"
#include "mode_level.h"
#include "mode_hover.h"
#include "mode_rate.h"
#include "mode_safe.h"
#include "mixer.h"
#include "telemetry.h"
#include "logging.h"

extern Telemetry telemetry;

PID pid_roll  = PID(1, -1, 0.0, 0.0, 0.0);
PID pid_pitch = PID(1, -1, 0.0, 0.0, 0.0);
PID pid_yaw   = PID(1, -1, 0.0, 0.0, 0.0);

void Gyro::detect_mode(uint16_t us)
{
    const rx_config_gyro_mode_pos_t *modes = config.GetGyroModePos();
    const uint16_t width = (GYRO_US_MAX - GYRO_US_MIN) / 5;
    uint8_t channel_position = (us - GYRO_US_MIN) / width;
    channel_position = channel_position > 4 ? 4 : channel_position;
    gyro_mode_t selected_mode;
    switch (channel_position)
    {
        case 0: selected_mode = (gyro_mode_t) modes->val.pos1; break;
        case 1: selected_mode = (gyro_mode_t) modes->val.pos2; break;
        case 2: selected_mode = (gyro_mode_t) modes->val.pos3; break;
        case 3: selected_mode = (gyro_mode_t) modes->val.pos4; break;
        case 4: selected_mode = (gyro_mode_t) modes->val.pos5; break;
        default: selected_mode = GYRO_MODE_OFF; break;
    }
    if (gyro_mode != selected_mode)
        switch_mode(selected_mode);
}

void Gyro::switch_mode(gyro_mode_t mode)
{
    DBGLN("Gyro: Switching mode to %d", mode);
    gyro_mode = mode;
    switch (mode)
    {
    case GYRO_MODE_NORMAL:
        normal_controller_initialize();
        break;

    case GYRO_MODE_RATE:
        rate_controller_initialize();
        break;

    case GYRO_MODE_LEVEL:
        level_controller_initialize();
        break;

    case GYRO_MODE_SAFE:
        safe_controller_initialize();
        break;

    case GYRO_MODE_HOVER:
        hover_controller_initialize();
        break;

    default:
        break;
    }
}

void Gyro::detect_gain(uint16_t us)
{
    gain = (float(us - GYRO_US_MIN) / (GYRO_US_MAX - GYRO_US_MIN)) * 500;
}

/**
 * Apply gyro servo output mixing and detect gyro mode
 */
void Gyro::mixer(uint8_t ch, uint16_t *us)
{
    // We get called before the gyro configuration is initialized
    if (!initialized) return;

    mixer_channel_update(ch, *us);

    gyro_output_channel_function_t output_mode = config.GetGyroChannelOutputMode(ch);
    gyro_input_channel_function_t input_mode = config.GetGyroChannelInputMode(ch);

    if (input_mode == FN_IN_NONE && output_mode == FN_NONE)
        return;

    switch (input_mode)
    {
    case FN_IN_GYRO_MODE:
        detect_mode(*us);
        return;
    case FN_IN_GYRO_GAIN:
        gain = (us_command_to_float(*us) + 1) / 2;
        return;
    default:
        break;
    }

    if (output_mode == FN_NONE)
        return;

    uint16_t new_us = *us;
    float correction = 0.0;
    switch (gyro_mode)
    {
    case GYRO_MODE_NORMAL:
        correction = normal_controller_out(output_mode, *us);
        break;

    case GYRO_MODE_RATE:
        correction = rate_controller_out(output_mode, *us);
        break;

    case GYRO_MODE_LEVEL:
        correction = level_controller_out(output_mode, *us);
        break;

    case GYRO_MODE_SAFE:
        correction = safe_controller_out(output_mode, *us);
        break;

    case GYRO_MODE_HOVER:
        correction = hover_controller_out(output_mode, *us);
        break;

    default:
        return;
    }

    if (correction == 0.0)
        return;

    // If the channel is inverted, also invert the correction
    if (config.GetPwmChannelInverted(ch))
        correction *= -1;

    // Gyro specific channel inversion. We might remove this later if not needed.
    if (config.GetGyroChannelOutputInverted(ch))
        correction *= -1;

    // Gyro gain modulation
    correction *= gain;

    // float command = us_command_to_float(*us);

    new_us = float_to_us(ch, correction);

    if (abs(*us - new_us) <= GYRO_DEADBAND)
        return;

    // Limit min and max µS values is done in servo outputs now
    *us = new_us;
}

static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
    while (angle_decidegree > 1800)
    {
        angle_decidegree -= 3600;
    }
    while (angle_decidegree < -1800)
    {
        angle_decidegree += 3600;
    }
    return (int16_t)((M_PI / 180.0f) * 1000.0f * angle_decidegree);
}

void Gyro::send_telemetry()
{
    // Get yaw/pitch/roll in decidegrees and convert to uint16_t
    uint16_t ypr16[3] = {0};
    ypr16[0] = (uint16_t)(gyro.ypr[0] * 1800 / M_PI);
    ypr16[1] = (uint16_t)(gyro.ypr[1] * 1800 / M_PI);
    ypr16[2] = (uint16_t)(gyro.ypr[2] * 1800 / M_PI);

    CRSF_MK_FRAME_T(crsf_sensor_attitude_t)
    crsfAttitude = {0};
    crsfAttitude.p.pitch = htobe16(decidegrees2Radians10000(ypr16[1]));
    crsfAttitude.p.roll = htobe16(decidegrees2Radians10000(ypr16[2]));
    crsfAttitude.p.yaw = htobe16(decidegrees2Radians10000(ypr16[0]));

    CRSF::SetHeaderAndCrc((uint8_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
    telemetry.AppendTelemetryPackage((uint8_t *)&crsfAttitude);
}

bool Gyro::read_device()
{
    if (dev->read())
    {
        // Calculate gyro update rate in HZ
        update_rate = 1.0 /
                      ((micros() - last_update) / 1000000.0);

        last_update = micros();
        send_telemetry();
    }
    return DURATION_IMMEDIATELY;
}

// #define GYRO_PID_DEBUG_TIME 100
unsigned long gyro_debug_time = 0;

#ifdef GYRO_PID_DEBUG_TIME
void _make_gyro_debug_string(PID *pid, char *str) {
    sprintf(str, "Setpoint: %5.2f PV: %5.2f I:%5.2f Error: %5.2f Out: %5.2f",
        pid->setpoint, pid->pv, pid->Iout, pid->error, pid->output);

}
#endif

void Gyro::tick()
{
    if ((micros() - pid_delay) < 1000 ) return; // ~1k PID loop
    pid_delay = micros();

    switch (gyro_mode)
    {
    case GYRO_MODE_NORMAL:
        normal_controller_calculate_pid();
        break;

    case GYRO_MODE_RATE:
        rate_controller_calculate_pid();
        break;

    case GYRO_MODE_LEVEL:
        level_controller_calculate_pid();
        break;

    case GYRO_MODE_SAFE:
        safe_controller_calculate_pid();
        break;

    case GYRO_MODE_HOVER:
        hover_controller_calculate_pid();
        break;

    default:
        break;
    }

    #ifdef GYRO_PID_DEBUG_TIME
    if (gyro_mode != GYRO_MODE_OFF &&
        micros() - gyro_debug_time > GYRO_PID_DEBUG_TIME * 1000
    ) {
        char piddebug[128];
        _make_gyro_debug_string(&pid_pitch, piddebug);
        DBGLN("\nPID Pitch %s", piddebug)
        _make_gyro_debug_string(&pid_roll, piddebug);
        DBGLN("PID Roll  %s", piddebug)
        _make_gyro_debug_string(&pid_yaw, piddebug);
        DBGLN("PID Yaw   %s", piddebug)
        gyro_debug_time = micros();
    }
    #endif
}

#endif