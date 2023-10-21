#include "targets.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "gyro.h"
#include "gyro_types.h"
#include "mixer.h"
#include "device.h"
#include "gyro_mpu6050.h"
#include "mode_level.h"
#include "mode_hover.h"
#include "mode_rate.h"
#include "mode_safe.h"
#include "telemetry.h"
#include "logging.h"

extern Telemetry telemetry;

PID pid_roll  = PID(1, -1, 0.0, 0.0, 0.0);
PID pid_pitch = PID(1, -1, 0.0, 0.0, 0.0);
PID pid_yaw   = PID(1, -1, 0.0, 0.0, 0.0);

#ifdef GYRO_BOOT_JITTER
uint8_t boot_jitter_times = 0;
uint32_t boot_jitter_time = 0;
int8_t boot_jitter_offset = GYRO_BOOT_JITTER_US;

bool boot_jitter(uint16_t *us)
{
    if (boot_jitter_times > GYRO_BOOT_JITTER_TIMES)
        return false;

    if ((millis() - boot_jitter_time) > GYRO_BOOT_JITTER_MS)
    {
        boot_jitter_times++;
        boot_jitter_time = millis();
        boot_jitter_offset *= -1;
    }

    *us = *us + boot_jitter_offset;
    return true;
}
#endif

volatile gyro_event_t gyro_event = GYRO_EVENT_NONE;

void Gyro::calibrate()
{
    dev->calibrate();
    #ifdef GYRO_BOOT_JITTER
    boot_jitter_times = 0;
    boot_jitter_time = 0;
    #endif
}

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

/**
 * Trigger a gyro re-initialization of the current gyro mode
*/
void Gyro::reload()
{
    gyro_mode = GYRO_MODE_OFF;
}

void Gyro::switch_mode(gyro_mode_t mode)
{
    DBGLN("Gyro: Switching mode to %d", mode);
    gyro_mode = mode;
    switch (mode)
    {
    case GYRO_MODE_RATE:
        rate_controller_initialize();
        break;

    case GYRO_MODE_LEVEL:
        level_controller_initialize();
        break;

    case GYRO_MODE_LAUNCH:
        level_controller_initialize(config.GetGyroLaunchAngle());
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

    #ifdef GYRO_BOOT_JITTER
    if (boot_jitter(us))
        return;
    #endif

    return; // We no longer manipulate the us values...

    gyro_output_channel_function_t output_mode = config.GetGyroChannelOutputMode(ch);
    gyro_input_channel_function_t input_mode = config.GetGyroChannelInputMode(ch);

    if (input_mode == FN_IN_NONE && output_mode == FN_NONE)
        return;

    #ifdef GYRO_BOOT_JITTER
    if (boot_jitter(us))
        return;
    #endif

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

    // Normalize the µs value to a +-1.0 keeping in mind subtrim and max throws
    float command = us_command_to_float(ch, *us);
    float correction = 0.0;

    switch (gyro_mode)
    {
    case GYRO_MODE_RATE:
        correction = rate_controller_out(output_mode, command);
        break;

    case GYRO_MODE_LEVEL:
    case GYRO_MODE_LAUNCH:
        correction = level_controller_out(output_mode, command);
        break;

    case GYRO_MODE_SAFE:
        correction = safe_controller_out(output_mode, command);
        break;

    case GYRO_MODE_HOVER:
        correction = hover_controller_out(output_mode, command);
        break;

    default:
        return;
    }

    // Gyro specific channel inversion. We might remove this later if not needed.
    if (config.GetGyroChannelOutputInverted(ch))
        correction *= -1;

    switch (gyro_mode)
    {
    case GYRO_MODE_RATE:
    case GYRO_MODE_HOVER:
        // Limit correction as set from gain input channel
        correction *= gyro.gain;

        // Modulate the correction depending on how much axis stick command
        correction *= 1 - fabs(command);
        break;

    case GYRO_MODE_LEVEL:
    case GYRO_MODE_LAUNCH:
        // In this mode, the correction is the command
        *us = float_to_us(ch, correction);
        return;
    case GYRO_MODE_SAFE:
        // In this mode we do not allow the correction to be limited
    default: ;
    }

    // Limit of min and max µS values is done in devServoOutput
    *us = float_to_us(ch, command + correction);
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
    sprintf(str, "Setpoint: %5.2f PV: %5.2f I:%5.2f D:%5.2f Error: %5.2f Out: %5.2f",
        pid->setpoint, pid->pv, pid->Iout, pid->Dout, pid->error, pid->output);

}
#endif

float Gyro::apply_gain(float correction, mix_destination_t destination)
{
    float command = 0.0;
    for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
    {
        const rx_config_mix_t *mix = config.GetMix(mix_number);
        if (!mix->val.active || mix->val.destination != destination)
            continue;

        command = CRSF_to_FLOAT((uint16_t) ChannelMixedData[mix->val.source]);
        break;
    }

    switch (gyro_mode)
    {
    case GYRO_MODE_RATE:
    case GYRO_MODE_HOVER:
        // Limit correction as set from gain input channel
        correction *= gyro.gain;

        // Modulate the correction depending on how much axis stick command
        // command = CRSF_to_FLOAT((uint16_t) (ChannelMixedData[channel]);
        correction *= 1 - fabs(command);
        break;

    case GYRO_MODE_LEVEL:
    case GYRO_MODE_LAUNCH:
        // In this mode, the correction is the command
        // TODO FIXME
        // *us = float_to_us(ch, correction);
        return correction;
        // return;
    case GYRO_MODE_SAFE:
        // In this mode we do not allow the correction to be limited
    default: ;
    }
    return correction;
}

void Gyro::tick()
{
    if (!initialized) return;

    if (boot_jitter_times < GYRO_BOOT_JITTER_TIMES) return;

    if ((micros() - pid_delay) < 1000 ) return; // ~1k PID loop
    pid_delay = micros();

    detect_mode(CRSF_to_US(ChannelMixedData[MIX_DESTINATION_GYRO_MODE]));
    // detect_gain(CRSF_to_US(ChannelMixedData[MIX_DESTINATION_GYRO_GAIN]));
    // gain = (float(us - GYRO_US_MIN) / (GYRO_US_MAX - GYRO_US_MIN)) * 500;
    // convert the -1 to +1 float to 0 - 1.0
    gain = (1 + CRSF_to_FLOAT((uint16_t) ChannelMixedData[MIX_DESTINATION_GYRO_GAIN])) / 2;

    switch (gyro_mode)
    {
    case GYRO_MODE_RATE:
        rate_controller_calculate_pid();
        break;

    case GYRO_MODE_LEVEL:
    case GYRO_MODE_LAUNCH:
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

    ChannelData[MIX_SOURCE_GYRO_ROLL] =
        constrain(
            CRSF_CHANNEL_VALUE_MID + apply_gain(pid_roll.output, MIX_DESTINATION_GYRO_ROLL) * (CRSF_CHANNEL_VALUE_MID),
            CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX
        );
    ChannelData[MIX_SOURCE_GYRO_PITCH] =
        constrain(
            CRSF_CHANNEL_VALUE_MID + apply_gain(pid_pitch.output, MIX_DESTINATION_GYRO_PITCH) * (CRSF_CHANNEL_VALUE_MID),
            CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX
        );
    ChannelData[MIX_SOURCE_GYRO_YAW] =
        constrain(
            CRSF_CHANNEL_VALUE_MID + apply_gain(pid_yaw.output, MIX_DESTINATION_GYRO_YAW) * (CRSF_CHANNEL_VALUE_MID),
            CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX
        );

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
        DBGLN("GAIN %f", gain)
        gyro_debug_time = micros();
    }
    #endif
}

void configure_pids(float roll_limit, float pitch_limit, float yaw_limit)
{
    const rx_config_gyro_gains_t *roll_gains =
        config.GetGyroGains(GYRO_AXIS_ROLL);
    const rx_config_gyro_gains_t *pitch_gains =
        config.GetGyroGains(GYRO_AXIS_PITCH);
    const rx_config_gyro_gains_t *yaw_gains =
        config.GetGyroGains(GYRO_AXIS_YAW);

    configure_pid_gains(&pid_roll, roll_gains, roll_limit, -1.0 * roll_limit);
    configure_pid_gains(&pid_pitch, pitch_gains, pitch_limit,
                        -1.0 * pitch_limit);
    configure_pid_gains(&pid_yaw, yaw_gains, yaw_limit, -1.0 * yaw_limit);
}

void configure_pid_gains(PID *pid, const rx_config_gyro_gains_t *gains,
                         float max, float min)
{
    DBGLN("Config gains: P %d I %d D %d G %d", gains->p, gains->i, gains->d, gains->gain)
    if (max == 0.0 && min == 0.0) {
        // not gyro correction on this axis
        pid->configure(0.0, 0.0, 0.0, 0.0, 0.0);
    } else {
        float p = gains->gain * gains->p / 1000.0;
        float i = gains->gain * gains->i / 1000.0;
        float d = gains->gain * gains->d / 1000.0;
        DBGLN("PID gains: P %f I %f D %f", p, i, d)

        pid->configure(p, i, d, max, min);
    }
    pid->reset();
}

#endif
