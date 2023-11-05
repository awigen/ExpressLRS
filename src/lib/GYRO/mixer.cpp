#include "targets.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "mixer.h"
#include "gyro.h"
#include "logging.h"

#define GYRO_SUBTRIM_INIT_SAMPLES 10
uint8_t subtrim_init = 0;

// Channel configuration for minimum, subtrim and maximum us values. If no
// values are specified we default to full range and autodetection of midpoint.
uint16_t midpoint[GYRO_MAX_CHANNELS] = {};

bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS] = {};
bool auto_subtrim_complete = false;

bool mixer_initialize()
{
    bool valid = true;
    // Called once during boot to fill arrays and sanity check
    for (int i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        // Note that if we have no channel configuration all values start at
        // zero, in that case apply defaults.
        if (midpoint[i] == 0) {
            midpoint[i] = GYRO_US_MID;
            ch_map_auto_subtrim[i] = true;
        }
    }

    return valid;
}

void auto_subtrim(uint8_t ch, uint16_t us)
{
    // Set midpoint (subtrim) from an average of a set of samples
    gyro_input_channel_function_t channel = config.GetGyroChannelInputMode(ch);
    switch (channel)
    {
    case FN_IN_ROLL:
    case FN_IN_PITCH:
    case FN_IN_YAW:
        if (ch_map_auto_subtrim[ch] && subtrim_init < GYRO_SUBTRIM_INIT_SAMPLES) {
            midpoint[ch] = (((midpoint[ch] * subtrim_init) / subtrim_init) + us) / 2;
        }
        break;

    default:
        break;
    }
}

void mixer_channel_update(uint8_t ch, uint16_t us)
{
    if (!auto_subtrim_complete) {
        if (ch == 0 && ++subtrim_init > GYRO_SUBTRIM_INIT_SAMPLES) {
            auto_subtrim_complete = true;
            for (unsigned i = 0; i < GYRO_MAX_CHANNELS; i++) {
                DBGLN("Subtrim channel %d: %d", i, midpoint[i])
            }
        } else {
            auto_subtrim(ch, us);
        }
    }
}

/**
 *  Apply a correction to a servo PWM value
 */
float us_command_to_float(uint16_t us)
{
    // TODO: this will take into account subtrim and max throws
    return us <= GYRO_US_MID
        ? float(us - GYRO_US_MID) / (GYRO_US_MID - GYRO_US_MIN)
        : float(us - GYRO_US_MID) / (GYRO_US_MAX - GYRO_US_MID);
}

/**
 * Convert a channel µs value to a float command
 *
 * This takes into account subtrim and max throws.
 */
float us_command_to_float(uint8_t ch, uint16_t us)
{
    // TODO: inverted matters?
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(ch);
    const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;
    return us <= mid
        ? float(us - mid) / (mid - limits->val.min)
        : float(us - mid) / (limits->val.max - mid);
}

/**
 * Convert +-1.0 float into µs
 */
uint16_t float_to_us(float value)
{
    if (value < 0)
        return GYRO_US_MID + ((GYRO_US_MID - GYRO_US_MIN) * value);
    return GYRO_US_MID + ((GYRO_US_MAX - GYRO_US_MID) * value);
}

/**
 * Convert +-1.0 float into µs for an output channel
 *
 * This takes into account subtrim and max throws.
 */
uint16_t float_to_us(uint8_t ch, float value)
{
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(ch);
    const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;

    // if (value < 0 && !config.GetPwmChannelInverted(ch))
    if (value < 0)
        return mid + ((mid - limits->val.min) * value);
    return mid + ((limits->val.max - mid) * value);
}
#endif
