#include "targets.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "mixer.h"
#include "gyro.h"
#include "logging.h"
#include "crsf_protocol.h"

#define GYRO_SUBTRIM_INIT_SAMPLES 50
uint8_t subtrim_init = 0;

// Channel configuration for minimum, subtrim and maximum us values. If no
// values are specified we default to full range and autodetection of midpoint.
uint16_t midpoint[GYRO_MAX_CHANNELS] = {};

bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS] = {};
bool auto_subtrim_complete = false;

unsigned long time_of_first_packet = 0;

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
    if (ch_map_auto_subtrim[ch] && subtrim_init < GYRO_SUBTRIM_INIT_SAMPLES) {
        midpoint[ch] = (midpoint[ch] + us) / 2;
        //midpoint[ch] = (((midpoint[ch] * subtrim_init) / subtrim_init) + us) / 2;
    }
}

void mixer_channel_update(uint8_t ch, uint16_t us)
{
    if (time_of_first_packet == 0) {
        time_of_first_packet = millis();
        return;
    }
    // If we don't wait the subtrims are incorrect... hmm
    if (millis() - time_of_first_packet < 1000)
        return;

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
 * Convert a CRSF value to a float
 */
float crsf_command_to_float(uint16_t command)
{
    return command <= CRSF_CHANNEL_VALUE_MID
        ? float (command - CRSF_CHANNEL_VALUE_MID) / (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN)
        : float (command - CRSF_CHANNEL_VALUE_MID) / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID);
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
 * Convert +-1.0 float into µs for an output channel
 *
 * This takes into account subtrim and max throws.
 */
uint16_t float_to_us(uint8_t ch, float value)
{
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(ch);
    const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;

    return value < 0
        ? mid + ((mid - limits->val.min) * value)
        : mid + ((limits->val.max - mid) * value);
}
#endif
