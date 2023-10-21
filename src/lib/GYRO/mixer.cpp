#if defined(HAS_GYRO)
#include "config.h"
#include "mixer.h"
#include "gyro.h"
#include "targets.h"
#include "logging.h"

#define GYRO_SUBTRIM_INIT_SAMPLES 3
uint8_t subtrim_init = 0;

// Channel configuration for minimum, subtrim and maximum us values. If no
// values are specified we default to full range and autodetection of midpoint.
uint16_t ch_us[GYRO_MAX_CHANNELS][3] = {
    // {GYRO_US_MIN, GYRO_US_MID, GYRO_US_MAX}
    // {GYRO_US_MIN, 0, GYRO_US_MAX} <- autodetect midpoint
};

#define CH_US_MIN 0
#define CH_US_MID 1
#define CH_US_MAX 2

bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS] = {};

// Last known channel values
uint16_t ch_values[GYRO_MAX_CHANNELS];

bool mixer_initialize()
{
    bool valid = true;
    // Called once during boot to fill arrays and sanity check
    for (int i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        // Note that if we have no channel configuration all values start at
        // zero, in that case apply defaults.
        if (ch_us[i][1] == 0) {
            ch_us[i][1] = GYRO_US_MID;
            ch_map_auto_subtrim[i] = true;
        }
        if (ch_us[i][0] < GYRO_US_MIN)
            ch_us[i][0] = GYRO_US_MIN;
        if (ch_us[i][2] < GYRO_US_MIN || ch_us[i][2] > GYRO_US_MAX)
            ch_us[i][2] = GYRO_US_MAX;

        // Sanity checks, if the fail we will refuse to run the gyro at all
        if (
            ch_us[i][0] < GYRO_US_MIN ||
            ch_us[i][0] > GYRO_US_MAX ||
            ch_us[i][2] < GYRO_US_MIN ||
            ch_us[i][2] > GYRO_US_MAX ||
            ch_us[i][0] > ch_us[i][2] ||
            ch_us[i][0] + 10 > ch_us[i][1] ||
            ch_us[i][2] - 10 < ch_us[i][1]
        ) {
            DBGLN("Gyro configuration invalid, channel %d: %d %d %d",
                i, ch_us[i][0], ch_us[i][1], ch_us[i][2]
            )
            valid = false;
        }
    }

    /*
    DBGLN("MIXER MAP:")
    for (uint8_t i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        DBGLN("Channel %d: %d %d %d", i, ch_us[i][0], ch_us[i][1], ch_us[i][2])
    }
    */

    return valid;
}

void mixer_channel_update(uint8_t ch, uint16_t us)
{
    // We only track channels in our input map
    if (config.GetGyroChannelInputMode(ch) == FN_IN_NONE) return;

    // TODO: Subtrim autodetection
    // Set midpoint (subtrim) from an average of a set of samples
    /*
    if (ch == 0) subtrim_init++;
    if (ch_map_auto_subtrim[ch] && subtrim_init < GYRO_SUBTRIM_INIT_SAMPLES) {
        ch_us[ch][1] = ((ch_us[ch][1] * subtrim_init) + us) / subtrim_init + 1;
        DBGLN("LOOP %d CH %d %d", subtrim_init, ch, ch_us[ch][1])
    }*/

    // Store current commanded value for use in the PID loop
    ch_values[ch] = us;
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
 * Convert +-1.0 float into us
 */
uint16_t float_to_us(float value)
{
    // TODO: this will take into account subtrim and max throws
    if (value < 0)
        return GYRO_US_MID + ((GYRO_US_MID - GYRO_US_MIN) * value);
    return GYRO_US_MID + ((GYRO_US_MAX - GYRO_US_MID) * value);
}

/**
 * Convert +-1.0 float into us for an output channel
 */
uint16_t float_to_us(uint16_t ch, float value)
{
    if (value < 0)
        return ch_us[ch][CH_US_MID] + ((ch_us[ch][CH_US_MID] - ch_us[ch][CH_US_MIN]) * value);
    return ch_us[ch][CH_US_MID] + ((ch_us[ch][CH_US_MAX] - ch_us[ch][CH_US_MID]) * value);
}
#endif
