#pragma once

#include "targets.h"
#include "elrs_eeprom.h"
#include "options.h"
#include "common.h"
#include "gyro_types.h"

#if defined(PLATFORM_ESP32)
#include <nvs_flash.h>
#include <nvs.h>
#endif

// CONFIG_MAGIC is ORed with CONFIG_VERSION in the version field
#define CONFIG_MAGIC_MASK   (0b11U << 30)
#define TX_CONFIG_MAGIC     (0b01U << 30)
#define RX_CONFIG_MAGIC     (0b10U << 30)

#define TX_CONFIG_VERSION   7U
#if defined(HAS_GYRO)
// This will force a wipe of the EEPROM for our new config structure
#define RX_CONFIG_VERSION   10U
#else
#define RX_CONFIG_VERSION   9U
#endif
#define UID_LEN             6

#if defined(TARGET_TX)
typedef enum {
    HT_OFF,
    HT_ON,
    HT_AUX1_UP,
    HT_AUX1_DN,
    HT_AUX2_UP,
    HT_AUX2_DN,
    HT_AUX3_UP,
    HT_AUX3_DN,
    HT_AUX4_UP,
    HT_AUX4_DN,
    HT_AUX5_UP,
    HT_AUX5_DN,
    HT_AUX6_UP,
    HT_AUX6_DN,
    HT_AUX7_UP,
    HT_AUX7_DN,
    HT_AUX8_UP,
    HT_AUX8_DN,
} headTrackingEnable_t;

typedef struct {
    uint32_t    rate:4,
                tlm:4,
                power:3,
                switchMode:2,
                boostChannel:3, // dynamic power boost AUX channel
                dynamicPower:1,
                modelMatch:1,
                txAntenna:2,    // FUTURE: Which TX antenna to use, 0=Auto
                ptrStartChannel:4,
                ptrEnableChannel:5,
                _unused:3;
} model_config_t;

typedef struct {
    uint8_t     pressType:1,    // 0 short, 1 long
                count:3,        // 1-8 click count for short, .5sec hold count for long
                action:4;       // action to execute
} button_action_t;

typedef union {
    struct {
        uint8_t color;                  // RRRGGGBB
        button_action_t actions[2];
        uint8_t unused;
    } val;
    uint32_t raw;
} tx_button_color_t;

typedef struct {
    uint32_t        version;
    uint8_t         vtxBand;    // 0=Off, else band number
    uint8_t         vtxChannel; // 0=Ch1 -> 7=Ch8
    uint8_t         vtxPower;   // 0=Do not set, else power number
    uint8_t         vtxPitmode; // Off/On/AUX1^/AUX1v/etc
    uint8_t         powerFanThreshold:4; // Power level to enable fan if present
    model_config_t  model_config[64];
    uint8_t         fanMode;            // some value used by thermal?
    uint8_t         motionMode:2,       // bool, but space for 2 more modes
                    dvrStopDelay:3,
                    backpackDisable:1,  // bool, disable backpack via EN pin if available
                    backpackTlmEnabled:1,  // bool, enable telemetry via ESPNOW from backpack
                    unused:1;          // FUTURE available
    uint8_t         dvrStartDelay:3,
                    dvrAux:5;
    tx_button_color_t buttonColors[2];  // FUTURE: TX RGB color / mode (sets color of TX, can be a static color or standard)
                                        // FUTURE: Model RGB color / mode (sets LED color mode on the model, but can be second TX led color too)
                                        // FUTURE: Custom button actions
} tx_config_t;

class TxConfig
{
public:
    TxConfig();
    void Load();
    void Commit();

    // Getters
    uint8_t GetRate() const { return m_model->rate; }
    uint8_t GetTlm() const { return m_model->tlm; }
    uint8_t GetPower() const { return m_model->power; }
    bool GetDynamicPower() const { return m_model->dynamicPower; }
    uint8_t GetBoostChannel() const { return m_model->boostChannel; }
    uint8_t GetSwitchMode() const { return m_model->switchMode; }
    uint8_t GetAntennaMode() const { return m_model->txAntenna; }
    bool GetModelMatch() const { return m_model->modelMatch; }
    bool     IsModified() const { return m_modified; }
    uint8_t  GetVtxBand() const { return m_config.vtxBand; }
    uint8_t  GetVtxChannel() const { return m_config.vtxChannel; }
    uint8_t  GetVtxPower() const { return m_config.vtxPower; }
    uint8_t  GetVtxPitmode() const { return m_config.vtxPitmode; }
    uint8_t GetPowerFanThreshold() const { return m_config.powerFanThreshold; }
    uint8_t  GetFanMode() const { return m_config.fanMode; }
    uint8_t  GetMotionMode() const { return m_config.motionMode; }
    uint8_t  GetDvrAux() const { return m_config.dvrAux; }
    uint8_t  GetDvrStartDelay() const { return m_config.dvrStartDelay; }
    uint8_t  GetDvrStopDelay() const { return m_config.dvrStopDelay; }
    bool     GetBackpackDisable() const { return m_config.backpackDisable; }
    bool     GetBackpackTlmEnabled() const { return m_config.backpackTlmEnabled; }
    tx_button_color_t const *GetButtonActions(uint8_t button) const { return &m_config.buttonColors[button]; }
    model_config_t const &GetModelConfig(uint8_t model) const { return m_config.model_config[model]; }
    uint8_t GetPTRStartChannel() const { return m_model->ptrStartChannel; }
    uint8_t GetPTREnableChannel() const { return m_model->ptrEnableChannel; }

    // Setters
    void SetRate(uint8_t rate);
    void SetTlm(uint8_t tlm);
    void SetPower(uint8_t power);
    void SetDynamicPower(bool dynamicPower);
    void SetBoostChannel(uint8_t boostChannel);
    void SetSwitchMode(uint8_t switchMode);
    void SetAntennaMode(uint8_t txAntenna);
    void SetModelMatch(bool modelMatch);
    void SetDefaults(bool commit);
    void SetStorageProvider(ELRS_EEPROM *eeprom);
    void SetVtxBand(uint8_t vtxBand);
    void SetVtxChannel(uint8_t vtxChannel);
    void SetVtxPower(uint8_t vtxPower);
    void SetVtxPitmode(uint8_t vtxPitmode);
    void SetPowerFanThreshold(uint8_t powerFanThreshold);
    void SetFanMode(uint8_t fanMode);
    void SetMotionMode(uint8_t motionMode);
    void SetDvrAux(uint8_t dvrAux);
    void SetDvrStartDelay(uint8_t dvrStartDelay);
    void SetDvrStopDelay(uint8_t dvrStopDelay);
    void SetButtonActions(uint8_t button, tx_button_color_t actions[2]);
    void SetBackpackDisable(bool backpackDisable);
    void SetBackpackTlmEnabled(bool enabled);
    void SetPTRStartChannel(uint8_t ptrStartChannel);
    void SetPTREnableChannel(uint8_t ptrEnableChannel);

    // State setters
    bool SetModelId(uint8_t modelId);

private:
#if !defined(PLATFORM_ESP32)
    void UpgradeEepromV5ToV6();
    void UpgradeEepromV6ToV7();
#endif

    tx_config_t m_config;
    ELRS_EEPROM *m_eeprom;
    uint8_t     m_modified;
    model_config_t *m_model;
    uint8_t     m_modelId;
#if defined(PLATFORM_ESP32)
    nvs_handle  handle;
#endif
};

extern TxConfig config;

#endif

///////////////////////////////////////////////////

#if defined(TARGET_RX)

typedef enum {
    MIX_SOURCE_CH1,
    MIX_SOURCE_CH2,
    MIX_SOURCE_CH3,
    MIX_SOURCE_CH4,
    MIX_SOURCE_CH5,
    MIX_SOURCE_CH6,
    MIX_SOURCE_CH7,
    MIX_SOURCE_CH8,
    MIX_SOURCE_CH9,
    MIX_SOURCE_CH10,
    MIX_SOURCE_CH11,
    MIX_SOURCE_CH12,
    MIX_SOURCE_CH13,
    MIX_SOURCE_CH14,
    MIX_SOURCE_CH15,
    MIX_SOURCE_CH16,
    MIX_SOURCE_GYRO_ROLL,
    MIX_SOURCE_GYRO_PITCH,
    MIX_SOURCE_GYRO_YAW,
    MIX_SOURCE_FAILSAFE,
} mix_source_t;

typedef enum {
    MIX_DESTINATION_CH1,
    MIX_DESTINATION_CH2,
    MIX_DESTINATION_CH3,
    MIX_DESTINATION_CH4,
    MIX_DESTINATION_CH5,
    MIX_DESTINATION_CH6,
    MIX_DESTINATION_CH7,
    MIX_DESTINATION_CH8,
    MIX_DESTINATION_CH9,
    MIX_DESTINATION_CH10,
    MIX_DESTINATION_CH11,
    MIX_DESTINATION_CH12,
    MIX_DESTINATION_CH13,
    MIX_DESTINATION_CH14,
    MIX_DESTINATION_CH15,
    MIX_DESTINATION_CH16,
    MIX_DESTINATION_GYRO_MODE,
    MIX_DESTINATION_GYRO_GAIN,
    MIX_DESTINATION_GYRO_ROLL,
    MIX_DESTINATION_GYRO_PITCH,
    MIX_DESTINATION_GYRO_YAW,
} mix_destination_t;

constexpr uint8_t PWM_MAX_CHANNELS = 16;

// The first 16 mixes are used for remapping outputs. This includes serial
// outputs and PWM outputs.
constexpr uint8_t MAX_MIXES = 30; // json library seems to crash with more that 30 list elements

typedef union {
    struct {
        uint32_t max:12,
                 min:12,
                 unused: 8;
    } val;
    uint32_t raw;
} rx_config_pwm_limits_t;

typedef union {
    struct {
        uint32_t failsafe:11,    // us output during failsafe
                 inputChannel:4, // 0-based input channel
                 inverted:1,     // invert channel output
                 mode:4,         // Output mode (eServoOutputMode)
                 narrow:1,       // Narrow output mode (half pulse width)
                 unused:11;      // FUTURE: When someone complains "everyone" uses inverted polarity PWM or something :/
    } val;
    uint32_t raw;
} rx_config_pwm_t;

typedef union {
    struct {
        uint64_t active:1,
                 source:6,          // mix_source_t
                 destination:6,     // mix_destination_t
                 weight_negative:8, // -100% - +100% (signed int)
                 weight_positive:8, // -100% - +100% (signed int)
                 offset:11,         // us or CRSF value... (signed int)
                //  active:1,          // enable/disable the mix
                 unused:24;
    } val;
    uint64_t raw;
} rx_config_mix_t;

typedef struct {
    uint32_t    version;
    uint8_t     uid[UID_LEN];
    uint8_t     loanUID[UID_LEN];
    uint16_t    vbatScale;          // FUTURE: Override compiled vbat scale
    uint8_t     isBound:1,
                onLoan:1,
                power:4,
                antennaMode:2;      // 0=0, 1=1, 2=Diversity
    uint8_t     powerOnCounter:3,
                forceTlmOff:1,
                rateInitialIdx:4;   // Rate to start rateCycling at on boot
    uint8_t     modelId;
    uint8_t     serialProtocol:4,
                failsafeMode:2,
                unused:2;
    rx_config_pwm_t pwmChannels[PWM_MAX_CHANNELS];
    rx_config_pwm_limits_t pwmLimits[PWM_MAX_CHANNELS];
    rx_config_mix_t mixes[MAX_MIXES];
    #if defined(HAS_GYRO)
    rx_config_gyro_channel_t gyroChannels[PWM_MAX_CHANNELS];
    rx_config_gyro_timings_t gyroTimings[PWM_MAX_CHANNELS];
    rx_config_gyro_mode_pos_t gyroModes; // Gyro functions for switch positions
    rx_config_gyro_gains_t gyroGains[GYRO_N_AXES]; // PID gains for each axis
    rx_config_gyro_calibration_t accelCalibration;
    rx_config_gyro_calibration_t gyroCalibration;
    uint8_t gyroSensorAlignment:4,
            calibrateGyro:1,
            gyroUnused:3;
    uint8_t gyroLaunchAngle;
    uint8_t gyroSAFEPitch;
    uint8_t gyroSAFERoll;
    uint8_t gyroLevelPitch;
    uint8_t gyroLevelRoll;
    uint8_t gyroHoverStrength:4,
            gyroHoverUnused:4;
    #endif
} rx_config_t;

class RxConfig
{
public:
    RxConfig();

    void Load();
    void Commit();

    // Getters
    bool     GetIsBound() const { return firmwareOptions.hasUID || m_config.isBound; }
    const uint8_t* GetUID() const { return m_config.uid; }
    bool GetOnLoan() const { return m_config.onLoan; }
    const uint8_t* GetOnLoanUID() const { return m_config.loanUID; }
    uint8_t  GetPowerOnCounter() const { return m_config.powerOnCounter; }
    uint8_t  GetModelId() const { return m_config.modelId; }
    uint8_t GetPower() const { return m_config.power; }
    uint8_t GetAntennaMode() const { return m_config.antennaMode; }
    bool     IsModified() const { return m_modified; }
    #if defined(GPIO_PIN_PWM_OUTPUTS)
    const rx_config_pwm_t *GetPwmChannel(uint8_t ch) const { return &m_config.pwmChannels[ch]; }
    const bool GetPwmChannelInverted(uint8_t ch) const { return m_config.pwmChannels[ch].val.inverted; }
    const rx_config_pwm_limits_t *GetPwmChannelLimits(uint8_t ch) const { return &m_config.pwmLimits[ch]; }
    #endif
    const rx_config_mix_t *GetMix(uint8_t mixNumber) const { return &m_config.mixes[mixNumber]; }
    #if defined(HAS_GYRO)
    const rx_config_gyro_channel_t *GetGyroChannel(uint8_t ch) const { return &m_config.gyroChannels[ch]; }
    gyro_input_channel_function_t GetGyroChannelInputMode(uint8_t ch) { return ( gyro_input_channel_function_t) m_config.gyroChannels[ch].val.input_mode; }
    gyro_output_channel_function_t GetGyroChannelOutputMode(uint8_t ch) { return ( gyro_output_channel_function_t) m_config.gyroChannels[ch].val.output_mode; }
    const bool GetGyroChannelOutputInverted(uint8_t ch) { return m_config.gyroChannels[ch].val.inverted; }
    const rx_config_gyro_timings_t *GetGyroChannelTimings(uint8_t ch) const { return &m_config.gyroTimings[ch]; }
    const rx_config_gyro_gains_t *GetGyroGains(gyro_axis_t axis) const { return &m_config.gyroGains[axis]; }

    const rx_config_gyro_mode_pos_t *GetGyroModePos() const { return &m_config.gyroModes;}
    const int8_t GetGyroInputChannelNumber(gyro_input_channel_function_t mode);
    const int8_t GetGyroOutputChannelNumber(gyro_output_channel_function_t mode);
    const gyro_sensor_align_t GetGyroSensorAlignment() const { return (gyro_sensor_align_t) m_config.gyroSensorAlignment; }
    const bool GetCalibrateGyro() const { return m_config.calibrateGyro; }
    const rx_config_gyro_calibration_t *GetAccelCalibration() const { return &m_config.accelCalibration; }
    const rx_config_gyro_calibration_t *GetGyroCalibration() const { return &m_config.gyroCalibration; }
    const uint8_t GetGyroLaunchAngle() const { return m_config.gyroLaunchAngle; }
    const uint8_t GetGyroSAFEPitch() const { return m_config.gyroSAFEPitch; }
    const uint8_t GetGyroSAFERoll() const { return m_config.gyroSAFERoll; }
    const uint8_t GetGyroLevelPitch() const { return m_config.gyroLevelPitch; }
    const uint8_t GetGyroLevelRoll() const { return m_config.gyroLevelRoll; }
    const uint8_t GetGyroHoverStrength() const { return m_config.gyroHoverStrength; }
    #endif
    bool GetForceTlmOff() const { return m_config.forceTlmOff; }
    uint8_t GetRateInitialIdx() const { return m_config.rateInitialIdx; }
    eSerialProtocol GetSerialProtocol() const { return (eSerialProtocol)m_config.serialProtocol; }
    eFailsafeMode GetFailsafeMode() const { return (eFailsafeMode)m_config.failsafeMode; }

    // Setters
    void SetIsBound(bool isBound);
    void SetUID(uint8_t* uid);
    void SetOnLoan(bool loaned);
    void SetOnLoanUID(uint8_t* uid);
    void SetPowerOnCounter(uint8_t powerOnCounter);
    void SetModelId(uint8_t modelId);
    void SetPower(uint8_t power);
    void SetAntennaMode(uint8_t antennaMode);
    void SetDefaults(bool commit);
    void SetStorageProvider(ELRS_EEPROM *eeprom);
    #if defined(GPIO_PIN_PWM_OUTPUTS)
    void SetPwmChannel(uint8_t ch, uint16_t failsafe, uint8_t inputCh, bool inverted, uint8_t mode, bool narrow);
    void SetPwmChannelRaw(uint8_t ch, uint32_t raw);
    void SetPwmChannelLimits(uint8_t ch, uint16_t min, uint16_t max);
    void SetPwmChannelLimitsRaw(uint8_t ch, uint32_t raw);
    #if defined(HAS_GYRO)
    void SetGyroChannel(uint8_t ch, uint8_t input_mode, uint8_t output_mode, bool inverted);
    void SetGyroChannelRaw(uint8_t ch, uint32_t raw);
    void SetGyroModePos(uint8_t pos, gyro_mode_t mode);
    void SetGyroPIDRate(gyro_axis_t axis, gyro_rate_variable_t var, uint8_t value);
    void SetGyroPIDGain(gyro_axis_t axis, uint8_t value);
    void SetGyroSensorAlignment(gyro_sensor_align_t);
    void SetCalibrateGyro(bool);
    void SetAccelCalibration(uint16_t, uint16_t, uint16_t);
    void SetGyroCalibration(uint16_t, uint16_t, uint16_t);
    void SetGyroLaunchAngle(uint8_t);
    void SetGyroSAFEPitch(uint8_t);
    void SetGyroSAFERoll(uint8_t);
    void SetGyroLevelPitch(uint8_t);
    void SetGyroLevelRoll(uint8_t);
    void SetGyroHoverStrength(uint8_t strength);
    #endif
    #endif
    void SetMixer(
        uint8_t mixNumber, mix_source_t source, mix_destination_t destination,
        int8_t weight_negative, int8_t weight_positive, uint16_t offset,
        bool active
    );
    void SetMixerRaw(uint8_t mixNumber, uint64_t raw);
    void SetForceTlmOff(bool forceTlmOff);
    void SetRateInitialIdx(uint8_t rateInitialIdx);
    void SetSerialProtocol(eSerialProtocol serialProtocol);
    void SetFailsafeMode(eFailsafeMode failsafeMode);

private:
    void UpgradeEepromV4();
    void UpgradeEepromV5();
    void UpgradeEepromV6();
    void UpgradeEepromV7();
    void UpgradeEepromV8();

    void debugGyroConfiguration();

    rx_config_t m_config;
    ELRS_EEPROM *m_eeprom;
    bool        m_modified;
};

extern RxConfig config;

#endif
