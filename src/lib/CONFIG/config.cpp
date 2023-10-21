#include "config.h"
#include "config_legacy.h"
#include "common.h"
#include "gyro.h"
#include "POWERMGNT.h"
#include "OTA.h"
#include "helpers.h"
#include "logging.h"

#if defined(TARGET_TX)

#define MODEL_CHANGED       bit(1)
#define VTX_CHANGED         bit(2)
#define MAIN_CHANGED        bit(3) // catch-all for global config item
#define FAN_CHANGED         bit(4)
#define MOTION_CHANGED      bit(5)
#define BUTTON_CHANGED      bit(6)
#define ALL_CHANGED         (MODEL_CHANGED | VTX_CHANGED | MAIN_CHANGED | FAN_CHANGED | MOTION_CHANGED | BUTTON_CHANGED)

// Really awful but safe(?) type punning of model_config_t/v6_model_config_t to and from uint32_t
template<class T> static const void U32_to_Model(uint32_t const u32, T * const model)
{
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint32_t)-sizeof(T)];
        } val;
        uint32_t u32;
    } converter = { .u32 = u32 };

    *model = converter.val.model;
}

template<class T> static const uint32_t Model_to_U32(T const * const model)
{
    // clear the entire union because the assignment will only fill sizeof(T)
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint32_t)-sizeof(T)];
        } val;
        uint32_t u32;
    } converter = { 0 };

    converter.val.model = *model;
    return converter.u32;
}

static uint8_t RateV6toV7(uint8_t rateV6)
{
#if defined(RADIO_SX127X)
    if (rateV6 == 0)
    {
        // 200Hz stays same
        return 0;
    }

    // 100Hz, 50Hz, 25Hz all move up one
    // to make room for 100Hz Full
    return rateV6 + 1;
#else // RADIO_2400
    switch (rateV6)
    {
        case 0: return 4; // 500Hz
        case 1: return 6; // 250Hz
        case 2: return 7; // 150Hz
        case 3: return 9; // 50Hz
        default: return 4; // 500Hz
    }
#endif // RADIO_2400
}

static uint8_t RatioV6toV7(uint8_t ratioV6)
{
    // All shifted up for Std telem
    return ratioV6 + 1;
}

static uint8_t SwitchesV6toV7(uint8_t switchesV6)
{
    // 0 was removed, Wide(2) became 0, Hybrid(1) became 1
    switch (switchesV6)
    {
        case 1: return (uint8_t)smHybridOr16ch;
        case 2:
        default:
            return (uint8_t)smWideOr8ch;
    }
}

static void ModelV6toV7(v6_model_config_t const * const v6, model_config_t * const v7)
{
    v7->rate = RateV6toV7(v6->rate);
    v7->tlm = RatioV6toV7(v6->tlm);
    v7->power = v6->power;
    v7->switchMode = SwitchesV6toV7(v6->switchMode);
    v7->modelMatch = v6->modelMatch;
    v7->dynamicPower = v6->dynamicPower;
    v7->boostChannel = v6->boostChannel;
}

TxConfig::TxConfig() :
    m_model(m_config.model_config)
{
}

#if defined(PLATFORM_ESP32)
void TxConfig::Load()
{
    m_modified = 0;

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    ESP_ERROR_CHECK(nvs_open("ELRS", NVS_READWRITE, &handle));

    // Try to load the version and make sure it is a TX config
    uint32_t version = 0;
    if (nvs_get_u32(handle, "tx_version", &version) == ESP_OK && ((version & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC))
        version = version & ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    // Can't upgrade from version <5, or when flashing a previous version, just use defaults.
    if (version < 5 || version > TX_CONFIG_VERSION)
    {
        SetDefaults(true);
        return;
    }

    SetDefaults(false);

    uint32_t value;
    uint8_t value8;
    // vtx (v5)
    if (nvs_get_u32(handle, "vtx", &value) == ESP_OK)
    {
        m_config.vtxBand = value >> 24;
        m_config.vtxChannel = value >> 16;
        m_config.vtxPower = value >> 8;
        m_config.vtxPitmode = value;
    }

    // fanthresh (v5)
    if (nvs_get_u8(handle, "fanthresh", &value8) == ESP_OK)
        m_config.powerFanThreshold = value8;

    // Both of these were added to config v5 without incrementing the version
    if (nvs_get_u32(handle, "fan", &value) == ESP_OK)
        m_config.fanMode = value;
    if (nvs_get_u32(handle, "motion", &value) == ESP_OK)
        m_config.motionMode = value;

    if (version >= 6)
    {
        // dvr (v6)
        if (nvs_get_u8(handle, "dvraux", &value8) == ESP_OK)
            m_config.dvrAux = value8;
        if (nvs_get_u8(handle, "dvrstartdelay", &value8) == ESP_OK)
            m_config.dvrStartDelay = value8;
        if (nvs_get_u8(handle, "dvrstopdelay", &value8) == ESP_OK)
            m_config.dvrStopDelay = value8;
    }
    else
    {
        // Need to write the dvr defaults
        m_modified |= MAIN_CHANGED;
    }

    if (version >= 7) {
        // load button actions
        if (nvs_get_u32(handle, "button1", &value) == ESP_OK)
            m_config.buttonColors[0].raw = value;
        if (nvs_get_u32(handle, "button2", &value) == ESP_OK)
            m_config.buttonColors[1].raw = value;
        // backpackdisable was actually added after 7, but if not found will default to 0 (enabled)
        if (nvs_get_u8(handle, "backpackdisable", &value8) == ESP_OK)
            m_config.backpackDisable = value8;
        if (nvs_get_u8(handle, "backpacktlmen", &value8) == ESP_OK)
            m_config.backpackTlmEnabled = value8;
    }

    for(unsigned i=0; i<64; i++)
    {
        char model[10] = "model";
        itoa(i, model+5, 10);
        if (nvs_get_u32(handle, model, &value) == ESP_OK)
        {
            if (version >= 7)
            {
                U32_to_Model(value, &m_config.model_config[i]);
            }
            else
            {
                // Upgrade v6 to v7 directly writing to nvs instead of calling Commit() over and over
                v6_model_config_t v6model;
                U32_to_Model(value, &v6model);
                model_config_t * const newModel = &m_config.model_config[i];
                ModelV6toV7(&v6model, newModel);
                nvs_set_u32(handle, model, Model_to_U32(newModel));
            }
        }
    } // for each model

    if (version != TX_CONFIG_VERSION)
    {
        Commit();
    }
}
#else  // STM32/ESP8266
void TxConfig::Load()
{
    m_modified = 0;
    m_eeprom->Get(0, m_config);

    uint32_t version = 0;
    if ((m_config.version & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC)
        version = m_config.version & ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    // If version is current, all done
    if (version == TX_CONFIG_VERSION)
        return;

    // Can't upgrade from version <5, or when flashing a previous version, just use defaults.
    if (version < 5 || version > TX_CONFIG_VERSION)
    {
        SetDefaults(true);
        return;
    }

    // Upgrade EEPROM, starting with defaults
    SetDefaults(false);

    if (version == 5)
    {
        UpgradeEepromV5ToV6();
        version = 6;
    }

    if (version == 6)
    {
        UpgradeEepromV6ToV7();
    }
}

void TxConfig::UpgradeEepromV5ToV6()
{
    v5_tx_config_t v5Config;
    v6_tx_config_t v6Config = { 0 }; // default the new fields to 0

    // Populate the prev version struct from eeprom
    m_eeprom->Get(0, v5Config);

    // Copy prev values to current config struct
    // This only workse because v5 and v6 are the same up to the new fields
    // which have already been set to 0
    memcpy(&v6Config, &v5Config, sizeof(v5Config));
    v6Config.version = 6U | TX_CONFIG_MAGIC;
    m_eeprom->Put(0, v6Config);
    m_eeprom->Commit();
}

void TxConfig::UpgradeEepromV6ToV7()
{
    v6_tx_config_t v6Config;

    // Populate the prev version struct from eeprom
    m_eeprom->Get(0, v6Config);

    // Manual field copying as some fields have moved
    #define LAZY(member) m_config.member = v6Config.member
    LAZY(vtxBand);
    LAZY(vtxChannel);
    LAZY(vtxPower);
    LAZY(vtxPitmode);
    LAZY(powerFanThreshold);
    LAZY(fanMode);
    LAZY(motionMode);
    LAZY(dvrAux);
    LAZY(dvrStartDelay);
    LAZY(dvrStopDelay);
    #undef LAZY

    for (unsigned i=0; i<64; i++)
    {
        ModelV6toV7(&v6Config.model_config[i], &m_config.model_config[i]);
    }

    m_modified = ALL_CHANGED;

    // Full Commit now
    m_config.version = 7U | TX_CONFIG_MAGIC;
    Commit();
}
#endif

void
TxConfig::Commit()
{
    if (!m_modified)
    {
        // No changes
        return;
    }
#if defined(PLATFORM_ESP32)
    // Write parts to NVS
    if (m_modified & MODEL_CHANGED)
    {
        uint32_t value = Model_to_U32(m_model);
        char model[10] = "model";
        itoa(m_modelId, model+5, 10);
        nvs_set_u32(handle, model, value);
    }
    if (m_modified & VTX_CHANGED)
    {
        uint32_t value =
            m_config.vtxBand << 24 |
            m_config.vtxChannel << 16 |
            m_config.vtxPower << 8 |
            m_config.vtxPitmode;
        nvs_set_u32(handle, "vtx", value);
    }
    if (m_modified & FAN_CHANGED)
    {
        uint32_t value = m_config.fanMode;
        nvs_set_u32(handle, "fan", value);
    }
    if (m_modified & MOTION_CHANGED)
    {
        uint32_t value = m_config.motionMode;
        nvs_set_u32(handle, "motion", value);
    }
    if (m_modified & MAIN_CHANGED)
    {
        nvs_set_u8(handle, "fanthresh", m_config.powerFanThreshold);

        nvs_set_u8(handle, "backpackdisable", m_config.backpackDisable);
        nvs_set_u8(handle, "backpacktlmen", m_config.backpackTlmEnabled);
        nvs_set_u8(handle, "dvraux", m_config.dvrAux);
        nvs_set_u8(handle, "dvrstartdelay", m_config.dvrStartDelay);
        nvs_set_u8(handle, "dvrstopdelay", m_config.dvrStopDelay);
    }
    if (m_modified & BUTTON_CHANGED)
    {
        nvs_set_u32(handle, "button1", m_config.buttonColors[0].raw);
        nvs_set_u32(handle, "button2", m_config.buttonColors[1].raw);
    }
    nvs_set_u32(handle, "tx_version", m_config.version);
    nvs_commit(handle);
#else
    // Write the struct to eeprom
    m_eeprom->Put(0, m_config);
    m_eeprom->Commit();
#endif
    m_modified = 0;
}

// Setters
void
TxConfig::SetRate(uint8_t rate)
{
    if (GetRate() != rate)
    {
        m_model->rate = rate;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetTlm(uint8_t tlm)
{
    if (GetTlm() != tlm)
    {
        m_model->tlm = tlm;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetPower(uint8_t power)
{
    if (GetPower() != power)
    {
        m_model->power = power;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetDynamicPower(bool dynamicPower)
{
    if (GetDynamicPower() != dynamicPower)
    {
        m_model->dynamicPower = dynamicPower;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetBoostChannel(uint8_t boostChannel)
{
    if (GetBoostChannel() != boostChannel)
    {
        m_model->boostChannel = boostChannel;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetSwitchMode(uint8_t switchMode)
{
    if (GetSwitchMode() != switchMode)
    {
        m_model->switchMode = switchMode;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetAntennaMode(uint8_t txAntenna)
{
    if (GetAntennaMode() != txAntenna)
    {
        m_model->txAntenna = txAntenna;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetModelMatch(bool modelMatch)
{
    if (GetModelMatch() != modelMatch)
    {
        m_model->modelMatch = modelMatch;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetVtxBand(uint8_t vtxBand)
{
    if (m_config.vtxBand != vtxBand)
    {
        m_config.vtxBand = vtxBand;
        m_modified |= VTX_CHANGED;
    }
}

void
TxConfig::SetVtxChannel(uint8_t vtxChannel)
{
    if (m_config.vtxChannel != vtxChannel)
    {
        m_config.vtxChannel = vtxChannel;
        m_modified |= VTX_CHANGED;
    }
}

void
TxConfig::SetVtxPower(uint8_t vtxPower)
{
    if (m_config.vtxPower != vtxPower)
    {
        m_config.vtxPower = vtxPower;
        m_modified |= VTX_CHANGED;
    }
}

void
TxConfig::SetVtxPitmode(uint8_t vtxPitmode)
{
    if (m_config.vtxPitmode != vtxPitmode)
    {
        m_config.vtxPitmode = vtxPitmode;
        m_modified |= VTX_CHANGED;
    }
}

void
TxConfig::SetPowerFanThreshold(uint8_t powerFanThreshold)
{
    if (m_config.powerFanThreshold != powerFanThreshold)
    {
        m_config.powerFanThreshold = powerFanThreshold;
        m_modified |= MAIN_CHANGED;
    }
}

void
TxConfig::SetStorageProvider(ELRS_EEPROM *eeprom)
{
    if (eeprom)
    {
        m_eeprom = eeprom;
    }
}

void
TxConfig::SetFanMode(uint8_t fanMode)
{
    if (m_config.fanMode != fanMode)
    {
        m_config.fanMode = fanMode;
        m_modified |= FAN_CHANGED;
    }
}

void
TxConfig::SetMotionMode(uint8_t motionMode)
{
    if (m_config.motionMode != motionMode)
    {
        m_config.motionMode = motionMode;
        m_modified |= MOTION_CHANGED;
    }
}

void
TxConfig::SetDvrAux(uint8_t dvrAux)
{
    if (GetDvrAux() != dvrAux)
    {
        m_config.dvrAux = dvrAux;
        m_modified |= MAIN_CHANGED;
    }
}

void
TxConfig::SetDvrStartDelay(uint8_t dvrStartDelay)
{
    if (GetDvrStartDelay() != dvrStartDelay)
    {
        m_config.dvrStartDelay = dvrStartDelay;
        m_modified |= MAIN_CHANGED;
    }
}

void
TxConfig::SetDvrStopDelay(uint8_t dvrStopDelay)
{
    if (GetDvrStopDelay() != dvrStopDelay)
    {
        m_config.dvrStopDelay = dvrStopDelay;
        m_modified |= MAIN_CHANGED;
    }
}

void
TxConfig::SetBackpackDisable(bool backpackDisable)
{
    if (m_config.backpackDisable != backpackDisable)
    {
        m_config.backpackDisable = backpackDisable;
        m_modified |= MAIN_CHANGED;
    }
}

void
TxConfig::SetBackpackTlmEnabled(bool enabled)
{
    if (m_config.backpackTlmEnabled != enabled)
    {
        m_config.backpackTlmEnabled = enabled;
        m_modified |= MAIN_CHANGED;
    }
}

void
TxConfig::SetButtonActions(uint8_t button, tx_button_color_t *action)
{
    if (m_config.buttonColors[button].raw != action->raw) {
        m_config.buttonColors[button].raw = action->raw;
        m_modified |= BUTTON_CHANGED;
    }
}

void
TxConfig::SetPTRStartChannel(uint8_t ptrStartChannel)
{
    if (ptrStartChannel != m_model->ptrStartChannel) {
        m_model->ptrStartChannel = ptrStartChannel;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetPTREnableChannel(uint8_t ptrEnableChannel)
{
    if (ptrEnableChannel != m_model->ptrEnableChannel) {
        m_model->ptrEnableChannel = ptrEnableChannel;
        m_modified |= MODEL_CHANGED;
    }
}

void
TxConfig::SetDefaults(bool commit)
{
    // Reset everything to 0/false and then just set anything that zero is not appropriate
    memset(&m_config, 0, sizeof(m_config));

    m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
    m_config.powerFanThreshold = PWR_250mW;
    m_modified = ALL_CHANGED;

    if (commit)
    {
        m_modified = ALL_CHANGED;
    }

    // Set defaults for button 1
    tx_button_color_t default_actions1 = {
        .val = {
            .color = 226,   // R:255 G:0 B:182
            .actions = {
                {false, 2, ACTION_BIND},
                {true, 0, ACTION_INCREASE_POWER}
            }
        }
    };
    m_config.buttonColors[0].raw = default_actions1.raw;

    // Set defaults for button 2
    tx_button_color_t default_actions2 = {
        .val = {
            .color = 3,     // R:0 G:0 B:255
            .actions = {
                {false, 1, ACTION_GOTO_VTX_CHANNEL},
                {true, 0, ACTION_SEND_VTX}
            }
        }
    };
    m_config.buttonColors[1].raw = default_actions2.raw;

    for (unsigned i=0; i<64; i++)
    {
        SetModelId(i);
        #if defined(RADIO_SX127X)
            SetRate(enumRatetoIndex(RATE_LORA_200HZ));
        #elif defined(RADIO_SX128X)
            SetRate(enumRatetoIndex(RATE_LORA_250HZ));
        #endif
        SetPower(POWERMGNT::getDefaultPower());
#if defined(PLATFORM_ESP32)
        // ESP32 nvs needs to commit every model
        if (commit)
        {
            m_modified |= MODEL_CHANGED;
            Commit();
        }
#endif
    }

#if !defined(PLATFORM_ESP32)
    // STM32/ESP8266 just needs one commit
    if (commit)
    {
        Commit();
    }
#endif

    SetModelId(0);
    m_modified = 0;
}

/**
 * Sets ModelId used for subsequent per-model config gets
 * Returns: true if the model has changed
 **/
bool
TxConfig::SetModelId(uint8_t modelId)
{
    model_config_t *newModel = &m_config.model_config[modelId];
    if (newModel != m_model)
    {
        m_model = newModel;
        m_modelId = modelId;
        return true;
    }

    return false;
}
#endif

/////////////////////////////////////////////////////

#if defined(TARGET_RX)

RxConfig::RxConfig()
{
}

void RxConfig::Load()
{
    m_modified = false;
    m_eeprom->Get(0, m_config);

    uint32_t version = 0;
    if ((m_config.version & CONFIG_MAGIC_MASK) == RX_CONFIG_MAGIC)
        version = m_config.version & ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    // If version is current, all done
    if (version == RX_CONFIG_VERSION)
    {
        DBGVLN("Limits:")
        for (uint8_t i = 0; i < PWM_MAX_CHANNELS; i++)
        {
            const rx_config_pwm_limits_t *limits = GetPwmChannelLimits(i);
            DBGVLN("Channel %d: %d - %d", i, limits->val.min, limits->val.max);
        }
        DBGVLN("Active mixes:")
        for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
        {
            const rx_config_mix_t *mix = config.GetMix(mix_number);
            if (!mix->val.active)
                continue;

            DBGVLN("Mix %d: source %d destination %d wlow %d whigh %d offset %d",
                mix_number,
                (uint8_t) mix->val.source,
                (uint8_t) mix->val.destination,
                (int8_t) mix->val.weight_negative,
                (int8_t) mix->val.weight_positive,
                (uint16_t) mix->val.offset
            )
        }
        return;
    }

    // Can't upgrade from version <4, or when flashing a previous version, just use defaults.
    if (version < 4 || version > RX_CONFIG_VERSION)
    {
        SetDefaults(true);
        return;
    }

    // Upgrade EEPROM, starting with defaults
    SetDefaults(false);
    UpgradeEepromV4();
    UpgradeEepromV5();
    UpgradeEepromV6();
    UpgradeEepromV7();
    UpgradeEepromV8();
    m_config.version = RX_CONFIG_VERSION | RX_CONFIG_MAGIC;
    m_modified = true;
    Commit();
}

// ========================================================
// V4 Upgrade

static void PwmConfigV4(v4_rx_config_pwm_t const * const v4, rx_config_pwm_t * const current)
{
    current->val.failsafe = v4->val.failsafe;
    current->val.inputChannel = v4->val.inputChannel;
    current->val.inverted = v4->val.inverted;
}

void RxConfig::UpgradeEepromV4()
{
    v4_rx_config_t v4Config;
    m_eeprom->Get(0, v4Config);

    if ((v4Config.version & ~CONFIG_MAGIC_MASK) == 4)
    {
        m_config.isBound = v4Config.isBound;
        m_config.modelId = v4Config.modelId;
        memcpy(m_config.uid, v4Config.uid, sizeof(v4Config.uid));
        #if defined(GPIO_PIN_PWM_OUTPUTS)
        // OG PWMP had only 8 channels
        for (unsigned ch=0; ch<8; ++ch)
        {
            PwmConfigV4(&v4Config.pwmChannels[ch], &m_config.pwmChannels[ch]);
        }
        #endif
    }
}

// ========================================================
// V5 Upgrade

static void PwmConfigV5(v5_rx_config_pwm_t const * const v5, rx_config_pwm_t * const current)
{
    current->val.failsafe = v5->val.failsafe;
    current->val.inputChannel = v5->val.inputChannel;
    current->val.inverted = v5->val.inverted;
    current->val.narrow = v5->val.narrow;
    current->val.mode = v5->val.mode;
    if (v5->val.mode > som400Hz)
    {
        current->val.mode += 1;
    }
}

void RxConfig::UpgradeEepromV5()
{
    v5_rx_config_t v5Config;
    m_eeprom->Get(0, v5Config);
    if ((v5Config.version & ~CONFIG_MAGIC_MASK) == 5)
    {
        memcpy(m_config.uid, v5Config.uid, sizeof(v5Config.uid));
        m_config.vbatScale = v5Config.vbatScale;
        m_config.isBound = v5Config.isBound;
        m_config.power = v5Config.power;
        m_config.antennaMode = v5Config.antennaMode;
        m_config.forceTlmOff = v5Config.forceTlmOff;
        m_config.rateInitialIdx = v5Config.rateInitialIdx;
        m_config.modelId = v5Config.modelId;

        #if defined(GPIO_PIN_PWM_OUTPUTS)
        for (unsigned ch=0; ch<16; ++ch)
        {
            PwmConfigV5(&v5Config.pwmChannels[ch], &m_config.pwmChannels[ch]);
        }
        #endif
    }
}

// ========================================================
// V6 Upgrade
static void PwmConfigV6(v6_rx_config_pwm_t const * const v6, rx_config_pwm_t * const current)
{
    current->val.failsafe = v6->val.failsafe;
    current->val.inputChannel = v6->val.inputChannel;
    current->val.inverted = v6->val.inverted;
    current->val.narrow = v6->val.narrow;
    current->val.mode = v6->val.mode;
}

void RxConfig::UpgradeEepromV6()
{
    v6_rx_config_t v6Config;
    m_eeprom->Get(0, v6Config);
    if ((v6Config.version & ~CONFIG_MAGIC_MASK) == 6)
    {
        memcpy(m_config.uid, v6Config.uid, sizeof(v6Config.uid));
        m_config.vbatScale = v6Config.vbatScale;
        m_config.isBound = v6Config.isBound;
        m_config.power = v6Config.power;
        m_config.antennaMode = v6Config.antennaMode;
        m_config.forceTlmOff = v6Config.forceTlmOff;
        m_config.rateInitialIdx = v6Config.rateInitialIdx;
        m_config.modelId = v6Config.modelId;

        #if defined(GPIO_PIN_PWM_OUTPUTS)
        for (unsigned ch=0; ch<16; ++ch)
        {
            PwmConfigV6(&v6Config.pwmChannels[ch], &m_config.pwmChannels[ch]);
        }
        #endif
    }
}

// ========================================================
// V7 Upgrade
static void PwmConfigV7(v6_rx_config_pwm_t * const current)
{
    if (current->val.mode > somOnOff)
    {
        current->val.mode += 1;
    }
}

void RxConfig::UpgradeEepromV7()
{
    #if defined(GPIO_PIN_PWM_OUTPUTS)
    uint32_t version;
    m_eeprom->Get(0, version);
    if ((version & ~CONFIG_MAGIC_MASK) == 7)
    {
        for (unsigned ch=0; ch<16; ++ch)
        {
            PwmConfigV7((v6_rx_config_pwm_t *) &m_config.pwmChannels[ch]);
        }
    }
    #endif
}

// ========================================================

// ========================================================
// V8 Upgrade
static void PwmConfigV7toV8(v6_rx_config_pwm_t * const old, rx_config_pwm_t * const current)
{
    // Version 7 used 10 bits:
    // us output during failsafe +988 (e.g. 512 here would be 1500us).
    constexpr unsigned SERVO_FAILSAFE_MIN = 988U;

    // Version 8 uses 11 bits, so we can use a direct us output setting.
    if (old->val.failsafe != 0) {
        current->val.failsafe = old->val.failsafe + SERVO_FAILSAFE_MIN;
    }
}

void RxConfig::UpgradeEepromV8()
{
    v7_rx_config_t v7Config;

    // Populate the prev version struct from eeprom
    m_eeprom->Get(0, v7Config);

    if ((v7Config.version & ~CONFIG_MAGIC_MASK) != 7)
        return;

    // Manual field copying as some fields have moved
    memcpy(m_config.uid, v7Config.uid, sizeof(v7Config.uid));
    #define COPY(member) m_config.member = v7Config.member
    COPY(vbatScale);
    COPY(isBound);
    COPY(onLoan);
    COPY(power);
    COPY(antennaMode);
    COPY(powerOnCounter);
    COPY(forceTlmOff);
    COPY(rateInitialIdx);
    COPY(modelId);
    COPY(serialProtocol);
    COPY(failsafeMode);
    #undef LAZY

    // PWM failsafe field width expanded in this version
    for (unsigned ch=0; ch<16; ++ch) {
        // Upgrade failsafe field width
        PwmConfigV7toV8(&v7Config.pwmChannels[ch], &m_config.pwmChannels[ch]);

        // PWM limits were introduced in v8, set sane defaults
        m_config.pwmLimits[ch].val.min = 885; // allow extended range
        m_config.pwmLimits[ch].val.max = 2135; // allow extended range
    }
}

// ========================================================

void
RxConfig::Commit()
{
    if (!m_modified)
    {
        // No changes
        return;
    }

    // Write the struct to eeprom
    m_eeprom->Put(0, m_config);
    m_eeprom->Commit();

    m_modified = false;
}

// Setters
void
RxConfig::SetIsBound(bool isBound)
{
    if (m_config.isBound != isBound)
    {
        m_config.isBound = isBound;
        m_modified = true;
    }
}

void
RxConfig::SetUID(uint8_t* uid)
{
    for (uint8_t i = 0; i < UID_LEN; ++i)
    {
        m_config.uid[i] = uid[i];
    }
    m_modified = true;
}

void
RxConfig::SetOnLoan(bool isLoaned)
{
    if (m_config.onLoan != isLoaned)
    {
        m_config.onLoan = isLoaned;
        m_modified = true;
    }
}

void
RxConfig::SetOnLoanUID(uint8_t* uid)
{
    for (uint8_t i = 0; i < UID_LEN; ++i)
    {
        m_config.loanUID[i] = uid[i];
    }
    m_modified = true;
}

void
RxConfig::SetPowerOnCounter(uint8_t powerOnCounter)
{
    if (m_config.powerOnCounter != powerOnCounter)
    {
        m_config.powerOnCounter = powerOnCounter;
        m_modified = true;
    }
}

void
RxConfig::SetModelId(uint8_t modelId)
{
    if (m_config.modelId != modelId)
    {
        m_config.modelId = modelId;
        m_modified = true;
    }
}

void
RxConfig::SetPower(uint8_t power)
{
    if (m_config.power != power)
    {
        m_config.power = power;
        m_modified = true;
    }
}


void
RxConfig::SetAntennaMode(uint8_t antennaMode)
{
    //0 and 1 is use for gpio_antenna_select
    // 2 is diversity
    if (m_config.antennaMode != antennaMode)
    {
        m_config.antennaMode = antennaMode;
        m_modified = true;
    }
}

void
RxConfig::SetDefaults(bool commit)
{
    // Reset everything to 0/false and then just set anything that zero is not appropriate
    memset(&m_config, 0, sizeof(m_config));

    m_config.version = RX_CONFIG_VERSION | RX_CONFIG_MAGIC;
    m_config.modelId = 0xff;
    m_config.power = POWERMGNT::getDefaultPower();
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
        m_config.antennaMode = 2; // 2 is diversity
    if (GPIO_PIN_NSS_2 != UNDEF_PIN)
        m_config.antennaMode = 0; // 0 is diversity for dual radio

#if defined(GPIO_PIN_PWM_OUTPUTS)
    for (unsigned int ch=0; ch<PWM_MAX_CHANNELS; ++ch)
    {
        uint8_t mode = som50Hz;
        // setup defaults for hardware defined I2C pins that are also IO pins
        if (ch < GPIO_PIN_PWM_OUTPUTS_COUNT)
        {
            if (GPIO_PIN_PWM_OUTPUTS[ch] == GPIO_PIN_SCL)
            {
                mode = somSCL;
            }
            else if (GPIO_PIN_PWM_OUTPUTS[ch] == GPIO_PIN_SDA)
            {
                mode = somSDA;
            }
        }
        SetPwmChannel(ch, 1500, ch, false, mode, false);
        SetPwmChannelLimits(ch, 885, 2135);
        #if defined(HAS_GYRO)
        SetGyroChannel(ch, 0, 0, 0);
        #endif
    }
    SetPwmChannel(2, 988, 2, false, 0, false); // ch2 is throttle, failsafe it to 988
#endif
#if defined(HAS_GYRO)
    SetGyroSAFEPitch(45);
    SetGyroSAFERoll(45);
    SetGyroLevelPitch(60);
    SetGyroLevelRoll(60);
    SetGyroLaunchAngle(10);
    SetGyroHoverStrength(8);
#endif

#if defined(RCVR_INVERT_TX)
    m_config.serialProtocol = PROTOCOL_INVERTED_CRSF;
#else
    m_config.serialProtocol = PROTOCOL_CRSF;
#endif

    // Configure a mix from each input channel to each output channel
    for (unsigned int ch=0; ch<CRSF_NUM_CHANNELS; ch++)
        SetMixer(ch, (mix_source_t) ch, (mix_destination_t) ch, (int8_t) 100, 100, 0, true);

    if (commit)
    {
        m_modified = true;
        Commit();
    }
}

void
RxConfig::SetStorageProvider(ELRS_EEPROM *eeprom)
{
    if (eeprom)
    {
        m_eeprom = eeprom;
    }
}

#if defined(GPIO_PIN_PWM_OUTPUTS)
#if defined(HAS_GYRO)

void
RxConfig::debugGyroConfiguration()
{
    DBGLN("Gyro configuration:")
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++)
    {
        rx_config_gyro_channel_t *config = &m_config.gyroChannels[ch];
        if (config->val.input_mode == FN_IN_NONE && config->val.output_mode == FN_NONE)
            continue;
        DBGLN("CH %d: %d %d %s",
              ch, config->val.input_mode, config->val.output_mode, config->val.inverted ? "inverted" : "")
    }
}

void
RxConfig::SetGyroChannel(uint8_t ch, uint8_t input_mode, uint8_t output_mode, bool inverted)
{
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_gyro_channel_t *config = &m_config.gyroChannels[ch];
    rx_config_gyro_channel_t newConfig;
    newConfig.val.input_mode = input_mode;
    newConfig.val.output_mode = output_mode;
    newConfig.val.inverted = inverted;

    if (config->raw == newConfig.raw)
        return;

    config->raw = newConfig.raw;
    debugGyroConfiguration();
    m_modified = true;
}

void
RxConfig::SetGyroChannelRaw(uint8_t ch, uint32_t raw)
{
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_gyro_channel_t *config = &m_config.gyroChannels[ch];
    if (config->raw == raw)
        return;

    config->raw = raw;
    m_modified = true;
    debugGyroConfiguration();
}

void
RxConfig::SetGyroModePos(uint8_t pos, gyro_mode_t mode)
{
    if (pos > 4)
        return;

    rx_config_gyro_mode_pos_t *modes = &m_config.gyroModes;
    rx_config_gyro_mode_pos_t newModes;
    newModes.raw = modes->raw;

    switch (pos)
    {
    case 0:
        newModes.val.pos1 = mode;
        break;
    case 1:
        newModes.val.pos2 = mode;
        break;
    case 2:
        newModes.val.pos3 = mode;
        break;
    case 3:
        newModes.val.pos4 = mode;
        break;
    case 4:
        newModes.val.pos5 = mode;
        break;
    }
    if (modes->raw == newModes.raw)
        return;

    modes->raw = newModes.raw;
    m_modified = true;
}

/**
 * Return the first channel matching input `mode` or -1 if not found.
*/
const int8_t RxConfig::GetGyroInputChannelNumber(gyro_input_channel_function_t mode)
{
    for (int8_t i = 0; i < GYRO_MAX_CHANNELS; i++)
        if (GetGyroChannelInputMode(i) == mode)
            return i;
    return -1;
}

/**
 * Return the first channel matching output `mode` or -1 if not found.
*/
const int8_t RxConfig::GetGyroOutputChannelNumber(gyro_output_channel_function_t mode)
{
    for (uint8_t i = 0; i < GYRO_MAX_CHANNELS; i++)
        if (GetGyroChannelOutputMode(i) == mode)
            return i;
    return -1;
}

void
RxConfig::SetGyroPIDRate(gyro_axis_t axis, gyro_rate_variable_t var, uint8_t new_value)
{
    rx_config_gyro_gains_t *config = &m_config.gyroGains[axis];

    uint8_t old_value = 0;
    switch (var)
    {
    case GYRO_RATE_VARIABLE_P:
        old_value = config->p;
        break;
    case GYRO_RATE_VARIABLE_I:
        old_value = config->i;
        break;
    case GYRO_RATE_VARIABLE_D:
        old_value = config->d;
        break;
    }

    if (new_value == old_value)
        return;

    switch (var)
    {
    case GYRO_RATE_VARIABLE_P:
        config->p = new_value;
        break;
    case GYRO_RATE_VARIABLE_I:
        config->i = new_value;
        break;
    case GYRO_RATE_VARIABLE_D:
        config->d = new_value;
        break;
    }

    m_modified = true;
    gyro.reload();
    debugGyroConfiguration();
}

void
RxConfig::SetGyroPIDGain(gyro_axis_t axis, uint8_t new_value)
{
    rx_config_gyro_gains_t *config = &m_config.gyroGains[axis];

    if (config->gain == new_value)
        return;

    config->gain = new_value;
    m_modified = true;
    gyro.reload();
    debugGyroConfiguration();
}

void
RxConfig::SetGyroSensorAlignment(gyro_sensor_align_t newAlignment)
{
    if (m_config.gyroSensorAlignment != newAlignment) {
        m_config.gyroSensorAlignment = newAlignment;
        m_modified = true;
    }
}

void
RxConfig::SetCalibrateGyro(bool value)
{
    if (m_config.calibrateGyro != value) {
        m_config.calibrateGyro = value;
        m_modified = true;
    }
}

void
RxConfig::SetGyroLaunchAngle(uint8_t angle)
{
    if (m_config.gyroLaunchAngle != angle) {
        m_config.gyroLaunchAngle = angle;
        m_modified = true;
        gyro.reload();
    }
}

void
RxConfig::SetGyroSAFEPitch(uint8_t angle)
{
    if (m_config.gyroSAFEPitch != angle) {
        m_config.gyroSAFEPitch = angle;
        m_modified = true;
        gyro.reload();
    }
}

void
RxConfig::SetGyroSAFERoll(uint8_t angle)
{
    if (m_config.gyroSAFERoll != angle) {
        m_config.gyroSAFERoll = angle;
        m_modified = true;
        gyro.reload();
    }
}

void
RxConfig::SetGyroLevelPitch(uint8_t angle)
{
    if (m_config.gyroLevelPitch != angle) {
        m_config.gyroLevelPitch = angle;
        m_modified = true;
        gyro.reload();
    }
}

void
RxConfig::SetGyroLevelRoll(uint8_t angle)
{
    if (m_config.gyroLevelRoll != angle) {
        m_config.gyroLevelRoll = angle;
        m_modified = true;
        gyro.reload();
    }
}

void
RxConfig::SetGyroHoverStrength(uint8_t strength)
{
    if (m_config.gyroHoverStrength != strength) {
        m_config.gyroHoverStrength = strength;
        m_modified = true;
        gyro.reload();
    }
}

void
RxConfig::SetAccelCalibration(uint16_t x, uint16_t y, uint16_t z)
{
    rx_config_gyro_calibration_t *accel = &m_config.accelCalibration;
    accel->x = x;
    accel->y = y;
    accel->z = z;
    m_modified = true;
}

void
RxConfig::SetGyroCalibration(uint16_t x, uint16_t y, uint16_t z)
{
    rx_config_gyro_calibration_t *gyro = &m_config.gyroCalibration;
    gyro->x = x;
    gyro->y = y;
    gyro->z = z;
    m_modified = true;
}

#endif // HAS_GYRO

void
RxConfig::SetPwmChannel(uint8_t ch, uint16_t failsafe, uint8_t inputCh, bool inverted, uint8_t mode, bool narrow)
{
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_pwm_t *pwm = &m_config.pwmChannels[ch];
    rx_config_pwm_t newConfig;
    newConfig.val.failsafe = failsafe;
    newConfig.val.inputChannel = inputCh;
    newConfig.val.inverted = inverted;
    newConfig.val.mode = mode;
    newConfig.val.narrow = narrow;
    if (pwm->raw == newConfig.raw)
        return;

    pwm->raw = newConfig.raw;
    m_modified = true;
}

void
RxConfig::SetPwmChannelRaw(uint8_t ch, uint32_t raw)
{
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_pwm_t *pwm = &m_config.pwmChannels[ch];
    if (pwm->raw == raw)
        return;

    pwm->raw = raw;
    m_modified = true;
}

void
RxConfig::SetPwmChannelLimits(uint8_t ch, uint16_t min, uint16_t max)
{
    DBGLN("*** Store PWM limits ch %d min %d max %d", ch, min, max)
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_pwm_limits_t *limits = &m_config.pwmLimits[ch];
    rx_config_pwm_limits_t new_limits;
    new_limits.val.min = min;
    new_limits.val.max = max;

    if (limits->raw == new_limits.raw)
        return;

    limits->raw = new_limits.raw;
    m_modified = true;
}

void
RxConfig::SetPwmChannelLimitsRaw(uint8_t ch, uint32_t raw)
{
    DBGLN("*** Store PWM limits")
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_pwm_limits_t *pwm = &m_config.pwmLimits[ch];
    if (pwm->raw == raw)
        return;

    pwm->raw = raw;
    DBGLN("*** Stored new PWM Limits for channel %d: Min: %d Max: %d\n",
          ch, (uint16_t) pwm->val.min, (uint16_t) pwm->val.max)
    m_modified = true;
}
#endif

void
RxConfig::SetMixer(uint8_t mixNumber, mix_source_t source, mix_destination_t destination, int8_t weight_negative, int8_t weight_positive, uint16_t offset, bool active) {
    if (mixNumber > MAX_MIXES)
        return;

    rx_config_mix_t *mix = &m_config.mixes[mixNumber];
    rx_config_mix_t new_mix;
    new_mix.raw = mix->raw;

    new_mix.val.source = source;
    new_mix.val.destination = destination;
    new_mix.val.weight_negative = weight_negative;
    new_mix.val.weight_positive = weight_positive;
    new_mix.val.offset = offset;
    new_mix.val.active = active;

    if (new_mix.raw == mix->raw)
        return;

    mix->raw = new_mix.raw;
    m_modified = true;
}

void
RxConfig::SetMixerRaw(uint8_t mixNumber, uint64_t raw)
{
    if (mixNumber > MAX_MIXES)
        return;

    rx_config_mix_t *mix = &m_config.mixes[mixNumber];

    rx_config_mix_t nmix;
    nmix.raw = raw;

    // for(;i<size*8;++i){
    //     // print last bit and shift left.
    //     printf("%u ",num&maxPow ? 1 : 0);
    //     num = num<<1;
    // }
    char line[128];
    sprintf(
        line,
        "A: %d S:%d D:%d N:%d/%d P:%d/%d Offset: %d, Raw: %llu %llu",
        (uint8_t) nmix.val.active,
        (uint8_t) nmix.val.source,
        (uint8_t) nmix.val.destination,
        (uint8_t) nmix.val.weight_negative,
        (int8_t) nmix.val.weight_negative,
        (uint8_t) nmix.val.weight_positive,
        (int8_t) nmix.val.weight_positive,
        (uint16_t) nmix.val.offset,
        raw,
        nmix.raw
    );
    DBGLN("SetMixRaw: %d %s", mixNumber, line)

    if (mix->raw == raw)
        return;

    mix->raw = raw;
    m_modified = true;
}

void
RxConfig::SetForceTlmOff(bool forceTlmOff)
{
    if (m_config.forceTlmOff != forceTlmOff)
    {
        m_config.forceTlmOff = forceTlmOff;
        m_modified = true;
    }
}

void
RxConfig::SetRateInitialIdx(uint8_t rateInitialIdx)
{
    if (m_config.rateInitialIdx != rateInitialIdx)
    {
        m_config.rateInitialIdx = rateInitialIdx;
        m_modified = true;
    }
}

void RxConfig::SetSerialProtocol(eSerialProtocol serialProtocol)
{
    if (m_config.serialProtocol != serialProtocol)
    {
        m_config.serialProtocol = serialProtocol;
        m_modified = true;
    }
}

void RxConfig::SetFailsafeMode(eFailsafeMode failsafeMode)
{
    if (m_config.failsafeMode != failsafeMode)
    {
        m_config.failsafeMode = failsafeMode;
        m_modified = true;
    }
}
#endif
