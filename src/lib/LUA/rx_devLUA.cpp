#ifdef TARGET_RX

#include "rxtx_devLua.h"
#include "devCRSF.h"
#include "devServoOutput.h"
#ifdef HAS_GYRO
#include "gyro.h"
#endif

extern void deferExecution(uint32_t ms, std::function<void()> f);
extern void reconfigureSerial();

extern bool InLoanBindingMode;
extern bool returnModelFromLoan;

static char modelString[] = "000";
#if defined(PLATFORM_ESP32)
static const char *pwmModes = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;DShot";
static const char *noDShot = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off";
static const char *txModes = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;DShot;Serial TX";
static const char *rxModes = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;DShot;Serial RX";
#else
static const char *pwmModes = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off";
static const char *txModes = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;Serial TX";
static const char *rxModes = "50Hz;60Hz;100Hz;160Hz;333Hz;400Hz;10kHzDuty;On/Off;Serial RX";
#endif

#if defined(HAS_GYRO)
// Must match gyro_sensor_align_t
static const char *gyroAlign = "0;90;180;270;Flip;F90;F180;F270";
// Must match mixer.h: gyro_input_channel_function_t
static const char *gyroInputChannelModes = "None;Roll;Pitch;Yaw;Mode;Gain";
// Must match mixer.h: gyro_output_channel_function_t
static const char *gyroOutputChannelModes = "None;Aileron;Elevator;Rudder;Elevon;V Tail";
// Must match gyro.h gyro_mode_t
static const char *gyroModes = "Off;Rate;SAFE;Level;Launch;Hover";
// Must match gyro_axis_t
static const char *gyroAxis = "Roll;Pitch;Yaw";
#endif

static struct luaItem_selection luaSerialProtocol = {
    {"Protocol", CRSF_TEXT_SELECTION},
    0, // value
    "CRSF;Inverted CRSF;SBUS;Inverted SBUS;SUMD;DJI RS Pro;HoTT Telemetry",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaFailsafeMode = {
    {"Failsafe Mode", CRSF_TEXT_SELECTION},
    0, // value
    "No Pulses;Last Pos",
    STR_EMPTYSPACE
};

#if defined(POWER_OUTPUT_VALUES)
static struct luaItem_selection luaTlmPower = {
    {"Tlm Power", CRSF_TEXT_SELECTION},
    0, // value
    strPowerLevels,
    "mW"
};
#endif

static struct luaItem_selection luaRateInitIdx = {
    {"Init Rate", CRSF_TEXT_SELECTION},
    0, // value
    STR_LUA_PACKETRATES,
    STR_EMPTYSPACE
};

#if defined(GPIO_PIN_ANT_CTRL)
static struct luaItem_selection luaAntennaMode = {
    {"Ant. Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Antenna A;Antenna B;Diversity",
    STR_EMPTYSPACE
};
#endif

// Gemini Mode
#if defined(GPIO_PIN_NSS_2)
static struct luaItem_selection luaDiversityMode = {
    {"Rx Mode", CRSF_TEXT_SELECTION},
    0, // value
    "Diversity;Gemini",
    STR_EMPTYSPACE
};
#endif

//----------------------------Info-----------------------------------

static struct luaItem_string luaModelNumber = {
    {"Model Id", CRSF_INFO},
    modelString
};

static struct luaItem_string luaELRSversion = {
    {version, CRSF_INFO},
    commit
};

//----------------------------Info-----------------------------------

//---------------------------- WiFi -----------------------------


//---------------------------- WiFi -----------------------------

// --------------------------- Gyro Setup ---------------------------------

#if defined(HAS_GYRO)

static struct luaItem_int8 luaGyroLaunchAngle = {
  {"Launch Angle", CRSF_UINT8},
  {
    {
      (uint8_t)10, // value, not zero-based
      0,           // min
      45,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroSAFEPitch = {
  {"SAFE Pitch", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroSAFERoll = {
  {"SAFE Roll", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroLevelPitch = {
  {"Level Pitch", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static struct luaItem_int8 luaGyroLevelRoll = {
  {"Level Roll", CRSF_UINT8},
  {
    {
      (uint8_t)40, // value, not zero-based
      10,           // min
      60,          // max
    }
  },
  "deg"
};

static void luaparamGyroAlign(struct luaPropertiesCommon *item, uint8_t arg)
{
  arg++;
  config.SetGyroSensorAlignment((gyro_sensor_align_t) arg);
}

static void luaparamGyroSAFEPitch(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroSAFEPitch(arg); }

static void luaparamGyroSAFERoll(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroSAFERoll(arg); }

static void luaparamGyroLevelPitch(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroLevelPitch(arg); }

static void luaparamGyroLevelRoll(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroLevelRoll(arg); }

static void luaparamGyroLaunchAngle(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroLaunchAngle(arg); }

static void luaparamGyroCalibrate(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Calibrate gyro?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Calibrating gyro";
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
    gyro_event = GYRO_EVENT_CALIBRATE;
    devicesTriggerEvent();
    return;
  }
  else if (arg == lcsQuery)
  {
    msg = "Calibrating gyro";
    newStep = lcsExecuting;
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}

static void luaparamGyroSubtrims(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set subtrims?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting subtrims";
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
    gyro_event = GYRO_EVENT_SUBTRIMS;
    devicesTriggerEvent();
    return;
  }
  else if (arg == lcsQuery)
  {
    msg = "Setting subtrims";
    newStep = lcsExecuting;
    sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}

static struct luaItem_command luaGyroCalibrate = {
    {"Calibrate Gyro", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static struct luaItem_command luaGyroSubtrims = {
    {"Set Subtrims", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static struct luaItem_folder luaGyroModesFolder = {
    {"Gyro Modes", CRSF_FOLDER},
};

static struct luaItem_folder luaGyroGainFolder = {
    {"Gyro Gains", CRSF_FOLDER},
};

static struct luaItem_folder luaGyroInputFolder = {
    {"Gyro Inputs", CRSF_FOLDER},
};

static struct luaItem_int8 luaGyroInputChannel = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroInputMode = {
    {"Function", CRSF_TEXT_SELECTION},
    0, // value
    gyroInputChannelModes,
    STR_EMPTYSPACE
};

static struct luaItem_folder luaGyroOutputFolder = {
    {"Gyro Outputs", CRSF_FOLDER},
};

static struct luaItem_int8 luaGyroOutputChannel = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)1,       // value, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroOutputMode = {
    {"Function", CRSF_TEXT_SELECTION},
    0, // value
    gyroOutputChannelModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroOutputInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos1 = {
    {"Position 1", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos2 = {
    {"Position 2", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos3 = {
    {"Position 3", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos4 = {
    {"Position 4", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaGyroModePos5 = {
    {"Position 5", CRSF_TEXT_SELECTION},
    0, // value
    gyroModes,
    STR_EMPTYSPACE
};

static struct luaItem_folder luaGyroSettingsFolder = {
    {"Gyro Settings", CRSF_FOLDER},
};

static void luaparamGyroInputChannel(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaUint8Value(&luaGyroInputChannel, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent();
}
static void luaparamGyroInputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
    const uint8_t ch = luaGyroInputChannel.properties.u.value - 1;
    rx_config_gyro_channel_t newCh;
    newCh.raw = config.GetGyroChannel(ch)->raw;
    newCh.val.input_mode = arg;
    config.SetGyroChannelRaw(ch, newCh.raw);
}

static void luaparamGyroOutputChannel(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaUint8Value(&luaGyroOutputChannel, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent();
}
static void luaparamGyroOutputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
    const uint8_t ch = luaGyroOutputChannel.properties.u.value - 1;
    rx_config_gyro_channel_t newCh;
    newCh.raw = config.GetGyroChannel(ch)->raw;
    newCh.val.output_mode = arg;
    config.SetGyroChannelRaw(ch, newCh.raw);
}

static void luaparamGyroOutputInverted(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaGyroOutputChannel.properties.u.value - 1;
  rx_config_gyro_channel_t newCh;
  newCh.raw = config.GetGyroChannel(ch)->raw;
  newCh.val.inverted = arg;

  config.SetGyroChannelRaw(ch, newCh.raw);
}

static void luaparamGyroModePos1(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(0, (gyro_mode_t) arg); }
static void luaparamGyroModePos2(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(1, (gyro_mode_t) arg); }
static void luaparamGyroModePos3(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(2, (gyro_mode_t) arg); }
static void luaparamGyroModePos4(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(3, (gyro_mode_t) arg); }
static void luaparamGyroModePos5(struct luaPropertiesCommon *item, uint8_t arg)
{ config.SetGyroModePos(4, (gyro_mode_t) arg); }

static struct luaItem_selection luaGyroAlign = {
    {"Gyro Align", CRSF_TEXT_SELECTION},
    0, // value
    gyroAlign,
    STR_EMPTYSPACE
};

// contents of "Gyro Gains" folder, per axis subfolders
static struct luaItem_selection luaGyroGainAxis = {
    {"Gyro Axis", CRSF_TEXT_SELECTION},
    0, // value
    gyroAxis,
    STR_EMPTYSPACE
};

static void luaparamGyroGainAxis(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaTextSelectionValue(&luaGyroGainAxis, arg);
  // Trigger reload of values for the selected channel
  devicesTriggerEvent();
}

static struct luaItem_int8 luaGyroPIDRateP = {
  {"P Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};
static struct luaItem_int8 luaGyroPIDRateI = {
  {"I Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};
static struct luaItem_int8 luaGyroPIDRateD = {
  {"D Rate", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      100            // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaGyroPIDGain = {
  {"Axis Gain", CRSF_UINT8},
  {
    {
      (uint8_t)1,    // value
      0,             // min
      255            // max
    }
  },
  STR_EMPTYSPACE
};

static void luaparamGyroPIDRateP(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_P, arg);
}

static void luaparamGyroPIDRateI(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_I, arg);
}

static void luaparamGyroPIDRateD(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDRate(axis, GYRO_RATE_VARIABLE_D, arg);
}

static void luaparamGyroPIDGain(struct luaPropertiesCommon *item, uint8_t arg)
{
  const gyro_axis_t axis = (gyro_axis_t) luaGyroGainAxis.value;
  config.SetGyroPIDGain(axis, arg);
}

#endif // USE_GYRO

// --------------------------- Gyro Setup ---------------------------------

//---------------------------- Output Mapping -----------------------------

#if defined(GPIO_PIN_PWM_OUTPUTS)
static struct luaItem_folder luaMappingFolder = {
    {"Output Mapping", CRSF_FOLDER},
};

static struct luaItem_int8 luaMappingChannelOut = {
  {"Output Ch", CRSF_UINT8},
  {
    {
      (uint8_t)5,       // value - start on AUX1, value is 1-16, not zero-based
      1,                // min
      PWM_MAX_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_int8 luaMappingChannelIn = {
  {"Input Ch", CRSF_UINT8},
  {
    {
      0,                 // value
      1,                 // min
      CRSF_NUM_CHANNELS, // max
    }
  },
  STR_EMPTYSPACE
};

static struct luaItem_selection luaMappingOutputMode = {
    {"Output Mode", CRSF_TEXT_SELECTION},
    0, // value
    pwmModes,
    STR_EMPTYSPACE
};

static struct luaItem_selection luaMappingInverted = {
    {"Invert", CRSF_TEXT_SELECTION},
    0, // value
    "Off;On",
    STR_EMPTYSPACE
};

static struct luaItem_command luaSetFailsafe = {
    {"Set Failsafe Pos", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

const char STR_US[] = "us";
static struct luaItem_int16 luaMappingChannelLimitMin = {
  {"Limit Min us", CRSF_UINT16},
  {
    {
      0U,  // value
      0U,  // min
      65535U, // max
    }
  },
  STR_US
};

static struct luaItem_int16 luaMappingChannelLimitMax = {
  {"Limit Max us", CRSF_INT16},
  {
    {
      2135, // value
      1501, // min
      2135, // max
    }
  },
  STR_US
};

#endif // GPIO_PIN_PWM_OUTPUTS

//---------------------------- Output Mapping -----------------------------

//---------------------------- Model Loan Out -----------------------------

static struct luaItem_command luaLoanModel = {
    {"Loan Model", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

static struct luaItem_command luaReturnModel = {
    {"Return Model", CRSF_COMMAND},
    lcsIdle, // step
    STR_EMPTYSPACE
};

//---------------------------- Model Loan Out -----------------------------

#if defined(GPIO_PIN_PWM_OUTPUTS)

static void luaparamMappingChannelOut(struct luaPropertiesCommon *item, uint8_t arg)
{
  setLuaUint8Value(&luaMappingChannelOut, arg);
  // Must trigger an event because this is not a persistent config item
  if (GPIO_PIN_PWM_OUTPUTS[arg-1] == 3)
  {
    luaMappingOutputMode.options = rxModes;
  }
  else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == 1)
  {
    luaMappingOutputMode.options = txModes;
  }
  #if defined(PLATFORM_ESP32)
  else if (GPIO_PIN_PWM_OUTPUTS[arg-1] == 0)
  {
    luaMappingOutputMode.options = noDShot;
  }
  #endif
  else
  {
    luaMappingOutputMode.options = pwmModes;
  }
  devicesTriggerEvent();
}

static void luaparamMappingChannelIn(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inputChannel = arg - 1; // convert 1-16 -> 0-15

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static uint8_t configureSerialPin(uint8_t pin, uint8_t sibling, uint8_t oldMode, uint8_t newMode)
{
  for (int ch=0 ; ch<GPIO_PIN_PWM_OUTPUTS_COUNT ; ch++)
  {
    if (GPIO_PIN_PWM_OUTPUTS[ch] == sibling)
    {
      // set sibling pin channel settings based on this pins settings
      rx_config_pwm_t newPin3Config;
      if (newMode == somSerial)
      {
        newPin3Config.val.mode = somSerial;
      }
      else
      {
        newPin3Config.val.mode = som50Hz;
      }
      config.SetPwmChannelRaw(ch, newPin3Config.raw);
      break;
    }
  }
  if (oldMode != newMode)
  {
    deferExecution(100, [](){
      reconfigureSerial();
    });
  }
  return newMode;
}

static void luaparamMappingOutputMode(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  uint8_t oldMode = newPwmCh.val.mode;
  newPwmCh.val.mode = arg;

  // Check if pin == 1/3 and do other pin adjustment accordingly
  if (GPIO_PIN_PWM_OUTPUTS[ch] == 1)
  {
    newPwmCh.val.mode = configureSerialPin(1, 3, oldMode, arg);
  }
  else if (GPIO_PIN_PWM_OUTPUTS[ch] == 3)
  {
    newPwmCh.val.mode = configureSerialPin(3, 1, oldMode, arg);
  }
  else if (arg == somSerial)
  {
    newPwmCh.val.mode = oldMode;
  }
  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamMappingInverted(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_t newPwmCh;
  newPwmCh.raw = config.GetPwmChannel(ch)->raw;
  newPwmCh.val.inverted = arg;

  config.SetPwmChannelRaw(ch, newPwmCh.raw);
}

static void luaparamSetFailsafe(struct luaPropertiesCommon *item, uint8_t arg)
{
  luaCmdStep_e newStep;
  const char *msg;
  if (arg == lcsClick)
  {
    newStep = lcsAskConfirm;
    msg = "Set failsafe to curr?";
  }
  else if (arg == lcsConfirmed)
  {
    // This is generally not seen by the user, since we'll disconnect to commit config
    // and the handset will send another lcdQuery that will overwrite it with idle
    newStep = lcsExecuting;
    msg = "Setting failsafe";

    for (unsigned ch=0; ch<(unsigned)GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
      rx_config_pwm_t newPwmCh;
      newPwmCh.raw = config.GetPwmChannel(ch)->raw;
      newPwmCh.val.failsafe = CRSF_to_US(constrain(ChannelData[config.GetPwmChannel(ch)->val.inputChannel], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX));
      // DBGLN("FSCH(%u) crsf=%u us=%u", ch, ChannelData[ch], newPwmCh.val.failsafe);
      config.SetPwmChannelRaw(ch, newPwmCh.raw);
    }
  }
  else
  {
    newStep = lcsIdle;
    msg = STR_EMPTYSPACE;
  }

  sendLuaCommandResponse((struct luaItem_command *)item, newStep, msg);
}
#include "logging.h"
static void luaparamMappingChannelLimitMin(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_limits_t limits;
  limits.raw = config.GetPwmChannelLimits(ch)->raw;
  // limits.val.min = arg;
  char dbgline[128] = "";
  uint16_t num = luaMappingChannelLimitMin.properties.u.value;
  sprintf(dbgline, "ul: %ul ud: %ud", num, num);
  DBGLN(dbgline)
  DBGLN("*** lua min value: %d", luaMappingChannelLimitMin.properties.u.value)
  DBGLN("*** lua min value: %d", luaMappingChannelLimitMin.properties.s.value)
  limits.val.min = (uint16_t) luaMappingChannelLimitMin.properties.u.value;
  config.SetPwmChannelLimitsRaw(ch, limits.raw);
}

static void luaparamMappingChannelLimitMax(struct luaPropertiesCommon *item, uint8_t arg)
{
  const uint8_t ch = luaMappingChannelOut.properties.u.value - 1;
  rx_config_pwm_limits_t limits;
  limits.raw = config.GetPwmChannelLimits(ch)->raw;
  // limits.val.max = arg;
  limits.val.max = luaMappingChannelLimitMax.properties.u.value;
  config.SetPwmChannelLimitsRaw(ch, limits.raw);
}

#endif // GPIO_PIN_PWM_OUTPUTS

#if defined(POWER_OUTPUT_VALUES)

static void luaparamSetPower(struct luaPropertiesCommon* item, uint8_t arg)
{
  uint8_t newPower = arg + POWERMGNT::getMinPower();
  if (newPower > POWERMGNT::getMaxPower())
  {
    newPower = PWR_MATCH_TX;
  }

  config.SetPower(newPower);
  // POWERMGNT::setPower() will be called in updatePower() in the main loop
}

#endif // POWER_OUTPUT_VALUES

static void registerLuaParameters()
{
  registerLUAParameter(&luaSerialProtocol, [](struct luaPropertiesCommon* item, uint8_t arg){
    config.SetSerialProtocol((eSerialProtocol)arg);
    if (config.IsModified()) {
      deferExecution(100, [](){
        reconfigureSerial();
      });
    }
  });

  if (config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_SBUS || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
  {
    registerLUAParameter(&luaFailsafeMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetFailsafeMode((eFailsafeMode)arg);
    });
  }

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    registerLUAParameter(&luaAntennaMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg);
    });
  }

  // Gemini Mode
  if (isDualRadio())
  {
    registerLUAParameter(&luaDiversityMode, [](struct luaPropertiesCommon* item, uint8_t arg){
      config.SetAntennaMode(arg); // Reusing SetAntennaMode since both GPIO_PIN_ANTENNA_SELECT and GPIO_PIN_NSS_2 will not be defined together.
    });
  }

#if defined(POWER_OUTPUT_VALUES)
  luadevGeneratePowerOpts(&luaTlmPower);
  registerLUAParameter(&luaTlmPower, &luaparamSetPower);
#endif
  registerLUAParameter(&luaRateInitIdx, [](struct luaPropertiesCommon* item, uint8_t arg) {
    uint8_t newRate = RATE_MAX - 1 - arg;
    config.SetRateInitialIdx(newRate);
  });
  registerLUAParameter(&luaLoanModel, [](struct luaPropertiesCommon* item, uint8_t arg){
    // Do it when polling for status i.e. going back to idle, because we're going to lose conenction to the TX
    if (arg == 6) {
      deferExecution(200, [](){ InLoanBindingMode = true; });
    }
    sendLuaCommandResponse(&luaLoanModel, arg < 5 ? lcsExecuting : lcsIdle, arg < 5 ? "Sending..." : "");
  });
  registerLUAParameter(&luaReturnModel, [](struct luaPropertiesCommon* item, uint8_t arg){
    // Do it when polling for status i.e. going back to idle, because we're going to lose conenction to the TX
    if (arg == 6) {
      deferExecution(200, []() { returnModelFromLoan = true; });
    }
    sendLuaCommandResponse(&luaReturnModel, arg < 5 ? lcsExecuting : lcsIdle, arg < 5 ? "Sending..." : "");
  });
#if defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    registerLUAParameter(&luaMappingFolder);
    registerLUAParameter(&luaMappingChannelOut, &luaparamMappingChannelOut, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingChannelIn, &luaparamMappingChannelIn, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingOutputMode, &luaparamMappingOutputMode, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingInverted, &luaparamMappingInverted, luaMappingFolder.common.id);
    registerLUAParameter(&luaSetFailsafe, &luaparamSetFailsafe);
    registerLUAParameter(&luaMappingChannelLimitMin, &luaparamMappingChannelLimitMin, luaMappingFolder.common.id);
    registerLUAParameter(&luaMappingChannelLimitMax, &luaparamMappingChannelLimitMax, luaMappingFolder.common.id);

    #if defined(HAS_GYRO)
    registerLUAParameter(&luaGyroModesFolder);
    registerLUAParameter(&luaGyroModePos1, &luaparamGyroModePos1, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos2, &luaparamGyroModePos2, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos3, &luaparamGyroModePos3, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos4, &luaparamGyroModePos4, luaGyroModesFolder.common.id);
    registerLUAParameter(&luaGyroModePos5, &luaparamGyroModePos5, luaGyroModesFolder.common.id);

    registerLUAParameter(&luaGyroGainFolder);
    registerLUAParameter(&luaGyroGainAxis, &luaparamGyroGainAxis, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDRateP, &luaparamGyroPIDRateP, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDRateI, &luaparamGyroPIDRateI, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDRateD, &luaparamGyroPIDRateD, luaGyroGainFolder.common.id);
    registerLUAParameter(&luaGyroPIDGain, &luaparamGyroPIDGain, luaGyroGainFolder.common.id);

    registerLUAParameter(&luaGyroInputFolder);
    registerLUAParameter(&luaGyroInputChannel, &luaparamGyroInputChannel, luaGyroInputFolder.common.id);
    registerLUAParameter(&luaGyroInputMode, &luaparamGyroInputMode, luaGyroInputFolder.common.id);

    registerLUAParameter(&luaGyroOutputFolder);
    registerLUAParameter(&luaGyroOutputChannel, &luaparamGyroOutputChannel, luaGyroOutputFolder.common.id);
    registerLUAParameter(&luaGyroOutputMode, &luaparamGyroOutputMode, luaGyroOutputFolder.common.id);
    registerLUAParameter(&luaGyroOutputInverted, &luaparamGyroOutputInverted, luaGyroOutputFolder.common.id);

    registerLUAParameter(&luaGyroSettingsFolder);
    registerLUAParameter(&luaGyroAlign, &luaparamGyroAlign, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroCalibrate, &luaparamGyroCalibrate, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroSubtrims, &luaparamGyroSubtrims, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroSAFEPitch, &luaparamGyroSAFEPitch, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroSAFERoll, &luaparamGyroSAFERoll, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroLevelPitch, &luaparamGyroLevelPitch, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroLevelRoll, &luaparamGyroLevelRoll, luaGyroSettingsFolder.common.id);
    registerLUAParameter(&luaGyroLaunchAngle, &luaparamGyroLaunchAngle, luaGyroSettingsFolder.common.id);
    #endif
  }
#endif

  registerLUAParameter(&luaModelNumber);
  registerLUAParameter(&luaELRSversion);
  registerLUAParameter(NULL);
}

static int event()
{
  setLuaTextSelectionValue(&luaSerialProtocol, config.GetSerialProtocol());
  setLuaTextSelectionValue(&luaFailsafeMode, config.GetFailsafeMode());

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    setLuaTextSelectionValue(&luaAntennaMode, config.GetAntennaMode());
  }

  // Gemini Mode
  if (isDualRadio())
  {
    setLuaTextSelectionValue(&luaDiversityMode, config.GetAntennaMode()); // Reusing SetAntennaMode since both GPIO_PIN_ANTENNA_SELECT and GPIO_PIN_NSS_2 will not be defined together.
  }

#if defined(POWER_OUTPUT_VALUES)
  // The last item (for MatchTX) will be MaxPower - MinPower + 1
  uint8_t luaPwrVal = (config.GetPower() == PWR_MATCH_TX) ? POWERMGNT::getMaxPower() + 1 : config.GetPower();
  setLuaTextSelectionValue(&luaTlmPower, luaPwrVal - POWERMGNT::getMinPower());
#endif
  setLuaTextSelectionValue(&luaRateInitIdx, RATE_MAX - 1 - config.GetRateInitialIdx());

#if defined(GPIO_PIN_PWM_OUTPUTS)
  if (OPT_HAS_SERVO_OUTPUT)
  {
    const rx_config_pwm_t *pwmCh = config.GetPwmChannel(luaMappingChannelOut.properties.u.value - 1);
    setLuaUint8Value(&luaMappingChannelIn, pwmCh->val.inputChannel + 1);
    setLuaTextSelectionValue(&luaMappingOutputMode, pwmCh->val.mode);
    setLuaTextSelectionValue(&luaMappingInverted, pwmCh->val.inverted);
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(luaMappingChannelOut.properties.u.value - 1);
    setLuaUint16Value(&luaMappingChannelLimitMin, (uint16_t) limits->val.min);
    setLuaUint16Value(&luaMappingChannelLimitMax, (uint16_t) limits->val.max);

    #if defined(HAS_GYRO)
    const rx_config_gyro_channel_t *gyroChIn = config.GetGyroChannel(luaGyroInputChannel.properties.u.value - 1);
    setLuaTextSelectionValue(&luaGyroInputMode, gyroChIn->val.input_mode);
    const rx_config_gyro_channel_t *gyroChOut = config.GetGyroChannel(luaGyroOutputChannel.properties.u.value - 1);
    setLuaTextSelectionValue(&luaGyroOutputMode, gyroChOut->val.output_mode);
    setLuaTextSelectionValue(&luaGyroOutputInverted, gyroChOut->val.inverted);

    const rx_config_gyro_mode_pos_t *gyroModes = config.GetGyroModePos();
    setLuaTextSelectionValue(&luaGyroModePos1, gyroModes->val.pos1);
    setLuaTextSelectionValue(&luaGyroModePos2, gyroModes->val.pos2);
    setLuaTextSelectionValue(&luaGyroModePos3, gyroModes->val.pos3);
    setLuaTextSelectionValue(&luaGyroModePos4, gyroModes->val.pos4);
    setLuaTextSelectionValue(&luaGyroModePos5, gyroModes->val.pos5);

    const rx_config_gyro_gains_t *gyroGains = config.GetGyroGains((gyro_axis_t) (luaGyroGainAxis.value));
    setLuaUint8Value(&luaGyroPIDRateP, gyroGains->p);
    setLuaUint8Value(&luaGyroPIDRateI, gyroGains->i);
    setLuaUint8Value(&luaGyroPIDRateD, gyroGains->d);
    setLuaUint8Value(&luaGyroPIDGain, gyroGains->gain);

    setLuaTextSelectionValue(&luaGyroAlign, config.GetGyroSensorAlignment() - 1);
    setLuaUint8Value(&luaGyroSAFEPitch, config.GetGyroSAFEPitch());
    setLuaUint8Value(&luaGyroSAFERoll, config.GetGyroSAFERoll());
    setLuaUint8Value(&luaGyroLevelPitch, config.GetGyroLevelPitch());
    setLuaUint8Value(&luaGyroLevelRoll, config.GetGyroLevelRoll());
    setLuaUint8Value(&luaGyroLaunchAngle, config.GetGyroLaunchAngle());
    #endif // HAS_GYRO
  }
#endif

  if (config.GetModelId() == 255)
  {
    setLuaStringValue(&luaModelNumber, "Off");
  }
  else
  {
    itoa(config.GetModelId(), modelString, 10);
    setLuaStringValue(&luaModelNumber, modelString);
  }
  return DURATION_IMMEDIATELY;
}

static int timeout()
{
  luaHandleUpdateParameter();
  // Receivers can only `UpdateParamReq == true` every 4th packet due to the transmitter cadence in 1:2
  // Channels, Downlink Telem Slot, Uplink Telem (the write command), Downlink Telem Slot...
  // (interval * 4 / 1000) or 1 second if not connected
  return (connectionState == connected) ? ExpressLRS_currAirRate_Modparams->interval / 250 : 1000;
}

static int start()
{
  registerLuaParameters();
  event();
  return DURATION_IMMEDIATELY;
}

device_t LUA_device = {
  .initialize = NULL,
  .start = start,
  .event = event,
  .timeout = timeout
};

#endif
