#include "targets.h"

#if defined(HAS_GYRO)
#include "gyro.h"
#include "gyro_mpu6050.h"
#include "mixer.h"
#include "logging.h"
#include "elrs_eeprom.h" // only needed to satisfy PIO
#include "config.h"
#include "MPU6050.h"

extern boolean i2c_enabled;

Gyro gyro = Gyro();

static void initialize()
{
    if (!i2c_enabled)
    {
        return;
    }
#ifdef GYRO_DEVICE_MPU6050
    Wire.setClock(400000);
    auto mpu = MPU6050();
    if (mpu.testConnection())
    {
        gyro.dev = new GyroDevMPU6050();
        gyro.dev->initialize();
        DBGLN("Detected MPU6050 Gyro");
    }
#endif
}

static bool gyro_detect() {
    return gyro.dev != nullptr;
}

static int start()
{
    if (!mixer_initialize() || !gyro_detect()) {
        DBGLN("Gyro initialization failed")
        return DURATION_NEVER;
    }
    gyro.initialized = true;
    if (config.GetCalibrateGyro()) {
        gyro.dev->calibrate();
    }
    return gyro.dev->start(false);
}

static int timeout()
{
    if (gyro.dev->read()) {
        gyro.last_update = micros();
        gyro.send_telemetry();
    }
    gyro.tick();
    return DURATION_IMMEDIATELY;
}

extern bool auto_subtrim_complete;
extern uint8_t subtrim_init;

static int event()
{
    switch (gyro_event)
    {
    case GYRO_EVENT_CALIBRATE:
        gyro_event = GYRO_EVENT_NONE;
        gyro.dev->calibrate();
        config.SetCalibrateGyro(false);
        break;

    case GYRO_EVENT_SUBTRIMS:
        gyro_event = GYRO_EVENT_NONE;
        auto_subtrim_complete = false;
        subtrim_init = 0;
        break;

    default: ;
    }

    return DURATION_IGNORE;
}

device_t Gyro_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
};

#endif