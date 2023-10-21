#if defined(HAS_GYRO)
#include "targets.h"
#include "devGyro.h"
#include "gyro.h"
#include "gyro_mpu6050.h"
#include "mixer.h"
#include "logging.h"

Gyro gyro = Gyro();

#ifdef GYRO_DEVICE_MPU6050
static GyroDevMPU6050 gyro_device = GyroDevMPU6050();
#else
#error HAS_GYRO is defined but no valid GYRO_DEVICE_* defined
#endif

static void initialize()
{
    gyro.dev = &gyro_device;
    gyro.dev->initialize();
}

static bool gyro_detect() {
    // FIXME: Add a detection routine
    return true;
}

static int start()
{
    if (!mixer_initialize() || !gyro_detect()) {
        DBGLN("Gyro initialization failed")
        return DURATION_NEVER;
    }
    gyro.initialized = true;
    return gyro.dev->start();
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

device_t Gyro_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
};

#endif