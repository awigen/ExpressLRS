#include "targets.h"

#if defined(HAS_GYRO)
#include "gyro_mpu6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "logging.h"
#include "config.h"

#define I2C_MASTER_FREQ_HZ 400000

#define gscale ((250. / 32768.0) / 100) // gyro default 250 LSB per d/s

// MPU control/status vars
bool dmpReady = false;    // set true if DMP init was successful
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

unsigned long last_gyro_update;

#ifdef DEBUG_GYRO_STATS
/**
 * For debugging print useful gyro state
 */
void GyroDevMPU6050::print_gyro_stats()
{
    if (millis() - last_gyro_stats_time < 500)
        return;

    // Calculate gyro update rate in HZ
    int update_rate = 1.0 /
                      ((micros() - last_gyro_update) / 1000000.0);

    char rate_str[5]; sprintf(rate_str, "%4d", update_rate);

    char pitch_str[8]; sprintf(pitch_str, "%6.2f", gyro.ypr[1] * 180 / M_PI);
    char roll_str[8]; sprintf(roll_str, "%6.2f", gyro.ypr[2] * 180 / M_PI);
    char yaw_str[8]; sprintf(yaw_str, "%6.2f", gyro.ypr[0] * 180 / M_PI);

    char gyro_x[8]; sprintf(gyro_x, "%6.2f", gyro.v_gyro.x);
    char gyro_y[8]; sprintf(gyro_y, "%6.2f", gyro.v_gyro.y);
    char gyro_z[8]; sprintf(gyro_z, "%6.2f", gyro.v_gyro.z);

    char gravity_x[8]; sprintf(gravity_x, "%4.2f", gyro.gravity.x);
    char gravity_y[8]; sprintf(gravity_y, "%4.2f", gyro.gravity.y);
    char gravity_z[8]; sprintf(gravity_z, "%4.2f", gyro.gravity.z);

    char debug_line[128];
    sprintf(debug_line,
        "Pitch: %.2f Roll: %.2f Yaw: %.2f"
        , gyro.ypr[1], gyro.ypr[2], gyro.ypr[0]
    );
    DBGLN(debug_line);

    // Uncomment lines needed for debugging
    DBGLN(
        // "%s HZ "
        "Gain %f "
        "Pitch:%s Roll:%s Yaw:%s "
        // "e1: %f, e2: %f, e3: %f "
        "Qw: %f Qx: %f Qy: %f Qz: %f "
        // "Gyro x: %s Gyro y: %s Gyro z: %s "
        "Gravity gX: %s gY: %s gZ: %s "
        // ,rate_str
        , gyro.gain
        ,pitch_str, roll_str, yaw_str
        // ,gyro.euler[0], gyro.euler[1], gyro.euler[2]
        ,gyro.q.w, gyro.q.x, gyro.q.y, gyro.q.z
        // ,gyro_x, gyro_y, gyro_z
        ,gravity_x, gravity_y, gravity_z
        );

    last_gyro_stats_time = millis();
}
#endif

MPU6050 mpu = MPU6050();

void GyroDevMPU6050::initialize() {
    Wire.setClock(I2C_MASTER_FREQ_HZ);
}

void GyroDevMPU6050::calibrate()
{
    mpu.reset();
    vTaskDelay(50 * portTICK_PERIOD_MS);

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setMasterClockSpeed(13); // 400kHz
    mpu.setRate(0);              // Max rate?

    // INT_PIN_CFG
    mpu.setInterruptMode(0);         // INT_LEVEL_HIGH
    mpu.setInterruptDrive(0);        // INT_OPEN_DIS (push-pull)
    mpu.setInterruptLatch(0);        // LATCH_INT_DIS (50us)
    mpu.setInterruptLatchClear(0);   // INT_RD_CLEAR_DIS (read-only)
    mpu.setFSyncInterruptLevel(0);   // FSYNC_INT_LEVEL_HIGH (active-high)
    mpu.setFSyncInterruptEnabled(0); // FSYNC_INT_DIS
    mpu.setI2CBypassEnabled(1);      // I2C_BYPASS_EN
    mpu.setClockOutputEnabled(0);    // CLOCK_DIS
    mpu.setExternalFrameSync(0);

    mpu.dmpInitialize();

    // Run the calibration
    mpu.CalibrateAccel(8);
    mpu.CalibrateGyro(8);

    // Store the calibration offsets
    config.SetAccelCalibration(
        mpu.getXAccelOffset(),
        mpu.getYAccelOffset(),
        mpu.getZAccelOffset()
    );
    config.SetGyroCalibration(
        mpu.getXGyroOffset(),
        mpu.getYGyroOffset(),
        mpu.getZGyroOffset()
    );

    start(false);
}

uint8_t GyroDevMPU6050::start(bool calibrate) {
    DBGLN("Initialize_mpu");
    mpu.reset();
    vTaskDelay(50 * portTICK_PERIOD_MS);

    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setMasterClockSpeed(13); // 400kHz
    mpu.setRate(0);              // Max rate?

    // INT_PIN_CFG
    mpu.setInterruptMode(0);         // INT_LEVEL_HIGH
    mpu.setInterruptDrive(0);        // INT_OPEN_DIS (push-pull)
    mpu.setInterruptLatch(0);        // LATCH_INT_DIS (50us)
    mpu.setInterruptLatchClear(0);   // INT_RD_CLEAR_DIS (read-only)
    mpu.setFSyncInterruptLevel(0);   // FSYNC_INT_LEVEL_HIGH (active-high)
    mpu.setFSyncInterruptEnabled(0); // FSYNC_INT_DIS
    mpu.setI2CBypassEnabled(1);      // I2C_BYPASS_EN
    mpu.setClockOutputEnabled(0);    // CLOCK_DIS

    mpu.setExternalFrameSync(0);
    mpu.dmpInitialize();

    const rx_config_gyro_calibration_t *offsets;
    offsets = config.GetAccelCalibration();
    mpu.setXAccelOffset(offsets->x);
    mpu.setYAccelOffset(offsets->y);
    mpu.setZAccelOffset(offsets->z);
    offsets = config.GetGyroCalibration();
    mpu.setXGyroOffset(offsets->x);
    mpu.setYGyroOffset(offsets->y);
    mpu.setZGyroOffset(offsets->z);

    mpu.setDMPEnabled(true);

    return DURATION_IMMEDIATELY; // Call timeout() immediately
}

uint8_t GyroDevMPU6050::event() {
    return DURATION_IGNORE;
}


/**
 * This method is used instead of mpu.dmpGetYawPitchRoll() as that method has
 * issues when gravity switches at high pitch angles.
*/
void GyroDevMPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);

    // NOTE: This is buggy at high pitch angles when gravity flips
    // if (gravity -> z < 0) {
    //     if(data[1] > 0) {
    //         data[1] = PI - data[1];
    //     } else {
    //         data[1] = -PI - data[1];
    //     }
    // }
}

bool GyroDevMPU6050::read() {
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        DBGLN("Resetting gyro FIFO buffer");
        mpu.resetFIFO();
        return false;
    }
    else if (mpuIntStatus & 0x02)
    {
        int result = mpu.GetCurrentFIFOPacket(fifoBuffer, 28);
        if (result != 1)
            return DURATION_IMMEDIATELY;

        mpu.dmpGetGyro(&gyro.v_gyro, fifoBuffer);
        mpu.dmpGetQuaternion(&gyro.q, fifoBuffer); // [w, x, y, z] quaternion container

        const gyro_sensor_align_t alignment = config.GetGyroSensorAlignment();
        if (alignment != GYRO_ALIGN_CW0_DEG)
        {
            Quaternion v_rotation(0, 0, 0, 0);
            Quaternion q_rotation(0, 0, 0, 0);

            switch (alignment)
            {
            case GYRO_ALIGN_CW90_DEG:
                q_rotation.w = 0.7071;
                q_rotation.z = 0.7071;
                v_rotation.w = 0.7071;
                v_rotation.z = 0.7071;
                break;

            case GYRO_ALIGN_CW180_DEG:
                q_rotation.z = 1;
                v_rotation.z = 1;
                break;

            case GYRO_ALIGN_CW270_DEG:
                q_rotation.w = 0.7071;
                q_rotation.z = -0.7071;
                v_rotation.w = 0.7071;
                v_rotation.z = -0.7071;
                break;

            case GYRO_ALIGN_CW0_DEG_FLIP:
                v_rotation.x = 1;
                q_rotation.x = 1;
                break;

            case GYRO_ALIGN_CW180_DEG_FLIP:
                // TODO
            case GYRO_ALIGN_CW90_DEG_FLIP:
                // TODO
            case GYRO_ALIGN_CW270_DEG_FLIP:
                // TODO
            default: ;
            }

            gyro.v_gyro = gyro.v_gyro.getRotated(&v_rotation);
            gyro.q = gyro.q.getProduct(q_rotation);
        }

        gyro.f_gyro[0] = gyro.v_gyro.x * gscale; // Roll
        gyro.f_gyro[1] = gyro.v_gyro.y * gscale; // Pitch
        gyro.f_gyro[2] = gyro.v_gyro.z * gscale; // Yaw
        mpu.dmpGetEuler(gyro.euler, &gyro.q);
        mpu.dmpGetGravity(&gyro.gravity, &gyro.q);
        dmpGetYawPitchRoll(gyro.ypr, &gyro.q, &gyro.gravity);
    }
    else
    {
        return false;
    }

    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats();
    #endif

        // unsigned long time_since_update = micros() - last_gyro_update;
        last_gyro_update = micros();
    return true;
}
#endif
