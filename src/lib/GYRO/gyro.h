#pragma once
#if defined(HAS_GYRO)
#include "device.h"
#include "pid.h"
#include "gyro_types.h"
#include <math.h>
#include "helper_3dmath.h"

#define GYRO_DEV_MPU6050

#define GYRO_DEADBAND 2

#define GYRO_US_MIN 988
#define GYRO_US_MID 1500
#define GYRO_US_MAX 2012

class GyroDevice;

extern PID pid_roll;
extern PID pid_pitch;
extern PID pid_yaw;

class Gyro
{
public:
    gyro_mode_t getMode(void);
    void mixer(uint8_t ch, uint16_t *us);
    void send_telemetry();
    bool read_device();
    void tick();

    GyroDevice *dev;

    float gain = 500;
// protected:

    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorInt16 v_gyro;
    VectorFloat gravity; // [x, y, z]            gravity vector
    float f_gyro[3];
    float euler[3];      // [psi, theta, phi]    Euler angle container
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    uint16_t update_rate;
    unsigned long last_update;
    bool initialized;

private:
    gyro_mode_t gyro_mode;
    void detect_gain(uint16_t us);
    void detect_mode(uint16_t us);
    void switch_mode(gyro_mode_t mode);

    unsigned long pid_delay;
};

extern Gyro gyro;

#endif