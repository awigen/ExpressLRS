#pragma once
#include "targets.h"

#if defined(HAS_GYRO)
#include "device.h"
#include "config.h"
#include "pid.h"
#include "gyro_types.h"
#include <math.h>
#include "helper_3dmath.h"

#define GYRO_US_MIN 988
#define GYRO_US_MID 1500
#define GYRO_US_MAX 2012

/**
 * Add some servo jitter feedback to the pilot after the gyro has initialized.
 */
#define GYRO_BOOT_JITTER
#ifdef GYRO_BOOT_JITTER
#define GYRO_BOOT_JITTER_US 45
#define GYRO_BOOT_JITTER_MS 175
#define GYRO_BOOT_JITTER_TIMES 4
#endif

class GyroDevice;

extern PID pid_roll;
extern PID pid_pitch;
extern PID pid_yaw;

extern volatile gyro_event_t gyro_event;

class Gyro
{
public:
    gyro_mode_t getMode(void);
    void mixer(uint8_t ch, uint16_t *us);
    void send_telemetry();
    bool read_device();
    void tick();
    void calibrate();
    void reload();

    GyroDevice *dev = nullptr;

    float gain = 1.0;
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
    float apply_gain(float correction, mix_destination_t channel);

    unsigned long pid_delay;
};

extern Gyro gyro;

// configure PID controllers from LUA gains for each axis with the specified limit
// (typically 1.0). Set a limit to 0.0 to disable PID control on an axis.
void configure_pids(float roll_limit, float pitch_limit, float yaw_limit);

// Helper method to configure a PID controller instance use the rx config values
void configure_pid_gains(PID* pid, const rx_config_gyro_gains_t* gains, float max, float min);
#endif