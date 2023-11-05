#pragma once

#include "stdint.h"
#include "gyro.h"

// See betaflight src/main/drivers/accgyro/accgyro_mpu6050.c

class Gyro;

class GyroDevice
{
    public:
        virtual void initialize();
        virtual uint8_t start(bool calibrate);
        virtual uint8_t event();
        virtual bool read();
        virtual void calibrate();
};

class GyroDevMPU6050 : public GyroDevice
{
    using GyroDevice::GyroDevice;
    public:
        void initialize();
        uint8_t start(bool calibrate);
        uint8_t event();
        bool read();
        void calibrate();
    private:
        void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
        #ifdef DEBUG_GYRO_STATS
        void print_gyro_stats();
        unsigned long last_gyro_stats_time;
        #endif
};
