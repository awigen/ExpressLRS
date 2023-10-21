#pragma once

#include "stdint.h"
#include "gyro.h"

// See betaflight src/main/drivers/accgyro/accgyro_mpu6050.c

class Gyro;

class GyroDevice
{
    public:
        virtual void initialize() = 0;
        virtual uint8_t start() = 0;
        virtual uint8_t event() = 0;
        virtual bool read();
};

class GyroDevMPU6050 : public GyroDevice
{
    using GyroDevice::GyroDevice;
    public:
        void initialize();
        uint8_t start();
        uint8_t event();
        bool read();
    private:
        void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
        #ifdef GYRO_STATS
        void print_gyro_stats();
        unsigned long last_gyro_stats_time;
        #endif
};
