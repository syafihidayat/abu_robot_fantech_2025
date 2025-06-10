#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU
{
private:
    Adafruit_BNO055 bno;
    uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
    double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
    double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;

public:
    struct imu
    {
        float x;
        float y;
        float z;
    };
    IMU();

    bool startSensor();

    imu readAccelerometer();

    imu readGyroscope();
    imu readOrientation();
};

#endif