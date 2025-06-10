#include <imu.h>
IMU::IMU()
{
}

bool IMU::startSensor()
{
    if (!bno.begin())
    {
        return false;
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    return true;
}

IMU::imu IMU::readAccelerometer()
{

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu accel;
    accel.x = event.acceleration.x; // Konversi ke m/s²
    accel.y = event.acceleration.y;
    accel.z = event.acceleration.z;
    return accel;
}
IMU::imu IMU::readGyroscope()
{

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

    imu gyro;
    gyro.x = event.gyro.x; // Data sudah dalam rad/s
    gyro.y = event.gyro.y;
    gyro.z = event.gyro.z * -1;

    return gyro;
}

IMU::imu IMU::readOrientation()
{
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    imu orient;
    orient.x = event.orientation.x;
    orient.y = event.orientation.y;
    orient.z = event.orientation.z;
    return orient;
}