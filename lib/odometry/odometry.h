#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry
{
private:
    float x_pos_;
    float y_pos_;
    float heading_;

    const void euler_to_quat(float roll, float pitch, float yaw, float *q);

public:
    struct odom
    {
        float x;
        float y;
        float theta;
    }odom_pose;
    Odometry(/* args */);
    odom getData();
    void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
};

#endif