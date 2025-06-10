#include <kinematic.h>
Kinematic::Kinematic(base robot_base, int motor_max_rps, float max_rps_ratio,
                     float motor_operating_voltage, float motor_power_max_voltage,
                     float wheel_diameter, float robot_diameter)
    : base_platform_(robot_base),
      wheel_circumference_(PI * wheel_diameter),
      robot_circumference_(PI * robot_diameter),
      total_wheels_(getTotalWheels(robot_base))
{
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rps_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rps) * max_rps_ratio;
}

Kinematic::velocities Kinematic::getVelocities(float rps1, float rps2, float rps3)
{

    Kinematic::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // convert average r    evolutions per minute to revolutions per second
    average_rps_x = ((float)(cos(toRad(30)) * rps1 + cos(toRad(150)) * rps2 + cos(toRad(270)) * rps3)); // rps
    vel.linear_x = average_rps_x * wheel_circumference_;                                                // m/s

    // convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(sin(toRad(30)) * rps1 + sin(toRad(150)) * rps2 + sin(toRad(270)) * rps3)); // rps
    vel.linear_y = average_rps_y * wheel_circumference_;                                                // m/s

    // convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(1 * rps1 + 1 * rps2 + 1 * rps3));
    vel.angular_z = (average_rps_a * robot_circumference_) / (robot_circumference_); //  rad/s

    return vel;
}

Kinematic::rps Kinematic::getRPS(float linear_x, float linear_y, float angular_z, float imu_angular_z)
{
    return calculateRPS(linear_x, linear_y, angular_z, imu_angular_z);
}

float Kinematic::getMaxRPS()
{
    return max_rps_;
}
float Kinematic::toRad(float deg)
{
    return deg * M_PI / 180;
}
float Kinematic::toDeg(float rad)
{
    return rad * 180 / M_PI;
}

Kinematic::rps Kinematic::calculateRPS(float linear_x, float linear_y, float angular_z, float imu_angular_z)
{

    float tangential_vel = angular_z * (robot_circumference_);
    // float tangential_vel = angular_z;

    // convert m/s to m/min
    float linear_vel_x_mins = linear_x;
    float linear_vel_y_mins = linear_y;
    // convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel/ wheel_circumference_;

    float x_mps = linear_vel_x_mins / wheel_circumference_;
    float y_mps = linear_vel_y_mins / wheel_circumference_;
    float tan_mps = tangential_vel_mins / wheel_circumference_;

    float a_x_mps = fabs(x_mps);
    float a_y_mps = fabs(y_mps);
    float a_tan_mps = fabs(tan_mps);

    float xy_sum = a_x_mps + a_y_mps;
    float xtan_sum = a_x_mps + a_tan_mps;
    if (xy_sum >= max_rps_ && angular_z == 0)
    {
        float vel_scaler = max_rps_ / xy_sum;

        x_mps *= vel_scaler;
        y_mps *= vel_scaler;
    }

    else if (xtan_sum >= max_rps_ && linear_y == 0)
    {
        float vel_scaler = max_rps_ / xtan_sum;

        x_mps *= vel_scaler;
        tan_mps *= vel_scaler;
    }

    Kinematic::rps rps;


    // float rps_motor1 = cos(toRad(120 )) * x_mps + cos(toRad(120 )) * y_mps + robot_radius_ * tan_mps;
    float rps_motor1 = (-sin(toRad(120 + imu_angular_z)) * x_mps )+( cos(toRad(120 + imu_angular_z)) * y_mps) + (robot_radius_ * tan_mps);
    rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

    // rear-left motor
    float rps_motor2 = (-sin(toRad(240 + imu_angular_z )) * x_mps) + (cos(toRad(240 + imu_angular_z)) * y_mps) + (robot_radius_ * tan_mps);
    rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

    // rear-right motor
    float rps_motor3 = (-sin(toRad(360 + imu_angular_z )) * x_mps) + (cos(toRad(360 + imu_angular_z)) * y_mps) + (robot_radius_ * tan_mps);
    rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

    // front-right motor
    // float rps_motor4 = -sin(toRad(315 + imu_angular_z)) * x_mps + cos(toRad(315 + imu_angular_z)) * y_mps + robot_radius_ * tan_mps;
    // rps.motor4 = fmax(-max_rps_, fmin(rps_motor4, max_rps_));
    /*
    motor1 = -sin(torad(30)) * x + cos(torad(30)) * y + robot_radius * tan
    motor2 = -sin(torad(150)) * x + cos(torad(150)) * y + robot_radius * tan
    motor3 =
    */
    return rps;
}

int Kinematic::getTotalWheels(base robot_base)
{
    switch (robot_base)
    {
    case DIFFERENTIAL_DRIVE:
        return 2;
    case SKID_STEER:
        return 4;
    case MECANUM:
        return 4;
    case OMNI:
        return 3;
    default:
        return 4;
    }
}