#include <movement_smoother.h>
#include <cmath>
MovementSmoother::MovementSmoother(double max_accel)
{
    max_acceleration_ = max_accel;
}

void MovementSmoother::calculate_smooth_vel(float &current_output, float input_value, double deltaT)
{
    double target = input_value;

    if (input_value == 0.0 && abs(current_output) > 1e-6)
    {
        target = 0.0;
    }
    double max_delta = 10.0 * deltaT;

    double delta_target = target - current_output;

    if (abs(delta_target) > max_delta)
    {
        delta_target = (delta_target > 0.0) ? max_delta : -max_delta;
    }

    current_output += delta_target;
}
