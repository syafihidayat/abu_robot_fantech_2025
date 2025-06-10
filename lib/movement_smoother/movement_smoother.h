#ifndef MOVEMENT_SMOOTHER
#define MOVEMENT_SMOOTHER

class MovementSmoother
{
private:
    double max_acceleration_ = 0;
public:
    struct joystick
    {
        double x = 0.0;
        double y = 0.0;
        double head = 0.0;
    } joy;
    MovementSmoother(double max_accel);

    void calculate_smooth_vel(float &current_output, float input_value, double deltaT);
};

#endif