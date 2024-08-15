#ifndef KINEMATICS_H
#define KINEMATICS_H

struct WheelSpeeds {
    float front_left;
    float front_right;
    float back_left;
    float back_right;
};

class Kinematics {
public:
    Kinematics(float wheel_radius, float wheel_base_width);
    WheelSpeeds inverseKinematics(float linear_velocity, float angular_velocity);
    void forwardKinematics(const WheelSpeeds &wheel_speeds, float &linear_velocity, float &angular_velocity);

private:
    float wheel_radius;
    float wheel_base_width;
};

#endif // KINEMATICS_H
