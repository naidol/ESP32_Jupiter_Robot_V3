#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "kinematics.h"

class Odometry {
public:
    Odometry(float wheel_radius, float wheel_base_width);
    void update(float front_left_velocity, float front_right_velocity, float back_left_velocity, float back_right_velocity, float dt);
    void getPose(float &x, float &y, float &theta);

private:
    float x, y, theta;
    Kinematics kinematics;
};

#endif // ODOMETRY_H
