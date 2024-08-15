#include "odometry.h"
#include "math.h"

Odometry::Odometry(float wheel_radius, float wheel_base_width)
    : x(0), y(0), theta(0), kinematics(wheel_radius, wheel_base_width) {}

void Odometry::update(float front_left_velocity, float front_right_velocity, float back_left_velocity, float back_right_velocity, float dt) {
    float linear_velocity, angular_velocity;
    WheelSpeeds speeds = {front_left_velocity, front_right_velocity, back_left_velocity, back_right_velocity};
    kinematics.forwardKinematics(speeds, linear_velocity, angular_velocity);

    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;
    theta += angular_velocity * dt;
}

void Odometry::getPose(float &x_out, float &y_out, float &theta_out) {
    x_out = x;
    y_out = y;
    theta_out = theta;
}
