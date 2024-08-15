#include "kinematics.h"

Kinematics::Kinematics(float wheel_radius, float wheel_base_width) 
    : wheel_radius(wheel_radius), wheel_base_width(wheel_base_width) {}

WheelSpeeds Kinematics::inverseKinematics(float linear_velocity, float angular_velocity) {
    WheelSpeeds speeds;
    float v_l = linear_velocity - (angular_velocity * wheel_base_width / 2.0);
    float v_r = linear_velocity + (angular_velocity * wheel_base_width / 2.0);
    
    speeds.front_left = v_l / wheel_radius;
    speeds.back_left = v_l / wheel_radius;
    speeds.front_right = v_r / wheel_radius;
    speeds.back_right = v_r / wheel_radius;

    return speeds;
}

void Kinematics::forwardKinematics(const WheelSpeeds &wheel_speeds, float &linear_velocity, float &angular_velocity) {
    float v_l = (wheel_speeds.front_left + wheel_speeds.back_left) / 2.0;
    float v_r = (wheel_speeds.front_right + wheel_speeds.back_right) / 2.0;

    linear_velocity = wheel_radius * (v_l + v_r) / 2.0;
    angular_velocity = wheel_radius * (v_r - v_l) / wheel_base_width;
}
