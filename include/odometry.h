#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "kinematics.h"
#include <nav_msgs/msg/odometry.h>

class Odometry {
public:
    // Constructor
    Odometry(float wheel_radius, float wheel_base_width);

    // Update function that calculates the robot's position and orientation
    void update(float front_left_velocity, float front_right_velocity, float back_left_velocity, float back_right_velocity, float dt);

    // Function to get the current pose
    void getPose(float &x_out, float &y_out, float &theta_out);

    // Function to return the constructed Odometry message for publishing
    nav_msgs__msg__Odometry getOdometryMsg() const;

private:
    float x;     // X position
    float y;     // Y position
    float theta; // Orientation (yaw)

    Kinematics kinematics; // Kinematics object for calculating velocities

    // Odometry message to be populated and published
    nav_msgs__msg__Odometry odometry_msg;
};

// Helper function to convert yaw (theta) to quaternion
void yawToQuaternion(float yaw, geometry_msgs__msg__Quaternion &q);

#endif // ODOMETRY_H



// -------------------------------------Original Code -----------------------------------------
// #ifndef ODOMETRY_H
// #define ODOMETRY_H

// #include "kinematics.h"

// class Odometry {
// public:
//     Odometry(float wheel_radius, float wheel_base_width);
//     void update(float front_left_velocity, float front_right_velocity, float back_left_velocity, float back_right_velocity, float dt);
//     void getPose(float &x, float &y, float &theta);

// private:
//     float x, y, theta;
//     Kinematics kinematics;
// };

// #endif // ODOMETRY_H
