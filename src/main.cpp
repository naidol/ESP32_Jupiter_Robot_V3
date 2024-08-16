//#######################################################################################################
// Name:             main.cpp
// Purpose:          Jupiter Robot ESP32 firmware
// Description:      Robot motor drivers, IMU, LED are controlled using PID and communicates to Host PC
//                   using Micro-ROS.  This firmware reads cmd_vel msgs from ROS2 host and publishes
//                   imu/data and odom/unfiltered msgs back to the host so that ROS2 Navigation can compute
//                   the robots position and orientation and determine velocity feedback to the ESP32
//                   Also included are other modules that drive the attached OLED display and Onboard LED
//                   to indicate when the Robot is listening to voice commands.
// Related Files:    this firmware is built to compile on VS CODE using the PLATFORMIO plugin     
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include <Arduino.h>                            // needed if using Platformio and VS Code IDE
#include <micro_ros_platformio.h>               // use if using platformio, otherwise #include <miro_ros_arduino.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <utility/imumaths.h>
#include <nav_msgs/msg/odometry.h>

// Include the local header files
#include "jupiter_config.h"
#include "imu_bno055.h"
#include "encoder.h"
#include "kinematics.h"
#include "odometry.h"
#include "pid_controller.h"


// ROS-related variables
rcl_publisher_t odom_publisher;

// Declare the cmd_vel subscriber and twist message variable
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Declare the IMU publisher
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;  

// Declare the ROS & micro-ROS node interfaces
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// ----------------- NEW GPT CODE ----------------------
// Instances for each wheel
Encoder front_left_encoder(FRONT_LEFT_ENCODER_PIN_A, FRONT_LEFT_ENCODER_PIN_B);
Encoder front_right_encoder(FRONT_RIGHT_ENCODER_PIN_A, FRONT_RIGHT_ENCODER_PIN_B);
Encoder back_left_encoder(BACK_LEFT_ENCODER_PIN_A, BACK_LEFT_ENCODER_PIN_B);
Encoder back_right_encoder(BACK_RIGHT_ENCODER_PIN_A, BACK_RIGHT_ENCODER_PIN_B);

Kinematics kinematics(WHEEL_RADIUS, WHEEL_BASE_WIDTH);
Odometry odometry(WHEEL_RADIUS, WHEEL_BASE_WIDTH);

PIDController pid_front_left(PID_KP, PID_KI, PID_KD);
PIDController pid_front_right(PID_KP, PID_KI, PID_KD);
PIDController pid_back_left(PID_KP, PID_KI, PID_KD);
PIDController pid_back_right(PID_KP, PID_KI, PID_KD);

float target_linear_velocity = 0;
float target_angular_velocity = 0;

void cmdVelCallback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    target_linear_velocity = msg->linear.x;
    target_angular_velocity = msg->angular.z;

    // Debug print
    // Serial.print("Received cmd_vel: linear=");
    // Serial.print(target_linear_velocity);
    // Serial.print(", angular=");
    // Serial.println(target_angular_velocity);
}

void setMotor(int pwm_channel, int dir_channel, float output) {
    if (output > 0) {
        ledcWrite(dir_channel, 255);  // Set direction forward
    } else {
        ledcWrite(dir_channel, 0);    // Set direction backward
        output = -output;             // Make output positive
    }
    ledcWrite(pwm_channel, constrain(output, 0, 255));
}

void stopMotors() {
    // Set all motor PWM outputs to zero to stop the motors
    ledcWrite(0, 0); // Front Left Motor PWM
    ledcWrite(1, 0); // Front Right Motor PWM
    ledcWrite(2, 0); // Back Left Motor PWM
    ledcWrite(3, 0); // Back Right Motor PWM

    // Set all direction PWM outputs to zero to ensure full stop
    ledcWrite(4, 0); // Front Left Direction
    ledcWrite(5, 0); // Front Right Direction
    ledcWrite(6, 0); // Back Left Direction
    ledcWrite(7, 0); // Back Right Direction
}

// -----------------END NEW GPT CODE -------------------

void timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    
    // Get & Publish the IMU message
    get_imu_data(&imu_msg);
    rcl_ret_t ret_imu_ok = rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);

    // ------------------ NEW GPT CODE -------------------------
    // Initialize PWM channels for motor control
    ledcSetup(0, 5000, 8);  // Channel 0, 5 KHz, 8-bit resolution
    ledcSetup(1, 5000, 8);
    ledcSetup(2, 5000, 8);
    ledcSetup(3, 5000, 8);

    // Initialize PWM channels for direction control
    ledcSetup(4, 5000, 8);  // Channel 4 for Front Left Direction
    ledcSetup(5, 5000, 8);  // Channel 5 for Front Right Direction
    ledcSetup(6, 5000, 8);  // Channel 6 for Back Left Direction
    ledcSetup(7, 5000, 8);  // Channel 7 for Back Right Direction

    
    // Attach the PWM channels to the motor pins
    ledcAttachPin(FRONT_LEFT_MOTOR_PIN_A, 0);
    ledcAttachPin(FRONT_RIGHT_MOTOR_PIN_A, 1);
    ledcAttachPin(BACK_LEFT_MOTOR_PIN_A, 2);
    ledcAttachPin(BACK_RIGHT_MOTOR_PIN_A, 3);

    // Attach the PWM channels to the direction pins
    ledcAttachPin(FRONT_LEFT_MOTOR_PIN_B, 4);
    ledcAttachPin(FRONT_RIGHT_MOTOR_PIN_B, 5);
    ledcAttachPin(BACK_LEFT_MOTOR_PIN_B, 6);
    ledcAttachPin(BACK_RIGHT_MOTOR_PIN_B, 7);

    // Set initial PWM values to zero to prevent motors from moving at startup
    ledcWrite(0, 0);  // Front Left Motor PWM
    ledcWrite(1, 0);  // Front Right Motor PWM
    ledcWrite(2, 0);  // Back Left Motor PWM
    ledcWrite(3, 0);  // Back Right Motor PWM

    ledcWrite(4, 0);  // Front Left Direction PWM
    ledcWrite(5, 0);  // Front Right Direction PWM
    ledcWrite(6, 0);  // Back Left Direction PWM
    ledcWrite(7, 0);  // Back Right Direction PWM
    // ------------------END NEW GPT CODE ----------------------

    // Set Micro-ROS transport
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // Create init_options and support
    rclc_support_init(&support, 0, NULL, &allocator);

    // ---------------------  NEW GPT CODE -------------------------------
    rcl_node_t node = rcl_get_zero_initialized_node();
    // ---------------------- END NEW GPT CODE ---------------------------

    // Create node
    rclc_node_init_default(&node, "esp32_node", "", &support);


    // Create IMU publisher
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");

    // Create cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // Create odometry publisher
    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom/unfiltered");
    
    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100), // Publish IMU and any other callback data every 100 ms
        timerCallback);


    // ---------------------  NEW GPT CODE -------------------------------
    // executor = rclc_executor_get_zero_initialized_executor();
    // ---------------------- END NEW GPT CODE ---------------------------

    // Create executor which only handles timer and subscriber callbacks
    rclc_executor_init(&executor, &support.context, 2, &allocator);  // adjust the number of handles for each callback added
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);

    // Initialise (setup) IMU and OLED
    setup_imu(&imu_msg);
    setup_oled_display();

    // Set up LED pin
    pinMode(ESP32_LED, OUTPUT);
    digitalWrite(ESP32_LED, LOW);

    
}

void loop() {
    float dt = 1.0 / CONTROL_LOOP_HZ;

    // Spin the ROS 2 executor to handle callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    // Stop motors if both target velocities are zero
    if (target_linear_velocity == 0 && target_angular_velocity == 0) {
        stopMotors();
    } else {
        // Update encoders
        front_left_encoder.update();
        front_right_encoder.update();
        back_left_encoder.update();
        back_right_encoder.update();

        // Calculate wheel velocities
        float fl_velocity = front_left_encoder.getWheelVelocity(WHEEL_RADIUS, dt);
        float fr_velocity = front_right_encoder.getWheelVelocity(WHEEL_RADIUS, dt);
        float bl_velocity = back_left_encoder.getWheelVelocity(WHEEL_RADIUS, dt);
        float br_velocity = back_right_encoder.getWheelVelocity(WHEEL_RADIUS, dt);

        // Update odometry
        odometry.update(fl_velocity, fr_velocity, bl_velocity, br_velocity, dt);
        // Get the odometry message
        nav_msgs__msg__Odometry odom_msg = odometry.getOdometryMsg();

        // Publish the Odometry message
        rcl_ret_t ret_odom_ok = rcl_publish(&odom_publisher, &odom_msg, NULL);

        // Calculate PID outputs
        WheelSpeeds target_speeds = kinematics.inverseKinematics(target_linear_velocity, target_angular_velocity);
        float fl_output = pid_front_left.calculate(target_speeds.front_left, fl_velocity, dt);
        float fr_output = pid_front_right.calculate(target_speeds.front_right, fr_velocity, dt);
        float bl_output = pid_back_left.calculate(target_speeds.back_left, bl_velocity, dt);
        float br_output = pid_back_right.calculate(target_speeds.back_right, br_velocity, dt);

        // Drive motors using PWM and direction control
        setMotor(0, 4, fl_output);  // Front Left Motor
        setMotor(1, 5, fr_output);  // Front Right Motor
        setMotor(2, 6, bl_output);  // Back Left Motor
        setMotor(3, 7, br_output);  // Back Right Motor
    }

    delay(1000 / CONTROL_LOOP_HZ);
}