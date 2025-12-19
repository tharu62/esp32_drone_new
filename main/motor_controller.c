#include "motor_controller.h"

float throttle_input = 0.0f;
float roll_input = 0.0f;
float pitch_input = 0.0f;
float yaw_input = 0.0f;

void motor_control_init(void)
{
    throttle_input = 0.0f;
    roll_input = 0.0f;
    pitch_input = 0.0f;
    yaw_input = 0.0f;
    return;
}

void motor_control(float throttle, float roll, float pitch, float yaw)
{
    float motor1_pwm = throttle + pitch + roll - yaw; // Front Left
    float motor2_pwm = throttle + pitch - roll + yaw; // Front Right
    float motor3_pwm = throttle - pitch - roll - yaw; // Rear Right
    float motor4_pwm = throttle - pitch + roll + yaw; // Rear Left
    return;
}

