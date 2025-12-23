#include "motor_controller.h"

float throttle_input = 0.0f;
float roll_input = 0.0f;
float pitch_input = 0.0f;
float yaw_input = 0.0f;

void motor_controller_init(void)
{
    throttle_input = 0.0f;
    roll_input = 0.0f;
    pitch_input = 0.0f;
    yaw_input = 0.0f;
    return;
}

// @todo: Implement motor controller logic
void motor_controller(float throttle, float* rotation_rate_output)
{
    float motor1_pwm = throttle + rotation_rate_output[1] + rotation_rate_output[0] - rotation_rate_output[2]; // Front Left
    float motor2_pwm = throttle + rotation_rate_output[1] - rotation_rate_output[0] + rotation_rate_output[2]; // Front Right
    float motor3_pwm = throttle - rotation_rate_output[1] - rotation_rate_output[0] - rotation_rate_output[2]; // Rear Right
    float motor4_pwm = throttle - rotation_rate_output[1] + rotation_rate_output[0] + rotation_rate_output[2]; // Rear Left
    return;
}

