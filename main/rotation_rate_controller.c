#include "rotation_rate_controller.h"

#define P_ROLL_RATE 0.6f
#define I_ROLL_RATE 3.5f
#define D_ROLL_RATE 0.03f
#define P_PITCH_RATE 0.6f
#define I_PITCH_RATE 3.5f
#define D_PITCH_RATE 0.03f
#define P_YAW_RATE 2.0f
#define I_YAW_RATE 0.0f
#define D_YAW_RATE 0.0f


float previous_roll_rate_error = 0.0f;
float previous_pitch_rate_error = 0.0f;
float previous_yaw_rate_error = 0.0f;

float roll_rate_integral = 0.0f;
float pitch_rate_integral = 0.0f;
float yaw_rate_integral = 0.0f;


void rotation_rate_controller_init(void) 
{
    previous_roll_rate_error = 0.0f;
    previous_pitch_rate_error = 0.0f;
    previous_yaw_rate_error = 0.0f;
    roll_rate_integral = 0.0f;
    pitch_rate_integral = 0.0f;
    yaw_rate_integral = 0.0f;
    return;
}

void rotation_rate_pid_controller(float roll_rate_error, float pitch_rate_error, float yaw_rate_error, float dt,
                                    float* roll_output, float* pitch_output, float* yaw_output)
{
    roll_rate_integral +=  roll_rate_error * dt;
    pitch_rate_integral +=  pitch_rate_error * dt;
    yaw_rate_integral +=  yaw_rate_error * dt;
    
    float roll_rate_derivative = (roll_rate_error - previous_roll_rate_error) / dt;
    float pitch_rate_derivative = (pitch_rate_error - previous_pitch_rate_error) / dt;
    float yaw_rate_derivative = (yaw_rate_error - previous_yaw_rate_error) / dt;
    
    *roll_output = P_ROLL_RATE * roll_rate_error + I_ROLL_RATE * roll_rate_integral + D_ROLL_RATE * roll_rate_derivative;
    *pitch_output = P_PITCH_RATE * pitch_rate_error + I_PITCH_RATE * pitch_rate_integral + D_PITCH_RATE * pitch_rate_derivative;
    *yaw_output = P_YAW_RATE * yaw_rate_error + I_YAW_RATE * yaw_rate_integral + D_YAW_RATE * yaw_rate_derivative;
    
    previous_roll_rate_error = roll_rate_error;
    previous_pitch_rate_error = pitch_rate_error;
    previous_yaw_rate_error = yaw_rate_error;
    return;
}