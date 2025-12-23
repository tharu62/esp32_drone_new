#include "angle_controller.h"

#define P_ROLL_ANGLE 0.6f
#define I_ROLL_ANGLE 3.5f
#define D_ROLL_ANGLE 0.03f
#define P_PITCH_ANGLE 0.6f
#define I_PITCH_ANGLE 3.5f
#define D_PITCH_ANGLE 0.03f
#define P_YAW_ANGLE 2.0f
#define I_YAW_ANGLE 12.0f
#define D_YAW_ANGLE 0.0f


float previous_roll_angle_error = 0.0f;
float previous_pitch_angle_error = 0.0f;
float previous_yaw_angle_error = 0.0f;

float roll_angle_integral = 0.0f;
float pitch_angle_integral = 0.0f;
float yaw_angle_integral = 0.0f;


void angle_controller_init(void) 
{
    previous_roll_angle_error = 0.0f;
    previous_pitch_angle_error = 0.0f;
    previous_yaw_angle_error = 0.0f;
    roll_angle_integral = 0.0f;
    pitch_angle_integral = 0.0f;
    yaw_angle_integral = 0.0f;
    return;
}

void angle_pid_controller(float* desired_angles, State* drone_state, float dt, float* desired_rotation_rate)
{
    float roll_angle_error = desired_angles[0] - drone_state->m_angle[0];
    float pitch_angle_error = desired_angles[1] - drone_state->m_angle[1];
    roll_angle_integral +=  roll_angle_error * dt;
    pitch_angle_integral +=  pitch_angle_error * dt;
    
    float roll_angle_derivative = (roll_angle_error - previous_roll_angle_error) / dt;
    float pitch_angle_derivative = (pitch_angle_error - previous_pitch_angle_error) / dt;
    
    desired_rotation_rate[0] = P_ROLL_ANGLE * roll_angle_error + I_ROLL_ANGLE * roll_angle_integral + D_ROLL_ANGLE * roll_angle_derivative;
    desired_rotation_rate[1] = P_PITCH_ANGLE * pitch_angle_error + I_PITCH_ANGLE * pitch_angle_integral + D_PITCH_ANGLE * pitch_angle_derivative;
    
    previous_roll_angle_error = roll_angle_error;
    previous_pitch_angle_error = pitch_angle_error;
    return;
}