#include "kalman_filter.h"

#define STANDARD_DEV_ACCEL_NOISE_SQRD 4.0f*4.0f  // Standard deviation squared of acceleration noise (deg)
#define STANDARD_DEV_GYRO_NOISE_SQRD 3.0f*3.0f   // Standard deviation squared of gyro

float k_uncertainty_roll = 2.0f*2.0f;   // Initial estimation uncertainty for roll
float k_uncertainty_pitch = 2.0f*2.0f;  // Initial estimation uncertainty for pitch

float k_gain_roll = 0.0f;   // Kalman gain for roll
float k_gain_pitch = 0.0f;  // Kalman gain for pitch

void reset_kalman_filter() 
{
    k_uncertainty_roll = 2.0f * 2.0f;
    k_uncertainty_pitch = 2.0f * 2.0f;

    k_gain_roll = 0.0f;
    k_gain_pitch = 0.0f;
}

void kalman_filter(State *state, float dt) 
{
    // Predict roll angle by integrating angular velocity
    state->k_angle[0] += state->angular_velocity[0] * dt;  
    state->k_angle[1] += state->angular_velocity[1] * dt;  

    // Update uncertainty from rotation integration
    
    k_uncertainty_roll += dt * dt * STANDARD_DEV_GYRO_NOISE_SQRD; 
    k_uncertainty_pitch += dt * dt * STANDARD_DEV_GYRO_NOISE_SQRD;

    // Calculate Kalman gain for roll and pitch
    k_gain_roll = k_uncertainty_roll / (1.0f*k_uncertainty_roll + STANDARD_DEV_ACCEL_NOISE_SQRD);          
    k_gain_pitch = k_uncertainty_pitch / (1.0f*k_uncertainty_pitch + STANDARD_DEV_ACCEL_NOISE_SQRD);

    // Update roll and pitch angle with measurement
    state->k_angle[0] += k_gain_roll * (state->m_angle[0] - state->k_angle[0]);       
    state->k_angle[1] += k_gain_pitch * (state->m_angle[1] - state->k_angle[1]);      

    // Update uncertainty for roll and pitch
    k_uncertainty_roll *= (1 - k_gain_roll);                
    k_uncertainty_pitch *= (1 - k_gain_pitch);              
}