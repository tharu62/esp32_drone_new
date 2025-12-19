/**
 * Control rotation rate of the drone.
 * The controller uses PID control to maintain desired rotation rates.
 * 
 * 
 * Author: Deshan Thraindu Edirisinghe Edirisinghe Mudiyanselage
 * Date: 2025-..-..
 * 
 */
#ifndef ROTATION_RATE_CONTROLLER_H
#define ROTATION_RATE_CONTROLLER_H

//Initialization code for the rotation rate controller
void rotation_rate_controller_init(void);


/**
 * @brief PID controller for rotation rate cntroll.
 * @param roll_rate_error 
 * @param pitch_rate_error 
 * @param yaw_rate_error 
 * @param dt
 * @param roll_output
 * @param pitch_output
 * @param yaw_output 
 */
void rotation_rate_pid_controller(float roll_rate_error, float pitch_rate_error, float yaw_rate_error, float dt,
                                    float* roll_output, float* pitch_output, float* yaw_output);

#endif // ROTATION_RATE_CONTROLLER_H