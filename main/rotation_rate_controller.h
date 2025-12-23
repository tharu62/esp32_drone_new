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

#include "state.h"

//Initialization code for the rotation rate controller
void rotation_rate_controller_init(void);

void rotation_rate_pid_controller(float* desired_rotation_rate, State* drone_state, float dt, float* rotation_rate_output);

#endif // ROTATION_RATE_CONTROLLER_H