/**
 * Motor control implementation,
 * handles motor speed adjustments based on input commands.
 * 
 */
#ifndef MOTOR_H
#define MOTOR_H

// Initialize the motor controller variables.
void motor_controller_init(void);


/**
 * @brief Control motors based on input commands
 * @param throttle 
 * @param rotation_rate_output
 */
void motor_controller(float throttle, float* rotation_rate_output);

#endif // MOTOR_H