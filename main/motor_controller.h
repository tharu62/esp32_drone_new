/**
 * Motor control implementation,
 * handles motor speed adjustments based on input commands.
 * 
 */
#ifndef MOTOR_H
#define MOTOR_H

// Initialize the motor controller variables.
void motor_control_init(void);


/**
 * @brief Control motors based on input commands
 * @param throttle 
 * @param roll
 * @param pitch
 * @param yaw 
 */
void motor_control(float throttle, float roll, float pitch, float yaw);

#endif // MOTOR_H