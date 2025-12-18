/**
 * @file motor.h
 * 
 * Motor control implementation,
 * handles motor speed adjustments based on input commands.
 * 
 */
#ifndef MOTOR_H
#define MOTOR_H


/**
 * @brief Control motors based on input commands
 */
void motor_control(float throttle, float roll, float pitch, float yaw);

#endif // MOTOR_H