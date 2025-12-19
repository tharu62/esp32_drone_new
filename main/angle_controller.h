/**
 * 
 */
#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

void angle_controller_init(void);
void angle_pid_controller(float roll_angle_error, float pitch_angle_error, float yaw_angle_error, float dt,
                                    float* roll_output, float* pitch_output);

#endif // ANGLE_CONTROLLER_H