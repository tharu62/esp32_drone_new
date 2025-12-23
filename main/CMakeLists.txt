idf_component_register(SRCS "main.c" "angle_controller.c" "kalman_filter.c" "motor_controller.c" "mpu6050.c" "rotation_rate_controller.c" "state.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c)
