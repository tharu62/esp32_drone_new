/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* Simple Firmaware for ESP32

   This code initializes the I2C bus and communicates with a MPU6050 sensor
   to read accelerometer and gyroscope data in a loop.

*/

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "state.h"
#include "mpu6050.h"
#include "angle_controller.h"
#include "rotation_rate_controller.h"
#include "motor_controller.h"
#include "kalman_filter.h"

static const char *TAG = "drone";

/**
 * @brief Main application
 */
void app_main(void)
{
    uint8_t data[10]; // Buffer for I2C data, oversized for safety.
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    State drone_state;
    init_state(&drone_state);

    mpu6050_setup(dev_handle, data);
    mpu6050_calibration(dev_handle, data);

    angle_controller_init();
    rotation_rate_controller_init();
    motor_controller_init();
    reset_kalman_filter();

    // // Read accelerometer and gyroscope data in a loop
    // while(true){

    //     /************* ACCELEROMETER *********/
    //     mpu6050_get_angle(dev_handle, data, &drone_state.m_angle[0], &drone_state.m_angle[1]);
    //     ESP_LOGI("angles : ", "x=%f y=%f z=%f [deg]", drone_state.m_angle[0], drone_state.m_angle[1], drone_state.m_angle[2]);
    //     /************* GYROSCOPE *************/
    //     mpu6050_get_rotation_rate(dev_handle, data, &drone_state.angular_velocity[0], &drone_state.angular_velocity[1], &drone_state.angular_velocity[2]);
    //     ESP_LOGI("gyroscope : ", "x=%f y=%f z=%f [deg/s]", drone_state.angular_velocity[0], drone_state.angular_velocity[1], drone_state.angular_velocity[2]);

    //     //if(???) break;
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }



    float desired_angles[3] = {0.0f, 0.0f, 0.0f};           // Placeholder for desired angles (roll, pitch, yaw)
    float throttle = 0.5f;                                  // Placeholder for throttle input
    float desired_rotation_rate[3] = {0.0f, 0.0f, 0.0f};    // Placeholder for desired rotation rates (roll rate, pitch rate, yaw rate)
    float rotation_rate_output[3] = {0.0f, 0.0f, 0.0f};     // Placeholder for rotation rate controller outputs

    float dt = 0.01f; // Time step for control loop (10 ms)
    // Main control loop
    while (true) {
        // Read Input from user (desired angles)
        // @todo: Implement user input reading here

        // Run angle controller pid to get desired rotation rates
        angle_pid_controller(desired_angles, &drone_state, dt, desired_rotation_rate);

        // Run rotation rate controller pid to get motor commands
        rotation_rate_pid_controller(desired_rotation_rate, &drone_state, dt, rotation_rate_output);

        // Update motor speeds by pwm signals
        motor_controller(throttle, rotation_rate_output);

        // Read sensor data
        mpu6050_get_angle(dev_handle, data, &drone_state.m_angle[0], &drone_state.m_angle[1]);
        mpu6050_get_rotation_rate(dev_handle, data, &drone_state.angular_velocity[0], &drone_state.angular_velocity[1], &drone_state.angular_velocity[2]);

        // RUn Kalman filter to update state estimation
        kalman_filter(&drone_state, dt); // Assuming a fixed time step of 10ms

        vTaskDelay(dt / portTICK_PERIOD_MS); // Delay for 10ms
    }

    // Cleanup
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT)); /* Resetting the MPU6050 */
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}


