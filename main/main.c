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
#include "mpu6050.c"
#include "motor.h"
#include "pid_controller.h"

static const char *TAG = "drone";

/**
 * @brief Main application
 */
void app_main(void)
{
    uint8_t data[10];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    mpu6050_setup(dev_handle, data);
    mpu6050_calibration(dev_handle, data);

    // Read accelerometer and gyroscope data in a loop
    while(true){

        /************* ACCELEROMETER *********/
        ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_ACCELEROMETER_DATA_REG_ADDR, data, 6));
        int16_t RAWX = (data[0]<<8) | data[1];
        int16_t RAWY = (data[2]<<8) | data[3];
        int16_t RAWZ = (data[4]<<8) | data[5];
        // UPDATED FOR ±8g RANGE  → 4096 LSB/g
        float xg = (float) RAWX / 4096.0f;
        float yg = (float) RAWY / 4096.0f;
        float zg = (float) RAWZ / 4096.0f;
        ESP_LOGI("acceleration : ", "x=%f y=%f z=%f", xg, yg, zg);

        /************* GYROSCOPE *************/
        ESP_ERROR_CHECK(mpu6050_register_read(dev_handle, MPU6050_GYROSCOPE_DATA_REG_ADDR, data, 6));
        int16_t GYRO_RAWX = (data[0] << 8) | data[1];
        int16_t GYRO_RAWY = (data[2] << 8) | data[3];
        int16_t GYRO_RAWZ = (data[4] << 8) | data[5];
        float gyroX = (float) GYRO_RAWX / 131.0f - ROLL_CALIBRATION_OFFSET;
        float gyroY = (float) GYRO_RAWY / 131.0f - PITCH_CALIBRATION_OFFSET;
        float gyroZ = (float) GYRO_RAWZ / 131.0f - YAW_CALIBRATION_OFFSET;
        ESP_LOGI("gyroscope : ", "x=%f y=%f z=%f [deg/s]", gyroX, gyroY, gyroZ);

        //if(???) break;
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    // Cleanup
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT)); /* Resetting the MPU6050 */
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}


