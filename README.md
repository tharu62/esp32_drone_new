| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- |

# DRONE configuration and description

## Overview

This project desribes the configuration and setup of a 4-motor drone runnig with, 
an ESP32 as microcontroller, a MPU6050 as sensor and 4 DC motors controlled 
by hand made motor drivers.  

## How to use example

/...

### Hardware Required

To run this project, you should have an Espressif development board based on a chip
 listed in supported targets as well as a MPU6050. MPU6050 is a inertial measurement 
 unit, which contains a accelerometer, gyroscope as well as a temperature sensor, 
 for more information about it, you can read the **MPU-6000.pdf** .

#### Pin Assignment

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| MPU6050 Sensor   | SDA            | SCL            |

For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note:** There's no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash
I (328) example: I2C initialized successfully
I (338) example: WHO_AM_I = 71
I (338) example: I2C de-initialized successfully
```
