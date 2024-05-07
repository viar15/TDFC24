// #pragma once
// #include <Arduino.h>
// #include <Wire.h>
// #include <imu.h>

// #include "BNO055_support.h"  //Contains the bridge code between the API and Arduino

// float gxrs_bno;
// float gyrs_bno;
// float gzrs_bno;
// float vertical_velocity;
// float roll_error = -2.81f, pitch_error = 0.94f;

// struct bno055_t myBNO;
// unsigned char accelCalibStatus = 0;  // Variable to hold the calibration status of the Accelerometer
// unsigned char magCalibStatus = 0;    // Variable to hold the calibration status of the Magnetometer
// unsigned char gyroCalibStatus = 0;   // Variable to hold the calibration status of the Gyroscope
// unsigned char sysCalibStatus = 0;    // Variable to hold the calibration status of the System (BNO055's MCU)

// unsigned long last_Time = 0;

// struct bno055_euler myEulerData;
// struct bno055_gyro gyroData;
// struct bno055_accel accelData;

// void bno055_calibration() {
//     if ((millis() - last_Time) >= 200)  // To read calibration status at 5 Hz without using additional timers
//     {
//         last_Time = millis();

//         Serial.print("Time Stamp: ");  // To read out the Time Stamp
//         Serial.println(lastTime);

//         bno055_get_accelcalib_status(&accelCalibStatus);
//         Serial.print("Accelerometer Calibration Status: ");  // To read out the Accelerometer Calibration Status (0-3)
//         Serial.println(accelCalibStatus);

//         bno055_get_magcalib_status(&magCalibStatus);
//         Serial.print("Magnetometer Calibration Status: ");  // To read out the Magnetometer Calibration Status (0-3)
//         Serial.println(magCalibStatus);

//         bno055_get_magcalib_status(&gyroCalibStatus);
//         Serial.print("Gyroscope Calibration Status: ");  // To read out the Gyroscope Calibration Status (0-3)
//         Serial.println(gyroCalibStatus);

//         bno055_get_syscalib_status(&sysCalibStatus);
//         Serial.print("System Calibration Status: ");  // To read out the Magnetometer Calibration Status (0-3)
//         Serial.println(sysCalibStatus);

//         Serial.println();  // To separate between packets
//     }
// }

// void bno055_update() {
//     // while(1){
//     bno055_read_gyro_xyz(&gyroData);
//     gxrs = (float(gyroData.x) / 16.0) * 0.01745329;       // degrees to radians
//     gyrs = -1 * (float(gyroData.y) / 16.0) * 0.01745329;  // degrees to radians
//     gzrs = (float(gyroData.z) / 16.0) * 0.01745329;       // degrees to radians
//     bno055_read_euler_hrp(&myEulerData);
//     roll = (float(myEulerData.r) / 16.00) - roll_error;
//     pitch = (float(myEulerData.p) / 16.00) - pitch_error;
//     yaw = (float(myEulerData.h) / 16.00);
//     float yaw2 = 360 - yaw;
//     if (yaw2 < 0) {
//         yaw2 += 360;
//     }
//     // yaw =yaw2;
//     yaw_sp = yaw - last_yaw;
//     last_yaw = yaw;
//     bno055_read_accel_xyz(&accelData);
//     // vertical_velocity = float(accelData.z) / 1000.0;
//     // }
// }

// void bno055_init() {
//     // Initialize I2C communication
//     Wire2.begin();
//     // Initialization of the BNO055
//     BNO_Init(&myBNO);  // Assigning the structure to hold information about the device
//     // Configuration to NDoF mode
//     bno055_set_operation_mode(OPERATION_MODE_NDOF);
//     delay(1);

//     // bno055_calibration();
//     // threads.addThread(bno055_update);
// }
