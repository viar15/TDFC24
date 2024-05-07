#include <../lib/TeensyThreads/TeensyThreads.h>
#include <../include/drivers/ms4525.h>
#include <baro.h>
#include <airspeed.h>
// #include <AHRS.h>
// #include <Actuator.h>
#include <Arduino.h>
// #include <Copter_Control.h>
// #include <Radio.h>
// #include <Telemetry.h>
// #include <Transition.h>
#define SERIAL_USB Serial
#define SERIAL_UART Serial2
// #define CALIBRATE_MOTORS false
uint32_t loop_timer = 0, debug_timer = 0;
uint32_t time_now;
uint32_t dt;
uint32_t time_last;
// Telemetry telem;
// Metro control_mc_metro = Metro(40);
// Metro control_fw_metro = Metro(30);
// Metro update_fw_metro = Metro(30);
// Metro update_mc_metro = Metro(40);
// Metro update_trans_metro = Metro(45);

void printUSB() {
    // Serial.print("roll = ");
    // Serial.print(roll);
    // Serial.print(" ");
    // Serial.print("pitch = ");
    // Serial.print(pitch);
    // Serial.print(" ");
    // Serial.print("yaw = ");
    // Serial.print(yaw);
    // Serial.print(" gx = ");
    // Serial.print(gxrs);
    // Serial.print(" gy = ");
    // Serial.print(gyrs);
    // Serial.print(" gz = ");
    // Serial.print(gzrs);
    // Serial.print("yawsp = ");
    // Serial.print(yaw_setpoint);
    // Serial.print(" rawAcX = ");
    // Serial.print(AcX);
    // Serial.print(" rawAcY = ");
    // Serial.print(AcY);
    // Serial.print(" rawAcZ = ");
    // Serial.print(AcZ);
    // Serial.print(" rawGyX = ");
    // Serial.print(GyX);
    // Serial.print(" rawGyY = ");
    // Serial.print(GyY);
    // Serial.print(" rawGyZ = ");
    // Serial.print(GyZ);
    // Serial.print(" alt = ");
    // Serial.print(altitude);

    Serial.print(" as = ");
    Serial.print(true_airspeed);
    Serial.print("    ");
    Serial.print("Alt = ");
    Serial.print(altitude);
    Serial.print("    ");
    Serial.print("Aspeed mps = ");
    Serial.print(airspeed_mps);
    Serial.print("    ");

    Serial.println();
    // Serial.print(" head = ");
    // Serial.print(true_heading);
    // Serial.print(" lat = ");
    // Serial.print(latitude, 7);
    // Serial.print(" lon = ");
    // Serial.print(longitude, 7);
    // Serial.print(" hdop = ");
    // Serial.print(hdop);
    // Serial.print(" sat = ");
    // Serial.print(satellites);
    // Serial.print(" u1 = ");
    // Serial.print(m1_pwm);
    // Serial.print(" u2 = ");
    // Serial.print(m2_pwm);
    // Serial.print(" u3 = ");
    // Serial.print(m3_pwm);
    // Serial.print(" u4 = ");
    // Serial.print(m4_pwm);
    // Serial.print(" phase = ");
    // if (mode_now == 1)
    // {
    //   Serial.print("1");
    // }
    // else if (mode_now == 2)
    // {
    //   Serial.print("2");
    // }
    // else if (mode_now == 3)
    // {
    //   Serial.print("3");
    // }
    // else if (mode_now == 4)
    // {
    //   Serial.print("4");
    // }
    // Serial.println("");
    // Serial.print(" VTOLphase = ");
    // if (transition_phase1)
    // {
    //   Serial.print("0");
    // }
    // else if (transition_phase2)
    // {
    //   Serial.print("1");
    // }
    // else if (transition_phase3)
    // {
    //   Serial.print("2");
    // }
    // Serial.print(" mode = ");
    // if (mode_manual)
    // {
    //   Serial.println("manu");
    // }
    // else if (mode_fbwa_plane)
    // {
    //   Serial.println("fbwa");
    // }
}
// void telemetry_thread() {
//     while (1) {
//         // telem.send_vehicle_attitude_data(); /// senda data protobuf
//         telem.print_log_fw_telem();  /// send data from telem
//         threads.yield();
//     }
// }
// void radio_thread() {
//     while (1) {
//         remote_loop();
//         threads.yield();
//     }
// }
void printUSB_thread() {
    while (1) {
        printUSB();
        threads.yield();
    }
}
// void controlThd() {
//     while (1) {
//         Transition_sequence_manual();
//         threads.yield();
//     }
// }
// void bnoThd() {
//     while (1) {
//         bno055_update();
//         // Transition_sequence_manual();
//         threads.yield();
//     }
// }
// void gps_thread() {
//   while (1)
//   {
//     gpsUpdate();
//     threads.yield();
//   }

// }
// void calibrateMotor() {
//     motor1.writeMicroseconds(2012);
//     motor2.writeMicroseconds(2012);
//     motor3.writeMicroseconds(2012);
//     motor4.writeMicroseconds(2012);
//     // pusher.writeMicroseconds(2000);
//     delay(3000);
//     motor1.writeMicroseconds(988);
//     motor2.writeMicroseconds(988);
//     motor3.writeMicroseconds(988);
//     motor4.writeMicroseconds(988);
//     // pusher.writeMicroseconds(1000);
//     delay(1000);
// }
void setup() {
    SERIAL_USB.begin(115200);
    SERIAL_UART.begin(57600);
    pinMode(29, OUTPUT);
    baro_init();
    airspeed_init();
    // init_actuator();
    // remote_setup();
    // ahrs_init();
    // threads.addThread(telemetry_thread, 1);
    // threads.addThread(radio_thread, 1);
    // threads.addThread(controlThd,1);
    // threads.addThread(bnoThd, 1);
    // threads.addThread(printUSB_thread, 1);
    // if (CALIBRATE_MOTORS) {
    //     calibrateMotor();
    // }
    // MODE = initMODE->mode();
    loop_timer = micros();
}
void loop() {

    if(!lowpass_ok){
        Serial.println("Baro is not ready");
    }else{
        printUSB();
    }
    // ahrs_update();
    // remote_loop();
    // copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, roll, pitch, yaw, gyrs, gxrs, gzrs);
    // copter_calcOutput(ch_throttle);
    // updateFBWA_FW();
    // stabilize();

    // Transition_sequence_manual();

    // Transition_sequence_auto();
}