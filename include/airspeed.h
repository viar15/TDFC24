#pragma once
#include <Arduino.h>
// #include "simple_i2c.h"
// #include <Wire.h>
#include "drivers/ms4525.h"
#define I2C_ADDRESS_MS4525DO    (0x28)
#define ADDRESS_READ_MR         (0x00)
void airspeed_update();
void calculateAirspeed();
float MAX(float a, float b) {return (a > b) ? a : b;}
void airspeed_calibrate()
{
    float sum;
    for (int i = 0; i < 2000; i++){
        airspeed_update();
        // sum += pressure_pa;
        sum += airspeed_pressure; 
    }
    offset = sum/2000;
}
void airspeed_init(){
    // Wire2.begin();
    ms4525_init();
    airspeed_calibrate();
}
void airspeed_update(){
    airspeed_GetData();
    calculateAirspeed();
    // moving average filter : 
}
void calculateAirspeed()
{
    filtered_pressure = 0.7f * filtered_pressure + 0.3f * airspeed_pressure;
    true_airspeed  = sqrtf(MAX(filtered_pressure,0) * ratio)*0.5f;
    if (isnan(true_airspeed)) {true_airspeed = 0;}
    airspeed_mps = true_airspeed;// neeed filter again
}