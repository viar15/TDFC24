#pragma once
#include <../lib/Math/Filter/DerivativeFilter.h>
#include <Arduino.h>
#include <bno055.h>

#include "Kalman.h"
#include "delay_millis.h"
#include "drivers/ms5611.h"

// constant
#define ISA_LAPSE_RATE 0.0065f
#define ISA_GAS_CONSTANT 287.26f
#define C_TO_KELVIN 273.15f
// Standard Sea Level values
// Ref: https://en.wikipedia.org/wiki/Standard_sea_level
#define SSL_AIR_DENSITY 1.225f          // kg/m^3
#define SSL_AIR_PRESSURE 101325.01576f  // Pascal
#define SSL_AIR_TEMPERATURE 288.15f     // K

float _last_altitude_EAS2TAS;

float _last_alt;
float _EAS2TAS;
volatile float estimated_airspeed;
double referencePress = 101325;
float new_sea_pressure = 100020.92;
volatile int32_t temperature;
volatile float altitude, pressure;
float last_press, last_temp, last_press_filtered, last_temp_filtered;
float press_filtered;
float temp_filtered;
double rp;
uint32_t tt0 = 0;
uint32_t tt1 = 0;
DerivativeFilterFloat_Size7 _climb_rate_filter;
bool lowpass_ok = 0;
float initial_alt = 0.5;

// double _referencePress = 101325

// LPF for discrete
struct LowPass {
    float alpha;
    float y_last;
};

void lowpass_init(struct LowPass *lp, float f_s, float f_c) {
    lp->alpha = f_c / (f_s + f_c);
    lp->y_last = 0.0f;
}
float lowpass_update(struct LowPass *lp, const float x) {
    lp->y_last = (1.0f - lp->alpha) * lp->y_last + lp->alpha * x;
    return lp->y_last;
}
void lowpass_reset(struct LowPass *lp) {
    lp->y_last = 0.0f;
}

struct Butterworth {};

struct LowPass pressure_lpf;
struct LowPass temperature_lpf;

void CalculateTemperature(bool compensation = false) {
    uint32_t D2 = RawTemperature;
    int32_t dT = D2 - (uint32_t)C[4] * 256;
    int32_t TEMP = 2000 + ((int64_t)dT * C[5]) / 8388608;
    TEMP2 = 0;
    if (compensation) {  // datasheet page 8-9
        if (TEMP < 2000) {
            TEMP2 = (dT * dT) / (2 << 30);
        }
    }
    TEMP -= TEMP2;
    temperature = ((double)TEMP / 100);
}

void readPressure(bool compensation = false) {
    uint32_t D1 = RawPressure;
    // readRawPressure(&D1);
    uint32_t D2 = RawTemperature;
    // readRawTemperature(&D2);
    int32_t dT = D2 - (uint32_t)C[4] * 256;
    int64_t OFF = (int64_t)C[1] * 65536 + (int64_t)C[3] * dT / 128;
    int64_t SENS = (int64_t)C[0] * 32768 + (int64_t)C[2] * dT / 256;
    if (compensation) {
        int32_t TEMP = 2000 + ((int64_t)dT * C[5]) / 8388608;
        OFF2 = 0;
        SENS2 = 0;
        if (TEMP < 2000) {
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        }
        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }
    pressure = (D1 * SENS / 2097152 - OFF) / 32768;
}

double getAltitude(double _temperature, double _pressure, double seaLevelPressure) {
    const double R = 8.31432;    // gas constant in J/(mol·K)
    const double M = 0.0289644;  // molar mass of Earth's air in kg/mol
    const double g = 9.80665;    // gravity in m/s^2
    double altitude = (R * _temperature) / (M * g) * log(seaLevelPressure / _pressure);
    return altitude * 10;
}

double getSeaLevel(double _pressure, double _altitude) {
    const double R = 8.31432;    // gas constant in J/(mol·K)
    const double M = 0.0289644;  // molar mass of Earth's air in kg/mol
    const double g = 9.80665;    // gravity in m/s^2
    double seaLevelPressure = _pressure * exp((M * g * _altitude) / (R * 288.15));
    return seaLevelPressure;
}
// validation : https://keisan.casio.com/exec/system/1224575267
float getEAS2TAS() {
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 25.0f) && _EAS2TAS != 0) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }
    float tempK = temperature + C_TO_KELVIN - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / ((float)pressure / (ISA_GAS_CONSTANT * tempK));
    if (eas2tas_squared < 0) {
        return 1.0;
    }
    _EAS2TAS = sqrtf(eas2tas_squared);
    _last_altitude_EAS2TAS = altitude;
    return _EAS2TAS;
}

float get_climb_rate() {
    return _climb_rate_filter.slope() * 1.0e3f;
    return 0;
}

void baro_update() {
    readRaw();
    CalculateTemperature();
    readPressure();

    press_filtered = lowpass_update(&pressure_lpf, pressure);
    temp_filtered = lowpass_update(&temperature_lpf, temperature);

    altitude = getAltitude(temp_filtered, press_filtered, new_sea_pressure);

    // altitude = getAltitude(temperature, pressure, new_sea_pressure);
    // Serial.print(altitude);
    // altitude = kalman_filter(altitude);
    uint32_t ttnow = millis();

    // float delta_alt = altitude - _last_alt;
    // * (ttnow - tt1);
    // tt1 = ttnow;
    // _last_alt = altitude;
    estimated_airspeed = getEAS2TAS();
    // Serial.print(", \t");
    // Serial.print(altitude);
    // Serial.print(", \t");
    // Serial.print(new_sea_pressure);
    // Serial.println(" ");

    _climb_rate_filter.update(altitude, millis());
}

void lowpass_init() {
    while (altitude > 0.99) {
        baro_update();
        Serial.println("Baro is not ready yet");
        Serial.print(altitude);
        Serial2.println("Baro is not ready yet");
        lowpass_ok = 0;
    }
    lowpass_ok = true;
}
void baro_update_thd() {
    while (1) {
        baro_update();
    }
}

void baro_init() {
    ms5611_Init();
    lowpass_init(&pressure_lpf, 100, 6);
    lowpass_init(&temperature_lpf, 100, 4);
    // readRaw();
    // CalculateTemperature();
    // update_raw();
    // threads.addThread(readRawTemperature);

    readPressure();
    baro_update();
    delay(100);
    new_sea_pressure = getSeaLevel(pressure, 0.15);
    delay(100);
    altitude = 5000;
    lowpass_init();
    threads.addThread(baro_update_thd);
    // Serial.print(new_sea_pressure);
    // delay(1000);
}