#pragma once

#include <Arduino.h>
#include "devices/i2c.h"
#include "../baro.h"

#define I2C_ADDRESS_MS4525DO    (0x28)
#define ADDRESS_READ_MR         (0x00)

float true_airspeed;
float qairspeed_pressure;
float filtered_pressure;
uint8_t airspeed_status;
uint16_t dp_raw          = 0;
uint16_t dp_raw2         = 0;
uint16_t dt_raw          = 0;
uint16_t dt_raw2         = 0;
float as_pressure        = 0.0f;
float as_pressure2       = 0.0f;
float as_temperature     = 0.0f;
float as_temperature2    = 0.0f;
float pressureSum     = 0.0f;
uint8_t pressureCount = 0;
uint8_t temperatureCount = 0;
float temperatureSum  = 0.0f;
float temporaryRatio  = 0.0f;
const float psi_range = 1.0f;
const float PSI_to_Pa = 6894.757f;
const float airDensitySeaLevel = 1.225f;
const float airGasConstant = 287.058f;
const float absoluteNullCelcius = -273.15f;
float ratio = 2.0f;
float temperature_c = 0;
float pressure_pa = 0;
float offset = 0;
float airspeed_mps;

void airspeed_calibrate();
void calculateAveragePT();
float calculatePressure(uint16_t dp_raw);
float calculateTemperature(uint16_t dt_raw);
float calculateRatio(float press, float temp);
void ms4525_init();

void airspeed_GetData()
{
  uint32_t currentTime = 0;
  uint32_t previousTime = 0;
  uint8_t data[4] = {0, 0, 0, 0};
  uint8_t data2[4] = {0, 0, 0, 0};
  _I2C2.ReadBytes(I2C_ADDRESS_MS4525DO, ADDRESS_READ_MR, 4, &data[0]);
  _I2C2.ReadBytes(I2C_ADDRESS_MS4525DO, ADDRESS_READ_MR, 4, &data2[0]);
  airspeed_status = (data[0] & 0xC0) >> 6;
  if (airspeed_status == 2 || airspeed_status == 3)
  {
    return;
  }

  // combine MSB and LSB
  dp_raw = (data[0] << 8 ) + data[1];
  dp_raw = dp_raw & 0x3FFF;             // 14-bit raw pressure
  dt_raw = (data[2] << 8 ) + data[3];
  dt_raw = dt_raw & 0x7FF;             // 11-bit raw temperature

  dp_raw2 = (data2[0] << 8 ) + data2[1];
  dp_raw2 = dp_raw2 & 0x3FFF;             // 14-bit raw pressure
  dt_raw2 = (data2[2] << 8 ) + data2[3];
  dt_raw2 = dt_raw2 & 0x7FF;             // 11-bit raw temperature

  if ( dp_raw == 0x3FFF || dp_raw == 0x00 || dt_raw == 0x7FF || dt_raw == 0x00 ||
       dp_raw2 == 0x3FFF || dp_raw2 == 0x00 || dt_raw2 == 0x7FF || dt_raw2 == 0x00 )
  {
    return;
  }
  // get the pressure and temperature from ms4524
  as_pressure   = calculatePressure(dp_raw);
  as_pressure2  = calculatePressure(dp_raw2);
  as_temperature = calculateTemperature(dt_raw);
  as_temperature2 = calculateTemperature(dt_raw2);
  calculateAveragePT();
  // kalman_estimate :
  currentTime = millis();
  // check current ratio every 5 seconds
  if ( (currentTime - previousTime ) >= 5000 )
  {
    previousTime = currentTime;
    temporaryRatio  = calculateRatio(pressure, temperature);//baro
    // update ratio if the changes more than 5%
    if ( temporaryRatio > 1.05f * ratio ||
         temporaryRatio < 0.95f * ratio )
    {
      ratio = temporaryRatio;
    }
  }
  //   // get the healthy status
  // healthy = pressure_pa;
  // if(usekalman)
  // {
  //     kalman._current_estimate;
  // }
}

void ms4525_init()
{
  _I2C2.init();
  // airspeed_calibrate();
}

float calculatePressure(uint16_t dp_raw)
{
  /*
    this equation is an inversion of the equation in the
    pressure transfer function figure on page 4 of the datasheet
    We negate the result so that positive differential pressures
    are generated when the bottom port is used as the static
    port on the pitot and top port is used as the dynamic port
  */
  // this part is confusing
  const float P_max = 1.0f;
  const float P_min = -P_max;

  float diff_press_PSI  = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
  float press  = diff_press_PSI * PSI_to_Pa; // negate to generate positive value
  return press;
}

float calculateTemperature(uint16_t dt_raw)
{
  float temp  = ((200.0f * dt_raw) / 2047) - 50;
  return temp;
}

void calculateAveragePT()
{
  pressureSum += as_pressure + as_pressure2;
  pressureCount += 2;
  temperatureSum += temperature + as_temperature2;
  temperatureCount += 2;
  if (pressureCount > 0)
  {
    pressure_pa = (pressureSum / pressureCount);
    pressure_pa = pressure_pa - offset;
    airspeed_pressure = pressureSum / pressureCount;
    airspeed_pressure -= offset;
  }
  if (temperatureCount > 0)
  {
    temperature_c = temperatureSum / temperatureCount;
  }
  pressureSum = 0;
  pressureCount = 0;
  temperatureSum = 0;
  temperatureCount = 0;
}


float calculateRatio(float press, float temp)
{
  float airDensity;
  // celcius to kelvin
  temp = temp - absoluteNullCelcius;
  // get the air density using ideal gas formula
  airDensity = press / ( airGasConstant * temp ); // get the rho=P/R*T

  return (2.0f / airDensity); // ratio is 2/rho
}