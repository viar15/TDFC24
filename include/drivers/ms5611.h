#pragma once
#include <Arduino.h>
#include "devices/i2c.h"
#include "delay_millis.h"
#include <TeensyThreads.h>
// #include "mpu6050.h"

//  MS5611 Registers
#define MS5611_ADDRESS                (0x77)
#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

#define MS5611_ULTRA_HIGH_RES   (0x08)
#define MS5611_HIGH_RES         (0x06)
#define MS5611_STANDARD         (0x04)
#define MS5611_LOW_POWER        (0x02)
#define MS5611_ULTRA_LOW_POWER  (0x00)

uint16_t C[6];
uint8_t ct;
uint8_t uosr;
int32_t TEMP2;
int64_t OFF2, SENS2;
volatile uint32_t RawTemperature, RawPressure;

// I2C1 _I2C1(Wire);

void  setOversampling()
{
  ct = 10;
}

void reset(void)
{
  _I2C1.SendByte(MS5611_ADDRESS, MS5611_CMD_RESET);
}

void readPROM(void)
{
  for (uint8_t offset = 0; offset < 6; offset++)
  {
    _I2C1.ReadWord(MS5611_ADDRESS, MS5611_CMD_READ_PROM + (offset * 2), &C[offset]);
  }
}

bool ms5611_Init()
{
  _I2C1.init();
  reset();
  setOversampling();
  delay_millis(100);
  readPROM();
  return true;
}

// uint32_t readRawTemperature()
// {
//     _I2C1.SendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + uosr);
//     delay(ct);
//     uint32_t raw_data;
//     _I2C1.ReadWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_data);//&RawTemperature
//     return raw_data;
// }

// uint32_t readRawPressure()
// {
//     _I2C1.SendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + uosr);
//     delay(ct);
//     uint32_t raw_data;
//     _I2C1.ReadWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_data); //&RawPressure
//     return raw_data;
// }

// void readRawTemperature()
// {
//     while(1){
//     _I2C1.SendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + uosr);
//     threads.delay(ct);
//     uint32_t raw_data;
//     _I2C1.ReadWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_data);//&RawTemperature
//     RawTemperature = raw_data;
//     // Serial.print("@");
//     }
// }
void readRaw()
{
  // while(true){
  // mpu6050_get_data();
  _I2C1.SendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + uosr);
  threads.delay(ct);
  uint32_t raw_data;
  _I2C1.ReadWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_data);//&RawTemperature
  RawTemperature = raw_data;
  // return raw_data;
  _I2C1.SendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + uosr);
  threads.delay(ct);
  uint32_t raw_dataP;
  _I2C1.ReadWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_dataP); //&RawPressure
  RawPressure =  raw_dataP;
  // Serial.print("@");
  // }
}

uint32_t update_raw()
{
  // RawTemperature = readRawTemperature();
  // RawPressure = readRawPressure();
}