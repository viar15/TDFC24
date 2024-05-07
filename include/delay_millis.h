#pragma once
#ifndef Arduino_h
#include <Arduino.h>
#endif

void delay_millis(uint32_t _delay_time)
{
    uint32_t start_time = millis();
    while (millis() - start_time < _delay_time){/*Waiting . . .*/}
}