#pragma once
/*i2c_driver.h ini digunakan sebagai interface library Wire  
teensy untuk komunikasi sensor i2c seperti imu, baro, airspeed
source : program khageswara lama (drivers/i2c.h & i2c.cpp)
*/

#include <Arduino.h>
#include <Wire.h>

/** 
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 */

class I2C {
public:
    I2C(TwoWire& wireInterface = Wire);
    virtual ~I2C(){}
    void init(int clk = 400000);
	
    uint8_t ReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data );
	uint8_t ReadBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data );
	uint8_t ReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data );
	uint8_t ReadBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data );
	uint8_t ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data );
	uint16_t ReadWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data );
	uint16_t ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data );
	uint32_t ReadWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint32_t *data );

	void WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
	void WriteBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
	void WriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
	void WriteBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
	void WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
	void WriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
	void WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
	void WriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

    void SendByte(uint8_t address, uint8_t data)
    {
        _wire.beginTransmission(address);
        _wire.write(data);
        _wire.endTransmission();
    }

private:
    TwoWire& _wire;
};

I2C::I2C(TwoWire& wireInterface) : _wire(wireInterface) {}

void I2C::init(int clk){
    _wire.begin();
    _wire.setClock(clk);
}

uint8_t I2C::ReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data )
{
    uint8_t byteValue;
    uint8_t count = ReadByte(devAddr, regAddr, &byteValue);
    *data = byteValue & (1 << bitNum);
    return count;
}

uint8_t I2C::ReadBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data )
{
    /*read specific bit*/
}

uint8_t I2C::ReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data )
{
    uint8_t byteValue;
    uint8_t count = ReadByte(devAddr, regAddr, &byteValue);
    uint8_t mask = (1 << length) - 1;
    *data = (byteValue >> bitStart) & mask;
    return count;
}

uint8_t I2C::ReadBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data )
{}

uint8_t I2C::ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data )
{
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.endTransmission(false);
    _wire.requestFrom(devAddr, (uint8_t) 1);
    if (_wire.available())
        data = _wire.read();
    return data;
}

uint16_t I2C::ReadWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data )
{
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.endTransmission(false);
    _wire.requestFrom(devAddr, (uint8_t) 2);
    if (_wire.available() >= 2)
    {
        uint8_t vha = _wire.read();
        uint8_t vla = _wire.read();
        *data = (vha << 8) | vla;
        return 2;
    }
    else
    {
        return 0;
    }
}

uint16_t I2C::ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data )
{
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.endTransmission(false);
    _wire.requestFrom(devAddr, length);
    uint8_t count = 0;
    while (_wire.available()) { //&& count < length
        data[count++] = _wire.read();
    }
    return *data;
}

uint32_t I2C::ReadWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint32_t *data )
{
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.endTransmission(false);
    _wire.requestFrom(devAddr, length);
    if (_wire.available())
    {
        uint8_t vxa = _wire.read();
        uint8_t vha = _wire.read();
        uint8_t vla = _wire.read();
        *data = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;
        return 3;
    }
    else
    {
        return 0;
    }
}

void I2C::WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t byteValue;
    int8_t count = ReadByte(devAddr, regAddr, &byteValue);
    byteValue = (data != 0) ? (byteValue | (1 << bitNum)) : (byteValue & ~(1 << bitNum));
    return WriteByte(devAddr, regAddr, byteValue);
}

void I2C::WriteBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data)
{}

void I2C::WriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t byteValue;
    int8_t count = ReadByte(devAddr, regAddr, &byteValue);
    uint8_t mask = ((1 << length) - 1) << bitStart;
    data <<= bitStart;
    data &= mask;
    byteValue &= ~(mask);
    byteValue |= data;
    return WriteByte(devAddr, regAddr, byteValue);
}

void I2C::WriteBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{}

void I2C::WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.write(data);
    _wire.endTransmission();
}

void I2C::WriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{}

void I2C::WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    for (uint8_t i = 0; i < length; i++) {
        _wire.write(data[i]);
    }
    _wire.endTransmission();
}

void I2C::WriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{}

extern I2C _I2C(Wire);
extern I2C _I2C1(Wire1);
extern I2C _I2C2(Wire2);