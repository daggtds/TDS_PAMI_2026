#pragma once
//  ----------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
//  ----------------------------------------------------------------------------------------------
#define CLK_TIMING  3   // µs
//  ----------------------------------------------------------------------------------------------
class PAW3805
{
public:
    struct mouseSample
    {
        float dx_mm;
        float dy_mm;
        int16_t raw_dx;
        int16_t raw_dy;
    };

    PAW3805(int sdio_pin, int sclk_pin, int cs_pin, int motion_pin, int mfio_pin)
    {
        _sdio_pin   = sdio_pin;
        _sclk_pin   = sclk_pin;
        _cs_pin     = cs_pin;
        _motion_pin = motion_pin;
        _mfio_pin   = mfio_pin; 
        pinMode(_cs_pin,OUTPUT);
        digitalWrite(_cs_pin,HIGH);
        pinMode(_sdio_pin,OUTPUT);
        digitalWrite(_sdio_pin,HIGH);
        pinMode(_sclk_pin,OUTPUT);
        digitalWrite(_sclk_pin,HIGH);
        pinMode(_motion_pin,INPUT);
        pinMode(_mfio_pin,INPUT);
    };

    bool begin()
    {
        bool init_ok = true;
        setWriteProtect(0x5A);  // Write Protect Disable  
        setConfiguration(0x80); // reset
        delay(100);
        setCPIX(0x7C);          // max resolution
        setCPIY(0x7F);          // max resolution
        setLEDCurrent(0x1F);    // max led power
        setOperationMode(0xA1); // no sleep mode - wake up
        setWriteProtect(0x00);  // Write Protect Enable
        read_DX();
        read_DY();

        init_ok &= (getCPIX() == 0x7C);
        init_ok &= (getCPIY() == 0x7F);
        return init_ok;
    };

    bool update()
    {
        if(_timer < MOUSE_PERIOD_US)    return false;
        _timer -= MOUSE_PERIOD_US;

        if(!digitalRead(_motion_pin))
        {
            if(bitRead(getMotionStatus(),7))
            {
                _sample.raw_dx = read_DX();
                _sample.raw_dy = read_DY();
                _sample.dx_mm = (float)_sample.raw_dx * CPI_TO_MM;
                _sample.dy_mm = (float)_sample.raw_dy * CPI_TO_MM;
                return true;
            }
        }
        return false;
    };

    mouseSample getMove()   {return _sample;};
    
    uint8_t getProductID1()             {return readByte(0x00);};
    uint8_t getProductID2()             {return readByte(0x01);};
    uint8_t getMotionStatus()           {return readByte(0x02);};
    uint8_t getDeltaXLo()               {return readByte(0x03);};
    uint8_t getDeltaYLo()               {return readByte(0x04);};
    uint8_t getOperationMode()          {return readByte(0x05);};
    uint8_t getConfiguration()          {return readByte(0x06);};
    uint8_t getWriteProtect()           {return readByte(0x09);};
    uint8_t getSleep1()                 {return readByte(0x0A);};
    uint8_t getSleep2()                 {return readByte(0x0B);};
    uint8_t getSleep3()                 {return readByte(0x0C);};
    uint8_t getCPIX()                   {return readByte(0x0D);};
    uint8_t getCPIY()                   {return readByte(0x0E);};
    uint8_t getDeltaXHi()               {return readByte(0x11);};
    uint8_t getDeltaYHi()               {return readByte(0x12);};
    uint8_t getIQ()                     {return readByte(0x13);};
    uint8_t getShutter()                {return readByte(0x15);};
    uint8_t getOrientation()            {return readByte(0x19);};
    uint8_t getMFIOConfig()             {return readByte(0x26);};
    uint8_t getLEDCurrent()             {return readByte(0x51);};
    uint8_t getFrameAvg()               {return readByte(0x61);};
    void setOperationMode(uint8_t val)  {writeByte(0x05,val);};
    void setConfiguration(uint8_t val)  {writeByte(0x06,val);};
    void setWriteProtect(uint8_t val)   {writeByte(0x09,val);};
    void setSleep1(uint8_t val)         {writeByte(0x0A,val);};
    void setSleep2(uint8_t val)         {writeByte(0x0B,val);};
    void setSleep3(uint8_t val)         {writeByte(0x0C,val);};
    void setCPIX(uint8_t val)           {writeByte(0x0D,val);};
    void setCPIY(uint8_t val)           {writeByte(0x0E,val);};
    void setDeltaXHi(uint8_t val)       {writeByte(0x11,val);};
    void setDeltaYHi(uint8_t val)       {writeByte(0x12,val);};
    void setOrientation(uint8_t val)    {writeByte(0x19,val);};
    void setMFIOConfig(uint8_t val)     {writeByte(0x26,val);};
    void setLEDCurrent(uint8_t val)     {writeByte(0x51,val);};

private:
    int             _sdio_pin;
    int             _sclk_pin;
    int             _cs_pin;
    int             _motion_pin;
    int             _mfio_pin;
    mouseSample     _sample;
    elapsedMicros   _timer;

    void writeBit(uint8_t d)
    {
        digitalWrite(_sdio_pin,d);
        digitalWrite(_sclk_pin, LOW);
        delayMicroseconds(CLK_TIMING);
        digitalWrite(_sclk_pin, HIGH);
        delayMicroseconds(CLK_TIMING);
    };

    uint8_t readBit()
    {
        uint8_t d = 0;
        digitalWrite(_sclk_pin, LOW);
        delayMicroseconds(CLK_TIMING);
        d = digitalRead(_sdio_pin);
        digitalWrite(_sclk_pin, HIGH);
        delayMicroseconds(CLK_TIMING);
        return(d);
    };
    
    void writeByte(uint8_t address, uint8_t data)
    {
        uint16_t trame = ((address | 0x80) << 8) + data;   // write operation
        
        pinMode(_sdio_pin,OUTPUT);
        digitalWrite(_sdio_pin,HIGH);
        digitalWrite(_cs_pin,LOW);      // begin transmission

        for(uint8_t i=0;i<16;i++)
        {
            writeBit(bitRead(trame,15-i));
        }

        digitalWrite(_cs_pin,HIGH);      // end transmission
    };

    uint8_t readByte(uint8_t address)
    {
        uint8_t add = address & 0x7F;   // read operation
        uint8_t data = 0x00;

        pinMode(_sdio_pin,OUTPUT);
        digitalWrite(_sdio_pin,HIGH);
        digitalWrite(_cs_pin,LOW);      // begin transmission

        for(uint8_t i=0;i<8;i++)
        {
            writeBit(bitRead(add,7-i));
        }

        pinMode(_sdio_pin,INPUT);
        delayMicroseconds(CLK_TIMING);

        for(uint8_t i=0;i<8;i++)
        {
            bitWrite(data,7-i,readBit());
        }

        digitalWrite(_cs_pin,HIGH);      // end transmission
        pinMode(_sdio_pin,OUTPUT);
        return(data);
    };

    int16_t read_DX()   {return((getDeltaXHi() << 8) + getDeltaXLo());};
    int16_t read_DY()   {return((getDeltaYHi() << 8) + getDeltaYLo());};
};
