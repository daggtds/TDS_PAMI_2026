#pragma once
//  ----------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <SCServo.h>
SCServo scs;
//  ----------------------------------------------------------------------------------------------
static constexpr uint32_t   SCS_BAUDRATE    = 115200;
static constexpr uint8_t    SCS_BRAS_ID     = 1;        
static constexpr uint16_t   SCS_POS_MAX     = 1020;
static constexpr uint16_t   SCS_POS_MIN     = 3;
static constexpr int16_t    SCS_SPEED_MAX   = 500;  // ms
static constexpr int16_t    SCS_SPEED_MIN   = 2000; // ms
static constexpr int16_t    SCS_BRAS_HIPOS  = 780;
static constexpr int16_t    SCS_BRAS_LOPOS  = 320;
//  ----------------------------------------------------------------------------------------------
class SCS0009
{
    public:
        SCS0009(HardwareSerial *serial)
        {
            pserial = serial;
        };

        void begin()
        {
            pserial->begin(SCS_BAUDRATE);
            scs.pSerial = pserial;
        };

        void enable(uint8_t id)
        {
            scs.EnableTorque(id, 1);
        };

        void disable(uint8_t id)
        {
            scs.EnableTorque(id, 0);
        };

        void goTo(uint8_t id, uint16_t pos, uint16_t speed)
        {
            pos     = constrain(pos,SCS_POS_MIN,SCS_POS_MAX);
            speed   = constrain(speed,SCS_SPEED_MIN,SCS_SPEED_MAX);
            scs.WritePos(id,pos,speed);
        };

    private:
        HardwareSerial *pserial;
};





