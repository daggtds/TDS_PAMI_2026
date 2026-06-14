#pragma once
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
#include <SCServo.h>
//-----------------------------------------------------------------------------
class Actuator 
{
    public:
        Actuator(HardwareSerial *serial)
        {
            _serial = serial;
        };
        
        void        begin();
        void        setPoseBras(uint16_t pos);
        void        setPoseMain(uint16_t pos);
        void        setPower(uint8_t power) {_power = power;};
        void        activatePump()      {analogWrite(PIN_GENERAL_GPIO_2,_power);};
        void        desactivatePump()   {digitalWrite(PIN_GENERAL_GPIO_2,LOW);};
        void        activateVane()      {digitalWrite(PIN_GENERAL_GPIO_3,HIGH);};
        void        desactivateVane()   {digitalWrite(PIN_GENERAL_GPIO_3,LOW);};
        void        takeLow();
        void        takeHigh();
        void        depose();
        void        turn();
        void        teleplot();   
        void        dance();            

private:
        enum class State : uint8_t
        {
            AVAILABLE,
            WAIT
        };

        elapsedMillis       _servoTimer = 0;
        HardwareSerial*     _serial;
        SCServo             _servo;
        bool                _servoState = false;
        uint16_t            _posBrasCmd = 500;
        uint16_t            _posMainCmd = 500;
        bool                _pumpState = false;
        bool                _vaneState = false;
        State               _state = State::AVAILABLE;
        uint8_t             _power = 0;
};

    
