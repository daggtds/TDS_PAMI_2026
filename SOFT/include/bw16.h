#pragma once
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
//-----------------------------------------------------------------------------
class IHM
{
    public:
        IHM(HardwareSerial *_serial)
        {
            pserial = _serial;
        };
        
        void            begin();
        void            update();  
        uint32_t        getTime()       {return _matchTimer;};
        char*           getColor()      {return _teamColor;};   
        int             getID()         {return _idRobot;};
        bool            isReceived()    {return _isReceived;};      

    private:
        HardwareSerial  *pserial;
        elapsedMillis   _ihmTimer   = 0;
        char            _messBW16[MAX_MESS_SIZE];
        uint8_t         _messIdx = 0;
        char            _messTeensy[MAX_MESS_SIZE];
        char            _teamColor[255];
        uint32_t        _matchTimer = 0;
        bool            _isReceived = false;
        int             _idRobot = 0;
        void            receiveMess();       
};