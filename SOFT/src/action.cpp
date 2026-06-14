#include "action.h"
// ════════════════════════════════════════════════════════════════
//  begin()
// ════════════════════════════════════════════════════════════════
void    Actuator::begin()
{
    _serial->begin(115200); 
    _servo.pSerial = _serial; 
    pinMode(PIN_GENERAL_GPIO_1,OUTPUT); // ENABLE
    pinMode(PIN_GENERAL_GPIO_8,OUTPUT); // IN 1  
    pinMode(PIN_GENERAL_GPIO_3,OUTPUT); // IN 2 
    pinMode(PIN_GENERAL_GPIO_2,OUTPUT); // IN 3
    pinMode(PIN_GENERAL_GPIO_6,OUTPUT); // IN 4
    digitalWrite(PIN_GENERAL_GPIO_1, HIGH);
    digitalWrite(PIN_GENERAL_GPIO_8, LOW);
    digitalWrite(PIN_GENERAL_GPIO_3, LOW);
    digitalWrite(PIN_GENERAL_GPIO_2, LOW);
    digitalWrite(PIN_GENERAL_GPIO_6, LOW);
    setPoseBras(700);
    delay(500);
    setPoseMain(730);
    delay(500);
    _state = State::AVAILABLE;
}
// ════════════════════════════════════════════════════════════════
//  setPoseBras()
// ════════════════════════════════════════════════════════════════
void  Actuator::setPoseBras(uint16_t pos)
{
    _posBrasCmd = pos;
    _servo.WritePos(SCS_ID_BRAS, _posBrasCmd, SCS_TRAVEL_TIME);
}
// ════════════════════════════════════════════════════════════════
//  setPoseMain()
// ════════════════════════════════════════════════════════════════
void  Actuator::setPoseMain(uint16_t pos)
{
    _posMainCmd = pos;  
    _servo.WritePos(SCS_ID_MAIN, _posMainCmd, SCS_TRAVEL_TIME);
}
// ════════════════════════════════════════════════════════════════
//  takeLow()
// ════════════════════════════════════════════════════════════════
void  Actuator::takeLow()
{

}
// ════════════════════════════════════════════════════════════════
//  takeHigh()
// ════════════════════════════════════════════════════════════════
void  Actuator::takeHigh()
{

}
// ════════════════════════════════════════════════════════════════
//  depose()
// ════════════════════════════════════════════════════════════════
void  Actuator::depose()
{

}
// ════════════════════════════════════════════════════════════════
//  turn()
// ════════════════════════════════════════════════════════════════
void  Actuator::turn()
{

}
// ════════════════════════════════════════════════════════════════
//  teleplot()
// ════════════════════════════════════════════════════════════════
void  Actuator::teleplot()
{
    
}
// ════════════════════════════════════════════════════════════════
//  dance()
// ════════════════════════════════════════════════════════════════
void Actuator::dance()
{
    if(_servoTimer > 1000)
    {
        if(_servoState)
        {
            setPoseBras(700);
        }
        else
        {
            setPoseBras(600);
        }
        _servoState = !_servoState;
        _servoTimer = 0;
    }
}