#pragma once
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include <TMC2209.h>
#include "config.h"
//-----------------------------------------------------------------------------
#define MOTOR_T_ID 0
#define MOTOR_D_ID 1
#define MOTOR_S_ID 2
//-----------------------------------------------------------------------------
struct SpeedVector
{
    float Vx;
    float Vy;
    float Wz;
};
//-----------------------------------------------------------------------------
class Propulser 
{
    public:
        Propulser()
        {
            _motor_T.setHardwareEnablePin(PIN_MOTORS_ENB);
            _motor_T.setup(motT_serial);
            _motor_T.setRunCurrent(MOTOR_CURRENT_PERCENT);
            _motor_T.enableAutomaticCurrentScaling();
            _motor_T.enableAutomaticGradientAdaptation();
            _motor_T.enableCoolStep();
            _motor_T.moveAtVelocity(0);
            _motor_T.disable();
            _motor_T.setMicrostepsPerStep(MOTOR_USTEP_PER_STEP);
                
            _motor_D.setHardwareEnablePin(PIN_MOTORS_ENB);
            _motor_D.setup(motD_serial);
            _motor_D.setRunCurrent(MOTOR_CURRENT_PERCENT);
            _motor_D.enableAutomaticCurrentScaling();
            _motor_D.enableAutomaticGradientAdaptation();
            _motor_D.enableCoolStep();
            _motor_D.moveAtVelocity(0);
            _motor_D.disable();
            _motor_D.setMicrostepsPerStep(MOTOR_USTEP_PER_STEP);
                    
            _motor_S.setHardwareEnablePin(PIN_MOTORS_ENB);
            _motor_S.setup(motS_serial);
            _motor_S.setRunCurrent(MOTOR_CURRENT_PERCENT);
            _motor_S.enableAutomaticCurrentScaling();
            _motor_S.enableAutomaticGradientAdaptation();
            _motor_S.enableCoolStep();
            _motor_S.moveAtVelocity(0);
            _motor_S.disable();
            _motor_S.setMicrostepsPerStep(MOTOR_USTEP_PER_STEP);
        }
        
        void    begin();
        void    update();
        void    enable();
        void    disable();
        int32_t getVelocity(uint8_t idx);
        void    setGoal(const Pose2D &goal) {_goal = goal;_onPosition = false;};
        Pose2D  getGoal()   {return _goal;};
        void    setSpeedMax(float vmax=XY_SPEED_MAX)     {_speedMax = vmax;};
        bool    getOnPosition()             {return _onPosition;};
            void            brake();

private:
    enum class State : uint8_t
    {
        DISABLED,
        RUNNING,
        ARRIVED
    };

    HardwareSerial &motT_serial = Serial5;
    HardwareSerial &motD_serial = Serial3;
    HardwareSerial &motS_serial = Serial4;

    bool            _onPosition = false;
    elapsedMicros   _loopTimer = 0;
    elapsedMicros   _propTimer = 0;
    uint8_t         _nArrived = 0;
    uint8_t         _nBlocked = 0;
    State           _state = State::DISABLED;
    TMC2209         _motor_T;
    TMC2209         _motor_D;
    TMC2209         _motor_S;  
    bool            _enabled;
    int32_t         _wheel_T_speed_cmd;
    int32_t         _wheel_D_speed_cmd;
    int32_t         _wheel_S_speed_cmd;
    int32_t         _wheel_T_speed_cmd_old;
    int32_t         _wheel_D_speed_cmd_old;
    int32_t         _wheel_S_speed_cmd_old;
    Pose2D          _goal;
    Pose2D          _previousPose;
    float           _speedMax;


    void            setVelocity(float robot_speed_x, float robot_speed_y, float robot_speed_z);
    void            teleplot(); 
    void            sendCommands();
    int32_t         limitAccel(int32_t target, int32_t current);
              
};


