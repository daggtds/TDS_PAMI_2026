#include "propulsion.h"
#include "localisation.h"
#include "pathfinding.h"
extern Localizer loc;
extern PathFinder pf;
// ════════════════════════════════════════════════════════════════
//  begin()
// ════════════════════════════════════════════════════════════════
void    Propulser::begin()
{
    _wheel_T_speed_cmd      =   0;  // µstep/s
    _wheel_D_speed_cmd      =   0;  // µstep/s
    _wheel_S_speed_cmd      =   0;  // µstep/s 
    _wheel_T_speed_cmd_old  =   0;  // µstep/s
    _wheel_D_speed_cmd_old  =   0;  // µstep/s
    _wheel_S_speed_cmd_old  =   0;  // µstep/s 
    _onPosition             =   false;
    _nArrived               =   0;
    _propTimer              =   0;
    _state                  =   State::DISABLED;
    _enabled                =   false;
    disable();
}
// ════════════════════════════════════════════════════════════════
//  update()
// ════════════════════════════════════════════════════════════════
void Propulser::update()
{
    if(_propTimer >= PROP_PERIOD_US)
    {
        _propTimer = 0;
        _loopTimer = 0;

        // current pose & goal
        const Pose2D &p = loc.getPose();

        // dist & angle errors
        float dy        = _goal.y - p.y;
        float dx        = _goal.x - p.x;
        float errDist   = sqrtf(dy*dy + dx*dx);
        float wz        = wrapAngle(_goal.theta - p.theta);

        switch(_state)
        {
            case State::DISABLED:
            {
                _nArrived = 0;
                _onPosition = false;
                if(_enabled)
                {
                    enable();
                    _state = State::RUNNING;
                }    
                break;
            }

            case State::RUNNING:
            {
                if(!_enabled)
                {
                    disable();
                    _state = State::DISABLED;
                }    
                else
                {
                    if((errDist < ARRIVED_DIST) && (fabsf(wz) < ARRIVED_ANG))
                    {
                        brake();
                        _onPosition = true;
                        _state = State::ARRIVED;
                    }
                    else
                    {
                        float speedAng  = wrapAngle(atan2f(dy,dx) - p.theta);
                        float speedNorm = fmaxf(fminf(_speedMax, XY_SPEED_MAX),XY_SPEED_MIN);
                        //speedNorm       = fmaxf(speedNorm, _speedMax);
                        float sx        = speedNorm * cosf(speedAng);
                        float sy        = speedNorm * sinf(speedAng);
                        setVelocity(sx,sy,wz);
                        _onPosition = false;
                    }
                }
                break;
            }

            case State::ARRIVED:
            {
                if(!_enabled)
                {
                    disable();
                    _state = State::DISABLED;
                }   
                else
                {
                    if((errDist >= ARRIVED_DIST + JITTER_DIST) || (fabsf(wz) >= ARRIVED_ANG + JITTER_ANG))
                    {
                        _state = State::RUNNING;
                    }
                }
                break;
            }
        }
    }    
}
// ════════════════════════════════════════════════════════════════
//  setVelocity()
// ════════════════════════════════════════════════════════════════ 
void Propulser::setVelocity(float robot_speed_x, float robot_speed_y, float robot_speed_z)
{
    _wheel_D_speed_cmd = (int32_t)(0.5 * ((-M_SQRT3 * robot_speed_x) + robot_speed_y) + ROBOT_RADIUS * robot_speed_z);
    _wheel_T_speed_cmd = (int32_t)(0.5 * ((M_SQRT3 * robot_speed_x) + robot_speed_y) + ROBOT_RADIUS * robot_speed_z);
    _wheel_S_speed_cmd = (int32_t)(-robot_speed_y + ROBOT_RADIUS * robot_speed_z);
        
    _wheel_T_speed_cmd = (int32_t)(((double)_wheel_T_speed_cmd / WHEEL_PERIMETER) * MOTOR_USTEP_PER_TURN);
    _wheel_D_speed_cmd = (int32_t)(((double)_wheel_D_speed_cmd / WHEEL_PERIMETER) * MOTOR_USTEP_PER_TURN);
    _wheel_S_speed_cmd = (int32_t)(((double)_wheel_S_speed_cmd / WHEEL_PERIMETER) * MOTOR_USTEP_PER_TURN);            

    _wheel_T_speed_cmd = limitAccel(_wheel_T_speed_cmd,_wheel_T_speed_cmd_old);
    _wheel_D_speed_cmd = limitAccel(_wheel_D_speed_cmd,_wheel_D_speed_cmd_old);
    _wheel_S_speed_cmd = limitAccel(_wheel_S_speed_cmd,_wheel_S_speed_cmd_old);

    _wheel_T_speed_cmd_old = _wheel_T_speed_cmd;
    _wheel_D_speed_cmd_old = _wheel_D_speed_cmd;
    _wheel_S_speed_cmd_old = _wheel_S_speed_cmd;
    
    sendCommands();
};
// ════════════════════════════════════════════════════════════════
//  sendCommands()
// ════════════════════════════════════════════════════════════════ 
void Propulser::sendCommands()
{
    _motor_T.moveAtVelocity(_wheel_T_speed_cmd);
    _motor_D.moveAtVelocity(_wheel_D_speed_cmd);
    _motor_S.moveAtVelocity(_wheel_S_speed_cmd);
};
// ════════════════════════════════════════════════════════════════
//  getVelocity()
// ════════════════════════════════════════════════════════════════ 
int32_t Propulser::getVelocity(uint8_t idx)
{
    if(idx == MOTOR_T_ID)       return _wheel_T_speed_cmd;
    else if(idx == MOTOR_D_ID)  return _wheel_D_speed_cmd;
    else if(idx == MOTOR_S_ID)  return _wheel_S_speed_cmd;
    else                        return 0;
};
// ════════════════════════════════════════════════════════════════
//  limitAccel()
// ════════════════════════════════════════════════════════════════ 
int32_t Propulser::limitAccel(int32_t target, int32_t current)
{
    int32_t lo = current - (int32_t)((float)MOTOR_ACC_MAX * (float)PROP_PERIOD_US * 1e-3f);
    int32_t hi = current + (int32_t)((float)MOTOR_ACC_MAX * (float)PROP_PERIOD_US * 1e-3f);
    return constrain(target,lo,hi);
};
// ════════════════════════════════════════════════════════════════
//  enable()
// ════════════════════════════════════════════════════════════════ 
void Propulser::enable()
{
    brake();
    _motor_T.enable();
    _motor_D.enable();
    _motor_S.enable();
    _enabled = true;
};
// ════════════════════════════════════════════════════════════════
//  disable()
// ════════════════════════════════════════════════════════════════ 
void Propulser::disable()
{
    brake();
    delay(500);
    _motor_T.disable();
    _motor_D.disable();
    _motor_S.disable();
    _enabled = false;
};
// ════════════════════════════════════════════════════════════════
//  break()
// ════════════════════════════════════════════════════════════════ 
void Propulser::brake()
{
    _wheel_T_speed_cmd = 0;
    _wheel_D_speed_cmd = 0;
    _wheel_S_speed_cmd = 0;
    _wheel_T_speed_cmd_old = 0;
    _wheel_D_speed_cmd_old = 0;
    _wheel_S_speed_cmd_old = 0;
    sendCommands();            
};
// ════════════════════════════════════════════════════════════════
//  teleplot()
// ════════════════════════════════════════════════════════════════ 
void Propulser::teleplot()
{
    // speed
    Serial.print(F(">mot_T_speed,mot:"));
    Serial.println(_wheel_T_speed_cmd);
    Serial.print(F(">mot_D_speed,mot:"));
    Serial.println(_wheel_D_speed_cmd);
    Serial.print(F(">mot_S_speed,mot:"));
    Serial.println(_wheel_S_speed_cmd);    
}
