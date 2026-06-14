#include "bw16.h"
#include "localisation.h"
#include <string.h>
extern Localizer loc;
// ════════════════════════════════════════════════════════════════
//  begin()
// ════════════════════════════════════════════════════════════════
void    IHM::begin()
{
    pserial->begin(115200);
    sprintf(_teamColor,"UNDEFINED");
}
// ════════════════════════════════════════════════════════════════
//  update()
// ════════════════════════════════════════════════════════════════
void IHM::update()
{
    if(_ihmTimer >= IHM_PERIOD_MS)
    {
        _ihmTimer = 0;

        // current pose & goal
        const Pose2D &p = loc.getPose();

        // send new pose
        sprintf(_messTeensy, "POSE%dX%dY%dTHETA",(uint16_t)p.x,(uint16_t)p.y,(int16_t)(p.theta*RAD_TO_DEG));
        pserial->println(_messTeensy);

        // message received from BW16 ?
        receiveMess();
    }  
}
// ════════════════════════════════════════════════════════════════
//  receiveMess()
// ════════════════════════════════════════════════════════════════ 
void IHM::receiveMess()
{
    _isReceived = false;
    //_messIdx = 0;
    while(pserial->available() > 0)
    {
        _messBW16[_messIdx] = pserial->read();
        if(_messBW16[_messIdx] == '\n')
        {
            _messBW16[_messIdx+1]  = '\0';
            Serial.println(_messBW16);
            sscanf(_messBW16,"ID:%d:TIME:%lu:TEAM:%s",&_idRobot, &_matchTimer, _teamColor);
            Serial.println(_idRobot);
            Serial.println(_matchTimer);
            Serial.println(_teamColor);
            _isReceived = true;
            _messIdx = 0;
        }
        else
        {
            _messIdx++;
            _isReceived = false;
        }
    }
    while(pserial->available() > 0)
    {
        pserial->read();
    }
}