#include <Arduino.h>
#include "config.h"

//  ----- LEDS
#include "drivers/Leds.h"
LED rgb_led;

//  ----- LOCALISATION
#include "localisation.h"
Localizer loc;

//  ----- PROPULSION
#include "propulsion.h"
Propulser prop;
uint8_t      arrived = 0;
Pose2D    last_valid_pose;

//  ----- PATHFINDING
#include "pathfinding.h"
PathFinder pf;

//  ----- BW16
#include "bw16.h"
IHM       bw16(&Serial7);

//  ----- Actuator
#include "action.h"
Actuator  bras(&Serial2);

//  ----- SUPERVISION
elapsedMillis matchTimer = 0;
uint8_t       teamColor = TEAM_UNDEFINED;
uint8_t       idRobot = 0;
uint32_t      startDelay = 0;

enum class GOAL_ZONE: uint8_t
{
  GOAL_BOTTOM_YELLOW,
  GOAL_BOTTOM_MIDDLE,
  GOAL_BOTTOM_BLUE,
  GOAL_MIDDLE_BORDER_YELLOW,
  GOAL_MIDDLE_YELLOW,
  GOAL_MIDDLE_MIDDLE,
  GOAL_MIDDLE_BLUE,
  GOAL_MIDDLE_BORDER_BLUE,
  GOAL_TOP_YELLOW,
  GOAL_TOP_BLUE,
  GOAL_UNDEFINED
};

void setup() 
{
  Serial.begin(115200);
  delay(3000);

  // LED
  rgb_led.setConfig(LED::LED_BLINK,CRGB::Green,500);

  // ACTION
  bras.begin(); 
  bras.setPower(255);
  bras.setPoseBras(700);
  delay(500);
  bras.setPoseMain(730);
  delay(1000); 

  // BW16
  bw16.begin();

  // LOCALISATION
  if(!loc.begin())
  {
    Serial.println("FATAL: echec init capteurs");
    rgb_led.setConfig(LED::LED_CONSTANT,CRGB::Red);
    rgb_led.update();
    while (true) 
    { 
      delay(1000);
    };    
  }

  // Color & Id reception
  while((teamColor == TEAM_UNDEFINED) && (idRobot == 0))
  {
    bw16.update();
    bras.dance();
    if(bw16.isReceived())
    {
      if(strncmp(bw16.getColor(),"blue",4) == 0)          teamColor = TEAM_BLUE;
      else if(strncmp(bw16.getColor(),"yellow",6) == 0)   teamColor = TEAM_YELLOW;
      idRobot = bw16.getID();
      if(idRobot > 3) idRobot = 0;
    }
  }
  // PATHFINDING
  switch(idRobot)
  {
    case 1:
      if(teamColor == TEAM_YELLOW)
      {
        loc.setPose(75.0f,1650.0f,0.0f);
        pf.begin(teamColor,1);  
      }  
      else
      {
        loc.setPose(2925.0f,1650.0f,M_PI);
        pf.begin(teamColor,1);  
      }                          
    break;
     
    case 2:
      startDelay = 1000;
      if(teamColor == TEAM_YELLOW)
      {
        loc.setPose(75.0f,1775.0f,0.0f);
        pf.begin(teamColor,0);  // zone 2
      } 
      else
      {
        Serial.println("INIT BLUE !");
        loc.setPose(2925.0f,1775.0f,M_PI);
        pf.begin(teamColor,2);  // zone 0
      }                        
    break;

    case 3:
      startDelay = 2000;
      if(teamColor == TEAM_YELLOW)
      {
        loc.setPose(75.0f,1900.0f,0.0f);
        pf.begin(teamColor,5);  // zone 6
      }  
      else
      {
        loc.setPose(2925.0f,1900.0f,M_PI);
        pf.begin(teamColor,5);  // zone 4
      }                         
    break;
  }
  
  // PROPULSION
  prop.begin();
  matchTimer = bw16.getTime();

  bras.setPoseBras(700);
  bras.setPoseMain(730);
  
  while(matchTimer < (85000 + startDelay))
  {
    loc.update();
    pf.update();
    bw16.update();
    if(bw16.isReceived())
    {
      matchTimer = bw16.getTime();
    } 
    //Serial.print(">time:");Serial.println(millis());
  }
  prop.enable();
  last_valid_pose = loc.getPose();
}

void loop() 
{
  loc.update();
  pf.update();
  if(pf.freshPath()) 
  {
    float l = (float)pf.getPathLength() * MAP_RESOL_MM;
    prop.setSpeedMax(l);
    prop.setGoal(pf.getTarget());
  } 
  else
  {
    prop.brake();
    prop.setSpeedMax(XY_SPEED_MAX * 0.5f);
    rgb_led.setConfig(LED::LED_CONSTANT,CRGB::Red);
    rgb_led.update();   
  } 
  prop.update();

  if(matchTimer > 100000 || prop.getOnPosition())
  {
    prop.disable();
    rgb_led.setConfig(LED::LED_BLINK,CRGB::Magenta,500);
    while(1)
    {
      rgb_led.update();
      bras.dance();
    }    
  }
}
