// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot_quad.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <sstream>
#include <string>
#include <iostream>
#include <vector>       // std::vector


#include <ros/callback_queue.h>

#include <math.h>
#include "tf/LinearMath/Transform.h"


#include "argos_bridge/GetCmds.h"

CArgosRosBotQuad::CArgosRosBotQuad() :
  m_pcProximity(NULL),
  m_pcRangeBearing(NULL),
  m_pcPositioning(NULL),
  m_pcSpeedAct(NULL)
{
}


void CArgosRosBotQuad::Init(TConfigurationNode& t_node) {

  m_pcPositioning   = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   m_pcSpeedAct  = GetActuator<CCI_QuadRotorSpeedActuator   >("quadrotor_speed"   );

}


void CArgosRosBotQuad::Reset() {


   /* Switch to state start */
   // m_eState = STATE_TAKE_OFF;
   m_eState = STATE_MOVE;

}

void CArgosRosBotQuad::ControlStep() {


   switch(m_eState) {
      case STATE_TAKE_OFF:
         TakeOff();
         break;
      case STATE_MOVE:
         Move();
         break;
   }

}

void CArgosRosBotQuad::TakeOff() {

   //Set target height to 0.5 for take off
   m_cTargetPos = m_pcPositioning->GetReading().Position;
   m_cTargetPos.SetZ(0.5f);

   CVector3 v = CVector3(0.0f, 0.0f, 0.1f);
   m_pcSpeedAct->SetLinearVelocity(v);

   // std::cout << "Position:" << std::endl;
   // std::cout << m_pcPosSens->GetReading().Position << std::endl;

   if(m_pcPositioning->GetReading().Position.GetZ() > m_cTargetPos.GetZ()) {
      Move();
   }

}

void CArgosRosBotQuad::Move() {

   CVector3 vel = CVector3(0.0f, 0.0f, 0.0f);
   m_pcSpeedAct->SetLinearVelocity(vel);

}



REGISTER_CONTROLLER(CArgosRosBotQuad, "argos_ros_bot_quad_controller")
