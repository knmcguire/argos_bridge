// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot_quad.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>

#include <sstream>
#include <string>
#include <iostream>
#include <vector>       // std::vector


#include <ros/callback_queue.h>

#include <math.h>
#include "tf/LinearMath/Transform.h"


#include "argos_bridge/GetCmds.h"




using namespace std;
using namespace argos_bridge;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosBotQuad::nodeHandle = initROS();


CArgosRosBotQuad::CArgosRosBotQuad() :
          m_pcProximity(NULL),
          m_pcRangeBearing(NULL),
          m_pcPositioning(NULL),
          m_pcSpeedAct(NULL)
{
}


void CArgosRosBotQuad::Init(TConfigurationNode& t_node) {
  client = nodeHandle->serviceClient<argos_bridge::GetCmds>("/bot0/get_vel_cmd");

  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcProximity = GetSensor<CCI_EyeBotProximitySensor>("eyebot_proximity");
  m_pcPositioning   = GetSensor  <CCI_PositioningSensor        >("positioning"       );
  m_pcSpeedAct  = GetActuator<CCI_QuadRotorSpeedActuator   >("quadrotor_speed"   );

  stringstream poseTopic;
  poseTopic << "/" << GetId() << "/position";
  posePub = nodeHandle->advertise<geometry_msgs::PoseStamped>(poseTopic.str(), 1000);

}


void CArgosRosBotQuad::Reset() {
  first_run = true;


  /* Switch to state start */
  // m_eState = STATE_TAKE_OFF;
  m_eState = STATE_TAKE_OFF;

}

void CArgosRosBotQuad::ControlStep() {

  /* Get readings from proximity sensor */

  const CCI_EyeBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  proxList.header.seq = globalSteps;
  for (size_t i = 0; i < proxList.n; ++i) {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);
  }

  //Get readings from range and bearing sensor
  const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRangeBearing->GetReadings();
  RangebearingList RabList;
  RabList.Rangebearings.resize(tRabReads.size());
  RabList.n = tRabReads.size();
  for (size_t i = 0; i < RabList.n; ++i) {
    Rangebearing Rab;
    Rab.range = tRabReads[i].Range;
    Rab.angle = tRabReads[i].HorizontalBearing.GetValue();
    RabList.Rangebearings.at(i)=Rab;
  }

  /*Read out position of bot*/
  const CCI_PositioningSensor::SReading& sPosRead = m_pcPositioning->GetReading();
  PosQuat.header.frame_id = GetId();
  PosQuat.pose.position.x = sPosRead.Position.GetX();
  PosQuat.pose.position.y = sPosRead.Position.GetY();
  PosQuat.pose.position.z = sPosRead.Position.GetZ();
  PosQuat.pose.orientation.x = sPosRead.Orientation.GetX();
  PosQuat.pose.orientation.y = sPosRead.Orientation.GetY();
  PosQuat.pose.orientation.z = sPosRead.Orientation.GetZ();
  PosQuat.pose.orientation.w = sPosRead.Orientation.GetW();

  posePub.publish(PosQuat);

  CRadians cXAngle, cYAngle, cZAngle;
  sPosRead.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

  if(GetId()!="bot1")
  {

    argos_bridge::GetCmds srv;


    srv.request.RabList = RabList;
    srv.request.PosQuat = PosQuat;
    srv.request.proxList = proxList;
    if(first_run)
    {
      srv.request.reset = true;
      first_run = false;
    }
    else
      srv.request.reset = false;



    switch(m_eState) {
      case STATE_TAKE_OFF:
        TakeOff();
        break;
      case STATE_MOVE:
        client.call(srv);
        Move(srv.response.cmd_vel.linear.x,srv.response.cmd_vel.angular.z,cZAngle.GetValue());
        break;
    }



  }
  globalSteps ++;

}

void CArgosRosBotQuad::TakeOff() {

  //Set target height to 0.5 for take off
  m_cTargetPos = m_pcPositioning->GetReading().Position;
  m_cTargetPos.SetZ(0.3f);

  CVector3 v = CVector3(0.0f, 0.0f, -0.1f);
  m_pcSpeedAct->SetLinearVelocity(v);

  // std::cout << "Position:" << std::endl;
  // std::cout << m_pcPosSens->GetReading().Position << std::endl;

  cout<<m_pcPositioning->GetReading().Position.GetZ()<<endl;
  if(m_pcPositioning->GetReading().Position.GetZ() > m_cTargetPos.GetZ()) {
    m_eState = STATE_MOVE;
    Move(0.0,0.0,0.0);
  }

}

void CArgosRosBotQuad::Move(float vel_lin_x,float vel_rot_z,float current_heading) {


  cout<<"Controller: "<<vel_lin_x<<" "<<vel_rot_z<<endl;

  //if(vel_rot_z<0.1)
   // m_pcSpeedAct->Reset();
   // vel_rot_z = 0.0;
  //vel_rot_z = 0.0001;




  m_pcSpeedAct->SetRotationalSpeed(CRadians(vel_rot_z));
  m_pcSpeedAct->SetLinearVelocity(
      CVector3(vel_lin_x*cos(current_heading),vel_lin_x*sin(current_heading),0.0));

}



REGISTER_CONTROLLER(CArgosRosBotQuad, "argos_ros_bot_quad_controller")
