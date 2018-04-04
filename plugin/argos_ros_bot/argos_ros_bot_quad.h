/*
 * AUTHOR: Andrew Vardy <av@mun.ca>
 *
 * Connects an ARGoS robot with a particular configuration to ROS by publishing
 * sensor values and subscribing to a desired wheel speeds topic.
 *
 */

#ifndef ARGOS_ROS_BOT_QUAD_H
#define ARGOS_ROS_BOT_QUAD_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>


#include <ros/ros.h>
 #include <tf/transform_broadcaster.h>

#include "argos_bridge/Proximity.h"
#include "argos_bridge/ProximityList.h"
#include "argos_bridge/Rangebearing.h"
#include "argos_bridge/RangebearingList.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

using namespace argos;

#define NUMOFBOTS 2

class CArgosRosBotQuad : public CCI_Controller {

public:

  CArgosRosBotQuad();
  virtual ~CArgosRosBotQuad() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset();

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /*
   * The callback method for getting new commanded speed on the cmd_vel topic.
   */
  void cmdVelCallback(const geometry_msgs::Twist& twist);

  void otherBotPoseCallback(const geometry_msgs::PoseStamped& pose);

  /*
   * The callback method for getting the desired state of the gripper.
   */
//  void gripperCallback(const std_msgs::Bool& value);

private:

  CCI_FootBotProximitySensor* m_pcProximity;
  CCI_RangeAndBearingSensor* m_pcRangeBearing;
  CCI_PositioningSensor* m_pcPositioning;
  CCI_QuadRotorSpeedActuator* m_pcSpeedAct;
  CVector3 m_cTargetPos;

//  CCI_FootBotGripperActuator* m_pcGripper;



  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><argos_ros_bot_controller> section.
   */

  // The number of time steps from the time step of the last callback
  // after which leftSpeed and rightSpeed will be set to zero.  Useful to
  // shutdown the robot after the controlling code on the ROS side has quit.

  // The number of time steps since the last callback.
  int stepsSinceCallback;
  int globalSteps;

  // Most recent left and right wheel speeds, converted from the ROS twist
  // message.
  Real leftSpeed, rightSpeed;

  // The state of the gripper.
//  bool gripping;


  // Proximity sensor publisher
  ros::Publisher proximityPub;

  // Range and Bearing sensor publisher
  ros::Publisher rangebearingPub;

  // Position and angle sensor publisher
  ros::Publisher posePub;

  // Subscriber for cmd_vel (Twist message) topic.
  ros::Subscriber cmdVelSub;
  ros::Subscriber otherBotSub[NUMOFBOTS];


  geometry_msgs::PoseStamped PosQuat;
  argos_bridge::RangebearingList RabList;

  ros::ServiceClient client;


  bool first_run;


  /* Current robot state */
  enum EState {
     STATE_TAKE_OFF,
     STATE_MOVE
  };

  EState m_eState;

  /*
   * Takes off the robot.
   */
  void TakeOff();

  /*
   * Lets the robot perform flocking.
   */
  void Move();



  // Subscriber for gripper (Bool message) topic.
//  ros::Subscriber gripperSub;

public:
  // We need only a single ROS node, although there are individual publishers
  // and subscribers for each instance of the class.
  static ros::NodeHandle* nodeHandle;
};

#endif