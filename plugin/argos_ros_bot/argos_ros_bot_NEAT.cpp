// ROS Stuff #include "ros/ros.h"

/* Include the controller definition */
#include "argos_ros_bot_NEAT.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <sstream>
#include <string>
#include <iostream>

#include <ros/callback_queue.h>

#include <math.h>
#include "tf/LinearMath/Transform.h"


using namespace std;

/****************************************/
/****************************************/

CArgosRosBotNEAT::CArgosRosBotNEAT() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcOmniCam(NULL),
  m_pcRangeBearing(NULL),
//  m_pcGripper(NULL),
  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  leftSpeed(0),
  rightSpeed(0)//,
//  gripping(false)
{
}

void CArgosRosBotNEAT::Init(TConfigurationNode& t_node) {

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcOmniCam = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
  m_pcRangeBearing = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
  m_pcOmniCam->Enable();

  std::ifstream iFile ("ibug_working_directory/starting_genomes/start_genome");
  //org = new NEAT::Organism(0.0, genom, 1);
  NEAT::Genome *start_genome = new NEAT::Genome(1,iFile);
  NEAT::Organism *neatOrg = new NEAT::Organism(0.0,start_genome,1);
  m_net = neatOrg->net;

  net_inputs.resize(m_net->inputs.size());
  net_outputs.resize(m_net->outputs.size());

  net_inputs[0] = 1.0;                            //Bias node

  //Initialise inputs
  for(int i = 1; i < m_net->inputs.size(); i++) {

     net_inputs[i] = 1.0;

  }
  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}

void CArgosRosBotNEAT::ControlStep() {


   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

  // Get readings from range and bearing sensor
   const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRangeBearing->GetReadings();

   /*Read out position of bot*/
   const CCI_PositioningSensor::SReading& sPosRead = m_pcPositioning->GetReading();
   //sPosRead.Position.GetX();
   //sPosRead.Orientation.GetX();


   std::vector<float> inputs(m_net->inputs.size());
   int h =0;
   for(int i = 0; i < tRabReads.size(); i++) {
	 net_inputs[h+1] = tRabReads[i].Range;
      h++;
   }
   for(int i = 0; i < tProxReads.size()+tRabReads.size(); i++) {

	 net_inputs[h+1] = tProxReads[i].Value;
       h++;
   }

   m_net->load_sensors(inputs);
   if (!(m_net->activate())) std::cout << "Inputs disconnected from output!";

   //Get outputs
   std::vector<double> outputs(m_net->outputs.size());
   std::vector<NEAT::NNode*>::iterator it;

   for(it = m_net->outputs.begin(); it != m_net->outputs.end(); it++) {

      outputs[it-m_net->outputs.begin()] = (*it)->activation;
   }

   net_outputs = outputs;

   ConvertDifferentialDriveToSpeed(net_outputs[1], net_outputs[2]);

  // Wait for any callbacks to be called.
  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}




void CArgosRosBotNEAT::ConvertDifferentialDriveToSpeed(Real linear_x, Real angular_z) {

  Real v = linear_x;// Forward speed
  Real w = angular_z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}



/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CArgosRosBotNEAT, "argos_ros_bot_neat_controller")
