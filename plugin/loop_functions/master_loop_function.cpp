/*
 * fitness_score_loop_function.cpp
 *
 *  Large amount of content is copied from coverage_loop_functions from the repository jamesbut/Coverage_crazyflie
 *
 *  Created on: Nov 10, 2017
 *      Author: knmcguire
 */

#include "master_loop_function.h"

extern int regen_env;
extern std::string file_name_env;

#define RANDOM_ENVIRONMENT_GEN_ON true
#define RANDOM_STARTING_ORIEN_ON false
#define RANDOM_STARTING_POSITION_ON false

// Copied from argos_ros_bot.cpp
// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* FitnessScoreLoopFunction::nodeHandle = initROS();


MasterLoopFunction::MasterLoopFunction(){
}
MasterLoopFunction::~MasterLoopFunction(){
}

/*Init: Get all the footbot entities and initialize distance for fitness score
 *
 */
void MasterLoopFunction::Init(TConfigurationNode& t_node)
{

  fitnessScoreLoopFunction.Init(t_node);
  trajectoryLoopFunction.Init(t_node);
#if(RANDOM_ENVIRONMENT_GEN_ON)
 randomEnvironmentGenerator.Init( t_node);
#endif
}


/*Reset: reinitialize fitnesscore
 *
 */
void MasterLoopFunction::Reset(){






  fitnessScoreLoopFunction.Reset();
  trajectoryLoopFunction.Reset();
#if(RANDOM_ENVIRONMENT_GEN_ON)
  if(regen_env==1) {
    std::string file_name_empty = "";
    randomEnvironmentGenerator.ClearEnvironment();
#if RANDOM_STARTING_ORIEN_ON || RANDOM_STARTING_POSITION_ON

   randomEnvironmentGenerator.ClearEnvironment();
   SetRobotPosition();

#endif
    randomEnvironmentGenerator.Reset(file_name_empty);
  }else if(regen_env==3)
    randomEnvironmentGenerator.Reset(file_name_env);
#endif


}

void MasterLoopFunction::SetRobotPosition() {
  SInitSetup robot_allocation;

  bool not_far_enough = true;
  while(not_far_enough)
  {
    srand (time(NULL));

    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = tFBMap.begin();
        it != tFBMap.end();
        ++it) {

      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      CEmbodiedEntity*  embEntity = GetEmbodiedEntity(pcFB);


      CRadians cOrient = (CRadians)(((double)rand() / RAND_MAX) * 2 * M_PI);
      // double Xrandom = (((double)rand()/ RAND_MAX)*(double)environment_width)-(double)environment_width / 2;
      //double Yrandom = (((double)rand()/ RAND_MAX)*(double)environment_height)-(double)environment_height / 2;


      int Xrandom_int=(  (rand() % (environment_width-2 +1))-(environment_width-2)/2);
      int Yrandom_int=(  (rand() % (environment_height -2 +1))-(environment_height-2)/2);
      double Xrandom = 0;
      double Yrandom = 0;
      // This does not fix the problem for any size of environnment!!
      CVector3 rob_pos = GetRobotPositionFromXML();

      if(environment_width ==10)
      {
        Xrandom = (double)(Xrandom_int/2)*2;
        Yrandom = (double)(Yrandom_int/2)*2;
      }else if(environment_width ==20)
      {
        Xrandom = (double)(Xrandom_int/2)*2 + 1;
        Yrandom = (double)(Yrandom_int/2)*2 + 1;
      }else
      {
        std::cout<<"RANDOM POSITION DOES NOT WORK!"<<std::endl;
        Xrandom = rob_pos.GetX();
        Yrandom = rob_pos.GetY();
      }


#if RANDOM_STARTING_POSITION_ON

      robot_allocation.Position.Set(Xrandom, Yrandom, rob_pos.GetZ());
      robot_allocation.Orientation.FromEulerAngles(
          cOrient,        // rotation around Z
          CRadians::ZERO, // rotation around Y
          CRadians::ZERO  // rotation around X
      );
#endif


      //Get position from XML file
#if RANDOM_STARTING_ORIEN_ON
      robot_allocation.Position.Set(rob_pos.GetX(), rob_pos.GetY(), rob_pos.GetZ());
      robot_allocation.Orientation.FromEulerAngles(
          cOrient,        // rotation around Z
          CRadians::ZERO, // rotation around Y
          CRadians::ZERO  // rotation around X
      );
#endif

      while (!MoveEntity(
          *embEntity,     // move the body of the robot
          robot_allocation.Position,                // to this position
          robot_allocation.Orientation,             // with this orientation
          false                                 // this is not a check, leave the robot there
      )) {

        LOGERR << "Can't move robot in <"
            << robot_allocation.Position
            << ">, <"
            << robot_allocation.Orientation
            << ">"
            << std::endl;

      }




    }
    if(GetDistancesBetweenRobots()>2*environment_width/3)
      not_far_enough = false;
  }


}


/*PreStep: Execute a function at every step of the simulation
 * *
 */
void MasterLoopFunction::PreStep()
{
  trajectoryLoopFunction.PostStep();
  fitnessScoreLoopFunction.PreStep();

  // fitnessScoreLoopFunction.PreStep();
  // /* Get the map of all foot-bots from the space */
  // CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
  // /* Go through them */
  // for(CSpace::TMapPerType::iterator it = tFBMap.begin();
  //     it != tFBMap.end();
  //     ++it) {
  //
  //    /* Create a pointer to the current foot-bot */
  //    CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
  //    CEmbodiedEntity*  embEntity = GetEmbodiedEntity(pcFB);
  //
  //    if(pcFB->GetId()=="bot0") {
  //
  //       std::cout << embEntity->GetOriginAnchor().Position << std::endl;
  //
  //     }
  //
  // }
}

/*PreStep: Before the simulation,
 *
 */
void MasterLoopFunction::PostExperiment()
{
  fitnessScoreLoopFunction.PostExperiment();
  trajectoryLoopFunction.PostExperiment();

}

CEmbodiedEntity* MasterLoopFunction::GetEmbodiedEntity(CEntity* pc_entity) {
   /* Is the entity embodied itself? */
   CEmbodiedEntity* pcEmbodiedTest = dynamic_cast<CEmbodiedEntity*>(pc_entity);
   if(pcEmbodiedTest != NULL) {
      return pcEmbodiedTest;
   }
   /* Is the entity composable with an embodied component? */
   CComposableEntity* pcComposableTest = dynamic_cast<CComposableEntity*>(pc_entity);
   if(pcComposableTest != NULL) {
      if(pcComposableTest->HasComponent("body")) {
         return &(pcComposableTest->GetComponent<CEmbodiedEntity>("body"));
      }
   }
   /* No embodied entity found */
   return NULL;
}


CVector3 MasterLoopFunction::GetRobotPositionFromXML() {

   // Get robot position
   try {

      const CVector3& cArenaSize = CSimulator::GetInstance().GetSpace().GetArenaSize();
      environment_width = (int)(cArenaSize.GetX());
      environment_height=(int)(cArenaSize.GetY());

      argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();

      argos::TConfigurationNode& arena_node = GetNode(cSimulator.GetConfigurationRoot(), "arena");

      argos::TConfigurationNodeIterator itArenaItem;
      for(itArenaItem = itArenaItem.begin(&arena_node);
          itArenaItem != itArenaItem.end();
          ++itArenaItem) {

             std::string arena_item_name;
             GetNodeAttribute(*itArenaItem, "id", arena_item_name);

             if(arena_item_name.compare("bot0") == 0) {

                argos::TConfigurationNode& body_node = GetNode(*itArenaItem, "body");

                CVector3 position;

                GetNodeAttributeOrDefault(body_node, "position", position, CVector3());

                return position;

            }

      }

   }

   catch(CARGoSException& ex) {

      std::cout << "Can't get position of robot from XML file" << std::endl;

   }


}

float MasterLoopFunction::GetDistancesBetweenRobots(){
  std::ofstream initial_positions;
  initial_positions.open ("init_position.txt");
  std::vector<position_bot_t> position_bots;
  /* Get the map of all foot-bots from the space */
  CSpace::TMapPerType& tFBMap =  CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
  /* Go through them */
  int id_it = 0;
  for(CSpace::TMapPerType::iterator it = tFBMap.begin();
      it != tFBMap.end();
      ++it) {
    /* Create a pointer to the current foot-bot */
    CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
    struct position_bot_t position_bot;
    position_bot.position =pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
    position_bot.bot_id_number = id_it;
    /* Add the current position of the foot-bot if it's sufficiently far from the last */
    position_bots.push_back(position_bot);


    initial_positions << position_bot.position.GetX() << ", " <<position_bot.position.GetY()<<"\n";


    id_it ++;
  }
  initial_positions.close();

float distance = 0.0f;
  for(int i = 0; i < position_bots.size(); i++) {

    for(int j = (i + 1); j < position_bots.size(); j++) {

      float dist = Distance(position_bots[i].position,
          position_bots[j].position);
      if(dist > distance) distance = dist;
    }
  }
  return distance;
}


REGISTER_LOOP_FUNCTIONS(MasterLoopFunction, "master_loop_function");
