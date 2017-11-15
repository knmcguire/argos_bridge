/*
 * random_environment_generator.h
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#ifndef ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_
#define ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_

//ARGoS libraries
#include <argos3/core/simulator/loop_functions.h>

//Standard C++ libraries
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>    // std::rotate
#include <vector>       // std::vector
#include <stdlib.h>     /* srand, rand */


using namespace argos;

struct grid_element_status_t{
  bool is_agent_present;
  std::vector<std::vector<int>> circ_action;
};

class RandomEnvironmentGenerator : public CLoopFunctions {
public:
  RandomEnvironmentGenerator();
  virtual void Init(TConfigurationNode& t_node);
  void initializeGrid();
  void initializeAgents();
  void findAgents();
  void decideNextAction(std::vector<int> current_bot_position);
  void updateGrid();


private:
  std::vector<std::vector<grid_element_status_t>> environment_grid;
  int environment_width;
  int environment_height;
  std::vector<std::vector<int>> initial_bot_positions;
  std::vector<std::vector<int>> current_agent_positions;
  float change_agent_gostraight;


};


#endif /* ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_ */
