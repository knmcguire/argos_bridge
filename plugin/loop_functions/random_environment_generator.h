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

using namespace argos;


class randomEnvironmentGenerator : public CLoopFunctions {
public:
  virtual void Init(TConfigurationNode& t_node);

};


#endif /* ARGOS_BRIDGE_PLUGIN_LOOP_FUNCTIONS_RANDOM_ENVIRONMENT_GENERATOR_H_ */
