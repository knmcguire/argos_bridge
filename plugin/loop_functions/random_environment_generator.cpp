/*
 * random_environment_generator.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: knmcguire
 */

#include "random_environment_generator.h"

using namespace std;
RandomEnvironmentGenerator::RandomEnvironmentGenerator() :
	environment_width(14),
	environment_height(14),
	change_agent_gostraight(0.85f){}

void RandomEnvironmentGenerator::Init(TConfigurationNode& t_node)
{
  initializeGrid();
  initializeAgents();
  findAgents();
  for(int itx = 0; itx<current_agent_positions.size();itx++){
      for(int ity = 0; ity<current_agent_positions.at(itx).size();ity++){
	  std::cout<<current_agent_positions.at(itx).at(ity)<<" ";
      }
      std::cout<<""<<std::endl;
  }
  for(int it = 0; it< current_agent_positions.size();it++)
    {
      decideNextAction(current_agent_positions.at(it));
    }
}

void RandomEnvironmentGenerator::initializeGrid(void)
{

  vector<vector<int>> circ_action_init{{0,0},{0,0},{0,0},{0,0}};

  //Resizing environment grid
  environment_grid.resize(environment_width);
  for(int it=0;it<environment_width;it++)
    environment_grid[it].resize(environment_height);

  //TODO: get this like trajectory_loop_function does
  for(int itx = 0; itx<environment_width;itx++){
      for(int ity = 0; ity<environment_height;ity++){
	  environment_grid.at(itx).at(ity).is_agent_present = false;
	  environment_grid.at(itx).at(ity).circ_action=circ_action_init;
	  /*	  for(int itk = 0; itk<4;itk++)
	    {
	      environment_grid.at(itx).at(ity).circ_action.at(itk).at(0).resize(2);
	      environment_grid.at(itx).at(ity).circ_action.at(itk).at(1).resize(2);
	      }*/
	  std::cout<<environment_grid.at(itx).at(ity).is_agent_present<<" ";
      }
      std::cout<<""<<std::endl;
  }
}

void RandomEnvironmentGenerator::initializeAgents(void)
{

  current_agent_positions.resize(2);
  // initial robot positions, place agent where they are
  vector<vector<int>> circ_action_init{{0,1},{1,0},{0,-1},{-1,0}};

  initial_bot_positions.resize(2);
  initial_bot_positions.at(0).resize(2);
  initial_bot_positions.at(1).resize(2);

  vector<int> bot_1_position = {3,0};
  vector<int> bot_2_position = {4,13};

  initial_bot_positions.at(0)=bot_1_position;
  initial_bot_positions.at(1)=bot_2_position;


  for(int itx = 0; itx<2;itx++){
      for(int ity = 0; ity<2;ity++){
	  std::cout<<initial_bot_positions.at(itx).at(ity)<<" ";
      }
      std::cout<<""<<std::endl;
  }


  for(int it = 0;it<2;it++)
    {
      environment_grid.at(initial_bot_positions.at(it).at(0))
				  .at(initial_bot_positions.at(it).at(1)).is_agent_present = true;

      std::rotate(circ_action_init.begin(), circ_action_init.begin()+it, circ_action_init.end());
      environment_grid.at(initial_bot_positions.at(it).at(0))
				  .at(initial_bot_positions.at(it).at(1)).circ_action = circ_action_init;

      for(int itx = 0; itx<4;itx++){
	  for(int ity = 0; ity<2;ity++){
	      std::cout<<environment_grid.at(initial_bot_positions.at(it).at(0))
					  .at(initial_bot_positions.at(it).at(1)).circ_action.at(itx).at(ity)<<" ";
	  }
	  std::cout<<""<<std::endl;
      }
    }
}

void RandomEnvironmentGenerator::findAgents(void)
{
  current_agent_positions.clear();

  int k= 0;
  for(int itx = 0; itx<environment_width;itx++){
      for(int ity = 0; ity<environment_height;ity++){
	  if( environment_grid.at(itx).at(ity).is_agent_present)
	    {
	      current_agent_positions.resize(k+1);
	      current_agent_positions.at(k).resize(2);
	      current_agent_positions.at(k).at(0)=itx;
	      current_agent_positions.at(k).at(1)=ity;
	      k++;
	    }
      }
  }

}

void RandomEnvironmentGenerator::decideNextAction(std::vector<int> current_bot_position)
{
  float random_percentage = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

  float percentage_rest = 1.0f - change_agent_gostraight;


  vector<vector<int>>circ_action_temp = environment_grid.at(current_bot_position.at(0)).at(current_bot_position.at(1)).circ_action;

  string state;
  if(random_percentage<=change_agent_gostraight)
    {
      state = "GO_STRAIGHT";
    } else if(random_percentage>change_agent_gostraight &&
	random_percentage<=change_agent_gostraight + percentage_rest/2.0f ){
	state="GO_LEFT";
	std::rotate(circ_action_temp.begin(), circ_action_temp.begin()-1, circ_action_temp.end());

    }else if(random_percentage>change_agent_gostraight + percentage_rest/2.0f &&
	random_percentage<=1){
	state="GO_RIGHT";
	std::rotate(circ_action_temp.begin(), circ_action_temp.begin()+1, circ_action_temp.end());
    }
  std::cout<<state<<std::endl;

}




REGISTER_LOOP_FUNCTIONS(RandomEnvironmentGenerator, "random_environment_generator");
