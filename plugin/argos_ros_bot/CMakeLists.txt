set(EXT_LIB_INCLUDES
    ${CMAKE_SOURCE_DIR}/neat_ros/ext_lib/)

add_library(argos_ros_bot MODULE argos_ros_bot.h argos_ros_bot.cpp)
target_link_libraries(argos_ros_bot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(argos_ros_bot_neat MODULE argos_ros_bot_NEAT.h argos_ros_bot_NEAT.cpp
                                      entm_memory.h entm_memory.cpp)
target_link_libraries(argos_ros_bot_neat
  neatGRULibRos
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

target_include_directories(argos_ros_bot_neat PUBLIC
   ${EXT_LIB_INCLUDES})

add_library(footbot_manualcontrol SHARED footbot_manualcontrol.h footbot_manualcontrol.cpp)
target_link_libraries(footbot_manualcontrol
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

add_library(argos_ros_bot_wf MODULE argos_ros_bot_WF.h argos_ros_bot_WF.cpp)
target_link_libraries(argos_ros_bot_wf
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(argos_ros_bot_wf_neat MODULE argos_ros_bot_WF_NEAT.h argos_ros_bot_WF_NEAT.cpp)
target_link_libraries(argos_ros_bot_wf_neat
  neatGRULibRos
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

target_include_directories(argos_ros_bot_wf_neat PUBLIC
   ${EXT_LIB_INCLUDES})
