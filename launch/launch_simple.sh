#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=1

LAUNCH_FILE=/tmp/argos_bridge.launch

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t<group ns=\"$namespace\">"
    echo -e "\t\t<node pkg=\"argos_bridge\" type=\"simple_controller.py\" name=\"simple_controller\" output=\"screen\" />"
    echo -e "\t</group>"
done >> $LAUNCH_FILE

echo -e "<node pkg=\"argos_bridge\" type=\"argos_ros_start_sim_server\" name=\"argos_ros_start_sim_server\" output=\"screen\"/>"  >> $LAUNCH_FILE

echo -e "</node>"  >> $LAUNCH_FILE


echo -e "</launch>" >> $LAUNCH_FILE



roslaunch $LAUNCH_FILE
