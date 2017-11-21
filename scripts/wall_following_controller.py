#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
import math, random
import numpy
import time
from argos_bridge.msg import Puck
from argos_bridge.msg import PuckList
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList
from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following 
import receive_rostopics


class WallFollowController:
    state = "FORWARD"
    cmdVelPub = None
    puckList = None

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()

    distance_to_wall = 0;

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=10)
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        
    def rosLoop(self):

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.stateMachine()
            rate.sleep()

    
    def stateMachine(self):
                
                # Handle State transition
        if self.state == "FORWARD":
            if self.RRT.getLowestValue()<self.distance_to_wall+0.1 and self.RRT.getLowestValue() != 0.0:
                self.transition("WALL_FOLLOWING")
                
        # Handle actions   
        if self.state == "FORWARD":
            twist=self.WF.twistForward()
        elif self.state == "WALL_FOLLOWING":
            twist = self.WF.wallFollowingController(self.RRT.getRangeLeft(),self.RRT.getRangeFront(),
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),
                                                    self.RRT.getOdometry(),self.RRT.getArgosTime(),
                                                    self.RRT.getAngleToWall(),self.RRT.getRightObstacleBorder())        
    
        print self.state
                
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist

    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.RRT.getArgosTime()

    
if __name__ == '__main__':
    rospy.init_node("wall_following")
    controller = WallFollowController()
    
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass

