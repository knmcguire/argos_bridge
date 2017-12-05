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
from geometry_msgs.msg import PoseStamped
from neat_ros.srv import StartSim

from scipy.stats._continuous_distns import beta
import wall_following
import receive_rostopics


class WallFollowController:
    state = "FORWARD"
    cmdVelPub = None
    puckList = None

    WF = wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()

    distance_to_wall = 0;
    direction = 1;

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=10)
        rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=10)

        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        rospy.wait_for_service('/start_sim')
        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            start_sim(2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def rosLoop(self):

        rate = rospy.Rate(100)
        rospy.sleep(4)
        while not rospy.is_shutdown():
            self.stateMachine()
            rate.sleep()

    
    def stateMachine(self,RRT):
        
        self.RRT = RRT
        
        range_front = 1000.0
        range_side = 1000.0
        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()

                
        # Handle State transition
        if self.state == "FORWARD":
            if self.RRT.getLowestValue()<self.distance_to_wall+0.1 and self.RRT.getLowestValue() != 0.0:
                self.transition("WALL_FOLLOWING")
                
        # Handle actions   
        if self.state == "FORWARD":
            twist=self.WF.twistForward()
        elif self.state == "WALL_FOLLOWING":
            twist = self.WF.wallFollowingController(range_side,range_front,
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),self.RRT.getArgosTime(),self.direction)     
        print self.state
                
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist
        return twist

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

