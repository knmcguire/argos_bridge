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
from argos_bridge.msg import Rangebearing
from argos_bridge.msg import RangebearingList
from geometry_msgs.msg import PoseStamped


from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following 
import receive_rostopics

class ComBugController:
    state = "ROTATE_TO_GOAL"
    cmdVelPub = None
    puckList = None
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;
    
    
    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5


    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=10)
        rospy.Subscriber('rangebearing', RangebearingList, self.RRT.rab_callback,queue_size=10)
        rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=10)

        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        
        
    def rosLoop(self):

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.stateMachine()
            rate.sleep()

    
    def stateMachine(self):
        
        
        
        # Handle State transition
        if self.state == "FORWARD":
            print self.distance_to_wall
            print self.RRT.getLowestValue()
            if self.RRT.getLowestValue()<0.6:
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING": 
            if self.RRT.getRangeFront()>=2.0: # and self.logicIsCloseTo(0, self.RRT.getUWBBearing(), 0.1):
                self.transition("ROTATE_TO_GOAL")
                self.WF.init()
        elif self.state=="ROTATE_TO_GOAL":
            print self.RRT.getLowestValue()
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) :
                self.transition("FORWARD")
            elif self.RRT.getLowestValue()<1.2:
                self.transition("WALL_FOLLOWING")


                
        # Handle actions   
        if self.state == "FORWARD":
            twist=self.WF.twistForward()
        elif self.state == "WALL_FOLLOWING":
            print self.RRT.getLeftObstacleBorder()
            twist = self.WF.wallFollowingController(self.RRT.getRangeLeft(),self.RRT.getRangeFront(),
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),
                                                    self.RRT.getOdometry(),self.RRT.getArgosTime(),
                                                    self.RRT.getAngleToWall(),self.RRT.getLeftObstacleBorder())
        elif self.state=="ROTATE_TO_GOAL":
            if (self.RRT.getArgosTime() - self.stateStartTime)<20:
                twist=self.WF.twistForward()
            else:
                twist = self.twistRotateToGoal()
    
        print self.state
                
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist
        

    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.RRT.getArgosTime()
        
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):
        
        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True 
        else:
            return False
        
    def numRangeMax(self, value = 0.0):

        if value == 0.0:
            return 1000.0
        else:
            return value 
    
    def twistRotateToGoal(self):
        v = 0
        w = self.MAX_ROTATION_SPEED * numpy.sign(self.RRT.getUWBBearing())
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
        


    
if __name__ == '__main__':
    rospy.init_node("com_bug")
    controller = ComBugController()
    
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass

