#!/usr/bin/env python

"""
Com_bug_controller
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
from neat_ros.srv import StartSim


from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following 
import receive_rostopics

class IBugController:
    state = "ROTATE_TO_GOAL"
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;
    previous_leave_point =1000.0;
    previous_hit_point =1000.0;
    first_rotate = True
    direction = 1
    
    hitpoint = PoseStamped()
    last_bearing = 0

    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5


    def __init__(self):
        #Get Desired distance from the wall
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.direction = self.WF.getDirectionTurn();
        self.hitpoint = PoseStamped();
        self.last_bearing = 0;
        self.stateStartTime = 0;
        self.first_rotate = True;
        self.previous_leave_point =1000.0;
        self.previous_hit_point =1000.0;
        self.heading_before_turning = 0;
        self.state =  "ROTATE_TO_GOAL"
    
    def stateMachine(self, RRT):
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
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1: #If an obstacle comes within the distance of the wall
                self.previous_hit_point = self.RRT.getUWBRange()
                self.hitpoint = self.RRT.getPoseBot();
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING": 
            bot_pose = self.RRT.getPoseBot();
             #If wall is lost by corner, rotate to goal again
            if range_front>=2.0 and self.RRT.getUWBRange()<self.previous_hit_point and \
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.05)!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.05)!=True)): 
                self.transition("ROTATE_TO_GOAL")
                self.last_bearing = self.RRT.getUWBBearing()
                self.WF.init()
        elif self.state=="ROTATE_TO_GOAL":
            self.previous_leave_point = self.RRT.getUWBRange()
            print self.RRT.getRealDistanceToWall()
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) :
                self.first_rotate = False
                self.transition("FORWARD")
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1:
                self.transition("WALL_FOLLOWING")
                self.first_rotate = False


                
        # Handle actions   
        if self.state == "FORWARD":
            twist=self.WF.twistForward() #Go forward with maximum speed
        elif self.state == "WALL_FOLLOWING":
            # Wall following controller of wall_following.py
            twist = self.WF.wallFollowingController(range_side,range_front,
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),self.RRT.getArgosTime(),self.direction)     
        elif self.state=="ROTATE_TO_GOAL":
            #First go forward for 2 seconds (to get past any corner, and then turn
            if self.first_rotate or\
              (self.last_bearing<0 and self.direction == 1) or\
              (self.last_bearing>0 and self.direction == -1):
                twist = self.WF.twistTurnInCorner(self.direction)
            else:
                if (self.RRT.getArgosTime() - self.stateStartTime)<20:
                    twist=self.WF.twistForward()
                else:
                    twist = self.WF.twistTurnAroundCorner(self.distance_to_wall+0.4,self.direction)

    
        print self.state
                
        self.lastTwist = twist
        
        return twist
        

    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.RRT.getArgosTime()
        
    # See if a value is within a margin from the wanted value
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):
        
        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True 
        else:
            return False
        
    def twistRotateToGoal(self):
        v = 0
        w = self.MAX_ROTATION_SPEED * numpy.sign(self.RRT.getUWBBearing())
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
        

