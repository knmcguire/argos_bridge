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

class Alg2Controller:
    state = "ROTATE_TO_GOAL"
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;
    first_rotate = True

    hitpoint = PoseStamped()
    heading_before_turning = 0
    hit_points = []
    direction = 1
    init_direction = 1
    
    last_bearing = 0
    

    rotated_half_once = False

    def __init__(self):
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        # FIX THIS HACK!
        self.direction = 1;#self.WF.getDirectionTurn();
        self.init_direction = 1;#self.WF.getDirectionTurn();
        self.hitpoint = PoseStamped();
        self.bot_tower_slope = 0;
        self.hit_points = []
        self.last_bearing = 0;
        self.stateStartTime = 0;
        self.first_run = 0;
        self.first_rotate = True;
        self.heading_before_turning = 0;
        self.state =  "ROTATE_TO_GOAL"
        
    # Ros loop were the rate of the controller is handled
    def rosLoop(self):
        rate = rospy.Rate(30)
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
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1: #If an obstacle comes within the distance of the wall
                self.hitpoint = self.RRT.getPoseBot();
                self.transition("WALL_FOLLOWING")
                self.hitpoint = self.RRT.getPoseBot();
                if self.checkHitPoints(self.hitpoint):
                    print "already hit point!"
                    self.rotated_half_once = True
                    self.direction = -1*self.direction
                else:
                    print "Did not hit point"
        elif self.state == "WALL_FOLLOWING":
            bot_pose = self.RRT.getPoseBot();
            #If wall is lost by corner, rotate to goal again
            if self.checkHitPoints(self.RRT.getPoseBot()) and self.rotated_half_once == False and \
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,self.WF.getLocationPrecision())!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,self.WF.getLocationPrecision())!=True)):
                self.transition("ROTATE_180")
                self.WF.init()
                self.direction = -1*self.direction
                self.heading_before_turning = self.RRT.getHeading() 
            if range_front>=2.0 and \
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.05)!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.05)!=True)): 
                self.transition("ROTATE_TO_GOAL")
                self.last_bearing = self.RRT.getUWBBearing()
                self.WF.init()
                self.hit_points.append(self.hitpoint)
                print "saved hitpoint"
        elif self.state=="ROTATE_TO_GOAL":
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) :
                self.first_rotate = False
                self.rotated_half_once = False
                self.direction = self.init_direction
                self.transition("FORWARD")
            else:
                if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1:
                    self.transition("WALL_FOLLOWING")
                    self.first_rotate = False
        elif self.state=="ROTATE_180":
            print math.fabs(self.wrap_pi(self.RRT.getHeading()-self.heading_before_turning))
            if math.fabs(self.wrap_pi(self.RRT.getHeading()-self.heading_before_turning))>3.04:
                self.rotated_half_once = True
                self.transition("TURN_COMP") 
        elif self.state=="TURN_COMP":
            if (self.RRT.getArgosTime() - self.stateStartTime)<2:
                self.transition("WALL_FOLLOWING") 




        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()

                
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
                if (self.RRT.getArgosTime() - self.stateStartTime)<self.WF.getDistanceAroundCorner90()/0.35 * 10:
                    twist=self.WF.twistForward()
                else:
                    twist = self.WF.twistTurnAroundCorner(self.distance_to_wall+0.3,self.direction)
        elif self.state=="ROTATE_180":
            twist = self.WF.twistTurnInCorner(-self.direction)
        elif self.state=="TURN_COMP":
            twist = self.WF.twistTurnInCorner(-self.direction)

    
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
        
    
    def checkHitPoints(self,bot_pose):
        
        for i in range(0,len(self.hit_points)):
            if ((self.logicIsCloseTo(self.hit_points[i].pose.position.x, bot_pose.pose.position.x,self.WF.getLocationPrecision())==True ) and \
            (self.logicIsCloseTo(self.hit_points[i].pose.position.y, bot_pose.pose.position.y,self.WF.getLocationPrecision())==True)):
                return True
        return False
    
    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi  
        
        
