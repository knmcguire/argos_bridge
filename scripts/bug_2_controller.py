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

class Bug2Controller:
    state = "ROTATE_TO_GOAL"
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;    
    bot_init_position = PoseStamped()
    pose_tower = PoseStamped()
    bot_tower_slope = 0;
    
    hitpoint = PoseStamped()
    
    direction = 1
    first_run = 1
    obstacle_is_hit = 0


    def __init__(self):
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.direction = self.WF.getDirectionTurn();
        self.hitpoint = PoseStamped();
        self.bot_init_position = PoseStamped()
        self.pose_tower = PoseStamped()
        self.first_rotate = True;
        self.last_bearing = 0;
        self.stateStartTime = 0;
        self.obstacle_is_hit = 0;
        self.first_run = 0;
        self.state =  "ROTATE_TO_GOAL"

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
    
        bot_tower_slope_run = 0;
        if self.first_run:
            self.pose_tower= self.RRT.getPoseTower();
            self.bot_init_position = self.RRT.getPoseBot();
            if math.fabs(self.pose_tower.pose.position.x -self.bot_init_position.pose.position.x)>0:
                self.bot_tower_slope = (self.pose_tower.pose.position.y -self.bot_init_position.pose.position.y)/(self.pose_tower.pose.position.x -self.bot_init_position.pose.position.x);
                self.first_run = 0
        else:
            bot_pose = self.RRT.getPoseBot();
            self.pose_tower= self.RRT.getPoseTower();
            bot_tower_slope_run = (self.pose_tower.pose.position.y -bot_pose.pose.position.y)/(self.pose_tower.pose.position.x -bot_pose.pose.position.x);
            bot_tower_y_diff = self.pose_tower.pose.position.y -bot_pose.pose.position.y
            bot_tower_x_diff = self.pose_tower.pose.position.x -bot_pose.pose.position.x
            
        # Handle State transition
        if self.state == "FORWARD": 
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall: #If an obstacle comes within the distance of the wall
                self.hitpoint = self.RRT.getPoseBot();
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING": 
            if self.logicIsCloseTo(self.bot_tower_slope, bot_tower_slope_run,0.1) and\
            bot_tower_x_diff>0 and\
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,self.WF.getLocationPrecision())!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,self.WF.getLocationPrecision())!=True)): 
                self.transition("ROTATE_TO_GOAL")
                self.last_bearing = self.RRT.getUWBBearing()
                self.WF.init()
        elif self.state=="ROTATE_TO_GOAL":
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) :
                self.transition("FORWARD")

                
        # Handle actions   
        if self.state == "FORWARD":
            twist=self.WF.twistForward() #Go forward with maximum speed
        elif self.state == "WALL_FOLLOWING":
            # Wall following controller of wall_following.py
            twist = self.WF.wallFollowingController(range_side,range_front,
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),self.RRT.getArgosTime(),self.direction)     

        elif self.state=="ROTATE_TO_GOAL":
            #First go forward for 2 seconds (to get past any corner, and then turn
            print self.last_bearing
            if self.last_bearing>0:
                twist = self.WF.twistTurnInCorner(-1)
            else:
                twist = self.WF.twistTurnInCorner(1)
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


