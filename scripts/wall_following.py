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



class WallFollowing:
    state = "ROTATE_IN_CORNER"
    time = 0
    stateStartTime = 0
    lastTwist = None
    lastRangeFrontBeforeLost = 0
    lastRangeLeft = 0
    lastRangeLeftBeforeLost = 0
    left_range_lost_first_time = True

    distance_from_wall = 0.5
    
    MAX_FORWARD_SPEED=1.0
    MAX_ROTATION_SPEED=2.5

    def wallFollowingController(self, range_left, range_front, closest_obstacle, time_argos):

        self.time = time_argos
        #
        # Handle state transitions
        #
        if self.state == "WALL_FOLLOWING":    
            if closest_obstacle<self.distance_from_wall+0.1 and closest_obstacle != 0.0:
                self.transition("ROTATE_IN_CORNER")
            if range_front > 2.0:
                self.lastRangeFrontBeforeLost = self.lastRangeFront
                self.lastRangeLeftBeforeLost =  self.lastRangeLeft
                self.transition("ROTATE_AROUND_CORNER")
        elif self.state=="ROTATE_IN_CORNER":
            if self.logicIsCloseTo(range_left,  range_front*math.cos(numpy.deg2rad(60)) , 0.1) and range_left != 0.0:
                self.transition("STOP_MOVING")
        elif self.state=="ROTATE_AROUND_CORNER":
            print "ROTATE_AROUND_CORNER"
            if self.logicIsCloseTo(range_left,range_front*math.cos(numpy.deg2rad(60)) , 0.1) and range_left != 0.0:
                self.transition("STOP_MOVING")
        elif self.state=="STOP_MOVING":
            self.left_range_lost_first_time = True;
            self.transition("WALL_FOLLOWING")
        else:
            die("Error: Invalid state")
        
        #
        # Handle state actions
        #
        print "State WallFollowing: " + self.state
        twist = None
        if self.state == "WALL_FOLLOWING":
            twist = self.twistWallFollowing(range_left, range_front)           
        elif self.state == "ROTATE_IN_CORNER":
            twist = self.twistTurnInCorner()
                
        elif self.state == "ROTATE_AROUND_CORNER":
            #TODO: do this based on odometry!
            #if range_left<2.0 and self.left_range_lost_first_time:
            if (self.time - self.stateStartTime)<20:
                print "Go forward"
                twist = self.twistForward()
            else:
                print "now turn"
                twist = self.twistTurnAroundCorner(self.lastRangeLeftBeforeLost+0.2)
                self.left_range_lost_first_time = False

            
        elif self.state == "STOP_MOVING":
            twist = self.twistStop()
            
        else:
            die("Error: Invalid state")

        self.lastRangeFront = range_front;
        self.lastRangeLeft = range_left;
        
        return twist


    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.time
            
    # Wall following logic
    def twistWallFollowing(self, range_side = 0.0, range_front=0.0):
        v = self.MAX_FORWARD_SPEED
        w = 0.0
        
        #Calculating height of triangle if the angle between sides is known
        # combination of:
        # 1- SAS for triangle area
        # 2- Half base times height method
        # 3- Cosinus Rule
        beta = numpy.deg2rad(60)
        
        height=(range_side*range_front*math.sin(beta))/math.sqrt(math.pow(range_side,2)+math.pow(range_front,2)-2*range_side*range_front*math.cos(beta))
        
        #This is to compare the range side with to keep it perpendicular to the wall
        range_equal_to =  range_front*math.cos(numpy.deg2rad(60))
                
        #Most important, first check if the robot if away from the wall!        
        if self.logicIsCloseTo(height, self.distance_from_wall, 0.1):
            #If so, just try to align to the wall by changing the angle
            print "Align with wall"
            if range_side > range_equal_to -0.05 and range_front!=0.0:
                w=-0.15
            elif range_side < range_equal_to + 0.05 and range_front!= 0.0:
                w=0.15
            else:
                w=0
        else: 
            #if not, increase or decrease the distance by changing the heading
             print "Keep distance from wall"
             if height > self.distance_from_wall :
                 w=0.15
             elif height < self.distance_from_wall and range_front<1.8:
                 w=-0.15
             elif height < self.distance_from_wall and range_front>1.8:
                 w=0.15

             else:
                 w=0
    
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistTurnInCorner(self):
        v = 0
        w = -self.MAX_ROTATION_SPEED
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistTurnAroundCorner(self,radius):
        v = self.MAX_FORWARD_SPEED
        w = v/radius
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistForward(self):
        v = self.MAX_FORWARD_SPEED
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistStop(self):
        v = 0
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):
        
        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True 
        else:
            return False
    
    def getWantedDistanceToWall(self):
        return self.distance_from_wall;

