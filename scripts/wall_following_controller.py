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
from argos_bridge.msg import Puck
from argos_bridge.msg import PuckList
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList
from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta

class SimpleController:

    cmdVelPub = None
    puckList = None
    state = "FORWARD"
    time = 0
    stateStartTime = 0
    lastTwist = None
    
    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.prox_callback,queue_size=10)


    def prox_callback(self, proxList):
        self.time += 1

        # Find the closest obstacle (other robot or wall).  The closest obstacle
        # is the one with the greatest 'value'.
        closestObs = None
        lowestValue = 1000.0
        range_left = 1000.0
        range_right = 1000.0
        
        #TODO: base this on the angle getting from the angle but this is broken in the simulator)
        for it in range(4,len(proxList.proximities)-1):
            if proxList.proximities[it].value < lowestValue:
                closestObs = proxList.proximities[it]
                lowestValue = proxList.proximities[it].value
                
        range_right = proxList.proximities[3].value;  
        range_left = proxList.proximities[1].value;
        #
        # Handle state transitions
        #
        if self.state == "WALL_FOLLOWING":    
            print "WALL_FOLLOWING"
            if closestObs != None and lowestValue<0.6 and lowestValue != 0.0:
                self.transition("ROTATE_TO_WALL")
        elif self.state == "FORWARD":
            print "FORWARD"
            print lowestValue
            if closestObs != None and lowestValue<0.6 and lowestValue != 0.0:
                self.transition("ROTATE_TO_WALL")
        elif self.state=="ROTATE_TO_WALL":
            print "ROTATE_TO_WALL"
            print range_left
            print proxList.proximities[23].value*math.cos(numpy.deg2rad(60))
            
            if self.logicIsCloseTo(range_left,  proxList.proximities[23].value*math.cos(numpy.deg2rad(60)) , 0.1) and range_left< proxList.proximities[23].value and range_left != 0.0:
                self.transition("STOP_MOVING")
        elif self.state=="STOP_MOVING":
            self.transition("WALL_FOLLOWING")


        else:
            die("Error: Invalid state")
        
        #
        # Handle state actions
        #
        """print "State: " + self.state"""
        twist = None
        if self.state == "WALL_FOLLOWING":
            twist = self.twistWallFollowing(range_left, proxList.proximities[23].value)

        elif self.state == "FORWARD":
            twist = self.twistForward()
            
        elif self.state == "ROTATE_TO_WALL":
            twist = self.twistTurnTowardsWall()
            
        elif self.state == "STOP_MOVING":
            twist = self.twistStop()
            
        else:
            die("Error: Invalid state")
        
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist

    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.time
            
    # Wall following logic
    def twistWallFollowing(self, range_side = 0.0, range_front=0.0):
        v = self.MAX_FORWARD_SPEED
        w = 0.0
        distance_from_wall = 0.5
        
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
        if self.logicIsCloseTo(height, distance_from_wall, 0.05):
            #If so, just try to align to the wall by changing the angle
            print "Align with wall"
            if range_side > range_equal_to -0.05 and range_front!=0.0:
                w=-0.1
            elif range_side < range_equal_to + 0.05 and range_front!= 0.0:
                w=0.1
            else:
                w=0
        else: 
            #if not, increase or decrease the distance by changing the heading
             print "Keep distance from wall"
             if height > distance_from_wall and range_front!=0.0:
                 w=0.1
             elif height < distance_from_wall and range_front!=0.0:
                 w=-0.1
             else:
                 w=0
    
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistTurnTowardsWall(self, backwards=False):
        v = 0
        w = -self.MAX_ROTATION_SPEED
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistForward(self, backwards=False):
        v = self.MAX_FORWARD_SPEED
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistStop(self, backwards=False):
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

if __name__ == '__main__':
    rospy.init_node("simple_controller")
    controller = SimpleController()
    rospy.spin()
