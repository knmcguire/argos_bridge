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

class SimpleController:

    cmdVelPub = None
    puckList = None
    
    WF=wall_following.WallFollowing()


    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5


    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.prox_callback,queue_size=10)


    def prox_callback(self, proxList):
        
     

        # Find the closest obstacle (other robot or wall).  The closest obstacle
        # is the one with the greatest 'value'.
        closestObs = None
        lowestValue = 1000.0
        range_left = 1000.0
        range_right = 1000.0
        range_front =  1000.0
        
        #TODO: base this on the angle getting from the angle but this is broken in the simulator)
        for it in range(4,len(proxList.proximities)-1):
            if self.numRangeMax(proxList.proximities[it].value) < lowestValue:
                closestObs = proxList.proximities[it]
                lowestValue = self.numRangeMax(proxList.proximities[it].value)
                
        range_right = self.numRangeMax(proxList.proximities[3].value);  
        range_left = self.numRangeMax(proxList.proximities[1].value);
        range_front = self.numRangeMax( proxList.proximities[23].value);
        
        twist = self.WF.wallFollowingController(range_left,range_front,lowestValue,proxList.header.seq)
                
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist

    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.time
        
    def numRangeMax(self, value = 0.0):

        if value == 0.0:
            return 1000.0
        else:
            return value 

            
   
if __name__ == '__main__':
    rospy.init_node("simple_controller")
    controller = SimpleController()
    rospy.spin()
