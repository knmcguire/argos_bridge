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

class RecieveROSTopic:
    closestObs = None
    lowestValue = 1000.0
    range_left = 1000.0
    range_right = 1000.0
    range_front =  1000.0 
    
    argos_time = 0
    



    def prox_callback(self, proxList):
        

        # Find the closest obstacle (other robot or wall).  The closest obstacle
        # is the one with the greatest 'value'.
        self.closestObs = None
        self.lowestValue = 1000.0
        self.range_left = 1000.0
        self.range_right = 1000.0
        self.range_front =  1000.0
        
        # TODO: base this on the angle getting from the angle but this is broken in the simulator)
        for it in range(4,len(proxList.proximities)-1):
            if self.numRangeMax(proxList.proximities[it].value) < self.lowestValue:
                self.closestObs = proxList.proximities[it]
                self.lowestValue = self.numRangeMax(proxList.proximities[it].value)
                
        self.range_right = self.numRangeMax(proxList.proximities[3].value);  
        self.range_left = self.numRangeMax(proxList.proximities[1].value);
        self.range_front = self.numRangeMax( proxList.proximities[23].value);
        
        self.argos_time = proxList.header.seq
        
    def getRangeLeft(self):
        return self.range_left
    def getRangeFront(self):
        return self.range_front
    def getArgosTime(self):
        return self.argos_time
    def getLowestValue(self):
        return self.lowestValue;

        
    def numRangeMax(self, value = 0.0):

        if value == 0.0:
            return 1000.0
        else:
            return value 

