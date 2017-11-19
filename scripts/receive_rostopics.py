#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
import roslib
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
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
from sklearn import linear_model, datasets


class RecieveROSTopic:
    closestObs = None
    lowestValue = 1000.0
    range_left = 1000.0
    range_right = 1000.0
    range_front =  1000.0 
    
    argos_time = 0
    
    heading=0;
    
    angle_wall=2000.0
    real_distance_to_wall =1000.0
    
    range=1000.0
    bearing = 2000.0
    
    def pose_callback(self,pose):
        (roll,pitch,yaw) = euler_from_quaternion([pose.pose.orientation.x, \
        pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.heading = yaw;
            
    def rab_callback(self,rab_list):
        for it in range(0,rab_list.n): 
            self.range = rab_list.Rangebearings[it].range
            angle = rab_list.Rangebearings[it].angle 
            self.bearing = self.wrap_pi(angle)


    def prox_callback(self, proxList):
        

        # Find the closest obstacle (other robot or wall).  The closest obstacle
        # is the one with the greatest 'value'.
        self.closestObs = None
        self.lowestValue = 1000.0
        self.range_left = 1000.0
        self.range_right = 1000.0
        self.range_front =  1000.0
        self.angle_wall = 2000.0
        self.real_distance_to_wall =1000.0

        
        
        # TODO: base this on the angle getting from the angle but this is broken in the simulator)
        for it in range(4,len(proxList.proximities)-1):
            if self.numRangeMax(proxList.proximities[it].value) < self.lowestValue:
                self.closestObs = proxList.proximities[it]
                self.lowestValue = self.numRangeMax(proxList.proximities[it].value)
                
        self.range_right = self.numRangeMax(proxList.proximities[3].value);  
        self.range_left = self.numRangeMax(proxList.proximities[1].value);
        self.range_front = self.numRangeMax( proxList.proximities[23].value);
        
        self.argos_time = proxList.header.seq

        #RANSAC
        if self.closestObs is not None:
            X=numpy.empty((0,0))[numpy.newaxis];
            Y=numpy.empty((0,0))[numpy.newaxis];
            deg_new=numpy.empty((0,0))[numpy.newaxis];
            
            deg=numpy.linspace(-0.52,0.52,num=20)
    
            for it in range(4,len(proxList.proximities)):
                if self.numRangeMax(proxList.proximities[it].value) <2.0:
                    Y= numpy.append(Y,self.numRangeMax(proxList.proximities[it].value))
                    X= numpy.append(X,math.sin(deg[it-4])*proxList.proximities[it].value)
                    deg_new=numpy.append(deg_new,deg[it-4])
            #X=numpy.transpose(X)
            #Y=numpy.transpose(Y)
            
            if len(X)!=1:
                
            
                X=numpy.reshape(X,(-1, 1))
                Y=numpy.reshape(Y,(-1, 1))
                deg_new=numpy.reshape(deg_new,(-1,1))
                #print X
                #print Y 
                #print deg_new                  
                ransac = linear_model.RANSACRegressor()
                ransac.fit(X,Y)
                inlier_mask = ransac.inlier_mask_
                outlier_mask = numpy.logical_not(inlier_mask)
                line_y_ransac = ransac.predict(X)
                
                self.angle_wall = ransac.estimator_.coef_[0][0]
                
                self.real_distance_to_wall = 0;
                sum_distance = 0;
                
                for it in range(0,len(X)):
                    sum_distance = sum_distance+Y[it][0]*math.cos(-self.angle_wall - deg_new[it])
                
                self.real_distance_to_wall = sum_distance/len(X)
            
                #print("WALL ANGLE IS: ", ransac.estimator_.coef_)
                #print("real_distance_is: ", self.real_distance_to_wall)

                      
                
        
        
    def getRangeLeft(self):
        return self.range_left
    def getRangeFront(self):
        return self.range_front
    def getArgosTime(self):
        return self.argos_time
    def getLowestValue(self):
        return self.lowestValue;
    def getUWBRange(self):
        return self.range;
    def getUWBBearing(self):
        return self.bearing;
    def getRealDistanceToWall(self):
        return self.real_distance_to_wall
    def getAngleToWall(self):
        return self.angle_wall
    def getHeading(self):
        return self.heading
        
        
    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi
    

        
    def numRangeMax(self, value = 0.0):

        if value == 0.0:
            return 1000.0
        else:
            return value 

