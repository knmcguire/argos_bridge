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
    cmdVelPub = None
    puckList = None
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;
    previous_leave_point =0;
    
    bot_init_position = PoseStamped()
    pose_tower = PoseStamped()
    bot_tower_slope = 0;
    
    hitpoint = PoseStamped()
    
    heading_before_turning = 0
    hit_points = []
    
    direction = 1
    init_direction = 1
    first_run = 1
    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5
    
    obstacle_is_hit = 0
    rotated_half_once = False
    


    def __init__(self):
        # Subscribe to topics and init a publisher 
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=10)
        rospy.Subscriber('rangebearing', RangebearingList, self.RRT.rab_callback,queue_size=10)
        rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=10)
        rospy.Subscriber('/bot1/position', PoseStamped, self.RRT.pose_callback_tower,queue_size=10)
        rospy.wait_for_service('/start_sim')
        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            start_sim(2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #Get Desired distance from the wall
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.direction = self.init_direction

        
    # Ros loop were the rate of the controller is handled
    def rosLoop(self):
        rate = rospy.Rate(30)
        rospy.sleep(4)
        while not rospy.is_shutdown():
            self.stateMachine()
            rate.sleep()

    
    def stateMachine(self): 
        range_front = 1000.0
        range_side = 1000.0
        
        bot_pose = self.RRT.getPoseBot();
        
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
            self.bot_tower_slope = (self.pose_tower.pose.position.y -self.bot_init_position.pose.position.y)/(self.pose_tower.pose.position.x -self.bot_init_position.pose.position.x);
            self.first_run = 0
        else:
            self.pose_tower= self.RRT.getPoseTower();
            bot_tower_slope_run = (self.pose_tower.pose.position.y -bot_pose.pose.position.y)/(self.pose_tower.pose.position.x -bot_pose.pose.position.x);
            bot_tower_y_diff = self.pose_tower.pose.position.y -bot_pose.pose.position.y
            bot_tower_x_diff = self.pose_tower.pose.position.x -bot_pose.pose.position.x

        # Handle State transition
        if self.state == "FORWARD": 
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1: #If an obstacle comes within the distance of the wall
                self.hitpoint = self.RRT.getPoseBot();
                if self.checkHitPoints(self.hitpoint):
                    print "already hit point!"
                    self.rotated_half_once = True
                    self.direction = -1*self.direction
                else:
                    print "Did not hit point"
                
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING": 
            if self.logicIsCloseTo(self.bot_tower_slope, bot_tower_slope_run,0.02) and \
             bot_tower_x_diff>0 and\
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.5)!=True) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.5)!=True)):
                self.transition("ROTATE_TO_GOAL")
                self.WF.init()
                self.hit_points.append(self.hitpoint)
                print "saved hitpoint"
            print("already rotated", self.rotated_half_once)
            if self.checkHitPoints(self.RRT.getPoseBot()) and self.rotated_half_once == False and \
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.5)!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.5)!=True)):
                self.transition("ROTATE_180")
                self.WF.init()
                self.direction = -1*self.direction
                self.heading_before_turning = self.RRT.getHeading() 
        elif self.state=="ROTATE_TO_GOAL":
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) :
                self.rotated_half_once = False
                self.direction = self.init_direction
                self.transition("FORWARD")
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1:
                self.obstacle_is_hit=1;
                self.transition("WALL_FOLLOWING")
        elif self.state=="ROTATE_180":
            if math.fabs(self.wrap_pi(self.RRT.getHeading()-self.heading_before_turning))>3.04:
                self.rotated_half_once = True
                self.transition("WALL_FOLLOWING") 
        
        
        # Handle actions   
        if self.state == "FORWARD":
            twist=self.WF.twistForward() #Go forward with maximum speed
        elif self.state == "WALL_FOLLOWING":
            # Wall following controller of wall_following.py
            twist = self.WF.wallFollowingController(range_side,range_front,
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),self.RRT.getArgosTime(),self.direction)     

            if(self.RRT.getArgosTime()-self.stateStartTime>10):
                self.obstacle_is_hit=0

                

        elif self.state=="ROTATE_TO_GOAL":
            #First go forward for 2 seconds (to get past any corner, and then turn
            twist = self.WF.twistTurnInCorner()

#             if (self.RRT.getArgosTime() - self.stateStartTime)<20:
#                 twist=self.WF.twistForward()
#             else:
#                 twist = self.WF.twistTurnAroundCorner(self.distance_to_wall+0.2)
        elif self.state=="ROTATE_180":
            twist = self.WF.twistTurnInCorner()

#     
        print self.state
                
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist
        

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
    
    def checkHitPoints(self,bot_pose):
        
        for i in range(0,len(self.hit_points)):
            if ((self.logicIsCloseTo(self.hit_points[i].pose.position.x, bot_pose.pose.position.x,0.5)==True ) and \
            (self.logicIsCloseTo(self.hit_points[i].pose.position.y, bot_pose.pose.position.y,0.5)==True)):
                return True
        return False
            
    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi  
        
    
if __name__ == '__main__':
    rospy.init_node("I_bug")
    controller = IBugController()
    
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass

