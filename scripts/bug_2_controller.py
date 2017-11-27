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
    
    direction = 1
    first_run = 1
    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5
    
    obstacle_is_hit = 0


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
        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()    
    
        bot_tower_slope_run = 0;
        if self.first_run:
            self.bot_init_position = self.RRT.getPoseBot();
            self.bot_tower_slope = (self.pose_tower.pose.position.y -self.bot_init_position.pose.position.y)/(self.pose_tower.pose.position.x -self.bot_init_position.pose.position.x);
            self.first_run = 0
        else:
            bot_pose = self.RRT.getPoseBot();
            self.pose_tower= self.RRT.getPoseTower();
            bot_tower_slope_run = (self.pose_tower.pose.position.y -bot_pose.pose.position.y)/(self.pose_tower.pose.position.x -bot_pose.pose.position.x);
        
        # Handle State transition
        if self.state == "FORWARD": 
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall: #If an obstacle comes within the distance of the wall
                self.hitpoint = self.RRT.getPoseBot();
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING": 
            if self.logicIsCloseTo(self.bot_tower_slope, bot_tower_slope_run,0.01) and\
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.3)!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.3)!=True)): 
                self.transition("ROTATE_TO_GOAL")
                self.WF.init()
        elif self.state=="ROTATE_TO_GOAL":
            self.previous_leave_point = self.RRT.getUWBRange()
            print self.RRT.getRealDistanceToWall()
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) :
                self.transition("FORWARD")
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1:
                self.obstacle_is_hit=1;
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
        
    
if __name__ == '__main__':
    rospy.init_node("I_bug")
    controller = IBugController()
    
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass
