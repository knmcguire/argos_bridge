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
from argos_bridge.srv import GetCmds
from argos_bridge.srv import GetCmdsResponse
from argos_bridge.srv import SwitchBug
from std_msgs.msg import String
from std_msgs.msg import Bool

import com_bug_controller
import bug_2_controller
import i_bug_controller
import alg_1_controller
import alg_2_controller
import wall_following_controller
import blind_bug_controller

from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following 
import receive_rostopics

class BugAlgorithms:
    cmdVelPub = None
    bug_type="com_bug";
    bug_controller =  com_bug_controller.ComBugController()
    RRT = receive_rostopics.RecieveROSTopic()
    WF=wall_following.WallFollowing()
    reset_bug = False
    random_environment = False;

    def getController(self,argument):
        switcher = {
            "com_bug": com_bug_controller.ComBugController(),
            "bug_2": bug_2_controller.Bug2Controller(),
            "i_bug": i_bug_controller.IBugController(),
            "alg_1": alg_1_controller.Alg1Controller(),
            "alg_2": alg_2_controller.Alg2Controller(),
            "wf": wall_following_controller.WallFollowController(),
            "blind_bug": blind_bug_controller.BlindBugController(),
        }
        
        return switcher.get(argument, False)
    
    def __init__(self):
        # Subscribe to topics and init a publisher 
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=100)
        #rospy.Subscriber('rangebearing', RangebearingList, self.RRT.rab_callback,queue_size=100)
        #rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=100)
        rospy.Subscriber('/bot1/position', PoseStamped, self.RRT.pose_callback_tower,queue_size=10)
        rospy.Subscriber('/switch_bug', String, self.switchBug,queue_size=10)
        rospy.Subscriber('/random_environment', Bool, self.random_environment,queue_size=10)

        rospy.wait_for_service('/start_sim')
        s1 = rospy.Service('get_vel_cmd',GetCmds,self.runStateMachine)
        #s2 = rospy.Service('switch_bug',SwitchBug,self.switchBug)

        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            start_sim(2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        #full_param_name = rospy.search_param('bug_type')
        bug_type =  "wf"#rospy.get_param(full_param_name)
        
        self.bug_controller = self.getController(bug_type);
        if self.bug_controller == False:
            print "Wrong bug type!"

        
    # Ros loop were the rate of the controller is handled
    def rosLoop(self):
        
        rospy.spin()
#         rate = rospy.Rate(1000)
# 
#         while not rospy.is_shutdown():
#             rospy.wait_for_message("proximity", ProximityList)
#             twist = self.bug_controller.stateMachine(self.RRT);
#             self.cmdVelPub.publish(twist)
#             rate.sleep()

    def switchBug(self,req):
        self.bug_controller = self.getController(req.data);
        self.reset_bug = True
        print req.data
        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            if(self.random_environment):
                start_sim(1)
                self.random_environment = False
            else:
                start_sim(2)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        if self.bug_controller == False:
            print "Wrong bug type!"
            
    def runStateMachine(self, req):   
        
        self.RRT.prox_callback(req.proxList);
        self.RRT.rab_callback(req.RabList);
        self.RRT.pose_callback(req.PosQuat);
        
        if req.reset or self.reset_bug:
            self.WF.init()
            self.bug_controller.__init__()
            self.reset_bug = False
           
           
        print self.RRT.getUWBRange()    
        if (self.RRT.getUWBRange()>100):
            return GetCmdsResponse(self.bug_controller.stateMachine(self.RRT))
        else: 
            print "bug has reached goal"
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            return GetCmdsResponse(twist)
        
        
    def random_environment(self,req):
        self.random_environment = req.data;

    
if __name__ == '__main__':
    rospy.init_node("bug_algorithms")
    controller = BugAlgorithms()
    #rospy.wait_for_service('get_vel_cmd')
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass

