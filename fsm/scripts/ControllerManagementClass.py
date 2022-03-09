"""
Author: Mingfeng Zhang @ McGill University
Date: 2014
"""
#!/usr/bin/env python
import roslib; roslib.load_manifest('fsm')

import sys

import rospy
#from beginner_tutorials.srv import *
from quadrotor_input.srv import *

class ControllerManagementClass():
    def __init__(self):
        while True:
            try:
                rospy.wait_for_service('controller/command', 1.0)
                break
            except rospy.ROSException , error:
                print error.message        
            except :
                print "break"
                break
        
        self.handle_service = rospy.ServiceProxy('controller/command', CommandController)   
        
        #Added on May 26, 2015, to select source of vehicle states (mcptam or px4flow)
        self.handle_service_select_source = rospy.ServiceProxy('multi_box/select_topic', SelectStateSource)     
        
    def ControllerClient(self, service_in):
        try:
            resp = self.handle_service(service_in.running, service_in.path, service_in.gains)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
    
    #Added on May 26, 2015, to select source of vehicle states (mcptam or px4flow)        
    def SelectStateSource(self, state_source):
        try:
            resp = self.handle_service_select_source(state_source.strPoseTopicSelected, state_source.strVelocityTopicSelected)
            return resp.bSuccess
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

