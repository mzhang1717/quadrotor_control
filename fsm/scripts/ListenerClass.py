"""
Author: Mingfeng Zhang @ McGill University
Date: 2014
"""

#Listener class object
#!/usr/bin/env python#

PKG = 'fsm' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy
import time
import message_filters
#from message_filters import Subscriber, Cache
from std_msgs.msg import Float64, Bool, Int16
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point, QuaternionStamped, PoseStamped
from quadrotor_msgs.msg import RadioControl, BatteryStatus, ControllerError
from rospy.numpy_msg import numpy_msg 
from Utility import *
from DataMagazineClass import DataMagazineClass 
import collections  

import tf.transformations
import math

class ListenerClass():
    def __init__(self,queue_size,dictionary):
        print("Initializing Listener Object ...")
        #Define Dictionary 
        self.dictionary       = dictionary
        #Define topics to which the listener should subscribe (should be imported from a parameter server later)
        self.StartedListening = rospy.Time.now()
        batteryVoltage_topic  = 'battery'
        poseStamped_topic     = 'poseStamped'
        ControllerError_topic = 'ControllerError'
        RadioControl_topic    = 'RadioControl'
        ControllerStatus_topic = 'ControllerStatus'
        #Create subscribers to those topics (msg_type should also be loaded from server)
        print("FSM Starts Listening ...")
        
        self.controllerStatus = True
        self.datagear_previous = 2
        
        self.totakeoff = False
        self.toland = False
        self.tomove = False
        self.toreset = False
        self.wpIndex = 0
        
        self.changeStateSource = False
        self.stateSource = '';
         
                #Initialize the data loggers for the listener 
        self.batteryVoltage     = float
        self.poseStampedQueue   = collections.deque([],queue_size)
        self.ctrlThrottle       = float #Tx throttle stick position [-1,1]
        self.AutoPilotSwitch    = True #UI for menifold designation
        self.MissionGoSwitch    = True #UI for mission persistence (Different from FSM 'OUTBOUND'/'INBOUND' )
        self.runningStatPose    = [DataMagazineClass('x',queue_size),
                                  DataMagazineClass('y',queue_size),
                                  DataMagazineClass('z',queue_size)]
        self.runningStatError   = [DataMagazineClass('error_x',queue_size),
                                   DataMagazineClass('error_y',queue_size),
                                   DataMagazineClass('error_z',queue_size)]
        self.runningStatError_d = [DataMagazineClass('d_error_x',queue_size),
                                   DataMagazineClass('d_error_y',queue_size),
                                   DataMagazineClass('d_error_z',queue_size)]    
                
        # Testing
        self.publisher_ahrs  = rospy.Publisher('euler_ahrs', Point) 
        self.publisher_viconpose = rospy.Publisher('euler_vicon', Point) 
        self.publisher_filterpose = rospy.Publisher('euler_filter', Point) 
        
        self.subscriber_RadioControl   = rospy.Subscriber(RadioControl_topic,
                                                          RadioControl,
                                                          self.RadioControl_callback)        
        
        self.subscriber_batteryVoltage = rospy.Subscriber(batteryVoltage_topic,
                                                          BatteryStatus,
                                                          self.battery_callback)        

        self.subscriber_ControllerError = rospy.Subscriber(ControllerError_topic,
                                                          ControllerError,
                                                          self.ControllerError_callback)        

        self.subscriber_pose           = rospy.Subscriber(poseStamped_topic,
                                                          PoseStamped,
                                                          self.poseStamped_callback)       
                                                          
        self.subscriber_ControllerStatus = rospy.Subscriber(ControllerStatus_topic,
                                                          Bool,
                                                          self.controllerStatus_callback) 
                                                          
        # Testing
        self.subscribe_ahrs = rospy.Subscriber('x8/output/ahrs',
                                                QuaternionStamped,
                                                self.ahrs_callback) 
        self.subscribe_viconpose = rospy.Subscriber('vicon_pose',
                                                    PoseStamped,
                                                    self.viconpose_callback)
                                                    
        # subscribe to topics from virtual pilot                                          
        rospy.Subscriber("takeoff", Empty, self.callback_takeoff)
        rospy.Subscriber("land", Empty, self.callback_land)
        rospy.Subscriber("reset", Empty, self.callback_reset)                                            
        rospy.Subscriber("move", Empty, self.callback_move)                                            
        
        rospy.Subscriber("source_px4flow", Empty, self.callback_px4flow)
        rospy.Subscriber("source_mcptam", Empty, self.callback_mcptam)
        
        rospy.Subscriber("/controller/wp_index", Int16, self.callback_wpindex)
        
   #################################     
    def callback_wpindex(self, data):
        self.wpIndex = data.data
        
    def callback_takeoff(self, data):
        rospy.loginfo(rospy.get_caller_id()+ " To Take-off!")
        self.totakeoff = True
    
    def callback_land(self, data):
        rospy.loginfo(rospy.get_caller_id()+ " To land!")
        self.toland = True
    
    def callback_move(self, data):
        rospy.loginfo(rospy.get_caller_id()+ " To Move!") 
        self.tomove = True
        
    def callback_reset(self, data):
        rospy.loginfo(rospy.get_caller_id()+ " To Reset!")  
        self.toreset = True 
        
    def callback_px4flow(self, data):
        rospy.loginfo(rospy.get_caller_id()+ " To PX4FLOW!")  
        self.changeStateSource = True  
        self.stateSource = 'px4flow'   
        
    def callback_mcptam(self, data):
        rospy.loginfo(rospy.get_caller_id()+ " To MCPTAM!")  
        self.changeStateSource = True  
        self.stateSource = 'mcptam'   

    def ahrs_callback(self, data):
        euler = tf.transformations.euler_from_quaternion((data.quaternion.x,data.quaternion.y,data.quaternion.z,data.quaternion.w))
        radTodeg = 180.0/math.pi
        outMsg = Point(radTodeg*euler[0], radTodeg*euler[1], radTodeg*euler[2])
        self.publisher_ahrs.publish(outMsg)
        
    def viconpose_callback(self, data):
        euler = tf.transformations.euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))
        
        phi = euler[0] - math.pi
        
        while phi > math.pi:
            phi = phi -2.0 *math.pi
            
        while phi < -1.0 * math.pi:
            phi = phi + 2.0 *math.pi
        
        radTodeg = 180.0/math.pi
        outMsg = Point(radTodeg*phi, radTodeg*euler[1], radTodeg*euler[2])
        self.publisher_viconpose.publish(outMsg)
     
    def controllerStatus_callback(self, data):
        self.controllerStatus = data.data
        
    def RadioControl_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        # --------------------------------------------        
        # gear  |  AutoPilotSwitch  |   MissionGoSwitch |
        # --------------------------------------------
        #  -1   |        OFF        |   OFF (Irrelevant)|
        #   0   |        ON         |   OFF             |
        #   1   |        ON         |   ON              |
        self.ctrlThrottle     = data.throttle
        
        if data.gear == self.datagear_previous:
            self.AutoPilotSwitch  = data.gear >= 0
            self.MissionGoSwitch  = data.gear == 1
        else:
            self.datagear_previous = data.gear    
##        print "self.AutoPilotSwitch : ", self.AutoPilotSwitch           
##        print "self.MissionGoSwitch : ", self.MissionGoSwitch  

		## Added by Mingfeng, Jan. 10, 2014
        #self.TaskEngaged = data.flap == -1
        
        
    def battery_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data.voltage)
        self.batteryVoltage  = data.voltage
        
    def poseStamped_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        self.runningStatPose[self.dictionary['x']].push(data.pose.position.x)        
        self.runningStatPose[self.dictionary['y']].push(data.pose.position.y)
        self.runningStatPose[self.dictionary['z']].push(data.pose.position.z)
        self.poseStampedQueue.append(data) #Append the latest msg to the deque, column-wise
        
        euler = tf.transformations.euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
        #phi = euler[0] #- math.pi
        
        #while phi > math.pi:
            #phi = phi -2.0 *math.pi
            
        #while phi < -1.0 * math.pi:
            #phi = phi + 2.0 *math.pi
            
        radTodeg = 180.0/math.pi
        
        outMsg = Point(radTodeg*euler[0], radTodeg*euler[1], radTodeg*euler[2])
        self.publisher_filterpose.publish(outMsg)
               
    def ControllerError_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)

        #Error , e.g. :(z_ref-z)
        self.runningStatError[self.dictionary['x']].push(abs(data.error.x))
        self.runningStatError[self.dictionary['y']].push(abs(data.error.y))
        self.runningStatError[self.dictionary['z']].push(abs(data.error.z))
        
        #Derivative of Error  , e.g. :(d/dt)(z_ref-z) or (velocity_des-velocity)
        self.runningStatError_d[self.dictionary['x']].push(data.derivative.x)
        self.runningStatError_d[self.dictionary['y']].push(data.derivative.y)
        self.runningStatError_d[self.dictionary['z']].push(data.derivative.z)
        
    def clear_vpcmds(self):
        self.totakeoff = False
        self.toland = False
        self.tomove = False
        self.toreset = False
    
