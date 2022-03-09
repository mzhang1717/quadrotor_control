"""
Author: Mingfeng Zhang @ McGill University
Date: 2014
"""

#!/usr/bin/env python#

#A Flight Status Class
#Implements the methods needed to determine flight status such as battery condition, state convergence, distance from home location
#Most function return a boolean indicator pertaining to the query requested.

PKG = 'fsm' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy
import math
from rospy.numpy_msg import numpy_msg
from Utility import *
import copy
import sys
import time
import random
import message_filters
#from message_filters import Subscriber, Cache
from std_msgs.msg import Float64, Bool, Int16
from geometry_msgs.msg import Pose, Point, Quaternion
from rospy.numpy_msg import numpy_msg 
#Import the Listerner and Logger class
from ListenerClass import ListenerClass

from dynamic_reconfigure.server import Server as DynamicReconfigServer
from fsm.cfg import fsm_monitorConfig

class FlightStatusClass():
    """
    Implements the methods needed to determine flight status such as battery condition, state convergence, distance from home location
    """
    def __init__(self,dictionary,queue_size,MinBattVol, ceilingAltitude, safeAltitude, homeAltitude, groundlev, throttleThreshold, MaxTime, home, FSM_refreshRate, tolerance,stable):
        print("Initializing Flight Status Object ...")
        self.listener                = ListenerClass(queue_size,dictionary) #An instance of class listener used as a subscriber and data logger
        self._minimalBatteryVoltage  = MinBattVol                           #Minimal battery voltage allowed for flights
        self._groundLevel            = groundlev                            #Ground altitude in /world frame of reference
        self._ceiling                = ceilingAltitude                      # High limit of operational altitude
        self._safeAltitude           = safeAltitude                         # Safe (Low limit) operational altitude in /world frame of reference
        self._homeAltitude           = homeAltitude
        self._throttleThreshold      = throttleThreshold                    #Throttle stick (TX) threshold above which FSM would consider pilot permission for takeoff
        self._missionStartTime       = rospy.Time.now().to_sec()            #Mission State Time (in ROS)
        self._missionMaxTime         = MaxTime                              #Alloted time for mission
        self._stable                 = stable                               #stable vehicle?
        self.tolerance               = tolerance                            #Distance tolerance [meters] used for convergence and arrival indication
        self.sleepTime               = FSM_refreshRate                      #For debugging - A time delay when entering each state, set to 0.0 when operational
        self.setTargetPose()                                                #Sets default target pose
        self.wpLen = 0
        self.wpIndex = 0
        
        
        
        rospy.sleep(0.1) # wait until self.listener obtains the first reading of the vehicle's position
        if home == []:
            temp_pose = self.getCurrentPose()
            self.setHomePose(Point(temp_pose.position.x, temp_pose.position.y, self._homeAltitude), temp_pose.orientation)
        else:
            self.setHomePose(home.position, home.orientation)                   #Sets initial home pose
        
        self._waypoints = [] # Way-points for the vehicle to follow
        self.switchToFollowWP = False
        
        ############## May 27
        self._state_source_dictionary = {}

        self.landtakeoff_state_id = ''
        self.non_landtakeoff_state_id = ''
        self._state_source_insue = []
        self._state_id_insue = ''
        ##############
        
        # create a Dynamic Reconfiguration Server
        dyn_reconf_server = DynamicReconfigServer(fsm_monitorConfig, self.reconfig_callback) 
    
    def reconfig_callback(self, config, level):
        #rospy.loginfo("Configure {isToWaypoint}".format(**config))
        self.switchToFollowWP = config["isToWaypoint"]
        if self.switchToFollowWP:
            rospy.loginfo("Manually switched to follow waypoints ...")
        return config
        
    def setWaypoints(self, waypoints):
        self._waypoints = copy.deepcopy(waypoints)
        return
        
    ############### May 27
    def setStateSource(self, state_source_dictionary):
        self._state_source_dictionary = state_source_dictionary
        return
        
    def getStateSource(self, State_Select_Service):
        self._state_id_insue = self._state_source_insue[0]
        State_Select_Service.strPoseTopicSelected = self._state_source_insue[1]
        State_Select_Service.strVelocityTopicSelected = self._state_source_insue[2]
        return
        
    def chooseStateSource(self, stateSource):
        self._state_source_insue = self._state_source_dictionary[stateSource]
        self._state_id_insue = self._state_source_insue[0]
        return
    #################   
          
    def changeStable(self):
        """
        :return: void
        
        Accesor function 
        """
        self._stable = not self._stable
        
    def getStable(self):
        """
        :return: boolean
        
        Accesor function 
        """
        return True #self._stable 
    
    def getMinimalBatteryVoltage(self):
        """
        :return: Minimal Allowed Battery Voltage
        
        Accesor function 
        """
        return self._minimalBatteryVoltage
    
    def getGroundLevel(self):
        """
        :return: Ground level in meters in world frame
        
        Accesor function 
        """
        return self._groundLevel

    def getSafeAltitude(self):
        """
        :return: Flight Status Safe Altitude
        
        Accesor function 
        """
        return self._safeAltitude

    def getThrottleThreshold(self):
        """
        :return: Tx Throttle threshold for considering human pilot intention
        
        Accesor Function 
        """        
        return self._throttleThreshold
    
    def getMissionStartTime(self):
        """
        :return: ROS time object designating when the mission has started
        
        Accesor Function 
        """        
        return self._missionStartTime
    
    def getMissionMaxTime(self):
        """
        :return: ROS time object designating maximal allowed mission duration
        
        Accesor Function 
        """        
        return self._missionMaxTime
    
    def getHomePose(self):
        """
        :return: ROS msg of type "geometry_msgs\Pose.msg" designating the [x,y,z] corrdinates of HOME and an arbitrary quaternion
        
        Accesor Function 
        """        
        return self._homePose

    def getCurrentThrottle(self):
        """
        :return: current Tx throttle level
        
        Accesor Function 
        """        
        return self.listener.ctrlThrottle
     
    def getCurrentBatteryVoltage(self):
        """
        :return: current battery voltage
        
        Accesor Function 
        """        
        return self.listener.batteryVoltage
    
    def getCurrentPoseStamped(self):
        """
        :return: Current (latest) Stamped Pose msg type
        
        Accesor Function 
        """        
        #print self.listener.poseStampedQueue    
        return copy.deepcopy(self.listener.poseStampedQueue[-1])
    
    def getCurrentPose(self):
        """
        :return: Current (latest) Stamped Pose msg type
        
        Accesor Function 
        """        
        poseStamped = self.getCurrentPoseStamped()
 
        return copy.deepcopy(poseStamped.pose)
    
    def getCurrentState(self,str_state):
        """
        :param: str_state: string of the state to be returned , either 'x','y','z'
        :return: Current (latest) position attribute, either 'x','y','z'
        
        Accesor Function 
        """
        return copy.deepcopy(getattr(self.getCurrentPose().position,str_state)   )
    
    def getCurrentAltitude(self):
        """
        :return: Current (latest) altitude 'z' 
        
        Accesor Function 
        """        
        return self.getCurrentState('z')
    
    def getMissionDuration(self):
        """
        :return: Mission duration in seconds thus far
        
        Accesor Function for mission duration thus far [seconds]
        """
        return rospy.Time.now().to_sec()-self.getMissionStartTime()
    
    def getTargetPose(self):
        """
        :return: pose
        
        Accesor Function to get the current target pose of the vehicle (to where the controller is aiming to drive)
        """
        return copy.deepcopy(self._targetPose)
        
    def setTargetPose(self,position = Point(0.0,0.0,2.0) ,orientation = Quaternion(0.0,0.0,0.0,1.0) ):
        """
        :return: void
        
        Accesor Function to set the current target pose of the vehicle (to where the controller is aiming to drive)
        """
        
        #Ensure the target pose altitude is above safeAltitude
        #position.z = position.z if position.z > self.getSafeAltitude() else self.getSafeAltitude()
        self._targetPose = Pose(copy.copy(position),copy.copy(orientation))
        
        #for str in 'xyz':
            #self.listener.runningStatError[self.listener.dictionary[str]].clear()
            #self.listener.runningStatError_d[self.listener.dictionary[str]].clear()
        
        return 
    
    def getHomePose(self):
        """
        :return: pose
        
        Accesor Function to get the current home pose of the vehicle 
        """
        return self._homePose 
    
    def setHomePose(self,position = Point(0.0,0.0,2.0) ,orientation = Quaternion(0.0,0.0,0.0,1.0) ):
        """
        :return: void
        
        Accesor Function to set the current target pose of the vehicle (to where the controller is aiming to drive)
        """
        self._homePose = Pose(copy.deepcopy(position),copy.deepcopy(orientation))
        return 
    
    def IsThrottleUp(self):
        """
        :return: A boolean indicating whether TX throttle level is above threshold
        
        Function indicating whether TX throttle level is above threshold
        """
        #Modified so that the throttle must cross two thresholds
        #tempBool = False
        #if self.listener.ctrlThrottle <= 0.6 * self._throttleThreshold:
            #rospy.sleep(0.1)
            #if self.listener.ctrlThrottle >= self._throttleThreshold:
                #tempBool = True
                
        #return tempBool
        
        return self.listener.ctrlThrottle > self._throttleThreshold    
    
    def IsTimeExceeded(self):
        """
        :return: A boolean indicating whether mission duration has exceeded alloted time
        
        Function indicated whether alloted time for mission has be exceeded
        """
        #return self.getMissionDuration()>self.getMissionMaxTime()
        
        return False
        
    def IsHome(self):
        """
        :return: A boolean indicating whether vehicle is within a predefined threshold of the distance home (2D - planar)
        
        Function indicates if vehicle is in home coordinates
        """
        return self.DistanceToHome(3)<self.tolerance['mean_tol']
    
    def ErrorConverge(self,str_attribute):
        """
        :param: str_attribute: string of the state to be returned , either 'x','y','z'
        :return: A boolean indicating whether state has converged 
        
        Utility function to determine if error of controller has converged
        """        
        #Distance to Target Pose
        #print "Check if Error has converged: " 
        #print "\nStart pose: " ,      self.getCurrentPoseStamped().pose
        #print "\nTarget pose: " ,      self.getTargetPose()
        #if self.listener.runningStatError[self.listener.dictionary[str_attribute]].m_n < 1 or self.listener.runningStatError_d[self.listener.dictionary[str_attribute]].m_n < 1:
            #return False
        
        #Controller Errors
        e_mean, e_var = self.listener.runningStatError[self.listener.dictionary[str_attribute]].Mean_Variance()
        e_d_mean, e_d_var = self.listener.runningStatError_d[self.listener.dictionary[str_attribute]].Mean_Variance()
        
        print "\nError Mean: ", e_mean
        print "Error Var: ", e_var       
     
        print "\nError Derivative Mean: ", e_d_mean
        print "Error Derivative Var: " , e_d_var

        bool_error   =  abs(e_mean) < self.tolerance['mean_tol'] and e_var< math.pow(self.tolerance['std_tol'],2)
        bool_error_d =  abs(e_d_mean)  < self.tolerance['mean_tol'] and e_d_var< 10.0*math.pow(self.tolerance['std_tol'],2)
        if bool_error and bool_error_d:
            print "Error and var in " , str_attribute ,"converged"
            return True
        elif not bool_error:
            print "Error in " , str_attribute ,"does NOT converge"
            return False
        else:
            print "Var in " , str_attribute ,"does NOT converge"
            return False    
    
    def PositionErrorConverge(self):
        """
        :param: void
        
        :return: A boolean indicating whether position error has converged and velocity is ~zero (vehicle is hovering)
        """
        distance_to_target = self.DistanceToTarget(3)
##        print "\nDistance to TARGET: " , distance_to_target
        
        temp_bool    = abs(distance_to_target) < self.tolerance['mean_tol'] #Close to target
        if not temp_bool:
            return False
        else:
            for str in 'xyz':
                temp_bool *= self.ErrorConverge(str)
                if not temp_bool: return temp_bool
            return temp_bool
    
    def AltitudeErrorConverge(self):
        distance_to_target = self.DistanceToTarget(3)
##        print "\nDistance to TARGET: " , distance_to_target
        
        temp_bool    = abs(distance_to_target) < self.tolerance['mean_tol'] #Close to target
        if not temp_bool:
            return False
        else:
            temp_bool *= self.ErrorConverge('z') 
            return temp_bool
            
    def TouchDown(self, dist):
        temp_bool = False
        
        if math.fabs(self.getCurrentPose().position.z - self.getTargetPose().position.z) < dist :
            temp_bool = True
       
        return temp_bool        

    def ErrorDiverge(self,str_attribute):
        """
        :param: str_attribute: string of the state to be returned , either 'x','y','z'
        :return: A boolean indicating whether state is in the process of diverging 
        
        Utility function to determine if error of controller is divering / unstable
        """        
        e_d_mean, e_d_var = self.listener.runningStatError_d[self.listener.dictionary[str_attribute]].Mean_Variance()
        e_mean, e_var = self.listener.runningStatError[self.listener.dictionary[str_attribute]].Mean_Variance()
        
##        print "\n------------------------------------------------------------------------------------------------------------------\n"
##        print "\nError Mean" , str_attribute, e_mean
##        print "Error Var" , str_attribute, e_var       
##     
##        print "\nError Derivative Mean of " , str_attribute , e_d_mean
##        print "Error Derivative Var" , str_attribute , e_d_var
                
        #bool_error   = self.listener.runningStatError[self.listener.dictionary[str_attribute]].Mean()    < 1       
        #print "e_d_mean", e_d_mean
##        bool_error   = abs(e_var)  > 100*math.pow(self.tolerance,2)           
        bool_error_d = abs(e_d_mean)  > 500*self.tolerance['mean_tol']       
        if bool_error_d     :
            print "Error derivative mean or variance of position in " ,str_attribute , "diverged"            
            return True
            
        else :
##            print "Error derivative in " ,str_attribute ,"Did not Diverge"            
            return False

    def PositionErrorDiverge(self):
        """
        :return: A boolean indicating whether ANY is in the process of diverging 
        
        Utility function to determine if any errors / states are divering / unstable
        """                
        #bool = True 
        #for str in 'xyz':
            #bool *= not self.ErrorDiverge(str)
            #if not bool:
                #self.changeStable() 
                #return not bool
        return False #not bool

    def DistanceToHome(self,dim):
        """
        :return: A float representing the dim-D Euclidean distance of the vehicle from home
    
        """        
        dist = Distance('Euclidean',
                        PoseMsg2NumpyArrayPosition(self.getCurrentPose() ),
                        PoseMsg2NumpyArrayPosition(self.getHomePose()    ),
                        dim)
        return dist


    def DistanceToTarget(self,dim):
        """
        :return: A float representing the dim-D Euclidean distance of the vehicle from target pose
    
        """        
        dist = Distance('Euclidean',
                        PoseMsg2NumpyArrayPosition(self.getCurrentPose() ),
                        PoseMsg2NumpyArrayPosition(self.getTargetPose()    ),
                        dim)
        return dist
    
    
    def VoltageNeededToGetHome(self):
        """
        :return: A float representing the estimated voltage needed to return home from present location
        
        Utility function computing/estimating the needed voltage to get home from present location
        Can be later implemented as a table lookup [Euclidean Dist, Voltage] or an energy mapping of a trajectory generated by a motion planner called
        """        
        #Presently implements an arbitrary scaling from distance to voltage 
##        CurrentStampedPose = self.getCurrentPoseStamped()
        dist = self.DistanceToHome(2)       
##        print("Vehicle is a distance of %s meters away from home " %dist)
        return 0.0 #dist*0.001 #<<<--- !!!
        
        
    def IsBatteryOK(self):
        """
        :return: A boolean indicating whether battery status is ok
        
        Function indicating whether battery status is ok (sufficient voltage to continue mission) 
        taking into account minimal safe voltage level,distance from home coordinates and present voltage
        """
        #Compute whether battery level is sufficient based on present voltage level, battery predefined threshold distance to home
##        print ('\n\nCurrent Battery Voltage' , self.listener.batteryVoltage )        
##        print ('Minimal Batt Voltage allowed :' , self.minimalBatteryVoltage )
##        print ('Battery to get HOME:' , self.VoltageNeededToGetHome() )
        current_batt = self.getCurrentBatteryVoltage()
        if current_batt < self.getMinimalBatteryVoltage() + self.VoltageNeededToGetHome() : 
            print('Battery Low ... V_BATT = %f' %current_batt)            
            return False
        else :
            print ('Battery OK ... V_BATT = %f' %current_batt)
            return True
    
##    def window(self,size):
##        return numpy.ones(size)/float(size) #Uniform weights
    
   

 
