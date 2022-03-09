"""
Author: Mingfeng Zhang @ McGill University
Date: 2014
"""

#!/usr/bin/env python
#General Imports
import roslib #roslib.load_manifest('smach_tutorials')
roslib.load_manifest('fsm')
import rospy
import time
import random
import math 
import numpy
import smach
import smach_ros
#For dealing with msgs
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# The Listener object used in the FSM
#from ListenerClass import ListenerClass
#from beginner_tutorials.srv import *
from Utility import *
from quadrotor_input.srv import *
import tf.transformations
import dynamic_reconfigure.client

#landtakeoff_state_source = 'px4flow'
#non_landtakeoff_state_source = ''

# define state MANUAL
class MANUAL(smach.State):
     def __init__(self, flightStatus, controlManagement):
        smach.State.__init__(self, outcomes=['Finish',
                                            'Monitor',
                                            'TOAUTONOMOUS'])
        self.flightStatus = flightStatus
        self.controlManagement   = controlManagement
        self.dict = {'True': 'ON', 'False':'OFF'}
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state MANUAL')
        rospy.sleep(self.flightStatus.sleepTime)
##        print "\nstable:",   self.flightStatus.getStable() 

        if self.flightStatus.listener.controllerStatus == True:          
            Service_in = CommandControllerRequest()      # defined in /quadrotor_input/srv_gen/cpp/include/quadrotor_input/CommandController.h           
            # Designated whether there are controller gains to be adjusted (Default)
            #Service_in.input_gain_flag = True
            # Controller PID gains (regarded only if flag is TRUE, Default)
            #Service_in.gains = [1.0,2.0,3.0] 
            # Default  - Controller should turn OFF
            Service_in.running = False # Designated that the controller should turn ON.
            #Service_in.path.poses = getWaypoints(self.flightStatus._waypoints) 
            Service_in.path.poses = getTrajectory(self.flightStatus.getCurrentPose(),
                                              self.flightStatus.getCurrentPose())
                                              
            if self.controlManagement.ControllerClient(Service_in):
                print("Controller SUCCEEDED to turn " + self.dict[str(Service_in.running)])
                rospy.logwarn("MANUAL turn off controller!")
            else:
                print("Controller FAILED to turn " + self.dict[str(Service_in.running)])


        if ( self.flightStatus.IsBatteryOK() ) and ( self.flightStatus.listener.AutoPilotSwitch == True ): # and self.flightStatus.getStable() == True  :
            print ("MANUAL: AutoPilot switch is ON; Battery is OK ----> Entering AUTO mode")                 
            return 'TOAUTONOMOUS'
             
        if self.flightStatus.IsTimeExceeded() : 
            print ("MANUAL: Mission Duration Exceeded ---> Finish") 
            return 'Finish'
        else:
            print (">>> Manual ...")    
            return 'Monitor'
            
class AUTONOMOUS_INIT(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self,outcomes=['ToIdle',
                                            'ToHover',
                                            'ToLand',
                                            'Failure'])
##                                  input_keys=['AutoInit_mission_stage_in'],
##                                  output_keys=['AutoInit_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state AUTONOMOUS_INIT')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.IsBatteryOK() and self.flightStatus.listener.AutoPilotSwitch == True : 
        
            self.flightStatus.listener.clear_vpcmds() # clear history cmds from virtual pilot
            
            z = self.flightStatus.getCurrentAltitude()
            if (z - self.flightStatus.getGroundLevel() ) > self.flightStatus.getSafeAltitude(): #Safe
                print ("AUTONOMOUS_INIT: Vehicle above minimal safe altitude ---> goto HOVER")                 
                return 'ToHover'
            #elif (z - self.flightStatus.getGroundLevel()) < 0.3*self.flightStatus.tolerance['mean_tol']: #On the GROUND
            elif (z - self.flightStatus.getGroundLevel()) < 0.05: #On the GROUND
                print ("AUTONOMOUS_INIT: Vehicle seems to be still on the ground ---> goto IDLE")                                 
                return 'ToIdle'
            else :
                print ("AUTONOMOUS_INIT: Vehicle in intermediate altitude ---> goto LAND") 
                return 'ToLand' #Intermediate altitude - LAND!
        else:
            return 'Failure'


# define state TAKEOFF
class TAKEOFF(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Success',
                                             'Aborted_NoBatt',
                                            'Aborted_Diverge',
                                            'Maintain'])
##                                    input_keys  = ['TakeOff_mission_stage_in'],
##                                    output_keys = ['TakeOff_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state TAKEOFF')
        rospy.sleep(self.flightStatus.sleepTime)        
        if ( self.flightStatus.listener.AutoPilotSwitch == False ) or  ( self.flightStatus.PositionErrorDiverge()== True ):
            print ("TAKEOFF: Either pilot wants control back or vehicle is unstable ---> goto MANUAL")                 
            return 'Aborted_Diverge'
        elif (self.flightStatus.listener.MissionGoSwitch == False ) or ( self.flightStatus.IsBatteryOK()== False ):
            print ("TAKEOFF: Either pilot wants vehicle to go home or Battery is low ---> goto LAND")#Later should be mapped to GOHOME state
            return 'Aborted_NoBatt'
        #if self.flightStatus.PositionErrorConverge(): 
        if self.flightStatus.AltitudeErrorConverge(): 
            print ("TAKEOFF: Takeoff completed ---> goto HOVER")
            self.flightStatus.listener.clear_vpcmds() # clear history cmds from virtual pilot
            return 'Success'
        print (">>> TakingOff ...")#Later should be mapped to GOHOME state        
        return 'Maintain'

            
# define state HOVER
class HOVER(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Aborted_NoBatt',
                                            'Aborted_Diverge',
                                            'Maintain',
                                            'GoHome',
                                            'FollowTraj',
                                            'ReHover'])
##                                    input_keys  = ['Hover_mission_stage_in'],
##                                    output_keys = ['Hover_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state HOVER')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.PositionErrorDiverge() or (self.flightStatus.listener.AutoPilotSwitch is False) :
            print ("HOVER: Either pilot wants control back or vehicle is unstable ---> goto MANUAL")                 
            return 'Aborted_Diverge' #->Manual!
        if ( not self.flightStatus.IsBatteryOK() ) or (self.flightStatus.listener.MissionGoSwitch is False)  or self.flightStatus.IsTimeExceeded() : 
            #print ("HOVER: Either pilot wants vehicle to go home, duration exceeded or Battery is low\n")#Later should be mapped to GOHOME state
            #print "Battery Voltage Level: " ,self.flightStatus.getCurrentBatteryVoltage()
            #print "MissionGo Switch: " ,self.flightStatus.listener.MissionGoSwitch 
             
            if self.flightStatus.IsHome():
                print ("HOVER: Vehicle is home, duration exceeded or Battery is low ---> goto LAND")
                return 'Aborted_NoBatt' #->Vehicle should LAND
            else:
                print ("HOVER: Either pilot wants vehicle to go home, duration exceeded or Battery is low ---> goto Go_Home") 
                return 'GoHome' #->Vehicle should return home
                
        if (self.flightStatus.listener.toland is True) :
            self.flightStatus.listener.clear_vpcmds()
            print ("HOVER: Virtual Pilot wants to land---> goto LAND")
            return 'Aborted_NoBatt' #->Vehicle should LAND
         
        if (self.flightStatus.listener.changeStateSource is True) :
            self.flightStatus.listener.changeStateSource = False
            self.flightStatus.chooseStateSource(self.flightStatus.listener.stateSource)
            return 'ReHover' #->Restart hovering, allows for a change to reset the hovering position
            
        #print "self.flightStatus.DistanceToTarget(3)", self.flightStatus.DistanceToTarget(3)  
        
        ## Modified by Mingfeng, Jan. 10. 2014      
        #if self.flightStatus.DistanceToTarget(3) > 15 * self.flightStatus.tolerance: # 3D distance
        if ((self.flightStatus.switchToFollowWP is True) or (self.flightStatus.listener.tomove is True)):
            rospy.loginfo("Manually switched to follow waypoints: %d", self.flightStatus.switchToFollowWP)
            #self.flightStatus.switchToFollowWP = False
            
            self.flightStatus.listener.clear_vpcmds()

            print("HOVER: Manually switched to follow waypoints ---> goto FollowTraj")
            return 'FollowTraj'
        print(">>> Hovering ...")
        return 'Maintain'
            
                
# define state LAND
class LAND(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Success',
                                            'Failure',
                                            'Maintain'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state LAND')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.PositionErrorDiverge() or self.flightStatus.listener.AutoPilotSwitch==False:
            print ("LAND: Vehicle is unstable ---> goto MANUAL")                 
            return 'Failure' #->Manual!
        #if self.flightStatus.PositionErrorConverge():    
        #if self.flightStatus.AltitudeErrorConverge():
        if self.flightStatus.TouchDown(0.05):
            print ("LAND: Vehicle has landed ---> goto IDLE")
            self.flightStatus.listener.clear_vpcmds()                 
            return 'Success' #->Idle
        print (">>> Landing ...")                  
        return 'Maintain' #Remain in Land!
     
               
# define state IDLE
class IDLE(smach.State):
     def __init__(self,flightStatus, controlManagement):
        smach.State.__init__(self, outcomes=['Finish',
                                             'Start',
                                            'Maintain'])
##                                   input_keys  = ['Idle_mission_stage_in'],
##                                    output_keys = ['Idle_mission_stage_out'])
        self.controlManagement   = controlManagement
        self.flightStatus = flightStatus
                
     def execute(self, userdata):
##        rospy.loginfo('Executing state IDLE')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.listener.AutoPilotSwitch == False or  not self.flightStatus.IsBatteryOK():
            print ('IDLE: All controllers are turned off') 
            #Waited for a while in idle or one of the switches is OFF
            print ("IDLE: AutoPilot is OFF or battery is low ---> goto MANUAL") 
            return 'Finish' #- to manual
        elif (self.flightStatus.IsThrottleUp() or self.flightStatus.listener.totakeoff) and self.flightStatus.IsBatteryOK() and self.flightStatus.listener.MissionGoSwitch == True : 
            
            #Throttle is up or virtual pilot wants to take off, and there is enough battery
            self.flightStatus.listener.clear_vpcmds()
            print ("IDLE: Pilot wants to take off and battery is OK ---> goto TAKEOFF") 
            return 'Start'  #- to takeoff
        #elif (self.flightStatus.listener.changeStateSource is True) :
            #self.flightStatus.listener.changeStateSource = False
            #self.flightStatus.chooseStateSource(self.flightStatus.listener.stateSource)    
            #State_Select_Service = SelectStateSourceRequest()
            #self.flightStatus.getStateSource(State_Select_Service)
            #self.controlManagement.SelectStateSource(State_Select_Service)
        else:
            self.flightStatus.listener.clear_vpcmds()
            
        print(">>> Idle ...")    
        return 'Maintain'

# define state FOLLOWTRAJECTORY  (This state should be is a template for GoHome or any other follow traj, the only difference is what is the trajectory to follow)
class FOLLOW_TRAJECTORY(smach.State):
     def __init__(self,flightStatus,str_ParentStateName):
        smach.State.__init__(self, outcomes=['Arrived',
                                            'Aborted_Diverge',
                                            'Maintain'])
##                                   input_keys  = ['TrajFol_mission_stage_in'],
##                                   output_keys = ['TrajFol_mission_stage_out'])
        self.flightStatus = flightStatus
        self.str_ParentStateName = str_ParentStateName
                
     def execute(self, userdata):
##        rospy.loginfo('Executing state FOLLOW_TRAJECTORY inside %s', self.str_ParentStateName )
        rospy.sleep(self.flightStatus.sleepTime)
        #Should add that is in follow trajectory and target pose is homepose, then maintain...
        if self.flightStatus.PositionErrorDiverge() or self.flightStatus.listener.AutoPilotSwitch == False:
            print (self.str_ParentStateName + " : Pilot wants control back or vehicle is unstable ---> goto MANUAL")                 
            return 'Aborted_Diverge' #--->>>Manual!
                
        if self.flightStatus.PositionErrorConverge() and (self.flightStatus.listener.wpIndex == self.flightStatus.wpLen) : #Regardless of parent container, if arrived at destination, should goto HOVER
        #if self.flightStatus.PositionErrorConverge(): 
        #if self.flightStatus.AltitudeErrorConverge():           
            print (self.str_ParentStateName + " : Vehicle arrived at destination ---> goto HOVER") 
            self.flightStatus.listener.clear_vpcmds() # clear history cmds from virtual pilot
            self.flightStatus.wpIndex = self.flightStatus.wpIndex + 1
            if self.flightStatus.wpIndex >= self.flightStatus.wpLen:
                self.flightStatus.wpIndex = 0
            return 'Arrived'
        
        for case in switch(self.str_ParentStateName):
            if case('GO_HOME'):
                if self.flightStatus.IsBatteryOK() and not self.flightStatus.IsTimeExceeded() and self.flightStatus.listener.MissionGoSwitch == True :
                    return 'Arrived' #->Vehicle should go to HOVER
                else:
                    print (">>> GO_HOME: Vehicle returning home ...")
                    return 'Maintain' #->Vehicle should continue going home
                break
            if case('FOLLOW_TRAJ'):
                if self.flightStatus.listener.MissionGoSwitch == False or not self.flightStatus.IsBatteryOK() or self.flightStatus.IsTimeExceeded() :
                    self.flightStatus.listener.clear_vpcmds() # clear history cmds from virtual pilot
                    return 'Arrived' #->Vehicle should go to HOVER
                break

        print(">>> " + self.str_ParentStateName + " : Following Trajectory ...")
        return 'Maintain'
            

# define state InitContoller
class CONTROLLER_INIT(smach.State):
    def __init__(self, flightStatus, controlManagement , str_ParentStateName):
        smach.State.__init__(self, outcomes=['Success','Failure'])
        self.flightStatus        = flightStatus
        self.controlManagement   = controlManagement
        self.str_ParentStateName = str_ParentStateName #Used to indicate which controller should be turned ON
        self.dict = {'True': 'ON', 'False':'OFF'}
        #self.stateDictionary = {'IDLE': 1, 'HOVER':2,'LAND':3,'TAKEOFF':4,'GO_HOME':5}

    def execute(self,userdata):
##        rospy.loginfo('Executing state ControllerInit in %s' , self.str_ParentStateName )
        rospy.sleep(self.flightStatus.sleepTime)
        # Create a service client
        Service_in = CommandControllerRequest()      # defined in /quadrotor_input/srv_gen/cpp/include/quadrotor_input/CommandController.h   
        
        ##### May 27
        State_Select_Service = SelectStateSourceRequest()    # defined in /quadrotor_input/srv_gen/cpp/include/quadrotor_input/SelectStateSource.h    
            
        # Designated whether there are controller gains to be adjusted (Default)
        #Service_in.input_gain_flag = True
        # Controller PID gains (regarded only if flag is TRUE, Default)
        #Service_in.gains = [1.0,2.0,3.0] 
        # Default  - Controller should turn ON
        Service_in.running = True # Designated that the controller should turn ON.
                
        if self.str_ParentStateName is 'IDLE':
##            print "SwitchCase IDLE"
            if self.flightStatus.listener.AutoPilotSwitch == False or self.flightStatus.listener.MissionGoSwitch == False or not self.flightStatus.IsBatteryOK():
                print ('IDLE INIT: All Controllers should be turned off...') 
                # Designated that the controller should turn OFF
                Service_in.running = False
            else: ## Set the current position as the target position, and turn ON the controller.
                #print("IDLE: Getting ready to start mission...\n")
                Service_in.running = False
                print ("IDLE INIT: Creating a target pose to as a constant reference ...")
                self.flightStatus.setTargetPose(self.flightStatus.getCurrentPose().position, self.flightStatus.getCurrentPose().orientation)
                self.flightStatus._targetPose.position.z = self.flightStatus.getCurrentAltitude()
            
        else:
            for case in switch(self.str_ParentStateName):
                               
                if case('HOVER'): ## Set the current position as the target position, and turn ON the controller.
##                    print "SwitchCase HOVER"
##                    print("Starting Controller for HOVER")

                
                    ################ May 27
                    self.flightStatus.listener.changeStateSource = False
                    
                    self.flightStatus.getStateSource(State_Select_Service)
                    self.controlManagement.SelectStateSource(State_Select_Service)
                    ################

                    print ("HOVER INIT: Creating hovering pose ...")
                    
                    poseStamped = self.flightStatus.getCurrentPoseStamped()
                    while poseStamped.header.frame_id != self.flightStatus._state_id_insue:
                        poseStamped = self.flightStatus.getCurrentPoseStamped()
                        
                    currentPose = poseStamped.pose
                    
                    #currentPose = self.flightStatus.getCurrentPose()
                    self.flightStatus.setTargetPose(currentPose.position, currentPose.orientation)            
                    break
##                Service_in.path.poses.append(self.flightStatus.getCurrentPoseStamped()) 
                    
                if case('LAND'):
##                    print "SwitchCase LAND"
                    ################ May 27
                    #self.flightStatus.getStateSourceForLandTakeoff(State_Select_Service)
                    #self.controlManagement.SelectStateSource(State_Select_Service)
                    ################
                    print 'LAND INIT: Creating landing pose ...'
                    
                    #poseStamped = self.flightStatus.getCurrentPoseStamped()
                    #while poseStamped.header.frame_id != self.flightStatus.landtakeoff_state_id:
                        #poseStamped = self.flightStatus.getCurrentPoseStamped()
                        
                    #currentPose = poseStamped.pose
                    currentPose = self.flightStatus.getCurrentPose()
                    self.flightStatus.setTargetPose(currentPose.position, currentPose.orientation)                
                    self.flightStatus._targetPose.position.z = self.flightStatus.getGroundLevel()
                    break

                if case('TAKEOFF'):
##                    print "SwitchCase TAKEOFF"
                    ################ May 27
                    #self.flightStatus.getStateSourceForLandTakeoff(State_Select_Service)
                    #self.controlManagement.SelectStateSource(State_Select_Service)
                    ################
                    
                    print 'TAKEOFF INIT: Creating target pose of take-off ...'
                    # current_quat = self.flightStatus.getCurrentPose().orientation
                    # euler = tf.transformations.euler_from_quaternion((current_quat.x,current_quat.y,current_quat.z,current_quat.w))
                    # desired_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, euler[2]) 
                    # Quaternion(desired_quat[0], desired_quat[1], desired_quat[2], desired_quat[3])
                    #poseStamped = self.flightStatus.getCurrentPoseStamped()
                    #while poseStamped.header.frame_id != self.flightStatus.landtakeoff_state_id:
                        #poseStamped = self.flightStatus.getCurrentPoseStamped()
                        
                    #currentPose = poseStamped.pose
                    
                    currentPose = self.flightStatus.getCurrentPose()
                    self.flightStatus.setTargetPose(currentPose.position, currentPose.orientation)                
                    self.flightStatus._targetPose.position.z = self.flightStatus._homeAltitude #Modify the z value of the private targetPose attribute
                    break
                
                if case('GO_HOME'):
##                    print "SwitchCase GOHOME"
                    ################ May 27
                    #self.flightStatus.getStateSourceForLandTakeoff(State_Select_Service)
                    #self.controlManagement.SelectStateSource(State_Select_Service)
                    ################
                    print 'GO_HOME INIT: Creating home pose ...'
                    #quat_1 = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi) 
                    self.flightStatus.setTargetPose(self.flightStatus.getHomePose().position, self.flightStatus.getHomePose().orientation)# Quaternion(quat_1[0],quat_1[1],quat_1[2],quat_1[3])) 
                    #self.flightStatus._targetPose.position.z = self.flightStatus._homeAltitude
                    break
                
                ## Added by Mingfeng, Jan. 10, 2014
                if case('FOLLOW_TRAJ'):
##					print "SwitchCase FOLLOW_TRAJ"
                    print 'FOLLOW_TRAJ INIT: Creating waypoings ...'
                    ############ Move between two points ##############################
                    #current_position = self.flightStatus.getCurrentPose().position
                    #current_x = current_position.x
                    #current_y = current_position.y
                    #tgt_1 = Point(0.1, 0.0, self.flightStatus._homeAltitude)
                    #tgt_2 = Point(-0.3, -0.6, self.flightStatus._homeAltitude)
                    #quat_1 = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0*math.pi/2.0) 
                    #quat_2 = tf.transformations.quaternion_from_euler(0.0, 0.0, -0.0*math.pi/2.0) 
                    #d_1 = (current_x -tgt_1.x)*(current_x -tgt_1.x) + (current_y -tgt_1.y)*(current_y -tgt_1.y) 
                    #d_2 = (current_x -tgt_2.x)*(current_x -tgt_2.x) + (current_y -tgt_2.y)*(current_y -tgt_2.y) 
                    
                    #if d_1 > d_2:
                        #self.flightStatus.setTargetPose(tgt_1, Quaternion(quat_1[0],quat_1[1],quat_1[2],quat_1[3]))
                    #else:
                        #self.flightStatus.setTargetPose(tgt_2, Quaternion(quat_2[0],quat_2[1],quat_2[2],quat_2[3]))
                    ####################################################
                        
                    ######## Change height ############################    
                    #self.flightStatus.setTargetPose(self.flightStatus.getCurrentPose().position) 
                    #self.flightStatus._targetPose.position.z = self.flightStatus._homeAltitude    
                    #if math.fabs (self.flightStatus.getCurrentPose().position.z - self.flightStatus._homeAltitude) < math.fabs (self.flightStatus.getCurrentPose().position.z - self.flightStatus._homeAltitude -0.8):
                        #self.flightStatus._targetPose.position.z = self.flightStatus._homeAltitude + 0.5
                        
                    ######################################
                    self.flightStatus.setTargetPose(self.flightStatus._waypoints[-1].position, self.flightStatus._waypoints[-1].orientation)   
                    #self.flightStatus.setTargetPose(self.flightStatus._waypoints[self.flightStatus.wpIndex].position, self.flightStatus._waypoints[self.flightStatus.wpIndex].orientation)           
                    Service_in.path.poses = getWaypoints(self.flightStatus._waypoints) 
                    #waypoints = []
                    #waypoints.append(self.flightStatus._targetPose)
                    #Service_in.path.poses = getWaypoints(waypoints)
                    self.flightStatus.wpLen = len(self.flightStatus._waypoints)
                    break
					
##        print "Prior to generating a trajectory"
##        print "Current:" , self.flightStatus.getCurrentPose()
##        print "Target:", self.flightStatus.getTargetPose()
        # Call a function that generates a trajectory for the controller to follow - - >>>> SHOULD BE A SERVICE PROVIDOR     
        if not (self.str_ParentStateName is 'FOLLOW_TRAJ'):  
            waypoints = []
            waypoints.append(self.flightStatus._targetPose)
            Service_in.path.poses = getWaypoints(waypoints)
            self.flightStatus.wpLen = 1            
            #Service_in.path.poses = getTrajectory(self.flightStatus.getCurrentPose(),
            #                                    self.flightStatus.getTargetPose())
        
        if self.controlManagement.ControllerClient(Service_in):
            print(self.str_ParentStateName + " Controller SUCCEEDED to turn " + self.dict[str(Service_in.running)])
            return 'Success'
        else:
            print(self.str_ParentStateName + " Controller FAILED to turn " + self.dict[str(Service_in.running)])
            return 'Failure'

