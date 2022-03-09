"""
Author: Mingfeng Zhang @ McGill University
Date: 2014
"""

#!/usr/bin/env python


import roslib; roslib.load_manifest('fsm')
import rospy
import numpy
import string
#from message_filters import Subscriber, Cache
from std_msgs.msg import  Bool
from std_msgs.msg import  Int16
from smach_msgs.msg import SmachContainerStatus

import time

class FsmMenifoldIndicatorClass():
    def __init__(self):
        rospy.init_node('manifold_indicator')
        print("Initializing manifold indicator object ...")
        self.topic_in   = 'topic_in'
        self.topic_out  = 'topic_out'
        self.subscriber = rospy.Subscriber(self.topic_in,
                                           SmachContainerStatus,
                                           self.callback)        
        self.publisher  = rospy.Publisher(self.topic_out, Int16) 
        
        #nowTime = time.strftime("%Y-%m-%y-%H-%M-%S")
        #self.logfilename = '/home/mingfeng/' + nowTime + '-fsm.txt'
        
        #f = open(self.logfilename, 'w')
        #f.write('%Time\t' + 'Path\t' + 'State\n')
        #f.close()
        
    def callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        outMsg = Int16(1)
        
        #Extract the string            
        path_str = data.path
##        print str
##        print data.active_states[0]
        #find whether there is MANUAL or AUTONOMOUS pharse in the string 
               
        if path_str != '/SM_TOP/AUTONOMOUS' :
            return 
            
        if data.active_states[0]=='None':
##            print "->>>>>>>>>>>MANUAL"
            outMsg.data = 0
        elif data.active_states[0]=='IDLE':
            outMsg.data = 2
        else:
            outMsg.data = 1 #Auto mode except IDLE
##            print "->>>>>>>>>>>AUTONOMOUS"
            
        #publish that bool
        self.publisher.publish(outMsg)  
        
        #f = open(self.logfilename, 'a')
        #try:
            #f.write(time.strftime("%H:%M:%S") + '\t' + path_str + '\t' + data.active_states[0] + '\n')
        #finally:
            #f.close()


if __name__ == '__main__':
    try:
        converter = FsmMenifoldIndicatorClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
