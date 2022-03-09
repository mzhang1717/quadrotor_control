#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rostopic import *
import sys

max_delay = -1e10
min_delay = 1e10

def callback(msg):
    
    global max_delay
    global min_delay
  
    nowTime = rospy.Time.now()
    deltaTime = nowTime - msg.header.stamp
    
    if deltaTime.to_sec() > max_delay:
      max_delay = deltaTime.to_sec()
      
    if deltaTime.to_sec() < min_delay:
      min_delay = deltaTime.to_sec()
  
    deltaTimeString = "Delta: %f" % deltaTime.to_sec()
    minString = "Min: %f" % min_delay
    maxString = "Max: %f" % max_delay
    
    rospy.loginfo(deltaTimeString + " | " + minString + " | " + maxString)


def listener():
    rospy.init_node('measure_delay')
    
    topic = sys.argv[1]
    msg_class, real_topic, _ = get_topic_class(topic, blocking=True)
    
    print msg_class
    print real_topic
    
    sub = rospy.Subscriber(real_topic, msg_class, callback)
    #rospy.loginfo("Listing to: %s" % 
    rospy.spin()


if __name__ == '__main__':
    listener()
