#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'state_estimation'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import math

# ROS messages.
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

rad2deg = 180.0 / math.pi

class MagToYaw():
    def __init__(self):
        self.got_new_msg = False
        self.yaw_msg = Float32()
        
        self.degrees = rospy.get_param('~degrees', True)

        # Create subscribers and publishers.
        sub_mag  = rospy.Subscriber("magnetometer", Vector3Stamped, self.mag_callback)
        pub_yaw = rospy.Publisher("yaw", Float32)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_yaw.publish(self.yaw_msg)
                self.got_new_msg = False

    # Magnetometer callback function.
    def mag_callback(self, msg):
        self.yaw_msg.data = math.atan2(msg.vector.y, msg.vector.x)
        if self.degrees:
          self.yaw_msg.data = self.yaw_msg.data * rad2deg
        
        self.got_new_msg = True
      

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('mag_to_yaw', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        mag_to_yaw = MagToYaw()
    except rospy.ROSInterruptException: pass
