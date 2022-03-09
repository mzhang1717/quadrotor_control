#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'state_estimation'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import math

# ROS messages.
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped

rad2deg = 180.0 / math.pi

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Vector3Stamped()
        
        self.degrees = rospy.get_param('~degrees', True)
        
        self.br = tf.TransformBroadcaster()

        # Create subscribers and publishers.
        self.sub_quat  = rospy.Subscriber("quaternion", QuaternionStamped, self.quat_callback)
        self.sub_pose  = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        pub_euler = rospy.Publisher("euler", Vector3Stamped)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                self.got_new_msg = False

    # Odometry callback function.
    def pose_callback(self, msg):
      
        self.br.sendTransform((0,0,0),
                     [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
                     rospy.Time.now(),
                     self.sub_pose.name + "_rot",
                     msg.header.frame_id)
        
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # IMU callback function.
    def quat_callback(self, msg):
      
        self.br.sendTransform((0,0,0),
                     [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w],
                     rospy.Time.now(),
                     self.sub_quat.name,
                     msg.header.frame_id)
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
        self.fill_euler_msg(msg, r, p, y)

    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.euler_msg.header.stamp = msg.header.stamp
        if self.degrees:
          self.euler_msg.vector.x = r * rad2deg
          self.euler_msg.vector.y = p * rad2deg
          self.euler_msg.vector.z = y * rad2deg
        else:
          self.euler_msg.vector.x = r
          self.euler_msg.vector.y = p
          self.euler_msg.vector.z = y
          
        self.got_new_msg = True

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass
