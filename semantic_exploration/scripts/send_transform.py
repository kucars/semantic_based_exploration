#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def handle_odom_pose(msg, frame):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = frame
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x    = msg.pose.pose.orientation.x
    t.transform.rotation.y    = msg.pose.pose.orientation.y
    t.transform.rotation.z    = msg.pose.pose.orientation.z
    t.transform.rotation.w    = msg.pose.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_gazebo_groundtruth')
    frame = "odom"
    rospy.Subscriber('/odometry',
                     Odometry,
                     handle_odom_pose,
                     frame)
    rospy.spin()

