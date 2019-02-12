#!/usr/bin/env python  
import rospy
import math
import tf
import tf2_ros
import geometry_msgs.msg
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

def handle_fcu_pose(msg, frameName):
    print "I received a pose:",msg.pose.position.x ,msg.pose.position.y,msg.pose.position.z
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = frameName
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z  
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('fcu_to_world_broadcaster')
    poseTopicName = '/odm'
    frameName = 'fcu'
    mavros.set_namespace('/uav_1/mavros')
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'),SP.PoseStamped,handle_fcu_pose,frameName)
    rospy.spin() 
