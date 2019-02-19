#!/usr/bin/env python
#
# Copyright 2019 .
# Author Tarek Taha : tarek@tarektaha.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

from __future__ import print_function

import sys
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt,pow
from tf import TransformListener
import tf

testName = 'test_navigation'

test_pose_x   = -2
test_pose_y   = 2
test_pose_z   = 1
test_pose_yaw = 1.57079632679

class NavigationTest(unittest.TestCase):
    def __init__(self, *args):
        super(NavigationTest, self).__init__(*args)
        rospy.init_node(testName)
    
    # to run before each test
    def setUp(self):
        self.testpose_reached = False
    
    #to run after each test
    #def tearDown():
    
    def test_navigation(self):
        
        sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.callback)
        pub = rospy.Publisher("semantic_exploration_viewpoint",PoseStamped)

        poseMsg_    = PoseStamped()
        transd_pose = PoseStamped()

        poseMsg_.header.stamp = rospy.Time.now()
        poseMsg_.header.frame_id = "/world"
        poseMsg_.pose.position.x = test_pose_x
        poseMsg_.pose.position.y = test_pose_y
        poseMsg_.pose.position.z = test_pose_z

        quat = quaternion_from_euler (0, 0, test_pose_yaw)

        poseMsg_.pose.orientation.x = quat[0]
        poseMsg_.pose.orientation.y = quat[1]
        poseMsg_.pose.orientation.z = quat[2]
        poseMsg_.pose.orientation.w = quat[3]
    
        listener = TransformListener()
        target_frame = "local_origin"
        transformed = False
        
        while not self.testpose_reached:
            rospy.loginfo("Waiting to reach Location %f %f %f",test_pose_x,test_pose_y,test_pose_z)
            poseMsg_.header.stamp = rospy.Time.now()
            if not transformed:
                try:
                    listener.waitForTransform(poseMsg_.header.frame_id, target_frame, poseMsg_.header.stamp, rospy.Duration(0.1))
                    #position, quaternion = listener.lookupTransform("world", "fcu", rospy.Time())
                    transd_pose = listener.transformPose(target_frame,poseMsg_)
                    transformed = True
                    rospy.loginfo("Pose is: %f %f %f Transformed Pose: %f %f %f",poseMsg_.pose.position.x,poseMsg_.pose.position.y,poseMsg_.pose.position.z, transd_pose.pose.position.x,transd_pose.pose.position.y,transd_pose.pose.position.z)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn('Couldn\'t Transform from "{}" to "{}"  before timeout. Are you updating TF tree sufficiently?'.format(poseMsg_.header.frame_id, target_frame))
                    continue
            else:
                rospy.loginfo("Pose is: %f %f %f Transformed Pose: %f %f %f",poseMsg_.pose.position.x,poseMsg_.pose.position.y,poseMsg_.pose.position.z, transd_pose.pose.position.x,transd_pose.pose.position.y,transd_pose.pose.position.z)
                transd_pose.header.stamp = rospy.Time.now()
                pub.publish(transd_pose)
            rospy.sleep(0.1)
		
        rospy.loginfo("Finished")
        assert(self.testpose_reached)


    def callback(self, msg):
        #rospy.loginfo("Position Received (%f)", msg.pose.position.x)
        dist2Goal = sqrt( pow((msg.pose.position.x - test_pose_x),2)  + pow((msg.pose.position.y - test_pose_y),2) + pow((msg.pose.position.z - test_pose_z),2))
        if  dist2Goal <= 0.1:
            self.testpose_reached = True  
		 

if __name__ == '__main__':
    time.sleep(0.75)
    try:
        rostest.run('test_navigation', testName, NavigationTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting") 
