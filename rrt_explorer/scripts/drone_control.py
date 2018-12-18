#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
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
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233

import rospy
import thread
import threading
import time
from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub

        self.rospy = rospy

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.to = 0.1
        self.currentX = 0
        self.currentY = 0
        self.currentZ = 0
        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

        sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)

    def navigate(self):
        rate = self.rospy.Rate(10) # 10hz
        
        msg = PoseStamped()

        msg.header = Header() 
        msg.header.frame_id = "base_footprint"

        while 1:            
            print "Sending",self.x,self.y,self.z
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = Quaternion(*quaternion)

            self.pub.publish(msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass   

    def set(self, x, y, z,to,delay=0, wait=True):
        rate = rospy.Rate(10)  # Hz
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.to= to
        print x,y,z
        if wait:
            while not self.done:
                rate.sleep()
        
        time.sleep(delay)

    def poseCallback(self, topic):
        print topic.pose.position.x,topic.pose.position.y,topic.pose.position.z
        self.currentX = topic.pose.position.x 
        self.currentY = topic.pose.position.y 
        self.currentZ = topic.pose.position.z 
        if abs(topic.pose.position.x - self.x) < self.to and abs(topic.pose.position.y - self.y) < self.to and abs(topic.pose.position.z - self.z) < self.to:
            self.done = True
            print "Current Pose Drone 03:",topic.pose.position.x,topic.pose.position.y,topic.pose.position.z
            print "Set Pose Drone 03 :",self.x,self.y,self.z
        self.done_evt.set()

def setpoint_demo():
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)    

    rospy.init_node('pose', anonymous=True)
    rate = rospy.Rate(10) 

    setpoint = Setpoint(pub, rospy)
    print "Takeoff here"
    setpoint.set(2.69,1.4,2.8,0.1,10)
    
    print "Done moving"

    while not rospy.is_shutdown():
        print "Closing the code"
        print "NOT MANUAL" 
        rate.sleep()

if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
