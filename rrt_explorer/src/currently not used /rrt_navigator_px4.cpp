/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <rrt_explorer/rrt_srv.h>
#include <geometry_msgs/PoseStamped.h>

#include <fstream>
/* sqrt example */
#include <stdio.h>      /* printf */
#include <math.h>       /* sqrt */
//float distance;
bool distanceFlag = false;
using namespace std;
double traveledDistance ;
geometry_msgs::PoseStamped currentPose;
geometry_msgs::PoseStamped pastPose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  if (!distanceFlag)
  {
    distanceFlag = true ;
    //current_pose  = *pose;
    pastPose.header.stamp       = ros::Time::now();
    pastPose.header.frame_id    = pose->header.frame_id;
    pastPose.pose.position.x    = pose->pose.position.x;
    pastPose.pose.position.y    = pose->pose.position.y;
    pastPose.pose.position.z    = pose->pose.position.z;
    pastPose.pose.orientation   = pose->pose.orientation;
    traveledDistance = 0.0;
  }
  else {
    //current_pose  = *pose;
    currentPose.header.stamp       = ros::Time::now();
    currentPose.header.frame_id    = pose->header.frame_id;
    currentPose.pose.position.x    = pose->pose.position.x;
    currentPose.pose.position.y    = pose->pose.position.y;
    currentPose.pose.position.z    = pose->pose.position.z;
    currentPose.pose.orientation   = pose->pose.orientation;
    double diffX = currentPose.pose.position.x - pastPose.pose.position.x;
    double diffY = currentPose.pose.position.y - pastPose.pose.position.y;
    double diffZ = currentPose.pose.position.z - pastPose.pose.position.z;
    traveledDistance += sqrt((diffX*diffX) +(diffY*diffY) + (diffZ*diffZ) ) ;
    //std::cout << "TraveledDistance " << traveledDistance << std::endl ;
    pastPose.pose.position.x    = currentPose.pose.position.x;
    pastPose.pose.position.y    = currentPose.pose.position.y;
    pastPose.pose.position.z    = currentPose.pose.position.z;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt_exploration");
  ros::NodeHandle nh;

  ros::Publisher  pose_pub_      = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/rrt_waypoint", 100);
  ros::Subscriber local_pos_sub  = nh.subscribe<geometry_msgs::PoseStamped> ("/uav_1/mavros/local_position/pose", 100, pose_cb);
  geometry_msgs::PoseStamped poseMsg_ ;

  std::string logFilePath_ = ros::package::getPath("rrt_explorer") + "/data/params";
  system(("mkdir -p " + logFilePath_).c_str());
  logFilePath_ += "/";

  ROS_INFO("Started exploration");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused)
  {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused)
  {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else
  {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  double dt = 1.0;
  std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt))
  {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",(ns + "/nbvp/dt").c_str());
    return -1;
  }

  int seqNum = 0;

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();
  ros::Rate rate(20.0);

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  ros::Time startTime = ros::Time::now() ;
  while (ros::ok())
  {
    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
    rrt_explorer::rrt_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.seq = iteration;
    planSrv.request.header.frame_id = "world";
    ros::service::waitForService("rrt_planner",ros::Duration(1.0));
    if (ros::service::call("rrt_planner", planSrv))
    {
      seqNum++;
      if (planSrv.response.path.size() == 0)
      {
        ROS_INFO(" ****************** No Path *************************");
        ros::Duration(1.0).sleep();
      }
      else
        ROS_INFO(" #################### Path Found #######################");
      for (int i = 0; i < planSrv.response.path.size(); i++)
      {
        poseMsg_.header.seq = seqNum ;
        poseMsg_.header.stamp = ros::Time::now();
        poseMsg_.header.frame_id = "world";
        poseMsg_.pose.position.x = planSrv.response.path[i].position.x ;
        poseMsg_.pose.position.y = planSrv.response.path[i].position.y ;
        poseMsg_.pose.position.z = planSrv.response.path[i].position.z ;

        tf::Pose pose;
        tf::poseMsgToTF(planSrv.response.path[i], pose);
        double yaw = tf::getYaw(pose.getRotation());
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);

        poseMsg_.pose.orientation.x = quat[0] ;
        poseMsg_.pose.orientation.y = quat[1] ;
        poseMsg_.pose.orientation.z = quat[2] ;
        poseMsg_.pose.orientation.w = quat[3] ;

        pose_pub_.publish(poseMsg_);
        ros::spinOnce();
        ros::Duration(dt).sleep();
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1, "Planner not reachable");
      ros::Duration(1.0).sleep();
    }
    iteration++;

    // ********* save the travled distance and total time **************** //
    if (iteration == 30 )
    {
      ros::Time lastTime = ros::Time::now() ;
      ros::Duration diff = lastTime - startTime ;
      ofstream fileTimeAndDistance_;
      fileTimeAndDistance_.open (logFilePath_+"distanceAndTime.csv",std::ios_base::app);
      fileTimeAndDistance_ << "Time,distance\n";
      fileTimeAndDistance_ << diff << ","<<traveledDistance <<  "\n";
    }
    ros::spinOnce();
    rate.sleep();
  }
}
