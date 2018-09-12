#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <rrt_explorer/rrt_core.h>
#include <rrt_explorer/rrt_tree.h>
#include <thread>
#include <chrono>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <rrt_explorer/rrt_planner.h>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt_explorer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  rrtNBV::RRTPlanner planner(nh,nh_private);

  ros::spin();
  return 0;
}
