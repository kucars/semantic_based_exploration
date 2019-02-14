#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <semantic_exploration/rrt_core.h>
#include <semantic_exploration/rrt_planner.h>
#include <semantic_exploration/rrt_tree.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <thread>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_exploration");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    rrtNBV::RRTPlanner planner(nh, nh_private);

    ros::spin();
    return 0;
}
