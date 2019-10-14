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
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "semantic_exploration/common.h"
//rrtNBV::RRTPlanner* planner ; // (nh, nh_private);
//TimeProfiler timer;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_exploration");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");


    /*std::string serialization_file = "nbv_serialization.dat";
    if (!ros::param::get( "/nbv_serialization_file", serialization_file))
    {
        ROS_WARN("No serialization_file value specified %s", serialization_file );
    }*/

  //bool is_save_state , is_load_state;
  //std::string serialization_file = "~/catkin_ws/src/nbv_serialization.dat";
  //ros::param::param("~debug_save_state", is_save_state, false);
  //ros::param::param("~debug_load_state", is_load_state, false);
  //is_save_state = true ;  
  /*if (is_load_state)
  {
    try
    {
      // Create and open an archive for input
      std::ifstream ifs(serialization_file);
      boost::archive::text_iarchive ia(ifs);
      // read class state from archive
      ia >> planner;
    }
    catch (...)
    {
    is_load_state = false;
    planner = new rrtNBV::RRTPlanner(nh, nh_private);
    }
  }
  else
  {
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    planner = new rrtNBV::RRTPlanner(nh, nh_private);
  }*/

  rrtNBV::RRTPlanner planner(nh, nh_private);
  //planner = new rrtNBV::RRTPlanner(nh, nh_private);

 

  /*if (is_save_state)
  {  
      // Save data to archive
      std::ofstream ofs(serialization_file);
      boost::archive::text_oarchive oa(ofs);
      oa << planner;
  } */

  ros::spin();
  return 0;
}
