/**
BSD 3-Clause License

Copyright (c) 2018, Khalifa University Robotics Institute
Copyright (c) 2018, Tarek Taha tarek@tarektaha.com
Copyright (c) 2018, Reem Ashour reemashour1@gmail.com
Copyright (c) 2020, Mohamed Abdelkader mohamedashraf123@gmail.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
