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
#include <rrt_explorer/rrt_planner_deep.h>
#include <fstream>

enum planningMethod { CLASSIC=1, DENSIT=2, DISTANCE=3,DEFAULT=4 };
planningMethod m = DEFAULT;
int i ;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt_explorer_deep");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //rrtNBV::RRTPlannerDeep plannerDeep(nh,nh_private);
  //ros::param::get("/planMethod",i );

  //while (m == DEFAULT)
  //  continue;
  //int planningMethod = 2  ;
  rrtNBV::RRTPlannerDeep plannerDeep(nh,nh_private,1);
  ros::spin();
  return 0;
}
