#ifndef RRT_PLANNER_DEEP_H
#define RRT_PLANNER_DEEP_H
#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <rrt_explorer/rrt_tree.h>
#include <rrt_explorer/mesh_structure.h>
#include <rrt_explorer/rrt_srv_pose.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Float32.h>
#include <rrt_explorer/information_gain_base.h>
#include <rrt_explorer/information_gain_classic.h>
#include <rrt_explorer/information_gain_point_density.h>
#include <rrt_explorer/information_gain_average_entropy.h>

namespace rrtNBV {
class RRTPlannerDeep{
public:
  RRTPlannerDeep(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  RRTPlannerDeep(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private , int planning_method_ );
  ~RRTPlannerDeep();
  void posStampedCallback(const geometry_msgs::PoseStamped& pose);
  bool plannerCallbackDeep(rrt_explorer::rrt_srv_pose::Request& req, rrt_explorer::rrt_srv_pose::Response& res);
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  bool isReady();
  void setHandlers(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  bool setParams();
  rrtNBV::Params getParams();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  int planning_method_ ;

  ros::Subscriber posStampedClient_;
  ros::Subscriber pointcloud_sub_;
  ros::ServiceServer plannerServiceDeep_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;
  rrtNBV::RrtTree *rrtTree;
  //rrtNBV::InformationGainBase *treeBase;
  rrtNBV::InformationGainClassic *treeClassical;
  rrtNBV::InformationGainPointDensity *treePointDensity;
  rrtNBV::InformationGainAverageEntropy *treeAverageEntropy;

  bool ready_;
  rrtNBV::Params params_;
  std::string logFilePathR_;

};
}
#endif // RRT_PLANNER_DEEP_H
