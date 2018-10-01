/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
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

#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>
#include <rrt_explorer/mesh_structure.h>
#include <tf/transform_datatypes.h>
namespace rrtNBV {

struct Params
{
  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;

  double igProbabilistic_;
  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double igArea_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  bool exact_root_;
  int initIterations_;
  int cuttoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  bool softBounds_;
  Eigen::Vector3d boundingBox_;

  double meshResolution_;

  ros::Publisher inspectionPath_;
  ros::Publisher sampledPoints_;
  ros::Publisher explorationarea_;
  ros::Publisher transfromedPoseDebug;
  ros::Publisher rootNodeDebug;
  ros::Publisher sensor_pose_pub_ ; 
  
  //ros::Publisher camboundries_;
  //ros::Publisher fovHyperplanes;
  std::string navigationFrame_;

  //debugging
  visualization_msgs::MarkerArray pa;
  visualization_msgs::Marker camParam;

  bool log_;
  double log_throttle_;
  double pcl_throttle_;
  double inspection_throttle_;
  
  
  std::string output_file_name_ ; 
  int utility_mathod_ ; 
};

class Node
{
 public:
  Node();
  ~Node();
  Eigen::Vector4d state_;
  Node * parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;
};

class TreeBase
{
 protected:
  typedef Eigen::Vector4d StateVec;
  Params params_;
  int counter_;
  double bestGain_;
  Node * bestNode_;
  Node * rootNode_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;
  StateVec root_;
  StateVec exact_root_;
  std::vector<std::string> agentNames_;
 public:
  TreeBase();
  TreeBase(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~TreeBase();
  virtual void setStateFromPoseStampedMsg(const geometry_msgs::PoseStamped& pose) = 0;
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose) = 0;
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose) = 0;
  virtual bool iterate(int iterations) = 0;  
  virtual void initialize() = 0;
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame) = 0;
  virtual void clear() = 0;
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame) = 0;
  virtual void memorizeBestBranch() = 0;
 

  //virtual void initializeDeep() =0 ;
  //virtual  bool iterateDeep(int iterations, double informationGain, int numOfSamples)=0;
  //virtual geometry_msgs::Pose getBestEdgeDeep(std::string targetFrame)=0;
  void setParams(Params params);
  int getCounter();
  bool gainFound();
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  double getGain() ; 
};
}
#endif
