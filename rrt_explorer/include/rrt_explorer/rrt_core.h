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

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <rrt_explorer/rrt_tree.h>
#include <rrt_explorer/mesh_structure.h>
#include <rrt_explorer/rrt_srv.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711


namespace rrtNBV {

class RrtTree : public TreeBase
{
 public:
  RrtTree();
  RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseStampedMsg(const geometry_msgs::PoseStamped& pose);
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
  virtual void initialize();
  virtual bool iterate(int iterations);
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);
  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();

  void publishNode(Node * node);
  double gain(StateVec state); // Volumetric Infromation Ref [2] RRT 
  double gain_rsvs(StateVec state, bool &objectGainFound); // Rear side voxel - Ref[1] 
  double gain_rsv(StateVec state, bool &objectGainFound); // Semantic rear side voxel - Proposed 
  double gain_rse(StateVec state, bool &objectGainFound) ; // Rear side entropy - Ref [1] 
  double gain_rses(StateVec state, bool &objectGainFound); // Semantic rear side entropy - Proposed 
  double gain_occlusion_aware(StateVec state, bool &objectGainFound); // Occlusion Aware Ref-[1]
  double gain_unobserved_voxel(StateVec state, bool & objectGainFound) ;// Unobserved Voxel Ref[1] 
  double gain_pure_entropy(StateVec state) ; 
  double gain_avg_entropy(StateVec state) ; 

  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end, std::string targetFrame);

  // Modified functions
  void initializeDeep();
  bool iterateDeep(int iterations, double informationGain, int numOfSamples);
  geometry_msgs::Pose getBestEdgeDeep(std::string targetFrame);
  double gainPureEntropy(StateVec state);
  double gainDinsity(StateVec state, int &dinsity);
  virtual double getBestGain() ;
  virtual Eigen::Vector4d getRootNode();
  

 protected:  
  kdtree * kdTree_;
  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::fstream fileCoverageAndGain_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;
};
}

#endif
