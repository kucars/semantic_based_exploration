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

#ifndef INFORMATIONGAINAVERAGEENTROPY_H_
#define INFORMATIONGAINAVERAGEENTROPY_H_

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
#include <rrt_explorer/information_gain_base.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace rrtNBV {

class InformationGainAverageEntropy : public InformationGainBase
{
public:
  InformationGainAverageEntropy();
  InformationGainAverageEntropy(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~InformationGainAverageEntropy();

  virtual void initialize();
  virtual bool iterate(int iterations);
  double gain(StateVec state);


};
}

#endif
