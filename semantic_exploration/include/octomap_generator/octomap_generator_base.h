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
#ifndef OCTOMAP_GENERATOR_BASE_H
#define OCTOMAP_GENERATOR_BASE_H

#include <kindr/minimal/quat-transformation.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/PCLPointCloud2.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

typedef kindr::minimal::QuatTransformation Transformation;

/**
 * Interface for octomap_generator for polymorphism
 * \author Xuan Zhang
 * \data Mai-July 2018
 */

enum VoxelStatus
{
    kUnknown = 0,
    kOccupied,
    kFree
};

inline octomap::point3d pointEigenToOctomap(const Eigen::Vector3d& point)
{
    return octomap::point3d(point.x(), point.y(), point.z());
}

inline Eigen::Vector3d pointOctomapToEigen(const octomap::point3d& point)
{
    return Eigen::Vector3d(point.x(), point.y(), point.z());
}

class OctomapGeneratorBase
{
  public:
    /// Desturctor
    virtual ~OctomapGeneratorBase()
    {
    }

    virtual void readFile(const char* filename) =0 ; 
    virtual void writeFile(const char* filename) =0 ; 

    /// Set max range for point cloud insertion
    virtual void setMaxRange(float max_range) = 0;

    /// Set max range to perform raycasting on inserted points
    virtual void setRayCastRange(float raycast_range) = 0;

    /// Set clamping_thres_min, parameter for octomap
    virtual void setClampingThresMin(float clamping_thres_min) = 0;

    /// Set clamping_thres_max, parameter for octomap
    virtual void setClampingThresMax(float clamping_thres_max) = 0;

    /// Set resolution, parameter for octomap
    virtual void setResolution(float resolution) = 0;

    /// Set occupancy_thres, parameter for octomap
    virtual void setOccupancyThres(float occupancy_thres) = 0;

    /// Set prob_hit, parameter for octomap
    virtual void setProbHit(float prob_hit) = 0;

    /// Set prob_miss, parameter for octomap
    virtual void setProbMiss(float prob_miss) = 0;

    /**
     * \brief Insert point cloud into octree
     * \param cloud converted ros cloud to be inserted
     * \param sensorToWorld transform from sensor frame to world frame
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud,
                                  const Eigen::Matrix4f& sensorToWorld) = 0;

    virtual void insertPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                                  const std::string& to_frame) = 0;

    /// Set whether use semantic color for serialization
    virtual void setUseSemanticColor(bool use) = 0;

    /// Get whether use semantic color for serialization
    virtual bool isUseSemanticColor() = 0;

    /// Get octree
    virtual octomap::AbstractOcTree* getOctree() = 0;

    /// Save octomap to a file. NOTE: Not tested
    virtual bool save(const char* filename) = 0;

    virtual double getResolution() const = 0;

    virtual VoxelStatus getBoundingBoxStatus(const Eigen::Vector3d& center,
                                             const Eigen::Vector3d& bounding_box_size,
                                             bool stop_at_unknown_voxel) = 0;

    virtual VoxelStatus getLineStatus(const Eigen::Vector3d& start, const Eigen::Vector3d& end) = 0;

    virtual VoxelStatus getLineStatusBoundingBox(const Eigen::Vector3d& start,
                                                 const Eigen::Vector3d& end,
                                                 const Eigen::Vector3d& bounding_box_size) = 0;

    virtual VoxelStatus getVisibility(const Eigen::Vector3d& view_point,
                                      const Eigen::Vector3d& voxel_to_test,
                                      bool stop_at_unknown_cell) = 0;

    virtual VoxelStatus getCellProbabilityPoint(const Eigen::Vector3d& point,
                                                double* probability) = 0;
    
    virtual octomap::ColorOcTreeNode::Color getVoxelColor(const Eigen::Vector3d& point) =0;

    virtual int getCellConfidence(const Eigen::Vector3d& point) = 0;

    virtual Eigen::Vector3d getMapSize() = 0;

    virtual double getVisibilityLikelihood(const Eigen::Vector3d& view_point,
                                           const Eigen::Vector3d& voxel_to_test) = 0;

    virtual bool getRearSideVoxel(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test) = 0;

    virtual int getCellIneterestCellType(double x, double y, double z) = 0;

    //virtual void octomapReadData(std::ostream& s) =0 ; 
    
    virtual double getCellIneterestGain(const Eigen::Vector3d& point) = 0;

    virtual uint getCellNumOfVisits(const Eigen::Vector3d& point) =0 ; 

    virtual bool lookupTransformation(const std::string& from_frame, const std::string& to_frame,
                                      const ros::Time& timestamp, Transformation* transform) = 0;

    virtual void setSematicColoredLabels(std::map<std::string,octomap::ColorOcTreeNode::Color> scl) = 0;

    virtual void setObjectsOfInterest(std::vector<std::string> ooi) =0;
    virtual void setConfidenceThreshold(float ooi) =0 ; 
    virtual void setNumOfVisitsThreshold(int ooi) =0 ; 
};

#endif  //OCTOMAP_GENERATOR_BASE
