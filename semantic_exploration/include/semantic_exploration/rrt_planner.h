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
#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdtree/kdtree.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_generator/octomap_generator.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <semantic_exploration/GetPath.h>
#include <semantic_exploration/rrt_core.h>
#include <semantic_exploration/rrt_tree.h>
#include <semantics_octree/semantics_octree.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <sstream>
#include "semantic_exploration/drone_commander.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "utilities/time_profiler.h"


namespace rrtNBV
{

class RRTPlanner
{
  public:
    RRTPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    RRTPlanner(){};

    ~RRTPlanner();
    void computeCameraFOV();
    void posStampedCallback(const geometry_msgs::PoseStamped& pose);
    void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void odomCallback(const nav_msgs::Odometry& pose);
    bool plannerCallback(semantic_exploration::GetPath::Request& req,
                         semantic_exploration::GetPath::Response& res);
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    bool isReady();
    void setHandlers(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    bool setParams();
    void setupLog();
    void getSemanticLabelledColors();
    rrtNBV::Params getParams();
    void drawPath(geometry_msgs::Pose p, int id);
    //std::istream input_file ; 
    //std::ostream output_file ; 

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber posClient_;
    ros::Subscriber posStampedClient_;
    ros::Subscriber odomClient_;
    ros::ServiceServer plannerService_;
    ros::ServiceServer toggleSemanticService;  ///<ROS service to toggle semantic color display
    bool toggleUseSemanticColor(
        std_srvs::Empty::Request& request,
        std_srvs::Empty::Response&
            response);  ///<Function to toggle whether write semantic color or rgb color as when serializing octree
    ros::Publisher fullmapPub_;  ///<ROS publisher for octomap message
    ros::Subscriber pointcloud_sub_;

    OctomapGeneratorBase* octomap_generator_;
    rrtNBV::RrtTree* rrtTree;
    visualization_msgs::Marker area_marker_;
    visualization_msgs::Marker explorationAreaInit();
    bool ready_;
    rrtNBV::Params params_;
    std::string logFilePathName_;
    std::ofstream file_path_;
    std::ofstream objects_file_path_;

    visualization_msgs::Marker line_strip;
    ros::Publisher marker_pub_;
    ros::Publisher sample_viewpoint_array_pub_;

    //visualization_msgs::MarkerArray sample_points_array  ; 

    //usar_exploration::extractView srv;
    // Global variables
    double traveled_distance = 0;
    double information_gain = 0;
    bool firstPoseCalled = true;
    geometry_msgs::Pose prePose;
    int iteration_num = 0;
    geometry_msgs::PoseArray viewpoints2;
    double accumulativeGain = 0;
    octomap_msgs::Octomap map_msg_;  ///<ROS octomap message
    std::map<std::string,octomap::ColorOcTreeNode::Color> semanticColoredLabels;
    std::vector<std::string> objectsOfInterest;
    std::vector<std::string> datasetObjects;
    float confidenceThreshold;
    float numOfVisitsThreshold;
    double globalObjectGain;
    double globalVolumetricGain;
    std::vector<geometry_msgs::Pose> selected_poses;
    bool debug_save_state_param ; 
    bool debug_load_state_param ; 
    std::ofstream output_file; 
    std::ifstream input_file; 
    std::string output_file_path_;
    std::string  input_file_path_;
    std::vector<int> Objectarray ; 
    std::vector<std::array<int, 3> > colorArray ; 
    int logging_period ; 
 
/*private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    //ar & octomap_generator_;
    //ar & rrtTree;
    ar & ready_;
    //ar & params_;
    //ar & logFilePathName_;
    //ar & file_path_;
    //ar & line_strip;
   
    // Global variables
    ar & traveled_distance ;
    ar & information_gain ;
    ar & firstPoseCalled ;
    ar & prePose;
    ar & iteration_num ;
    //ar & viewpoints2;
    ar & accumulativeGain;
    //ar & map_msg_;  ///<ROS octomap message
    //ar & semanticColoredLabels;
    ar & objectsOfInterest;
    ar & confidenceThreshold;
    ar & numOfVisitsThreshold;
    ar & globalObjectGain;
    ar & globalVolumetricGain;
    ar & selected_poses;

  }*/
};

}  // namespace rrtNBV


/*namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, geometry_msgs::Pose & g, const unsigned int version)
{
    ar & g.position.x & g.position.y & g.position.z;
    ar & g.orientation.x & g.orientation.y & g.orientation.z & g.orientation.w;
}

} // namespace serialization
}*/  // namespace boost
#endif  // RRT_PLANNER_H
