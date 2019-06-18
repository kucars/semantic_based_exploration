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

namespace rrtNBV
{
class RRTPlanner
{
  public:
    RRTPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
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
    void MaxGainPose(geometry_msgs::Pose p, int id);

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
    float confidenceThreshold;
    float numOfVisitsThreshold;
    double globalObjectGain;
    double globalVolumetricGain;

};

}  // namespace rrtNBV
#endif  // RRT_PLANNER_H
