#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H
#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <semantic_exploration/rrt_core.h>
#include <semantic_exploration/rrt_tree.h>
#include <semantic_exploration/mesh_structure.h>
#include <semantic_exploration/GetPath.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>

namespace rrtNBV {

class RRTPlanner{
public:
    RRTPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~RRTPlanner();
    void posStampedCallback(const geometry_msgs::PoseStamped& pose);
    void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void odomCallback(const nav_msgs::Odometry& pose);
    bool plannerCallback(semantic_exploration::GetPath::Request& req, semantic_exploration::GetPath::Response& res);
    void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
    void insertPointcloudWithTfCamUp(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
    void insertPointcloudWithTfCamDown(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
    bool isReady();
    void setHandlers(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    bool setParams();
    rrtNBV::Params getParams();
    void MaxGainPose(geometry_msgs::Pose p , int id);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber posClient_;
    ros::Subscriber posStampedClient_;
    ros::Subscriber odomClient_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber pointcloud_sub_cam_up_;
    ros::Subscriber pointcloud_sub_cam_down_;
    ros::ServiceServer plannerService_;
    mesh::StlMesh * mesh_;
    volumetric_mapping::OctomapManager * manager_;
    rrtNBV::RrtTree *rrtTree;
    visualization_msgs::Marker area_marker_;
    visualization_msgs::Marker explorationAreaInit();
    bool ready_;
    rrtNBV::Params params_;
    std::string logFilePathName_;
    std::ofstream file_path_;
    visualization_msgs::Marker line_strip ;
    ros::Publisher marker_pub_;
    //usar_exploration::extractView srv;
    // Global variables
    double traveled_distance = 0 ;
    double information_gain =0;
    bool FirstPoseCalled=true;
    geometry_msgs::Pose prePose;
    int iteration_num = 0 ;
    geometry_msgs::PoseArray viewpoints2;
    double accumulativeGain = 0 ;

    double globalObjectGain ;
    double globalVolumetricGain ;
};

}
#endif // RRT_PLANNER_H
