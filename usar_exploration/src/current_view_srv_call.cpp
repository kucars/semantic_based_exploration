#include "ros/ros.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <cmath>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>
#include <culling/occlusion_culling.h>

// octomap
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <string>

#include "usar_exploration/extractView.h"

typedef pcl::PointXYZRGB pointType;
bool receivedNewPose = false ;

typedef pcl::PointCloud<pointType> PointCloud;
geometry_msgs::PoseStamped current_pose ;



class ExtractViewPoints {
public:
    ExtractViewPoints(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) ;
    bool extractCurrentViewFunction(usar_exploration::extractView::Request  &req, usar_exploration::extractView::Response &res) ;
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    PointCloud tempCloud,tempCloudOut,tempCloudAdd,frustumCloud;
    sensor_msgs::PointCloud2 originalCloudMsg,currentViewCloudMsg,accomulatedCloudMsg,currentTransferedViewCloudMsg,currentCloudFrustumMsg,occupancyGridCloud;
    visualization_msgs::MarkerArray sensorFOV;
    visualization_msgs::Marker rayLines;
    std::string pcdFileName,robotFrame,worldFrame;
    std::string path ;
    std::string pcdFilePath;

    // Publishers and subscribers
    ros::Publisher originalPointcloudPub ;
    ros::Publisher currentPointcloudTransferedPub;
    ros::Publisher currentPointcloudFrustumPub ;
    ros::Publisher currentPointcloudPub  ;
    ros::Publisher accumulatedPointcloudPub ;
    ros::Publisher sensorFOVPub   ;
    ros::Publisher rayPub    ;
    ros::Publisher occupancyGridPub  ;
    ros::Subscriber current_pose_sub ;



};
ExtractViewPoints::ExtractViewPoints(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): nh_(nh),
    nh_private_(nh_private){
    ROS_INFO("Call Base Class constructor");

    // Publishers and subscribers
    originalPointcloudPub            = nh_.advertise<sensor_msgs::PointCloud2>("original_pointcloud", 10);
    currentPointcloudTransferedPub   = nh_.advertise<sensor_msgs::PointCloud2>("current_viewpoint_pointcloud_transfered", 10);
    currentPointcloudFrustumPub      = nh_.advertise<sensor_msgs::PointCloud2>("current_viewpoint_frustum_pointcloud", 10);
    currentPointcloudPub             = nh_.advertise<sensor_msgs::PointCloud2>("current_viewpoint_pointcloud", 10);
    accumulatedPointcloudPub         = nh_.advertise<sensor_msgs::PointCloud2>("accumulated_pointclouds", 10);
    sensorFOVPub                     = nh_.advertise<visualization_msgs::MarkerArray>("sensor_fov", 100);
    rayPub                           = nh_.advertise<visualization_msgs::Marker>("ray_casts", 10);
    occupancyGridPub                 = nh_.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 10);
    //current_pose_sub                = nh.subscribe("current_pose", 10, CurrentPoseCallback); // I will recive ir from the request

    // PointCloud and sensor msgs defenitions
    PointCloud::Ptr originalCloud(new PointCloud);

    nh_.param<std::string>("pcd_input_file", pcdFileName, std::string("env2.pcd"));
    nh_.param<std::string>("robot_frame", robotFrame, std::string("/base_point_cloud"));
    nh_.param<std::string>("tf_frame", worldFrame, std::string("/world"));

    ROS_INFO("PCD File %s, Robot Frame:%s, tf_frame:%s",pcdFileName.c_str(), robotFrame.c_str(), worldFrame.c_str());

    // Load the original map
    path = ros::package::getPath("usar_exploration");
    pcdFilePath = path + "/resources/pcd/" + pcdFileName;


    ROS_INFO("Loading File%s",pcdFilePath.c_str());
    pcl::io::loadPCDFile<pointType> (pcdFilePath, *originalCloud);
    ROS_INFO("Done, size:%d",originalCloud->points.size());

    //Publish the original map once
    pcl::toROSMsg(*originalCloud, originalCloudMsg); //cloud of original
    originalCloudMsg.header.stamp = ros::Time::now();
    originalCloudMsg.header.frame_id = "world";
    originalPointcloudPub.publish(originalCloudMsg);

}

bool ExtractViewPoints::extractCurrentViewFunction(usar_exploration::extractView::Request  &req, usar_exploration::extractView::Response &res)
{

    PointCloud::Ptr currentViewConvertedCloudOutPtr(new PointCloud);
    PointCloud::Ptr currentViewCloudOutPtr(new PointCloud);
    PointCloud::Ptr accomulatedCloudPtr(new PointCloud);
    PointCloud::Ptr frustumCloudPtr(new PointCloud);
    sensor_msgs::PointCloud2 currentViewCloudMsg,accomulatedCloudMsg,currentTransferedViewCloudMsg,currentCloudFrustumMsg,occupancyGridCloud;

    OcclusionCulling<pointType> occlusionCulling(nh_,pcdFilePath);
    // create listener for the sensor position
    tf::TransformListener listener;
    tf::StampedTransform transform;

    listener.waitForTransform(robotFrame, worldFrame, ros::Time(0), ros::Duration(0.1));
    try{
        listener.lookupTransform(robotFrame, worldFrame,ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    current_pose.pose = req.currentPose ;
    // change the pose to request msg not the resutls
    ROS_INFO ("view point position %f" , current_pose.pose.position.x ) ;

    ros::Time tic = ros::Time::now();    // change the pose to request msg not the resutls

    tempCloud = occlusionCulling.extractVisibleSurface(current_pose.pose);  // point cloud in world frame
    ros::Time toc = ros::Time::now();
    ROS_INFO("Occlusion Culling took:%f", toc.toSec() - tic.toSec());

    frustumCloud = occlusionCulling.getFrustumCloud();
    // rayLines = occlusionCulling.getRays();
    // occupancyGridCloud = occlusionCulling.getOccupancyGridCloud();
    //  sensorFOV = occlusionCulling.getFOV();

    receivedNewPose = false;
    //sensorFOVPub.publish(sensorFOV);

    // rayLines.header.frame_id = worldFrame;
    // rayLines.header.stamp = ros::Time::now();
    // rayPub.publish(rayLines);

    //occupancyGridPub.publish(occupancyGridCloud);

    currentViewCloudOutPtr->points = tempCloud.points;
    pcl::toROSMsg(*currentViewCloudOutPtr, currentViewCloudMsg);

    frustumCloudPtr->points = frustumCloud.points;
    pcl::toROSMsg(*frustumCloudPtr, currentCloudFrustumMsg);

    pcl_ros::transformPointCloud(tempCloud, tempCloudOut, transform); // convert the point cloud in camera frame
    currentViewConvertedCloudOutPtr->points = tempCloudOut.points;
    pcl::toROSMsg(*currentViewConvertedCloudOutPtr, currentTransferedViewCloudMsg);

    res.currentViewPointcloud = currentTransferedViewCloudMsg ;
    //res.currentViewPointcloud.header.frame_id = robotFrame;


    tempCloudAdd += tempCloud;
    accomulatedCloudPtr->points = tempCloudAdd.points;
    pcl::toROSMsg(*accomulatedCloudPtr, accomulatedCloudMsg);
    // Publishing Point Cloud msgs

    currentCloudFrustumMsg.header.stamp = ros::Time::now();
    currentCloudFrustumMsg.header.frame_id = worldFrame;
    // currentPointcloudFrustumPub.publish(currentCloudFrustumMsg);

    currentViewCloudMsg.header.stamp = ros::Time::now();
    currentViewCloudMsg.header.frame_id = worldFrame;
    //currentPointcloudPub.publish(currentViewCloudMsg);

    accomulatedCloudMsg.header.stamp = ros::Time::now();
    accomulatedCloudMsg.header.frame_id = worldFrame;
    // accumulatedPointcloudPub.publish(accomulatedCloudMsg);

    currentTransferedViewCloudMsg.header.stamp = ros::Time::now();
    currentTransferedViewCloudMsg.header.frame_id = robotFrame;
res.currentViewPointcloud = currentTransferedViewCloudMsg ;


    // send this one as the service results

    return true;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_view_server");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;


    ExtractViewPoints a(nh,nh_private);
    ros::ServiceServer service = nh.advertiseService("current_view", &ExtractViewPoints::extractCurrentViewFunction, &a);
    ROS_INFO("Ready to extract the new view.");
    ros::spin();

    return 0;
}


