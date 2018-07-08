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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
//#include <voxel_grid_occlusion_estimation.h>
//#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>
#include <usar_exploration/occlusion_culling.h>

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

using namespace std;

geometry_msgs::PoseStamped current_pose ;
geometry_msgs::PoseStamped loc ;
bool current_pose_NOT_recieved_flag = true ;

void CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    std::cout << "current pose callback " << std::endl ;
    current_pose = *msg ;
    current_pose_NOT_recieved_flag= false ;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "current_view_extraction");
    ros::NodeHandle n;

    ros::Publisher original_cloud_pub          = n.advertise<sensor_msgs::PointCloud2>("original_pointcloud", 100);
    ros::Publisher predicted_cloud_pub         = n.advertise<sensor_msgs::PointCloud2>("old_pointcloud", 40);
    ros::Publisher accumulated_cloud_Pub       = n.advertise<sensor_msgs::PointCloud2>("accumulated_pointcloud", 100);
    ros::Publisher converted_cloud_pub         = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 40);
    ros::Subscriber current_pose_sub         = n.subscribe("current_pose", 1, CurrentPoseCallback);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predictedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accomulatedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloudOutPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string path = ros::package::getPath("usar_exploration");
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (path+"/resources/pcd/house_colored4.pcd", *originalCloud); // for visualization
    OcclusionCulling occlusionCulling(n,"house_colored4.pcd");

    pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloudOut;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloudAdd;
    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 cloud3;
    sensor_msgs::PointCloud2 cloud4;

    pcl::toROSMsg(*originalCloud, cloud1); //cloud of original
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "world";
    original_cloud_pub.publish(cloud1);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        listener.waitForTransform("/base_point_cloud", "/world", ros::Time(0), ros::Duration(0.1));
        try{
            listener.lookupTransform("/base_point_cloud", "/world",ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        if(!current_pose_NOT_recieved_flag)
        {
            std::cout<< "view point position " << current_pose.pose.position.x << "  "  <<current_pose.pose.position.y << "  "  <<  current_pose.pose.position.z << std::endl ;
            std::cout<< "view point orientation " << current_pose.pose.orientation.x << "  "  <<current_pose.pose.orientation.y << "  "  <<  current_pose.pose.orientation.z << " " << current_pose.pose.orientation.w << std::endl ;
            //std::cout << "recived" << std::endl ;
            tempCloud = occlusionCulling.extractColoredVisibleSurface(current_pose.pose);  // point cloud in world frame
            predictedCloudPtr->points = tempCloud.points;
            pcl_ros::transformPointCloud(tempCloud,tempCloudOut,transform) ; // convert the point cloud in camera frame

            tempCloudOutPtr->points = tempCloudOut.points ;
            pcl::toROSMsg(*tempCloudOutPtr, cloud4);

            tempCloudAdd += tempCloud ;
            accomulatedCloudPtr->points = tempCloudAdd.points ;

            pcl::toROSMsg(*accomulatedCloudPtr, cloud3); //
            pcl::toROSMsg(*predictedCloudPtr, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling
            // Publishing Point Cloud msgs
            cloud2.header.stamp = ros::Time::now();
            cloud2.header.frame_id = "world";
            predicted_cloud_pub.publish(cloud2);
            cloud3.header.stamp = ros::Time::now();
            cloud3.header.frame_id = "world";
            accumulated_cloud_Pub.publish(cloud3);
            cloud4.header.stamp = ros::Time::now();
            cloud4.header.frame_id = "base_point_cloud";
            converted_cloud_pub.publish(cloud4);
            current_pose_NOT_recieved_flag = true;
        }
        else
            std::cout << "Not recived Yet" << std::endl << std::flush ;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


