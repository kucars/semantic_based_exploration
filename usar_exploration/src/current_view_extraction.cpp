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
//#include <voxel_grid_occlusion_estimation.h>
//#include "fcl_utility.h"
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

using namespace std;

geometry_msgs::PoseStamped current_pose ;
bool current_pose_NOT_recieved_flag = true ;
typedef pcl::PointXYZRGB pointType;
typedef pcl::PointCloud<pointType> PointCloud;

void CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    ROS_INFO("current pose callback ") ;
    current_pose = *msg ;
    current_pose_NOT_recieved_flag= false ;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "current_view_extraction");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Publishers and subscribers
    ros::Publisher original_cloud_pub                 = nh.advertise<sensor_msgs::PointCloud2>("original_pointcloud", 1);
    ros::Publisher current_view_converted_cloud_pub   = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1); // subscribed from volumetric mapping pkg, converted means the point cloud are on the sensor frame
    ros::Publisher current_view_cloud_pub             = nh.advertise<sensor_msgs::PointCloud2>("current_view_pointcloud", 40); // point cloud in the word frame used for visualization
    ros::Publisher accumulated_cloud_Pub              = nh.advertise<sensor_msgs::PointCloud2>("accumulated_pointcloud", 100); // in world frame
    ros::Subscriber current_pose_sub                  = nh.subscribe("current_pose", 10, CurrentPoseCallback);

    // PointCloud and sensor msgs defenitions
    PointCloud::Ptr originalCloud(new PointCloud);
    PointCloud::Ptr currentViewConvertedCloudOutPtr(new PointCloud);
    PointCloud::Ptr currentViewCloudOutPtr(new PointCloud);
    PointCloud::Ptr accomulatedCloudPtr(new PointCloud);
    
    PointCloud tempCloud,tempCloudOut,tempCloudAdd;
    sensor_msgs::PointCloud2 cloud1,cloud2,cloud3,cloud4;

    double sensor_roll, sensor_pitch, sensor_yaw, sensor_x, sensor_y, sensor_z;
    std::string pcdFileName;
    // Load Params
    nh.param<std::string>("pcd_input_file", pcdFileName, std::string("env2.pcd"));
    // Load the original map
    std::string path = ros::package::getPath("usar_exploration");

    std::string pcdFilePath = path + "/resources/pcd/" + pcdFileName;

    
    ROS_INFO("Loading File%s",pcdFilePath.c_str());
    pcl::io::loadPCDFile<pointType> (pcdFilePath, *originalCloud); // for visualization
    ROS_INFO("Done, size:%d",originalCloud->points.size()); 

    //Publish the original map once
    pcl::toROSMsg(*originalCloud, cloud1); //cloud of original
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "world";

    ROS_INFO("SIZE %d" , cloud1.width);
    ROS_INFO("SIZE %d" , cloud1.height);
    ROS_INFO("SIZE2 %d" , originalCloud->points.size());

    original_cloud_pub.publish(cloud1);

    OcclusionCulling<pointType> occlusionCulling(nh, pcdFilePath);

    // create listener for the sensor position
    tf::TransformListener listener;
    tf::StampedTransform transform;

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
            ROS_INFO ("view point position %f" , current_pose.pose.position.x ) ;

            tempCloud = occlusionCulling.extractVisibleSurface(current_pose.pose);  // point cloud in world frame

            currentViewCloudOutPtr->points = tempCloud.points;
            pcl::toROSMsg(*currentViewCloudOutPtr, cloud2);

            pcl_ros::transformPointCloud(tempCloud,tempCloudOut,transform) ; // convert the point cloud in camera frame
            currentViewConvertedCloudOutPtr->points = tempCloudOut.points ;
            pcl::toROSMsg(*currentViewConvertedCloudOutPtr, cloud4);

            tempCloudAdd += tempCloud ;
            accomulatedCloudPtr->points = tempCloudAdd.points ;
            pcl::toROSMsg(*accomulatedCloudPtr, cloud3);
            // Publishing Point Cloud msgs

            cloud2.header.stamp = ros::Time::now();
            cloud2.header.frame_id = "world";
            current_view_cloud_pub.publish(cloud2);

            cloud3.header.stamp = ros::Time::now();
            cloud3.header.frame_id = "world";
            accumulated_cloud_Pub.publish(cloud3);

            cloud4.header.stamp = ros::Time::now();
            cloud4.header.frame_id = "base_point_cloud";
            current_view_converted_cloud_pub.publish(cloud4);

            current_pose_NOT_recieved_flag = true;
        }
        else
            ROS_INFO("Not Recived Yet");

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



