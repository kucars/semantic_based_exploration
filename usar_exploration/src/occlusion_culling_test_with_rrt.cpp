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
#include <culling/occlusion_culling.h>

// octomap
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <tf/transform_broadcaster.h>

//#include <voxel_grid_occlusion_estimation.h>
//#include "fcl_utility.h"


// octomap
#include <octomap_world/octomap_manager.h>
#include <string>
#include <octomap/Pointcloud.h>

using namespace std;
using namespace octomap;
//using namespace fcl;
typedef pcl::PointXYZRGB pointType;

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale);
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, std::vector<float> rpy, std::vector<float> xyz);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "testing");
    ros::NodeHandle n;

    //ros::Publisher originalPointcloudPub             = n.advertise<sensor_msgs::PointCloud2>("originalpointcloud", 100);
    //ros::Publisher predictedCloudPub            = n.advertise<sensor_msgs::PointCloud2>("pointcloud_1", 100);
    ros::Publisher currentPosePub               = n.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
    //ros::Publisher FrustumCloudPub              = n.advertise<sensor_msgs::PointCloud2>("frustum_cloud", 100);
    //ros::Publisher sensorPosePub                = n.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    //ros::Publisher generagedPathPub             = n.advertise<visualization_msgs::Marker>("generated_path", 10);
   // ros::Publisher currentViewPointcloud        = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);

    //pcl::PointCloud<pcl::PointXYZRGB> predictedCloud;

   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr predictedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr FrustumCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr FrustumCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   // sensor_msgs::PointCloud2 originalCloudMsg,currentViewCloudMsg,accomulatedCloudMsg;//,currentCloudFrustumMsg;

   // std::string pcdFileName;
   // n.param<std::string>("pcd_input_file", pcdFileName, std::string("env2.pcd")); //house_colored4 // semantic_area6

    // Load the original map
    std::string path = ros::package::getPath("usar_exploration");
  //  std::string pcdFilePath = path + "/resources/pcd/" + pcdFileName;
   // ROS_INFO("Loading File%s",pcdFilePath.c_str());

  //  pcl::io::loadPCDFile<pointType> (pcdFilePath, *originalCloud);
   // ROS_INFO("Done, size:%d",originalCloud->points.size());

    //Publish the original map once
   // pcl::toROSMsg(*originalCloud, originalCloudMsg); //cloud of original
  //  originalCloudMsg.header.stamp = ros::Time::now();
  //  originalCloudMsg.header.frame_id = "world";

    //originalPointcloudPub.publish(originalCloudMsg);
 //   OcclusionCulling<pointType> occlusionCulling(n, pcdFilePath);

    // read positiongs from txt file // only add the start position
    //std::string txtFileName;
    //n.param<std::string>("txt_input_file", txtFileName, std::string("occlusionTestIndoorScene2_high.txt")); //occlusionTestPoints
    std::string txtFilePath = path + "/resources/txt/occlusionTestIndoorScene2_high.txt";
    ROS_INFO("Loading File%s",txtFilePath.c_str());

    double locationx,locationy,locationz,yaw; // file variables
    geometry_msgs::PoseArray viewpoints;
    geometry_msgs::PoseStamped loc; // exploration positions
    double viewPointCount=0;
    double timeSum=0;

    ros::Rate loop_rate(10);
    tf::TransformBroadcaster br;
    tf::Transform transform;

    const char * filename1 = txtFilePath.c_str();
    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open the File";
        fclose(file1);
    }
    

    while (!feof(file1) && ros::ok() )
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        std::cout << "location in test: " << locationx << " " << locationy << " " << locationz <<  "  " << yaw << std::endl ;
        // Broadcast the TF of the camera location
        transform.setOrigin(tf::Vector3(locationx, locationy, locationz) );
        tf::Quaternion tf_q1 ;
        tf_q1 = tf::createQuaternionFromYaw(yaw);
        transform.setRotation(tf::Quaternion(tf_q1.getX(),  tf_q1.getY(), tf_q1.getZ(),  tf_q1.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        
        loc.pose.position.x = locationx;
        loc.pose.position.y = locationy;
        loc.pose.position.z = locationz;
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw);
        loc.pose.orientation.x =tf_q.getX();
        loc.pose.orientation.y = tf_q.getY();
        loc.pose.orientation.z = tf_q.getZ();
        loc.pose.orientation.w =tf_q.getW();
        loc.header.frame_id="world";
        currentPosePub.publish(loc); // Step 1 
        viewpoints.poses.push_back(loc.pose);



//        ////////////////////////////// Frustum culling - visiualization only //////////////////////////////////////////////////////
//        pcl::FrustumCullingTT fc(true);
//        fc.setInputCloud (originalCloud);
//        fc.setVerticalFOV (45);
//        fc.setHorizontalFOV (58);
//        fc.setNearPlaneDistance (0.3);
//        fc.setFarPlaneDistance (3.0);
//        Eigen::Matrix4f camera_pose;
//        Eigen::Matrix3d Rd;
//        Eigen::Matrix3f Rf;
//        camera_pose.setZero();
//        tf::Quaternion qt;
//        qt.setX(loc.pose.orientation.x);
//        qt.setY(loc.pose.orientation.y);
//        qt.setZ(loc.pose.orientation.z);
//        qt.setW(loc.pose.orientation.w);
//        tf::Matrix3x3 R_tf(qt);
//        tf::matrixTFToEigen(R_tf,Rd);
//        Rf = Rd.cast<float>();
//        camera_pose.block (0, 0, 3, 3) = Rf;
//        Eigen::Vector3f T;
//        T (0) = loc.pose.position.x; T (1) = loc.pose.position.y; T (2) = loc.pose.position.z;
//        camera_pose.block (0, 3, 3, 1) = T;
//        camera_pose (3, 3) = 1;
//        fc.setCameraPose (camera_pose);
//        fc.filter (*FrustumCloud);

//        ///////////////////////////////////////////////////////////////////////////
//        // 2 *****Occlusion Culling*******

//         std::vector<float> rpy;
//         rpy.push_back(0) ;
//         rpy.push_back(0.093) ;
//         rpy.push_back(0) ;
//         //(0,0.093,0);
//         //std::cout << "2.5 " << std::endl ;

//         std::vector<float> xyz;//(0,0.0,-0.055);
//         xyz.push_back(0) ;
//         xyz.push_back(0) ;
//         xyz.push_back(-0.055) ;

//         //xyz[2] = -0.055 ;

//        geometry_msgs::Pose pt;
//        pt.position.x = locationx; pt.position.y = locationy; pt.position.z = locationz;
//        tf::Quaternion tf_q2 ;
//        tf_q2 = tf::createQuaternionFromYaw(yaw);
//        pt.orientation.x = tf_q2.getX();
//        pt.orientation.y = tf_q2.getY();
//        pt.orientation.z = tf_q2.getZ();
//        pt.orientation.w = tf_q2.getW();
//        geometry_msgs::Pose c = uav2camTransformation( pt,  rpy,  xyz);


/*
        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;

        ros::Time tic = ros::Time::now();
        tempCloud = occlusionCulling.extractVisibleSurface(loc.pose);  // point cloud in world frame
        ROS_INFO("DONE");

        ros::Time toc = ros::Time::now();
        double elapsed =  toc.toSec() - tic.toSec();
        ROS_INFO("Occlusion Culling took in (s) :%f",elapsed );

        timeSum +=elapsed;
        viewPointCount++;

        predictedCloud += tempCloud;
        predictedCloudPtr->points = predictedCloud.points; // accumalated
        pcl::toROSMsg(tempCloud, currentViewCloudMsg); //cloud of original
        currentViewCloudMsg.header.stamp = ros::Time::now();
        currentViewCloudMsg.header.frame_id = "base_point_cloud";
        currentViewPointcloud.publish(currentViewCloudMsg);

        pcl::toROSMsg(*predictedCloudPtr, accomulatedCloudMsg); //cloud of the not occluded voxels (blue) using occlusion culling
        accomulatedCloudMsg.header.stamp = ros::Time::now();
        accomulatedCloudMsg.header.frame_id = "world";
        predictedCloudPub.publish(accomulatedCloudMsg);

        viewpoints.header.frame_id= "world";
        viewpoints.header.stamp = ros::Time::now();
        sensorPosePub.publish(viewpoints);

        loc.header.frame_id="world";
        loc.header.stamp = ros::Time::now();
        currentPosePub.publish(loc); // Step 1
*/
        sleep(5);

        ros::spinOnce();
        loop_rate.sleep();
        sleep(5);
    }
   // std::cout<<"On Average Occulision Culling takes (s) = "<<timeSum/viewPointCount<<"\n";

    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10000.0);
    std_msgs::ColorRGBA color;
    //    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    if(c_color == 1)
    {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(c_color == 2)
    {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else
    {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}



geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, std::vector<float> rpy, std::vector<float> xyz)
{
    Eigen::Matrix4d uav_pose, uav2cam, cam_pose;
    //UAV matrix pose
    Eigen::Matrix3d R; Eigen::Vector3d T1(pose.position.x,pose.position.y,pose.position.z);
    tf::Quaternion qt(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);
    uav_pose.setZero ();
    uav_pose.block (0, 0, 3, 3) = R;
    uav_pose.block (0, 3, 3, 1) = T1;
    uav_pose (3, 3) = 1;

    //transformation matrix
    qt = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);
    tf::Matrix3x3 R2(qt);Eigen::Vector3d T2(xyz[0],xyz[1],xyz[2]);
    tf::matrixTFToEigen(R2,R);
    uav2cam.setZero ();
    uav2cam.block (0, 0, 3, 3) = R;
    uav2cam.block (0, 3, 3, 1) = T2;
    uav2cam (3, 3) = 1;

    //preform the transformation
    cam_pose = uav_pose * uav2cam;

    Eigen::Matrix4d cam2cam;
    //the transofrmation is rotation by +90 around x axis of the camera
    cam2cam <<   1, 0, 0, 0,
            0, 0,-1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
    Eigen::Matrix4d cam_pose_new = cam_pose * cam2cam;
    geometry_msgs::Pose p;
    Eigen::Vector3d T3;Eigen::Matrix3d Rd; tf::Matrix3x3 R3;
    Rd = cam_pose_new.block (0, 0, 3, 3);
    tf::matrixEigenToTF(Rd,R3);
    T3 = cam_pose_new.block (0, 3, 3, 1);
    p.position.x=T3[0];p.position.y=T3[1];p.position.z=T3[2];
    R3.getRotation(qt);
    p.orientation.x = qt.getX(); p.orientation.y = qt.getY();p.orientation.z = qt.getZ();p.orientation.w = qt.getW();
    return p;
}

