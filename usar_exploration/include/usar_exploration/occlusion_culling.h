/****************************************************************************************
 *   Copyright (C) 2015 - 2017 by                                                   	*
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                                    	*
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                            	*
 *                                                                         	    	*
 *      The package was modified by Reem Ashour as follows:                         	*
 *      1-  All functions were modified to deal with XYZRGB points instead of XYZ   	*
 *      2-  Parameters were changed to work with environment exploration
 *      3-  Coverage caluation function's were removed                              	*
 *      Reem Ashour <reem.ashour@ku.ac.ae>                                	        *

 *
 *   This program is free software; you can redistribute it and/or modify  		*
 *   it under the terms of the GNU General Public License as published by  		*
 *   the Free Software Foundation; either version 2 of the License, or     		*
 *   (at your option) any later version.                                   		*
 *                                                                        		*
 *   This program is distributed in the hope that it will be useful,      		*
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        		*
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         		*
 *   GNU General Public License for more details.                          		*
 *                                                                         		*
 *   You should have received a copy of the GNU General Public License     		*
 *   along with this program; if not, write to the                         		*
 *   Free Software Foundation, Inc.,                                       		*
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          		*
 ****************************************************************************************/
#ifndef OCCLUSION_H_
#define OCCLUSION_H_

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
#include <visualization_msgs/MarkerArray.h>

//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <usar_exploration/frustum_culling.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <usar_exploration/voxel_grid_occlusion_estimation.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>

class OcclusionCulling
{
public:
    //attributes
    ros::NodeHandle  nh;
    std::string model;
    //     ros::Publisher original_pub;
    //     ros::Publisher visible_pub;
    ros::Publisher fov_pub;
    ros::Publisher frustum_pub;
    ros::Publisher original_cloud_pub ;
    ros::Publisher occluded_free_pub;
    ros::Publisher out_ray_pub;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColored;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_colored_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occlusionFreeCloud;//I can add it to accumulate cloud if I want to extract visible surface from multiple locations
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr FrustumCloud;//frustum cull
    pcl::PointCloud<pcl::PointXYZRGB> FreeCloud;
    pcl::PointCloud<pcl::PointXYZRGB> FreeCloudColored;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rayCloud ;
    float voxelRes, OriginalVoxelsSize;
    double id;
    pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
    Eigen::Vector3i  max_b1, min_b1;
    visualization_msgs::Marker linesList1,linesList2,linesList3,linesList4;
    visualization_msgs::MarkerArray marker_array;
    pcl::FrustumCullingTT fc;
    double maxAccuracyError, minAccuracyError;
    bool AccuracyMaxSet;

    //methods
    OcclusionCulling(ros::NodeHandle &n,  std::string modelName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    OcclusionCulling(ros::NodeHandle &n, std::string modelName);
    OcclusionCulling(std::string modelName);
    OcclusionCulling();
    ~OcclusionCulling();
    // pcl::PointCloud<pcl::PointXYZ> extractVisibleSurface(geometry_msgs::Pose location);
    pcl::PointCloud<pcl::PointXYZRGB> extractColoredVisibleSurface(geometry_msgs::Pose location);

    //    float calcCoveragePercent(geometry_msgs::Pose location);
    float calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
    double calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud);
    double calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose);
    void transformPointMatVec(tf::Vector3 translation, tf::Matrix3x3 rotation, geometry_msgs::Point32 in, geometry_msgs::Point32& out);
    pcl::PointCloud<pcl::PointXYZ> pointCloudViewportTransform(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose);
    void SSMaxMinAccuracy(std::vector<geometry_msgs::PoseArray> sensorsPoses);
    void visualizeFOV(geometry_msgs::Pose location);
    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[]);
    void visualizeOriginalPointcloud() ;


};

#endif 
