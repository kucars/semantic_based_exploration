/****************************************************************************************
 *   Copyright (C) 2015 - 2017 by                                                       *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                                        *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                                *
 *                                                                                      *
 *      The package was modified by Reem Ashour as follows:                         	*
 *      1-  All functions were modified to deal with XYZRGB points instead of XYZ   	*
 *      2-  Parameters were changed to work with environment exploration
 *      3-  Coverage caluation function's were removed                              	*
 *      Reem Ashour <reem.ashour@ku.ac.ae>                                	        *
 *
 *   This program is free software; you can redistribute it and/or modify               *
 *   it under the terms of the GNU General Public License as published by               *
 *   the Free Software Foundation; either version 2 of the License, or                  *
 *   (at your option) any later version.                                                *
 *                                                                                      *
 *   This program is distributed in the hope that it will be useful,                    *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of                     *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      *
 *   GNU General Public License for more details.                                       *
 *                                                                                      *
 *   You should have received a copy of the GNU General Public License                  *
 *   along with this program; if not, write to the                                      *
 *   Free Software Foundation, Inc.,                                                    *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.                       *
 ****************************************************************************************/

#include "usar_exploration/occlusion_culling.h"

OcclusionCulling::OcclusionCulling(ros::NodeHandle &n, std::string modelName):
    nh(n),
    model(modelName),
    fc(true)
{
    ROS_INFO("Occlusion culling constructor ");
    rayCloud =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>); // cloud of original model
    //occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    //FrustumCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    std::string path = ros::package::getPath("usar_exploration");
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (path+"/resources/pcd/"+model, *cloud);
    std::cout << "DIR: " << path+"/resources/pcd/"+model << std::endl ; 
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;
    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.3);
    fc.setFarPlaneDistance (3.0);
}

OcclusionCulling::~OcclusionCulling()
{
}

pcl::PointCloud<pcl::PointXYZRGB> OcclusionCulling::extractColoredVisibleSurface(geometry_msgs::Pose location)
{
    ROS_INFO ("extractColoredVisibleSurface \n" ) ;
    std::cout << "location: " << location.position.x << " " <<location.position.y << " " <<location.position.z << std::endl << std::flush ;

    //    // 1 *****Frustum Culling*******
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occlusionFreeCloud_local(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;
    camera_pose.setZero();
    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    camera_pose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    ros::Time tic = ros::Time::now();
    fc.filter (*output);
    ros::Time toc = ros::Time::now();
    std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec() << std::endl << std::flush;
    ROS_INFO ("frustum size %d \n" , output->size() ) ;


    //2:****voxel grid occlusion estimation *****
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT voxelFilter;
    voxelFilter.setInputCloud (output);
    voxelFilter.setLeafSize (0.03279f, 0.03279f, 0.03279f);
    //voxelFilter.setLeafSize (1.0f, 1.0f, 1.0f);
    //voxelFilter.setLeafSize (0.5f, 0.5f, 0.5f);

    voxelFilter.initializeVoxelGrid();
    int state,ret;
    pcl::PointXYZRGB p1,p2;
    pcl::PointXYZRGB point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
    //std::vector<geometry_msgs::Point> lineSegments;
    //geometry_msgs::Point linePoint;

    // iterate over the entire frustum points
    for ( int i = 0; i < (int)output->points.size(); i ++ )
    {
        pcl::PointXYZRGB ptest = output->points[i];
        //std::cout << "ptest" << ptest.x << "   ptest"<< (int)ptest.r <<  "   ptest"<< (int)ptest.g <<std::endl;
        Eigen::Vector3i ijk = voxelFilter.getGridCoordinates( ptest.x, ptest.y, ptest.z);
        // process all free voxels
        int index = voxelFilter.getCentroidIndexAt (ijk);
        Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
        point = pcl::PointXYZRGB(0,244,0);
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];

        if(index!=-1 )
        {
            out_ray.clear();
            ret = voxelFilter.occlusionEstimation(state,out_ray, ijk);
            if(state == 1)
            {
                // WARNING : The following code produce a meomery leakage specialy when using a dense and continous point cloud extraction
                // Only used for debugging 
                //                for(uint j=0; j< out_ray.size(); j++)
                //                {
                //                    Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (out_ray[j]);
                //                    pcl::PointXYZRGB p = pcl::PointXYZRGB(255,255,0);
                //                    p.x = centroid[0];
                //                    p.y = centroid[1];
                //                    p.z = centroid[2];
                //                    rayCloud->points.push_back(p);
                //                   // std::cout<<"Ray X:"<<p.x<<" y:"<< p.y<<" z:"<< p.z<<"\n";
                //                }
            }
            else
            {
                // ROS_INFO("state %d \n" , state ) ;
                // estimate direction to target voxel
                Eigen::Vector4f direction = centroid - cloud->sensor_origin_;
                direction.normalize ();
                // estimate entry point into the voxel grid
                float tmin = voxelFilter.rayBoxIntersection (cloud->sensor_origin_, direction,p1,p2);
                if(tmin!=-1)
                {
                    occlusionFreeCloud_local->points.push_back(ptest);
                    //occlusionFreeCloud->points.push_back(ptest);
                }
            }
        }
    }
    FreeCloud.points = occlusionFreeCloud_local->points;
    return FreeCloud ;
}


