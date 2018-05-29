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
    std::cout << "Occlusion culling constructor " << std::endl ;

    fov_pub              = n.advertise<visualization_msgs::MarkerArray>("fov", 10);
    original_cloud_pub   = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 100);
    frustum_pub          = n.advertise<sensor_msgs::PointCloud2>("frustum_cloud", 100);
    occluded_free_pub   = n.advertise<sensor_msgs::PointCloud2>("predicted_cloud", 100);
    rayCloud =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    //    filtered_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);

    std::string path = ros::package::getPath("usar_exploration");
    std::cout << "model name is  "  << model << std::endl ;

    pcl::io::loadPCDFile<pcl::PointXYZRGB> (path+"/resources/pcd/"+model, *cloud);


    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.3);
    fc.setFarPlaneDistance (3.0);

}
OcclusionCulling::OcclusionCulling(std::string modelName):
    model(modelName),
    fc(true)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);

    filtered_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);

    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (path+"/resources/pcd/"+model, *cloud);
    voxelRes = 0.1;
    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

}
OcclusionCulling::OcclusionCulling():
    model(NULL),
    fc(true)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    filtered_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>);
    std::string path = ros::package::getPath("usar_exploration");
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (path+"/resources/pcd/sphere.pcd", *cloud);
    voxelRes = 2.0;
    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);



}
OcclusionCulling::~OcclusionCulling()
{
}

pcl::PointCloud<pcl::PointXYZRGB> OcclusionCulling::extractColoredVisibleSurface(geometry_msgs::Pose location)
{
    //std::cout << "location: " << location.position.x << " " <<location.position.y << " " <<location.position.z << std::endl ;
    // 1 *****Frustum Culling*******
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

    std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec();

    FrustumCloud->points= output->points;

    //2:****voxel grid occlusion estimation *****
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT voxelFilter;
    voxelFilter.setInputCloud (output);
    // change in this RES
    voxelFilter.setLeafSize (0.03279f, 0.03279f, 0.03279f);
    //voxelFilter.setLeafSize (1.0f, 1.0f, 1.0f);
    //voxelFilter.setLeafSize (0.5f, 0.5f, 0.5f);

    voxelFilter.initializeVoxelGrid();

    int state,ret;

    pcl::PointXYZRGB pt,p1,p2;
    pcl::PointXYZRGB point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;

    // or send to a function estimateOcclusion

    // iterate over the entire frustum points
    for ( int i = 0; i < (int)output->points.size(); i ++ )
    {
        pcl::PointXYZRGB ptest = output->points[i];
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
           // std::cout<<"State is:"<<state<<"\n";

            if(state == 1)
            {
                //                if(count++>=10)
                //                {
                //                    foundBug = true;
                //                    break;
                //                }
              //  std::cout<<"Number of voxels in ray:"<<out_ray.size()<<"\n";
                for(uint j=0; j< out_ray.size(); j++)
                {
                    Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (out_ray[j]);
                    pcl::PointXYZRGB p = pcl::PointXYZRGB(255,255,0);
                    p.x = centroid[0];
                    p.y = centroid[1];
                    p.z = centroid[2];
                    rayCloud->points.push_back(p);
                   // std::cout<<"Ray X:"<<p.x<<" y:"<< p.y<<" z:"<< p.z<<"\n";
                }
            }

            if(state != 1)
            {
                // estimate direction to target voxel
                Eigen::Vector4f direction = centroid - cloud->sensor_origin_;
                direction.normalize ();
                // estimate entry point into the voxel grid
                float tmin = voxelFilter.rayBoxIntersection (cloud->sensor_origin_, direction,p1,p2);
                if(tmin!=-1)
                {
                    // coordinate of the boundary of the voxel grid
                    Eigen::Vector4f start = cloud->sensor_origin_ + tmin * direction;
                    linePoint.x = cloud->sensor_origin_[0]; linePoint.y = cloud->sensor_origin_[1]; linePoint.z = cloud->sensor_origin_[2];
                    //std::cout<<"Box Min X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = centroid[0]; linePoint.y = centroid[1]; linePoint.z = centroid[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    occlusionFreeCloud_local->points.push_back(ptest);
                    occlusionFreeCloud->points.push_back(ptest);

                }
            }
        }
    }
    FreeCloud.points = occlusionFreeCloud_local->points;
    visualizeFOV(location) ;
    return FreeCloud;
}

void OcclusionCulling::visualizeFOV(geometry_msgs::Pose location)
{

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud <pcl::PointXYZRGB>);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;

    camera_pose.setZero ();

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
    fc.filter (*output);

    //*** visualization the FOV *****
    std::vector<geometry_msgs::Point> fov_points;
    int c_color[3];
    geometry_msgs::Point point1;
    point1.x=fc.fp_bl[0];point1.y=fc.fp_bl[1];point1.z=fc.fp_bl[2]; fov_points.push_back(point1);//0
    point1.x=fc.fp_br[0];point1.y=fc.fp_br[1];point1.z=fc.fp_br[2]; fov_points.push_back(point1);//1
    point1.x=fc.fp_tr[0];point1.y=fc.fp_tr[1];point1.z=fc.fp_tr[2]; fov_points.push_back(point1);//2
    point1.x=fc.fp_tl[0];point1.y=fc.fp_tl[1];point1.z=fc.fp_tl[2]; fov_points.push_back(point1);//3

    point1.x=fc.np_bl[0];point1.y=fc.np_bl[1];point1.z=fc.np_bl[2]; fov_points.push_back(point1);//4
    point1.x=fc.np_br[0];point1.y=fc.np_br[1];point1.z=fc.np_br[2]; fov_points.push_back(point1);//5
    point1.x=fc.np_tr[0];point1.y=fc.np_tr[1];point1.z=fc.np_tr[2]; fov_points.push_back(point1);//6
    point1.x=fc.np_tl[0];point1.y=fc.np_tl[1];point1.z=fc.np_tl[2]; fov_points.push_back(point1);//7

    std::vector<geometry_msgs::Point> fov_linesNear;
    fov_linesNear.push_back(fov_points[4]);fov_linesNear.push_back(fov_points[5]);
    fov_linesNear.push_back(fov_points[5]);fov_linesNear.push_back(fov_points[6]);
    fov_linesNear.push_back(fov_points[6]);fov_linesNear.push_back(fov_points[7]);
    fov_linesNear.push_back(fov_points[7]);fov_linesNear.push_back(fov_points[4]);
    c_color[0]=1; c_color[1]=0; c_color[2]=1;
    linesList1 = drawLines(fov_linesNear,id++,c_color);//purple

    std::vector<geometry_msgs::Point> fov_linesFar;
    fov_linesFar.push_back(fov_points[0]);fov_linesFar.push_back(fov_points[1]);
    fov_linesFar.push_back(fov_points[1]);fov_linesFar.push_back(fov_points[2]);
    fov_linesFar.push_back(fov_points[2]);fov_linesFar.push_back(fov_points[3]);
    fov_linesFar.push_back(fov_points[3]);fov_linesFar.push_back(fov_points[0]);
    c_color[0]=1; c_color[1]=1; c_color[2]=0;
    linesList2 = drawLines(fov_linesFar,id++,c_color);//yellow


    std::vector<geometry_msgs::Point> fov_linestop;
    fov_linestop.push_back(fov_points[7]);fov_linestop.push_back(fov_points[3]);//top
    fov_linestop.push_back(fov_points[6]);fov_linestop.push_back(fov_points[2]);//top
    c_color[0]=0; c_color[1]=1; c_color[2]=0;
    linesList3 = drawLines(fov_linestop,id++,c_color);//green

    std::vector<geometry_msgs::Point> fov_linesbottom;
    fov_linesbottom.push_back(fov_points[5]);fov_linesbottom.push_back(fov_points[1]);//bottom
    fov_linesbottom.push_back(fov_points[4]);fov_linesbottom.push_back(fov_points[0]);//bottom
    c_color[0]=0; c_color[1]=0; c_color[2]=1;
    linesList4 = drawLines(fov_linesbottom,id++,c_color);//blue

    marker_array.markers.push_back(linesList1);
    marker_array.markers.push_back(linesList2);
    marker_array.markers.push_back(linesList3);
    marker_array.markers.push_back(linesList4);
    fov_pub.publish(marker_array);
}

visualization_msgs::Marker OcclusionCulling::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[])
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="base_point_cloud"; //change to "base_point_cloud" if it is used in component test package
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.08;//0.03
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration();
    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;
    //    if(c_color == 1)
    //    {
    //        color.r = 1.0;
    //        color.g = 0.0;
    //        color.b = 0.0;
    //        color.a = 1.0;
    //    }
    //    else if(c_color == 2)
    //    {
    //        color.r = 0.0;
    //        color.g = 1.0;
    //        color.b = 0.0;
    //        color.a = 1.0;
    //    }
    //    else
    //    {
    //        color.r = 0.0;
    //        color.g = 0.0;
    //        color.b = 1.0;
    //        color.a = 1.0;
    //    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}


void OcclusionCulling::visualizeOriginalPointcloud()
{
    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 cloud3;

    pcl::toROSMsg(*cloud, cloud1); //cloud of original
    pcl::toROSMsg(*occlusionFreeCloud, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling
    pcl::toROSMsg(*FrustumCloud, cloud3);

    cloud1.header.stamp = ros::Time::now();
    cloud2.header.stamp = ros::Time::now();
    cloud3.header.stamp = ros::Time::now();

    cloud1.header.frame_id = "base_point_cloud";
    cloud2.header.frame_id = "base_point_cloud";
    cloud3.header.frame_id = "base_point_cloud";

    original_cloud_pub.publish(cloud1);
    occluded_free_pub.publish(cloud2);
    frustum_pub.publish(cloud3);

}

