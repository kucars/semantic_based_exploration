#ifndef EXPLORATIONPLANNER_H_
#define EXPLORATIONPLANNER_H_

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
//#include <voxel_grid_occlusion_estimation.h>
//#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>
//#include <usar_exploration/occlusion_culling.h>
// octomap
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_world/octomap_manager.h>
#include <string>
#include <tf/transform_broadcaster.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <thread>
#include <chrono>

#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <rrt_explorer/rrt_srv.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

using namespace std;
using namespace octomap;


struct Params
{
    // Initial position params
    double init_loc_x ;
    double init_loc_y ;
    double init_loc_z ;
    double init_loc_yaw ;
    bool iflog ;
};

class ExplorationPlanner {

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Params params_;
    double locationx_,locationy_,locationz_,yaw_; // current location information (target position)
    geometry_msgs::PoseStamped loc_; // exploration positions

    // Publishers and subscribers
    ros::Publisher current_pose_pub_;
//    ros::Subscriber occlusion_cloud_sub_;

    // logging
    std::string log_file_path_;
    ofstream file_path_;


public:
    //ExplorationPlanner();
    ExplorationPlanner (const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) ;
    ~ExplorationPlanner();

    //void init() ;
    bool SetParams();
    void RunStateMachine() ;

    // callback functions
    //void OcclusionCloudCallback(sensor_msgs::PointCloud2::ConstPtr msg);
};

#endif //

ExplorationPlanner::~ExplorationPlanner()
{
    if (file_path_.is_open()) {
        file_path_.close();
    }
}

ExplorationPlanner::ExplorationPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{
    // publishers
    current_pose_pub_      = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 10);

    if (!ExplorationPlanner::SetParams())
    {
        ROS_ERROR("Could not start. Parameters missing!");
    }


    // setup logging files
    /*if (params_.iflog) {
        time_t rawtime;
        struct tm * ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
        log_file_path_ = ros::package::getPath("usar_exploration") + "/data/"
                + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
                + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
                + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
        system(("mkdir -p " + log_file_path_).c_str());
        log_file_path_ += "/";
        file_path_.open((log_file_path_ + "gains.csv").c_str(), std::ios::out);
    }*/

}


void ExplorationPlanner::RunStateMachine()
{
    // read initial position from param file
    locationx_ = params_.init_loc_x ;
    locationy_ = params_.init_loc_y;
    locationz_ = params_.init_loc_z;
    yaw_       = params_.init_loc_yaw  ;
    ROS_INFO("initial Position %f , %f , %f , %f", locationx_ ,locationy_,locationz_ ,yaw_);

    int iteration_flag = 0 ;
    ros::Rate loop_rate(10);

    std_srvs::Empty srv;
    //unsigned int i = 0;

    double dt = 1.0;
    int iteration = 0;
    int seqNum = 0;

    
    // simulate the camera position publisher by broadcasting the /tf
    tf::TransformBroadcaster br;
    tf::Transform transform;
    
       
    transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
    tf::Quaternion tf_q ;
    tf_q = tf::createQuaternionFromYaw(yaw_);
    transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
    loc_.pose.position.x = locationx_;
    loc_.pose.position.y = locationy_;
    loc_.pose.position.z = locationz_;
    loc_.pose.orientation.x = tf_q.getX();
    loc_.pose.orientation.y = tf_q.getY();
    loc_.pose.orientation.z = tf_q.getZ();
    loc_.pose.orientation.w = tf_q.getW();
    loc_.header.frame_id="world";
    loc_.header.stamp=ros::Time::now();
    current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        
    for (int i = 0 ; i < 5 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationz_ = locationz_ + 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f",i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
        
    for (int i = 0 ; i < 40 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        yaw_ = yaw_ + 0.15 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
        }
   
       
    for (int i = 0 ; i < 5 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationz_ = locationz_ + 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i,loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
        
    for (int i = 0 ; i < 40 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        yaw_ = yaw_ + 0.15 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(2) ; 
    }
            
        /*for (int i = 0 ; i < 5 ; i++ )
        {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationz_ = locationz_ + 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("poseMsg %f , %f , %f , %f", loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(2) ; 
        }
        
          for (int i = 0 ; i < 40 ; i++ )
        {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        yaw_ = yaw_ + 0.15 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("poseMsg %d %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(2) ; 
        }
        
             for (int i = 0 ; i < 5 ; i++ )
        {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationz_ = locationz_ + 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("poseMsg %f , %f , %f , %f", loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(2) ; 
        }
        
          for (int i = 0 ; i < 40 ; i++ )
        {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        yaw_ = yaw_ + 0.15 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("poseMsg %d %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(2) ; 
        }
        */
    for (int i = 0 ; i < 10 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationx_ = locationx_ - 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
        
    for (int i = 0 ; i < 10 ; i++ )
    {

        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationx_ = locationx_ + 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%i poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
        
        
    for (int i = 0 ; i < 10 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationy_ = locationy_ - 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
         
    for (int i = 0 ; i < 10 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationy_ = locationy_ + 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
        
    for (int i = 0 ; i < 5 ; i++ )
    {
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        locationz_ = locationz_ - 0.1 ; 
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("%d poseMsg %f , %f , %f , %f", i, loc_.pose.position.x ,loc_.pose.position.y,loc_.pose.position.z ,yaw_);
        sleep(1) ; 
    }
        
    // Start planning: The planner is called and the computed path sent to the controller.
    ros::Time startTime = ros::Time::now() ;
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
        
       // Broadcast the TF of the camera location
        transform.setOrigin(tf::Vector3(locationx_, locationy_, locationz_) );
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw_);
        transform.setRotation(tf::Quaternion(tf_q.getX(),  tf_q.getY(), tf_q.getZ(),  tf_q.getW()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/world", "/base_point_cloud"));
        loc_.pose.position.x = locationx_;
        loc_.pose.position.y = locationy_;
        loc_.pose.position.z = locationz_;
        loc_.pose.orientation.x = tf_q.getX();
        loc_.pose.orientation.y = tf_q.getY();
        loc_.pose.orientation.z = tf_q.getZ();
        loc_.pose.orientation.w = tf_q.getW();        
        loc_.header.frame_id="world";
        loc_.header.stamp=ros::Time::now();
        current_pose_pub_.publish(loc_) ; // publish it for the current view extraction code
        ROS_INFO("sent In iterate %f %f  %f %f %f %f %f ", loc_.pose.position.x,loc_.pose.position.y,loc_.pose.position.z,loc_.pose.orientation.x,loc_.pose.orientation.y,loc_.pose.orientation.z,loc_.pose.orientation.w);

        geometry_msgs::PoseStamped poseMsg_ ; // ??? 
        
        rrt_explorer::rrt_srv planSrv;
        planSrv.request.header.stamp = ros::Time::now();
        planSrv.request.header.seq = iteration;
        planSrv.request.header.frame_id = "world";
        ros::service::waitForService("rrt_planner",ros::Duration(1.0));
        if(iteration > 25 ) 
            break; 
    
        if (ros::service::call("rrt_planner", planSrv))
        {
            seqNum++;
            if (planSrv.response.path.size() == 0)
            {
                ROS_INFO(" ****************** No Path *************************");
                ros::Duration(1.0).sleep();
            }
            else
                ROS_INFO(" #################### Path Found #######################");
        
            std::cout<< "planSrv.response.path.size()" << planSrv.response.path.size() << std::endl ; 
        
            for (int i = 0; i < planSrv.response.path.size(); i++)
            {
                poseMsg_.header.seq = seqNum ;
                poseMsg_.header.stamp = ros::Time::now();
                poseMsg_.header.frame_id = "world";
                poseMsg_.pose.position.x = planSrv.response.path[i].position.x ;
                poseMsg_.pose.position.y = planSrv.response.path[i].position.y ;
                poseMsg_.pose.position.z = planSrv.response.path[i].position.z ;
                tf::Pose pose;
                tf::poseMsgToTF(planSrv.response.path[i], pose);
                double yaw = tf::getYaw(pose.getRotation());
                tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
                poseMsg_.pose.orientation.x = quat[0] ;
                poseMsg_.pose.orientation.y = quat[1] ;
                poseMsg_.pose.orientation.z = quat[2] ;
                poseMsg_.pose.orientation.w = quat[3] ;
                current_pose_pub_.publish(poseMsg_);
                // Update next position 
                locationx_ = poseMsg_.pose.position.x ;
                locationy_ = poseMsg_.pose.position.y ;
                locationz_ = poseMsg_.pose.position.z ;
                yaw_ = yaw; 
                ros::spinOnce();
                sleep(1);
                //ros::Duration(dt).sleep();
            }
            iteration++;

        }
        else
        {
            ROS_WARN_THROTTLE(1, "Planner not reachable");
            locationx_ = locationx_ ;
            locationy_ = locationy_ ;
            locationz_ = locationz_;
            yaw_ = yaw_ ; 
            sleep(1);
            //ros::Duration(1.0).sleep();
    }
    
    ROS_INFO("next Position %f , %f , %f , %f", locationx_ ,locationy_,locationz_ ,yaw_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::Time endTime = ros::Time::now() ;
  double elapsed =  endTime.toSec() - startTime.toSec();
  std::cout<< "The planning Time " << elapsed << std::endl ; 
}

bool ExplorationPlanner::SetParams()
{
    std::string ns = ros::this_node::getName();
    std::cout<<"Node name is:"<<ns<<"\n";
    bool ret = true;

    // Environment Params
    // Note1: using (ns + "env/bbx/minX").c_str() will not work. It should be (ns + "/env/bbx/minX").c_str()
    // Note2: ros::param::get uses the params related to the namespace

    

    // Exploration Algorithm Params
    // 1- Termination
    //params_.num_of_iteration = 60;
    //if (!ros::param::get( ns+"/exp/ter/num_iteration", params_.num_of_iteration))
   // {
   //     ROS_WARN("No number of iteration for termination specified. Looking for %s.", ( "exp/ter/num_iteration"));
   // }

    // initial position params

    params_.init_loc_x = 0 ;
    if (!ros::param::get( ns+"/init/pose/x",  params_.init_loc_x))
    {
        ROS_WARN("No initial position in X is specified. Looking for %s.",  "/init/pose/x");
    }
    params_.init_loc_y = 0 ;
    if (!ros::param::get( ns+"/init/pose/y",  params_.init_loc_y))
    {
        ROS_WARN("No initial position in Y is specified. Looking for %s.",  "/init/pose/y");
    }
    params_.init_loc_yaw = 0 ;
    if (!ros::param::get( ns+"/init/pose/yaw",  params_.init_loc_yaw))
    {
        ROS_WARN("No initial position in yaw is specified. Looking for %s.",  "/init/pose/yaw");
    }
    params_.init_loc_z =0.5;
    if (!ros::param::get( ns+"/init/pose/z",  params_.init_loc_z))
    {
        ROS_WARN("No initial position in Z is specified. Looking for %s.",  "/init/pose/z");
    }
    params_.iflog = false;
    if (!ros::param::get( ns+"/log/on",  params_.iflog))
    {
        ROS_WARN("No log is specified. Looking for %s.",  "/log/on");
    }

   
 //    // Octomap manager parameters
 /*   nh_.setParam(("/tf_frame"), "world");
    nh_.setParam("/robot_frame", "base_point_cloud");
    nh_.setParam(("/resolution"), 0.15);
    nh_.setParam(("/mesh_resolution"), 1.0);
    nh_.setParam(("/visualize_max_z"), 5);
    nh_.setParam(("/sensor_max_range"), 5);
    nh_.setParam(("/map_publish_frequency"), 0.08);
    nh_.setParam(("/probability_hit"), 0.7);
    nh_.setParam(("/probability_miss"), 0.4);
    nh_.setParam(("/threshold_min"), 0.12);
    nh_.setParam(("/threshold_max"), 0.97);
    nh_.setParam(("/threshold_occupancy"), 0.7);
    nh_.setParam(("/treat_unknown_as_occupied"), false);
    nh_.setParam(("/latch_topics"), false);
  */
    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ExplorationPlanner expObj(nh , nh_private )  ;
    expObj.RunStateMachine() ;
    return 0;
}
