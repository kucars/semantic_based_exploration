#ifndef EXPLORATIONBASE_H_
#define EXPLORATIONBASE_H_

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
#include <usar_exploration/occlusion_culling.h>
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

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

using namespace std;
using namespace octomap;
bool pointcloud_recieved_sub_Flag = false ;
double pure_entropy_gain = 0;
visualization_msgs::Marker line_strip ;

struct CamParams
{
    std::vector<double> cam_pitch ;
    std::vector<double> cam_horizontal;
    std::vector<double> cam_vertical;
    std::vector<std::vector<Eigen::Vector3d> > cam_bound_normals;
};

struct Params
{
    // Environmanet
    double env_bbx_x_min  ;
    double env_bbx_x_max  ;
    double env_bbx_y_min  ;
    double env_bbx_y_max  ;
    double env_bbx_z_min  ;
    double env_bbx_z_max  ;

    // Exploration Algorithm
    int num_of_iteration ;
    double gain_range ;
    // 1- Viewpoints generation params (Geometric generation)
    double res_x   ;
    double res_y   ;
    double res_z  ;
    double res_yaw ;

    // Initial position params
    double init_loc_x ;
    double init_loc_y ;
    double init_loc_z ;
    double init_loc_yaw ;
    bool iflog ;
};

class ExplorationBase {

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Params params_;
    CamParams  cam_params_  ;
    volumetric_mapping::OctomapManager * manager_;
    double locationx_,locationy_,locationz_,yaw_; // current location information (target position)
    geometry_msgs::PoseStamped loc_; // exploration positions
    Eigen::Vector3d collision_bounding_box_;

    // for visualization
    geometry_msgs::PoseArray viewpoints;
    visualization_msgs::Marker area_marker_ ;
    std::vector<geometry_msgs::Pose> generated_poses_;
    std::vector<geometry_msgs::Pose> rejected_poses_;
    std::vector<geometry_msgs::Pose> FOVPoints_;
    std::vector<geometry_msgs::Pose> range_points_;

    // Publishers and subscribers
    ros::Publisher regected_poses_pub_;
    ros::Publisher generated_poses_pub_;
    ros::Publisher evaluated_voxels_pub_;
    ros::Publisher range_voxels_pub_;
    ros::Publisher current_pose_pub_;
    ros::Publisher max_point_pub_;
    ros::Publisher exploration_area_pub_;
    ros::Publisher sensor_pose_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher marker_text_pub_;

    ros::Subscriber occlusion_cloud_sub_;

    // logging
    std::string log_file_path_;
    ofstream file_path_;


public:
    //ExplorationBase();
    ExplorationBase (const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) ;
    ~ExplorationBase();

    //void init() ;
    bool SetParams();
    void InitCameraParams() ;
    void RunStateMachine() ;
    bool IterationTerminate(int iteration_flag) ;
    std::vector<geometry_msgs::Pose> GenerateViewPoints(int id);
    std::vector<geometry_msgs::Pose> GenerateViewPointsRandom( int id);
    bool IsInsideBounds(geometry_msgs::Pose p) ;
    bool IsValidViewpoint(geometry_msgs::Pose p ) ;
    bool IsSafe(geometry_msgs::Pose p) ;
    bool IsCollide(geometry_msgs::Pose p) ;
    double EvaluateViewPoints(geometry_msgs::Pose, int id);

    // callback functions
    void OcclusionCloudCallback(sensor_msgs::PointCloud2::ConstPtr msg);

    // visualization functions
    visualization_msgs::Marker AcceptedPoses(std::vector<geometry_msgs::Pose> accepted, int id);
    visualization_msgs::Marker RegectedPoses(std::vector<geometry_msgs::Pose> regected, int id) ;
    visualization_msgs::Marker RangePoses(std::vector<geometry_msgs::Pose> regected, int id) ;
    visualization_msgs::Marker FOVPoses(std::vector<geometry_msgs::Pose> regected, int id) ;
    visualization_msgs::Marker MaxGainPose(geometry_msgs::Pose p, int id) ;
    visualization_msgs::Marker ExplorationAreaInit() ;
    visualization_msgs::MarkerArray  MarkerPoses(std::vector<geometry_msgs::Pose> a);

};

#endif //

ExplorationBase::~ExplorationBase()
{
    if (manager_) {
      delete manager_;
    }
    if (file_path_.is_open()) {
        file_path_.close();
    }
}

ExplorationBase::ExplorationBase(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{
    manager_ =  new volumetric_mapping::OctomapManager(nh_, nh_private_);
    // Subscriber
    occlusion_cloud_sub_   = nh_.subscribe("pointcloud", 40, &ExplorationBase::OcclusionCloudCallback , this);
    // publishers
    current_pose_pub_      = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 40);
    exploration_area_pub_  = nh_.advertise<visualization_msgs::Marker>("explorationArea", 1);
    generated_poses_pub_   = nh_.advertise<visualization_msgs::Marker>("accepted_Poses", 10);
    regected_poses_pub_    = nh_.advertise<visualization_msgs::Marker>("regected_Poses", 10);
    // for debugging
    evaluated_voxels_pub_  = nh_.advertise<visualization_msgs::Marker>("evaluated_vox", 10);
    range_voxels_pub_      = nh_.advertise<visualization_msgs::Marker>("range_vox", 10);
    max_point_pub_         = nh_.advertise<visualization_msgs::Marker>("maxGainPoint", 10);
    sensor_pose_pub_       = nh_.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    marker_text_pub_       = nh_.advertise<visualization_msgs::Marker>("textPose", 1);
    marker_pub_            =  nh_.advertise<visualization_msgs::Marker>("PATH", 1);

    if (!ExplorationBase::SetParams())
    {
        ROS_ERROR("Could not start. Parameters missing!");
    }
    ExplorationBase::InitCameraParams();

    area_marker_ = ExplorationBase::ExplorationAreaInit() ;

    // setup logging files
    if (params_.iflog) {
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
    }

}

void ExplorationBase::InitCameraParams()
{
    // Precompute the camera field of view boundaries. The normals of the separating hyperplanes are stored
    cam_params_.cam_bound_normals.clear();
    // This loop will only be executed once because we have one camera
    // std::cout << "cam_params_.cam_pitch.size()" <<cam_params.cam_pitch.size()<< std::endl ;
    // Camera Params pitch: [15.0] -  horizontal: [58.0] - vertical: [45.0]
    for (int i = 0; i < cam_params_.cam_pitch.size(); i++) {
        double pitch = M_PI * cam_params_.cam_pitch[i] / 180.0; // conert to R
        double camTop = (pitch - M_PI * cam_params_.cam_vertical[i] / 360.0) + M_PI / 2.0;
        double camBottom = (pitch + M_PI * cam_params_.cam_vertical[i] / 360.0) - M_PI / 2.0;
        double side = M_PI * (cam_params_.cam_horizontal[i]) / 360.0 - M_PI / 2.0;
        Eigen::Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
        Eigen::Vector3d top(cos(camTop), 0.0, -sin(camTop));
        Eigen::Vector3d right(cos(side), sin(side), 0.0);
        Eigen::Vector3d left(cos(side), -sin(side), 0.0);
        Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        Eigen::Vector3d rightR = m * right;
        Eigen::Vector3d leftR = m * left;
        rightR.normalize();
        leftR.normalize();
        std::vector<Eigen::Vector3d> cam_bound_normals;
        cam_bound_normals.push_back(Eigen::Vector3d(bottom.x(), bottom.y(), bottom.z()));
        cam_bound_normals.push_back(Eigen::Vector3d(top.x(), top.y(), top.z()));
        cam_bound_normals.push_back(Eigen::Vector3d(rightR.x(), rightR.y(), rightR.z()));
        cam_bound_normals.push_back(Eigen::Vector3d(leftR.x(), leftR.y(), leftR.z()));
        cam_params_.cam_bound_normals.push_back(cam_bound_normals);
    }
}

void ExplorationBase::RunStateMachine()
{
    // read initial position from param file
    locationx_ =params_.init_loc_x ;
    locationy_ = params_.init_loc_y;
    locationz_ = params_.init_loc_z;
    yaw_       = params_.init_loc_yaw  ;
    ROS_INFO("initial Position %f , %f , %f , %f", locationx_ ,locationy_,locationz_ ,yaw_);

    int iteration_flag = 0 ;
    ros::Rate loop_rate(10);
    double information_gain = 0;

    //    *************read positions from txt file ****************
    //    // only add the start position
    //    std::string path = ros::package::getPath("usar_exploration");
    //    std::string str1 = path+"/resources/txt/points.txt";
    //    //std::string str1 = path+"/resources/txt/testPoses.txt";
    //    const char * filename1 = str1.c_str();
    //    assert(filename1 != NULL);
    //    filename1 = strdup(filename1);
    //    FILE *file1 = fopen(filename1, "r");
    //    if (!file1)
    //    {
    //        std::cout<<"\nCan not open the File";
    //        fclose(file1);
    //    }
    // ***************************************************************

    // simulate the camera position publisher by broadcasting the /tf
    tf::TransformBroadcaster br;
    tf::Transform transform;

    while (ros::ok())
    {
        exploration_area_pub_.publish(area_marker_);

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

        if (!ExplorationBase::IterationTerminate(iteration_flag) && pointcloud_recieved_sub_Flag ) {
        //  if (!ExplorationBase::IterationTerminate(iteration_flag) && !feof(file1) && pointcloud_recieved_sub_Flag  ) {
        //  fscanf(file1,"%lf %lf %lf %lf\n",&locationx_,&locationy_,&locationz_,&yaw_);

            std::cout << "In the " << iteration_flag <<"th Iteration" << std::endl << std::flush ;
            double global_gain = 0 ;
            int global_index = 0 ;
           // std::vector<double> gains;

            // Publish current position;
            // viewpoints.poses.push_back(loc_.pose);
            // viewpoints.header.frame_id= "world";
            // viewpoints.header.stamp = ros::Time::now();
            // sensor_pose_pub_.publish(viewpoints);

            // ------------------------------------------ Viewpoints Generation------------------------------------
            //std::vector<geometry_msgs::Pose> view_points_list = ExplorationBase::GenerateViewPoints(iteration_flag) ;
            std::vector<geometry_msgs::Pose> view_points_list = ExplorationBase::GenerateViewPointsRandom(iteration_flag) ;
            // ----------------------------------------------------------------------------------------------------

            if (view_points_list.size() == 0 ) // stay in the current position TODO: create a counter in case it got stuck
            {
                ROS_INFO("No Valid Viewpoints");
                locationx_ = locationx_  ;
                locationy_ = locationy_  ;
                locationz_ = locationz_  ;
                yaw_ = yaw_ ;
                continue ; // in order to count only the iteration where the sensor acctually move
            }
            else
            {
                ROS_INFO("Points Evaluation");
                geometry_msgs::PoseArray viewpoints2;

                for (int k = 0 ; k< view_points_list.size() ; k++){
                    // ----------------------------- Viewpoints Evaluation ------------------------------
                    double viewpoint_gain  = ExplorationBase::EvaluateViewPoints(view_points_list[k], k) ;
                    //gains.push_back(viewpoint_gain) ;
                    // ----------------------------------------------------------------------------------

                    // -------------- Visualize the generated view points -------------------------------
                    geometry_msgs::PoseStamped locc ;
                    locc.pose.position = view_points_list[k].position;
                    locc.pose.orientation = view_points_list[k].orientation;
                    locc.header.frame_id="world";
                    locc.header.stamp=ros::Time::now();
                    viewpoints2.poses.push_back(locc.pose);
                    viewpoints2.header.frame_id= "world";
                    viewpoints2.header.stamp = ros::Time::now();
                    sensor_pose_pub_.publish(viewpoints2);
                    // ---------------------------------------------------------------------------------
                  //  std::cout << "The current generated gain added to the list is: " << viewpoint_gain << std::endl ;
                   // std::cout << "global_gain before" << global_index  << " " << global_gain << std::endl <<std::flush ;

                    if (viewpoint_gain > global_gain )
                    {
                        global_gain = viewpoint_gain;
                        global_index = k;
                    }
                    //std::cout<< "view point orientation " << view_points_list[k].position.x << "  "  <<view_points_list[k].position.y << "  "  <<  view_points_list[k].position.z << std::endl ;
                    //std::cout<< "view point orientation " << view_points_list[k].orientation.x << "  "  <<view_points_list[k].orientation.y << "  "  <<  view_points_list[k].orientation.z << " " << view_points_list[k].orientation.w << std::endl ;
                    //std::cout << "Gain " << k << " " << gains[k] << std::endl <<std::flush ;
                }

                //std::cout << "global_gain after " << global_index  << " " << global_gain << std::endl <<std::flush ;
                // change the target pose
                locationx_ = view_points_list[global_index].position.x ;
                locationy_ = view_points_list[global_index].position.y ;
                locationz_ = view_points_list[global_index].position.z ;
                tf::Quaternion q(view_points_list[global_index].orientation.x, view_points_list[global_index].orientation.y, view_points_list[global_index].orientation.z, view_points_list[global_index].orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch;
                m.getRPY(roll, pitch, yaw_);

                // ----------------- Visualization ---------------------------
                information_gain = information_gain + global_gain ;
                //std::cout << "Information gain " << information_gain <<std::endl ;
                visualization_msgs::Marker highest_gain_point =  ExplorationBase::MaxGainPose(view_points_list[global_index],iteration_flag) ;
                max_point_pub_.publish(highest_gain_point) ;

                //**************** logging results ************************************************************************** //
                double res_map = 0.15 ;
                Eigen::Vector3d vec;
                double x , y , z ;
                double  all_cells_counter =0 , free_cells_counter =0 ,unKnown_cells_counter = 0 ,occupied_cells_counter =0 ;
                double pure_entropy_gain = 0 , probability,Entropy =0 ;
                for (x = params_.env_bbx_x_min; x <= params_.env_bbx_x_max- res_map; x += res_map) {
                    for (y = params_.env_bbx_y_min; y <= params_.env_bbx_y_max- res_map ; y += res_map) {
                        // TODO: Check the boundries
                        for (z = params_.env_bbx_z_min; z<= params_.env_bbx_z_max  - res_map; z += res_map) {
                            all_cells_counter++;
                            vec[0] = x; vec[1] = y ; vec[2] = z ;
                            volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                            double p = 0.5 ;
                            if (probability != -1)
                            {
                                p = probability;
                               // ROS_INFO("probability %f \n", p);
                            }
                            // TODO: Revise the equation
                            Entropy = -p * std::log(p) - ((1-p) * std::log(1-p));
                            pure_entropy_gain = pure_entropy_gain + Entropy ;
                            if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {unKnown_cells_counter++;}
                            if (node == volumetric_mapping::OctomapManager::CellStatus::kFree) {free_cells_counter++;}
                            if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {occupied_cells_counter++;}
                        }
                    }
                }
                double theoretical_cells_value = ((params_.env_bbx_x_max - params_.env_bbx_x_min) *(params_.env_bbx_y_max  -params_.env_bbx_y_min) *(params_.env_bbx_z_max - params_.env_bbx_z_min)) /(res_map*res_map*res_map);
                double knownCells = free_cells_counter+ occupied_cells_counter ;
                double calculated_coverage = ((free_cells_counter+occupied_cells_counter) / all_cells_counter)*100.0 ;
               // std::cout << "Calculated Coverage " << free_cells_counter << "  " << occupied_cells_counter << "  "  << free_cells_counter+occupied_cells_counter << "  " << all_cells_counter << "  "  << (free_cells_counter+occupied_cells_counter)/all_cells_counter << "  " << ((free_cells_counter+occupied_cells_counter)/all_cells_counter) * 100 << std::endl ;
                double gain_average_entropy = pure_entropy_gain / all_cells_counter ;
                file_path_ <<  calculated_coverage << "," << pure_entropy_gain<< "," << gain_average_entropy<< "," << information_gain <<","<< free_cells_counter << "," << occupied_cells_counter <<"," << unKnown_cells_counter  << "," << knownCells << "," << all_cells_counter << "," <<theoretical_cells_value<< "\n";
                //***********************************************************************************************************//
            }
            iteration_flag++ ;
            pointcloud_recieved_sub_Flag = false ;
            sleep(1); // I change it according to the dense data
        } // Finished one iteration of exploration

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ExplorationBase::OcclusionCloudCallback(sensor_msgs::PointCloud2::ConstPtr msg)
{
    // ROS_INFO("******************************************************************************") ;
    std::cout << "Frame ID " << msg->header.frame_id << std::endl <<std::flush ;
    //manager_->insertPointcloudWithTf(msg); // not used becuase volumetric mapping pkg is performing the mapping
    pointcloud_recieved_sub_Flag = true ;
}

bool ExplorationBase::IterationTerminate(int iteration_flag)
{
    if (iteration_flag == params_.num_of_iteration)
        return true ;
    return false ;
}

std::vector<geometry_msgs::Pose> ExplorationBase::GenerateViewPoints( int iteration_flag){
    //std::cout << "In the GenerareViewPoints Function" << std::endl << std::flush ;
    bool generate_at_current_location = true ;
    std::vector<geometry_msgs::Pose> initial_poses;
    double start_x=-1 * params_.res_x; // how much far the candidate view point. Default 0.5
    double start_y=-1 *params_.res_y ;
    double end_x= params_.res_x;
    double end_y= params_.res_y;
    double start_z=-1 *params_.res_z ;
    double end_z= params_.res_z;

    //Generating 3-D state lattice as z-axis movement is restrained (fixed)
    for (double i_x=start_x; i_x<=end_x; i_x=i_x+params_.res_x)
    {
        for (double i_y=start_y; i_y<=end_y; i_y=i_y+params_.res_y)
        {
            for (double i_z=start_z; i_z<=end_z; i_z=i_z+params_.res_z) //
            {
                for (double i_yaw=-M_PI; i_yaw<M_PI; i_yaw+=params_.res_yaw)
                {
                    // Do not generate any viewpoints in current location // it will skip the 7 orientations for at the same position
                    if (!generate_at_current_location && i_x==0 && i_y==0 && i_z==0)
                        //if (!generate_at_current_location && i_x==0 && i_y==0 )
                        continue;

                    // i_yaw = 0 means in the cuurent orientation
                    if (i_x==0 && i_y==0 &&i_z ==0 && i_yaw==0)
                        //if (i_x==0 && i_y==0 && i_yaw==0)
                        continue;

                    geometry_msgs::Pose p;
                    p.position.x = loc_.pose.position.x + i_x*cos(yaw_) + i_y*sin(yaw_);
                    p.position.y = loc_.pose.position.y - i_x*sin(yaw_) + i_y*cos(yaw_);
                    p.position.z = loc_.pose.position.z  + params_.res_z*i_z; // z-axis movement is fixed
                    tf::Quaternion tf_q ;
                    tf_q = tf::createQuaternionFromYaw(yaw_ + i_yaw);
                    p.orientation.x = tf_q.getX() ;
                    p.orientation.y = tf_q.getY() ;
                    p.orientation.z = tf_q.getZ() ;
                    p.orientation.w = tf_q.getW() ;
                    //p.orientation. =  pose_conversion::getQuaternionFromYaw(yaw_ + i_yaw);
                    initial_poses.push_back(p);

                }
            }
        }
    }

    rejected_poses_.clear()  ;
    generated_poses_.clear() ;
    //  std::cout << "initial_poses.size() "  <<initial_poses.size()  << std::endl << std::flush;
    for (int i=0; i<initial_poses.size(); i++)
    {
        if ( ExplorationBase::IsValidViewpoint(initial_poses[i]))
        {
            generated_poses_.push_back(initial_poses[i]);
        }
        else
        {
            rejected_poses_.push_back(initial_poses[i]);
        }
    }
    std::cout << "[ViewGenerator] Generated " << generated_poses_.size() << " poses (" << rejected_poses_.size() << " rejected)" << std::endl << std::flush;
    visualization_msgs::Marker generated_poses_list =  ExplorationBase::AcceptedPoses(generated_poses_,iteration_flag) ;
    visualization_msgs::Marker regected_poses_list =  ExplorationBase::RegectedPoses(rejected_poses_,iteration_flag) ;
    generated_poses_pub_.publish(generated_poses_list);
    regected_poses_pub_.publish(regected_poses_list);
    return generated_poses_ ;
}

std::vector<geometry_msgs::Pose> ExplorationBase::GenerateViewPointsRandom( int iteration_flag){

    std::vector<geometry_msgs::Pose> initial_poses;
    double extension_range = 0.5 ;

    int num_of_viewpoints= 100 ;
    double radius = sqrt(
                SQ(params_.env_bbx_x_min - params_.env_bbx_x_max) + SQ(params_.env_bbx_y_min - params_.env_bbx_y_max)
                + SQ(params_.env_bbx_z_min - params_.env_bbx_z_max));

    for (int it = 0 ; it < num_of_viewpoints ; it++)
    {

        // Random Point;
        double i_x = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        double i_y = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        double i_z = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);

        Eigen::Vector3d origin(loc_.pose.position.x, loc_.pose.position.y, loc_.pose.position.z);
        Eigen::Vector3d direction(i_x - origin[0], i_y - origin[1], i_z - origin[2]);

        if (direction.norm() >extension_range)
        {
            direction = extension_range * direction.normalized();
        }

       double i_yaw = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5); // ?????? Check if it is radian or degree
        geometry_msgs::Pose p;
        p.position.x = origin[0] + direction[0];
        p.position.y = origin[1] + direction[1];
        p.position.z = origin[2] + direction[2];
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw_ + i_yaw);
        p.orientation.x = tf_q.getX() ;
        p.orientation.y = tf_q.getY() ;
        p.orientation.z = tf_q.getZ() ;
        p.orientation.w = tf_q.getW() ;
        initial_poses.push_back(p);

    }

    rejected_poses_.clear()  ;
    generated_poses_.clear() ;
    //  std::cout << "initial_poses.size() "  <<initial_poses.size()  << std::endl << std::flush;
    for (int i=0; i<initial_poses.size(); i++)
    {
        if ( ExplorationBase::IsValidViewpoint(initial_poses[i]))
        {
            generated_poses_.push_back(initial_poses[i]);
        }
        else
        {
            rejected_poses_.push_back(initial_poses[i]);
        }
    }
    std::cout << "[ViewGenerator] Generated " << generated_poses_.size() << " poses (" << rejected_poses_.size() << " rejected)" << std::endl << std::flush;
    visualization_msgs::Marker generated_poses_list =  ExplorationBase::AcceptedPoses(generated_poses_,iteration_flag) ;
    visualization_msgs::Marker regected_poses_list =  ExplorationBase::RegectedPoses(rejected_poses_,iteration_flag) ;
    generated_poses_pub_.publish(generated_poses_list);
    regected_poses_pub_.publish(regected_poses_list);
    return generated_poses_ ;
}

bool ExplorationBase::IsInsideBounds(geometry_msgs::Pose p)
{
    if (p.position.x < params_.env_bbx_x_min || p.position.x > params_.env_bbx_x_max ||
            p.position.y < params_.env_bbx_y_min || p.position.y > params_.env_bbx_y_max ||
            p.position.z < params_.env_bbx_z_min || p.position.z > params_.env_bbx_z_max)
    {
        return false;
    }
    return true;
}

bool ExplorationBase::IsSafe(geometry_msgs::Pose p)
{
    Eigen::Vector3d loc_current(p.position.x, p.position.y,p.position.z);
    Eigen::Vector3d box_size(0.3,0.3,0.3);
    // Eigen::Vector3d box_size(0.1,0.1,0.1);
    // cell status 0: free , 1: occupied , 2: unknown
    // both unknown and free will be accepted to be visited x !=1
    // If only free views are accepted, then the condition should be changed to if X==0
    if (manager_->getCellStatusBoundingBox(loc_current,box_size)==0)
    {
        return true;
    }
    return false;
}

bool ExplorationBase::IsCollide(geometry_msgs::Pose p) {
    return false;

    // Check for collision of new connection plus some overshoot distance.
    collision_bounding_box_[0]=0.3;
    collision_bounding_box_[1]=0.3;
    collision_bounding_box_[2]=0.3;

    Eigen::Vector3d origin(loc_.pose.position.x , loc_.pose.position.y ,loc_.pose.position.z);
    Eigen::Vector3d end(p.position.x, p.position.y, p.position.z);

    //  Eigen::Vector3d direction(p.position.x - origin[0], p.position.y - origin[1], p.position.z - origin[2]);
    //  if (direction.norm() > params_.extensionRange) {
    //    direction = params_.extensionRange * direction.normalized();
    //  }

    volumetric_mapping::OctomapManager::CellStatus cell_status;
    //std::cout << "Pose: "<< p << " NewPose: " << direction + origin + direction.normalized() * dOvershoot_ << std::endl;
    cell_status = manager_->getLineStatusBoundingBox(
                origin,
                end,
                collision_bounding_box_);
    std::cout << "status is: " << cell_status << std::endl;//|| volumetric_mapping::OctomapManager::CellStatus::kUnknown
    if (cell_status == volumetric_mapping::OctomapManager::CellStatus::kFree  )
        return false;

    return true;
}

bool ExplorationBase::IsValidViewpoint(geometry_msgs::Pose p )
{
    //check current loc is free.
    if (!IsInsideBounds(p) ){
        ROS_INFO("rejected by validity");
        return false;
    }
    if (!IsSafe(p)){
        ROS_INFO("rejected by Safety" );
        return false;
    }

    if (manager_ == NULL) {
        ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
        return false ;
    }

    if (IsCollide(p)){
        ROS_INFO("rejected by collision" );
        return false;
    }

    if (manager_->getMapSize().norm() <= 0.0) {
        ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
        return false ;
    }

    return true;
}

double ExplorationBase::EvaluateViewPoints(geometry_msgs::Pose p , int id){

    Eigen::Vector3d origin(p.position.x, p.position.y, p.position.z);
    tf::Quaternion or_orientation(p.orientation.x,p.orientation.y, p.orientation.z, p.orientation.w);
    tf::Matrix3x3 m(or_orientation);
    double yaw_p , roll_p, pitch_p;
    m.getRPY(roll_p, pitch_p, yaw_p);


    // This function computes the gain
    double gain = 0.0;
    const double disc = manager_->getResolution();
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gain_range, 2.0);
    //FOVPoints_.clear() ;
    //range_points_.clear();
    //visualization_msgs::Marker fov_poses_list ;
    //visualization_msgs::Marker range_poses_list ;
    int numOfVoxelInOneView = 0 ;
    double startX = std::max(p.position.x - params_.gain_range, params_.env_bbx_x_min) ;
    double endX = std::min(p.position.x + params_.gain_range, params_.env_bbx_x_max) ;
    double startY = std::max(p.position.y - params_.gain_range, params_.env_bbx_y_min)  ;
    double endY= std::min(p.position.y + params_.gain_range, params_.env_bbx_y_max) ;
    double startZ= std::max(p.position.z - params_.gain_range, params_.env_bbx_z_min)  ;
    double endZ= std::min(p.position.z + params_.gain_range, params_.env_bbx_z_max)  ;
    // old limits
    //    for (vec[0] = std::max(p.position.x - params_.gain_range, params_.env_bbx_x_min);
    //                //         vec[0] < std::min(p.position.x + params_.gain_range, params_.env_bbx_x_max); vec[0] += disc) {
    //                //        for (vec[1] = std::max(p.position.y - params_.gain_range, params_.env_bbx_y_min);
    //                //             vec[1] < std::min(p.position.y + params_.gain_range, params_.env_bbx_y_max); vec[0] += disc) {
    //                //            for (vec[2] = std::max(p.position.z - params_.gain_range, params_.env_bbx_z_min);
    //                //                 vec[2] < std::min(p.position.z + params_.gain_range, params_.env_bbx_z_max); vec[2] += disc) {

    for (double x = startX ; x <endX; x =x+disc){
        for (double y = startY; y <endY;y=y+disc){
            for (double z =startZ ; z<endZ ; z=z+disc){
                vec[0] = x; vec[1] = y ; vec[2] = z ;
                numOfVoxelInOneView++;
                //  for debugging
                // geometry_msgs::Pose rpp ;
                // rpp.position.x = vec[0] ;
                // rpp.position.y = vec[1] ;
                // rpp.position.z = vec[2] ;
                //  range_points_.push_back(rpp);
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq) {

                    geometry_msgs::Pose rp ;
                    rp.position.x = vec[0] ;
                    rp.position.y = vec[1] ;
                    rp.position.z = vec[2] ;
                    //std::cout << "Regicted because of the far distance" << std::endl ;

                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = cam_params_
                     .cam_bound_normals.begin(); itCBN != cam_params_.cam_bound_normals.end(); itCBN++) {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++) {
                        Eigen::Vector3d normal = Eigen::AngleAxisd(yaw_p, Eigen::Vector3d::UnitZ())
                                * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());
                        if (val < SQRT2 * disc) {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView) {
                        insideAFieldOfView = true;

                        break;
                    }
                }
                if (!insideAFieldOfView) {
                    continue;
                }
                //  for debugging
                // geometry_msgs::Pose rp ;
                // rp.position.x = vec[0] ;
                // rp.position.y = vec[1] ;
                // rp.position.z = vec[2] ;
                // FOVPoints_.push_back(rp);

                // Volumetric Gain 
                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
                            vec, &probability);
                // std::cout << "probability" << probability <<  std::endl ;
                double entropy , p ;
                if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown )
                    p = 0.5 ;
                else
                    p = probability ;
                entropy= -p * std::log(p) - ((1-p) * std::log(1-p));
                
                // Semantic gain 
                double semantic_entropy ;
               // volumetric_mapping::OctomapManager::CellStatus node2 = manager_->getCellIneterestGainPoint(
               //             vec, &semantic_gain);
                double s_gain = manager_->getCellIneterestGain(vec);
                semantic_entropy= -s_gain * std::log(s_gain) - ((1-s_gain) * std::log(1-s_gain));
                
                
               // std::cout << "entropy" << entropy <<  std::endl ;
               // std::cout << "semantic_entropy" << semantic_entropy <<  std::endl ;
               // std::cout << "gain" << gain <<  std::endl ;
               // std::flush ;
                //gain += abs(entropy);
                gain = gain + entropy + semantic_entropy ;

            }
        }
    }
    std::cout << "*******************numOfVoxelInOneView**************" << numOfVoxelInOneView << std::endl ;
    //fov_poses_list =  ExplorationBase::FOVPoses(FOVPoints_, id ) ;
    //evaluated_voxels_pub_.publish(fov_poses_list);
    //range_poses_list =  ExplorationBase::RangePoses(range_points_, id ) ;
    //range_voxels_pub_.publish(range_poses_list);
    std::cout << "EVALUATE FUNCTION GAIN is " << gain << std::endl ;
    return gain;
}

visualization_msgs::Marker ExplorationBase::FOVPoses(std::vector<geometry_msgs::Pose> accepted , int id)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::POINTS;
    linksMarkerMsg.scale.x = 0.1;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10);
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0;
    std::vector<geometry_msgs::Point> acc;
    geometry_msgs::Point s ;
    for (int i = 0 ; i < accepted.size() ; i++)
    {
        s.x = accepted[i].position.x  ;
        s.y = accepted[i].position.y  ;
        s.z = accepted[i].position.z  ;
        acc.push_back(s) ;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = acc.begin();linksIterator != acc.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}

visualization_msgs::Marker ExplorationBase::RegectedPoses(std::vector<geometry_msgs::Pose> accepted , int id)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::POINTS;
    linksMarkerMsg.scale.x = 0.1;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(2);
    std_msgs::ColorRGBA color;

    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;

    std::vector<geometry_msgs::Point> acc;
    geometry_msgs::Point s ;
    for (int i = 0 ; i < accepted.size() ; i++)
    {
        s.x = accepted[i].position.x  ;
        s.y = accepted[i].position.y  ;
        s.z = accepted[i].position.z  ;
        acc.push_back(s) ;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = acc.begin();linksIterator != acc.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}

visualization_msgs::Marker ExplorationBase::AcceptedPoses(std::vector<geometry_msgs::Pose> accepted , int id)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::POINTS;
    linksMarkerMsg.scale.x = 0.1;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(2.0);
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;
    std::vector<geometry_msgs::Point> acc;
    geometry_msgs::Point s ;
    for (int i = 0 ; i < accepted.size() ; i++)
    {
        s.x = accepted[i].position.x  ;
        s.y = accepted[i].position.y  ;
        s.z = accepted[i].position.z  ;
        acc.push_back(s) ;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = acc.begin();linksIterator != acc.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);

    }
    return linksMarkerMsg;
}

visualization_msgs::Marker ExplorationBase::RangePoses(std::vector<geometry_msgs::Pose> accepted , int id)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::POINTS;
    linksMarkerMsg.scale.x = 0.1;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1.0);
    std_msgs::ColorRGBA color;
    color.r = 1.0 * (id*8);
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;
    std::vector<geometry_msgs::Point> acc;
    geometry_msgs::Point s ;
    for (int i = 0 ; i < accepted.size() ; i++)
    {
        s.x = accepted[i].position.x  ;
        s.y = accepted[i].position.y  ;
        s.z = accepted[i].position.z  ;
        acc.push_back(s) ;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = acc.begin();linksIterator != acc.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);

    }
    return linksMarkerMsg;
}

visualization_msgs::Marker ExplorationBase::MaxGainPose(geometry_msgs::Pose p , int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "marker";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.position.x;
    marker.pose.position.y = p.position.y;
    marker.pose.position.z = p.position.z;
    marker.pose.orientation.x = p.orientation.x;
    marker.pose.orientation.y =  p.orientation.y;
    marker.pose.orientation.z =  p.orientation.z;
    marker.pose.orientation.w =  p.orientation.w;
    marker.lifetime  = ros::Duration();
    marker.scale.x = 0.4;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.id = id;
    line_strip.action =  visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Point p1;
    p1.x = p.position.x;
    p1.y = p.position.y;
    p1.z = p.position.z;
    line_strip.points.push_back(p1);
    line_strip.pose.orientation.w =  1.0;
    line_strip.scale.x = 0.05;
    line_strip.color.a = 1;
    line_strip.color.g = 1;
    line_strip.lifetime  = ros::Duration();

    visualization_msgs::Marker textPose ;

    textPose.header.frame_id = "world";
    textPose.header.stamp = ros::Time::now();
    textPose.ns = "points_and_lines";
    textPose.id = id;
    textPose.action =  visualization_msgs::Marker::ADD;
    textPose.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textPose.pose.position.x = p.position.x;
    textPose.pose.position.y = p.position.y;
    textPose.pose.position.z = p.position.z +0.1;
    textPose.pose.orientation.x =0;
    textPose.pose.orientation.y =  0;
    textPose.pose.orientation.z =  0;
    textPose.pose.orientation.w = 1;
    textPose.scale.z = 0.3;
    textPose.color.r = 1.0f;
    textPose.color.g = 0.0f;
    textPose.color.b = 0.0f;
    textPose.color.a = 1.0;

    textPose.lifetime  = ros::Duration();
    std::string  m = std::to_string(id) ;
    textPose.text = m ;

    marker_text_pub_.publish(textPose) ;
    marker_pub_.publish(line_strip);
    return marker;

}

visualization_msgs::Marker  ExplorationBase::ExplorationAreaInit()
{
    visualization_msgs::Marker  p ;
    p.header.stamp = ros::Time::now();
    p.header.seq = 0;
    p.header.frame_id = "world";
    p.id = 0;
    p.ns = "workspace";
    p.type = visualization_msgs::Marker::CUBE;
    p.action = visualization_msgs::Marker::ADD;
    //  std::cout << "params_.env_bbx_x_min + params_.env_bbx_x_max"  << params_.env_bbx_x_min + params_.env_bbx_x_max<< std::endl ;

    p.pose.position.x = 0.5 * (params_.env_bbx_x_min + params_.env_bbx_x_max);
    p.pose.position.y = 0.5 * (params_.env_bbx_y_min + params_.env_bbx_y_max);
    p.pose.position.z = 0.5 * (params_.env_bbx_z_min + params_.env_bbx_z_max);
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, 0.0);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = params_.env_bbx_x_max - params_.env_bbx_x_min;
    p.scale.y = params_.env_bbx_y_max - params_.env_bbx_y_min;
    p.scale.z = params_.env_bbx_z_max - params_.env_bbx_z_min;
    p.color.r = 200.0 / 255.0;
    p.color.g = 100.0 / 255.0;
    p.color.b = 0.0;
    p.color.a = 0.3;
    p.lifetime = ros::Duration();
    p.frame_locked = false;
    return p ;
}

bool ExplorationBase::SetParams()
{
    std::string ns = ros::this_node::getName();
    std::cout<<"Node name is:"<<ns<<"\n";
    bool ret = true;

    // Environment Params
    // Note1: using (ns + "env/bbx/minX").c_str() will not work. It should be (ns + "/env/bbx/minX").c_str()
    // Note2: ros::param::get uses the params related to the namespace

    // Exploration area params
    params_.env_bbx_x_min = -100;
    if (!ros::param::get( (ns + "/env/bbx/minX").c_str(), params_.env_bbx_x_min))
    {
        ROS_WARN("No environment bounding box X specified. Looking for %s.",  (ns + "/env/bbx/minX").c_str());
    }

    params_.env_bbx_x_max = 100;
    if (!ros::param::get( ns+ "/env/bbx/maxX", params_.env_bbx_x_max))
    {
        ROS_WARN("No environment bounding box X specified. Looking for %s.", ( "env/bbx/maxX"));
    }

    params_.env_bbx_y_min = -100;
    if (!ros::param::get(ns + "/env/bbx/minY", params_.env_bbx_y_min))
    {
        ROS_WARN("No environment bounding box Y specified. Looking for %s. ", ( "env/bbx/minY"));
    }
    params_.env_bbx_y_max = 100;
    if (!ros::param::get(ns+"/env/bbx/maxY", params_.env_bbx_y_max))
    {
        ROS_WARN("No environment bounding box Y specified. Looking for %s. ", ("env/bbx/maxY"));
    }

    params_.env_bbx_z_min = -100;
    if (!ros::param::get( ns+"/env/bbx/minZ", params_.env_bbx_z_min))
    {
        ROS_WARN("No environment bounding box Z specified. Looking for %s.", ("env/bbx/minZ"));
    }
    params_.env_bbx_z_max = 100;
    if (!ros::param::get(ns+"/env/bbx/maxZ", params_.env_bbx_z_max))
    {
        ROS_WARN("No environment bounding box Z specified. Looking for %s.", ( "env/bbx/maxZ"));
    }


    // Exploration Algorithm Params
    // 1- Termination
    params_.num_of_iteration = 60;
    if (!ros::param::get( ns+"/exp/ter/num_iteration", params_.num_of_iteration))
    {
        ROS_WARN("No number of iteration for termination specified. Looking for %s.", ( "exp/ter/num_iteration"));
    }

    //2- Geometric Viewpoints Generation
    params_.res_yaw = M_PI/4.0;
    //    if (!ros::param::get(ns+"/exp/gen/res_yaw", params_.res_yaw))
    //    {
    //        ROS_WARN("No number of view point generation distanse YAW specified. Looking for %s.", ("exp/gen/res_yaw"));
    //    }

    params_.res_x = 0.5; // step size
    if (!ros::param::get(ns+ "/exp/gen/res_x", params_.res_x))
    {
        ROS_WARN("No number of view point generation distanse X  specified. Looking for %s.", ( "exp/gen/res_x"));
    }
    params_.res_y = 0.5;
    if (!ros::param::get(ns+ "/exp/gen/res_y", params_.res_y))
    {
        ROS_WARN("No number of view point generation distanse Y specified. Looking for %s.", "exp/gen/res_y");
    }
    params_.res_z = 0.5;
    if (!ros::param::get( ns+"/exp/gen/res_z", params_.res_z))
    {
        ROS_WARN("No number of view point generation distanse Z  specified. Looking for %s.", "exp/gen/res_z");
    }


    // 3- Viewpoints Evaluation
    params_.gain_range = 1.0;
    if (!ros::param::get( ns+"/exp/gain/range", params_.gain_range))
    {
        ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.","/exp/gain/range");
    }

    // Camera Params
    cam_params_.cam_pitch = {15.0};
    if (!ros::param::get( ns+"/camera/pitch", cam_params_.cam_pitch))
    {
        ROS_WARN("No camera pitch specified. Looking for %s.",  "camera/pitch");
    }
    cam_params_.cam_horizontal = {58.0};
    if (!ros::param::get( ns+"/camera/horizontal", cam_params_.cam_horizontal))
    {
        ROS_WARN("No camera horizontal specified. Looking for %s.",  "camera/horizontal");
    }

    cam_params_.cam_vertical = {45.0};
    if (!ros::param::get(ns+ "/camera/vertical", cam_params_.cam_vertical))
    {
        ROS_WARN("No camera vertical specified. Looking for %s.", "camera/vertical");
    }

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

    // Octomap manager parameters
    nh_.setParam((ns+"/tf_frame").c_str(), "world");
    nh_.setParam((ns+"/robot_frame").c_str(), "base_point_cloud");
    nh_.setParam((ns+"/resolution").c_str(), 0.15);
    nh_.setParam((ns+"/mesh_resolution").c_str(), 1.0);
    nh_.setParam((ns+"/visualize_max_z").c_str(), 5);
    nh_.setParam((ns+"/sensor_max_range").c_str(), 5);
    nh_.setParam((ns+"/map_publish_frequency").c_str(), 0.08);
    nh_.setParam((ns+"/probability_hit").c_str(), 0.7);
    nh_.setParam((ns+"/probability_miss").c_str(), 0.4);
    nh_.setParam((ns+"/threshold_min").c_str(), 0.12);
    nh_.setParam((ns+"/threshold_max").c_str(), 0.97);
    nh_.setParam((ns+"/threshold_occupancy").c_str(), 0.7);
    nh_.setParam((ns+"/treat_unknown_as_occupied").c_str(), false);
    nh_.setParam((ns+"/latch_topics").c_str(), false);

    //    // Octomap manager parameters
    nh_.setParam(("/tf_frame"), "world");
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

    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ExplorationBase expObj(nh , nh_private )  ;
    expObj.RunStateMachine() ;
    return 0;
}

