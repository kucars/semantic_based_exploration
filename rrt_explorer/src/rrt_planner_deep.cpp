#include <thread>
#include <chrono>

#include <cstdlib>
#include <rrt_explorer/rrt_core.h>
#include <rrt_explorer/rrt_srv_pose.h>
#include <rrt_explorer/rrt_tree.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <rrt_explorer/rrt_planner_deep.h>

double informationGain = 0  ;
double informationGainForSelectedViews[70] ;
using namespace std;

using namespace Eigen;
rrtNBV::RRTPlannerDeep::RRTPlannerDeep(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{
    manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);
    // Set up the topics and services
    params_.sampledPoints_   	 = nh_.advertise<visualization_msgs::Marker>("SampledPoints", 1000);
    params_.inspectionPath_   	 = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
    posStampedClient_            = nh_.subscribe("current_pose",     40, &rrtNBV::RRTPlannerDeep::posStampedCallback, this);
    plannerServiceDeep_          = nh_.advertiseService("rrt_planner_deep", &rrtNBV::RRTPlannerDeep::plannerCallbackDeep, this);
    pointcloud_sub_           	 = nh_.subscribe("pointcloud", 40,  &rrtNBV::RRTPlannerDeep::insertPointcloudWithTf,this);
    params_.transfromedPoseDebug = nh_.advertise<geometry_msgs::PoseStamped>("transformed_pose", 1000);
    params_.rootNodeDebug        = nh_.advertise<geometry_msgs::PoseStamped>("root_node", 1000);

    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);

    //logFilePathR_ = ros::package::getPath("rrt_explorer") + "/data/gain"
    //        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
     //       + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
     //       + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);

    //system(("mkdir -p " + logFilePathR_).c_str());
   // logFilePathR_ += "/";


    if (!setParams())
    {
        ROS_ERROR("Could not start the planner. Parameters missing!");
    }\
    // Precompute the camera field of view boundaries. The normals of the separating hyperplanes are stored
    params_.camBoundNormals_.clear();
    int g_ID_ = 0 ;
    // This loop will only be executed once
    for (int i = 0; i < params_.camPitch_.size(); i++) {
        double pitch = M_PI * params_.camPitch_[i] / 180.0;
        double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
        double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
        double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
        Eigen::Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
        Eigen::Vector3d top(cos(camTop), 0.0, -sin(camTop));
        Eigen::Vector3d right(cos(side), sin(side), 0.0);
        Eigen::Vector3d left(cos(side), -sin(side), 0.0);
        Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        Eigen::Vector3d rightR = m * right;
        Eigen::Vector3d leftR = m * left;
        rightR.normalize();
        leftR.normalize();
        std::vector<Eigen::Vector3d> camBoundNormals;
        camBoundNormals.push_back(Eigen::Vector3d(bottom.x(), bottom.y(), bottom.z()));
        camBoundNormals.push_back(Eigen::Vector3d(top.x(), top.y(), top.z()));
        camBoundNormals.push_back(Eigen::Vector3d(rightR.x(), rightR.y(), rightR.z()));
        camBoundNormals.push_back(Eigen::Vector3d(leftR.x(), leftR.y(), leftR.z()));
        params_.camBoundNormals_.push_back(camBoundNormals);
        // I added the sleep here to make a time difference between the creation of the publisher and publishing function
        //sleep(10) ;
    }
    std::string ns = ros::this_node::getName();
    std::string stlPath = "";
    mesh_ = NULL;
    if (ros::param::get(ns + "/stl_file_path", stlPath))
    {
        if (stlPath.length() > 0)
        {
            if (ros::param::get(ns + "/mesh_resolution", params_.meshResolution_))
            {
                std::fstream stlFile;
                stlFile.open(stlPath.c_str());
                if (stlFile.is_open())
                {
                    mesh_ = new mesh::StlMesh(stlFile);
                    mesh_->setResolution(params_.meshResolution_);
                    mesh_->setOctomapManager(manager_);
                    mesh_->setCameraParams(params_.camPitch_, params_.camHorizontal_, params_.camVertical_,params_.gainRange_);
                }
                else
                {
                    ROS_WARN("Unable to open STL file");
                }
            }
            else
            {
                ROS_WARN("STL mesh file path specified but mesh resolution parameter missing!");
            }
        }
    }
    ROS_INFO("********************* The Pointcloud topic name is:%s",pointcloud_sub_.getTopic().c_str());
    // Initialize the tree instance.
    ROS_INFO("*************************** rrt generated ******************************");
    //rrtTree = new rrtNBV::RrtTree(mesh_, manager_);
    //treePointDensity = new rrtNBV::InformationGainPointDensity (mesh_, manager_);
    //treeClassical = new rrtNBV::InformationGainClassic (mesh_, manager_);
    // treePointDensity->setParams(params_);
    //treeClassical->setParams(params_);
    // rrtTree->setParams(params_);
    ready_ = false;
}

rrtNBV::RRTPlannerDeep::RRTPlannerDeep(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, int planningMethod)
    : nh_(nh), nh_private_(nh_private), planning_method_(planningMethod)
{
    manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);

    // Set up the topics and services
    params_.sampledPoints_   	   = nh_.advertise<visualization_msgs::Marker>("SampledPoints", 1000);
    params_.inspectionPath_   	 = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
    posStampedClient_            = nh_.subscribe("current_pose",    10, &rrtNBV::RRTPlannerDeep::posStampedCallback, this);
    plannerServiceDeep_          = nh_.advertiseService("rrt_planner_deep", &rrtNBV::RRTPlannerDeep::plannerCallbackDeep, this);
    pointcloud_sub_           	 = nh_.subscribe("pointcloud", 1,  &rrtNBV::RRTPlannerDeep::insertPointcloudWithTf,this);
    params_.transfromedPoseDebug = nh_.advertise<geometry_msgs::PoseStamped>("transformed_pose", 1000);
    params_.rootNodeDebug        = nh_.advertise<geometry_msgs::PoseStamped>("root_node", 1000);

    if (!setParams())
    {
        ROS_ERROR("Could not start the planner. Parameters missing!");
    }
    // Precompute the camera field of view boundaries. The normals of the separating hyperplanes are stored
    params_.camBoundNormals_.clear();
    int g_ID_ = 0 ;
    // This loop will only be executed once
    for (int i = 0; i < params_.camPitch_.size(); i++) {
        double pitch = M_PI * params_.camPitch_[i] / 180.0;
        double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
        double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
        double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
        Eigen::Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
        Eigen::Vector3d top(cos(camTop), 0.0, -sin(camTop));
        Eigen::Vector3d right(cos(side), sin(side), 0.0);
        Eigen::Vector3d left(cos(side), -sin(side), 0.0);
        Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        Eigen::Vector3d rightR = m * right;
        Eigen::Vector3d leftR = m * left;
        rightR.normalize();
        leftR.normalize();
        std::vector<Eigen::Vector3d> camBoundNormals;
        camBoundNormals.push_back(Eigen::Vector3d(bottom.x(), bottom.y(), bottom.z()));
        camBoundNormals.push_back(Eigen::Vector3d(top.x(), top.y(), top.z()));
        camBoundNormals.push_back(Eigen::Vector3d(rightR.x(), rightR.y(), rightR.z()));
        camBoundNormals.push_back(Eigen::Vector3d(leftR.x(), leftR.y(), leftR.z()));
        params_.camBoundNormals_.push_back(camBoundNormals);
    }
    std::string ns = ros::this_node::getName();
    std::string stlPath = "";
    mesh_ = NULL;
    if (ros::param::get(ns + "/stl_file_path", stlPath))
    {
        if (stlPath.length() > 0)
        {
            if (ros::param::get(ns + "/mesh_resolution", params_.meshResolution_))
            {
                std::fstream stlFile;
                stlFile.open(stlPath.c_str());
                if (stlFile.is_open())
                {
                    mesh_ = new mesh::StlMesh(stlFile);
                    mesh_->setResolution(params_.meshResolution_);
                    mesh_->setOctomapManager(manager_);
                    mesh_->setCameraParams(params_.camPitch_, params_.camHorizontal_, params_.camVertical_,params_.gainRange_);
                }
                else
                {
                    ROS_WARN("Unable to open STL file");
                }
            }
            else
            {
                ROS_WARN("STL mesh file path specified but mesh resolution parameter missing!");
            }
        }
    }
    
    ROS_INFO("********************* The Pointcloud topic name is:%s",pointcloud_sub_.getTopic().c_str());
    // Initialize the tree instance.
    ROS_INFO("*************************** rrt generated ******************************");
    ROS_INFO("****************************** Pure Entropy ***************************************");
    treeClassical = new rrtNBV::InformationGainClassic (mesh_, manager_);
    treeClassical->setParams(params_);
    ready_ = false;
   
}

rrtNBV::RRTPlannerDeep::~RRTPlannerDeep()
{
    if (manager_) {
        delete manager_;
    }
    if (mesh_) {
        delete mesh_;
    }
}

bool rrtNBV::RRTPlannerDeep::plannerCallbackDeep(rrt_explorer::rrt_srv_pose::Request& req, rrt_explorer::rrt_srv_pose::Response& res)
{
    ROS_ERROR("CALL THE PLANNER "); 
    //ros::Time computationTime = ros::Time(0);
    ros::Time computationTime = ros::Time::now();
    // Check that planner is ready to compute path.
    if (!ros::ok()) {
        ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
        return true;
    }

    if (!ready_) {
        ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
        return true;
    }
    if (manager_ == NULL) {
        ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
        return true;
    }
    if (manager_->getMapSize().norm() <= 0.0) {
        ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
        return true;
    }
    
    
    int numOfSamples = 10 ;

    ROS_ERROR("numOfSamples "); 
    treeClassical->clear() ;
        // Clear old tree and reinitialize.
        treeClassical->initialize();
        for (int i = 0 ; i < numOfSamples ; i++ )
        {

            treeClassical->iterate(i) ;
        }
                std::cout << "INFORMATION GAIN " << informationGain << std::endl;
        std::cout << "treeClassical->getBestGain() " << treeClassical->getBestGain() << std::endl;
        if(treeClassical->getBestGain()  == 0 )
            return false;
        informationGain = informationGain + treeClassical->getBestGain() ;
        res.pointSelected =  treeClassical->getBestEdgePoint(req.header.frame_id);
        
  

    return true;
}

void rrtNBV::RRTPlannerDeep::posStampedCallback(const geometry_msgs::PoseStamped& pose)
{    
    std::cout << "posStampedCallback"<< std::endl;

    treeClassical->setStateFromPoseStampedMsg(pose);
    ready_ = true;
}

void rrtNBV::RRTPlannerDeep::insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{      
    
    treeClassical->insertPointcloudWithTf(pointcloud);

    //std::cout<<"FAME IS:"<<pointcloud->header.frame_id<<"\n";
    //ROS_INFO("Received PointCloud");
   /* static double last = ros::Time::now().toSec();
    if (last + params_.pcl_throttle_ < ros::Time::now().toSec())
    {          //manager_->insertPointcloudWithTf(pointcloud);
        last += params_.pcl_throttle_;
    }*/
}

bool rrtNBV::RRTPlannerDeep::isReady()
{
    return this->ready_;
}

rrtNBV::Params rrtNBV::RRTPlannerDeep::getParams()
{
    return this->params_;
}

bool rrtNBV::RRTPlannerDeep::setParams()
{
    ROS_WARN("Params"); 
    std::string ns = ros::this_node::getName();
    std::cout<<"Node name is:"<<ns<<"\n";
    bool ret = true;
    params_.v_max_ = 0.25;
    if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
        ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
                 (ns + "/system/v_max").c_str());
    }
    params_.dyaw_max_ = 0.5;
    if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
        ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
                 (ns + "/system/yaw_max").c_str());
    }
    params_.camPitch_ = {15.0};
    if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
        ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
                 (ns + "/system/camera/pitch").c_str());
    }
    params_.camHorizontal_ = {90.0};
    if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_)) {
        ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
                 (ns + "/system/camera/horizontal").c_str());
    }
    params_.camVertical_ = {60.0};
    if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
        ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.",
                 (ns + "/system/camera/vertical").c_str());
    }
    if(params_.camPitch_.size() != params_.camHorizontal_.size() ||params_.camPitch_.size() != params_.camVertical_.size() ){
        ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same length! Setting to default.");
        params_.camPitch_.clear();
        params_.camPitch_ = {15.0};
        params_.camHorizontal_.clear();
        params_.camHorizontal_ = {90.0};
        params_.camVertical_.clear();
        params_.camVertical_ = {60.0};
    }
    params_.igProbabilistic_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_)) {
        ROS_WARN(
                    "No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
                    (ns + "/nbvp/gain/probabilistic").c_str());
    }
    params_.igFree_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_)) {
        ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/free").c_str());
    }
    params_.igOccupied_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_)) {
        ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/occupied").c_str());
    }
    params_.igUnmapped_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_)) {
        ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
                 (ns + "/nbvp/gain/unmapped").c_str());
    }
    params_.igArea_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/area", params_.igArea_)) {
        ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
                 (ns + "/nbvp/gain/area").c_str());
    }
    params_.degressiveCoeff_ = 0.25;
    if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_)) {
        ROS_WARN(
                    "No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
                    (ns + "/nbvp/gain/degressive_coeff").c_str());
    }
    params_.extensionRange_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_)) {
        ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/tree/extension_range").c_str());
    }
    params_.initIterations_ = 15;
    if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_)) {
        ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
                 (ns + "/nbvp/tree/initial_iterations").c_str());
    }
    params_.dt_ = 0.1;
    if (!ros::param::get(ns + "/nbvp/dt", params_.dt_)) {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                 (ns + "/nbvp/dt").c_str());
    }
    params_.gainRange_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_)) {
        ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/gain/range").c_str());
    }
    if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
        ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
        ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
        ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
        ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
        ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
        ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
        ret = false;
    }
    params_.softBounds_ = false;
    if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_)) {
        ROS_WARN(
                    "Not specified whether scenario bounds are soft or hard. Looking for %s. Default is false",
                    (ns + "/bbx/softBounds").c_str());
    }
    params_.boundingBox_[0] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
        ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/x").c_str());
    }
    params_.boundingBox_[1] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
        ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/y").c_str());
    }
    params_.boundingBox_[2] = 0.3;
    if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
        ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
                 (ns + "/system/bbx/z").c_str());
    }
    params_.cuttoffIterations_ = 200;
    if (!ros::param::get(ns + "/nbvp/tree/cuttoff_iterations", params_.cuttoffIterations_)) {
        ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
                 (ns + "/nbvp/tree/cuttoff_iterations").c_str());
    }
    params_.zero_gain_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_)) {
        ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/zero").c_str());
    }
    params_.dOvershoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
        ROS_WARN(
                    "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
                    (ns + "/system/bbx/overshoot").c_str());
    }
    params_.log_ = false;
    if (!ros::param::get(ns + "/nbvp/log/on", params_.log_)) {
        ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/nbvp/log/on").c_str());
    }
    params_.log_throttle_ = 0.5;
    if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_)) {
        ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
                 (ns + "/nbvp/log/throttle").c_str());
    }
    params_.navigationFrame_ = "world";
    if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
        ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
                 (ns + "/tf_frame").c_str());
    }
    params_.pcl_throttle_ = 0.333;
    if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_)) {
        ROS_WARN(
                    "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.333.",
                    (ns + "/pcl_throttle").c_str());
    }
    params_.inspection_throttle_ = 0.25;
    if (!ros::param::get(ns + "/inspection_throttle", params_.inspection_throttle_)) {
        ROS_WARN(
                    "No throttle time constant for the inspection view insertion specified. Looking for %s. Default is 0.1.",
                    (ns + "/inspection_throttle").c_str());
    }
    params_.exact_root_ = true;
    if (!ros::param::get(ns + "/nbvp/tree/exact_root", params_.exact_root_)) {
        ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
                 (ns + "/nbvp/tree/exact_root").c_str());
    }
    return ret;
}
