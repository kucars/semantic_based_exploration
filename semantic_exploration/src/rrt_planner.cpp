/**
BSD 3-Clause License

Copyright (c) 2018, Khalifa University Robotics Institute
Copyright (c) 2018, Tarek Taha tarek@tarektaha.com
Copyright (c) 2018, Reem Ashour reemashour1@gmail.com
Copyright (c) 2020, Mohamed Abdelkader mohamedashraf123@gmail.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <chrono>
#include <thread>

#include <semantic_exploration/rrt_core.h>
#include <cstdlib>

#include <semantic_exploration/GetDroneState.h>
#include <semantic_exploration/rrt_planner.h>
#include <semantic_exploration/rrt_tree.h>
#include <semantic_hazard_cloud/GetSemanticColoredLabels.h>
#include <semantic_hazard_cloud/SemanticColoredLabels.h>

#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include "semantic_exploration/common.h"

using namespace std;
using namespace Eigen;

double calculateDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    return sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) +
                (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y) +
                (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
}

rrtNBV::RRTPlanner::RRTPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    if (!setParams())
    {
        ROS_ERROR("Could not start the planner. Parameters missing!");
    }

    // Set up the topics and services
    params_.inspectionPath_ = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 100);
    params_.explorationarea_ = nh_.advertise<visualization_msgs::Marker>("explorationarea", 100);
    params_.transfromedPoseDebug =
       nh_.advertise<geometry_msgs::PoseStamped>("transformed_pose", 100);
    params_.rootNodeDebug = nh_.advertise<geometry_msgs::PoseStamped>("root_node", 100);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("path", 1);
    params_.sampledPoints_ = nh_.advertise<visualization_msgs::Marker>("samplePoint", 1);
    params_.sensor_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("PathPoses", 1);
    params_.gain_pub_ = nh_.advertise<visualization_msgs::Marker>("gainPubText", 1);
    // params_.evaluatedPoints_  = nh_.advertise<visualization_msgs::Marker>("evaluatedPoint", 1);
    plannerService_ =
    nh_.advertiseService("rrt_planner", &rrtNBV::RRTPlanner::plannerCallback, this);
    params_.sample_viewpoint_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sample_points_array", 1);
    params_.sample_viewpoint_pub_ = nh_.advertise<visualization_msgs::Marker>("sample_points_", 1);
    // Either use perfect positioning from gazebo, or get the px4 estimator position through mavros
    if (params_.use_gazebo_ground_truth_)
    {
        odomClient_ = nh_.subscribe("odometry", 10, &rrtNBV::RRTPlanner::odomCallback, this);
    }
    else
    {
        posStampedClient_ = nh_.subscribe("/mavros/local_position/pose", 10,
                                          &rrtNBV::RRTPlanner::posStampedCallback, this);
        posClient_ = nh_.subscribe("pose", 10, &rrtNBV::RRTPlanner::posCallback, this);
    }

    traveled_distance = 0;
    information_gain = 0;
    firstPoseCalled = true;
    iteration_num = 0;
    accumulativeGain = 0;


    debug_load_state_param = false ; 
    if (!ros::param::get("debug_load_state", debug_load_state_param))
    {
        ROS_WARN("No option for debug_load_state. Looking for debug_load_state");
    }

    debug_save_state_param = false ; 
    if (!ros::param::get("debug_save_state", debug_save_state_param))
    {
        ROS_WARN("No option for debug_save_state. Looking for debug_load_state");
    }


    output_file_path_ = "~/map.bt";
    if (!ros::param::get("debug_write_file", output_file_path_))
    {
        ROS_WARN("No option for function. Looking for debug_write_file. Default is ~/map.bt.");
    }
    input_file_path_ = "~/map.bt";
    if (!ros::param::get("debug_read_file", input_file_path_))
    {
        ROS_WARN("No option for function. Looking for debug_read_file. Default is ~/map.bt.");
    }

    // Initiate octree
    if (params_.treeType_ == SEMANTICS_OCTREE_BAYESIAN || params_.treeType_ == SEMANTICS_OCTREE_MAX)
    {
        if (params_.treeType_ == SEMANTICS_OCTREE_BAYESIAN)
        {
            ROS_INFO("Semantic octomap generator [bayesian fusion]");
            if (debug_load_state_param)
                octomap_generator_ =  new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>(input_file_path_.c_str());
            else
                octomap_generator_ =  new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
        }
        else
        {
            ROS_INFO("Semantic octomap generator [max fusion]");
            if (debug_load_state_param)
                octomap_generator_ =  new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>(input_file_path_.c_str());
            else
                octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
        }
        toggleSemanticService = nh_.advertiseService(
            "toggle_use_semantic_color", &rrtNBV::RRTPlanner::toggleUseSemanticColor, this);
    }
    else
    {
        ROS_INFO("Color octomap generator");
        if (debug_load_state_param)
            octomap_generator_ =  new OctomapGenerator<PCLColor, ColorOcTree>(input_file_path_.c_str());
        else
            octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();

    }

    fullmapPub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    pointcloud_sub_ = nh_.subscribe(params_.pointCloudTopic_, 1, &rrtNBV::RRTPlanner::insertCloudCallback, this);

    area_marker_ = explorationAreaInit();
    computeCameraFOV();
    setupLog();
    getSemanticLabelledColors();

    octomap_generator_->setClampingThresMin(params_.clampingThresMin_);
    octomap_generator_->setClampingThresMax(params_.clampingThresMax_);
    octomap_generator_->setResolution(params_.octomapResolution_);
    octomap_generator_->setOccupancyThres(params_.occupancyThres_);
    octomap_generator_->setProbHit(params_.probHit_);
    octomap_generator_->setProbMiss(params_.probMiss_);
    octomap_generator_->setRayCastRange(params_.rayCastRange_);
    octomap_generator_->setMaxRange(params_.maxRange_);
    octomap_generator_->setSematicColoredLabels(semanticColoredLabels);
    octomap_generator_->setObjectsOfInterest(objectsOfInterest);
    //octomap_generator_->setDatasetObjects(datasetObjects);
    octomap_generator_->setConfidenceThreshold(confidenceThreshold);
    octomap_generator_->setNumOfVisitsThreshold(numOfVisitsThreshold);
    
    //debug
    //params_.camboundries_        getBestEdgeDeep= nh_.advertise<visualization_msgs::Marker>("camBoundries", 10);
    //params_.fovHyperplanes       = nh_.advertise<visualization_msgs::MarkerArray>( "hyperplanes", 100 );

    std::string ns = ros::this_node::getName();
    ROS_INFO("********************* The topic name is:%s", posStampedClient_.getTopic().c_str());
    // Initialize the tree instance.
    ROS_INFO("*************************** rrt generated ******************************");
    rrtTree = new rrtNBV::RrtTree(octomap_generator_);
    rrtTree->setParams(params_);
    // Not yet ready. need a position msg first.
    ready_ = false;
}

rrtNBV::RRTPlanner::~RRTPlanner()
{
    if (octomap_generator_)
    {
        octomap_generator_->save(params_.octomapSavePath_.c_str());
        ROS_INFO("OctoMap saved.");
       
        if (debug_save_state_param)
            octomap_generator_->writeFile(output_file_path_.c_str()) ; 

        delete octomap_generator_;
    }
    if (file_path_.is_open())
    {
        file_path_.close();
    }
    if (objects_file_path_.is_open())
    {
        objects_file_path_.close();
    }
}

void rrtNBV::RRTPlanner::getSemanticLabelledColors()
{
    semantic_hazard_cloud::GetSemanticColoredLabels getSemanticColoredLabels;
    ROS_INFO("Getting Semantic Colored Labels");

    ros::service::waitForService("get_semantic_colored_labels",ros::Duration(5.0));
    if(ros::service::call("get_semantic_colored_labels", getSemanticColoredLabels))
    {
        semantic_hazard_cloud::SemanticColoredLabels res = getSemanticColoredLabels.response.semantic_colored_labels;
        int j = 0 ; 
        for(auto i = res.semantic_colored_labels.begin(); i != res.semantic_colored_labels.end(); i++ )
        {
            octomap::ColorOcTreeNode::Color color = octomap::ColorOcTreeNode::Color((*i).color_r,
                                                                                    (*i).color_g,
                                                                                    (*i).color_b);
            std::array<int, 3> a = {int(color.b),int(color.g),int(color.r)};
            colorArray.push_back(a) ;
            ROS_INFO("%d %d %d ",a[0],a[1],a[2]);                                                                        
            ROS_WARN("%d %d %d ",colorArray[j][0],colorArray[j][1],colorArray[j][2]);  
            ROS_ERROR("%d %d %d ",color.b,color.g,color.r ); 
            j = j + 1;
            semanticColoredLabels.insert(std::make_pair((*i).label,color));
        }
    }
    else
    {
        ROS_WARN("Failed to get Semantic Colored Labels");
    }

    octomap::ColorOcTreeNode::Color bookColor = semanticColoredLabels["book"];
    ROS_INFO("Book Colors are:%d %d %d",bookColor.r,bookColor.g,bookColor.b);
}

void rrtNBV::RRTPlanner::computeCameraFOV()
{
    // Precompute the camera field of view boundaries. The normals of the separating hyperplanes are stored
    params_.camBoundNormals_.clear();
    // This loop will only be executed once
    for (uint i = 0; i < params_.camPitch_.size(); i++)
    {
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
}

void rrtNBV::RRTPlanner::setupLog()
{
    // setup logging files
    if (params_.log_)
    {
        time_t rawtime;
        struct tm* ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
        logFilePathName_ = ros::package::getPath("semantic_exploration") + "/data/" +
                           std::to_string(ptm->tm_year + 1900) + "_" +
                           std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) +
                           "_" + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) +
                           "_" + std::to_string(ptm->tm_sec);
        system(("mkdir -p " + logFilePathName_).c_str());
        logFilePathName_ += "/";
        file_path_.open((logFilePathName_ + params_.output_file_name_).c_str(), std::ios::out);
        file_path_ << "iteration_num"
                   << ","
                   << "volumetric_coverage"
                   << ","
                   << "traveled_distance"
                   << ","
                   << "information_gain_entropy"
                   << ","
                   << "semantic_gain_entropy"
                   << ","
                   << "total_gain"
                   << ","
                   << "free_cells_counter"
                   << ","
                   << "occupied_cells_counter"
                   << ","
                   << "unknown_cells_counter"
                   << ","
                   << "all_cells_counter"
                   << ","
                   << "position.x"
                   << ","
                   << "position.y"
                   << ","
                   << "position.z"
                   << ","
                   << "orientation.x"
                   << ","
                   << "orientation.y"
                   << ","
                   << "orientation.z"
                   << ","
                   << "orientation.w"
                   << ","
                   << "accumulativeGain"
                   << ","
                   << "rrt_gain"
                   << ","
                   << "freeCells"
                   << ","
                   << "UnknownCells"
                   << ","
                   << "occupiedCellsNotLabeled"
                   << ","
                   << "occupiedCellsLowConfidance"
                   << ","
                   << "occupiedCellsHighConfidance"
                   << ","
                   << "loggingTime" 
                   << ","
                   << "evaluationTime"
                   << "\n" ; 
        objects_file_path_.open((logFilePathName_ + params_.output_objects_file_name_).c_str(), std::ios::out);
        objects_file_path_ << "wall"   << "," << "building" << "," << "sky" << "," << "floor" << "," << "tree" << "," << "ceiling"<< "," << "road"<< "," << "bed "<< "," << "windowpane"<< "," << "grass"<< "," 
                   << "cabinet"<< "," << "sidewalk" << "," << "person"<< "," << "earth"<< "," << "door"<< "," << "table"<< "," << "mountain"<< "," << "plant"<< "," << "curtain"<< "," << "chair"<< ","
                   << "car"    << "," << "water"    << "," << "painting"<< "," << "sofa"<< "," << "shelf"<< "," << "house"<< "," << "sea"<< "," << "mirror"<< "," << "rug"<< "," << "field"<< "," 
                   << "armchair"<< "," << "seat"<< "," << "fence"<< "," << "desk"<< "," << "rock"<< "," << "wardrobe"<< "," << "lamp"<< "," << "bathtub"<< "," << "railing"<< "," << "cushion"<< "," 
                   << "base"<< "," << "box"<< "," << "column"<< "," << "signboard"<< "," << "chest of drawers"<< "," << "counter"<< "," << "sand"<< "," << "sink"<< "," << "skyscraper"<< "," << "fireplace"<< "," 
                   << "refrigerator"<< "," << "grandstand"<< "," << "path"<< "," << "stairs"<< "," << "runway"<< "," << "case"<< "," << "pool table"<< "," << "pillow"<< "," << "screen door"<< "," << "stairway"<< ","
                   << "river"<< "," << "bridge"<< "," << "bookcase"<< "," << "blind"<< "," << "coffee table"<< "," << "toilet"<< "," << "flower"<< "," << "book"<< "," << "hill"<< "," << "bench"<< "," 
                   << "countertop"<< "," << "stove"<< "," << "palm"<< "," << "kitchen island"<< "," << "computer"<< "," << "swivel chair"<< "," << "boat"<< "," << "bar"<< "," << "arcade machine"<< "," << "hovel"<< "," 
                   << "bus"<< "," << "towel"<< "," << "light"<< "," << "truck"<< "," << "tower"<< "," << "chandelier"<< "," << "awning"<< "," << "streetlight"<< "," << "booth"<< "," << "television"<< ","
                   << "airplane"<< "," << "dirt track"<< "," << "apparel"<< "," << "pole"<< "," << "land"<< "," << "bannister"<< "," << "escalator"<< "," << "ottoman"<< "," << "bottle"<< "," << "buffet"<< "," 
                   << "poster"<< "," << "stage"<< "," << "van"<< "," << "ship"<< "," << "fountain"<< "," << "conveyer belt"<< "," << "canopy"<< "," << "washer"<< "," << "plaything"<< "," << "swimming pool"<< ","
                   << "stool"<< "," << "barrel"<< "," << "basket"<< "," << "waterfall"<< "," << "tent"<< "," << "bag"<< "," << "minibike"<< "," << "cradle"<< "," << "oven"<< "," << "ball"<< "," 
                   << "food"<< "," << "step"<< "," << "tank"<< "," << "trade name"<< "," << "microwave"<< "," << "pot"<< "," << "animal"<< "," << "bicycle"<< "," << "lake"<< "," << "dishwasher"<< "," 
                   << "screen"<< "," << "blanket"<< "," << "sculpture"<< "," << "hood"<< "," << "sconce"<< "," << "vase"<< "," << "traffic light"<< "," << "tray"<< "," << "ashcan"<< "," << "fan"<< "," 
                   << "pier"<< "," << "crt screen"<< "," << "plate"<< "," << "monitor"<< "," << "bulletin board"<< "," << "shower"<< "," << "radiator"<< "," << "glass"<< "," << "clock" << "," << "flag"
                   << "\n";
                       // **** logging **** // 
        for (int i = 0; i < 150 ; i++ ) 
            Objectarray.push_back(0) ; 
    }
}

bool rrtNBV::RRTPlanner::toggleUseSemanticColor(std_srvs::Empty::Request& request,
                                                std_srvs::Empty::Response& response)
{
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());
   
    if (octomap_generator_->isUseSemanticColor())
        ROS_INFO("Using semantic color");
    else
        ROS_INFO("Using rgb color");

    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmapPub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing OctoMap");
    return true;
}

void rrtNBV::RRTPlanner::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    /*
     * Here we can restrict cloud insertion to a certain drone state
     * /
    semantic_exploration::GetDroneState droneStateReq;
    ROS_INFO("Getting Drone State");

    ros::service::waitForService("get_drone_state",ros::Duration(1.0));
    if(ros::service::call("get_drone_state", droneStateReq))
    {
        ROS_INFO("Current Drone State is:%d",droneStateReq.response.drone_state);
        // Don't map during takeoff
        if(droneStateReq.response.drone_state == DroneCommander::TAKE_OFF ||
           droneStateReq.response.drone_state == DroneCommander::INITIALIZATION)
            return;
    }
    else {
        return;
    }
    */
    //ROS_INFO("Received PointCloud");
    static double last = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - last > params_.pcl_throttle_)
    {
        //ROS_INFO_THROTTLE(1.0, "inserting point cloud into rrtTree");
        ros::Time tic = ros::Time::now();
        octomap_generator_->insertPointCloud(cloud_msg, params_.worldFrameId_);
        // Publish octomap
        map_msg_.header.frame_id = params_.worldFrameId_;
        map_msg_.header.stamp = cloud_msg->header.stamp;
        if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
            fullmapPub_.publish(map_msg_);
        else
            ROS_ERROR("Error serializing OctoMap");

        ros::Time toc = ros::Time::now();
        //ROS_INFO("PointCloud Insertion Took: %f", (toc - tic).toSec());
        last = ros::Time::now().toSec();
    }
}

visualization_msgs::Marker rrtNBV::RRTPlanner::explorationAreaInit()
{
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = 0;
    p.header.frame_id = params_.navigationFrame_;
    p.id = 0;
    p.ns = "workspace";
    p.type = visualization_msgs::Marker::CUBE;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
    p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
    p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, 0.0);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = params_.maxX_ - params_.minX_;
    p.scale.y = params_.maxY_ - params_.minY_;
    p.scale.z = params_.maxZ_ - params_.minZ_;
    p.color.r = static_cast<float>(200.0 / 255.0);
    p.color.g = static_cast<float>(100.0 / 255.0);
    p.color.b = 0.0f;
    p.color.a = 0.3f;
    p.lifetime = ros::Duration();
    p.frame_locked = false;
    return p;
}

bool rrtNBV::RRTPlanner::plannerCallback(semantic_exploration::GetPath::Request& req,
                                         semantic_exploration::GetPath::Response& res)
{
    timer.start("[NBVLoop]PlannerCallback");
    ROS_INFO("########### New Planning Iteration ###########");

    params_.explorationarea_.publish(area_marker_);
    
    if (!ros::ok())
    {
        ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
        return true;
    }

    // Check that planner is ready to compute path.
    if (!ready_)
    {
        ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
        return true;
    }

    if (octomap_generator_ == nullptr)
    {
        ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
        return true;
    }

    if (octomap_generator_->getMapSize().norm() <= 0.0)
    {
        ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
        return true;
    }

    res.path.clear();
    params_.marker_id = 0 ; 
    // Clear old tree and reinitialize.
    rrtTree->clear();
    rrtTree->clearInspectionPath();

    timer.start("[NBVLoop]initializeTree");
    rrtTree->initialize();
    timer.stop("[NBVLoop]initializeTree");

    ROS_INFO("Tree Initilization called");

    int loopCount = 0;
    int k = 1;

    timer.start("[NBVLoop]RRTLoop");
    while ((!rrtTree->gainFound() || rrtTree->getCounter() < params_.initIterations_) && ros::ok())
    {
        ROS_INFO_THROTTLE(0.1, "Counter:%d Cuttoff Iterations:%d GainFound:%d BestGain:%f",
                          rrtTree->getCounter(), params_.cuttoffIterations_, rrtTree->gainFound(),
                          rrtTree->getBestGain());

        if (rrtTree->getCounter() > params_.cuttoffIterations_)
        {
            ROS_WARN("No gain found, shutting down");
            ros::shutdown();
            return true;
        }
        if (loopCount > 1000 * (rrtTree->getCounter() + 1))
        {
            ROS_WARN("Exceeding maximum failed iterations, return to previous point!");           
            timer.start("[NBVLoop]GetPathBackToPrevious");
            res.path = rrtTree->getPathBackToPrevious(req.header.frame_id);
            timer.stop("[NBVLoop]GetPathBackToPrevious");
            return true;
        }
      
        timer.start("[NBVLoop]Iterate");
        if (rrtTree->iterate(1))
        {
            
        }
        timer.stop("[NBVLoop]Iterate");

        loopCount++;
        k++;
    }
    ROS_INFO("Best Gain Final :%f", rrtTree->getBestGain());
    timer.stop("[NBVLoop]RRTLoop");

    ROS_INFO("Done RRT");

    // Extract the best edge.
    res.path = rrtTree->getBestEdge(req.header.frame_id);

    //accumulativeGain += rrtTree->getBestGain();
    //bool ObjectFoundFlag = rrtTree->getObjectFlag();
    ROS_INFO("SIZE OF THE PATH %d ", res.path.size());
    
    rrtTree->memorizeBestBranch();
 
    drawPath(res.path[res.path.size() - 1], iteration_num);
    //selected_poses.push_back(res.path[0]);

    //sleep(15) ; 
  
    ros::Time tic_log = ros::Time::now();
    double res_map = octomap_generator_->getResolution();
    Eigen::Vector3d vec;
    double x, y, z;
    double all_cells_counter = 0, free_cells_counter = 0, unknown_cells_counter = 0, occupied_cells_counter = 0;
    int free_type_counter = 0, unknown_type_count = 0, occ_intr_not_vis_type_count = 0,
    occ_intr_vis_type_count = 0, occ_not_intr_type_count = 0;
    double information_gain_entropy = 0, occupancy_entropy = 0;
    double semantic_gain_entropy = 0, semantic_entropy = 0;
    double total_gain = 0;
    double probability;
    double maxThreshold = -0.5 * std::log(0.5) - ((1 - 0.5) * std::log(1 - 0.5));
    double rrt_gain = 0;
    double cGain = 0 ;
    int freeCells = 0 ;
    int UnknownCells = 0 ;
    int occupiedCellsNotLabeled = 0 ;
    int occupiedCellsLowConfidance  = 0 ;
    int occupiedCellsHighConfidance = 0 ;
    
    timer.start("[NBVLoop]InternalGainCalculation");
    for (x = params_.minX_; x <= params_.maxX_ - res_map; x += res_map)
    {
        for (y = params_.minY_; y <= params_.maxY_ - res_map; y += res_map)
        {
            // TODO: Check the boundries
            for (z = params_.minZ_; z <= params_.maxZ_ - res_map; z += res_map)
            {
                vec[0] = x;
                vec[1] = y;
                vec[2] = z;

                all_cells_counter++;

                // calculate information_gain
                VoxelStatus node = octomap_generator_->getCellProbabilityPoint(vec, &probability);
                double p = 0.5;
                if (probability != -1)
                {
                    p = probability;
                    //ROS_INFO("probability %f \n", p);
                }
                //ROS_INFO("2");
                /*if (iteration_num % 10 == 0 )*/
                       
                //ROS_INFO("3");
                int conf = octomap_generator_->getCellConfidence(vec); 
                switch (conf)
                {
                    case 0:
                    freeCells++ ;
                    break ;
                    case 1:
                    // ROS_INFO("4.1");
                    UnknownCells++;
                    break;
                    case 2:  //  occupied not semantically labeled
                    // ROS_INFO("4.2");
                    occupiedCellsNotLabeled++ ;                   
                    break ;
                    case 3:
                    // ROS_INFO("4.3");
                    occupiedCellsLowConfidance++ ;
                    break ;
                    case 4:
                    // ROS_INFO("4.4");
                    occupiedCellsHighConfidance++;                     
                    break;
                    default:
                    //ROS_INFO("4.5");      
                    break;    
                 }


                // TODO: Revise the equation
                //                occupancy_entropy = -p * std::log(p) - ((1-p) * std::log(1-p));
                //                occupancy_entropy = occupancy_entropy / maxThreshold ;
                //                information_gain_entropy += occupancy_entropy ;

                // Calculate semantic_gain
                //                double semantic_gain  = octomap_generator_->getCellIneterestGain(vec);
                //                semantic_entropy= -semantic_gain * std::log(semantic_gain) - ((1-semantic_gain) * std::log(1-semantic_gain));
                //                semantic_entropy = semantic_entropy /maxThreshold ;
                //                semantic_gain_entropy += semantic_entropy ;

                //                total_gain += (information_gain_entropy + semantic_gain_entropy) ;

                if (node == VoxelStatus::kUnknown)
                {
                     rrt_gain += params_.igUnmapped_;
                     unknown_cells_counter++;
                }
                if (node == VoxelStatus::kFree)
                {
                     free_cells_counter++;
                     rrt_gain += params_.igFree_;
                }
                if (node == VoxelStatus::kOccupied)
                {
                    //ROS_ERROR ("LOGGING OCCUPIED");
                    rrt_gain += params_.igOccupied_;
                    occupied_cells_counter++;

                    if (iteration_num % logging_period == 0 ) // ******** every 10 iterations log the data ********** // 
                    {
                        octomap::ColorOcTreeNode::Color voxelColor = octomap_generator_->getVoxelColor(vec);   
                        //ROS_INFO("%d %d %d ",int(voxelColor.r) , int(voxelColor.g) ,int(voxelColor.b) ) ;
                        //ROS_INFO("Color Size %d ", colorArray.size()) ;
                        if ( voxelColor.r != 255) // ******** Not in the data set ********** //  
                        {
                            int iter ; 
                            for ( iter = 0 ; iter < colorArray.size() ; iter++)
                            {                    
                                //ROS_INFO ("iter %d", iter) ; 
                                if (voxelColor.r == colorArray[iter][0] && voxelColor.g == colorArray[iter][1] && voxelColor.b== colorArray[iter][2]) 
                                    {
                                        //ROS_ERROR("%d %d %d ",colorArray[iter][0] , colorArray[iter][1] ,colorArray[iter][2]  ) ;
                                        //ROS_INFO("%d %d %d ",int(voxelColor.r) , int(voxelColor.g) ,int(voxelColor.b) ) ;
                                        Objectarray[iter] = Objectarray[iter] + 1 ; 
                                        break ; 
                                    }
                            }
                            if (iter == 150)
                            {
                                ROS_INFO("%d %d %d ",voxelColor.r , voxelColor.g ,voxelColor.b ) ;
                                ROS_ERROR("ERROR: SEMANTIC COLOR NOT FROM THE DATASET ");
                            }
                        }
                        else 
                        {
                            ROS_WARN("FREE - UNKNOWN _ OCCUPIED NOT SEMANTICALLY LABELED: %f", voxelColor.r);
                        }
                    }
                }
            }
        }
    }
    timer.stop("[NBVLoop]InternalGainCalculation");
    //    double theoretical_cells_value =
    //            ((params_.maxX_ - params_.minX_) * (params_.maxY_ - params_.minY_) *
    //             (params_.maxZ_ - params_.minZ_)) /
    //            (res_map * res_map * res_map);
    //    double known_cells_counter = free_cells_counter + occupied_cells_counter;
    double volumetric_coverage =
            ((free_cells_counter + occupied_cells_counter) / all_cells_counter) * 100.0;

    if (iteration_num % logging_period == 0 ) 
    {
        ROS_INFO("Logging Iteration: %d" , iteration_num ) ; 
        for (int k = 0 ; k < 150 ; k++ )
            objects_file_path_ << Objectarray[k] << "," ; 
        objects_file_path_ <<  "\n";
        std::fill(Objectarray.begin(), Objectarray.end(), 0);
    }
    
    iteration_num++;
    ros::Time toc_log = ros::Time::now();
    ROS_INFO("logging Filter took:%f", (toc_log.toSec() - tic_log.toSec()));

    file_path_ << iteration_num << ","
               << volumetric_coverage << ","
               << traveled_distance << ","
               << information_gain_entropy<< ","
               << semantic_gain_entropy << ","
               << total_gain << ","
               << free_cells_counter << ","
               << occupied_cells_counter << ","
               << unknown_cells_counter << ","
               << all_cells_counter << ","
               << res.path[0].position.x << ","
               << res.path[0].position.y << ","
               << res.path[0].position.z << ","
               << res.path[0].orientation.x << ","
               << res.path[0].orientation.y << ","
               << res.path[0].orientation.z << ","
               << res.path[0].orientation.w << ","
               << accumulativeGain << ","
               << rrt_gain << ","
               << freeCells << ","
               << UnknownCells << ","
               << occupiedCellsNotLabeled << ","
               << occupiedCellsLowConfidance << ","
               << occupiedCellsHighConfidance << "," 
               << toc_log.toSec() - tic_log.toSec() << "\n" ;

    //for (const auto &e : Objectarray) objects_file_path_ << e << ",";
    //objects_file_path_ <<  "\n";

    timer.stop("[NBVLoop]PlannerCallback");
    timer.dump();
    return true;
}

void rrtNBV::RRTPlanner::posStampedCallback(const geometry_msgs::PoseStamped& pose)
{
    if (firstPoseCalled)
    {
        firstPoseCalled = false;
        prePose = pose.pose;
        return;
    }
    else
    {
        traveled_distance += calculateDistance(prePose, pose.pose);
        prePose = pose.pose;
    }

    rrtTree->setStateFromPoseStampedMsg(pose);
    // Planner is now ready to plan.
    ready_ = true;
}

void rrtNBV::RRTPlanner::posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    rrtTree->setStateFromPoseMsg(pose);
    // Planner is now ready to plan.
    ready_ = true;
}

void rrtNBV::RRTPlanner::odomCallback(const nav_msgs::Odometry& pose)
{
    if (firstPoseCalled)
    {
        firstPoseCalled = false;
        prePose = pose.pose.pose;
        return;
    }
    else
    {
        traveled_distance += calculateDistance(prePose, pose.pose.pose);
        prePose = pose.pose.pose;
    }
    rrtTree->setStateFromOdometryMsg(pose);
    // Planner is now ready to plan.
    ready_ = true;
}

bool rrtNBV::RRTPlanner::isReady()
{
    return this->ready_;
}

rrtNBV::Params rrtNBV::RRTPlanner::getParams()
{
    return this->params_;
}

bool rrtNBV::RRTPlanner::setParams()
{
    //std::string ns = ros::this_node::getName();
    //std::cout<<"Node name is:"<<ns<<"\n";
    std::string ns = "";
    bool ret = true;
    params_.v_max_ = 0.25;
    if (!ros::param::get(ns + "/system/v_max", params_.v_max_))
    {
        ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
                 (ns + "/system/v_max").c_str());
    }
    params_.dyaw_max_ = 0.5;
    if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_))
    {
        ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
                 (ns + "/system/yaw_max").c_str());
    }
    params_.camPitch_ = {15.0};
    if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_))
    {
        ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
                 (ns + "/system/camera/pitch").c_str());
    }
    params_.camHorizontal_ = {90.0};
    if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_))
    {
        ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
                 (ns + "/system/camera/horizontal").c_str());
    }
    params_.camVertical_ = {60.0};
    if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_))
    {
        ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.",
                 (ns + "/system/camera/vertical").c_str());
    }
    params_.use_gazebo_ground_truth_ = false;
    if (!ros::param::get(ns + "/system/localization/use_gazebo_ground_truth",
                         params_.use_gazebo_ground_truth_))
    {
        ROS_WARN("using localization ground truth is not specified in the parameters while Looking "
                 "for %s. Default is false",
                 (ns + "/system/localization/use_gazebo_ground_truth").c_str());
    }
    if (params_.camPitch_.size() != params_.camHorizontal_.size() ||
        params_.camPitch_.size() != params_.camVertical_.size())
    {
        ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same "
                 "length! Setting to default.");
        params_.camPitch_.clear();
        params_.camPitch_ = {15.0};
        params_.camHorizontal_.clear();
        params_.camHorizontal_ = {90.0};
        params_.camVertical_.clear();
        params_.camVertical_ = {60.0};
    }
    params_.igProbabilistic_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_))
    {
        ROS_WARN("No gain coefficient for probability of cells specified. Looking for %s. Default "
                 "is 0.0.",
                 (ns + "/nbvp/gain/probabilistic").c_str());
    }
    params_.igFree_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_))
    {
        ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/free").c_str());
    }
    params_.igOccupied_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_))
    {
        ROS_WARN(
            "No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
            (ns + "/nbvp/gain/occupied").c_str());
    }
    params_.igUnmapped_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_))
    {
        ROS_WARN(
            "No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
            (ns + "/nbvp/gain/unmapped").c_str());
    }
    params_.igArea_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/area", params_.igArea_))
    {
        ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
                 (ns + "/nbvp/gain/area").c_str());
    }
    params_.degressiveCoeff_ = 0.25;
    if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_))
    {
        ROS_WARN("No degressive factor for gain accumulation specified. Looking for %s. Default is "
                 "0.25.",
                 (ns + "/nbvp/gain/degressive_coeff").c_str());
    }
    params_.extensionRange_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_))
    {
        ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/tree/extension_range").c_str());
    }
    params_.initIterations_ = 15;
    if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_))
    {
        ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
                 (ns + "/nbvp/tree/initial_iterations").c_str());
    }
    params_.dt_ = 0.1;
    if (!ros::param::get(ns + "/nbvp/dt", params_.dt_))
    {
        ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
                 (ns + "/nbvp/dt").c_str());
    }
    params_.gainRange_ = 1.0;
    if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_))
    {
        ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
                 (ns + "/nbvp/gain/range").c_str());
    }
    if (!ros::param::get(ns + "/bbx/minX", params_.minX_))
    {
        ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minY", params_.minY_))
    {
        ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_))
    {
        ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_))
    {
        ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_))
    {
        ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
        ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_))
    {
        ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
        ret = false;
    }
    params_.softBounds_ = false;
    if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_))
    {
        ROS_WARN("Not specified whether scenario bounds are soft or hard. Looking for %s. Default "
                 "is false",
                 (ns + "/bbx/softBounds").c_str());
    }
    params_.boundingBox_[0] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0]))
    {
        ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/x").c_str());
    }
    params_.boundingBox_[1] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1]))
    {
        ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
                 (ns + "/system/bbx/y").c_str());
    }
    params_.boundingBox_[2] = 0.3;
    if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2]))
    {
        ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
                 (ns + "/system/bbx/z").c_str());
    }
    params_.cuttoffIterations_ = 200;
    if (!ros::param::get(ns + "/nbvp/tree/cuttoff_iterations", params_.cuttoffIterations_))
    {
        ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
                 (ns + "/nbvp/tree/cuttoff_iterations").c_str());
    }
    params_.zero_gain_ = 0.0;
    if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_))
    {
        ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
                 (ns + "/nbvp/gain/zero").c_str());
    }
    params_.dOvershoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_))
    {
        ROS_WARN("No estimated overshoot value for collision avoidance specified. Looking for %s. "
                 "Default is 0.5m.",
                 (ns + "/system/bbx/overshoot").c_str());
    }
    params_.log_ = false;
    if (!ros::param::get(ns + "/nbvp/log/on", params_.log_))
    {
        ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/nbvp/log/on").c_str());
    }

    params_.log_throttle_ = 0.5;
    if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_))
    {
        ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
                 (ns + "/nbvp/log/throttle").c_str());
    }
    params_.navigationFrame_ = "world";
    if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_))
    {
        ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
                 (ns + "/tf_frame").c_str());
    }
    params_.pcl_throttle_ = 0.333;
    if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_))
    {
        ROS_WARN("No throttle time constant for the point cloud insertion specified. Looking for "
                 "%s. Default is 0.333.",
                 (ns + "/pcl_throttle").c_str());
    }
    params_.inspection_throttle_ = 0.25;
    if (!ros::param::get(ns + "/inspection_throttle", params_.inspection_throttle_))
    {
        ROS_WARN("No throttle time constant for the inspection view insertion specified. Looking "
                 "for %s. Default is 0.1.",
                 (ns + "/inspection_throttle").c_str());
    }
    params_.exact_root_ = true;
    if (!ros::param::get(ns + "/nbvp/tree/exact_root", params_.exact_root_))
    {
        ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
                 (ns + "/nbvp/tree/exact_root").c_str());
    }
    params_.output_file_name_ = "gains.csv";
    if (!ros::param::get(ns + "/output/file/name", params_.output_file_name_))
    {
        ROS_WARN("No option for output file name. Looking for %s. Default is true.",
                 (ns + "/output/file/name").c_str());
    }
    params_.output_objects_file_name_ = "gains.csv";
    if (!ros::param::get(ns + "/output/objects/file/name", params_.output_objects_file_name_))
    {
        ROS_WARN("No option for output file name. Looking for %s. Default is true.",
                 (ns + "/output/objects/file/name").c_str());
    }
    params_.utility_method_ = 1;
    if (!ros::param::get(ns + "/utility/method", params_.utility_method_))
    {
        ROS_WARN("No option for utility  function. Looking for %s. Default is true.",
                 (ns + "/utility/method").c_str());
    }
    params_.treeType_ = 1;
    if (!ros::param::get(ns + "/octomap/tree_type", params_.treeType_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 1.",
                 (ns + "/octomap/tree_type").c_str());
    }
    params_.octomapSavePath_ = "~/map.bt";
    if (!ros::param::get(ns + "/octomap/save_path", params_.octomapSavePath_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is ~/map.bt.",
                 (ns + "/octomap/save_path").c_str());
    }
    params_.pointCloudTopic_ = "/semantic_pcl/semantic_pcl";
    if (!ros::param::get(ns + "/octomap/pointcloud_topic", params_.pointCloudTopic_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is /semantic_pcl/semantic_pcl.",
                 (ns + "/octomap/pointcloud_topic").c_str());
    }
    params_.worldFrameId_ = "world";
    if (!ros::param::get(ns + "/octomap/world_frame_id", params_.worldFrameId_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is world.",
                 (ns + "/octomap/world_frame_id").c_str());
    }
    params_.octomapResolution_ = static_cast<float>(0.02);
    if (!ros::param::get(ns + "/octomap/resolution", params_.octomapResolution_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.02",
                 (ns + "/octomap/resolution").c_str());
    }
    params_.maxRange_ = 5.0f;
    if (!ros::param::get(ns + "/octomap/max_range", params_.maxRange_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 5.0",
                 (ns + "/octomap/max_range").c_str());
    }
    params_.rayCastRange_ = 2.0f;
    if (!ros::param::get(ns + "/octomap/raycast_range", params_.rayCastRange_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 2.0",
                 (ns + "/octomap/raycast_range").c_str());
    }
    params_.clampingThresMin_ = 0.12f;
    if (!ros::param::get(ns + "/octomap/clamping_thres_min", params_.clampingThresMin_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.12",
                 (ns + "/octomap/clamping_thres_min").c_str());
    }
    params_.clampingThresMax_ = 0.97f;
    if (!ros::param::get(ns + "/octomap/clamping_thres_max", params_.clampingThresMax_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.97",
                 (ns + "/octomap/clamping_thres_max").c_str());
    }
    params_.occupancyThres_ = 0.5f;
    if (!ros::param::get(ns + "/octomap/occupancy_thres", params_.occupancyThres_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.5",
                 (ns + "/octomap/occupancy_thres").c_str());
    }
    params_.probHit_ = 0.7f;
    if (!ros::param::get(ns + "/octomap/prob_hit", params_.probHit_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.7",
                 (ns + "/octomap/prob_hit").c_str());
    }
    params_.probMiss_ = 0.4f;
    if (!ros::param::get(ns + "/octomap/prob_miss", params_.probMiss_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.4",
                 (ns + "/octomap/prob_miss").c_str());
    }

    if (!ros::param::get(ns + "/objects_of_interest", objectsOfInterest))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/objects_of_interest").c_str());
    }

   /* if (!ros::param::get(ns + "/dataset_labels_ade20k", datasetObjects))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/dataset_labels_ade20k").c_str());
    }*/

    if (!ros::param::get(ns + "/confidence_threshold", confidenceThreshold))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/confidence_threshold").c_str());
    }
    //std::cout << "confidence_threshold " << confidenceThreshold<<std::endl ;

    if (!ros::param::get(ns + "/num_of_visits_threshold", numOfVisitsThreshold))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/num_of_visits_threshold").c_str());
    }
    //std::cout << "num_of_visits_threshold " << numOfVisitsThreshold<<std::endl ;
    logging_period = 10 ; 
    if (!ros::param::get(ns + "/logging_iteration", logging_period))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/logging_iteration").c_str());
    }

    return ret;
}

void rrtNBV::RRTPlanner::drawPath(geometry_msgs::Pose p, int id)
{
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.id = id;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Point p1;
    p1.x = p.position.x;
    p1.y = p.position.y;
    p1.z = p.position.z;
    line_strip.points.push_back(p1);
    line_strip.pose.orientation.w = 1.0;
    line_strip.scale.x = 0.005;
    line_strip.color.a = 1;
    line_strip.color.g = 1;
    line_strip.lifetime = ros::Duration();
    if(line_strip.points.size()>1)
        marker_pub_.publish(line_strip);
    viewpoints2.poses.push_back(p);
    viewpoints2.header.frame_id = "world";
    viewpoints2.header.stamp = ros::Time::now();
    params_.sensor_pose_pub_.publish(viewpoints2);
}
