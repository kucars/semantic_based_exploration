/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_
#include <math.h>
#include <semantic_exploration/rrt_core.h>
#include <semantic_exploration/rrt_tree.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>
TimeProfiler timer;

using namespace std;
std::ofstream outfile;
rrtNBV::RrtTree::RrtTree() : rrtNBV::TreeBase::TreeBase()
{
    setup();
}

rrtNBV::RrtTree::RrtTree(OctomapGeneratorBase *octomap_generator_)
    : rrtNBV::TreeBase::TreeBase(octomap_generator_)
{
    srand(time(0)); 
    setup();
}

void rrtNBV::RrtTree::setup()
{
    
    kdTree_ = kd_create(3);
    iterationCount_ = 0;
    for (int i = 0; i < 4; i++)
    {
        inspectionThrottleTime_.push_back(ros::Time::now().toSec());
    }
    // get method
    utilityFunction = UtilityFunctionType::VOLUMETRIC;

    if (!ros::param::get("/utility/method", utilityFunction))
    {
        ROS_WARN("No option for utility  function. Looking for /utility/method. Default is 0.");
    }
    ROS_INFO("Utility Method %d ", utilityFunction);

    alphaGain = 0.5 ;
    if (!ros::param::get("/alpha_gain", alphaGain))
    {
        ROS_WARN("No option for alphaGain. Looking for alpha_gain. Default is 0.5");
    }
    betaGain = 0.5 ;
    if (!ros::param::get("/beta_gain", betaGain))
    {
        ROS_WARN("No option for alphaGain. Looking for beta_gain. Default is 0.5");
    }
    ROS_INFO("alphaGain , betaGian %f %f ", alphaGain , betaGain );
   
    numOfOrientations = 6.0;
    if (!ros::param::get("/number_of_orientation_intervals", numOfOrientations))
    {
        ROS_WARN("No option for number_of_orientation_intervals. Looking for number_of_orientation_intervals. Default is 6");
    }


    // If logging is required, set up files here
    bool ifLog = false;
    std::string ns = ros::this_node::getName();

    ros::param::get(ns + "/nbvp/log/on", ifLog);
    if (ifLog)
    {
        time_t rawtime;
        struct tm *ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
        logFilePath_ = ros::package::getPath("semantic_exploration") + "/data/" +
                std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) +
                "_" + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) +
                "_" + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
        system(("mkdir -p " + logFilePath_).c_str());
        logFilePath_ += "/";
        fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
        filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
    }
    markerCounter = 0;

    debugParam = false;
    if (!ros::param::get("/nbvp/debug", debugParam))
    {
        ROS_WARN("Debugging is off by default. Turn on with /nbvp/debug " ) ;
    }
}

rrtNBV::RrtTree::~RrtTree()
{
    delete rootNode_;
    kd_free(kdTree_);
    if (fileResponse_.is_open())
    {
        fileResponse_.close();
    }
    if (fileTree_.is_open())
    {
        fileTree_.close();
    }
    if (filePath_.is_open())
    {
        filePath_.close();
    }
}

void rrtNBV::RrtTree::setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);

    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.log_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;
    }
}

void rrtNBV::RrtTree::setStateFromPoseStampedMsg(const geometry_msgs::PoseStamped &pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);

    // debug
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = params_.navigationFrame_;
    poseMsg.pose.position.x = root_[0];
    poseMsg.pose.position.y = root_[1];
    poseMsg.pose.position.z = root_[2];
    tf::Quaternion quat2;
    quat2.setEuler(0.0, 0.0, root_[3]);
    poseMsg.pose.orientation.x = quat2.x();
    poseMsg.pose.orientation.y = quat2.y();
    poseMsg.pose.orientation.z = quat2.z();
    poseMsg.pose.orientation.w = quat2.w();
    params_.transfromedPoseDebug.publish(poseMsg);

    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.log_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }

    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;
    }
}

void rrtNBV::RrtTree::setStateFromOdometryMsg(const nav_msgs::Odometry &pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);

    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.log_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;
    }
}

void rrtNBV::RrtTree::clearInspectionPath()
{
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = params_.navigationFrame_;
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::DELETEALL;  
    params_.inspectionPath_.publish(p);
}

bool rrtNBV::RrtTree::iterate(int iterations)
{
    // In this function a new configuration is sampled and added to the tree.
    StateVec newState;
    // Sample over a sphere with the radius of the maximum diagonal of the exploration
    // space. Throw away samples outside the sampling region it exiting is not allowed
    // by the corresponding parameter. This method is to not bias the tree towards the
    // center of the exploration space.

    double radius = sqrt(SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_) +
                         SQ(params_.minZ_ - params_.maxZ_));
    //ROS_INFO("radius %f minX:%f maxX:%f minY:%f maxY:%f minZ:%f maxZ:%f",radius,params_.minX_,params_.maxX_,params_.minY_,params_.maxY_,params_.minZ_,params_.maxZ_);

    bool solutionFound = false;
    //timer.start("[RrtTree]IterateViewPointSampleing");
    while (!solutionFound)
    {
        //ROS_INFO_THROTTLE(1.0,"Finding Root");
        for (int i = 0; i < 3; i++)
        {
            newState[i] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
        }
       /* visualization_msgs::Marker marker_random_samples;
        marker_random_samples.header.frame_id = params_.worldFrameId_;
        marker_random_samples.header.stamp = ros::Time();
        marker_random_samples.ns = "my_namespace";
        marker_random_samples.id = params_.marker_id ;
        params_.marker_id++ ; 
        marker_random_samples.type = visualization_msgs::Marker::SPHERE;
        marker_random_samples.action = visualization_msgs::Marker::ADD;
        marker_random_samples.pose.position.x = newState[0];
        marker_random_samples.pose.position.y = newState[1];
        marker_random_samples.pose.position.z = newState[2]; 
        marker_random_samples.pose.orientation.x = 0.0;
        marker_random_samples.pose.orientation.y = 0.0;
        marker_random_samples.pose.orientation.z = 0.0;
        marker_random_samples.pose.orientation.w = 1.0;
        marker_random_samples.scale.x = 0.1;
        marker_random_samples.scale.y = 0.1;
        marker_random_samples.scale.z = 0.1;
        marker_random_samples.color.a = 1.0; // Don't forget to set the alpha!
        marker_random_samples.color.r = 0.0;
        marker_random_samples.color.g = 0.0;
        marker_random_samples.color.b = 1.0;
        marker_random_samples.lifetime = ros::Duration(); *///tried 0, and 10,0
        //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
        //sleep(1);

        if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
        {
            //marker_random_samples.color.r = 1.0;
            //marker_random_samples.color.g = 1.0;
            //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
            //sleep(1);
            continue;
        }

        // Offset new state by root
        newState += rootNode_->state_;
        if (!params_.softBounds_)
        {
            if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x())
            {
                //marker_random_samples.color.r = 1.0;
                //marker_random_samples.color.b = 0.0;
                //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
                //sleep(1);
                continue;
            }
            else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y())
            {
                //marker_random_samples.color.g = 1.0;
                //marker_random_samples.color.b = 0.0;
                //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
                //sleep(1);
                continue;
            }
            else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z())
            {
                //marker_random_samples.color.r = 1.0;
                //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
                //sleep(1);
                continue;
            }
            else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x())
            {
                //marker_random_samples.color.r = 1.0;
                //marker_random_samples.color.b = 0.0;
               // params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
                //sleep(1);
                continue;
            }
            else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y())
            {
                //marker_random_samples.color.g = 1.0;
                //marker_random_samples.color.b = 0.0;
                //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
                //sleep(1);
                continue;
            }
            else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z())
            {
                //marker_random_samples.color.r = 1.0;
                //params_.sample_viewpoint_pub_.publish(marker_random_samples) ; 
                //sleep(1);
                continue;
            }
        }
        solutionFound = true;
    }
    //timer.stop("[RrtTree]IterateViewPointSampleing");

    if (debugParam)
    {
        publishDebugNode(newState , 0) ; 
    }
    //*********************** DEBUG ************************** //
    //ROS_INFO("Sample Point genrated inside the exploration aera & 
    // NOT in collision with the bounding box - the bounding box is the robot dimensions-");
    //ROS_INFO("New Sample: %f %f %f " ,  newState.x() , newState.y() ,  newState.z() );

    // Find nearest neighbour
    //timer.start("[RrtTree]Iterate_kd_nearest3");
    kdres *nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    //timer.stop("[RrtTree]Iterate_kd_nearest3");

    //timer.start("[RrtTree]IterateDefinition");

    if (kd_res_size(nearest) <= 0)
    {
        ROS_ERROR("Return False");
        kd_res_free(nearest);
        return false;
    }

    rrtNBV::Node *newParent = (rrtNBV::Node *)kd_res_item_data(nearest);
    kd_res_free(nearest);

    if (debugParam)
    {
        // Node
        //Eigen::Vector4d parent_debug ; 
       // parent_debug[0] = newParent->state_[0] ; 
        //parent_debug[1] = newParent->state_[1] ; 
        //parent_debug[2] = newParent->state_[2] ; 
        //parent_debug[3] = newParent->state_[3] ; 
        publishDebugNode (newParent->state_ , 1) ; 
    }
    // Check for collision of new connection plus some overshoot distance.
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
            newState[2] - origin[2]);

    if (direction.norm() > params_.extensionRange_)
    {
        direction = params_.extensionRange_ * direction.normalized();
    }

    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];

    // ********************* debug *************************** //
    Eigen::Vector3d startPoint = origin;
    Eigen::Vector3d endPoint = direction + origin + direction.normalized() * params_.dOvershoot_;
    //timer.stop("[RrtTree]IterateDefinition");
   
   
    //timer.start("[RrtTree]IterateGetLineStatusBoundingBox");
    VoxelStatus cellStatus;
    cellStatus = manager_->getLineStatusBoundingBox(origin, endPoint, params_.boundingBox_);
    //timer.stop("[RrtTree]IterateGetLineStatusBoundingBox");

    //ROS_INFO("params_.boundingBox_ %f    %f    %f  ",params_.boundingBox_[0], params_.boundingBox_[1], params_.boundingBox_[2]);
    //ROS_INFO("params_.dOvershoot_ %f     ",params_.dOvershoot_);
    //ROS_INFO("Direction + origin + direction.normalized() * params_.dOvershoot_ x%f,y:%f,z:%f",endPoint[0],endPoint[1],endPoint[2]);
    //ROS_INFO("New State x%f,y:%f,z:%f",newState[0],newState[1],newState[2]);

  
   if (debugParam)
    { 
        publishDebugNode (newState , 2); 
        //sleep(2);
    }

    bool orientationFlag = false  ; 
    if (!ros::param::get("orientation_flag", orientationFlag))
    {
        ROS_WARN("Debugging is off by default. Turn on with /orientationFlag " ) ;
    }

    orientationDebugFile_ = "orientationDebugging.txt";
    if (!ros::param::get("/orientation_debugging_file", orientationDebugFile_))
    {
        ROS_WARN("No option for function. Looking for debug_read_file. Default is ~/map.bt.");
    }

    if (cellStatus == VoxelStatus::kFree)  // || cellStatus == VoxelStatus::kUnknown)
    {
        if (debugParam)
        { 
        publishDebugStatus(newState, 2);
        //sleep(2);
        }
    
    // visualization_msgs::Marker marker;
        
        /*
        if(cellStatus == VoxelStatus::kFree)
        {
            ROS_INFO("   - Ray is Free");
        }
        else
        {
            ROS_INFO("   - Ray is Unknown - here");
        }
        */
        if (orientationFlag)
        {
            rrtNBV::Node *newNode ;//= new rrtNBV::Node;
            double orientationSteps = 360.0/numOfOrientations ; 

            for (double i = 0.0 ; i < numOfOrientations ; i++) 
            {
                newState[3] = i * ((orientationSteps*M_PI)/180.0);

                if (debugParam)
                {
                     publishDebugNode(newState , 3) ; 
                }
                // Create new node and insert into tree
                //rrtNBV::Node *newNode = new rrtNBV::Node;
                newNode = new rrtNBV::Node;
                newNode->state_ = newState;
                newNode->parent_ = newParent;
                newNode->distance_ = newParent->distance_ + direction.norm();
                newParent->children_.push_back(newNode);
                // Object found in one view
                bool objectGainFound = false;
                timer.start("[RrtTree]getGain");
                newNode->gain_ = newParent->gain_ + getGain(newNode->state_, objectGainFound);    
                timer.stop("[RrtTree]getGain");
                
                if (newNode->gain_ > bestGain_)
                {
                    bestGain_ = newNode->gain_;
                    bestNode_ = newNode;
                }
                if (debugParam)
                {
                    publishDebugGain(newNode->state_,newNode->gain_,3); 
                }
            }
            // last thing because we want to make sure from the orientation first 
            kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);
            // Display new node
            publishNode(newNode);
            counter_++;
            ROS_INFO("bestGain_ is:%f", bestGain_);
            if (debugParam)
            {
                // The choosen one 
                publishDebugGain(newNode->state_,newNode->gain_,2); 
            }
        }
        else 
        {
            // Sample the new orientation
            //newState[3] = 2.0 * M_PI * (((double)rand()) / ((double)RAND_MAX));
            newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5) ;
            //outfile.open(orientationDebugFile_.c_str(), std::ios_base::app);
            //outfile << newState[3] << std::endl ;  
            //outfile.close();
            if (debugParam)
            {
                publishDebugNode(newState , 3) ; 
            }
            // Create new node and insert into tree
            rrtNBV::Node *newNode = new rrtNBV::Node;
            newNode->state_ = newState;
            newNode->parent_ = newParent;
            newNode->distance_ = newParent->distance_ + direction.norm();
            newParent->children_.push_back(newNode);
            // Object found in one view
            bool objectGainFound = false;

            timer.start("[RrtTree]GetGain");
            newNode->gain_ = newParent->gain_ + getGain(newNode->state_, objectGainFound);
            timer.stop("[RrtTree]GetGain");

            //ROS_INFO("newParent->gain_:%f", newParent->gain_);
            //ROS_INFO("Branch Gain IS:%f", newNode->gain_);
            //timer.start("[RrtTree]IterateInsertingpoint");
            kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);
            // Display new node
            publishNode(newNode);

            if (objectGainFound)
            {
                // Object found in one of the views
                oneViewObjectFound = true;
                if (newNode->gain_ > bestObjectGain_)
                {
                    ROS_INFO("ITERATE, OBJECT FOUND");
                    bestObjectGain_ = newNode->gain_;
                    bestObjectNode_ = newNode;
                }
            }
            else
            {
                ROS_INFO("ITERATE- VOLUMETRIC GAIN ONLY");
                // Update best IG and node if applicable
                if (newNode->gain_ > bestGain_)
                {
                    bestGain_ = newNode->gain_;
                    bestNode_ = newNode;
                }
            }
            //timer.stop("[RrtTree]IterateInsertingpoint");
            counter_++;
            ROS_INFO("bestGain_ is:%f", bestGain_);
            if (debugParam)
            {
                publishDebugGain(newNode->state_,newNode->gain_,2); 
            }
        }
        return true;
    }
    else
    {
        if (debugParam)
        { 
        publishDebugStatus(newState, 1);
        //sleep(2);
        }
        //ROS_INFO("In Collision, not free!");
    }

    //timer.dump(); 
    return false;
}

void rrtNBV::RrtTree::initialize()
{
    //timer.start("[RrtTree]initializeVariables");
    // This function is to initialize the tree, including insertion of remainder of previous best branch.
    g_ID_ = 0;
    // Remove last segment from segment list (multi agent only)
    for (uint i = 0; i < agentNames_.size(); i++)
    {
        if (agentNames_[i].compare(params_.navigationFrame_) == 0)
        {
            break;
        }
    }

    // Initialize kd-tree with root node and prepare log file
    kdTree_ = kd_create(3);

    if (params_.log_)
    {
        if (fileTree_.is_open())
        {
            fileTree_.close();
        }
        fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(),
                       std::ios::out);
    }

    rootNode_ = new Node;
    rootNode_->distance_ = 0.0;
    rootNode_->gain_ = params_.zero_gain_;
    rootNode_->parent_ = nullptr;

    if (params_.exact_root_)
    {
        if (iterationCount_ <= 1)
        {
            exact_root_ = root_;
        }
        rootNode_->state_ = exact_root_;
    }
    else
    {
        rootNode_->state_ = root_;
    }

    ROS_INFO("ROOT NODE %f,%f,%f", rootNode_->state_.x(), rootNode_->state_.y(),
             rootNode_->state_.z());

    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(),
               rootNode_);
    iterationCount_++;

    //debug
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = params_.navigationFrame_;
    poseMsg.pose.position.x = rootNode_->state_.x();
    poseMsg.pose.position.y = rootNode_->state_.y();
    poseMsg.pose.position.z = rootNode_->state_.z();
    // The orientation does not really matter
    tf::Quaternion quat2;
    quat2.setEuler(0.0, 0.0, root_[3]);
    poseMsg.pose.orientation.x = quat2.x();
    poseMsg.pose.orientation.y = quat2.y();
    poseMsg.pose.orientation.z = quat2.z();
    poseMsg.pose.orientation.w = quat2.w();
    params_.rootNodeDebug.publish(poseMsg);
    //timer.stop("[RrtTree]initializeVariables");

    timer.start("[RrtTree]initializBestBranchMemory");

    // Insert all nodes of the remainder of the previous best branch, checking for collisions and
    // recomputing the gain.
    for (typename std::vector<StateVec>::reverse_iterator iter = bestBranchMemory_.rbegin();
         iter != bestBranchMemory_.rend(); ++iter)
    {
        ROS_WARN("Memorize best Branch");
        StateVec newState = *iter;
        kdres *nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
        if (kd_res_size(nearest) <= 0)
        {
            kd_res_free(nearest);
            continue;
        }
        rrtNBV::Node *newParent = (rrtNBV::Node *)kd_res_item_data(nearest);
        kd_res_free(nearest);

        // Check for collision
        Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
        Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                newState[2] - origin[2]);

        if (direction.norm() > params_.extensionRange_)
        {
            direction = params_.extensionRange_ * direction.normalized();
        }

        newState[0] = origin[0] + direction[0];
        newState[1] = origin[1] + direction[1];
        newState[2] = origin[2] + direction[2];

        if (VoxelStatus::kFree ==
                manager_->getLineStatusBoundingBox(
                    origin, direction + origin + direction.normalized() * params_.dOvershoot_,
                    params_.boundingBox_))
        {
            // Create new node and insert into tree
            rrtNBV::Node *newNode = new rrtNBV::Node;
            newNode->state_ = newState;
            newNode->parent_ = newParent;
            newNode->distance_ = newParent->distance_ + direction.norm();
            newParent->children_.push_back(newNode);

            // Object found in one view
            bool objectGainFound = false;
            timer.start("[RrtTree]init_gain");
            newNode->gain_ = newParent->gain_ + getGain(newNode->state_, objectGainFound);
            timer.stop("[RrtTree]init_gain");
            kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

            // Display new node
            publishNode(newNode);

           if (objectGainFound)
            {
                // Object found in one of the view
                oneViewObjectFound = true;
                if (newNode->gain_ > bestObjectGain_)
                {
                    ROS_INFO(" INITIALAIZE, OBJECT FOUND ");
                    bestObjectGain_ = newNode->gain_;
                    bestObjectNode_ = newNode;
                }
            }
            else
            {
                 ROS_INFO(" INITIALAIZE - OBJECT NOT FOUND");
                // std::cout << "  " << std::endl;
                // Update best IG and node if applicable
                if (newNode->gain_ > bestGain_)
                {
                    bestGain_ = newNode->gain_;
                    bestNode_ = newNode;
                }
            }
            counter_++;
            // Draw the gain number on the branch
            if (debugParam)
            {
                publishDebugGain(newNode->state_,newNode->gain_,1);
            }
        }
    }
    timer.stop("[RrtTree]initializBestBranchMemory");
}

bool rrtNBV::RrtTree::getObjectFlag()
{
    std::cout << "oneViewObjectFound   " << oneViewObjectFound << std::endl << std::flush;
    return oneViewObjectFound;
}

std::vector<geometry_msgs::Pose> rrtNBV::RrtTree::getBestEdge(std::string targetFrame)
{

    //params_.sample_viewpoint_array_pub_.publish(sample_points_array) ; 

    ROS_INFO("return best Edge");
    std::vector<geometry_msgs::Pose> ret;
    outfile.open("path.txt", std::ios_base::app);
    //TODO: What is this oneViewObjectFound
    //if (!oneViewObjectFound)
    //{
        rrtNBV::Node *current = bestNode_;
        if (current->parent_ != nullptr)
        {
            while (current->parent_ != rootNode_ && current->parent_ != nullptr)
            {
                current = current->parent_;
            }

            geometry_msgs::Pose ret_egde;
            ret_egde.position.x = current->state_[0];
            ret_egde.position.y = current->state_[1];
            ret_egde.position.z = current->state_[2];
            float yaw = current->state_[3];
            tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
            ret_egde.orientation.x = q[0];
            ret_egde.orientation.y = q[1];
            ret_egde.orientation.z = q[2];
            ret_egde.orientation.w = q[3];
            ret.push_back(ret_egde);

            ROS_INFO("Best Edge x:%f y:%f z:%f %f %f %f %f size:%f", current->state_[0], current->state_[1],
                    current->state_[2], q[0], q[1], q[2], q[3],ret.size());

            //ret = samplePath(current->parent_->state_, current->state_, targetFrame);
            history_.push(current->parent_->state_);
            exact_root_ = current->state_;
        }
        return ret;
    /*}
    else
    {
        std::vector<geometry_msgs::Pose> ret;
        rrtNBV::Node *current = bestObjectNode_;
        if (current->parent_ != nullptr)
        {
            while (current->parent_ != rootNode_ && current->parent_ != nullptr)
            {
                current = current->parent_;
            }

            geometry_msgs::Pose ret_egde;
            ret_egde.position.x = current->state_[0];
            ret_egde.position.y = current->state_[1];
            ret_egde.position.z = current->state_[2];
            float yaw = current->state_[3];
            tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
            ret_egde.orientation.x = q[0];
            ret_egde.orientation.y = q[1];
            ret_egde.orientation.z = q[2];
            ret_egde.orientation.w = q[3];
            ret.push_back(ret_egde);
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("ret %f %f %f %f %f %f %f", current->state_[0], current->state_[1],
                    current->state_[2], q[0], q[1], q[2], q[3]);
            ROS_INFO("ret size %d", ret.size());
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            filePath_ << current->state_[0] << ",";
            filePath_ << current->state_[1] << ",";
            filePath_ << current->state_[2] << ",";
            filePath_ << current->state_[3] << "\n";
            outfile << current->state_[0] << "," << current->state_[1] << "," << current->state_[2]
                                          << "," << current->state_[3] << "\n";
            outfile << ret_egde.position.x << "," << ret_egde.position.y << ","
                    << ret_egde.position.z << "," << current->state_[3] << "," << yaw << "\n";

            // ret = samplePath(current->parent_->state_, current->state_, targetFrame);
            history_.push(current->parent_->state_);
            exact_root_ = current->state_;
        }
        return ret;
    }*/
}

double rrtNBV::RrtTree::getBestGain()
{
    return bestGain_;

    /*if (!oneViewObjectFound)
    {
        ROS_ERROR("GetBestGain ********************************************************************************************");
        return bestGain_;
    }
    else
        return bestObjectGain_;*/
}

Eigen::Vector4d rrtNBV::RrtTree::getRootNode()

{
    return rootNode_->state_;
}

geometry_msgs::Pose rrtNBV::RrtTree::getBestEdgeDeep(std::string targetFrame)
{
    //if (!oneViewObjectFound)
    //{
        //ROS_INFO("the best gain in this iteration is %f", bestGain_) ;
        //std::cout << "bestNode NODE " << bestNode_->state_[0] ;
        // This function returns the first edge of the best branch
        geometry_msgs::Pose ret;
        ret.position.x = bestNode_->state_[0];
        ret.position.y = bestNode_->state_[1];
        ret.position.z = bestNode_->state_[2];
        float yaw = bestNode_->state_[3];
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        ret.orientation.x = q[0];
        ret.orientation.y = q[1];
        ret.orientation.z = q[2];
        ret.orientation.w = q[3];
        return ret;
    /*}
    else
    {
        //ROS_INFO("the best gain in this iteration is %f", bestGain_) ;
        //std::cout << "bestNode NODE " << bestNode_->state_[0] ;
        // This function returns the first edge of the best branch
        geometry_msgs::Pose ret;
        ret.position.x = bestObjectNode_->state_[0];
        ret.position.y = bestObjectNode_->state_[1];
        ret.position.z = bestObjectNode_->state_[2];
        float yaw = bestObjectNode_->state_[3];
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        ret.orientation.x = q[0];
        ret.orientation.y = q[1];
        ret.orientation.z = q[2];
        ret.orientation.w = q[3];
        return ret;
    }*/
}

double rrtNBV::RrtTree::getGain(StateVec state, bool &objectGainFound)
{
    double gainValue = 0;
    double tic, toc;
    switch (utilityFunction)
    {
    case VOLUMETRIC:
        ROS_INFO("Volumetric");
        gainValue = gain_volumetric(state, objectGainFound);
        ROS_INFO("Calculating Gain took:%f", toc - tic);
        break;
    case REAR_SIDE_VOXEL:
        ROS_INFO("Iterate Rear side VOXEL");
        gainValue = gain_rsv(state, objectGainFound);
        break;
    case SEMANTIC_REAR_SIDE_VOXEL:
        // Count the rays that ends up with object of interest
        ROS_INFO("Semantic rear side VOXEL");
        gainValue = gain_rsvs(state, objectGainFound);
        break;
    case REAR_SIDE_ENTROPY:
        ROS_INFO("rear side ENTROPY");
        gainValue = gain_rse(state, objectGainFound);
        break;
    case SEMANTIC_REAR_SIDE_ENTROPY:
        ROS_INFO("Semantic rear side ENTROPY");
        gainValue = gain_rses(state, objectGainFound);
        break;
    case PURE_ENTROPY:
        ROS_INFO("Pure ENTROPY");
        gainValue = gain_pure_entropy(state, objectGainFound);
        break;
    case AVERAGE_ENTROPY:
        ROS_INFO("AVG ENTROPY");
        gainValue = gain_avg_entropy(state, objectGainFound);
        break;
    case OCCLUSION_AWARE:
        ROS_INFO("occlusion aware ENTROPY");
        gainValue = gain_occlusion_aware(state, objectGainFound);
        break;
    case UNOBSERVED_VOXEL:
        ROS_INFO("unobserved voxel");
        gainValue = gain_unobserved_voxel(state, objectGainFound);
        break;
    case SEMANTIC_VISIBLE_VOXEL:
        ROS_INFO("Semantic Visible Voxels");
        gainValue = gain_svv(state, objectGainFound);
        break;
    case SEMANTIC_OCCLUSION_AWARE:
        ROS_INFO("Semantic Occlusion aware Voxels");
        gainValue = gain_semantic_occlusion_aware(state, objectGainFound);
        break;
    case SEMANTIC_OBJ_INTEREST_NUM_OF_VISITS:
        ROS_INFO("Semantic obj of interest- Num of visits");
        gainValue = gain_semantic_obj_interest_num_visits(state, objectGainFound);
        break ;
    default:
        ROS_WARN("Utility function type not found!");
    }
    return gainValue;
}

// volumetric information using rrt package
double rrtNBV::RrtTree::gain_volumetric(StateVec state, bool &objectGainFound)
{
    ROS_INFO("Volumetric GAIN");
    objectGainFound = false;
    // This function computes the gain
    double gain = 0.0;
    const double disc = manager_->getResolution();
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    double minX = std::max(state[0] - params_.gainRange_, params_.minX_); 
    double maxX = std::min(state[0] + params_.gainRange_, params_.maxX_);
    double minY = std::max(state[1] - params_.gainRange_, params_.minY_); 
    double maxY = std::min(state[1] + params_.gainRange_, params_.maxY_);
    double minZ = std::max(state[2] - params_.gainRange_, params_.minZ_);
    double maxZ = std::min(state[2] + params_.gainRange_, params_.maxZ_);
    // Iterate over all nodes within the allowed distance
    for (vec[0] = minX; vec[0] < maxX; vec[0] += disc)
    {
        for (vec[1] = minY; vec[1] < maxY; vec[1] += disc)
        {
            for (vec[2] = minZ; vec[2] < maxZ; vec[2] += disc)
            {
                timer.start("[RrtTree]DistCheck");
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    timer.stop("[RrtTree]DistCheck");
                    continue;
                }
                timer.stop("[RrtTree]DistCheck");
                // ROS_INFO_THROTTLE(1.0,
                //                   " Resolution is:%f vec[0]:%f vec[1]:%f vec[2]:%f gain Range:%f",
                //                   disc, vec[0], vec[1], vec[2], params_.gainRange_);
                // ROS_INFO_THROTTLE(1.0, "   MinX_:%f minY_:%f minZ_:%f maxX_:%f maxY_:%f maxZ_:%f",
                //                   params_.minX_, params_.minY_, params_.minZ_, params_.maxX_,
                //                   params_.maxY_, params_.maxZ_);
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                timer.start("[RrtTree]InboundCheck");
                ROS_INFO_THROTTLE(1.0,"Cambound Size:%d ",params_.camBoundNormals_.size());
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());
                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }
                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                timer.stop("[RrtTree]InboundCheck");
                if (!insideAFieldOfView)
                {
                    continue;
                }               

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;

                timer.start("[RrtTree]getCellProbabilityPoint");
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                timer.stop("[RrtTree]getCellProbabilityPoint");
                
                // TODO: Add probabilistic gain
                timer.start("[RrtTree]getVisibility");
                if (node == VoxelStatus::kUnknown)
                {
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gain += params_.igUnmapped_;
                    }
                }
                else if (node == VoxelStatus::kOccupied)
                {
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gain += params_.igOccupied_;
                    }
                }
                else
                {
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gain += params_.igFree_;
                    }
                }
                timer.stop("[RrtTree]getVisibility");
            }
        }
    }
    // Scale with volume
    gain *= pow(disc, 3.0);//* exp(-params_.degressiveCoeff_ * newNode->distance_);
    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// This function computes the gain using "Rear side voxel". ref[1] " A comparison of a volumetric information gain metrics for active 3D object reconstruction"
double rrtNBV::RrtTree::gain_rsv(StateVec state, bool &objectGainFound)
{
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainObjOfInt = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    int numOfRearSideVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                if (node == VoxelStatus::kUnknown)
                {
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainUnknown += 1;
                        numOfUnknownVisibleVoxels++;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainOccupied += 0;
                        numOfOccupiedVisibleVoxels++;

                        bool rearSideVoxel = this->manager_->getRearSideVoxel(origin, vec);
                        //std::cout << " THE RESUTLS IS ** " << rearSideVoxel << std::endl  ;
                        if (rearSideVoxel)
                        {
                            gainObjOfInt += 1;
                            numOfRearSideVoxels++;
                        }
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainFree += 0;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Visibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Rear Side Voxel In One View is " << gainObjOfInt << std::endl
              << std::flush;
    ;
    std::cout << " number Of Visible Rear Side Interest Voxels In One View is "
              << numOfRearSideVoxels << std::endl
              << std::flush;
    ;

    if (gainObjOfInt > 0)
    {
        gain = gainObjOfInt;
        objectGainFound = true;
        std::cout << "Object Gain " << gain << std::endl;
    }
    else
    {
        gain = gainUnknown;
        std::cout << "Volumetric Gain " << gain << std::endl;
    }

    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    // Scale with volume
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f", gain);
    return gain;
}

// proposed semantic rear side voxel
double rrtNBV::RrtTree::gain_rsvs(StateVec state, bool &objectGainFound)
{
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainObjOfInt = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    int numOfRearSideVoxels = 0;
    int numOfVisibleInterestVoxels = 0;
    int numOfInvisibleInterestVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                if (node == VoxelStatus::kUnknown)
                {
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainUnknown += 1;
                        numOfUnknownVisibleVoxels++;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        // Since it is visible then for sure it has either a rear side (free or unknown)
                        // it should be used after knowning that the occupied voxel belongs to obj of interest.
                        gainOccupied += 0;
                        numOfOccupiedVisibleVoxels++;

                        //double s_gain = manager_->getCellIneterestGain(vec);
                        //if(s_gain == 0.5)
                        int voxelType = manager_->getCellIneterestCellType(vec[0], vec[1], vec[2]);
                        if (voxelType == 2)
                        {
                            numOfVisibleInterestVoxels++;
                            bool rearSideVoxel = this->manager_->getRearSideVoxel(origin, vec);
                            std::cout << " THE RESUTLS IS ** " << rearSideVoxel << std::endl;
                            if (rearSideVoxel)
                            {
                                gainObjOfInt += 1;
                                numOfRearSideVoxels++;
                            }
                        }
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainFree += 0;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Visibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;
    //std::cout << " number Of Rear Side Voxel In One View is " << gainObjOfInt << std::endl << std::flush ; ;
    std::cout << " number Of Visible Interest Voxels In One View is " << numOfVisibleInterestVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Visible  Rear Side Interest Voxels In One View is "
              << numOfRearSideVoxels << std::endl
              << std::flush;
    ;

    if (gainObjOfInt > 0)
    {
        gain = gainObjOfInt;
        objectGainFound = true;
        std::cout << "Object Gain " << gain << std::endl;
    }
    else
    {
        gain = gainUnknown;
        std::cout << "Volumetric Gain " << gain << std::endl;
    }

    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    // Scale with volume
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f", gain);
    return gain;
}

// rear side entropy. ref[1] "A comparison of a volumetric information gain metrics for active 3D object reconstruction"
double rrtNBV::RrtTree::gain_rse(StateVec state, bool &objectGainFound)
{
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainObjOfInt = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    int numOfRearSideVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                if (node == VoxelStatus::kUnknown)
                {
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainUnknown += 1;
                        numOfUnknownVisibleVoxels++;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        // Since it is visible then for sure it has either a rear side (free or unknown)
                        // it should be used after knowning that the occupied voxel belongs to obj of interest.
                        gainOccupied += 0;
                        numOfOccupiedVisibleVoxels++;

                        bool rearSideVoxel = this->manager_->getRearSideVoxel(origin, vec);

                        ROS_INFO("The result is %d ", rearSideVoxel);  // " THE RESUTLS IS ** " << rearSideVoxel << std::endl  ;
                        if (rearSideVoxel)
                        {
                            double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                            double entropy = -probability * std::log(probability) -
                                    ((1 - probability) * std::log(1 - probability));
                            double Iv = Pv * entropy;
                            std::cout << "**************** Rear Side Entropy *********************"
                                      << std::endl;
                            //double rearSideEntropy = this->manager_->getRearSideEntropy(origin, vec);
                            gainObjOfInt = +Iv;
                            numOfRearSideVoxels++;
                        }
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainFree += 0;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Invisibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;
    //std::cout << " number Of Rear Side Voxel In One View is " << gainObjOfInt << std::endl << std::flush ; ;
    std::cout << " number Of Rear Side Interest Voxels In One View is " << numOfRearSideVoxels
              << std::endl
              << std::flush;
    ;

    if (gainObjOfInt > 0)
    {
        gain = gainObjOfInt;
        objectGainFound = true;
        std::cout << "Object Gain " << gain << std::endl;
    }
    else
    {
        gain = gainUnknown;
        std::cout << "Volumetric Gain " << gain << std::endl;
    }

    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    // Scale with volume
    // TODO: Remove the scaling from the entropy functions
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// proposed semantic rear side entropy
double rrtNBV::RrtTree::gain_rses(StateVec state, bool &objectGainFound)
{
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainObjOfInt = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    int numOfRearSideVoxels = 0;
    int numOfVisibleInterestVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                if (node == VoxelStatus::kUnknown)
                {
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainUnknown += 1;
                        numOfUnknownVisibleVoxels++;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        // Since it is visible then for sure it has either a rear side (free or unknown)
                        // it should be used after knowning that the occupied voxel belongs to obj of interest.
                        gainOccupied += 0;
                        numOfOccupiedVisibleVoxels++;
                        double s_gain = manager_->getCellIneterestGain(vec);
                        if (s_gain == 0.5)
                        {
                            numOfVisibleInterestVoxels++;
                            bool rearSideVoxel = this->manager_->getRearSideVoxel(origin, vec);
                            ROS_INFO(
                                        "The result is %d ",
                                        rearSideVoxel);  // " THE RESUTLS IS ** " << rearSideVoxel << std::endl  ;
                            if (rearSideVoxel)
                            {
                                double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                                double entropy = -probability * std::log(probability) -
                                        ((1 - probability) * std::log(1 - probability));
                                double Iv = Pv * entropy;
                                std::cout
                                        << "**************** Rear Side Entropy *********************"
                                        << std::endl;
                                //double rearSideEntropy = this->manager_->getRearSideEntropy(origin, vec);
                                gainObjOfInt = +Iv;
                                numOfRearSideVoxels++;
                            }
                        }
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainFree += 0;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Invisibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;
    //std::cout << " number Of Rear Side Voxel In One View is " << gainObjOfInt << std::endl << std::flush ; ;
    std::cout << " number Of Visible Interest Voxels In One View is " << numOfVisibleInterestVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Rear Side Interest Voxels In One View is " << numOfRearSideVoxels
              << std::endl
              << std::flush;
    ;

    if (gainObjOfInt > 0)
    {
        gain = gainObjOfInt;
        objectGainFound = true;
        std::cout << "Object Gain " << gain << std::endl;
    }
    else
    {
        gain = gainUnknown;
        std::cout << "Volumetric Gain " << gain << std::endl;
    }

    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    // Scale with volume
    // TODO: Remove the scaling from the entropy functions
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// Occlusion aware. Ref[1] "A comparison of a volumetric information gain metrics for active 3D object reconstruction"
double rrtNBV::RrtTree::gain_occlusion_aware(StateVec state, bool &objectGainFound)
{
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainOcclusionAware = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                probability = 0.5;  // becasue it is unknown/ ummapped voxel ;

                if (node == VoxelStatus::kUnknown)
                {
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                        double entropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        double Iv = Pv * entropy;
                        gainOcclusionAware += Iv;
                        numOfUnknownVisibleVoxels++;
                        gainUnknown += gainUnknown + 1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                        double entropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        double Iv = Pv * entropy;
                        gainOcclusionAware += Iv;
                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                        double entropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        double Iv = Pv * entropy;
                        gainOcclusionAware += Iv;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    /*std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
    //         << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Invisibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;*/
     

    gain = gainOcclusionAware;
    //int traversedVoxels =
     //       numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels;
    //gain = gain / traversedVoxels ; 

    objectGainFound = false;
    //std::cout << "gain before scaling " << gain << std::endl << std::flush;
    // Scale with volume
    gain *= pow(disc, 3.0);
    //std::cout << "gain after scaling " << gain << std::endl << std::flush;
    ROS_INFO("Occlusion aware gain %f ", gain);
    return gain;
}

//// Occlusion aware. Ref[1] "A comparison of a volumetric information gain metrics for active 3D object reconstruction"
//double rrtNBV::RrtTree::gain_occlusion_aware(StateVec state, bool & objectGainFound)
//{
//    // gain variables
//    double gain = 0.0;
//    double gainOcclusionAware = 0.0;

//    const double disc = manager_->getResolution();
//    std::cout << "RESOLUTION " << disc <<std::endl << std::flush ;
//    Eigen::Vector3d origin(state[0], state[1], state[2]);
//    Eigen::Vector3d vec;
//    double rangeSq = pow(params_.gainRange_, 2.0);
//    int i = 0 ;
//    // Iterate over all nodes within the allowed distance
//    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
//         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc) {
//        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
//             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc) {
//            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
//                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc) {

//                Eigen::Vector3d dir = vec - origin;
//                // Skip if distance is too large
//                if (dir.transpose().dot(dir) > rangeSq) {
//                    continue;
//                }
//                bool insideAFieldOfView = false;
//                // Check that voxel center is inside one of the fields of view.
//                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_
//                     .camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
//                    bool inThisFieldOfView = true;
//                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
//                         itSingleCBN != itCBN->end(); itSingleCBN++) {
//                        Eigen::Vector3d normal = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ())
//                                * (*itSingleCBN);
//                        double val = dir.dot(normal.normalized());

//                        if (val < SQRT2 * disc) {
//                            inThisFieldOfView = false;
//                            break;
//                        }
//                    }

//                    if (inThisFieldOfView) {
//                        insideAFieldOfView = true;
//                        break;
//                    }
//                }
//                if (!insideAFieldOfView) {
//                    continue;
//                }

//                // Check cell status and add to the gain considering the corresponding factor.
//                double probability;
//                VoxelStatus node = manager_->getCellProbabilityPoint(
//                            vec, &probability);
//                if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false)) {
//                    double Pv = this->manager_->getVisibilityLikelihood(origin, vec) ;
//                    double  entropy= -probability * std::log(probability) - ((1-probability) * std::log(1-probability));
//                    double Iv = Pv * entropy ;
//                    gainOcclusionAware+= Iv ;
//                }

//            }
//        }
//    }

//    gain = gainOcclusionAware  ;
//    objectGainFound = true ;
//    std::cout << "Occlusion Aware Gain " << gain << std::endl ;

//    std::cout<<"gain before scaling " << gain  << std::endl << std::flush ;
//    // Scale with volume
//    // TODO: Remove the scaling from the entropy functions
//    gain *= pow(disc, 3.0);
//    std::cout<<"gain after scaling " << gain  << std::endl << std::flush ;

//    // Check the gain added by inspectable surface
//    if (mesh_) {
//        // ROS_INFO("****gain added by inspectable surface*****");
//        tf::Transform transform;
//        transform.setOrigin(tf::Vector3(state.x(), state.y(), state.z()));
//        tf::Quaternion quaternion;
//        quaternion.setEuler(0.0, 0.0, state[3]);
//        transform.setRotation(quaternion);
//        gain += params_.igArea_ * mesh_->computeInspectableArea(transform);
//    }
//    ROS_INFO("GAIN %f ",gain);
//    return gain;
//}

// Unobserved Voxel. Ref[1] "A comparison of a volumetric information gain metrics for active 3D object reconstruction"
double rrtNBV::RrtTree::gain_unobserved_voxel(StateVec state, bool &objectGainFound)
{
    ROS_INFO("UNOBSERVED VOXEL ");
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainUnobservedVoxel = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);

                if (node == VoxelStatus::kUnknown)
                {
                    probability = 0.5;  // becasue it is unknown/ ummapped voxel ;
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                        double entropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        double Iv = Pv * entropy;
                        gainUnobservedVoxel += Iv;
                        numOfUnknownVisibleVoxels++;
                        gainUnknown += gainUnknown + 1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainUnobservedVoxel += 0;
                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        gainUnobservedVoxel += 0;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Invisibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;

    //if (gainUnobservedVoxel > 0 )
    //{
    gain = gainUnobservedVoxel;
    objectGainFound = false;
    std::cout << "unobserved voxel Gain " << gain << std::endl;
    // }
    // else
    // {
    //    gain = gainUnknown ;
    //     std::cout << "Volumetric Gain " << gain << std::endl;
    // }
    //    gain = gainUnobservedVoxel  ;
    //    objectGainFound = true ;
    //    std::cout << "unobserved voxel Gain " << gain << std::endl ;

    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    // Scale with volume
    // TODO: Remove the scaling from the entropy functions
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// Pure Entropy Ref[1] "A comparison of a volumetric information gain metrics for active 3D object reconstruction"
double rrtNBV::RrtTree::gain_pure_entropy(StateVec state, bool &objectGainFound)
{
    objectGainFound = false;
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainUnobservedVoxel = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability = 1;
                double voxelEntropy = 0;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                if (node == VoxelStatus::kUnknown)
                {
                    probability = 0.5;  // becasue it is unknown/ummapped voxel ;
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfUnknownVisibleVoxels++;
                        voxelEntropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        gain += voxelEntropy;
                        gainUnknown += +1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        voxelEntropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        gain += voxelEntropy;
                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        voxelEntropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        gain += voxelEntropy;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    std::cout << " number Of Voxels In One View is "
              << numOfFreeVoxels + numOfOccupiedVoxels + numOfUnknownVoxels << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Invisibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;

    // Scale with volume
    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// Average Entropy Ref[1] "A comparison of a volumetric information gain metrics for active 3D object reconstruction"
double rrtNBV::RrtTree::gain_avg_entropy(StateVec state, bool &objectGainFound)
{
    ROS_INFO("Average Entropy");
    objectGainFound = false;
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainUnobservedVoxel = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                // Check cell status and add to the gain considering the corresponding factor.
                double probability = 1;
                double voxelEntropy = 0;
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                if (node == VoxelStatus::kUnknown)
                {
                    probability = 0.5;  // becasue it is unknown/ummapped voxel ;
                    numOfUnknownVoxels++;

                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        voxelEntropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        gain += voxelEntropy;
                        numOfUnknownVisibleVoxels++;
                        gainUnknown += +1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        voxelEntropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        gain += voxelEntropy;
                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        voxelEntropy = -probability * std::log(probability) -
                                ((1 - probability) * std::log(1 - probability));
                        gain += voxelEntropy;
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    int traversedVoxels =
            numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels;
    gain = gain / traversedVoxels;
    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
              << std::endl
              << std::flush;
    std::cout << " number Of Voxels In One View is "
              << numOfFreeVoxels + numOfOccupiedVoxels + numOfUnknownVoxels << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted visible Voxels In One View is "
              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
              << std::endl
              << std::flush;
    ;
    std::cout << " number Of Accepted Invisibal Voxels In One View is "
              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
                 numOfUnknownInvisibleVoxels
              << std::endl
              << std::flush;
    ;

    // Scale with volume
    std::cout << "gain before scaling " << gain << std::endl << std::flush;
    gain *= pow(disc, 3.0);
    std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// Semantic Visible Voxels Proposed (counting number of obj of interest - not sufficiently visited voxels)
double rrtNBV::RrtTree::gain_svv(StateVec state, bool &objectGainFound)
{
    ROS_INFO("SEMANTIC VISIBLE VOXEL");

    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainObjOfInt = 0.0;
    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                double probability = 1;
                //double voxelEntropy = 0 ;
                // Check cell status and add to the gain considering the corresponding factor.
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                // Unknown
                if (node == VoxelStatus::kUnknown)
                {
                    probability = 0.5;  // becasue it is unknown/ummapped voxel ;
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfUnknownVisibleVoxels++;
                        gainUnknown += +1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied) // Here we check for semantic voxels
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        timer.start("[RrtTree]getCellIneterestGain") ; 
                        double semantic_gain = manager_->getCellIneterestGain(vec);
                        timer.stop("[RrrtTree]getCellIneterestGain") ; 

                        // for debugging
                        if (semantic_gain == 1)
                        {
                            //ROS_INFO("OBJECT OF INTEREST FOUND");
                            gainObjOfInt++;
                        }
                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    int traversedVoxels =
            numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels;

    //    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
    //              << std::endl
    //              << std::flush;
    //    std::cout << " number Of Voxels In One View is "
    //              << numOfFreeVoxels + numOfOccupiedVoxels + numOfUnknownVoxels << std::endl
    //              << std::flush;
    //    std::cout << " number Of Accepted visible Voxels In One View is "
    //              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
    //              << std::endl
    //              << std::flush;
    //    std::cout << " number Of Accepted Invisibal Voxels In One View is "
    //              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels + numOfUnknownInvisibleVoxels
    //              << std::endl
    //              << std::flush;

    gain =(alphaGain*gainUnknown) + (betaGain *gainObjOfInt);
    gain = gain / traversedVoxels;

    // Scale with volume
    //std::cout << "gain before scaling " << gain << std::endl << std::flush;
    gain *= pow(disc, 3.0);
    //std::cout << "gain after scaling " << gain << std::endl << std::flush;
    std::cout << "Volumetric Gain" <<  (alphaGain*gainUnknown) << "  Semantic Gain " << betaGain *gainObjOfInt << std::endl << std::flush;
    ROS_INFO("GAIN %f ", gain);
    return gain;
}

// Semantic Visible Voxels Proposed (counting number of obj of interest - not sufficiently visited voxels)
double rrtNBV::RrtTree::gain_semantic_occlusion_aware(StateVec state, bool &objectGainFound)
{
    ROS_INFO("SEMANTIC VISIBLE VOXEL");
    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainObjOfInt = 0.0;

    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                double probability = 1;
                //double voxelEntropy = 0 ;
                // Check cell status and add to the gain considering the corresponding factor.
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                // Unknown
                if (node == VoxelStatus::kUnknown)
                {
                    probability = 0.5;  // becasue it is unknown/ummapped voxel ;
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfUnknownVisibleVoxels++;
                        gainUnknown += +1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied)
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {

                        double s_gain = manager_->getCellIneterestGain(vec);
                        //int s_gain = manager_->getCellNumOfVisits(vec);

                        if (s_gain == 1)
                        {
                            double Pv = this->manager_->getVisibilityLikelihood(origin, vec);
                            double entropy = -probability * std::log(probability) -
                                    ((1 - probability) * std::log(1 - probability));
                            double Iv = Pv * entropy;
                            gainObjOfInt += Iv;
                            //ROS_ERROR("OBJECT OF INTEREST FOUND");
                        }

                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }
    int traversedVoxels =
            numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels;
    gain = gain / traversedVoxels;
    //    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
    //              << std::endl
    //              << std::flush;
    //    std::cout << " number Of Voxels In One View is "
    //              << numOfFreeVoxels + numOfOccupiedVoxels + numOfUnknownVoxels << std::endl
    //              << std::flush;
    //    ;
    //    std::cout << " number Of Accepted visible Voxels In One View is "
    //              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
    //              << std::endl
    //              << std::flush;
    //    ;
    //    std::cout << " number Of Accepted Invisibal Voxels In One View is "
    //              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels +
    //                     numOfUnknownInvisibleVoxels
    //              << std::endl
    //              << std::flush;
    //    ;

    if (gainObjOfInt > 0)
    {
        gain = gainObjOfInt;
        objectGainFound = true;
        std::cout << "Object Gain Found" << gain << std::endl << std::flush;
    }
    else
    {
        gain = gainUnknown;
        std::cout << "Volumetric Gain " << gain << std::endl << std::flush;
    }

    // Scale with volume
    //std::cout << "gain before scaling " << gain << std::endl << std::flush;
    //gain *= pow(disc, 3.0);
    //std::cout << "gain after scaling " << gain << std::endl << std::flush;

    ROS_INFO("Gain %f ", gain);
    return gain;
}

// Semantic Visible Voxels Proposed (counting number of obj of interest - not sufficiently visited voxels)
double rrtNBV::RrtTree::gain_semantic_obj_interest_num_visits(StateVec state, bool &objectGainFound)
{
    //ROS_INFO("SEMANTIC VISIBLE VOXEL OBJ of INTEREST VISITS ");

    // gain variables
    double gain = 0.0;
    double gainUnknown = 0.0;
    double gainFree = 0.0;
    double gainOccupied = 0.0;
    double gainObjOfInt = 0.0;
    // counting variables
    int numberOfAcceptedVoxelInOneView = 0;
    int numOfFreeVoxels = 0;
    int numOfFreeVisibleVoxels = 0;
    int numOfFreeInvisibleVoxels = 0;

    int numOfOccupiedVoxels = 0;
    int numOfOccupiedVisibleVoxels = 0;
    int numOfOccupiedInvisibleVoxels = 0;

    int numOfUnknownVoxels = 0;
    int numOfUnknownVisibleVoxels = 0;
    int numOfUnknownInvisibleVoxels = 0;

    const double disc = manager_->getResolution();
    std::cout << "RESOLUTION " << disc << std::endl << std::flush;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    int i = 0;
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc)
    {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc)
        {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc)
            {
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN =
                     params_.camBoundNormals_.begin();
                     itCBN != params_.camBoundNormals_.end(); itCBN++)
                {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                         itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++)
                    {
                        Eigen::Vector3d normal =
                                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
                        double val = dir.dot(normal.normalized());

                        if (val < SQRT2 * disc)
                        {
                            inThisFieldOfView = false;
                            break;
                        }
                    }

                    if (inThisFieldOfView)
                    {
                        insideAFieldOfView = true;
                        break;
                    }
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }

                double probability = 1;

                // Check cell status and add to the gain considering the corresponding factor.
                VoxelStatus node = manager_->getCellProbabilityPoint(vec, &probability);
                // Unknown
                if (node == VoxelStatus::kUnknown)
                {
                    probability = 0.5;  // becasue it is unknown/ummapped voxel ;
                    numOfUnknownVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfUnknownVisibleVoxels++;
                        gainUnknown += +1;
                    }
                    else
                        numOfUnknownInvisibleVoxels++;
                }

                else if (node == VoxelStatus::kOccupied) // Here we check for semantic voxels
                {
                    numOfOccupiedVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        int semantic_gain = manager_->getCellNumOfVisits(vec);
                        // for debugging
                        if (semantic_gain == 1)
                        {
                            //ROS_INFO("OBJECT OF INTEREST FOUND");
                            gainObjOfInt++;
                        }
                        numOfOccupiedVisibleVoxels++;
                    }
                    else
                        numOfOccupiedInvisibleVoxels++;
                }
                else
                {
                    numOfFreeVoxels++;
                    // Rayshooting to evaluate inspectability of cell
                    if (VoxelStatus::kOccupied != this->manager_->getVisibility(origin, vec, false))
                    {
                        numOfFreeVisibleVoxels++;
                    }
                    else
                        numOfFreeInvisibleVoxels++;
                }

                numberOfAcceptedVoxelInOneView++;
            }
        }
    }

    int traversedVoxels =
            numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels;



    //    std::cout << " number Of Accepted Voxels In One View is " << numberOfAcceptedVoxelInOneView
    //              << std::endl
    //              << std::flush;
    //    std::cout << " number Of Voxels In One View is "
    //              << numOfFreeVoxels + numOfOccupiedVoxels + numOfUnknownVoxels << std::endl
    //              << std::flush;
    //    std::cout << " number Of Accepted visible Voxels In One View is "
    //              << numOfFreeVisibleVoxels + numOfOccupiedVisibleVoxels + numOfUnknownVisibleVoxels
    //              << std::endl
    //              << std::flush;
    //    std::cout << " number Of Accepted Invisibal Voxels In One View is "
    //              << numOfFreeInvisibleVoxels + numOfOccupiedInvisibleVoxels + numOfUnknownInvisibleVoxels
    //              << std::endl
    //              << std::flush;

    gain =(alphaGain*gainUnknown) + (betaGain *gainObjOfInt);
    gain = gain / traversedVoxels;
    
    ROS_INFO("Volumetric Gain %f Semantic Gain %f ", alphaGain*gainUnknown ,betaGain *gainObjOfInt ) ;
    // Scale with volume
    //std::cout << "gain before scaling " << gain << std::endl << std::flush;
    gain *= pow(disc, 3.0);
    //std::cout << "gain after scaling " << gain << std::endl << std::flush;
    ROS_INFO("object visited gain %f ", gain);
    return gain;
}


std::vector<geometry_msgs::Pose> rrtNBV::RrtTree::getPathBackToPrevious(std::string targetFrame)
{
    std::vector<geometry_msgs::Pose> ret;
    if (history_.empty())
    {
        return ret;
    }

    ret = samplePath(root_, history_.top(), targetFrame);
    history_.pop();
    return ret;
}

void rrtNBV::RrtTree::memorizeBestBranch()
{
    //if (!oneViewObjectFound)
    //{
        bestBranchMemory_.clear();
        Node *current = bestNode_;
        while (current->parent_ && current->parent_->parent_)
        {
            bestBranchMemory_.push_back(current->state_);
            current = current->parent_;
        }
    /*}
    else
    {
        bestBranchMemory_.clear();
        Node *current = bestObjectNode_;
        while (current->parent_ && current->parent_->parent_)
        {
            bestBranchMemory_.push_back(current->state_);
            current = current->parent_;
        }
    }*/
}

void rrtNBV::RrtTree::clear()
{
    oneViewObjectFound = false;
    delete rootNode_;
    rootNode_ = nullptr;

    counter_ = 0;
    bestGain_ = params_.zero_gain_;
    bestObjectGain_ = params_.zero_gain_;

    bestNode_ = nullptr;
    bestObjectNode_ = nullptr;

    kd_free(kdTree_);
}

void rrtNBV::RrtTree::publishNode(Node *node)
{
    //visualization_msgs::Marker::DELETEALL
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_.navigationFrame_;
    p.id = g_ID_;
    g_ID_++;
    p.ns = "vp_tree";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = std::max(node->gain_ / 20.0, 0.05);
    p.scale.y = 0.1;
    p.scale.z = 0.1;
    p.color.r = 167.0 / 255.0;;
    p.color.g = 167.0 / 255.0;;
    p.color.b = 0.0;  // blue
    p.color.a = 1.0;
    p.lifetime = ros::Duration(100);
    p.frame_locked = false;
    params_.inspectionPath_.publish(p);

    if (!node->parent_)
    {
        return;
    }

    p.id = g_ID_;
    g_ID_++;
    p.ns = "vp_branches";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->parent_->state_[0];
    p.pose.position.y = node->parent_->state_[1];
    p.pose.position.z = node->parent_->state_[2];
    Eigen::Quaternion<float> q;
    Eigen::Vector3f init(1.0, 0.0, 0.0);
    Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
            node->state_[1] - node->parent_->state_[1],
            node->state_[2] - node->parent_->state_[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.scale.x = dir.norm();
    p.scale.y = 0.03;
    p.scale.z = 0.03;
    p.color.r = 1.0;// 100.0 / 255.0;;  //red
    p.color.g = 100.0 / 255.0;;
    p.color.b = 0.7;
    p.color.a = 1.0;
    p.lifetime = ros::Duration(100);
    p.frame_locked = false;
    params_.inspectionPath_.publish(p);

    if (params_.log_)
    {
        for (int i = 0; i < node->state_.size(); i++)
        {
            fileTree_ << node->state_[i] << ",";
        }
        fileTree_ << node->gain_ << ",";
        for (int i = 0; i < node->parent_->state_.size(); i++)
        {
            fileTree_ << node->parent_->state_[i] << ",";
        }
        fileTree_ << node->parent_->gain_ << "\n";
    }
}

void rrtNBV::RrtTree::publishDebugNode(StateVec node , int Nodetype)
{
    // 0 - Random Point blue 
    // 1 - Parent Green 
    // 2 - candidate to be added 
    // 3 - Direction 
    visualization_msgs::Marker marker;
    marker.header.frame_id = params_.worldFrameId_;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = params_.marker_id ;
    params_.marker_id++ ; 
    marker.pose.position.x = node[0];
    marker.pose.position.y = node[1];
    marker.pose.position.z = node[2]; 
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node[3]);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 

    if (Nodetype == 0 )
    {
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    else if (Nodetype == 1)
    {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else if (Nodetype == 2)
    {
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else if (Nodetype == 3)
    {
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.scale.x = 0.2;
    }
    else 
    {
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    }
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(30); //tried 0, and 10,0
    params_.sample_viewpoint_pub_.publish(marker) ; 
}

void rrtNBV::RrtTree::publishDebugGain(StateVec node , double gain, int type)
{

    // Draw the gain number on the branch
    params_.gainMarker.header.stamp = ros::Time::now();
    params_.gainMarker.header.seq = g_ID_;
    params_.gainMarker.header.frame_id = params_.navigationFrame_;
    params_.gainMarker.id = g_ID_;
    g_ID_++;
    params_.gainMarker.ns = "vp_tree";
    params_.gainMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    params_.gainMarker.action = visualization_msgs::Marker::ADD;
    params_.gainMarker.pose.position.x = node[0] ;
    params_.gainMarker.pose.position.y = node[1];
    params_.gainMarker.pose.position.z = node[2]- 0.1;
    params_.gainMarker.scale.x = 0.1;
    params_.gainMarker.scale.y = 0.1;
    params_.gainMarker.scale.z = 0.1;
    params_.gainMarker.color.r = 0; // red
    params_.gainMarker.color.g = 0;
    params_.gainMarker.color.b = 0;
    params_.gainMarker.color.a = 1.0;
    if (type == 1)
        params_.gainMarker.color.b = 1;
    if (type == 2 )
        params_.gainMarker.color.r = 1;

    params_.gainMarker.color.a = 1.0;
    params_.gainMarker.lifetime = ros::Duration(30);
    string s = std::to_string(gain) ;
    //std::cout << "##############" ;
    //std::cout << s << std::endl ;
    params_.gainMarker.text= s;
    params_.gain_pub_.publish(params_.gainMarker);
    // **********************************************
}
void rrtNBV::RrtTree::publishDebugStatus(StateVec node , int status)
{
    // Draw the status number on the branch
    // 1 - unknown 
    // 2 - free
    params_.gainMarker.header.stamp = ros::Time::now();
    params_.gainMarker.header.seq = g_ID_;
    params_.gainMarker.header.frame_id = params_.navigationFrame_;
    params_.gainMarker.id = g_ID_;
    g_ID_++;
    params_.gainMarker.ns = "vp_tree";
    params_.gainMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    params_.gainMarker.action = visualization_msgs::Marker::ADD;
    params_.gainMarker.pose.position.x = node[0];
    params_.gainMarker.pose.position.y = node[1];
    params_.gainMarker.pose.position.z = node[2] + 0.1;
    params_.gainMarker.scale.x = 0.1;
    params_.gainMarker.scale.y = 0.1;
    params_.gainMarker.scale.z = 0.1;
    params_.gainMarker.color.r = 1; // red
    params_.gainMarker.color.g = 1;
    params_.gainMarker.color.b = 1;
    params_.gainMarker.color.a = 1.0;
    params_.gainMarker.lifetime = ros::Duration(30);
    string s = std::to_string(status) ;
    //std::cout << "##############" ;
    //std::cout << s << std::endl ;
    params_.gainMarker.text= s;
    params_.gain_pub_.publish(params_.gainMarker);
    // **********************************************
}
std::vector<geometry_msgs::Pose> rrtNBV::RrtTree::samplePath(StateVec start, StateVec end,
                                                             std::string targetFrame)
{
    std::vector<geometry_msgs::Pose> ret;
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return ret;
    }
    Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
    double yaw_direction = end[3] - start[3];
    if (yaw_direction > M_PI)
    {
        yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI)
    {
        yaw_direction += 2.0 * M_PI;
    }
    double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                           params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
    std::cout << "STEP SIZE" << disc << std::endl << std::flush;
    assert(disc > 0.0);

    for (double it = 0.0; it <= 1.0; it += disc)
    {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) * start[1] + it * end[1],
                (1.0 - it) * start[2] + it * end[2]);
        double yaw = start[3] + yaw_direction * it;
        if (yaw > M_PI)
            yaw -= 2.0 * M_PI;
        if (yaw < -M_PI)
            yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        origin = transform * origin;
        quat = transform * quat;
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        ret.push_back(pose);
        // save only the current position
        if (params_.log_ && it == 0.0)
        {
            filePath_ << poseTF.getOrigin().x() << ",";
            filePath_ << poseTF.getOrigin().y() << ",";
            filePath_ << poseTF.getOrigin().z() << ",";
            filePath_ << tf::getYaw(poseTF.getRotation()) << "\n";
        }
    }
    return ret;
}
#endif
