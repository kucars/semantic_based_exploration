/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INFORMATIONGAINPOINTDENSITY_HPP_
#define INFORMATIONGAINPOINTDENSITY_HPP_
#include <thread>
#include <chrono>
#include <cstdlib>
#include <rrt_explorer/information_gain_base.h>
#include <rrt_explorer/information_gain_classic.h>
#include <rrt_explorer/information_gain_point_density.h>

#include <rrt_explorer/rrt_tree.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <fstream>
/* sqrt example */
#include <stdio.h>      /* printf */
#include <math.h>       /* sqrt */
#include <tf/transform_datatypes.h>
using namespace std;

rrtNBV::InformationGainPointDensity::InformationGainPointDensity()
    : rrtNBV::InformationGainBase::InformationGainBase()
{}

rrtNBV::InformationGainPointDensity::InformationGainPointDensity(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager)
{
    mesh_ = mesh;
    manager_ = manager;
    kdTree_ = kd_create(3);
    iterationCount_ = 0;
    for (int i = 0; i < 4; i++)
    {
        inspectionThrottleTime_.push_back(ros::Time::now().toSec());
    }
    // If logging is required, set up files here
    bool ifLog = false;
    std::string ns = ros::this_node::getName();
    ros::param::get(ns + "/nbvp/log/on", ifLog);
    if (ifLog) {
        time_t rawtime;
        struct tm * ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
        logFilePath_ = ros::package::getPath("rrt_explorer") + "/data/"
                + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
                + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
                + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
        system(("mkdir -p " + logFilePath_).c_str());
        logFilePath_ += "/";
        fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
        filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
    }
    ROS_INFO("****************************** Density ********************************************");
}

rrtNBV::InformationGainPointDensity::~InformationGainPointDensity()
{
    delete rootNode_;
    kd_free(kdTree_);
    if (fileResponse_.is_open()) {
        fileResponse_.close();
    }
    if (fileTree_.is_open()) {
        fileTree_.close();
    }
    if (filePath_.is_open()) {
        filePath_.close();
    }
}

void rrtNBV::InformationGainPointDensity::initialize()
{
    // ROS_WARN("EXPLORATION AREA");
    // Publish visualization of total exploration area
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
    p.color.r = 200.0 / 255.0;
    p.color.g = 100.0 / 255.0;
    p.color.b = 0.0;
    p.color.a = 0.1;
    p.lifetime = ros::Duration(0.0);
    p.frame_locked = false;
    params_.inspectionPath_.publish(p);

    // Create The Root Node with is the current location of the robot
    // root_ variable is assigned from the position callbak function
    rootNode_ = new Node;
    rootNode_->distance_ = 0.0;
    rootNode_->gain_ = params_.zero_gain_; // ?? is it information gain the global variable or zero_gain
    rootNode_->parent_ = NULL;
    rootNode_->state_ = root_;

    // create tree and push the root node to it as the first node
    kdTree_ = kd_create(3);
    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(),rootNode_);
    if (params_.log_) {
        if (fileTree_.is_open()) {
            fileTree_.close();
        }
        fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(),
                       std::ios::out);
    }
    iterationCount_++;

    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp       = ros::Time::now();
    poseMsg.header.frame_id    = params_.navigationFrame_;
    poseMsg.pose.position.x    = rootNode_->state_.x();
    poseMsg.pose.position.y    = rootNode_->state_.y();
    poseMsg.pose.position.z    = rootNode_->state_.z();
    // The orientation does not really matter
    tf::Quaternion quat2;
    quat2.setEuler(0.0, 0.0, root_[3]);
    poseMsg.pose.orientation.x = quat2.x();
    poseMsg.pose.orientation.y = quat2.y();
    poseMsg.pose.orientation.z = quat2.z();
    poseMsg.pose.orientation.w = quat2.w();
    params_.rootNodeDebug.publish(poseMsg);
}

double rrtNBV::InformationGainPointDensity::gain(StateVec state )
{
    double density = 0 ;
    double numOfOccupiedCells = 0.0 ;
    double numOfCells = 0.0 ;
    visualization_msgs::Marker p;
    // This function computes the gain
    double gain = 0.0;
    const double disc = manager_->getResolution();
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d vec;
    double rangeSq = pow(params_.gainRange_, 2.0);
    // Iterate over all nodes within the allowed distance
    for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
         vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc) {
        for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
             vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc) {
            for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
                 vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc) {
                numOfCells++;
                Eigen::Vector3d dir = vec - origin;
                // Skip if distance is too large
                if (dir.transpose().dot(dir) > rangeSq) {
                    continue;
                }
                bool insideAFieldOfView = false;
                // Check that voxel center is inside one of the fields of view.
                for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_
                     .camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
                    bool inThisFieldOfView = true;
                    for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
                         itSingleCBN != itCBN->end(); itSingleCBN++) {
                        Eigen::Vector3d normal = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ())
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
                // Check cell status and add to the gain considering the corresponding factor.
                double probability;
                volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
                            vec, &probability);

                double entropy , p ;
                if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown )
                    p = 0.5 ;
                else
                    p = probability ;


                entropy= -p * std::log(p) - ((1-p) * std::log(1-p));
                gain += entropy;

                //ROS_INFO("p IS:%f",p);
                if(p>=0.8)
                {
                    //std::cout << "It is Occupied" << std::endl ;
                    numOfOccupiedCells++ ;
                }

            }
        }
    }
    ROS_INFO("numOfCells IS:%f",numOfCells);
    ROS_INFO("numOfOccupiedCells IS:%f",numOfOccupiedCells);

    if (numOfOccupiedCells == 0 )
        return -1 ;

    density = numOfOccupiedCells/numOfCells ;
    double newGain = gain / density ;

    return newGain;

}

bool rrtNBV::InformationGainPointDensity::iterate(int iterations)
{
    double viewGain = 0 ;
    // In this function a new configuration is sampled and added to the tree.
    StateVec newState;
    // Sample over a sphere with the radius of the maximum diagonal of the exploration
    // space. Throw away samples outside the sampling region it exiting is not allowed
    // by the corresponding parameter. This method is to not bias the tree towards the
    // center of the exploration space.
    double radius = sqrt(
                SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
                + SQ(params_.minZ_ - params_.maxZ_));

    bool solutionFound = false;
    while (!solutionFound)
    {
        for (int i = 0; i < 3; i++)
        {
            newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        }
        if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
            continue;
        // Offset new state by root
        newState += rootNode_->state_;
        if (!params_.softBounds_) {
            if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
                continue;
            } else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
                continue;
            } else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
                continue;
            } else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
                continue;
            } else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
                continue;
            } else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
                continue;
            }
        }
        solutionFound = true;
    }
    //ROS_WARN("Sample Point genrated inside the exploration aera and not in collision with the bounding box -the bounding box is the robot dimensions");
    visualization_msgs::Marker samplePoint;
    tf::Quaternion quatSamplePoint;
    samplePoint.header.stamp = ros::Time::now();
    samplePoint.header.seq = iterations ;
    samplePoint.header.frame_id = params_.navigationFrame_;
    samplePoint.id = iterations ;
    samplePoint.ns = "workspace";
    samplePoint.type = visualization_msgs::Marker::CUBE;
    samplePoint.action = visualization_msgs::Marker::ADD;
    samplePoint.pose.position.x = newState.x();
    samplePoint.pose.position.y = newState.y();
    samplePoint.pose.position.z = newState.z();
    quatSamplePoint.setEuler(0.0, 0.0, 0.0);
    samplePoint.pose.orientation.x = quatSamplePoint.x();
    samplePoint.pose.orientation.y = quatSamplePoint.y();
    samplePoint.pose.orientation.z = quatSamplePoint.z();
    samplePoint.pose.orientation.w = quatSamplePoint.w();
    samplePoint.scale.x = 0.5;
    samplePoint.scale.y = 0.5;
    samplePoint.scale.z = 0.5;
    samplePoint.color.r = 1;
    samplePoint.color.g = 0;
    samplePoint.color.b = 0;
    samplePoint.color.a = 1;
    samplePoint.lifetime = ros::Duration(0.0);
    samplePoint.frame_locked = false;
    params_.sampledPoints_.publish(samplePoint);

    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0)
    {
        kd_res_free(nearest);
        return false;
    }
    rrtNBV::Node * newParent = (rrtNBV::Node*) kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Origin Point
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    // Check for collision of new connection plus some overshoot distance.
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
    // default extension range is = 1
    if (direction.norm() > params_.extensionRange_)
    {
        direction = params_.extensionRange_ * direction.normalized();
    }
    // New Random Sampled Point called NewState
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];
    //std::cout << "SAMPLE POSE "  << newState[0] << "  " << newState[1] << "  " << newState[2] << std::endl << std::flush ;
    // std::cout << "PARENT POSE "  << origin[0] << "  " << origin[1] << "  " << origin[2] << std::endl << std::flush ;
    Eigen::Vector3d  startPoint = origin ;
    Eigen::Vector3d  endPoint = direction + origin + direction.normalized() * params_.dOvershoot_ ;

    // This shows the nearset point from the current point to the direction of the sampled point
    samplePoint.header.stamp = ros::Time::now();
    samplePoint.header.seq = iterations+100 ;
    samplePoint.header.frame_id = params_.navigationFrame_;
    samplePoint.id = iterations+100 ;
    samplePoint.ns = "workspace";
    samplePoint.type = visualization_msgs::Marker::CUBE;
    samplePoint.action = visualization_msgs::Marker::ADD;
    samplePoint.pose.position.x = newState[0];
    samplePoint.pose.position.y = newState[1];
    samplePoint.pose.position.z = newState[2];
    tf::Quaternion quatSamplePointParentNew;
    quatSamplePointParentNew.setEuler(0.0, 0.0, 0.0);
    samplePoint.pose.orientation.x = quatSamplePointParentNew.x();
    samplePoint.pose.orientation.y = quatSamplePointParentNew.y();
    samplePoint.pose.orientation.z = quatSamplePointParentNew.z();
    samplePoint.pose.orientation.w = quatSamplePointParentNew.w();
    samplePoint.scale.x = 0.5;
    samplePoint.scale.y = 0.5;
    samplePoint.scale.z = 0.5;
    samplePoint.color.r = 0;
    samplePoint.color.g = 0;
    samplePoint.color.b = 1;
    samplePoint.color.a = 1;
    samplePoint.lifetime = ros::Duration(0.0);
    samplePoint.frame_locked = false;
    params_.sampledPoints_.publish(samplePoint);
    //sleep(1.0) ;
    // This shows the origin point
    samplePoint.header.stamp = ros::Time::now();
    samplePoint.header.seq = iterations+10 ;
    samplePoint.header.frame_id = params_.navigationFrame_;
    samplePoint.id = iterations+10 ;
    samplePoint.ns = "workspace";
    samplePoint.type = visualization_msgs::Marker::CYLINDER;
    samplePoint.action = visualization_msgs::Marker::ADD;
    samplePoint.pose.position.x = startPoint[0];
    samplePoint.pose.position.y = startPoint[1];
    samplePoint.pose.position.z = startPoint[2];
    tf::Quaternion quatSamplePointParent;
    quatSamplePointParent.setEuler(0.0, 0.0, 0.0);
    samplePoint.pose.orientation.x = quatSamplePointParent.x();
    samplePoint.pose.orientation.y = quatSamplePointParent.y();
    samplePoint.pose.orientation.z = quatSamplePointParent.z();
    samplePoint.pose.orientation.w = quatSamplePointParent.w();
    samplePoint.scale.x = 0.5;
    samplePoint.scale.y = 0.5;
    samplePoint.scale.z = 0.5;
    samplePoint.color.r = 0;
    samplePoint.color.g = 1;
    samplePoint.color.b = 0;
    samplePoint.color.a = 1;
    samplePoint.lifetime = ros::Duration(0.0);
    samplePoint.frame_locked = false;
    params_.sampledPoints_.publish(samplePoint);
    //sleep(1.0) ;
    //ROS_WARN("End Point");
    // This shows the nearset point from the current point to the direction of the sampled point
    samplePoint.header.stamp = ros::Time::now();
    samplePoint.header.seq = iterations+300 ;
    samplePoint.header.frame_id = params_.navigationFrame_;
    samplePoint.id = iterations+300 ;
    samplePoint.ns = "workspace";
    samplePoint.type = visualization_msgs::Marker::SPHERE;
    samplePoint.action = visualization_msgs::Marker::ADD;
    samplePoint.pose.position.x = endPoint[0];
    samplePoint.pose.position.y = endPoint[1];
    samplePoint.pose.position.z = endPoint[2];
    tf::Quaternion quatEndPoint;
    quatEndPoint.setEuler(0.0, 0.0, 0.0);
    samplePoint.pose.orientation.x = quatEndPoint.x();
    samplePoint.pose.orientation.y = quatEndPoint.y();
    samplePoint.pose.orientation.z = quatEndPoint.z();
    samplePoint.pose.orientation.w = quatEndPoint.w();
    samplePoint.scale.x = 0.5;
    samplePoint.scale.y = 0.5;
    samplePoint.scale.z = 0.5;
    samplePoint.color.r = 0;
    samplePoint.color.g = 0;
    samplePoint.color.b = 0;
    samplePoint.color.a = 1;
    samplePoint.lifetime = ros::Duration(0.0);
    samplePoint.frame_locked = false;
    params_.sampledPoints_.publish(samplePoint);
    //sleep(1.0) ;
    // This shows the bounding box
    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = 10000;
    p.header.frame_id = params_.navigationFrame_;
    p.id = 10000;
    p.ns = "workspace";
    p.type = visualization_msgs::Marker::CUBE;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = origin[0] ;
    p.pose.position.y = origin[1] ;
    p.pose.position.z = origin[2] ;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, 0.0);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = params_.boundingBox_[0] ;
    p.scale.y = params_.boundingBox_[1] ;
    p.scale.z = params_.boundingBox_[2] ;
    p.color.r = 200.0 / 255.0;
    p.color.g = 100.0 / 255.0;
    p.color.b = 50.0 / 225.0;
    p.color.a = 1;
    p.lifetime = ros::Duration(0.0);
    p.frame_locked = false;
    params_.inspectionPath_.publish(p);

    //ROS_INFO("Check if the path from the origin to the sampled point is free or not");
    volumetric_mapping::OctomapManager::CellStatus cellStatus;
    cellStatus = manager_->getLineStatusBoundingBox(startPoint, endPoint, params_.boundingBox_);
    rrtNBV::Node * newNode = new rrtNBV::Node;

    if (cellStatus == volumetric_mapping::OctomapManager::CellStatus::kFree)// || cellStatus == volumetric_mapping::OctomapManager::CellStatus::kUnknown)
    {
        // ROS_WARN("SAMPLED CELL IS FREE");
        // sample a random Orientation
        newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        // Create new node and insert into tree
        newNode->state_ = newState;
        newNode->parent_ = newParent;   // same as origin same as start point
        newNode->distance_ = newParent->distance_ +  direction.norm();;
        //ROS_INFO (" direction.norm()%f\n", direction.norm());
        //ROS_INFO ("newParent->distance_%f\n", newParent->distance_) ;

        // ROS_INFO("Calculate the View IG by summing all the entropies for all the cells in the view");
        //viewGain = gainDeep(newNode->state_);
        //newNode->gain_ = newParent->gain_ + gainPureEntropy(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);
        double g =  gain(newNode->state_) ;

        if (g == -1 )
        {
            newNode->gain_  = 0 ;
            ROS_INFO("Occupied Cells are zero");
        }
        else
        {
            newNode->gain_ =  newParent->gain_ + g ;//* exp(-params_.degressiveCoeff_ * newNode->distance_);
            ROS_INFO("There is GAIN");

        }
        ROS_INFO("GAIN IS:%f",newNode->gain_);

        // Display new node
        publishNode(newNode);
        // Update best IG and node if applicable
        if (newNode->gain_ > bestGain_)
        {
            bestGain_ = newNode->gain_;
            bestNode_ = newNode;
        }
    }

}

#endif


