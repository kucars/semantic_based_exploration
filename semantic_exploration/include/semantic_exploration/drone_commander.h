/**
BSD 3-Clause License

Copyright (c) 2018, Khalifa University Robotics Institute
Copyright (c) 2018, Tarek Taha
Copyright (c) 2018, Reem Ashour
Copyright (c) 2020, Mphamed Abdelkader mohamedashraf123@gmail.com
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
#ifndef DRONE_COMMANDER_H
#define DRONE_COMMANDER_H

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <semantic_exploration/GetDroneState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <chrono>
#include <thread>


class DroneCommander
{
  public:
    DroneCommander(const ros::NodeHandle& _nh, const ros::NodeHandle& _nhPrivate);
    ~DroneCommander()
    {
    }
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    bool droneStateService(semantic_exploration::GetDroneState::Request& request,
                           semantic_exploration::GetDroneState::Response& response);
    enum
    {
        INITIALIZATION,
        TAKE_OFF,
        ROTATING,
        READY_FOR_WAYPOINTS
    };

  private:
    bool takeOff();
    bool rotateOnTheSpot();
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;
    ros::Time goalLastReceived;
    ros::Subscriber stateSub;
    ros::Subscriber goalSub;
    ros::Publisher localPosePub;
    ros::Subscriber localPoseSub;
    ros::Publisher rotationDonePub;
    ros::ServiceClient armingClinet;
    ros::ServiceClient setModeClient;
    geometry_msgs::PoseStamped goalReceived;
    geometry_msgs::PoseStamped currentPose;
    geometry_msgs::PoseStamped hoverPose;
    geometry_msgs::PoseStamped commandPose;
    mavros_msgs::SetMode offBoardSetMode;
    mavros_msgs::CommandBool armingCommand;
    ros::Time lastRequestTime;
    geometry_msgs::PoseStamped currentGoal;
    mavros_msgs::State currentState;
    std_msgs::Bool rotationDone;
    ros::Rate rate = 50;
    bool isGoalReceived = false;
    ros::ServiceServer service;
    int droneCommanderState;

};

#endif
