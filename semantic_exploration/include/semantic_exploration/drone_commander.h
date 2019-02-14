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
