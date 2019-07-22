#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <string>

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <chrono>
#include <thread>

#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <semantic_exploration/GetPath.h>
#include <waypoint_navigator/GoToPoseWaypoints.h>
#include <geometry_msgs/PoseArray.h>
#include <controller_msgs/FlatTarget.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

using namespace std;
using namespace octomap;

struct Params
{
    double dt;
    int numIterations;
    bool iflog;
    int root_note_delay_indicator ; 
};

class ExplorationPlanner
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Params params_;
    geometry_msgs::PoseStamped currentPose, explorationViewPointPose;
    ros::Publisher explorationViewpointPub;
    ros::Publisher rotationPublisher;
    ros::Subscriber localPoseSub;
    ros::Subscriber odomSub;
    ros::Subscriber stateSub;
    bool currentPositionUpdated;
    mavros_msgs::State currentState;

public:
    ExplorationPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~ExplorationPlanner();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    bool SetParams();
    void RunStateMachine();
    void rotateOnSpot(float duration);
    void moveOnSpot(float duration);
    void odomCallback(const nav_msgs::Odometry& pose);
};

ExplorationPlanner::~ExplorationPlanner()
{
    
}

ExplorationPlanner::ExplorationPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    currentPositionUpdated = false;
    std::string droneName = ros::this_node::getName();
    std::string droneNameSpace = ros::this_node::getNamespace();

    ROS_INFO("Drone Name:%s NameSpace:%s", droneName.c_str(), droneNameSpace.c_str());

    explorationViewpointPub =
            nh_.advertise<geometry_msgs::PoseStamped>("semantic_exploration_viewpoint", 10);
    
    //TODO: make this a parameter and make it global
    bool use_gazebo_ground_truth_ = true;

    // Either use perfect positioning from gazebo, or get the px4 estimator position through mavros
    if (use_gazebo_ground_truth_)
    {
        odomSub = nh_.subscribe("odometry", 10, &ExplorationPlanner::odomCallback, this);
    }
    else
    {
        localPoseSub = nh_.subscribe<geometry_msgs::PoseStamped>(droneNameSpace +
                                                                 "/mavros/local_position/pose", 10, &ExplorationPlanner::poseCallback, this);
    }
    
    rotationPublisher = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 100);
    stateSub = nh_.subscribe("/mavros/state", 10, &ExplorationPlanner::stateCallback, this);

    if (!ExplorationPlanner::SetParams())
    {
        ROS_ERROR("Could not start. Parameters missing!");
    }
}

void ExplorationPlanner::rotateOnSpot(float duration)
{

    double sleepTime = duration / 360.0f;
    double lowerAltitude = 0.05;
    double altitudeIterations = 3.0f;
    double altitudeSteps = (currentPose.pose.position.z - lowerAltitude) / altitudeIterations;
    ros::Rate loopRate(1/sleepTime);
    for(int j=0;j<=altitudeIterations;j++)
    {
        for(int i=0; i<360; i++)
        {
            double yaw = i * 3.14 / 180.0;
            controller_msgs::FlatTarget yawCommand;
            yawCommand.type_mask = 2;
            yawCommand.position.x = this->currentPose.pose.position.x + j*0.2;
            yawCommand.position.y = this->currentPose.pose.position.y - j*0.2;
            yawCommand.position.z = lowerAltitude + j*altitudeSteps;
            yawCommand.snap.z     = yaw;
            ROS_INFO_THROTTLE(0.5,"Pose Yaw sent:%f sleep time:%f x:%f y:%f z:%f", yaw, sleepTime,yawCommand.position.x ,yawCommand.position.y, yawCommand.position.z);
            rotationPublisher.publish(yawCommand);
            loopRate.sleep();
        }
    }
}
void ExplorationPlanner::moveOnSpot(float duration)
{
    double sleepTime = duration / 360.0f;
    double lowerAltitude = 1;
    double altitudeIterations = 3.0f;
    ros::Rate loopRate(1/sleepTime);
    for(int j=0;j<=altitudeIterations;j++)
    {
        for(int i=0; i<360; i++)
        {
            double yaw = 0;
            controller_msgs::FlatTarget movrCommand;
            movrCommand.type_mask = 2;
            movrCommand.position.x = this->currentPose.pose.position.x + j*0.2;
            movrCommand.position.y = this->currentPose.pose.position.y;
            movrCommand.position.z = this->currentPose.pose.position.z;
            movrCommand.snap.z     = yaw;
            ROS_INFO_THROTTLE(0.5,"Pose Yaw 2 sent:%f sleep time:%f x:%f y:%f z:%f", yaw, sleepTime,movrCommand.position.x ,movrCommand.position.y, movrCommand.position.z);
            rotationPublisher.publish(movrCommand);
            loopRate.sleep();
        }
    }
    ROS_INFO("####################### DONE ###################################");
}

void ExplorationPlanner::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
}

void ExplorationPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->currentPose = *msg;
    currentPositionUpdated = true;
}

void ExplorationPlanner::odomCallback(const nav_msgs::Odometry& pose)
{
    this->currentPose.pose = pose.pose.pose;
    currentPositionUpdated = true;
}

void ExplorationPlanner::RunStateMachine()
{
    ros::Rate loopRate(10);

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused)
    {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused)
    {
        ROS_FATAL("Could not wake up Gazebo.");
        return;
    }
    else
    {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    while (!currentPositionUpdated)
    {
        ROS_INFO_THROTTLE(1, "Waiting for Pose");
        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("initial Position %f , %f , %f", currentPose.pose.position.x,
             currentPose.pose.position.y, currentPose.pose.position.z);
    int iteration = 0;
    int seqNum = 0;
    
    while(currentState.mode != "OFFBOARD" && currentState.system_status !=4)
    {
        ROS_INFO_THROTTLE(0.1,"Mode:%s Status:%d",currentState.mode.c_str(), currentState.system_status);
        ros::spinOnce();
        loopRate.sleep();
    }
    
    // Dirty hack, please fix Wait for the drone to take off
    for(int i=0;i<100;i++)
    {
        ros::spinOnce();
        loopRate.sleep();
    }
    
    //slowly rotate on the spot for 10 seconds
    rotateOnSpot(20);
    moveOnSpot(10);

    // simulate the camera position publisher by broadcasting the /tf
    tf::TransformBroadcaster br;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    std::string targetFrame = "local_origin";
    geometry_msgs::PoseStamped transformedPose;

    // Start planning: The planner is called and the computed path sent to the controller.
    ros::Time startTime = ros::Time::now();
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);

        /* TODO: handling the frames need improvement.
         * Either the service call should pass the target frame and recieved the already transformed
         * target frames back, or make target frame something that could be modified in a yaml file.
         * Currently it's hard coded above
         */
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::PoseStamped poseMsg_;
        semantic_exploration::GetPath planSrv;
        planSrv.request.header.stamp = ros::Time::now();
        planSrv.request.header.seq = static_cast<uint>(iteration);
        planSrv.request.header.frame_id = "world";
        waypoint_navigator::GoToPoseWaypoints gotoPoseWaypointsSrv;

        ROS_INFO("Planner Call");
        ros::service::waitForService("rrt_planner", ros::Duration(1.0));
        if (ros::service::call("rrt_planner", planSrv))
        {
            ROS_INFO("Planner Call Successfull");
            seqNum++;
            if (planSrv.response.path.size() == 0)
            {
                ROS_INFO(" ****************** No Path *************************");
                ros::Duration(1.0).sleep();
            }
            else
            {
                ROS_INFO(" #################### Path Found #######################");

                ROS_INFO("planSrv.response.path.size():%zu", planSrv.response.path.size());

                for (uint i = 0; i < planSrv.response.path.size(); i++)
                {
                    poseMsg_.header.seq = static_cast<uint>(seqNum);
                    poseMsg_.header.stamp = ros::Time::now();
                    poseMsg_.header.frame_id = "world";
                    poseMsg_.pose.position.x = planSrv.response.path[i].position.x;
                    poseMsg_.pose.position.y = planSrv.response.path[i].position.y;
                    poseMsg_.pose.position.z = planSrv.response.path[i].position.z;

                    tf::Pose pose;
                    tf::poseMsgToTF(planSrv.response.path[i], pose);
                    double yaw = tf::getYaw(pose.getRotation());
                    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
                    poseMsg_.pose.orientation.x = quat[0];
                    poseMsg_.pose.orientation.y = quat[1];
                    poseMsg_.pose.orientation.z = quat[2];
                    poseMsg_.pose.orientation.w = quat[3];
                    try
                    {
                        listener.lookupTransform(poseMsg_.header.frame_id, targetFrame, ros::Time(0),
                                                 transform);
                    }
                    catch (tf::TransformException ex)
                    {
                        ROS_ERROR("%s", ex.what());
                        continue;
                    }
                    listener.transformPose(targetFrame, poseMsg_, transformedPose);

                    ROS_DEBUG("Sent In iterate %f %f  %f %f %f %f %f ", poseMsg_.pose.position.x,
                              poseMsg_.pose.position.y, poseMsg_.pose.position.z,
                              poseMsg_.pose.orientation.x, poseMsg_.pose.orientation.y,
                              poseMsg_.pose.orientation.z, poseMsg_.pose.orientation.w);

                    ROS_INFO("Received: %f %f %f Transformed: %f %f %f", poseMsg_.pose.position.x,
                             poseMsg_.pose.position.y, poseMsg_.pose.position.z,
                             transformedPose.pose.position.x, transformedPose.pose.position.y,
                             transformedPose.pose.position.z);

                    std_msgs::String mission_id;
                    mission_id.data = "mission_id" + std::to_string(iteration);
                    waypoints.clear();
                    waypoints.push_back(poseMsg_.pose);
                    gotoPoseWaypointsSrv.request.waypoints = waypoints;
                    gotoPoseWaypointsSrv.request.mission_id = mission_id;

                    ros::service::waitForService("go_to_pose_waypoints", ros::Duration(1.0));

                    if(!ros::service::call("go_to_pose_waypoints", gotoPoseWaypointsSrv))
                    {
                        ROS_WARN("Calling waypoint navigator service failed");
                    }

                    //explorationViewpointPub.publish(transformedPose);
                    ros::spinOnce();
                    sleep(params_.root_note_delay_indicator); // An indication that the drone reached the NBV
                    ros::Duration(params_.dt).sleep();
                }
            }
        }
        else
        {
            ROS_WARN_THROTTLE(1, "Planner not reachable");
            ros::Duration(1.0).sleep();
        }

        if (++iteration > params_.numIterations)
        {
            ROS_INFO("Finished all iterations:%d, exiting", iteration);
            break;
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    ros::Time endTime = ros::Time::now();
    double elapsed = endTime.toSec() - startTime.toSec();
    std::cout << "The planning Time " << elapsed << std::endl;
}

bool ExplorationPlanner::SetParams()
{
    //std::string ns = ros::this_node::getName();
    //std::cout<<"Node name is:"<<ns<<"\n";
    std::string ns = "";
    bool ret = true;

    // Exploration Algorithm Params
    // 1- Termination

    params_.numIterations = 50;
    if (!ros::param::get(ns + "/exploration/num_iterations", params_.numIterations))
    {
        ROS_WARN("No number of iteration for termination specified. Looking for %s",
                 (ns + "/exploration/num_iterations").c_str());
    }

    params_.dt = 1.0;
    if (!ros::param::get(ns + "/exploration/dt", params_.dt))
    {
        ROS_WARN("Delta T between Iterations not found. Looking for %s",
                 (ns + "/exploration/dt").c_str());
    }
    params_.iflog = false;
    if (!ros::param::get(ns + "/exploration/log/on", params_.iflog))
    {
        ROS_WARN("No log is specified. Looking for %s.", (ns + "/exploration/log/on").c_str());
    }

    params_.root_note_delay_indicator = 5;
    if (!ros::param::get(ns + "/exploration/drone_arrival_duration", params_.root_note_delay_indicator))
    {
        ROS_WARN("No number of iteration for termination specified. Looking for %s",
                 (ns + "/exploration/drone_arrival_duration").c_str());
    }

    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ExplorationPlanner expObj(nh, nh_private);
    expObj.RunStateMachine();
    return 0;
}
