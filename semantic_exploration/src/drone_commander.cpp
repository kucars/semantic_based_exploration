#include "semantic_exploration/drone_commander.h"

DroneCommander::DroneCommander(const ros::NodeHandle &_nh, const ros::NodeHandle &_nhPrivate):
    nh(_nh),
    nhPrivate(_nhPrivate)
{
    std::string droneName = ros::this_node::getNamespace();
    std::cout<<"Drone Namespace is:"<<droneName << std::endl;
    stateSub          = nh.subscribe<mavros_msgs::State>(droneName + "/mavros/state", 10, &DroneCommander::stateCallback,this);
    localPoseSub      = nh.subscribe<geometry_msgs::PoseStamped>(droneName + "/mavros/local_position/pose", 10, &DroneCommander::poseCallback,this);
    goalSub           = nh.subscribe<geometry_msgs::PoseStamped>(droneName + "/semantic_exploration_viewpoint", 100,&DroneCommander::goalCallback,this);
    localPosePub      = nh.advertise<geometry_msgs::PoseStamped>(droneName + "/mavros/setpoint_position/local", 10);
    armingClinet      = nh.serviceClient<mavros_msgs::CommandBool>(droneName + "/mavros/cmd/arming");
    setModeClient     = nh.serviceClient<mavros_msgs::SetMode>(droneName + "/mavros/set_mode");
    rotationDonePub   = nh.advertise<std_msgs::Bool> (droneName + "/rotation/done", 10);
    service           = nh.advertiseService("get_drone_state", &DroneCommander::droneStateService, this);
    std_srvs::Empty srv;
    bool unpaused = false;
    uint i =0;
    // Trying to unpause Gazebo for 10 seconds.
    while(i <= 10 && !unpaused)
    {
      ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      unpaused = ros::service::call("/gazebo/unpause_physics", srv);
      ++i;
    }
    if(!unpaused)
    {
      ROS_FATAL("Could not wake up Gazebo.");
      return;
    }
    else
    {
      ROS_INFO("Unpaused the Gazebo simulation.");
    }

    // wait for FCU connection
    while(ros::ok() && currentState.connected)
    {
        ROS_INFO("Connected");
        ROS_INFO("Connecting: Current State:%s", currentState.mode.c_str());
        ros::spinOnce();
        rate.sleep();
    }

    goalLastReceived = ros::Time::now();
    commandPose = currentPose;
    offBoardSetMode.request.custom_mode = "OFFBOARD";
    armingCommand.request.value = true;
    lastRequestTime = ros::Time::now();

    droneCommanderState = DroneCommander::INITIALIZATION;
    while(ros::ok())
    {
        switch(droneCommanderState)
        {
        case DroneCommander::INITIALIZATION:
            ROS_INFO_THROTTLE(1.0,"INITILIZATION");
            ROS_INFO_THROTTLE(1.0,"Current State:%s", currentState.mode.c_str());
            if( currentState.mode != "OFFBOARD" && (ros::Time::now() - lastRequestTime > ros::Duration(5.5)))
            {
                if( setModeClient.call(offBoardSetMode) && offBoardSetMode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                lastRequestTime = ros::Time::now();
            }
            else
            {
                if( !currentState.armed && (ros::Time::now() - lastRequestTime > ros::Duration(5.5)))
                {
                    if( armingClinet.call(armingCommand) && armingCommand.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        droneCommanderState = DroneCommander::TAKE_OFF;
                    }
                    lastRequestTime = ros::Time::now();
                }
            }
            localPosePub.publish(commandPose);
            break;
        case DroneCommander::TAKE_OFF:
            ROS_INFO("TAKE_OFF");
            if(takeOff())
                droneCommanderState = DroneCommander::ROTATING;
            break;
        case DroneCommander::ROTATING:
            ROS_INFO("ROTATING");
            if(rotateOnTheSpot())
                droneCommanderState = DroneCommander::READY_FOR_WAYPOINTS;
            hoverPose = currentPose;
            ROS_INFO_THROTTLE(1.0,"   - Current Pose  is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);
            break;
        case DroneCommander::READY_FOR_WAYPOINTS:
            /*
             * if we receive a goal, then this will be our hovering pose
             * otherwise, use the last one
             */
            ROS_INFO_THROTTLE(1.0,"   - Current Pose  is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);

            if(isGoalReceived)
            {
                ROS_INFO_THROTTLE(1.0,"READY_FOR_WAYPOINTS ---> Navigating to WayPoint");
                currentGoal = goalReceived;
                currentGoal.header.stamp  = ros::Time::now();
                currentGoal.header.frame_id = "world";
                // save for when we don't get anymore
                hoverPose = currentGoal;
            }
            else {
                currentGoal = hoverPose;
            }
            ROS_INFO_THROTTLE(1.0,"   - Current Goal is: [%f %f %f]",currentGoal.pose.position.x,currentGoal.pose.position.y,currentGoal.pose.position.z);
            localPosePub.publish(currentGoal);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void DroneCommander::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
}

void DroneCommander::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    this->currentPose.pose = msg->pose;
}

void DroneCommander::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goalLastReceived  = ros::Time::now();
    goalReceived.pose = msg->pose;
    isGoalReceived    = true;
}

bool DroneCommander::takeOff()
{
    ROS_INFO("Starting the planner: Performing initialization motion -- Take off ");
    uint seqNum=0 ;
    double takeOffAltitude = 1.0;
    double intermediateAltitude = 0;
    geometry_msgs::PoseStamped takeOffPose;
    takeOffPose.pose.position.x  = currentPose.pose.position.x;
    takeOffPose.pose.position.y  = currentPose.pose.position.y;
    takeOffPose.pose.position.z  = currentPose.pose.position.z + takeOffAltitude;
    takeOffPose.pose.orientation = currentPose.pose.orientation;

    double distance2HoverPose = 1000;
    while(distance2HoverPose > 0.2)
    {
        distance2HoverPose =  fabs(currentPose.pose.position.x - takeOffPose.pose.position.x) +
                fabs(currentPose.pose.position.y - takeOffPose.pose.position.y) +
                fabs(currentPose.pose.position.z - takeOffPose.pose.position.z);
        ROS_INFO_THROTTLE(1.0,"Taking off");
        ROS_INFO_THROTTLE(1.0,"   - Trying to  reach: [%f %f %f]",takeOffPose.pose.position.x,takeOffPose.pose.position.y,takeOffPose.pose.position.z);
        ROS_INFO_THROTTLE(1.0,"   - Current Pose  is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);
        ROS_INFO_THROTTLE(1.0,"   - Distance to pose: [%f]",distance2HoverPose);
        if(fabs(currentPose.pose.position.z - takeOffPose.pose.position.z)>0.2)
        {
            intermediateAltitude = currentPose.pose.position.z + 0.2;
        }
        else
            intermediateAltitude = takeOffPose.pose.position.z;
        commandPose = takeOffPose;
        commandPose.pose.position.z = intermediateAltitude;
        commandPose.header.seq = seqNum++ ;
        commandPose.header.stamp = ros::Time::now();
        commandPose.header.frame_id = "world";
        localPosePub.publish(commandPose);
        ros::spinOnce();
        rate.sleep();
    }
    return true;
}

bool DroneCommander::rotateOnTheSpot()
{
    ROS_INFO("Starting the planner: Performing initialization motion -- rotation");
    geometry_msgs::PoseStamped rotatingPose;
    rotatingPose.pose.position.x = currentPose.pose.position.x;
    rotatingPose.pose.position.y = currentPose.pose.position.y;
    rotatingPose.pose.position.z = currentPose.pose.position.z;
    tf::Quaternion quat(currentPose.pose.orientation.x,
                        currentPose.pose.orientation.y,
                        currentPose.pose.orientation.z,
                        currentPose.pose.orientation.w);

    uint seqNum=0 ;
    double angleStep  = 0.025;
    double angle = quat.getAngle();
    bool done = false;
    double deltaTime = 1.0;
    ros::Time lastTimeTurnTime = ros::Time::now();
    while(!done)
    {
        if(ros::Time::now() - lastTimeTurnTime > ros::Duration(deltaTime))
        {
            ROS_INFO_THROTTLE(0.2,"Turning around");
            commandPose.header.seq         = seqNum++;
            commandPose.header.stamp       = ros::Time::now();
            commandPose.header.frame_id    = "world";
            commandPose.pose.position.x    = rotatingPose.pose.position.x;
            commandPose.pose.position.y    = rotatingPose.pose.position.y;
            commandPose.pose.position.z    = rotatingPose.pose.position.z;
            commandPose.pose.orientation.x = quat[0] ;
            commandPose.pose.orientation.y = quat[1] ;
            commandPose.pose.orientation.z = quat[2] ;
            commandPose.pose.orientation.w = quat[3] ;
            ROS_INFO("   - Trying to  rotate to: [%f %f %f %f]",commandPose.pose.position.x,commandPose.pose.position.y,commandPose.pose.position.z,180*angle/M_PI);
            ROS_INFO("   - Current Pose      is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);
            angle+=(angleStep*2.0*M_PI);
            quat.setRPY(0,0,angle);
            if(angle>=2.0*M_PI)
                done = true;
            lastTimeTurnTime = ros::Time::now();
        }
        localPosePub.publish(commandPose);
        ros::spinOnce();
        rate.sleep();
    }
    return true;
}

bool DroneCommander::droneStateService(semantic_exploration::GetDroneState::Request& request, semantic_exploration::GetDroneState::Response& response)
{
    response.drone_state = static_cast<uint8_t>(droneCommanderState);
    response.success     = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_commander");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    DroneCommander droneCommander(nh,nh_private);
    return 0;
}
