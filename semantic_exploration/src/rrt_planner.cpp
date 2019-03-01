#include <chrono>
#include <thread>

#include <semantic_exploration/rrt_core.h>
#include <cstdlib>

#include <semantic_exploration/GetDroneState.h>
#include <semantic_exploration/rrt_planner.h>
#include <semantic_exploration/rrt_tree.h>
#include <semantic_cloud/GetSemanticColoredLabels.h>
#include <semantic_cloud/SemanticColoredLabels.h>

#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

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
    // params_.evaluatedPoints_  = nh_.advertise<visualization_msgs::Marker>("evaluatedPoint", 1);
    plannerService_ =
        nh_.advertiseService("rrt_planner", &rrtNBV::RRTPlanner::plannerCallback, this);

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

    // Initiate octree
    if (params_.treeType_ == SEMANTICS_OCTREE_BAYESIAN || params_.treeType_ == SEMANTICS_OCTREE_MAX)
    {
        if (params_.treeType_ == SEMANTICS_OCTREE_BAYESIAN)
        {
            ROS_INFO("Semantic octomap generator [bayesian fusion]");
            octomap_generator_ =
                new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
        }
        else
        {
            ROS_INFO("Semantic octomap generator [max fusion]");
            octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
        }
        toggleSemanticService = nh_.advertiseService(
            "toggle_use_semantic_color", &rrtNBV::RRTPlanner::toggleUseSemanticColor, this);
    }
    else
    {
        ROS_INFO("Color octomap generator");
        octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();
    }

    fullmapPub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    pointcloud_sub_ =
        nh_.subscribe(params_.pointCloudTopic_, 1, &rrtNBV::RRTPlanner::insertCloudCallback, this);

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
        delete octomap_generator_;
    }
    if (file_path_.is_open())
    {
        file_path_.close();
    }
}

void rrtNBV::RRTPlanner::getSemanticLabelledColors()
{
    semantic_cloud::GetSemanticColoredLabels getSemanticColoredLabels;
    ROS_INFO("Getting Semantic Colored Labels");

    ros::service::waitForService("get_semantic_colored_labels",ros::Duration(5.0));
    if(ros::service::call("get_semantic_colored_labels", getSemanticColoredLabels))
    {
        semantic_cloud::SemanticColoredLabels res = getSemanticColoredLabels.response.semantic_colored_labels;
        for(auto i = res.semantic_colored_labels.begin(); i != res.semantic_colored_labels.end(); i++ )
        {
            octomap::ColorOcTreeNode::Color color = octomap::ColorOcTreeNode::Color((*i).color_r,
                                                                                    (*i).color_g,
                                                                                    (*i).color_b);
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
                   << "known_cells_counter"
                   << ","
                   << "all_cells_counter"
                   << ","
                   << "traveled_distance"
                   << ","
                   << "free_type_counter"
                   << ","
                   << "unknown_type_count"
                   << ","
                   << "occ_intr_not_vis_type_count"
                   << ","
                   << "occ_intr_vis_type_count"
                   << ","
                   << "occ_not_intr_type_count"
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
                   << "objectFound"
                   << "\n";
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
    ROS_INFO("Received PointCloud");
    static double last = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - last > params_.pcl_throttle_)
    {
        ROS_INFO_THROTTLE(1.0, "inserting point cloud into rrtTree");
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
        ROS_INFO("PointCloud Insertion Took: %f", (toc - tic).toSec());
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
    ROS_INFO("########### New Planning Iteration ###########");
    ros::Time computationTime = ros::Time::now();
    params_.explorationarea_.publish(area_marker_);

    // Check that planner is ready to compute path.
    if (!ros::ok())
    {
        ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
        return true;
    }

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

    // Clear old tree and reinitialize.
    rrtTree->clear();
    rrtTree->initialize();

    ROS_INFO("Tree Initilization called");

    int loopCount = 0;
    int k = 1;

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
            res.path = rrtTree->getPathBackToPrevious(req.header.frame_id);
            return true;
        }
        if (rrtTree->iterate(1))
        {
            ROS_INFO("########## BEST GAIN ############## :%f", rrtTree->getBestGain());
        }
        loopCount++;
        k++;
    }
    ROS_INFO("Done RRT");

    // Extract the best edge.
    res.path = rrtTree->getBestEdge(req.header.frame_id);
    accumulativeGain += rrtTree->getBestGain();
    bool ObjectFoundFlag = rrtTree->getObjectFlag();
    std::cout << " ########## BEST GAIN ############## " << rrtTree->getBestGain() << std::endl
              << std::flush;
    std::cout << "SIZE OF THE PATH " << res.path.size() << std::endl << std::flush;
    rrtTree->memorizeBestBranch();
    ROS_INFO("Path computation lasted %2.3fs", (ros::Time::now() - computationTime).toSec());

    MaxGainPose(res.path[res.path.size() - 1], iteration_num);

    ros::Time tic_log = ros::Time::now();
    //**************** logging results ************************************************************************** //
    double res_map = octomap_generator_->getResolution();
    Eigen::Vector3d vec;
    double x, y, z;
    double all_cells_counter = 0, free_cells_counter = 0, unknown_cells_counter = 0,
           occupied_cells_counter = 0;
    int free_type_counter = 0, unknown_type_count = 0, occ_intr_not_vis_type_count = 0,
        occ_intr_vis_type_count = 0, occ_not_intr_type_count = 0;
    double information_gain_entropy = 0, occupancy_entropy = 0;
    double semantic_gain_entropy = 0, semantic_entropy = 0;
    double total_gain = 0;
    double probability;
    double maxThreshold = -0.5 * std::log(0.5) - ((1 - 0.5) * std::log(1 - 0.5));
    double rrt_gain = 0;
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

                // Counting Cell Types
                int cellType = octomap_generator_->getCellIneterestCellType(x, y, z);
                switch (cellType)
                {
                    case 0:
                        free_type_counter++;
                        break;
                    case 1:
                        unknown_type_count++;
                        break;
                    case 2:
                        occ_intr_not_vis_type_count++;
                        break;
                    case 3:
                        occ_intr_vis_type_count++;
                        break;
                    case 4:
                        occ_not_intr_type_count++;
                        break;
                }

                all_cells_counter++;

                // calculate information_gain
                VoxelStatus node = octomap_generator_->getCellProbabilityPoint(vec, &probability);
                double p = 0.5;
                if (probability != -1)
                {
                    p = probability;
                    // ROS_INFO("probability %f \n", p);
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
                    unknown_cells_counter++;
                }
                if (node == VoxelStatus::kFree)
                {
                    free_cells_counter++;
                }
                if (node == VoxelStatus::kOccupied)
                {
                    occupied_cells_counter++;
                }

                // *************** RRT IG *********************************** //

                if (node == VoxelStatus::kUnknown)
                {
                    rrt_gain += params_.igUnmapped_;
                }
                else if (node == VoxelStatus::kOccupied)
                {
                    rrt_gain += params_.igOccupied_;
                }
                else
                {
                    rrt_gain += params_.igFree_;
                }
            }
        }
    }

    double theoretical_cells_value =
        ((params_.maxX_ - params_.minX_) * (params_.maxY_ - params_.minY_) *
         (params_.maxZ_ - params_.minZ_)) /
        (res_map * res_map * res_map);
    double known_cells_counter = free_cells_counter + occupied_cells_counter;
    double volumetric_coverage =
        ((free_cells_counter + occupied_cells_counter) / all_cells_counter) * 100.0;
    iteration_num++;
    file_path_ << iteration_num << "," << volumetric_coverage << "," << information_gain_entropy
               << "," << semantic_gain_entropy << "," << total_gain << "," << free_cells_counter
               << "," << occupied_cells_counter << "," << unknown_cells_counter << ","
               << known_cells_counter << "," << all_cells_counter << "," << traveled_distance << ","
               << free_type_counter << "," << unknown_type_count << ","
               << occ_intr_not_vis_type_count << "," << occ_intr_vis_type_count << ","
               << occ_not_intr_type_count << ",";
    file_path_ << res.path[0].position.x << ",";
    file_path_ << res.path[0].position.y << ",";
    file_path_ << res.path[0].position.z << ",";
    file_path_ << res.path[0].orientation.x << ",";
    file_path_ << res.path[0].orientation.y << ",";
    file_path_ << res.path[0].orientation.z << ",";
    file_path_ << res.path[0].orientation.w << ",";
    file_path_ << accumulativeGain << ",";
    file_path_ << rrt_gain << "\n";
    ros::Time toc_log = ros::Time::now();
    std::cout << "logging Filter took:" << toc_log.toSec() - tic_log.toSec() << std::endl
              << std::flush;
    //***********************************************************************************************************//
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

    return ret;
}

void rrtNBV::RRTPlanner::MaxGainPose(geometry_msgs::Pose p, int id)
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
    line_strip.scale.x = 0.05;
    line_strip.color.a = 1;
    line_strip.color.g = 1;
    line_strip.lifetime = ros::Duration();
    marker_pub_.publish(line_strip);
    viewpoints2.poses.push_back(p);
    viewpoints2.header.frame_id = "world";
    viewpoints2.header.stamp = ros::Time::now();
    params_.sensor_pose_pub_.publish(viewpoints2);
}
