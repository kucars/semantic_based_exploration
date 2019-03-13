#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>
#include <octomap_generator/octomap_generator.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <semantics_point_type/semantics_point_type.h>
#include <cmath>
#include <cstring>
#include <sstream>

template <class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::OctomapGenerator()
    : octomap_(0.05), max_range_(1.), raycast_range_(1.)
{
}

template <class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::~OctomapGenerator()
{
}

template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::setUseSemanticColor(bool use)
{
    octomap_.setUseSemanticColor(use);
}

template <>
void OctomapGenerator<PCLColor, ColorOcTree>::setUseSemanticColor(bool use)
{
}

template <class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::isUseSemanticColor()
{
    return octomap_.isUseSemanticColor();
}

template <>
bool OctomapGenerator<PCLColor, ColorOcTree>::isUseSemanticColor()
{
    return false;
}

template <class CLOUD, class OCTREE>
VoxelStatus OctomapGenerator<CLOUD, OCTREE>::getBoundingBoxStatus(
        const Eigen::Vector3d& center, const Eigen::Vector3d& bounding_box_size,
        bool stop_at_unknown_voxel)
{
    return VoxelStatus::kFree;
}

template <class CLOUD, class OCTREE>
VoxelStatus OctomapGenerator<CLOUD, OCTREE>::getLineStatusBoundingBox(
        const Eigen::Vector3d& start, const Eigen::Vector3d& end,
        const Eigen::Vector3d& bounding_box_size)
{
    // Probably best way would be to get all the coordinates along
    // the line, then make a set of all the OcTreeKeys in all the bounding boxes
    // around the nodes... and then just go through and query once.

    const double epsilon = 0.001;  // Small offset
    VoxelStatus ret = VoxelStatus::kFree;
    const double& resolution = getResolution();

    // ROS_INFO("getLineStatusBoundingBox \n resolution %f",resolution);

    // Check corner connections and depending on resolution also interior:
    // Discretization step is smaller than the octomap resolution, as this way
    // no cell can possibly be missed
    //ROS_INFO("bounding_box_size.x()  %f",bounding_box_size.x() );

    double x_disc = bounding_box_size.x() / ceil((bounding_box_size.x() + epsilon) / resolution);

    double y_disc = bounding_box_size.y() / ceil((bounding_box_size.y() + epsilon) / resolution);
    double z_disc = bounding_box_size.z() / ceil((bounding_box_size.z() + epsilon) / resolution);

    // Ensure that resolution is not infinit
    if (x_disc <= 0.0)
        x_disc = 1.0;
    if (y_disc <= 0.0)
        y_disc = 1.0;
    if (z_disc <= 0.0)
        z_disc = 1.0;
    // ROS_INFO("x_disc %f",x_disc);
    // ROS_INFO("y_disc %f",y_disc);
    // ROS_INFO("z_disc %f",z_disc);

    const Eigen::Vector3d bounding_box_half_size = bounding_box_size * 0.5;
    //ROS_INFO("bounding_box_size %f %f %f",bounding_box_size.x() , bounding_box_size.y() , bounding_box_size.z() );
    //ROS_INFO("bounding_box_half_size %f %f %f %f ",bounding_box_half_size.x(),bounding_box_half_size.y(),bounding_box_half_size.z());

    for (double x = -bounding_box_half_size.x(); x <= bounding_box_half_size.x(); x += x_disc)
    {
        for (double y = -bounding_box_half_size.y(); y <= bounding_box_half_size.y(); y += y_disc)
        {
            for (double z = -bounding_box_half_size.z(); z <= bounding_box_half_size.z();
                 z += z_disc)
            {
                Eigen::Vector3d offset(x, y, z);
                ret = getLineStatus(start + offset, end + offset);
                if (ret != VoxelStatus::kFree)
                {
                    return ret;
                }
            }
        }
    }
    return VoxelStatus::kFree;
}

template <>
double OctomapGenerator<PCLColor, ColorOcTree>::getCellIneterestGain(const Eigen::Vector3d& point)
{
    // Not implemented yet (cannot access the getNumvisits())
    octomap::ColorOcTreeNode* node = octomap_.search(point.x(), point.y(), point.z());
    std::cout << "Color: " << node->getColor()<< std::endl;
    octomap::ColorOcTreeNode::Color interestColor;
    //std::map<std::string,octomap::ColorOcTreeNode::Color>::iterator it;
    std::vector<std::string>::iterator it;
    for (it = objectsOfInterest.begin(); it != objectsOfInterest.end(); ++it)
    {
        interestColor = semanticColoredLabels[*it];
        if(node->getColor() == interestColor)
        {
            return 1.0;
            // Do something to the interest color
        }
    }

    return 0 ;

}

template <>
double OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>::getCellIneterestGain(const Eigen::Vector3d& point)
{
    SemanticsOcTreeNodeMax* node = octomap_.search(point.x(), point.y(), point.z());

    bool isSemantic = false;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics().confidence << std::endl;

    // Check if the voxel has been semantically labelled (visited)
    isSemantic = node->isSemanticsSet();
    //std::cout << "Semantics: " << node->getSemantics().confidence << std::endl << std::flush;

    if(isSemantic)
    {

        //ROS_INFO ("confidenceThreshold %f" , confidenceThreshold ) ;
        if (node->getSemantics().confidence < confidenceThreshold)
        {
            return (1.0 - node->getSemantics().confidence) ; // low confidance  + occupied
        }
        else
            return 0.0; // high confidance + occupied
    }
    else
    {
        //std::cout << "NOT Semanticly labeled: " << std::endl << std::flush;

        return 0.0; // not semantically labeled
    }

}

template <>
double OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>::getCellIneterestGain(const Eigen::Vector3d& point)
{
    // Not implemented yet
    SemanticsOcTreeNodeBayesian* node = octomap_.search(point.x(), point.y(), point.z());
    std::cout << "Number of visits: " << node->getNumVisits()<< std::endl;
    bool isSemantic = false;

}

/*
// returns an interest value for the different voxels type.
template <class CLOUD, class OCTREE>
double OctomapGenerator<CLOUD, OCTREE>::getCellIneterestGain(const Eigen::Vector3d& point)
{

}
*/

template <>
uint OctomapGenerator<PCLColor, ColorOcTree>::getCellNumOfVisits(const Eigen::Vector3d& point)
{
    // Not implemented yet
    octomap::ColorOcTreeNode* node = octomap_.search(point.x(), point.y(), point.z());
    bool isSemantic = false;
    std::cout << "Color: " << node->getColor().r<< std::endl;
    return 0 ;

}

template <>
uint OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>::getCellNumOfVisits(const Eigen::Vector3d& point)
{
    SemanticsOcTreeNodeMax* node = octomap_.search(point.x(), point.y(), point.z());
    octomap::ColorOcTreeNode::Color color ;
    color = node->getSemantics().semantic_color;
    //ROS_INFO("node colors are :%d %d %d",node->getSemantics().semantic_color.r,node->getSemantics().semantic_color.g,node->getSemantics().semantic_color.b);
    SemanticsOcTreeNodeMax::Color interestColor;

    //std::map<std::string,octomap::ColorOcTreeNode::Color>::iterator it;
    std::vector<std::string>::iterator it;
    for (it = objectsOfInterest.begin(); it != objectsOfInterest.end(); ++it)
    {
        interestColor = semanticColoredLabels[*it];

        if(color == interestColor)
        {
            // for debugging ROS_Error is used
            //ROS_ERROR("interest Colors are:%d %d %d",interestColor.r,interestColor.g,interestColor.b); // worked
            //ROS_INFO("Number of visits: %d",node->getNumVisits()); // Not initialized
            //ROS_INFO("numOfVisitsThreshold: %d",numOfVisitsThreshold); // Not initialized
            if (node->getNumVisits() < numOfVisitsThreshold)
            {
                return 1;
            }

        }
    }
    return 0 ;
}


template <>
uint OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>::getCellNumOfVisits(const Eigen::Vector3d& point)
{
    // Not implemented yet
    SemanticsOcTreeNodeBayesian* node = octomap_.search(point.x(), point.y(), point.z());
    bool isSemantic = false;
    return 0 ;
}


// returns a number that indicates the proposed type from octomap
template <class CLOUD, class OCTREE>
int OctomapGenerator<CLOUD, OCTREE>::getCellIneterestCellType(double x, double y, double z)
{
    //TODO: this has to be re-written
    /*
    octomap::LabelOcTreeNode* node = octree_->search(x, y, z);
    if (node == NULL) {
        return 1;
    } else {
        if (octree_->isNodeOccupied(node)) {
            octomap::LabelOcTreeNode::Label& label = node->getLabel() ;
            if(label.type == octomap::LabelOcTreeNode::Label::VOXEL_OCCUPIED_INTEREST_NOT_VISITED)
                return 2;
            else if (label.type == octomap::LabelOcTreeNode::Label::VOXEL_OCCUPIED_INTEREST_VISITED)
                return 3;
            else
                return 4;
        } else {
            return 0;
        }
    }
    */
    return 0;
}


// returns true if the last voxel in the ray was unknown(rear side).
template <class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::getRearSideVoxel(const Eigen::Vector3d& view_point,
                                                       const Eigen::Vector3d& voxel_to_test)
{
    // Get all node keys for this line.
    // This is actually a typedef for a vector of OcTreeKeys.
    // Can't use the key_ray_ temp member here because this is a const function.
    key_ray.reset();

    octomap_.computeRayKeys(pointEigenToOctomap(view_point), pointEigenToOctomap(voxel_to_test),
                            key_ray);

    const octomap::OcTreeKey& voxel_to_test_key =
            octomap_.coordToKey(pointEigenToOctomap(voxel_to_test));
    int i = 1;
    int s = key_ray.size();
    for (octomap::OcTreeKey key : key_ray)
    {
        std::cout << " Counter : " << i << std::endl;

        if (key != voxel_to_test_key)
        {
            octomap::OcTreeNode* node = octomap_.search(key);
            if (node == NULL)
            {
                std::cout << "CellStatus::kUnknown" << std::endl;
            }
            else if (octomap_.isNodeOccupied(node))
            {
                std::cout << "CellStatus::kOccupied" << std::endl;
            }
            else
                std::cout << "CellStatus::kFree" << std::endl;

            if (i == s)
            {
                std::cout << "Last voxel" << std::endl;
                if (node == NULL)
                    return true;
                else
                    return false;
            }
        }
        i = i + 1;
    }
}

//returns the visibility likelihood for a ray from voxel 1 to n-1
template <class CLOUD, class OCTREE>
double OctomapGenerator<CLOUD, OCTREE>::getVisibilityLikelihood(
        const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test)
{
    // Get all node keys for this line.
    // This is actually a typedef for a vector of OcTreeKeys.
    // Can't use the key_ray_ temp member here because this is a const function.
    key_ray.reset();

    double visibilityLikelihood = 1;
    double probability = 0;
    octomap_.computeRayKeys(pointEigenToOctomap(view_point), pointEigenToOctomap(voxel_to_test),
                            key_ray);

    const octomap::OcTreeKey& voxel_to_test_key =
            octomap_.coordToKey(pointEigenToOctomap(voxel_to_test));
    //std::cout << "Before the loop " << std::endl << std::flush;

    // Now check if there are any unknown or occupied nodes in the ray,
    // except for the voxel_to_test key.
    for (octomap::OcTreeKey key : key_ray)
    {
        if (key != voxel_to_test_key)
        {
            octomap::OcTreeNode* node = octomap_.search(key);

            if (node == nullptr)
                probability = 0.5;
            else
                probability = node->getOccupancy();

            double probabilityBar = 1 - probability;
            visibilityLikelihood =
                    visibilityLikelihood * probabilityBar;  // 1 - node->getOccupancy();
            //std::cout << "visibilityLikelihood" << visibilityLikelihood << std::endl << std::flush;
        }
    }
    return visibilityLikelihood;
}

template <class CLOUD, class OCTREE>
Eigen::Vector3d OctomapGenerator<CLOUD, OCTREE>::getMapSize()
{
    // Metric min and max z of the map:
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octomap_.getMetricMin(min_x, min_y, min_z);
    octomap_.getMetricMax(max_x, max_y, max_z);

    return Eigen::Vector3d(max_x - min_x, max_y - min_y, max_z - min_z);
}

template <class CLOUD, class OCTREE>
VoxelStatus OctomapGenerator<CLOUD, OCTREE>::getCellProbabilityPoint(const Eigen::Vector3d& point,
                                                                     double* probability)
{
    octomap::OcTreeNode* node = octomap_.search(point.x(), point.y(), point.z());
    if (node == nullptr)
    {
        if (probability)
        {
            *probability = -1.0;
        }
        return VoxelStatus::kUnknown;
    }
    else
    {
        if (probability)
        {
            *probability = node->getOccupancy();
        }
        if (octomap_.isNodeOccupied(node))
        {
            return VoxelStatus::kOccupied;
        }
        else
        {
            return VoxelStatus::kFree;
        }
    }
}

template <class CLOUD, class OCTREE>
VoxelStatus OctomapGenerator<CLOUD, OCTREE>::getVisibility(const Eigen::Vector3d& view_point,
                                                           const Eigen::Vector3d& voxel_to_test,
                                                           bool stop_at_unknown_cell)
{
    // Get all node keys for this line.
    // This is actually a typedef for a vector of OcTreeKeys.
    // Can't use the key_ray_ temp member here because this is a const function.
    key_ray.reset();

    octomap::point3d octoViewPoint =
            octomap::point3d(view_point.x(), view_point.y(), view_point.z());
    octomap::point3d octoVoxel2Test =
            octomap::point3d(voxel_to_test.x(), voxel_to_test.y(), voxel_to_test.z());

    octomap_.computeRayKeys(octoViewPoint, octoVoxel2Test, key_ray);

    const octomap::OcTreeKey& voxel_to_test_key = octomap_.coordToKey(octoVoxel2Test);

    // Now check if there are any unknown or occupied nodes in the ray,
    // except for the voxel_to_test key.
    for (octomap::OcTreeKey key : key_ray)
    {
        if (key != voxel_to_test_key)
        {
            octomap::OcTreeNode* node = octomap_.search(key);
            if (node == nullptr)
            {
                if (stop_at_unknown_cell)
                {
                    return VoxelStatus::kUnknown;
                }
            }
            else if (octomap_.isNodeOccupied(node))
            {
                return VoxelStatus::kOccupied;
            }
        }
    }
    return VoxelStatus::kFree;
}

template <class CLOUD, class OCTREE>
VoxelStatus OctomapGenerator<CLOUD, OCTREE>::getLineStatus(const Eigen::Vector3d& start,
                                                           const Eigen::Vector3d& end)
{
    // Get all node keys for this line.
    // This is actually a typedef for a vector of OcTreeKeys.
    // Can't use the key_ray_ temp member here because this is a const function.
    key_ray.reset();

    octomap::point3d octoStart = octomap::point3d(start.x(), start.y(), start.z());
    octomap::point3d octoEnd = octomap::point3d(end.x(), end.y(), end.z());

    octomap_.computeRayKeys(octoStart, octoEnd, key_ray);

    // Now check if there are any unknown or occupied nodes in the ray.
    for (octomap::OcTreeKey key : key_ray)
    {
        octomap::OcTreeNode* node = octomap_.search(key);
        if (node == nullptr)
        {
            if (true)
            {
                return VoxelStatus::kOccupied;
            }
            else
            {
                return VoxelStatus::kUnknown;
            }
        }
        else if (octomap_.isNodeOccupied(node))
        {
            return VoxelStatus::kOccupied;
        }
    }
    return VoxelStatus::kFree;
}

template <class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::lookupTransformation(const std::string& from_frame,
                                                           const std::string& to_frame,
                                                           const ros::Time& timestamp,
                                                           Transformation* transform)
{
    tf::StampedTransform tf_transform;
    ros::Time time_to_lookup = timestamp;
    // If this transform isn't possible at the time, then try to just look up
    // the latest (this is to work with bag files and static transform publisher,
    // etc).
    if (!tf_listener_.canTransform(to_frame, from_frame, time_to_lookup))
    {
        return false;
        /*
        ros::Duration timestamp_age = ros::Time::now() - time_to_lookup;
        if (timestamp_age < tf_listener_.getCacheLength())
        {
            time_to_lookup = ros::Time(0);
            ROS_WARN("Using latest TF transform instead of timestamp match.");
        }
        else
        {
            ROS_ERROR("Requested transform time older than cache limit.");
            return false;
        }
        */
    }

    try
    {
        tf_listener_.lookupTransform(to_frame, from_frame, time_to_lookup, tf_transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("Error getting TF transform from sensor data: " << ex.what());
        return false;
    }

    tf::transformTFToKindr(tf_transform, transform);
    return true;
}

template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(
        const sensor_msgs::PointCloud2::ConstPtr& cloud_in, const std::string& to_frame)
{
    Transformation sensor_to_world;
    if (lookupTransformation(cloud_in->header.frame_id, to_frame, cloud_in->header.stamp,
                             &sensor_to_world))
    {
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*cloud_in, *cloud);

        // Voxel filter to down sample the point cloud
        // Create the filtering object
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        // Perform voxel filter
        float voxel_flt_size = static_cast<float>(octomap_.getResolution());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxel_flt_size, voxel_flt_size, voxel_flt_size);
        sor.filter(*cloud_filtered);
        // Convert to PCL pointcloud
        CLOUD pcl_cloud;
        pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
        pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_to_world.getTransformationMatrix());
        octomap::point3d origin(static_cast<float>(sensor_to_world.getPosition().x()),
                                static_cast<float>(sensor_to_world.getPosition().y()),
                                static_cast<float>(sensor_to_world.getPosition().z()));
        // Point cloud to be inserted with ray casting
        octomap::Pointcloud raycast_cloud;
        int endpoint_count = 0;  // total number of endpoints inserted
        for (typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
        {
            // Check if the point is invalid
            if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
            {
                float dist = sqrt((it->x - origin.x()) * (it->x - origin.x()) +
                                  (it->y - origin.y()) * (it->y - origin.y()) +
                                  (it->z - origin.z()) * (it->z - origin.z()));
                // Check if the point is in max_range
                if (dist <= max_range_)
                {
                    // Check if the point is in the ray casting range
                    if (dist <=
                            raycast_range_)  // Add to a point cloud and do ray casting later all together
                    {
                        raycast_cloud.push_back(it->x, it->y, it->z);
                    }
                    else  // otherwise update the occupancy of node and transfer the point to the raycast range
                    {
                        octomap::point3d direction =
                                (octomap::point3d(it->x, it->y, it->z) - origin).normalized();
                        octomap::point3d new_end =
                                origin + direction * (raycast_range_ + octomap_.getResolution() * 2);
                        raycast_cloud.push_back(new_end);
                        octomap_.updateNode(
                                    it->x, it->y, it->z, true,
                                    false);  // use lazy_eval, run updateInnerOccupancy() when done
                    }
                    endpoint_count++;
                }

            }
        }
        // Do ray casting for points in raycast_range_
        if (raycast_cloud.size() > 0)
            octomap_.insertPointCloud(
                        raycast_cloud, origin, raycast_range_, false,
                        true);  // use lazy_eval, run updateInnerOccupancy() when done, use discretize to downsample cloud
        // Update colors and semantics, differs between templates
        updateColorAndSemantics(&pcl_cloud);
        // updates inner node occupancy and colors
        if (endpoint_count > 0)
            octomap_.updateInnerOccupancy();
    }
    else
    {
        ROS_INFO("Failed to find transformation from %s to %s", cloud_in->header.frame_id.c_str(),
                 to_frame.c_str());
    }
}

template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud,
                                                       const Eigen::Matrix4f& sensorToWorld)
{
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    // Perform voxel filter
    float voxel_flt_size = octomap_.getResolution();
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_flt_size, voxel_flt_size, voxel_flt_size);
    sor.filter(*cloud_filtered);
    // Convert to PCL pointcloud
    CLOUD pcl_cloud;
    pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
    //std::cout << "Voxel filtered cloud size: "<< pcl_cloud.size() << std::endl;
    // Transform coordinate
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);

    //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
    //octomap::point3d origin(originTf[0], originTf[1], originTf[2]);
    octomap::point3d origin(static_cast<float>(sensorToWorld(0, 3)),
                            static_cast<float>(sensorToWorld(1, 3)),
                            static_cast<float>(sensorToWorld(2, 3)));
    octomap::Pointcloud raycast_cloud;  // Point cloud to be inserted with ray casting
    int endpoint_count = 0;             // total number of endpoints inserted
    for (typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            //it->num_of_visits = 0 ;
            float dist = sqrt((it->x - origin.x()) * (it->x - origin.x()) +
                              (it->y - origin.y()) * (it->y - origin.y()) +
                              (it->z - origin.z()) * (it->z - origin.z()));
            // Check if the point is in max_range
            if (dist <= max_range_)
            {
                // Check if the point is in the ray casting range
                if (dist <=
                        raycast_range_)  // Add to a point cloud and do ray casting later all together
                {
                    raycast_cloud.push_back(it->x, it->y, it->z);
                }
                else  // otherwise update the occupancy of node and transfer the point to the raycast range
                {
                    octomap::point3d direction =
                            (octomap::point3d(it->x, it->y, it->z) - origin).normalized();
                    octomap::point3d new_end =
                            origin + direction * (raycast_range_ + octomap_.getResolution() * 2);
                    raycast_cloud.push_back(new_end);
                    octomap_.updateNode(
                                it->x, it->y, it->z, true,
                                false);  // use lazy_eval, run updateInnerOccupancy() when done
                }
                endpoint_count++;
            }
        }
    }
    // Do ray casting for points in raycast_range_
    if (raycast_cloud.size() > 0)
        octomap_.insertPointCloud(
                    raycast_cloud, origin, raycast_range_, false,
                    true);  // use lazy_eval, run updateInnerOccupancy() when done, use discretize to downsample cloud
    // Update colors and semantics, differs between templates
    updateColorAndSemantics(&pcl_cloud);
    // updates inner node occupancy and colors
    if (endpoint_count > 0)
        octomap_.updateInnerOccupancy();
}

template <>
void OctomapGenerator<PCLColor, ColorOcTree>::updateColorAndSemantics(PCLColor* pcl_cloud)
{
    for (PCLColor::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
    {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
        }
    }
    octomap::ColorOcTreeNode* node =
            octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
}

template <>
void OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>::updateColorAndSemantics(
        PCLSemanticsMax* pcl_cloud)
{

    for (PCLSemanticsMax::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
    {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);

            SemanticsOcTreeNodeMax* node = octomap_.search(it->x, it->y, it->z) ;
            if (node == nullptr)
            {
                //ROS_INFO("CellStatus::Unknown") ;
                continue ;
            }
            else if (octomap_.isNodeOccupied(node))
            {
                bool isSemantic = false;
                isSemantic = node->isSemanticsSet();
                //ROS_INFO("2") ;
                octomap::SemanticsMax sem = node->getSemantics();
                if(!isSemantic)
                    sem.numVisits = 0 ;
                else
                {
                    //ROS_INFO("n->getNumVisits() %d ", n->getNumVisits()) ;
                    sem.numVisits = sem.numVisits + 1 ; // n->incrementNumVisits();
                    //ROS_INFO("sem.numVisits %d ", sem.numVisits) ;
                }
                // Get semantics
                uint32_t rgb;
                std::memcpy(&rgb, &it->semantic_color, sizeof(uint32_t));

                sem.semantic_color.b = (rgb >> 16) & 0x0000ff;
                sem.semantic_color.g = (rgb >> 8) & 0x0000ff;
                sem.semantic_color.r = (rgb)&0x0000ff;
                sem.confidence = it->confidence;
                octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);

            }
            //else
            //ROS_INFO("CellStatus::kFree") ;

        }
    }
    //SemanticsOcTreeNodeMax* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

template <>
void OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>::updateColorAndSemantics(
        PCLSemanticsBayesian* pcl_cloud)
{
    for (PCLSemanticsBayesian::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
    {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);

            // Get semantics
            octomap::SemanticsBayesian sem;
            for (int i = 0; i < 3; i++)
            {
                uint32_t rgb;
                std::memcpy(&rgb, &it->data_sem[i], sizeof(uint32_t));
                sem.data[i].color.r = (rgb >> 16) & 0x0000ff;
                sem.data[i].color.g = (rgb >> 8) & 0x0000ff;
                sem.data[i].color.b = (rgb)&0x0000ff;
                sem.data[i].confidence = it->data_conf[i];
            }
            octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);
        }
    }
    SemanticsOcTreeNodeBayesian* node =
            octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

template <class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::save(const char* filename)
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open())
    {
        std::cout << "Writing octomap to " << filename << std::endl;
        octomap_.write(outfile);
        outfile.close();
        std::cout << "Color tree written " << filename << std::endl;
        return true;
    }
    else
    {
        std::cout << "Could not open " << filename << " for writing" << std::endl;
        return false;
    }
}

//Explicit template instantiation
template class OctomapGenerator<PCLColor, ColorOcTree>;
template class OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>;
template class OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>;
