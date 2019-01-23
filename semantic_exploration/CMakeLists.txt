cmake_minimum_required(VERSION 2.8.3)
project(semantic_exploration)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world
  std_msgs
  tf
  kdtree
  trajectory_msgs
  mav_msgs
  cmake_modules
  rospy
  pcl_ros
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  rviz_visual_tools
  culling
)

find_package(Eigen REQUIRED)

#find_package(octomap REQUIRED)
include("/usr/local/share/octomap/octomap-config.cmake")

add_service_files(
  DIRECTORY srv
  FILES
  GetPath.srv
  SelectPose.srv
  ExtractView.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree trajectory_msgs mav_msgs rospy std_msgs pcl_ros genmsg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
)

add_executable(rrt_planner src/rrt_node.cpp src/mesh_structure.cpp src/rrt_core.cpp src/rrt_tree.cpp src/rrt_planner.cpp)

target_link_libraries(rrt_planner
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(rrt_planner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

add_executable(exploration_planner_node src/exploration_planner_node.cpp)
add_dependencies(exploration_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_planner_node ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(drone_commander src/drone_commander.cpp)
target_link_libraries(drone_commander ${catkin_LIBRARIES} )
add_dependencies(drone_commander ${catkin_EXPORTED_TARGETS})
