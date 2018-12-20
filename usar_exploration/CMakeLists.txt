cmake_minimum_required(VERSION 2.8.3)
project(usar_exploration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_definitions(-std=c++11)

#find_package(octomap REQUIRED)
include("/usr/local/share/octomap/octomap-config.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  #shape_msgs
  rviz_visual_tools
  octomap_world
  culling
  message_generation
  voxblox_ros
)

add_service_files(

  DIRECTORY srv
  FILES
  extractView.srv
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES ${OCTOMAP_LIBRARIES} ${OCTOMAP_LIBRARIES_DIR}
  #LIBRARIES usar_exploration occlusion_culling
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros octomap_world message_runtime genmsg voxblox_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/usar_exploration.cpp
# )

#add_executable(occlusion_culling_test src/occlusion_culling_test.cpp src/occlusion_culling.cpp src/voxel_grid_occlusion_estimation.cpp src/frustum_culling.cpp)
#add_dependencies(occlusion_culling_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(occlusion_culling_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})


add_executable(occlusion_culling_test_with_rrt src/occlusion_culling_test_with_rrt.cpp )
add_dependencies(occlusion_culling_test_with_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(occlusion_culling_test_with_rrt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(exploration_node src/exploration_node.cpp src/voxblox_map_manager.cpp)
add_dependencies(exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})


add_executable(current_view_extraction src/current_view_extraction.cpp)
add_dependencies(current_view_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(current_view_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(drone_commander src/drone_commander.cpp)
target_link_libraries(drone_commander ${catkin_LIBRARIES} )
add_dependencies(drone_commander ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

