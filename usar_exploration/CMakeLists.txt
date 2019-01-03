cmake_minimum_required(VERSION 2.8.3)
project(usar_exploration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_definitions(-std=c++11)

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
  rviz_visual_tools
  octomap_world
  culling
  message_generation
)

#find_package(octomap REQUIRED)
include("/usr/local/share/octomap/octomap-config.cmake")

add_service_files(
  DIRECTORY srv
  FILES
  extractView.srv
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros octomap_world message_runtime genmsg
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
#target_link_libraries(occlusion_culling_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(exploration_node src/exploration_node.cpp)
add_dependencies(exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_node ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(occlusion_culling_test_with_rrt src/occlusion_culling_test_with_rrt.cpp )
add_dependencies(occlusion_culling_test_with_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(occlusion_culling_test_with_rrt ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(current_view_extraction src/current_view_extraction.cpp)
add_dependencies(current_view_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(current_view_extraction ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})


#add_executable(tfbr src/tfbr.cpp )
#add_dependencies(tfbr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(tfbr ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})


#add_executable(current_view_srv_call src/current_view_srv_call.cpp)
#target_link_libraries(current_view_srv_call ${catkin_LIBRARIES})
#add_dependencies(current_view_srv_call ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} usar_exploration_gencpp ${PROJECT_NAME}_generate_messages_cpp ${usar_exploration_EXPORTED_TARGETS})

#add_executable(exploration_node_client src/exploration_node_client.cpp)
#target_link_libraries(exploration_node_client ${catkin_LIBRARIES})
#add_dependencies(exploration_node_client ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} usar_exploration_gencpp ${PROJECT_NAME}_generate_messages_cpp ${usar_exploration_EXPORTED_TARGETS})




