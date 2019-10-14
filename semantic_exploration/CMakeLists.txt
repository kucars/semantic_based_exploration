cmake_minimum_required(VERSION 2.8.3)
project(semantic_exploration)

add_definitions(-std=c++11)

# SET OCTOMAP_OMP to disable/enable OPENMP (experimental for octomap, default disabled)
SET(OCTOMAP_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{OCTOMAP_OMP})
  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
ENDIF(DEFINED ENV{OCTOMAP_OMP})
IF(OCTOMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OCTOMAP_OMP)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
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
  pcl_conversions
  std_srvs
  octomap_msgs
  minkindr
  minkindr_conversions
  controller_msgs
)

find_package(Eigen REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  GetPath.srv
  SelectPose.srv
  ExtractView.srv
  GetDroneState.srv
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
  LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs tf kdtree trajectory_msgs mav_msgs rospy std_msgs pcl_ros genmsg
  DEPENDS octomap PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${controller_msgs_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(rrt_planner  src/rrt_node.cpp
                            src/rrt_core.cpp
                            src/rrt_tree.cpp
                            src/rrt_planner.cpp
                            src/octomap_generator/octomap_generator.cpp
                            src/semantic_octree/semantics_bayesian.cpp
                            src/semantic_octree/semantics_max.cpp
                            src/utilities/time_profiler.cpp
                            )

target_link_libraries(rrt_planner
  ${LINK_LIBS}
   ${Boost_DATE_TIME_LIBRARY}
)

add_dependencies(rrt_planner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

add_executable(exploration_planner_node src/exploration_planner_node.cpp)
add_dependencies(exploration_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_planner_node ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(drone_commander src/drone_commander.cpp)
add_dependencies(drone_commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_commander ${catkin_LIBRARIES})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  #roslint_cpp()
  #roslint_python()
  #roslint_add_test()
  add_rostest(test/navigation.test)
endif()

