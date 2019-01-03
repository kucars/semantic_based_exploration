cmake_minimum_required(VERSION 2.8.3)
project(rrt_explorer)

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
)

find_package(Eigen REQUIRED)

#find_package(octomap REQUIRED)
include("/usr/local/share/octomap/octomap-config.cmake")

add_service_files(
  DIRECTORY srv
  FILES
  rrt_srv.srv
  rrt_srv_pose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree trajectory_msgs mav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(rrt_planner src/rrt_node.cpp src/mesh_structure.cpp src/rrt_core.cpp src/rrt_tree.cpp src/rrt_planner.cpp)

target_link_libraries(rrt_planner
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(rrt_planner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

#Rotor Simulator
#add_executable(rrt_navigator src/rrt_navigator.cpp)
#target_link_libraries(rrt_navigator ${catkin_LIBRARIES} )
#add_dependencies(rrt_navigator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

#PX4 with rrt original service call (returns a path)
#add_executable(rrt_navigator_px4 src/rrt_navigator_px4.cpp)
#target_link_libraries(rrt_navigator_px4 ${catkin_LIBRARIES} )
#add_dependencies(rrt_navigator_px4 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )


#PX4 Commander
#add_executable(drone_commander src/drone_commander.cpp)
#target_link_libraries(drone_commander ${catkin_LIBRARIES} )
#add_dependencies(drone_commander ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

# New planner, new node file
#add_executable(rrt_planner_deep src/rrt_node_deep.cpp src/mesh_structure.cpp src/rrt_tree.cpp src/rrt_planner_deep.cpp src/information_gain_base.cpp src/information_gain_classic.cpp src/information_gain_point_density.cpp src/information_gain_average_entropy.cpp)
#target_link_libraries(rrt_planner_deep
#  ${catkin_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#)
#add_dependencies(rrt_planner_deep ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

#PX4 with rrt original service call (returns a point)
#add_executable(rrt_navigator_px4_deep src/rrt_navigator_px4_deep.cpp)
#target_link_libraries(rrt_navigator_px4_deep ${catkin_LIBRARIES} )
#add_dependencies(rrt_navigator_px4_deep ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

#add_executable(exploration_planner_node src/exploration_planner_node.cpp)
#target_link_libraries(exploration_planner_node ${catkin_LIBRARIES} )
#add_dependencies(exploration_planner_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )

add_executable(drone_commander src/drone_commander.cpp)
target_link_libraries(drone_commander ${catkin_LIBRARIES} )
add_dependencies(drone_commander ${catkin_EXPORTED_TARGETS})
