# Semantic Based Exploration
The repository aims to provide an exploration algorithm and relevant packages to enable a semantic-aware autonomous exploration task for robots in indoor environments.  


# Building 
create a new workspace and clone all the modules into that workspace.

```
$ mkdir -p ~/catkin_ws/src 
$ cd ~/catkin_ws
$ wstool init src
$ wstool set -t src semantic_based_exploration https://github.com/kucars/semantic_based_exploration.git --git
$ wstool merge -t src https://raw.githubusercontent.com/kucars/semantic_based_exploration/master/semantic_exploration.rosinstall?token=AAR3chHe7p7e_GQmqZBvrNqbHrpATs7Sks5b2YtIwA%3D%3D
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ cd ~/catkin_ws/src/semantic_based_exploration
$ git submodule update --init --recursive
$ cd ~/catkin_ws/src/octomap/
$ mkdir build 
$ cd build 
$ cmake ..
$ sudo make install 
```

It is important to add octomap lib location to your LD_LIBRARY_PATH to avoid conflict with previously installed octomap library (or the one that comes with ROS)
```
echo "export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH" >> ~/.bashrc
```

Then build all (use -DCATKIN_WHITELIST_PACKAGES and -DCATKIN_BLACKLIST_PACKAGES to control the order if needed)
```
$ cd 
$ cd catkin_ws/
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ catkin build
```

We tested this repository with ROS Kinect on Ubuntu 16.04.

#  Current State 
Still under development  


#  Notes
If any changes in octomap pkg, re-build is essential  
```
$ cd catkin_ws/src/octomap
$ sudo rm -rf bin/ lib/ build/
$ cd ..
$ mkdir build 
$ cd build 
$ sudo cmake ..
$ sudo make install 
$ cd 
$ cd catkin_ws/
$ sudo rm -rf build/ devel/
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ catkin build
```

# Test 
```
roscore 
rosrun usar_exploration occlusion_culling_test
rosrun octomap_world octomap_manager
rosrun rviz rviz 
```

# RUN
```
roslaunch usar_exploration exploration.launch 
rosrun usar_exploration current_view_extraction
```

# RUN using rrt package 
Choose the utility function in exploration.yaml located at /home/reem/catkin_ws/src/semantic_based_exploration/rrt_explorer/resource 

```
roslaunch rrt_explorer exploration_planner_2.launch
roslaunch rrt_explorer nav_2.launch 
rosrun usar_exploration current_view_extraction
```

