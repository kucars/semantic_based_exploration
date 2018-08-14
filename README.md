# Semantic Based Exploration
The repository aims to provide an exploration algorithm and relevant packages to enable a semantic-aware autonomous exploration task for robots in indoor environments.  


# Building 
create a new workspace and clone all the modules into that workspace.

```
$ mkdir catkin_ws && cd catkin_ws && mkdir src && cd src 
$ git clone https://github.com/kucars/semantic_based_exploration.git
$ git clone https://github.com/reem90/volumetric_mapping.git
$ git clone https://github.com/reem90/octomap.git
$ git submodule update --init --recursive
$ cd /octomap
$ mkdir build 
$ cd build 
$ sudo cmake ..
$ sudo make install 
```

It is important to change the lib directory in volumetric_mapping/octomap_world/CMakeLists.txt
```
from home/kuri/catkin_ws/src/octomap/lib to /Your/Diroctory
```
Then build all
```
$ cd 
$ cd catkin_ws/
$ sudo rm -rf build_isolated/ devel_isolated/
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
$ sudo rm -rf build_isolated/ devel_isolated/
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ catkin build
```

