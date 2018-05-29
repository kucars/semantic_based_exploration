# usar_exploration
Urban Seacrh &amp; Rescue (USAR) Robotic Exploration Package 

usar_exploration related tasks implementation

In order run the simulation environment the following Packages are needed: 

- volumetric mapping follow the instruction in the following website to install it 
```
https://github.com/ethz-asl/volumetric_mapping
```


## Installing 
```
$ cd catkin_ws\src
$ git clone http://kuri.kustar.ac.ae:30000/reem.ashour/usar_kuri.git
$ cd ..
$ catkin_make 
```


## Running
To run the main program, run the following commands in two separate terminals:
```
$ roslaunch usar_exploration exploration.launch 
$ rosrun usar_exploration current_view_extraction 
```

## Overview 

### current view extraction node 
This node uses the occlusion culling to estimate the point cloud that can be seen from a current position. 

Subscribers:
- current position 

Publishers:
- original point cloud ( loaded from pcd file)
- current viewed point cloud
- accumulated point cloud from what has been seen 

### exploration node 

This node performs the exploration process and create a volumetric map utilizing the volumetric mapping package functionality. 
Subscribers: 
- pointcloud to camera frame 
- current location 

Publishers:
- occupancy grid (octomap) 
- helper visualization tools 


