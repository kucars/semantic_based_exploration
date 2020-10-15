#!/bin/bash

read -p "Enter user password please: " -s pass

CATKIN_WS=${HOME}/catkin_ws
CATKIN_SRC=${HOME}/catkin_ws/src

# Install some packages from binaries
echo $pass | sudo -S apt-get install -y ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-rviz-visual-tools
# Required for minkindr
echo $pass | sudo -S apt-get install -y ros-$ROS_DISTRO-tf-conversions
# Useful pkgs
echo $pass | sudo -S apt-get install -y ros-$ROS_DISTRO-rqt-tf-tree ros-$ROS_DISTRO-rqt-graph

if [ ! -d "$CATKIN_WS"]; then
	echo "Creating $CATKIN_SRC ... "
	mkdir -p $CATKIN_SRC
    cd $CATKIN_WS
    catkin init
    catkin config --merge-devel
    catkin config --extend /opt/ros/$ROS_DISTRO
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build
fi

#Adding catkin_simple
if [ ! -d "$CATKIN_SRC/catkin_simple" ]; then
    echo "Cloning the catkin_simple repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/catkin/catkin_simple
    cd catkin_simple
    git checkout 0e62848
else
    echo "catkin_simple already exists. Just pulling ..."
    cd $CATKIN_SRC/catkin_simple
    git pull
    cd ../ 
fi

#Adding eigen_catkin
if [ ! -d "$CATKIN_SRC/eigen_catkin" ]; then
    echo "Cloning the eigen_catkin repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/eigen_catkin
    cd eigen_catkin
    git checkout 00b5eb2
    cd ../
else
    echo "eigen_catkin already exists. Just pulling ..."
    cd $CATKIN_SRC/eigen_catkin
    git pull
    cd ../ 
fi

#Adding mav_comm
if [ ! -d "$CATKIN_SRC/mav_comm" ]; then
    echo "Cloning the mav_comm repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/mav_comm
    cd mav_comm
    git checkout 87b9039
else
    echo "mav_comm already exists. Just pulling ..."
    cd $CATKIN_SRC/mav_comm
    git pull
    cd ../ 
fi


#Adding mav_trajectory_generation
if [ ! -d "$CATKIN_SRC/mav_trajectory_generation" ]; then
    echo "Cloning the mav_trajectory_generation repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/mav_trajectory_generation.git
    cd mav_trajectory_generation
    git checkout ca0b4d4
else
    echo "mav_trajectory_generation already exists. Just pulling ..."
    cd $CATKIN_SRC/mav_trajectory_generation
    git pull
    cd ../ 
fi

#Adding geodetic_utils
if [ ! -d "$CATKIN_SRC/geodetic_utils" ]; then
    echo "Cloning the geodetic_utils repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/geodetic_utils.git
    cd geodetic_utils
    git checkout c619b7f
else
    echo "geodetic_utils already exists. Just pulling ..."
    cd $CATKIN_SRC/geodetic_utils
    git pull
    cd ../ 
fi

#Adding eigen_checks
if [ ! -d "$CATKIN_SRC/eigen_checks" ]; then
    echo "Cloning the eigen_checks repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/eigen_checks.git
    cd eigen_checks
    git checkout 22a6247
else
    echo "eigen_checks already exists. Just pulling ..."
    cd $CATKIN_SRC/eigen_checks
    git pull
    cd ../ 
fi

#Adding gflags_catkin
if [ ! -d "$CATKIN_SRC/gflags_catkin" ]; then
    echo "Cloning the gflags_catkin repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/gflags_catkin.git
    cd gflags_catkin
    git checkout 796fe7b
else
    echo "gflags_catkin already exists. Just pulling ..."
    cd $CATKIN_SRC/gflags_catkin
    git pull
    cd ../ 
fi

#Adding glog_catkin
if [ ! -d "$CATKIN_SRC/glog_catkin" ]; then
    echo "Cloning the glog_catkin repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/glog_catkin.git
    cd glog_catkin
    git checkout a952c89
else
    echo "glog_catkin already exists. Just pulling ..."
    cd $CATKIN_SRC/glog_catkin
    git pull
    cd ../ 
fi

#Adding minkindr
if [ ! -d "$CATKIN_SRC/minkindr" ]; then
    echo "Cloning the minkindr repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/minkindr.git
    cd minkindr
    git checkout bc4503c
else
    echo "minkindr already exists. Just pulling ..."
    cd $CATKIN_SRC/minkindr
    git pull
    cd ../ 
fi

#Adding minkindr
if [ ! -d "$CATKIN_SRC/minkindr" ]; then
    echo "Cloning the minkindr repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/minkindr.git
    cd minkindr
    git checkout bc4503c
else
    echo "minkindr already exists. Just pulling ..."
    cd $CATKIN_SRC/minkindr
    git pull
    cd ../ 
fi

#Adding minkindr_ros
if [ ! -d "$CATKIN_SRC/minkindr_ros" ]; then
    echo "Cloning the minkindr_ros repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/minkindr_ros.git
    cd minkindr_ros
    git checkout 88e0bd4
else
    echo "minkindr_ros already exists. Just pulling ..."
    cd $CATKIN_SRC/minkindr_ros
    git pull
    cd ../ 
fi

#Adding mavros_controllers
if [ ! -d "$CATKIN_SRC/geometric_controller" ]; then
    echo "Cloning the geometric_controller repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/mzahana/geometric_controller.git
else
    echo "mavros_controllers already exists. Just pulling ..."
    cd $CATKIN_SRC/geometric_controller
    git pull
fi

#Adding nlopt
if [ ! -d "$CATKIN_SRC/nlopt" ]; then
    echo "Cloning the nlopt repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/nlopt.git
else
    echo "nlopt already exists. Just pulling ..."
    cd $CATKIN_SRC/nlopt
    git pull
fi

#Adding waypoint_navigator
if [ ! -d "$CATKIN_SRC/waypoint_navigator" ]; then
    echo "Cloning the waypoint_navigator repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/waypoint_navigator.git
    cd waypoint_navigator
    git checkout d63d96f
else
    echo "waypoint_navigator already exists. Just pulling ..."
    cd $CATKIN_SRC/waypoint_navigator
    git pull
fi

#Adding octomap_rviz_plugins
if [ ! -d "$CATKIN_SRC/octomap_rviz_plugins" ]; then
    echo "Cloning the octomap_rviz_plugins repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/OctoMap/octomap_rviz_plugins.git
    cd octomap_rviz_plugins
    git checkout kinetic-devel
    
else
    echo "octomap_rviz_plugins already exists. Just pulling ..."
    cd $CATKIN_SRC/octomap_rviz_plugins
    git pull
fi


#Adding semantic_based_exploration
# if [ ! -d "$CATKIN_SRC/semantic_based_exploration" ]; then
#     echo "Cloning the semantic_based_exploration repo ..."
#     cd $CATKIN_SRC
#     git clone https://github.com/kucars/semantic_based_exploration.git
#     cd semantic_based_exploration
#     git checkout pre_release
# else
#     echo "semantic_based_exploration already exists. Just pulling ..."
#     cd $CATKIN_SRC/semantic_based_exploration
#     git pull
# fi

# Build
cd $CATKIN_WS
catkin build minkindr
catkin build minkindr_ros
catkin build waypoint_navigator
catkin build

# Copy PX4 SITL startup script of the iris_depth_camera model to Frimware folder (if it exists)
if [ -d "$HOME/Firmware/ROMFS/px4fmu_common/init.d-posix" ]; then
    echo "Found PX4 startup scripts folder ..."
    echo "Copying 10017_iris_depth_camera ..."
    cp $CATKIN_SRC/semantic_based_exploration/semantic_exploration/config/10017_iris_depth_camera $HOME/Firmware/ROMFS/px4fmu_common/init.d-posix
    echo "---- Done ----"
else
    echo " Did NOT find PX4 startup scripts folder. Check if PX4 Frimware folder is setup properly."
    echo "Skipping copying iris_depth_camera PX4 startup file."
fi

echo "Updating .bashrc ..."
# Add 'source ~/catkin_ws/devel/setup.bash' in .bashrc
# First delete it if it exists
sed -i '/source ~\/catkin_ws\/devel\/setup.bash/d' $HOME/.bashrc
# Add it after 'source /opt/ros/melodic/setup.bash'
sed -i 's+source /opt/ros/melodic/setup.bash+source /opt/ros/melodic/setup.bash\nsource ~/catkin_ws/devel/setup.bash+g' $HOME/.bashrc
# Add models to Gazebo path
sed -i '/GAZEBO_MODEL_PATH/d' $HOME/.bashrc
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$HOME/catkin_ws/src/semantic_based_exploration/semantic_exploration/models" >> $HOME/.bashrc

echo "Done updating .bashrc ..."