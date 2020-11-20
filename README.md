# Semantic Based Exploration
The repository aims to provide an exploration algorithm and relevant packages to enable a semantic-aware autonomous exploration task for robots in indoor environments.  

# Installation 
This setup is tested with:
* Ubuntu 18
* ROS Melodic
* CUDA 10.1
* PX4 v10.1

We prepared a docker image which includes all the neccessary installation steps. This is the fastest way to run and test this package.

## Docker image installation
First, you need to install docker. You can use the [setup_docker.sh](https://github.com/kucars/semantic_based_exploration/blob/pre_release/setup_docker.sh) script for that.
```sh
# Make it executable
chmod +x setup_docker.sh 
# Run
./setup_docker.sh
```

**NOTE** Make sure that you have the latest Nvidia drivers installed on your machine. You can check if it's installed properly using `nvidia-smi` command

Next, run the [semantic_mapping_docker.sh](https://github.com/kucars/semantic_based_exploration/blob/pre_release/semantic_mapping_docker.sh) script.
```sh
# Make it executable
chmod +x semantic_mapping_docker.sh 
# Run
./semantic_mapping_docker.sh sem_mapping
```
`sem_mapping` is an optional name that is passed to the script which will be used as the container name. This script also creates a folder in the `$HOME` folder of the host's machine (named `<container_name>_shared_volume`) which is linked to another folder inside the container's home folder named `shared_volume`. This is useful when you need to exchange files between the container and the host's machine.

You will be logged into the container with its own terminal and user named `arrow` (with password `arrow` in case it's needed). Should look something similar like the following,
```sh
arrow@ffcd644bbfd7:~$ 
```

When you run the `semantic_mapping_docker.sh` for the first time, it will build the `catkin_ws` automatically for you in order to be ready for running the packages.

You can then run the exploration and semantic mapping setup by running the following from the terminal of the container.
```sh
roslaunch semantic_exploration semantic_explorer.launch
```

## Manual installation
This invloves several steps including the installation of 
* PX4 10.1
* [semantic_hazard_cloud](https://github.com/kucars/semantic_hazard_cloud) and its dependencies
* Other dependencies, see the installation script in this package (https://github.com/kucars/semantic_based_exploration/blob/pre_release/install/setup_semantic_mapping.sh)

# Citation
[TODO]
