# Semantic Based Exploration
This repository aims to provide a simulation environment of UAV system for autonomous exploration with different semantic mapping algoritgms.

# Installation 
This setup is tested with:
* Ubuntu 18
* ROS Melodic
* CUDA 10.1
* PX4 v1.10.1

We provided a docker image which includes all the neccessary packages already installed. This is the fastest way to run and test this package.

## Docker image installation
First, you need to install docker. You can use the [setup_docker.sh](https://github.com/kucars/semantic_based_exploration/blob/pre_release/setup_docker.sh) script for that.
```sh
# Make it executable
chmod +x setup_docker.sh 
# Run
./setup_docker.sh
# logout and login to apply changes
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

You can then run the exploration and semantic mapping setup by running the following commands from the terminal of the container.

### Semantic mapping
* First make sure that the neural network model `pspnet_50_ade20k.pth` is copied to `~/catkin_ws/src/semantic_cloud/models_trained/` folder. You can download the model from [this link](https://drive.google.com/drive/folders/1yS92J8LU2PChqqeGUxb0km3lA0yBF7tg?usp=sharing), and extract the model from the `.zip` file.

```sh
roslaunch semantic_exploration semantic_explorer.launch
```

### Risk-based semantic mapping
* The pre-trained model is already included in the `semantic_hazard_cloud` package. You can directly run the following command.
```sh
roslaunch semantic_exploration semantic_risk_explorer.launch
```

## Manual installation
This invloves several steps including the installation of 
* PX4 1.10.1, or PX4 v1.11.2
* [semantic_hazard_cloud](https://github.com/kucars/semantic_hazard_cloud) and its dependencies
* Other dependencies, see the installation script in this package (https://github.com/kucars/semantic_based_exploration/blob/pre_release/install/setup_semantic_mapping.sh)

# Citation
* If you find this code of semantic mapping useful, please cite this in your work.

  ```
  @article{ashour2020exploration,
    title={Exploration for Object Mapping Guided by Environmental Semantics using UAVs},
    author={Ashour, Reem and Taha, Tarek and Dias, Jorge Manuel Miranda and Seneviratne, Lakmal and Almoosa, Nawaf},
    journal={Remote Sensing},
    volume={12},
    number={5},
    pages={891},
    year={2020},
    publisher={Multidisciplinary Digital Publishing Institute}
  }
  ```

* If you find this code of semantic hazard mapping useful, please cite this in your work.

  [TODO]
