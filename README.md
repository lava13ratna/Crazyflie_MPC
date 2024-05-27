# Dependencies
## Acados 
Acados can be installed by following their [documentation](https://docs.acados.org/installation/index.html).

## ROS 2 Humble
ROS 2 Humble can be found [here](https://docs.ros.org/en/humble/Installation.html).

## Crazyswarm2 
Setup the crazyswarm2 package.Official documentation about setting up this package can be found [here](https://imrclab.github.io/crazyswarm2/installation.html). Here clone our  modified version of Crazyswarm2 package with attitude control.


# Crazyflie_MPC Setup and Launch Instructions

This guide provides step-by-step instructions to set up the  environment and launch the necessary nodes for simulation and control.

## Prerequisites

Ensure you have the following dependencies installed:
- ROS 2
- ACADOS library
- crazyswarm2
- Vicon Tracking motion capture system

## Setup
### Setup the crazyflie 
Create object with correct marker positions in vicon tracker.
In the crazyswarm2 package go to config -> crazyflie.yaml file and config your crazyflie radio uri. you can find the uri by call cfclient in the terminal.and change the name of crazyflie as your vicon tracker object.
In the motioncapture.yaml file configure your correct marker positions.

### Activate the Environment

Activate the Python virtual environment(Highly recommended to use python envirment):
```sh
source ~pathto/acados/env/bin/activate
```

### Set Environment Variables
Source the ROS2 workspace
Export the necessary environment variables in bashrc and the current terminal:
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"pathto/acados/lib"
export ACADOS_SOURCE_DIR="pathto/acados"
export PYTHONPATH=pathto/crazyflie-firmware/build:$PYTHONPATH
```

## Launch Simulation

### Launch the Crazyflie Simulation

Start the Crazyflie simulation backend:
```sh
ros2 launch crazyflie launch.py backend:=sim
```

For real drones
```sh
ros2 launch crazyflie launch.py
```

### Launch the multi agent mpc
```sh
ros2 launch crazyflie_mpc crazyflie_multiagent_mpc_launch.py
```

### Checking the topics before execute takeoff
check the available topic names and find the poses
```sh
ros2 topic list
```
check the named drone publishing its positional and orientational data
```sh
ros2 topic echo cf_1/pose
```
### Takeoff Command

Publish a take off command to all drones for testing:
```sh
ros2 topic pub /all/mpc_takeoff std_msgs/msg/Empty "{}"
```
### Find the mpc solution and path and optimized control input from running this topics
check the mpc solution path
```sh
ros2 topic echo cf_1/mpc_solution_path
```
check the optimized control input
```sh
ros2 topic echo cf_1/cmd_attitude_setpoint
```

## Notes

- Ensure all dependencies are correctly installed and paths are set properly.
- Refer to the respective documentation for any issues related to ROS 2, Crazyflie, or ACADOS.
- In all terminal source ros2 and activate the acados.



Feel free to open issues or contribute to this project. Your feedback and contributions are welcome!

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
```
