# Dependencies
## Acados 
Acados can be installed by following their [documentation](https://docs.acados.org/installation/index.html).

## ROS 2 Humble
ROS 2 Humble can be found [here](https://docs.ros.org/en/humble/Installation.html).

## Crazyswarm2 
Setup the crazyswarm2 package.Official documentation about setting up this package can be found [here](https://imrclab.github.io/crazyswarm2/installation.html). Here clone our  modified version of Crazyswarm2 package with attitude control.

## Motion Capture Pacakge

# Crazyflie_MPC Setup and Launch Instructions

This guide provides step-by-step instructions to set up the  environment and launch the necessary nodes for simulation and control.

## Prerequisites

Ensure you have the following dependencies installed:
- ROS 2
- ACADOS library

## Setup

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
### Launch the Crazyflie Multi-Agent MPC

if needed modify the Crazyflie counts in crazyflie_multiagent_mpc.py N_AGENTS

Launch the multi-agent model predictive control (MPC):
```sh
ros2 launch crazyflie_mpc crazyflie_multiagent_mpc_launch.py
```

### Hover Command

Publish a hover command to all drones for testing:
```sh
ros2 topic pub /all/mpc_hover std_msgs/msg/Empty "{}"
```

## Notes

- Ensure all dependencies are correctly installed and paths are set properly.
- Refer to the respective documentation for any issues related to ROS 2, Crazyflie, or ACADOS.

## Contribution

Feel free to open issues or contribute to this project. Your feedback and contributions are welcome!

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
```
