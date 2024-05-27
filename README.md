# From Ground to Air: Crazyflie Drone Takeoff with Model Predictive Control

## Table of Contents
1. [Overview](#overview)
2. [Problem Statement](#problem_statement)
3. [Results](#results)
4. [Running the Project](#running_project)
5. [Appendix](#appendix)
6. [Bibliography](#bibliography)
7. [Directory Structure](#directory_struct)

## Overview <a id="overview"></a>
This section provides a brief introduction to the project conducted as part of the Advanced Control Methods course at Skoltech in 2024. It includes the fundamental objectives of the project, information about the team members, and a link to the final presentation.

- Course: Advanced Control Methods, Skoltech, 2024
- Team Members: Lavanya Ratnabala , Selamawit Asfaw
- Final Presentation: [MPC Takeoff](https://docs.google.com/presentation/d/1lm82zUs13wzlbzYoQ151rFkZ4N4f6DnRfFaLImc0srU/edit?usp=sharing)

### Why Takeoff is important
In the last decade, drones have gained immense popularity, revolutionizing sectors like agriculture, surveillance, delivery, and entertainment with their efficiency, precision, and automation. However  takeoff phase involves complex dynamics due to the interactions between the UAV’s propulsion system and environmental factors like wind and turbulence.  Improper takeoff can lead to crashes or unstable flight paths, posing safety risks to nearby people and structures and potential mission failure. 

## Problem Statement <a id="problem_statement"></a>
Traditional control methods like PID controllers often struggle with the precision and proactive adjustments required during the takeoff phase of drone flights, particularly in handling dynamic external forces such as wind and turbulence. These external forces can introduce instabilities that are not effectively anticipated by reactive control systems, leading to potential safety risks and inefficiencies during takeoff. Model Predictive Control (MPC) to manage the Crazyflie drone's takeoff phase. MPC enhances takeoff stability and accuracy by using a dynamic model to predict future states and adjust controls in real-time. 


## Results <a id="results"></a>
The implemented MPC controller successfully worked on real drones using motion capture and VICON tracking stabilizes the drone at the desired takeoff point:

-By leveraging MPC, the drone is able to proactively predict and achive the takesoff. Additionally, the use of MPC reduces the occurrence of instabilities and enhances the safety and reliability of the takeoff process.
-Empirical data shows a marked small deviation from the planned takeoff path and a more consistent achievement of desired altitude and orientation targets.


### Visual Results
We conduct this testing  in real drone. Here we attach our results from ros2.

1. **MPC Response for point stabilization with terminal cost**
   
   ![Point Stabilization for N = 6 with terminal cost](results/point_stable_n_6_tp.gif)

2. **Trajectory Tracking to pass through gates**
   
   ![Passing through Gates trajectory](results/trajectory.jpeg)
   
3. **Obstalce Avoidance (2D View)**

   ![Obstacle Avoidance](results/drone_obs_avoid.gif)

## Dependencies
### Acados 
Acados can be installed by following their [documentation](https://docs.acados.org/installation/index.html).

### ROS 2 Humble
ROS 2 Humble can be found [here](https://docs.ros.org/en/humble/Installation.html).

### Crazyswarm2 
Setup the crazyswarm2 package.Official documentation about setting up this package can be found [here](https://imrclab.github.io/crazyswarm2/installation.html). Here clone our  modified version of Crazyswarm2 package with attitude control.


## Crazyflie_MPC Setup and Launch Instructions

This guide provides step-by-step instructions to set up the  environment and launch the necessary nodes for simulation and control.

### Prerequisites

Ensure you have the following dependencies installed:
- ROS 2
- ACADOS library
- crazyswarm2
- Vicon Tracking motion capture system

### Setup
#### Setup the crazyflie 
Create object with correct marker positions in vicon tracker.
In the crazyswarm2 package go to config -> crazyflie.yaml file and config your crazyflie radio uri. you can find the uri by call cfclient in the terminal.and change the name of crazyflie as your vicon tracker object.
In the motioncapture.yaml file configure your correct marker positions.

#### Activate the Environment

Activate the Python virtual environment(Highly recommended to use python envirment):
```sh
source ~pathto/acados/env/bin/activate
```

#### Set Environment Variables
Source the ROS2 workspace
Export the necessary environment variables in bashrc and the current terminal:
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"pathto/acados/lib"
export ACADOS_SOURCE_DIR="pathto/acados"
export PYTHONPATH=pathto/crazyflie-firmware/build:$PYTHONPATH
```

### Launch Simulation

#### Launch the Crazyflie Simulation

Start the Crazyflie simulation backend:
```sh
ros2 launch crazyflie launch.py backend:=sim
```

For real drones
```sh
ros2 launch crazyflie launch.py
```

#### Launch the multi agent mpc
```sh
ros2 launch crazyflie_mpc crazyflie_multiagent_mpc_launch.py
```

#### Checking the topics before execute takeoff
check the available topic names and find the poses
```sh
ros2 topic list
```
check the named drone publishing its positional and orientational data
```sh
ros2 topic echo cf_1/pose
```
#### Takeoff Command

Publish a take off command to all drones for testing:
```sh
ros2 topic pub /all/mpc_takeoff std_msgs/msg/Empty "{}"
```
#### Find the mpc solution and path and optimized control input from running this topics
check the mpc solution path
```sh
ros2 topic echo cf_1/mpc_solution_path
```
check the optimized control input
```sh
ros2 topic echo cf_1/cmd_attitude_setpoint
```

### Notes

- Ensure all dependencies are correctly installed and paths are set properly.
- Refer to the respective documentation for any issues related to ROS 2, Crazyflie, or ACADOS.
- In all terminal source ros2 and activate the acados.

## Appendix <a id="appendix"></a>

### System Dynamics and MPC Configuration

The Crazyflie drone's dynamics are governed by a model predictive control (MPC) strategy, designed to handle the complexities of real-time aerial navigation. Below is a detailed breakdown of the system's state variables, control inputs, dynamics model, and the MPC setup.

#### State Variables and Control Inputs

**State Variables (`x`):**
- `px, py, pz`: Position coordinates in the inertial frame (meters).
- `vx, vy, vz`: Velocity components along each axis (meters/second).
- `roll, pitch, yaw`: Euler angles for orientation (radians).

**Control Inputs (`u`):**
- `roll_c, pitch_c, yaw_c`: Commanded angles for roll, pitch, and yaw (radians).
- `thrust`: Commanded thrust (Newtons).

```plaintext
State vector (x): [px, py, pz, vx, vy, vz, roll, pitch, yaw]
Control vector (u): [roll_c, pitch_c, yaw_c, thrust]
```



## Bibliography <a id="bibliography"></a>

1. M. Islam, M. Okasha, and M. M. Idres, “Dynamics and control of quadcopter using linear model predictive control approach,” IOP Conf. Ser. Mater. Sci. Eng., vol. 270, p. 012007, Dec. 2017, doi: 10.1088/1757-899X/270/1/012007.
2. F. Adıgüzel and T. V. Mumcu, “Discrete-time Backstepping Attitude Control of a Quadrotor UAV,” in 2019 International Artificial Intelligence and Data Processing Symposium (IDAP), Sep. 2019, pp. 1–5. doi: 10.1109/IDAP.2019.8875891.
3. “Nonlinear Model Predictive Control of a Quadrotor.” Accessed: May 27, 2024. [Online]. Available: https://upcommons.upc.edu/handle/2117/98503
4. “Applied Sciences | Free Full-Text | A Comparative Study for Control of Quadrotor UAVs.” Accessed: May 27, 2024. [Online]. Available: https://www.mdpi.com/2076-3417/13/6/3464
5. “(2) MPC and MHE implementation in Matlab using Casadi - YouTube.” Accessed: May 27, 2024. [Online]. Available: https://www.youtube.com/playlist?list=PLK8squHT_Uzej3UCUHjtOtm5X7pMFSgAL
6. M. W. Mehrez, “MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi.” May 19, 2024. Accessed: May 27, 2024. [Online]. Available: https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi
7.	“Energies | Free Full-Text | Non-Linear Model Predictive Control Using CasADi Package for Trajectory Tracking of Quadrotor.” Accessed: May 27, 2024. [Online]. Available: https://www.mdpi.com/1996-1073/16/5/2143



