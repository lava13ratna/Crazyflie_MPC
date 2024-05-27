# MPC for self-driving car control

## Overview
This section provides a brief introduction to the project conducted as part of the Advanced Control Methods course at Skoltech in 2024. It includes the fundamental objectives of the project, information about the team members, and a link to the final presentation.

- Course: Advanced Control Methods, Skoltech, 2024
- Team Members: Zakhar Yagudin, Artem Myshlyaev, Aibek Akhmetkazy
- Final Presentation: [Link to Presentation]

![](structure.png)
---

## Table of Contents

- [Overview](#overview)
- [Problem Statement](#problem-statement)
- [Results](#results)
- [Run the Project](#run-the-project)
- [References](#references)

---

## Problem Statement
The number of car crashes and accidents are rising every year. The goal of this project work is to decrease these traffic fatalities by improving tracking accuracy and ensuring safety constraints using Model Predictive Control for self-driving cars.

[//]: # (This section delves into the specifics of the challenge tackled during the project. It provides context, outlines the objectives, and discusses the significance of the problem.)

[//]: # (### Subsection &#40;if any&#41;)
[//]: # (Subsections may be added to further break down the problem, provide background information, or elaborate on specific aspects that are crucial to understanding the project's scope.)

---

## Results
[//]: # (Below should be links to gifs)

Simulation of FMPC implementation for the car and visual comparison on a desired trajectory:
![](mpc.gif)

Animation of a car movement on a different trajectory with obstacles:
![](obstacle.gif)

Animation of a car movement on a different track without obstacles:
![](track.gif)

Animation of a car movement on a different track with 1 obstacle:
![](track_obstacle.gif)

Animation of a car movement on a different track with 3 obstacles:
![](track_obstacles.gif)

[//]: # (Detailed explanation of the findings, performance metrics, and outcomes of the project. This section may include graphs, tables, and other visual aids to support the results.)

[//]: # (### Subsection &#40;if any&#41;)
[//]: # (Subsections may be used to organize results into categories, discuss different algorithms or methods used, or compare various scenarios within the project.)

---

## Run the Project
Step-by-step instructions on how to replicate the results obtained in this project. This should be clear enough for someone with basic knowledge of the tools used to follow.

### Requirements
List of prerequisites, dependencies, and environment setup necessary to run the project.

### Setup and Installation
Instructions for setting up the project environment, which may include:
- Setting up a virtual environment
- Installing dependencies: `pip install -r requirements.txt`

### Running the Code
Exact commands to execute the project, such as:

```bash
python MPC_SIM.py
```
or alternatively:

```bash
python track.py
```
to launch simulation on the track.

### Documentation
- [PDF](https://www.overleaf.com/read/tjjmdndmnvnt#5d0725)
- [Presentation](AutunomousCar_MPC.pptx)
---

## References
- [Differential Flatness](https://www.cds.caltech.edu/~murray/preprints/mrs95-imece.pdf)
- [Bicycle Model](https://www.researchgate.net/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles)
- [MPC](https://en.wikipedia.org/wiki/Model_predictive_control)
