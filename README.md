# **Self-Driving Car**
# **Project: Kidnapped Vehicle**
# **Sparse Localization Using Particle Filter**

## MK

Overview

Implemented a 2 dimensional particle filter in C++. Particle filter inputs: map, initial localization information (analogous to GPS data), and at each time step observation and control data.

#

[//]: # (Image References)

[image1]: ./Writeup_IV/PF_AlgorithmFlowChart.png "PF_AlgorithmFlowChart"
[image2]: ./Writeup_IV/PF_Initialization.png "PF_Initialization"
[image3]: ./Writeup_IV/PF_MUEqs.png "PF_MUEqs"
[image4]: ./Writeup_IV/PF_NNDAPC.png "PF_NNDAPC"

#
A brief overview of Particle filter algorithm process and implementation details can be accessed at [Link](./PFA.md)

![][image1]
Steps in Particle Filter Algorithm
- Initialization
- Prediction
- Update
- Resampling
- New Particle Set

#
Initialization Step

The most practical way to initialize particles and generate real time output, is to make an initial estimate using GPS input. As with all sensor based operations, this step is impacted by noise.

Particles initialization was implemented by sampling a Gaussian distribution, taking into account Gaussian sensor noise around the initial GPS position and heading estimates. Using the C++ standard library normal distribution and C++ standard library random engine functions to sample positions around GPS measurements.

#
Prediction Step

Post particles initialization, next step would be to predict the vehicle's position. Using motion models predict where the vehicle would be at the next time step, by updating based on yaw rate and velocity, while accounting for Gaussian sensor noise.

Equations for updating x, y and the yaw angle when the yaw rate is not equal to zero:
![][image3]

#
Data Association

Prior to using landmark measurements of the objects around the vehicle to update the belief of vehicle position, need to address the data association problem. Data association is the problem of matching landmark measurements to objects in the real world, like map landmarks.

One of the techniques is the nearest neighborhood. Below are the pros and cons of the nearest neighborhood technique
![][image4]
