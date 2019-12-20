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
[image3]: ./Writeup_IV/.png ""

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
Initialization

The most practical way to initialize particles and generate real time output, is to make an initial estimate using GPS input. As with all sensor based operations, this step is impacted by noise.

Particles initialization was implemented by sampling a Gaussian distribution, taking into account Gaussian sensor noise around the initial GPS position and heading estimates. Using the C++ standard library normal distribution and C++ standard library random engine functions to sample positions around GPS measurements.
