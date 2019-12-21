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
[image5]: ./Writeup_IV/PF_UpdateStep.png "PF_UpdateStep"
[image6]: ./Writeup_IV/PF_MVGD.png "PF_MVGD"

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

#
Update Step

Incorporated velocity and yaw rate measurement inputs into the filter. Next would be to update particle weights based on LIDAR and RADAR readings of landmarks.

![][image5]

The landmark measurements are used to compute the update step. Instead of the feature measurements directly affecting the prediction of the state of the car, the measurements will instead inform the weight of each particle. One way to update the weights of the particles is to use the multivariate Gaussian probability density function for each measurement and combine the likelihoods of all the measurements by taking their product.

![][image6]

This function tells how likely a set of landmark measurements, given predicted state of the car and the assumption that the sensors have Gaussian noise. Under the assumption, each landmark measurement is independent, therefore take the product of the likelihoods over all measurements.

Here, `x_i` represents the ith landmark measurement for one particular particle. `mu_i` represents the predicted measurement for the map landmark corresponding to the ith measurement. `m` is the total number of measurements for one particle.

And finally, sigma is the covariance of the measurement.

The covariance matrix sigma is a symmetric square matrix that contains the variance, or uncertainty, of each variable in the sensor measurement, as well as the covariance, or correlation, between these variables. In the case of lidar, the variables in question would be the `x` and `y` position of the landmark and vehicle coordinates. The diagonal terms of the covariance matrix are the variance of each variable, which is the standard deviation of the variable squared.

Think of the covariance matrix as an inverse matrix of weights. The smaller the diagonal term for a certain variable, the more you can trust this variable in the measurement and the higher the weight we can put on it. The off diagonal terms of the covariance matrix represent the correlation between the two variables.

For the project, it was assumed, variables in the sensor measurement are independent and therefore the off diagonal terms are 0. However, this is often not the case in practice.
After weights update for each particle, resample the particles with probability proportional to these weights.
