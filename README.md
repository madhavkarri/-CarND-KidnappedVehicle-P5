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

#
#
Project Code

[`main.cpp`](./CarND-Kidnapped-Vehicle-Project/src/main.cpp)

- This file runs particle filter as well as measure its runtime and calculate the weighted error at each time step.
- Set the uncertainties for the different measurements.
- Next the main function reads in the map data for each time step.

At the end of each time step, calculated and printed out the weighted error.

Finally, after the particle filter has gone through the entire driving sequence, the main function will calculate the runtime for the filter.

#
[`particle_filter.cpp`](./CarND-Kidnapped-Vehicle-Project/src/particle_filter.cpp)

Implemented most of the particle filter code in `particlefilter.cpp`. This file contains all of the implementations of the functions of the particle filter class.

[`init`](./CarND-Kidnapped-Vehicle-Project/src/particle_filter.cpp#L30-L67)

This function takes as input a GPS position, an initial heading estimate, and an array of uncertainties for these measurements.

It then samples from a Gaussian distribution centered around these measurements to initialize all the particles. It initializes all particle weights to 1.

For further details, access files `particle_struct` and `particle_filter.h`

The particle filter class has an internal structure of particles that is updated. So nothing is returned from this function.

`prediction`

This function takes as input the amount of time between time steps, the velocity and yaw rate measurement uncertainties, and the current time step velocity and yaw rate measurements. Using these measurements, it updated each particle's position estimates and accounted for sensor noise by adding Gaussian noise. Added Gaussian noise by sampling from a Gaussion distribution with mean equal to the updated particle position, and standard deviation equal to the standard deviation of the measurements.

data association
The data association function here takes as input two vectors of landmark obs objects. Access the definition for this struct in helperfunctions.h. The first vector is the prediction measurements between one particular particle and all of the map landmarks within sensor range. Th other vector here is the actual landmark measurements gathered from the LIDAR. This function accomplishes nearest neighbor data association and assign each sensor observation the map landmark ID associated with it.

update weights function

This function takes the range of the sensor, the landmark measurement uncertainties, a vector of landmark measurements, and the map landmarks as input.

The first step was to predict measurements to all the map landmarks within sensor range for each particle. Use the predicted landmark measurements, and data association function to associate the sensor measurements to map landmarks. These associations would be necessary to calculate the new weight of each particle by using the multivariate Gaussian probability density function. As a final step, to normalize these weights so that they would be in the range 0 to 1. Use these weights as probabilities for resampling.

resample function.

Used the weights of the particles in the particle filter and c++ standard libraries discrete distribution function to update particles to the Bayesian posterior distribution.

particle filter evaluation

Evaluate the particle filter by calculating the weighted error. This function takes the ground truth position at a particular time step as input and calculates the weighted error of the particle filter using the weights of each particle.

To build and run the code, open a terminal and go to the localization particle filter home directory, and type build.sh and then execute run.sh to run particle filter.
