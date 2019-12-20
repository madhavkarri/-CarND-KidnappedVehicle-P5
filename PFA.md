#

[//]: # (Image References)

[image1]: ./Writeup_IV/PF_AlgorithmFlowChart.png "PF_AlgorithmFlowChart"
[image2]: ./Writeup_IV/PF_Initialization.png "PF_Initialization"
[image3]: ./Writeup_IV/PF_Prediction.png "PF_Prediction"
[image4]: ./Writeup_IV/PF_Update.png "PF_Update"
[image5]: ./Writeup_IV/PF_Resampling.png "PF_Resampling"
[image6]: ./Writeup_IV/PF_NPS.png "PF_NPS"

#
Particle filter algorithm process and implementation details

Particle Filter Algorithm Steps and Inputs

The flowchart below represents the steps of the particle filter algorithm as well as its inputs.

![][image1]

Code

Steps to implement a particle filter for localizing an autonomous vehicle. The code steps correspond to the steps in the algorithm flow chart, initialization, prediction, particle weight updates, and resampling.

#
Initialization

At the initialization step, estimate position from GPS input. The subsequent steps in the process would refine this estimate to localize vehicle.
![][image2]

#
Prediction

During the prediction step, add control input (yaw rate & velocity) for all particles
![][image3]

#
Update

During the update step, update particle weights using map landmark positions and feature measurements.
![][image4]

# 
Resampling

During resampling, resample M times (M is range of 0 to length_of_particleArray) drawing a particle i (i is the particle index) proportional to its weight.
![][image5]

#
New Particle Set

The new set of particles represents the Bayes filter posterior probability. The final output would be a refined estimate of the vehicles position based on input evidence.
![][image6]

