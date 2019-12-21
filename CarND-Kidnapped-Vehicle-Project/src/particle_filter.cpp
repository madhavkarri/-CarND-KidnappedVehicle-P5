/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Modified on: Sep, 2019
 * Author: MK
 */

#include "particle_filter.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::normal_distribution;
using std::numeric_limits;
using std::string;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  if(is_initialized) {
    return;
  }
  
  std::default_random_engine gen;
  num_particles = 10;  // TODO: Set the number of particles
  
  // This line creates a normal (Gaussian) distribution for x, y, theta
  normal_distribution<double> dist_x(x, std[0]);  
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++)
    {
     // TODO: Sample from these normal distributions like this: 
     //   sample_x = dist_x(gen);
     //   where "gen" is the random engine initialized earlier.
     Particle p;
     p.id = i;
     p.x = dist_x(gen);
     p.y = dist_y(gen);
     p.theta = dist_theta(gen);
     p.weight = 1.0;
     particles.push_back (p); // append Particle to particles vector
     weights.push_back(p.weight); // append weight to weights vector
    }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  
  // create normal (Gaussian) distribution for x, y, theta
  // with zero mean to mimic noise addition
  normal_distribution<double> dist_x(0, std_pos[0]);  
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  for (int i = 0; i < num_particles; i++)
    {
      // avoid dividing by zero
      if (fabs(yaw_rate) < 1E-5)
      {
        // motion model with constant yaw rate
        particles[i].x +=  velocity * delta_t * cos(particles[i].theta);
        particles[i].y +=  velocity * delta_t * sin(particles[i].theta);
      }
      else
      {
        // motion model with change in yaw rate
        particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));                          
        particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
        particles[i].theta += yaw_rate * delta_t;
      }    

      // Gaussian noise addition to movement
      particles[i].x = particles[i].x + dist_x(gen);
      particles[i].y = particles[i].y + dist_y(gen);
      particles[i].theta = particles[i].theta + dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (int i_obs = 0; i_obs < observations.size(); i_obs++)
  {
    double min_diff = numeric_limits<double>::max(); // maximum finite value representable by double type
                                                     // https://en.cppreference.com/w/cpp/types/numeric_limits/max
    int id_val = -1; // An ID that possibly cannot belong to a map landmark
    for (int i_pred = 0; i_pred < predicted.size(); i_pred++)
    {
      double diff = dist(observations[i_obs].x, observations[i_obs].y, predicted[i_pred].x, predicted[i_pred].y);
      if (diff < min_diff)
      {
        min_diff = diff;
        id_val = predicted[i_pred].id;
      }
      observations[i_obs].id = id_val;
    }
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];
  double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
  double weight_sum = 0.0;

  // loop through all particles, 
  // for each particle perform sequence of steps listed below
  // step1: transform observations from vehicle to map co-ordinates
  // step2: filter map landmarks that fall within sensor range
  // step3: associate map landmarks and tranformed observations
  // step4: calculate particle weigths

  for (int i_p = 0; i_p < num_particles; i_p++)
  {
    double x_p = particles[i_p].x;
    double y_p = particles[i_p].y;
    double theta_p = particles[i_p].theta;
    
    // vector to hold transformed observations
    vector<LandmarkObs> TransformObs;
    
    // apply transformation to convert observations from vehicle to map co-ordinates
    for (int i_obs = 0; i_obs < observations.size(); i_obs++)
    {
      double x_map = observations[i_obs].x * cos(theta_p) - observations[i_obs].y * sin(theta_p) + x_p;
      double y_map = observations[i_obs].x * sin(theta_p) + observations[i_obs].y * cos(theta_p) + y_p;
      TransformObs.push_back (LandmarkObs{observations[i_obs].id, x_map, y_map});
    }
    
    // vector to hold map landmarks that falls within sensor range
    vector<LandmarkObs> mml; 
    
    // filter map landmarks that fall within sensor range
    for (int i_ml = 0; i_ml < map_landmarks.landmark_list.size(); i_ml++)
    {
      int ml_idi = map_landmarks.landmark_list[i_ml].id_i;
      double ml_xf = map_landmarks.landmark_list[i_ml].x_f;
      double ml_yf = map_landmarks.landmark_list[i_ml].y_f;
      double p_dist = dist(x_p, y_p, ml_xf, ml_yf);
      if (p_dist <= sensor_range)
      {
        mml.push_back(LandmarkObs{ml_idi, ml_xf, ml_yf});
      }
    }

    // associate filtered map landmarks to transformed observations
    dataAssociation(mml, TransformObs);
    
    // vectors to append particle associations  
    vector<int> association;
    vector<double> sense_x;
    vector<double> sense_y;

    // reset particle weight to 1
    particles[i_p].weight = 1.0;

    // calculate each particle weight
    double pw_x, pw_y, pw_mux, pw_muy, exponent;
    for (int i_tobs = 0; i_tobs < TransformObs.size(); i_tobs++)
    {
      pw_x =  TransformObs[i_tobs].x;
      pw_y =  TransformObs[i_tobs].y;
      for (int i_mml = 0; i_mml < mml.size(); i_mml++)
      {
        // associate map landmarks with transformed observations
        if (mml[i_mml].id == TransformObs[i_tobs].id)
        {
          pw_mux = mml[i_mml].x;
          pw_muy = mml[i_mml].y;
        }
      }

      // Multivariate gaussian
      exponent = (pow(pw_x - pw_mux, 2) / (2 * pow(sig_x, 2))) + (pow(pw_y - pw_muy, 2) / (2 * pow(sig_y, 2)));
      
      // multiply observation to determine particle weight
      particles[i_p].weight = particles[i_p].weight * gauss_norm * exp(-exponent);
      
      // append particle associations
      association.push_back(TransformObs[i_tobs].id);
      sense_x.push_back(pw_x);
      sense_y.push_back(pw_y);
    }
    
    // weights sum
    weight_sum = weight_sum + particles[i_p].weight;
    weights[i_p] = particles[i_p].weight;
    
    // SetAssociation for debugging
    SetAssociations(particles[i_p], association, sense_x, sense_y);
    
  }

  // normalize particle weights
  for (int i = 0; i < num_particles; i++)
  {
    particles[i].weight = particles[i].weight/weight_sum;
    weights[i] = particles[i].weight;
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // random engine initialized
  std::default_random_engine gen;
  
  // maximum weight of particles
  double weight_max = numeric_limits<double>::min(); // minimum finite value represented by double type
  for (int i_p = 0; i_p < num_particles; i_p++)
  {
    if (weights[i_p] > weight_max)
    {
      weight_max = weights[i_p];
    }
  }

  // create uniform distribution to draw uniformly from index of particles
  // https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
  uniform_int_distribution<int> udist_pi(0, num_particles-1);
  
  // create uniform distribution to draw uniformly between 0 and (2* maximum weight of particles)
  // http://www.cplusplus.com/reference/random/uniform_real_distribution/
  uniform_real_distribution<double> udist_pw(0.0, weight_max);

  // initialize particle index drawn from uniform distribution of particle indexes
  int i_p = udist_pi(gen);

  // initialize better variable on weights pie-chart
  double beta = 0.0;

  // create new particle vector
  vector<Particle> new_particles;

  // pie-chart/wheel algorithm
  // loop through total number of particle indexes
  for (int ic = 0; ic < num_particles; ic++)
  {
    beta = beta + 2.0 * udist_pw(gen);
    while (beta > weights[i_p])
    {
      beta = beta - weights[i_p];
      i_p = (i_p + 1)%num_particles;
    }
    
    // collect resampled particle
    new_particles.push_back(particles[i_p]);
  }

  // assign resampled particles to particles
  particles = new_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}