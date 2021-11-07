/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <cassert>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  for (auto i = 0; i < num_particles; i++) {
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = weights[i];
    particles[i].id = i; // make particle's identifier a particle's initial position in the particles array
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  double yaw_rate_times_delta_t = 0;
  double velocity_div_yaw_rate = 0;
  
  std::default_random_engine gen;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  // for each particle predict its position and add Gaussian noise
  for (auto &particle : particles) {
    // use different formulas to handle both yaw rate ~ 0.0 and yaw rate !~ 0.0
    if (fabs(yaw_rate) < EPS) {
      // predict without Gaussian noise
      particle.x += velocity * delta_t * cos(particle.theta);
      particle.y += velocity * delta_t * sin(particle.theta);
    } else {
      // predict without Gaussian noise
      yaw_rate_times_delta_t= yaw_rate * delta_t;
      velocity_div_yaw_rate = velocity / yaw_rate;
      particle.x += velocity_div_yaw_rate * (sin(particle.theta + yaw_rate_times_delta_t) - sin(particle.theta));
      particle.y += velocity_div_yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate_times_delta_t));
      particle.theta += yaw_rate_times_delta_t;
    }

    // adding noise
    particle.x += dist_x(gen);
    particle.y += dist_y(gen);
    particle.theta += dist_theta(gen);  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  
   for (auto &observation : observations) {
    // set initial minimal value to maximum possible double
    double min_dist = std::numeric_limits<double>::max();

    // set initial closest particle id to -1 to ensure that the mapping was found for observation
    observation.id = -1;

    // find the closest match
    for (auto const &pred_observation : predicted) {
      double cur_dist = dist(pred_observation.x, pred_observation.y, observation.x, observation.y);

      // update the closest match if found closer particle
      if (cur_dist <= min_dist) {
        min_dist = cur_dist;
        observation.id = pred_observation.id;
      }
    }

    // ensuring that we found a mapping
    assert(observation.id != -1);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {

  for (size_t j = 0; j < particles.size(); j++) {
    Particle const &particle = particles[j];

    //transform observations to particle frame
    vector<LandmarkObs> transformed_observations(observations.size());
    for (size_t i = 0; i < observations.size(); i++) {
      double cos_theta = cos(particle.theta);
      double sin_theta = sin(particle.theta);

      transformed_observations[i].x = particle.x + cos_theta * observations[i].x - sin_theta * observations[i].y;
      transformed_observations[i].y = particle.y + sin_theta * observations[i].x + cos_theta * observations[i].y;
      transformed_observations[i].id = -1;  // set to -1 for unknown landmarks
    }

    //search for landmarks in sensor range
    vector<LandmarkObs> landmarks;
    for (auto const &landmark : map_landmarks.landmark_list) {
      if (dist(particle.x, particle.y, landmark.x_f, landmark.y_f) <= sensor_range) {
        LandmarkObs lm_obs = {
            .id = landmark.id_i,
            .x = static_cast<double>(landmark.x_f),
            .y = static_cast<double>(landmark.y_f),
        };
        landmarks.push_back(lm_obs);
      }
    }

    // associate transformed observations with landmarks
    dataAssociation(landmarks, transformed_observations);

    //update weights for this particle based on transformed observations
    vector<double> observation_probabilities(transformed_observations.size());
    particles[j].weight = 1.0;  
    double x_term = 0;
    double y_term = 0;
    
    for (size_t i = 0; i < observations.size(); i++) {
      LandmarkObs tobs = transformed_observations[i];
      LandmarkObs nearest_landmark = {
          .id = -1,  
          .x = static_cast<double>(map_landmarks.landmark_list[tobs.id - 1].x_f), // landmark indices start at 1
          .y = static_cast<double>(map_landmarks.landmark_list[tobs.id - 1].y_f),
      };

      x_term = pow(tobs.x - nearest_landmark.x, 2.0) / (2 * pow(std_landmark[0], 2.0));
      y_term = pow(tobs.y - nearest_landmark.y, 2.0) / (2 * pow(std_landmark[1], 2.0));

      //compute probabiltiy for observation with multivariate gaussian
      observation_probabilities[i] = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1])) *
                                     exp(-(x_term + y_term));

      particles[j].weight *= observation_probabilities[i];
    }

    // set calculated particle weight in the weights array
    weights[j] = particles[j].weight;
  }

}

void ParticleFilter::resample() {
  
  std::default_random_engine gen;
  std::discrete_distribution<size_t> dist_index(weights.begin(), weights.end());

  vector<Particle> resampled_particles(particles.size());

  for (size_t i = 0; i < particles.size(); i++) {
    resampled_particles[i] = particles[dist_index(gen)];
  }

  particles = resampled_particles;
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