#include "particle_filter.h"
#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <sstream>

Particle::Particle(double x, double y, double theta, double weight) {
  x_ = x;
  y_ = y;
  theta_ = theta;
  weight_ = weight;
}
Particle::Particle(GroundTruthMap &map) {
  // initialize particle randomly in free space
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distx(map.observed_min_x,
                                         map.observed_max_x);
  std::uniform_real_distribution<> disty(map.observed_min_y,
                                         map.observed_max_y);
  std::uniform_real_distribution<> disttheta(-3.14, 3.14);
  x_ = 0;
  y_ = 0;
  while (map.prob[int(y_)][int(x_)] >= 0.1 or map.prob[int(y_)][int(x_)] < 0) {
    x_ = double(distx(gen));
    y_ = double(disty(gen));
  }
  theta_ = double(disttheta(gen));
  weight_ = 0;
}
void Particle::motionModel(const OdometryParser odom_previous,
                           const OdometryParser odom_current) {
  // Input: Two poses in odometry frame

  // Relative Odometry is transformed into a sequence of three steps: a
  // rotation (delta_rot_1), followed by a straight line motion (delta_trans),
  // and another rotation (delta_rot_2).
  // TODO: This operation is same for all particles. Move to ParticleFilter
  // data structure to calculate only once.
  float delta_rot_1 = atan2(odom_current.y_ - odom_previous.y_,
                            odom_current.x_ - odom_previous.x_) -
                      odom_previous.theta_;
  if (delta_rot_1 + odom_previous.theta_ == 0) {
    // degenerate case- pure rotation, no translation
    delta_rot_1 = 0;
  }
  float delta_trans = sqrt(pow(odom_current.x_ - odom_previous.x_, 2) +
                           pow(odom_current.y_ - odom_previous.y_, 2));
  float delta_rot_2 = odom_current.theta_ - odom_previous.theta_ - delta_rot_1;

  // Calculate posterior distributions
  float kAlpha1 = 0.025; // rotation
  float kAlpha2 = 0.025; // rotation
  float kAlpha3 = 0.4;   // translation
  float kAlpha4 = 0.4;   // translation

  float variance_rot1 =
      kAlpha1 * pow(delta_rot_1, 2) + kAlpha2 * pow(delta_trans, 2);
  float variance_trans = kAlpha3 * pow(delta_trans, 2) +
                         kAlpha4 * pow(delta_rot_1, 2) +
                         kAlpha4 * pow(delta_rot_2, 2);
  float variance_rot2 =
      kAlpha1 * pow(delta_rot_2, 2) + kAlpha2 * pow(delta_trans, 2);

  // Sample from the posterior probability
  float delta_rot_1_hat = delta_rot_1 - sample(0, variance_rot1);
  float delta_trans_hat = delta_trans - sample(0, variance_trans);
  float delta_rot_2_hat = delta_rot_2 - sample(0, variance_rot2);

  // Update particle pose based on obometry
  x_ += delta_trans_hat * cos(theta_ + delta_rot_1_hat);
  y_ += delta_trans_hat * sin(theta_ + delta_rot_1_hat);
  theta_ += delta_rot_1_hat + delta_rot_2_hat;
}
void Particle::observationModel(const GroundTruthMap &map,
                                const ScanParser obs) {
  // find the weight/importance of particle based on obs using the
  // beam_range_finder_model
  float kSigmaHit = 2;     // Measurement Noise (cm)
  float kMaxRange = 8183;  // Maximum Scanner Range (cm)
  float kLamdaShort = 1.5; // Define exponential distribution
  float kWeightHit = 0.25;
  float kWeightShort = 0.25;
  float kWeightMax = 0.25;
  float kWeightRand = 0.25;

  float likelihood = 1;
  // Iterate over each ray
  for (int i = 0; i < 180; i++) {
    // calculate predicted ray length (z_k_star) using raycasting
    // ray thetas are assumed -90 to 90 deg relative to particle pose
    float theta_deg = theta_ + (i - 90);
    float theta = theta_deg * (M_PI / 180.0);
    float predicted_ray_length = castSingleRay(theta, map); // z_k_star
    float observed_ray_length = obs.r_[i];                  // z_k

    // calculate p_hit
    float p_hit = 0;
    if (observed_ray_length > 0 and observed_ray_length < kMaxRange) {
      // p_hit = normalDist(z_k, z_kstar_, sigma_hit)
      p_hit =
          prob(observed_ray_length, predicted_ray_length, pow(kSigmaHit, 2));
    }

    // calculate p_short
    float p_short = 0;
    if (observed_ray_length > 0 and
        observed_ray_length < predicted_ray_length) {
      float normalizer =
          1 / (1 - exp(-1.0 * kLamdaShort * predicted_ray_length));
      float p_short = normalizer * kLamdaShort *
                      exp(-1.0 * kLamdaShort * observed_ray_length);
    }

    // calculate p_max
    float p_max = 0;
    if (observed_ray_length == kMaxRange) {
      p_max = 1;
    }

    // calculate p_rand
    float p_rand = 0;
    if (observed_ray_length > 0 and observed_ray_length < kMaxRange) {
      p_rand = 1 / kMaxRange;
    }

    // calculate the likelihood of observed (z_k) ray length
    float likelihood_single_ray = kWeightHit * p_hit + kWeightShort * p_short +
                                  kWeightMax * p_max + kWeightRand * p_rand;
    likelihood *= likelihood_single_ray;
  }
  weight_ = likelihood;
};

float Particle::castSingleRay(float theta, const GroundTruthMap &map) {
  // Implements the DDA (Digital Differential Analyzer) Algorithm in the
  // Computer Graphics domain. Google `DDA algorithm grid ray` or `ray grid
  // intersection` for useful and relevant links.

  float kMaxDistance = 3000;      // Lidar max range is 30m
  float kOccupiedThreshold = 0.5; // Probability above which cell is occupied

  // calculate direction cosines
  float dir_x = std::cos(theta);
  float dir_y = std::sin(theta);
  // cache some values to avoid recalculation
  float ray_travel_per_unit_x = 1 / dir_x;
  float ray_travel_per_unit_y = 1 / dir_y;

  // calculate index of starting cell and check if it's occupied
  int cell_x = static_cast<int>(x_);
  int cell_y = static_cast<int>(y_);
  if (map.prob[cell_x][cell_y] > kOccupiedThreshold) {
    return 0;
  }

  // initialize step direction and first step (get to first cell edge)
  int step_x, step_y;
  float ray_length_after_step_x, ray_length_after_step_y;
  if (dir_x > 0) {
    step_x = 1;
    ray_length_after_step_x = static_cast<float>(cell_x + 1) - x_;
  } else {
    step_x = -1;
    ray_length_after_step_x = x_ - static_cast<float>(cell_x);
  }
  if (dir_y > 0) {
    step_y = 1;
    ray_length_after_step_y = static_cast<float>(cell_y + 1) - y_;
  } else {
    step_y = -1;
    ray_length_after_step_y = y_ - static_cast<float>(cell_y);
  }

  // raycast untill an occupied cell is found
  bool hit_occupied_cell = false;
  float current_ray_length = 0;
  while (not hit_occupied_cell and current_ray_length < kMaxDistance) {
    // Walk one cell
    if (ray_length_after_step_x < ray_length_after_step_y) {
      // move in x-direction
      cell_x += step_x;
      current_ray_length = ray_length_after_step_x;
      ray_length_after_step_x += ray_travel_per_unit_x;
    } else {
      // move in y-direction
      cell_y += step_y;
      current_ray_length = ray_length_after_step_y;
      ray_length_after_step_y += ray_travel_per_unit_y;
    }

    // ray went out of occupancy grid without hitting an obstacle
    if (cell_x < map.observed_min_x or cell_x > map.observed_max_x or
        cell_y < map.observed_min_y or cell_y > map.observed_max_y) {
      break;
    }

    // check for obstacle
    if (map.prob[cell_x][cell_y] > kOccupiedThreshold) {
      hit_occupied_cell = true;
    }
  }

  if (hit_occupied_cell) {
    return current_ray_length;
  }
  return -1;
}

float Particle::sample(float mean, float variance) {
  // randomly sample from a Normal distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dist(mean, sqrt(variance));
  return dist(gen);
}

float Particle::prob(float x, float mean, float variance) {
  // get the probability density at x for a normal distrubution
  // NOTE: this function returns `probablility density`, NOT probability
  // Probability is always less than or equal to 1. Probability density can be
  // greater than one.
  return exp(-pow(x - mean, 2) / (2 * variance)) / sqrt(2 * M_PI * variance);
}

void Particle::print() {
  std::cout << "(" << x_ << ", " << y_ << ", " << theta_ << "), " << weight_
            << std::endl;
}

ParticleFilter::ParticleFilter(int num, GroundTruthMap &map)
    : num_particles_{num}, map_{map} {
  // initialize a randon point cloud
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_.push_back(Particle(map));
  }
}
void ParticleFilter::addOdometry(OdometryParser odom_obs) {
  if (not odom_initialized) {
    last_odom_obs_ = odom_obs;
    return;
  }
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_[i].motionModel(last_odom_obs_, odom_obs);
    last_odom_obs_ = odom_obs;
  }
}
void ParticleFilter::addMeasurement(ScanParser lidar_obs) {
  // Calculate importance of each particle
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_[i].observationModel(map_, lidar_obs);
  }

  // normalize importance
  float sum_weight = 0;
  for (int i = 0; i < num_particles_; i++) {
    sum_weight += particle_cloud_[i].weight_;
  }
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_[i].weight_ = particle_cloud_[i].weight_ / sum_weight;
  }

  // Resample based on importance
  resample();
}
void ParticleFilter::resample() {
  std::vector<Particle> new_particle_cloud;
  // low variance resampler
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0, 1.0 / num_particles_);
  float r = double(dist(gen));
  float c = particle_cloud_[0].weight_;
  int i = 0;
  for (int m = 0; m < num_particles_; m++) {
    float u = r + (float)m / num_particles_;
    while (u > c) {
      i += 1;
      c += particle_cloud_[i].weight_;
    }
    new_particle_cloud.push_back(particle_cloud_[i]);
    new_particle_cloud[m].weight_ = 1.0 / num_particles_;
  }
  particle_cloud_ = new_particle_cloud;
}
void ParticleFilter::plot() {
  // get occupancy grid image
  cv::Mat image = map_.getImage();

  // plot particles on top
  int kArrowLength = 10;
  cv::Scalar color_red(0, 0, 255);
  for (int i = 0; i < num_particles_; i++) {
    cv::Point start_point(particle_cloud_[i].x_, particle_cloud_[i].y_);
    cv::Point end_point(
        start_point.x + kArrowLength * std::cos(particle_cloud_[i].theta_),
        start_point.y + kArrowLength * std::sin(particle_cloud_[i].theta_));
    cv::arrowedLine(image, start_point, end_point, color_red);
  }

  cv::namedWindow("Ground Truth Map", cv::WINDOW_AUTOSIZE);
  cv::imshow("Ground Truth Map", image);
  cv::waitKey(0);
}