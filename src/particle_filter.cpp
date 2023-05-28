#include "particle_filter.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <sstream>

int kNumParticles = 1000;

Particle::Particle(double x, double y, double theta, double weight) {
  x_ = x;
  y_ = y;
  theta_ = theta;
  weight_ = weight;
}
void Particle::motionModel(const OdometryParser odom) {
  // TODO:
  // x_(t+1) = x_t + odom + noise
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
      p_hit = (1.0 / (kSigmaHit * sqrt(2 * M_PI))) *
              exp((-(1 / 2) *
                   pow((observed_ray_length - predicted_ray_length) / kSigmaHit,
                       2)));
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
  if (map.prob_[cell_x][cell_y] > kOccupiedThreshold) {
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
    if (cell_x < map.observed_min_x_ or cell_x > map.observed_max_x_ or
        cell_y < map.observed_min_y_ or cell_y > map.observed_max_y_) {
      break;
    }

    // check for obstacle
    if (map.prob_[cell_x][cell_y] > kOccupiedThreshold) {
      hit_occupied_cell = true;
    }
  }

  if (hit_occupied_cell) {
    return current_ray_length;
  }
  return -1;
}

void Particle::print() {
  std::cout << "(" << x_ << ", " << y_ << ", " << theta_ << "), " << weight_
            << std::endl;
}

ParticleFilter::ParticleFilter(int num, GroundTruthMap &map)
    : num_particles_{num}, map_{map} {
  // initialize a randon point cloud
  // TODO: improve initialization by using free space only
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0, 800);
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_.push_back(Particle(double(dist(gen)), dist(gen), dist(gen),
                                       1.0 / num_particles_));
  }
}
void ParticleFilter::addOdometry(OdometryParser odom_obs) {
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_[i].motionModel(odom_obs);
  }
}
void ParticleFilter::addMeasurement(ScanParser lidar_obs) {
  // Calculate importance of each particle
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_[i].observationModel(map_, lidar_obs);
  }

  // Resample based on importance
  resample();
}
void ParticleFilter::resample() {
  // TODO:Algo to resample and create a new pointcloud
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

Logtool::Logtool(std::string map_path, std::string log_path) {
  ground_truth_map_path_ = map_path;
  log_file_path_ = log_path;
}
void Logtool::replayLog() {
  // load map
  GroundTruthMap map(ground_truth_map_path_);
  // map.plot();

  // create particle filter object
  ParticleFilter particle_filter(kNumParticles, map);
  particle_filter.plot();

  // start replaying the specified log
  std::ifstream infile(log_file_path_);
  if (infile.is_open()) {
    for (std::string line; std::getline(infile, line);) {
      char observation_type = line[0];
      std::string observation_data = line.substr(2);

      if (observation_type == 'O') {
        OdometryParser odom_obs(observation_data);
        // odom_obs.print();
        // particle_filter.addOdometry(odom_obs);
      } else if (observation_type == 'L') {
        ScanParser lidar_obs(observation_data);
        // lidar_obs.print();
        // particle_filter.addMeasurement(lidar_obs);
      }
    }
    std::cout << "DONE REPLAYING LOG" << std::endl;
  } else {
    std::cout << "WARNING: Failed to open specified Log" << std::endl;
  }
}