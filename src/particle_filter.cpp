#include "particle_filter.h"
#include <fstream>
#include <iostream>
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
                                const ScanParser obs){
    // TODO:
    // find the weight/importance of particle based on obs given the pose
    // (x,y,theta)
};
void Particle::print() {
  std::cout << "(" << x_ << ", " << y_ << ", " << theta_ << "), " << weight_
            << std::endl;
}

ParticleFilter::ParticleFilter(int num, GroundTruthMap &map) {
  num_particles_ = num;
  map_ = map;

  // initialize a randon point cloud
  // TODO: optimize by using free space only
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0, 800);
  for (int i = 0; i < num_particles_; i++) {
    particle_cloud_.push_back(Particle(double(dist(gen)), dist(gen), dist(gen),
                                       1.0 / num_particles_));
  }
}
void ParticleFilter::addOdometry(OdometryParser odom_obs) {
  // TODO:
  // Loop over each particle
  // MotionModel(obs)
}
void ParticleFilter::addMeasurement(ScanParser lidar_obs) {
  // TODO:
  // Loop over each particle
  // ObservationModel(obs)

  // After updating importance of all particles, resample()
}
void ParticleFilter::resample() {
  // TODO:
  // Algo to resample
  // Create new pointcloud
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
  ground_truth_map_path = map_path;
  log_file_path = log_path;
}
void Logtool::replayLog() {
  // load map
  GroundTruthMap map(ground_truth_map_path);
  // map.plot();

  // create particle filter object
  ParticleFilter particle_filter(kNumParticles, map);
  particle_filter.plot();

  // start replaying the specified log
  std::ifstream infile(log_file_path);
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