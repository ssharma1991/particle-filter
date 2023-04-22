#include "particle_filter.h"
#include <fstream>
#include <iostream>
#include <sstream>

void Particle::freeSpaceInitialize(const GroundTruthMap &map) {}
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

ParticleFilter::ParticleFilter(int num_particles) {
  num_particles = num_particles;
  // initialize a randon point cloud,
  // TODO: optimize by using free space only
  particle_cloud.resize(num_particles);
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

Logtool::Logtool(std::string map_path, std::string log_path) {
  ground_truth_map_path = map_path;
  log_file_path = log_path;
}
void Logtool::replayLog() {
  // load map
  GroundTruthMap map(ground_truth_map_path);
  map.plot();

  // create particle filter object
  // ParticleFilter particleFilter(num_particles);

  // start replaying the specified log
  std::ifstream infile(log_file_path);
  if (infile.is_open()) {
    for (std::string line; std::getline(infile, line);) {
      char observation_type = line[0];
      std::string observation_data = line.substr(2);

      if (observation_type == 'O') {
        OdometryParser odom_obs(observation_data);
        odom_obs.print();
        // particleFilter.addOdometry(odom_obs);
      } else if (observation_type == 'L') {
        ScanParser lidar_obs(observation_data);
        lidar_obs.print();
        // particleFilter.addMeasurement(lidar_obs);
      }
    }
  } else {
    std::cout << "WARNING: Failed to open specified Log" << std::endl;
  }
}