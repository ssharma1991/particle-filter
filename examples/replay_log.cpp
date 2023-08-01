#include "src/particle_filter.h"
#include "src/particle_filter_helper.h"
#include <fstream>
#include <iostream>

std::string kMapPath = "../data/ground_truth_map/wean.dat";
std::string kLogPath = "../data/logs/robotdata1.log";
int kInitNumParticles = 100000;
int kStableNumParticles = 1000; // after 100 observations

void replayLog(std::string map_path, std::string log_path) {
  // load map
  GroundTruthMap map(map_path);

  // create particle filter object
  ParticleFilter particle_filter(kInitNumParticles, map);
  particle_filter.plot(1000);

  // start replaying the specified log
  int num_lidar_obs = 0;
  std::ifstream infile(log_path);
  if (infile.is_open()) {
    for (std::string line; std::getline(infile, line);) {
      char observation_type = line[0];
      std::string observation_data = line.substr(2);

      if (observation_type == 'O') {
        OdometryParser odom_obs(observation_data);
        particle_filter.addOdometry(odom_obs);
      } else if (observation_type == 'L') {
        num_lidar_obs++;
        std::cout << num_lidar_obs << std::flush;
        ScanParser lidar_obs(observation_data);
        // lidar_obs.print();
        particle_filter.addMeasurement(lidar_obs);
        // Resample based on importance
        if (num_lidar_obs < 100) {
          // Use larger number of particles initially
          particle_filter.resample(kInitNumParticles / num_lidar_obs);
        } else {
          particle_filter.resample(kStableNumParticles);
        }
      }
      std::cout << "." << std::flush;
      particle_filter.plot();
    }
    std::cout << "DONE REPLAYING LOG" << std::endl;
  } else {
    std::cout << "WARNING: Failed to open specified Log" << std::endl;
  }
}

int main() {
  replayLog(kMapPath, kLogPath);
  return 0;
}