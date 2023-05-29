#include "particle_filter.h"
#include "particle_filter_helper.h"
#include <fstream>
#include <iostream>

std::string kMapPath = "../ground_truth_map/wean.dat";
std::string kLogPath = "../logs/robotdata1.log";
int kNumParticles = 1000;

void replayLog(std::string map_path, std::string log_path) {
  // load map
  GroundTruthMap map(map_path);
  // map.plot();

  // create particle filter object
  ParticleFilter particle_filter(kNumParticles, map);
  particle_filter.plot();

  // start replaying the specified log
  std::ifstream infile(log_path);
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
      std::cout << ".";
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