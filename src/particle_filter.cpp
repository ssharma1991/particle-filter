#include "particle_filter_helper.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class Particle {
private:
  double x = 0, y = 0, theta = 0, weight = 0;

public:
  Particle() = default;
  ~Particle() = default;
  void freeSpaceInitialize(const GroundTruthMap &map) {}
  void motionModel(const OdometryParser odom) {
    // TODO:
    // x_(t+1) = x_t + odom + noise
  }
  void observationModel(const GroundTruthMap &map, const ScanParser obs) {
    // TODO:
    // find the weight/importance of particle based on obs given the pose
    // (x,y,theta)
  }
};

class ParticleFilter {
private:
  int num_particles;
  std::vector<Particle> particle_cloud;
  // GroundTruthMap map;

public:
  ParticleFilter(int num_particles) {
    num_particles = num_particles;
    // initialize a randon point cloud,
    // TODO: optimize by using free space only
    particle_cloud.resize(num_particles);
  }
  void addOdometry(OdometryParser odom_obs) {
    // TODO:
    // Loop over each particle
    // MotionModel(obs)
  }
  void addMeasurement(ScanParser lidar_obs) {
    // TODO:
    // Loop over each particle
    // ObservationModel(obs)

    // After updating importance of all particles, resample()
  }
  void resample() {
    // TODO:
    // Algo to resample
    // Create new pointcloud
  }
};

class Logtool {
private:
  std::string logFilePath, groundTruthMapPath;

public:
  Logtool() {
    logFilePath = "";
    groundTruthMapPath = "";
  }
  void addLogFilePath(std::string path) { logFilePath = path; }
  void addGroungTruthMapPath(std::string path) {
    groundTruthMapPath = path;
    GroundTruthMap map(groundTruthMapPath);
    map.plot();
  }
  void replayLog() {
    // check paths were specified
    if (logFilePath.empty()) {
      std::cout << "WARNING: Replay log path not specified" << std::endl;
      return;
    }
    if (groundTruthMapPath.empty()) {
      std::cout << "WARNING: Ground truth map path not specified" << std::endl;
      return;
    }

    // create particle filter object
    // ParticleFilter particleFilter(num_particles);

    // start replaying the specified log
    std::ifstream infile(logFilePath);
    if (infile.is_open()) {
      for (std::string line; std::getline(infile, line);) {
        char observation_type = line[0];
        std::string observation_data = line.substr(2);

        if (observation_type == 'O') {
          OdometryParser odom_obs(observation_data);
          // odom_obs.print();
          // particleFilter.addOdometry(odom_obs);
        } else if (observation_type == 'L') {
          ScanParser lidar_obs(observation_data);
          // lidar_obs.print();
          // particleFilter.addMeasurement(lidar_obs);
        }
      }
    } else {
      std::cout << "WARNING: Failed to open specified Log" << std::endl;
    }
  }
};

int main() {
  Logtool logtool;
  logtool.addLogFilePath("../logs/robotdata1.log");
  logtool.addGroungTruthMapPath("../ground_truth_map/wean.dat");
  logtool.replayLog();
  return 0;
}