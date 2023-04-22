#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include "particle_filter_helper.h"
#include <string>

class Particle {
private:
  double x = 0, y = 0, theta = 0, weight = 0;

public:
  Particle() = default;
  ~Particle() = default;
  void freeSpaceInitialize(const GroundTruthMap &map);
  void motionModel(const OdometryParser odom);
  void observationModel(const GroundTruthMap &map, const ScanParser obs);
};

class ParticleFilter {
private:
  int num_particles;
  std::vector<Particle> particle_cloud;
  // GroundTruthMap map;
public:
  ParticleFilter(int num_particles);
  void addOdometry(OdometryParser odom_obs);
  void addMeasurement(ScanParser lidar_obs);
  void resample();
};

class Logtool {
private:
  std::string ground_truth_map_path, log_file_path;

public:
  Logtool(std::string map_path, std::string log_path);
  void replayLog();
};

#endif // PARTICLE_FILTER_HPP_