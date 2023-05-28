#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include "particle_filter_helper.h"
#include <string>

class Particle {
public:
  double x_, y_, theta_, weight_;

public:
  Particle(double x, double y, double theta, double weight);
  void motionModel(const OdometryParser odom);
  void observationModel(const GroundTruthMap &map, const ScanParser obs);
  float castSingleRay(float theta, const GroundTruthMap &map);
  void print();

private:
  std::vector<float> predictScan(const GroundTruthMap &map);
};

class ParticleFilter {
private:
  int num_particles_;
  std::vector<Particle> particle_cloud_;
  GroundTruthMap map_;

public:
  ParticleFilter(int num_particles, GroundTruthMap &map);
  void addOdometry(OdometryParser odom_obs);
  void addMeasurement(ScanParser lidar_obs);
  void resample();
  void plot();
};

class Logtool {
private:
  std::string ground_truth_map_path_, log_file_path_;

public:
  Logtool(std::string map_path, std::string log_path);
  void replayLog();
};

#endif // PARTICLE_FILTER_HPP_