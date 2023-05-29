#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include "particle_filter_helper.h"

class Particle {
public:
  double x_, y_, theta_, weight_;

public:
  Particle(double x, double y, double theta, double weight);
  Particle(GroundTruthMap &map);
  void motionModel(const OdometryParser odom_previous,
                   const OdometryParser odom_current);
  void observationModel(const GroundTruthMap &map, const ScanParser obs);
  float castSingleRay(float theta, const GroundTruthMap &map);
  float sample(float mean, float variance);
  float prob(float x, float mean, float variance);
  void print();

private:
  std::vector<float> predictScan(const GroundTruthMap &map);
};

class ParticleFilter {
private:
  int num_particles_;
  std::vector<Particle> particle_cloud_;
  GroundTruthMap map_;
  OdometryParser last_odom_obs_ = OdometryParser("");
  bool odom_initialized = false;

public:
  ParticleFilter(int num_particles, GroundTruthMap &map);
  void addOdometry(OdometryParser odom_obs);
  void addMeasurement(ScanParser lidar_obs);
  void resample();
  void plot();
};

#endif // PARTICLE_FILTER_HPP_