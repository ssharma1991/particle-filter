#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include "particle_filter_helper.h"

class Particle {
public:
  double x_, y_, theta_, weight_;

public:
  Particle(double x, double y, double theta, double weight);
  Particle(GroundTruthMap &map);
  void motionModel(float delta_rot_1, float delta_trans, float delta_rot_2,
                   float variance_rot1, float variance_trans,
                   float variance_rot2);
  void observationModel(const GroundTruthMap &map, const ScanParser obs);
  float castSingleRay(float x, float y, float theta, const GroundTruthMap &map);
  void rayPlot(const GroundTruthMap &map, float x, float y, float theta,
               float length);
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
  OdometryParser odom_previous = OdometryParser("");
  bool odom_initialized = false;

public:
  ParticleFilter(int num_particles, GroundTruthMap &map);
  void addOdometry(OdometryParser odom_current);
  void addMeasurement(ScanParser lidar_obs);
  void resample(int new_num_particles);
  void plot(int ms = 1);
};

#endif // PARTICLE_FILTER_HPP_