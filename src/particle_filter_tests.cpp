#include "external/eigen/Eigen/Dense"
#include "particle_filter.h"
#include "particle_filter_helper.h"
#include "gtest/gtest.h"
#include <fstream>
#include <iostream>

// This test initializes a particle at known initial position (4143, 3997, 3)
// Then, predicted lidar observation are calculated using raycasting.
// These predicted rays are compared to actual observation.
// Test fails if the difference is above accuracy_threshold
TEST(ParticleFilterTest, castSingleRay) {
  const int accuracy_threshold = 70;
  // Get the first laser reading in log as string
  std::ifstream infile("../../data/logs/robotdata1.log");
  std::string observation_data;
  if (infile.is_open()) {
    for (std::string line; std::getline(infile, line);) {
      char observation_type = line[0];
      observation_data = line.substr(2);
      if (observation_type == 'L') {
        break;
      }
    }
    std::cout << "DONE REPLAYING LOG" << std::endl;
  } else {
    std::cout << "WARNING: Failed to open specified Log" << std::endl;
  }

  // Create data structures
  ScanParser lidar_obs(observation_data);
  GroundTruthMap map("../../data/ground_truth_map/wean.dat");

  // Known approximate initial location (found emperically)
  Particle particle(4161, 3997, 2.98, 1);

  // Known scanner pose in vehicle frame (vcs)
  Eigen::VectorXd scan_loc_vcs{{25, 0, 1}}; // homogenous coordinate: (x, y, 1)
  double scan_theta_vcs = 0;

  // Calculate scanner pose in ground frame (gcs)
  Eigen::MatrixXd vcs2gcs{
      {cos(particle.theta_), -sin(particle.theta_), particle.x_},
      {sin(particle.theta_), cos(particle.theta_), particle.y_},
      {0, 0, 1}};
  Eigen::VectorXd scan_loc_gcs = vcs2gcs * scan_loc_vcs;
  double scan_theta_gcs = particle.theta_ + scan_theta_vcs;
  double scan_x_gcs = scan_loc_gcs(0);
  double scan_y_gcs = scan_loc_gcs(1);

  for (int i = 0; i < 180; i++) {
    // calculate predicted ray length (z_k_star) using raycasting
    // ray thetas are assumed -90 to 90 deg relative to particle pose
    float theta_deg = scan_theta_gcs * (180.0 / M_PI) + (i * 180.0 / 179) - 90;
    float ray_theta = theta_deg * (M_PI / 180.0);
    float predicted_ray_length = particle.castSingleRay(
        scan_x_gcs, scan_y_gcs, ray_theta, map); // z_k_star
    float observed_ray_length = lidar_obs.r_[i]; // z_k
    // std::cout << ray_theta << "\t,\t" << predicted_ray_length << "\t,\t"
    //           << observed_ray_length << std::endl;
    // particle.rayPlot(map, scan_x_gcs, scan_y_gcs, ray_theta,
    //                  predicted_ray_length);

    EXPECT_LT(abs(observed_ray_length - predicted_ray_length),
              accuracy_threshold);
  }
}