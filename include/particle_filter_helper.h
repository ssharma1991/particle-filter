#ifndef PARTICLE_FILTER_HELPER_HPP_
#define PARTICLE_FILTER_HELPER_HPP_

#include <opencv2/core.hpp>
#include <string>
#include <vector>

struct GroundTruthMap {
public:
  int size_x, size_y;                 // dimension of occupancy grid
  int resolution;                     // value a unit cell represents
  int observed_min_x, observed_max_x; // cache explored area of occupancy grid
  int observed_min_y, observed_max_y; // cache explored area of occupancy grid
  float offset_x, offset_y;           // ?? coordinate system offset?
  float **prob;
  //  -1  = don't know
  //  any value in [0;1] is a probability for occupancy:
  //      1   = occupied with probability 1
  //      0   = unoccupied with probability 1
  //      0.5 = occupied with probability 0.5
public:
  GroundTruthMap(std::string path);
  GroundTruthMap(const GroundTruthMap &map);
  void swap(GroundTruthMap &first, GroundTruthMap &second);
  GroundTruthMap &operator=(GroundTruthMap map);
  ~GroundTruthMap();
  float getCoordProb(float coord_x, float coord_y) const;
  float getCellProb(int cell_x, int cell_y) const;
  void plot();
  cv::Mat getImage() const;
};

struct OdometryParser {
public:
  double x_, y_, theta_; // coordinates of the robot in standard odometry frame
                         // (cm, cm, rad)
  double timestamp_; // timestamp of odometry reading (0 at start of run) (sec)
public:
  OdometryParser(std::string data);
  void print();
};

struct ScanParser {
public:
  // The laser on the robot is approximately 25 cm offset forward from the true
  // center of the robot.
  double x_, y_,
      theta_; // coodinates of the robot in standard odometry frame when
              // laser reading was taken (interpolated) (cm, cm, rad)
  double xl_, yl_,
      thetal_; // coordinates of the *laser* in standard odometry frame when the
               // laser reading was taken (interpolated) (cm, cm, rad)
  std::vector<double> r_ = std::vector<double>(
      180); // r1 .. r180 - 180 range readings of laser in cm.  The 180 readings
            // span 180 degrees *STARTING FROM THE RIGHT AND GOING LEFT*  Just
            // like angles, the laser readings are in counterclockwise order.
  double timestamp_;

public:
  ScanParser(std::string data);
  void print();
};

#endif // PARTICLE_FILTER_HELPER_HPP_