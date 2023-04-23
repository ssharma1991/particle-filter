#ifndef PARTICLE_FILTER_HELPER_HPP_
#define PARTICLE_FILTER_HELPER_HPP_

#include <opencv2/core.hpp>
#include <string>
#include <vector>

class GroundTruthMap {
private:
  int resolution_, size_x_, size_y_;
  float offset_x_, offset_y_;
  int observed_min_x_, observed_max_x_, observed_min_y_,
      observed_max_y_; // specify useful part of occupancy grid
  float **prob_;
  //  -1  = don't know
  //  any value in [0;1] is a probability for occupancy:
  //      1   = occupied with probability 1
  //      0   = unoccupied with probability 1
  //      0.5 = occupied with probability 0.5
public:
  GroundTruthMap();
  GroundTruthMap(std::string path);
  ~GroundTruthMap();
  void plot();
  cv::Mat getImage();
};

class OdometryParser {
private:
  double x_, y_, theta_; // coordinates of the robot in standard odometry frame
                         // (cm, cm, rad)
  double timestamp_; // timestamp of odometry reading (0 at start of run) (sec)
public:
  OdometryParser(std::string data);
  void print();
};

class ScanParser {
private:
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