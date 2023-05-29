#include "particle_filter_helper.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

GroundTruthMap::GroundTruthMap(std::string path) {
  std::ifstream infile(path);
  if (infile.is_open()) {
    // parse the hyper parameters
    std::string line;
    while (std::getline(infile, line) &&
           (line.compare(0, 13, "global_map[0]") != 0)) {
      std::stringstream ss(line);
      std::string param;
      int value;
      ss >> param >> value;

      if (param == "robot_specifications->resolution") {
        resolution = value;
      } else if (param == "robot_specifications->autoshifted_x") {
        offset_x = value;
      } else if (param == "robot_specifications->autoshifted_y") {
        offset_y = value;
      }
    }
    {
      std::stringstream ss(line.substr(15));
      ss >> size_y >> size_x;
    }

    // construct and initialize the occupancy grid
    prob = new float *[size_y]; // first index is row, aka y coordinate
    for (int y = 0; y < size_y; y++) {
      prob[y] = new float[size_x];
    }
    observed_min_x = size_x;
    observed_max_x = 0;
    observed_min_y = size_y;
    observed_max_y = 0;

    for (int y = 0; std::getline(infile, line) && y < size_y; y++) {
      std::stringstream ss(line);
      for (int x = 0; x < size_x; x++) {
        float val;
        ss >> val;
        if (val >= 0) {
          prob[y][x] = 1 - val;
          if (y < observed_min_y) {
            observed_min_y = y;
          } else if (y > observed_max_y) {
            observed_max_y = y;
          }
          if (x < observed_min_x) {
            observed_min_x = x;
          } else if (x > observed_max_x) {
            observed_max_x = x;
          }
        } else {
          prob[y][x] = val;
        }
      }
    }
  } else {
    std::cout << "WARNING: Failed to open specified Ground Truth Map"
              << std::endl;
  }
}
GroundTruthMap::GroundTruthMap(const GroundTruthMap &map) {
  resolution = map.resolution;
  size_x = map.size_x;
  size_y = map.size_y;
  offset_x = map.offset_x;
  offset_y = map.offset_y;
  observed_min_x = map.observed_min_x;
  observed_max_x = map.observed_max_x;
  observed_min_y = map.observed_min_y;
  observed_max_y = map.observed_max_y;
  prob = new float *[size_y]; // first index is row, aka y coordinate
  for (int y = 0; y < size_y; y++) {
    prob[y] = new float[size_x];
    for (int x = 0; x < size_x; x++) {
      prob[y][x] = map.prob[y][x];
    }
  }
}
void GroundTruthMap::swap(GroundTruthMap &first, GroundTruthMap &second) {
  std::swap(first.resolution, second.resolution);
  std::swap(first.size_x, second.size_x);
  std::swap(first.size_y, second.size_y);
  std::swap(first.observed_min_x, second.observed_min_x);
  std::swap(first.observed_max_x, second.observed_max_x);
  std::swap(first.observed_min_y, second.observed_min_y);
  std::swap(first.observed_max_y, second.observed_max_y);
  std::swap(first.prob, second.prob);
}
GroundTruthMap &GroundTruthMap::operator=(GroundTruthMap map) {
  // Using copy-and-swap-idiom to reduce code duplication
  // Note parameter taken in by value
  swap(*this, map);
  return *this;
}
GroundTruthMap::~GroundTruthMap() {
  if (not prob) {
    return;
  }

  for (int y = 0; y < size_y; y++) {
    delete[] prob[y];
  }
  delete[] prob;
}
void GroundTruthMap::plot() {
  cv::namedWindow("Ground Truth Map", cv::WINDOW_AUTOSIZE);
  cv::imshow("Ground Truth Map", getImage());
  cv::waitKey(0);
}
cv::Mat GroundTruthMap::getImage() {
  cv::Scalar color_blue(255, 0, 0);
  cv::Mat image(size_x, size_y, CV_8UC3, color_blue);
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (prob[y][x] >= 0.0) {
        int value = 255 * (1 - prob[y][x]);
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
      }
    }
  }
  return image;
}

OdometryParser::OdometryParser(std::string data) {
  std::stringstream ss(data);
  ss >> x_ >> y_ >> theta_ >> timestamp_;
}
void OdometryParser::print() {
  std::cout << "ODOMETRY: timestamp=" << timestamp_ << ", x=" << x_
            << ", y=" << y_ << ", theta=" << theta_ << std::endl;
}

ScanParser::ScanParser(std::string data) {
  std::stringstream ss(data);
  ss >> x_ >> y_ >> theta_;
  ss >> xl_ >> yl_ >> thetal_;
  for (int i = 0; i < 180; i++) {
    ss >> r_[i];
  }
  ss >> timestamp_;
}
void ScanParser::print() {
  std::cout << "LIDAR:\t timestamp=" << timestamp_ << std::endl;
  std::cout << "\t x=" << x_ << ", y=" << y_ << ", theta=" << theta_
            << std::endl;
  std::cout << "\t xl=" << xl_ << ", yl=" << yl_ << ", thetal=" << thetal_
            << std::endl;
  std::cout << "\t raw point cloud =";
  for (auto data : r_) {
    std::cout << " " << data;
  }
  std::cout << std::endl;
}