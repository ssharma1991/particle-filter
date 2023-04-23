#include "particle_filter_helper.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

GroundTruthMap::GroundTruthMap() { prob_ = nullptr; }
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
        resolution_ = value;
      } else if (param == "robot_specifications->autoshifted_x") {
        offset_x_ = value;
      } else if (param == "robot_specifications->autoshifted_y") {
        offset_y_ = value;
      }
    }
    {
      std::stringstream ss(line.substr(15));
      ss >> size_y_ >> size_x_;
    }

    // construct and initialize the occupancy grid
    prob_ = new float *[size_y_]; // first index is row, aka y coordinate
    for (int x = 0; x < size_x_; x++) {
      prob_[x] = new float[size_x_];
    }
    observed_min_x_ = size_x_;
    observed_max_x_ = 0;
    observed_min_y_ = size_y_;
    observed_max_y_ = 0;

    for (int y = 0; std::getline(infile, line) && y < size_y_; y++) {
      std::stringstream ss(line);
      for (int x = 0; x < size_x_; x++) {
        float val;
        ss >> val;
        if (val >= 0) {
          prob_[y][x] = 1 - val;
          if (y < observed_min_y_) {
            observed_min_y_ = y;
          } else if (y > observed_max_y_) {
            observed_max_y_ = y;
          }
          if (x < observed_min_x_) {
            observed_min_x_ = x;
          } else if (x > observed_max_x_) {
            observed_max_x_ = x;
          }
        } else {
          prob_[y][x] = val;
        }
      }
    }
  } else {
    std::cout << "WARNING: Failed to open specified Ground Truth Map"
              << std::endl;
  }
}
GroundTruthMap::~GroundTruthMap() {
  std::cout << "Destroying map object" << std::endl;
  if (not prob_) {
    return;
  }

  for (int y = 0; y < size_y_; y++) {
    delete[] prob_[y];
  }
  delete[] prob_;
}
void GroundTruthMap::plot() {
  cv::namedWindow("Ground Truth Map", cv::WINDOW_AUTOSIZE);
  cv::imshow("Ground Truth Map", getImage());
  cv::waitKey(0);
}
cv::Mat GroundTruthMap::getImage() {
  cv::Scalar color_blue(255, 0, 0);
  cv::Mat image(size_x_, size_y_, CV_8UC3, color_blue);
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (prob_[y][x] >= 0.0) {
        int value = 255 * (1 - prob_[y][x]);
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