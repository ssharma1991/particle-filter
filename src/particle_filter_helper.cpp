#include "particle_filter_helper.h"
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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
    prob = new float *[size_x];
    for (int i = 0; i < size_x; i++) {
      prob[i] = new float[size_y];
    }
    observed_min_x = size_x;
    observed_max_x = 0;
    observed_min_y = size_y;
    observed_max_y = 0;

    for (int x = 0; std::getline(infile, line) && x < size_x; x++) {
      std::stringstream ss(line);
      for (int y = 0; y < size_y; y++) {
        float val;
        ss >> val;
        if (val >= 0) {
          prob[x][y] = 1 - val;
          if (x < observed_min_x) {
            observed_min_x = x;
          } else if (x > observed_max_x) {
            observed_max_x = x;
          }
          if (y < observed_min_y) {
            observed_min_y = y;
          } else if (y > observed_max_y) {
            observed_max_y = y;
          }
        }
      }
    }
  } else {
    std::cout << "WARNING: Failed to open specified Ground Truth Map"
              << std::endl;
  }
}
GroundTruthMap::~GroundTruthMap() {
  for (int i = 0; i < size_x; i++) {
    delete[] prob[i];
  }
  delete[] prob;
}
void GroundTruthMap::plot() {
  cv::Mat image_map = cv::Mat::zeros(size_x, size_y, CV_32FC1);
  for (int i = 0; i < image_map.rows; i++) {
    for (int j = 0; j < image_map.cols; j++) {
      if (prob[i][j] >= 0.0)
        image_map.at<float>(i, j) = 1 - prob[i][j];
    }
  }

  cv::namedWindow("Ground Truth Map", cv::WINDOW_AUTOSIZE);
  cv::imshow("Ground Truth Map", image_map);
  cv::waitKey(0);
}

OdometryParser::OdometryParser(std::string data) {
  std::stringstream ss(data);
  ss >> x >> y >> theta >> timestamp;
}
void OdometryParser::print() {
  std::cout << "ODOMETRY: timestamp=" << timestamp << ", x=" << x << ", y=" << y
            << ", theta=" << theta << std::endl;
}

ScanParser::ScanParser(std::string data) {
  std::stringstream ss(data);
  ss >> x >> y >> theta;
  ss >> xl >> yl >> thetal;
  for (int i = 0; i < 180; i++) {
    ss >> r[i];
  }
  ss >> timestamp;
}
void ScanParser::print() {
  std::cout << "LIDAR:\t timestamp=" << timestamp << std::endl;
  std::cout << "\t x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
  std::cout << "\t xl=" << xl << ", yl=" << yl << ", thetal=" << thetal
            << std::endl;
  std::cout << "\t raw point cloud =";
  for (auto data : r) {
    std::cout << " " << data;
  }
  std::cout << std::endl;
}