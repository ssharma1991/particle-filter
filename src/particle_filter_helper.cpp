#include "particle_filter_helper.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>

groundTruthMap::groundTruthMap(std::string path){
    std::ifstream infile(path);
    if (infile.is_open()) {
        //parse the hyper parameters
        std::string line;
        while (std::getline(infile, line) && (line.compare(0, 13, "global_map[0]") != 0)){
            std::stringstream ss(line);
            std::string param;
            int value;
            ss >> param >> value;

            if (param == "robot_specifications->resolution"){
                resolution = value;
            } else if (param == "robot_specifications->autoshifted_x"){
                offset_x = value;
            } else if (param == "robot_specifications->autoshifted_y"){
                offset_y = value;
            }
        }
        {
            std::stringstream ss(line.substr(15));
            ss >> size_y >> size_x;
        }

        //construct and initialize the occupancy grid
        prob = new float*[size_x];
        for (int i = 0; i < size_x; i++){
            prob[i] = new float[size_y];
        }
        observed_min_x = size_x;
        observed_max_x = 0;
        observed_min_y = size_y;
        observed_max_y = 0;

        for(int x = 0; std::getline(infile, line) && x < size_x; x++){
            std::stringstream ss(line);
            for(int y = 0; y < size_y; y++) {
                float val;
                ss >> val;
                if (val >= 0){
                    prob[x][y] = 1 - val;
                    if (x < observed_min_x){
                        observed_min_x = x;
                    } else if (x > observed_max_x){
                        observed_max_x = x;
                    }
                    if (y < observed_min_y){
                        observed_min_y = y;
                    } else if (y > observed_max_y){
                        observed_max_y = y;
                    }
                }
            }
        }
    }
    else {
        std::cout << "WARNING: Failed to open specified Ground Truth Map" << std::endl;
    }
}
groundTruthMap::~groundTruthMap(){
    for (int i = 0; i < size_x; i++){
        delete[] prob[i];
    }
    delete[] prob;
}
void groundTruthMap::plot(){
}

odometryObservation::odometryObservation(std::string data){
    std::stringstream ss(data);
    ss >> x >> y >> theta >> timestamp;
}
void odometryObservation::print(){
    std::cout << "ODOMETRY: timestamp=" << timestamp << ", x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
}

lidarObservation::lidarObservation(std::string data){
    std::stringstream ss(data);
    ss >> x >> y >> theta;
    ss >> xl >> yl >> thetal; 
    for (int i = 0; i<180; i++){
        ss >> r[i];
    }
    ss >> timestamp;
}
void lidarObservation::print(){
    std::cout << "LIDAR:\t timestamp=" << timestamp << std::endl;
    std::cout << "\t x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
    std::cout << "\t xl=" << xl << ", yl=" << yl << ", thetal=" << thetal << std::endl;
    std::cout << "\t raw point cloud =";
    for (auto data : r){
        std::cout << " " << data;
    }
    std::cout << std::endl;
}