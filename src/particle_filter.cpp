#include "particle_filter_helper.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

class Logtool {
    std::string logFilePath, groundTruthMapPath;
  public:
    Logtool() {
        logFilePath = "";
        groundTruthMapPath = "";    
    }
    void addLogFilePath(std::string path){
        logFilePath = std::move(path);
    }
    void addGroundTruthMapPath(std::string path){
        groundTruthMapPath = std::move(path);
    }
    void replayLog(){
        // check paths were specified
        if (logFilePath.empty()){
            std::cout << "WARNING: Replay log path not specified" << std::endl;
            return;
        }
        if (groundTruthMapPath.empty()){
            std::cout << "WARNING: Ground truth map path not specified" << std::endl;
            return;
        }

        // load ground truth map
        GroundTruthMap map(groundTruthMapPath);
        map.make_cv_map();

        // start replaying the specified log
        std::ifstream infile(logFilePath);
        if (infile.is_open()) {
            for (std::string line; std::getline(infile, line);){
                char observation_type = line[0];
                std::string observation_data = line.substr(2);

                if (observation_type == 'O'){
                    OdometryParser odom_obs(observation_data);
                    //odom_obs.print();
                    //particleFilter.addOdometry(odom_obs);
                }
                else if (observation_type == 'L'){
                    ScanParser lidar_obs(observation_data);
                    lidar_obs.print();
                    map.cast_rays(lidar_obs.r, lidar_obs.xl, lidar_obs.yl, lidar_obs.theta);
                    map.display_cv_map();
                    //lidar_obs.print();
                    //particleFilter.addMeasurement(lidar_obs);
                }
            }
        } else {
            std::cout << "WARNING: Failed to open specified Log" << std::endl;
        }
    }
};

int main(){
    Logtool logtool;
    logtool.addLogFilePath("../logs/robotdata1.log");
    logtool.addGroundTruthMapPath("../ground_truth_map/wean.dat");
    logtool.replayLog();
    return 0;
}