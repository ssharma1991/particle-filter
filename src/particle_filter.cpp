#include "particle_filter_helper.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

class Logtool {
    std::string logFilePath, groundTruthMapPath;
  public:
    Logtool() {
        logFilePath = "";
        groundTruthMapPath = "";    
    }
    void addLogFilePath(std::string path){
        logFilePath = path;
    }
    void addGroungTruthMapPath(std::string path){
        groundTruthMapPath = path;
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
        groundTruthMap map(groundTruthMapPath);
        map.plot();

        // start replaying the specified log
        std::ifstream infile(logFilePath);
        if (infile.is_open()) {
            for (std::string line; std::getline(infile, line);){
                char observation_type = line[0];
                std::string observation_data = line.substr(2);

                if (observation_type == 'O'){
                    odometryObservation odom_obs(observation_data);
                    //odom_obs.print();
                    //particleFilter.addOdometry(odom_obs);
                }
                else if (observation_type == 'L'){
                    lidarObservation lidar_obs(observation_data);
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
    logtool.addGroungTruthMapPath("../ground_truth_map/wean.dat");
    logtool.replayLog();
    return 0;
}