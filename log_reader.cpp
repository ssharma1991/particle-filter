#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class odometryObservation {
    double x, y, theta; //coordinates of the robot in standard odometry frame
    double timestamp;   //timestamp of odometry reading (0 at start of run)
  public:
    odometryObservation(std::string data){
        std::stringstream ss(data);
        ss >> x >> y >> theta >> timestamp;
    }
    void print(){
        std::cout << "ODOMETRY: timestamp=" << timestamp << ", x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
    }
};

class lidarObservation {
    double x, y, theta; //coodinates of the robot in standard odometry frame when laser reading was taken (interpolated)
    double xl, yl, thetal; //coordinates of the *laser* in standard odometry frame when the laser reading was taken (interpolated)
    std::vector<double> r = std::vector<double> (180); //r1 .. r180 - 180 range readings of laser in cm.  The 180 readings span 180 degrees *STARTING FROM THE RIGHT AND GOING LEFT*  Just like angles, the laser readings are in counterclockwise order.
    double timestamp; 
  public:
    lidarObservation(std::string data){
        std::stringstream ss(data);
        ss >> x >> y >> theta;
        ss >> xl >> yl >> thetal; 
        for (int i = 0; i<180; i++){
            ss >> r[i];
        }
        ss >> timestamp;
    }
    void print(){
        std::cout << "LIDAR:\t timestamp=" << timestamp << std::endl;
        std::cout << "\t x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
        std::cout << "\t xl=" << xl << ", yl=" << yl << ", thetal=" << thetal << std::endl;
        std::cout << "\t raw point cloud =";
        for (auto data : r){
            std::cout << " " << data;
        }
        std::cout << std::endl;
    }
};

class GroundTruthMap {
    
};

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
        // if (groundTruthMapPath.empty()){
        //     std::cout << "WARNING: Ground truth map path not specified" << std::endl;
        //     return;
        // }

        // start replaying the specified log
        std::ifstream infile(logFilePath);
        if (infile.is_open()) {
            for (std::string line; std::getline(infile, line);){
                char observation_type = line[0];
                std::string observation_data = line.substr(2);

                if (observation_type == 'O'){
                    odometryObservation odom_obs(observation_data);
                    odom_obs.print();
                    //particleFilter.addOdometry(odom_obs);
                }
                else if (observation_type == 'L'){
                    lidarObservation lidar_obs(observation_data);
                    lidar_obs.print();
                    //particleFilter.addMeasurement(lidar_obs);
                }
            }
        } else {
            std::cout << "WARNING: Failed to open specified Log" << std::endl;
        }
    }
};

int main(){
    Logtool l;
    l.addLogFilePath("./input_data/log/robotdata1.log");
    //l.addGroungTruthMapPath("");
    l.replayLog();
    return 0;
}