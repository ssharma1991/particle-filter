#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class groundTruthMap {
    int resolution, size_x, size_y;
	float offset_x, offset_y;
	int observed_min_x, observed_max_x, observed_min_y, observed_max_y; //specify useful part of occupancy grid
	float** prob;
    //  -1  = don't know
    //  any value in [0;1] is a probability for occupancy:
    //      1   = occupied with probability 1
    //      0   = unoccupied with probability 1
    //      0.5 = occupied with probability 0.5
  public:
    groundTruthMap(std::string path){
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
    ~groundTruthMap(){
        for (int i = 0; i < size_x; i++){
            delete[] prob[i];
        }
        delete[] prob;
    }
    void plot(){

    }
};

class odometryObservation {
    double x, y, theta; //coordinates of the robot in standard odometry frame (cm, cm, rad)
    double timestamp;   //timestamp of odometry reading (0 at start of run) (sec)
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
    //The laser on the robot is approximately 25 cm offset forward from the true center of the robot.
    double x, y, theta; //coodinates of the robot in standard odometry frame when laser reading was taken (interpolated) (cm, cm, rad)
    double xl, yl, thetal; //coordinates of the *laser* in standard odometry frame when the laser reading was taken (interpolated) (cm, cm, rad)
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