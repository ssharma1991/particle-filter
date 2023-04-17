#ifndef PARTICLE_FILTER_HELPER_HPP_
#define PARTICLE_FILTER_HELPER_HPP_

#include <string>
#include <vector>

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
    groundTruthMap(std::string path);
    ~groundTruthMap();
    void plot();
};

class odometryObservation {
    double x, y, theta; //coordinates of the robot in standard odometry frame (cm, cm, rad)
    double timestamp;   //timestamp of odometry reading (0 at start of run) (sec)
  public:
    odometryObservation(std::string data);
    void print();
};

class lidarObservation {
    //The laser on the robot is approximately 25 cm offset forward from the true center of the robot.
    double x, y, theta; //coodinates of the robot in standard odometry frame when laser reading was taken (interpolated) (cm, cm, rad)
    double xl, yl, thetal; //coordinates of the *laser* in standard odometry frame when the laser reading was taken (interpolated) (cm, cm, rad)
    std::vector<double> r = std::vector<double> (180); //r1 .. r180 - 180 range readings of laser in cm.  The 180 readings span 180 degrees *STARTING FROM THE RIGHT AND GOING LEFT*  Just like angles, the laser readings are in counterclockwise order.
    double timestamp;
  public:
    lidarObservation(std::string data);
    void print();
};

#endif // PARTICLE_FILTER_HELPER_HPP_