# Monte Carlo Localization

## About the Project
This project is a C++ implementation of particle filter for a mobile robot operating with a range sensor and odometry readings in an indoor hallway. The ground truth map is available in the file named `wean.dat` and the odometry readings along with laser scan observations are available in different log files found under `logs` subdirectory. The goal of this project is to implement a Monte Carlo Localization model for helping the lost robot find its way.

## Authors
Arun Kumar
Shashank Sharma

## Dependencies
* Python3
* Numpy
* Matplotlib
* CMake
* C++ 14 Compiler
* Good internet connection for fetching additional packages automatically (by FetchContent in CMakeLists.txt)

## How to build and run?
```
git clone git@github.com:ssharma1991/particle-filter.git
cd particle-filter
mkdir -p build && cd build
cmake .. && make
./particle_filter
```

## References

### Beam measurement models

1. https://people.eecs.berkeley.edu/~pabbeel/cs287-fa11/slides/beam-sensor-model.pdf
2. https://calvinfeng.gitbook.io/probabilistic-robotics/basics/robot-perception/01-beam-models-of-range-finders
3. https://www.cs.cmu.edu/~16831-f14/notes/F12/16831_lecture03_mtaylormshomin.pdf
4. http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/07-sensor-models.pdf
5. https://www.classes.cs.uchicago.edu/archive/2021/spring/20600-1/class_meeting_06.html

### Particle filters
1. [ Particle Filters: A Hands-On Tutorial ](https://www.mdpi.com/955336)
2. [ Mobile Robot Localization - University of Texas ](https://amrl.cs.utexas.edu/interactive-particle-filters/)
