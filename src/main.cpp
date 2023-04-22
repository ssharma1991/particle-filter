#include "particle_filter.h"

int main() {
  Logtool logtool("../ground_truth_map/wean.dat", "../logs/robotdata1.log");
  logtool.replayLog();
  return 0;
}