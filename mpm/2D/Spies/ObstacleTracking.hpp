#ifndef OBSTACLE_TRACKING_HPP
#define OBSTACLE_TRACKING_HPP

#include "Spy.hpp"

#include "vec2.hpp"
//#include <vector>

//#include <slicedRange.hpp>

struct ObstacleTracking : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  int obstacleNumber;
  std::string filename;
  std::ofstream file;
};

#endif /* end of include guard: OBSTACLE_TRACKING_HPP */
