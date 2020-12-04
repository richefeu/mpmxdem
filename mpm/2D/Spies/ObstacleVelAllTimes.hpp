#ifndef OBSTACLEVELALLTIMES_HPP
#define OBSTACLEVELALLTIMES_HPP

#include "Spy.hpp"

#include <vec2.hpp>
#include <vector>

#include <slicedRange.hpp>

struct ObstacleVelAllTimes : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  int obstacleNumber;
  //bool touch;
  std::string filename;
  std::ofstream file;
};

#endif /* end of include guard: OBSTACLEVELALLTIMES_HPP */
