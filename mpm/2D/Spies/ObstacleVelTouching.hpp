#ifndef OBSTACLEVELTOUCHING_HPP
#define OBSTACLEVELTOUCHING_HPP

#include "Spy.hpp"

#include <vec2.hpp>
#include <vector>

#include <slicedRange.hpp>

struct ObstacleVelTouching : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  int obstacleNumber;
  bool touch;
  std::string filename;
  std::ofstream file;
};

#endif /* end of include guard: OBSTACLEVELTOUCHING_HPP */
