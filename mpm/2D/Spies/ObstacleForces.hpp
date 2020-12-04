#ifndef OBSTACLEFORCES_HPP
#define OBSTACLEFORCES_HPP

#include "Spy.hpp"

#include <vec2.hpp>
#include <vector>

#include <slicedRange.hpp>

struct ObstacleForces : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  int obstacleNumber;
  std::vector<vec2r> pos, force;
};

#endif /* end of include guard: OBSTACLEFORCES_HPP */
