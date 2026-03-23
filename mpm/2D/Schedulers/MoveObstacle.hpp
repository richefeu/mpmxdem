#ifndef MOVE_OBSTACLE_HPP
#define MOVE_OBSTACLE_HPP

#include "Scheduler.hpp"

#include "vec2.hpp"

#include <vector>

struct MoveObstacle : public Scheduler {
  void read(std::istream &is) override;
  void write(std::ostream &os) override;
  void check() override;

private:
  int groupNumber{0};

  // Piecewise-constant imposed velocity defined by a list of time stamps.
  // For t < times[0]: do nothing. For t in [times[i], times[i+1]): impose vels[i].
  // For t >= times.back(): impose vels.back().
  std::vector<double> times;
  std::vector<vec2r> vels;
};

#endif /* end of include guard: MOVE_OBSTACLE_HPP */
