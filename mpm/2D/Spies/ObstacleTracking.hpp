#pragma once

#include "Spy.hpp"

#include "vec2.hpp"
#include <limits>
#include <vector>

struct ObstacleTracking : public Spy {
  void read(std::istream &is);

  void exec();
  void record();
  void end();

  // Called when the Obstacles[] vector is compacted (e.g. after RemoveObstacle).
  // oldToNew[oldIndex] gives the new index, or -1 if the obstacle was removed.
  void remapObstacleNumber(const std::vector<int> &oldToNew);

private:
  int obstacleNumber;
  std::string filename;
  std::ofstream file;
};
