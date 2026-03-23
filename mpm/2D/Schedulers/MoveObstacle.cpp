#include "MoveObstacle.hpp"

#include "Core/MPMbox.hpp"
#include "Obstacles/Obstacle.hpp"

#include <sstream>

void MoveObstacle::read(std::istream &is) {
  // Syntax:
  //   Scheduled MoveObstacle <group> t0 vx0 vy0 t1 vx1 vy1 ...
  is >> groupNumber;

  std::string line;
  std::getline(is >> std::ws, line);
  std::istringstream ls(line);

  std::vector<double> nums;
  nums.reserve(16);

  double x{0.0};
  while (ls >> x) { nums.push_back(x); }

  if (!ls.eof()) { Logger::warn("MoveObstacle: could not parse schedule line: '{}'", line); }

  times.clear();
  vels.clear();

  if (nums.empty()) {
    Logger::warn("MoveObstacle: empty schedule (expected: triplets t vx vy)");
    return;
  }

  if (nums.size() % 3 != 0) {
    Logger::warn("MoveObstacle: expected triplets (t vx vy) after group {}, got {} values: '{}'", groupNumber,
                 nums.size(), line);
    return;
  }

  const size_t n = nums.size() / 3;
  times.reserve(n);
  vels.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const size_t base = 3 * i;
    vec2r vi;
    vi.x = nums[base + 1];
    vi.y = nums[base + 2];
    times.push_back(nums[base]);
    vels.push_back(vi);
  }
}

void MoveObstacle::write(std::ostream &os) {
  os << "MoveObstacle " << groupNumber;
  for (size_t i = 0; i < times.size(); ++i) { os << ' ' << times[i] << ' ' << vels[i].x << ' ' << vels[i].y; }
  os << '\n';
}

void MoveObstacle::check() {
  const double t = box->t;

  if (times.empty()) { return; }

  // Before the first time mark: keep initial velocity.
  if (t < times.front()) { return; }

  // Find last i such that times[i] <= t.
  size_t idx = 0;
  for (size_t i = 1; i < times.size(); ++i) {
    if (t < times[i]) { break; }
    idx = i;
  }

  const vec2r imposed = vels[idx];
  for (size_t o = 0; o < box->Obstacles.size(); ++o) {
    if (box->Obstacles[o]->group == groupNumber) { box->Obstacles[o]->vel = imposed; }
  }
}
