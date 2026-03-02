#include "MoveObstacle.hpp"

#include "Core/MPMbox.hpp"
#include "Obstacles/Obstacle.hpp"

void MoveObstacle::read(std::istream &is) {
  int n{0};
  is >> groupNumber >> n;

  times.clear();
  vels.clear();
  times.reserve(static_cast<size_t>(n));
  vels.reserve(static_cast<size_t>(n));

  for (int i = 0; i < n; ++i) {
    double ti{0.0};
    vec2r vi;
    is >> ti >> vi.x >> vi.y;
    times.push_back(ti);
    vels.push_back(vi);
  }
}

void MoveObstacle::write(std::ostream &os) {
  os << "MoveObstacle " << groupNumber << ' ' << times.size();
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
