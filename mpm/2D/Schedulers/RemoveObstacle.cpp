#include "RemoveObstacle.hpp"

#include "Core/MPMbox.hpp"
#include "Obstacles/Obstacle.hpp"
#include "Spies/ObstacleTracking.hpp"

void RemoveObstacle::read(std::istream &is) {
  is >> groupNumber >> removeTime;
}

void RemoveObstacle::write(std::ostream &os) {
  os << "RemoveObstacle " << groupNumber << ' ' << removeTime << '\n';
}

void RemoveObstacle::check() {
  if (removeTime >= box->t && removeTime <= box->t + box->dt) {
    const size_t oldSize = box->Obstacles.size();
    std::vector<int> oldToNew(oldSize, -1);

    std::vector<Obstacle *> Obs_swap;
    for (size_t i = 0; i < box->Obstacles.size(); i++) {
      if (box->Obstacles[i]->group == groupNumber) {
        delete (box->Obstacles[i]);
      } else {
        oldToNew[i] = static_cast<int>(Obs_swap.size());
        Obs_swap.push_back(box->Obstacles[i]);
      }
    }
    Obs_swap.swap(box->Obstacles);

    // Remap spies that track obstacles by index so they keep pointing to the same obstacle
    // after the Obstacles[] vector has been compacted.
    for (size_t s = 0; s < box->Spies.size(); ++s) {
      if (auto *ot = dynamic_cast<ObstacleTracking *>(box->Spies[s])) { ot->remapObstacleNumber(oldToNew); }
    }
  }
}