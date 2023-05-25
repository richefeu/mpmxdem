#include "RemoveObstacle.hpp"

#include "Core/MPMbox.hpp"
#include "Obstacles/Obstacle.hpp"

void RemoveObstacle::read(std::istream& is) {
  is >> groupNumber >> removeTime;
}

void RemoveObstacle::write(std::ostream& os) {
  os << "RemoveObstacle " << groupNumber << ' ' << removeTime <<  '\n';
}

void RemoveObstacle::check() {
  if (removeTime >= box->t && removeTime <= box->t + box->dt) {
    std::vector<Obstacle*> Obs_swap;
    for (size_t i = 0; i < box->Obstacles.size(); i++) {
      if (box->Obstacles[i]->group == groupNumber) {
        delete (box->Obstacles[i]);
      } else {
        Obs_swap.push_back(box->Obstacles[i]);
      }
    }
    Obs_swap.swap(box->Obstacles);
  }
}