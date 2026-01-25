#include "ObstacleTracking.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"

void ObstacleTracking::read(std::istream& is) {
  std::string Filename;
  is >> obstacleNumber >> nrec >> Filename;
  nstep = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;
  
  // Only open file in computation mode to prevent overwriting during visualization
  if (box->computationMode) {
    file.open(filename.c_str());
  }
}

void ObstacleTracking::exec() {}

void ObstacleTracking::record() {
  // Only record if we're in computation mode
  if (file.is_open()) {
    file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    file << box->t << ' ' << box->Obstacles[obstacleNumber]->pos << ' ' << box->Obstacles[obstacleNumber]->vel << ' '
         << box->Obstacles[obstacleNumber]->force << std::endl;
  }
}

void ObstacleTracking::end() {}
