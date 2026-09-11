#include "ObstacleTracking.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"

void ObstacleTracking::read(std::istream &is) {
  std::string Filename;
  is >> obstacleNumber >> nrec >> Filename;
  nstep    = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;

  // Only open file in computation mode to prevent overwriting during visualization
  std::cout << "ObstacleTracking: filename is " << filename << std::endl;
  if (box->computationMode) { file.open(filename.c_str()); }
}

void ObstacleTracking::exec() {}

void ObstacleTracking::remapObstacleNumber(const std::vector<int> &oldToNew) {
  if (obstacleNumber < 0) { return; }
  const size_t oldIdx = static_cast<size_t>(obstacleNumber);
  if (oldIdx >= oldToNew.size()) {
    obstacleNumber = -1;
    return;
  }
  obstacleNumber = oldToNew[oldIdx];
}

void ObstacleTracking::record() {
  // Only record if we're in computation mode
  if (file.is_open()) {
    if (start){
    file << "#time(s)\t pos.x \t pos.y\t vel.x\t  vel.y\t force.x\t force.y\t acc.x\t acc.y\t" << std::endl;
    start = false;
    }

    if (obstacleNumber < 0) { return; }
    const size_t idx = static_cast<size_t>(obstacleNumber);
    if (idx >= box->Obstacles.size()) { return; }

    file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    file << box->t << ' ' << box->Obstacles[idx]->pos << ' ' << box->Obstacles[idx]->vel << ' '
         << box->Obstacles[idx]->force << ' ' << box->Obstacles[idx]->acc << std::endl;
  }
}

void ObstacleTracking::end() {}
