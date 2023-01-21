#include "ObstaclePosition.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>

#include <fileTool.hpp>

void ObstaclePosition::read(std::istream& is) {
  std::string Filename;
  is >> obstacleNumber >> nrec >> Filename;
  nstep = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "ObstaclePosition: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void ObstaclePosition::exec() {}

void ObstaclePosition::record() { file << box->t << " " << box->Obstacles[obstacleNumber]->pos << std::endl; }

void ObstaclePosition::end() {}
