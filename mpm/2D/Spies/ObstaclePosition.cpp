#include "ObstaclePosition.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>

#include <fileTool.hpp>

//#include <factory.hpp>
//static Registrar<Spy, ObstaclePosition> registrar("ObstaclePosition");

void ObstaclePosition::read(std::istream& is) {
  std::string Filename;
  is >> obstacleNumber >> nrec >> Filename;
  nstep = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "Work: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void ObstaclePosition::exec() {}

void ObstaclePosition::record() { file << box->t << " " << box->Obstacles[obstacleNumber]->pos << std::endl; }

void ObstaclePosition::end() {}
