#include "ObstacleVelAllTimes.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>

#include <fileTool.hpp>

//#include <factory.hpp>
//static Registrar<Spy, ObstacleVelAllTimes> registrar("ObstacleVelAllTimes");

void ObstacleVelAllTimes::read(std::istream& is) {
  std::string Filename;
  is >> obstacleNumber >> nrec >> Filename;
  nstep = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "Work: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void ObstacleVelAllTimes::exec() {}

void ObstacleVelAllTimes::record() {

  file << box->t << " " << norm(box->Obstacles[obstacleNumber]->vel) << " " << box->Obstacles[obstacleNumber]->vel
       << " " << box->Obstacles[obstacleNumber]->vrot << std::endl;
}

void ObstacleVelAllTimes::end() {}
