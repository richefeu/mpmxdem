#include "ObstacleVelTouching.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>

#include <fileTool.hpp>

//#include <factory.hpp>
//static Registrar<Spy, ObstacleVelTouching> registrar("ObstacleVelTouching");

void ObstacleVelTouching::read(std::istream& is) {
  std::string Filename;
  is >> obstacleNumber >> nrec >> Filename;
  nstep = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "Work: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void ObstacleVelTouching::exec() {}

void ObstacleVelTouching::record() {
  touch = false;
  for (size_t nn = 0; nn < box->Obstacles[obstacleNumber]->Neighbors.size(); ++nn) {
    if (box->Obstacles[obstacleNumber]->Neighbors[nn].dn < 0.0) {
      touch = true;
    }
  }

  if (touch == true) {
    file << box->t << " " << norm(box->Obstacles[obstacleNumber]->vel) << " " << box->Obstacles[obstacleNumber]->vel
         << " " << box->Obstacles[obstacleNumber]->vrot << std::endl;
  }
}

void ObstacleVelTouching::end() {}
