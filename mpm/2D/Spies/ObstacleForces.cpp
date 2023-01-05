#include "ObstacleForces.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>

#include <fileTool.hpp>

void ObstacleForces::read(std::istream& is) {
  is >> obstacleNumber >> nrec;
  nstep = nrec;
}

void ObstacleForces::exec() {
  /*
  pos.clear();
  force.clear();
  size_t o = obstacleNumber;
  for (size_t nn = 0 ; nn < box->Obstacles[o]->Neighbors.size() ; ++nn) {
          if ( box->Obstacles[o]->Neighbors[nn].dn >= 0.0 ) continue;
          MaterialPoint * mp = &(box->MP[ box->Obstacles[o]->Neighbors[nn].PointNumber ]);
          pos.push_back(mp->pos);
          double vn,vt;
          vec2r n, t;
          box->Obstacles[o]->getContactData(*mp, 0, vn, vt, n, t);
          force.push_back(box->Obstacles[o]->Neighbors[nn].fn * n + box->Obstacles[o]->Neighbors[nn].ft * t);
  }
  */
}

void ObstacleForces::record() {
  /*
  char filename[256];
  snprintf(filename, 256, "%s/obsForce%d.txt", box->result_folder.c_str(), box->step);
  std::ofstream file(filename);

  for (size_t i = 0 ; i < pos.size() ; i++) {
          file << pos[i] << " " << force[i] << std::endl;
  }
  * */
}

void ObstacleForces::end() {}
