#include "ElasticBeamDev.hpp"
// -> rename ElasticBeamDev.hpp

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
//#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"

void ElasticBeamDev::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename;
  nstep = nrec;

  
  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "KinTotal: filename is " << filename << std::endl;
 
  file.open(filename.c_str());
  file << std::scientific << std::setprecision(15);
}

void ElasticBeamDev::exec() {
  KinEnergyTot = 0.0;
  
  for (size_t p = 0; p < box->MP.size(); p++) {
    // il faudra faire un lissage pour les vitesses
    // mais pour un premier essais on prend la valeur telle qu'elle est.
    vec2r vel = box->MP[p].vel;
    KinEnergyTot += 0.5*box->MP[p].mass * vel * vel;
  }
}

void ElasticBeamDev::record() {
  file << box->t << " " << KinEnergyTot << std::endl;
}

void ElasticBeamDev::end() {
  // file.close();
}