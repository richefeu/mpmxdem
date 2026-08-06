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
 
  // In visualisation mode -- see, cut -- MPMbox::read runs this very function on
  // the conf-file, and opening the output file in write mode TRUNCATES it. The
  // results of a computation must survive being looked at.
  if (box->computationMode == true) { file.open(filename.c_str()); }
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
  if (file.is_open() == false) { return; }
  file << box->t << " " << KinEnergyTot << std::endl;
}

void ElasticBeamDev::end() {
  // file.close();
}