#include "PBC3D.hpp"
#include <iomanip>

using namespace std;

int main(int argc, char *argv[]) {
  PBC3Dbox box;
  if (argc == 2) {
    box.loadConf(argv[1]);
  } else {
    cerr << "usage: " << argv[0] << " confFilename" << endl;
  }

  ofstream out("REVini.dat");
  out << scientific << setprecision(10);
  out.setf(std::ios::left);
  out.width(20);

  double I = 1e-4; // Maximum inertial number for driving the cell

  //  out << box.kn << " " << box.kt << " " << box.mu << " " << I << endl; // AKN AKT AFRIC ARI
  out << box.kn << " " << box.kt << " " << box.mu << " " << box.density << " " << box.dampRate << " " << box.Cell.mass
      << " " << I << " " << box.fcoh << endl; // AKN AKT AFRIC ADEN ADAMP AMASS ARI FCOH
  out << box.Particles.size() << endl;
  for (size_t i = 0; i < box.Particles.size(); i++) {
    out << box.Particles[i].pos.x << " " << box.Particles[i].pos.y << " " << box.Particles[i].pos.z << " "
        << box.Particles[i].radius << endl;
  }

  out << box.Cell.h.xx << " " << box.Cell.h.xy << " " << box.Cell.h.xz << endl;
  out << box.Cell.h.yx << " " << box.Cell.h.yy << " " << box.Cell.h.yz << endl;
  out << box.Cell.h.zx << " " << box.Cell.h.zy << " " << box.Cell.h.zz << endl;

  out << box.Interactions.size() << endl;
  for (size_t i = 0; i < box.Interactions.size(); i++) {
    out << box.Interactions[i].i + 1 << " " << box.Interactions[i].j + 1 << " " << box.Interactions[i].n << " "
        << box.Interactions[i].fn << " " << box.Interactions[i].ft << endl;
  }
  return 0;
}
