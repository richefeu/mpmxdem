#include "PBC3D.hpp"

int main(int argc, char const *argv[]) {
  PBC3Dbox box;
  box.loadConf(argv[1]);
  box.initOutputFiles();
  box.updateNeighborList(box.dVerlet);
  box.showBanner();
  box.computeSampleData();
  box.transform(box.Load.f,box.tmax,"conf"); // velocity gradient time step
  return 0;
}
