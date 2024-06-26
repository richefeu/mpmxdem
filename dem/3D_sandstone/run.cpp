#include "PBC3D.hpp"

int main(int argc, char const *argv[]) {
  INIT_TIMERS();
  PBC3Dbox box;

  if (argc < 2) {
    box.setSample();
    box.saveConf("input.txt");
    return 0;
  } else {
    box.loadConf(argv[1]);
  }

  box.initOutputFiles();

  box.updateNeighborList(box.dVerlet);

  box.showBanner();
  box.integrate();

  PRINT_TIMERS("PBC3D");
  return 0;
}
