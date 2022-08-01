#include "PBC3D.hpp"

int main(int argc, char const *argv[]) {
  INIT_TIMERS();
  PBC3Dbox box;

  if (argc < 2) {
    box.setSample();
    box.saveConf(0);
    return 0;
  } else {
    box.loadConf(argv[1]);
  }

  box.initOutputFiles();

  // box.saveConf(99);exit(0);
  //box.updateNeighborList(box.dVerlet);
  // std::cout << "box.Interactions.size() = " << box.Interactions.size() << '\n';

  box.showBanner();
  box.freeze();
  box.saveConf(0);
  box.dataOutput();
  return 0;
}
