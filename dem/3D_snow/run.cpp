#include "PBC3D.hpp"

int main(int argc, char const *argv[]) {
  PBC3Dbox box;
  box.showBanner();

  if (argc < 2) {
		/*
    box.setSample();
    box.saveConf(0);
		*/
    return 0;
  } else {
    box.loadConf(argv[1]);
  }

  //box.initOutputFiles();

  box.updateNeighborList(box.dVerlet);
  
  
  box.integrate();

  return 0;
}
