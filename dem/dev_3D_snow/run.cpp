#include "PBC3D.hpp"
#include "stackTracer.hpp"

int main(int argc, char const* argv[]) {
  StackTracer::initSignals();

  PBC3Dbox box;
  box.showBanner();

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " input-conf-file\n";
    return 0;
  } else {
    box.loadConf(argv[1]);
  }

  box.updateNeighborList(box.dVerlet);

  box.integrate();

  return 0;
}
