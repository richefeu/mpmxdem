#include "set_MP_grid.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "factory.hpp"
static Registrar<Command, set_MP_grid> registrar("set_MP_grid");

void set_MP_grid::read(std::istream& is) { 
  is >> groupNb >> modelName >> rho >> x0 >> y0 >> x1 >> y1 >> size;
}

void set_MP_grid::exec() {
std::cout << "entering set_MP_grid::exec" << std::endl;
  if (box->Grid.lx / size > 3.0 || box->Grid.ly / size > 3.0) {
    std::cerr << "@set_MP_grid::exec, Check Grid size - MP size ratio (should not be more than 3)" << std::endl;
    exit(0);
  }

  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    std::cerr << "@set_MP_grid::exec, model " << modelName << " not found" << std::endl;
  }
  ConstitutiveModel* CM = itCM->second;

  double halfSizeMP = 0.5 * size;

  MaterialPoint P(groupNb, size, rho, CM);
  CM->init(P);
  int counter = 0;

  // new loop 15/05/2018 (bug should still exist but its working better now)
  double nbMPX = (x1 - x0) / size;
  double nbMPY = (y1 - y0) / size;

  // https://stackoverflow.com/questions/9695329/c-how-to-round-a-double-to-an-int
  nbMPX += 0.5;
  nbMPY += 0.5;
  nbMPX = (int)nbMPX;
  nbMPY = (int)nbMPY;

  for (int i = 0; i < nbMPY; i++) {
    for (int j = 0; j < nbMPX; j++) {
      P.pos.set(x0 + halfSizeMP + size * j, y0 + halfSizeMP + size * i);
      P.nb = counter;
      counter++;
      box->MP.push_back(P);
    }
  }

  for (size_t p = 0; p < box->MP.size(); p++) {
    box->MP[p].updateCornersFromF(); // utile ????
    /*
    if(CM->fileName!=nullptr){
      box->MP[p].PBC.loadConf(CM->fileName);
      box->MP[p].PBC.initOutputFiles();
      box->MP[p].PBC.updateNeighborList(box->MP[p].PBC.dVerlet);
      box->MP[p].PBC.saveConf(p,"titi");
    }
   std::cout << "PBC3Dbox fully created" << std::endl;
   */
  }

  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].pos.x > box->Grid.Nx * box->Grid.lx || box->MP[p].pos.x < 0.0 ||
        box->MP[p].pos.y > box->Grid.Ny * box->Grid.ly || box->MP[p].pos.y < 0.0) {
      std::cerr << "@set_MP_grid::exec, Check before simulation: Some MPs are not inside the grid" << std::endl;
      exit(0);
    }
  }

  std::cout << "@set_MP_grid::exec, total number of MPs: " << box->MP.size() << std::endl;
}
