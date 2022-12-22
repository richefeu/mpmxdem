#include "set_MP_grid.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

void set_MP_grid::read(std::istream& is) { is >> groupNb >> modelName >> rho >> x0 >> y0 >> x1 >> y1 >> size; }

void set_MP_grid::exec() {
  if (box->Grid.lx / size < 2.0 || box->Grid.ly / size < 2.0) {
    box->console->warn("@set_MP_grid::exec, Check Grid size - MP size ratio (should not be more than 2)");
    exit(0);
  }

  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    box->console->error("@set_MP_grid::exec, model {} not found", modelName);
  }
  ConstitutiveModel* CM = itCM->second;

  double halfSizeMP = 0.5 * size;

  int counter = 0;

  // new loop 15/05/2018 (bug should still exist but its working better now)
  double nbMPX = (x1 - x0) / size;
  double nbMPY = (y1 - y0) / size;

  // https://stackoverflow.com/questions/9695329/c-how-to-round-a-double-to-an-int
  nbMPX += 0.5;
  nbMPY += 0.5;
  nbMPX = (int)nbMPX;
  nbMPY = (int)nbMPY;
  
  box->console->info("@set_MP_grid::exec, nbMPX = {}, nbMPY = {}", nbMPX, nbMPY);
  for (int i = 0; i < nbMPY; i++) {
    for (int j = 0; j < nbMPX; j++) {
      MaterialPoint P(groupNb, size, rho, CM);
      CM->init(P);
      if (P.isDoubleScale == true) {
        double Vcell = fabs(P.PBC->Cell.h.det());
        P.density = P.PBC->Cell.mass / Vcell;        
      }
      P.pos.set(x0 + halfSizeMP + size * j, y0 + halfSizeMP + size * i);
      P.nb = counter;
      counter++;
      box->MP.push_back(P);
    }
  }

  for (size_t p = 0; p < box->MP.size(); p++) {
    box->MP[p].updateCornersFromF();
  }

  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].pos.x > (double)box->Grid.Nx * box->Grid.lx || box->MP[p].pos.x < 0.0 ||
        box->MP[p].pos.y > (double)box->Grid.Ny * box->Grid.ly || box->MP[p].pos.y < 0.0) {
      box->console->error("@set_MP_grid::exec, Check before simulation: Some MPs are not inside the grid");
      exit(0);
    }
  }
}
